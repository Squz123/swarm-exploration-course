#!/usr/bin/env python3
"""Execute story one on the ORIGINAL coursework ROS simulator.

The original simulator, controller, map generator, and teacher checker remain
unchanged. Planning reads the received point cloud; no saved replay is flown.
"""
import json
import math
import os
import re
import sys
import threading
import time
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, Imu
from rosgraph_msgs.msg import Log
from std_msgs.msg import Float64MultiArray, Bool, Float64
import benchmark as b


class LiveWorld(b.World):
    """Conservative pillar bounds derived from the received point cloud."""
    def __init__(self,cloud,target_grid=11,fill_holes=True,target_extent=(22.5,22.5),target_points=None):
        self.cloud=cloud
        ij=np.trunc((cloud[:,:2].astype(float)+25)/.1).astype(int)
        good=(ij>=0).all(axis=1)&(ij<500).all(axis=1)
        # region_vis first rejects points outside the exploration rectangle,
        # before C++'s truncating coordinate conversion (important below -25).
        good &= (cloud[:,:2]>=-25).all(axis=1)&(cloud[:,:2]<=25).all(axis=1)
        self.obstacle=np.zeros((500,500),bool)
        self.obstacle[ij[good,1],ij[good,0]]=True
        self.free=~self.obstacle
        # Dilation only joins neighbouring samples for conservative navigation
        # boxes; it never alters the coverage denominator or teacher map.
        labels,n=ndimage.label(ndimage.binary_dilation(self.obstacle,iterations=1),np.ones((3,3)))
        clipped=np.clip(ij,0,499)
        groups=labels[clipped[:,1],clipped[:,0]]
        boxes=[]
        for k in range(1,n+1):
            points=cloud[groups==k]
            if len(points):boxes.append(np.r_[points.min(axis=0),points.max(axis=0)])
        self.boxes=np.array(boxes,dtype=float)
        self.expanded=self.boxes[self.boxes[:,5]>=b.Z-b.OBS_MARGIN][:,[0,1,3,4]].copy()
        self.expanded[:,:2]-=b.OBS_MARGIN;self.expanded[:,2:]+=b.OBS_MARGIN
        self.targets=[]
        nx,ny=(target_grid,target_grid) if np.isscalar(target_grid) else target_grid
        raw=target_points if target_points is not None else [(x,y) for y in np.linspace(-target_extent[1],target_extent[1],ny) for x in np.linspace(-target_extent[0],target_extent[0],nx)]
        for xy in raw:self.targets.append(self.safe_near(np.array(xy)))
        union=np.zeros(250000,bool)
        for xy in self.targets:union[b.footprint([*xy,b.Z],self.free)]=True
        missing=self.free.ravel()&~union
        while fill_holes and missing.any():
            idx=int(np.flatnonzero(missing)[0]);y,x=divmod(idx,500)
            xy=self.safe_near(np.array([b.AXIS[x],b.AXIS[y]]))
            ids=b.footprint([*xy,b.Z],self.free)
            if not np.any(missing[ids]):raise RuntimeError('A free cell has no safe covering viewpoint')
            self.targets.append(xy);union[ids]=True;missing=self.free.ravel()&~union
        self.targets=np.array(self.targets)
        self.footprints=[b.footprint([*xy,b.Z],self.free) for xy in self.targets]
        self.make_graph()


def unpack_cloud(msg):
    endian='>' if msg.is_bigendian else '<'
    fields={f.name:f for f in msg.fields}
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                    'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':msg.point_step})
    data=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    cloud=np.column_stack([data[k].ravel() for k in ['x','y','z']])
    return cloud[np.isfinite(cloud).all(axis=1)]


def reference(history,t):
    p=history[0].a.copy();v=np.zeros(3);a=v.copy();j=v.copy()
    for seg in history:
        if t<seg.start:break
        u=min(1.,max(0.,(t-seg.start)/seg.T));d=seg.b-seg.a
        p=seg.a+(10*u**3-15*u**4+6*u**5)*d
        if u<1:
            v=(30*u**2-60*u**3+30*u**4)*d/seg.T
            a=(60*u-180*u**2+120*u**3)*d/seg.T**2
            j=(60-360*u+360*u**2)*d/seg.T**3
        else:v=np.zeros(3);a=v.copy();j=v.copy()
    return p,v,a,j


class Flight:
    def __init__(self):
        self.variant=rospy.get_param('~variant','B')
        self.scale=float(rospy.get_param('~time_scale',1.35))
        self.out=Path(rospy.get_param('~output',str(Path(__file__).resolve().parent/'ros_results/test')))
        self.out.mkdir(parents=True,exist_ok=True)
        self.lock=threading.Lock();self.poses={};self.velocities={};self.grid=None;self.cloud=None
        self.coverage=0.;self.maxcoverage=0.;self.grid_updates=0;self.lastgrid=0.
        self.teacher_success=False;self.teacher_duration=None;self.teacher_violations=[];self.source_logs=[]
        self.running=False;self.maxacc=0.;self.maxspeed=0.;self.maxheight=0.;self.t95=None;self.t100=None
        self.trace=[];self.cloud_tree=None
        self.server_ready=False;self.max_command_gap=0.
        self.trajectory_pub=rospy.Publisher('/course_story1/trajectories',Float64MultiArray,queue_size=1,tcp_nodelay=True,latch=True)
        rospy.Subscriber('/course_story1/server_ready',Bool,lambda msg:setattr(self,'server_ready',msg.data),queue_size=1)
        rospy.Subscriber('/course_story1/max_command_gap',Float64,lambda msg:setattr(self,'max_command_gap',msg.data),queue_size=1)
        for i in range(6):
            rospy.Subscriber('/uav%d/sim/odom'%(i+2),Odometry,self.odom,callback_args=i,queue_size=1,tcp_nodelay=True)
            rospy.Subscriber('/uav%d/sim/imu'%(i+2),Imu,self.imu,callback_args=i,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/mock_map',PointCloud2,self.map,queue_size=1,buff_size=2**24)
        rospy.Subscriber(rospy.get_param('~coverage_topic','/exploration_grid'),OccupancyGrid,self.cover,queue_size=1,buff_size=2**22)
        rospy.Subscriber('/rosout_agg',Log,self.logs,queue_size=1000)

    def odom(self,msg,i):
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        pp=np.array([p.x,p.y,p.z]);vv=np.array([v.x,v.y,v.z])
        with self.lock:
            self.poses[i]=pp;self.velocities[i]=vv
            if self.running:
                self.maxspeed=max(self.maxspeed,float(np.linalg.norm(vv)))
                self.maxheight=max(self.maxheight,float(pp[2]))

    def imu(self,msg,i):
        # In this repository, quadToImuMsg publishes world acceleration
        # quad.getAcc(), already including gravity subtraction.
        if self.running:
            a=msg.linear_acceleration
            self.maxacc=max(self.maxacc,math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))

    def map(self,msg):
        if self.cloud is None:self.cloud=unpack_cloud(msg)

    def cover(self,msg):
        now=time.monotonic()
        if now-self.lastgrid<.2:return
        self.lastgrid=now
        if msg.info.width!=500 or msg.info.height!=500:return
        if abs(msg.info.resolution-.1)>1e-6:raise RuntimeError('Unexpected teacher grid resolution')
        data=np.asarray(msg.data,dtype=np.int8)
        free=int(np.count_nonzero(data!=100));covered=int(np.count_nonzero(data==0))
        self.grid=data;self.coverage=covered/free if free else 0.
        self.maxcoverage=max(self.maxcoverage,self.coverage);self.grid_updates+=1
        if self.running:
            elapsed=time.monotonic()-self.motion_start
            if self.t95 is None and self.coverage>=.95:self.t95=elapsed
            if self.t100 is None and covered==free:self.t100=elapsed

    def logs(self,msg):
        if 'region_visualizer' not in msg.name:return
        text=msg.msg
        if 'MISSION SUCCESS' in text:self.teacher_success=True
        match=re.search(r'Mission completed in: ([0-9.]+) seconds',text)
        if match:self.teacher_duration=float(match.group(1))
        if any(k in text for k in ['collided with obstacle','Collision between','overspeed','exceeded height limit']):
            self.teacher_violations.append(text)
        if msg.level>=Log.WARN or any(k in text for k in ['MISSION','statistics','violations','mission time','progress: 100']):
            self.source_logs.append({'stamp':msg.header.stamp.to_sec(),'level':int(msg.level),'text':text})

    def send(self,histories,t):
        values=[self.motion_epoch]
        for history in histories:
            values.append(float(len(history)))
            for seg in history:values.extend([seg.start,seg.T,*seg.a.tolist(),*seg.b.tolist()])
        self.trajectory_pub.publish(Float64MultiArray(data=values))

    def execute(self):
        ready_deadline=time.monotonic()+40
        while not rospy.is_shutdown() and time.monotonic()<ready_deadline:
            if len(self.poses)==6 and self.cloud is not None and self.grid is not None and self.server_ready and self.trajectory_pub.get_num_connections():break
            rospy.sleep(.1)
        if not (len(self.poses)==6 and self.cloud is not None and self.grid is not None and self.server_ready):
            raise RuntimeError('Original simulator odometry/map/grid did not become ready')
        np.save(self.out/'actual_map.npy',self.cloud)
        self.cloud_tree=cKDTree(self.cloud)
        # The stock simulator starts its motors after processes connect. Wait
        # for its own initial hover rather than treating startup sag as a
        # different launch configuration or commanding a discontinuous jump.
        settle_deadline=time.monotonic()+12
        while time.monotonic()<settle_deadline and not rospy.is_shutdown():
            with self.lock:
                initial=np.array([self.poses[i].copy() for i in range(6)])
                speeds=np.array([np.linalg.norm(self.velocities[i]) for i in range(6)])
            if np.max(np.linalg.norm(initial-b.STARTS,axis=1))<.07 and speeds.max()<.15:break
            rospy.sleep(.05)
        initial=np.array([self.poses[i] for i in range(6)])
        (self.out/'initial_actual_positions.json').write_text(json.dumps(initial.tolist()),encoding='utf-8')
        if np.max(np.linalg.norm(initial-b.STARTS,axis=1))>.15:raise RuntimeError('Initial hover differs from launch: '+str(initial.tolist()))
        if self.cloud_tree.query(initial)[0].min()<.2:raise RuntimeError('Initial state already collides')
        # Scale ALL comparison variants identically to allow stock-controller
        # tracking margin. Scaling these three constants scales every segment
        # duration and its time/energy optimum by exactly time_scale.
        b.VMAX/=self.scale;b.AMAX/=self.scale**2;b.ENERGY_WEIGHT*=self.scale**4
        before=time.monotonic();world=LiveWorld(self.cloud)
        expected_free=int(world.free.sum())
        if expected_free!=int(np.count_nonzero(self.grid!=100)):raise RuntimeError('Point cloud and teacher obstacle denominator disagree')
        queues,changes=b.initial_routes(world,self.variant)
        plan_wall=time.monotonic()-before
        initial_queues=[q[:] for q in queues]
        takeoff=float(b.duration(b.Z-1))
        histories=[[b.Segment(0,takeoff,b.STARTS[i],np.r_[b.STARTS[i,:2],b.Z])] for i in range(6)]
        nodes=list(range(6));endtimes=[takeoff]*6
        self.motion_start=time.monotonic();self.motion_epoch=rospy.Time.now().to_sec();self.running=True
        self.send(histories,0.)
        help_events=[];waits=0.;blocked=0;frames=[];last_record=-1.;no_work=None;status='timeout'
        minpair=float('inf');minobs=float('inf');tracking=0.;dispatches=0
        late_plans=0;rate=rospy.Rate(20)
        rospy.loginfo('COURSE_STORY1_BEGIN variant=%s plan_wall=%.3f tasks=%d',self.variant,plan_wall,len(world.targets))
        while not rospy.is_shutdown():
            t=time.monotonic()-self.motion_start
            if t>300:break
            with self.lock:poses=np.array([self.poses[i].copy() for i in range(6)])
            if t-last_record>=.1:
                last_record=t
                pair=min(np.linalg.norm(poses[i]-poses[j]) for i in range(6) for j in range(i))
                obs=float(self.cloud_tree.query(poses)[0].min());minpair=min(minpair,float(pair));minobs=min(minobs,obs)
                ref=np.array([reference(histories[i],t)[0] for i in range(6)])
                tracking=max(tracking,float(np.linalg.norm(poses-ref,axis=1).max()))
                frames.append([round(t,4),self.coverage,*poses.round(6).ravel().tolist()])
                if pair<.55 or obs<.28 or self.maxheight>7.98 or self.maxspeed>5.95:
                    status='safety_guard_stop';break
            if t>=takeoff:
                seen=self.grid==0
                b.clean_queues(world,queues,seen)
                changed=False
                for i in range(6):
                    if t<endtimes[i]:continue
                    if self.variant=='C':b.help_once(world,i,queues,nodes,endtimes,t,seen,help_events)
                    if not queues[i]:continue
                    task=queues[i][0]
                    # Plan ahead of the streaming clock. A completed plan may
                    # never be installed with a start already in the past.
                    plan_now=time.monotonic()-self.motion_start
                    segments,wait=b.schedule(world,nodes[i],task,plan_now+.25,histories,i)
                    if segments and segments[0].start<time.monotonic()-self.motion_start+.05:
                        late_plans+=1;continue
                    if segments is None:blocked+=1;continue
                    queues[i].pop(0);histories[i].extend(segments)
                    endtimes[i]=segments[-1].end if segments else t;nodes[i]=task
                    waits+=wait;dispatches+=1;changed=True
                    # Install this accepted plan immediately; do not let a
                    # later vehicle's expensive planning consume its lead.
                    self.send(histories,time.monotonic()-self.motion_start)
                if changed:self.send(histories,t)
            if not any(queues) and all(t>=et for et in endtimes):
                if no_work is None:no_work=t
                if t-no_work>5:
                    status='complete' if self.maxcoverage==1. and self.teacher_success else 'incomplete_coverage'
                    break
            if int(t)//10>int(max(0,t-.01))//10:rospy.loginfo('COURSE_STORY1_PROGRESS t=%.1f coverage=%.6f queued=%d',t,self.coverage,sum(map(len,queues)))
            try:rate.sleep()
            except rospy.ROSInterruptException:break
        self.running=False
        # Hold each actual position on failure; normal completion retains the
        # final smooth trajectory's stationary endpoint for the runner to stop.
        if status!='complete':
            hold=[[b.Segment(0,1.,poses[i],poses[i])] for i in range(6)]
            self.motion_epoch=rospy.Time.now().to_sec()
            self.send(hold,1.);rospy.sleep(1.)
        result={'scope':'Original coursework ROS/SO3 simulator and unchanged teacher checker',
                'variant':self.variant,'status':status,'time_scale':self.scale,
                'execution_revision':'v2_independent_cpp_streamer',
                'display_mode':os.environ.get('COURSE_DISPLAY','original'),
                'max_command_stream_gap_s':self.max_command_gap,'plans_deferred_to_prevent_past_start':late_plans,
                'coverage':self.maxcoverage,'teacher_success_log_seen':self.teacher_success,
                'teacher_mission_duration_wall_s':self.teacher_duration,
                't95_since_motion_start_wall_s':self.t95,'t100_since_motion_start_wall_s':self.t100,
                'planning_wall_s':plan_wall,'motion_and_settle_wall_s':t,
                'map_points':len(self.cloud),'conservative_connected_boxes':len(world.boxes),
                'viewpoints':len(world.targets),'initial_ownership_changes':changes,
                'help_events':help_events,'reservation_wait_s':waits,'blocked_schedules':blocked,
                'dispatched_tasks':dispatches,'max_actual_speed_mps':self.maxspeed,
                'max_actual_acceleration_mps2_from_simulator_imu':self.maxacc,
                'max_actual_height_m':self.maxheight,'min_actual_pair_distance_sample_m':minpair,
                'min_actual_obstacle_point_distance_sample_m':minobs,'max_tracking_error_m':tracking,
                'teacher_rule_warnings':self.teacher_violations,'grid_updates_observed':self.grid_updates,
                'actual_limits_pass':self.maxspeed<=6 and self.maxacc<=3 and self.maxheight<=8 and minpair>=.4 and minobs>=.2,
                'all_original_parameters_preserved':True}
        (self.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        (self.out/'teacher_messages.json').write_text(json.dumps(self.source_logs,ensure_ascii=False,indent=2),encoding='utf-8')
        (self.out/'flight_trace.json').write_text(json.dumps({'frames':frames,'initial_queues':initial_queues,
            'targets':world.targets.tolist(),'boxes':world.boxes.tolist(),
            'segments':[[s.serial() for s in history] for history in histories]},separators=(',',':')),encoding='utf-8')
        if self.grid is not None:np.save(self.out/'last_teacher_grid.npy',self.grid)
        rospy.loginfo('COURSE_STORY1_RESULT %s',json.dumps(result))
        return result


def main():
    rospy.init_node('course_story1_flight')
    flight=Flight()
    try:result=flight.execute()
    except Exception as exc:
        (flight.out/'error.json').write_text(json.dumps({'error':str(exc),'type':type(exc).__name__},ensure_ascii=False,indent=2),encoding='utf-8')
        raise
    return 0 if result['status']=='complete' and result['actual_limits_pass'] and not result['teacher_rule_warnings'] else 2


if __name__=='__main__':sys.exit(main())

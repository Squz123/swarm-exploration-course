#!/usr/bin/env python3
"""Faster continuous-flight experiment on the unchanged original simulator."""
import json
import math
import time
import sys
import os
import numpy as np
from scipy.spatial import cKDTree
import rospy
from std_msgs.msg import Float64MultiArray
import benchmark as b
from ros_flight import Flight,LiveWorld
import fast_plan as fp


class FastFlight(Flight):
    def __init__(self):
        super().__init__()
        self.trajectory_pub=rospy.Publisher('/course_fast/polynomials',Float64MultiArray,queue_size=1,latch=True,tcp_nodelay=True)
        self.audit=None
        self.acceleration_peak_events=[]
        self.high_acceleration_samples=[]
        rospy.Subscriber('/course_fast/audit',Float64MultiArray,lambda msg:setattr(self,'audit',list(msg.data)),queue_size=1)
    def imu(self,msg,i):
        old=self.maxacc
        super().imu(msg,i)
        if self.running:
            a=msg.linear_acceleration;norm=math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)
            if norm>2.5:self.high_acceleration_samples.append([time.monotonic()-self.motion_start,i+2,norm])
        if self.running and self.maxacc>old and self.maxacc>2.5:
            self.acceleration_peak_events.append({'t':time.monotonic()-self.motion_start,'drone':i+2,'acceleration':self.maxacc})
    def execute(self):
        deadline=time.monotonic()+40
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            if len(self.poses)==6 and self.cloud is not None and self.grid is not None and self.server_ready and self.trajectory_pub.get_num_connections():break
            rospy.sleep(.05)
        if not(self.server_ready and self.grid is not None and self.cloud is not None):raise RuntimeError('Simulator not ready')
        np.save(self.out/'actual_map.npy',self.cloud);self.cloud_tree=cKDTree(self.cloud)
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            initial=np.array([self.poses[i] for i in range(6)])
            if np.linalg.norm(initial-b.STARTS,axis=1).max()<.07 and max(np.linalg.norm(v) for v in self.velocities.values())<.15:break
            rospy.sleep(.05)
        if np.linalg.norm(initial-b.STARTS,axis=1).max()>.15:raise RuntimeError('Unexpected initial hover')
        vmax=float(rospy.get_param('~vmax',5.7));amax=float(rospy.get_param('~amax',2.7))
        altitude=float(rospy.get_param('~altitude',7.85));grid=int(rospy.get_param('~target_grid',10));alpha=float(rospy.get_param('~alpha',1.))
        b.Z=altitude
        fp.CURVE_MODE=str(rospy.get_param('~curve','zero_acc'))
        fp.TAKEOFF_BUFFER=float(rospy.get_param('~takeoff_buffer',0.))
        fp.ENTRY_AMAX=float(rospy.get_param('~entry_amax',amax))
        before=time.monotonic();world=LiveWorld(self.cloud,grid)
        if world.free.sum()!=np.count_nonzero(self.grid!=100):raise RuntimeError('Teacher map denominator mismatch')
        prune=bool(rospy.get_param('~prune',False))
        histories,routes,check=fp.plan(world,self.cloud,vmax,amax,alpha,prune=prune)
        planning=time.monotonic()-before
        (self.out/'preflight.json').write_text(json.dumps(check,indent=2))
        (self.out/'plan.json').write_text(json.dumps({'polynomials':[[s.serial() for s in h] for h in histories],'routes':routes}))
        self.motion_epoch=rospy.Time.now().to_sec()+.25
        values=[self.motion_epoch]
        for h in histories:
            values.append(len(h))
            for s in h:values.extend([s.start,s.T,*s.coefficients.ravel().tolist()])
        self.trajectory_pub.publish(Float64MultiArray(data=values))
        self.motion_start=time.monotonic()+.25;self.running=True
        rospy.loginfo('COURSE_FAST_BEGIN planned_finish=%.3f planning=%.3f vmax=%.3f amax=%.3f',check['planned_finish_s'],planning,vmax,amax)
        frames=[];tracking=0.;status='timeout';lastlog=-10.;rate=rospy.Rate(50)
        finish=max(h[-1].end for h in histories)
        while not rospy.is_shutdown():
            t=time.monotonic()-self.motion_start
            if t<0:rate.sleep();continue
            with self.lock:poses=np.array([self.poses[i].copy() for i in range(6)])
            ref=np.array([fp.sample(h,np.array([t]))[0] for h in histories])
            tracking=max(tracking,float(np.linalg.norm(poses-ref,axis=1).max()))
            frames.append([round(t,4),self.coverage,*poses.ravel().tolist()])
            if self.audit and (self.audit[3]<.48 or self.audit[4]<.24 or self.audit[2]>8.01 or self.audit[0]>6.05):
                status='safety_guard_stop';break
            if t-lastlog>=5:
                lastlog=t;rospy.loginfo('COURSE_FAST_PROGRESS t=%.1f coverage=%.6f tracking=%.3f',t,self.coverage,tracking)
            if t>=finish+1.5:
                status='complete' if self.teacher_success and self.maxcoverage==1. else 'incomplete_coverage';break
            if t>finish+10:break
            rate.sleep()
        self.running=False
        audit=self.audit or [float('inf')]*7
        speed=max(self.maxspeed,audit[0]);acc=max(self.maxacc,audit[1]);height=max(self.maxheight,audit[2])
        result={'scope':'Unchanged original coursework ROS simulator, controller, map and teacher checker',
                'execution_revision':'v3_continuous_polynomial','status':status,'variant':'FAST',
                'coverage':self.maxcoverage,'teacher_success_log_seen':self.teacher_success,
                'teacher_mission_duration_wall_s':self.teacher_duration,'t100_since_motion_start_wall_s':self.t100,
                'planning_wall_s':planning,'planned_finish_s':finish,'motion_and_settle_wall_s':t,
                'max_actual_speed_mps':speed,'max_actual_acceleration_mps2_from_simulator_imu':acc,
                'max_actual_height_m':height,'min_actual_pair_distance_sample_m':audit[3],
                'min_actual_obstacle_point_distance_sample_m':audit[4],'max_tracking_error_m':tracking,
                'max_command_stream_gap_s':self.max_command_gap,'teacher_rule_warnings':self.teacher_violations,
                'actual_limits_pass':speed<=6 and acc<=3 and height<=8 and audit[3]>=.4 and audit[4]>=.2,
                'audit_odom_samples':audit[5],'audit_imu_samples':audit[6],
                'requested_vmax':vmax,'requested_amax':amax,'requested_altitude':altitude,'target_grid':grid,'alpha':alpha,
                'prune':prune,
                'curve':fp.CURVE_MODE,
                'takeoff_buffer_s':fp.TAKEOFF_BUFFER,
                'entry_amax':fp.ENTRY_AMAX,
                'preflight':check,'all_original_parameters_preserved':True,'display_mode':os.environ.get('COURSE_DISPLAY','smooth')}
        (self.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        (self.out/'teacher_messages.json').write_text(json.dumps(self.source_logs,ensure_ascii=False,indent=2))
        (self.out/'acceleration_peak_events.json').write_text(json.dumps(self.acceleration_peak_events,indent=2))
        (self.out/'high_acceleration_samples.json').write_text(json.dumps(self.high_acceleration_samples,separators=(',',':')))
        (self.out/'flight_trace.json').write_text(json.dumps({'frames':frames,'boxes':world.boxes.tolist()},separators=(',',':')))
        np.save(self.out/'last_teacher_grid.npy',self.grid)
        rospy.loginfo('COURSE_FAST_RESULT %s',json.dumps(result))
        return result


if __name__=='__main__':
    rospy.init_node('course_fast_flight');flight=FastFlight()
    try:r=flight.execute()
    except Exception as exc:
        (flight.out/'error.json').write_text(json.dumps({'type':type(exc).__name__,'error':str(exc)}));raise
    sys.exit(0 if r['status']=='complete' and r['actual_limits_pass'] and not r['teacher_rule_warnings'] else 2)

#!/usr/bin/env python3
"""Round clock and independent examiner; never selects a task or route.

Six external online_agent processes calculate bids, allocate and check paths.
This process may only release matching votes or abort the round. Teacher grid
and all-UAV audit subscriptions stay here, outside the decision processes.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import numpy as np
import rospy
from std_msgs.msg import String, Float64MultiArray
from ros_flight import Flight
import online_protocol as op

class OnlineFlight(Flight):
    def __init__(self):
        super().__init__()
        self.pub=rospy.Publisher('/course_online/clock',String,queue_size=50,tcp_nodelay=True)
        self.box={}; self.events=[]; self.rounds=[]; self.audit=None; self.acks={}
        self.guard=threading.RLock(); self.children=[]; self.logs_open=[]
        rospy.Subscriber('/course_online/peers',String,self.receive,queue_size=1000,tcp_nodelay=True)
        rospy.Subscriber('/course_online/ack',Float64MultiArray,self.ack,queue_size=100)
        rospy.Subscriber('/course_fast/audit',Float64MultiArray,lambda m:setattr(self,'audit',list(m.data)),queue_size=1)

    def ack(self,msg):
        if len(msg.data)==4:self.acks[(int(msg.data[0]),int(msg.data[1]))]=list(msg.data)

    def receive(self,msg):
        m=json.loads(msg.data)
        with self.guard:
            self.box.setdefault((m['round'],m['kind']),{})[m['sender']]=m
            self.events.append({'received_ros_time':rospy.Time.now().to_sec(),**m})

    def clock(self,stage,round_id=-1,**values):
        self.pub.publish(String(data=json.dumps({'stage':stage,'round':round_id,**values})))

    def wait(self,kind,round_id,timeout):
        end=time.monotonic()+timeout
        while not rospy.is_shutdown() and time.monotonic()<end:
            if any(p.poll() is not None for p in self.children):raise RuntimeError('A peer process exited')
            with self.guard:messages=dict(self.box.get((round_id,kind),{}))
            if len(messages)==6:return messages
            rospy.sleep(.02)
        return None

    def launch_agents(self):
        for i in range(6):
            log=(self.out/('peer%d.log'%(i+2))).open('w');self.logs_open.append(log)
            self.children.append(subprocess.Popen([sys.executable,str(Path(__file__).with_name('online_agent.py')),
                                                  '_index:='+str(i)],stdout=log,stderr=subprocess.STDOUT))

    def execute(self):
        self.launch_agents()
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if self.server_ready and self.cloud is not None and self.grid is not None and len(self.poses)==6:break
            rospy.sleep(.1)
        if not(self.server_ready and self.cloud is not None and self.grid is not None and len(self.poses)==6):
            raise RuntimeError('Original simulator not ready')
        if not self.wait('available',-1,30):raise RuntimeError('Six peer processes not available')
        settle=time.monotonic()+12
        while time.monotonic()<settle:
            if max(np.linalg.norm(self.poses[i]-np.array([[-2,-1,1],[-2,0,1],[-2,1,1],[-1,-1,1],[-1,0,1],[-1,1,1]])[i]) for i in range(6))<.07 and max(np.linalg.norm(v) for v in self.velocities.values())<.15:break
            rospy.sleep(.1)
        pause_uav=int(rospy.get_param('~pause_uav',-1))
        self.clock('config',mission_epoch=0,altitude=7.5,vmax=4.5,amax=2.2,target_grid=10,
                   pause_uav=pause_uav,pause_at=float(rospy.get_param('~pause_at',25)),
                   pause_for=float(rospy.get_param('~pause_for',25)),
                   drop_bid_round=int(rospy.get_param('~drop_bid_round',-1)))
        ready=self.wait('ready',-1,60)
        if not ready:raise RuntimeError('Peer map preparation failed')
        epoch=rospy.Time.now().to_sec()+2
        self.clock('takeoff',epoch=epoch)
        if not self.wait('takeoff_ready',-1,1.5):raise RuntimeError('Takeoff preparation timed out')
        until=time.monotonic()+1
        while time.monotonic()<until and len(self.acks)<6:rospy.sleep(.02)
        if len(self.acks)<6:raise RuntimeError('Executor did not acknowledge six takeoff plans')
        self.motion_start=time.monotonic()+(epoch-rospy.Time.now().to_sec());self.running=True
        np.save(self.out/'actual_map.npy',self.cloud)
        frames=[];status='timeout';consecutive_aborts=0;round_id=0;last_trace=0;idle_rounds=0
        max_seconds=float(rospy.get_param('~max_seconds',600))
        while not rospy.is_shutdown() and time.monotonic()-self.motion_start<max_seconds:
            elapsed=time.monotonic()-self.motion_start
            if elapsed-last_trace>.2:
                with self.lock:positions=np.array([self.poses[i].copy() for i in range(6)])
                frames.append([round(elapsed,4),self.coverage,*positions.ravel().tolist()]);last_trace=elapsed
            if self.teacher_success and self.maxcoverage==1:
                status='complete';break
            round_id+=1;record={'round':round_id,'elapsed':elapsed,'coverage':self.coverage};self.clock('state',round_id)
            states=self.wait('state',round_id,2)
            if states:
                self.clock('bid',round_id);bids=self.wait('bid',round_id,2)
            else:bids=None
            if bids:
                cutover=rospy.Time.now().to_sec()+3
                self.clock('plan',round_id,cutover=cutover);proposals=self.wait('proposal',round_id,1.6)
            else:proposals=None
            if proposals:
                self.clock('vote',round_id);votes=self.wait('vote',round_id,.8)
            else:votes=None
            if votes and op.unanimous(votes) and rospy.Time.now().to_sec()<cutover-.4:
                digest=votes[0]['digest'];self.clock('commit',round_id,digest=digest)
                record.update(status='committed',digest=digest,accepted=votes[0]['accepted'])
                consecutive_aborts=0
                if not votes[0]['accepted']:idle_rounds+=1
                else:idle_rounds=0
            else:
                self.clock('abort',round_id)
                missing='state' if not states else 'bid' if not bids else 'proposal' if not proposals else 'vote_or_deadline'
                record.update(status='aborted',reason=missing);consecutive_aborts+=1
            self.rounds.append(record)
            rospy.loginfo('ONLINE_ROUND %s',json.dumps(record))
            # Keep examining actual flight while waiting for the next round.
            until=time.monotonic()+.45
            while time.monotonic()<until:
                elapsed=time.monotonic()-self.motion_start
                with self.lock:positions=np.array([self.poses[i].copy() for i in range(6)])
                frames.append([round(elapsed,4),self.coverage,*positions.ravel().tolist()]);rospy.sleep(.1)
            if consecutive_aborts>=8:status='communication_failure';break
            if self.audit and (self.audit[0]>6 or self.audit[1]>3 or self.audit[2]>8 or self.audit[3]<.4 or self.audit[4]<.2):
                status='actual_limit_failure';break
            if idle_rounds>15 and states and all(s['idle'] for s in states.values()):
                status='no_remaining_task_but_coverage_incomplete';break
        # Drain already accepted routes to their rest endpoints and include this
        # interval in the measured maxima, even if coverage finished earlier.
        drain_until=max([rospy.Time.now().to_sec()]+[a[3] for a in self.acks.values()])+.8
        while not rospy.is_shutdown() and rospy.Time.now().to_sec()<drain_until:
            elapsed=time.monotonic()-self.motion_start
            with self.lock:positions=np.array([self.poses[i].copy() for i in range(6)])
            frames.append([round(elapsed,4),self.coverage,*positions.ravel().tolist()]);rospy.sleep(.1)
        self.running=False
        audit=self.audit or [float('inf')]*7
        assignments=[e for e in self.events if e['kind']=='assignment']
        missing_acks=[(e['sender'],e['version']) for e in assignments if (e['sender'],e['version']) not in self.acks]
        result={'status':status,'coverage':self.maxcoverage,'teacher_success_log_seen':self.teacher_success,
                'teacher_mission_duration_wall_s':self.teacher_duration,'teacher_rule_warnings':self.teacher_violations,
                'max_actual_speed_mps':audit[0],'max_actual_acceleration_mps2_from_simulator_imu':audit[1],
                'max_actual_height_m':audit[2],'min_actual_pair_distance_sample_m':audit[3],
                'min_actual_obstacle_point_distance_sample_m':audit[4],
                'actual_limits_pass':audit[0]<=6 and audit[1]<=3 and audit[2]<=8 and audit[3]>=.4 and audit[4]>=.2,
                'protocol':'six independent ROS peers; all six votes required; clock has no allocation function',
                'teacher_grid_used_for_decisions':False,'global_obstacle_map_used_for_navigation':True,
                'online_assignments':len(assignments),'committed_rounds':sum(r['status']=='committed' for r in self.rounds),
                'mission_epoch':epoch,'assignment_acknowledgements_missing':missing_acks,
                'aborted_rounds':sum(r['status']=='aborted' for r in self.rounds),'pause_uav':pause_uav,
                'motion_and_settle_wall_s':time.monotonic()-self.motion_start,
                'all_original_parameters_preserved':True,'vmax':4.5,'amax':2.2,'altitude':7.5}
        (self.out/'result.json').write_text(json.dumps(result,indent=2))
        (self.out/'rounds.json').write_text(json.dumps(self.rounds,indent=2))
        (self.out/'messages.json').write_text(json.dumps(self.events,separators=(',',':')))
        (self.out/'teacher_messages.json').write_text(json.dumps(self.source_logs,indent=2))
        (self.out/'flight_trace.json').write_text(json.dumps({'frames':frames},separators=(',',':')))
        np.save(self.out/'last_teacher_grid.npy',self.grid)
        print(json.dumps(result,indent=2),flush=True)
        return result

    def close(self):
        for p in self.children:
            if p.poll() is None:p.terminate()
        for p in self.children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in self.logs_open:f.close()

if __name__=='__main__':
    rospy.init_node('course_online_examiner');flight=OnlineFlight()
    try:
        result=flight.execute()
    except Exception as exc:
        (flight.out/'error.json').write_text(json.dumps({'type':type(exc).__name__,'error':str(exc)}))
        raise
    finally:flight.close()
    sys.exit(0 if result['status']=='complete' and result['actual_limits_pass'] and not result['teacher_rule_warnings'] and not result['assignment_acknowledgements_missing'] else 2)

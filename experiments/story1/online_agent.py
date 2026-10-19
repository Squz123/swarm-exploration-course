#!/usr/bin/env python3
"""One UAV decision process. Only own odometry, obstacle map and peer messages.

Never subscribes to teacher exploration_grid or another UAV's odometry.
The supervisor provides round timing and an all-votes-matching release gate;
it does not supply coverage, bids, task assignments or planned routes.
"""
import json
import threading
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64MultiArray
import benchmark as b
import online_protocol as op
import fast_plan as fp
from ros_flight import LiveWorld, unpack_cloud

class Agent:
    def __init__(self):
        self.i = int(rospy.get_param('~index'))
        self.pose = None; self.velocity = None; self.cloud = None; self.world = None
        self.messages = {}; self.lock = threading.RLock(); self.config = None
        self.own_seen = np.zeros(250000, bool); self.done = set()
        self.history = []; self.node = self.i; self.target = None; self.version = 0
        self.current_round = -1; self.pending = None
        self.pub = rospy.Publisher('/course_online/peers', String, queue_size=200, tcp_nodelay=True)
        self.command = rospy.Publisher('/course_online/segments', Float64MultiArray, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber('/uav%d/sim/odom'%(self.i+2), Odometry, self.odom, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/mock_map', PointCloud2, self.map, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/course_online/peers', String, self.receive, queue_size=500, tcp_nodelay=True)
        rospy.Subscriber('/course_online/clock', String, self.clock, queue_size=50, tcp_nodelay=True)
        self.phases = []

    def send(self, kind, **values):
        m = {'kind':kind, 'sender':self.i, 'round':self.current_round, **values}
        self.pub.publish(String(data=json.dumps(m, separators=(',',':'))))

    def odom(self, msg):
        p = msg.pose.pose.position; v = msg.twist.twist.linear
        with self.lock:
            self.pose = np.array([p.x,p.y,p.z]); self.velocity = np.array([v.x,v.y,v.z])

    def map(self, msg):
        if self.cloud is None:
            self.cloud = unpack_cloud(msg)

    def receive(self, msg):
        m = json.loads(msg.data)
        if m['kind'] in ('state','bid','proposal','vote'):
            with self.lock:
                self.messages.setdefault((m['round'],m['kind']),{})[m['sender']] = m

    def clock(self, msg):
        with self.lock:
            self.phases.append(json.loads(msg.data))

    def get(self, kind):
        deadline = time.monotonic()+1.0
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            with self.lock:
                value = dict(self.messages.get((self.current_round,kind),{}))
            if len(value) == 6:
                return value
            rospy.sleep(.01)
        raise RuntimeError('Peer messages incomplete: '+kind)

    def install(self, history):
        self.version += 1
        values = [self.i, self.version, len(history)]
        for s in history:
            c = fp.coefficient(s.a,s.b,np.zeros(3),np.zeros(3),s.T)
            values.extend([s.start,s.T,*c.ravel().tolist()])
        self.command.publish(Float64MultiArray(data=values))
        self.history = history

    def sample_local_coverage(self):
        if self.world is None or self.pose is None:
            return
        self.own_seen[b.footprint(self.pose,self.world.free)] = True
        if self.target is not None and rospy.Time.now().to_sec()>self.history[-1].end+.35:
            goal = self.history[-1].b
            if np.linalg.norm(self.pose-goal)<.12 and np.linalg.norm(self.velocity)<.15:
                self.done.add(self.target); self.node = self.target; self.target = None

    def phase(self, m):
        self.current_round = m.get('round',-1)
        stage = m['stage']; now = rospy.Time.now().to_sec()
        if stage == 'config':
            self.config = m
            b.Z=m['altitude']; b.VMAX=m['vmax']; b.AMAX=m['amax']
            self.world = LiveWorld(self.cloud,m['target_grid'])
            self.send('ready', targets=len(self.world.targets))
        elif stage == 'takeoff':
            t = m['epoch']; end = np.r_[b.STARTS[self.i,:2],b.Z]
            self.config['mission_epoch'] = t
            seg = b.Segment(t,float(b.duration(b.Z-1)),b.STARTS[self.i],end)
            self.install([seg]); self.send('takeoff_ready', epoch=t)
        elif stage == 'state':
            if not self.history:
                return
            self.pending = None
            elapsed = now-self.config['mission_epoch']
            paused = self.i == self.config['pause_uav'] and self.config['pause_at']<=elapsed<self.config['pause_at']+self.config['pause_for']
            self.send('state', pose=self.pose.tolist(), node=int(self.node),
                      idle=self.target is None and now>self.history[-1].end+.35,
                      enabled=not paused, active=self.target, visited=sorted(self.done),
                      observed=op.pack_seen(self.own_seen), history=op.serialize(self.history[-1:] if self.target is None else self.history))
        elif stage == 'bid':
            states = self.get('state'); mine = states[self.i]
            seen = np.zeros(250000,bool); visited=set(); active=set()
            for s in states.values():
                seen |= op.unpack_seen(s['observed']); visited.update(s['visited'])
                if s['active'] is not None: active.add(s['active'])
            offers = []
            for task in range(6,len(self.world.targets)+6):
                if task in visited or task in active: continue
                ids = self.world.footprints[task-6]
                gain = np.count_nonzero(~seen[ids])
                if gain == 0: continue
                cost = float(self.world.cost[mine['node'],task])
                # Prefer short travel, then larger NEW footprint; no full route ownership.
                offers.append([task,cost+.5*(1-gain/max(1,len(ids)))])
            if self.i == 0 and self.current_round == self.config['drop_bid_round']:
                self.send('fault_injected', action='one_bid_omitted'); return
            self.send('bid', idle=mine['idle'], enabled=mine['enabled'], offers=offers,
                      local_seen=int(self.own_seen.sum()), shared_seen=int(seen.sum()))
        elif stage == 'plan':
            states = self.get('state'); bids = self.get('bid')
            assignment = op.allocation(bids); task = assignment.get(self.i)
            segments = []
            if task is not None:
                histories = [op.deserialize(states[i]['history']) for i in range(6)]
                segments,_ = b.schedule(self.world, states[self.i]['node'],task,m['cutover'],histories,self.i)
                segments = segments or []
            self.send('proposal', task=task, segments=op.serialize(segments))
        elif stage == 'vote':
            states=self.get('state'); bids=self.get('bid'); proposals=self.get('proposal')
            accepted,digest=op.decision(states,bids,proposals,b.VMAX)
            self.pending=(accepted,proposals,digest)
            self.send('vote', digest=digest, accepted=accepted)
        elif stage == 'commit':
            votes=self.get('vote')
            if not op.unanimous(votes) or self.pending is None or m['digest']!=self.pending[2]:
                raise RuntimeError('Commit rejected: no six-peer agreement')
            accepted,proposals,_=self.pending
            if self.i in accepted:
                segments=op.deserialize(proposals[self.i]['segments'])
                if segments[0].start < now+.10:
                    raise RuntimeError('Commit too late')
                self.install(segments); self.target=accepted[self.i]
                self.send('assignment',task=self.target,source_node=int(self.node),
                          begins=segments[0].start,ends=segments[-1].end,version=self.version)
        elif stage == 'abort':
            self.pending=None

    def run(self):
        rate=rospy.Rate(10); last_ready=0
        while not rospy.is_shutdown():
            if self.cloud is not None and self.pose is not None and self.world is None and time.monotonic()-last_ready>1:
                self.send('available'); last_ready=time.monotonic()
            self.sample_local_coverage()
            with self.lock:
                phase=self.phases.pop(0) if self.phases else None
            if phase is not None:
                try:self.phase(phase)
                except Exception as exc:
                    self.send('agent_error',stage=phase['stage'],error=str(exc))
                    rospy.logerr('ONLINE_PEER_%d %s',self.i+2,str(exc))
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('course_online_agent',anonymous=True)
    Agent().run()

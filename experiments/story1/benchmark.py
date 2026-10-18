"""Native Windows, explicitly non-ROS, coursework story-one experiment.

Only numpy and the standard library are required. All original sources remain
untouched. See PLAN.md for model limits and predeclared acceptance conditions.
"""
from __future__ import annotations
import argparse
import heapq
import itertools
import json
import math
import platform
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
Z = 7.5
DT = 0.1
VMAX = 5.5
AMAX = 2.5
ENERGY_WEIGHT = 0.12
OBS_MARGIN = 0.5
RESERVE_MARGIN = 0.9
STARTS = np.array([[-2,-1,1],[-2,0,1],[-2,1,1],[-1,-1,1],[-1,0,1],[-1,1,1]],float)
AXIS = -25 + (np.arange(500)+0.5)*0.1


class LibstdRandom:
    """Explicit assumption: GCC default_random_engine is minstd_rand0."""
    def __init__(self, seed): self.state = seed % 2147483647 or 1
    def next(self):
        self.state = self.state*16807 % 2147483647
        return self.state
    def uniform(self, lo, hi):
        # generate_canonical<double,53> needs two 31-bit engine outputs;
        # the first generated output occupies the less significant digits.
        r = 2147483646.0
        a,b = self.next()-1, self.next()-1
        return lo+(hi-lo)*((a+b*r)/(r*r))


def generate_map(seed):
    rng = LibstdRandom(seed)
    boxes, projected, clouds = [], [], []
    for _ in range(50):
        x,y = rng.uniform(-25,25),rng.uniform(-25,25)
        w,h = rng.uniform(1,2),rng.uniform(0,10)
        half = math.ceil(w/0.1)//2  # C++ integer division toward zero
        nheight = math.ceil(h/0.1)
        xs=np.asarray(x+np.arange(-half,half)*0.1,dtype=np.float32)
        ys=np.asarray(y+np.arange(-half,half)*0.1,dtype=np.float32)
        zs=np.asarray(np.arange(nheight)*0.1,dtype=np.float32)
        boxes.append([float(xs.min()),float(ys.min()),0,float(xs.max()),float(ys.max()),float(zs.max())])
        xx,yy=np.meshgrid(xs,ys,indexing='ij')
        projected.append(np.c_[xx.ravel(),yy.ravel()])
        # Exact hollow shell construction: bottom/top full caps, sides only.
        ix,iy,iz=np.meshgrid(np.arange(len(xs)),np.arange(len(ys)),np.arange(len(zs)),indexing='ij')
        shell=(ix==0)|(ix==len(xs)-1)|(iy==0)|(iy==len(ys)-1)|(iz==0)|(iz==len(zs)-1)
        clouds.append(np.c_[xs[ix[shell]],ys[iy[shell]],zs[iz[shell]]])
    obstacle=np.zeros((500,500),bool)
    xy=np.concatenate(projected).astype(float)
    ij=np.trunc((xy+25)/0.1).astype(int)
    valid=(ij>=0).all(axis=1)&(ij<500).all(axis=1)
    obstacle[ij[valid,1],ij[valid,0]]=True
    return np.array(boxes),obstacle,np.concatenate(clouds)


def box_dist(points, boxes):
    points=np.atleast_2d(points)
    d=np.maximum(np.maximum(boxes[None,:,:3]-points[:,None,:], points[:,None,:]-boxes[None,:,3:]),0)
    return np.sqrt((d*d).sum(axis=2)).min(axis=1)


def duration(length):
    length=np.asarray(length)
    # With q(u)=10u^3-15u^4+6u^5, max q'=1.875,
    # max |q''|=10/sqrt(3), integral(q''^2)=120/7.
    optimum=(3*ENERGY_WEIGHT*(120/7)*length**2)**0.25
    ans=np.maximum.reduce([1.875*length/VMAX,np.sqrt((10/math.sqrt(3))*length/AMAX),optimum])
    return np.maximum(ans,0.01)


def footprint(pos, free):
    radius=max(0,pos[2]*0.5-0.15)
    x0=max(0,int(math.ceil((pos[0]-radius+25)/0.1-0.5)))
    x1=min(500,int(math.floor((pos[0]+radius+25)/0.1-0.5))+1)
    y0=max(0,int(math.ceil((pos[1]-radius+25)/0.1-0.5)))
    y1=min(500,int(math.floor((pos[1]+radius+25)/0.1-0.5))+1)
    if x1<=x0 or y1<=y0:return np.empty(0,int)
    mask=(AXIS[x0:x1][None,:]-pos[0])**2+(AXIS[y0:y1][:,None]-pos[1])**2<=radius**2
    mask &= free[y0:y1,x0:x1]
    yy,xx=np.nonzero(mask)
    return (yy+y0)*500+xx+x0


class World:
    def __init__(self,seed):
        self.seed=seed
        self.boxes,self.obstacle,self.cloud=generate_map(seed)
        self.free=~self.obstacle
        self.expanded=self.boxes[self.boxes[:,5]>=Z-OBS_MARGIN][:,[0,1,3,4]].copy()
        self.expanded[:,:2]-=OBS_MARGIN
        self.expanded[:,2:]+=OBS_MARGIN
        self.targets=[]
        # Identical geometric target set for all comparisons.
        for y in np.linspace(-22.5,22.5,11):
            for x in np.linspace(-22.5,22.5,11):
                self.targets.append(self.safe_near(np.array([x,y])))
        union=np.zeros(250000,bool)
        for xy in self.targets:union[footprint([*xy,Z],self.free)]=True
        missing=self.free.ravel()&~union
        while missing.any():
            idx=int(np.flatnonzero(missing)[0])
            y,x=divmod(idx,500)
            xy=self.safe_near(np.array([AXIS[x],AXIS[y]]))
            ids=footprint([*xy,Z],self.free)
            if not np.any(missing[ids]):raise RuntimeError('Cannot safely cover all free cells')
            self.targets.append(xy)
            union[ids]=True
            missing=self.free.ravel()&~union
        self.targets=np.array(self.targets)
        self.footprints=[footprint([*xy,Z],self.free) for xy in self.targets]
        self.make_graph()

    def clear(self,xy):
        xy=np.atleast_2d(xy)
        if len(self.expanded)==0:return np.ones(len(xy),bool)
        inside=((xy[:,None,:]>=self.expanded[None,:,:2])&(xy[:,None,:]<=self.expanded[None,:,2:])).all(axis=2).any(axis=1)
        return ~inside

    def safe_near(self,xy):
        if self.clear(xy)[0]:return xy
        # Project onto a conservative safe navigation lattice, without moving
        # original obstacles, deleting cells, or silently changing the map.
        aa=np.arange(-24.75,25,0.25)
        xx,yy=np.meshgrid(aa,aa)
        candidates=np.c_[xx.ravel(),yy.ravel()]
        good=self.clear(candidates)
        candidates=candidates[good]
        return candidates[np.argmin(((candidates-xy)**2).sum(axis=1))]

    def clear_segments(self, a, ends):
        if not len(self.expanded):return np.ones(len(ends),bool)
        d=ends-a
        dd=np.where(np.abs(d)<1e-12,1e-30,d)
        lo=(self.expanded[None,:,:2]-a)/dd[:,None,:]
        hi=(self.expanded[None,:,2:]-a)/dd[:,None,:]
        tlo=np.minimum(lo,hi).max(axis=2)
        thi=np.maximum(lo,hi).min(axis=2)
        intersect=(np.maximum(tlo,0)<=np.minimum(thi,1))
        return ~intersect.any(axis=1)

    def make_graph(self):
        corners=[]
        for x0,y0,x1,y1 in self.expanded:
            corners.extend([[x0-.025,y0-.025],[x0-.025,y1+.025],[x1+.025,y0-.025],[x1+.025,y1+.025]])
        corners=np.array(corners).reshape(-1,2)
        if len(corners):
            corners=corners[self.clear(corners)&(np.abs(corners)<24.95).all(axis=1)]
        self.nodes=np.vstack([STARTS[:,:2],self.targets,corners])
        n=len(self.nodes)
        self.edges=np.full((n,n),np.inf)
        for i,xy in enumerate(self.nodes):
            ok=self.clear_segments(xy,self.nodes)
            self.edges[i,ok]=duration(np.linalg.norm(self.nodes[ok]-xy,axis=1))
            self.edges[i,i]=0
        self.adj=[[(int(j),float(self.edges[i,j])) for j in np.flatnonzero(np.isfinite(self.edges[i])) if j!=i] for i in range(n)]
        ns=6+len(self.targets)
        self.cost=np.full((ns,ns),np.inf)
        self.paths={}
        for source in range(ns):
            dist=np.full(n,np.inf);dist[source]=0
            parent=np.full(n,-1,int);q=[(0.,source)]
            while q:
                dd,u=heapq.heappop(q)
                if dd>dist[u]+1e-9:continue
                for v,w in self.adj[u]:
                    nd=dd+w
                    if nd<dist[v]-1e-9:
                        dist[v]=nd;parent[v]=u;heapq.heappush(q,(nd,v))
            self.cost[source]=dist[:ns]
            for dest in range(ns):
                if not np.isfinite(dist[dest]):continue
                path=[dest]
                while path[-1]!=source:path.append(int(parent[path[-1]]))
                self.paths[source,dest]=path[::-1]
        if not np.isfinite(self.cost).all():raise RuntimeError('Disconnected navigation graph')


def route_cost(w,start,route):
    if not route:return 0.
    return float(w.cost[start,route[0]]+sum(w.cost[a,b] for a,b in zip(route,route[1:])))


def nearest_order(w,start,tasks):
    remaining=set(tasks);order=[];at=start
    while remaining:
        nxt=min(remaining,key=lambda v:(w.cost[at,v],v))
        order.append(nxt);remaining.remove(nxt);at=nxt
    return order


def initial_routes(w,variant):
    angle=np.arctan2(w.targets[:,1],w.targets[:,0])
    sector=np.floor((angle+math.pi)/(2*math.pi/6)).astype(int)%6
    groups=[[int(k+6) for k in np.flatnonzero(sector==g)] for g in range(6)]
    orders=[[nearest_order(w,i,g) for g in groups] for i in range(6)]
    # Fair baseline: best bijection between starts and six equal-angle areas.
    perm=min(itertools.permutations(range(6)),key=lambda pp:sum(route_cost(w,i,orders[i][pp[i]]) for i in range(6)))
    routes=[orders[i][perm[i]][:] for i in range(6)]
    if variant=='A':return routes,0
    changes=0
    # Same target set, same execution. Only initial ownership changes.
    for _ in range(80):
        loads=np.array([route_cost(w,i,r) for i,r in enumerate(routes)])
        old=float(loads.max()+0.015*loads.sum())
        best=None;bestscore=old-0.25
        for donor in np.argsort(-loads)[:2]:
            for k,task in enumerate(routes[donor]):
                reduced=routes[donor][:k]+routes[donor][k+1:]
                dc=route_cost(w,int(donor),reduced)
                for receiver in range(6):
                    if receiver==donor:continue
                    for j in range(len(routes[receiver])+1):
                        rr=routes[receiver][:j]+[task]+routes[receiver][j:]
                        rc=route_cost(w,receiver,rr)
                        ll=loads.copy();ll[donor]=dc;ll[receiver]=rc
                        score=float(ll.max()+0.015*ll.sum())
                        if score<bestscore:
                            bestscore=score;best=(int(donor),receiver,reduced,rr)
        if best is None:break
        donor,receiver,dr,rr=best;routes[donor]=dr;routes[receiver]=rr;changes+=1
    return routes,changes


@dataclass
class Segment:
    start:float
    T:float
    a:np.ndarray
    b:np.ndarray
    @property
    def end(self):return self.start+self.T
    def positions(self,ts):
        u=np.clip((np.asarray(ts)-self.start)/self.T,0,1)
        q=u**3*(10+u*(-15+6*u))
        return self.a+q[...,None]*(self.b-self.a)
    def serial(self):return [self.start,self.T,self.a.tolist(),self.b.tolist()]


def trajectory_positions(segments,ts,initial):
    ts=np.asarray(ts)
    out=np.broadcast_to(initial,(*ts.shape,3)).copy()
    for seg in segments:
        mask=ts>=seg.start
        if mask.any():out[mask]=seg.positions(ts[mask])
    return out


def make_segments(w,source,dest,start):
    segments=[]
    path=w.paths[source,dest]
    for a,b in zip(path,path[1:]):
        aa=np.r_[w.nodes[a],Z];bb=np.r_[w.nodes[b],Z]
        seg=Segment(start,float(duration(np.linalg.norm(bb-aa))),aa,bb)
        segments.append(seg);start=seg.end
    return segments


def detour_around_hovers(w,source,dest,t,histories,drone):
    """Shared execution-layer fallback, applied identically to A/B/C.

    Waiting cannot solve a route blocked by another vehicle's final hover.
    Add temporary geometric exclusions instead of waiting forever.
    """
    holds=np.array([hh[-1].b[:2] for j,hh in enumerate(histories) if j!=drone])
    key=(source,dest,tuple(np.round(holds.ravel(),4)))
    if not hasattr(w,'detour_cache'):w.detour_cache={}
    if key in w.detour_cache:return w.detour_cache[key]
    r=RESERVE_MARGIN+.05
    rects=np.c_[holds-r,holds+r]
    points=np.concatenate([np.array([[x0-.025,y0-.025],[x0-.025,y1+.025],[x1+.025,y0-.025],[x1+.025,y1+.025]]) for x0,y0,x1,y1 in rects])
    points=points[w.clear(points)&(np.abs(points)<24.95).all(axis=1)]
    nodes=np.vstack([w.nodes,points]);n=len(nodes);nold=len(w.nodes)
    edges=np.full((n,n),np.inf);edges[:nold,:nold]=w.edges
    for i in range(n):
        if i<nold:
            ends=nodes[nold:];inds=np.arange(nold,n)
        else:
            ends=nodes;inds=np.arange(n)
        good=w.clear_segments(nodes[i],ends)
        edges[i,inds[good]]=duration(np.linalg.norm(ends[good]-nodes[i],axis=1))
    for i,xy in enumerate(nodes):
        d=nodes-xy;dd=np.where(np.abs(d)<1e-12,1e-30,d)
        lo=(rects[None,:,:2]-xy)/dd[:,None,:]
        hi=(rects[None,:,2:]-xy)/dd[:,None,:]
        cross=(np.maximum(np.minimum(lo,hi).max(axis=2),0)<=np.minimum(np.maximum(lo,hi).min(axis=2),1)).any(axis=1)
        edges[i,cross]=np.inf
    dist=np.full(n,np.inf);dist[source]=0.;parent=np.full(n,-1,int);q=[(0.,source)]
    while q:
        dd,u=heapq.heappop(q)
        if u==dest:break
        if dd>dist[u]+1e-9:continue
        for v in np.flatnonzero(np.isfinite(edges[u])):
            nd=dd+edges[u,v]
            if nd<dist[v]-1e-9:
                dist[v]=nd;parent[v]=u;heapq.heappush(q,(nd,int(v)))
    if not np.isfinite(dist[dest]):
        w.detour_cache[key]=None;return None
    path=[dest]
    while path[-1]!=source:path.append(int(parent[path[-1]]))
    answer=nodes[path[::-1]]
    w.detour_cache[key]=answer
    return answer


def schedule(w,source,dest,t,histories,drone):
    base=make_segments(w,source,dest,t)
    if not base:return [],0.
    # Reserve continuous trajectories, not simply occupied destination points.
    for delay in np.arange(0,12.01,0.4):
        segs=[Segment(s.start+float(delay),s.T,s.a,s.b) for s in base]
        horizon=max([segs[-1].end]+[hh[-1].end for j,hh in enumerate(histories) if j!=drone and hh])
        ts=np.arange(t,horizon+0.05001,0.05)
        own=trajectory_positions(segs,ts,np.r_[w.nodes[source],Z])
        good=True
        for j,history in enumerate(histories):
            if j==drone:continue
            other=trajectory_positions(history,ts,STARTS[j])
            if np.min(np.linalg.norm(own-other,axis=1))<RESERVE_MARGIN:
                good=False;break
        if good:return segs,float(delay)
    points=detour_around_hovers(w,source,dest,t,histories,drone)
    if points is not None:
        alternate=[];start=t
        for a,b in zip(points,points[1:]):
            aa=np.r_[a,Z];bb=np.r_[b,Z]
            sg=Segment(start,float(duration(np.linalg.norm(bb-aa))),aa,bb)
            alternate.append(sg);start=sg.end
        for delay in np.arange(0,12.01,.4):
            segs=[Segment(s.start+float(delay),s.T,s.a,s.b) for s in alternate]
            horizon=max([segs[-1].end]+[hh[-1].end for j,hh in enumerate(histories) if j!=drone and hh])
            ts=np.arange(t,horizon+.05001,.05)
            own=trajectory_positions(segs,ts,np.r_[w.nodes[source],Z])
            if all(np.linalg.norm(own-trajectory_positions(hh,ts,STARTS[j]),axis=1).min()>=RESERVE_MARGIN for j,hh in enumerate(histories) if j!=drone):
                return segs,float(delay)
    return None,0.


def clean_queues(w,queues,seen):
    for i,q in enumerate(queues):
        queues[i]=[task for task in q if not seen[w.footprints[task-6]].all()]


def help_once(w,i,queues,nodes,endtimes,t,seen,events):
    if queues[i]:return
    loads=[max(t,endtimes[j])+route_cost(w,nodes[j],queues[j]) for j in range(6)]
    old=max(loads)
    best=None;gain=1.0
    for donor in range(6):
        if donor==i:continue
        for k,task in enumerate(queues[donor]):
            if not np.any(~seen[w.footprints[task-6]]):continue
            # Nearby work only. The helper's travel is fully charged.
            if np.linalg.norm(w.nodes[nodes[i]]-w.nodes[task])>20:continue
            remain=queues[donor][:k]+queues[donor][k+1:]
            ll=loads.copy()
            ll[donor]=max(t,endtimes[donor])+route_cost(w,nodes[donor],remain)
            ll[i]=t+w.cost[nodes[i],task]
            benefit=old-max(ll)
            if benefit>gain:
                gain=benefit;best=(donor,k,task)
    if best:
        donor,k,task=best
        del queues[donor][k];queues[i].append(task)
        events.append({'time':round(t,2),'from':donor+2,'to':i+2,'task':int(task-6),'estimated_team_seconds_saved':round(gain,3)})


def run(seed,variant,outdir,save_replay=True,timeout=600):
    wall0=time.perf_counter()
    boxes,obstacle,cloud=generate_map(seed)
    cloud_dist=np.sqrt(((STARTS[:,None,:]-cloud[None,:,:])**2).sum(axis=2)).min(axis=1)
    initial_clearance=float(cloud_dist.min())
    if initial_clearance<0.2:
        result={'seed':seed,'variant':variant,'status':'invalid_initial_state',
                'initial_obstacle_point_distance_m':initial_clearance,
                'initial_vehicle_point_distances_m':cloud_dist.tolist(),
                'reason':'At least one launch position is already within the coursework 0.2 m obstacle-point collision threshold. Map was not regenerated.'}
        (outdir/f'{seed}_{variant}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return result
    w=World(seed)
    queues,changes=initial_routes(w,variant)
    initial_plan_seconds=time.perf_counter()-wall0
    print(f'Prepared seed={seed} variant={variant} targets={len(w.targets)} initial_plan_wall_s={initial_plan_seconds:.3f}',flush=True)
    initial_queues=[q[:] for q in queues]
    predicted=[route_cost(w,i,q) for i,q in enumerate(queues)]
    takeoff=float(duration(Z-1))
    histories=[[Segment(0,takeoff,STARTS[i],np.r_[STARTS[i,:2],Z])] for i in range(6)]
    nodes=list(range(6));endtimes=[takeoff]*6
    seen=np.zeros(250000,bool);total=int(w.free.sum())
    events=[];waits=0.;t95=None;done=None;log=[];frames=[]
    assignments=0;blocked_attempts=0;no_work_since=None
    steps=int(timeout/DT)+1
    for step in range(steps):
        t=step*DT
        poses=np.array([trajectory_positions(histories[i],[t],STARTS[i])[0] for i in range(6)])
        for pos in poses:seen[footprint(pos,w.free)]=True
        count=int(seen.sum());coverage=count/total
        if step%500==0:
            print(f'Progress seed={seed} variant={variant} simulated_s={t:.1f} coverage={coverage:.6f} queued={sum(map(len,queues))} blocked={blocked_attempts}',flush=True)
        if t95 is None and coverage>=.95:t95=t
        if step%5==0:
            log.append([round(t,2),round(coverage,8)])
            frames.append([round(t,2),*poses.round(4).ravel().tolist()])
        if count==total:done=t;break
        if t<takeoff:continue
        # Identical completion-aware cancellation in all methods.
        clean_queues(w,queues,seen)
        for i in range(6):
            if t+1e-8<endtimes[i]:continue
            if variant=='C':help_once(w,i,queues,nodes,endtimes,t,seen,events)
            if not queues[i]:continue
            task=queues[i][0]
            segs,delay=schedule(w,nodes[i],task,t,histories,i)
            if segs is None:
                blocked_attempts+=1;continue
            queues[i].pop(0)
            histories[i].extend(segs)
            endtimes[i]=segs[-1].end if segs else t
            nodes[i]=task
            waits+=delay;assignments+=1
        if not any(queues) and all(t>=et for et in endtimes):
            if no_work_since is None:no_work_since=t
            if t-no_work_since>1:break
    finish=done if done is not None else t
    safety=verify(w,histories,finish)
    elapsed=time.perf_counter()-wall0
    result={'seed':seed,'variant':variant,'status':'complete' if done is not None else 'incomplete',
            'coverage':coverage,'uncovered_cells':total-count,'t95_s':t95,'t100_s':done,
            'elapsed_sim_s':round(finish,3),'world_and_initial_plan_wall_s':round(initial_plan_seconds,4),
            'experiment_wall_s':round(elapsed,4),'free_cells':total,'viewpoints':len(w.targets),
            'initial_ownership_changes':changes,'help_transfers':len(events),
            'initial_predicted_route_s':[round(v,3) for v in predicted],
            'reservation_wait_s':round(waits,3),'blocked_schedule_attempts':blocked_attempts,
            'dispatched_tasks':assignments,'safety':safety,'help_events':events}
    outdir.mkdir(parents=True,exist_ok=True)
    (outdir/f'{seed}_{variant}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    if save_replay:
        replay={'seed':seed,'variant':variant,'boxes':w.boxes.tolist(),'targets':w.targets.tolist(),
                'initial_queues':initial_queues,'curves':log,'frames':frames,'result':result,
                'segments':[[s.serial() for s in hh] for hh in histories]}
        (outdir/f'{seed}_{variant}_replay.json').write_text(json.dumps(replay,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
    return result


def verify(w,histories,finish,dt=.02):
    """Independent dense replay plus bounds; never reused by scheduling."""
    ts=np.arange(0,finish+1e-8,dt)
    if ts[-1]<finish:ts=np.r_[ts,finish]
    poses=np.stack([trajectory_positions(h,ts,STARTS[i]) for i,h in enumerate(histories)],axis=1)
    minpair=float('inf')
    for i in range(6):
        for j in range(i):minpair=min(minpair,float(np.linalg.norm(poses[:,i]-poses[:,j],axis=1).min()))
    minobs=float('inf')
    for i in range(6):
        minobs=min(minobs,float(box_dist(poses[:,i],w.boxes).min()))
    vmax=amax=energy=distance=0.
    for hh in histories:
        for seg in hh:
            if seg.start>=finish:continue
            L=float(np.linalg.norm(seg.b-seg.a))
            vmax=max(vmax,1.875*L/seg.T)
            amax=max(amax,(10/math.sqrt(3))*L/seg.T**2)
            # Integrate only the flown part, not cancelled future reservations.
            u=min(1.,(finish-seg.start)/seg.T)
            integral=120**2*u**7/7-2*120*180*u**6/6+(180**2+2*120*60)*u**5/5-2*180*60*u**4/4+60**2*u**3/3
            energy+=L**2/seg.T**3*integral
            distance+=L*(10*u**3-15*u**4+6*u**5)
    # Any point is <=dt/2 from a replay sample; relative speed <=2*vmax.
    pair_bound=minpair-vmax*dt
    obs_bound=minobs-vmax*dt/2
    height=float(poses[:,:,2].max())
    safe=pair_bound>=.4 and obs_bound>=.2 and vmax<=6+1e-8 and amax<=3+1e-8 and height<=8
    return {'pass':bool(safe),'replay_dt_s':dt,'min_pair_sample_m':round(minpair,6),
            'min_pair_certified_lower_bound_m':round(pair_bound,6),
            'min_solid_obstacle_sample_m':round(minobs,6),
            'min_obstacle_certified_lower_bound_m':round(obs_bound,6),
            'analytic_max_speed_mps':round(vmax,6),'analytic_max_acceleration_mps2':round(amax,6),
            'max_height_m':height,'flown_distance_m':round(distance,3),
            'acceleration_squared_integral_proxy':round(energy,3)}


def self_check():
    rng=LibstdRandom(1)
    assert [rng.next() for _ in range(3)]==[16807,282475249,1622650073]
    assert abs(LibstdRandom(1).uniform(0,1)-0.13153778773876065)<1e-15
    T=float(duration(10))
    seg=Segment(0,T,np.zeros(3),np.array([10,0,0]))
    ts=np.linspace(0,T,10001);pp=seg.positions(ts)
    v=np.gradient(pp,ts,axis=0);a=np.gradient(v,ts,axis=0)
    assert np.linalg.norm(v,axis=1).max()<=VMAX+1e-4
    assert np.linalg.norm(a,axis=1).max()<=AMAX+1e-3
    assert np.allclose(pp[0],seg.a) and np.allclose(pp[-1],seg.b)
    # Independent safety checker must reject a deliberate mid-flight crossing.
    class Blank: boxes=np.array([[100,100,0,101,101,1.]])
    hist=[[Segment(0,T,np.array([-5,0,Z]),np.array([5,0,Z]))],
          [Segment(0,T,np.array([0,-5,Z]),np.array([0,5,Z]))]]
    for i in range(2,6):
        pos=np.array([40+i*2,40,Z]);hist.append([Segment(0,T,pos,pos)])
    assert not verify(Blank(),hist,T)['pass']
    print('Self checks passed: RNG fixtures, trajectory limits, collision negative control.',flush=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--seeds',nargs='+',type=int,default=[511])
    ap.add_argument('--variants',nargs='+',default=['A','B','C'])
    ap.add_argument('--out',type=Path,default=ROOT/'results'/'development')
    ap.add_argument('--self-check',action='store_true')
    args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    if args.self_check:self_check()
    for seed in args.seeds:
        for variant in args.variants:
            try:
                result=run(seed,variant,args.out)
            except Exception as error:
                result={'seed':seed,'variant':variant,'status':'error','error':f'{type(error).__name__}: {error}'}
                (args.out/f'{seed}_{variant}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
            print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()

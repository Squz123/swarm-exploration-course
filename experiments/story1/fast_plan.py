"""Continuous trajectories, explicit time/acceleration-energy cost, and preflight checks."""
from dataclasses import dataclass
import itertools
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline
import benchmark as b
CURVE_MODE='zero_acc'
TAKEOFF_BUFFER=0.
ENTRY_AMAX=float('inf')


@dataclass
class Polynomial:
    start: float
    T: float
    coefficients: np.ndarray  # six ascending powers of normalized time, xyz
    @property
    def end(self):return self.start+self.T
    def at(self,t,derivative=0):
        u=np.clip((np.asarray(t)-self.start)/self.T,0,1)
        c=self.coefficients.copy()
        for _ in range(derivative):c=c[1:]*np.arange(1,len(c))[:,None]/self.T
        return np.polynomial.polynomial.polyval(u,c).T
    def serial(self):return [self.start,self.T,self.coefficients.tolist()]


def coefficient(p,q,v,w,T,a0=None,a1=None):
    d=q-p;v=v*T;w=w*T
    aa=np.zeros(3) if a0 is None else a0*T*T
    bb=np.zeros(3) if a1 is None else a1*T*T
    return np.array([p,v,aa/2,10*d-6*v-4*w-1.5*aa+.5*bb,-15*d+8*v+7*w+1.5*aa-bb,6*d-3*v-3*w-.5*aa+.5*bb])


def exact_max_norm(c):
    norm2=np.zeros(1)
    for i in range(3):norm2=np.polynomial.polynomial.polyadd(norm2,np.polynomial.polynomial.polymul(c[:,i],c[:,i]))
    derivative=np.polynomial.polynomial.polyder(norm2)
    roots=np.polynomial.polynomial.polyroots(np.trim_zeros(derivative,'b')) if np.any(derivative) else []
    ts=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<r.real<1]
    return math.sqrt(max(0,float(np.max(np.polynomial.polynomial.polyval(ts,norm2)))))


def limits(segment):
    c=segment.coefficients
    v=c[1:]*np.arange(1,6)[:,None]/segment.T
    a=v[1:]*np.arange(1,5)[:,None]/segment.T
    return exact_max_norm(v),exact_max_norm(a)


def energy(segment):
    c=segment.coefficients[2:]*np.array([2,6,12,20])[:,None]/segment.T**2
    squared=np.zeros(1)
    for i in range(3):squared=np.polynomial.polynomial.polyadd(squared,np.polynomial.polynomial.polymul(c[:,i],c[:,i]))
    return float(np.dot(squared,1/np.arange(1,len(squared)+1))*segment.T)


def sample(history,ts):
    ts=np.asarray(ts);out=np.zeros((len(ts),3))
    indices=np.searchsorted([s.start for s in history],ts,side='right')-1
    for i,s in enumerate(history):
        mask=indices==i
        if mask.any():out[mask]=s.at(ts[mask])
    out[indices<0]=history[0].at(0)
    return out


def simplify(points):
    result=[]
    for p in points:
        if result and np.linalg.norm(p-result[-1])<1e-6:continue
        result.append(np.asarray(p,float))
        while len(result)>2:
            a=result[-2]-result[-3];d=result[-1]-result[-2]
            if np.dot(a,d)>0 and np.linalg.norm(np.cross(a,d))<1e-7:result.pop(-2)
            else:break
    return np.array(result)


def smooth(points,vmax,amax,tree,alpha=1.0):
    points=simplify(points)
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
    durations=np.maximum(lengths/vmax,.15)
    blend=np.full(len(points),alpha);blend[[0,-1]]=0
    for iteration in range(35):
        velocity=np.zeros_like(points)
        velocity[1:-1]=(points[2:]-points[:-2])/(durations[:-1]+durations[1:])[:,None]*blend[1:-1,None]
        acceleration=np.zeros_like(points)
        if CURVE_MODE=='cubic_energy':
            knots=np.r_[0,np.cumsum(durations)]
            spline=CubicSpline(knots,points,bc_type=((1,np.zeros(3)),(1,np.zeros(3))))
            velocity=spline(knots,1)*blend[:,None]
            acceleration=spline(knots,2)*blend[:,None]
            acceleration[[0,-1]]=0
        segments=[];start=0.;factors=[]
        for i,T in enumerate(durations):
            s=Polynomial(start,float(T),coefficient(points[i],points[i+1],velocity[i],velocity[i+1],T,acceleration[i],acceleration[i+1]))
            segments.append(s);start=s.end
            local_amax=min(amax,ENTRY_AMAX) if i==0 else amax
            v,a=limits(s);factors.append(max(1.,v/vmax,math.sqrt(a/local_amax)))
        factors=np.array(factors)
        if factors.max()>1.00001:
            durations*=np.where(factors>1.00001,factors*1.003,1.)
            continue
        changed=False
        for i,s in enumerate(segments):
            ts=np.linspace(s.start,s.end,max(4,int(s.T/.02)+2));ps=s.at(ts)
            clearance=tree.query(ps)[0].min()
            if clearance<.39:
                # Reduce the rounding only around this obstacle; regular
                # observation points keep a nonzero passing velocity.
                if max(blend[i],blend[i+1])<.005:
                    raise RuntimeError('Even the original straight route is too close to obstacles')
                blend[i:i+2]*=.45;changed=True
        if not changed:return segments
    raise RuntimeError('Continuous trajectory did not converge to dynamic and geometric limits')


def full_route(world,drone,route,vmax,amax,tree,alpha):
    points=[np.r_[b.STARTS[drone,:2],b.Z]];at=drone
    for target in route:
        points.extend(np.r_[world.nodes[k],b.Z] for k in world.paths[at,target][1:]);at=target
    cruise=smooth(points,vmax,amax,tree,alpha)
    T=max(1.875*(b.Z-1)/vmax,math.sqrt((10/math.sqrt(3))*(b.Z-1)/amax))
    takeoff=Polynomial(0,T,coefficient(b.STARTS[drone],np.r_[b.STARTS[drone,:2],b.Z],np.zeros(3),np.zeros(3),T))
    prefix=[takeoff]
    if TAKEOFF_BUFFER>0:
        p=np.r_[b.STARTS[drone,:2],b.Z]
        prefix.append(Polynomial(T,TAKEOFF_BUFFER,coefficient(p,p,np.zeros(3),np.zeros(3),TAKEOFF_BUFFER)))
    for s in cruise:s.start+=T+TAKEOFF_BUFFER
    return prefix+cruise


def schedule_fleet(histories,vmax):
    # Delays happen before takeoff, not at every observation point. Every
    # already-finished vehicle remains present during collision prediction.
    ts=np.arange(0,max(h[-1].end for h in histories)+12,.02)
    base=[sample(h,ts) for h in histories]
    best=None
    orders=[list(np.argsort([-h[-1].end for h in histories])),list(range(6)),list(reversed(range(6)))]
    for order in orders:
        offsets=np.zeros(6);accepted={};failed=False
        for i in order:
            for delay in np.arange(0,10.001,.1):
                ps=sample(histories[i],ts-delay)
                # The certificate uses relative speed <= 2*vmax and the
                # nearest sample at most dt/2 away: subtract vmax*dt.
                if all(np.linalg.norm(ps-other,axis=1).min()>=.72+vmax*.02 for other in accepted.values()):
                    offsets[i]=delay;accepted[i]=ps;break
            else:failed=True;break
        if failed:continue
        score=max(h[-1].end+delay for h,delay in zip(histories,offsets))
        if best is None or score<best[0]:best=(score,offsets)
    if best is None:raise RuntimeError('No safe fleet departure schedule')
    for h,delay in zip(histories,best[1]):
        for s in h:s.start+=float(delay)
    return best[1].tolist()


def certificate(histories,cloud,vmax,amax):
    tree=cKDTree(cloud);dt=.01
    ts=np.arange(0,max(h[-1].end for h in histories)+dt,dt)
    positions=np.stack([sample(h,ts) for h in histories],axis=1)
    pair=min(np.linalg.norm(positions[:,i]-positions[:,j],axis=1).min() for i in range(len(histories)) for j in range(i))
    obstacle=tree.query(positions.reshape(-1,3))[0].min()
    maxima=np.array([limits(s) for h in histories for s in h]).max(axis=0)
    lower_pair=float(pair-vmax*dt);lower_obs=float(obstacle-vmax*dt/2)
    result={'max_command_speed_exact_mps':float(maxima[0]),'max_command_acceleration_exact_mps2':float(maxima[1]),
            'min_pair_continuous_lower_bound_m':lower_pair,'min_obstacle_continuous_lower_bound_m':lower_obs,
            'max_command_height_m':float(positions[:,:,2].max()),
            'planned_finish_s':max(h[-1].end for h in histories),
            'acceleration_energy':sum(energy(s) for h in histories for s in h)}
    result['pass']=bool(maxima[0]<=vmax*1.0001 and maxima[1]<=amax*1.0001 and lower_pair>=.65 and lower_obs>=.32 and result['max_command_height_m']<8)
    if not result['pass']:raise RuntimeError('Preflight rejected: '+json.dumps(result))
    return result


def swept_coverage(histories,world,dt=.05):
    seen=np.zeros(250000,bool)
    ts=np.arange(0,max(h[-1].end for h in histories)+dt,dt)
    positions=np.stack([sample(h,ts) for h in histories],axis=1)
    for frame in positions:
        for p in frame:seen[b.footprint(p,world.free)]=True
    missing=world.free.ravel()&~seen
    return missing


def prune_routes(world,routes):
    """Remove redundant observations while preserving the swept ground area."""
    cache={};counts=np.zeros(250000,np.int16);routes=[r[:] for r in routes]
    def edge(a,d):
        key=(a,d)
        if key in cache:return cache[key]
        cells=[];radius=b.Z*.5-.23
        points=world.nodes[world.paths[a,d]]
        for p,q in zip(points,points[1:]):
            lo=np.maximum(0,np.floor((np.minimum(p,q)-radius+25)/.1-.5).astype(int))
            hi=np.minimum(500,np.ceil((np.maximum(p,q)+radius+25)/.1-.5).astype(int)+1)
            xx,yy=np.meshgrid(b.AXIS[lo[0]:hi[0]],b.AXIS[lo[1]:hi[1]])
            delta=q-p;norm=float(np.dot(delta,delta))
            if norm<1e-12:continue
            t=np.clip(((xx-p[0])*delta[0]+(yy-p[1])*delta[1])/norm,0,1)
            mask=(xx-p[0]-t*delta[0])**2+(yy-p[1]-t*delta[1])**2<=radius**2
            mask &= world.free[lo[1]:hi[1],lo[0]:hi[0]]
            y,x=np.nonzero(mask);cells.append((y+lo[1])*500+x+lo[0])
        cache[key]=np.unique(np.concatenate(cells)) if cells else np.empty(0,int)
        return cache[key]
    for i,r in enumerate(routes):
        for a,d in zip([i]+r,r):counts[edge(a,d)]+=1
    if np.any(world.free.ravel()&(counts==0)):return routes,[]
    removed=[]
    for _ in range(90):
        loads=np.array([b.route_cost(world,i,r) for i,r in enumerate(routes)])
        candidates=[]
        for i,r in enumerate(routes):
            if len(r)<3:continue
            for k,t in enumerate(r):
                a=i if k==0 else r[k-1];d=r[k+1] if k+1<len(r) else None
                gain=world.cost[a,t]+(world.cost[t,d]-world.cost[a,d] if d is not None else 0)
                ll=loads.copy();ll[i]-=gain
                priority=loads.max()-ll.max()+.04*gain
                candidates.append((priority,i,k,a,t,d))
        accepted=False
        for _,i,k,a,t,d in sorted(candidates,reverse=True):
            old1=edge(a,t);old2=edge(t,d) if d is not None else np.empty(0,int)
            new=edge(a,d) if d is not None else np.empty(0,int)
            counts[old1]-=1;counts[old2]-=1;counts[new]+=1
            affected=np.union1d(old1,old2)
            if np.all(counts[affected]>0):
                routes[i].pop(k);removed.append((i,k,t));accepted=True;break
            counts[old1]+=1;counts[old2]+=1;counts[new]-=1
        if not accepted:break
    return routes,removed


def plan(world,cloud,vmax=5.7,amax=2.7,alpha=1.,prune=False):
    tree=cKDTree(cloud)
    routes,changes=b.initial_routes(world,'B')
    removed=[]
    if prune:routes,removed=prune_routes(world,routes)
    original_removals=len(removed)
    for attempt in range(original_removals+1):
        histories=[full_route(world,i,r,vmax,amax,tree,alpha) for i,r in enumerate(routes)]
        offsets=schedule_fleet(histories,vmax)
        check=certificate(histories,cloud,vmax,amax)
        missing=swept_coverage(histories,world) if prune else None
        if missing is None or not missing.any():break
        if not removed:raise RuntimeError('Swept coverage is incomplete: '+str(missing.sum()))
        i,k,t=removed.pop();routes[i].insert(k,t)
    # Time is the main objective; squared acceleration remains an explicit
    # energy sub-objective required by the course. Used to compare candidates.
    check['time_plus_energy_cost']=check['planned_finish_s']+.0005*check['acceleration_energy']
    check['departure_delays_s']=offsets;check['ownership_changes']=changes
    check['redundant_observations_removed']=len(removed)
    if missing is not None:check['uncovered_conservative_swept_cells']=int(missing.sum())
    return histories,routes,check


def stripe_world(cloud):
    from ros_flight import LiveWorld
    ys=[-22.5,-16.5,-10.5,-5.5,0.,5.5,10.5,16.5,22.5]
    xs=[-24.4,-.6,.6,24.4]
    return LiveWorld(cloud,fill_holes=False,target_points=[(x,y) for y in ys for x in xs])


def plan_stripes(world,cloud,vmax,amax,alpha=1.):
    tree=cKDTree(cloud);routes=[]
    for i in range(6):
        side=i//3;band=i%3
        rows=[[2,1,0],[3,4,5],[6,7,8]][band]
        inner,outer=(1,0) if side==0 else (2,3)
        r=[]
        for j,row in enumerate(rows):
            cols=[inner,outer] if j%2==0 else [outer,inner]
            r.extend(6+row*4+col for col in cols)
        routes.append(r)
    added=0
    for repair in range(5):
        histories=[full_route(world,i,r,vmax,amax,tree,alpha) for i,r in enumerate(routes)]
        offsets=schedule_fleet(histories,vmax)
        check=certificate(histories,cloud,vmax,amax)
        missing=swept_coverage(histories,world)
        if not missing.any():break
        if repair==4:break
        new=[]
        remaining=missing.copy()
        xx,yy=np.meshgrid(np.arange(-24,25,2.),np.arange(-24,25,2.))
        candidates=np.c_[xx.ravel(),yy.ravel()];candidates=candidates[world.clear(candidates)]
        for _ in range(8):
            ids=np.flatnonzero(remaining)
            if not len(ids):break
            positions=np.c_[b.AXIS[ids%500],b.AXIS[ids//500]]
            cover_tree=cKDTree(positions)
            counts=cover_tree.query_ball_point(candidates,b.Z*.5-.2,return_length=True)
            p=candidates[int(np.argmax(counts))]
            if max(counts)==0:raise RuntimeError('No safe repair point can cover remaining area')
            new.append(p);remaining[b.footprint([*p,b.Z],world.free)]=False
        start=len(world.targets)+6
        world.targets=np.vstack([world.targets,new])
        world.footprints=[b.footprint([*p,b.Z],world.free) for p in world.targets];world.make_graph()
        for target in range(start,start+len(new)):
            loads=np.array([b.route_cost(world,i,r) for i,r in enumerate(routes)]);best=None
            for i,r in enumerate(routes):
                for k in range(len(r)+1):
                    rr=r[:k]+[target]+r[k:];ll=loads.copy();ll[i]=b.route_cost(world,i,rr)
                    score=ll.max()+.03*ll.sum()
                    if best is None or score<best[0]:best=(score,i,rr)
            routes[best[1]]=best[2]
        added+=len(new)
    check['uncovered_conservative_swept_cells']=int(missing.sum())
    check['time_plus_energy_cost']=check['planned_finish_s']+.0005*check['acceleration_energy']
    check['departure_delays_s']=offsets
    check['extra_gap_filling_observations']=added
    if missing.any():raise RuntimeError('Stripe swept coverage incomplete: '+str(missing.sum()))
    return histories,routes,check


def self_check():
    p=np.array([0.,0,1]);q=np.array([4.,2,1]);v=np.array([1.,0,0]);w=np.array([0.,1,0])
    s=Polynomial(2.,4.,coefficient(p,q,v,w,4.))
    assert np.allclose(s.at(2),p) and np.allclose(s.at(6),q)
    assert np.allclose(s.at(2,1),v) and np.allclose(s.at(6,1),w)
    assert np.allclose(s.at(2,2),0) and np.allclose(s.at(6,2),0)
    ts=np.linspace(2,6,10001);maxima=limits(s)
    assert np.linalg.norm(s.at(ts,1),axis=1).max()<=maxima[0]+1e-7
    assert np.linalg.norm(s.at(ts,2),axis=1).max()<=maxima[1]+1e-7
    assert max(abs(np.linalg.norm(s.at(ts,d),axis=1).max()-maxima[d-1]) for d in (1,2))<1e-5
    aa=np.array([.2,-.4,0]);bb=np.array([-.3,.1,0])
    joined=Polynomial(0,4,coefficient(p,q,v,w,4,aa,bb))
    assert np.allclose(joined.at(0,2),aa) and np.allclose(joined.at(4,2),bb)
    following=Polynomial(4,3,coefficient(q,q+np.array([2,3,0]),w,v,3,bb,aa))
    for d in range(3):assert np.allclose(joined.at(4,d),following.at(4,d))
    # Deliberately crossing trajectories must fail the independent certificate.
    crossing=[[Polynomial(0,5,coefficient(np.array([-2.,0,1]),np.array([2.,0,1]),np.zeros(3),np.zeros(3),5))],
              [Polynomial(0,5,coefficient(np.array([0.,-2,1]),np.array([0.,2,1]),np.zeros(3),np.zeros(3),5))]]
    try:certificate(crossing, np.array([[100.,100,100]]),6,3)
    except RuntimeError:pass
    else:raise AssertionError('Collision negative control accepted')
    print('Continuous polynomial boundary, exact dynamic extrema, and negative-control checks passed.')


if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--cloud',type=Path);ap.add_argument('--output',type=Path)
    ap.add_argument('--vmax',type=float,default=5.7);ap.add_argument('--amax',type=float,default=2.7)
    ap.add_argument('--grid',type=int,default=10);ap.add_argument('--alpha',type=float,default=1.)
    args=ap.parse_args();self_check()
    if args.cloud:
        from ros_flight import LiveWorld
        begin=time.monotonic();b.Z=7.85
        cloud=np.load(args.cloud);world=LiveWorld(cloud,args.grid)
        histories,routes,check=plan(world,cloud,args.vmax,args.amax,args.alpha)
        check['planning_wall_s']=time.monotonic()-begin
        print(json.dumps(check,indent=2))
        if args.output:args.output.write_text(json.dumps({'preflight':check,'histories':[[s.serial() for s in h] for h in histories],'routes':routes}))

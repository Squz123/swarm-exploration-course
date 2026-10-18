import json
import numpy as np
import benchmark as b
from recheck_coverage import raster_footprint
from pathlib import Path
data=json.loads((Path(__file__).parent/'results/development/511_C_replay.json').read_text())
_,obstacle,_=b.generate_map(511);free=~obstacle
hist=[[b.Segment(st,T,np.array(a),np.array(bb)) for st,T,a,bb in hh] for hh in data['segments']]
seen=np.zeros(250000,bool);large=seen.copy()
finish=data['result']['elapsed_sim_s']
ts=np.arange(0,finish+.0001,.1)
poses=np.stack([b.trajectory_positions(h,ts,b.STARTS[i]) for i,h in enumerate(hist)],axis=1)
bad=0
for t,frame in zip(ts,poses):
    for p in frame:
        small=b.footprint(p,free); big=raster_footprint(p)
        seen[small]=True;large[big]=True
        if not np.isin(small,big).all():
            if bad<3:print('local disagreement',t,p,len(small),len(big),np.count_nonzero(~np.isin(small,big)))
            bad+=1
print('conservative',int(seen.sum()),'of',int(free.sum()),'source style',np.count_nonzero(large&free.ravel()),'badlocal',bad)
print('uncovered xy',np.c_[b.AXIS[np.flatnonzero(free.ravel()&~large)%500],b.AXIS[np.flatnonzero(free.ravel()&~large)//500]][:30])

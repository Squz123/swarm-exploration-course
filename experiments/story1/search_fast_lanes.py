"""Search fewer, wider-spaced passes; verify swept coverage, retain all failures."""
import json
import time
from pathlib import Path
import numpy as np
from scipy import ndimage
import benchmark as b
from ros_flight import LiveWorld
import fast_plan as fp

ROOT=Path(__file__).resolve().parent
cloud=np.load(ROOT/'ros_results/20261018_B_v2_smooth_trial1/actual_map.npy')
b.Z=7.97
results=[]
for nx,ny in [(8,7),(9,7),(8,8),(9,8),(9,9),(10,8)]:
    for alpha in [1.,1.25]:
        begin=time.monotonic();row={'columns':nx,'rows':ny,'alpha':alpha,'altitude':b.Z,'vmax':5.97,'amax':2.52,'extent':[24.4,21.9]}
        try:
            world=LiveWorld(cloud,(nx,ny),False,(24.4,21.9))
            for repair in range(3):
                h,r,c=fp.plan(world,cloud,5.97,2.52,alpha)
                missing=fp.swept_coverage(h,world)
                if not missing.any():break
                labels,n=ndimage.label(missing.reshape(500,500),np.ones((3,3)))
                counts=np.bincount(labels.ravel());counts[0]=0
                new=[]
                for k in np.argsort(-counts)[:min(n,12)]:
                    yy,xx=np.nonzero(labels==k)
                    p=world.safe_near(np.array([b.AXIS[int(np.median(xx))],b.AXIS[int(np.median(yy))]]))
                    new.append(p)
                world.targets=np.vstack([world.targets,new]);world.footprints=[b.footprint([*p,b.Z],world.free) for p in world.targets];world.make_graph()
            row.update(c);row['uncovered_conservative_cells']=int(missing.sum());row['viewpoints']=len(world.targets)
            row['pass']=row['pass'] and not missing.any()
            row['planning_wall_s']=time.monotonic()-begin
            if row['pass']:
                (ROOT/f'fast_lanes_{nx}_{ny}_{alpha}.json').write_text(json.dumps({'candidate':row,'polynomials':[[s.serial() for s in hh] for hh in h],'routes':r,'targets':world.targets.tolist()}))
        except Exception as exc:row.update({'pass':False,'rejection':str(exc),'planning_wall_s':time.monotonic()-begin})
        results.append(row)
        (ROOT/'fast_lanes_candidates.json').write_text(json.dumps({'scope':'Offline only, not ROS flight results','candidates':results},indent=2))
        print(json.dumps(row),flush=True)
valid=[r for r in results if r['pass']]
if valid:print('BEST '+json.dumps(min(valid,key=lambda r:r['time_plus_energy_cost']+r['planning_wall_s'])),flush=True)

import json
from pathlib import Path
import time
import numpy as np
import benchmark as b
from ros_flight import LiveWorld
import fast_plan as fp
ROOT=Path(__file__).resolve().parent
cloud=np.load(ROOT/'ros_results/20261018_B_v2_smooth_trial1/actual_map.npy')
b.Z=7.97;fp.CURVE_MODE='cubic_energy';world=LiveWorld(cloud,10);results=[]
fp.self_check()
for alpha in [.7,1.,1.15]:
    begin=time.monotonic();row={'alpha':alpha,'vmax':5.98,'amax':2.82,'altitude':b.Z,'curve':fp.CURVE_MODE}
    try:
        h,r,c=fp.plan(world,cloud,5.98,2.82,alpha)
        missing=fp.swept_coverage(h,world)
        row.update(c);row['uncovered_conservative_cells']=int(missing.sum());row['planning_wall_s']=time.monotonic()-begin
        row['pass']=row['pass'] and not missing.any()
    except Exception as exc:row.update({'pass':False,'rejection':str(exc)})
    results.append(row);print(json.dumps(row),flush=True)
(ROOT/'fast_curve_candidates.json').write_text(json.dumps(results,indent=2))

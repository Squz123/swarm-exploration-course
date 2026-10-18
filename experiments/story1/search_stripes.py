import json
import time
from pathlib import Path
import numpy as np
import benchmark as b
import fast_plan as fp
ROOT=Path(__file__).resolve().parent
cloud=np.load(ROOT/'ros_results/20261018_B_v2_smooth_trial1/actual_map.npy');b.Z=7.97
world=fp.stripe_world(cloud);results=[]
for alpha in [.7,1.,1.25,1.5]:
    before=time.monotonic();row={'alpha':alpha,'vmax':5.98,'amax':2.52,'altitude':7.97}
    try:
        h,r,c=fp.plan_stripes(world,cloud,5.98,2.52,alpha)
        row.update(c);row['planning_wall_s']=time.monotonic()-before
        (ROOT/f'fast_stripes_{alpha}.json').write_text(json.dumps({'candidate':row,'routes':r,'polynomials':[[s.serial() for s in hh] for hh in h]}))
    except Exception as exc:row.update({'pass':False,'rejection':str(exc),'planning_wall_s':time.monotonic()-before})
    results.append(row);print(json.dumps(row),flush=True)
(ROOT/'fast_stripe_candidates.json').write_text(json.dumps(results,indent=2))

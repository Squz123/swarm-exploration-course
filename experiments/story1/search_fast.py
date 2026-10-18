"""Offline candidate selection on a saved actual map; every rejection is retained."""
import json
from pathlib import Path
import time
import numpy as np
import benchmark as b
from ros_flight import LiveWorld
from fast_plan import plan

ROOT=Path(__file__).resolve().parent
cloud=np.load(ROOT/'ros_results/20261018_B_v2_smooth_trial1/actual_map.npy')
b.Z=7.85
rows=[];begin=time.monotonic()
for grid in [8,9,10,11]:
    world=LiveWorld(cloud,grid)
    for alpha in [.7,1.,1.25]:
        row={'target_grid':grid,'alpha':alpha,'vmax':5.92,'amax':2.93,'actual_map_points':len(cloud)}
        try:
            histories,routes,check=plan(world,cloud,5.92,2.93,alpha)
            row.update(check)
        except Exception as exc:row.update({'pass':False,'rejection':str(exc)})
        rows.append(row)
        (ROOT/'fast_candidates.json').write_text(json.dumps({'scope':'Offline planning only, not flight results','candidates':rows},indent=2))
        print(json.dumps(row),flush=True)
best=min((r for r in rows if r['pass']),key=lambda r:r['time_plus_energy_cost'])
print('BEST '+json.dumps(best),flush=True)
print('SEARCH_SECONDS '+str(time.monotonic()-begin),flush=True)

import json
from pathlib import Path
import numpy as np
import benchmark as b

root=Path(__file__).resolve().parent/'ros_results'
for name in ['20261018_B_trial3','20261018_B_trial4']:
    data=json.loads((root/name/'flight_trace.json').read_text())
    frames=np.array(data['frames']);ts=frames[:,0];actual=frames[:,2:].reshape(-1,6,3)
    histories=[[b.Segment(st,T,np.array(a),np.array(bb)) for st,T,a,bb in h] for h in data['segments']]
    reference=np.stack([b.trajectory_positions(h,ts,b.STARTS[i]) for i,h in enumerate(histories)],axis=1)
    errors=np.linalg.norm(actual-reference,axis=2)
    k,i=np.unravel_index(np.argmax(errors),errors.shape)
    print(name,'peak tracking time',ts[k],'uav',i+2,'error',errors[k,i],'maximum monitor gap',np.diff(ts).max())
    print('first task starts',[(j+2,h[1].start) for j,h in enumerate(histories)])
    print('largest gaps',sorted(zip(np.diff(ts),ts[:-1]),reverse=True)[:5])

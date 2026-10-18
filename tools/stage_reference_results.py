"""Stage only selected historical evidence, excluding raw ROS logs and caches."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
assets=[]
for directory in ['results','ros_results','fast_results']:
    for p in (ROOT/'experiments/story1'/directory).rglob('*'):
        if any(q in ('roslogs','__pycache__') for q in p.relative_to(ROOT).parts):continue
        if not p.is_file():continue
        keep=p.suffix in ('.json','.html','.md','.csv')
        if p.name=='last_teacher_grid.npy':keep=True
        if p.name=='actual_map.npy' and p.parent.name in ['20261018_B_v2_smooth_trial1','20261018_fast_v3_trial7_repeat']:keep=True
        if keep:assets.append(p.relative_to(ROOT).as_posix())
subprocess.run(['git','-c','safe.directory='+ROOT.as_posix(),'-C',str(ROOT),
    '--literal-pathspecs','add','-f','--pathspec-from-file=-','--pathspec-file-nul'],
    input=b'\0'.join(p.encode() for p in assets)+b'\0',check=True)
print('Staged curated evidence files:',len(assets))

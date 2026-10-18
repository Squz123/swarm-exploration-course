"""Check the actual Git index before publication; never prints matched secrets."""
from pathlib import Path
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
def git(*args):return subprocess.check_output(['git','-c','safe.directory='+ROOT.as_posix(),'-C',str(ROOT),*args])
patterns=[rb'-----BEGIN (?:OPENSSH|RSA|EC|DSA) PRIVATE KEY-----',
          rb'gh[pousr]_[A-Za-z0-9]{25,}',rb'github_pat_[A-Za-z0-9_]{35,}',
          rb'AKIA[0-9A-Z]{16}']
files=git('ls-files','-z').decode().split('\0');errors=[];count=0;total=0
for name in files:
    if not name or name=='swarm_intelligent_robot_course_task':continue
    if any(part in ('.runtime','roslogs','downloads','__pycache__') for part in Path(name).parts):errors.append('Unexpected generated directory: '+name)
    if name in ['local-settings.json','.publication-staging'] or name.endswith(('.vhdx','.msi','.log')):errors.append('Unexpected local artifact: '+name)
    data=git('show',':'+name);count+=1;total+=len(data)
    if len(data)>10*1024*1024:errors.append('Unexpected file over 10 MiB: '+name)
    if any(re.search(pattern,data) for pattern in patterns):errors.append('Possible secret in '+name+' (value not displayed)')
for match in re.finditer(r'\]\(([^)]+)\)',(ROOT/'README.md').read_text(encoding='utf-8')):
    link=match.group(1)
    if '://' not in link and not link.startswith('#') and not (ROOT/link.split('#')[0]).exists():errors.append('README link missing: '+link)
for name in ['common.sh','build_course.sh','install_ros_noetic.sh','run_trial.sh','run_fast.sh','prepare_display.py']:
    text=(ROOT/'setup/ros'/name).read_text(encoding='utf-8')
    if '/mnt/d/gpt-workspace/robots' in text or '/opt/robotics-course/ws' in text:errors.append('Machine-specific runtime path: '+name)
entry=git('ls-files','--stage','swarm_intelligent_robot_course_task').decode().split()
if entry[:2]!=['160000','d8360a9dc82405d88ebae89e2a8c9429ece579c9']:errors.append('Course submodule pin is incorrect')
if errors:raise SystemExit('\n'.join(errors))
print(json.dumps({'tracked_files_checked':count,'tracked_file_bytes':total,'course_submodule_pinned':True,'secret_and_artifact_checks':'pass','readme_links':'pass'},indent=2))

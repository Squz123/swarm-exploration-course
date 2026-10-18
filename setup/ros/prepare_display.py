"""Create display-only overrides; verify every original task element is preserved."""
from pathlib import Path
import copy
import json
import xml.etree.ElementTree as ET
import yaml
import os

ROOT = Path(os.environ.get('COURSE_ROOT',Path(__file__).resolve().parents[2]))
HERE = Path(os.environ.get('COURSE_DISPLAY_DIR',ROOT/'.runtime/display'))
HERE.mkdir(parents=True,exist_ok=True)
PACKAGE = ROOT/'swarm_intelligent_robot_course_task/src/Swarm-Simulator/src/uav_simulator/so3_quadrotor_simulator'
original = yaml.safe_load((PACKAGE/'config/rviz.rviz').read_text())
probe = {'Class':'course_story1/RenderStats', 'Name':'Actual frame rate', 'Enabled':True, 'Value':True}
baseline = copy.deepcopy(original)
baseline['Visualization Manager']['Displays'].append(probe)
baseline['Preferences']['PromptSaveOnExit'] = False
(HERE/'original_probe.rviz').write_text(yaml.safe_dump(baseline, sort_keys=False))
config = copy.deepcopy(original)
manager = config['Visualization Manager']
keep = []
for display in manager['Displays']:
    kind = display['Class']
    topic = display.get('Topic', display.get('Marker Topic', ''))
    if kind in ('rviz/Axes', 'rviz/Grid'):
        if kind == 'rviz/Grid':
            display['Plane Cell Count'] = 50
        keep.append(display)
    elif kind == 'rviz/PointCloud2' and topic == '/mock_map':
        display['Queue Size'] = 1
        display['Selectable'] = False
        display['Style'] = 'Points'
        display['Size (Pixels)'] = 3
        keep.append(display)
    elif kind == 'rviz/Map' and topic == '/exploration_grid':
        display['Topic'] = '/course_display' + topic
        keep.append(display)
    elif kind == 'rviz/Marker' and (topic == '/collision_markers' or
            any(topic == '/uav%d/%s' % (i, suffix) for i in range(2,8)
                for suffix in ('odom_visualization/robot', 'fov_marker'))):
        if topic != '/collision_markers':
            display['Marker Topic'] = '/course_display' + topic
        display['Queue Size'] = 1
        keep.append(display)
manager['Displays'] = keep + [probe]
manager['Global Options']['Frame Rate'] = 60
manager['Tools'] = [t for t in manager['Tools'] if t['Class'].startswith('rviz/')]
config['Preferences']['PromptSaveOnExit'] = False
(HERE/'smooth.rviz').write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))

source = ET.parse(PACKAGE/'launch/task3.launch')
baseline_launch = copy.deepcopy(source)
baseline_launch.getroot().find("node[@name='rviz']").set('args', '-d '+str(HERE/'original_probe.rviz'))
baseline_launch.write(HERE/'task3_display_probe.launch', encoding='unicode')
target = copy.deepcopy(source)
rviz = target.getroot().find("node[@name='rviz']")
rviz.set('args', '-d ' + str(HERE/'smooth.rviz'))
check = copy.deepcopy(target)
check.getroot().find("node[@name='rviz']").set('args', source.getroot().find("node[@name='rviz']").get('args'))
assert ET.tostring(check.getroot()) == ET.tostring(source.getroot()), 'Unexpected task modification'
gui=os.environ.get('COURSE_GUI','1')!='0'
if not gui:target.getroot().remove(rviz)
ET.SubElement(target.getroot(), 'node', {'name':'course_display_relay', 'pkg':'course_story1', 'type':'display_relay'})
target.write(HERE/'task3_smooth.launch', encoding='unicode')
(HERE/'display_override.json').write_text(json.dumps({
    'only_original_launch_change':'RViz configuration path',
    'added_node':'display-only message relay',
    'original_task_elements_match':True,
    'original_displays':len(original['Visualization Manager']['Displays']),
    'active_displays':len(keep), 'requested_render_fps':60,
    'display_grid_hz':5, 'display_robot_max_hz':60, 'display_fov_max_hz':30,
    'cloud_style':'Points; all original points retained',
    'flight_and_teacher_topics_unchanged':True,'rviz_enabled':gui}, indent=2))
print((HERE/'display_override.json').read_text())

#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo 'ROS Noetic is missing. Run setup/ros/install_ros_noetic.sh first.' >&2; exit 2
fi
if [ ! -f "$ROOT/swarm_intelligent_robot_course_task/src/Swarm-Simulator/src/uav_simulator/so3_quadrotor_simulator/launch/task3.launch" ]; then
  echo 'Course submodule is missing. Run git submodule update --init --recursive.' >&2; exit 2
fi
exec > >(tee -a "$ROOT/setup/ros/logs/build-course.log") 2>&1
mkdir -p "$WS/src"
/usr/bin/python3 - "$ROOT" "$WS/src" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
root,dest=map(Path,sys.argv[1:])
def link_package(name,source):
    link=dest/name
    if link.is_symlink():
        if link.resolve()!=source.resolve():raise RuntimeError('Workspace belongs to a different checkout: choose another COURSE_WS')
    elif link.exists():raise RuntimeError('Refusing to overwrite existing workspace package: '+str(link))
    else:link.symlink_to(source,target_is_directory=True)
for manifest in (root/'swarm_intelligent_robot_course_task/src').rglob('package.xml'):
    name=ET.parse(manifest).getroot().findtext('name').strip()
    link_package(name,manifest.parent)
link_package('course_story1',root/'experiments/story1/ros_package')
PY
set +u
source /opt/ros/noetic/setup.bash
set -u
cd "$WS"
catkin_make -j"${COURSE_BUILD_JOBS:-6}" -l"${COURSE_BUILD_JOBS:-6}" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_WHITELIST_PACKAGES='cmake_utils;quadrotor_msgs;uav_utils;pose_utils;so3_quadrotor_simulator;so3_control;odom_visualization;mockamap;fov_visualization;rule_check;course_story1'
printf '\nCOURSE_BUILD_COMPLETE\n'

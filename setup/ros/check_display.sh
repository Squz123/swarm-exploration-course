#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
OUT="$ROOT/setup/ros/logs/${2:-display-check-with-fps}"
mkdir -p "$OUT"
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
PORT=${COURSE_ROS_PORT:-11321}
export ROS_MASTER_URI=http://127.0.0.1:$PORT ROS_IP=127.0.0.1
export ROS_LOG_DIR="$OUT/roslogs"
/usr/bin/python3 "$ROOT/setup/ros/prepare_display.py" > "$OUT/display-settings.json"
LAUNCH_PID=''; RVIZ_PID=''; RELAY_PID=''
cleanup() {
  trap - EXIT INT TERM
  for pid in "$RVIZ_PID" "$RELAY_PID"; do
    if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; fi
  done
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
if [ "${1:-both}" = smooth_only ]; then
  setsid roslaunch -p "$PORT" "$DISPLAY_DIR/task3_smooth.launch" > "$OUT/smooth-launch.log" 2>&1 &
  LAUNCH_PID=$!
  sleep 10
  python3 "$ROOT/setup/ros/measure_display.py" "$OUT/smooth.json" 15 smooth_points
  exit
fi
setsid roslaunch -p "$PORT" "$DISPLAY_DIR/task3_display_probe.launch" > "$OUT/original.log" 2>&1 &
LAUNCH_PID=$!
sleep 10
python3 "$ROOT/setup/ros/measure_display.py" "$OUT/original.json" 15 original
rosnode kill /rviz
sleep 3
rosrun course_story1 display_relay > "$OUT/relay.log" 2>&1 &
RELAY_PID=$!
rosrun rviz rviz -d "$DISPLAY_DIR/smooth.rviz" > "$OUT/smooth.log" 2>&1 &
RVIZ_PID=$!
sleep 8
python3 "$ROOT/setup/ros/measure_display.py" "$OUT/smooth.json" 15 smooth

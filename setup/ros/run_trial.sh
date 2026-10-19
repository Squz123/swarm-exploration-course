#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
VARIANT=${1:-B}
NAME=${2:-$(date +%Y%m%d_%H%M%S)_$VARIANT}
PLANNER=${COURSE_PLANNER:-stop}
RESULT_FOLDER=ros_results
if [ "$PLANNER" = fast ]; then RESULT_FOLDER=fast_results; fi
if [ "$PLANNER" = online ]; then RESULT_FOLDER=online_results; fi
OUT="$ROOT/experiments/story1/$RESULT_FOLDER/$NAME"
mkdir -p "$OUT"
if [ ! -f "$WS/devel/setup.bash" ]; then
  echo 'Build workspace first: bash setup/ros/build_course.sh' >&2; exit 2
fi
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
PORT=${COURSE_ROS_PORT:-11321}
export ROS_MASTER_URI=http://127.0.0.1:$PORT
export ROS_IP=127.0.0.1
export ROS_LOG_DIR="$OUT/roslogs"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONUNBUFFERED=1
if [ -n "${COURSE_GPU_ADAPTER:-}" ]; then export MESA_D3D12_DEFAULT_ADAPTER_NAME="$COURSE_GPU_ADAPTER"; fi
export COURSE_DISPLAY=${COURSE_DISPLAY:-smooth}
LAUNCH_PID=''
SERVER_PID=''
PROBE_PID=''
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$PROBE_PID" ]; then
    kill -TERM "$PROBE_PID" 2>/dev/null || true
    wait "$PROBE_PID" 2>/dev/null || true
  fi
  if [ -n "$SERVER_PID" ] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill -TERM "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for attempt in $(seq 1 100); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
      sleep .1
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then kill -TERM "$LAUNCH_PID" 2>/dev/null || true; fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
if [ "$COURSE_DISPLAY" = smooth ]; then
  /usr/bin/python3 "$ROOT/setup/ros/prepare_display.py" > "$OUT/display-settings.json"
  LAUNCH_FILE="$DISPLAY_DIR/task3_smooth.launch"
elif [ "$COURSE_DISPLAY" = original ]; then
  LAUNCH_FILE="$(rospack find so3_quadrotor_simulator)/launch/task3.launch"
else
  echo 'COURSE_DISPLAY must be original or smooth' >&2
  exit 2
fi
/usr/bin/python3 - <<'PY'
import rosgraph
try:rosgraph.Master('/course_port_check').getPid()
except Exception:pass
else:raise SystemExit('ROS port already in use; choose a different COURSE_ROS_PORT. Existing processes were not changed.')
PY
setsid roslaunch -p "$PORT" "$LAUNCH_FILE" > "$OUT/original-launch.log" 2>&1 &
LAUNCH_PID=$!
printf '%s\n' "$LAUNCH_PID" > "$OUT/launch.pid"
# rospy applies private command-line parameters before its usual retry loop,
# so explicitly wait for the freshly started master rather than racing it.
/usr/bin/python3 - <<'PY'
import time
import rosgraph
deadline=time.monotonic()+30
while time.monotonic()<deadline:
    try:
        rosgraph.Master('/course_startup_probe').getPid()
        break
    except Exception:
        time.sleep(.1)
else:
    raise RuntimeError('Course ROS master did not become ready')
PY
SERVER=trajectory_server
SCRIPT=ros_flight.py
PROBE_DELAY=30
PROBE_DURATION=30
EXTRA_ARGS=()
if [ "$PLANNER" = fast ]; then
  SERVER=fast_trajectory_server
  SCRIPT=fast_flight.py
  PROBE_DELAY=5
  PROBE_DURATION=20
  EXTRA_ARGS=( _coverage_topic:=/course_display/exploration_grid _vmax:=${COURSE_VMAX:-5.7} _amax:=${COURSE_AMAX:-2.7} _altitude:=${COURSE_ALTITUDE:-7.85} _target_grid:=${COURSE_TARGET_GRID:-10} _alpha:=${COURSE_ALPHA:-1.0} _prune:=${COURSE_PRUNE:-false} _curve:=${COURSE_CURVE:-zero_acc} _takeoff_buffer:=${COURSE_TAKEOFF_BUFFER:-0.0} _entry_amax:=${COURSE_ENTRY_AMAX:-${COURSE_AMAX:-2.7}} )
fi
if [ "$PLANNER" = online ]; then
  SERVER=online_trajectory_server
  SCRIPT=online_flight.py
  PROBE_DELAY=5
  PROBE_DURATION=20
  EXTRA_ARGS=( _coverage_topic:=/course_display/exploration_grid _pause_uav:=${COURSE_ONLINE_PAUSE_UAV:--1} _pause_at:=${COURSE_ONLINE_PAUSE_AT:-25} _pause_for:=${COURSE_ONLINE_PAUSE_FOR:-25} _drop_bid_round:=${COURSE_ONLINE_DROP_ROUND:--1} _max_seconds:=${COURSE_ONLINE_TIMEOUT:-600} )
fi
rosrun course_story1 "$SERVER" > "$OUT/trajectory-server.log" 2>&1 &
SERVER_PID=$!
/usr/bin/python3 "$ROOT/setup/ros/measure_display.py" "$OUT/display-performance.json" "$PROBE_DURATION" "$COURSE_DISPLAY" "$PROBE_DELAY" > "$OUT/display-performance.log" 2>&1 &
PROBE_PID=$!
/usr/bin/python3 -u "$ROOT/experiments/story1/$SCRIPT" \
  _variant:="$VARIANT" _time_scale:=1.35 _output:="$OUT" "${EXTRA_ARGS[@]}" > "$OUT/planner.log" 2>&1
cat "$OUT/result.json"

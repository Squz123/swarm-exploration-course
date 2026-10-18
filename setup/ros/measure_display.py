"""Measure process CPU and received display topic rate; does not claim measured FPS."""
from pathlib import Path
import json
import os
import sys
import time
import rospy
from std_msgs.msg import Float64
import statistics
from collections import defaultdict

out, seconds, mode = Path(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
counts = defaultdict(int)
sizes = defaultdict(int)
fps = []
def callback(msg, topic):
    counts[topic] += 1
    sizes[topic] += len(msg._buff)
def processes():
    result = {}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            name = (p/'comm').read_text().strip()
            if name not in ('rviz','region_visualiz','display_relay','fov_visualizer'):continue
            stat = (p/'stat').read_text().split(') ',1)[1].split()
            result[p.name] = (name, int(stat[11])+int(stat[12]))
        except (OSError, ValueError):pass
    return result
rospy.init_node('course_display_measure', anonymous=True)
topics = ['/exploration_grid','/course_display/exploration_grid','/uav2/odom_visualization/robot',
          '/course_display/uav2/odom_visualization/robot']
subs = [rospy.Subscriber(t, rospy.AnyMsg, callback, callback_args=t, queue_size=1) for t in topics]
subs.append(rospy.Subscriber('/course_display/render_fps', Float64, lambda msg:fps.append(msg.data), queue_size=1))
time.sleep(float(sys.argv[4]) if len(sys.argv)>4 else 2)
counts.clear();sizes.clear()
fps.clear()
before=processes(); start=time.monotonic()
time.sleep(seconds)
elapsed=time.monotonic()-start; after=processes(); hz=os.sysconf('SC_CLK_TCK')
data={'display_mode':mode,'sample_seconds':elapsed,'cpu_percent_of_one_core':{},
      'topics':{t:{'received_hz':counts[t]/elapsed,'received_MB_per_s':sizes[t]/elapsed/1e6} for t in topics},
      'render_fps_measured':bool(fps),
      'actual_render_fps':{'mean':statistics.mean(fps),'min':min(fps),'max':max(fps),'samples':len(fps)} if fps else None}
for pid,(name,ticks) in after.items():
    if pid in before:data['cpu_percent_of_one_core'][name]=100*(ticks-before[pid][1])/hz/elapsed
out.write_text(json.dumps(data,indent=2)); print(json.dumps(data,indent=2))

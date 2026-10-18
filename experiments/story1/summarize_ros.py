"""Summarize real ROS runs separately from the historical simplified tests."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'ros_results'


def main():
    trials=[]
    for folder in sorted(OUT.iterdir(), key=lambda p:(p/'result.json').stat().st_mtime if (p/'result.json').exists() else p.stat().st_mtime):
        if not folder.is_dir():continue
        path=folder/'result.json'
        if path.exists():
            result=json.loads(path.read_text(encoding='utf-8'))
            result['run']=folder.name;trials.append(result)
        elif (folder/'error.json').exists():
            trials.append({'run':folder.name,'status':'initialization_failed',
                           'details':json.loads((folder/'error.json').read_text(encoding='utf-8'))})
        elif (folder/'planner.log').exists():
            text=(folder/'planner.log').read_text(encoding='utf-8',errors='replace')
            trials.append({'run':folder.name,'status':'startup_failed' if 'Traceback' in text else 'not_yet_finished'})
    completed_all=[r for r in trials if r['status']=='complete']
    complete=[r for r in completed_all if r.get('execution_revision')=='v2_independent_cpp_streamer']
    a=next(r for r in reversed(complete) if r.get('variant')=='A')
    bs=[r for r in complete if r.get('variant')=='B' and r['actual_limits_pass'] and not r['teacher_rule_warnings']]
    b=bs[-1]
    assert a.get('display_mode','original') == b.get('display_mode','original'), 'Compare matching display conditions'
    gain=100*(1-b['teacher_mission_duration_wall_s']/a['teacher_mission_duration_wall_s'])
    summary={'scope':'Actual original coursework ROS/SO3 simulation and teacher checker',
             'date':'2026-10-18','map_seed':511,'final_execution_revision':'v2_independent_cpp_streamer',
             'completed_flights_in_final_revision':len(complete),
             'completed_flights_in_all_revisions':len(completed_all),
             'latest_pair':{'baseline_run':a['run'],'story1_run':b['run'],
                            'display_mode':b.get('display_mode','original'),
                            'baseline_teacher_seconds':a['teacher_mission_duration_wall_s'],
                            'story1_teacher_seconds':b['teacher_mission_duration_wall_s'],
                            'improvement_percent':gain},
             'successful_B_teacher_seconds':[r['teacher_mission_duration_wall_s'] for r in bs],
             'all_completed_runs_pass':all(r['coverage']==1 and r['teacher_success_log_seen'] and r['actual_limits_pass'] and not r['teacher_rule_warnings'] for r in complete),
             'earlier_acceleration_failure_runs':[r['run'] for r in completed_all if not r['actual_limits_pass']],
             'trials':trials,
             'planner_sha256':hashlib.sha256((ROOT/'ros_flight.py').read_bytes()).hexdigest(),
             'limits':'Speed, height and acceleration statistics start when motion commands start; the unchanged teacher checker also runs through initialization.',
             'caveats':['Default original map only; not a multi-map or statistical generalization claim.',
                        'B is the measured-time ownership version. Real ROS variant C is implemented but untested.',
                        'First two startup failures remain in the log folders.',
                        'First successful B flight had a runner EOF during teardown because the runner was edited while executing; subsequent full runs validate the saved runner.',
                        'The earlier Python-only execution had an acceleration spike; all earlier attempts remain recorded. Final comparisons use the same independent C++ command streamer.']}
    assert summary['all_completed_runs_pass'], 'Resolve final-revision flight failures before publishing success report'
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    times='、'.join(f'{r["teacher_mission_duration_wall_s"]:.3f} 秒' for r in bs)
    performance_path=OUT/b['run']/'display-performance.json'
    performance=json.loads(performance_path.read_text()) if performance_path.exists() else {}
    fps=performance.get('actual_render_fps') or {}
    body=f'''# 原作业三的本机 ROS 验证结果

已在本机真实运行原仓库的 ROS 仿真，老师检测程序确认 100% 覆盖和零规则违规。
方案停在故事一的主要改进：按预计用时安排分工，不再扩展其他故事。
本页保留逐点停走版本的验证；最新连续快速版本见 [提速实测记录](../fast_results/README.md)。

## 环境

历史测量在 Windows 的独立 WSL Ubuntu 20.04 与 ROS Noetic 内进行，没有使用 Conda 运行 ROS。
发布版自动识别项目位置，默认构建目录为 `$HOME/.local/share/swarm-exploration-course/ws`。
跨平台安装和启动步骤以仓库根目录 README 为准。
原仓库源码、地图、飞行限制、控制器和检测程序未修改。
最终两侧使用同一套独立显示配置，只改 RViz 的画法和显示用消息副本；原任务其余启动元素经过逐项一致性核对。

## 实际测到什么

固定分区 A 的老师计时：{a['teacher_mission_duration_wall_s']:.3f} 秒。
按预计用时分工 B 的成功运行计时：{times}。
最近一次 B 与 A 的用时差为 {gain:.2f}%，两者都达到 100% 覆盖、老师检测零违规。
这些数字来自原检测节点的总计时，包括启动等待和规划，并非之前简化实验的模拟秒。
这里使用原作业默认地图编号 511，没有修改参数换图。因此这只是默认地图上的初步对照，不是多地图平均结果。

最终对照使用同一版连续指令发送节点，并保留相同的 1.35 倍轨迹时间余量。
最新推荐方案的实际状态记录如下。

- 最大速度 {b['max_actual_speed_mps']:.3f} 米每秒，限制 6。
- 最大加速度 {b['max_actual_acceleration_mps2_from_simulator_imu']:.3f} 米每秒每秒，限制 3。
- 最大高度 {b['max_actual_height_m']:.3f} 米，限制 8。
- 记录到的最小机间距离 {b['min_actual_pair_distance_sample_m']:.3f} 米，碰撞阈值 0.4。
- 记录到的最近障碍点距离 {b['min_actual_obstacle_point_distance_sample_m']:.3f} 米，碰撞阈值 0.2。
- 连续指令发送记录到的最大间隔 {b['max_command_stream_gap_s']:.4f} 秒。

## 画面卡顿的处理

原窗口已经使用 RTX 5070，日志中的显卡名称为 `D3D12 (NVIDIA GeForce RTX 5070)`。
原覆盖图每秒向窗口发送约 600 次，共约 150 MB/s；另外，密集障碍物的小方片画法给显示增加了负担。
保留所有原地图点，将障碍物改成点状画法，并把显示用覆盖图限制为每秒约 5 次。
无人机模型显示副本最高每秒 60 次，视野显示副本最高每秒 30 次，窗口目标帧率为 60。
真实位置、飞行控制、避障地图、覆盖计算和老师检测主题及参数不变。

静止悬停对照实测：原显示平均 9.36 帧/秒，仅减少消息量为 16.15 帧/秒，再改点状画法后为 58.56 帧/秒。
相应窗口 CPU 占用约从一个核心的 88% 降到 35%。这些是本机约 15 秒采样，不是所有场景下的帧率保证。
最新完整飞行还在途中采样 30 秒，实测平均帧率为 {fps.get('mean',float('nan')):.2f} 帧/秒。
数据来自实际渲染窗口的统计接口，并非将配置中的目标帧率当成实测值。
性能记录在 `setup/ros/logs/display-check-with-fps`、`setup/ros/logs/display-check-points` 和每次运行目录的 `display-performance.json`。

上述额外运动统计从规划器开始发飞行指令时算起；老师原检测程序从初始化开始一直运行。
本轮只验证了故事一的主要分工版本 B。空闲帮忙版本 C 已接入代码，但没有把它当作已经通过的 ROS 运行结果。

## 怎么再跑一次

在仓库根目录双击 `run-baseline.cmd` 可复现本页版本。
它会启动任务三与推荐方案，默认使用优化后的显示配置，完成后保存结果并关闭本次仿真进程。
也可以在终端运行：

`bash setup/ros/run_trial.sh B`

## 记录在哪里

- 最新推荐方案：`{b['run']}/result.json`。
- 对照方案：`{a['run']}/result.json`。
- 每个运行目录包含原启动日志、规划日志、老师检测信息、实际位置轨迹和实际地图。
- `summary.json` 汇总全部尝试，包括两次未进入探索的启动失败。
- `replay.html` 使用仿真实际位置记录进行左右对比。

初次接入时先后遇到主节点未就绪、初始电机下沉尚未稳定两个问题，均已在接入脚本中处理；对应失败日志仍保留。
修复前的复跑曾记录到 3.676 米每秒每秒的加速度，超过要求，尽管老师脚本仍判定成功。该次运行明确记录为额外运动检查未通过。
轨迹记录显示规划与发指令共用循环时曾停顿约 0.3 秒，之后改用独立发送节点，并要求新轨迹在未来开始。最终对照与复跑均使用修复后的同一版执行方式。
首次成功飞行的启动脚本在收尾时报过 EOF，原因是运行中修改了启动脚本；此后未再边运行边修改，并通过完整复跑。
RViz 原配置的可选 Goal3D 工具有未编译插件提示，不参与本任务的自动规划和规则检测；新的显示配置移除了这个未使用的工具，原仓库仍保留。
'''
    (OUT/'README.md').write_text(body,encoding='utf-8')
    write_viewer(a,b,summary)
    print(json.dumps(summary['latest_pair'],ensure_ascii=False))


def write_viewer(a,b,summary,trace_paths=None):
    payload=[]
    for index,result in enumerate([a,b]):
        path=trace_paths[index] if trace_paths else OUT/result['run']/'flight_trace.json'
        data=json.loads(path.read_text(encoding='utf-8'))
        payload.append({'result':result,'frames':data['frames'],'boxes':data['boxes']})
    css=(ROOT/'viewer_template.html').read_text(encoding='utf-8').split('<style>')[1].split('</style>')[0]
    html='''<!doctype html><html lang="zh-CN"><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>作业三原仿真实验结果</title><style>__CSS__</style>
<header><h1>作业三已经在原仿真中完成</h1><p>六架无人机 · 原始地图 511 · 原控制器与检测规则保留</p><p>老师检测确认：两侧均达到 100% 覆盖，未检测到规则违规。推荐方案已完整复跑。</p><p class="notes">下面是默认地图的一组实际对照，不是多地图平均结果。时间轴从开始发飞行指令算起；顶部数字来自老师的总计时，包含启动和规划。</p></header>
<div class="stats"><div><strong id="aTime"></strong><span>固定分区 · 老师总计时</span></div><div><strong id="bTime"></strong><span>按预计用时分工 · 老师总计时</span></div><div><strong id="gain"></strong><span>这一组对照减少的时间</span></div></div>
<div class="controls"><button id="play">播放</button><input aria-label="飞行时间" id="time" type="range" min="0" max="150" step="any" value="0"><span id="clock">0.0 秒</span><select id="speed" aria-label="播放速度"><option value="4">4 倍速</option><option value="10" selected>10 倍速</option></select></div>
<div class="panels"><section class="panel"><h2>固定分区</h2><div id="leftMetric" class="metric"></div><canvas id="left" width="700" height="700"></canvas></section><section class="panel"><h2>按预计用时分工</h2><div id="rightMetric" class="metric"></div><canvas id="right" width="700" height="700"></canvas></section></div>
<p class="notes">坐标单位：米。线条来自仿真实际位置记录，圆圈表示当前位置的向下视野，灰色是障碍物投影。部分矮障碍可以安全飞越。覆盖率来自老师发布的地图。</p>
<p id="repeat" class="notes"></p><p>以后可双击根目录的 运行作业三.cmd 再跑。先保留当前方案，不增加其他故事。</p>
<script>const DATA=__DATA__,SUMMARY=__SUMMARY__;
const $=id=>document.getElementById(id),colors=['#276c91','#cc7145','#438d65','#8a66a8','#a48b32','#537eb0'];let playing=false,last=0;
$('aTime').textContent=DATA[0].result.teacher_mission_duration_wall_s.toFixed(3)+' 秒';$('bTime').textContent=DATA[1].result.teacher_mission_duration_wall_s.toFixed(3)+' 秒';$('gain').textContent=SUMMARY.latest_pair.improvement_percent.toFixed(1)+'%';$('repeat').textContent='推荐方案成功运行计时：'+SUMMARY.successful_B_teacher_seconds.map(v=>v.toFixed(3)+' 秒').join('、')+'。本页展示最近一次。';
$('time').max=Math.max(...DATA.map(d=>d.frames[d.frames.length-1][0]));
function frame(d,t){let f=d.frames,lo=0,hi=f.length-1;while(lo<hi){let m=Math.ceil((lo+hi)/2);if(f[m][0]<=t)lo=m;else hi=m-1;}let a=f[lo],b=f[Math.min(lo+1,f.length-1)],u=b[0]>a[0]?Math.min(1,Math.max(0,(t-a[0])/(b[0]-a[0]))):0;return a.map((x,k)=>k<2?x:x+u*(b[k]-x));}
function draw(id,d,t){let cv=$(id),c=cv.getContext('2d'),w=cv.width,pad=43,sc=(w-2*pad)/50,X=x=>pad+(x+25)*sc,Y=y=>w-pad-(y+25)*sc;c.clearRect(0,0,w,w);c.fillStyle='#fbfcfc';c.fillRect(pad,pad,w-2*pad,w-2*pad);c.strokeStyle='#dce3e6';c.lineWidth=1;c.font='14px system-ui';c.fillStyle='#697780';for(let a=-25;a<=25;a+=10){c.beginPath();c.moveTo(X(a),Y(-25));c.lineTo(X(a),Y(25));c.stroke();c.beginPath();c.moveTo(X(-25),Y(a));c.lineTo(X(25),Y(a));c.stroke();c.fillText(a,X(a)-9,w-pad+23);c.fillText(a,6,Y(a)+5);}c.fillText('横向位置 米',w/2-35,w-5);c.save();c.translate(15,w/2+30);c.rotate(-Math.PI/2);c.fillText('纵向位置 米',0,0);c.restore();for(const b of d.boxes){c.fillStyle=b[5]>=7?'#8d979c':'#c8ced1';c.fillRect(X(b[0]),Y(b[4]),(b[3]-b[0])*sc,(b[4]-b[1])*sc);}let now=frame(d,t);for(let i=0;i<6;i++){let k=2+i*3,p=now.slice(k,k+3);c.strokeStyle=colors[i];c.lineWidth=2.1;c.beginPath();let start=true;for(const f of d.frames){if(f[0]>t)break;if(start){c.moveTo(X(f[k]),Y(f[k+1]));start=false;}else c.lineTo(X(f[k]),Y(f[k+1]));}c.lineTo(X(p[0]),Y(p[1]));c.stroke();c.globalAlpha=.12;c.fillStyle=colors[i];c.beginPath();c.arc(X(p[0]),Y(p[1]),.5*p[2]*sc,0,Math.PI*2);c.fill();c.globalAlpha=1;c.beginPath();c.arc(X(p[0]),Y(p[1]),5,0,Math.PI*2);c.fill();c.fillStyle='#1c2d36';c.font='bold 13px system-ui';c.fillText(i+2,X(p[0])+7,Y(p[1])-7);}c.strokeStyle='#566771';c.strokeRect(pad,pad,w-2*pad,w-2*pad);return now[1];}
function render(){let t=Number($('time').value);$('clock').textContent=t.toFixed(1)+' 秒';let a=draw('left',DATA[0],t),b=draw('right',DATA[1],t);$('leftMetric').textContent='当前覆盖 '+(a*100).toFixed(3)+'%';$('rightMetric').textContent='当前覆盖 '+(b*100).toFixed(3)+'%';}
$('time').oninput=render;$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'暂停':'播放';};function tick(now){if(playing){let t=Number($('time').value)+(last?(now-last)/1000:0)*Number($('speed').value);if(t>=Number($('time').max)){t=Number($('time').max);playing=false;$('play').textContent='播放';}$('time').value=t;render();}last=now;requestAnimationFrame(tick);}render();requestAnimationFrame(tick);
</script></html>'''
    html=html.replace('__CSS__',css).replace('__DATA__',json.dumps(payload,separators=(',',':'))).replace('__SUMMARY__',json.dumps(summary,ensure_ascii=False,separators=(',',':')))
    (OUT/'replay.html').write_text(html,encoding='utf-8')


if __name__=='__main__':main()

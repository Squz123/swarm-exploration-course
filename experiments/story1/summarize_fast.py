"""Publish measured fast-flight results, including every failed trial."""
from pathlib import Path
import hashlib
import json
import statistics
import summarize_ros

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'fast_results'
PARAMS=['requested_vmax','requested_amax','requested_altitude','target_grid','alpha','prune','curve','takeoff_buffer_s','entry_amax']

def valid(r):return r.get('status')=='complete' and r.get('coverage')==1 and r.get('teacher_success_log_seen') and r.get('actual_limits_pass') and not r.get('teacher_rule_warnings')

def main():
    trials=[]
    for p in sorted(OUT.glob('*/result.json'),key=lambda p:p.stat().st_mtime):
        r=json.loads(p.read_text());r['run']=p.parent.name;trials.append(r)
    legal=[r for r in trials if valid(r)]
    best=min(legal,key=lambda r:r['teacher_mission_duration_wall_s'])
    repeats=[r for r in legal if all(r.get(k)==best.get(k) for k in PARAMS)]
    assert len(repeats)>=2, 'Repeat the fastest passing preset before promoting it'
    baseline=json.loads((ROOT/'ros_results/20261018_B_v2_smooth_trial1/result.json').read_text());baseline['run']='20261018_B_v2_smooth_trial1'
    gain=100*(1-best['teacher_mission_duration_wall_s']/baseline['teacher_mission_duration_wall_s'])
    report={'scope':'Actual original ROS task 3; original map seed 511, original controller and teacher checker',
            'baseline_teacher_seconds':baseline['teacher_mission_duration_wall_s'],
            'fastest_passing_run':best['run'],'fastest_teacher_seconds':best['teacher_mission_duration_wall_s'],
            'reduction_percent':gain,'passing_repeats_same_preset':[{'run':r['run'],'seconds':r['teacher_mission_duration_wall_s']} for r in repeats],
            'measured_limit_peaks_across_repeats':{
                'speed_mps':max(r['max_actual_speed_mps'] for r in repeats),
                'acceleration_mps2':max(r['max_actual_acceleration_mps2_from_simulator_imu'] for r in repeats),
                'height_m':max(r['max_actual_height_m'] for r in repeats)},
            'selected_parameters':{k:best.get(k) for k in PARAMS},'all_trials':trials,
            'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'fast_plan.py',ROOT/'fast_flight.py',ROOT/'ros_package/src/fast_trajectory_server.cpp']},
            'limits':['These are measured results on default map 511, not a proof of a global minimum time.',
                      'Teacher total includes initialization and planning; reported motion-only time is separate.',
                      'All acceleration-failing trials are retained even when the teacher checker reports success.',
                      'Offline candidate checks are not counted as actual flights.']}
    (OUT/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    lines=[]
    for r in trials:
        state='合格' if valid(r) else '不合格，不能采用'
        lines.append(f"- {r['run']}：老师计时 {r['teacher_mission_duration_wall_s']:.3f} 秒；实际最大加速度 {r['max_actual_acceleration_mps2_from_simulator_imu']:.4f}；{state}。")
    peaks=report['measured_limit_peaks_across_repeats']
    body=f'''# 作业三连续快速飞行的实测记录

本轮保留故事一的按用时分工，把逐点停走改成连续飞行，并在原作业限制内提速。
原仓库源码、地图、控制器、速度/高度限制和老师检测程序未修改。

## 最终成绩

此前停走版本老师计时为 {baseline['teacher_mission_duration_wall_s']:.3f} 秒。
当前最快合格运行是 {best['teacher_mission_duration_wall_s']:.3f} 秒，减少 {gain:.2f}%。
同一组参数的合格复跑计时为 {'、'.join(f"{r['teacher_mission_duration_wall_s']:.3f} 秒" for r in repeats)}。
这些运行均达到老师确认的 100% 覆盖、零规则违规，并通过额外的实际运动检查。
计时包含初始化和规划，没有把计算路线的时间藏起来。

同组复跑中记录到的最大值：

- 实际速度 {peaks['speed_mps']:.4f} 米/秒，上限 6。
- 实际加速度 {peaks['acceleration_mps2']:.4f} 米/秒平方，上限 3。
- 实际高度 {peaks['height_m']:.4f} 米，上限 8。
- 最快一轮的最小机间距离 {best['min_actual_pair_distance_sample_m']:.4f} 米，碰撞阈值 0.4。
- 最快一轮的最近障碍点距离 {best['min_actual_obstacle_point_distance_sample_m']:.4f} 米，碰撞阈值 0.2。

这里是默认地图 511 的实测最快合格版本，不代表已经证明所有路线中的绝对最优时间。

## 为什么能更快

整条路线在起飞前安排好，普通观察位置直接经过，不再到一个点就停下、计算下一段。
相邻路段的位置、速度和加速度连续；实际发送节点独立运行，持续发送完整轨迹指令。
采用接近 8 米的合法高度，利用更大的向下视野。
第一段横向起步单独留出跟踪余量，后面巡航保持较高速度；最终没有反复停车，也没有采用无效的起飞等待。
时间是主要目标，加速度平方的累计值作为能量子目标参与候选方案比较。
先检查连续轨迹的速度、加速度、障碍间距和机间距，再在原仿真里检查实际运动。

## 没有把不合格尝试当作成绩

{chr(10).join(lines)}

试过的稀疏扫线会在转弯和绕障处漏扫，飞前逐格检查未通过的候选没有拿来冒充完整覆盖。
删减观察点的版本虽然飞行更短，但规划耗时约 22 秒，老师总计时反而更慢，因此没有作为默认方案。
一开始只看计划加速度还不够；实际记录显示横向起步会冲高，因此额外记录所有无人机的实际速度与 IMU 加速度。

## 如何运行和复查

双击根目录 `运行作业三.cmd`，默认使用本轮最快合格参数。
原停走版本保留在 `运行作业三_保守版.cmd`，可用于对照。
最快一轮目录：`{best['run']}`。
每次飞行保存实际地图、全部连续轨迹、实际位置、老师信息、飞前检查、运动检查及显示性能。
`replay.html` 左侧是此前停走版本，右侧是本轮快速版本；位置来自实际飞行记录。
`summary.json` 保留所有实飞尝试、最终参数与源文件校验值。
'''
    (OUT/'README.md').write_text(body,encoding='utf-8')
    old_out=summarize_ros.OUT
    summarize_ros.OUT=OUT
    # The existing viewer consumes measured frame records only.
    data_summary={'latest_pair':{'improvement_percent':gain},'successful_B_teacher_seconds':[r['teacher_mission_duration_wall_s'] for r in repeats]}
    summarize_ros.write_viewer(baseline,best,data_summary,trace_paths=[
        ROOT/'ros_results'/baseline['run']/'flight_trace.json',OUT/best['run']/'flight_trace.json'])
    html=(OUT/'replay.html').read_text(encoding='utf-8')
    html=html.replace('作业三已经在原仿真中完成','作业三：停走与连续快速飞行的实测对照')
    html=html.replace('固定分区','此前停走版本').replace('按预计用时分工','连续快速版本')
    html=html.replace('本页展示最近一次。','本页展示最快合格一次。')
    html=html.replace('灰色是障碍物投影','灰色表示规划时采用的保守障碍边界')
    (OUT/'replay.html').write_text(html,encoding='utf-8')
    summarize_ros.OUT=old_out
    print(json.dumps({k:report[k] for k in ['fastest_passing_run','fastest_teacher_seconds','reduction_percent','measured_limit_peaks_across_repeats']},ensure_ascii=False))

if __name__=='__main__':main()

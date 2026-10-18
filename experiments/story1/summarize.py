"""Produce an auditable summary and a dependency-free local replay viewer."""
import csv
import hashlib
import json
import platform
import statistics
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    results=[]
    for split in ['development','holdout']:
        for path in sorted((ROOT/'results'/split).glob('*.json')):
            if '_replay' in path.stem or '_coverage_recheck' in path.stem:continue
            row=json.loads(path.read_text(encoding='utf-8'));row['split']=split
            results.append(row)
    holdout=[r for r in results if r['split']=='holdout']
    means={v:statistics.mean(r['t100_s'] for r in holdout if r['variant']==v and r['status']=='complete') for v in 'ABC'}
    permap=[]
    for seed in range(514,524):
        rr={r['variant']:r for r in holdout if r['seed']==seed}
        permap.append({'seed':seed,'A_s':rr['A'].get('t100_s'),'B_s':rr['B'].get('t100_s'),
                       'C_s':rr['C'].get('t100_s'),
                       'A_to_C_improvement_percent':100*(1-rr['C']['t100_s']/rr['A']['t100_s']) if all(rr[v]['status']=='complete' for v in 'ABC') else None})
    rechecks=[]
    for split in ['development','holdout']:
        for path in sorted((ROOT/'results'/split).glob('*coverage_recheck.json')):
            rechecks.append(json.loads(path.read_text(encoding='utf-8')))
    safety=all(r['status']=='complete' and r['safety']['pass'] for r in holdout)
    required_rechecks=[r for r in rechecks if 514<=r['seed']<=523 and r['variant']=='C']
    coverage_recheck=len(required_rechecks)==10 and all(r['coverage']==1 for r in required_rechecks)
    gain=100*(1-means['C']/means['A'])
    wins=sum(r['A_to_C_improvement_percent'] is not None and r['A_to_C_improvement_percent']>0 for r in permap)
    worst=min(r['A_to_C_improvement_percent'] for r in permap if r['A_to_C_improvement_percent'] is not None)
    accepted=safety and coverage_recheck and gain>=5 and wins>=7 and worst>=-10
    summary={'scope':'Windows local kinematic benchmark, NOT original ROS simulator acceptance',
             'environment':{'os':platform.platform(),'python':platform.python_version()},
             'holdout_maps':list(range(514,524)),'mean_t100_simulated_seconds':means,
             'C_vs_A_mean_improvement_percent':gain,'B_vs_A_mean_improvement_percent':100*(1-means['B']/means['A']),
             'C_vs_B_mean_extra_seconds_saved':means['B']-means['C'],
             'improved_maps':wins,'all_holdout_coverage_and_safety_pass':safety,
             'source_style_100hz_C_coverage_recheck_all_pass':coverage_recheck,
             'predeclared_local_stop_condition_pass':accepted,
             'holdout_help_transfers':sum(r.get('help_transfers',0) for r in holdout if r['variant']=='C'),
             'invalid_input_maps':[r for r in results if r['status']=='invalid_initial_state' and r['variant']=='A'],
             'per_map':permap,'coverage_rechecks':rechecks,
             'file_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'benchmark.py',ROOT/'PLAN.md',ROOT/'recheck_coverage.py']},
             'limitations':['No ROS/SO3/PID tracking simulation or teacher-node execution.',
                            'Map PRNG emulates usual Ubuntu libstdc++ defaults; no live map equality check.',
                            'Global source point map is available to the planner, as in assignment.',
                            'Simulation time excludes offline planning and online scheduling CPU time; wall times are separate.',
                            'Deterministic geometry tests, not repeated physical/noisy flight trials.',
                            'No direct execution of M-TARE, C2-Explorer, or URGE; their ideas inform a smaller coursework planner.']}
    (ROOT/'results'/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    with (ROOT/'results'/'all_runs.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=['split','seed','variant','status','t95_s','t100_s','coverage','help_transfers','world_and_initial_plan_wall_s','experiment_wall_s']
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    report=f'''# 故事一的本机实验结果

结论：在这轮本机简化实验中，故事一已达到事先写下的停手标准，先停在这里，不再继续故事二到四。
这不是原作业仿真的验收结果。本机没有可运行的 Ubuntu/WSL 与 ROS，因此尚未运行原仓库的无人机动力学、控制器和老师的检测节点。

## 实际做了什么

在 Windows 上运行新写的实验程序，按作业源码重建 50 米 × 50 米、50 个随机障碍的场地，六架从作业规定的位置出发。
三种方案使用完全相同的观察位置、绕障方式、无人机避让、飞行高度、速度和加速度限制。
飞行按时间连续推进，有起飞、加减速、绕行和等待，并非只比较路线长度。
不过这里假设无人机能够准确跟随计划路线，没有模拟真实控制误差。

A 固定分区：六架各自负责一个方向，在里面优先处理近的任务。
B 按用时分工：提前估计各架多久做完，调整少量任务，让最忙的无人机少承担一些。
C 加入空闲帮忙：在 B 上，让已经空闲的无人机接走别人尚未开始、且确实能缩短全队时间的工作。

## 测到的结果

作业默认编号 511 的地图：A 为 87.50 个模拟秒，B 为 77.30 秒，C 也是 77.30 秒，减少约 11.7%。
另外十张地图使用编号 514 到 523，测试前已冻结方法，没有根据这些结果调整参数。
A 平均 {means['A']:.2f} 个模拟秒；B 平均 {means['B']:.2f} 秒；C 平均 {means['C']:.2f} 秒。
C 相对 A 平均减少 {gain:.2f}%；十张图都更快，没有发现倒退。
这些秒数是程序里的飞行时间，不能直接当作原作业的总用时。规划所花的本机时间另有记录。

三种方案在十张独立地图上都完成了全部可探索格子的覆盖，安全复查也都通过。
安全复查比运行步长更细，还检查了采样时刻之间可能发生的接近，并检查整段路线的速度、加速度和高度上限。
故事一 C 的十张图又按作业源码的覆盖统计逻辑，以每秒 100 次独立重放，全部确认 100% 覆盖。
这仍然是源码逻辑的独立实现，不能代替在 ROS 中运行老师的检测程序。

## 两个改进点各有什么用

第一个“按预计用时分工”是主要收益来源。B 已经比 A 快约 {100*(1-means['B']/means['A']):.2f}%。
第二个“空闲帮忙”在十张独立地图上总共触发一次，平均再省 {means['B']-means['C']:.2f} 秒。
因此不能宣称第二个点效果很大：当前环境已知、飞行也很理想，提前分工做好后，留给途中帮忙的机会不多。
后面若在原仿真里出现实际绕行、等待和跟踪误差，是否更需要帮忙，还要另测。

## 测试中发现并处理的情况

调试时发现，别的无人机停在路线上时，只等待不一定能解决问题。已经为三种方案统一加入绕开停留位置的处理。
独立覆盖复查最初约为 99%，原因是复查代码改变了源码反复加 0.1 的计算顺序，导致边界格子的取整结果不同。
修正复查程序的加法顺序后，默认图及十张测试图均确认完整覆盖；没有修改地图、规则或原仓库代码。
调试地图 513 的固定起点距障碍点只有约 0.172 米，小于作业 0.2 米的碰撞阈值，因此记录为初始输入无效。
没有换种子把这个情况隐藏起来，也没有用它计算有效任务的速度提升；十张独立地图没有这种情况。

## 建议怎么继续

方案先固定为故事一，不再增加其他故事。若以少做、做稳为优先，先把按预计用时分工作为主方案；空闲帮忙保留为小幅补充。
下一阶段需要 Ubuntu 20.04 与 ROS Noetic 环境，把同一套任务安排接到原作业的轨迹指令，再由老师的检测节点检查。
在这一步完成前，结论应写成“本机简化实验支持采用故事一”，不应写成“已经完成作业三”。

## 文件和复现

- PLAN.md：先写下的规则、地图编号和停手条件。
- benchmark.py：三种方案共用的本机实验程序，只依赖 numpy。
- recheck_coverage.py：按作业源码统计方式独立重放覆盖情况。
- results/all_runs.csv：包括无效输入的全部最终运行记录。
- results/summary.json：汇总、每张图结果、文件校验值与复查结果。
- results/development 与 results/holdout：逐次记录和连续轨迹。
- replay.html：可直接打开的本地回放，左右对比简单分工和故事一。

运行示例：python benchmark.py --self-check --seeds 511 --variants A B C
完整独立测试：python benchmark.py --seeds 514 515 516 517 518 519 520 521 522 523 --variants A B C --out results/holdout

原作业仓库保持未修改。本次并未直接运行三个参考项目，而是借鉴它们的分工思路做了更小的作业方案。
'''
    (ROOT/'README.md').write_text(report,encoding='utf-8')
    build_viewer(summary,results)
    print(json.dumps({k:summary[k] for k in ['mean_t100_simulated_seconds','C_vs_A_mean_improvement_percent','predeclared_local_stop_condition_pass','source_style_100hz_C_coverage_recheck_all_pass']},ensure_ascii=False))


def build_viewer(summary,results):
    # Actual recorded trajectories and curves, no synthetic chart data.
    payload={}
    for split in ['development','holdout']:
        for seed in ([511,512] if split=='development' else list(range(514,524))):
            pp={}
            for v in 'ABC':
                src=ROOT/'results'/split/f'{seed}_{v}_replay.json'
                if src.exists():pp[v]=json.loads(src.read_text(encoding='utf-8'))
            if pp:payload[str(seed)]=pp
    html=(ROOT/'viewer_template.html').read_text(encoding='utf-8')
    html=html.replace('__PAYLOAD__',json.dumps(payload,ensure_ascii=False,separators=(',',':')))
    html=html.replace('__SUMMARY__',json.dumps(summary,ensure_ascii=False,separators=(',',':')))
    (ROOT/'replay.html').write_text(html,encoding='utf-8')


if __name__=='__main__':main()

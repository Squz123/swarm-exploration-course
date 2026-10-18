"""Record the release-copy checks separately from the original benchmark."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
DIR=ROOT/'experiments/story1/fast_results'
names=['portable_validation_gui','20261018_154042_FAST','portable_validation_headless']
rows=[]
for name in names:
    r=json.loads((DIR/name/'result.json').read_text())
    assert r['status']=='complete' and r['coverage']==1 and r['teacher_success_log_seen'] and r['actual_limits_pass'] and not r['teacher_rule_warnings'],name
    rows.append({'run':name,'teacher_seconds':r['teacher_mission_duration_wall_s'],
                 'actual_speed':r['max_actual_speed_mps'],'actual_acceleration':r['max_actual_acceleration_mps2_from_simulator_imu'],
                 'actual_height':r['max_actual_height_m'],'pass':True})
data={'date':'2026-10-18','scope':'Separate publication checkout, Ubuntu 20.04 in WSL2, original ROS simulator',
      'checks':['fresh catkin workspace build','polynomial endpoint and continuity checks','exact dynamic extrema checks',
                'deliberate collision negative control','exact obstacle raster denominator','30 frozen offline holdout runs',
                'shell syntax','Windows launcher full flight','Linux shell full flight','headless full flight without DISPLAY/Wayland'],
      'runs':rows,
      'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'experiments/story1/fast_plan.py',ROOT/'experiments/story1/fast_flight.py',ROOT/'experiments/story1/ros_flight.py']}}
dest=ROOT/'docs';dest.mkdir(exist_ok=True)
(dest/'validation.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
lines=[f"- `{r['run']}`：{r['teacher_seconds']:.3f} 秒；实际最高速度 {r['actual_speed']:.4f}、加速度 {r['actual_acceleration']:.4f}、高度 {r['actual_height']:.4f}；通过。" for r in rows]
(dest/'VALIDATION.md').write_text('''# 发布副本验证记录

2026-10-18，在独立发布目录中完成以下验证。这里的记录与之前调参阶段的历史结果分开保留。

## 真正运行过的启动路径

'''+ '\n'.join(lines)+'''

第一项从 Linux shell 启动，第二项实际经过 Windows 的 `run-fast.cmd` 和 PowerShell 启动器，第三项取消 DISPLAY/Wayland 后无窗口运行。
三项均使用课程原地图 511、原控制器和原老师检测，达到 100% 覆盖、老师零违规，并通过额外的实际速度、加速度、高度、距离检查。
独立 ROS 端口用于避免影响原项目的服务；没有修改作业规则。

## 构建与检查

- 从新的 catkin 构建目录重新编译课程所需包和本项目 C++ 节点，完成成功。
- 连续轨迹起终位置、速度、加速度和接段连续性检查通过。
- 计划速度和加速度的极值计算检查通过。
- 故意设置的交叉碰撞负例被拒绝。
- 实际障碍点包含关系、老师覆盖分母、任务归属检查通过。
- 早期 30 次冻结离线记录、独立覆盖复查与停止条件检查通过。
- Bash 启动脚本语法检查通过。
- 提交前再对实际 Git 索引检查大文件、误加的虚拟磁盘/日志/本机配置和常见凭据形式，并核对 README 相对链接。

## 验证范围

这是已安装依赖的 WSL Ubuntu 20.04 环境中的重新构建与运行，不是再次安装一台全新的空白 Ubuntu。
原生 Linux 安装步骤使用同一套 Ubuntu 20.04 脚本，但没有在另一台物理 Linux 主机独立验收显示驱动。
安装脚本已核对系统版本守卫、依赖包清单、签名公钥和语法；没有关闭签名验证。

路径通用化修改了脚本及默认输出位置，因此历史实验文件中原先记录的源文件哈希不一定等于发布副本。
它们保留为当时测量的证据，不改写成新的哈希。当前发布源文件的哈希另见 `validation.json`。
三次发布副本运行也保存了完整实际轨迹和结果，便于对照。
''',encoding='utf-8')
print(json.dumps(data['runs'],ensure_ascii=False))

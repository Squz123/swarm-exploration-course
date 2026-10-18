# 多无人机探索课程作业：分工均衡与连续快速飞行

本项目完成 [课程仓库](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task) 的**任务三：集群探索**。
六架无人机从中心出发，用向下的有限视野覆盖 50 米 × 50 米区域，同时避障、避免互撞。
目标是在满足作业限制的前提下尽快完成 100% 覆盖。

最终方案保留老师的地图、动力学、控制器和检测程序，只增加自己的规划、轨迹执行和检查节点。
课程环境以 Git 子模块固定在 `d8360a9dc82405d88ebae89e2a8c9429ece579c9`。

**默认地图实测：从停走版本的 115.048 秒降到 50.168 秒，减少 56.39%。**
相同参数完整复跑两次为 50.174 秒、50.168 秒，均 100% 覆盖、老师检测零违规，额外实际运动检查也通过。
这是目前试过方案中的最快合格版本，不是全局最短时间的数学证明。

## 先看效果和证据

- [实际飞行左右对比回放](experiments/story1/fast_results/replay.html)：下载后用浏览器打开，无需联网。
- [提速实验记录](experiments/story1/fast_results/README.md)：保留原调参阶段七次实飞，包括加速度超限的失败尝试；发布副本验证另列于文末链接。
- [机器可读汇总](experiments/story1/fast_results/summary.json)。
- [最终复跑结果](experiments/story1/fast_results/20261018_fast_v3_trial7_repeat/result.json)。
- [此前停走版本](experiments/story1/ros_results/README.md) 和 [早期 Windows 简化实验](experiments/story1/README.md)。

GitHub 文件预览不会直接播放 HTML；可以下载打开，或在仓库根目录运行：

```bash
python3 -m http.server 8835 --bind 127.0.0.1
```

然后打开 `http://127.0.0.1:8835/experiments/story1/fast_results/replay.html`。
回放位置来自实际飞行记录，不是另画的理想路线。

## 相较于“传统方案”提升了什么

这里的“传统方案”指本项目实现的朴素对照，并非老师给出的官方最优算法，也不是三个参考论文的运行结果。

- **A，固定分区：** 六架分别负责一个方向，优先处理附近位置；老师计时 129.100 秒。
- **B，按用时分工但逐点停走：** 调整少量任务，让最忙的无人机少承担一些；老师计时 115.048 秒，相对 A 减少约 10.88%。
- **最终快速版本：** 连续经过观察位置，并调整观察布局、高度和动态余量；两次为 50.174 秒、50.168 秒。最快一次相对 B 减少 56.39%，相对 A 减少约 61.14%。

所有这些 ROS 对照都使用默认地图 511 和老师原检测程序，时间包含初始化与规划。
快速版本同时改变了轨迹衔接、观察位置、高度和速度设置，因此 56.39% 是整套方案的改善，不能全归功于某一个单独改动。
尚未用最终 ROS 快速版本完成多地图统计；早期十张地图的简化实验不能替代这个验证。

最终两次复跑中实际记录到的最高值：

- 速度 **5.8316 米/秒**，作业上限 6。
- 加速度 **2.9964 米/秒平方**，作业上限 3。
- 高度 **7.9720 米**，作业上限 8。
- 最快一次的最小机间距离约 **0.8769 米**，碰撞阈值 0.4；最近障碍点约 **0.4666 米**，碰撞阈值 0.2。

这些是实际采样记录。老师的脚本不单独检查加速度，所以本项目另外订阅仿真 IMU，超限即不采用该次成绩。
接近上限的参数针对当前已测环境；换地图、控制器或运行负载后应重新检查实际结果。

## 两个主要改进点

### 1. 按“多久干完”分工

平均分面积或平均分任务数，可能让某一架频繁绕障、最后独自收尾。
这里先估计每架完成当前路线所需的时间，再把少量任务移给相对空闲的无人机。
每次调整都优先看全队最晚完成的时间，避免只有总路程变短、最慢的一架仍然拖后腿。

这是课程层面的分工改进，没有宣称新的学术算法。
“空闲后途中帮忙”的 C 版本只做了早期离线实验，未作为最终 ROS 快速版启用，不把它算作已验证的 ROS 创新收益。

### 2. 连续飞过观察位置，把额度用在有效移动上

停走版本每一小段都要从静止加速、再减速到静止，到点后才安排下一段。
快速版本在起飞前接好整条路线，使相邻段的位置、速度和加速度连续，普通观察位置直接飞过。
拐弯、绕障时按加速度限制降速，长直线提高速度；利用接近 8 米的合法高度扩大向下视野。
第一段横向起步单独收缓，避免控制器实际响应冲过加速度上限。

候选方案以完成时间为主要目标，同时保留加速度平方累计值这个能量子目标。
当前比较分数为 `全队计划完成时间 + 0.0005 × 累计加速度平方积分`。
这不是一个保证全局最优的求解器；路线候选比较、时间分配和可行性检查共同构成实现。

## 方案从输入到完成的流程

1. **等原仿真准备好。** 接收六架真实位置、`/mock_map` 点云和老师发布的覆盖网格，等初始悬停稳定。
2. **整理地图。** 保留老师统计覆盖率所用的原始障碍格子；另用较保守的障碍边界安排绕行，不改变覆盖分母。
3. **安排观察位置。** 在合法高度布置观察位置，遇到高障碍就移到附近安全位置；检查是否还需要补观察点。
4. **先分工、再均衡。** 从六个方向形成初始任务，再按预计用时调整少量归属。
5. **接成连续轨迹。** 生成每段完整的位置、速度、加速度和变化率；调整每段用时，使计划速度和加速度不超限。
6. **起飞前检查。** 检查连续轨迹的动态极值；对障碍间距和机间距采用带运动余量的采样下界检查。必要时只错开起飞时间，不让每个观察点都停车。
7. **交给原控制器执行。** 独立 C++ 节点每秒发送 100 次完整轨迹指令，规划计算不会卡住指令发送。
8. **同时验收。** 老师程序确认覆盖与规则；额外 C++ 记录实际速度、IMU 加速度和安全距离。只有 100% 覆盖且两侧检查都通过才算合格。

本任务给出了全局地图，因此这里不是未知环境中的实时建图，也没有复现通信受限的分布式系统。

## 支持的环境

- 已实测：**Windows 11 + WSL2 + Ubuntu 20.04 + ROS Noetic**，系统 Python 3.8。
- Linux 启动使用同一套 Ubuntu 20.04 脚本，已在独立 Linux 编译工作区重编并运行。未在另一台原生 Linux 物理机单独验收图形驱动。
- Ubuntu 22.04/24.04、ROS2 与其他控制器没有作为兼容平台验证。Noetic 属于较旧的课程依赖，建议使用独立的 Ubuntu 20.04 环境复现。
- **不需要 Conda。** ROS 使用 Ubuntu 的 `/usr/bin/python3`，避免与 Windows 或 Conda 的 Python 混用。
- GPU 加速主要用于 RViz 画面。规划与仿真计算仍主要由 CPU 完成，不需要安装 CUDA 工具包来运行本项目。

### Linux：Ubuntu 20.04

```bash
sudo apt-get update
sudo apt-get install -y git
git clone --recurse-submodules https://github.com/Squz123/swarm-exploration-course.git
cd swarm-exploration-course
bash setup/ros/install_ros_noetic.sh
bash setup/ros/build_course.sh
bash setup/ros/run_fast.sh
```

安装脚本只接受 Ubuntu 20.04，安装 ROS Noetic、RViz、PCL、编译工具和 Python 科学计算依赖。
不关闭软件包签名验证，不修改系统代理或用户已有 Conda。
默认编译目录在 `$HOME/.local/share/swarm-exploration-course/ws`，源码目录不放构建产物。

没有桌面时可运行无窗口检查，地图、控制器、覆盖统计和规则检查仍运行：

```bash
COURSE_GUI=0 bash setup/ros/run_fast.sh
```

### Windows：WSL2 中运行课程程序

Windows 端只负责启动；ROS 和 Linux C++ 依赖安装在 WSL 的 Ubuntu 20.04 内。
建议使用支持 WSLg 的 Windows 11。

1. 安装 Git for Windows。管理员 PowerShell 按 [微软 WSL 安装说明](https://learn.microsoft.com/en-us/windows/wsl/install) 安装 WSL，并安装 Ubuntu 20.04：

   ```powershell
   wsl --install -d Ubuntu-20.04
   ```

   按系统提示重启并完成 Linux 用户初始化。已有合适发行版可直接使用，不必再安装。

2. 在 Windows 的 D 盘克隆项目，例如：

   ```powershell
   cd D:\robotics
   git clone --recurse-submodules https://github.com/Squz123/swarm-exploration-course.git
   ```

3. 进入 Ubuntu，在对应目录安装、编译：

   ```bash
   cd /mnt/d/robotics/swarm-exploration-course
   bash setup/ros/install_ros_noetic.sh
   bash setup/ros/build_course.sh
   ```

4. 回到 Windows，双击 `run-fast.cmd` 或 `运行作业三.cmd`。它会自动转换当前仓库路径，不依赖作者的 D 盘目录。
   双击 `run-baseline.cmd` 或 `运行作业三_保守版.cmd` 可跑停走对照 B。

启动器默认使用名为 `Ubuntu-20.04` 的发行版。
如果已有环境叫 `RoboticsCourseUbuntu20` 或其他名字，把 `local-settings.example.json` 复制成 `local-settings.json` 并修改其中的 `distribution`。
`local-settings.json` 已忽略，不会提交到 Git。
Windows 入口使用系统自带 PowerShell，执行策略选项只作用于本次启动进程，不更改全局策略。

如果 Git 报 `basename`、`sed` 找不到，请在 Git Bash 中执行克隆或子模块初始化命令。
如果忘了递归克隆，在仓库根目录执行 `git submodule update --init --recursive` 后再编译。
WSL 图形配置见 [微软 GUI 应用说明](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)。

## 如何运行对照、调整参数和查看结果

```bash
# 固定分区 A：朴素对照
bash setup/ros/run_trial.sh A
# 按用时分工 B，但仍逐点停走
bash setup/ros/run_trial.sh B
# 最终连续快速版
bash setup/ros/run_fast.sh
```

默认参数在 [fast-defaults.sh](setup/ros/fast-defaults.sh)。以下是规划参数，合格与否仍以实际运动为准：

- `COURSE_VMAX=5.98`：计划速度上限。
- `COURSE_AMAX=2.84`：巡航计划加速度上限。
- `COURSE_ENTRY_AMAX=2.4`：第一段横向起步的计划加速度上限。
- `COURSE_ALTITUDE=7.97`：巡航高度。
- `COURSE_TARGET_GRID=10`：初始观察位置布局。
- `COURSE_WS`：可选，指定另一个编译工作区；编译和运行必须使用同一个值。
- `COURSE_ROS_PORT`：默认 11321；已被占用时可换一个端口，脚本不会停止已有 ROS 服务。
- `COURSE_GPU_ADAPTER=NVIDIA`：可选，在 WSLg 中明确选择 NVIDIA 适配器；默认尊重系统选择。

例如在空闲端口运行一次，并指定结果目录名称：

```bash
COURSE_ROS_PORT=11331 bash setup/ros/run_fast.sh my_trial
```

快速结果在 `experiments/story1/fast_results/<运行名称>/`；停走对照在 `experiments/story1/ros_results/<运行名称>/`。
重点看 `result.json` 的 `coverage`、`teacher_success_log_seen`、`teacher_rule_warnings` 和 `actual_limits_pass`，不能只看窗口打开或老师打印成功。
`plan.json`、`flight_trace.json`、`teacher_messages.json` 分别保存完整轨迹、实际位置和老师信息。
历史结果是固定证据，新运行的日志和输出默认被 Git 忽略。

## 显示为什么比较流畅

原窗口已经能使用显卡，但覆盖底图每秒约 600 次、约 150 MB/s，密集障碍物的小方片画法也有负担。
现在只把显示用的覆盖图副本降到每秒约 5 次，并将障碍物改为点状画法；原点云、飞行位置和老师检测频率不变。
本机静止悬停对照约从 9.36 提高到 58.56 帧/秒，完整飞行也做了实际渲染窗口采样。
这是显示优化，不把它当作探索策略的创新点或 56.39% 时间收益的独立贡献。

## 代码在哪里

- `experiments/story1/ros_flight.py`：真实地图整理、分工与停走对照。
- `experiments/story1/fast_plan.py`：连续轨迹、时间分配、候选评分和飞前检查。
- `experiments/story1/fast_flight.py`：连接原 ROS 仿真并记录快速实验。
- `experiments/story1/ros_package/src/fast_trajectory_server.cpp`：独立发送完整轨迹并记录实际运动。
- `setup/ros/`：Ubuntu 安装、编译、启动和显示配置生成。
- `swarm_intelligent_robot_course_task/`：固定版本的课程子模块。

## 检查与参考

```bash
# Linux/WSL，先加载 ROS Python 包路径
source /opt/ros/noetic/setup.bash
python3 experiments/story1/fast_plan.py
python3 experiments/story1/check_live_geometry.py
# 早期离线实验的独立检查
python3 experiments/story1/check_results.py
```

离线 Windows 实验可在单独 Python 环境安装 `requirements-offline.txt` 后运行 `benchmark.py`，不代表已运行 ROS。
源代码和环境移植验证记录见 [发布验证记录](docs/VALIDATION.md)。

思路参考 [M-TARE](https://github.com/caochao39/mtare_planner)、[C2-Explorer](https://github.com/Robotics-STAR-Lab/C2-Explorer)、[URGE](https://github.com/NKU-MobFly-Robotics/URGE)。
本项目没有直接移植或统一复现这三个规划器。来源和许可边界见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
本仓库的原创新增代码使用 BSD-3-Clause；课程子模块与依赖保持各自的原始授权。

# 来源和致谢

课程环境来自 [SKBT-lab/swarm_intelligent_robot_course_task](https://github.com/SKBT-lab/swarm_intelligent_robot_course_task)。
本仓库通过 Git 子模块固定使用 `d8360a9dc82405d88ebae89e2a8c9429ece579c9`，不修改其中的地图、动力学、控制器或老师检测代码。
上游及其内含依赖保持各自的原始许可和作者信息；本仓库的 BSD-3-Clause 不替代上游授权。

思路参考：

- [M-TARE](https://github.com/caochao39/mtare_planner)：参考多机器人分工与全队完成时间的视角。
- [C2-Explorer](https://github.com/Robotics-STAR-Lab/C2-Explorer)：参考协调探索、减少重复工作的思路。
- [URGE](https://github.com/NKU-MobFly-Robotics/URGE)：参考围绕剩余未覆盖区域安排工作的思路。

本项目没有直接移植这三个项目的规划器，也没有在统一环境中复现或对比它们。
这里实现的是适合课程给定地图和检测规则的简化方案，不能把本仓库的用时提升解读为超过上述论文方法。

`setup/ros/ros.key` 是 ROS 官方公开的软件包签名公钥，来源为
<https://raw.githubusercontent.com/ros/rosdistro/master/ros.key>，不是用户的私钥或访问凭据。
签名检查保持启用；安装脚本核对其指纹。

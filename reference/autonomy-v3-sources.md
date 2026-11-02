# 自主算法升级的开源参考 · 2026-11-01

本次是现有 Python/NumPy 平台内的独立实现，没有下载或运行外部二进制、引入 ROS 全栈、复制受限源码或训练新模型。保留厂家 SLAM、定位与运动底层。

| 参考 | 固定版本与许可 | 本平台实际采用的部分 |
|---|---|---|
| [m-explore / explore_lite](https://github.com/hrnr/m-explore/tree/712bdd41027b645c9c876a4c0071478f090825ea) | `712bdd41027b645c9c876a4c0071478f090825ea`，2.1.4，BSD-3-Clause | 边界聚类、收益与路径代价权衡、保持有效目标；额外加入遮挡信息收益和已扫区域新视角评分。详见 [exploration_sources.md](exploration_sources.md)。 |
| [Nav2 MPPI](https://github.com/ros-navigation/navigation2/blob/c633de75b9663e54dbe48d8177b1a2cd11cfd986/nav2_mppi_controller/README.md) 与 [PathFollowCritic](https://github.com/ros-navigation/navigation2/blob/c633de75b9663e54dbe48d8177b1a2cd11cfd986/nav2_mppi_controller/src/critics/path_follow_critic.cpp) | `c633de75b9663e54dbe48d8177b1a2cd11cfd986`，Apache-2.0 | 候选轨迹前向预测、路径推进/偏离/障碍/控制变化评分、每帧滚动决策。平台使用确定性采样与两段转向/制动，**不是 MPPI 移植**，没有采用其随机优化、插件或调好参数。 |
| [Nav2 全局规划](https://github.com/ros-navigation/navigation2/tree/c633de75b9663e54dbe48d8177b1a2cd11cfd986/nav2_smac_planner) | 同上，Apache-2.0 | 障碍距离启发、完整机身与旋转空间、路径简化。平台保持原有栅格格式与 Python 实现，详见 [单点规划升级](../docs/architecture.md)。 |
| Removert、ERASOR、KISS-ICP | 各固定 SHA、许可与研究范围见 [地图算法来源](map_algorithm_sources.md) | 遮挡/看穿的区分、局部稳健对应及独立结构验证。没有复制 CC BY-NC-SA 或 GPL 代码。 |

## 训练判断

当前故障集中在探索目标过密、规划启发弱、运动候选不足和跨扫描结构偏差；这些可以用已有几何测量与可重复闭环直接改进。因此本轮不新增模型训练、数据集下载或推理服务，也不接入 TypeSafe。

YOLO 人员、安全帽检测继续使用原有模型。对沙地、山地、隧道的地形识别和学习式通过策略，本轮没有训练或实机验证；不能把室内几何回放的结果外推到这些环境。

## 运行分工

- Mac：探索目标选择、全局路径、实时局部轨迹、任务调度和界面。
- NOS/GOS 平台服务：新鲜点云与地面观测整理；NOS 地图服务执行配准、融合和候选保存。
- 厂商：腿轮与平衡控制、原厂 SLAM/定位；可选的原厂单点导航继续独立保留。
- 平台自主寻路选择普通遥控模式，通过既有归一化三轴接口输出。它不是原厂导航的参数调整，也不是原厂避障关闭后的同一执行器。平台用新鲜地面/障碍与完整机身轨迹检查，实机验证必须单独进行。

控制器内部速度增益只用于短时预测，并从速度反馈缓慢调整。离线回放假定的归一化输入到速度关系不能称为本机运动标定，不能据此提高物理速度上限。

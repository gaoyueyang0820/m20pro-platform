# 自主探索目标层参考资料 · 2026-11-01

研究并核对了 [hrnr/m-explore](https://github.com/hrnr/m-explore) 中 explore_lite 2.1.4，固定 commit `712bdd41027b645c9c876a4c0071478f090825ea`：

- [`frontier_search.cpp`](https://github.com/hrnr/m-explore/blob/712bdd41027b645c9c876a4c0071478f090825ea/explore/src/frontier_search.cpp)：从可达区域搜索未知边界，连通边界成簇，以边界大小与距离排序。
- [`explore.cpp`](https://github.com/hrnr/m-explore/blob/712bdd41027b645c9c876a4c0071478f090825ea/explore/src/explore.cpp)：保留有效目标、按进展判断失败、跳过失败目标。
- [`LICENSE`](https://github.com/hrnr/m-explore/blob/712bdd41027b645c9c876a4c0071478f090825ea/LICENSE)：BSD 三条款。`explore/package.xml` 声明 BSD，版本 2.1.4。

平台的 `exploration_frontiers.py` 是独立实现，没有复制该仓库代码或引入 ROS1 运行依赖。采用上述通用 frontier 原理，另外按本平台实际约束实现：带墙壁遮挡的射线信息收益、可达图路径代价、转向代价、可支撑观察位置、观察基线稀疏化、旧图未知区收益、有限时间退避与观测变化后重试。

未知区域仅影响目标价值，绝不自动变成可通行地面。真实动作仍需新鲜机身点云与机身包络检查。纯几何多房间回放用于比较目标选择，不能证明轮腿动力学、真实雷达与现场门口通过。

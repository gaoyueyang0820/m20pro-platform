# 首次导入检查 · 2026-11-02

这是独立协作仓库的首次导入验证，不修改或重启原工作区，也不连接机器人。

- 按现有 `tools/build_delivery.py:source_files` 取得稳定源码快照，203 个源文件，共 51,607,411 字节。
- 应用实现保持快照原样；只新增协作文档、CI 和排除规则，原 README 保存为 PLATFORM_GUIDE.md。
- 121 个 Python 文件 AST 语法检查通过。
- 3 个 YOLO 权重与 models/sources.json 的 SHA-256 一致。
- `node --test tests/test_*.cjs`：67 项通过，0 失败，0 跳过。
- 以下 Python 测试：82 项通过，0 失败；本机使用现有 Python 环境，在禁止网络的 OS 沙箱中执行。

```bash
PYTHONPATH=.:tests python -m unittest -v \
  test_exploration_frontiers test_navigation_planning_v3 \
  test_local_trajectory test_map_change_policy test_pose_monitor \
  test_latest_observation test_start_pose_recovery
```

- 在独立临时数据目录及临时端口启动 `--demo`，HTTP 页面和 `/api/platform` 均正常，状态为 demo；该进程已退出，外网通信被沙箱禁止。
- 凭据模式扫描的命中已核对：仅测试中的假 token/假 RTSP 凭据和部署代码中的变量拼接，没有发现需上传的真实密钥。data/ 中仅保留合成视频夹具。

本次没有运行全部可选模型/平台/现场回放测试，没有验证 Windows 原生运行，也没有进行机器人实机验收。GitHub Actions 的执行结果以对应提交的远端记录为准。

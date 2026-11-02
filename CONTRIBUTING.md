# 协作修改

main 是共享开发基线。每个功能建立独立分支，通过 Pull Request 合并。

```bash
git clone https://github.com/gaoyueyang0820/m20pro-platform.git
cd m20pro-platform
git switch -c feature/your-change
# 修改、验证后：
git add <本次修改的文件>
git commit -m "Describe the behavior change"
git push -u origin feature/your-change
```

在 GitHub 创建 Pull Request，写明触发条件、修改后行为和验证结果。合并后更新本地：

```bash
git switch main
git pull --ff-only
```

## 本地验证

建议 Python 3.11 或 3.12、Node.js 22。使用各自虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-video.txt
python -m unittest discover -s tests -v
node --test tests/test_*.cjs
```

完整测试含本地回环 HTTP 服务。需要真实现场回放、可选视觉依赖或原生库的测试可能跳过；FFmpeg 测试需要本机 FFmpeg。可选模型依赖使用仓库安装脚本；原生感知库用 `python tools/build_native_sensing.py` 编译。不要为通过测试自动连接机器人。

CI 只执行语法、前端回归以及指定的合成几何/状态测试，不代表完整测试或真机验收。`tools/offline_acceptance.py` 面向原开发机的 macOS 隔离验收与私有现场回放，其缺少回放/基线的提示不应通过伪造 PASS 消除。

开发演示使用独立端口与目录：

```bash
python m20_platform.py --demo --http 8088 --data-dir data/demo
```

## 与原工作区的关系

首次导入是 2026-11-02 的开发源码快照，来源清单见 docs/source-snapshot.json。原工作区继续调试产生的修改不会自动同步到这里，协作仓库的修改也不会自动部署到机器人。

后续选择一个共同开发基线，以提交和 PR 同步；迁回原工作区或部署前先对比差异，并保留该机器自己的 data/。不要用整目录覆盖正在调试的工作区。

## 提交内容

提交源码、测试、合成夹具、构建资源和必要说明。模型权重已包含，保留 models/sources.json 和许可文件；更换权重时同时记录来源与 SHA-256。

不提交真实连接资料、令牌、私钥、授权码、现场地图、录像、日志、虚拟环境、node_modules 或本机编译产物。仓库中 data/network/camera-codec-fixture.mp4 是既有打包规则明确允许的合成视频测试素材。

本次未为整个项目新增开源许可证；已有第三方代码、模型与素材继续受各自许可约束。

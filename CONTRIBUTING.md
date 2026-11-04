# 协作修改

`main` 是共享开发基线。每个功能使用独立分支，通过 Pull Request 合并；不要直接把正在调试的整目录覆盖到另一份工作区。

```bash
git clone https://github.com/gaoyueyang0820/m20pro-platform.git
cd m20pro-platform
git switch -c feature/your-change
# 修改并完成适用验证后，只加入本次文件：
git add <本次修改的文件>
git commit -m "Describe the behavior change"
git push -u origin feature/your-change
```

PR 写清触发条件、修改后行为、实际执行的验证及尚未验证的部分。合并后再更新本地基线：

```bash
git switch main
git pull --ff-only
```

首次导入的来源见 `docs/source-snapshot.json`。原作者后续工作区已经接入同一 Git 工作树，新的修改也应在本地功能分支提交并经 PR 同步。文件修改不会自行推送，Git 同步也不会自动部署机器人。每台机器自己的运行数据与连接资料保持本地，不随源码覆盖。

## 开发环境

便携检查使用 CPython 3.11/3.12、Node.js 22；macOS/Linux 的完整便携组还需要支持 C++17 的本机编译器。每个协作者使用自己的虚拟环境。

macOS / Linux：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
npm ci
.venv/bin/python tools/check_portable.py --report .artifacts/portable/report.json
```

Windows PowerShell：

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
npm ci
.venv\Scripts\python.exe tools/check_portable.py --groups algorithms,navigation,sensing,ui --report .artifacts/portable/report.json
```

Windows 命令是明确的子集，报告为 `partial`；没有验证 POSIX 服务操作或 C++ 原生感知。Windows 原生构建由独立 MSVC CI 作业验证，不能把纯 Python/Node 子集通过写成原生库通过。macOS/Linux 的完整便携检查会在临时目录显式编译原生库，并要求测试确实调用原生实现，不能全程回退到 NumPy 后报通过。

`requirements-dev.txt` 只装 NumPy 和无 GUI 的 OpenCV 测试依赖，不安装 ROS、Torch、Ultralytics、机器人 SDK 或模型权重。运行视频/模型功能另用已有安装说明，勿在同一虚拟环境同时安装多个 OpenCV 发行包。测试脚本不会自动下载或安装缺失依赖。

## 独立演示与真机边界

演示使用独立端口和独立数据目录：

```bash
python m20_platform.py --demo --http 8088 --data-dir .dev/demo
```

打开 `http://127.0.0.1:8088/`。使用新建的演示目录，不复用真机运行数据。`--demo` 必须显式指定；不要把启动默认进程当作离线隔离的证明。

只有在用户明确授权的任务范围内，才能连接或控制真机、部署代理、切换服务或启用地图。已经确定的会话授权按约定复用，纯离线代码检查无需重复确认。多人协作时由一个明确的执行者管理真机运动，其他协作者使用本地测试或只读证据，避免多个后台争用同一机器人。

`tools/retest_bounded.py`、连接/部署工具和现场排障脚本不属于便携 CI。不要为了让测试通过而连接设备、改路由、停服务或生成虚构的现场录制。

## 验证与原生构建

默认先运行便携检查；修改三维代码后另运行 `npm run build` 并核对相关源码与产物。更完整的本机录制回放、系统网络隔离检查及实机验证，见 [验证层级](docs/verification.md)。CI 中两个工作流并存：既有基础检查和新增便携检查，都不是全平台或实机认证。

`tools/check_portable.py` 将被审查的源码复制到临时目录，排除运行数据、真实回放 JSON、模型权重及已有本机二进制。它记录执行、跳过和未验证范围；新增测试不会仅因命名为 `test_*` 就自动获准在 CI 执行。加入白名单前应审查导入时行为、网络/子进程/摄像头调用、夹具来源和 OS 依赖。

手动编译原生后端时也使用独立输出，避免覆盖其他正在运行的进程所用库：

```bash
python tools/build_native_sensing.py --output .artifacts/native/libsensing.so
```

当前输出名为 `.so`，实际文件仍属于构建它的 OS/CPU/ABI；不能把某台电脑的结果直接当作机器人或另一 OS 的产物。导入 `native_sensing` 不会编译或下载库。数值边界上的回退是设计的一部分，普通完整点云必须有真实原生命中证据。

## 提交内容与来源

提交源码、合成测试、构建输入和必要说明。保留仓库已有 `models/sources.json`、第三方许可和来源说明；新增或替换模型时记录来源、固定版本、SHA-256 和许可证声明。未知许可不能自动视为已获再分发授权，也不要为 CI 下载大模型。

不提交运行数据、真实连接配置、令牌、私钥、SDK 授权码、现场地图/点云、真实录像、日志、个人配置、虚拟环境、`node_modules` 或本机编译库。审查测试代码中是否嵌入了真实坐标或录制数组；忽略一个 JSON 路径并不能消除源码里的数据副本。真实回放保留在本地忽略目录，缺失时明确记录跳过，不能以合成数据假冒同一现场用例。

仓库既有合成视频测试素材属于明确标识的例外，不等同于允许加入现场视频。项目没有因此获得一个统一的新开源许可证；第三方代码、模型和素材继续保留各自许可边界。

## PR 最小说明

- 具体问题与用户可见的修改后行为。
- 修改涉及的层：页面、电脑端算法、连接代理、原生模块或机器人部署。
- 已执行命令、报告路径、执行/跳过数量与原因；有原生修改时附本机构建及非全回退证据。
- 尚未进行的真实录制、Windows 原生、ROS/SDK 或实机验证。

以实际测试的新提交为准，不把历史机器人成功记录复制为本次验收结论。

# M20 Pro 操作平台

面向 Deep Robotics M20 Pro 的统一机器人作业平台。电脑连接机器人网络后，通过网页完成手动驾驶、视频查看、建图、导航巡检、地图更新、视觉识别及 SDK 开发操作。

平台保留原厂运控、SLAM 和定位系统；电脑负责界面、任务调度、探索与路径规划，机器人侧代理负责感知和设备接口。

## 功能

| 工作区 | 提供的操作 |
| --- | --- |
| 手动驾驶 | 方向、步态、姿态、照明、前后相机、停止与设备状态 |
| 建图与地图 | 新建地图、点云与栅格查看、地图包导入导出、切换、改名和版本管理 |
| 导航与巡检 | 地图点选目标、自动寻路、实时绕障、多点巡检、暂停与继续 |
| 自主作业 | 自主探索、原图续建、人员跟随、任务恢复与采集保存 |
| 行走中优化地图 | 积累导航或手动行走观测，与完整旧图融合，生成待启用的新版本 |
| 视觉识别 | 人员检测、安全帽/PPE 检测、人数与轨迹统计 |
| SDK 与开发 | SDK 环境和进程管理、关节/单腿操作、训练工作站任务 |

源码已包含这些入口和实现。各功能的实机验证范围见 [功能状态](docs/feature-status.md)；训练、SDK 授权与复杂地形能力取决于对应设备环境。

## 启动

### macOS / Linux

需要 Python 3.9+。在仓库目录执行：

```bash
python3 -m venv data/runtime-venv
data/runtime-venv/bin/python -m pip install -r requirements-video.txt
data/runtime-venv/bin/python m20_platform.py
```

打开 <http://127.0.0.1:8080/>。macOS 后续可直接双击根目录的 `启动平台.command`。

人员与 PPE 识别还需安装推理依赖。macOS 可运行 [安装人员识别依赖.command](scripts/macos/安装人员识别依赖.command)；视频安装与共享入口也集中在 [scripts/macos/](scripts/macos/)。

### Windows

安装标准 64 位 Python 3.11–3.13，依次运行根目录的 `Setup-Windows.cmd` 和 `Start-Windows.cmd`。其他启动、演示和检查入口放在 [windows/](windows/)。详见 [Windows 使用说明](docs/windows.md)。

### 独立演示

演示无需机器人，使用独立数据目录：

```bash
python3 m20_platform.py --demo --http 8088 --data-dir .dev/demo
```

打开 <http://127.0.0.1:8088/>。网页和三维资源已构建，日常运行不需要 Node.js、npm 或电脑端 ROS。

## 连接机器人

1. 电脑连接机器人所在网络，打开“连接中心”。
2. 填写当前可达的 AOS 地址，完成首次连接。首次安装需要机器人账号权限，之后复用保存的连接。
3. 平台同时维护 AOS、NOS、GOS：AOS 是外部入口，NOS/GOS 经 AOS 中转，无需手动添加电脑路由。

机器人换 Wi-Fi 后，AOS 地址可能变化。平台会查找已配对机器人的新地址；服务凭据不一致时自动同步，不要求重复输入密码或重装服务。普通遥控和导航不需要开启底层 SDK 模式。

同一私人网络的其他电脑可访问已运行平台的共享地址。详细步骤、独立电脑安装和视频配置见 [连接与视频](docs/local-connection-and-video.md)。

## 日常作业

- **去一个目标点**：选择导航地图，在地图上点选目的地，点击“前往目标”。平台负责计算路径；不必手工画完整路线。
- **多点巡检**：保存航点与路线后执行。中间点自动接续，最后一个点完成后结束并停车。
- 选择 **GOS 自主寻路**，可在指定航点停稳后转动云台、统计人数并读取物联网设备数据。 [GOS 自主寻路与到点作业](docs/gos-patrol.md)
- **新建或扩充地图**：在“自主作业”选择新建地图或续建活动地图。续建结果保留完整旧图，并融合本轮有效观测。
- **平时行走更新地图**：开启驾驶/导航页的“行走中优化地图”。平台生成融合候选，由操作者点击启用；行走途中不会切换地图。

操作细节见 [自主作业与地图更新](docs/autonomous-operations.md)、[地图管理](docs/map-library.md) 和 [导航断联策略](docs/disconnected-navigation.md)。

## 仓库结构

```text
m20_platform.py        网页平台启动入口
m20/                   电脑端服务、算法、协议与运行时
bridge/                机器人端感知、地图、SDK 和训练代理
platform_web/          网页、三维源码和已构建资源
native/                可选 C++ 点云加速
models/                识别权重、来源和许可
scripts/macos/         macOS 安装与共享工具
windows/               Windows 安装、启动和检查工具
tools/                 构建、检查、基准及维护工具
tests/                 测试与合成素材
docs/                  操作手册、接口、架构和历史说明
reference/             算法依据、原始示例及第三方许可
```

`data/`、`.dev/`、`.artifacts/` 和 `dist/` 是本地运行、测试与打包目录，不进入 Git。真实地图、连接资料、私钥、录像和虚拟环境保留在各自电脑上。

## 协作开发

```bash
git switch -c feature/your-change
python tools/check_portable.py --jobs 4
```

开发依赖与 PR 流程见 [CONTRIBUTING.md](CONTRIBUTING.md)。修改三维源码后运行 `npm ci && npm run build`；构建入口是 `platform_web/viewer-src.js`。

更新源码使用 `git pull --ff-only`，保留本机运行数据。机器人端代理通过连接中心单独更新；GitHub 同步不会自动部署机器人服务。

[文档目录](docs/README.md) · [API](docs/api.md) · [架构](docs/architecture.md) · [变更记录](CHANGELOG.md) · [来源与许可](docs/sources.md)

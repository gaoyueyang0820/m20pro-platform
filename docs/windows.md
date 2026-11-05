# Windows 使用说明

需要标准 64 位 Python 3.11–3.13。网页、三维资源及三份识别权重已随源码提供，日常使用不需要 Node.js 或 ROS。

## 首次安装

1. 下载或克隆完整仓库，解压到本地目录。
2. 安装 Python，勾选添加到 PATH。
3. 双击根目录的 `Setup-Windows.cmd`，联网安装视频与识别依赖。
4. 双击根目录的 `Start-Windows.cmd`，打开 <http://127.0.0.1:8080/>。

安装使用仓库内的 `data\runtime-venv` 独立环境，默认安装 CPU 版推理依赖。运行平台时保持终端窗口打开。

## 其他入口

| 入口 | 用途 |
| --- | --- |
| `windows\Demo-Windows.cmd` | 独立演示；不会把现有实机平台切成演示 |
| `windows\Share-Windows.cmd` | 在所选私人网段共享平台 |
| `windows\Check-Windows.cmd` | 检查 Python、依赖、模型与资源 |
| `windows\Setup-Video-Windows.cmd` | 仅安装视频所需依赖 |

中文便携入口也集中在 `windows/`。存在根目录 `runtime\python.exe` 时，`windows\启动平台.cmd`、`windows\演示模式.cmd` 和 `windows\重新安装依赖.cmd` 使用便携运行时；没有运行时时调用标准安装与启动流程。

## 连接与换网

电脑连接机器人所在网络后，在“连接中心”填写当前 AOS 地址并完成首次连接。首次安装输入机器人账号信息，主机配对由平台完成，不需要手动读取和比对指纹。

此后平台保存各主机连接。换 Wi-Fi 时查找已配对机器人的新地址，NOS/GOS 继续经 AOS 中转；凭据不一致时自动同步。地图和服务管理需要 Windows OpenSSH Client，电脑不需要安装 OpenSSH Server。

更详细的操作见 [连接与视频](local-connection-and-video.md)。普通连接不等于底层 SDK 授权。

## 运行数据和升级

标准 Windows 启动器把用户数据放在 `%LOCALAPPDATA%\M20ProPlatform`。便携入口使用仓库根目录的 `userdata/`，独立演示使用 `userdata-demo/`。

升级源码时保留这些目录与运行环境；它们包含本机连接、地图和任务资料。不要用旧 ZIP 整目录覆盖现有安装。通过其他电脑的共享网页使用平台时，打开运行后端电脑的局域网地址；各电脑的 `127.0.0.1` 只指向自身。

## 视频、识别和原生加速

安装脚本提供 FFmpeg、OpenCV、NumPy 及识别依赖。人员、PPE 和人数统计权重默认从 `models/` 读取，不需要填写另一台电脑的模型路径。

可选 C++ 点云加速需要 Visual Studio x64 C++ 生成工具：

```powershell
python tools/build_native_sensing.py
```

生成的 `native/libsensing.pyd` 仅用于当前 OS/CPU。缺少编译环境时继续使用 NumPy；模型及第三方许可见 [来源说明](sources.md)。

## 常见问题

- **找不到 Python**：确认安装了标准 64 位版本，添加到 PATH 后重新打开终端。
- **Torch DLL / WinError 126**：安装 [Visual C++ x64 运行库](https://aka.ms/vs/17/release/vc_redist.x64.exe)，再运行检查。
- **端口已占用**：启动器会复用已有平台，不会关闭其他进程；需要独立演示时使用另一个端口及数据目录。
- **共享页无法打开**：使用后端电脑的局域网地址，并按 Windows 提示允许 Python 访问所选私人网络。
- **机器人服务不通**：确认安装 OpenSSH Client；平台会自动恢复已配对连接。设备重装或更换后，在高级管理中重新配对。

持续集成覆盖 Windows Python/Node、MSVC 原生数值与 SSH 参数，以及视频依赖安装和合成解码。真实设备与现场任务的验证范围见 [功能状态](feature-status.md)。

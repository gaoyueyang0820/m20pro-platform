# M20 Pro 操作平台 · Windows 使用说明

仅安装前后视视频依赖可运行 `Setup-Video-Windows.cmd`，它会安装 OpenCV、NumPy、FFmpeg 并用合成视频验证。完整安装仍使用 `Setup-Windows.cmd`。首次联网安装；换网与首次连接说明见 [本地连接与视频](docs/local-connection-and-video.md)。

这是当前平台的完整源码、已构建网页、三维模型及三份识别权重。继续使用 `m20_platform.py` 和 `http://127.0.0.1:8080/`，没有另一套平台。

## 第一次使用

1. 解压整个压缩包，例如放到 `C:\M20Pro-Platform`。不要直接在 ZIP 内运行文件。
2. 安装标准 **Python 3.13 64 位**，勾选 **Add python.exe to PATH**。也支持标准 Python 3.12 / 3.11；不支持 32 位及 free-threaded 版本。[Python 官方下载](https://www.python.org/downloads/windows/)
3. 双击 **Setup-Windows.cmd**。首次需要联网，依赖装进本目录 `data\runtime-venv`，默认安装 CPU 版 PyTorch，不需要 NVIDIA 显卡。正常安装可能需数分钟，完成后会自动检查。
4. 双击 **Start-Windows.cmd**，浏览器自动打开平台。运行期间保留终端窗口。

其他入口：

| 文件 | 用途 |
|---|---|
| `Demo-Windows.cmd` | 离线演示；已有实机进程时不会替它切换模式 |
| `Share-Windows.cmd` | 同一私人 Wi-Fi 共享；输入实际私人网段，如 `172.20.10.0/28` |
| `Check-Windows.cmd` | 检查 Python、视频/识别依赖、模型和本地资源，不连接机器人 |

Windows 运行数据放在 `%LOCALAPPDATA%\M20ProPlatform`，包含后续自己设置的连接、地图和任务资料。升级时保留此目录；本包没有复制原 Mac 的任何连接凭据、私钥、真实地图或调试录像。首次使用创建独立配置；不要把两台电脑同时当成机器狗的控制端。

## 连接机器人

先让 Windows 电脑连接机器人所在网络，再通过平台“连接中心”填写当前可达的 AOS 地址。基础 UDP 控制、相机和电脑端识别不要求 Windows 安装 ROS。

地图/NOS/GOS/SDK 服务管理还需要 **OpenSSH Client**。若检查脚本提示缺少 ssh、ssh-keygen 或 ssh-keyscan，在 Windows“设置 → 应用 → 可选功能”添加 OpenSSH 客户端，安装后重开平台。无需为这台控制电脑安装 OpenSSH Server。[Microsoft 官方说明](https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse)

首次配置仍要核对机器人主机指纹，输入本机适用的账号。Windows 密码引导使用 Paramiko，保持已确认主机密钥校验；后续公钥隧道使用 Windows OpenSSH。机器人端目标路径仍是 Linux 路径，例如 `/usr/bin/python3`，不能改填 Windows Python 路径。

## 视频与识别

- 安装脚本通过 `imageio-ffmpeg` 提供 Windows FFmpeg；平台优先查找 PATH 中已有的 FFmpeg 6+，其次使用随 Python 依赖安装的二进制。使用严格解码处理打开/重连时的 HEVC 灰闪。
- 保留 YOLO11n、PPE 安全帽模型和用户提供的 YOLO12n 人数统计权重，无须重新寻找模型文件。默认 CPU，实际帧率取决于电脑性能。
- 模型来源和许可在 `models/sources.json` 与同目录 LICENSE 文件。YOLO12n 权重来自用户提供文件，未补造官方下载或许可声明。
- 机器人休眠或相机关闭时没有实时视频；页面会显示对应原因。视频帧率不等于机器人运动已验收。

## 常见问题

- **找不到 Python**：安装上述 64 位标准版本，勾选 PATH，关掉旧终端后重试。
- **Torch 导入提示 DLL / WinError 126**：安装 [Microsoft Visual C++ x64 运行库](https://aka.ms/vs/17/release/vc_redist.x64.exe)，然后重跑检查。
- **端口 8080 已占用**：脚本不会杀掉原进程。已有平台会复用；其他程序占用时先自行处理，或临时设置 `M20_HTTP_PORT`。
- **其他电脑无法访问共享页**：使用运行平台那台 Windows 的局域网地址；`127.0.0.1` 只代表当前电脑。按 Windows 提示允许 Python 访问所用私人网络，不要关闭整机防火墙。
- **路径含中文或空格**：Windows 入口启用 UTF-8，并处理本地 SSH 路径引用。建议先在简单目录解压，避免超长路径。
- **升级本机原有安装**：不要用旧发布包整目录覆盖现有工作区；保留自己的用户数据目录。

## 验证范围

本包已做文件完整性、Python 语法与解压后离线启动检查。Windows 专用启动、网卡识别、密码 SSH 引导和路径兼容代码已加入，但**尚未在真实 Windows 电脑上做安装、相机或机器人验收**。本包不是 Windows 原生 EXE，首次使用需要 Python 和联网安装依赖。

原平台 Mac 的真实视频修复证据不代表 Windows 实测通过。SDK、关节控制及训练执行环境仍以机器人和 NVIDIA 工作站的版本与授权为准；Windows 控制台不会把普通连接当作 SDK 授权，也不会在普通电脑上虚构训练能力。

# 本地安装、换网与视频

## 其他电脑如何使用

每台独立运行平台后台的电脑，首次在“连接中心”填写自己的机器人地址并完成一次 SSH 指纹核对与连接公钥配置。完成后保存本机 `data/` 目录；更新源码时不要覆盖它。无需切换机器人底层 SDK 模式来建立普通 NOS/GOS 管理连接。

以后机器人换 Wi-Fi：电脑连接同一网络，在连接中心填写新的 AOS 地址并点击“连接机器人”。已经确认的机器人身份会跨 IP 保留，NOS/GOS 继续使用原来的内网地址，经 AOS 中转。SDK 与 AOS 同机时，其地址和账号同步跟随；SDK 在独立电脑时取消高级设置中的跟随选项。

这不是无条件信任新 IP。平台把已核对的主机公钥保存在稳定身份记录中，实际 SSH 握手继续严格验证。地址填错、换了另一台机器或系统重装导致主机密钥改变时，连接会拒绝，需要人工重新核对。旧配置会从当前地址已有的可信记录迁移；若旧版本已经丢失了地址与身份的关联，首次升级仍需核对一次当前 AOS 指纹。

“UDP 在线”不代表 NOS/GOS 的 SSH 和服务接口已经在线；以各主机“服务已连接”的状态为准。换网不改变或自动恢复旧运动任务。

私钥、密码、连接令牌、主机信任文件和真实地图不随 GitHub 分发，不能复制他人的运行数据目录冒充首次配置。仅通过另一台电脑访问已运行的平台网页时，可用该平台的局域网共享入口，不必在浏览器电脑再安装后端依赖。

## 视频依赖

`requirements-video.txt` 固定 NumPy、OpenCV 和 imageio-ffmpeg 版本。后者提供适合运行平台的 FFmpeg 二进制；普通前后视不要求安装 Torch/YOLO。首次安装需要联网下载公开发行包。

- macOS：双击 `安装视频依赖.command`，然后从原“启动平台”入口启动。
- Windows：标准 64 位 Python 3.11–3.13，双击 `Setup-Video-Windows.cmd` 安装视频部分。`Setup-Windows.cmd` 安装包含视频、模型推理与 SSH 的完整环境。
- Linux 或命令行：在独立虚拟环境安装 `pip install --only-binary=:all: -r requirements-video.txt`。

安装后运行 `python tools/check_video.py --bundled`（使用平台虚拟环境的 Python），检查包内 FFmpeg 6+、合成 HEVC 解码与 JPEG 转换。不连接机器人，不把此结果当成相机推流已经正常。

GitHub 的 Video runtime installation 工作流在 Windows/macOS/Linux 安装视频依赖并解码同一合成素材。实际 USB 相机、网络防火墙和机器人端推流需在对应机器确认。ARM/Linux 若没有匹配的二进制发行包，应使用该系统可用的 FFmpeg 6+，不能复制另一种 OS/CPU 的文件。

## 实现依据

电脑端 OpenSSH 使用 [HostKeyAlias](https://man.openbsd.org/ssh_config#HostKeyAlias) 保留机器人身份；Windows 密码首次配置沿用 [Paramiko 主机密钥核对](https://docs.paramiko.org/en/stable/api/client.html)，不会使用 AutoAddPolicy。所有信任仍来自该电脑之前明确确认的主机公钥。

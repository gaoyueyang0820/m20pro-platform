# M20 Pro 可视化操作平台

面向 M20 Pro 的浏览器操作平台，集中管理 AOS/NOS/GOS 连接、驾驶与视频、建图与地图版本、导航、自主探索、SDK、关节、训练接口和 YOLO 识别。

这是 **2026-11-02 的协作开发快照**，包含正在开发的功能，不是已经完成全部真机验收的正式发布版。历史功能与验证记录见 [平台说明](PLATFORM_GUIDE.md)；本次导入范围见 [源码快照清单](docs/source-snapshot.json)。

## 快速启动

建议 Python 3.11 或 3.12。前端已构建，启动页面不需要 Node.js、npm 或本机 ROS2。

```bash
git clone https://github.com/gaoyueyang0820/m20pro-platform.git
cd m20pro-platform
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-video.txt
python m20_platform.py --demo --http 8088 --data-dir data/demo
```

打开 <http://127.0.0.1:8088/>。该启动方式使用独立的演示数据目录，不读取原开发机的连接配置。

Windows 参考 [README-Windows.md](README-Windows.md)，也提供 Setup/Start/Demo/Check 启动脚本。Mac 可使用现有 `.command` 启动器。视频推荐安装 FFmpeg；YOLO 额外依赖可通过 `安装人员识别依赖.command` 安装。仓库保留 `ppe.pt`、`yolo11n.pt`、`yolo12n.pt` 和来源/许可说明。

实机使用时由操作者在连接中心配置自己的机器人与服务。仓库不包含已配对电脑的 SSH 私钥、令牌或连接配置；克隆源码不等于已连接或部署机器人。高级功能所需的 NOS/GOS/SDK/训练环境见平台说明。

## 协作

请阅读 [CONTRIBUTING.md](CONTRIBUTING.md)，采用功能分支和 Pull Request。仓库设为私有，协作者需要仓库所有者授予访问权限。现有电脑的调试工作区与本仓库独立，修改不会自动同步或部署。

| 目录/文件 | 内容 |
| --- | --- |
| `m20_platform.py`、`platform_hub.py` | 平台后端与任务协调 |
| `platform_web/`、`viewer-src.js` | 操作界面与三维视图 |
| `autonomy_*`、`navigation_*`、`point_navigation.py` | 探索、感知、路径规划与执行 |
| `map_*`、`localization_*` | 地图融合、更新与定位 |
| `bridge/` | NOS/GOS/SDK/训练主机服务 |
| `native/` | 可选原生点云加速源码，按目标主机编译 |
| `models/`、`reference/` | YOLO 权重、参考实现和来源/许可 |
| `tests/`、`tools/` | 回归测试、离线验证与构建工具 |

修改三维视图后执行 `npm ci`、`npm run build`。接口说明见 [API.md](API.md)。

GitHub Actions 执行语法、前端回归及指定的合成几何/状态测试；完整测试、私有现场回放和实机验收是不同层次。历史文档中提到的 `data/reviews/`、`调试交接/` 及父目录厂家资料未上传，其结果不能作为本次快照的验收结论。

运行数据、现场地图、录像、日志、模型运行环境和连接凭据留在各自电脑。唯一随仓库保留的 data 文件是合成视频测试素材。第三方来源和许可保持在原目录，本次未新增整体开源许可证。

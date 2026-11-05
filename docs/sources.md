# 来源与许可

平台从用户提供的 `default.zip` 遥控器扩展而来，沿用普通控制协议、前后相机及原 YOLO 适配思路。当前运行协议实现在 `m20/m20_console.py`，初始协作导入保存在 Git 提交 `c5bdb17`。旧版网页仅作为参考保留在 [reference/legacy-controller/](../reference/legacy-controller/)，不属于当前平台入口。

当前代码和操作说明以主分支为准。初始 README、上游副本及导入时文件哈希可以从上述提交取回，不再作为现行文件重复维护。

## 模型和外部资源

| 资源 | 来源与许可记录 |
| --- | --- |
| 人员、安全帽、人数统计模型 | [models/sources.json](../models/sources.json)、同目录许可文件 |
| 用户提供人数统计示例 | [reference/people_counter/](../reference/people_counter/) |
| 探索、导航与地图算法参考 | [算法来源](../reference/autonomy-v3-sources.md)、[探索来源](../reference/exploration_sources.md)、[地图算法来源](../reference/map_algorithm_sources.md) |
| Three.js、URDF Loader | [Three.js 许可](../reference/three-LICENSE.txt)、[URDF Loader 许可](../reference/urdf-loader-LICENSE.txt) |
| M20 URDF 与网格 | `platform_web/assets/model/`，沿用已提供的厂商模型资源 |

项目没有声明一个覆盖全部代码、模型和素材的新统一许可证。各资源继续遵循自身的许可与来源说明；用户提供的 YOLO12n 没有可核对的官方发行元数据。

`tests/fixtures/video/` 中的视频是生成的编解码测试图样，不是机器人录像。真实地图、点云、连接资料和现场录像不随仓库分发。

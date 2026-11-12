# GOS 自主寻路与到点作业

单点导航和巡检默认使用 **GOS 机载寻路**。巡检也可选择原厂导航。GOS 接收活动栅格和任务，负责路线预览，并结合机内雷达计算和执行行进路线。电脑接收状态并维护任务授权。持续 5 秒未收到电脑心跳时，GOS 请求停车。外接球机和物联网采集使用 GOS。

GOS 执行器使用敏捷平地步态和正向机身坐标。普通已保存路线可以直接执行。包含充电航点的路线自动使用专用回桩流程。

点击「执行巡检」会执行当前航点和设置，并在任务中心留下执行记录。点击「保存路线」才会将草稿加入保存的路线或更新已保存的版本。

全局规划先尝试二维通路导引和整段机身扫掠，再用航向搜索处理窄道。空旷直行复用地图区段，只需核对最新雷达下的前向及制动包络。障碍挡路时，先找可接回主路线的位置或重新规划通往目标的路线，再尝试局部绕行。此分层方式参考自动驾驶的参考路线、行为规划与速度约束。 [Autoware 行为规划](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner/)

已处于原厂辅助模式时沿用该模式。原厂定位状态 `2002/1` 的 `ObsState` 和基础状态 `1002/6` 的 `OOA` 可在跟踪受阻时提前触发复核。 [定位回包](https://support.deeprobotics.cn/docs/#/259675879) · [辅助遥控](https://support.deeprobotics.cn/docs/#/259675901)

## 沿途采集

原厂导航可使用前视或后视相机做到点人数统计。平台等待原厂报告当前航点完成并确认停稳，由电脑采样三帧后按路线继续。采用到点统计的原厂路线使用断联停车。

巡检路线编辑器常显「沿途采集」。可为人数统计选择相机，为物联网读数从设备目录选择设备，再分别设置采集间隔。配置随路线保存，可以和到点作业一起使用。

使用 GOS 寻路时，采集在行进阶段使用低优先级独立进程。选择原厂导航时，电脑可以使用前视或后视相机统计沿途人数，断联期间暂停采集。任务中心显示最近一帧人数、任务峰值和设备读数，采集记录可下载为 JSON。峰值是采集期间单帧中最多的人数。采集过程中的临时故障显示在对应采集项。

暂停导航时暂停采集，任务结束后可在任务中心下载已记录的数据。

## 到点人数统计

每个航点可选择前视、后视或外接球机进行到点人数统计。外接球机可在当前位置统计，也可选择短转向左或向右。机身停稳后才执行动作。球机先收到方向和停止请求，等待画面稳定窗口后取三帧，记录其中最高的同帧人数。可在「到点读取设备」选择物联网设备，读数会写入同一条作业记录。

普通航点自动接续。作业航点停稳并完成统计后继续，终点停车。人数统计失败时，任务停在当前航点并报告原因。在转动过程中中止作业时会请求云台停止。

## 物联网设备与任务

连接机器人并更新 GOS 服务后，在「连接中心 → 物联网平台」填写 GOS 可达的 API 地址。地址包含现场网关实际使用的端口和接口前缀。使用有设备清单、实时读数权限的账号，或填写已授权的 API 令牌。需要验证码时在同一窗口完成验证。验证通过后，GOS 保存连接配置。

「任务中心 → 物联网设备」可查询设备名称、查看在线状态并读取最新值。设备目录每次打开时从物联网平台读取，也可以手动刷新。新增设备接入并获得查询权限后，可以直接在任务中选用。

从「任务中心 → 新建巡检任务」设置航点。可以在「沿途采集」选择设备和采集间隔，也可以展开任意航点，在「到点读取设备」选设备。每项可选择最多八台设备。到点读取可以单独使用，也可以与相机人数统计一起执行。保存路线后，可在任务中心执行并查看到点结果或下载沿途采集记录。某台设备读取失败时，记录该设备的原因，其余设备继续读取。

设备目录和读数使用中铁信科对接文档的查询接口。读数适配设备上报的数值、布尔值和物模型字段。现场对设备下发动作需要相应的控制接口和权限。 [FastBee 接口说明](https://www.fastbee.cn/doc/open/v2/dev/api.html)

## 联调配置

1. 更新 NOS 和 GOS 平台代理。在连接中心为 GOS 填写已核对的机内 AOS 地址，再部署 GOS 服务。
2. 导航使用 GOS 本地 NumPy 和只读感知服务。人数统计可使用 Ultralytics 和 `yolo12n.pt`，也可使用同一模型导出的 ONNX 和板载 CPU 运行时。把经过验证的 `yolo12n.onnx` 和 `arm64-python/` 放进电脑数据目录的 `gos-vision-runtime/` 后，更新 GOS 服务会带上该运行时。
3. 物联网连接通过连接中心保存到 GOS 的私有 `gos-patrol.local.json`。已有球机设置可继续放在同一文件中。现场网络、鉴权和摄像头控制需在联通后核对。

```json
{
  "ball_rtsp": "",
  "iot_url": "http://现场物联网网关/实际API前缀",
  "iot_user": "",
  "iot_password": "",
  "sip_device": "",
  "sip_channel": "",
  "fastbee_ptz": false
}
```

`ball_rtsp` 可填写已有球机 RTSP。仅配置国标通道时，服务通过 `/sip/player/play` 取得实时流。云台控制适配 FastBee 开源 v2 的 `/sip/ptz/ptz` 方向和停止接口，核对现场版本后可启用 `fastbee_ptz`。图形验证码开启时，使用平台返回的结构化验证码或已有的短期 `iot_token`。当前 API 适配读数查询，人数结果保存在本平台任务记录。 [FastBee v2 云台控制源码](https://github.com/beecue/FastBee/blob/fc285f09fef16eb2a931618e566547e23c6434ea/springboot/fastbee-open-api/src/main/java/com/fastbee/data/controller/media/PtzController.java)

## 接口

- `POST /api/nav/plan`、`/api/nav/goal`、`/api/nav/start` 默认派发到 GOS。巡检可用 `explicit_execution: true` 随路线保存或在执行时提交 `engine` 和 `disconnect_policy`，也可在巡检预览时指定 `engine`。演示模式使用本地模拟，充电航点交给专用回桩流程。
- `GET /api/iot/settings` 返回连接状态，`POST /api/iot/settings` 验证并保存连接。`GET /api/iot/devices?page=1&name=` 分页查询目录，`GET /api/iot/read?device_id=42` 读取当前值。
- `POST /api/route/save` 的航点可带 `station_action: {camera, pan, iot_device_ids}`。`camera: "none"` 可用于单独读取设备，方向支持 `none`、`left`、`right`。已有 `iot_device_id` 继续兼容。
- `GET /api/tasks/collectors` 返回采集项和字段定义。通过 `POST /api/route/save` 的 `options.collectors` 保存沿途采集配置，例如 `[{"kind":"people_count","camera":"front","interval_s":2},{"kind":"iot_readings","device_ids":["42","43"],"interval_s":5}]`。
- `GET /api/tasks/collection?run_id=<任务执行编号>&offset=0` 读取当前或历史任务的采集记录，按响应的 `next_offset` 分页。记录包含采集类型、采集时间、采集请求时的航点索引及读数或错误。
- `GET /api/nav/resident/capabilities` 的 `gos` 返回 GOS 可用性和作业能力。执行状态及 `station_results` 随导航状态同步。

新增采集项可在 `m20/task_collectors.py` 的 `COLLECTORS` 中注册字段、可用性判断及采样类。采样类提供 `sample()` 和 `close()`，采样结果包含 `sampled_at` 和 `values`。路线编辑器使用字段定义生成表单，GOS 使用同一契约调度采集和保存记录。

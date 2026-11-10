# GOS 自主寻路与到点作业

在单点导航或多点巡检中选择 **GOS 自主寻路**。电脑提交活动栅格和任务，GOS 从机内雷达读取最新观测，计算路线并通过机内 AOS 连接执行。电脑接收状态并维护任务授权。短暂丢包与雷达观测过期分别处理，持续 5 秒未收到电脑心跳时 GOS 请求停车。

GOS 执行器使用敏捷平地步态和正向机身坐标。包含进桩的路线仍从原厂导航入口执行。

## 沿途采集

在巡检路线编辑器展开「沿途采集」，勾选人数统计、物联网读数或两项一起使用。为人数统计选择相机，为物联网读数填写设备 ID，再设置每项的采集间隔。配置随路线保存，可以和到点作业一起使用。

采集在行进阶段使用低优先级独立进程，相机每次取一张新鲜帧。任务中心显示最近一帧人数、任务峰值和设备读数，采集记录可下载为 JSON。峰值是采集期间单帧中最多的人数。采集过程中的临时故障显示在对应采集项，导航继续按现有安全约束执行。

暂停或结束导航时随任务结束本段采集。继续路线会产生一个新的执行段，每段记录可在任务中心单独下载。

## 到点人数统计

展开航点的「位置、朝向与任务」，选择前视、后视或外接球机。外接球机可在当前位置统计，也可选择短转向左或向右。机身停稳后才执行动作。球机先收到方向和停止请求，等待画面稳定窗口后取三帧，记录其中最高的同帧人数。配置物联网设备 ID 时，同一条记录还会附上该设备的实时读数。

普通航点自动接续。作业航点停稳并完成统计后继续，终点停车。人数统计失败时，任务停在当前航点并报告原因。在转动过程中中止作业时会请求云台停止。

## 联调配置

1. 更新 NOS 和 GOS 平台代理。在连接中心为 GOS 填写已核对的机内 AOS 地址，再部署 GOS 服务。
2. 导航使用 GOS 本地 NumPy 和只读感知服务。人数统计可使用 Ultralytics 和 `yolo12n.pt`，也可使用同一模型导出的 ONNX 和板载 CPU 运行时。把经过验证的 `yolo12n.onnx` 和 `arm64-python/` 放进电脑数据目录的 `gos-vision-runtime/` 后，更新 GOS 服务会带上该运行时。
3. 按需在 GOS 数据目录 `/var/lib/m20-platform` 放置私有文件 `gos-patrol.local.json`。现场网络、FastBee 鉴权和摄像头控制需在联通后核对。

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

- `POST /api/nav/plan`、`/api/nav/goal`、`/api/nav/start` 接受 `engine: "gos"`。
- `POST /api/route/save` 的航点可带 `station_action: {camera, pan, iot_device_id}`。相机支持 `front`、`rear`、`ball`，方向支持 `none`、`left`、`right`。
- `GET /api/tasks/collectors` 返回采集项和字段定义。通过 `POST /api/route/save` 的 `options.collectors` 保存沿途采集配置，例如 `[{"kind":"people_count","camera":"front","interval_s":2},{"kind":"iot_readings","device_id":"42","interval_s":5}]`。
- `GET /api/tasks/collection?run_id=<任务执行编号>&offset=0` 读取当前或历史任务的采集记录，按响应的 `next_offset` 分页。记录包含采集类型、采集时间、采集请求时的航点索引及读数或错误。
- `GET /api/nav/resident/capabilities` 的 `gos` 返回 GOS 可用性和作业能力。执行状态及 `station_results` 随导航状态同步。

新增采集项可在 `m20/task_collectors.py` 的 `COLLECTORS` 中注册字段、可用性判断及采样类。采样类提供 `sample()` 和 `close()`，采样结果包含 `sampled_at` 和 `values`。路线编辑器使用字段定义生成表单，GOS 使用同一契约调度采集和保存记录。

# M20 Pro 平台 API v2.5

写接口均为 POST + application/json，同源请求。`--share` 在选定的私人网段免口令访问，不使用共享会话或控制权租约。旧版显式 token 启动方式仍兼容。失败返回 `{"ok":false,"err":"原因"}`。`delivery=sent` 和 `launched` 不代表硬件业务完成。

耗时操作返回 `{ok:true,job_id}`，使用 `GET /api/operations?id=<job_id>` 查询。`jobs` 为单个任务，含 status、progress、message、result、error。传输/解析进度与厂商建图业务进度分开。

v2.3：`/api/platform` 增加 `cameras`（解码可用性、每路 fresh/age/frames/size/error/viewers）。普通 `/video/front|rear` 共享解码连接；无首帧时在发送视频头前返回 503。`/api/maps/build/status` 增加 phase、in_progress、message、result_maps，启动命令结束不等于采集结束。地图 apply 返回同步后的 map 与 navigation_ready；sync 复用同一地图缓存编号。

`/api/route/save` 接收 `options:{nav_mode,speed,gait,manner,pause_at_point}` 并持久化；`/api/nav/start` 未明确传入时沿用已保存选项。最后点完成或任务点暂停时先发送取消导航请求，使用原 cancelling 状态等待匹配回包，通过 after_cancel/stop_reason 区分收尾完成与到点暂停。`completed` 只在收尾请求确认后记录。暂停时 index 指向继续后要执行的点。

## 状态与连接

共享模式增加以下接口：

| 方法 | 路径 | 参数 / 用途 |
| --- | --- | --- |
| GET | /api/sharing | 私人网段与共享地址，authentication_required=false，无需控制权申请 |
| POST | /api/sharing/control | 兼容旧调用，直接返回当前共享状态 |
| POST | /api/sharing/password | 410；已取消访问口令 |

`GET /api/auth` 在共享模式返回 required=false。所选私人网段与本机可以直接读写；不因标签页切换或租约过期取消导航。手动速度的按住/松开与超时归零保持不变。

| 方法 | 路径 | 参数 / 用途 |
| --- | --- | --- |
| GET | /api/platform、/api/status | 状态、真实数据来源与后台任务 |
| GET | /api/catalog、/api/export | 错误码与诊断导出 |
| GET | /api/connections | 脱敏的保存配置与连接状态，不返回代理令牌 |
| POST | /api/connections/save | {aos_ip,aos_port,auto_connect,sensor_source,targets:{nos,gos,sdk,train}} |
| POST | /api/connections/scan | {role}，异步扫描主机指纹 |
| POST | /api/connections/pair | {role, replace?:true}，异步首次配对；replace 仅用于主动更换设备身份 |
| POST | /api/connections/discover | {}，异步查找已配对机器人，返回 found、connections、message |
| POST | /api/connections/trust | {scan_id,confirmed:true} |
| POST | /api/connections/deploy | {role,password?,admin_password?,calibration?,confirmed:true}，异步安装 |
| POST | /api/connections/connect | {mode:demo或live}，使用保存配置 |
| POST | /api/connections/target | {role,action:start或stop}，维护后台连接 |
| POST | /api/connections/service | {role,action:start或stop或restart或status或logs}，异步操作 |

AOS 控制使用 UDP 地址与端口；NOS/GOS 是独立的 SSH/代理通道，可同时连接。SDK 是可选服务的执行端，不代表第四台物理主机。sensor_source 为 nos 或 gos，旧配置默认 nos；不会自动迁移 SDK 或修改原控制器执行位置。

连接检查优先调用受相同令牌保护的机器人代理 `GET /health`（`{ok:true,health_protocol:1}`），不读取地图或启动 ROS。旧代理明确不支持该接口时兼容 `/status`；鉴权失败和传输错误不回退。单次超时显示正在重试，连续超时显示服务无响应。服务在线只表示代理可达；地图、定位、感知新鲜度和运动准备继续由各业务接口独立核对。

目标参数包含 host、user、enabled、ssh_port、local_port、remote_port、python、ros_setup（绝对路径列表）、domain_id；SDK 增加 sdk_workspace、sdk_admin；训练增加 train_root。SDK 管理员身份需在首次部署明确确认。密码仅用于当次请求，连接令牌与专用私钥只保存在电脑后端。

兼容接口 `POST /api/connect {mode,ip,port}`、`POST /api/settings {sdk,train,nos}` 仍可使用，URL 设置会保存本机；启用连接中心目标后优先使用受管理通道。

## 运动、关节与导航

```text
POST /api/cmd {action}                  姿态/步态/模式/灯光/方向/stop/estop
POST /api/velocity {forward,side,yaw}    归一化 [-1,1]，需持续更新
POST /api/joints/preview {leg,height}   只计算抬腿预览，不启用输出
POST /api/joints/arm {rig_confirmed:true} -> session
POST /api/joints/lease {session}        200ms 续约
POST /api/joints/set {session,index,value,duration}
POST /api/joints/leg {session,leg,height,duration}
POST /api/joints/lower {session,duration}
POST /api/joints/pose {session,positions:[16],duration}
POST /api/joints/stop {}
POST /api/pose/save {name}              保存实际反馈
POST /api/route/save {id?,name,map_id,points:[{x,y,yaw,type}]}
POST /api/route/delete {id}
POST /api/nav/start {id,nav_mode,speed,gait,manner}
POST /api/nav/pause {}                  取消当前点并等待对应确认
POST /api/nav/resume {}                 重新下发暂停点
POST /api/nav/cancel {}
POST /api/localize {PosX,PosY,PosZ,Yaw} m / rad
POST /api/charge {charge:0|1|2}
POST /api/sleep {sleep?,auto?,minutes?}
POST /api/maintenance {action:led,...}  或 gps-time / external-time
```

leg 为 FL/FR/HL/HR，height 为 0–0.08 m，duration 为 1–8 s。关节与 SDK 控制互斥，需有效校准与新鲜反馈。自主导航仅允许厂商文档约束下的平地、低速组合；任务完成依据匹配目标编号和时间的新鲜业务结果，不能用通用成功回包推进任务。

## 地图、点云与文件

```text
GET  /api/maps/library
POST /api/maps/details {name}                 原始 ID 映射与文件目录
GET  /api/maps/build/status                   采集/后处理命令状态与日志
POST /api/maps/help {}                        异步读取目标版本 drmap 帮助
POST /api/maps/library/build {action:start|stop,name?} -> job_id
POST /api/maps/library/apply {name,grid_id?,cleanup_previous?:boolean} -> job_id
POST /api/maps/library/rename {name,display_name} -> job_id，修改显示名，内部编号不变
POST /api/maps/library/delete {name}          -> job_id，移入可恢复回收站，禁止删除活动图
GET  /api/maps/library/trash                  -> {maps:[{id,name,display_name,deleted_at,reason}]}
POST /api/maps/library/restore {id}           -> job_id，不覆盖同编号地图，不自动启用
POST /api/maps/library/purge {id,confirmed_name} -> job_id，逐张永久删除已回收地图；名称必须匹配，不可恢复
POST /api/maps/library/sync {name,grid_id}     栅格及真实编号同步到编辑器
POST /api/maps/package/export {name}          -> job_id，结果含文件与 format
POST /api/maps/package/import {id,name}       -> job_id，不覆盖/自动激活
POST /api/maps/import {name,pgm:BASE64,resolution,origin:[x,y,0]}  仅二维编辑器
POST /api/maps/sync {}                        兼容：同步活动栅格 0
POST /api/files/start {name,size,kind,resume_id?} -> file,chunk_size
POST /api/files/chunk {id,offset,data:BASE64}  -> offset
POST /api/files/finish {id,sha256?}            -> job_id
POST /api/files/inspect {id}                  ZIP 检查结果
GET  /api/files                              完成文件列表
GET  /api/files/download/<id>                完整原文件
POST /api/cloud/preview {kind:map|file,key}    -> job_id -> cloud_id
GET  /api/cloud/data/<cloud_id>              采样 XYZ、范围、编码、来源
GET  /api/sensors/status                     可选 GOS/NOS 只读 IMU/ODOM/话题接收状态
POST /api/sensors/cloud {topic}               当前 ROS2 点云采样、frame_id、age
GET  /api/nos/status、/api/nos/logs
GET  /api/gos/status、/api/gos/logs             GOS 主机状态与平台服务日志
POST /api/nos/action {action,name?}           兼容 drmap 路由，同样返回后台任务
```

kind 为 pcd / map-package。id 与 cloud_id 是后端生成的文件标识；不接受任意本机路径。文件最大 4 GiB、块最大 2 MiB。地图导出 format 为 drmap 或 directory-backup。实机导入保持包内原地图目录名，调用厂商 unpack；apply 调用厂商命令与定位服务重启。数据接收新鲜不等于定位正常。

## SDK 软件流程

```text
GET  /api/sdk/status                         流程状态、能力及确认来源
POST /api/sdk/action {action,confirmed?}     prepare/enter/confirm_handset/exit/confirm_exit
GET  /api/sdk/policy/status
GET  /api/sdk/logs
POST /api/sdk/policy/prepare {confirmed:true} -> job_id，官方固定版本源码
POST /api/sdk/policy/build {confirmed:true}   -> job_id，备份并改用 kRos2 编译
POST /api/sdk/policy/start {}                 显式启动厂商控制程序
POST /api/sdk/policy/stop {}                  阻尼并等待 idle 后退出进程
POST /api/sdk/policy/state {state}            stand/rl_control/lie_down/damping
POST /api/sdk/policy/velocity {forward,side,yaw} m/s、m/s、rad/s
POST /api/sdk/policy/direct-stop {}           停止独立关节进程
GET  /calibration.example.json               未验证的厂商参数示例
```

实机 enter 不猜测私有固件指令。confirm_handset 记录操作者已按手柄流程授权切换，状态来源明确为 operator_confirmed；不能宣称是固件确认。代理默认不启动厂商 rl_deploy，也不启动关节输出。策略程序和独立关节控制器互斥。

## 训练与视觉

```text
GET  /api/train/status、/api/train/checkpoints
POST /api/train/preview {kind,task,envs,iterations,seed,checkpoint?}
POST /api/train/start {同 preview}
POST /api/train/stop {id}
GET  /video/front、/video/rear、/video/ppe
POST /api/ppe/start {camera:front|rear}
POST /api/ppe/stop {}
POST /api/faults/inject {code,component}       仅演示
POST /api/faults/clear-demo {}                 仅演示
```

训练 kind 为 train/play/export，task 为 rough/flat。真实执行使用工作站兼容环境；演示曲线为明确标注的合成数据。硬急停只有状态展示，不能在页面解除。没有语义模型/TypeSafe 依赖。

## v2.4 简化流程接口

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | /api/motion/catalog | 三种使用模式、四项现行步态和兼容高台步态、资料依据 |
| POST | /api/maps/start-here | `{name}`；后台任务：停止行进、等待导航停止、切回手动并核对停稳、开始建图 |
| POST | /api/maps/extension-plan | `{base,name,strategy:"new_coverage"}`；保存重新覆盖计划，名称必须不同，不能直接改写原图 |
| GET | /api/maps/extensions | 计划列表、native_extension_supported=false，未提供未经证实的原图续建接口 |
| POST | /api/nav/manual | 暂停路线并切回手动，原路线和下一目标保留，可 resume |
| POST | /api/sdk/setup | 一次性准备、编译、重载 SDK 环境；实机返回后台 job_id |
| POST | /api/sdk/connect | 调用者表明手柄已开启 SDK，自动启动连接服务并等待就绪，再接入或启动 SDK 策略 |

SDK `/status` 增加 environment：ready、stage、message、workspace、input。源码输入选择忽略注释、支持空白及命名空间；无法匹配的工程另建工作区。硬件授权仍使用厂商手柄流程。普通 SDK 策略不使用独立关节标定门槛。

## v2.4.1 AOS 中转

`POST /api/connections/save` 可设置 `aos_ssh_user`、`aos_ssh_port` 和 `targets.nos/gos.via_aos=true`。中转地址沿用 `aos_ip`，目标仍为各内部主机地址；后台隧道、服务调用和首次指纹读取均走同一中转。修改中转地址或账号会重新建立相关通道，保留目标部署状态。SDK / train 保持直接连接。

连接状态新增 `unreachable`；`connecting` 不再被当作通道建立。只有本地转发端口可连接但服务接口失败才报告 `waiting_service`，并给出响应错误；`ready` 需实际读取状态接口。

## v2.5 单点目标、续建与视频

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | /api/nav/goal | `{map_id,x,y,yaw}`；yaw 为弧度。创建内存中的单点目标，固定自主规划、低速、平地、避障；不要求保存路线，完成后自动停止 |
| POST | /api/maps/extend-here | `{base,grid_id:0}`；base 必须为当前活动地图。检查定位、停稳后记录起始位姿，启动独立采集轮次；旧图保持完整 |
| GET | /api/maps/project?name=… | 地图版本与补扫记录（root_map、revision、sessions）、extension_ready、pipeline_version；配准残差不是绝对精度 |
| GET | /api/camera/frame?camera=front&after=-1 | 返回最新 JPEG，`X-Frame-Sequence` 为帧序号、`X-Frame-Age` 为后端接收后的帧龄；单次请求，不累积帧队列 |

续建仍使用 `/api/maps/library/build {action:"stop"}` 结束采集。NOS 在原厂本轮优化完成后调用 map_fusion：起始定位与第一帧建立初值、PCL ICP 配准、质量检查、融合完整点云与栅格、原子发布新目录、切换并重启定位。`/api/maps/build/status` 在融合期间维持 saving，成功时返回 result_maps、fusion、revision。失败保留旧图和本轮扫描，不把原始新扫描误报为融合成功。重复 stop 不会重复发布版本。

补扫不与原厂导航任务并行；本轮建图接管定位/规划服务。跨轮融合保留历史扫描及坐标变换，不等同于对全部历史位姿图再次全局求解。旧 `/api/maps/extension-plan` 仅作为重新建图计划兼容接口。

开发时可用 `--demo --data-dir <独立目录> --http 8088` 验证离线流程，避免修改主工作区。


## v2.6 自主作业

- `GET /api/autonomy/status`：任务状态、run_id、观测栅格、路径、提示与视觉状态。也随 `/api/platform` 返回。
- `POST /api/autonomy/start`：`mode=explore`，可选 `radius`（2–100 m）、`minutes`（0.1–60）；`extend=true, base, grid_id` 表示续建；`use_current=true` 接管当前采集。`scope=map` 会随新观测到的通道逐步扩展初始地图范围（至 100 m），`scope=radius` 使用指定半径。`mode=follow` 使用对应视觉响应的 `sequence, person`（从 0 起）锁定人物。
- `POST /api/autonomy/lease`：兼容旧客户端的 `run_id` 心跳。已启动任务由后台执行，不依赖网页可见性；任务时限、感知有效期、连接归属与显式暂停继续生效。探索范围为 `scope: "radius" | "map"`，旧 `room/floor` 值兼容映射；按地图范围仅用于已有地图更新。
- `POST /api/autonomy/stop`：撤销当前移动任务；探索中的地图采集保留。
- `POST /api/autonomy/save`：停车并请求停止保存，后台核对最终地图结果后才完成。
- `POST /api/autonomy/vision/start`：可选 `model_path`，使用服务器本机已存在的 `.pt` 文件。不会自动下载权重。
- `GET /api/autonomy/vision`：同帧 JPEG data URL、归一化 XYXY 人物框、sequence、源图像 age。慢推理不会刷新源图像时间。
- `POST /api/autonomy/vision/stop`：停止识别及依赖它的跟随任务。
- 机器人代理 `GET /sensors/status` 新增 `autonomy_protocol=1`；`POST /sensors/autonomy {mapping: boolean}` 提供完整雷达归约的 floor/obstacles、0.2 m 栅格、源数据 age 与可选同步 SLAM pose。该接口按需启用约 5 Hz 处理，不改变 1 Hz 的 UI 预览通道。
- `/sensors/autonomy` 可携带 `after_stamp` 表示上次已接收的扫描。尚未读取的新帧立即返回，重复帧短暂等待后继帧；旧代理忽略该可选字段。网络读取失败不会立刻丢弃仍有效的缓存，缓存保留原失效时间。几何错误、过期或连接切换仍会失效。

自主指令继续经过普通运动状态检查和现有速度超时。详细输入契约、模型要求和验证边界见 `自主作业说明.md`。


## v2.6.1 预训练视觉模型

- `GET /api/vision/models`：人员与 PPE 权重是否存在、来源、许可、哈希和类别说明；`/api/platform` 中同样返回 `vision_models`。
- `POST /api/vision/check {kind: "person" | "ppe"}`：后台任务在服务器本机运行对应预训练模型，输入是随包的公开样例，不访问机器人相机。返回 `job_id`，完成后包含实际检测框、分类计数、单次推理耗时和结果图片地址。
- `GET /api/vision/check-image/<id>`：本机样例检测结果 JPEG；适用平台现有访问范围。
- `GET /api/ppe/frame?after=<sequence>`：最新 PPE 检测画面，包含 `X-Frame-Sequence` 与 `X-Frame-Age`，不积压历史帧。仍保留 `/video/ppe` 兼容入口。
- `POST /api/ppe/start {camera: "front" | "rear"}` / `POST /api/ppe/stop`：使用随包 `models/ppe.pt`，或用户显式设置的 `M20_PPE_MODEL` / `--ppe-model`。模型必须包含 Hardhat、NO-Hardhat、Person 类别。演示模式不会连接机器人相机。

人员跟随默认 `models/yolo11n.pt`，自定义路径仍可覆盖。模型和图像在本机运行，不接入来源项目的邮件、Webhook 或云端推理服务。

## v2.6.2 相机恢复与诊断

- `POST /api/camera/reconnect`：`{"camera":"front"}`（或 `rear`），重建选定相机的共享 RTSP 解码连接，不重启机器人服务。
- `GET /api/camera/frame?camera=front&after=<sequence>`：等待更新帧；`X-Frame-Age` 是该帧在电脑完成解码后的年龄，和返回 JPEG 属于同一帧，不含相机曝光到解码的延迟。
- `/api/status` 的 `cameras.front/rear` 增加 `fps`、`skipped`、`decoder_threads`；`fresh` 表示最近一秒有解码图像。前端另外统计实际显示帧率，不用服务端帧率替代浏览器接收帧率。

## v2.7.0 行走优化与路径预览

- `POST /api/maps/refinement`，`action` 为 `enable`、`disable` 或 `generate`。开启不启动 SLAM，不改变运动状态，不切换活动地图。`/api/status.map_learning` 返回基准地图、开关、当前批次/累计帧、状态、候选版本。生成的是完整融合图；启用复用现有 `/api/maps/library/apply`。
- `POST /api/nav/plan`：`map_id`、`x`、`y`、可选 `yaw`（省略/null 为沿路径）。只计算预览，不下发运动。返回 `plan_id`、`path`、自动 `points`、长度和实时障碍来源说明。
- `POST /api/nav/goal`：传 `map_id` 与 `plan_id` 执行已预览路线。兼容直接传 x/y/yaw，但仍先做地图检查与规划。计划有效期 60 秒，地图和起点需匹配。
- NOS `/maps/refinement` 提供状态/开启/关闭/续约/生成；感知服务 `/sensors/observation` 提供同步完整定位点云与质量数据，均不发布运动指令。

## v2.7.1 人数统计

- `POST /api/people`：`action` 为 `start`、`pause`、`resume`、`reset`、`stop` 或 `configure`。启动参数 `camera` 为 front/rear，`conf` 为 0.1–0.9（默认 0.3）。切换相机需先停止，重新启动会开始新一轮。
- `/api/status.people_counts` 返回 `current`（过期/暂停时 null）、`peak`、`total_tracks`、`enabled`、`paused`、`fresh`、`fps`、`infer_ms`、`age` 等。累计值是轨迹数量，不是身份去重人数。
- `GET /api/people/frame?after=<sequence>`：共享的最新带框 JPEG；含 `X-Frame-Sequence`、`X-Frame-Age`。
- `GET /api/people/snapshot`：下载新鲜统计画面，不保存陈旧画面冒充实时截图。
- `POST /api/vision/check` 传 `{"kind":"counting"}` 可在未连接机器人时使用附带公开图片验证 YOLO12n。

## 2026-11-01 平台单点执行器（离线已验收，实机待同步）

`POST /api/nav/goal` 的既有 `map_id`、`plan_id` 保留，新增 `engine: "platform" | "native"`。省略时保留原 API 的 native 行为；导航页面显式传入所选执行器，默认 platform。platform 调用电脑全局规划和实时轨迹控制，通过普通模式三轴接口驱动；native 调用原厂目标接口。任务过程中不自动切换执行器。

平台任务返回 `run_id`；`POST /api/nav/lease` 保留兼容旧客户端，后台导航不依赖浏览器续租。感知失效会停止输出，任务时限和连接切换仍生效。`/api/nav/pause`、`/api/nav/cancel`、`/api/nav/resume` 和停止按钮沿用；恢复平台任务从当前位姿重新规划。

状态 `nav.engine=platform`，包括 `controller`（候选数、选中轨迹、归一化 axes、计算耗时）、`replans`、`distance_remaining` 与 `progress_detail`。完成要求到达且取得新鲜实际停稳反馈，不使用原厂旧目标 ACK。`dense_path` 中 yaw 是机身朝向，侧移/倒退时不必等于路径切线。

自主感知扩展 `geometry_version=2`、`obstacle_bounds`（每个障碍格内实测点 XY 边界，按 obstacles 顺序）、`measured_floor`（含混合格中的实际地面观测）。旧字段保留。地图候选始终由用户手动启用。

## 实时通行、起点恢复与显示定位

`POST /api/nav/plan` 接受 `engine: "platform" | "native"`。平台模式用连续新鲜地面观测和定位读回来构建临时通行层；原厂模式保留原厂执行语义。执行器改变后必须重新预览，不能把平台临时通路交给原厂当作同一路径证明。

规划结果可包含 `recovery`：`required`、`mode`、`prefix`、`recovery_distance_m`、`constraints`。预览只是计算结果；平台在执行前重新核对扫描，先完成独立、有限的平移退让与实际停稳，再从实际位置接回原目标。无法安全自动执行的近墙旋转、多段变向或真实机身冲突会给出原因。

`requires_live_revalidation` 表示参考路径使用了规划快照。较长的纯计算不会反复被单帧有效期打断，但执行阶段仍须重新获得有效通路；预览本身不续传感器或运动租约。

`GET /api/status` 新增：

- `server_instance`：本次后台实例标识，用于核对本机和共享网址是否访问同一后台。
- `pose_view.display_pose`：仅用于地图标记的显示位置；可能为空。
- `pose_view.health`：定位未就绪、过期、小噪声平滑、漂移或跳变状态。
- `pose`：未平滑的原始控制定位，仍供原有控制和诊断使用。
- `autonomy.observation_pump`：接收、发布、重复、乱序、过期及迟到响应计数。缓存保持原始截止时间，重复帧不能延长有效期。

字段中的单调时钟期限只属于生成它的进程，不得跨电脑直接比较或当作新的运动授权。

地图保存结果区分 `observations_saved`、`fusion_status=awaiting_evidence`、`candidate_created` 和 `result_maps`。只保存原始观测时不得显示为融合候选已可导航；原图版本始终由操作者启用。

## 驻机断联任务

导航/路线选项增加 `disconnect_policy: stop|finish|return|finish_return`，默认stop。后三项需机器人端报告驻机执行能力；使用既有准备与停止接口。GET `/api/nav/resident/capabilities` 核对配置，快照 `resident_navigation` 反映远端任务状态。首次下发完整路线；重连仅对账，禁止自动重播启动。详见 [断联任务](docs/disconnected-navigation.md)。

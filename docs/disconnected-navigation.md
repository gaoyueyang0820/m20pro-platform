# 导航与巡检的断联策略

单点导航与多点巡检由 GOS 机载寻路执行。电脑持续失联 5 秒时，GOS 请求停车。平台维护任务心跳，浏览器负责显示任务状态。普通巡检重新执行时采用当前的断联停车策略。 [机载寻路与巡检](gos-patrol.md)

## 任务状态

重连时，电脑先核对原任务与停车反馈。操作者点击「结束任务」后，收到对应回执并观察到停稳反馈，页面才显示停止完成。雷达、定位和控制反馈由 GOS 在机内持续核对。

## 接口

- `GET /api/nav/resident/capabilities` 查看 GOS 机载服务能力。
- `POST /api/nav/cancel` 请求结束当前任务。平台快照的 `resident_navigation` 同步机载状态。
- GOS 提供 `/missions/capabilities`、`/missions/status`、`/missions/start`、`/missions/pulse` 和 `/missions/cancel`。已有 NOS 驻机任务的引用仍可用于状态与停车核对。

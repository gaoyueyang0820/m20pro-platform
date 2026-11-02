"""M20 Pro 打招呼能力：灯语与原地动作序列的纯逻辑生成。Python 3.9+。

本模块不做任何 I/O：只生成待发送的协议帧与按 20Hz 采样的轴指令序列；
发送、保活与安全仲裁由调用方完成（独立用法见 m20_greet.py）。

协议依据（查证记录见 docs/greeting.md，均为官方支持站 2026-10-23 版）：
- 自定义灯语 Type=1101 Command=14：CustomMode 开关；Led[0]=头灯 Led[1]=尾灯；
  Led[].Type：0常亮 / 1填充流水 / 2移动流水 / 4呼吸 / 5单色闪烁；
  Led[].Color：0黑 / 1白 / 2绿 / 3蓝；Cycle 为整数秒，仅呼吸/闪烁生效。
  CustomMode=false 恢复默认灯效。
- 轴指令 Type=2 Command=21：仅常规/辅助模式；各轴归一化 [-1,1]；
  建议 20Hz；官方注明"当前开放的四种移动步态仅有 X、Y、Yaw 生效"，
  Z/Pitch 在站立静止状态下的行为未经文档背书，相关序列仅供实机验证。
- 超时保护：超过 500ms 未收到新的速度指令，机器人自动减速进入安全状态，
  因此任何轴序列结束后必须跟随一段零轴帧（≥0.4s）。
- 运动状态 MotionState：站立=1 / 关节阻尼/软急停=2 / 趴下=4 / RL控制=17。
"""
import json
import math
import struct

SYNC = b"\xEB\x91\xEB\x90"          # 帧头（官方 SDK 源码确认：EB 91 外部用户）
ASDU_FORMAT_JSON = 0x01

AXIS_TYPE, AXIS_CMD = 2, 21          # 运动控制（轴指令）
LIGHT_TYPE, LIGHT_CMD = 1101, 14     # 自定义灯语

# MotionState 取值（Type=1002 Cmd=6 BasicStatus）
STATE_IDLE, STATE_STAND, STATE_ESTOP, STATE_BOOT_DAMP, STATE_SIT, STATE_RL = 0, 1, 2, 3, 4, 17

# 灯语预设：名称 -> (Led[].Type, Color, Cycle 整数秒)
LIGHT_STYLES = {
    'breathe': (4, 2, 1),   # 绿色呼吸灯（最直观的"友好"信号）
    'flow':    (2, 3, 1),   # 蓝色移动流水灯（移动流水不依赖 Cycle）
    'blink':   (5, 1, 1),   # 白色单色闪烁（Cycle 整数秒，最短 1s）
    'solid':   (0, 2, 1),   # 绿色常亮
}

AXIS_HZ = 20                         # 官方建议 20Hz
ZERO_HOLD_S = 0.4                    # 序列结束后零轴保持时长（覆盖 500ms 超时保护）


def frame(type_, command, items, seq=0, at=None):
    """组 16 字节协议头 + JSON ASDU 的完整 UDP 帧。"""
    import time as _time
    body = json.dumps(
        {"PatrolDevice": {"Type": type_, "Command": command,
                          "Time": at or _time.strftime("%Y-%m-%d %H:%M:%S"),
                          "Items": items}},
        separators=(",", ":")).encode("utf-8")
    if len(body) > 65535:
        raise ValueError("ASDU 过长")
    header = SYNC + struct.pack("<HH", len(body), seq & 0xFFFF) \
        + bytes([ASDU_FORMAT_JSON]) + b"\x00" * 7
    return header + body


def _led_entry(led_type, color, cycle):
    if led_type not in (0, 1, 2, 4, 5):
        raise ValueError("灯光模式 Type 只能是 0/1/2/4/5")
    if color not in (0, 1, 2, 3):
        raise ValueError("灯光颜色 Color 只能是 0/1/2/3")
    if led_type in (4, 5) and int(cycle) < 1:
        raise ValueError("呼吸/闪烁的 Cycle 必须是 ≥1 的整数秒")
    return {"Type": led_type, "Color": [color], "Cycle": int(cycle)}


def light_on_items(style='breathe', tail=None):
    """打招呼灯语 Items（Type=1101 Command=14）。tail 缺省与头灯同款。"""
    if style not in LIGHT_STYLES:
        raise ValueError("灯语预设必须是 %s" % '/'.join(LIGHT_STYLES))
    led_type, color, cycle = LIGHT_STYLES[style]
    head = _led_entry(led_type, color, cycle)
    tail_entry = head if tail is None else _led_entry(*tail)
    return {"CustomMode": True, "Led": [head, tail_entry]}


def light_off_items():
    """退出自定义灯语，恢复默认灯效。"""
    return {"CustomMode": False}


def _axis_items(x=0., y=0., z=0., roll=0., pitch=0., yaw=0.):
    values = dict(X=x, Y=y, Z=z, Roll=roll, Pitch=pitch, Yaw=yaw)
    for name, value in values.items():
        if not -1.0 <= value <= 1.0:
            raise ValueError("轴 %s 超出 [-1,1]：%s" % (name, value))
    return {k: round(v, 3) for k, v in values.items()}


# 动作幅度（归一化 [-1,1]，表示相对最大速度的比例）。
# 2026-11-02 按用户要求放大，同时放慢周期：幅度大 + 周期短 会让机身甩得太急。
NOD_AMPLITUDE = .5      # 俯仰点头幅度（旧 .25）
NOD_PERIOD_S = 1.6      # 单次点头周期（旧 1.0）
BOB_AMPLITUDE = .45     # 升降起伏幅度（旧 .2）
BOB_PERIOD_S = 1.8      # 单次起伏周期（旧 1.2）


def nod_plan(amplitude=NOD_AMPLITUDE, periods=2, period_s=NOD_PERIOD_S, hz=AXIS_HZ):
    """站立状态下用 Pitch 轴"点头"的 20Hz 轴指令帧序列（实验性）。

    官方注明移动步态下 Z/Pitch 不生效；站立静止状态未获文档背书，
    上线前必须在实机按 docs/greeting.md 的验证清单确认。
    默认幅度 0.5（50% 最大俯仰角速度）、单周期 1.6s，全程 X=Y=Z=Roll=Yaw=0，
    不产生任何位移或转向。
    """
    if not 0 < amplitude <= .7:
        raise ValueError("点头幅度限制在 (0, 0.7]")
    if not 0 < periods <= 4:
        raise ValueError("点头次数限制在 (0, 4]")
    total = hz * periods * period_s
    frames = []
    for k in range(int(total)):
        t = k / hz
        frames.append(_axis_items(pitch=amplitude * math.sin(2 * math.pi * t / period_s)))
    return frames


def bob_plan(amplitude=BOB_AMPLITUDE, periods=2, period_s=BOB_PERIOD_S, hz=AXIS_HZ):
    """站立状态下用 Z 轴"起伏蹲起"的 20Hz 轴指令帧序列（实验性）。"""
    if not 0 < amplitude <= .6:
        raise ValueError("起伏幅度限制在 (0, 0.6]")
    if not 0 < periods <= 4:
        raise ValueError("起伏次数限制在 (0, 4]")
    total = hz * periods * period_s
    frames = []
    for k in range(int(total)):
        t = k / hz
        frames.append(_axis_items(z=amplitude * math.sin(2 * math.pi * t / period_s)))
    return frames


def zero_plan(seconds=ZERO_HOLD_S, hz=AXIS_HZ):
    """零轴保持帧：任何动作序列结束后发送，触发机器人安全减速停稳。"""
    count = max(1, int(seconds * hz))
    return [_axis_items() for _ in range(count)]


GAITS = (0x1001, 0x1003, 0x3002, 0x3003)   # 基础 / 标准楼梯 / 敏捷平地 / 敏捷楼梯


def locomotion_snapshot(basic):
    """打招呼前记录运行状态，供动作结束后恢复。

    实机现象（2026-11-02）：下发 Z/Pitch 轴指令后，机器人从敏捷平地（0x3002）
    变成 0x1003（标准运动模式楼梯），此后按原方式行走失效——标准运动模式不走
    RL 步态。因此必须记住步态与使用模式，动作结束后显式发回去。
    """
    status = (basic or {}).get('BasicStatus') or basic or {}
    gait = status.get('Gait')
    mode = status.get('ControlUsageMode')
    return dict(gait=gait if gait in GAITS else None,
                mode=mode if mode in (0, 1, 2) else None)


def restore_commands(snapshot):
    """生成"恢复原运行状态"的指令表 [(Type, Command, Items, label)]。

    顺序：先使用模式、再步态——官方说明切换使用模式会把步态重置为基础步态，
    所以步态必须最后发。调用方负责按顺序发送并留出确认时间。
    """
    out = []
    if not isinstance(snapshot, dict):
        return out
    mode = snapshot.get('mode')
    if mode in (0, 1, 2):
        out.append((1101, 5, {"Mode": mode}, "恢复使用模式 %d" % mode))
    gait = snapshot.get('gait')
    if gait in GAITS:
        out.append((2, 23, {"GaitParam": gait}, "恢复步态 0x%04X" % gait))
    return out


def precheck(basic, motion=None):
    """打招呼前置安全检查。

    basic/motion 为 Type=1002 Cmd=6 / Cmd=4 回报中的 Items（平台 record 的
    items 或独立脚本的 snapshots）。返回 (ok, message)。
    灯语本身不依赖运动状态（趴着也能亮灯），但点头/起伏必须可遥控且无故障。

    状态语义（与平台既有判据一致）：
    - **17 = RL 控制**：完成站立后进入的正常可遥控状态（标准/敏捷运动模式）。
      平台 `platform_core._velocity_tick` 正是要求 MotionState==17 才下发轴指令，
      `point_navigation` 也要求 ==17 才允许导航。**这是打招呼允许的状态。**
    - 1 = 站立（起立过渡/站立态），允许。
    - 0 空闲 / 2 关节阻尼软急停 / 3 开机阻尼 / 4 趴下：不允许动作类问候。
    """
    if not isinstance(basic, dict):
        return False, '未收到基础状态（1002/6），无法确认机器人状态'
    basic_status = basic.get('BasicStatus') or basic
    state = basic_status.get('MotionState')
    if basic_status.get('HES') not in (0, None):
        return False, '急停/故障标志 HES 非 0，先排除故障再打招呼'
    if basic_status.get('Sleep'):
        return False, '机器人处于休眠状态，先唤醒'
    if state == STATE_SIT:
        return False, '机器人正趴下（MotionState=4）：请先起立，或只用灯语问好'
    if state == STATE_ESTOP:
        return False, '机器人处于关节阻尼/软急停（MotionState=2），禁止动作类问候'
    if state in (STATE_IDLE, STATE_BOOT_DAMP):
        return False, '机器人未进入可遥控状态（MotionState=%s），请先起立' % state
    if state not in (STATE_RL, STATE_STAND):
        return False, '机器人未确认站立/可遥控（MotionState=%s），动作类问候不安全' % state
    if motion is not None and isinstance(motion, dict):
        motion_status = motion.get('MotionStatus') or motion
        moving = any(abs(motion_status.get(k, 0) or 0) > limit
                     for k, limit in (('LinearX', .08), ('LinearY', .08), ('OmegaZ', .12)))
        if moving:
            return False, '机器人仍在运动，请停稳后再打招呼'
    mode = basic_status.get('ControlUsageMode')
    if mode not in (0, 2, None):
        return False, '当前使用模式（%s）下轴指令无效，请切回常规模式' % mode
    return True, '就绪'

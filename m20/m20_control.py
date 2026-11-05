#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEEP Robotics Lynx M20 / M20 Pro —— 站立 / 趴下 控制脚本

协议依据：《Lynx M20 Series Software Development Manual》巡检通信协议 V0.0.4-0
  APDU = 16 字节协议头 + ASDU(JSON)
  协议头 = 4B同步(EB 91 EB 90) + 2B长度(小端) + 2B消息ID(小端) + 1B格式(0x01) + 7B保留
  机器人 = 服务器：UDP 10.21.31.103:30000   TCP 10.21.31.103:30001

用法（在 MobaXterm 的终端里执行，PC 需与机器人同网段）：
    python m20_control.py stand          # 站起来
    python m20_control.py lie            # 趴下/坐下
    python m20_control.py estop          # 软急停（也是趴下）
    python m20_control.py status         # 只看状态，不动作
可选参数：
    --ip 10.21.31.103   机器人 IP
    --port 30000        端口（--tcp 时用 30001）
    --tcp               使用 TCP（默认 UDP）
"""
import argparse
import json
import socket
import struct
import sys
import time

# ---- 协议常量 ----
SYNC = b"\xEB\x91\xEB\x90"   # 帧头前4字节，已由官方 SDK 源码确认：EB 91(外部用户) EB 90
ASDU_FORMAT_JSON = 0x01

# MotionParam 取值（Type=2, Command=22 运动状态切换）
MP_IDLE = 0        # 空闲
MP_STAND = 1       # 站立
MP_ESTOP = 2       # 软急停（趴下）
MP_DAMPING = 3     # 上电阻尼
MP_SIT = 4         # 坐/趴下
MP_STANDARD = 6    # 标准站立

_seq = 0


def _asdu(type_, command, items=None):
    return {
        "PatrolDevice": {
            "Type": type_,
            "Command": command,
            "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Items": items or {},
        }
    }


def apdu(type_, command, items=None):
    """组包：16 字节协议头 + JSON ASDU"""
    global _seq
    body = json.dumps(_asdu(type_, command, items),
                      separators=(",", ":")).encode("utf-8")
    if len(body) > 65535:
        raise ValueError("ASDU 过长")
    header = SYNC + struct.pack("<HH", len(body), _seq) \
        + bytes([ASDU_FORMAT_JSON]) + b"\x00" * 7
    _seq = (_seq + 1) & 0xFFFF
    return header + body


def heartbeat():
    return apdu(100, 100)


def switch_mode(mode):
    """Type=1101 Command=5 使用模式：0=常规(可发轴指令) 1=导航"""
    return apdu(1101, 5, {"Mode": mode})


def switch_motion(param):
    """Type=2 Command=22 运动状态切换"""
    return apdu(2, 22, {"MotionParam": param})


def parse(data):
    if len(data) < 16:
        return None
    length = struct.unpack("<H", data[4:6])[0]
    body = data[16:16 + length]
    try:
        return json.loads(body.decode("utf-8"))
    except Exception:
        return None


def main():
    p = argparse.ArgumentParser(description="Lynx M20 Pro 站立/趴下控制")
    p.add_argument("action", choices=["stand", "lie", "estop", "status"])
    p.add_argument("--ip", default="10.21.31.103")
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--tcp", action="store_true", help="使用 TCP（端口默认 30001）")
    args = p.parse_args()

    port = args.port if args.port else (30001 if args.tcp else 30000)
    addr = (args.ip, port)

    if args.tcp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect(addr)
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)

    def send(buf):
        if args.tcp:
            sock.sendall(buf)
        else:
            sock.sendto(buf, addr)

    # 1) 先发心跳，让机器人把实时状态回报到本机 IP:端口
    for _ in range(3):
        send(heartbeat())
        time.sleep(0.05)

    # 2) 如果是动作指令，先确保处于常规模式，再下发运动状态
    actions = {"stand": MP_STAND, "lie": MP_SIT, "estop": MP_ESTOP}
    if args.action in actions:
        send(switch_mode(0))
        time.sleep(0.1)
        send(switch_motion(actions[args.action]))
        print("已下发指令: %s (MotionParam=%d) -> %s" % (
            args.action, actions[args.action], addr))

    # 3) 收 3 秒状态回包（心跳之后机器人会主动上报）
    t0 = time.time()
    while time.time() - t0 < 3.0:
        try:
            data = sock.recv(65535) if args.tcp else sock.recvfrom(65535)[0]
        except socket.timeout:
            continue
        msg = parse(data)
        if not msg:
            continue
        pd = msg.get("PatrolDevice", {})
        cmd = pd.get("Command")
        if cmd == 6:      # 基础状态
            print("基础状态:", pd.get("Items"))
        elif cmd == 4:    # 运动控制状态
            print("运动状态:", pd.get("Items"))
    sock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

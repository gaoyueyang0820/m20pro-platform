#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEEP Robotics Lynx M20 Pro —— 打招呼脚本（灯语 + 可选点头/起伏）

只用 Python 标准库，在 PC 上执行（与机器人同网段）：
    python m20_greet.py                    # 灯语问好（绿色呼吸 4s，最安全档）
    python m20_greet.py --style flow       # 蓝色流水灯
    python m20_greet.py --style blink      # 白色闪烁
    python m20_greet.py --nod              # 灯语 + 俯仰点头（实验性，需实机验证）
    python m20_greet.py --bob              # 灯语 + 升降起伏（实验性，需实机验证）
    python m20_greet.py --status           # 只看状态不动作
可选：
    --ip 10.21.31.103  --port 30000  --seconds 4  --stand（趴下时先自动起立）

安全设计（依据官方《软件开发指南》/ basic_server 运动控制文档，见 docs/greeting.md）：
  1. 动作类问候前强制自检：MotionState=站立(1)、HES=0、未休眠、非 RL 接管、已停稳。
  2. 全程不产生位移：点头/起伏序列 X=Y=Yaw=Roll=0，原地执行。
  3. 轴指令 20Hz 连续下发，结束后零轴保持 0.4s（官方 500ms 超时保护兜底）。
  4. Ctrl+C / 异常退出时立即恢复默认灯效并归零所有轴。
  5. 点头/起伏依赖 Z/Pitch 轴，官方注明移动步态下不生效，站立状态下需实机验证。
"""
import argparse
import json
import socket
import sys
import threading
import time

import greeting as g

MP_STAND = 1


def apdu(type_, command, items, seq):
    return g.frame(type_, command, items, seq)


def heartbeat():
    return g.frame(100, 100, {}, 0)


def switch_motion(param):
    return g.frame(2, 22, {"MotionParam": param}, 0)


def switch_mode(mode):
    return g.frame(1101, 5, {"Mode": mode}, 0)


class Link:
    """极简 UDP 链路：发指令 + 收状态快照（独立于平台，便于现场排障时单独使用）。"""

    def __init__(self, ip, port):
        self.ip, self.port = ip, port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.seq = 0
        self.snapshots = {}      # Command -> Items
        self.last_rx = 0.
        self._stop = False
        threading.Thread(target=self._rx, daemon=True).start()
        self._hb_until = 0.

    def _next_seq(self):
        with self.lock:
            self.seq = (self.seq + 1) & 0xFFFF
            return self.seq

    def send(self, buf):
        self.sock.sendto(buf, (self.ip, self.port))

    def command(self, type_, cmd, items):
        self.send(apdu(type_, cmd, items, self._next_seq()))

    def _hb(self):
        """打招呼期间维持心跳 ≥1Hz，让机器人持续回报状态。"""
        while time.time() < self._hb_until and not self._stop:
            self.send(heartbeat())
            time.sleep(0.33)

    def _rx(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            if addr[0] != self.ip:
                continue
            msg = self._parse(data)
            if not msg:
                continue
            pd = msg.get("PatrolDevice", {})
            if not isinstance(pd.get("Items"), dict):
                continue
            with self.lock:
                self.snapshots[pd.get("Command")] = pd["Items"]
                self.last_rx = time.time()

    @staticmethod
    def _parse(data):
        if len(data) < 16 or data[:4] != g.SYNC:
            return None
        try:
            obj = json.loads(data[16:])
        except Exception:
            return None
        pd = obj.get("PatrolDevice")
        if not isinstance(pd, dict):
            return None
        return obj

    def record(self, cmd):
        with self.lock:
            return self.snapshots.get(cmd)

    def fresh(self, max_age=2.0):
        return self.last_rx and time.time() - self.last_rx < max_age

    def close(self):
        self._stop = True
        try:
            self.sock.close()
        except OSError:
            pass


def wait_state(link, seconds=2.5):
    t0 = time.time()
    while time.time() - t0 < seconds:
        basic = link.record(6)
        if basic and link.fresh():
            return basic, link.record(4)
        time.sleep(0.05)
    return None, None


def ensure_stand(link, args):
    """确认站立；趴下时可 --stand 自动起立并等待。返回 (ok, message)。"""
    basic, motion = wait_state(link)
    if basic is None:
        return False, '未收到机器人状态回报：请核对 IP/网段，并确认已打开手柄或已上电'
    basic_status = basic.get("BasicStatus") or basic
    state = basic_status.get("MotionState")
    print("  当前状态: MotionState=%s HES=%s Mode=%s" % (
        state, basic_status.get("HES"), basic_status.get("ControlUsageMode")))
    if state == MP_STAND:
        return True, "站立"
    if state == 4 and getattr(args, "stand", False):
        print("  正趴下 → 下发起立指令，等待约 8s ...")
        link.command(1101, 5, {"Mode": 0})
        time.sleep(0.1)
        link.command(2, 22, {"MotionParam": MP_STAND})
        t0 = time.time()
        while time.time() - t0 < 10:
            time.sleep(0.4)
            basic, _ = wait_state(link, 0.5)
            if basic and (basic.get("BasicStatus") or basic).get("MotionState") == MP_STAND:
                return True, "已起立"
        return False, "起立未确认，放弃动作类问候"
    ok, message = g.precheck(basic, motion)
    return ok, message


def run_light(link, style, seconds):
    """灯语：每秒重发一次保活，结束后恢复默认灯效。"""
    print("  灯语 [%s] 持续 %ss ..." % (style, seconds))
    items = g.light_on_items(style)
    deadline = time.time() + seconds
    while time.time() < deadline:
        link.command(g.LIGHT_TYPE, g.LIGHT_CMD, items)
        time.sleep(1.0)
    link.command(g.LIGHT_TYPE, g.LIGHT_CMD, g.light_off_items())
    print("  灯语已恢复默认")


def run_frames(link, frames):
    """按 20Hz 发送轴指令帧序列，随后零轴保持。"""
    period = 1.0 / g.AXIS_HZ
    began = time.monotonic()
    for k, items in enumerate(frames):
        target = began + k * period
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        link.command(g.AXIS_TYPE, g.AXIS_CMD, items)
    for items in g.zero_plan():
        link.command(g.AXIS_TYPE, g.AXIS_CMD, items)
        time.sleep(period)


def restore(link, before):
    """动作结束后把步态/使用模式发回原值并确认（轴指令会切换到标准运动模式）。"""
    commands = g.restore_commands(before)
    if not commands:
        return
    for type_, cmd, items, label in commands:
        link.command(type_, cmd, items)
        print("  已发送：" + label)
        time.sleep(0.5)
    deadline = time.time() + 3
    while time.time() < deadline:
        basic = link.record(6) or {}
        gait = (basic.get("BasicStatus") or basic).get("Gait")
        if gait == before.get("gait"):
            print("  ✓ 步态已恢复 0x%04X" % gait)
            return
        time.sleep(0.2)
    basic = link.record(6) or {}
    gait = (basic.get("BasicStatus") or basic).get("Gait")
    print("  ⚠ 步态恢复未确认（当前 0x%04X，期望 0x%04X），请手动切回" % (gait or 0, before.get("gait") or 0))


def main():
    p = argparse.ArgumentParser(description="M20 Pro 打招呼")
    p.add_argument("--ip", default="10.21.31.103")
    p.add_argument("--port", type=int, default=30000)
    p.add_argument("--style", default="breathe", choices=sorted(g.LIGHT_STYLES))
    p.add_argument("--seconds", type=float, default=4.0, help="灯语持续秒数")
    p.add_argument("--nod", action="store_true", help="附加俯仰点头（实验性）")
    p.add_argument("--bob", action="store_true", help="附加升降起伏（实验性）")
    p.add_argument("--stand", action="store_true", help="趴下时先自动起立")
    p.add_argument("--status", action="store_true", help="只看状态不动作")
    args = p.parse_args()

    link = Link(args.ip, args.port)
    link._hb_until = time.time() + 600
    threading.Thread(target=link._hb, daemon=True).start()

    def cleanup():
        link.command(g.LIGHT_TYPE, g.LIGHT_CMD, g.light_off_items())
        for items in g.zero_plan(0.2):
            link.command(g.AXIS_TYPE, g.AXIS_CMD, items)

    try:
        # 1) 先发三次心跳建立回报通道
        for _ in range(3):
            link.send(heartbeat())
            time.sleep(0.05)

        basic, motion = wait_state(link)
        if args.status:
            print("BasicStatus:", basic)
            print("MotionStatus:", motion)
            return
        if basic is None:
            print("✗ 未收到状态回报（检查 IP 是否为 %s、PC 是否同网段）" % args.ip)
            return

        want_motion = args.nod or args.bob
        ok, message = ensure_stand(link, args)
        if not ok:
            print("✗ 前置检查未通过：%s" % message)
            if not want_motion:
                print("  → 灯语不依赖运动状态，改用纯灯语问好")
                run_light(link, args.style, args.seconds)
            cleanup()
            return
        print("✓ 前置检查通过：%s" % message)

        # 记下原始步态/使用模式：轴指令会把机器人切到标准运动模式，动作后要恢复
        before = g.locomotion_snapshot(link.record(6))
        if before.get("gait"):
            print("  原始步态 0x%04X，打招呼结束后会恢复" % before["gait"])

        # 2) 灯语问好（最可靠的打招呼方式）
        run_light(link, args.style, args.seconds)

        # 3) 可选动作类问候（站立静止状态下执行，全程原地）
        if args.nod:
            print("  点头（实验性，移动步态下 Pitch 不生效，请观察实机反应）...")
            run_frames(link, g.nod_plan())
        if args.bob:
            print("  起伏（实验性，移动步态下 Z 不生效，请观察实机反应）...")
            run_frames(link, g.bob_plan())
        if want_motion:
            restore(link, before)
            print("✓ 打招呼完成，已归零、恢复默认灯效与原始步态")
        else:
            print("✓ 灯语问好完成")
    except KeyboardInterrupt:
        print("\n[中断] 正在恢复灯效并归零 ...")
        cleanup()
    except Exception as exc:
        print("✗ 异常：%s（正在清理）" % exc)
        cleanup()
        raise
    finally:
        time.sleep(0.2)
        link.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)

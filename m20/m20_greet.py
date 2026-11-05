#!/usr/bin/env python3
"""Standalone M20 greeting CLI; LEDs by default, explicit experimental axes.

Requires exclusive operator control: this UDP client cannot arbitrate another
client. Zero axes request stopping; they do not prove physical immobility.
"""
import argparse
import copy
import ipaddress
import json
import socket
import struct
import sys
import threading
import time

import m20.greeting as g


def apdu(type_, command, items, seq):
    return g.frame(type_, command, items, seq)


def heartbeat():
    return g.frame(100, 100, {}, 0)


class Link:
    """UDP receive records are independently timed and keyed by (Type, Command)."""
    def __init__(self, ip, port):
        self.ip, self.port = str(ipaddress.IPv4Address(ip)), port
        g._integer(port, 'port', 1, 65535)
        self.lock = threading.Lock()
        self.seq = 0
        self.snapshots = {}
        self.error = None
        self.axes_sent = self.light_sent = False
        self.stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('0.0.0.0', 0))
            self.sock.settimeout(.2)
        except BaseException:
            self.sock.close()
            raise
        self.threads = [threading.Thread(target=self._rx, daemon=True),
                        threading.Thread(target=self._hb, daemon=True)]
        for thread in self.threads:thread.start()

    def send(self, buf):
        self.sock.sendto(buf, (self.ip, self.port))

    def command(self, type_, cmd, items):
        with self.lock:
            self.seq = (self.seq + 1) & 0xffff
            seq = self.seq
        self.send(apdu(type_, cmd, items, seq))

    def _hb(self):
        while not self.stop.is_set():
            try:
                self.command(100, 100, {})
                self.command(1002, 6, {})
                self.command(1002, 4, {})
            except OSError as exc:
                self.error = str(exc)
                return
            self.stop.wait(.25)

    @staticmethod
    def _parse(data):
        if len(data) < 16 or data[:4] != g.SYNC or data[8] != 1:return None
        if struct.unpack_from('<H', data, 4)[0] != len(data)-16:return None
        try:
            obj = json.loads(data[16:])
            pd = obj.get('PatrolDevice') if isinstance(obj, dict) else None
            if not isinstance(pd, dict) or not isinstance(pd.get('Items'), dict):return None
            for key in ('Type', 'Command'):
                g._integer(pd.get(key), key, 0, 65535)
        except (ValueError, TypeError, UnicodeError):return None
        return obj

    def _accept(self, data, addr):
        # Never let an unrelated UDP peer freshen the robot's feedback.
        if addr[0] != self.ip:return
        obj = self._parse(data)
        if obj is None:return
        pd = obj['PatrolDevice']
        with self.lock:
            self.snapshots[(pd['Type'], pd['Command'])] = dict(
                items=copy.deepcopy(pd['Items']), at=time.time(), monotonic_at=time.monotonic())

    def _rx(self):
        while not self.stop.is_set():
            try:data, addr = self.sock.recvfrom(65535)
            except socket.timeout:continue
            except OSError as exc:
                if not self.stop.is_set():self.error = str(exc)
                return
            self._accept(data, addr)

    def record(self, type_, cmd):
        with self.lock:return copy.deepcopy(self.snapshots.get((type_, cmd), {}))

    def close(self):
        self.stop.set()
        self.sock.close()
        for thread in self.threads:thread.join(timeout=.35)


def wait_state(link, seconds=2.5):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if link.error:raise RuntimeError('连接异常：'+link.error)
        basic = link.record(1002, 6)
        if basic and g.feedback(basic)[1] <= 2:return basic, link.record(1002, 4)
        time.sleep(.05)
    return {}, {}


def _signature(basic):
    status = g._basic(basic)
    return (status.get('ControlUsageMode'), status.get('Gait'), status.get('Direction'))


def ready(link, before=None):
    if link.error:raise RuntimeError('连接异常：'+link.error)
    basic, motion = link.record(1002, 6), link.record(1002, 4)
    ok, reason = g.precheck(basic, motion)
    if not ok:raise RuntimeError(reason)
    if before is not None and _signature(basic) != before:
        raise RuntimeError('使用模式、步态或方向已变化；取消问候并保留当前设置')
    return basic, motion


def ensure_stand(link, args):
    """Only explicit --stand may request standing; never changes mode/gait."""
    basic, motion = wait_state(link)
    state = g._basic(basic).get('MotionState')
    if state == g.STATE_RL:return g.precheck(basic, motion)
    if state == g.STATE_SIT and args.stand:
        candidate = copy.deepcopy(basic)
        status = g._basic(candidate)
        status['MotionState'] = g.STATE_RL
        ok, reason = g.precheck(candidate, motion)
        if not ok:return False, reason
        before = _signature(basic)
        sent = time.monotonic()
        link.command(2, 22, {'MotionParam': 1})
    elif state == g.STATE_STAND:
        before = _signature(basic)
        sent = time.monotonic()
    else:return g.precheck(basic, motion)
    deadline = time.monotonic()+10
    while time.monotonic() < deadline:
        time.sleep(.05)
        basic, motion = link.record(1002, 6), link.record(1002, 4)
        status = g._basic(basic)
        if link.error:return False, '连接异常：'+link.error
        if g.feedback(basic)[1] > 2:return False, '起立时基础反馈过期'
        if _signature(basic) != before:return False, '起立时模式、步态或方向变化；保留当前设置'
        if status.get('HES') != 0 or status.get('Sleep') not in (0, False):return False, '起立期间急停或休眠状态变化'
        if status.get('MotionState') == g.STATE_RL and basic.get('monotonic_at', 0) > sent:
            return g.precheck(basic, motion)
        if status.get('MotionState') not in (g.STATE_SIT, g.STATE_STAND):return False, '起立被其他状态中断'
    return False, '未读回新鲜 MotionState=17，未发送问候轴指令'


def run_light(link, style, seconds, before=None):
    items = g.light_on_items(style)
    deadline = time.monotonic()+seconds
    next_send = 0.
    while time.monotonic() < deadline:
        if link.error:raise RuntimeError(link.error)
        if before is not None:ready(link, before)
        if time.monotonic() >= next_send:
            link.light_sent = True
            link.command(g.LIGHT_TYPE, g.LIGHT_CMD, items)
            next_send = time.monotonic()+1
        time.sleep(min(.05, max(0., deadline-time.monotonic())))
    link.command(g.LIGHT_TYPE, g.LIGHT_CMD, g.light_off_items())
    link.light_sent = False


def run_frames(link, frames, before):
    began = time.monotonic()
    for k, items in enumerate(list(frames)+g.zero_plan()):
        target = began+k/g.AXIS_HZ
        time.sleep(max(0., target-time.monotonic()))
        if time.monotonic()-target > .15:raise RuntimeError('发送节拍延迟，取消问候，不追发积压动作')
        ready(link, before)
        link.axes_sent = True  # Mark before I/O: a send failure does not prove no delivery.
        link.command(g.AXIS_TYPE, g.AXIS_CMD, items)
    last_zero = time.monotonic()
    deadline = last_zero+1
    while time.monotonic() < deadline:
        basic, motion = ready(link, before)
        if motion.get('monotonic_at', 0) > last_zero:
            link.axes_sent = False
            return
        time.sleep(.05)
    raise RuntimeError('已请求零轴，但没有更新的静止反馈，不能确认停止')


def cleanup(link):
    """Best effort for effects this invocation actually sent; never mode/gait."""
    failures = []
    if link.axes_sent:
        # One immediate stop request on cancellation; do not keep overwriting
        # an operator's new command stream for another 0.4 seconds.
        try:link.command(g.AXIS_TYPE, g.AXIS_CMD, g._axis_items())
        except (Exception, KeyboardInterrupt) as exc:failures.append('零轴请求失败：'+str(exc))
        link.axes_sent = False
    if link.light_sent:
        try:link.command(g.LIGHT_TYPE, g.LIGHT_CMD, g.light_off_items())
        except (Exception, KeyboardInterrupt) as exc:failures.append('默认灯效请求失败：'+str(exc))
        link.light_sent = False
    return failures


def main(argv=None):
    p = argparse.ArgumentParser(description='M20 Pro 灯语与显式实验问候；请先停止其他控制客户端')
    p.add_argument('--ip', default='10.21.31.103')
    p.add_argument('--port', type=int, default=30000)
    p.add_argument('--style', default='breathe', choices=sorted(g.LIGHT_STYLES))
    p.add_argument('--seconds', type=float, default=4.)
    p.add_argument('--nod', action='store_true', help='实验性 Pitch 序列')
    p.add_argument('--bob', action='store_true', help='实验性 Z 序列')
    p.add_argument('--stand', action='store_true', help='仅实验动作前，允许从趴下请求起立')
    p.add_argument('--status', action='store_true', help='只读状态，不发送灯光或运动命令')
    args = p.parse_args(argv)
    try:
        ipaddress.IPv4Address(args.ip)
        g._integer(args.port, 'port', 1, 65535)
        g._number(args.seconds, 'seconds', .05, 60.)
    except ValueError as exc:p.error(str(exc))
    link = None
    code = 0
    try:
        link = Link(args.ip, args.port)
        basic, motion = wait_state(link)
        if not basic:raise RuntimeError('未收到基础状态，请核对 IP 与网络')
        if args.status:
            print('BasicStatus:', basic)
            print('MotionStatus:', motion)
        else:
            before = None
            if args.nod or args.bob:
                ok, reason = ensure_stand(link, args)
                if not ok:raise RuntimeError(reason)
                before = _signature(ready(link)[0])
            run_light(link, args.style, args.seconds, before)
            if args.nod:run_frames(link, g.nod_plan(), before)
            if args.bob:run_frames(link, g.bob_plan(), before)
            print('问候序列已结束；灯效请求已恢复默认，动作实际效果以反馈为准')
    except KeyboardInterrupt:
        print('已中断；只清理本次已发送的灯效/轴请求，不切换模式或步态')
        code = 130
    except Exception as exc:
        print('问候未完成：'+str(exc), file=sys.stderr)
        code = 1
    finally:
        if link:
            failures = cleanup(link)
            for failure in failures:print(failure, file=sys.stderr)
            if failures and code == 0:code = 1
            try:link.close()
            except Exception as exc:
                print('关闭连接失败：'+str(exc), file=sys.stderr)
                if code == 0:code = 1
    return code


if __name__ == '__main__':
    sys.exit(main())

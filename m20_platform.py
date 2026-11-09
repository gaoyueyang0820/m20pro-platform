#!/usr/bin/env python3
"""M20 Pro operations platform, extending default.zip's Python controller."""
import argparse
from pathlib import Path
import base64
import copy
import hashlib
import hmac
import ipaddress
import json
import math
import mimetypes
import os
import re
import secrets
import struct
import threading
import time
import uuid
import zlib
from datetime import datetime
from http.cookies import SimpleCookie
from http.server import ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs,quote
from urllib.request import Request, urlopen

import m20.m20_console as original
from m20.navigation_validation import NavigationEvidenceError, validate_map_identity, error_summary
import m20.greeting as greeting
from m20.greeting_runtime import GreetingSession
from m20.platform_core import ROOT, JOINTS, LEGS, NEUTRAL, ERRORS, RobotLink, Store, Training, number, integer, rpc, explain_error, leg_target, age_of
from m20.platform_hub import Hub
from m20.shared_access import SharedAccess, lan_addresses, shared_listener
from m20.camera_streams import Cameras
from m20.autonomy_runtime import Autonomy
from m20.vision_models import ModelChecks, catalog as vision_catalog, default_path
from m20.ppe_runtime import SharedPPEDetector
from m20.map_learning_runtime import MapLearning
from m20.people_counting import PeopleCounter
from m20.patrol_actions import station_spec
from m20.point_navigation import PointNavigator, navigation_observation, stationary_feedback

if not os.environ.get('M20_PPE_MODEL'):
    original.PPE_MODEL_PATH=str(default_path('ppe'))


def exc_location(exc):
    """把异常压成一行可定位信息：最新一帧的文件:行号 函数 + 具体原因。
    诊断日志只写异常类型名时无法判断根因（例如哪条外部命令失败），故补齐这两项。"""
    import subprocess
    import traceback
    frames = traceback.extract_tb(exc.__traceback__)
    where = ''
    if frames:
        last = frames[-1]
        where = ' @ %s:%s %s' % (Path(last.filename).name, last.lineno, last.name)
    if isinstance(exc, subprocess.CalledProcessError):
        detail = '外部命令退出码 %s' % exc.returncode
    else:
        # Arbitrary messages may contain request tokens, passwords or argv.
        detail = type(exc).__name__
    detail = ' '.join(detail.split())
    if len(detail) > 220:
        detail = detail[:217] + '...'
    return where + (' | ' + detail if detail else '')


def import_pgm(encoded):
    try:
        raw = base64.b64decode(encoded, validate=True)
    except Exception:
        raise ValueError('PGM 文件编码无效')
    if len(raw) > 12_000_000:
        raise ValueError('地图文件不能超过 12 MB')
    tokens = re.finditer(rb'#[^\r\n]*|[^\s#]+', raw)
    header = []
    end = 0
    for match in tokens:
        if match.group().startswith(b'#'):
            continue
        header.append(match.group())
        end = match.end()
        if len(header) == 4:
            break
    if len(header) != 4 or header[0] not in [b'P5', b'P2']:
        raise ValueError('仅支持 PGM P5/P2 栅格地图')
    w, h, peak = [int(t) for t in header[1:]]
    if w < 1 or h < 1 or w*h > 4_000_000 or not 1 <= peak <= 255:
        raise ValueError('地图尺寸或灰度范围不支持（最多 400 万像素，8 位灰度）')
    if header[0] == b'P5':
        if end >= len(raw) or raw[end:end+1] not in b' \t\r\n':
            raise ValueError('PGM 缺少数据分隔符')
        offset = end + (2 if raw[end:end+2] == b'\r\n' else 1)
        pixels = raw[offset:]
    else:
        values = [int(x) for x in re.findall(rb'\d+', re.sub(rb'#[^\n]*', b'', raw[end:]))]
        if any(x < 0 or x > peak for x in values):
            raise ValueError('PGM 灰度值无效')
        pixels = bytes(values)
    if len(pixels) != w*h:
        raise ValueError('地图像素数量与宽高不匹配')
    if peak != 255:
        pixels = bytes(round(v / peak * 255) for v in pixels)
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind+data) & 0xffffffff)
    scan = b''.join(b'\x00' + pixels[y*w:(y+1)*w] for y in range(h))
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
    return w, h, png + chunk(b'IDAT', zlib.compress(scan)) + chunk(b'IEND', b'')


class Platform:
    def __init__(self, store=None):
        self.lock = threading.RLock()
        self.instance_id=uuid.uuid4().hex
        self._greet_busy=False
        self.greeting=GreetingSession(self)
        self._greet_demo=None  # (结束时刻, 'nod'/'bob', 开始时刻)：演示模式的合成问候姿态
        from m20.pose_monitor import PoseMonitor
        self.pose_monitor=PoseMonitor()
        self.store = store or Store()
        self.link = None
        self.demo = True
        self.config = {k: os.environ.get('M20_'+k.upper()+'_URL', '') for k in ['sdk', 'train', 'nos','gos']}
        self.config.update(self.store.value.get('settings',{}))
        self.bridge_token = os.environ.get('M20_BRIDGE_TOKEN')
        self.training = Training(demo=True)
        self.sdk = dict(connected=False, armed=False, backend='unavailable', joints=[])
        self.sdk_seen = 0
        self.lease_at = 0
        self.session = None
        self.events = []
        self.previous_faults = set()
        self.responses = []
        self.audit = []
        self.nav = dict(status='idle', index=0, route=None, progress=0)
        self.next_value = self.store.value.get('nav_value_highwater',0)
        self.positions = list(NEUTRAL)
        self.target = list(NEUTRAL)
        self.wheels = [0.]*4
        self.velocity = [0., 0., 0.]
        self.velocity_at = 0
        self.velocity_deadline = None
        self.pose = dict(PosX=-6., PosY=-3., PosZ=0., Yaw=.0, Location=0)
        self.basic = dict(MotionState=17, Gait=0x1001, Charge=0, HES=0, ControlUsageMode=0, Sleep=0, OOA=0, Version='PRO')
        self.lights = dict(Front=0, Back=0)
        self.reverse = False
        self.estopped = False
        self.leg_anchor = None
        self.faults = []
        self.raw = {}
        self.history = []
        self.mapping = False
        self.cancel_request = None
        self.done = False
        self.cameras=Cameras(original.cv2,lambda:self.link.ip if self.link and not self.demo else None,original.RTSP_PORT)
        self.add_log('system', '平台已启动，当前为演示模式。运动、遥测和训练曲线为模拟数据。')
        self.hub=Hub(self)
        self.autonomy=Autonomy(self)
        self.point_navigation=PointNavigator(self)
        self.model_checks=ModelChecks(self.store.path.parent)
        self.map_learning=MapLearning(self)
        self.goal_plans={}
        self._plan_generation=0;self._nav_start_generation=0
        from m20.navigation_intents import NavigationIntents
        self.navigation_intents=NavigationIntents()
        from m20.resident_navigation_client import ResidentNavigationClient
        self.resident_navigation=ResidentNavigationClient(self)
        self.people=PeopleCounter(self)
        from m20.localization_runtime import AutomaticLocalizer
        self.localizer=AutomaticLocalizer(self)

    def add_log(self, kind, text):
        self.audit.insert(0, dict(id=uuid.uuid4().hex[:10], t=time.strftime('%H:%M:%S'), at=time.time(), kind=kind, text=text))
        del self.audit[300:]

    def proxy(self, key, path, req=None, timeout=4):
        return self.hub.proxy(key,path,req,timeout)

    def connect(self, req):
        with self.lock:
            self.cancel_greeting('连接正在切换，问候已取消')
            self.autonomy.reset_context()
            self._plan_generation+=1;self.goal_plans.clear()
            self.map_learning.reset()
            self.autonomy.vision.stop()
            self.people.stop()
            self.people.control(dict(action='reset'))
            self.cameras.close()
            preserve_resident=self.resident_navigation.preserve()
            stopped=dict(warnings=[]) if preserve_resident else self.stop()
            connection_warnings=list(stopped.get('warnings',[]))
            if self.link and self.nav['status']=='cancelling':
                notice=self._close_navigation_connection()
                if notice['state']=='unconfirmed':connection_warnings.append(notice['message'])
                if notice.get('persistence_error'):connection_warnings.append(notice['persistence_error'])
            if original.PPE and original.PPE.enabled:
                original.PPE.stop()
            if self.link:
                if preserve_resident:self.link.close(stop=False)
                else:self.link.close()
                self.link = None
            original.LINK = None
            self.demo = req.get('mode', 'demo') == 'demo'
            if req.get('mode', 'demo') not in ['demo', 'live']:
                raise ValueError('连接模式无效')
            self.nav = dict(status='idle', index=0, route=None, progress=0)
            self.sdk = dict(connected=False, armed=False, backend='unavailable', joints=[])
            self.sdk_seen = 0
            self.previous_faults = set()
            self.responses = []
            self.hub.sdk_mode=dict(state='normal' if self.demo else 'unknown',source='demo' if self.demo else 'unavailable')
            if not self.demo:
                ip = str(ipaddress.IPv4Address(req.get('ip', '10.21.31.103')))
                port = integer(req.get('port', 30000), 1, 65535)
                self.link = RobotLink(ip, port)
                original.LINK = self.link
                original.PPE = SharedPPEDetector(self)
                self.add_log('system', '开始连接 %s:%s；收到有效状态前禁止运动' % (ip, port))
            else:
                self.basic['MotionState'] = 17
                self.basic['HES'] = 0
                self.estopped = False
                self.add_log('system', '已切换到演示模式')
            return dict(ok=True,warnings=connection_warnings,previous_connection_stop=copy.deepcopy(self.store.value.get('previous_connection_stop')))

    def live_ready(self):
        if self.demo:
            return
        rec = self.link.record(1002, 6) if self.link else {}
        if age_of(rec)>2:
            raise ValueError('没有新鲜的机器人基础状态，请先检查连接')
        b = rec.get('items', {}).get('BasicStatus', {})
        if b.get('HES') != 0:
            raise ValueError('硬急停状态未知或已触发，请检查实体急停')

    def invalidate_navigation_start(self):
        self._nav_start_generation+=1
        if self.nav.get('status')=='running' and self.nav.get('phase') in ['validating','preparing']:
            self.nav.update(status='cancelled',phase='stopped',execution_state='cancelled',message='导航准备已由手动操作取消')
            self.nav.pop('preparation',None)

    def cancel_navigation_intent(self,req):
        """Revoke even a delayed start; stop only the task carrying this ID."""
        with self.lock:
            token=self.navigation_intents.revoke(req,self.instance_id,self._plan_generation)
            matched=self.navigation_intents.owns(self.nav,token)
            if matched:
                if self.nav.get('engine')=='platform':
                    if self.nav.get('status')=='paused':self.nav.update(status='cancelled',phase='cancelled',execution_state='cancelled')
                    else:self.point_navigation.stop('cancelled','本次客户端导航意图已撤销，已请求停车')
                elif self.nav.get('status')=='running' and self.nav.get('phase') in ['validating','preparing']:
                    self.nav.update(status='cancelled',phase='stopped',execution_state='cancelled',message='本次导航准备已撤销')
                    self.nav.pop('preparation',None)
                    self.velocity=[0.,0.,0.];self.velocity_deadline=None
                    if self.link:self.link.set_velocity(0,0,0)
                elif self.nav.get('status')=='running':self._cancel_navigation('client_intent_revoked','cancelled')
                elif self.nav.get('status')=='cancelling':self.nav['after_cancel']='cancelled'
                elif self.nav.get('status')=='paused':self.nav.update(status='cancelled',phase='cancelled',execution_state='cancelled')
            return dict(ok=True,revoked=True,client_intent=token[2],matched=matched,
                        status=self.nav.get('status') if matched else 'revoked')

    def stop(self, estop=False):
        self.cancel_greeting()
        gos_owns_axes=self.resident_navigation.gos_busy()
        if self.resident_navigation.busy():self.resident_navigation.cancel()
        self.invalidate_navigation_start()
        self.autonomy.pause()
        if self.nav.get('engine')=='platform':self.point_navigation.stop()
        had_joint_session = bool(self.session or self.sdk.get('armed'))
        sdk_active = had_joint_session or self.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit'] or self.sdk.get('vendor_running')
        self.velocity = [0., 0., 0.]
        self.wheels = [0.]*4
        self.target = list(self.positions)
        self.session = None
        self.sdk['armed'] = False
        failures = []
        if sdk_active and not self.demo and (self.config.get('sdk') or self.hub.connections.value['targets']['sdk']['enabled']):
            try:
                self.proxy('sdk', '/stop', {})
            except ValueError as exc:
                failures.append(str(exc))
        if self.link:
            if not gos_owns_axes:self.link.set_velocity(0, 0, 0)
            if self.nav.get('engine')=='platform':pass
            elif self.nav['status']=='running' and self.nav.get('phase') in ['validating','preparing']:
                self.nav.update(status='cancelled',phase='stopped',execution_state='cancelled')
                self.nav.pop('preparation',None)
            elif self.nav['status'] == 'running':
                self._cancel_navigation('operator_stop','cancelled')
            elif self.nav['status']=='cancelling':
                # Reuse the outstanding receipt; repeated Stop must not keep
                # changing the sequence we wait for or restore a paused task.
                self.nav['after_cancel']='cancelled'
                if self.cancel_request is None:self._cancel_navigation('operator_stop','cancelled',retry=True)
            else:
                external=self._reported_active_navigation()
                if external:
                    self.nav=dict(status='idle',index=0,route=None,progress=0,value=external['items'].get('Value'),external_task=True)
                    self._cancel_navigation('operator_stop_external','cancelled')
            if estop:
                self.link.inhibit = True
                self.link.set_motion('estop')
        elif self.nav['status'] == 'running':
            self.nav.update(status='cancelled',phase='stopped',execution_state='cancelled')
        if estop:
            self.estopped = True
            if self.demo:
                self.basic['MotionState'] = 2
        unconfirmed=bool(self.link and self.nav['status']=='cancelling' and self.cancel_request is None)
        if unconfirmed and self.nav.get('cancel_error'):failures.append(self.nav['cancel_error'])
        return dict(ok=True, delivery='unconfirmed' if unconfirmed else 'sent' if self.link else 'simulated',
                    warnings=failures, message='停止请求尚未确认，请核对状态反馈' if unconfirmed else '已发停止请求；请核对状态反馈' if self.link else '演示动作已停止')

    def command(self, action):
        if self.resident_navigation.busy() and action not in ['stop','estop','heartbeat']:
            self.resident_navigation.cancel()
            raise ValueError('驻机任务已请求停车，请等待停止确认后手动操作')
        if action=='greet' or action.startswith('greet_'):
            return self.greet(**self._greet_args(action))
        if action!='heartbeat':self.cancel_greeting('已切换为手动操作，问候取消')
        if not action.startswith("light_") and action!='heartbeat':
            self.invalidate_navigation_start();self.autonomy.pause("已切换为手动操作")
        if action in ['stop', 'estop']:
            result = self.stop(action == 'estop')
        else:
            if self.hub.busy:raise ValueError('正在执行 '+self.hub.busy+'，请等待完成或使用停止按钮')
            if not self.demo and self.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit'] and not action.startswith(('light_','dir_')):
                raise ValueError('当前处于 SDK 流程，请先在 SDK 模式页面完成退出')
            self.live_ready()
            if self.sdk.get('armed'):
                raise ValueError('请先结束关节控制，再使用日常控制')
            if action in ['stand', 'lie', 'damping', 'idle', 'standard']:
                if self.nav['status'] in ['running', 'cancelling']:
                    raise ValueError('请先取消导航任务并确认停止')
                if self.link:
                    self.link.set_velocity(0, 0, 0)
                    self.link.set_motion(action)
                    if action in ['stand', 'standard']:
                        self.link.inhibit = False
                else:
                    self.basic['MotionState'] = dict(stand=17, lie=4, damping=2, idle=0, standard=17)[action]
                if action in ['stand', 'standard']:
                    self.estopped = False
            elif action.startswith('gait_'):
                gaits = dict(basic=0x1001, platform=0x1002, flat=0x3002, stair=0x3003, standard_stair=0x1003)
                key = action[5:]
                if key not in gaits:
                    raise ValueError('步态未收录')
                if self.basic.get('MotionState') != 17 or self.nav['status'] in ['running', 'cancelling'] or any(self.velocity):
                    raise ValueError('需处于 RL 控制、无导航任务且静止后切换步态')
                if self.link:
                    return self.link.command(2, 23, {'GaitParam': gaits[key]}, action)
                self.basic['Gait'] = gaits[key]
            elif action.startswith('mode_'):
                modes = dict(normal=0, nav=1, aux=2)
                key = action[5:]
                if key not in modes:
                    raise ValueError('未知模式')
                if self.nav['status'] in ['running', 'cancelling']:
                    raise ValueError('请先取消导航任务')
                self.velocity = [0., 0., 0.]
                if self.link:
                    self.link.set_velocity(0, 0, 0)
                    self.link.set_mode(modes[key])
                else:
                    self.basic.update(ControlUsageMode=modes[key],Gait=0x1001)
            elif action.startswith('dir_'):
                if action not in ['dir_forward', 'dir_backward']:
                    raise ValueError('无效方向')
                if any(self.velocity):
                    raise ValueError('请先停止运动')
                self.reverse = action == 'dir_backward'
                if self.link:
                    self.link.set_direction(not self.reverse)
            elif action == 'heartbeat':
                if self.link:
                    self.link.heartbeat()
            elif action.startswith('light_'):
                table = dict(light_front_on=(1,None), light_front_off=(0,None),
                             light_back_on=(None,1), light_back_off=(None,0), light_all_off=(0,0))
                if action not in table:
                    raise ValueError('未知灯光操作')
                f, b = table[action]
                if self.link:
                    self.link.set_flashlight(f, b)
                else:
                    if f is not None: self.lights['Front'] = f
                    if b is not None: self.lights['Back'] = b
            else:
                raise ValueError('未知指令')
            result = dict(ok=True, delivery='sent' if self.link else 'simulated',
                          message='已发送，实际执行以状态回报为准' if self.link else '演示操作完成')
        self.add_log('command', action)
        return result

    @staticmethod
    def _greet_args(action):
        """greet / greet_<style> / greet_nod / greet_bob → greet() 关键字参数。"""
        suffix = action[6:]
        if suffix in ('nod', 'bob'):
            return dict(style='breathe', motion=suffix)
        if suffix and suffix in greeting.LIGHT_STYLES:
            return dict(style=suffix)
        if suffix:
            raise ValueError('未知打招呼动作：' + action)
        return dict(style='breathe')

    def greet(self, style='breathe', motion=None):
        return self.greeting.start(style,motion)

    def cancel_greeting(self, reason='已由操作者停止问候'):
        self.greeting.cancel(reason)

    def set_velocity(self, req, autonomous=False, valid_until=None):
        if self.resident_navigation.gos_busy() and not any(req.get(k,0) for k in ('forward','side','yaw')):
            return dict(ok=True)
        if not autonomous and self.resident_navigation.busy() and any(req.get(k,0) for k in ('forward','side','yaw')):
            self.resident_navigation.cancel()
            raise ValueError('已请求驻机任务停车，停止确认前暂不可移动')
        if not autonomous:
            self.cancel_greeting("已由手动驾驶接管，问候取消")
            self.invalidate_navigation_start();self.autonomy.pause("已由手动驾驶接管")
            if self.point_navigation.active:self.point_navigation.stop('paused','已由手动驾驶接管')
        v = [number(req.get(k, 0), -1, 1) for k in ['forward', 'side', 'yaw']]
        if valid_until is None and 'hold_seconds' in req:
            valid_until=time.monotonic()+number(req['hold_seconds'],0,.5)
        if any(v):
            if autonomous and self._greet_busy:raise ValueError('问候动作持有控制权，请先停止问候')
            if autonomous and not self.demo:
                direction=self.link.record(1002,6) if self.link else {}
                if (self.reverse or age_of(direction)>2 or
                        direction.get('items',{}).get('BasicStatus',{}).get('Direction')!=0):
                    self.velocity=[0.,0.,0.];self.velocity_deadline=None;self.velocity_at=time.time()
                    if self.link:self.link.set_velocity(0,0,0)
                    raise ValueError('自主行走需要新鲜的头朝前反馈；请用手柄切回头朝前，并关闭平台尾向前后重试')
            if self.hub.busy:raise ValueError('正在执行 '+self.hub.busy+'，暂不可移动')
            if not self.demo and self.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit']:
                raise ValueError('请先退出 SDK 模式再使用日常遥控')
            self.live_ready()
            if self.estopped or self.basic.get('Sleep') or self.basic.get('MotionState') != 17:
                raise ValueError('机器人未就绪，请先唤醒、解除急停并站立')
            platform_owned=autonomous and self.point_navigation.active and self.nav.get('engine')=='platform' and self.nav['status']=='running'
            if self.sdk.get('armed') or (self.nav['status'] in ['running', 'cancelling'] and not platform_owned) or self.basic.get('ControlUsageMode') not in [0, 2]:
                raise ValueError('当前控制权不属于遥控器，请结束关节/导航任务并切换常规或辅助模式')
        self.velocity = v
        self.velocity_deadline=valid_until if any(v) else None
        self.velocity_at = time.time()
        if self.link:
            if valid_until is not None:
                self.link.set_velocity(*v,valid_until=valid_until)
            else:self.link.set_velocity(*v)
        return dict(ok=True)

    def joint_control(self, operation, req):
        if operation=='preview':
            if not self.demo and not self.sdk.get('fresh'):raise ValueError('需要新鲜的 SDK 关节反馈才能预览')
            return dict(ok=True,positions=leg_target(self.positions,req.get('leg'),req.get('height')))
        if operation == 'arm':
            if self._greet_busy:raise ValueError('请先停止问候动作，再开启关节控制')
            if self.autonomy.active:raise ValueError('请先停止自主任务')
            if self.hub.busy:raise ValueError('后台操作正在进行，请等待完成')
            if self.sdk.get('vendor_running') or (self.demo and self.hub.demo_policy.get('vendor_running')):
                raise ValueError('请先停止 SDK 策略程序，再开启独立关节调试')
            if not self.demo and (self.hub.sdk_mode.get('state')!='sdk' or self.hub.sdk_mode.get('source') not in ['operator_confirmed','vendor']):
                raise ValueError('请先在 SDK 模式页面完成模式确认和通道检查')
            if self.estopped or self.nav['status'] in ['running', 'cancelling'] or any(self.velocity):
                raise ValueError('请先停止运动、结束导航并解除软急停')
            if not req.get('rig_confirmed'):
                raise ValueError('请确认吊架/仿真环境与底层 SDK 独占控制')
            self.session = secrets.token_urlsafe(20)
            if not self.demo:
                result = self.proxy('sdk', '/arm', {'session': self.session, 'rig_confirmed': True, 'mode_confirmed':True},timeout=10)
                if result.get('backend') != 'hardware' or not result.get('calibration_verified'):
                    self.session = None
                    raise ValueError('SDK 代理必须处于经校准的 hardware 模式')
                self.sdk.update(result)
            self.sdk.update(armed=True)
            self.lease_at = time.time()
            self.add_log('command', '已开启关节控制会话（独占、反馈校验、失联转阻尼）')
            return dict(ok=True, session=self.session)
        if operation == 'stop':
            return self.stop()
        if not self.session or req.get('session') != self.session or not self.sdk.get('armed'):
            raise ValueError('关节会话未启用或已过期，请重新开启')
        self.lease_at = time.time()
        if operation == 'lease':
            if not self.demo:
                return self.proxy('sdk', '/lease', {'session': self.session})
            return dict(ok=True)
        positions = list(self.positions)
        for j in JOINTS:
            if j['part'] != 'Wheel':
                number(positions[j['index']], j['minimum'], j['maximum'])
        duration = number(req.get('duration', 2), 1, 8)
        wheels = [0.]*4
        next_anchor = self.leg_anchor
        if operation == 'set':
            index = integer(req.get('index'), 0, 15)
            j = JOINTS[index]
            value = number(req.get('value'), -2 if j['part'] == 'Wheel' else j['minimum'],
                           2 if j['part'] == 'Wheel' else j['maximum'])
            if j['part'] == 'Wheel':
                wheels[index//4] = value
            else:
                if abs(value - positions[index]) > .5:
                    raise ValueError('单次角度变化不得超过 0.5 rad，请分步调整')
                positions[index] = value
        elif operation == 'leg':
            if self.leg_anchor:
                raise ValueError('已有抬腿动作，请先落腿')
            positions = leg_target(positions, req.get('leg'), number(req.get('height', .04), .01, .08))
            next_anchor = list(self.positions)
        elif operation == 'lower':
            if not self.leg_anchor:
                raise ValueError('没有可恢复的抬腿起点')
            positions = list(self.leg_anchor)
            next_anchor = None
        elif operation == 'pose':
            values = req.get('positions')
            if not isinstance(values, list) or len(values) != 16:
                raise ValueError('姿态需包含 16 个关节值')
            for i, value in enumerate(values):
                if i % 4 != 3:
                    value = number(value, JOINTS[i]['minimum'], JOINTS[i]['maximum'])
                    if abs(value - positions[i]) > .5:
                        raise ValueError('姿态单次关节变化不得超过 0.5 rad')
                    positions[i] = value
        else:
            raise ValueError('未知关节操作')
        if not self.demo:
            self.proxy('sdk', '/trajectory', dict(session=self.session, positions=positions, wheels=wheels, duration=duration))
        self.leg_anchor = next_anchor
        self.target = positions
        self.wheels = wheels
        self.add_log('command', '关节动作 %s，时长 %.1fs' % (operation, duration))
        self.trajectory = dict(start=time.time(), duration=duration, origin=list(self.positions), target=positions)
        return dict(ok=True, positions=positions, delivery='simulated' if self.demo else 'accepted_by_bridge')

    def route_save(self, req, persist=True):
        name = str(req.get('name', '')).strip()[:80]
        points = req.get('points', [])
        if not name or not isinstance(points, list) or not 1 <= len(points) <= 100:
            raise ValueError('请填写路线名称并添加 1–100 个航点')
        map_ = next((m for m in self.maps() if m['id'] == req.get('map_id')), None)
        if not map_:
            raise ValueError('请选择地图')
        clean = []
        for p in points:
            clean.append(dict(x=number(p.get('x')), y=number(p.get('y')),
                              yaw=number(p.get('yaw', 0), -math.pi, math.pi),
                              type=integer(p.get('type', 1), 0, 3)))
            if clean[-1]['type'] not in [0, 1, 3]:
                raise ValueError('航点类型无效')
            action=station_spec(p.get('station_action'))
            if action:
                if clean[-1]['type']==3:raise ValueError('充电航点不可同时执行人数统计')
                clean[-1]['station_action']=action
            x0,y0,_ = map_['origin']
            if not (x0 <= clean[-1]['x'] <= x0+map_['width']*map_['resolution'] and
                    y0 <= clean[-1]['y'] <= y0+map_['height']*map_['resolution']):
                raise ValueError('航点必须位于当前地图范围内')
        value = dict(id=req.get('id') or uuid.uuid4().hex[:10], name=name, points=clean,
                     map_id=map_['id'], updated=time.time())
        if 'options' in req:
            options=req['options']
            if not isinstance(options,dict) or not isinstance(options.get('pause_at_point',False),bool):raise ValueError('路线参数无效')
            gait=options.get('gait','flat')
            if gait not in ['flat','stair']:raise ValueError('路线步态无效')
            value['options']=dict(nav_mode=integer(options.get('nav_mode',1),0,1),speed=integer(options.get('speed',1),0,2),
                                  gait=gait,manner=integer(options.get('manner',0),0,1),pause_at_point=options.get('pause_at_point',False),disconnect_policy=options.get('disconnect_policy','stop'))
            from m20.navigation_disconnect import POLICIES
            if value['options']['disconnect_policy'] not in POLICIES:raise ValueError('断联策略无效')
            if 'engine' in options:
                if options['engine'] not in ['platform','native','gos']:raise ValueError('未知寻路方式')
                value['options']['engine']=options['engine']
        if persist:
            self.store.value['routes'] = [r for r in self.store.value['routes'] if r['id'] != value['id']] + [value]
            self.store.save()
        return dict(ok=True, route=value)

    def maps(self):
        include_seed=self.demo
        seed = dict(id='demo', name='演示园区 · 一层', resolution=.1, origin=[-16,-11,0],
                    width=320, height=220, image='/assets/demo-map.svg', demo=True)
        if self.demo and hasattr(self,'hub'):
            source=self.hub.demo_maps.maps/'demo-yard/occ_grid.pgm'
            include_seed=source.is_file()
            if source.is_file():
                target=self.store.path.parent/'maps/demo.png';target.parent.mkdir(parents=True,exist_ok=True)
                if not target.exists() or target.stat().st_mtime<source.stat().st_mtime:
                    target.write_bytes(import_pgm(base64.b64encode(source.read_bytes()).decode())[2])
                seed['image']='/maps/demo.png'
        return ([seed] if include_seed else []) + [m for m in self.store.value['maps'] if not m.get('archived') and (self.demo or not m.get('demo'))]

    def import_map(self, req, existing_id=None):
        w, h, png = import_pgm(req.get('pgm', ''))
        resolution = number(req.get('resolution', .1), .001, 5)
        origin = req.get('origin', [0,0,0])
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError('地图原点需要 [x,y,yaw]')
        origin = [number(x) for x in origin]
        if abs(origin[2]) > .00001:
            raise ValueError('当前栅格编辑器仅支持 origin yaw=0 的地图')
        existing=next((m for m in self.store.value['maps'] if m['id']==existing_id),None)
        same_geometry=existing and all(existing.get(k)==v for k,v in dict(width=w,height=h,resolution=resolution,origin=origin).items())
        map_id=existing['id'] if same_geometry else uuid.uuid4().hex[:10]
        folder = self.store.path.parent / 'maps'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / (map_id+'.png')).write_bytes(png)
        (folder / (map_id+'.pgm')).write_bytes(base64.b64decode(req['pgm'],validate=True))
        result = dict(id=map_id, name=str(req.get('name', '导入地图'))[:80], resolution=resolution,
                      origin=origin, width=w, height=h, image='/maps/'+map_id+'.png',
                      robot_map_name=req.get('robot_map_name', ''), verified_for_robot=False,
                      negate=req.get('negate',0),free_thresh=req.get('free_thresh',.196),occupied_thresh=req.get('occupied_thresh',.65))
        self.store.value['maps']=[result if m['id']==map_id else m for m in self.store.value['maps']]
        if not same_geometry:self.store.value['maps'].append(result)
        self.store.save()
        return dict(ok=True, map=result)

    def navigation_surface(self,map_id,scan=None):
        """Pair an observed floor frame with actual independent control feedback."""
        link=self.link
        if self.demo or not link:raise ValueError('实时通行层需要实机连接')
        scan=scan if scan is not None else self.autonomy.scan_now(False)
        result=navigation_observation(link,scan,'%s:%s:%s'%(map_id,self._plan_generation,id(link)))
        if self.link is not link:raise ValueError('连接已改变，丢弃旧扫描')
        return result

    def plan_goal(self, req):
        from m20.navigation_planner import GridPlanner,live_obstacles
        engine=req.get('engine','native')
        if engine not in ['native','platform','gos']:raise ValueError('未知导航执行器')
        x,y=number(req.get('x')),number(req.get('y'));yaw=None if req.get('yaw') is None else number(req['yaw'],-math.pi,math.pi)
        with self.lock:
            generation=self._plan_generation;planning_link=self.link;planning_demo=self.demo
            map_=copy.deepcopy(next((m for m in self.maps() if m['id']==req.get('map_id')),None))
            if not map_:raise ValueError('请选择地图')
            self.live_ready()
            if not self.demo and (self.pose.get('Location')!=0 or not self.link or age_of(self.link.record(1007,2))>2):raise ValueError('请先完成地图定位，并等待新鲜定位反馈')
            start=[number(self.pose.get(k)) for k in ['PosX','PosY','Yaw']]
            planning_context=self.hub.context()
        def verify_map():
            if planning_demo:return None
            library=self.hub.remote_maps('/maps/list',context=planning_context,timeout=3)
            with self.lock:
                if generation!=self._plan_generation or self.link is not planning_link or self.demo!=planning_demo:
                    raise ValueError('连接已改变，请重新规划')
            return validate_map_identity(map_,library)
        map_identity=verify_map()
        source=self.store.path.parent/'maps'/(map_['id']+'.pgm')
        if map_.get('demo') and map_['id']=='demo':source=self.hub.demo_maps.maps/'demo-yard/occ_grid.pgm'
        if not source.is_file():
            if not map_.get('robot_map_name'):raise ValueError('缺少原始栅格，请重新导入地图')
            updated=self.hub.sync_map(dict(name=map_['robot_map_name'],grid_id=map_.get('grid_id',0)))['map']
            if updated['id']!=map_['id']:raise ValueError('地图已更新，请重新选择目标')
            map_=copy.deepcopy(updated)
        raw=source.read_bytes();fingerprint=hashlib.sha256(raw).hexdigest()
        if engine=='gos':
            if planning_demo:raise ValueError('GOS 寻路需要已连接的实机服务')
            from bridge.gos_navigation import pack_grid
            remote=self.proxy('gos','/missions/preview',dict(grid=pack_grid(raw,map_),start=start,goal=[x,y],yaw=yaw),timeout=6)
            result=dict(remote['plan'],map_identity=verify_map(),execution='gos_trajectory',
                        guide_points_preview_only=True,obstacle_note='GOS 开始行走后使用机内雷达实时避障')
            return self._save_goal_plan(map_,source,fingerprint,start,result,generation,planning_link,planning_demo,engine)
        obstacles=[];obstacle_note='实时避障由机器人执行';body_pose=None
        if not self.demo and engine=='native':
            try:
                began=time.monotonic();packet=self.proxy('nos','/sensors/observation',timeout=1)
                if packet.get('map_name')!=map_.get('robot_map_name'):raise ValueError('实时点云属于另一张活动地图')
                obstacles=live_obstacles(packet,start,time.monotonic()-began);obstacle_note='已结合当前雷达障碍，行走中继续实时避障'
                motion=self.link.record(1002,4) if self.link else {};values=motion.get('items',{}).get('MotionStatus',{})
                stationary=age_of(motion)<.5 and all(abs(values.get(k,999))<limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)])
                pp=packet.get('pose',[])
                if stationary and len(pp)==7 and math.dist(pp[:2],start[:2])<.05:
                    qx,qy,qz,qw=pp[3:];pyaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                    if abs(math.atan2(math.sin(pyaw-start[2]),math.cos(pyaw-start[2])))<.1:body_pose=start
            except (ValueError,KeyError,TypeError):obstacle_note='当前无可用的实时障碍快照；按地图规划，行走中由机器人实时避障'
        planner=GridPlanner(raw,map_['resolution'],map_['origin'],map_.get('negate',0),map_.get('free_thresh',.196),map_.get('occupied_thresh',.65),obstacles,robot_pose=body_pose,enable_live_surface=not self.demo and engine=='platform',soft_static_speckles=engine=='platform')
        latest_surface=None
        if not self.demo and engine=='platform':
            deadline=time.monotonic()+2.5;seen=set();last_error=None
            for _ in range(8):
                if time.monotonic()>=deadline:break
                with self.lock:
                    if generation!=self._plan_generation or self.link is not planning_link:raise ValueError('连接已改变，请重新规划')
                try:
                    scan,loc=self.navigation_surface(map_['id']);report=planner.update_live_surface(scan,loc)
                    if report.get('accepted'):
                        seen.add(scan.get('stamp'));latest_surface=(scan,loc)
                        if report.get('body_projection_cells',0) and planner.pose_fits(*scan['pose']):break
                    if len(seen)>=3 and report.get('state')!='waiting_evidence':break
                except ValueError as exc:last_error=str(exc)
                time.sleep(.04)
            surface=planner.live_surface_status()
            obstacle_note=('已结合实时通行层：临时更新 %s 个旧占据格；原图不改动'%surface.get('occupied_overrides',0)) if surface.get('occupied_overrides') else (last_error or surface.get('reason','按旧图规划，行走中检查现场通路'))
        if latest_surface:start=list(latest_surface[0]['pose'])
        try:result=planner.plan(start,[x,y],yaw,goal_tolerance=.65)
        except ValueError as exc:
            if not planning_demo:verify_map()
            if engine!='platform' or planning_demo:raise
            if not latest_surface:
                raise NavigationEvidenceError('没有足够的新鲜同步观测核对起点：'+(last_error or obstacle_note),
                    classification='observation_unavailable',reason_code='observation_unavailable',
                    recommended_action='revalidate_observation') from exc
            from m20.start_recovery_runtime import plan_with_recovery
            scan,loc=latest_surface
            if '_valid_until' in scan:scan['age']=max(0.,.65+time.monotonic()-scan['_valid_until'])
            record=planning_link.record(1002,6);basic=record.get('items',{}).get('BasicStatus',{})
            standing=age_of(record)<2 and basic.get('MotionState')==17 and basic.get('HES')==0 and not basic.get('Sleep')
            try:result=plan_with_recovery(planner,start,[x,y],yaw,scan,loc,standing,standard_error=exc)
            except ValueError as recovery_error:
                recovery=getattr(recovery_error,'recovery',{})
                if recovery.get('reason_code') not in ['live_evidence_expired','insufficient_ground_votes']:raise
                # One bounded read-only recheck, never an indefinite retry and
                # never a permission to move using old projected evidence.
                verify_map();until=time.monotonic()+1.2;newer=scan.get('stamp');refreshed=False
                for _ in range(4):
                    if time.monotonic()>=until:break
                    with self.lock:
                        if generation!=self._plan_generation or self.link is not planning_link:raise ValueError('连接已改变，请重新规划')
                    next_scan,next_loc=self.navigation_surface(map_['id'])
                    if newer is not None and next_scan.get('stamp',-1)<=newer:continue
                    update=planner.update_live_surface(next_scan,next_loc)
                    if not update.get('accepted'):continue
                    scan,loc=next_scan,next_loc;start=list(scan['pose']);newer=scan.get('stamp');refreshed=True
                if not refreshed:raise recovery_error
                record=planning_link.record(1002,6);basic=record.get('items',{}).get('BasicStatus',{})
                standing=age_of(record)<2 and basic.get('MotionState')==17 and basic.get('HES')==0 and not basic.get('Sleep')
                result=plan_with_recovery(planner,start,[x,y],yaw,scan,loc,standing)
                result['observation_recheck']=dict(performed=True,limit=1,motion_sent=False)
        if not planning_demo:map_identity=verify_map()
        result['map_identity']=map_identity
        if result.get('self_projection_ignored'):obstacle_note='已排除机身内部的新增投影；原图墙体和前方障碍保留，机器人实时避障保持开启'
        result.update(live_obstacles=len(obstacles),obstacle_note=obstacle_note,
                      execution='platform_trajectory' if engine=='platform' else 'native_single_goal',guide_points_preview_only=engine!='platform',live_surface=planner.live_surface_status())
        return self._save_goal_plan(map_,source,fingerprint,start,result,generation,planning_link,planning_demo,engine)

    def _save_goal_plan(self,map_,source,fingerprint,start,result,generation,planning_link,planning_demo,engine):
        key=uuid.uuid4().hex
        with self.lock:
            if generation!=self._plan_generation or self.link is not planning_link or self.demo!=planning_demo:
                raise ValueError('连接上下文已改变，请重新规划')
            self.goal_plans={k:v for k,v in self.goal_plans.items() if time.monotonic()-v['at']<60}
            self.goal_plans[key]=dict(at=time.monotonic(),map_id=map_['id'],source=str(source),fingerprint=fingerprint,start=start,result=result,
                                      connection_generation=generation,link=planning_link,demo=planning_demo,execution_engine=engine)
        return dict(result,plan_id=key,map_id=map_['id'],demo=self.demo)

    def start_goal(self, req):
        with self.lock:
            intent=self._nav_start_generation
            client_intent=self.navigation_intents.register(req,self.instance_id,self._plan_generation)
        if not req.get('plan_id'):req=dict(req,plan_id=self.plan_goal(req)['plan_id'])
        return self._start_planned_goal(req,intent,client_intent)

    def _start_planned_goal(self, req,intent=None,client_intent=None,route_override=None,start_index=0):
        with self.lock:
            self.navigation_intents.check(client_intent,self.instance_id,self._plan_generation)
            intent=self._nav_start_generation if intent is None else intent
            if intent!=self._nav_start_generation:raise ValueError('导航启动已取消')
            plan=self.goal_plans.get(req['plan_id'])
            if not plan or time.monotonic()-plan['at']>60 or plan['map_id']!=req.get('map_id'):raise ValueError('规划已过期或地图已改变，请重新规划')
            if req.get('engine','native')!=plan.get('execution_engine','native'):raise ValueError('寻路方式已改变，请重新规划')
            if plan.get('connection_generation')!=self._plan_generation or plan.get('link') is not self.link or plan.get('demo')!=self.demo:
                raise ValueError('规划所属连接已改变，请重新规划')
            if hashlib.sha256(Path(plan['source']).read_bytes()).hexdigest()!=plan['fingerprint']:raise ValueError('地图内容已更新，请重新规划')
            if math.hypot(self.pose['PosX']-plan['start'][0],self.pose['PosY']-plan['start'][1])>.3 or abs(math.atan2(math.sin(self.pose['Yaw']-plan['start'][2]),math.cos(self.pose['Yaw']-plan['start'][2])))>.3:
                raise ValueError('机器人位置或朝向已变化，请重新规划')
            result=copy.deepcopy(plan['result']);point=result['goal']
            if route_override is None:
                route=self.route_save(dict(name='前往目标',map_id=req.get('map_id'),points=[point],
                    options=dict(nav_mode=1,speed=1,gait='flat',manner=0,pause_at_point=False,disconnect_policy=req.get('disconnect_policy','stop'))),persist=False)['route']
                route['temporary']=True
            else:route=copy.deepcopy(route_override)
            route.update(preview_start=list(plan['start']),preview_source=plan['source'],preview_fingerprint=plan['fingerprint'],preview_path=result['path'])
        engine=req.get('engine','native')
        if engine not in ['native','platform','gos']:raise ValueError('未知导航执行器')
        if req.get('disconnect_policy','stop')!='stop' and engine!='native':raise ValueError('断联执行需使用机器人端寻路，请重新规划')
        if engine=='gos':
            return self._start_gos_route(dict(req,_start_generation=intent),client_intent,route,start_index)
        if engine=='platform':
            if start_index:return self.point_navigation.start(route,plan,intent,client_intent,start_index=start_index)
            return self.point_navigation.start(route,plan,intent,client_intent)
        started=self.start_route(dict(nav_mode=1,speed=1,gait='flat',pause_at_point=False,disconnect_policy=req.get('disconnect_policy','stop'),_start_generation=intent),goal=route,client_intent=client_intent)
        with self.lock:
            if self.nav.get('route',{}).get('id')==route['id']:self.nav['plan']=result
        message='正在确认模式与步态，完成后仅提交最终目标' if started.get('phase')=='preparing' else '已提交最终目标；预览路线供核对，机器人自主寻路并实时避障，终点自动结束'
        if self.demo:message='演示任务已开始；未向机器人发送指令'
        return dict(started,goal=point,planner=result['planner'],execution='native_single_goal',demo=self.demo,message=message)

    def _start_platform_route(self,req,client_intent=None,route=None,start_index=0):
        with self.lock:
            intent=self._nav_start_generation
            if req.get('_start_generation',intent)!=intent:raise ValueError('导航启动已取消')
            if client_intent is None:client_intent=self.navigation_intents.register(req,self.instance_id,self._plan_generation)
            route=copy.deepcopy(route or next((r for r in self.store.value['routes'] if r['id']==req.get('id')),None))
            if not route or not 0<=start_index<len(route.get('points',[])):raise ValueError('路线不存在或没有待执行点位')
            point=route['points'][start_index]
            query=dict(map_id=route['map_id'],x=point['x'],y=point['y'],yaw=point.get('yaw') if start_index==len(route['points'])-1 else None,engine='platform')
            cached=self.goal_plans.get(req.get('plan_id'))
            target=(cached or {}).get('result',{}).get('requested_goal') or (cached or {}).get('result',{}).get('goal',{})
            same_yaw=(target.get('yaw') is None and query['yaw'] is None or
                isinstance(target.get('yaw'),(int,float)) and isinstance(query['yaw'],(int,float)) and
                abs(math.atan2(math.sin(target['yaw']-query['yaw']),math.cos(target['yaw']-query['yaw'])))<1e-6)
            reusable=(cached and cached.get('map_id')==route['map_id'] and cached.get('execution_engine')=='platform'
                and time.monotonic()-cached['at']<60 and cached.get('connection_generation')==self._plan_generation
                and cached.get('link') is self.link and cached.get('demo')==self.demo
                and same_yaw and all(abs(target.get(k,float('inf'))-query[k])<1e-6 for k in ['x','y'])
                and math.hypot(self.pose['PosX']-cached['start'][0],self.pose['PosY']-cached['start'][1])<=.3
                and abs(math.atan2(math.sin(self.pose['Yaw']-cached['start'][2]),math.cos(self.pose['Yaw']-cached['start'][2])))<=.3
                and hashlib.sha256(Path(cached['source']).read_bytes()).hexdigest()==cached['fingerprint'])
        plan_id=req['plan_id'] if reusable else self.plan_goal(query)['plan_id']
        return self._start_planned_goal(dict(query,plan_id=plan_id),intent,client_intent,route,start_index)

    def _start_gos_route(self, req, client_intent=None, route=None, start_index=0, station_results=()):
        with self.lock:
            if self.demo:
                raise ValueError('GOS 寻路需要已连接的实机服务')
            if req.get('_start_generation', self._nav_start_generation) != self._nav_start_generation:
                raise ValueError('导航启动已取消')
            if client_intent is None:
                client_intent = self.navigation_intents.register(req, self.instance_id, self._plan_generation)
            self.navigation_intents.check(client_intent, self.instance_id, self._plan_generation)
            self.live_ready()
            self.hub.normal_mode()
            self.hub.idle()
            if self.mapping or self.reverse or self.estopped or self._reported_active_navigation():
                raise ValueError('请结束当前任务并核对正向行走状态')
            if any(self.velocity) or not stationary_feedback(self.link):
                raise ValueError('请先停稳再开始 GOS 导航')
            route = copy.deepcopy(route or next((r for r in self.store.value['routes'] if r['id']==req.get('id')), None))
            if not route or not 0 <= start_index < len(route['points']):
                raise ValueError('路线不存在或没有待执行航点')
            if any(p.get('type',1) == 3 for p in route['points']):
                raise ValueError('充电航点请使用原厂导航')
            saved = route.get('options', {})
            pause = req.get('pause_at_point', saved.get('pause_at_point', False))
            if type(pause) is not bool:
                raise ValueError('到点选项无效')
            policy = req.get('disconnect_policy', saved.get('disconnect_policy', 'stop'))
            if policy != 'stop':
                raise ValueError('GOS 自主寻路目前支持断联停车')
            selected = next((m for m in self.maps() if m['id'] == route['map_id']), None)
            if not selected or not selected.get('verified_for_robot') or selected.get('robot_ip') != self.link.ip:
                raise ValueError('请先同步当前活动地图')
            nav = dict(status='running', route=route, index=start_index, completed=start_index,
                       progress=start_index/len(route['points']), started=time.time(), pause_at_point=pause,
                       options=dict(NavMode=1, Gait=0x3002, Speed=1, ObsMode=0, Manner=0),
                       disconnect_policy=policy, station_results=list(station_results))
            nav.update(self.navigation_intents.nav_fields(client_intent))
            quiet_until = self.link.release_axis_stream()
            previous = self.nav
            self.nav = nav
            try:
                self.resident_navigation.prepared(nav, executor='gos', axis_quiet_until=quiet_until)
            except Exception:
                self.nav = previous
                raise
            return dict(ok=True, engine='resident', executor='gos', phase=nav['phase'],
                        message=nav['message'])

    def start_route(self, req, goal=None,client_intent=None):
        if goal is None:
            with self.lock:stored=next((r for r in self.store.value['routes'] if r['id']==req.get('id')),None)
            if stored and 'engine' not in req and stored.get('options',{}).get('engine'):
                req=dict(req,engine=stored['options']['engine'])
            if req.get('engine')=='gos':
                return self._start_gos_route(req,client_intent)
            if stored and any(p.get('station_action') for p in stored['points']):
                raise ValueError('含人数统计的巡检路线请选择 GOS 自主寻路')
        if goal is None and req.get('engine')=='platform':
            with self.lock:stored=next((r for r in self.store.value['routes'] if r['id']==req.get('id')),None)
            policy=req.get('disconnect_policy',(stored or {}).get('options',{}).get('disconnect_policy','stop'))
            if stored and policy=='stop' and not any(p.get('type')==3 for p in stored.get('points',[])):
                return self._start_platform_route(req,client_intent)
            # Docking remains on the robot's dedicated charging workflow.
            req=dict(req,engine='native')
        # Reserve an intent before remote reads, then release the state lock.
        # Stop/reconnect can invalidate it while a slow map request is pending.
        with self.lock:
            if req.get('_start_generation',self._nav_start_generation)!=self._nav_start_generation:raise ValueError('导航启动已取消')
            if self._greet_busy:raise ValueError('请先停止问候动作，再开始导航')
            if client_intent is None:client_intent=self.navigation_intents.register(req,self.instance_id,self._plan_generation)
            self.navigation_intents.check(client_intent,self.instance_id,self._plan_generation)
            if self.autonomy.active:raise ValueError('请先停止自主任务，再开始导航')
            if self.hub.busy:raise ValueError('后台操作正在进行，请等待完成')
            if not self.demo and self.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit']:
                raise ValueError('请先退出 SDK 流程再开始导航')
            self.live_ready()
            if self.estopped or self.sdk.get('armed'):raise ValueError('请先结束关节控制并解除软急停')
            if self.nav['status'] in ['running','cancelling']:raise ValueError('已有任务运行中')
            external=self._reported_active_navigation()
            previous_cancelled=self.nav.get('value') if self.nav.get('cancel_confirmed') else None
            if external and external['items'].get('Value')!=previous_cancelled:
                raise ValueError('机器人端仍有活动导航任务，请先停止并确认后再启动')
            route=copy.deepcopy(goal or next((r for r in self.store.value['routes'] if r['id']==req.get('id')),None))
            if not route:raise ValueError('路线不存在，请先保存路线')
            saved=route.get('options',{});pause=req.get('pause_at_point',saved.get('pause_at_point',False))
            from m20.navigation_disconnect import POLICIES
            disconnect_policy=req.get('disconnect_policy',saved.get('disconnect_policy','stop'))
            if disconnect_policy not in POLICIES:raise ValueError('未知断联策略')
            if disconnect_policy!='stop' and self.demo:raise ValueError('驻机断联任务需连接已配置的机器人端服务')
            if not isinstance(pause,bool):raise ValueError('到点暂停参数无效')
            options=dict(NavMode=integer(req.get('nav_mode',saved.get('nav_mode',0)),0,1),
                         Gait=0x3003 if req.get('gait',saved.get('gait'))=='stair' else 0x3002,
                         Speed=integer(req.get('speed',saved.get('speed',1)),0,2),ObsMode=0,
                         Manner=integer(req.get('manner',saved.get('manner',0)),0,1))
            if options['NavMode']==1 and (options['Gait']!=0x3002 or options['Speed']!=1):
                raise ValueError('自主导航仅支持敏捷平地与低速，请调整配置')
            map_=copy.deepcopy(next((m for m in self.maps() if m['id']==route['map_id']),{}))
            link=self.link;demo=self.demo
            nav=dict(status='running',phase='validating',execution_state='preparing',message='正在核对活动地图与建图状态',
                     validation_deadline=time.monotonic()+15,index=0,route=route,progress=0,completed=0,started=time.time(),
                     options=options,pause_at_point=pause,grid_id=map_.get('grid_id',0),previous_cancelled_value=previous_cancelled,disconnect_policy=disconnect_policy)
            nav.update(self.navigation_intents.nav_fields(client_intent))
            self.nav=nav;self.velocity=[0.,0.,0.]
            if link:link.set_velocity(0,0,0)
        def current():
            return (self.nav is nav and nav['status']=='running' and nav.get('phase')=='validating' and self.link is link and self.demo==demo
                    and self.navigation_intents.valid(client_intent,self.instance_id,self._plan_generation))
        try:
            if demo:
                if self.mapping:raise ValueError('建图采集或后处理尚未完成，请先停止建图并核对结果')
            else:
                build=self.hub.get('/api/maps/build/status',{})
                if build.get('in_progress',True) or build.get('phase') not in ['idle','saved','discarded','interrupted','empty']:
                    raise ValueError('建图采集或后处理尚未确认结束，请先核对建图结果')
                with self.lock:
                    if not current():raise ValueError('导航启动已取消或连接已改变')
                    location=link.record(1007,2)
                    if age_of(location)>2 or location.get('items',{}).get('Location')!=0:raise ValueError('地图定位异常或数据过期，请先重定位')
                remote=self.proxy('nos','/status')
                if remote.get('active')!=map_.get('robot_map_name'):
                    raise ValueError('路线地图与机器人当前活动地图不同，请先选择并同步活动地图')
                with self.lock:
                    if not current():raise ValueError('导航启动已取消或连接已改变')
                if not map_.get('verified_for_robot') or map_.get('robot_ip')!=link.ip:
                    synced=self.hub.sync_map(dict(name=map_['robot_map_name'],grid_id=map_.get('grid_id',0)))['map']
                    if synced['id']!=route['map_id'] or not synced['verified_for_robot']:
                        raise ValueError('地图栅格已更新，请选择同步后的地图并重新核对、保存路线')
            with self.lock:
                if not current():raise ValueError('导航启动已取消或连接已改变')
                self._begin_navigation_preparation()
                if nav['status']=='failed':raise ValueError(nav.get('error','导航准备失败'))
                self.add_log('navigation','开始路线：'+route['name'])
                return dict(ok=True,phase=nav.get('phase'))
        except Exception as exc:
            with self.lock:
                if current():nav.update(status='failed',phase='validation_failed',execution_state='failed',error=str(exc))
            raise

    def _begin_navigation_preparation(self):
        self.nav.update(phase='preparing',execution_state='preparing',
                        message='正在确认导航模式与步态',warning='准备完成前不会下发导航目标',
                        preparation=dict(deadline=time.monotonic()+8,mode_sent_at=None,gait_sent_at=None,wake_sent_at=None,nav_query_at=None))
        self._nav_link=self.link
        if self.demo:
            self.basic.update(ControlUsageMode=1,Gait=self.nav['options']['Gait'])
            self.nav.update(phase='running',execution_state='starting');self.nav.pop('warning',None)
            self._send_point()
        else:self._tick_navigation_preparation(time.time())

    def _tick_navigation_preparation(self,now):
        """Advance from current feedback only; no waiting under the state lock."""
        nav=self.nav
        if nav.get('status')!='running' or nav.get('phase')!='preparing':return
        def fail(message):
            nav.update(status='failed',phase='preparation_failed',execution_state='failed',error=message,message=message)
            nav.pop('preparation',None)
        if self.link is not getattr(self,'_nav_link',None) or not self.link:
            fail('连接已改变，已取消导航准备');return
        if self.sdk.get('armed') or self.sdk.get('vendor_running') or self.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit']:
            fail('控制权已转入 SDK，已取消导航准备');return
        prep=nav['preparation']
        def issue(key,operation):
            prep[key]=time.time();prep[key.replace('_at','_monotonic')]=time.monotonic()
            try:
                if operation() is False:raise ValueError('发送失败')
                return True
            except (ValueError,OSError) as exc:
                fail('导航准备指令发送失败，未下发目标：'+str(exc));return False
        def newer(record,key):
            return self._navigation_record_after(record,prep[key],prep.get(key.replace('_at','_monotonic')))
        if time.monotonic()>=prep['deadline']:
            fail('导航模式或步态未获新鲜反馈确认，未下发目标');return
        native=self.link.record(1007,1)
        if age_of(native)>2 or native.get('items',{}).get('Status') not in [0,1,2,3,4,5,255]:
            nav['message']='正在核对机器人端是否仍有旧导航任务'
            if prep['nav_query_at'] is None:
                issue('nav_query_at',lambda:self.link.command(1007,1,{},'导航准备查询现有任务'))
            return
        if prep['nav_query_at'] is not None and not newer(native,'nav_query_at'):return
        if native['items']['Status'] in [1,2,3,5,255]:
            if native['items'].get('Value')==nav.get('previous_cancelled_value'):
                nav['message']='旧任务停止请求已确认，等待机器人回到空闲';return
            fail('机器人端出现其他活动导航任务，未下发新目标');return
        rec=self.link.record(1002,6);b=rec.get('items',{}).get('BasicStatus',{})
        loc=self.link.record(1007,2)
        if age_of(rec)>2 or b.get('HES')!=0 or b.get('MotionState')!=17 or self.estopped:
            fail('机器人基础状态未就绪，已取消导航准备');return
        if b.get('Sleep'):
            nav['message']='正在唤醒已站立的机器人，保留原休眠设置'
            if prep['wake_sent_at'] is None:
                record=self.link.record(1101,7);settings=record.get('items',{})
                items=dict(Sleep=False)
                if age_of(record)<3 and settings.get('Auto') in [True,False] and isinstance(settings.get('Time'),int) and not isinstance(settings.get('Time'),bool) and 5<=settings['Time']<=30:
                    items.update(Auto=bool(settings['Auto']),Time=settings['Time'])
                # The vendor's wake example explicitly permits Sleep-only;
                # missing settings must not be replaced with invented defaults.
                issue('wake_sent_at',lambda:self.link.command(1101,6,items,'导航准备唤醒'))
            return
        if prep['wake_sent_at'] is not None and not newer(rec,'wake_sent_at'):return
        if age_of(loc)>2 or loc.get('items',{}).get('Location')!=0:
            fail('定位未就绪，已取消导航准备');return
        try:
            for key in ['PosX','PosY','Yaw']:number(loc['items'].get(key))
        except ValueError:
            fail('定位坐标无效，已取消导航准备');return
        if b.get('ControlUsageMode')!=1:
            nav['message']='正在切换导航模式，等待机器人确认'
            if prep['mode_sent_at'] is None:
                issue('mode_sent_at',lambda:self.link.set_mode(1))
            return
        if prep['mode_sent_at'] is not None and not newer(rec,'mode_sent_at'):return
        motion=self.link.record(1002,4);v=motion.get('items',{}).get('MotionStatus',{})
        still=age_of(motion)<.5 and all(abs(v.get(k,999))<limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)])
        if not still:
            nav['message']='等待机器人停稳后确认导航步态';return
        if b.get('Gait')!=nav['options']['Gait']:
            nav['message']='正在切换导航步态，等待机器人确认'
            if prep['gait_sent_at'] is None:
                issue('gait_sent_at',lambda:self.link.command(2,23,{'GaitParam':nav['options']['Gait']},'导航准备步态'))
            return
        for key in ['wake_sent_at','mode_sent_at','gait_sent_at']:
            if prep[key] is not None and (not newer(rec,key) or not newer(motion,key)):return
        route=nav['route']
        if route.get('preview_start'):
            start=route['preview_start'];pose=loc['items']
            if math.hypot(pose.get('PosX',0)-start[0],pose.get('PosY',0)-start[1])>.3 or abs(math.atan2(math.sin(pose.get('Yaw',0)-start[2]),math.cos(pose.get('Yaw',0)-start[2])))>.3:
                fail('机器人位置或朝向已变化，请重新规划');return
            try:current_fingerprint=hashlib.sha256(Path(route['preview_source']).read_bytes()).hexdigest()
            except OSError:
                fail('地图文件不可读取，请重新规划');return
            if current_fingerprint!=route['preview_fingerprint']:
                fail('地图内容已更新，请重新规划');return
        nav.update(phase='running',execution_state='starting',message='导航模式与步态已确认，正在下发目标')
        nav.pop('preparation',None);nav.pop('warning',None)
        self._send_point()

    def _reserve_navigation_value(self):
        # Persist the reservation before sending. A process restart must not reuse 101
        # and mistake an old task's late success for the new task's completion.
        with self.store.lock:
            previous=self.store.value.get('nav_value_highwater')
            if previous is None:previous=secrets.randbelow(1<<30)+1024
            if isinstance(previous,bool) or not isinstance(previous,int) or not 0<=previous<2147483647:
                raise ValueError('导航编号记录无效或已耗尽，未下发目标')
            value=previous+1;self.store.value['nav_value_highwater']=value
            self.store.save();self.next_value=value
            return value

    @staticmethod
    def _navigation_record_after(record,sent_at,sent_monotonic=None):
        if not math.isfinite(age_of(record)):return False
        try:
            if sent_monotonic is not None and 'monotonic_at' in record:return record['monotonic_at']>sent_monotonic
            return record.get('at',0)>sent_at
        except (TypeError,ValueError):return False

    def _reported_active_navigation(self):
        if self.demo or not self.link:return None
        record=self.link.record(1007,1)
        if age_of(record)<2 and record.get('items',{}).get('Status') in [1,2,3,5,255]:return record
        return None

    def _close_navigation_connection(self):
        """Bounded receipt wait; leaving the UI never implies hardware stopped."""
        notice=dict(robot=self.link.ip,value=self.nav.get('value'),state='unconfirmed',at=time.time(),
                    message='旧连接已关闭，但导航停止尚未确认；重连后先核对设备任务状态')
        deadline=time.monotonic()+.75
        while self.cancel_request and time.monotonic()<deadline:
            ack=self.link.record(1004,1)
            if self._navigation_record_after(ack,self.cancel_request['sent_at'],self.cancel_request.get('sent_monotonic')) and ack.get('seq')==self.cancel_request['seq']:
                if ack.get('items',{}).get('ErrorCode')==0:
                    notice.update(state='request_confirmed',message='旧导航停止请求已确认，实际停稳以机器人反馈为准')
                break
            time.sleep(.025)
        self.store.value['previous_connection_stop']=notice
        try:self.store.save()
        except OSError:notice['persistence_error']='停止记录未能写入磁盘，请保留本次未确认提示'
        return notice

    def _refresh_previous_navigation_stop(self):
        notice=self.store.value.get('previous_connection_stop')
        if not notice or not self.link or notice.get('robot')!=self.link.ip or notice.get('state') not in ['unconfirmed','active_after_reconnect']:return
        native=self.link.record(1007,1);motion=self.link.record(1002,4);values=motion.get('items',{}).get('MotionStatus',{})
        if age_of(native)>2:return
        status=native.get('items',{}).get('Status')
        changed=False
        if status in [0,4] and age_of(motion)<.5 and all(abs(values.get(k,999))<limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)]):
            notice.update(state='idle_observed_after_reconnect',resolved_at=time.time(),message='重连后已观察到导航空闲且速度反馈静止');changed=True
        elif status in [1,2,3,5,255]:
            observed=dict(value=native['items'].get('Value'),status=status)
            if notice.get('state')!='active_after_reconnect' or notice.get('observed_task')!=observed:
                notice.update(state='active_after_reconnect',observed_task=observed,message='旧设备仍反馈活动任务，请先停止；不会自动恢复或覆盖任务');changed=True
        if changed:
            try:self.store.save()
            except OSError:notice['persistence_error']='停止状态记录未能写入磁盘'

    def _send_point(self):
        if self.nav.get('status')!='running' or self.nav.get('phase')!='running':return
        if self.link:
            record=self.link.record(1002,6);b=record.get('items',{}).get('BasicStatus',{})
            if age_of(record)>2 or b.get('HES')!=0 or b.get('MotionState')!=17 or b.get('Sleep') or b.get('ControlUsageMode')!=1:
                self.nav['warning']='控制状态已改变，正在确认停止导航'
                self._cancel_navigation('control_state_changed','paused');return
        if self.nav.get('disconnect_policy','stop')!='stop':
            try:self.resident_navigation.prepared(self.nav)
            except Exception as error:self.nav.update(status='failed',phase='failed',error='驻机任务未能提交：'+str(error))
            return
        p = self.nav['route']['points'][self.nav['index']]
        try:value=self._reserve_navigation_value()
        except (ValueError,OSError) as exc:
            self.nav.update(status='failed',phase='dispatch_failed',execution_state='failed',error='目标编号未能保存，未下发导航：'+str(exc))
            raise ValueError(self.nav['error'])
        self.nav.update(value=value, sent_at=time.time(),sent_monotonic=time.monotonic(),progress_at=time.time(),progress_monotonic=time.monotonic(),
                        progress_position=None,progress_heading_error=None,execution_state='awaiting_progress',message='目标已下发，等待实际进展',demo_path_index=0)
        self.nav.pop('warning',None)
        if self.link:
            items = dict(Value=self.nav['value'], MapID=self.nav.get('grid_id',0), PosX=p['x'], PosY=p['y'], PosZ=0.,
                         AngleYaw=p['yaw'], PointInfo=p['type'], **self.nav['options'])
            try:self.nav['request'] = self.link.command(1003,1,items,'导航目标点 %s' % self.nav['value'])
            except (ValueError,OSError) as exc:
                self.nav['error']='导航目标发送结果未确认：'+str(exc)
                self._cancel_navigation('dispatch_unconfirmed','failed')
                raise ValueError(self.nav.get('failure_error',self.nav['error']))
        else:
            self.basic.update(ControlUsageMode=1, MotionState=17)

    def _advance(self):
        finished=self.nav['route']['points'][self.nav['index']]
        self.nav['index'] += 1
        self.nav['completed'] = self.nav['index']
        self.nav['progress'] = self.nav['index']/len(self.nav['route']['points'])
        if self.nav['index'] >= len(self.nav['route']['points']):
            self.stop()
            self.nav.update(after_cancel='completed',stop_reason='route_complete')
            if not self.link:self.nav['status']='completed';self._record_route_completion()
            else:self.add_log('navigation','全部航点已完成，已自动请求停止导航，等待确认')
        elif self.nav.get('pause_at_point') and finished.get('type',1)==1:
            self.stop()
            self.nav.update(after_cancel='paused',stop_reason='waypoint_pause')
            if not self.link:self.nav['status']='paused'
            self.add_log('navigation','航点已完成，自动暂停；点击继续前往下一点')
        else:
            self._send_point()

    def _record_route_completion(self, nav=None):
        nav=self.nav if nav is None else nav
        if nav.get('history_saved'):return
        nav['history_saved']=True;nav['ended']=time.time()
        self.add_log('navigation','路线已结束，停止请求已确认')
        self.store.value['history'].append(copy.deepcopy(nav))
        self.store.value['history']=self.store.value['history'][-50:];self.store.save()

    def resume_route(self):
        if self.nav.get('engine')=='resident':
            if self.nav.get('executor')=='gos' and self.nav.get('status')=='paused':
                return self._start_gos_route(dict(pause_at_point=self.nav.get('pause_at_point',False)),
                                             route=self.nav['route'],start_index=self.nav['index'],
                                             station_results=self.nav.get('station_results',[]))
            raise ValueError('驻机任务停止后请重新核对状态并开始新任务')
        if self._greet_busy:raise ValueError('请先停止问候动作，再继续导航')
        if self.nav.get('engine')=='platform':
            if self.nav.get('status')!='paused':raise ValueError('任务不在暂停状态')
            route=copy.deepcopy(self.nav['route']);index=self.nav.get('index',0)
            return self._start_platform_route(dict(id=route['id'],engine='platform'),route=route,start_index=index)
        with self.lock:
            if self.nav['status']!='paused':raise ValueError('任务不在暂停状态')
            if self._greet_busy:raise ValueError('请先停止问候动作，再继续导航')
            if self.nav['index']>=len(self.nav['route']['points']):raise ValueError('所有航点已完成，请重新执行路线')
            if self.hub.busy:raise ValueError('后台操作正在进行，请等待完成')
            self.live_ready()
            if self.estopped or self.sdk.get('armed') or self.sdk.get('vendor_running'):raise ValueError('请先结束关节控制并解除急停')
            nav=copy.deepcopy(self.nav);nav.update(status='running',phase='validating',execution_state='preparing',previous_cancelled_value=self.nav.get('value') if self.nav.get('cancel_confirmed') else None,
                validation_deadline=time.monotonic()+15,message='正在核对恢复导航的地图与状态')
            nav.pop('stop_reason',None);nav.pop('error',None);self.nav=nav
            link=self.link;demo=self.demo
            map_=copy.deepcopy(next((m for m in self.maps() if m['id']==nav['route']['map_id']),{}))
        def current():
            return self.nav is nav and nav['status']=='running' and nav.get('phase')=='validating' and self.link is link and self.demo==demo
        try:
            if not demo:
                build=self.hub.get('/api/maps/build/status',{})
                if build.get('in_progress',True) or build.get('phase') not in ['idle','saved','discarded','interrupted','empty']:raise ValueError('建图采集或后处理尚未确认结束')
                loc=link.record(1007,2)
                if age_of(loc)>2 or loc.get('items',{}).get('Location')!=0:raise ValueError('定位异常或数据过期')
                if self.proxy('nos','/status').get('active')!=map_.get('robot_map_name'):raise ValueError('活动地图已改变，请重新规划路线')
            elif self.mapping:raise ValueError('请先停止并保存建图')
            with self.lock:
                if not current():raise ValueError('导航恢复已取消或连接已改变')
                self._begin_navigation_preparation()
                if nav['status']=='failed':raise ValueError(nav.get('error','导航准备失败'))
                return dict(ok=True,phase=nav.get('phase'))
        except Exception as exc:
            with self.lock:
                if current():nav.update(status='failed',phase='validation_failed',execution_state='failed',error=str(exc))
            raise

    def _cancel_navigation(self,reason,after='paused',retry=False):
        if self.nav.get('engine')=='resident':
            self.resident_navigation.cancel(after);return
        if self.nav.get('engine')=='platform':
            self.point_navigation.stop(after,'平台导航已请求停车：'+reason);return
        if self.nav['status']=='cancelling' and not retry:return
        self.velocity=[0.,0.,0.]
        if after=='failed' and self.nav.get('error'):self.nav['failure_error']=self.nav['error']
        if self.link:
            self.link.set_velocity(0,0,0)
            self.cancel_request=None
            self.nav.update(status='cancelling',phase='stopping',execution_state='stopping',after_cancel=after,
                            cancel_monotonic=time.monotonic(),stop_reason=reason)
            try:self.cancel_request=self.link.command(1004,1,{},'停止导航：'+reason)
            except (ValueError,OSError):
                message='停止请求发送失败，确认前不会启动新任务'
                self.nav.update(phase='cancel_unconfirmed',execution_state='stop_unconfirmed',error=message,cancel_error=message)
        else:self.nav.update(status=after,phase=after,execution_state=after,stop_reason=reason)

    def _tick_navigation_progress(self,pose,now):
        """A task ACK/Running bit is not evidence of actual motion."""
        nav=self.nav;mono=time.monotonic()
        goal=nav['route']['points'][nav['index']]
        native=self.link.record(1007,1) if self.link else {};state=native.get('items',{})
        if goal['type']==3 and age_of(native)<2 and state.get('Value')==nav.get('value') and state.get('Status')==5:
            nav.update(execution_state='docking',message='原厂正在执行入桩充电流程')
            return  # Docking uses its own controller and the existing task timeout.
        try:
            position=(number(pose.get('PosX')),number(pose.get('PosY')));yaw=number(pose.get('Yaw'))
        except ValueError:
            nav['error']='定位坐标无效，正在停止导航';self._cancel_navigation('invalid_pose','failed');return
        distance=math.hypot(goal['x']-position[0],goal['y']-position[1])
        heading=goal['yaw'] if distance<.25 else math.atan2(goal['y']-position[1],goal['x']-position[0])
        error=abs(math.atan2(math.sin(heading-yaw),math.cos(heading-yaw)))
        previous=nav.get('progress_position');best=nav.get('progress_heading_error')
        if previous is None:
            nav.update(progress_position=position,progress_at=now,progress_monotonic=mono,progress_heading_error=error)
            nav['no_progress_s']=0.;return
        translated=math.dist(position,previous)>.15
        aligned=best is not None and best-error>.12
        if translated or aligned:
            nav.update(progress_position=position,progress_at=now,progress_monotonic=mono,
                       progress_heading_error=error,execution_state='moving',message='正在前往目标')
            nav.pop('warning',None)
        elif best is None:nav['progress_heading_error']=error
        elapsed=max(0,mono-nav.get('progress_monotonic',mono))
        nav['no_progress_s']=round(elapsed,1)
        if elapsed>=25:
            nav.update(warning='持续无位移或有效转向，正在确认停车；确认后可重新规划或手动调整位置',
                       recovery=dict(action='replan_or_reposition',automatic_resends=0))
            self._cancel_navigation('no_progress','paused')
        elif elapsed>=8:
            nav.update(execution_state='waiting_progress',
                       warning='任务仍被机器人接受，但暂未检测到位移或有效转向；可能在等待通路或局部规划')

    def tick(self, dt):
        with self.lock:
            now = time.time()
            if self.velocity_deadline is not None and time.monotonic()>=self.velocity_deadline:
                self.velocity=[0.,0.,0.];self.velocity_deadline=None
                if self.link:self.link.set_velocity(0,0,0)
            self.autonomy.watchdog()
            self.point_navigation.tick()
            if self.sdk.get('armed') and now-self.lease_at > .6:
                self.stop()
                self.add_log('warning', '关节控制会话超时，已停止并转阻尼；需要重新开启会话')
            if self.demo:
                if now-self.velocity_at > .5:
                    self.velocity = [0.,0.,0.]
                f, s, yaw = self.velocity
                theta = self.pose['Yaw']
                f *= -1 if self.reverse else 1
                self.pose['PosX'] += (math.cos(theta)*f-math.sin(theta)*s)*dt
                self.pose['PosY'] += (math.sin(theta)*f+math.cos(theta)*s)*dt
                self.pose['Yaw'] += yaw*dt
                if self.sdk.get('armed') and hasattr(self,'trajectory'):
                    t = min(1., (now-self.trajectory['start'])/self.trajectory['duration'])
                    blend = t*t*(3-2*t)
                    self.positions = [a+(b-a)*blend for a,b in zip(self.trajectory['origin'], self.target)]
                    for i, v in enumerate(self.wheels):
                        self.positions[i*4+3] += v*dt
                if self.nav['status'] == 'running' and self.nav.get('engine')!='platform':
                    p = self.nav['route']['points'][self.nav['index']]
                    reference=self.nav['route'].get('preview_path') if self.nav['route'].get('temporary') else None
                    if reference:
                        i=self.nav.get('demo_path_index',0)
                        while i<len(reference)-1 and math.hypot(reference[i]['x']-self.pose['PosX'],reference[i]['y']-self.pose['PosY'])<.08:i+=1
                        self.nav['demo_path_index']=i
                        if i<len(reference)-1:p=dict(reference[i],yaw=0,type=0)
                    dx,dy = p['x']-self.pose['PosX'],p['y']-self.pose['PosY']
                    d = math.hypot(dx,dy)
                    if d < .08:
                        self.pose['Yaw'] = p['yaw']
                        if p['type']==3: self.basic['Charge']=2
                        self._advance()
                    else:
                        self.nav['execution_state']='moving'
                        speed=min(d,1.4*dt)
                        self.pose['PosX'] += dx/d*speed
                        self.pose['PosY'] += dy/d*speed
                        self.pose['Yaw']=math.atan2(dy,dx)
                return
            self.resident_navigation.tick()
            if self.nav.get('engine')=='resident':
                if self.link:
                    self.basic=self.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                    self.pose=self.link.record(1007,2).get('items',{})
                return
            if not self.link:
                return
            b = self.link.record(1002,6)
            self._refresh_previous_navigation_stop()
            self.basic = b.get('items',{}).get('BasicStatus',{})
            self.pose = self.link.record(1007,2).get('items',{})
            dev = self.link.record(1002,5).get('items',{})
            self.lights = dev.get('Led',{}).get('Fill',{})
            m = self.link.record(1002,4).get('items',{})
            # Vendor MotorStatus units/zero frame are not guaranteed to be SDK radians.
            self.raw = {'basic':b,'motion':self.link.record(1002,4),'device':self.link.record(1002,5),
                        'pose':self.link.record(1007,2),'navigation':self.link.record(1007,1),
                        'errors':self.link.record(1002,3)}
            receipts=[]
            if hasattr(self.link,'events'):
                with self.link.lock:
                    receipts=list(self.link.events)
                    self.link.events.clear()
                for rec in receipts:
                    if 'ErrorCode' in rec['items']:
                        self.responses.insert(0,rec)
                        code=rec['items']['ErrorCode']
                        if isinstance(code,int) and code not in [0,12,0x2300,0x2302]:
                            detail=explain_error(code)
                            self.add_log('error','机器人响应 %s:%s %s %s'%(rec['type'],rec['command'],detail['hex'],detail['title']))
                del self.responses[50:]
                self.raw['responses']=self.responses
            errs = self.link.record(1002,3).get('items',{}).get('ErrorList',[])
            self.faults = [explain_error(e.get('errorCode',0),int(e.get('component',0))) for e in errs if isinstance(e,dict)]
            keys={(f['hex'],f['component']) for f in self.faults}
            for f in self.faults:
                if (f['hex'],f['component']) not in self.previous_faults:
                    self.events.insert(0,dict(f, at=now))
            self.previous_faults=keys
            del self.events[200:]
            if self.nav['status'] == 'cancelling' and self.nav.get('engine')!='platform' and self.cancel_request:
                ack=self.link.record(1004,1)
                if self._navigation_record_after(ack,self.cancel_request['sent_at'],self.cancel_request.get('sent_monotonic')) and ack.get('seq')==self.cancel_request['seq']:
                    if ack['items'].get('ErrorCode')==0:
                        self.nav['status']=self.nav.get('after_cancel','cancelled')
                        self.nav.update(phase=self.nav['status'],execution_state=self.nav['status'],cancel_confirmed=True)
                        cancel_error=self.nav.pop('cancel_error',None)
                        if cancel_error and self.nav.get('error')==cancel_error:self.nav.pop('error',None)
                        if self.nav.get('failure_error'):self.nav['error']=self.nav.pop('failure_error')
                        if self.nav.get('stop_reason')=='no_progress':
                            self.nav.update(message='持续无进展，停止请求已确认',warning='任务已暂停；核对停稳后可重新规划或手动调整位置')
                        if self.nav['status']=='completed':self._record_route_completion()
                    else:
                        message='机器人未接受停止请求，请核对现场；新任务暂不可启动'
                        self.nav.update(phase='cancel_rejected',execution_state='stop_unconfirmed',error=message,cancel_error=message)
                elif time.monotonic()-self.nav.get('cancel_monotonic',time.monotonic())>5:
                    message='停止请求未获确认，请核对现场；确认前不会启动新任务'
                    self.nav.update(phase='cancel_unconfirmed',execution_state='stop_unconfirmed',error=message,cancel_error=message)
            if self.nav['status']=='running' and self.nav.get('engine')!='platform' and self.nav.get('phase')=='preparing':
                self._tick_navigation_preparation(now)
            if self.nav['status']=='running' and self.nav.get('phase')=='validating' and time.monotonic()>self.nav['validation_deadline']:
                self.nav.update(status='failed',phase='validation_timeout',execution_state='failed',error='导航地图核对超时，未下发目标')
            if self.nav['status']=='running' and self.nav.get('engine')!='platform' and self.nav.get('phase','running')=='running':
                loc=self.link.record(1007,2)
                pose=loc.get('items',{})
                native=self.link.record(1007,1);ni=native.get('items',{})
                charging_goal=self.nav['route']['points'][self.nav['index']]['type']==3
                docking=charging_goal and ni.get('Value')==self.nav['value'] and ni.get('Status') in [4,5] and age_of(native)<2
                if time.monotonic()-self.nav.get('sent_monotonic',time.monotonic())>180 or age_of(loc)>3 or pose.get('Location')!=0:
                    self.nav['error']='导航超时或定位状态过期/丢失，正在确认停止'
                    self._cancel_navigation('localization_or_timeout','failed')
                elif age_of(b)>2 or self.basic.get('HES')!=0 or not docking and (self.basic.get('MotionState')!=17 or self.basic.get('Sleep')):
                    self.nav['warning']='机器人控制状态未就绪，正在确认停止导航'
                    self._cancel_navigation('control_state_changed','paused')
                elif self._navigation_record_after(native,self.nav['sent_at'],self.nav.get('sent_monotonic')) and ni.get('Value')==self.nav['value'] and ni.get('Status')==255:
                    self.nav['warning']='机器人端已暂停导航，正在确认停止；不会自动下发下一点'
                    self._cancel_navigation('robot_paused','paused')
                elif self._navigation_record_after(native,self.nav['sent_at'],self.nav.get('sent_monotonic')) and ni.get('Value')==self.nav['value'] and ni.get('Status')==0 and ni.get('ErrorCode')==0x2302:
                    self.nav.update(status='cancelled',phase='cancelled',execution_state='cancelled',stop_reason='robot_cancelled',message='机器人端已取消本次任务，不会继续下发下一点')
                else:
                    # Preserve short-lived terminal reports even if a later poll
                    # has already overwritten the latest record for this message.
                    records=[r for r in receipts if (r.get('type'),r.get('command')) in [(1003,1),(1007,1)]]
                    records += [self.link.record(1003,1),self.link.record(1007,1)]
                    for rec in sorted(records,key=lambda r:r.get('monotonic_at',r.get('at',0))):
                        it=rec.get('items',{})
                        if not self._navigation_record_after(rec,self.nav['sent_at'],self.nav.get('sent_monotonic')) or it.get('Value')!=self.nav['value']:
                            continue
                        code=it.get('ErrorCode',0);status=it.get('Status')
                        # This firmware returns to idle after NavWorker finishes:
                        # Status=0 + 0x2300 is a completed point, not just an ACK.
                        completed=(code==0x2300 and status in [0,4]) or (status==4 and code==0)
                        if completed:
                            self.nav['last_completion']=dict(value=self.nav['value'],status=status,error_code=code,at=rec['at'])
                            self.add_log('navigation','航点 %s/%s 已完成（Value=%s，Status=%s，0x%04X）'%(
                                self.nav['index']+1,len(self.nav['route']['points']),self.nav['value'],status,code))
                            self._advance()
                            break
                        elif code not in [0,12,0x2300]:
                            self.nav.update(status='failed',phase='failed',execution_state='failed',error=explain_error(code)['title'])
                            break
                    if self.nav['status']=='running':
                        if self.basic.get('ControlUsageMode')!=1:
                            self.nav['warning']='已离开导航控制模式，正在确认停止'
                            self._cancel_navigation('control_state_changed','paused')
                        else:self._tick_navigation_progress(pose,now)

    def bridge_poll(self):
        while not self.done:
            if not self.demo and (self.config.get('sdk') or self.hub.connections.value['targets']['sdk']['enabled']):
                try:
                    status = self.proxy('sdk','/status',timeout=1)
                    with self.lock:
                        self.sdk.update(status)
                        self.sdk_seen=time.time()
                        if status.get('fresh') and len(status.get('joints',[]))==16:
                            self.positions=[j['position'] for j in status['joints']]
                except ValueError:
                    with self.lock: self.sdk['connected']=False
            time.sleep(.25)

    def snapshot(self):
        with self.lock:
            now=time.time()
            online=self.demo or (self.link and age_of(self.link.record(1002,6))<2)
            if self.demo:
                t=now%300
                dev=dict(BatteryList=[dict(BatteryLevel=86,Voltage=76.2,battery_temperature=31),dict(BatteryLevel=84,Voltage=76.0,battery_temperature=32)],
                         CPU={host:dict(Temperature=42+i*3+math.sin(t/4),FrequencyApp=18+i*6) for i,host in enumerate(['AOS','NOS','GOS'])},
                         GPS=dict(FixQuality=4,NumSatellites=24,Latitude=30.2741,Longitude=120.1551,HDOP=.8),
                         DevEnable=dict(Lidar=dict(Front=1,Back=1),Video=dict(Front=1,Back=1),GPS=1))
                motion=dict(Roll=.01*math.sin(t),Pitch=.008*math.cos(t),Yaw=self.pose['Yaw'],LinearX=self.velocity[0],LinearY=self.velocity[1],OmegaZ=self.velocity[2],Height=.56)
                # 演示模式的打招呼：用合成姿态变化让界面可预览，不含任何实机指令。
                demo_greet=self._greet_demo
                if demo_greet:
                    until,mode,began=demo_greet
                    if time.monotonic()<until:
                        elapsed=time.monotonic()-began
                        if mode=='nod':
                            # 预览幅度 ±0.3 rad（约 17°），周期与实机序列一致
                            motion['Pitch']+=.3*math.sin(2*math.pi*elapsed/greeting.NOD_PERIOD_S)
                        else:
                            # 预览幅度 ±0.10 m，周期与实机序列一致
                            motion['Height']+=.10*math.sin(2*math.pi*elapsed/greeting.BOB_PERIOD_S)
                    else:
                        self._greet_demo=None
                joints=[dict(j,position=self.positions[i],target=self.target[i],velocity=self.wheels[i//4] if i%4==3 else 0.,
                             torque=0.,temperature=38+i%5+math.sin(t/7),fresh=True,source='demo') for i,j in enumerate(JOINTS)]
                sdk=dict(self.sdk,connected=True,backend='demo',fresh=True,vendor_running=self.hub.demo_policy.get('vendor_running',False))
            else:
                dev=self.link.record(1002,5).get('items',{}) if self.link else {}
                motion=self.link.record(1002,4).get('items',{}).get('MotionStatus',{}) if self.link else {}
                sdk=dict(self.sdk)
                sdk['connected']=sdk.get('connected',False) and age_of(dict(at=self.sdk_seen))<2
                joints=[dict(j,position=None,target=None,velocity=None,torque=None,temperature=None,fresh=False,source='unavailable') for j in JOINTS]
                temps=dev.get('DeviceTemperature',{}).get('Motor',[])
                for i,j in enumerate(joints):
                    if i<len(temps): j['temperature']=temps[i]
                if sdk['connected'] and sdk.get('fresh') and len(sdk.get('joints',[]))==16:
                    for i,j in enumerate(joints): j.update(sdk['joints'][i],source='sdk_calibrated')
            self.history.append(dict(t=now,temp=max([v.get('Temperature',0) for v in dev.get('CPU',{}).values()] or [0]),velocity=motion.get('LinearX',0)))
            del self.history[:-120]
            pose_view=self.pose_monitor.update(self.pose,self.velocity,motion,
                pose_age=0. if self.demo else age_of(self.link.record(1007,2)) if self.link else float('inf'),
                motion_age=0. if self.demo else age_of(self.link.record(1002,4)) if self.link else float('inf'),
                relocation_epoch=(self._plan_generation,id(self.link),self.pose.get('Relocation')))
            return dict(mode='demo' if self.demo else 'live',online=bool(online),server_instance=self.instance_id,pose_view=pose_view,resident_navigation=self.resident_navigation.snapshot(),
                        connection_generation=self._plan_generation,
                        navigation_preparation=dict(client_intent_protocol=1,verify_current_localization=True),
                        previous_connection_stop=copy.deepcopy(self.store.value.get('previous_connection_stop')),
                        ip=self.link.ip if self.link else None,basic=self.basic,motion=motion,device=dev,
                        pose=self.pose,velocity=self.velocity,lights=self.lights,reverse=self.reverse,
                        estopped=self.estopped,joints=joints,sdk=sdk,nav=self.nav,faults=self.faults,
                        events=self.events,audit=self.audit[:100],maps=self.maps(),routes=self.store.value['routes'],
                        poses=self.store.value['poses'],task_history=self.store.value['history'],history=self.history,
                        config={role:self.hub.connections.endpoint(role)[0] if self.hub.connections.value['targets'][role]['enabled'] else self.config.get(role,'') for role in ['sdk','nos','gos','train']}, mapping=self.mapping,server_time=now,
                        raw=self.raw,
                        greeting=self.greeting.snapshot(),cameras=self.cameras.status(),autonomy=self.autonomy.snapshot(),map_learning=self.map_learning.snapshot(),vision_models=vision_catalog(),people_counts=self.people.status(),
                        background_operation=self.hub.busy,
                        ppe=original.PPE.status() if original.PPE and not self.demo else dict(enabled=False,demo=self.demo),
                        capabilities=dict(udp=True,sdk=sdk.get('connected',False),training=bool(self.config.get('train')),nos=bool(self.config.get('nos'))))



APP=None
TOKEN=None
SESSIONS=set()
ACCESS=None


class Handler(original.Handler):
    def _cors(self):
        pass  # Operations platform is same-origin.

    def _ppe_frame(self,query):
        if APP.demo or not original.PPE or not original.PPE.enabled:
            return self._json(dict(ok=False,err='安全帽识别尚未启动'),409)
        packet=original.PPE.frame(int(query.get('after',['-1'])[0]),timeout=1)
        if not packet:return self._json(dict(ok=False,err=original.PPE.status().get('error') or '等待新检测画面'),503)
        sequence,jpeg,age=packet
        self.send_response(200);self.send_header('Content-Type','image/jpeg')
        self.send_header('Content-Length',str(len(jpeg)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Frame-Sequence',str(sequence));self.send_header('X-Frame-Age',str(age))
        self.end_headers();self.wfile.write(jpeg)

    def _camera_frame(self,query):
        if APP.demo:return self._json(dict(ok=False,err='演示模式没有实机视频'),409)
        cam=query.get('camera',['front'])[0];after=int(query.get('after',['-1'])[0])
        feed=APP.cameras.get(cam);feed.subscribe()
        try:
            frame=feed.read_packet(after=after,timeout=4 if after<0 else 1)
            if frame is None:return self._json(dict(ok=False,err=feed.status().get('error') or '等待相机画面'),503)
            seq,jpeg,age=frame
            self.send_response(200);self.send_header('Content-Type','image/jpeg')
            self.send_header('Content-Length',str(len(jpeg)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Frame-Sequence',str(seq));self.send_header('X-Frame-Age',str(round(age,4)))
            self.end_headers();self.wfile.write(jpeg)
        except (BrokenPipeError,ConnectionResetError):pass
        finally:feed.unsubscribe()

    def _people_frame(self,query,snapshot=False):
        packet=APP.people.frame(after=int(query.get('after',['-1'])[0]),timeout=1)
        if not packet:return self._json(dict(ok=False,err=APP.people.status().get('error') or '人数统计没有最新画面，请启动或继续统计'),409)
        seq,jpeg,age=packet
        self.send_response(200);self.send_header('Content-Type','image/jpeg');self.send_header('Content-Length',str(len(jpeg)))
        self.send_header('Cache-Control','no-store');self.send_header('X-Frame-Sequence',str(seq));self.send_header('X-Frame-Age',str(round(age,4)))
        if snapshot:self.send_header('Content-Disposition','attachment; filename="people-'+time.strftime('%Y%m%d-%H%M%S')+'.jpg"')
        self.end_headers();self.wfile.write(jpeg)

    def _video(self,cam):
        feed=APP.cameras.get(cam)
        try:feed.subscribe()
        except ValueError as exc:return self._json(dict(ok=False,err=str(exc)),503)
        try:
            first=feed.read(timeout=6)
            if first is None:return self._json(dict(ok=False,err=feed.status().get('error') or '相机连接超时，请重试'),503)
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control','no-store')
            self.send_header('Connection','close')
            self.end_headers()
            current=first;last_frame=time.monotonic()
            while not APP.done and APP.link and APP.link.ip==feed.ip and self._auth():
                if current:
                    seq,jpeg=current;last_frame=time.monotonic()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    self.wfile.flush()
                elif time.monotonic()-last_frame>8:return
                current=feed.read(after=seq,timeout=.5)
        except (BrokenPipeError,ConnectionResetError,OSError):pass
        finally:feed.unsubscribe()

    def _auth(self):
        if ACCESS:
            return ACCESS.admitted(self.client_address[0])
        if not TOKEN: return True
        if hmac.compare_digest(self.headers.get('X-API-Token',''),TOKEN): return True
        try:
            cookie=SimpleCookie(self.headers.get('Cookie',''))
            return cookie.get('m20_session') and cookie['m20_session'].value in SESSIONS
        except Exception:
            return False

    def _admitted(self):
        if not ACCESS:return True
        peer=self.client_address[0]
        try:hostname=urlparse('http://'+self.headers.get('Host','')).hostname
        except ValueError:hostname=None
        allowed=['localhost','127.0.0.1','::1']+[urlparse(url).hostname for url in ACCESS.urls]
        if not ACCESS.admitted(peer) or hostname not in allowed:
            self._json(dict(ok=False,err='此地址不在平台共享范围内'),403)
            return False
        return True

    def _json(self,obj,code=200):
        body=json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers();self.wfile.write(body)

    def _file(self,path,mime=None):
        if not path.is_file():
            return self._json(dict(ok=False,err='文件不存在'),404)
        body=path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type',mime or mimetypes.guess_type(str(path))[0] or 'application/octet-stream')
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-cache' if path.suffix in ['.html','.js','.css'] else 'private, max-age=3600')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        self.wfile.write(body)

    def _download(self,key):
        meta=APP.hub.files.get(key);path=APP.hub.files.path(key)
        self.send_response(200)
        self.send_header('Content-Type','application/zip' if meta['kind']=='map-package' else 'application/octet-stream')
        self.send_header('Content-Disposition',"attachment; filename*=UTF-8''"+quote(meta['name'],safe=''))
        self.send_header('Content-Length',str(meta['size']))
        self.send_header('Cache-Control','no-store')
        self.end_headers()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(1024**2),b''):self.wfile.write(block)

    def do_GET(self):
        if not self._admitted():return
        p=urlparse(self.path).path
        try:
            if p=='/api/auth':
                if ACCESS:return self._json(dict(ok=True,required=False,authenticated=True,sharing=True))
                return self._json(dict(ok=True,required=bool(TOKEN),authenticated=bool(self._auth())))
            if p.startswith(('/api/','/video/','/maps/','/export/')) and not self._auth():
                return self._json(dict(ok=False,err='需要登录控制台'),401)
            if p in ['/api/platform','/api/status']: return self._json(APP.snapshot())
            if p=='/api/camera/frame':return self._camera_frame(parse_qs(urlparse(self.path).query))
            if p=='/api/ppe/frame':return self._ppe_frame(parse_qs(urlparse(self.path).query))
            if p in ['/api/people/frame','/api/people/snapshot']:return self._people_frame(parse_qs(urlparse(self.path).query),p.endswith('snapshot'))
            if p=='/api/vision/models':return self._json(dict(ok=True,models=vision_catalog()))
            if p.startswith('/api/vision/check-image/'):
                from m20.map_formats import identifier
                return self._file(APP.model_checks.directory/(identifier(p.rsplit('/',1)[-1])+'.jpg'),'image/jpeg')
            if p=='/api/sharing':
                return self._json(ACCESS.status(peer=self.client_address[0]) if ACCESS else dict(ok=True,enabled=False))
            if p.startswith('/api/files/download/'):return self._download(p.rsplit('/',1)[-1])
            if p.startswith('/api/cloud/data/'):
                from m20.map_formats import identifier
                return self._file(APP.hub.previews/(identifier(p.rsplit('/',1)[-1])+'.json'),'application/json')
            if p.startswith('/api/'):
                result=APP.hub.get(p,parse_qs(urlparse(self.path).query))
                if result is not None:return self._json(result)
            if p=='/api/catalog': return self._json(dict(joints=JOINTS,errors=[explain_error(c) for c in ERRORS]))
            if p=='/api/train/status':
                return self._json(APP.training.status() if APP.demo else APP.proxy('train','/status'))
            if p=='/api/train/checkpoints':
                return self._json(dict(checkpoints=APP.training.checkpoints()) if APP.demo else APP.proxy('train','/checkpoints'))
            if p=='/api/nos/status': return self._json(APP.proxy('nos','/status'))
            if p=='/api/nos/logs': return self._json(APP.proxy('nos','/logs'))
            if p=='/api/export':
                return self._json(dict(exported_at=time.time(),workspace=APP.store.value,snapshot=APP.snapshot()))
            if p.startswith('/video/'):
                if APP.demo: return self._json(dict(ok=False,err='演示模式没有实机视频流'),409)
                if p=='/video/ppe': return self._video_ppe()
                if p.split('/')[-1] in ['front','rear']: return self._video(p.split('/')[-1])
            if p=='/docs': return self._file(ROOT/'docs/api.md','text/plain; charset=utf-8')
            if p=='/calibration.example.json':return self._file(ROOT/'bridge/calibration.example.json','application/json')
            base=APP.store.path.parent/'maps' if p.startswith('/maps/') else ROOT/'platform_web'
            rel=p[len('/maps/'):] if p.startswith('/maps/') else p.lstrip('/') or 'index.html'
            file=(base/rel).resolve()
            if base.resolve() not in file.parents: return self._json(dict(ok=False,err='路径无效'),403)
            self._file(file)
        except (BrokenPipeError,ConnectionResetError): pass
        except ValueError as exc: self._json(dict(ok=False,err=str(exc)),400)

    def do_POST(self):
        if not self._admitted():return
        try:
            origin=self.headers.get('Origin')
            if origin and urlparse(origin).netloc!=self.headers.get('Host'):
                return self._json(dict(ok=False,err='不允许跨站控制请求'),403)
            if not self.headers.get('Content-Type','').startswith('application/json'):
                return self._json(dict(ok=False,err='请求必须使用 application/json'),415)
            size=integer(self.headers.get('Content-Length',0),0,18_000_000)
            req=json.loads(self.rfile.read(size) or b'{}')
            if not isinstance(req,dict): raise ValueError('请求必须是 JSON 对象')
            p=urlparse(self.path).path
            if p=='/api/auth':
                if ACCESS:return self._json(dict(ok=True))
                if TOKEN and not hmac.compare_digest(str(req.get('token','')),TOKEN):
                    return self._json(dict(ok=False,err='令牌不正确'),401)
                session=secrets.token_urlsafe(32)
                SESSIONS.add(session)
                b=b'{"ok":true}'
                self.send_response(200)
                self.send_header('Set-Cookie','m20_session='+session+'; Path=/; HttpOnly; SameSite=Strict')
                self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(b)))
                self.end_headers();self.wfile.write(b);return
            if not self._auth(): return self._json(dict(ok=False,err='需要登录控制台'),401)
            if p=='/api/sharing/control':
                return self._json(ACCESS.status(peer=self.client_address[0]) if ACCESS else dict(ok=True,enabled=False))
            if p=='/api/sharing/password':return self._json(dict(ok=False,err='平台已取消访问口令'),410)
            if p=='/api/vision/check':
                return self._json(APP.hub.operations.submit('model-check',lambda progress:APP.model_checks.run(req.get('kind','ppe'),progress)))
            if p=='/api/maps/refinement':return self._json(APP.map_learning.control(req))
            if p=='/api/people':return self._json(APP.people.control(req))
            if p=='/api/nav/plan':return self._json(APP.plan_goal(req))
            if p=='/api/nav/goal':return self._json(APP.start_goal(req))
            if p=='/api/nav/lease':return self._json(APP.point_navigation.renew(req.get('run_id')))
            if p=='/api/nav/start':return self._json(APP.start_route(req))
            if p=='/api/nav/intent/cancel':return self._json(APP.cancel_navigation_intent(req))
            if p=='/api/nav/resume':return self._json(APP.resume_route())
            result=APP.autonomy.post(p,req)
            if result is not None:return self._json(result)
            result=APP.hub.post(p,req)
            if result is not None:return self._json(result)
            # Slow remote operations never block the local velocity watchdog.
            if p.startswith('/api/train/'):
                op=p.rsplit('/',1)[-1]
                if op not in ['preview','start','stop']: raise ValueError('未知训练操作')
                if APP.demo:
                    result=APP.training.preview(req) if op=='preview' else APP.training.start(req) if op=='start' else APP.training.stop(req.get('id'))
                else: result=APP.proxy('train','/'+op,req,timeout=10)
                return self._json(result)
            if p=='/api/nos/action':
                with APP.lock:
                    if APP._greet_busy:raise ValueError('请先停止问候动作，再操作建图服务')
                    if APP.nav['status'] in ['running','cancelling'] or any(APP.velocity) or APP.sdk.get('armed'):
                        raise ValueError('请先停止运动并结束导航/关节控制')
                result=APP.proxy('nos','/action',req,timeout=30)
                with APP.lock:
                    if req.get('action') in ['apply','stop_mapping','mapping']:
                        for map_ in APP.store.value['maps']: map_['verified_for_robot']=False
                        APP.store.save()
                    APP.mapping=req.get('action')=='mapping'
                    APP.add_log('mapping','NOS 操作: '+str(req.get('action')))
                return self._json(result)
            if p=='/api/maps/sync':
                result=APP.proxy('nos','/active-map',timeout=10)
                with APP.lock:
                    imported=APP.import_map(result)
                    imported['map']['verified_for_robot']=True
                    imported['map']['robot_ip']=APP.link.ip if APP.link else None
                    APP.store.save()
                return self._json(imported)
            with APP.lock:
                if p in ['/api/charge','/api/sleep','/api/maintenance','/api/localize']:
                    APP.cancel_greeting('操作者已切换设备操作，问候取消')
                if p in ['/api/connect','/api/config']: result=APP.connect(req)
                elif p=='/api/cmd': result=APP.command(str(req.get('action','')))
                elif p=='/api/velocity': result=APP.set_velocity(req)
                elif p=='/api/camera/reconnect':
                    APP.cameras.get(req.get('camera','front')).reconnect()
                    result=dict(ok=True)
                elif p.startswith('/api/joints/'): result=APP.joint_control(p.rsplit('/',1)[-1],req)
                elif p=='/api/route/save': result=APP.route_save(req)
                elif p=='/api/route/delete':
                    if APP.nav['status'] in ['running','cancelling']: raise ValueError('任务运行中不能删除路线')
                    APP.store.value['routes']=[r for r in APP.store.value['routes'] if r['id']!=req.get('id')]
                    APP.store.save();result=dict(ok=True)
                elif p=='/api/nav/goal': result=APP.start_goal(req)
                elif p in ['/api/nav/cancel','/api/nav/pause']:
                    after='paused' if p.endswith('pause') else 'cancelled'
                    if APP.resident_navigation.gos_busy():
                        APP.resident_navigation.cancel(after)
                        result=dict(ok=True,delivery='pending',message='正在等待 GOS 确认停车')
                    else:
                        result=APP.stop()
                        if APP.nav.get('engine')=='platform':
                            if APP.nav['status']=='cancelling':APP.point_navigation.after_stop=after
                            else:APP.nav.update(status=after,phase=after,execution_state=after)
                        elif APP.nav['status']=='cancelling': APP.nav['after_cancel']=after
                        else: APP.nav['status']=after
                elif p=='/api/maps/import': result=APP.import_map(req)
                elif p=='/api/pose/save':
                    pose=dict(id=uuid.uuid4().hex[:10],name=str(req.get('name','新姿态'))[:80],positions=list(APP.positions))
                    if not APP.demo and not APP.sdk.get('fresh'): raise ValueError('没有 SDK 关节反馈')
                    APP.store.value['poses'].append(pose);APP.store.save();result=dict(ok=True)
                elif p=='/api/localize':
                    APP.live_ready()
                    if APP.nav['status'] in ['running','cancelling'] or any(APP.velocity) or APP.sdk.get('armed'):
                        raise ValueError('请先停止运动并结束导航/关节控制')
                    pos={k:number(req.get(k,0)) for k in ['PosX','PosY','PosZ','Yaw']}
                    if APP.link: result=APP.link.command(2101,1,pos,'重定位')
                    else: APP.pose.update(pos,Location=0);result=dict(ok=True)
                elif p=='/api/charge':
                    APP.live_ready();charge=integer(req.get('charge'),0,2)
                    if APP.sdk.get('armed') or APP.nav['status'] in ['running','cancelling']: raise ValueError('请先结束关节/导航任务')
                    if APP.link: result=APP.link.command(2,24,{'Charge':charge},'充电控制')
                    else: APP.basic['Charge']=2 if charge==1 else 0;result=dict(ok=True)
                elif p=='/api/sleep':
                    APP.live_ready()
                    if req.get('sleep') and (APP.basic.get('MotionState')!=4 or APP.basic.get('ControlUsageMode') not in [0,2]):
                        raise ValueError('需先趴下且切换常规/辅助模式才能休眠')
                    items=dict(Auto=bool(req.get('auto',True)),Time=integer(req.get('minutes',5),5,30))
                    if 'sleep' in req: items['Sleep']=bool(req['sleep'])
                    if APP.link: result=APP.link.command(1101,6,items,'休眠控制')
                    else:
                        if 'Sleep' in items: APP.basic['Sleep']=int(items['Sleep'])
                        result=dict(ok=True)
                elif p=='/api/maintenance':
                    APP.live_ready()
                    if APP.nav['status'] in ['running','cancelling'] or any(APP.velocity) or APP.sdk.get('armed'):
                        raise ValueError('请先停止运动并结束导航/关节控制')
                    action=req.get('action')
                    if action=='led':
                        mode=integer(req.get('type',0),0,5)
                        if mode not in [0,1,2,4,5]: raise ValueError('灯语模式无效')
                        front=integer(req.get('front',2),0,3);rear=integer(req.get('rear',2),0,3)
                        cycle=integer(req.get('cycle',2),1,60)
                        items=dict(CustomMode=bool(req.get('enabled',True)),Led=[dict(Type=mode,Color=[c],Cycle=cycle) for c in [front,rear]])
                        cmd=14
                    elif action=='gps-time':
                        cmd=16;items=dict(Enable=bool(req.get('enabled',True)),Interval=integer(req.get('interval',12),1,168))
                    elif action=='external-time':
                        stamp=str(req.get('time',''))
                        try: datetime.strptime(stamp,'%Y-%m-%d %H:%M:%S')
                        except ValueError: raise ValueError('时间格式必须为 YYYY-MM-DD HH:MM:SS')
                        cmd=15;items=dict(Time=stamp)
                    else: raise ValueError('未知维护操作')
                    if APP.link: result=APP.link.command(1101,cmd,items,'维护: '+action)
                    else: result=dict(ok=True,delivery='simulated',message='演示维护请求已校验')
                    APP.add_log('maintenance',action)
                elif p=='/api/settings':
                    for key in ['sdk','train','nos','gos']:
                        val=str(req.get(key,'')).strip()
                        u=urlparse(val)
                        if val and (u.scheme not in ['http','https'] or not u.hostname or u.username or u.password or u.query or u.fragment):
                            raise ValueError('代理地址必须为 http(s) 地址，不能包含凭据或参数')
                        APP.config[key]=val.rstrip('/')
                    APP.store.value['settings']=dict(APP.config);APP.store.save()
                    result=dict(ok=True)
                elif p=='/api/faults/inject':
                    if not APP.demo: raise ValueError('故障注入仅在演示模式可用')
                    error=explain_error(integer(req.get('code',32769)),integer(req.get('component',1)))
                    APP.faults.append(error);APP.events.insert(0,dict(error,at=time.time()));result=dict(ok=True)
                elif p=='/api/faults/clear-demo':
                    if not APP.demo: raise ValueError('实机错误需按厂商流程解决，不能在页面清除')
                    APP.faults=[];result=dict(ok=True)
                elif p=='/api/ppe/start':
                    if APP.demo: raise ValueError('PPE 检测需要真机视频、OpenCV 和已配置的 YOLO 模型')
                    cam=req.get('camera','front')
                    if cam not in original.CAM: raise ValueError('相机必须为 front 或 rear')
                    ok,msg=original.PPE.start(cam);result=dict(ok=ok,message=msg,err=msg if not ok else None)
                elif p=='/api/ppe/stop':
                    if original.PPE: original.PPE.stop()
                    result=dict(ok=True)
                else: return self._json(dict(ok=False,err='接口不存在'),404)
            self._json(result)
        except PermissionError as exc:self._json(dict(ok=False,err=str(exc)),403)
        except (ValueError,TypeError,KeyError) as exc: self._json(dict(ok=False,err=str(exc),navigation=error_summary(exc)),400)
        except (BrokenPipeError,ConnectionResetError): pass
        except Exception as exc:
            APP.add_log('error','服务错误: '+type(exc).__name__+exc_location(exc))
            self._json(dict(ok=False,err='服务错误，请查看诊断日志'),500)


def main():
    global APP,TOKEN,ACCESS
    ap=argparse.ArgumentParser(description='M20 Pro 可视化操作平台')
    ap.add_argument('--ip',help='显式连接机器人；不传则为演示模式')
    ap.add_argument('--port',type=int,default=30000)
    ap.add_argument('--http',type=int,default=8080)
    ap.add_argument('--bind',default='127.0.0.1')
    ap.add_argument('--token',default=os.environ.get('M20_API_TOKEN'))
    ap.add_argument('--share',metavar='PRIVATE_CIDR',help='共享到指定私人网段，例如 172.20.10.0/28；无需访问口令')
    ap.add_argument('--demo',action='store_true')
    ap.add_argument('--data-dir',help='独立工作区数据目录，便于离线测试')
    ap.add_argument('--ppe-model',default=os.environ.get('M20_PPE_MODEL'))
    args=ap.parse_args()
    TOKEN=args.token
    if args.bind not in ['127.0.0.1','localhost','::1'] and not TOKEN and not args.share:
        ap.error('开放局域网访问需通过 M20_API_TOKEN 配置令牌')
    if args.ppe_model: original.PPE_MODEL_PATH=args.ppe_model
    store=Store(Path(args.data_dir)/'workspace.json') if args.data_dir else Store()
    from m20.platform_host import lock_workspace
    try:instance_lock=lock_workspace(store.path.parent)
    except ValueError as exc:ap.error(str(exc))
    APP=Platform(store)
    if args.share:
        try:
            ACCESS,args.bind,share_notice=shared_listener(APP.store.path.parent,args.share,args.http,args.token)
        except ValueError as exc:ap.error(str(exc))
        if share_notice:APP.add_log('network',share_notice);print(share_notice)
    if args.ip and not args.demo: APP.connect(dict(mode='live',ip=args.ip,port=args.port))
    elif not args.demo and APP.hub.connections.value.get('auto_connect'):
        saved=APP.hub.connections.value
        APP.connect(dict(mode='live',ip=saved['aos_ip'],port=saved['aos_port']))
    def loop():
        last=time.monotonic()
        while not APP.done:
            now=time.monotonic()
            try:
                APP.tick(min(now-last,.1))
            except Exception as exc: APP.add_log('error','状态处理错误: '+type(exc).__name__)
            last=now
            time.sleep(.05)
    threading.Thread(target=loop,daemon=True).start()
    threading.Thread(target=APP.bridge_poll,daemon=True).start()
    threading.Thread(target=APP.hub.loop,daemon=True).start()
    server=ThreadingHTTPServer((args.bind,args.http),Handler)
    server.daemon_threads=True
    print('M20 Pro 操作平台: http://%s:%s/  [%s]'%(args.bind,args.http,'演示' if APP.demo else '实机'))
    if ACCESS:print('私人网络共享: '+', '.join(ACCESS.urls)+'；同一 Wi-Fi 直接打开，无需口令')
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        preserve_resident=APP.resident_navigation.preserve()
        APP.done=True
        APP.resident_navigation.close()
        if not preserve_resident:APP.stop()
        APP.hub.close()
        APP.autonomy.vision.stop()
        APP.map_learning.close()
        APP.people.stop()
        APP.cameras.close()
        if APP.link: APP.link.close(stop=not preserve_resident)
        if original.PPE: original.PPE.stop()
        server.server_close()
        instance_lock.close()


if __name__=='__main__': main()

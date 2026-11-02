"""Finite, cancellable greeting sessions sharing the platform control owner.

No lock is held while waiting. LED-only sessions never send axes or gait/mode
commands. Body actions are experimental and never change gait to make them work.
"""
import math
import threading
import time
import uuid

import greeting
from platform_core import age_of

LIGHT_SECONDS = 4.
ZERO_SECONDS = .4
HZ = 20.


class GreetingCancelled(Exception):
    pass


class GreetingSession:
    def __init__(self, app):
        self.app=app;self.current=None
        self.state=dict(active=False,state='idle',message='尚未开始问候')

    def snapshot(self):
        with self.app.lock:return dict(self.state)

    def _owned(self, token):
        return (self.current is token and not token['cancel'].is_set()
                and self.app.link is token['link'] and self.app.demo==token['demo']
                and not self.app.done)

    def _ready(self, token):
        if not self._owned(token):raise GreetingCancelled('问候已取消或连接已变化')
        app=self.app;link=token['link']
        if token['demo']:return
        if not link:raise GreetingCancelled('机器人连接不可用')
        basic=link.record(1002,6)
        if age_of(basic)>2:raise GreetingCancelled('基础反馈已过期，问候已停止')
        if not token['motion']:return
        if (app.estopped or getattr(link,'inhibit',False) or app.sdk.get('armed') or app.sdk.get('vendor_running')
                or app.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit']
                or app.autonomy.active or app.mapping or app.hub.busy
                or app.nav.get('status') in ['running','cancelling'] or any(app.velocity)):
            raise GreetingCancelled('控制状态或控制权已变化，问候已停止')
        if app._reported_active_navigation():
            raise GreetingCancelled('机器人报告另有导航任务，问候已停止')
        motion=link.record(1002,4)
        ok,reason=greeting.precheck(basic,motion)
        if not ok:raise GreetingCancelled(reason)
        status=basic.get('items',{}).get('BasicStatus',{})
        before=token['locomotion']
        if status.get('Gait')!=before['gait'] or status.get('ControlUsageMode')!=before['mode']:
            raise GreetingCancelled('步态或使用模式已变化，问候已停止；保留当前状态，不覆盖手柄设置')

    def _zero(self, token):
        # Called under the platform lock before another operator action can
        # take over. Old sessions may never send cleanup to a new connection.
        link=token['link']
        if not token['demo'] and token['axes_sent'] and self.app.link is link:
            with link.drive_lock:
                link.set_velocity(0,0,0)
                if hasattr(link,'_zero_until'):link._zero_until=time.monotonic()+ZERO_SECONDS
            token['axes_sent']=False

    def _light_off(self, token):
        if (not token['demo'] and token['light_sent'] and self.app.link is token['link']):
            token['link'].command(greeting.LIGHT_TYPE,greeting.LIGHT_CMD,greeting.light_off_items(),'greet-light-off')
            token['light_sent']=False

    def cancel(self, reason='已停止问候'):
        with self.app.lock:
            token=self.current
            if token is None:return
            token['cancel'].set()
            # Complete cleanup before releasing ownership; the worker does not
            # later overwrite a new task, reconnect, gait or operator command.
            failures=[]
            for cleanup in (self._zero,self._light_off):
                try:cleanup(token)
                except Exception as exc:failures.append(str(exc))
            self.current=None;self.app._greet_busy=False;self.app._greet_demo=None
            self.state.update(active=False,state='cancelled',message=reason,cleanup_errors=failures)

    def start(self, style='breathe', motion=None):
        if style not in greeting.LIGHT_STYLES:raise ValueError('未知灯语预设')
        if motion not in [None,'nod','bob']:raise ValueError('未知问候动作')
        app=self.app
        with app.lock:
            if self.current is not None:raise ValueError('上一次问候还未结束，可先按停止')
            if motion:
                app.hub.normal_mode();app.hub.idle()
                if app.mapping or app.hub.busy:raise ValueError('请先结束建图或后台任务，再执行动作问候')
                if app.estopped or getattr(app.link,'inhibit',False):raise ValueError('请先解除急停并确认站稳')
                if app._reported_active_navigation():raise ValueError('机器人还有导航任务，请先结束后再问候')
            if not app.demo and app.link is None:raise ValueError('机器人未连接')
            link=app.link
            basic=link.record(1002,6) if link else {}
            if not app.demo:
                if age_of(basic)>2:raise ValueError('没有新鲜的基础状态，未开始问候')
                if motion:
                    ok,reason=greeting.precheck(basic,link.record(1002,4))
                    if not ok:raise ValueError('未开始动作问候：'+reason)
            token=dict(id=uuid.uuid4().hex,link=link,demo=app.demo,motion=motion,style=style,
                       cancel=threading.Event(),axes_sent=False,light_sent=False,
                       locomotion=greeting.locomotion_snapshot(basic.get('items',{})))
            self.current=token;app._greet_busy=bool(motion)
            self.state=dict(active=True,state='lighting',motion=motion,style=style,run_id=token['id'],
                            demo=app.demo,message='正在执行灯语问候'+('；随后尝试实验动作' if motion else ''))
            if motion:
                app.velocity=[0.,0.,0.];app.velocity_deadline=None
                if link:link.set_velocity(0,0,0)
        try:threading.Thread(target=self._run,args=(token,),daemon=True).start()
        except Exception:
            self.cancel('问候线程未启动');raise
        return dict(ok=True,delivery='simulated' if token['demo'] else 'accepted',run_id=token['id'],
                    message='问候请求已接受，可随时停止；'+('演示为合成姿态' if token['demo'] else '实际效果以机器人反馈为准'))

    def _wait(self, token, seconds):
        if token['cancel'].wait(max(0.,seconds)):raise GreetingCancelled('问候已取消')
        with self.app.lock:self._ready(token)

    def _send(self, token, type_, command, items, *, axes=False):
        with self.app.lock:
            self._ready(token)
            if token['demo']:return
            if axes:
                with token['link'].drive_lock:
                    self._ready(token)
                    # Mark before I/O, since a failed reply need not mean that
                    # no datagram reached the robot.
                    token['axes_sent']=token['axes_sent'] or any(items.values())
                    token['link'].command(type_,command,items,'greet-motion')
            else:
                token['light_sent']=bool(items.get('CustomMode'))
                token['link'].command(type_,command,items,'greet-light')

    def _run(self, token):
        state='completed';message='问候序列已结束；实际效果请核对机身反馈'
        try:
            self._send(token,greeting.LIGHT_TYPE,greeting.LIGHT_CMD,greeting.light_on_items(token['style']))
            until=time.monotonic()+LIGHT_SECONDS
            while time.monotonic()<until:self._wait(token,min(.1,until-time.monotonic()))
            if token['motion']:
                frames=greeting.nod_plan() if token['motion']=='nod' else greeting.bob_plan()
                began=time.monotonic()
                with self.app.lock:
                    self._ready(token);self.state.update(state='motion',message='正在尝试实验问候动作，可随时停止')
                    if token['demo']:self.app._greet_demo=(began+len(frames)/HZ,token['motion'],began)
                for k,items in enumerate(frames):
                    self._wait(token,began+k/HZ-time.monotonic())
                    if time.monotonic()-(began+k/HZ)>.15:
                        raise GreetingCancelled('问候发送节拍延迟，已停止，不追发积压动作')
                    self._send(token,greeting.AXIS_TYPE,greeting.AXIS_CMD,items,axes=True)
                for items in greeting.zero_plan(ZERO_SECONDS):
                    self._send(token,greeting.AXIS_TYPE,greeting.AXIS_CMD,items,axes=True)
                    self._wait(token,1/HZ)
        except GreetingCancelled as exc:state='cancelled';message=str(exc)
        except Exception as exc:state='failed';message='问候未完成：'+str(exc)
        finally:
            with self.app.lock:
                if self.current is token:
                    failures=[]
                    for cleanup in (self._zero,self._light_off):
                        try:cleanup(token)
                        except Exception as exc:failures.append(str(exc))
                    if failures:state='failed';message='问候已停止，收尾反馈异常：'+'；'.join(failures)
                    self.current=None;self.app._greet_busy=False;self.app._greet_demo=None
                    self.state.update(active=False,state=state,message=message,cleanup_errors=failures)
                    self.app.add_log('greet',message)

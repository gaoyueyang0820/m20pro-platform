"""Bounded preview policy and leases, shared by PC and the authenticated agent.

Constructing a registry starts no thread or camera. A preview lease owns only its
own subscription; closing a browser must never stop a separate vision task.
"""
from dataclasses import dataclass, field
import re
import threading
import time


PROTOCOL = 1
MAX_JPEG_BYTES = 48 * 1024  # Onboard JPEG only; base64 adds at most one third.
LEASE_SECONDS = 3.0
MAX_REMOTE_AGE = 6.0
PROFILES = {
    'standard': dict(label='标准预览',transport='direct_rtsp',fps=12,width=800,quality=65,max_streams=2,
                     upstream_reduced=False,description='机器人原码流；电脑向网页发送预览'),
    'economy': dict(label='电脑节流',transport='direct_rtsp',fps=2,width=480,quality=45,max_streams=1,
                    upstream_reduced=False,description='仅降低电脑到网页的图量；机器人上游码率不变'),
    'weak': dict(label='弱网预览',transport='onboard_jpeg',fps=2,width=480,quality=45,max_streams=1,
                 upstream_reduced=True,description='机载按需小图；单路低帧预览，不用于实时驾驶判断'),
    'snapshot': dict(label='按需快照',transport='onboard_jpeg',fps=0,width=480,quality=45,max_streams=1,
                     upstream_reduced=True,description='点击后仅取一张机载小图，不连续拉流'),
}


def preview_profile(value):
    if not isinstance(value,str) or value not in PROFILES:
        raise ValueError('视频档位必须为 standard、economy、weak 或 snapshot')
    return PROFILES[value]


def preview_camera(value):
    if value not in ('front','rear'):
        raise ValueError('相机必须为 front 或 rear')
    return value


def preview_consumer(value):
    if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z0-9_-]{16,96}',value):
        raise ValueError('预览会话标识无效，请重新打开画面')
    return value


def frame_after(value):
    if type(value) is not int or not -1<=value<=2**53-1:
        raise ValueError('视频帧序号无效')
    return value


def failure(code,message,retry_after=1.):
    return dict(ok=False,code=code,err=message,retry_after=round(max(.1,min(8.,retry_after)),3))


class PreviewDenied(ValueError):
    def __init__(self,code,message,retry_after=1.):
        super().__init__(message)
        self.result=failure(code,message,retry_after)


@dataclass(eq=False)
class PreviewLease:
    consumer: str
    camera: str
    profile: str
    resource: object
    deadline: float
    cancelled: threading.Event = field(default_factory=threading.Event)
    busy: bool = False
    last_start: float = float('-inf')
    sequence: int = 0
    remote_sequence: int = -1
    remote_stream: str = ''
    last_source: object = None


class PreviewLeases:
    """Finite, expiring demand with release tombstones for HTTP reordering.

    start(camera, profile) is local and non-blocking (e.g. feed.subscribe).
    stop(lease, reason) runs outside the registry lock. It must not stop unrelated
    feed subscribers. Expiry is independent of any waiting frame request.
    """
    def __init__(self,start,stop,*,clock=time.monotonic,ttl=LEASE_SECONDS,start_worker=True):
        self.start=start; self.stop=stop; self.clock=clock; self.ttl=ttl
        self.lock=threading.RLock(); self.leases={}; self.tombstones={}
        self.done=threading.Event(); self.worker=None; self.start_worker=start_worker
        self.closed=False

    def _drop_locked(self,consumer):
        lease=self.leases.pop(consumer,None)
        if lease is not None:lease.cancelled.set()
        return lease

    def _tombstone_locked(self,consumer,now):
        self.tombstones={key:until for key,until in self.tombstones.items() if until>now}
        self.tombstones[consumer]=now+30.
        while len(self.tombstones)>256:
            self.tombstones.pop(next(iter(self.tombstones)))

    def expire(self):
        now=self.clock()
        with self.lock:
            expired=[self._drop_locked(key) for key,lease in list(self.leases.items()) if lease.deadline<=now]
            # Expiry also invalidates an in-flight request. A delayed completion
            # cannot create fresh demand; a new UI opening uses a new consumer.
            for lease in expired:self._tombstone_locked(lease.consumer,now)
        for lease in expired:self.stop(lease,'expired')
        return len(expired)

    def _reap(self):
        while not self.done.wait(.2):
            self.expire()

    def begin(self,camera,profile,consumer,*,timeout=5.):
        preview_camera(camera); spec=preview_profile(profile); preview_consumer(consumer)
        self.expire()
        with self.lock:
            now=self.clock()
            if self.closed or self.tombstones.get(consumer,0)>now:
                raise PreviewDenied('cancelled','此预览已关闭，请重新打开画面',.1)
            lease=self.leases.get(consumer)
            if lease is not None and (lease.camera!=camera or lease.profile!=profile):
                raise PreviewDenied('session_changed','切换视角或档位后请使用新的预览会话',.1)
            if lease is None:
                if len(self.leases)>=16:
                    raise PreviewDenied('viewer_limit','预览会话已满，请关闭其他预览后重试',1.)
                active=list(self.leases.values())
                channels={(PROFILES[item.profile]['transport'],item.camera) for item in active}
                channels.add((spec['transport'],camera))
                limit=min([spec['max_streams']]+[PROFILES[item.profile]['max_streams'] for item in active])
                if len(channels)>limit:
                    raise PreviewDenied('stream_limit','当前视频档位仅允许单路预览，请先关闭其他视角',1.)
                resource=self.start(camera,profile)
                lease=PreviewLease(consumer,camera,profile,resource,now+self.ttl)
                self.leases[consumer]=lease
                if self.worker is None and self.start_worker:
                    self.worker=threading.Thread(target=self._reap,name='video-preview-leases',daemon=True)
                    self.worker.start()
            if lease.busy:
                raise PreviewDenied('busy','上一张预览尚未完成，请稍候',.25)
            period=1./spec['fps'] if spec['fps'] else 1.
            remaining=period-(now-lease.last_start)
            if remaining>0:
                raise PreviewDenied('rate_limited','等待下一张预览',remaining)
            lease.busy=True; lease.last_start=now
            lease.deadline=now+max(self.ttl,timeout+.5)
            return lease

    def finish(self,lease):
        with self.lock:
            if self.leases.get(lease.consumer) is lease and not lease.cancelled.is_set():
                lease.busy=False; lease.deadline=self.clock()+self.ttl

    def release(self,consumer,*,reason='released'):
        preview_consumer(consumer)
        with self.lock:
            self._tombstone_locked(consumer,self.clock())
            lease=self._drop_locked(consumer)
        if lease is not None:self.stop(lease,reason)
        return dict(ok=True,released=lease is not None)

    def reset(self,*,reason='reset'):
        with self.lock:
            now=self.clock()
            leases=[self._drop_locked(key) for key in list(self.leases)]
            for lease in leases:self._tombstone_locked(lease.consumer,now)
        for lease in leases:self.stop(lease,reason)

    def status(self):
        # No camera open, capability probe, network I/O, or renewal here.
        with self.lock:
            rows=[dict(camera=lease.camera,profile=lease.profile,busy=lease.busy,
                       expires_in=round(max(0.,lease.deadline-self.clock()),2))
                  for lease in self.leases.values()]
        return dict(leases=len(rows),active=rows,lease_seconds=self.ttl)

    def close(self):
        with self.lock:self.closed=True
        self.done.set(); self.reset(reason='closed')
        if self.worker and self.worker is not threading.current_thread():self.worker.join(timeout=.5)

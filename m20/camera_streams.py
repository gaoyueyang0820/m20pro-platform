"""One demand-driven RTSP decoder per camera, with latest-frame-only delivery.

JPEG sizing/encoding here changes PC output, not the upstream RTSP bitrate.
Independent vision/task subscriptions are not owned by a browser preview.
"""
from collections import deque
import os
import re
import threading
import time
from m20.ffmpeg_capture import FFmpegCapture, find_ffmpeg


# Do not force low_delay/nobuffer: neither proves an intact HEVC reference chain.
CAPTURE_OPTIONS = 'rtsp_transport;tcp|reorder_queue_size;0'
MAX_FRAME_AGE = 1.0
MAX_KEYFRAME_WAIT = 3.0
IDLE_TIMEOUT = 1.0
RETRY_MAX = 8.0
HEALTHY_RESET_SECONDS = 2.0


def capture_params(cv):
    params = [cv.CAP_PROP_OPEN_TIMEOUT_MSEC, 3500, cv.CAP_PROP_READ_TIMEOUT_MSEC, 1200]
    if hasattr(cv, 'CAP_PROP_N_THREADS'):
        params += [cv.CAP_PROP_N_THREADS, 1]
    return params


def decoded_frame_type(cv, capture):
    """FFmpeg reports ASCII I/P/B; raw PPM and older backends do not."""
    prop = getattr(cv, 'CAP_PROP_FRAME_TYPE', None)
    if prop is None or not hasattr(capture, 'get'):
        return None
    try:
        value = int(capture.get(prop))
    except (TypeError, ValueError, OverflowError):
        return None
    return chr(value) if value in (73, 80, 66) else None


class CameraFeed:
    def __init__(self, decoder, ip, camera, port=8554, *, open_capture=None, encode=True,
                 idle_timeout=IDLE_TIMEOUT, jpeg_width=800, jpeg_quality=65,
                 max_fps=None, max_jpeg_bytes=None, capture_slot=None):
        if camera not in ('front', 'rear'):
            raise ValueError('相机必须为 front 或 rear')
        self.decoder=decoder; self.ip=ip; self.camera=camera; self.port=port
        self.open_capture=open_capture; self.encode=encode
        self.idle_timeout=max(0.,float(idle_timeout))
        self.jpeg_width=jpeg_width; self.jpeg_quality=jpeg_quality
        self.max_fps=max_fps; self.max_jpeg_bytes=max_jpeg_bytes; self.capture_slot=capture_slot
        self.condition=threading.Condition(); self.thread=None; self.closed=False
        self.viewers=0; self.last_view=time.monotonic(); self.jpeg=None; self.at=0
        self.frame_mono=0; self.sequence=0; self.raw_sequence=0; self.raw=None
        self.generation=0; self.epoch=0; self.reopen=False
        self.error=None; self.width=0; self.height=0; self.reconnects=0
        self.encode_ms=0.; self.open_ms=0.; self.skipped=0; self.frame_times=deque(maxlen=120)
        self.consecutive_failures=0; self.retry_at=0.; self.phase='idle'
        self.awaiting_keyframe=False; self.frame_type=None; self.frame_type_supported=None
        self.startup_frames_discarded=0; self.keyframe_wait_at=0.; self.capture_backend='opencv'

    def _start_locked(self):
        if self.thread is None and self.viewers and not self.closed:
            self.generation+=1; self.epoch+=1; self.raw=None; self.jpeg=None
            self.thread=threading.Thread(target=self._run,args=(self.generation,),daemon=True)
            self.thread.start()

    def subscribe(self):
        with self.condition:
            if self.closed:
                raise ValueError('相机连接已关闭，请重试')
            if self.decoder is None:
                raise ValueError('当前 Python 环境缺少 OpenCV，相机解码尚未就绪')
            self.viewers+=1; self.last_view=time.monotonic()
            self._start_locked(); self.condition.notify_all()

    def unsubscribe(self, *, immediate=False):
        with self.condition:
            self.viewers=max(0,self.viewers-1)
            self.last_view=time.monotonic()-(self.idle_timeout+.001 if immediate and not self.viewers else 0)
            self.condition.notify_all()

    def _inactive(self, generation):
        with self.condition:
            return (self.closed or generation!=self.generation or
                    not self.viewers and time.monotonic()-self.last_view>=self.idle_timeout)

    def read_packet(self, after=-1, timeout=1, *, cancelled=None):
        deadline=time.monotonic()+max(0.,timeout)
        with self.condition:
            while not self.closed and not (cancelled and cancelled()):
                age=time.monotonic()-self.frame_mono
                if self.jpeg and self.sequence!=after and age<MAX_FRAME_AGE:
                    return self.sequence,self.jpeg,age
                remaining=deadline-time.monotonic()
                if remaining<=0:
                    return None
                self.condition.wait(min(remaining,.1) if cancelled else remaining)
            return None

    def read(self, after=-1, timeout=1):
        packet=self.read_packet(after,timeout)
        return packet[:2] if packet else None

    def read_raw(self, after, timeout):
        """Read a new, recent frame with the decoder's receive timestamp."""
        with self.condition:
            def ready():
                return self.raw is not None and self.raw[0]>after and time.monotonic()-self.raw[3]<MAX_FRAME_AGE
            self.condition.wait_for(lambda:self.closed or ready(),timeout)
            return self.raw[:4] if not self.closed and ready() else None

    def status(self):
        with self.condition:
            now=time.monotonic(); age=now-self.frame_mono if self.at else None
            fresh=bool(self.jpeg and age<MAX_FRAME_AGE and not self.closed)
            times=[t for t in self.frame_times if now-t<2]
            fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
            integrity=('keyframe_observed' if self.frame_type_supported and not self.awaiting_keyframe else
                       'decoder_checks_only' if self.capture_backend=='ffmpeg-strict' else 'unknown')
            return dict(fresh=fresh,age=round(age,3) if age is not None else None,frames=self.sequence,
                        width=self.width,height=self.height,viewers=self.viewers,error=self.error,
                        reconnects=self.reconnects,encode_ms=round(self.encode_ms,1),open_ms=round(self.open_ms,1),
                        fps=round(fps,1),skipped=self.skipped,phase=self.phase,capture_backend=self.capture_backend,
                        awaiting_keyframe=self.awaiting_keyframe,frame_type=self.frame_type,
                        frame_type_supported=self.frame_type_supported,startup_frames_discarded=self.startup_frames_discarded,
                        frame_integrity=integrity,source_bitrate_changed=False,idle_timeout=self.idle_timeout,
                        keyframe_wait_ms=round((now-self.keyframe_wait_at)*1000) if self.awaiting_keyframe and self.keyframe_wait_at else 0,
                        retry_in=round(max(0.,self.retry_at-now),1),decoder_threads=1 if hasattr(self.decoder,'CAP_PROP_N_THREADS') else None,
                        state='live' if fresh else 'idle' if self.phase=='idle' else 'waiting_keyframe' if self.phase=='waiting_keyframe' else 'error' if self.error else 'connecting')

    def reconnect(self):
        with self.condition:
            self.reopen=True; self.epoch+=1; self.raw=None; self.jpeg=None
            self.error=None; self.consecutive_failures=0; self.retry_at=0.
            self.frame_times.clear(); self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed=True; self.raw=None; self.jpeg=None; self.condition.notify_all()

    def _encode(self, generation, done):
        cv=self.decoder; last=0; next_encode=0.
        while not done.is_set():
            with self.condition:
                if self.closed or generation!=self.generation:
                    return
                remaining=next_encode-time.monotonic()
                if self.raw is None or self.raw[0]==last or remaining>0:
                    self.condition.wait(min(.25,remaining) if remaining>0 else .25)
                    continue
                seq,frame,wall,mono,epoch=self.raw
                if last:
                    self.skipped+=max(0,seq-last-1)
                last=seq
                if self.max_fps:
                    next_encode=time.monotonic()+1./self.max_fps
            try:
                h,w=frame.shape[:2]
                if w>self.jpeg_width:
                    frame=cv.resize(frame,(self.jpeg_width,max(1,int(h*self.jpeg_width/w))))
                began=time.monotonic()
                qualities=[self.jpeg_quality]
                if self.max_jpeg_bytes:
                    qualities+=sorted({min(self.jpeg_quality,35),min(self.jpeg_quality,25)},reverse=True)
                payload=None
                for quality in qualities:
                    ok,encoded=cv.imencode('.jpg',frame,[cv.IMWRITE_JPEG_QUALITY,quality])
                    if not ok:
                        raise ValueError('相机图像转换失败，正在等待下一帧')
                    payload=encoded.tobytes()
                    if not self.max_jpeg_bytes or len(payload)<=self.max_jpeg_bytes:
                        break
                if self.max_jpeg_bytes and len(payload)>self.max_jpeg_bytes:
                    raise ValueError('预览图片超过流量上限，请改用按需快照或检查相机图像')
                with self.condition:
                    if done.is_set() or self.closed or generation!=self.generation or epoch!=self.epoch:
                        continue
                    if time.monotonic()-mono>=MAX_FRAME_AGE:
                        continue
                    self.jpeg=payload; self.at=wall; self.frame_mono=mono; self.sequence=seq
                    self.encode_ms=(time.monotonic()-began)*1000
                    self.width=frame.shape[1]; self.height=frame.shape[0]; self.error=None
                    self.frame_times.append(mono); self.condition.notify_all()
            except Exception:
                with self.condition:
                    self.error='相机图像转换失败或超过预览大小限制，正在等待下一帧'
                    self.condition.notify_all()

    def _run(self, generation):
        cv=self.decoder; cap=None; done=threading.Event(); slot_held=False; healthy_since=None
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS']=CAPTURE_OPTIONS
        url='rtsp://%s:%s/%s'%(self.ip,self.port,'video1' if self.camera=='front' else 'video2')
        binary=find_ffmpeg() if self.open_capture is None and getattr(cv,'__name__',None)=='cv2' else None
        self.capture_backend='ffmpeg-strict' if binary else 'opencv'
        if self.encode:
            threading.Thread(target=self._encode,args=(generation,done),daemon=True).start()
        try:
            while not self._inactive(generation):
                if self.capture_slot is not None and not slot_held:
                    slot_held=self.capture_slot.acquire(timeout=.1)
                    if not slot_held:
                        with self.condition:self.phase='waiting_slot'
                        continue
                with self.condition:
                    reopen=self.reopen; self.reopen=False; epoch=self.epoch
                try:
                    if reopen and cap is not None:
                        cap.release(); cap=None
                    if cap is None:
                        opened=time.monotonic(); healthy_since=None
                        with self.condition:
                            self.phase='opening'; self.retry_at=0.; self.awaiting_keyframe=True
                            self.frame_type=None; self.frame_type_supported=None
                        if self.open_capture:
                            cap=self.open_capture()
                        else:
                            cap=(FFmpegCapture(cv,url,binary,cancelled=lambda:self._inactive(generation)) if binary else
                                 cv.VideoCapture(url,cv.CAP_FFMPEG,capture_params(cv)))
                        self.open_ms=(time.monotonic()-opened)*1000
                        if self._inactive(generation):
                            return
                        if not cap.isOpened():
                            view,path=('前视','video1') if self.camera=='front' else ('后视','video2')
                            # Raw decoder logs can contain credentials; never expose them.
                            if re.search(r'\b404\b[^\r\n]*\bnot found\b',str(getattr(cap,'error','')),re.I):
                                raise ValueError('%s RTSP /%s 返回 404：机器人视频服务当前未提供这路视频流；平台会自动重试，请在机器人端检查%s相机与推流进程'%(view,path,view))
                            raise ValueError('%s RTSP /%s 无法打开；请核对相机推流服务与路径，平台将自动重试'%(view,path))
                        with self.condition:
                            self.keyframe_wait_at=time.monotonic()
                            if isinstance(cap,FFmpegCapture):self.capture_backend='ffmpeg-strict'
                    with self.condition:
                        if not self.awaiting_keyframe:self.phase='reading'
                    ok,frame=cap.read()
                    if self._inactive(generation):
                        return
                    if not ok or frame is None:
                        raise ValueError('相机视频暂时中断，正在重连')
                    frame_type=decoded_frame_type(cv,cap)
                    with self.condition:
                        if epoch!=self.epoch:
                            continue
                        self.frame_type=frame_type; self.frame_type_supported=frame_type is not None
                        if self.awaiting_keyframe and frame_type in ('P','B'):
                            self.startup_frames_discarded+=1; self.phase='waiting_keyframe'
                            if time.monotonic()-self.keyframe_wait_at>=MAX_KEYFRAME_WAIT:
                                raise ValueError('未取得完整视频帧，正在重新同步解码连接')
                            self.error='视频已连接，正在等待完整视频帧；请勿连续重连'
                            self.condition.notify_all(); continue
                        # PPM has no I/P/B metadata: strict decoding is not proof of
                        # a complete reference chain. Do not invent a keyframe flag.
                        self.awaiting_keyframe=False; self.phase='reading'
                        now=time.monotonic()
                        if healthy_since is None:healthy_since=now
                        if now-healthy_since>=HEALTHY_RESET_SECONDS:self.consecutive_failures=0
                        self.retry_at=0.; self.raw_sequence+=1
                        self.raw=(self.raw_sequence,frame,time.time(),getattr(cap,'last_frame_mono',None) or now,epoch)
                        self.condition.notify_all()
                except Exception as exc:
                    if cap is not None:
                        cap.release(); cap=None
                    if self._inactive(generation):
                        return
                    with self.condition:
                        if self.phase=='waiting_keyframe':self.phase='reading'
                        self.epoch+=1; self.raw=None; self.jpeg=None
                        # Only deliberately constructed errors above are user-facing.
                        self.error=str(exc) if isinstance(exc,ValueError) and str(exc).startswith(('前视 RTSP','后视 RTSP','相机视频','未取得完整')) else '相机解码连接失败，正在退避重试'
                        self.reconnects+=1; healthy_since=None; self.consecutive_failures+=1
                        delay=min(RETRY_MAX,.5*2**min(self.consecutive_failures-1,5))
                        self.retry_at=time.monotonic()+delay; self.condition.notify_all()
                        deadline=self.retry_at
                        # Re-evaluate demand, not just close/reconnect. A missing
                        # source must not keep opening after its last viewer left.
                        while not self._inactive(generation) and not self.reopen:
                            left=deadline-time.monotonic()
                            if left<=0:break
                            self.condition.wait(min(left,.1))
        finally:
            done.set()
            if cap is not None:cap.release()
            if slot_held:self.capture_slot.release()
            with self.condition:
                if self.thread is threading.current_thread():
                    self.thread=None; self.phase='idle'; self.raw=None; self.jpeg=None
                    self.awaiting_keyframe=False; self.retry_at=0.; self.frame_times.clear()
                    # subscribe() may race with a worker exiting its idle grace.
                    self._start_locked()
                self.condition.notify_all()


class Cameras:
    def __init__(self,decoder,target,port=8554,*,feed_factory=None,max_active=2):
        self.decoder=decoder; self.target=target; self.port=port
        self.lock=threading.RLock(); self.feeds={}; self.feed_factory=feed_factory
        self.capture_slots=threading.BoundedSemaphore(max_active)

    def _discard_other_targets(self,ip):
        for key,feed in list(self.feeds.items()):
            if key[0]!=ip:
                feed.close(); self.feeds.pop(key)

    def get(self,camera):
        if camera not in ['front','rear']:
            raise ValueError('相机必须为 front 或 rear')
        ip=self.target()
        with self.lock:
            self._discard_other_targets(ip)
            if not ip:
                raise ValueError('请先连接真实机器人')
            key=ip,camera
            if key not in self.feeds:
                self.feeds[key]=(self.feed_factory(self.decoder,ip,camera,self.port,self.capture_slots) if self.feed_factory else
                                 CameraFeed(self.decoder,ip,camera,self.port,capture_slot=self.capture_slots))
            return self.feeds[key]

    def status(self):
        ip=self.target()
        with self.lock:
            self._discard_other_targets(ip)
            result={camera:self.feeds[(ip,camera)].status() if (ip,camera) in self.feeds else
                    dict(fresh=False,age=None,frames=0,state='idle',error=None,width=0,height=0,viewers=0,fps=0)
                    for camera in ['front','rear']}
        return dict(available=bool(self.decoder),**result)

    def close(self):
        with self.lock:
            for feed in self.feeds.values():feed.close()
            self.feeds.clear()

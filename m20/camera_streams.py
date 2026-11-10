"""One RTSP decoder per camera; capture and JPEG delivery never queue old frames."""
from collections import deque
import os
import re
import threading
import time
from m20.ffmpeg_capture import FFmpegCapture,find_ffmpeg


# Do not force AV_CODEC_FLAG_LOW_DELAY on the factory HEVC stream. In particular,
# it does not make a multi-frame decoder queue disappear. Use one decode thread.
CAPTURE_OPTIONS = 'rtsp_transport;tcp|reorder_queue_size;0'
MAX_FRAME_AGE = 1.0
MAX_KEYFRAME_WAIT = 3.0


def capture_params(cv):
    params = [cv.CAP_PROP_OPEN_TIMEOUT_MSEC, 3500, cv.CAP_PROP_READ_TIMEOUT_MSEC, 1200]
    if hasattr(cv, 'CAP_PROP_N_THREADS'):
        params += [cv.CAP_PROP_N_THREADS, 1]
    return params


def decoded_frame_type(cv, capture):
    """FFmpeg reports ASCII I/P/B; older backends may not support this property."""
    prop=getattr(cv,'CAP_PROP_FRAME_TYPE',None)
    if prop is None or not hasattr(capture,'get'):return None
    try:value=int(capture.get(prop))
    except (TypeError,ValueError,OverflowError):return None
    return chr(value) if value in (73,80,66) else None


class CameraFeed:
    def __init__(self, decoder, ip, camera, port=8554, *, open_capture=None, encode=True):
        self.decoder=decoder; self.ip=ip; self.camera=camera; self.port=port
        self.open_capture = open_capture
        self.encode = encode
        self.condition=threading.Condition(); self.thread=None; self.closed=False
        self.viewers=0; self.last_view=time.monotonic(); self.jpeg=None; self.at=0
        self.frame_mono=0; self.sequence=0; self.raw_sequence=0; self.raw=None
        self.generation=0; self.epoch=0; self.reopen=False
        self.error=None; self.width=0; self.height=0; self.reconnects=0
        self.encode_ms=0.; self.open_ms=0.; self.skipped=0; self.frame_times=deque(maxlen=120)
        self.consecutive_failures=0; self.retry_at=0.; self.phase='idle'
        self.awaiting_keyframe=False; self.frame_type=None; self.frame_type_supported=None
        self.startup_frames_discarded=0; self.keyframe_wait_at=0.; self.capture_backend='opencv'

    def subscribe(self):
        with self.condition:
            if self.closed: raise ValueError('相机连接已关闭，请重试')
            if self.decoder is None: raise ValueError('当前 Python 环境缺少 OpenCV，相机解码尚未就绪')
            self.viewers+=1; self.last_view=time.monotonic()
            if self.thread is None:
                self.generation+=1
                self.thread=threading.Thread(target=self._run,args=(self.generation,),daemon=True)
                self.thread.start()

    def unsubscribe(self):
        with self.condition:
            self.viewers=max(0,self.viewers-1); self.last_view=time.monotonic()

    def read_packet(self, after=-1, timeout=1):
        deadline=time.monotonic()+timeout
        with self.condition:
            while not self.closed:
                age=time.monotonic()-self.frame_mono
                if self.jpeg and self.sequence!=after and age<MAX_FRAME_AGE:
                    return self.sequence,self.jpeg,age
                remaining=deadline-time.monotonic()
                if remaining<=0: return None
                self.condition.wait(remaining)
            return None

    def read(self, after=-1, timeout=1):
        packet=self.read_packet(after,timeout)
        return packet[:2] if packet else None

    def read_raw(self, after, timeout):
        """Read a new, recent frame with the decoder's capture timestamp."""
        with self.condition:
            def ready():
                return self.raw is not None and self.raw[0] > after and time.monotonic()-self.raw[3] < MAX_FRAME_AGE
            self.condition.wait_for(lambda: self.closed or ready(), timeout)
            return self.raw[:4] if not self.closed and ready() else None

    def status(self):
        with self.condition:
            now=time.monotonic(); age=now-self.frame_mono if self.at else None
            fresh=bool(self.jpeg and age<MAX_FRAME_AGE and not self.closed)
            times=[t for t in self.frame_times if now-t<2]
            fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
            return dict(fresh=fresh,age=round(age,3) if age is not None else None,frames=self.sequence,
                        width=self.width,height=self.height,viewers=self.viewers,error=self.error,
                        reconnects=self.reconnects,encode_ms=round(self.encode_ms,1),open_ms=round(self.open_ms,1),
                        fps=round(fps,1),skipped=self.skipped,phase=self.phase,capture_backend=self.capture_backend,
                        awaiting_keyframe=self.awaiting_keyframe,frame_type=self.frame_type,
                        frame_type_supported=self.frame_type_supported,startup_frames_discarded=self.startup_frames_discarded,
                        keyframe_wait_ms=round((now-self.keyframe_wait_at)*1000) if self.awaiting_keyframe and self.keyframe_wait_at else 0,
                        retry_in=round(max(0.,self.retry_at-now),1),decoder_threads=1 if hasattr(self.decoder,'CAP_PROP_N_THREADS') else None,
                        state='live' if fresh else 'waiting_keyframe' if self.phase=='waiting_keyframe' else 'error' if self.error else 'connecting' if self.thread else 'idle')

    def reconnect(self):
        with self.condition:
            self.reopen=True; self.epoch+=1; self.raw=None; self.jpeg=None
            self.error=None; self.consecutive_failures=0; self.retry_at=0.
            self.frame_times.clear(); self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed=True; self.condition.notify_all()

    def _encode(self, generation, done):
        cv=self.decoder; last=0
        while not done.is_set():
            with self.condition:
                if self.closed or generation!=self.generation: return
                if self.raw is None or self.raw[0]==last:
                    self.condition.wait(.25); continue
                seq,frame,wall,mono,epoch=self.raw
                if last: self.skipped+=max(0,seq-last-1)
                last=seq
            try:
                h,w=frame.shape[:2]
                if w>800: frame=cv.resize(frame,(800,int(h*800/w)))
                began=time.monotonic()
                ok,encoded=cv.imencode('.jpg',frame,[cv.IMWRITE_JPEG_QUALITY,65])
                if not ok: raise ValueError('相机图像转换失败，正在等待下一帧')
                with self.condition:
                    if self.closed or generation!=self.generation or epoch!=self.epoch: continue
                    if time.monotonic()-mono>=MAX_FRAME_AGE: continue
                    self.jpeg=encoded.tobytes(); self.at=wall; self.frame_mono=mono; self.sequence=seq
                    self.encode_ms=(time.monotonic()-began)*1000
                    self.width=frame.shape[1]; self.height=frame.shape[0]; self.error=None
                    self.frame_times.append(mono); self.condition.notify_all()
            except Exception as exc:
                with self.condition: self.error=str(exc); self.condition.notify_all()

    def _run(self, generation):
        cv=self.decoder; cap=None; done=threading.Event()
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS']=CAPTURE_OPTIONS
        url='rtsp://%s:%s/%s'%(self.ip,self.port,'video1' if self.camera=='front' else 'video2')
        binary=find_ffmpeg() if self.open_capture is None and getattr(cv,'__name__',None)=='cv2' else None
        self.capture_backend='ffmpeg-strict' if binary else 'opencv'
        if self.encode:
            threading.Thread(target=self._encode,args=(generation,done),daemon=True).start()
        try:
            while True:
                with self.condition:
                    if self.closed or generation!=self.generation: return
                    if not self.viewers and time.monotonic()-self.last_view>10:
                        self.thread=None; self.phase='idle'; return
                    reopen=self.reopen; self.reopen=False
                    epoch=self.epoch
                try:
                    if reopen and cap is not None: cap.release(); cap=None
                    if cap is None:
                        opened=time.monotonic()
                        with self.condition:
                            self.phase='opening'; self.retry_at=0.; self.awaiting_keyframe=True
                            self.frame_type=None; self.frame_type_supported=None
                        if self.open_capture:
                            cap=self.open_capture()
                        else:
                            cap=FFmpegCapture(cv,url,binary) if binary else cv.VideoCapture(url,cv.CAP_FFMPEG,capture_params(cv))
                        self.open_ms=(time.monotonic()-opened)*1000
                        if not cap.isOpened():
                            view,path=('前视','video1') if self.camera=='front' else ('后视','video2')
                            # Keep a confirmed server rejection distinct from decode/open failures.
                            # Do not expose raw decoder logs, which may contain RTSP credentials.
                            if re.search(r'\b404\b[^\r\n]*\bnot found\b',str(getattr(cap,'error','')),re.I):
                                raise ValueError('%s RTSP /%s 返回 404：机器人视频服务当前未提供这路视频流；平台会自动重试，请在机器人端检查%s相机与推流进程'%(view,path,view))
                            raise ValueError('%s RTSP /%s 无法打开；请核对相机推流服务与路径，平台将自动重试'%(view,path))
                        with self.condition:self.keyframe_wait_at=time.monotonic()
                    with self.condition:
                        if not self.awaiting_keyframe:self.phase='reading'
                    ok,frame=cap.read()
                    if not ok or frame is None: raise ValueError('相机视频暂时中断，正在重连')
                    frame_type=decoded_frame_type(cv,cap)
                    with self.condition:
                        if epoch!=self.epoch: continue
                        self.frame_type=frame_type; self.frame_type_supported=frame_type is not None
                        if self.awaiting_keyframe and frame_type in ('P','B'):
                            self.startup_frames_discarded+=1; self.phase='waiting_keyframe'
                            if time.monotonic()-self.keyframe_wait_at>=MAX_KEYFRAME_WAIT:
                                raise ValueError('未取得完整视频帧，正在重新同步解码连接')
                            self.error='视频已连接，正在等待完整视频帧；请勿连续重连'
                            self.condition.notify_all(); continue
                        # read() can return concealed P/B images before the first
                        # intra frame. Never publish these as a fresh camera view.
                        # Unknown metadata retains compatibility with older decoders.
                        self.awaiting_keyframe=False; self.phase='reading'
                        self.consecutive_failures=0; self.retry_at=0.
                        self.raw_sequence+=1
                        self.raw=(self.raw_sequence,frame,time.time(),getattr(cap,'last_frame_mono',None) or time.monotonic(),epoch)
                        self.condition.notify_all()
                except Exception as exc:
                    if cap is not None: cap.release(); cap=None
                    with self.condition:
                        if self.phase=='waiting_keyframe':self.phase='reading'
                        self.epoch+=1; self.raw=None; self.error=str(exc); self.reconnects+=1
                        self.condition.notify_all()
                        self.consecutive_failures+=1
                        delay=min(4.,.5*2**min(self.consecutive_failures-1,3))
                        self.retry_at=time.monotonic()+delay
                        # A missing stream can fail immediately (e.g. RTSP 404).
                        # Back off without blocking status reads; manual retry wakes us.
                        self.condition.wait_for(lambda:self.closed or self.reopen,timeout=delay)
        finally:
            done.set()
            with self.condition:
                self.condition.notify_all()
                if self.thread is threading.current_thread(): self.thread=None
            if cap is not None: cap.release()


class Cameras:
    def __init__(self,decoder,target,port=8554):
        self.decoder=decoder; self.target=target; self.port=port
        self.lock=threading.RLock(); self.feeds={}

    def get(self,camera):
        if camera not in ['front','rear']: raise ValueError('相机必须为 front 或 rear')
        ip=self.target()
        if not ip: raise ValueError('请先连接真实机器人')
        with self.lock:
            for key,feed in list(self.feeds.items()):
                if key[0]!=ip: feed.close(); self.feeds.pop(key)
            key=ip,camera
            if key not in self.feeds: self.feeds[key]=CameraFeed(self.decoder,ip,camera,self.port)
            return self.feeds[key]

    def status(self):
        ip=self.target()
        with self.lock:
            result={camera:self.feeds[(ip,camera)].status() if (ip,camera) in self.feeds else
                    dict(fresh=False,age=None,frames=0,state='idle',error=None,width=0,height=0,viewers=0,fps=0)
                    for camera in ['front','rear']}
        return dict(available=bool(self.decoder),**result)

    def close(self):
        with self.lock:
            for feed in self.feeds.values(): feed.close()
            self.feeds.clear()

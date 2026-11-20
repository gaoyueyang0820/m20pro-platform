"""On-demand small JPEGs inside the existing authenticated GOS agent.

No listener, daemon, camera power control, or vendor-service modification. The
configured internal AOS is read-only RTSP input; callers cannot supply a URL.
Full-rate RTSP stays on the robot's internal link. Only bounded JPEG responses
cross the PC link, and only while a preview consumer has demand.
"""
import base64
import ipaddress
import threading
import uuid

from m20.camera_streams import CameraFeed, Cameras
from m20.ffmpeg_capture import FFmpegCapture, ffmpeg_major, find_ffmpeg
from m20.video_preview import (MAX_JPEG_BYTES, PROFILES, PROTOCOL, PreviewDenied,
                               PreviewLeases, failure, frame_after, preview_camera,
                               preview_consumer, preview_profile)


_PRIVATE_V4 = tuple(ipaddress.ip_network(value) for value in
                    ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16'))


def internal_aos(value):
    """Accept only an explicit RFC1918 IPv4 address, never a hostname or URL."""
    if not isinstance(value,str) or value!=value.strip():
        raise ValueError('机载预览未配置有效的 AOS 内网地址')
    try:
        address=ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError:
        raise ValueError('机载预览未配置有效的 AOS 内网地址') from None
    if not any(address in network for network in _PRIVATE_V4):
        raise ValueError('机载预览只接受已配置的 AOS 私有内网地址')
    return str(address)


class VideoPreviewAgent:
    def __init__(self,aos=None,*,decoder=None,open_capture=None,start_worker=True):
        self.aos=aos; self.decoder=decoder; self.open_capture=open_capture
        self.lock=threading.RLock(); self.pool=None; self.source=None
        self.stream=uuid.uuid4().hex; self.binary=None; self.legacy_version=None
        self.error=None; self.closed=False
        self.leases=PreviewLeases(self._start,self._stop,start_worker=start_worker)

    def _address(self):
        return internal_aos(self.aos() if callable(self.aos) else self.aos)

    def _prepare_locked(self):
        address=self._address()
        if self.closed:
            raise ValueError('机载预览服务已关闭')
        if self.source!=address:
            self.leases.reset()
            if self.pool:self.pool.close()
            self.pool=None; self.source=address; self.stream=uuid.uuid4().hex
        if self.pool is not None:
            return
        if self.decoder is None:
            try:
                import cv2
            except ImportError:
                raise ValueError('机载预览缺少 OpenCV，请由连接中心更新视频运行环境') from None
            self.decoder=cv2
        if self.open_capture is None:
            self.binary=find_ffmpeg() or find_ffmpeg(4)
            if not self.binary:
                raise ValueError('机载预览缺少 FFmpeg 4+，请检查 GOS 视频运行环境')
            major=ffmpeg_major(self.binary)
            self.legacy_version=major if major<6 else None
        self.pool=Cameras(self.decoder,lambda:self.source,feed_factory=self._feed,max_active=1)

    def _feed(self,decoder,address,camera,port,slot):
        spec=PROFILES['weak']
        feed=CameraFeed(decoder,address,camera,port,jpeg_width=spec['width'],
                        jpeg_quality=spec['quality'],max_fps=spec['fps'],
                        max_jpeg_bytes=MAX_JPEG_BYTES,capture_slot=slot)
        def open_stream():
            if self.open_capture is not None:
                return self.open_capture(camera)
            path='video1' if camera=='front' else 'video2'
            return FFmpegCapture(decoder,'rtsp://%s:%d/%s'%(address,port,path),self.binary,
                                 open_timeout=4.5,read_timeout=1.2,legacy_version=self.legacy_version,
                                 max_width=spec['width'],cancelled=lambda:feed._inactive(feed.generation))
        feed.open_capture=open_stream
        return feed

    def _start(self,camera,profile):
        feed=self.pool.get(camera); feed.subscribe()
        return feed

    @staticmethod
    def _stop(lease,reason):
        lease.resource.unsubscribe(immediate=True)

    def frame(self,request):
        if not isinstance(request,dict) or set(request)-{'camera','profile','consumer','after','stream'}:
            raise ValueError('预览参数无效；不接受相机地址或额外流参数')
        camera=preview_camera(request.get('camera','front'))
        profile=request.get('profile','weak'); preview_profile(profile)
        if profile not in ('weak','snapshot'):
            raise ValueError('机载预览仅支持 weak 或 snapshot')
        consumer=preview_consumer(request.get('consumer'))
        after=frame_after(request.get('after',-1))
        stream=request.get('stream','')
        if not isinstance(stream,str) or len(stream)>64:
            raise ValueError('视频流标识无效')
        lease=None
        try:
            with self.lock:
                self._prepare_locked()
                if stream!=self.stream:after=-1
                timeout=5. if after<0 else 1.2
                lease=self.leases.begin(camera,profile,consumer,timeout=timeout)
                generation=self.stream
            packet=lease.resource.read_packet(after,timeout,cancelled=lease.cancelled.is_set)
            if lease.cancelled.is_set():
                return failure('cancelled','预览已关闭',.1)
            if packet is None:
                status=lease.resource.status()
                return failure('waiting_frame',status.get('error') or '暂未取得新鲜相机预览',max(.5,status.get('retry_in',0)))
            sequence,jpeg,age=packet
            if len(jpeg)>MAX_JPEG_BYTES or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                return failure('invalid_frame','未取得完整且符合大小限制的预览图',1.)
            status=lease.resource.status()
            self.error=None
            return dict(ok=True,protocol=PROTOCOL,sequence=sequence,stream=generation,
                        jpeg_base64=base64.b64encode(jpeg).decode('ascii'),age=age,
                        age_basis='decode_receive',width=status['width'],height=status['height'],
                        profile=profile,transport='onboard_jpeg',upstream_reduced=True,
                        source_bitrate_changed=False,upstream_scope='this_preview_only')
        except PreviewDenied as exc:
            return exc.result
        except ValueError as exc:
            # Messages from configuration/dependency validation contain no URL,
            # token, subprocess output, or decoder log.
            self.error=str(exc)
            return failure('unavailable',self.error,4.)
        finally:
            if lease is not None:
                self.leases.finish(lease)
                if profile=='snapshot':self.leases.release(consumer)

    def status(self):
        try:
            self._address(); configured=True; error=self.error
        except ValueError as exc:
            configured=False; error=str(exc)
        # Purely local; no dependency subprocess, ROS, RTSP, or capability probe.
        return dict(ok=True,protocol=PROTOCOL,supported=True,configured=configured,
                    initialized=self.pool is not None,error=error,profiles=['weak','snapshot'],
                    max_streams=1,max_fps=2,max_width=480,max_jpeg_bytes=MAX_JPEG_BYTES,
                    source_bitrate_changed=False,age_basis='decode_receive',
                    demand=self.leases.status())

    def dispatch(self,path,request):
        if path=='/video/preview/status':return self.status()
        if path=='/video/preview/frame':return self.frame(request)
        if path=='/video/preview/release':
            if not isinstance(request,dict) or set(request)!={'consumer'}:
                raise ValueError('请提供需要关闭的预览会话')
            return self.leases.release(request['consumer'])
        raise ValueError('机载预览接口不存在')

    def close(self):
        with self.lock:
            self.closed=True; self.leases.close()
            if self.pool:self.pool.close()

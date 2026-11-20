"""PC preview helper; robot JPEGs use only the existing authenticated GOS RPC.

status/reset/construction do not contact the robot. Weak/snapshot never fall back
to direct RTSP. Preview leases do not own independent detection subscriptions.
"""
import base64
import binascii
import math
import threading
import time

from m20.camera_streams import MAX_FRAME_AGE
from m20.video_preview import (MAX_JPEG_BYTES, MAX_REMOTE_AGE, PROFILES, PROTOCOL,
                               PreviewDenied, PreviewLeases, failure, frame_after,
                               preview_camera, preview_consumer, preview_profile)


class VideoPreviewClient:
    def __init__(self,app,*,clock=time.monotonic,start_worker=True):
        self.app=app; self.clock=clock; self.lock=threading.RLock(); self.scope=None
        self.onboard=dict(state='unknown',error=None)
        self.leases=PreviewLeases(self._start,self._stop,clock=clock,start_worker=start_worker)

    def _start(self,camera,profile):
        if PROFILES[profile]['transport']=='onboard_jpeg':return None
        feed=self.app.cameras.get(camera); feed.subscribe()
        return feed

    def _stop(self,lease,reason):
        if lease.resource is not None:
            lease.resource.unsubscribe(immediate=True)
        elif reason=='released':
            # Best effort only. A failed release cannot prevent local revocation;
            # the robot has its own bounded lease expiry, including on PC loss.
            try:
                self.app.proxy('gos','/video/preview/release',dict(consumer=lease.consumer),timeout=1.)
            except Exception:
                pass

    def _sync_context_locked(self):
        cameras=self.app.cameras
        target=cameras.target()
        scope=(id(cameras),target)
        if scope!=self.scope:
            self.leases.reset(); self.scope=scope; self.onboard=dict(state='unknown',error=None)
        if not target or getattr(self.app,'demo',False):
            raise PreviewDenied('offline','请先连接真实机器人；演示模式没有实机视频',2.)

    def _direct_frame(self,lease,after,timeout):
        feed=lease.resource; profile=lease.profile; spec=PROFILES[profile]
        if profile=='standard':
            packet=feed.read_packet(after,timeout,cancelled=lease.cancelled.is_set)
            if packet is None:
                status=feed.status()
                return failure('waiting_frame',status.get('error') or '等待相机画面',max(.25,status.get('retry_in',0)))
            sequence,jpeg,age=packet
            status=feed.status(); width=status['width']; height=status['height']
        else:
            deadline=time.monotonic()+timeout; packet=None
            while not lease.cancelled.is_set():
                remaining=deadline-time.monotonic()
                if remaining<=0:break
                packet=feed.read_raw(after,min(.1,remaining))
                if packet is not None:break
            if packet is None:return failure('waiting_frame',feed.status().get('error') or '等待相机画面',.5)
            sequence,frame,_,captured=packet
            height,width=frame.shape[:2]; cv=self.app.cameras.decoder
            if width>spec['width']:
                frame=cv.resize(frame,(spec['width'],max(1,int(height*spec['width']/width))))
            ok,encoded=cv.imencode('.jpg',frame,[cv.IMWRITE_JPEG_QUALITY,spec['quality']])
            if not ok:return failure('invalid_frame','预览图像转换失败',1.)
            jpeg=encoded.tobytes(); age=time.monotonic()-captured
            height,width=frame.shape[:2]
            if age>=MAX_FRAME_AGE:return failure('stale_frame','相机图像已过期，等待新画面',.5)
        return dict(ok=True,jpeg=jpeg,sequence=sequence,age=age,profile=profile,
                    width=width,height=height,transport='direct_rtsp',upstream_reduced=False,
                    source_bitrate_changed=False,upstream_scope='pc_to_browser_only',age_basis='decode_receive')

    def _onboard_frame(self,lease,after,timeout):
        began=self.clock()
        request=dict(camera=lease.camera,profile=lease.profile,consumer=lease.consumer,
                     after=lease.remote_sequence,stream=lease.remote_stream)
        try:
            packet=self.app.proxy('gos','/video/preview/frame',request,timeout=timeout)
        except Exception:
            message='机载预览不可用，请检查 GOS 连接与视频代理；未改用原码率视频'
            self.onboard=dict(state='unavailable',error=message)
            return failure('unavailable',message,4.)
        elapsed=max(0.,self.clock()-began)
        if lease.cancelled.is_set():return failure('cancelled','预览已关闭',.1)
        if not isinstance(packet,dict):return failure('invalid_frame','机载预览返回格式不受支持',4.)
        if packet.get('ok') is not True:
            known={'unavailable','waiting_frame','stream_limit','viewer_limit','busy','rate_limited','cancelled','invalid_frame'}
            code=packet.get('code')
            if code not in known:
                code='unavailable'
                message='GOS 尚不支持机载预览，请更新代理；未改用原码率视频'
            else:
                message=str(packet.get('err') or '机载预览暂时不可用')[:400]
            retry=packet.get('retry_after',1.)
            if type(retry) not in (int,float) or not math.isfinite(retry):retry=1.
            if code=='unavailable':self.onboard=dict(state='unavailable',error=message)
            return failure(code,message,retry)
        try:
            if type(packet.get('protocol')) is not int or packet['protocol']!=PROTOCOL:raise ValueError()
            if packet.get('transport')!='onboard_jpeg' or packet.get('profile')!=lease.profile:raise ValueError()
            if packet.get('age_basis')!='decode_receive':raise ValueError()
            age=packet.get('age')
            if type(age) not in (int,float) or not math.isfinite(age) or age<0:raise ValueError()
            sequence=frame_after(packet.get('sequence'))
            if sequence<0:raise ValueError()
            stream=packet.get('stream')
            if not isinstance(stream,str) or not 1<=len(stream)<=64:raise ValueError()
            payload=packet.get('jpeg_base64')
            if not isinstance(payload,str) or not 4<=len(payload)<=4*((MAX_JPEG_BYTES+2)//3):raise ValueError()
            jpeg=base64.b64decode(payload,validate=True)
            if len(jpeg)>MAX_JPEG_BYTES or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):raise ValueError()
            width,height=packet.get('width'),packet.get('height')
            if type(width) is not int or type(height) is not int or not 1<=width<=480 or not 1<=height<=1080:raise ValueError()
        except (ValueError,TypeError,binascii.Error):
            return failure('invalid_frame','机载预览不是完整、有效或受支持的小图响应',2.)
        # Robot and PC clocks are independent. Adding the entire request RTT is
        # conservative, including startup; never timestamp an old JPEG as now.
        age+=elapsed
        if age>=MAX_REMOTE_AGE:return failure('stale_frame','机载预览传输时间过长，画面已过期',1.)
        source=(stream,sequence)
        if source==lease.last_source and after==lease.sequence:
            return failure('waiting_frame','机载相机尚无更新画面',.5)
        if source!=lease.last_source:
            lease.sequence+=1; lease.last_source=source
        lease.remote_stream=stream; lease.remote_sequence=sequence
        self.onboard=dict(state='ready',error=None)
        return dict(ok=True,jpeg=jpeg,sequence=lease.sequence,age=age,profile=lease.profile,
                    width=width,height=height,transport='onboard_jpeg',upstream_reduced=True,
                    source_bitrate_changed=False,upstream_scope='this_preview_only',age_basis='decode_receive')

    def frame(self,camera='front',profile='standard',consumer='',after=-1):
        preview_camera(camera); spec=preview_profile(profile); preview_consumer(consumer); frame_after(after)
        lease=None
        timeout=(7. if after<0 else 3.) if spec['transport']=='onboard_jpeg' else (5. if after<0 else 1.2)
        try:
            with self.lock:
                self._sync_context_locked()
                lease=self.leases.begin(camera,profile,consumer,timeout=timeout)
            if lease.cancelled.is_set():return failure('cancelled','预览已关闭',.1)
            result=(self._onboard_frame(lease,after,timeout) if spec['transport']=='onboard_jpeg' else
                    self._direct_frame(lease,after,timeout))
            if lease.cancelled.is_set():return failure('cancelled','预览已关闭',.1)
            return result
        except PreviewDenied as exc:
            return exc.result
        except Exception:
            return failure('unavailable','视频预览暂时不可用，请检查视频依赖与连接状态',2.)
        finally:
            if lease is not None:
                self.leases.finish(lease)
                if profile=='snapshot':self.leases.release(consumer)

    def release(self,consumer):
        return self.leases.release(consumer)

    def status(self):
        return dict(protocol=PROTOCOL,profiles=[dict(id=key,**value) for key,value in PROFILES.items()],
                    onboard=dict(self.onboard),demand=self.leases.status(),
                    source_bitrate_changed=False,independent_vision_subscriptions_unchanged=True)

    def reset(self):
        with self.lock:
            self.leases.reset(); self.scope=None; self.onboard=dict(state='unknown',error=None)

    def close(self):
        self.leases.close()

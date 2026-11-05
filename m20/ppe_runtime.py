"""PPE model adapter sharing the platform's camera decoder and latest-frame transport."""
import threading
import time
import m20.m20_console as original
from m20.autonomy_core import iou
from m20.vision_models import detections


def bbox(row):
    return [row[k] for k in ['x1','y1','x2','y2']]


class HelmetAlerts:
    """Persist explicit bare-head detections, separately for each observed head.

    A person without a visible head is unknown, not automatically non-compliant.
    One person's helmet never suppresses another person's bare-head detection.
    """
    def __init__(self,persist=2,cooldown=10):
        self.persist=persist;self.cooldown=cooldown;self.tracks={};self.next_id=0

    def update(self,rows,now):
        hats=[bbox(r) for r in rows if r['class_name']=='Hardhat']
        missing=[r for r in rows if r['class_name']=='NO-Hardhat' and r['confidence']>=.5
                 and not any(iou(bbox(r),hat)>.3 for hat in hats)]
        self.tracks={k:v for k,v in self.tracks.items() if now-v['seen']<.8}
        used=set();events=[]
        for row in missing:
            candidates=[(iou(bbox(row),v['box']),k) for k,v in self.tracks.items() if k not in used]
            score,key=max(candidates,default=(0,None))
            if score<.25:
                self.next_id+=1;key=self.next_id
                self.tracks[key]=dict(since=now,seen=now,last_alert=-float('inf'),box=bbox(row))
            track=self.tracks[key];used.add(key);track.update(seen=now,box=bbox(row))
            if now-track['since']>=self.persist and now-track['last_alert']>=self.cooldown:
                track['last_alert']=now
                events.append(dict(type='NO-Hardhat',cn='检测到未戴安全帽',track_id=key,
                                   confidence=round(row['confidence'],2),duration_s=round(now-track['since'],1)))
        # A fresh frame without this bare head breaks persistence immediately.
        self.tracks={k:v for k,v in self.tracks.items() if k in used}
        return events


class SharedPPEDetector(original.PPEDetector):
    def __init__(self,app):
        super().__init__();self.app=app;self.halt=threading.Event()
        self.condition=threading.Condition(self.lock);self.sequence=0
        self.alerts=HelmetAlerts(original.PPE_PERSIST,original.PPE_COOLDOWN)

    def start(self,cam='front'):
        from pathlib import Path
        if cam not in ['front','rear']:return False,'相机必须为 front 或 rear'
        if original.YOLO is None or original.cv2 is None:return False,'请安装人员识别依赖后重启平台'
        if not Path(original.PPE_MODEL_PATH).is_file():return False,'PPE 模型文件缺失，请检查 models/ppe.pt'
        with self.condition:
            if self._thread and self._thread.is_alive():
                return (False,'检测线程正在关闭，请稍后重试') if self.halt.is_set() else (True,'检测已启动')
            self.halt.clear();self.cam=cam;self.enabled=True;self.error=None
            self.alerts=HelmetAlerts(original.PPE_PERSIST,original.PPE_COOLDOWN)
            self._thread=threading.Thread(target=self._run,daemon=True);self._thread.start()
        return True,'安全帽识别已启动'

    def stop(self):
        with self.condition:
            self.halt.set();self.enabled=False;self.last_dets=[];self._annotated=None
            self.condition.notify_all()
        return True,'安全帽识别已停止'

    def _run(self):
        import numpy as np
        feed=None
        try:
            if not self._load_model():return
            if not {'Hardhat','NO-Hardhat','Person'}<=set(self.model.names.values()):
                raise ValueError('当前模型缺少 Hardhat / NO-Hardhat / Person 类别')
            if self.halt.is_set():return
            feed=self.app.cameras.get(self.cam);feed.subscribe();last=-1
            while not self.halt.is_set():
                packet=feed.read(last,.5)
                if not packet:
                    with self.condition:self.error=feed.status().get('error') or '等待最新相机画面'
                    continue
                last,jpeg=packet
                with feed.condition:
                    if last!=feed.sequence:continue
                    captured=feed.at
                frame=original.cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),original.cv2.IMREAD_COLOR)
                if frame is None:continue
                started=time.monotonic();rows=detections(self.model,frame,original.PPE_CONF)
                infer_ms=(time.monotonic()-started)*1000
                if time.time()-captured>1:
                    with self.condition:self.error='识别画面延迟过大，正在等待最新帧'
                    continue
                # Original renderer expects integer image coordinates.
                rows=[dict(r,**{k:round(r[k]) for k in ['x1','y1','x2','y2']}) for r in rows]
                annotated=self._draw(frame.copy(),rows)
                encoded=original.cv2.imencode('.jpg',annotated,[original.cv2.IMWRITE_JPEG_QUALITY,70])
                if not encoded[0]:continue
                with self.condition:
                    if self.halt.is_set():return
                    self.last_dets=rows;self.last_dets_time=captured;self.last_frame_time=captured
                    self.last_infer_ms=round(infer_ms,1);self.last_annotated=annotated;self.last_annotated_time=captured
                    self._annotated=encoded[1].tobytes();self.sequence+=1;self.error=None
                    for event in self.alerts.update(rows,time.monotonic()):
                        event.update(t=time.strftime('%H:%M:%S'),ts=time.time(),camera=self.cam)
                        self.events.insert(0,event);del self.events[50:]
                    self.condition.notify_all()
                self.halt.wait(original.PPE_INTERVAL)
        except Exception as exc:
            with self.condition:self.error='PPE 检测异常：'+str(exc)
        finally:
            if feed:feed.unsubscribe()
            with self.condition:self.enabled=False;self.condition.notify_all()

    def frame(self,after=-1,timeout=1):
        deadline=time.monotonic()+timeout
        with self.condition:
            while self.enabled:
                age=time.time()-self.last_frame_time
                if self._annotated and self.sequence!=after and age<2:
                    return self.sequence,self._annotated,age
                remaining=deadline-time.monotonic()
                if remaining<=0:break
                self.condition.wait(remaining)
        return None

    def status(self):
        status=super().status()
        with self.lock:
            status.update(counts={name:sum(r['class_name']==name for r in self.last_dets) for name in ['Hardhat','NO-Hardhat','Person','Safety Vest']},
                          model_name='Construction-PPE-Detection / ppe.pt',shared_camera=True,
                          fresh=bool(self.enabled and self.last_frame_time and time.time()-self.last_frame_time<2))
        return status

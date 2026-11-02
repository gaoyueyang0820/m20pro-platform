"""YOLO12 people counting over the shared camera; never emits motion commands."""
from collections import deque
import copy,math,threading,time
from pathlib import Path

ROOT=Path(__file__).resolve().parent
TRACKER=ROOT/'reference/people_counter/botsort.yaml'


class CountLedger:
    """A confirmed track counts once in a continuous tracking segment.

    Track IDs are not identities. Gaps/resume start a new segment, and somebody
    leaving/re-entering can have a new ID. Totals are labelled as track counts.
    """
    def __init__(self,min_frames=2):
        self.min_frames=min_frames;self.segment=0;self.streaks={};self.counted=set();self.total=0;self.peak=0;self.current=0

    def update(self,ids):
        ids=set(int(i) for i in ids)
        self.streaks={i:self.streaks.get(i,0)+1 for i in ids}
        confirmed={i for i,n in self.streaks.items() if n>=self.min_frames}
        for i in confirmed:
            key=(self.segment,i)
            if key not in self.counted:self.counted.add(key);self.total+=1
        self.current=len(confirmed);self.peak=max(self.peak,self.current)
        return confirmed

    def break_segment(self):
        self.segment+=1;self.streaks={};self.current=0


class PeopleCounter:
    def __init__(self,app,model_factory=None):
        self.app=app;self.model_factory=model_factory;self.condition=threading.Condition(threading.RLock())
        self.halt=threading.Event();self.thread=None;self.model=None;self.enabled=False;self.paused=False
        self.camera='front';self.conf=.3;self.epoch=0;self.tracker_reset=True;self.ledger=CountLedger()
        self.jpeg=None;self.captured=0;self.sequence=0;self.error=None;self.rows=[];self.infer_ms=0.;self.times=deque(maxlen=60)
        self.width=0;self.height=0;self.segment_breaks=0;self.session_started=None

    def status(self):
        with self.condition:
            now=time.time();age=now-self.captured if self.captured else None
            fresh=bool(self.enabled and not self.paused and age is not None and 0<=age<1)
            times=[t for t in self.times if time.monotonic()-t<2]
            fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>2 else 0
            return dict(enabled=self.enabled,paused=self.paused,fresh=fresh,camera=self.camera,conf=self.conf,
                        current=self.ledger.current if fresh else None,total_tracks=self.ledger.total,peak=self.ledger.peak,
                        frames=self.sequence,age=round(age,3) if age is not None else None,infer_ms=round(self.infer_ms,1),fps=round(fps,1),
                        width=self.width,height=self.height,error=self.error,model='yolo12n.pt',tracker='BoT-SORT',
                        segment_breaks=self.segment_breaks,session_started=self.session_started,shared_camera=True,
                        state='paused' if self.paused and self.enabled else 'live' if fresh else 'waiting' if self.enabled else 'error' if self.error else 'off')

    def start(self,camera='front',conf=.3):
        if self.app.demo:raise ValueError('实时人数统计需要连接机器人；可先点击“用图片检查人数”')
        if camera not in ['front','rear']:raise ValueError('请选择前置或后置相机')
        conf=self._confidence(conf)
        if not (ROOT/'models/yolo12n.pt').is_file():raise ValueError('人数统计模型缺失：models/yolo12n.pt')
        with self.condition:
            if self.thread and self.thread.is_alive():
                if not self.enabled or self.halt.is_set():raise ValueError('统计正在停止，请稍后重试')
                if camera!=self.camera:raise ValueError('请先停止统计，再切换相机')
                return dict(ok=True,**self.status())
            self.camera=camera;self.conf=conf;self.epoch+=1;self.ledger=CountLedger();self.tracker_reset=True
            self.jpeg=None;self.captured=0;self.rows=[];self.error=None;self.enabled=True;self.paused=False;self.segment_breaks=0
            self.session_started=time.time();self.times.clear();self.halt.clear()
            self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
            return dict(ok=True,**self.status())

    @staticmethod
    def _confidence(value):
        if isinstance(value,bool):raise ValueError('置信度必须为 0.1–0.9')
        try:value=float(value)
        except (TypeError,ValueError):raise ValueError('置信度必须为 0.1–0.9')
        if not math.isfinite(value) or not .1<=value<=.9:raise ValueError('置信度必须为 0.1–0.9')
        return value

    def control(self,req):
        action=req.get('action')
        if action=='start':return self.start(req.get('camera','front'),req.get('conf',.3))
        with self.condition:
            if action=='stop':
                self.halt.set();self.enabled=False;self.paused=False;self.epoch+=1;self.jpeg=None;self.captured=0;self.rows=[];self.error=None;self.condition.notify_all()
            elif action in ['pause','resume']:
                if not self.enabled:raise ValueError('请先启动人数统计')
                desired=action=='pause'
                if self.paused!=desired:
                    self.paused=desired;self.epoch+=1;self.tracker_reset=True;self.ledger.break_segment();self.segment_breaks+=1
                    self.captured=0;self.times.clear();self.error=None;self.condition.notify_all()
            elif action=='reset':
                self.epoch+=1;self.ledger=CountLedger();self.tracker_reset=True;self.jpeg=None;self.captured=0;self.rows=[];self.segment_breaks=0;self.session_started=time.time();self.condition.notify_all()
            elif action=='configure':
                self.conf=self._confidence(req.get('conf'));self.epoch+=1;self.ledger.streaks={};self.captured=0;self.condition.notify_all()
            else:raise ValueError('未知人数统计操作')
            return dict(ok=True,**self.status())

    def _reset_tracker(self):
        for tracker in getattr(getattr(self.model,'predictor',None),'trackers',[]):tracker.reset()

    def _run(self):
        import cv2,numpy as np
        feed=None;last=-1;last_capture=0;stale_reset=False
        try:
            if self.model is None:
                if self.model_factory:self.model=self.model_factory()
                else:
                    import importlib.util
                    if importlib.util.find_spec('lap') is None:raise ValueError('跟踪依赖尚未安装，请运行“安装人员识别依赖.command”')
                    from ultralytics import YOLO
                    self.model=YOLO(str(ROOT/'models/yolo12n.pt'))
            classes=[i for i,name in self.model.names.items() if str(name).lower()=='person']
            if not classes:raise ValueError('当前权重没有 person 类别')
            if self.halt.is_set():return
            feed=self.app.cameras.get(self.camera);feed.subscribe()
            while not self.halt.is_set():
                with self.condition:
                    if self.paused:self.condition.wait(.2);continue
                    epoch=self.epoch;conf=self.conf;reset=self.tracker_reset;self.tracker_reset=False
                if reset:self._reset_tracker()
                packet=feed.read(last,.4)
                if not packet:
                    with self.condition:
                        self.error=feed.status().get('error') or '等待最新相机画面'
                        if last_capture and time.time()-last_capture>1 and not stale_reset:
                            self.ledger.break_segment();self.tracker_reset=True;self.segment_breaks+=1;stale_reset=True
                    continue
                last,jpeg=packet
                with feed.condition:
                    if last!=feed.sequence:continue
                    captured=feed.at
                if time.time()-captured>1:continue
                if last_capture and captured-last_capture>1 and not stale_reset:
                    with self.condition:self.ledger.break_segment();self.segment_breaks+=1
                    self._reset_tracker()
                last_capture=captured;stale_reset=False
                frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                if frame is None:continue
                began=time.monotonic()
                results=self.model.track(frame,persist=True,tracker=str(TRACKER),classes=classes,conf=conf,imgsz=480,device='cpu',verbose=False)
                elapsed=(time.monotonic()-began)*1000;rows=[]
                for result in results:
                    if result.boxes is None or result.boxes.id is None:continue
                    for box in result.boxes:
                        if int(box.cls[0]) not in classes:continue
                        rows.append(dict(id=int(box.id[0]),confidence=float(box.conf[0]),box=[float(v) for v in box.xyxy[0]]))
                with self.condition:
                    if self.halt.is_set() or self.paused or epoch!=self.epoch:continue
                    if time.time()-captured>1:
                        self.error='统计处理过慢，等待最新画面';self.ledger.break_segment();self.tracker_reset=True;self.segment_breaks+=1;continue
                    confirmed=self.ledger.update(r['id'] for r in rows)
                    for r in rows:
                        x1,y1,x2,y2=map(round,r['box']);color=(100,190,60) if r['id'] in confirmed else (140,140,140)
                        cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
                        cv2.putText(frame,'Person #%d %.2f'%(r['id'],r['confidence']),(x1,max(18,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,2)
                    cv2.rectangle(frame,(0,0),(frame.shape[1],34),(25,38,30),-1)
                    cv2.putText(frame,'NOW %d   TRACKS %d   PEAK %d'%(self.ledger.current,self.ledger.total,self.ledger.peak),(12,23),cv2.FONT_HERSHEY_SIMPLEX,.6,(235,245,235),2)
                    ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,70])
                    if not ok:continue
                    self.jpeg=encoded.tobytes();self.captured=captured;self.sequence+=1;self.rows=rows;self.infer_ms=elapsed
                    self.width=frame.shape[1];self.height=frame.shape[0];self.error=None;self.times.append(time.monotonic());self.condition.notify_all()
                self.halt.wait(.02)
        except Exception as exc:
            with self.condition:
                if not self.halt.is_set():self.error='人数统计未就绪：'+str(exc)
        finally:
            if feed:feed.unsubscribe()
            with self.condition:self.enabled=False;self.paused=False;self.condition.notify_all()

    def frame(self,after=-1,timeout=1):
        deadline=time.monotonic()+timeout
        with self.condition:
            while self.enabled and not self.paused:
                age=time.time()-self.captured
                if self.jpeg and self.sequence!=after and 0<=age<1:return self.sequence,self.jpeg,age
                wait=deadline-time.monotonic()
                if wait<=0:break
                self.condition.wait(wait)
        return None

    def stop(self):return self.control(dict(action='stop'))

"""Computer-side autonomous sessions; no robot I/O until an explicit start.

ROS observation reduction is on NOS/GOS. Planning and YOLO run on the computer.
The existing 0.5 s robot-link velocity watchdog remains the last command expiry.
"""
import base64
import copy
import math
from pathlib import Path
import threading
import time
import uuid
from collections import deque

from autonomy_core import Explorer, PriorMap, PersonLock, clearance, RESOLUTION
from platform_core import number
from vision_models import default_path
from localization_runtime import OperationCancelled
from latest_observation import LatestObservationPump, ObservationContextChanged


class PersonVision:
    def __init__(self, app):
        self.app=app;self.lock=threading.RLock();self.thread=None;self.quit=threading.Event()
        self.row=dict(enabled=False,ready=False,detections=[],sequence=0,error=None)
        self.jpeg=None;self.frame_at=0;self.selected_path=self.app.store.value.get('person_model_path');self.selection_frames={}

    def start(self, path=''):
        import m20_console as original
        if self.app.demo:
            self.row.update(enabled=True,ready=True,error=None)
            return dict(ok=True,demo=True)
        with self.lock:
            if self.thread and self.thread.is_alive():
                if self.quit.is_set():raise ValueError('视觉线程正在关闭，请稍候再开启')
                return dict(ok=True)
            model_path=Path(path or self.selected_path or default_path('person')).expanduser()
            if not model_path.is_file():raise ValueError('人员检测模型不存在，请恢复 models/yolo11n.pt 或选择自定义模型')
            if model_path.suffix.lower()!='.pt':raise ValueError('当前人员跟随适配器使用 Ultralytics .pt 检测模型')
            if original.YOLO is None:raise ValueError('当前 Python 环境未安装 ultralytics，请先安装视觉依赖')
            self.selected_path=str(model_path.resolve());self.quit.clear()
            self.app.store.value['person_model_path']=self.selected_path;self.app.store.save()
            self.row.update(enabled=True,ready=False,error=None)
            self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
        return dict(ok=True)

    def _run(self):
        import m20_console as original
        import numpy as np
        feed=None
        try:
            model=original.YOLO(self.selected_path)
            names=model.names.values() if isinstance(model.names,dict) else model.names
            if not any(str(n).lower() in ['person','human','人','人员'] for n in names):
                raise ValueError('这个模型不含 person / Person 人员类别')
            feed=self.app.cameras.get('front');feed.subscribe();sequence=-1
            while not self.quit.is_set():
                result=feed.read(sequence,.5)
                if result is None:continue
                sequence,jpeg=result
                # Capture timestamp belongs to these bytes, not to inference finish.
                with feed.condition:
                    if sequence!=feed.sequence:continue
                    captured=feed.at
                frame=original.cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),original.cv2.IMREAD_COLOR)
                if frame is None:continue
                h,w=frame.shape[:2];began=time.monotonic()
                results=model.predict(frame,conf=.55,imgsz=416,verbose=False)
                detections=[]
                for r in results:
                    if r.boxes is None:continue
                    for b in r.boxes:
                        if str(model.names[int(b.cls[0])]).lower() not in ['person','human','人','人员']:continue
                        x1,y1,x2,y2=[float(v) for v in b.xyxy[0]]
                        detections.append(dict(box=[x1/w,y1/h,x2/w,y2/h],confidence=float(b.conf[0])))
                with self.lock:
                    self.jpeg=jpeg;self.frame_at=captured
                    self.row.update(ready=True,sequence=sequence,detections=detections,error=None,
                                    infer_ms=round((time.monotonic()-began)*1000),width=w,height=h)
                self.quit.wait(.03)
        except Exception as exc:
            with self.lock:self.row.update(ready=False,error=str(exc))
        finally:
            if feed:feed.unsubscribe()
            with self.lock:self.row['enabled']=False

    def selected(self,sequence):
        with self.lock:
            result=self.selection_frames.get(sequence)
            if not result or time.time()-result[0]>.65:raise ValueError('选中画面已过期，请在最新画面中重新点选')
            return result[1]

    def stop(self):
        self.quit.set()
        with self.lock:self.row.update(enabled=False,ready=False,detections=[]);self.jpeg=None

    def snapshot(self, image=False):
        with self.lock:
            row=copy.deepcopy(self.row)
            row['age']=time.time()-self.frame_at if self.frame_at else None
            row['fresh']=bool(row['ready'] and row['age'] is not None and row['age']<.65)
            row['model_name']=Path(self.selected_path or default_path('person')).name
            row['model_available']=Path(self.selected_path or default_path('person')).is_file()
            if image and self.jpeg:
                row['image']='data:image/jpeg;base64,'+base64.b64encode(self.jpeg).decode()
                self.selection_frames[row['sequence']]=(self.frame_at,copy.deepcopy(row['detections']))
                self.selection_frames={k:v for k,v in self.selection_frames.items() if time.time()-v[0]<.8}
            if self.app.demo and row['enabled']:
                height=.40+.035*math.sin(time.monotonic()*.6)
                center=.5+.10*math.sin(time.monotonic()*.3)
                row.update(demo=True,fresh=True,age=0,sequence=int(time.monotonic()*5),
                           detections=[dict(box=[center-.08,.35,center+.08,.35+height],confidence=.96)])
            return row


class MotionProgress:
    """Progress is new position/goal approach, not alternating yaw or commands."""
    def __init__(self):
        self.anchor=None;self.seen=set();self.goal=None;self.best_distance=None
        self.best_heading=None;self.best_area=0;self.started=None;self.moved_at=None;self.last_at=None
        self.last_motion=None

    def update(self,pose,goal,area,now,moving=None):
        x,y,yaw=pose
        if not all(math.isfinite(v) for v in pose):raise ValueError('进展位姿无效')
        key=(math.floor(x/.25),math.floor(y/.25));kind='none'
        target=tuple(goal) if goal is not None else None
        distance=math.dist((x,y),target) if target else None
        error=abs(math.atan2(math.sin(math.atan2(target[1]-y,target[0]-x)-yaw),math.cos(math.atan2(target[1]-y,target[0]-x)-yaw))) if target else None
        if self.anchor is None:
            self.anchor=(x,y);self.started=self.moved_at=self.last_at=now;self.seen.add(key)
        if moving is True:self.last_motion=now
        motion_supported=moving is not False or self.last_motion is not None and now-self.last_motion<1.
        same=target is not None and target==self.goal
        if not same:self.goal=target;self.best_distance=distance;self.best_heading=error
        approaching=same and distance<self.best_distance-.15
        if motion_supported and math.dist((x,y),self.anchor)>.15:
            self.anchor=(x,y)
            if key not in self.seen or approaching:
                self.last_at=self.moved_at=now;kind='translation'
            self.seen.add(key)
        if approaching and motion_supported:self.best_distance=distance
        if same and error<self.best_heading-.12:
            self.best_heading=error
            if now-self.moved_at<10:self.last_at=now;kind='alignment'
        # Initial scanning may reveal supported space, but cannot keep an
        # otherwise stationary task alive indefinitely through changing returns.
        if area>self.best_area+1.0:
            self.best_area=area
            if now-self.started<20:self.last_at=now;kind='initial_coverage'
        return dict(kind=kind,seconds_without_progress=round(now-self.last_at,2))

    def stalled(self,now):return self.last_at is not None and now-self.last_at>25


class Autonomy:
    def __init__(self,app):
        self.app=app;self.vision=PersonVision(app);self.active=False;self.run_id=None
        self.state='idle';self.mode=None;self.message='选择自主建图或跟随人员';self.lease=0
        self.explorer=None;self.target=None;self.started=0;self.scan=None;self.result_maps=[];self.map_view=None;self.finished_elapsed=0
        self.deadline=None;self.timing=deque(maxlen=180);self.scan_timing={}
        self.scope='room';self.prior_grid=None
        self.context=None;self.generation=0;self._save_token=None
        self.capture_id=None
        self.observation_pump=None

    def snapshot(self):
        p=self.app.hub.preparation
        cleanup=bool(self._save_token or p and p['kind']=='mapping' and p['cancelled'])
        uncertain=self.app.hub.mapping_uncertain
        unresolved=bool(uncertain and self.app.hub.same_context(uncertain['context']))
        return dict(active=self.active,run_id=self.run_id,state=self.state,mode=self.mode,message=self.message,
                    demo=self.app.demo,elapsed=round(time.monotonic()-self.started) if self.started and self.active else self.finished_elapsed,
                    map=self.map_view,target=self.target.box if self.target else None,
                    vision=self.vision.snapshot(),result_maps=self.result_maps,timing=list(self.timing),scope=self.scope,
                    cleanup_pending=cleanup,preparation_unresolved=unresolved,
                    observation_pump=self.observation_pump.status() if self.observation_pump else None,
                    can_start=not(self.active or self.state=='saving' or cleanup or unresolved or self.app.hub.busy))

    def pause(self,message='已停止自主移动，可手动接管'):
        # Always called while app.lock is held. No joins or network operations.
        preparation=self.app.hub.preparation
        self.app.hub.cancel_preparations()
        if self.observation_pump:self.observation_pump.cancel()
        if self.active:
            if preparation and preparation['kind']=='mapping' and preparation.get('dispatching'):
                message='已停止自主移动；正在收尾已发出的建图请求'
            self.finished_elapsed=round(time.monotonic()-self.started) if self.started else 0
            self.active=False;self.run_id=None;self.state='paused';self.message=message
            self.app.velocity=[0.,0.,0.]
            if self.app.link:self.app.link.set_velocity(0,0,0)

    def reset_context(self):
        """Connection changes invalidate callbacks without reconnecting anything."""
        self.pause('连接已切换，旧自主任务不会恢复')
        self.app.hub.detach_context()
        self.generation+=1;self._save_token=None;self.context=None
        self.state='idle';self.message='连接已切换，等待开始新任务';self.result_maps=[]
        self.map_view=None;self.scan=None;self.deadline=None;self.capture_id=None

    def _assert_run(self,run_id):
        with self.app.lock:
            if (not self.active or self.run_id!=run_id or
                    self.context is not None and not self.app.hub.same_context(self.context)):
                raise OperationCancelled('自主任务已取消或连接已变化')
            if self.deadline is not None and time.monotonic()>=self.deadline or time.monotonic()-self.lease>2:
                self.pause('自主准备或会话已到期，已停止')
                raise OperationCancelled('自主准备或会话已到期')

    def watchdog(self):
        if self.active and self.deadline is not None and time.monotonic()>=self.deadline:
            if self.mode=='explore' and (self.app.mapping or self.app.hub.has_preparation(self.run_id)):
                self.finish_save(self.run_id,'达到本轮时间，已停车，正在保存地图')
            else:self.pause('本轮时间已到，已停车')
        if self.active and time.monotonic()-self.lease>2:
            self.pause('操作页面已断开或进入后台，已停车；返回后重新开始')

    def _drive(self,run_id,forward,yaw,valid_until=None,side=0):
        began=time.monotonic()
        with self.app.lock:
            self.watchdog()
            if not self.active or self.run_id!=run_id or self.context is not None and not self.app.hub.same_context(self.context):return False
            if (forward or side or yaw) and not self.app.demo and (valid_until is None or time.monotonic()>valid_until):
                self.app.set_velocity(dict(forward=0,yaw=0),autonomous=True)
                self.message='已停车，等待新一帧感知后继续'
                self.timing.append(dict(self.scan_timing,reason='decision_expired',lock_ms=round((time.monotonic()-began)*1000,1)))
                return None
            self.app.set_velocity(dict(forward=forward,side=side,yaw=yaw),autonomous=True,valid_until=valid_until)
            self.timing.append(dict(self.scan_timing,reason='command',command=[forward,side,yaw],lock_ms=round((time.monotonic()-began)*1000,1),remaining_ms=round((valid_until-time.monotonic())*1000,1) if valid_until else None))
            return True

    def _check_start(self):
        if self.active or self.state=='saving':raise ValueError('请先停止当前自主任务')
        self.app.hub.normal_mode();self.app.hub.idle();self.app.live_ready()
        if self.app.hub.busy:raise ValueError('正在执行后台操作，请稍候')
        if self.app.reverse:raise ValueError('请将手动驾驶的前进方向切回头部朝前')
        if self.app.estopped or self.app.basic.get('MotionState')!=17:
            raise ValueError('请先让机器人站稳，再开始自主操作')
        if not self.app.demo and self.app.link:
            nav=self.app.link.record(1007,1)
            if time.time()-nav.get('at',0)<2 and nav.get('items',{}).get('Status') in [1,2,3,5,255]:
                raise ValueError('机器人仍有外部或上次导航任务，请先停止后再开始自主作业')

    def _check_observation(self,check=None):
        """Wake the on-demand reader before starting a new map or any movement."""
        if self.app.demo:return
        check=check or (lambda:None)
        last='等待雷达与地面观测';clear_frames=0;last_stamp=None
        for attempt in range(10):
            check()
            try:scan=self.scan_now(self.app.mapping)
            except ValueError as exc:
                last=str(exc)
                clear_frames=0
                if attempt<9:time.sleep(.1)
                continue
            check()
            clear,reason=clearance(scan,0,.1)
            # A narrow passage may permit straight travel without enough room
            # to turn in place. Check the actual chosen motion again in _run.
            if not clear:
                forward_clear,forward_reason=clearance(scan,.1,0)
                clear=forward_clear
                if not clear:reason=forward_reason
            if not clear:
                last='已收到雷达，但'+reason.replace('，已停车','');clear_frames=0
            elif scan.get('stamp')!=last_stamp:
                clear_frames+=1
                if clear_frames>=2:return
            last_stamp=scan.get('stamp');time.sleep(.1)
        raise ValueError('尚未开始自主移动：'+last)

    def start(self,req):
        mode=req.get('mode');scope=req.get('scope','room')
        if mode not in ['explore','follow']:raise ValueError('请选择自主建图或跟随人员')
        if scope not in ['room','floor']:raise ValueError('探索范围无效')
        if scope=='floor' and (mode!='explore' or not req.get('extend')):raise ValueError('整层探索需要从当前地图续建，才能同时更新旧区域')
        radius=number(req.get('radius',6),2,12);minutes=number(req.get('minutes',5),.1,60 if scope=='floor' else 20)
        forward=number(req.get('forward_limit',.35),.10,1);yaw=number(req.get('yaw_limit',.70),.1,1)
        with self.app.lock:
            self._check_start()
            if mode=='follow':
                if self.app.mapping:raise ValueError('请先停止并保存建图，再开始人员跟随')
                if self.app.basic.get('ControlUsageMode') not in [0,2]:raise ValueError('请先切换常规或辅助驾驶模式')
                vision=self.vision.snapshot()
                if not vision['fresh']:raise ValueError(vision.get('error') or '等待新鲜的 YOLO 人员检测画面')
                selected=vision['detections'] if self.app.demo else self.vision.selected(req.get('sequence'))
                index=req.get('person')
                if isinstance(index,bool) or not isinstance(index,int) or not 0<=index<len(selected):raise ValueError('请点选要跟随的人')
                box=selected[index]['box']
                if box[3]-box[1]<.12 or box[1]<.02 or box[3]>.98:raise ValueError('请让人物全身进入画面后再选择')
                self.target=PersonLock(box);self.target.update(vision['detections']);self.explorer=None
            else:
                if self.app.mapping and not req.get('use_current'):raise ValueError('当前已有建图采集，请选择继续当前采集，或先结束并保存')
                if not self.app.mapping and req.get('use_current'):raise ValueError('当前没有建图采集，请选择新建或续建')
                self.target=None;self.explorer=Explorer(radius,forward,yaw)
            self.generation+=1;self._save_token=None;self.context=self.app.hub.context()
            self.capture_id=None
            self.active=True;self.run_id=uuid.uuid4().hex;self.mode=mode;self.state='preparing';self.scope=scope;self.prior_grid=None
            self.message='正在检查实时感知与建图条件' if mode=='explore' else '已锁定人物，正在检查前方地面'
            self.started=time.monotonic();self.lease=self.started;self.result_maps=[];self.map_view=None;self.finished_elapsed=0
            self.deadline=self.started+minutes*60;self.timing.clear();run_id=self.run_id
        # Return the lease ID before slow sensor warmup or map I/O. Stop requests
        # can now cancel preparation instead of waiting for a held state lock.
        try:threading.Thread(target=self._prepare_and_run,args=(run_id,dict(req),minutes*60),daemon=True).start()
        except Exception:
            with self.app.lock:
                if self.run_id==run_id:self.pause('准备线程未能启动，未发送运动或建图请求')
            raise
        return dict(ok=True,run_id=run_id,state='preparing',message='正在检查并准备，不会自动恢复旧任务')

    def _prepare_and_run(self,run_id,req,duration):
        try:
            check=lambda:self._assert_run(run_id)
            check()
            if not self.app.demo:
                role=self.app.hub.connections.value.get('sensor_source','nos')
                status=self.app.proxy(role,'/sensors/status',timeout=2);check()
                if status.get('autonomy_protocol')!=1:raise ValueError('请先在连接中心更新 '+role.upper()+' 平台服务，以接通自主感知接口')
            job_id=None
            if req['mode']=='explore' and not req.get('use_current'):
                self._check_observation(check);check()
                grid=None
                if req.get('extend'):
                    grid=self.app.hub.remote_maps('/maps/grid',dict(name=req.get('base'),grid_id=req.get('grid_id',0)),context=self.context);check()
                with self.app.lock:
                    check();self.prior_grid=grid
                    if grid and self.scope=='floor':
                        from navigation_planner import pgm_pixels
                        w,h,_=pgm_pixels(base64.b64decode(grid['pgm'],validate=True))
                        self.explorer.radius=min(100,max(12,math.hypot(w,h)*grid['resolution']+6))
                    path='/api/maps/extend-here' if req.get('extend') else '/api/maps/start-here'
                    job_id=self.app.hub.post(path,dict(base=req.get('base'),grid_id=req.get('grid_id',0),name=req.get('name','m20-auto-map'),autonomy_gait=0x3002,_autonomy_run_id=run_id))['job_id']
            elif req['mode']=='explore':
                with self.app.lock:
                    check();job_id=self.app.hub.post('/api/maps/continue-here',dict(autonomy_gait=0x3002,_autonomy_run_id=run_id))['job_id']
            check();self._run(run_id,job_id,duration)
        except Exception as exc:
            with self.app.lock:
                if self.run_id==run_id:self.pause(str(exc));self.app.add_log('autonomy',str(exc))

    def scan_now(self,mapping,role=None):
        if self.app.demo:
            x,y,yaw=[self.app.pose[k] for k in ['PosX','PosY','Yaw']]
            c,s=math.cos(yaw),math.sin(yaw);floor=[];obstacles=[]
            # A finite synthetic room, clearly labelled in every status response.
            for ix in range(-23,24):
                for iy in range(-23,24):
                    px,py=ix*.2,iy*.2;wx,wy=x+c*px-s*py,y+s*px+c*py
                    if math.hypot(px,py)>4.5:continue
                    if -10<wx<0 and -7<wy<2:floor.append([ix,iy])
                    elif -10.3<wx<.3 and -7.3<wy<2.3:obstacles.append([ix,iy])
            stamp=time.monotonic()
            return dict(ok=True,protocol=1,demo=True,age=0,floor=floor,obstacles=obstacles,pose=[x,y,yaw],stamp=stamp,_valid_until=stamp+.65)
        role=role or self.app.hub.connections.value.get('sensor_source','nos')
        began=time.monotonic()
        scan=self.app.proxy(role,'/sensors/autonomy',dict(mapping=mapping),timeout=.7)
        if scan.get('protocol')!=1:raise ValueError('机器人连接服务尚未同步自主感知接口')
        rtt=time.monotonic()-began
        processing=number(scan.get('server_elapsed_s',0),0,5)
        # Returned age already includes server waiting/processing. Add network
        # transit conservatively, without counting the same processing twice.
        age=number(scan.get('age',99),0,100)+max(0,rtt-min(rtt,processing))
        timing=dict(stamp=scan.get('stamp'),rpc_ms=round(rtt*1000,1),server_ms=round(processing*1000,1),age_ms=round(age*1000,1))
        if age>.65:
            error=ValueError('雷达反馈延迟过大，已停车');error.scan_timing=timing;raise error
        scan['_timing']=timing
        scan['_valid_until']=time.monotonic()+.65-age
        return scan

    def _run(self,run_id,job_id,duration):
        mode=self.mode;explorer=self.explorer;target=self.target
        from local_trajectory import TrajectoryFollower
        follower=TrajectoryFollower(explorer.forward_limit,explorer.yaw_limit) if explorer else None
        trajectory_key=None;trajectory_detail={}
        last_plan=0;last_vision=None;blocked_attempts=0;waiting=time.monotonic();exhausted=0;progress=MotionProgress()
        last_yaw=None;scan_sweep=0.;scan_started=None
        sensing_lost=None;recovery_frames=0;recovery_stamp=None
        pump=None;pump_generation=None;pump_capture_id=None;consumed_sequence=None;last_cache_state=None;scanning=False
        captured_context=self.context
        source_role=self.app.hub.connections.value.get('sensor_source','nos')
        def fetch_observation(description,stop):
            def check_owner():
                with self.app.lock:
                    if (stop.is_set() or not self.active or self.run_id!=run_id
                            or captured_context is not None and not self.app.hub.same_context(captured_context)
                            or self.capture_id!=description['capture_id']):
                        raise ObservationContextChanged('自主任务、连接或建图采集会话已变化')
            # Deadline/save arbitration belongs to the control watchdog. The
            # reader must not turn a normal timed save into a competing pause.
            check_owner()
            previous=getattr(self.app.hub._operation_context,'value',None)
            self.app.hub._operation_context.value=captured_context
            try:packet=self.scan_now(mode=='explore',role=source_role)
            finally:self.app.hub._operation_context.value=previous
            check_owner()
            return packet
        map_health={}
        def read_build_status():
            while self.active and self.run_id==run_id and not self.app.done:
                if job_id is not None:time.sleep(.1);continue
                try:
                    result=self.app.hub.remote_maps('/maps/build/status',context=self.context,timeout=8)
                    with self.app.lock:
                        if self.active and self.run_id==run_id and self.app.hub.same_context(self.context):self.app.mapping=bool(result.get('in_progress'))
                except Exception as exc:result=dict(phase='error',message=str(exc))
                map_health['result']=(time.monotonic(),result)
                time.sleep(2)
        if mode=='explore':threading.Thread(target=read_build_status,daemon=True).start()
        try:
            while self.active and self.run_id==run_id and not self.app.done:
                now=time.monotonic()
                with self.app.lock:
                    self.watchdog()
                    if not self.active or self.run_id!=run_id:return
                    if captured_context is not None and not self.app.hub.same_context(captured_context):
                        raise OperationCancelled('自主任务连接已变化，已停止')
                    if pump is not None and self.capture_id!=pump_capture_id:
                        raise OperationCancelled('建图采集会话已变化，已停止自主移动')
                if job_id:
                    job=self.app.hub.operations.get(job_id)
                    if job['status']=='failed':raise ValueError(job['error'])
                    if job['status']!='completed':
                        with self.app.lock:
                            if self.run_id==run_id:self.message=job.get('message') or '正在准备建图'
                        time.sleep(.1);continue
                    if job.get('result',{}).get('cancelled'):
                        with self.app.lock:
                            if self.run_id==run_id:self.pause('准备已取消，未启动自主移动')
                        return
                    with self.app.lock:
                        if self.run_id!=run_id:return
                        self.capture_id=job.get('result',{}).get('request_id')
                    job_id=None;waiting=now
                if mode=='explore':
                    health_at,build=map_health.get('result',(0,{}))
                    if self.capture_id and health_at and build.get('mapping',{}).get('request_id')!=self.capture_id:
                        raise ValueError('采集归属已变化，已停止自主移动，未结束其他采集')
                    if build.get('phase')!='collecting' or now-health_at>5:
                        self._drive(run_id,0,0)
                        if self.state=='preparing' and now-waiting<30 and build.get('phase')!='error':
                            time.sleep(.2);continue
                        raise ValueError('建图状态未就绪：'+build.get('message','等待建图状态反馈'))
                if pump is None:
                    with self.app.lock:
                        self._assert_run(run_id)
                        pump=LatestObservationPump(fetch_observation)
                        self.observation_pump=pump
                        pump_capture_id=self.capture_id
                        pump_generation=pump.start(dict(run_id=run_id,capture_id=self.capture_id))
                cached=pump.read(pump_generation,consumed_sequence)
                if cached['state'] in ['context_changed','generation_changed']:
                    raise OperationCancelled(cached.get('error') or '自主感知连接代际已变化')
                scan=cached['observation']
                if scan is None:
                    sensing_lost=sensing_lost or time.monotonic();recovery_frames=0;recovery_stamp=None
                    if self.state=='preparing' and now-waiting<20 or time.monotonic()-sensing_lost<5:
                        if cached['state']!=last_cache_state:
                            if not self._drive(run_id,0,0):return
                        last_cache_state=cached['state']
                        with self.app.lock:
                            if self.run_id==run_id:
                                self.message='已停车，等待感知恢复：'+(cached.get('error') or '等待新鲜完整扫描')
                                self.scan_timing['observation_pump']=pump.status()['counters']
                        time.sleep(.02);continue
                    raise ValueError('感知持续无法及时更新：'+(cached.get('error') or cached['state']))
                last_cache_state=cached['state']
                new_scan=cached['sequence']!=consumed_sequence
                if not self.active or self.run_id!=run_id:return
                if sensing_lost is not None:
                    stamp=scan.get('stamp',now)
                    if recovery_stamp is None or stamp>recovery_stamp:recovery_frames+=1;recovery_stamp=stamp
                    if recovery_frames<2:
                        if not self._drive(run_id,0,0):return
                        time.sleep(.02);continue
                with self.app.lock:
                    if not self.active or self.run_id!=run_id:return
                    self.scan=scan;self.scan_timing=dict(scan.get('_timing',{}),cache_wait_ms=round(scan.get('_cache_wait_ms',0),1),
                                                       observation_pump=pump.status()['counters'])
                    self.scan_timing['received_age_ms']=self.scan_timing.get('age_ms')
                    self.scan_timing['age_ms']=round(scan['age']*1000,1)
                valid_until=scan.get('_valid_until')
                if not self.app.demo:scan['_velocity_body']=None
                if not self.app.demo and self.app.link:
                    motion=self.app.link.record(1002,4)
                    if time.time()-motion.get('at',0)<.25:
                        measured=motion.get('items',{}).get('MotionStatus',{})
                        velocity=[measured.get('LinearX'),measured.get('LinearY')]
                        if all(isinstance(v,(int,float)) and math.isfinite(v) for v in velocity):
                            scan['_velocity_body']=velocity;scan['_speed_mps']=math.hypot(*velocity)
                if now-self.started>=duration:
                    self._drive(run_id,0,0)
                    if mode=='explore':self.finish_save(run_id,'达到本轮时间，正在保存地图')
                    else:
                        with self.app.lock:self.pause('本轮跟随时间已到，已停车')
                    return
                if mode=='explore':
                    if self.prior_grid and explorer.prior is None:
                        anchor=build.get('mapping',{}).get('context',{}).get('anchor')
                        if not anchor and not self.app.demo:raise ValueError('缺少本轮旧图定位锚点，已停车')
                        if anchor:explorer.prior=PriorMap(self.prior_grid,anchor)
                    stage=time.monotonic()
                    if new_scan:
                        explorer.ingest(scan);consumed_sequence=cached['sequence']
                    else:explorer.current_scan=scan
                    with self.app.lock:
                        if self.run_id==run_id:self.map_view=explorer.view()
                    self.scan_timing['ingest_ms']=round((time.monotonic()-stage)*1000,1)
                    if new_scan:scanning=False
                    if new_scan and last_yaw is not None:
                        turned=abs(math.atan2(math.sin(scan['pose'][2]-last_yaw),math.cos(scan['pose'][2]-last_yaw)))
                        if scan_started is not None:scan_sweep+=turned
                    if new_scan:last_yaw=scan['pose'][2]
                    moving=None
                    if not self.app.demo and self.app.link:
                        feedback=self.app.link.record(1002,4)
                        if time.time()-feedback.get('at',0)<.5:
                            values=feedback.get('items',{}).get('MotionStatus',{})
                            moving=math.hypot(values.get('LinearX',0),values.get('LinearY',0))>.08
                    detail=(progress.update(scan['pose'],tuple(c*RESOLUTION for c in explorer.goal) if explorer.goal else None,len(explorer.free)*RESOLUTION**2,time.monotonic(),moving)
                            if new_scan else dict(kind='cached_observation',seconds_without_progress=round(time.monotonic()-progress.last_at,2) if progress.last_at is not None else 0.))
                    self.scan_timing['progress']=detail
                    if progress.stalled(time.monotonic()):raise ValueError('持续没有有效位移或目标接近，已暂停；转向摆动不会计为行走进展')
                    if new_scan and (now-last_plan>1 or not explorer.path):
                        stage=time.monotonic();outcome=explorer.plan();last_plan=now
                        self.scan_timing['plan_ms']=round((time.monotonic()-stage)*1000,1)
                        if outcome=='blocked':
                            self._drive(run_id,0,0);blocked_attempts+=1;explorer.path=[]
                            with self.app.lock:
                                if self.run_id==run_id:self.message='已停车，等待周边障碍移开或新一帧地面观测'
                            if blocked_attempts>=10 and not explorer.frontier_metrics.get('deferred_regions'):
                                raise ValueError('周围持续存在障碍或地面不可通行，已暂停探索')
                            time.sleep(.08);continue
                        if outcome=='exhausted' and math.dist(explorer.last_pose[:2],explorer.origin)<.15 and scan_sweep<1.8 and (scan_started is None or now-scan_started<20):
                            turn_clear,_=clearance(scan,0,.12)
                            if turn_clear:
                                scan_started=scan_started or now;scanning=True;exhausted=0
                        if not scanning:exhausted=exhausted+1 if outcome=='exhausted' else 0
                        if exhausted>=5:
                            self._drive(run_id,0,0);self.finish_save(run_id,'本轮没有更多可达探索点，正在保存地图');return
                    yaw_rate=0.
                    if not self.app.demo and self.app.link:
                        record=self.app.link.record(1002,4)
                        if time.time()-record.get('at',0)<.25:
                            measured=record.get('items',{}).get('MotionStatus',{})
                            yaw_rate=measured.get('OmegaZ',0.)
                            scan['_speed_mps']=math.hypot(measured.get('LinearX',0),measured.get('LinearY',0))
                    trajectory_verified=False
                    if not scanning and explorer.path and explorer.recovery is None:
                        key=tuple(explorer.path)
                        if key!=trajectory_key:follower.set_path([(x*RESOLUTION,y*RESOLUTION) for x,y in key]);trajectory_key=key
                        trajectory_detail=follower.command(scan,yaw_rate,allowed_pose=lambda x,y,a:math.dist((x,y),explorer.origin)<=explorer.radius)
                        if trajectory_detail['state']=='arrived':
                            explorer.visited.add(explorer.goal);explorer.path=[];last_plan=0
                            forward=side=yaw=0.;trajectory_verified=True
                        elif trajectory_detail['state']=='tracking':
                            forward,side,yaw=trajectory_detail['axes'];trajectory_verified=True;explorer.motion_hint=trajectory_detail['reason']
                        elif new_scan:forward,side,yaw=explorer.command_axes(yaw_rate,scan.get('age',0))
                        else:forward=side=yaw=0.;trajectory_verified=True
                    elif scanning:forward,side,yaw=0.,0.,explorer.yaw_limit
                    elif explorer.recovery is not None:
                        explorer._current_yaw_rate=yaw_rate
                        forward,side,yaw=explorer._recover_axes()
                    elif new_scan:forward,side,yaw=explorer.command_axes(yaw_rate,scan.get('age',0))
                    else:forward=side=yaw=0.;trajectory_verified=True
                else:
                    consumed_sequence=cached['sequence']
                    vision=self.vision.snapshot()
                    if not vision['fresh']:raise ValueError('人员识别或相机更新中断，已停车；请重新点选')
                    if vision['sequence']==last_vision:
                        # Never refresh a moving command using an old inference.
                        time.sleep(.03);continue
                    last_vision=vision['sequence'];forward,yaw=target.update(vision['detections']);side=0.
                    if not self.app.demo:valid_until=min(valid_until,time.monotonic()+.65-vision['age'])
                # The sampled controller has checked the actual swept arc on
                # this same fresh scan. Do not replace it with a 90-degree disk
                # test, which would reject a feasible gentle turn in a corridor.
                clear,reason=(True,'') if mode=='explore' and trajectory_verified else clearance(scan,forward,yaw,side=side)
                if not clear:
                    if mode=='explore' and blocked_attempts<10:
                        self._drive(run_id,0,0);blocked_attempts+=1
                        explorer.note_blocked(reason)
                        with self.app.lock:
                            if self.run_id==run_id:self.message='前方暂不可通行，正在选择其他探索点'
                        time.sleep(.2);continue
                    raise ValueError(reason)
                blocked_attempts=0
                with self.app.lock:
                    if not self.active or self.run_id!=run_id:return
                    if mode=='explore':self.map_view=dict(explorer.view(),controller=trajectory_detail)
                    self.state='exploring' if mode=='explore' else 'following'
                    self.message=('转向补充侧向地面观测' if scanning else (getattr(explorer,'motion_hint','') or {'expand':'正在扩展原地图未扫描区域','refine':'正在复查已扫描区域，积累地图更新','observe':'正在前往新的观察位置补扫'}[explorer.target_kind])) if mode=='explore' else '跟随选中的人 · 以选中时的距离为参考'
                driven=self._drive(run_id,forward,yaw,valid_until,side=side)
                if driven is False:return
                if driven is None:
                    sensing_lost=sensing_lost or time.monotonic();recovery_frames=0;recovery_stamp=None
                    if time.monotonic()-sensing_lost>5:raise ValueError('感知持续无法及时完成，已停车；请查看时序记录')
                    continue
                sensing_lost=None;recovery_frames=0
                time.sleep(.04)
        except Exception as exc:
            with self.app.lock:
                if self.run_id==run_id:
                    self.pause(str(exc));self.app.add_log('autonomy',str(exc))
        finally:
            if pump:pump.cancel()

    def _save_current(self,token):
        return self._save_token is token and self.generation==token['generation'] and self.app.hub.same_context(token['context'])

    def finish_save(self,run_id=None,message='已停车，正在保存地图'):
        with self.app.lock:
            if run_id and self.run_id!=run_id:return
            if self._save_token and self._save_current(self._save_token):
                result=self._save_token.get('result')
                if result:return result
                raise ValueError('地图保存请求正在提交')
            pending=self.app.hub.preparation_job(save=True)
            if not self.app.mapping and not pending:raise ValueError('当前没有正在采集或准备的地图')
            self.pause(message)
            token=dict(id=uuid.uuid4().hex,generation=self.generation,context=self.app.hub.context(),capture_id=self.capture_id,
                       preparation=self.app.hub.preparation if pending else None)
            self._save_token=token;self.state='saving';self.message=message
            try:
                stop=dict(action='stop',request_id=token['id'])
                if token['capture_id']:stop['expected_start_request_id']=token['capture_id']
                result=pending or self.app.hub.post('/api/maps/library/build',stop)
                token['result']=result
            except Exception as exc:
                if self._save_current(token):
                    self._save_token=None;self.state='failed';self.message='保存请求未成功：'+str(exc)
                raise
        def wait_saved():
            try:
                deadline=time.monotonic()+300
                while time.monotonic()<deadline and not self.app.done:
                    with self.app.lock:
                        if not self._save_current(token):return
                    job=self.app.hub.operations.get(result['job_id'])
                    if job['status']=='failed':raise ValueError(job['error'])
                    if job['status']=='completed':
                        if job.get('result',{}).get('cleanup')=='not_collecting':
                            with self.app.lock:
                                if not self._save_current(token):return
                                self.state='paused';self.message='准备已取消，未进入采集，无需保存';self._save_token=None
                            return
                        if token['preparation']:token['capture_id']=token['preparation'].get('request_id')
                        status=self.app.hub.remote_maps('/maps/build/status',context=token['context'],timeout=8)
                        with self.app.lock:
                            if not self._save_current(token):return
                            if token['capture_id'] and status.get('mapping',{}).get('request_id')!=token['capture_id']:
                                raise ValueError('采集归属已变化，请在地图库核对原保存请求；未采用另一采集的结果')
                            self.app.mapping=bool(status.get('in_progress'))
                        if status.get('phase')=='error':raise ValueError(status['message'])
                        if status.get('phase')=='saved':
                            with self.app.lock:
                                if not self._save_current(token):return
                                self.state='completed';self.message=status.get('message') or '地图已保存，可在地图库中检查结果'
                                self.result_maps=status.get('result_maps',[]);self._save_token=None
                            return
                    time.sleep(.2)
                raise ValueError('地图保存仍在进行，请到建图页查看实际进度')
            except Exception as exc:
                with self.app.lock:
                    if self._save_current(token):
                        self._save_token=None;self.state='failed';self.message=str(exc)
        threading.Thread(target=wait_saved,daemon=True).start()
        return result

    def post(self,path,req):
        if path=='/api/autonomy/start':return self.start(req)
        if path=='/api/autonomy/lease':
            with self.app.lock:
                if self.active and req.get('run_id')==self.run_id:self.lease=time.monotonic()
            return dict(ok=True)
        if path=='/api/autonomy/stop':
            with self.app.lock:self.pause()
            return dict(ok=True)
        if path=='/api/autonomy/save':return self.finish_save()
        if path=='/api/autonomy/vision/start':return self.vision.start(str(req.get('model_path','')))
        if path=='/api/autonomy/vision/stop':
            with self.app.lock:
                if self.mode=='follow':self.pause('人员识别已停止')
                self.vision.stop()
            return dict(ok=True)
        return None

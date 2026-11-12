"""Robot-resident mission policy and measured-route return; no hardware I/O.

The caller supplies fresh onboard observations and acknowledges emitted actions.
Losing the PC heartbeat is independent of losing onboard perception/control.
"""
import copy
import hashlib
import json
import math
import re
import threading
import uuid

POLICIES={'stop':'断联停车','finish':'断联后完成任务','return':'断联后沿原路返回',
          'finish_return':'断联后完成任务并原路返回'}
TERMINAL={'idle','completed','cancelled','failed','interrupted'}


def finite(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError('任务坐标和时间必须为有限数值')
    return float(value)


def point(value):
    if not isinstance(value,dict):raise ValueError('任务点格式无效')
    x,y,yaw=(finite(value.get(k)) for k in ('x','y','yaw'))
    if max(abs(x),abs(y))>10000 or not -math.pi<=yaw<=math.pi:raise ValueError('任务点超出范围')
    if type(value.get('type',1)) is not int or value.get('type',1) not in (0,1):raise ValueError('断联任务暂不支持充电或特殊类型点')
    return dict(x=x,y=y,yaw=yaw,type=1)


def identity(value):
    if not isinstance(value,str) or not re.fullmatch('[a-f0-9]{32}',value):raise ValueError('任务身份无效')
    return value


def specification(request):
    if not isinstance(request,dict):raise ValueError('任务请求格式无效')
    policy=request.get('disconnect_policy','stop')
    if not isinstance(policy,str) or policy not in POLICIES:raise ValueError('未知断联策略')
    points=request.get('points')
    if not isinstance(points,list) or not 1<=len(points)<=200:raise ValueError('路线需要1到200个点')
    map_=mission_map(request.get('map',{}))
    timeout=finite(request.get('disconnect_timeout_s',5))
    duration=finite(request.get('max_duration_s',1800))
    if not 3<=timeout<=30 or not 30<=duration<=3600:raise ValueError('断联等待或任务时限超出范围')
    return dict(id=identity(request.get('id')),owner=identity(request.get('owner')),
                disconnect_policy=policy,disconnect_timeout_s=timeout,max_duration_s=duration,
                map=map_,points=[point(p) for p in points])


def mission_map(map_):
    if not isinstance(map_,dict):raise ValueError('地图身份格式无效')
    if (not isinstance(map_.get('name'),str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',map_['name'])
            or type(map_.get('grid_id')) is not int or not 0<=map_['grid_id']<=999
            or any(not re.fullmatch('[a-f0-9]{64}',str(map_.get(k,''))) for k in ('sha256','yaml_sha256'))):
        raise ValueError('需要已核对的活动地图、栅格和指纹')
    resolution=finite(map_.get('resolution'));origin=map_.get('origin')
    if not 0<resolution<=1 or not isinstance(origin,list) or len(origin)!=3:raise ValueError('地图分辨率或原点无效')
    origin=[finite(v) for v in origin]
    if abs(origin[2])>1e-6:raise ValueError('暂不支持带原点旋转的地图')
    return dict(name=map_['name'],grid_id=map_['grid_id'],sha256=map_['sha256'],
                yaml_sha256=map_['yaml_sha256'],resolution=resolution,origin=origin)


class MeasuredTrace:
    """Ordered measurements, preserving loops/corners; never a shortest path."""
    def __init__(self,capacity=20000):self.points=[];self.capacity=capacity;self.last_stamp=None
    def add(self,pose,stamp):
        pose=[finite(v) for v in pose];stamp=finite(stamp)
        if len(pose)!=3:raise ValueError('轨迹需要XY和朝向')
        if self.last_stamp is not None and stamp<=self.last_stamp:return
        if self.points:
            old=self.points[-1];distance=math.dist(old[:2],pose[:2])
            if distance>1.:raise ValueError('定位出现跳变，返回轨迹不可确认')
            angle=abs(math.atan2(math.sin(pose[2]-old[2]),math.cos(pose[2]-old[2])))
            if distance<.25 and angle<math.radians(12):self.last_stamp=stamp;return
        if len(self.points)>=self.capacity:raise ValueError('返回轨迹容量已满，已请求停车')
        self.points.append(pose);self.last_stamp=stamp
    def reverse(self,pose):
        if not self.points:raise ValueError('缺少实际行走轨迹，不能原路返回')
        if math.dist(pose[:2],self.points[-1][:2])>1.:raise ValueError('当前位置与已走轨迹不连续')
        result=[];previous=list(pose)
        for p in reversed(self.points):
            if math.dist(previous[:2],p[:2])<.08:continue
            yaw=math.atan2(p[1]-previous[1],p[0]-previous[0])
            result.append(dict(x=p[0],y=p[1],yaw=yaw,type=1));previous=p
        home=self.points[0]
        if not result or math.dist([result[-1]['x'],result[-1]['y']],home[:2])>.01:
            result.append(dict(x=home[0],y=home[1],yaw=home[2],type=1))
        else:result[-1]['yaw']=home[2]
        return result
    def distance(self,pose):
        if not self.points:return math.inf
        best=math.dist(pose[:2],self.points[0][:2])
        for a,b in zip(self.points,self.points[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
            t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length)) if length else 0.
            best=min(best,math.hypot(pose[0]-a[0]-t*dx,pose[1]-a[1]-t*dy))
        return best


class ResidentMission:
    def __init__(self,save=lambda receipt:None):
        self.lock=threading.RLock();self.save=save;self.job=None;self.revoked=set();self.trace=MeasuredTrace()
        self.phase='idle';self.generation=0;self.pending=None;self.goal=None;self.last_pulse=0
    def public(self):
        with self.lock:
            if not self.job:return dict(protocol=1,phase=self.phase,active=False)
            row={k:copy.deepcopy(v) for k,v in self.job.items() if k not in ('owner','points','digest','return_points','cancel_receipt','owned_values')}
            return dict(row,protocol=1,phase=self.phase,active=self.phase not in TERMINAL,
                        trace_points=len(self.trace.points),goal=copy.deepcopy(self.goal),
                        stop_confirmed=self.phase in ('completed','cancelled','failed') and self.pending is None)
    def _persist(self):
        self.save(dict(job=copy.deepcopy(self.job),phase=self.phase,trace=copy.deepcopy(self.trace.points),
                       revoked=sorted(self.revoked)))
    def restore(self,receipt):
        """After service restart, never resume movement from a saved receipt."""
        with self.lock:
            self.revoked=set(receipt.get('revoked',[]));self.job=receipt.get('job');self.trace.points=receipt.get('trace',[])
            saved=receipt.get('phase')
            self.phase=saved if self.job and saved in ('completed','cancelled','failed') else 'interrupted' if self.job else 'idle'
            self.pending=None;self.goal=None
            if self.phase=='interrupted':self.job.update(reconciliation_required=True,message='驻机服务曾中断，先核对并确认停车；不会自动恢复任务')
    def start(self,request,frame,now):
        spec=specification(request);now=finite(now)
        digest=hashlib.sha256(json.dumps(spec,sort_keys=True).encode()).hexdigest()
        with self.lock:
            if spec['id'] in self.revoked:raise ValueError('本次启动已撤销')
            if self.job and self.job['id']==spec['id']:
                if digest!=self.job['digest']:raise ValueError('重复任务参数不一致')
                return self.public()
            if self.phase not in TERMINAL or self.job and self.job.get('reconciliation_required'):
                raise ValueError('旧任务或停止状态尚未确认')
            self._validate(frame,spec['map'])
            if frame.get('external_task'):raise ValueError('已有其他导航任务，未接管')
            self.trace=MeasuredTrace();self.trace.add(frame['pose'],frame['stamp'])
            self.generation+=1;self.phase='forward';self.pending=None;self.goal=None
            self.job=dict(spec,digest=digest,started=now,deadline=now+spec['max_duration_s'],
                          index=0,completed=0,lost_once=False,return_index=0,message='机器人端执行导航任务')
            self.last_pulse=now;self._persist();return self.public()
    def pulse(self,mission,owner,now):
        with self.lock:
            self._owner(mission,owner);self.last_pulse=finite(now);return self.public()
    def _owner(self,mission,owner):
        if not self.job or self.job['id']!=mission or self.job['owner']!=owner:raise ValueError('任务或控制端身份不匹配')
    def cancel(self,mission,owner,now):
        with self.lock:
            identity(mission);identity(owner)
            if self.job and self.job['id']==mission:
                self._owner(mission,owner)
                if self.phase not in ('completed','cancelled','failed'):self._stop('cancelled','操作者已取消任务',now)
            else:
                if len(self.revoked)>=10000:raise ValueError('撤销记录已满，请维护驻机服务')
                self.revoked.add(mission);self._persist()
            return self.public()
    def _validate(self,frame,map_):
        if frame.get('map')!=map_:raise ValueError('活动地图或栅格内容已变化')
        if frame.get('ok') is not True:raise ValueError(frame.get('error','机器人本地状态未就绪'))
        if not 0<=finite(frame.get('age',99))<=.65:raise ValueError('本地定位或感知过期')
        if frame.get('normal') is not True or frame.get('safe') is not True:raise ValueError('定位、急停或控制状态异常')
        if len(frame.get('pose',[]))!=3:raise ValueError('缺少定位')
        [finite(v) for v in frame['pose']];finite(frame['stamp'])
    def _stop(self,after,message,now):
        if self.phase=='stopping':
            if after!='return':self.job['after_stop']=after;self.job['message']=message
            return
        self.generation+=1;self.phase='stopping';self.pending=None;self.goal=None
        self.job.update(after_stop=after,message=message,stop_started=now);self._persist()
    def _action(self,kind,**fields):
        action=dict(token=uuid.uuid4().hex,generation=self.generation,kind=kind,**fields)
        self.pending=action;return copy.deepcopy(action)
    def accepts(self,action):
        with self.lock:return self.pending==action and action['generation']==self.generation
    def dispatched(self,action,receipt,now):
        with self.lock:
            if not self.accepts(action):return False
            self.pending=None
            if action['kind']=='cancel':self.job['cancel_receipt']=receipt
            else:self.goal=dict(action['point'],receipt=receipt,sent=now)
            self._persist();return True
    def fail(self,message,now):
        with self.lock:
            if self.job and self.phase not in TERMINAL:self._stop('failed',message,now)
    def dispatch_failed(self,action,message,now):
        with self.lock:
            if not self.accepts(action):return
            self.pending=None
            if action['kind']=='cancel':self.job['message']='停止发送未确认：'+message
            else:self._stop('failed','目标发送结果未确认：'+message,now)
    def step(self,frame,now):
        now=finite(now)
        with self.lock:
            if not self.job or self.phase in TERMINAL:return None
            if self.phase=='stopping':
                if 'cancel_receipt' not in self.job:
                    return None if self.pending else self._action('cancel')
                if frame.get('cancel_confirmed') is True and frame.get('still') is True:
                    self.job.pop('reconciliation_required',None)
                    after=self.job['after_stop'];self.job.pop('cancel_receipt',None)
                    if after=='return':
                        try:
                            self._validate(frame,self.job['map'])
                            self.trace.add(frame['pose'],frame['stamp']);self.job['return_points']=self.trace.reverse(frame['pose'])
                        except ValueError as error:
                            self.phase='failed';self.job.update(message=str(error));self._persist();return None
                        self.phase='returning';self.job['return_index']=0;self.job['message']='正在沿实际行走轨迹返回起点'
                    else:
                        self.phase=after
                        if after=='completed':self.job['message']='任务已完成，停车已确认'
                        elif after=='cancelled':self.job['message']='任务已取消，停车已确认'
                    self.pending=None;self._persist()
                elif now-self.job['stop_started']>8:self.job['message']='停止状态尚未确认，请现场核对；不会继续下发目标'
                return None
            try:
                self._validate(frame,self.job['map'])
                if now>=self.job['deadline']:raise ValueError('任务时间已到，已请求停车')
                if frame.get('manual_override'):raise ValueError('手动接管或机器人已取消任务')
                if self.phase=='forward':self.trace.add(frame['pose'],frame['stamp'])
                elif self.trace.distance(frame['pose'])>.6:raise ValueError('返程偏离已走路线，已请求停车')
            except ValueError as error:self._stop('failed',str(error),now);return self._action('cancel')
            if now-self.last_pulse>=self.job['disconnect_timeout_s'] and not self.job['lost_once']:
                self.job['lost_once']=True;policy=self.job['disconnect_policy']
                if policy in ('stop','return'):
                    self._stop('return' if policy=='return' else 'cancelled',POLICIES[policy],now)
                    return self._action('cancel')
                self.job['message']=POLICIES[policy];self._persist()
            if self.goal:
                # Transport ACKs cannot complete a point. The adapter must match
                # the goal value, fresh terminal receipt and measured arrival.
                if frame.get('goal_complete') is not True:return None
                home=(self.phase=='returning' and self.job['return_index']==len(self.job['return_points'])-1)
                yaw_error=abs(math.atan2(math.sin(frame['pose'][2]-self.goal['yaw']),math.cos(frame['pose'][2]-self.goal['yaw'])))
                if (math.dist(frame['pose'][:2],[self.goal['x'],self.goal['y']])>(.2 if home else .4)
                        or home and yaw_error>.35):
                    self._stop('failed','到点回报与实际定位不一致',now);return self._action('cancel')
                self.goal=None
                key='index' if self.phase=='forward' else 'return_index';self.job[key]+=1
                if self.phase=='forward':self.job['completed']=self.job['index']
                self._persist()
            points=self.job['points'] if self.phase=='forward' else self.job['return_points']
            index=self.job['index'] if self.phase=='forward' else self.job['return_index']
            if index>=len(points):
                returning=(self.phase=='forward' and self.job['lost_once'] and self.job['disconnect_policy']=='finish_return')
                self._stop('return' if returning else 'completed','任务目标已完成，正在确认停车',now)
                return self._action('cancel')
            if self.pending:return None
            return self._action('goal',point=points[index],returning=self.phase=='returning')

"""Preview adapter and bounded translational start-recovery execution policy.

Pure computation: no network or actuator I/O. The owning PointNavigator sends
commands under its existing run/context/observation leases. Prediction bounds
are explicit test assumptions, not calibration or a hardware guarantee.
"""
import copy
import math
import time
import numpy as np
from start_pose_recovery import recover_start,check_recovery_step,_Geometry,_validate,_wrap


def _xy(p):return np.array([p['x'],p['y']],float)


def _translation_prefix(recovery):
    prefix=recovery['recovery_dense_path'];anchor=np.array([prefix[0][k] for k in ['x','y','yaw']],float)
    goal=_xy(prefix[-1]);delta=goal-anchor[:2];length=float(np.linalg.norm(delta))
    if any(abs(_wrap(p['yaw']-anchor[2]))>1e-5 for p in prefix):raise ValueError('此起点恢复需要近墙旋转，当前自动恢复只执行已核验的平移')
    if length<1e-6:return anchor,goal,None,0.
    direction=delta/length
    projected=np.array([float((_xy(p)-anchor[:2])@direction) for p in prefix])
    cross=np.array([float(abs(np.cross(direction,_xy(p)-anchor[:2]))) for p in prefix])
    if np.max(cross)>.003 or np.any(np.diff(projected)<-.001):raise ValueError('此起点恢复需要多段变向，当前自动恢复只执行单向平移并读回停稳')
    return anchor,goal,direction,length


def plan_with_recovery(planner,start,goal,yaw,scan,loc,standing,*,budget=1.2,standard_error=None):
    """GridPlanner-compatible result plus optional translation-only recovery.

    `standard_error` may be the just-caught standard planner exception to avoid
    repeating the same work. An `ok` result is a preview; execution recomputes it.
    """
    began=time.monotonic()
    if standard_error is None:
        try:
            plan=planner.plan(start,goal,yaw,budget=min(.35,budget))
            return dict(plan,recovery=dict(required=False,preview_only=True))
        except ValueError as exc:standard_error=exc
    recovery=recover_start(planner,start,goal,yaw,scan,loc,standing=standing,max_turn=0.,
                           budget=max(.05,budget-(time.monotonic()-began)))
    if not recovery['ok']:
        message=recovery['reason']
        if recovery.get('last_route_error'):message+='；原目标：'+recovery['last_route_error']
        error=ValueError(message);error.recovery=recovery;raise error
    try:anchor,target,direction,length=_translation_prefix(recovery)
    except ValueError as exc:exc.recovery=recovery;raise
    global_plan=copy.deepcopy(recovery['global_plan'])
    prefix=recovery['recovery_dense_path'];dense=prefix+global_plan['dense_path'][1:]
    path=[]
    for p in dense:
        if not path or math.hypot(p['x']-path[-1]['x'],p['y']-path[-1]['y'])>1e-8:path.append(dict(x=p['x'],y=p['y']))
    metadata={k:copy.deepcopy(recovery[k]) for k in ['classification','reason','requested_start','requested_goal','support_seed','constraints']}
    metadata.update(required=length>.005,mode='translation_only',preview_only=True,prefix=prefix,
                    recovery_distance_m=recovery.get('recovery_distance_m',0.),global_plan=global_plan)
    return dict(global_plan,path=path,dense_path=dense,length_m=round(global_plan['length_m']+length,2),
                recovery=metadata,planner='footprint_any_angle_v3_start_recovery',
                message='已规划短程平移退让后接续原目标；执行前将按新扫描重新核验')


class TranslationRecovery:
    """Continuous leased input, feedback braking, and verified stopping envelope.

    Only one fixed-direction translation is supported. It first holds zero for
    250 ms with fresh stillness, then uses a useful normalized amplitude rather
    than assuming a sequence of tiny pulses equals physical travel. Each lease
    is at most 250 ms; the full command + braking envelope is verified.
    """
    def __init__(self,recovery,now=None,command_limit=.25,min_command=.20,response_gain_bound=2.5,braking_decel=.35):
        if not recovery.get('required'):raise ValueError('恢复计划没有需要执行的平移')
        inner=dict(recovery_dense_path=recovery['prefix'])
        self.anchor,self.target,self.direction,self.length=_translation_prefix(inner)
        self.seed=copy.deepcopy(recovery['support_seed']);self.constraints=copy.deepcopy(recovery['constraints'])
        self.started=time.monotonic() if now is None else float(now);self.last_stamp=None;self.last_pose=self.anchor.copy()
        self.last_now=self.started;self.travel=0.;self.turn=0.;self.progress=0.;self.progress_at=self.started
        self.settle_at=None;self.settle_pose=None;self.initial_settled=False;self.braking=False;self.completed=False
        self.limit=float(command_limit);self.minimum=float(min_command);self.gain_bound=float(response_gain_bound)
        self.decel=float(braking_decel);self.reaction=.08;self.lease=.25;self.last_axes=np.zeros(3)
        if not .15<=self.minimum<=self.limit<=.35 or not 2.<=self.gain_bound<=3. or not .2<=self.decel<=.8:raise ValueError('恢复控制模型参数超出验证范围')
        self.last={}

    def _result(self,axes,state,reason,now,scan,envelope=None):
        self.last_axes=np.array(axes,float)
        until=min(float(scan.get('_valid_until',now+.65-float(scan.get('age',.65)))),now+self.lease)
        if not any(axes):until=now
        self.last=dict(axes=list(map(float,axes)),state=state,reason=reason,valid_until=until,
                       travelled_m=round(self.travel,4),progress_m=round(self.progress,4),remaining_m=round(max(0.,self.length-self.progress),4),
                       envelope=envelope or [],model=dict(response_gain_bound=self.gain_bound,braking_decel_min=self.decel,
                       reaction_s=self.reaction,lease_s=self.lease,hardware_calibrated=False))
        return dict(self.last)

    def _pose_at(self,base,delta):return [base[0]+delta[0],base[1]+delta[1],base[2]]

    def _envelope(self,geometry,pose,world_v,command_world=None):
        """Cover measured-velocity stopping and a bounded worst-response command."""
        trajectories=[]
        for commanded in [None]+([command_world] if command_world is not None else []):
            pos=np.array(pose[:2],float);v=np.asarray(world_v,float).copy();path=[list(pose)];t=0.
            hold=self.reaction+(self.lease if commanded is not None else 0.)
            # The immediate worst-response branch upper-bounds fast acceleration;
            # the separate measured-velocity branch covers residual momentum.
            if commanded is not None:v=np.asarray(commanded,float).copy()
            total=hold+float(np.linalg.norm(v))/self.decel+.04;dt=.02
            while t<total:
                old=pos.copy();speed=float(np.linalg.norm(v))
                if t>=hold and speed>0:v*=max(0.,speed-self.decel*dt)/speed
                pos+=v*dt;t+=dt
                q=[float(pos[0]),float(pos[1]),pose[2]]
                if math.dist(q[:2],self.anchor[:2])>self.constraints.get('max_distance_m',.8)+1e-8:return None
                if abs(np.cross(self.direction,pos-self.anchor[:2]))>.07:return None
                # The complete predicted stopping path obeys the SAME body and
                # non-decreasing clearance test as the execution prefix.
                if geometry.edge([float(old[0]),float(old[1]),pose[2]],q,relaxed=not geometry.standard([float(old[0]),float(old[1]),pose[2]])) is None:return None
                path.append(q)
            trajectories.append(path)
        return max(trajectories,key=len)

    def command(self,planner,scan,loc,yaw_rate=0.,standing=False,now=None):
        now=time.monotonic() if now is None else float(now);pose=np.asarray(scan.get('pose',[]),float)
        zero=[0.,0.,0.]
        if self.completed:return self._result(zero,'complete','起点恢复已完成',now,scan)
        if now-self.started>8:return self._result(zero,'failed','起点恢复达到8秒期限，已请求停车',now,scan)
        try:
            error=_validate(scan,loc,list(pose),now)
            if error:return self._result(zero,'failed',error,now,scan)
            if not standing:return self._result(zero,'failed','站立反馈已变化',now,scan)
            if scan.get('stamp')==self.last_stamp:return self._result(zero,'waiting','等待新的独立扫描',now,scan)
            velocity=np.asarray(scan.get('_velocity_body'),float)
            if velocity.shape!=(2,) or not np.isfinite(velocity).all() or not math.isfinite(yaw_rate):return self._result(zero,'failed','运动反馈无效',now,scan)
            speed=float(np.linalg.norm(velocity));c,s=math.cos(pose[2]),math.sin(pose[2]);world_v=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1]])
            moved=float(np.linalg.norm(pose[:2]-self.last_pose[:2]))
            # Stationary localization jitter is not travelled distance or turn.
            # Absolute displacement from the anchor is still checked below.
            if speed>=.08:self.travel+=moved
            if abs(yaw_rate)>=.12:self.turn+=abs(_wrap(float(pose[2]-self.last_pose[2])))
            self.last_stamp=scan['stamp'];self.last_pose=pose.copy();self.last_now=now
            along=float((pose[:2]-self.anchor[:2])@self.direction);cross=abs(float(np.cross(self.direction,pose[:2]-self.anchor[:2])))
            if along>self.progress+.01:self.progress_at=now
            self.progress=max(self.progress,along)
            if self.travel>.8 or math.dist(pose[:2],self.anchor[:2])>.8 or self.turn>math.radians(20) or abs(_wrap(pose[2]-self.anchor[2]))>math.radians(20) or cross>.07 or along<-.025:
                return self._result(zero,'failed','实际移动偏离已核验恢复范围，已请求停车',now,scan)
            if speed>self.gain_bound*self.limit+.08:return self._result(zero,'failed','实际速度超过恢复预测范围，已请求停车',now,scan)
            geometry=_Geometry(planner,list(pose),scan,.8,standing,support_anchor=self.anchor,support_seed=self.seed)
            if not geometry.physical(pose):return self._result(zero,'failed','新的障碍或地面观测不再支持退让',now,scan)
            current_check=check_recovery_step(planner,list(pose),list(pose),scan,loc,list(self.anchor),standing=True,support_seed=self.seed)
            if not current_check['ok']:return self._result(zero,'failed',current_check['reason'],now,scan)
            if not self.initial_settled:
                if speed<.08 and abs(yaw_rate)<.12:
                    if self.settle_at is None:self.settle_at=now;self.settle_pose=pose.copy()
                    if math.dist(pose[:2],self.settle_pose[:2])>.01 or abs(_wrap(pose[2]-self.settle_pose[2]))>.02:self.settle_at=now;self.settle_pose=pose.copy()
                    if now-self.settle_at>=.25:self.initial_settled=True;self.progress_at=now
                else:self.settle_at=None
                return self._result(zero,'settling','先确认实际停稳，再执行短程平移',now,scan)
            if abs(yaw_rate)>.12 or abs(_wrap(pose[2]-self.anchor[2]))>.08:
                self.braking=True
                return self._result(zero,'braking','正在制动，恢复阶段不叠加近墙转向',now,scan)
            remaining=self.length-along
            stopping=speed*self.reaction+speed*speed/(2*self.decel)
            if remaining<=max(.018,stopping+.012):self.braking=True
            if self.braking:
                if speed<.08:
                    if along>=self.length-.035 and geometry.standard(pose):
                        self.completed=True
                        return self._result(zero,'complete','已读回标准净空及停稳，可从实际位置接续原目标',now,scan)
                    self.braking=False
                else:
                    envelope=self._envelope(geometry,pose,world_v)
                    if envelope is None:return self._result(zero,'failed','新障碍或缺测地面进入停车包络，已请求停车',now,scan)
                    return self._result(zero,'braking','按实际速度提前制动，等待停稳读回',now,scan,envelope)
            if now-self.progress_at>2:
                return self._result(zero,'failed','恢复指令未产生可确认位移，不继续假定小脉冲已经到位',now,scan)
            if speed>=.08 and float(world_v@self.direction)<.5*speed:
                return self._result(zero,'braking','先消除原有运动，再沿已核验方向退让',now,scan)
            for magnitude in list(dict.fromkeys([self.limit,self.minimum])):
                command_world=self.direction*magnitude*self.gain_bound
                # Residual very-low feedback after the settled check is treated
                # as the existing <0.08 m/s stopped-state tolerance.
                actual=world_v if speed>=.08 else np.zeros(2)
                envelope=self._envelope(geometry,pose,actual,command_world)
                if envelope is None:continue
                axes=[magnitude*(c*self.direction[0]+s*self.direction[1]),magnitude*(-s*self.direction[0]+c*self.direction[1]),0.]
                next_pose=self._pose_at(pose,self.direction*min(.025,max(.005,remaining)))
                step=check_recovery_step(planner,list(pose),next_pose,scan,loc,list(self.anchor),standing=True,support_seed=self.seed)
                if step['ok']:return self._result(axes,'moving','按实际位移连续退让，保持可停车包络',now,scan,envelope)
            return self._result(zero,'failed','当前空间不足以容纳指令持有及制动包络，未发送退让运动',now,scan)
        except (ValueError,TypeError,KeyError,OverflowError) as exc:return self._result(zero,'failed',str(exc),now,scan)

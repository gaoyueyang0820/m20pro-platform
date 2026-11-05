"""Body-frame person-following feasibility, not target association or navigation.

Consumes PersonLock's image-scale servo request; never infers metric distance
from a person box, changes targets, commands reverse, or invents ground. Pure
computation: the caller owns robot readiness, target lock and leased execution.
"""
import math
import time
from collections import Counter
import numpy as np
from m20.robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH


def _wrap(v):return math.atan2(math.sin(v),math.cos(v))
def _direction(p):
    x,y=p
    if abs(x)>2*abs(y):return 'front' if x>=0 else 'rear'
    if abs(y)>2*abs(x):return 'left' if y>=0 else 'right'
    return ('front' if x>=0 else 'rear')+('_left' if y>=0 else '_right')


class _Surface:
    def __init__(self,scan):
        self.res=float(scan.get('resolution',.2));self.floor=set(map(tuple,scan['floor']))
        cells=list(map(tuple,scan['obstacles']));bounds=np.asarray(scan.get('obstacle_bounds',[]),float).reshape(-1,4)
        precise=scan.get('geometry_version')==2 and len(bounds)==len(cells) and np.isfinite(bounds).all() and bool(np.all(bounds[:,2:]>=bounds[:,:2]))
        if precise:self.floor.update(map(tuple,scan.get('measured_floor',[])))
        else:
            self.floor.difference_update(cells)
            bounds=np.array([[(x-.5)*self.res,(y-.5)*self.res,(x+.5)*self.res,(y+.5)*self.res] for x,y in cells],float).reshape(-1,4)
        self.bounds=bounds.copy()
        if len(bounds):self.bounds[:,:2]-=.005;self.bounds[:,2:]+=.005
        # Only the CURRENT occupied body cells have the existing stance-support
        # assumption. Do not expand a 0.55 m disk, interpolate, or add ray space.
        self.stance=set(self._cells(0,0,0,BODY_HALF_LENGTH,BODY_HALF_WIDTH));self.cache={}

    def _cells(self,x,y,yaw,hx,hy):
        c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s);half=self.res/2
        rx=hx*a+hy*b+half;ry=hx*b+hy*a+half
        for ix in range(math.ceil((x-rx)/self.res),math.floor((x+rx)/self.res)+1):
            for iy in range(math.ceil((y-ry)/self.res),math.floor((y+ry)/self.res)+1):
                dx,dy=ix*self.res-x,iy*self.res-y
                if abs(c*dx+s*dy)<hx+half*(a+b)-1e-9 and abs(-s*dx+c*dy)<hy+half*(a+b)-1e-9:yield ix,iy

    def check(self,pose,padding=0.):
        x,y,yaw=pose;key=(round(x,5),round(y,5),round(yaw,5),round(padding,5))
        if key in self.cache:return self.cache[key]
        c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s)
        hx,hy=CLEARANCE_HALF_LENGTH+padding,CLEARANCE_HALF_WIDTH+padding
        hit=[]
        if len(self.bounds):
            centres=(self.bounds[:,:2]+self.bounds[:,2:])/2;half=(self.bounds[:,2:]-self.bounds[:,:2])/2
            dx,dy=centres[:,0]-x,centres[:,1]-y
            intersect=(abs(dx)<hx*a+hy*b+half[:,0])&(abs(dy)<hx*b+hy*a+half[:,1])
            intersect&=(abs(c*dx+s*dy)<hx+half[:,0]*a+half[:,1]*b)&(abs(-s*dx+c*dy)<hy+half[:,0]*b+half[:,1]*a)
            hit=np.flatnonzero(intersect).tolist()
        missing=[q for q in self._cells(x,y,yaw,BODY_HALF_LENGTH+padding,BODY_HALF_WIDTH+padding) if q not in self.floor and q not in self.stance]
        result=dict(clear=not hit and not missing,collision_indices=hit,missing_cells=missing)
        self.cache[key]=result;return result

    def occluded(self,cell):
        # The 2D summary does not distinguish positive objects from depressions.
        # This only marks a possible foreground obstruction, never proof that
        # an occluder is a person, and NEVER grants traversability.
        end=np.asarray(cell,float)*self.res
        if not len(self.bounds):return False
        for box in self.bounds:
            lo,hi=0.,1.
            for axis in [0,1]:
                if abs(end[axis])<1e-9:
                    if not box[axis]<=0<=box[axis+2]:lo,hi=1.,0.;break
                else:
                    a,b=box[axis]/end[axis],box[axis+2]/end[axis]
                    lo=max(lo,min(a,b));hi=min(hi,max(a,b))
            if hi>=lo and hi>=0 and lo<1:return True
        return False

    def diagnostic(self,checks):
        cells=sorted({tuple(p) for check in checks for p in check.get('missing_cells',[])},key=lambda p:math.hypot(*p))
        collisions=sorted({i for check in checks for i in check.get('collision_indices',[])})
        obstructed=[p for p in cells if self.occluded(p)];counts=Counter(_direction(p) for p in cells)
        kind='obstacle_in_sweep' if collisions else 'occluded_support' if cells and len(obstructed)==len(cells) else 'mixed_support_gap' if obstructed else 'unobserved_support' if cells else None
        return dict(reason_code=kind,missing=dict(count=len(cells),cells=[list(p) for p in cells[:32]],resolution=self.res,
                    directions=dict(counts),occlusion_possible_cells=[list(p) for p in obstructed[:32]],person_association='unverified'),
                    obstacles=dict(count=len(collisions),bounds=[self.bounds[i].tolist() for i in collisions[:16]]))


class FollowControl:
    def __init__(self,command_hold_s=.25,response_gain_bound=2.5):
        self.hold=float(command_hold_s);self.gain=float(response_gain_bound)
        if not .05<=self.hold<=.3 or not 1<=self.gain<=3:raise ValueError('跟随预测参数超出范围')
        self.vision_sequence=None;self.vision_deadline=0.;self.scan_stamp=None;self.scan_deadline=0.;self.target_id=None

    def _result(self,axes,state,reason,code,now,deadline,diagnostic=None,candidates=None,trajectory=None):
        diagnostic=diagnostic or dict(missing=dict(count=0,cells=[],directions={},occlusion_possible_cells=[],person_association='unverified'),obstacles=dict(count=0,bounds=[]))
        moving=any(axes);mode=state
        state={'following':'tracking','aligning':'tracking','slowing':'tracking','waiting':'waiting_ground','target_unavailable':'stale'}.get(state,state)
        if code=='motion_feedback_missing':state='waiting_feedback'
        if code in ['observation_expiring','invalid_or_stale_observation']:state='stale'
        detail=dict(missing=diagnostic['missing'],obstacles=diagnostic['obstacles'],candidates=candidates or [],person_association='unverified')
        return dict(ok=True,axes=list(map(float,axes)),state=state,mode=mode,reason=reason,reason_code=code,motion_allowed=moving,diagnostics=detail,
                    reselection_required=code in ['target_lock_lost','target_changed'],
                    trajectory_verified=True,valid_until=min(deadline,now+self.hold) if moving else now,
                    missing=diagnostic['missing'],obstacles=diagnostic['obstacles'],candidate_diagnostics=candidates or [],trajectory=trajectory or [],
                    distance_m=None,target_reference='selected_image_scale_not_metres',target_id=self.target_id,
                    model=dict(response_gain_bound=self.gain,linear_braking_mps2=.5,angular_braking_radps2=1.2,hardware_calibrated=False))

    def _rollout(self,surface,axes,feedback,age,hold):
        all_checks=[];representative=[];max_angle=0.;max_translation=0.
        if axes[0]>0:
            nose=math.ceil(BODY_HALF_LENGTH/surface.res)
            probes=[(nose,y) for y in range(-math.floor(BODY_HALF_WIDTH/surface.res),math.floor(BODY_HALF_WIDTH/surface.res)+1)]
            if not any(q in surface.floor for q in probes):
                return dict(clear=False,checks=[dict(missing_cells=probes,collision_indices=[])],trajectory=[],max_angle_deg=0.,max_translation_m=0.)
        branches=['feedback'] if not any(axes) else ['ramped','instant']
        for branch in branches:
            pose=np.zeros(3);velocity=feedback.copy();trajectory=[pose.tolist()];lead=age+.08
            target=np.array([axes[0],0.,axes[2]],float)*self.gain
            total=lead+hold+max(float(np.linalg.norm(velocity[:2])),float(np.linalg.norm(target[:2])))/.5+max(abs(velocity[2]),abs(target[2]))/1.2+.08
            if total>4:return dict(clear=False,checks=[dict(missing_cells=[],collision_indices=[])],reason='speed_envelope_too_long',trajectory=[],max_angle_deg=0.,max_translation_m=0.)
            elapsed=0.;checked=surface.check(pose);all_checks.append(checked)
            if not checked['clear']:return dict(clear=False,checks=all_checks,trajectory=trajectory,max_angle_deg=0.,max_translation_m=0.)
            while elapsed<total:
                dt=min(.025,total-elapsed);old=pose.copy()
                if elapsed>=lead+hold:
                    desired=np.zeros(3);limits=np.array([.5,.5,1.2])*dt
                    velocity+=np.clip(desired-velocity,-limits,limits)
                elif elapsed>=lead:
                    if branch=='instant':velocity=target.copy()
                    else:
                        limits=np.array([.8,.8,1.8])*dt;velocity+=np.clip(target-velocity,-limits,limits)
                theta=pose[2]+velocity[2]*dt/2;c,s=math.cos(theta),math.sin(theta)
                pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*dt
                elapsed+=dt
                middle=(old+pose)/2;pad=float(np.linalg.norm(pose[:2]-old[:2]))/2+math.hypot(CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)*abs(pose[2]-old[2])/2
                checked=surface.check(middle,pad);all_checks.append(checked);trajectory.append(pose.tolist())
                max_angle=max(max_angle,abs(float(pose[2])));max_translation=max(max_translation,float(np.linalg.norm(pose[:2])))
                if not checked['clear']:return dict(clear=False,checks=all_checks,trajectory=trajectory,max_angle_deg=math.degrees(max_angle),max_translation_m=max_translation)
            if len(trajectory)>len(representative):representative=trajectory
        return dict(clear=True,checks=all_checks,trajectory=representative,max_angle_deg=math.degrees(max_angle),max_translation_m=max_translation)

    def choose(self,scan,requested,*,vision,yaw_rate=0.,now=None):
        now=time.monotonic() if now is None else float(now);zero=[0.,0.,0.];deadline=now
        try:
            if not vision.get('locked',False):return self._result(zero,'target_unavailable','人员锁定已失效，请沿用原流程重新点选目标','target_lock_lost',now,now)
            if not vision.get('fresh',False):return self._result(zero,'target_unavailable','等待新鲜人员识别帧，不延续旧动作','vision_expired',now,now)
            age=float(vision.get('age',math.inf));sequence=vision.get('sequence')
            if not math.isfinite(age) or not 0<=age<=.65 or isinstance(sequence,bool) or not isinstance(sequence,(int,float)) or not math.isfinite(sequence):
                return self._result(zero,'target_unavailable','人员识别帧已经过期或序号无效','vision_expired',now,now)
            target_id=vision.get('target_id')
            if self.target_id is not None and target_id!=self.target_id:return self._result(zero,'target_unavailable','锁定人员标识已变化，不自动切换目标','target_changed',now,now)
            if self.target_id is None:self.target_id=target_id
            if self.vision_sequence is not None and sequence<self.vision_sequence:return self._result(zero,'target_unavailable','收到旧人员识别帧，不恢复旧动作','vision_reordered',now,now)
            candidate=now+.65-age
            self.vision_deadline=min(self.vision_deadline,candidate) if sequence==self.vision_sequence else candidate;self.vision_sequence=sequence
            if scan.get('protocol')!=1 and not scan.get('demo'):raise ValueError('雷达观测协议未核对')
            if scan.get('frame_id','base_link' if scan.get('demo') else None)!='base_link':raise ValueError('跟随雷达不是机身坐标，未发送动作')
            scan_age=float(scan.get('age',math.inf));stamp=float(scan['stamp'])
            if not all(math.isfinite(v) for v in [scan_age,stamp]) or not 0<=scan_age<=.65:raise ValueError('雷达观测过期')
            if self.scan_stamp is not None and stamp<self.scan_stamp:raise ValueError('雷达源帧回退')
            candidate=min(now+.65-scan_age,float(scan.get('_valid_until',now+.65-scan_age)))
            self.scan_deadline=min(self.scan_deadline,candidate) if stamp==self.scan_stamp else candidate;self.scan_stamp=stamp
            deadline=min(self.vision_deadline,self.scan_deadline)
            if deadline-now<.05:return self._result(zero,'waiting','等待新的雷达或识别帧，旧帧不延长运动期限','observation_expiring',now,deadline)
            forward,yaw=map(float,requested)
            if not all(math.isfinite(v) for v in [forward,yaw,yaw_rate]) or not 0<=forward<=1 or abs(yaw)>1:raise ValueError('跟随请求无效，不自动倒退')
            if forward>.35 or abs(yaw)>.7:raise ValueError('跟随请求超出当前短程模型范围')
            velocity=np.asarray(scan.get('_velocity_body'),float)
            if velocity.shape!=(2,) or not np.isfinite(velocity).all():return self._result(zero,'waiting','等待新鲜实际速度反馈','motion_feedback_missing',now,deadline)
            feedback=np.array([velocity[0],velocity[1],yaw_rate],float)
            surface=_Surface(scan);hold=min(self.hold,deadline-now)
            candidates=[('requested',[forward,0.,yaw])]
            if yaw:
                if forward:candidates.append(('align_first',[0.,0.,yaw]))
                candidates.append(('gentle_align',[0.,0.,yaw*.5]))
            if forward:
                candidates.append(('reduced_translation',[forward*.5,0.,yaw]))
                if yaw:candidates.append(('reduced_arc',[forward*.5,0.,yaw*.5]))
            candidates.append(('wait',zero));reports=[];first_issue=None;seen=set()
            for name,axes in candidates:
                key=tuple(axes)
                if key in seen:continue
                seen.add(key)
                check=self._rollout(surface,axes,feedback,scan_age,hold if any(axes) else 0.)
                detail=surface.diagnostic(check['checks']);summary=dict(name=name,axes=axes,clear=check['clear'],max_angle_deg=round(check['max_angle_deg'],3),max_translation_m=round(check['max_translation_m'],3),**detail)
                reports.append(summary)
                if not check['clear'] and first_issue is None:first_issue=detail
                if not check['clear']:continue
                if not any(axes):
                    moving=np.linalg.norm(feedback[:2])>=.08 or abs(feedback[2])>=.12
                    if not forward and not yaw:state='braking' if moving else 'holding';reason='目标无需继续靠近，正在停车' if moving else '保持当前人员画面尺度参考，不推断米制距离'
                    else:state='braking' if moving else 'waiting';reason='已停车等待目标方向的障碍或地面观测恢复'
                    return self._result(zero,state,reason,(first_issue or {}).get('reason_code') or 'holding',now,deadline,first_issue,reports,check['trajectory'])
                state='aligning' if not axes[0] else 'following' if name=='requested' else 'slowing'
                reason='按当前实测范围做短时跟随' if name=='requested' else '先做可通行的小角度对准，暂不前进' if not axes[0] else '缩短前进，只在已测地面内跟随'
                return self._result(axes,state,reason,name,now,deadline,first_issue,reports,check['trajectory'])
            diagnostic=first_issue or dict(missing=dict(count=0,cells=[],directions={},occlusion_possible_cells=[],person_association='unverified'),obstacles=dict(count=0,bounds=[]))
            code=diagnostic.get('reason_code') or 'stopping_envelope_unavailable'
            reason={'obstacle_in_sweep':'短时转向或停车包络内有实测障碍，已请求停车',
                    'occluded_support':'缺测位于前景障碍后方，可能受遮挡；未确认是人员，不将该区域当作地面',
                    'mixed_support_gap':'动作包络同时存在遮挡后方缺测和直接缺测，等待新的地面观测',
                    'unobserved_support':'实际动作包络内有未观测地面，已请求停车'}.get(code,'当前速度下无法核验停车包络，已请求停车')
            return self._result(zero,'braking' if np.linalg.norm(feedback[:2])>=.08 or abs(feedback[2])>=.12 else 'waiting',reason,code,now,deadline,diagnostic,reports)
        except (ValueError,TypeError,KeyError,OverflowError) as exc:return self._result(zero,'waiting',str(exc),'invalid_or_stale_observation',now,deadline)

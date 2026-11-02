"""Deterministic receding-horizon path following for normalized M20 axes.

Independent implementation of sampled-trajectory scoring, inspired by Nav2's
path/obstacle critics (see reference/autonomy-v3-sources.md). This is not MPPI,
nor a trained policy. Predictions are kinematic, never hardware calibration.
All candidate footprints stay on currently measured floor. No actuator I/O.
"""
import math
import time
import numpy as np
from robot_geometry import CLEARANCE_HALF_LENGTH, CLEARANCE_HALF_WIDTH, BODY_HALF_LENGTH, BODY_HALF_WIDTH

RES=.2


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class LocalSurface:
    """Continuous rectangular footprint against a discrete live surface."""
    def __init__(self,scan):
        from autonomy_core import supported_cells, disk
        self.free=supported_cells(scan)|set(disk(.55))
        self.blocked={tuple(p) for p in scan['obstacles']}
        bounds=np.asarray(scan.get('obstacle_bounds',[]),float).reshape(-1,4)
        self.precise=scan.get('geometry_version')==2 and len(bounds)==len(scan['obstacles']) and np.isfinite(bounds).all() and bool(np.all(bounds[:,2:]>=bounds[:,:2]))
        self.bounds=bounds
        if self.precise:self.free.update(tuple(p) for p in scan.get('measured_floor',[]))
        self.cache={}

    def valid(self,x,y,yaw):
        # SAT rectangle/cell intersection, with sub-cell translation retained.
        c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s);half=RES/2
        hx,hy=CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH
        if self.precise and len(self.bounds):
            centers=(self.bounds[:,:2]+self.bounds[:,2:])/2
            halfsize=(self.bounds[:,2:]-self.bounds[:,:2])/2+.005
            dx,dy=centers[:,0]-x,centers[:,1]-y
            hit=(abs(dx)<hx*a+hy*b+halfsize[:,0])&(abs(dy)<hx*b+hy*a+halfsize[:,1])
            hit&=(abs(c*dx+s*dy)<hx+halfsize[:,0]*a+halfsize[:,1]*b)&(abs(-s*dx+c*dy)<hy+halfsize[:,0]*b+halfsize[:,1]*a)
            if hit.any():return False
        rx=hx*a+hy*b+half;ry=hx*b+hy*a+half
        for ix in range(math.ceil((x-rx)/RES),math.floor((x+rx)/RES)+1):
            for iy in range(math.ceil((y-ry)/RES),math.floor((y+ry)/RES)+1):
                dx,dy=ix*RES-x,iy*RES-y;u,v=abs(c*dx+s*dy),abs(-s*dx+c*dy)
                if u>=hx+half*(a+b)-1e-8 or v>=hy+half*(a+b)-1e-8:continue
                if not self.precise and (ix,iy) in self.blocked:return False
                if (u<BODY_HALF_LENGTH+half*(a+b)-1e-8 and v<BODY_HALF_WIDTH+half*(a+b)-1e-8
                    and abs(dx)<BODY_HALF_LENGTH*a+BODY_HALF_WIDTH*b+half-1e-8
                    and abs(dy)<BODY_HALF_LENGTH*b+BODY_HALF_WIDTH*a+half-1e-8
                    and (ix,iy) not in self.free):return False
        return True


def dense_polyline(path,spacing=.12):
    result=[]
    for p in path:
        xy=np.array([p['x'],p['y']] if isinstance(p,dict) else p[:2],dtype=float)
        if not np.isfinite(xy).all():raise ValueError('路径包含无效坐标')
        if result:
            if np.linalg.norm(xy-result[-1])<1e-8:continue
            first=result[-1].copy();n=max(1,math.ceil(float(np.linalg.norm(xy-first))/spacing))
            result.extend(first+(xy-first)*i/n for i in range(1,n+1))
        else:result.append(xy)
    return np.array(result,dtype=float).reshape(-1,2)


class TrajectoryFollower:
    def __init__(self,forward_limit=.35,yaw_limit=.7):
        self.forward_limit=float(forward_limit);self.yaw_limit=float(yaw_limit)
        self.previous=np.zeros(3);self.gain=np.array([1.,1.,1.])
        self.path=np.empty((0,2));self.index=0;self.plan_key=None;self.last={}
        self.path_arc=np.empty(0);self.progress_pose=None;self.goal_braking=False

    def set_path(self,path):
        self.path=dense_polyline(path);self.index=0
        if not len(self.path):raise ValueError('路径为空')
        self.path_arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(self.path,axis=0),axis=1))]
        self.progress_pose=None;self.goal_braking=False

    def _feedback(self,scan,yaw_rate):
        v=scan.get('_velocity_body',[0.,0.])
        if v is None or len(v)!=2:return None
        feedback=np.array([v[0],v[1],yaw_rate],dtype=float)
        if not np.isfinite(feedback).all():return None
        # Adapt predicted motion to sustained observed response, not commanded
        # joystick magnitude. At rest, retain the nominal conservative model.
        for i in range(3):
            if abs(self.previous[i])>.12 and feedback[i]*self.previous[i]>.01:
                ratio=np.clip(abs(feedback[i]/self.previous[i]),.3,2.)
                self.gain[i]=.85*self.gain[i]+.15*ratio
        return feedback

    def command(self,scan,yaw_rate=0.,goal_yaw=None,allowed_pose=None):
        started=time.perf_counter();pose=np.asarray(scan['pose'],dtype=float)
        if len(pose)!=3 or not np.isfinite(pose).all():raise ValueError('位姿无效')
        feedback=self._feedback(scan,yaw_rate)
        if feedback is None:return self._result([0,0,0],'waiting_feedback','等待新鲜运动反馈',[],started)
        if not len(self.path):return self._result([0,0,0],'idle','没有待跟踪路径',[],started)
        # Advance only as far along the ordered path as observed displacement
        # supports. A nearby goal across a wall/loop is not a shortcut in progress.
        displacement=0. if self.progress_pose is None else float(np.linalg.norm(pose[:2]-self.progress_pose))
        advance=min(.75,max(.25,displacement+.15))
        end=min(len(self.path),int(np.searchsorted(self.path_arc,self.path_arc[self.index]+advance,side='right'))+1)
        ds=np.linalg.norm(self.path[self.index:end]-pose[:2],axis=1)
        self.index+=int(np.argmin(ds));self.progress_pose=pose[:2].copy()
        projected=[]
        for i in range(max(0,self.index-1),min(len(self.path)-1,self.index+1)):
            segment=self.path[i+1]-self.path[i];length=float(np.linalg.norm(segment))
            if length<1e-9:continue
            fraction=float(np.clip((pose[:2]-self.path[i])@segment/(length*length),0.,1.))
            residual=float(np.linalg.norm(pose[:2]-(self.path[i]+fraction*segment)))
            projected.append((residual,float(self.path_arc[i]+fraction*length)))
        travelled=min(projected)[1] if projected else float(self.path_arc[self.index])
        remaining_path=max(0.,float(self.path_arc[-1])-travelled)
        remain=self.path[self.index:]
        distance=float(np.linalg.norm(self.path[-1]-pose[:2]))
        if travelled>=float(self.path_arc[-1])-.001:remaining_path=max(remaining_path,distance)
        speed=float(np.linalg.norm(feedback[:2]));angular_speed=abs(float(feedback[2]))
        age=float(scan.get('_timing',{}).get('age_ms',float(scan.get('age',0.))*1000))/1000
        reaction=.15+min(.65,max(0.,age));deceleration=.5
        stopping=speed*reaction+speed*speed/(2*deceleration)
        at_goal=distance<.18 and remaining_path<.18
        heading_error=0. if goal_yaw is None else abs(wrap(goal_yaw-pose[2]))
        still=speed<.08 and angular_speed<.12
        self.last.update(remaining_path_m=round(remaining_path,3),stopping_distance_m=round(stopping,3),
                         arrival_tolerance_m=.18,stopping_model=dict(deceleration_min=deceleration,reaction_s=reaction,hardware_calibrated=False))
        if at_goal and heading_error<.14:
            if still:
                self.goal_braking=False
                return self._result([0,0,0],'arrived','已到达目标并读回停稳',[],started)
            self.goal_braking=True
            return self._result([0,0,0],'braking','已进入目标范围，正在制动；尚未确认停稳',[],started)
        if self.goal_braking:
            if not still:return self._result([0,0,0],'braking','正在制动，等待实际速度下降后再接近终点',[],started)
            self.goal_braking=False
        if speed>=.08 and (at_goal or remaining_path<=stopping+.10):
            self.goal_braking=True
            return self._result([0,0,0],'braking','按剩余路径与实际速度提前制动',[],started)
        if at_goal and goal_yaw is not None and feedback[2]*wrap(goal_yaw-pose[2])>0:
            angular_stopping=angular_speed*reaction+angular_speed**2/(2*1.2)
            if angular_speed>=.12 and heading_error<=angular_stopping+.025:
                self.goal_braking=True
                return self._result([0,0,0],'braking','终点朝向接近，正在制动转向',[],started)
        c,s=math.cos(pose[2]),math.sin(pose[2]);delta=remain-pose[:2]
        local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(local,axis=0),axis=1))]
        look=max(.65,min(1.4,.65+speed*.8))
        target=local[min(len(local)-1,int(np.searchsorted(arc,look)))]
        desired=wrap(goal_yaw-pose[2]) if at_goal and goal_yaw is not None else math.atan2(target[1],target[0])
        surface=LocalSurface(scan)
        f=self.forward_limit;t=self.yaw_limit
        terminal=remaining_path<.9
        if terminal:
            # Solve v*T + v^2/(2*a) <= available distance, using a 2x input
            # response floor until the measured gain is known. Preserve useful
            # input instead of asymptotically commanding below the dead zone;
            # the feedback braking branch above finishes the approach.
            available=max(0.,remaining_path-.10)
            allowed_speed=max(0.,math.sqrt((deceleration*reaction)**2+2*deceleration*available)-deceleration*reaction)
            response=max(2.,float(np.max(self.gain[:2])))
            minimum=min(.20,self.forward_limit)
            f=min(self.forward_limit,max(minimum,allowed_speed/response))
            self.last['terminal_speed_limit_mps']=round(allowed_speed,3)
        side=min(.25,f);reverse=min(.25,f)
        slow_f=max(min(.20,self.forward_limit),.7*f) if terminal else .7*f
        candidates=[(0.,0.,0.)]
        if not at_goal:
            for vx in list(dict.fromkeys([f,slow_f,0.,-reverse])):
                for vy in [0.,side,-side]:
                    for wz in [0.,t,-t]:
                        if vx==0 and vy==0 and wz==0:continue
                        candidates.append((vx,vy,wz))
        else:candidates.extend([(0,0,t),(0,0,-t)])
        candidates=np.asarray(candidates,dtype=float);velocities=candidates*self.gain
        # Every trajectory starts at observed velocity and decelerates before
        # reversing or changing translation axis. Commands changing >60 degrees
        # while moving are replaced with a brake cycle, never a sideways jerk.
        n=len(candidates);steps=14;dt=.12
        # Retain useful joystick amplitude near the goal, shorten its predicted
        # duration instead of choosing a sub-dead-zone command or never arriving.
        if remaining_path<.8:dt=max(.025,min(dt,.8*remaining_path/max(.1,f)/steps))
        if at_goal:dt=max(.025,min(dt,abs(desired)/max(.1,t)/steps))
        trajectories=np.zeros((n,steps,3));velocity=np.tile(feedback,(n,1));valid=np.ones(n,dtype=bool)
        speed=np.linalg.norm(feedback[:2])
        if speed>.08:
            lengths=np.linalg.norm(velocities[:,:2],axis=1)
            dot=velocities[:,:2]@feedback[:2]
            valid[1:] &= (lengths[1:]>0)&(dot[1:]>.5*lengths[1:]*speed)
        cost=np.zeros(n);obstacles=np.asarray(list(surface.blocked),float).reshape(-1,2)*RES
        terminal_braking=np.zeros(n,dtype=bool)
        endpoint_path=self.path[max(0,self.index-1):]-pose[:2]
        endpoint_path=np.column_stack((c*endpoint_path[:,0]+s*endpoint_path[:,1],-s*endpoint_path[:,0]+c*endpoint_path[:,1]))
        endpoint_segments=np.diff(endpoint_path,axis=0);endpoint_lengths=np.linalg.norm(endpoint_segments,axis=1)
        endpoint_arc=np.r_[0.,np.cumsum(endpoint_lengths)]
        for k in range(steps):
            old=trajectories[:,k-1] if k else np.zeros((n,3))
            target_velocity=velocities.copy()
            if terminal and not at_goal and len(endpoint_segments):
                relative=old[:,None,:2]-endpoint_path[None,:-1,:]
                fraction=np.clip(np.sum(relative*endpoint_segments[None,:,:],axis=2)/np.maximum(1e-9,endpoint_lengths**2),0.,1.)
                projections=endpoint_path[None,:-1,:]+fraction[:,:,None]*endpoint_segments[None,:,:]
                closest=np.argmin(np.linalg.norm(old[:,None,:2]-projections,axis=2),axis=1)
                along=endpoint_arc[closest]+fraction[np.arange(n),closest]*endpoint_lengths[closest]
                predicted_remaining=np.maximum(0.,endpoint_arc[-1]-along)
                predicted_speed=np.linalg.norm(velocity[:,:2],axis=1)
                terminal_braking |= predicted_remaining<=predicted_speed*reaction+predicted_speed**2/(2*deceleration)+.10
                target_velocity[terminal_braking,:2]=0.
                if k==0:candidates[terminal_braking,:2]=0.;velocities[terminal_braking,:2]=0.
            # A finite turn followed by braking, not an assumption that a
            # useful 70% yaw command must be held for the entire horizon. The
            # latter rejected small necessary heading changes near a corner.
            remaining=(desired-old[:,2]+math.pi)%(2*math.pi)-math.pi
            turn_done=(target_velocity[:,2]*remaining<=0)|(np.abs(remaining)<velocity[:,2]**2/3.6+.025)
            target_velocity[turn_done,2]=0
            if k==0:
                candidates[turn_done,2]=0;velocities[turn_done,2]=0
            change=np.clip(target_velocity-velocity,[-.7*dt,-.7*dt,-1.8*dt],[.7*dt,.7*dt,1.8*dt])
            change[terminal_braking,:2]=np.clip(target_velocity[terminal_braking,:2]-velocity[terminal_braking,:2],-deceleration*dt,deceleration*dt)
            velocity+=change
            theta=old[:,2]+velocity[:,2]*dt*.5
            trajectories[:,k,0]=old[:,0]+(np.cos(theta)*velocity[:,0]-np.sin(theta)*velocity[:,1])*dt
            trajectories[:,k,1]=old[:,1]+(np.sin(theta)*velocity[:,0]+np.cos(theta)*velocity[:,1])*dt
            trajectories[:,k,2]=old[:,2]+velocity[:,2]*dt
            for i in np.flatnonzero(valid):
                px,py,angle=trajectories[i,k]
                if not surface.valid(px,py,angle):valid[i]=False;continue
                if allowed_pose is not None and not allowed_pose(pose[0]+c*px-s*py,pose[1]+s*px+c*py,wrap(pose[2]+angle)):
                    valid[i]=False
        for i in np.flatnonzero(valid):
            track=trajectories[i];last=track[-1]
            distances=np.linalg.norm(local-last[:2],axis=1);closest=int(np.argmin(distances))
            progress=arc[closest]
            align=abs(wrap(desired-last[2]))
            clearance=0.
            if len(obstacles):
                nearest=float(np.linalg.norm(track[:,None,:2]-obstacles[None,:,:],axis=2).min())
                clearance=.12/max(.12,nearest)
            cost[i]=(3.2*float(np.linalg.norm(target-last[:2]))+1.4*distances[closest]-.8*progress
                     +(.95 if at_goal else .45)*align+clearance+.10*np.linalg.norm(candidates[i]-self.previous)
                     +(.12 if candidates[i,0]<0 else 0)+.08*abs(candidates[i,1]))
        cost[~valid]=np.inf
        best=int(np.argmin(cost));self.last['candidate_count']=n;self.last['valid_candidates']=int(valid.sum())
        if not np.isfinite(cost[best]):
            state='braking' if speed>=.08 or angular_speed>=.12 else 'blocked'
            return self._result([0,0,0],state,'当前扫描内没有可通行轨迹，先请求停车再重规划',[],started)
        # Brake is always available even if its future stopping envelope has
        # become obstructed. A zero result is never called progress.
        axes=candidates[best]
        if best and np.isfinite(cost[0]) and cost[best]>=cost[0]-.025:best=0;axes=candidates[0]
        state='tracking' if np.linalg.norm(axes) else 'braking' if speed>=.08 or angular_speed>=.12 else 'blocked'
        reason='连续跟踪通路' if state=='tracking' else '正在制动，等待实际停稳' if state=='braking' else '等待通路或重规划'
        if axes[0]<0:reason='后退调整到可通行轨迹'
        elif abs(axes[1])>.05:reason='侧移绕开障碍'
        elif axes[2]:reason='沿弧线转向' if axes[0] else '转向下一段通路'
        return self._result(axes,state,reason,trajectories[best].tolist(),started)

    def _result(self,axes,state,reason,trajectory,started):
        self.previous=np.array(axes,dtype=float)
        self.last.update(axes=[float(x) for x in axes],state=state,reason=reason,trajectory=trajectory,
                         path_index=self.index,path_points=len(self.path),compute_ms=round((time.perf_counter()-started)*1000,2),
                         velocity_gain=self.gain.tolist(),model='kinematic_feedback_adapted_not_calibrated')
        return dict(self.last)

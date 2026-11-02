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

    def set_path(self,path):
        self.path=dense_polyline(path);self.index=0
        if not len(self.path):raise ValueError('路径为空')

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
        # Keep monotonic progress on this path; do not jump across nearby loops.
        end=min(len(self.path),self.index+35)
        ds=np.linalg.norm(self.path[self.index:end]-pose[:2],axis=1)
        self.index+=int(np.argmin(ds))
        remain=self.path[self.index:]
        distance=float(np.linalg.norm(self.path[-1]-pose[:2]))
        at_goal=distance<.18 and self.index>=len(self.path)-3
        if at_goal and (goal_yaw is None or abs(wrap(goal_yaw-pose[2]))<.14):
            return self._result([0,0,0],'arrived','已到达目标，正在停车',[],started)
        c,s=math.cos(pose[2]),math.sin(pose[2]);delta=remain-pose[:2]
        local=np.column_stack((c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]))
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(local,axis=0),axis=1))]
        look=max(.65,min(1.4,.65+np.linalg.norm(feedback[:2])*.8))
        target=local[min(len(local)-1,int(np.searchsorted(arc,look)))]
        desired=wrap(goal_yaw-pose[2]) if at_goal and goal_yaw is not None else math.atan2(target[1],target[0])
        surface=LocalSurface(scan)
        f=self.forward_limit;t=self.yaw_limit;side=min(.25,f);reverse=min(.25,f)
        candidates=[(0.,0.,0.)]
        if not at_goal:
            for vx in [f,.7*f,0.,-reverse]:
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
        if distance<.8:dt=max(.025,min(dt,.8*distance/max(.1,f)/steps))
        if at_goal:dt=max(.025,min(dt,abs(desired)/max(.1,t)/steps))
        trajectories=np.zeros((n,steps,3));velocity=np.tile(feedback,(n,1));valid=np.ones(n,dtype=bool)
        speed=np.linalg.norm(feedback[:2])
        if speed>.08:
            lengths=np.linalg.norm(velocities[:,:2],axis=1)
            dot=velocities[:,:2]@feedback[:2]
            valid[1:] &= (lengths[1:]>0)&(dot[1:]>.5*lengths[1:]*speed)
        cost=np.zeros(n);obstacles=np.asarray(list(surface.blocked),float).reshape(-1,2)*RES
        for k in range(steps):
            old=trajectories[:,k-1] if k else np.zeros((n,3))
            target_velocity=velocities.copy()
            # A finite turn followed by braking, not an assumption that a
            # useful 70% yaw command must be held for the entire horizon. The
            # latter rejected small necessary heading changes near a corner.
            remaining=(desired-old[:,2]+math.pi)%(2*math.pi)-math.pi
            turn_done=(target_velocity[:,2]*remaining<=0)|(np.abs(remaining)<velocity[:,2]**2/3.6+.025)
            target_velocity[turn_done,2]=0
            if k==0:
                candidates[turn_done,2]=0;velocities[turn_done,2]=0
            velocity+=np.clip(target_velocity-velocity,[-.7*dt,-.7*dt,-1.8*dt],[.7*dt,.7*dt,1.8*dt])
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
        if not np.isfinite(cost[best]):return self._result([0,0,0],'blocked','当前扫描内没有可通行轨迹，需要重规划',[],started)
        # Brake is always available even if its future stopping envelope has
        # become obstructed. A zero result is never called progress.
        axes=candidates[best]
        if best and np.isfinite(cost[0]) and cost[best]>=cost[0]-.025:best=0;axes=candidates[0]
        state='tracking' if np.linalg.norm(axes) else 'blocked'
        reason='连续跟踪通路' if state=='tracking' else '等待通路或重规划'
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

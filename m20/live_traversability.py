"""Short-lived, evidence-backed traversability over an immutable occupancy map.

This is a local execution layer, never a map edit or a localization correction.
Measured ground can open local passages. The current physical body projection
can independently remove old-map residue strictly inside the occupied body;
this exception neither infers unseen ground nor opens space ahead of the robot.
A separate live obstacle layer always wins. All methods are local, without I/O.
"""
import math
import time
from collections import deque
import numpy as np
from m20.robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH
from m20.navigation_validation import poses_consistent


class LiveTraversability:
    NEAR_RADIUS=2.0
    MAX_SCAN_AGE=.65
    MAX_POSE_AGE=1.0
    MIN_CONFIDENCE=.85
    MIN_FRAMES=3
    MIN_SPAN=.4
    EVIDENCE_WINDOW=2.0
    BLIND_RADIUS=.55

    def __init__(self,base_free,occupied,resolution,origin):
        self.base_free=base_free
        self.occupied=occupied
        self.res=float(resolution);self.origin=list(origin);self.h,self.w=base_free.shape
        self.free_cells=set();self.blocked_cells=set();self.body_projection_cells=set();self.body_history={};self.evidence={}
        self.last_stamp=None;self.last_pose=None;self.epoch=None;self.source_highwater=None
        self.confirmed_cells=set();self.last_update_at=0.;self.last_real_update_at=0.
        self.expires_at=0.;self.real_expires_at=0.;self.revision=0
        self.status=dict(accepted=False,state='waiting_evidence',reason='等待连续实测地面',structure={})

    def cell(self,x,y):return math.floor((x-self.origin[0])/self.res),self.h-1-math.floor((y-self.origin[1])/self.res)
    def world(self,x,y):return self.origin[0]+(x+.5)*self.res,self.origin[1]+(self.h-y-.5)*self.res
    def inside(self,p):return 0<=p[0]<self.w and 0<=p[1]<self.h

    def _publish(self,free,blocked,**status):
        if free!=self.free_cells or blocked!=self.blocked_cells:self.revision+=1
        self.free_cells=set(free);self.blocked_cells=set(blocked);self.status.update(status)
        return self.snapshot()

    def snapshot(self):
        return dict(self.status,temporary_free_cells=len(self.free_cells),blocked_cells=len(self.blocked_cells),
                    occupied_overrides=sum(bool(self.occupied[y,x]) for x,y in self.free_cells),
                    unknown_overrides=sum(not bool(self.occupied[y,x]) for x,y in self.free_cells),
                    body_projection_cells=len(self.body_projection_cells),body_projection_policy='current_physical_body_only',
                    body_projection_size_m=[2*BODY_HALF_LENGTH,2*BODY_HALF_WIDTH],
                    surface_revision=self.revision,source_stamp=self.last_stamp,expires_at=self.expires_at,
                    radius_m=self.NEAR_RADIUS,persistent_map_changed=False)

    def clear(self,reason,reset_source=False):
        self.evidence.clear();self.last_stamp=None;self.last_pose=None;self.confirmed_cells.clear();self.body_projection_cells.clear();self.body_history.clear()
        self.expires_at=0.;self.real_expires_at=0.
        if reset_source:self.source_highwater=None
        return self._publish(set(),set(),accepted=False,state='invalid',reason=reason,structure={},carried_under_body=0)

    def expire(self,now=None):
        # Execution permission and voting history have different lifetimes.
        # A 0.5 s-old scan has only 0.15 s of command validity, but its genuine
        # observation can still vote with the next 0.2 s-period source frame.
        real_now=time.monotonic();clock=real_now if now is None else float(now)
        deadline=self.real_expires_at if now is None else self.expires_at
        updated=self.last_real_update_at if now is None else self.last_update_at
        before=self.revision
        if clock>=deadline and (self.free_cells or self.blocked_cells):
            self.body_projection_cells.clear()
            self._publish(set(),set(),accepted=False,state='expired',reason='实时通行许可已过期，等待新扫描恢复',carried_under_body=0)
        if clock-updated>self.EVIDENCE_WINDOW:
            self.evidence.clear();self.confirmed_cells.clear();self.body_history.clear();self.last_pose=None;self.last_stamp=None
        return self.revision!=before

    @staticmethod
    def _wrap(v):return math.atan2(math.sin(v),math.cos(v))

    def _validate(self,scan,loc,real_now):
        if not isinstance(loc,dict) or loc.get('normal') is not True:raise ValueError('定位未正常，已撤销地图投影')
        conf=float(loc.get('confidence',-1));age=float(loc.get('age',math.inf))
        if not math.isfinite(conf) or not self.MIN_CONFIDENCE<=conf<=1:raise ValueError('定位质量不足，已撤销地图投影')
        if not math.isfinite(age) or not 0<=age<=self.MAX_POSE_AGE:raise ValueError('定位反馈过期，已撤销地图投影')
        if scan.get('protocol')!=1 or scan.get('frame_id')!='base_link' or scan.get('pose_frame_id')!='map':
            raise ValueError('扫描与地图坐标系未核对，已撤销临时通行')
        if scan.get('geometry_version')!=2 or 'measured_floor' not in scan:raise ValueError('缺少实测地面证据，已撤销临时通行')
        resolution=float(scan.get('resolution',.2))
        if not math.isfinite(resolution) or not .05<=resolution<=.3:raise ValueError('地面栅格分辨率无效')
        stamp=float(scan['stamp']);pose_stamp=float(scan['pose_stamp']);scan_age=float(scan.get('age',math.inf))
        if not all(math.isfinite(v) for v in [stamp,pose_stamp,scan_age]) or not 0<=scan_age<=self.MAX_SCAN_AGE:
            raise ValueError('雷达扫描过期，已撤销临时通行')
        if abs(stamp-pose_stamp)>.100001:raise ValueError('扫描与位姿时刻不匹配，已撤销临时通行')
        pose=[float(v) for v in scan['pose']];reported=[float(v) for v in loc['pose']]
        if len(pose)!=3 or len(reported)!=3 or not all(math.isfinite(v) for v in pose+reported):raise ValueError('位姿无效')
        if not poses_consistent(pose,scan_age,loc):raise ValueError('定位与扫描暂未同步')
        life=self.MAX_SCAN_AGE-scan_age
        if '_valid_until' in scan:life=min(life,float(scan['_valid_until'])-real_now)
        if not math.isfinite(life) or life<=0:raise ValueError('扫描控制期限已到，已撤销临时通行')
        return stamp,pose,resolution,life

    @staticmethod
    def _precise_bounds(scan):
        if scan.get('geometry_version')!=2:return None
        try:boxes=np.asarray(scan.get('obstacle_bounds',[]),float).reshape(-1,4)
        except (ValueError,TypeError):return None
        if len(boxes)!=len(scan['obstacles']) or not np.isfinite(boxes).all() or not np.all(boxes[:,2:]>=boxes[:,:2]):return None
        boxes=boxes.copy()
        if len(boxes):boxes[:,:2]-=.005;boxes[:,2:]+=.005
        return boxes

    @staticmethod
    def _ray_clear(x,y,boxes):
        """Segment/box slabs; only measured ground is tested, never ray-filled."""
        if boxes is None or not len(boxes):return True
        enter=np.zeros(len(boxes));leave=np.ones(len(boxes));possible=np.ones(len(boxes),bool)
        for axis,value in enumerate([x,y]):
            if abs(value)<1e-12:possible&=(boxes[:,axis]<=0)&(boxes[:,axis+2]>=0)
            else:
                a=boxes[:,axis]/value;b=boxes[:,axis+2]/value
                enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        return not bool(np.any(possible&(enter<=leave)))

    @staticmethod
    def _visible_floor(scan,resolution):
        if LiveTraversability._precise_bounds(scan) is not None:
            # A 20 cm cell can contain both measured floor and a small obstacle
            # outside the map cell being considered. Exact obstacle projection
            # and per-map-cell sight lines below decide which part is usable.
            return {tuple(p) for p in scan['measured_floor'] if math.hypot(*p)*resolution<=2.2}
        floor=set(map(tuple,scan['floor']))&set(map(tuple,scan['measured_floor']))
        obstacles=set(map(tuple,scan['obstacles']))
        floor-=obstacles
        # A supported point behind a currently seen obstacle does not establish
        # an unobstructed corridor. Do not infer floor from the ray itself.
        visible=set()
        for x,y in floor:
            if math.hypot(x,y)*resolution>2.2:continue
            n=max(abs(x),abs(y));blocked=False
            for i in range(1,max(1,n)):
                if (round(x*i/n),round(y*i/n)) in obstacles:blocked=True;break
            if not blocked:visible.add((x,y))
        return visible

    def _project_blocks(self,scan,pose,resolution):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);cells=set();anchors=[]
        boxes=scan.get('obstacle_bounds')
        if not isinstance(boxes,list) or len(boxes)!=len(scan['obstacles']):
            boxes=[[(qx-.5)*resolution,(qy-.5)*resolution,(qx+.5)*resolution,(qy+.5)*resolution] for qx,qy in scan['obstacles']]
        for box in boxes:
            if len(box)!=4 or not all(math.isfinite(float(v)) for v in box):continue
            x0,y0,x1,y1=map(float,box)
            if x1<x0 or y1<y0:continue
            bx,by=(x0+x1)/2,(y0+y1)/2
            if math.hypot(bx,by)>self.NEAR_RADIUS+.3:continue
            wx,wy=x+c*bx-s*by,y+s*bx+c*by;anchors.append((wx,wy))
            hx,hy=(x1-x0)/2,(y1-y0)/2;half=self.res/2
            ex=hx*abs(c)+hy*abs(s);ey=hx*abs(s)+hy*abs(c)
            left,top=self.cell(wx-ex-half,wy+ey+half);right,bottom=self.cell(wx+ex+half,wy-ey-half)
            for iy in range(max(0,top),min(self.h,bottom+1)):
                for ix in range(max(0,left),min(self.w,right+1)):
                    cx,cy=self.world(ix,iy);dx,dy=cx-wx,cy-wy
                    if (abs(dx)<=ex+half and abs(dy)<=ey+half and
                        abs(c*dx+s*dy)<=hx+half*(abs(c)+abs(s)) and
                        abs(-s*dx+c*dy)<=hy+half*(abs(c)+abs(s))):cells.add((ix,iy))
        return cells,anchors

    def _candidates(self,visible,blocked,pose,resolution,occluders=None):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);result=set();r=self.NEAR_RADIUS;half=self.res/2
        left,top=self.cell(x-r,y+r);right,bottom=self.cell(x+r,y-r)
        for iy in range(max(0,top),min(self.h,bottom+1)):
            for ix in range(max(0,left),min(self.w,right+1)):
                if self.base_free[iy,ix] or (ix,iy) in blocked:continue
                wx,wy=self.world(ix,iy)
                # The whole map cell, not only its centre, must be within the
                # current observed floor union and bounded near field.
                ok=True
                for dx,dy in [(0,0),(-half,-half),(-half,half),(half,-half),(half,half)]:
                    px,py=wx+dx-x,wy+dy-y;u,v=c*px+s*py,-s*px+c*py
                    if (math.hypot(u,v)>r or (math.floor(u/resolution+.5),math.floor(v/resolution+.5)) not in visible
                            or not self._ray_clear(u,v,occluders)):ok=False;break
                if ok:result.add((ix,iy))
        return result

    def _body_projection(self,pose,blocked):
        """Old occupied cells wholly inside the robot cannot be its new wall.

        Use the actual .84 x .45 m body, WITHOUT clearance inflation, the sensor
        blind radius, or future swept poses. Whole-cell containment prevents a
        boundary cell from clearing a strip outside the physical body. Unknown
        cells and current measured obstacles are never granted this exception.
        These cells are not measured-floor votes. A short departure history is
        kept separately so the same residue cannot catch the moving rear body.
        """
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s)
        hx,hy=BODY_HALF_LENGTH,BODY_HALF_WIDTH;half=self.res/2
        ex=hx*a+hy*b;ey=hx*b+hy*a
        left,top=self.cell(x-ex,y+ey);right,bottom=self.cell(x+ex,y-ey)
        cells=set()
        for iy in range(max(0,top),min(self.h,bottom+1)):
            for ix in range(max(0,left),min(self.w,right+1)):
                if not self.occupied[iy,ix] or (ix,iy) in blocked:continue
                wx,wy=self.world(ix,iy);dx,dy=wx-x,wy-y
                if abs(c*dx+s*dy)+half*(a+b)<=hx+1e-9 and abs(-s*dx+c*dy)+half*(a+b)<=hy+1e-9:
                    cells.add((ix,iy))
        return cells

    def _structure(self,anchors,candidates):
        """Check stable structure OUTSIDE the potential clearing region.

        Parallel jambs can support normal alignment at an ordinary door. Their
        tangential ambiguity is reported; this is never global-pose proof.
        """
        anchors=list(dict.fromkeys((round(x,3),round(y,3)) for x,y in anchors))
        if len(anchors)>200:anchors=anchors[::math.ceil(len(anchors)/200)]
        matches=[];normals=[];residuals=[];radius=max(1,math.ceil(.28/self.res));seen=set()
        for wx,wy in anchors:
            ix,iy=self.cell(wx,wy);best=None
            for yy in range(max(0,iy-radius),min(self.h,iy+radius+1)):
                for xx in range(max(0,ix-radius),min(self.w,ix+radius+1)):
                    if not self.occupied[yy,xx] or (xx,yy) in candidates:continue
                    mx,my=self.world(xx,yy);d=math.hypot(mx-wx,my-wy)
                    if best is None or d<best[0]:best=(d,xx,yy,mx,my)
            if best is None or best[0]>.2:continue
            residuals.append(best[0]);key=best[1:3]
            if key in seen:continue
            seen.add(key);matches.append((wx,wy))
            _,xx,yy,_,_=best;n=max(2,math.ceil(.4/self.res));pts=[]
            for oy in range(max(0,yy-n),min(self.h,yy+n+1)):
                for ox in range(max(0,xx-n),min(self.w,xx+n+1)):
                    if self.occupied[oy,ox] and (ox,oy) not in candidates:pts.append(self.world(ox,oy))
            if len(pts)>=5:
                cov=np.cov(np.asarray(pts).T);values,vectors=np.linalg.eigh(cov)
                if values[-1]>max(1e-8,3*values[0]):normals.append(vectors[:,0])
        ratio=len(residuals)/max(1,len(anchors));span=float(np.linalg.norm(np.ptp(matches,axis=0))) if len(matches)>1 else 0.
        supported=len(matches)>=8 and ratio>=.65 and span>=.8
        if normals:
            values=np.linalg.eigvalsh(np.asarray(normals).T@np.asarray(normals));ratio_eigen=float(values[0]/max(1e-9,values[-1]))
        else:ratio_eigen=0.
        return dict(supported=supported,anchor_count=len(matches),observations=len(anchors),inlier_ratio=round(ratio,3),span_m=round(span,3),
                    residual_p90_m=round(float(np.percentile(residuals,90)),4) if residuals else None,
                    observability='two_directions' if ratio_eigen>.15 else 'tangent_ambiguous',
                    global_localization_proven=False)

    def update(self,scan,localization,now=None):
        real_now=time.monotonic();now=real_now if now is None else float(now)
        self.expire(now)
        try:stamp,pose,resolution,life=self._validate(scan,localization,real_now)
        except (ValueError,TypeError,KeyError,OverflowError) as exc:return self.clear(str(exc))
        epoch=localization.get('epoch')
        if self.epoch is not None and epoch!=self.epoch:
            self.epoch=epoch;return self.clear('定位会话已变化，重新积累地面观测',reset_source=True)
        if self.source_highwater is not None:
            if stamp==self.source_highwater:
                self.status.update(accepted=False,reason='重复源帧不累计通行证据');return self.snapshot()
            if stamp<self.source_highwater:return self.clear('扫描时刻回退，重新积累地面观测')
        if self.last_stamp is not None:
            dt=min(.5,stamp-self.last_stamp)
            if math.dist(pose[:2],self.last_pose[:2])>.35+1.2*dt or abs(self._wrap(pose[2]-self.last_pose[2]))>.25+1.8*dt:
                return self.clear('定位发生跳变，已撤销临时通行')
        try:
            visible=self._visible_floor(scan,resolution)
            blocked,anchors=self._project_blocks(scan,pose,resolution)
            candidates=self._candidates(visible,blocked,pose,resolution,self._precise_bounds(scan))
        except (ValueError,TypeError,KeyError,OverflowError) as exc:return self.clear('地面证据格式无效：'+str(exc))
        current_body=self._body_projection(pose,blocked)
        self.body_history={p:at for p,at in self.body_history.items() if now-at[0]<=self.EVIDENCE_WINDOW and real_now-at[1]<=self.EVIDENCE_WINDOW and p not in blocked}
        # Previously wholly occupied cells stay usable while the real rear
        # body is still leaving them, even at very low measured speed. This
        # renews ONLY an existing body seed, never a newly touched wall cell.
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s);half=self.res/2
        for p in self.body_history:
            wx,wy=self.world(*p);dx,dy=wx-x,wy-y
            if (abs(dx)<BODY_HALF_LENGTH*a+BODY_HALF_WIDTH*b+half-1e-9 and
                abs(dy)<BODY_HALF_LENGTH*b+BODY_HALF_WIDTH*a+half-1e-9 and
                abs(c*dx+s*dy)<BODY_HALF_LENGTH+half*(a+b)-1e-9 and
                abs(-s*dx+c*dy)<BODY_HALF_WIDTH+half*(a+b)-1e-9):
                self.body_history[p]=(now,real_now)
        for p in current_body:self.body_history[p]=(now,real_now)
        body_projection=set(self.body_history)
        structure=self._structure(anchors,candidates|body_projection)
        self.evidence={p:q for p,q in self.evidence.items() if q and now-q[-1][1]<=self.EVIDENCE_WINDOW and p not in blocked}
        for p in candidates:
            q=self.evidence.setdefault(p,deque(maxlen=12));q.append((stamp,now,real_now))
            while q and now-q[0][1]>self.EVIDENCE_WINDOW:q.popleft()
        approved=set()
        for p in candidates:
            q=self.evidence[p]
            if len(q)>=self.MIN_FRAMES and q[-1][0]-q[0][0]>=self.MIN_SPAN-1e-6:
                if not self.occupied[p[1],p[0]] or structure['supported']:approved.add(p)
        # Carry only previously confirmed floor while it is occluded by the
        # robot itself. Never invent unseen ground under an initially bad pose.
        carried=set()
        for p in self.confirmed_cells-approved-blocked:
            wx,wy=self.world(*p)
            # A map cell partly entering the sensor's circular blind zone
            # already loses full-cell measurement support. Carry its previously
            # proven area, rather than revoke at the centre-only boundary.
            dx=max(0.,abs(wx-pose[0])-self.res/2);dy=max(0.,abs(wy-pose[1])-self.res/2)
            if math.hypot(dx,dy)<=self.BLIND_RADIUS:carried.add(p)
        approved|=carried
        self.confirmed_cells=set(approved)
        self.body_projection_cells=body_projection
        approved|=body_projection
        self.source_highwater=stamp
        self.last_stamp=stamp;self.last_pose=pose;self.epoch=epoch;self.last_update_at=now;self.last_real_update_at=real_now
        self.expires_at=now+life;self.real_expires_at=real_now+life
        pending=sum(bool(self.occupied[y,x]) for x,y in candidates-approved)
        reason=('旧占据局部投影尚未核对，仅更新已确认未知地面和实测障碍' if pending and not structure['supported'] else
                '已自动处理当前机身内的旧图残影' if body_projection else
                '已建立本轮临时通路，原图保持不变' if approved else '正在积累独立实测地面帧')
        state='projection_unconfirmed' if pending and not structure['supported'] else 'active' if approved else 'waiting_evidence'
        return self._publish(approved,blocked,accepted=True,state=state,reason=reason,structure=structure,carried_under_body=len(carried),
                             observed_candidates=len(candidates),pending_occupied=pending,current_body_projection_cells=len(current_body),
                             departed_body_projection_cells=len(body_projection-current_body),
                             floor_geometry='precise_bounds' if self._precise_bounds(scan) is not None else 'conservative_cells')

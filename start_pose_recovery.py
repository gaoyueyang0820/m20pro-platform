"""Bounded start-pose recovery plans, without changing the start, goal or map.

A body that is physically clear but lacks the normal 3 cm margin can retreat
along a non-decreasing-clearance prefix. The original global planner is used
only AFTER a normal-margin pose is reached. No actuator/network/file I/O.
"""
import heapq
import math
import time
import numpy as np
from robot_geometry import BODY_HALF_LENGTH, BODY_HALF_WIDTH, CLEARANCE_HALF_LENGTH, CLEARANCE_HALF_WIDTH, CLEARANCE_MARGIN


def _wrap(v):return math.atan2(math.sin(v),math.cos(v))
def _pose(v):return [float(v[k]) for k in ['x','y','yaw']] if isinstance(v,dict) else list(map(float,v))


def _validate(scan,localization,start,now):
    if not isinstance(scan,dict) or not isinstance(localization,dict):return '需要新鲜雷达、同步定位和站立反馈'
    if localization.get('normal') is not True or not .85<=float(localization.get('confidence',-1))<=1:return '定位尚未可靠，不能把局部退让投影到旧图'
    if not 0<=float(localization.get('age',99))<=1:return '定位反馈过期'
    if scan.get('protocol')!=1 or scan.get('frame_id')!='base_link' or scan.get('pose_frame_id')!='map':return '雷达和地图坐标系尚未核对'
    if not 0<=float(scan.get('age',99))<=.65:return '扫描已经过期'
    if '_valid_until' in scan and float(scan['_valid_until'])<=now:return '扫描控制期限已过'
    times=[float(scan['stamp']),float(scan['pose_stamp'])]
    if not all(math.isfinite(v) for v in times) or abs(times[0]-times[1])>.100001:return '扫描与位姿时刻不同步'
    sp=_pose(scan['pose']);lp=_pose(localization['pose'])
    if len(sp)!=3 or len(lp)!=3 or not all(math.isfinite(v) for v in sp+lp):return '位姿无效'
    if math.dist(sp[:2],lp[:2])>.35 or abs(_wrap(sp[2]-lp[2]))>.25:return '定位读回与扫描不同步'
    # Near-wall centimetre recovery uses the synchronized scan pose, not a
    # quarter-second older display pose shifted by several decimetres.
    if math.dist(sp[:2],start[:2])>.03 or abs(_wrap(sp[2]-start[2]))>.05:return '退让起点与扫描位姿不同步，请使用最新同步位姿重新规划'
    if scan.get('geometry_version')!=2 or 'measured_floor' not in scan:return '缺少实测地面及障碍几何'
    return None


def _rect_gap(x,y,yaw,hx,hy,boxes):
    """A conservative separating-axis clearance lower bound; <0 is overlap."""
    if not len(boxes):return math.inf
    c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s)
    centres=(boxes[:,:2]+boxes[:,2:])/2;half=(boxes[:,2:]-boxes[:,:2])/2
    dx,dy=centres[:,0]-x,centres[:,1]-y
    gaps=np.column_stack((abs(dx)-(hx*a+hy*b+half[:,0]),abs(dy)-(hx*b+hy*a+half[:,1]),
                          abs(c*dx+s*dy)-(hx+half[:,0]*a+half[:,1]*b),
                          abs(-s*dx+c*dy)-(hy+half[:,0]*b+half[:,1]*a)))
    return float(np.min(np.max(gaps,axis=1)))


class _Geometry:
    def __init__(self,planner,start,scan,radius,standing,support_anchor=None,support_seed=None):
        self.planner=planner;self.start=start;self.scan=scan;self.scan_pose=_pose(scan['pose']);self.standing=standing
        self.support_anchor=start if support_anchor is None else support_anchor;self.seed=support_seed
        if support_seed:self.seed_floor=set(map(tuple,support_seed['floor']))
        self.res=float(scan.get('resolution',.2));self.floor=set(map(tuple,scan['measured_floor']))
        self.floor|=set(map(tuple,scan['floor']))
        self.obstacle_cells=set(map(tuple,scan['obstacles']))
        b=np.asarray(scan.get('obstacle_bounds',[]),float).reshape(-1,4)
        self.precise=len(b)==len(scan['obstacles']) and np.isfinite(b).all() and bool(np.all(b[:,2:]>=b[:,:2]))
        if not self.precise:
            b=np.array([[(x-.5)*self.res,(y-.5)*self.res,(x+.5)*self.res,(y+.5)*self.res] for x,y in scan['obstacles']],float).reshape(-1,4)
            self.floor-=self.obstacle_cells
        self.local_boxes=b.copy()
        if len(b):self.local_boxes[:,:2]-=.005;self.local_boxes[:,2:]+=.005
        bound=radius+math.hypot(CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)+.2
        x,y=start[:2];left,top=planner.cell(x-bound,y+bound);right,bottom=planner.cell(x+bound,y-bound)
        self.bounds=(planner.origin[0],planner.origin[1],planner.origin[0]+planner.w*planner.res,planner.origin[1]+planner.h*planner.res)
        working=planner.free
        self.refined_live_raster_cells=0
        layer=getattr(planner,'live_surface',None)
        if self.precise and layer is not None and layer.last_stamp==scan.get('stamp'):
            # Use exact CURRENT obstacle boxes for physical recovery, instead
            # of interpreting their conservative raster cover as a new solid
            # wall. Static occupied/unknown and independent overlays remain.
            working=planner.free.copy()
            for ix,iy in layer.blocked_cells:
                if planner.base_free[iy,ix] and (ix,iy) not in planner.overlay:
                    if not working[iy,ix]:self.refined_live_raster_cells+=1
                    working[iy,ix]=True
        self.map_boxes=self._boxes(working,left,right,top,bottom)
        self.base_boxes=self._boxes(planner.base_free,left,right,top,bottom)

    def _boxes(self,free,left,right,top,bottom):
        p=self.planner;left=max(0,left);right=min(p.w-1,right);top=max(0,top);bottom=min(p.h-1,bottom)
        yy,xx=np.nonzero(~free[top:bottom+1,left:right+1]);xx+=left;yy+=top
        x=p.origin[0]+xx*p.res;y=p.origin[1]+(p.h-yy-1)*p.res
        return np.column_stack((x,y,x+p.res,y+p.res))

    def local(self,pose):
        x,y,yaw=pose;sx,sy,syaw=self.scan_pose;c,s=math.cos(syaw),math.sin(syaw)
        return c*(x-sx)+s*(y-sy),-s*(x-sx)+c*(y-sy),_wrap(yaw-syaw)

    def map_gap(self,pose,hx=BODY_HALF_LENGTH,hy=BODY_HALF_WIDTH,base=False):
        x,y,yaw=pose;c,s=abs(math.cos(yaw)),abs(math.sin(yaw));ex=hx*c+hy*s;ey=hx*s+hy*c
        lo_x,lo_y,hi_x,hi_y=self.bounds
        return min(_rect_gap(x,y,yaw,hx,hy,self.base_boxes if base else self.map_boxes),x-ex-lo_x,hi_x-x-ex,y-ey-lo_y,hi_y-y-ey)

    def local_gap(self,pose,hx=BODY_HALF_LENGTH,hy=BODY_HALF_WIDTH):
        x,y,yaw=self.local(pose);return _rect_gap(x,y,yaw,hx,hy,self.local_boxes)

    def supported(self,pose):
        x,y,yaw=self.local(pose);c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s);half=self.res/2
        hx,hy=BODY_HALF_LENGTH,BODY_HALF_WIDTH
        rx=hx*a+hy*b+half;ry=hx*b+hy*a+half
        for ix in range(math.ceil((x-rx)/self.res),math.floor((x+rx)/self.res)+1):
            for iy in range(math.ceil((y-ry)/self.res),math.floor((y+ry)/self.res)+1):
                dx,dy=ix*self.res-x,iy*self.res-y
                if abs(c*dx+s*dy)>=hx+half*(a+b)-1e-9 or abs(-s*dx+c*dy)>=hy+half*(a+b)-1e-9:continue
                # Existing platform convention: current occupied body/blind
                # patch is supported only after positive standing feedback.
                # This is not extended around future poses along the path.
                sx,sy,syaw=self.scan_pose;sc,ss=math.cos(syaw),math.sin(syaw)
                wx=sx+sc*ix*self.res-ss*iy*self.res;wy=sy+ss*ix*self.res+sc*iy*self.res
                standing_support=self.standing and math.hypot(wx-self.support_anchor[0],wy-self.support_anchor[1])<=.55
                carried=False
                if self.seed and math.hypot(ix,iy)*self.res<=.55:
                    px,py,pyaw=self.seed['pose'];pc,ps=math.cos(pyaw),math.sin(pyaw);resolution=self.seed['resolution']
                    q=(round((pc*(wx-px)+ps*(wy-py))/resolution),round((-ps*(wx-px)+pc*(wy-py))/resolution))
                    carried=q in self.seed_floor
                if (ix,iy) not in self.floor and not standing_support and not carried:return False
        return True

    def clearance(self,pose):return min(self.map_gap(pose),self.local_gap(pose))
    def physical(self,pose):return self.clearance(pose)>=-1e-8 and self.supported(pose)
    def standard(self,pose):return self.map_gap(pose,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)>=-1e-8 and self.local_gap(pose,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)>=-1e-8 and self.supported(pose)

    def edge(self,a,b,relaxed=False):
        length=math.dist(a[:2],b[:2]);angle=_wrap(b[2]-a[2]);n=max(1,math.ceil(length/.025),math.ceil(abs(angle)/math.radians(2.5)))
        previous=self.clearance(a)
        # For non-rotating translation use a single swept rectangle, so an
        # unsampled thin obstacle cannot fall between the discrete poses.
        if abs(angle)<1e-8:
            c,s=math.cos(a[2]),math.sin(a[2]);dx,dy=b[0]-a[0],b[1]-a[1]
            base_hx,base_hy=(BODY_HALF_LENGTH,BODY_HALF_WIDTH) if relaxed else (CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)
            hx=base_hx+abs(c*dx+s*dy)/2;hy=base_hy+abs(-s*dx+c*dy)/2
            mid=[(a[0]+b[0])/2,(a[1]+b[1])/2,a[2]]
            swept_gap=min(self.map_gap(mid,hx,hy),self.local_gap(mid,hx,hy))
            if swept_gap<-1e-8 or relaxed and swept_gap<previous-.001:return None
        points=[]
        for i in range(1,n+1):
            p=[a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n,_wrap(a[2]+angle*i/n)]
            if not self.physical(p) or not relaxed and not self.standard(p):return None
            gap=self.clearance(p)
            if relaxed and gap<previous-.001:return None
            if abs(angle)>1e-8:
                bhx,bhy=(BODY_HALF_LENGTH,BODY_HALF_WIDTH) if relaxed else (CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)
                sweep=math.hypot(bhx,bhy)*abs(angle)/(2*n)
                if self.map_gap(p,bhx+sweep,bhy+sweep)<-1e-8 or self.local_gap(p,bhx+sweep,bhy+sweep)<-1e-8:return None
            points.append(p);previous=gap
        return points


def _knots(poses,geometry):
    result=[]
    for i,p in enumerate(poses):
        kind='start'
        if i:
            a=poses[i-1];dx,dy=p[0]-a[0],p[1]-a[1];c,s=math.cos(a[2]),math.sin(a[2])
            f,side=c*dx+s*dy,-s*dx+c*dy
            kind=('turn_left' if _wrap(p[2]-a[2])>0 else 'turn_right') if math.hypot(dx,dy)<1e-8 else ('forward' if f>0 else 'backward') if abs(f)>=abs(side) else ('left' if side>0 else 'right')
        result.append(dict(x=p[0],y=p[1],yaw=p[2],kind=kind,clearance_lower_bound_m=round(geometry.clearance(p),4)))
    return result


def recover_start(planner,start,goal,yaw=None,scan=None,localization=None,standing=False,max_distance=.8,max_turn=math.radians(20),budget=1.2):
    """Return a verified recovery prefix and the planner's goal policy.

    All outputs are planning evidence, never robot commands. Integrators must
    track the short prefix with live feedback and check_recovery_step(), then
    switch to ordinary full-margin path following. Do not pass the prefix to a
    controller that applies the normal margin at its initially deficient pose.
    """
    began=time.monotonic();deadline=began+max(0,float(budget));start=_pose(start);goal=list(map(float,goal))
    result=dict(ok=False,classification='unclassified',reason='',recovery_dense_path=[],global_plan=None,
                requested_start=list(start),requested_goal=list(goal),requires_live_revalidation=True,requires_field_validation=True,
                persistent_map_changed=False,motion_sent=False,constraints=dict(max_distance_m=float(max_distance),max_turn_rad=float(max_turn),
                normal_margin_m=CLEARANCE_MARGIN,max_total_translation_m=float(max_distance),max_total_rotation_rad=float(max_turn),
                must_follow_returned_prefix=True,recommended_max_duration_s=8,relaxed_prefix_policy='physical_body_clear_and_non_decreasing_clearance',
                support_assumption='standing-confirmed initial body blind patch; measured floor required outside it'))
    try:
        if len(start)!=3 or len(goal)!=2 or not all(math.isfinite(v) for v in start+goal):raise ValueError('起终点坐标无效')
        error=_validate(scan,localization,start,began)
        if error:result.update(classification='observation_unavailable',reason=error);return result
        if standing is not True:result.update(classification='standing_unconfirmed',reason='需要站立反馈确认，不能自动假定机身下方有支撑');return result
        if not 0<max_distance<=1 or not 0<=max_turn<=math.radians(30):raise ValueError('起点恢复范围超出短程限制')
        planner.live_surface_status();revision=planner.grid_revision
        geometry=_Geometry(planner,start,scan,max_distance,standing)
        result['support_seed']=dict(pose=list(scan['pose']),resolution=geometry.res,stamp=scan['stamp'],
            floor=[list(q) for q in sorted(geometry.floor) if math.hypot(*q)*geometry.res<=max_distance+.8],
            policy='only carry previously measured ground into the current body blind patch')
        result['live_obstacle_raster_refined_cells']=geometry.refined_live_raster_cells
        map_gap=geometry.map_gap(start);live_gap=geometry.local_gap(start)
        result['initial_clearance_lower_bound_m']=round(min(map_gap,live_gap),4)
        if live_gap<-1e-8:
            result.update(classification='live_body_collision',reason='真实机身与当前实测障碍包围范围重叠，需更精确核对；不能通过缩小机身或忽略回波自动脱困');return result
        if map_gap<-1e-8:
            result.update(classification='map_surface_disagreement',reason='真实机身仍与工作图占据重叠；需先验证局部旧图残影或恢复定位，不能单凭本帧清墙');return result
        if not geometry.supported(start):
            result.update(classification='unsupported_floor',reason='当前机身周边缺少实际地面支撑观测');return result
        relaxed=not geometry.standard(start)
        classification='clearance_only' if relaxed else 'temporary_surface_verified' if geometry.map_gap(start,base=True)<-1e-8 else 'ready'
        if not relaxed:
            try:
                plan=planner.plan(start,goal,yaw,budget=min(.35,max(.01,deadline-time.monotonic())))
                if planner.grid_revision!=revision:raise ValueError('实时通行层已变化')
                result.update(ok=True,classification=classification,reason='当前连续姿态有效，可直接接续原目标路线',
                              recovery_dense_path=_knots([start],geometry),global_plan=plan,dense_path=plan['dense_path'],recovery_distance_m=0.,expanded=0)
                return result
            except ValueError as exc:
                result['initial_plan_error']=str(exc)
                sx,sy=planner.cell(*start[:2]);cx,cy=planner.world(sx,sy);sh=round(start[2]/(math.pi/4))*math.pi/4
                # Only recover a demonstrated start discretization/orientation
                # issue. A blocked distant goal is not solved by moving blindly.
                center=[cx,cy,start[2]];snapped=[cx,cy,sh]
                if geometry.standard(center) and geometry.standard(snapped) and planner.rotation_fits(sx,sy,start[2],sh):
                    result.update(classification='valid_start_no_route',reason='起点姿态有效，原目标路线尚不可达；不以任意退让代替寻路');return result
                classification='start_quantization'
        result['classification']=classification
        step=.05;angle_step=math.radians(5);origin=(0,0,0);queue=[(0.,origin)];cost={origin:0.};parents={};poses={origin:start};edges={};travelled={origin:0.};turned={origin:0.}
        expanded=0;attempts=0;best_prefix=None
        while queue and time.monotonic()<deadline:
            score,key=heapq.heappop(queue)
            if score!=cost.get(key):continue
            p=poses[key];expanded+=1
            lattice_ready=False
            if key!=origin and geometry.standard(p):
                ix,iy=planner.cell(*p[:2]);cx,cy=planner.world(ix,iy);sh=round(p[2]/(math.pi/4))*math.pi/4
                center=[cx,cy,p[2]]
                lattice_ready=(geometry.standard(center) and geometry.edge(p,center,relaxed=False) is not None
                               and planner.rotation_fits(ix,iy,p[2],sh))
            if lattice_ready:
                prefix=[key]
                while prefix[-1]!=origin:prefix.append(parents[prefix[-1]])
                prefix.reverse();trajectory=[start]
                for q in prefix[1:]:trajectory.extend(edges[q])
                # The axis lattice may discover an L-shaped escape even when
                # one fixed-heading diagonal translation is clear. The runtime
                # executes only a single translation: shorten this prefix only
                # after checking its ENTIRE swept body and non-decreasing
                # clearance, including fresh measured floor along the segment.
                if abs(_wrap(p[2]-start[2]))<1e-8:
                    direct=geometry.edge(start,p,relaxed=relaxed)
                    if direct is not None:trajectory=[start]+direct
                best_prefix=_knots(trajectory,geometry)
                if attempts<8:
                    attempts+=1
                    try:
                        plan=planner.plan(p,goal,yaw,budget=min(.3,max(.01,deadline-time.monotonic())))
                        if planner.grid_revision!=revision:raise ValueError('实时通行层已变化')
                        distance=sum(math.dist(a[:2],b[:2]) for a,b in zip(trajectory,trajectory[1:]))
                        full=best_prefix+plan['dense_path'][1:]
                        result.update(ok=True,reason='已找到连续退让到标准净空的短轨迹，并接续原目标',recovery_dense_path=best_prefix,
                                      global_plan=plan,dense_path=full,recovery_distance_m=round(distance,4),expanded=expanded,
                                      recovered_start=list(p),planning_ms=round((time.monotonic()-began)*1000,2))
                        return result
                    except ValueError as exc:result['last_route_error']=str(exc)
            # Position coordinates are in the INITIAL body frame; every edge is
            # evaluated with its actual orientation and full continuous sweep.
            for dx,dy,da,weight in [(-1,0,0,1.),(0,1,0,1.05),(0,-1,0,1.05),(1,0,0,1.1),(0,0,1,1.7),(0,0,-1,1.7)]:
                nxt=(key[0]+dx,key[1]+dy,key[2]+da)
                travelled_next=travelled[key]+math.hypot(dx,dy)*step;turned_next=turned[key]+abs(da)*angle_step
                if travelled_next>max_distance+1e-9 or turned_next>max_turn+1e-9:continue
                if math.hypot(nxt[0],nxt[1])*step>max_distance+1e-9 or abs(nxt[2]*angle_step)>max_turn+1e-9:continue
                c,s=math.cos(start[2]),math.sin(start[2]);np_=[start[0]+c*nxt[0]*step-s*nxt[1]*step,start[1]+s*nxt[0]*step+c*nxt[1]*step,_wrap(start[2]+nxt[2]*angle_step)]
                if relaxed and geometry.clearance(np_)<max(geometry.clearance(p),result['initial_clearance_lower_bound_m'])-.001:continue
                edge=geometry.edge(p,np_,relaxed=relaxed)
                if edge is None:continue
                new=score+step*weight
                if new<cost.get(nxt,math.inf):
                    cost[nxt]=new;parents[nxt]=key;poses[nxt]=np_;edges[nxt]=edge;travelled[nxt]=travelled_next;turned[nxt]=turned_next;heapq.heappush(queue,(new,nxt))
            if expanded>=1500:break
        result.update(reason='没有找到同时满足真实机身、实测地面和原目标接续的短程退让路线',expanded=expanded,
                      candidate_recovery=best_prefix,planning_ms=round((time.monotonic()-began)*1000,2))
        return result
    except (ValueError,TypeError,KeyError,OverflowError) as exc:
        result.update(classification='invalid_or_changed_evidence',reason=str(exc));return result


def check_recovery_step(planner,current,next_pose,scan,localization,anchor,standing=False,max_distance=.8,max_turn=math.radians(20),support_seed=None):
    """Revalidate ONE <=8 cm / <=5 degree recovery step using a NEW observation.

    This permits only physical-body-clear, non-decreasing-clearance escape.
    It does not authorize arbitrary driving with a permanently smaller margin.
    """
    current=_pose(current);next_pose=_pose(next_pose);anchor=_pose(anchor)
    try:
        error=_validate(scan,localization,current,time.monotonic())
        if error:return dict(ok=False,reason=error)
        if not standing:return dict(ok=False,reason='站立反馈未确认')
        if math.dist(current[:2],next_pose[:2])>.080001 or abs(_wrap(next_pose[2]-current[2]))>math.radians(5)+1e-6:return dict(ok=False,reason='恢复单步过长')
        if math.dist(next_pose[:2],anchor[:2])>max_distance+1e-6 or abs(_wrap(next_pose[2]-anchor[2]))>max_turn+1e-6:return dict(ok=False,reason='已超出本轮短程恢复范围')
        planner.live_surface_status();geometry=_Geometry(planner,current,scan,max_distance,standing,support_anchor=anchor,support_seed=support_seed)
        if not geometry.physical(current):return dict(ok=False,reason='当前实测机身/地面条件不允许退让')
        edge=geometry.edge(current,next_pose,relaxed=not geometry.standard(current))
        return dict(ok=edge is not None,reason='退让单步已核对，仍须实际位移闭环' if edge is not None else '退让方向的障碍、地面或净空不满足',
                    current_clearance_m=geometry.clearance(current),next_clearance_m=geometry.clearance(next_pose),standard_at_next=geometry.standard(next_pose))
    except (ValueError,TypeError,KeyError,OverflowError) as exc:return dict(ok=False,reason=str(exc))

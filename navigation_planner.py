"""Footprint-aware global paths, obstacle-distance guidance and any-angle shortcuts.

Pure planning, no actuator I/O. Concepts reviewed against Nav2 Smac and Theta*
(Apache-2.0); this NumPy/Python implementation is independently written.
"""
import heapq,math,re,time
from robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH,CLEARANCE_MARGIN


class GoalSpaceError(ValueError):
    """The requested centre is known free, but the complete body cannot fit."""


def pgm_pixels(raw):
    tokens=re.finditer(rb'#[^\r\n]*|[^\s#]+',raw);header=[]
    for m in tokens:
        if m[0].startswith(b'#'):continue
        header.append(m[0])
        if len(header)==4:break
    if len(header)!=4 or header[0] not in [b'P2',b'P5']:raise ValueError('无效 PGM 地图')
    w,h,peak=map(int,header[1:]);end=m.end()
    if not 0<w*h<=4_000_000 or not 0<peak<=255:raise ValueError('地图尺寸超出范围')
    if header[0]==b'P5':
        if raw[end:end+1] not in [b' ',b'\t',b'\r',b'\n']:raise ValueError('PGM 缺少分隔符')
        pixels=raw[end+(2 if raw[end:end+2]==b'\r\n' else 1):]
    else:pixels=bytes(int(x) for x in re.sub(rb'#[^\n]*',b'',raw[end:]).split())
    if len(pixels)!=w*h:raise ValueError('PGM 像素数量不匹配')
    if peak!=255:pixels=bytes(round(p/peak*255) for p in pixels)
    return w,h,pixels


def _components(mask):
    """Eight-connected labels/stats; OpenCV is the installed fast path."""
    import numpy as np
    try:
        import cv2
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=8)
        return labels,stats[1:]
    except ImportError:
        labels=np.zeros(mask.shape,np.int32);rows=[];h,w=mask.shape;label=0
        for y,x in zip(*np.nonzero(mask)):
            if labels[y,x]:continue
            label+=1;todo=[(int(x),int(y))];labels[y,x]=label
            lo_x=hi_x=int(x);lo_y=hi_y=int(y);area=0
            while todo:
                xx,yy=todo.pop();area+=1;lo_x=min(lo_x,xx);hi_x=max(hi_x,xx);lo_y=min(lo_y,yy);hi_y=max(hi_y,yy)
                for dy in (-1,0,1):
                    for dx in (-1,0,1):
                        nx,ny=xx+dx,yy+dy
                        if 0<=nx<w and 0<=ny<h and mask[ny,nx] and not labels[ny,nx]:
                            labels[ny,nx]=label;todo.append((nx,ny))
            rows.append((lo_x,lo_y,hi_x-lo_x+1,hi_y-lo_y+1,area))
        return labels,np.asarray(rows,dtype=np.int32).reshape(-1,5)


def _dilate_cells(mask,steps):
    import numpy as np
    result=mask.copy()
    for _ in range(steps):
        padded=np.pad(result,1,constant_values=False)
        result=np.logical_or.reduce([padded[dy:dy+mask.shape[0],dx:dx+mask.shape[1]] for dy in range(3) for dx in range(3)])
    return result


def _static_speckles(probability,resolution,free_thresh,occupied_thresh):
    """Small isolated saved-map components become cost, not asserted free space.

    The decision concerns an old 2D map only. It cannot identify a person or
    prove a live object is absent. Live obstacles always override this layer.
    Continuous walls, boundary fragments and unobserved cells stay hard.
    """
    import numpy as np
    occupied=probability>occupied_thresh;unknown=(probability>=free_thresh)&~occupied
    labels,stats=_components(occupied)
    eligible=np.zeros(len(stats)+1,bool)
    if len(stats):
        eligible[1:]=(stats[:,4]*resolution**2<=.08000001)&(np.maximum(stats[:,2],stats[:,3])*resolution<=.45000001)
    small=eligible[labels]
    # Keep fragments attached/adjacent to a larger wall or an unknown boundary.
    boundary=unknown|(occupied&~small);boundary=boundary.copy()
    boundary[0]=True;boundary[-1]=True;boundary[:,0]=True;boundary[:,-1]=True
    near=_dilate_cells(boundary,max(1,round(.10/resolution)))
    touched=np.unique(labels[near&small]);eligible[touched]=False
    soft=eligible[labels]&occupied
    return soft,dict(enabled=True,components=int(eligible[1:].sum()),source_cells=int(soft.sum()),
                     max_component_area_m2=.08,max_component_extent_m=.45,persistent_map_changed=False)


class GridPlanner:
    """Heading lattice plus continuous, footprint-checked any-angle paths.

    Nominal body is .84 x .45 m (installed M20 profile); add 3 cm per side. Unknown
    cells and diagonal corner-cutting are blocked. Turn checks sample at most
    5 degrees and include a sweep margin for the interval between samples.
    """
    def __init__(self,raw,resolution,origin,negate=0,free_thresh=.196,occupied_thresh=.65,obstacles=(),robot_pose=None,enable_live_surface=False,soft_static_speckles=False):
        import numpy as np
        w,h,pixels=pgm_pixels(raw)
        probability=np.frombuffer(pixels,np.uint8).reshape(h,w).astype(float)/255
        if not negate:probability=1-probability
        free=probability<free_thresh
        self.soft_static_speckles=bool(soft_static_speckles)
        if self.soft_static_speckles:
            speckles,self.static_speckles=_static_speckles(probability,resolution,free_thresh,occupied_thresh)
        else:speckles=np.zeros_like(free);self.static_speckles=dict(enabled=False,components=0,source_cells=0,persistent_map_changed=False)
        self.original=probability;self.original_res=resolution;self.origin=origin
        factor=max(1,math.ceil(.10/resolution));self.res=resolution*factor
        # Preserve the world origin, padding on the top/right of the image only.
        oh,ow=math.ceil(h/factor)*factor,math.ceil(w/factor)*factor
        padded=np.zeros((oh,ow),bool);padded[oh-h:,:w]=free
        self.free=padded.reshape(oh//factor,factor,ow//factor,factor).all(axis=(1,3))
        self.h,self.w=self.free.shape
        self.base_free=self.free.copy()
        planning=np.zeros((oh,ow),bool);planning[oh-h:,:w]=free|speckles
        self.planning_free=planning.reshape(oh//factor,factor,ow//factor,factor).all(axis=(1,3))
        self.soft_cells=self.planning_free&~self.base_free
        self.free=self.planning_free.copy();self.soft_cost=np.zeros_like(self.free,dtype=float)
        # Cost covers the body vicinity; the centre should prefer passing away
        # from uncertain residue when another known corridor is readily usable.
        grown=self.soft_cells.copy();rings=max(1,math.ceil(.55/self.res))
        if np.any(grown):
            for ring in range(rings+1):
                self.soft_cost=np.maximum(self.soft_cost,grown*(2.0*(1-ring/(rings+1))))
                grown=_dilate_cells(grown,1)
        self.static_speckles.update(planning_cells=int(self.soft_cells.sum()),cost_radius_m=round(rings*self.res,3))
        self.overlay=set();self.grid_revision=0
        self.masks={};self._rotations={};self._potential_cache={}
        self.occupied_thresh=occupied_thresh;self.free_thresh=free_thresh
        self.self_projection_ignored=0;self._deadline=None;self._potential_expanded=0
        self.live_surface=None;self._live_revision=0;self._planning_revision=None
        self.goal_tolerance=0.
        if enable_live_surface:
            from live_traversability import LiveTraversability
            occupied=np.zeros((oh,ow),bool);occupied[oh-h:,:w]=probability>occupied_thresh
            occupied=occupied.reshape(oh//factor,factor,ow//factor,factor).any(axis=(1,3))
            self.live_surface=LiveTraversability(self.base_free,occupied,self.res,self.origin)
        self._refresh_clearance()
        self.update_obstacles(obstacles,robot_pose)

    def _refresh_clearance(self):
        import numpy as np
        self.clearance=np.zeros_like(self.free,dtype=float);inside=self.free.copy()
        for i in range(1,max(2,math.ceil(1.2/self.res))):
            self.clearance[inside]=i*self.res
            a=np.zeros_like(inside);a[1:-1,1:-1]=inside[1:-1,1:-1]&inside[:-2,1:-1]&inside[2:,1:-1]&inside[1:-1,:-2]&inside[1:-1,2:]&inside[:-2,:-2]&inside[:-2,2:]&inside[2:,:-2]&inside[2:,2:];inside=a

    def update_obstacles(self,obstacles,robot_pose=None):
        """Replace the current live layer; static occupied/unknown cells never clear.

        Caller owns freshness and frame validation. An unchanged observation reuses
        footprint and distance caches; a changed overlay invalidates those caches.
        """
        layer=set();ignored=0
        for x,y in obstacles:
            if not math.isfinite(x) or not math.isfinite(y):continue
            ix,iy=self.cell(x,y)
            if not (0<=ix<self.w and 0<=iy<self.h):continue
            if robot_pose is not None and not self.soft_static_speckles:
                dx,dy=x-robot_pose[0],y-robot_pose[1];c,s=math.cos(robot_pose[2]),math.sin(robot_pose[2])
                if abs(c*dx+s*dy)<BODY_HALF_LENGTH and abs(-s*dx+c*dy)<BODY_HALF_WIDTH:
                    if self.base_free[iy,ix]:ignored+=1
                    continue
            layer.add((ix,iy))
        self.self_projection_ignored=ignored
        changed=len(layer.symmetric_difference(self.overlay))
        if changed:
            self.overlay=layer;self._rebuild_layers()
        return dict(changed_cells=changed,grid_revision=self.grid_revision,obstacles=len(layer))

    def _rebuild_layers(self):
        import numpy as np
        current=self.planning_free.copy()
        if self.live_surface:
            for x,y in self.live_surface.free_cells:current[y,x]=True
            for x,y in self.live_surface.blocked_cells:current[y,x]=False
            self._live_revision=self.live_surface.revision
        for x,y in self.overlay:current[y,x]=False
        if not np.array_equal(current,self.free):
            self.free=current;self.grid_revision+=1
            self.masks.clear();self._rotations.clear();self._potential_cache.clear();self._refresh_clearance()

    def update_live_surface(self,scan,localization,now=None):
        """Opt-in ephemeral observed-floor layer; the original PGM never changes."""
        if self.live_surface is None:
            return dict(enabled=False,accepted=False,reason='本规划器未启用实时通行层',temporary_free_cells=0,
                        blocked_cells=0,grid_revision=self.grid_revision,persistent_map_changed=False)
        report=self.live_surface.update(scan,localization,now)
        if self.live_surface.revision!=self._live_revision:self._rebuild_layers()
        return dict(report,enabled=True,grid_revision=self.grid_revision)

    def _expire_live_surface(self):
        # Planning is a read-only snapshot, not permission to move. A search
        # may outlast one scan; execution queries resume expiration immediately
        # after plan() returns and must obtain fresh observations again.
        if self._planning_revision is not None:return
        if self.live_surface and self.live_surface.expire():self._rebuild_layers()

    def live_surface_status(self):
        self._expire_live_surface()
        if self.live_surface is None:return dict(enabled=False,temporary_free_cells=0,blocked_cells=0,grid_revision=self.grid_revision)
        return dict(self.live_surface.snapshot(),enabled=True,grid_revision=self.grid_revision)

    def cell(self,x,y):return math.floor((x-self.origin[0])/self.res),self.h-1-math.floor((y-self.origin[1])/self.res)
    def world(self,x,y):return self.origin[0]+(x+.5)*self.res,self.origin[1]+(self.h-y-.5)*self.res

    def mask(self,heading,sweep_margin=0.):
        self._expire_live_surface()
        import numpy as np
        deadline=getattr(self,'_deadline',None)
        if deadline is not None and time.monotonic()>=deadline:raise ValueError('地图搜索超时，请选择较近的目标再规划')
        heading=round(heading%32,10);sweep_margin=round(sweep_margin,10)
        key=(heading,sweep_margin)
        if key in self.masks:return self.masks[key]
        angle=heading*math.pi/16;c,s=math.cos(angle),math.sin(angle)
        # Separating-axis intersection, rather than a fixed extra margin for
        # every non-cardinal angle (which rejected even a 1-degree correction).
        halfx,halfy=CLEARANCE_HALF_LENGTH+sweep_margin,CLEARANCE_HALF_WIDTH+sweep_margin
        half=self.res/2;a,b=abs(c),abs(s)
        n=math.ceil((math.hypot(halfx,halfy)+self.res/math.sqrt(2))/self.res)
        valid=self.free.copy()
        for dy in range(-n,n+1):
            if deadline is not None and time.monotonic()>=deadline:raise ValueError('地图搜索超时，请选择较近的目标再规划')
            for dx in range(-n,n+1):
                x,y=dx*self.res,-dy*self.res
                if (abs(x)>halfx*a+halfy*b+half or abs(y)>halfx*b+halfy*a+half
                        or abs(c*x+s*y)>halfx+half*(a+b) or abs(-s*x+c*y)>halfy+half*(a+b)):continue
                shifted=np.zeros_like(valid)
                y0,y1=max(0,-dy),min(self.h,self.h-dy);x0,x1=max(0,-dx),min(self.w,self.w-dx)
                if y1>y0 and x1>x0:shifted[y0:y1,x0:x1]=self.free[y0+dy:y1+dy,x0+dx:x1+dx]
                valid &= shifted
        self.masks[key]=valid;return valid

    def fits(self,x,y,h,sweep_margin=0.):return 0<=x<self.w and 0<=y<self.h and bool(self.mask(h,sweep_margin)[y,x])

    def rotation_fits(self,x,y,a,b):
        delta=math.atan2(math.sin(b-a),math.cos(b-a));steps=max(1,math.ceil(abs(delta)/(math.pi/36)))
        key=(x,y,round(a%(2*math.pi),9),round(delta,9))
        if key not in self._rotations:
            sweep=math.hypot(CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)*abs(delta)/(2*steps)
            self._rotations[key]=all(self.fits(x,y,(a+delta*i/steps)/(math.pi/16),sweep) for i in range(steps+1))
        return self._rotations[key]

    def _check_time(self):
        if self._deadline is not None and time.monotonic()>=self._deadline:
            raise ValueError('地图搜索超时，请选择较近的目标再规划')

    def _rectangle_free(self,x,y,yaw,halfx,halfy):
        """Exact rectangle/cell SAT test at a continuous world pose."""
        import numpy as np
        self._expire_live_surface()
        c,s=math.cos(yaw),math.sin(yaw);a,b=abs(c),abs(s);half=self.res/2
        ex=halfx*a+halfy*b;ey=halfx*b+halfy*a
        left,top=self.cell(x-ex,y+ey);right,bottom=self.cell(x+ex,y-ey)
        if left<0 or top<0 or right>=self.w or bottom>=self.h:return False
        blocked_y,blocked_x=np.nonzero(~self.free[top:bottom+1,left:right+1])
        if not len(blocked_x):return True
        dx=self.origin[0]+(blocked_x+left+.5)*self.res-x
        dy=self.origin[1]+(self.h-(blocked_y+top)-.5)*self.res-y
        overlap=(abs(dx)<ex+half-1e-10)&(abs(dy)<ey+half-1e-10)
        overlap&=(abs(c*dx+s*dy)<halfx+half*(a+b)-1e-10)&(abs(-s*dx+c*dy)<halfy+half*(a+b)-1e-10)
        return not bool(np.any(overlap))

    def pose_fits(self,x,y,yaw):
        return self._rectangle_free(x,y,yaw,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)

    def segment_fits(self,a,b,yaw=None):
        """Continuous swept body; not a point/ray test and never through unknown."""
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        direction=math.atan2(dy,dx) if length>1e-9 else (yaw or 0.)
        heading=direction if yaw is None else yaw
        # Translation along the body axis is exactly one extended rectangle.
        delta=math.atan2(math.sin(direction-heading),math.cos(direction-heading))
        if abs(math.sin(delta))<1e-8:
            return self._rectangle_free((a[0]+b[0])/2,(a[1]+b[1])/2,heading,
                                        CLEARANCE_HALF_LENGTH+length/2,CLEARANCE_HALF_WIDTH)
        steps=max(1,math.ceil(length/(self.res*.4)));margin=length/(2*steps)
        return all(self._rectangle_free(a[0]+dx*i/steps,a[1]+dy*i/steps,heading,
                    CLEARANCE_HALF_LENGTH+margin,CLEARANCE_HALF_WIDTH+margin) for i in range(steps+1))

    def world_rotation_fits(self,p,a,b):
        delta=math.atan2(math.sin(b-a),math.cos(b-a));steps=max(1,math.ceil(abs(delta)/(math.pi/36)))
        margin=math.hypot(CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)*abs(delta)/(2*steps)
        return all(self._rectangle_free(p[0],p[1],a+delta*i/steps,CLEARANCE_HALF_LENGTH+margin,
                                       CLEARANCE_HALF_WIDTH+margin) for i in range(steps+1))

    def _potential(self,gx,gy):
        """Lazy reverse Dijkstra: obstacle-aware lower bound for the heading search."""
        import numpy as np
        key=(gx,gy)
        if key not in self._potential_cache:
            if len(self._potential_cache)>=3:self._potential_cache.pop(next(iter(self._potential_cache)))
            d=np.full((self.h,self.w),math.inf);d[gy,gx]=0.
            self._potential_cache[key]=(d,np.zeros_like(self.free),[(0.,gx,gy)])
        d,closed,queue=self._potential_cache[key]
        def value(tx,ty):
            expansions=0
            while queue and not closed[ty,tx]:
                cost,x,y=heapq.heappop(queue)
                if closed[y,x] or cost!=d[y,x]:continue
                closed[y,x]=True;expansions+=1
                if expansions%256==0:self._check_time()
                # Reverse edge charges the forward destination's clearance cost.
                penalty=1+.18/max(.1,self.clearance[y,x])+self.soft_cost[y,x]
                for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    nx,ny=x+dx,y+dy
                    if not (0<=nx<self.w and 0<=ny<self.h and self.free[ny,nx]):continue
                    if dx and dy and not (self.free[y,nx] and self.free[ny,x]):continue
                    candidate=cost+math.hypot(dx,dy)*self.res*penalty
                    if candidate<d[ny,nx]:d[ny,nx]=candidate;heapq.heappush(queue,(candidate,nx,ny))
            self._potential_expanded+=expansions
            return float(d[ty,tx]) if closed[ty,tx] else math.inf
        return value

    def _segment_soft_cost(self,a,b):
        if not self.static_speckles['components']:return 0.
        length=math.dist(a[:2],b[:2]);steps=max(1,math.ceil(length/self.res));cost=0.
        for i in range(1,steps+1):
            x,y=self.cell(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
            cost+=length/steps*self.soft_cost[y,x]
        return cost

    def _segment_cost(self,a,b):
        length=math.dist(a[:2],b[:2]);steps=max(1,math.ceil(length/self.res));total=0.
        for i in range(1,steps+1):
            x,y=self.cell(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps)
            total+=length/steps*(1+.18/max(.1,self.clearance[y,x])+self.soft_cost[y,x])
        return total

    def _smooth(self,poses):
        """Cost-aware shortcutting, with checked incoming/outgoing rotations.

        A valid lattice path remains the fallback. Shortcut failure never turns
        a valid route into a failure, nor silently removes a necessary turn.
        """
        result=[poses[0]];i=0;shortcuts=0
        prefix=[0.]
        for a,b in zip(poses,poses[1:]):prefix.append(prefix[-1]+self._segment_cost(a,b))
        while i<len(poses)-1:
            self._check_time()
            # Candidate endpoints at turns plus regularly spaced intermediate
            # points keep shortcut work bounded even on kilometre-scale grids.
            choices=[j for j in range(i+2,min(len(poses),i+161)) if j==len(poses)-1 or j%8==0 or
                     abs(math.atan2(math.sin(poses[j][2]-poses[j-1][2]),math.cos(poses[j][2]-poses[j-1][2])))>.05]
            picked=None
            for j in reversed(choices):
                self._check_time()
                a,b=poses[i],poses[j]
                if math.dist(a[:2],b[:2])<self.res:continue
                heading=math.atan2(b[1]-a[1],b[0]-a[0])
                if self._segment_cost(a,b)>1.03*(prefix[j]-prefix[i])+.01:continue
                if (self.segment_fits(a,b,heading) and self.world_rotation_fits(a,a[2],heading)
                        and self.world_rotation_fits(b,heading,b[2])):
                    picked=(j,heading);break
            if picked:
                j,heading=picked;a,b=poses[i],poses[j]
                result.extend([(a[0],a[1],heading),(b[0],b[1],heading),b]);i=j;shortcuts+=1
            else:i+=1;result.append(poses[i])
        return result,shortcuts

    def _result(self,poses,goal,expanded,diagnostics):
        self._expire_live_surface()
        if self._planning_revision is not None and self._planning_revision!=self.grid_revision:
            raise ValueError('规划期间实时通行观测已变化或过期，请更新扫描后重新规划')
        dense=[]
        def append(x,y,yaw,kind):
            yaw=math.atan2(math.sin(yaw),math.cos(yaw))
            row=dict(x=float(x),y=float(y),yaw=yaw,kind=kind)
            if dense and math.dist((x,y),(dense[-1]['x'],dense[-1]['y']))<1e-8 and abs(math.atan2(math.sin(yaw-dense[-1]['yaw']),math.cos(yaw-dense[-1]['yaw'])))<1e-8:return
            dense.append(row)
        append(*poses[0],'start')
        for a,b in zip(poses,poses[1:]):
            length=math.dist(a[:2],b[:2]);delta=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            if length<1e-8:
                steps=max(1,math.ceil(abs(delta)/(math.pi/36)))
                for i in range(1,steps+1):append(a[0],a[1],a[2]+delta*i/steps,'rotate')
            else:
                steps=max(1,math.ceil(length/self.res))
                for i in range(1,steps+1):
                    if i==steps:append(b[0],b[1],b[2],'travel')
                    else:append(a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps,b[2],'travel')
        path=[]
        for p in dense:
            if not path or math.hypot(p['x']-path[-1]['x'],p['y']-path[-1]['y'])>1e-8:path.append(dict(x=p['x'],y=p['y']))
        guide=[];last=0
        for i in range(1,len(path)-1):
            a,b,c=path[i-1],path[i],path[i+1]
            before=math.atan2(b['y']-a['y'],b['x']-a['x']);after=math.atan2(c['y']-b['y'],c['x']-b['x'])
            turn=abs(math.atan2(math.sin(after-before),math.cos(after-before)))
            if turn>.05 or math.hypot(b['x']-path[last]['x'],b['y']-path[last]['y'])>=2.5:
                guide.append(dict(b,yaw=after,type=0));last=i
        guide.append(dict(path[-1],yaw=dense[-1]['yaw'],type=1))
        length=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(path,path[1:]))
        diagnostics.update(live_surface=self.live_surface_status(),grid_revision=self.grid_revision,potential_expanded=self._potential_expanded,
                           dense_poses=len(dense),rotation_samples=sum(p['kind']=='rotate' for p in dense),static_speckles=dict(self.static_speckles))
        if self._planning_revision is not None and self._planning_revision!=self.grid_revision:
            raise ValueError('规划期间实时通行观测已变化或过期，请更新扫描后重新规划')
        return dict(ok=True,path=path,dense_path=dense,points=guide,length_m=round(length,2),expanded=expanded,
                    goal=guide[-1],requested_goal=dict(x=goal[0],y=goal[1]),clearance_margin_m=CLEARANCE_MARGIN,
                    self_projection_ignored=self.self_projection_ignored,diagnostics=diagnostics,static_speckles=dict(self.static_speckles),
                    planner='footprint_any_angle_v3',message='已生成机身包络验证的参考路径；执行器仍须依据实时观测跟踪与避障')

    def _nearby_goal_cells(self,goal,tolerance):
        """Stay in the clicked point's local free component, never across a wall."""
        from collections import deque
        gx,gy=self.cell(*goal)
        if not (0<=gx<self.w and 0<=gy<self.h) or not self.free[gy,gx]:return []
        todo=deque([(gx,gy)]);seen={(gx,gy)};result=[]
        while todo:
            self._check_time();x,y=todo.popleft();point=self.world(x,y)
            distance=math.dist(point,goal)
            if distance<=tolerance+1e-9:result.append((distance,x,y))
            for nx,ny in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
                if ((nx,ny) not in seen and 0<=nx<self.w and 0<=ny<self.h and self.free[ny,nx]
                        and math.dist(self.world(nx,ny),goal)<=tolerance+self.res*.71):
                    seen.add((nx,ny));todo.append((nx,ny))
        return sorted(result)

    def plan(self,start,goal,yaw=None,budget=3.,*,goal_tolerance=None):
        # The per-planner setting is retained for its start-recovery adapter;
        # callers that need an exact endpoint retain the default zero tolerance.
        if goal_tolerance is not None:
            value=float(goal_tolerance)
            if not math.isfinite(value) or not 0<=value<=1.:raise ValueError('目标调整范围必须为0到1米')
            self.goal_tolerance=value
        self._expire_live_surface();self._planning_revision=self.grid_revision
        self._deadline=time.monotonic()+max(0,float(budget));self._potential_expanded=0
        try:
            try:result=self._plan(start,goal,yaw,budget)
            except GoalSpaceError as original:
                if self.goal_tolerance<=0:raise
                result=None;attempts=0
                for distance,gx,gy in self._nearby_goal_cells(goal,self.goal_tolerance):
                    self._check_time();point=self.world(gx,gy)
                    headings=([yaw] if yaw is not None else
                              [math.atan2(point[1]-start[1],point[0]-start[0])]+[h*math.pi/4 for h in range(8)])
                    if not any(self.pose_fits(*point,h) for h in headings):continue
                    attempts+=1
                    try:result=self._plan(start,point,yaw,max(0.,self._deadline-time.monotonic()))
                    except ValueError:
                        self._check_time()
                        if attempts>=16:break
                        continue
                    result['requested_goal']=dict(x=float(goal[0]),y=float(goal[1]))
                    result['goal_adjustment']=dict(applied=True,requested=dict(x=float(goal[0]),y=float(goal[1]),yaw=yaw),
                        actual=dict(result['goal']),distance_m=round(distance,3),tolerance_m=self.goal_tolerance,
                        reason='requested_body_does_not_fit',approach_only=True,checked_candidates=attempts)
                    result['message']='目标附近空间不足，已规划到相距 %.2f 米的可站立位置'%distance
                    break
                if result is None:raise GoalSpaceError('目标及附近 %.2f 米内都没有已核验的可达站立位置'%self.goal_tolerance) from original
            result.setdefault('goal_adjustment',dict(applied=False,distance_m=0.,tolerance_m=self.goal_tolerance))
            result['requires_live_revalidation']=self.live_surface is not None
            return result
        finally:self._deadline=None;self._planning_revision=None

    def _plan(self,start,goal,yaw,budget):
        if not all(math.isfinite(float(v)) for v in [*start,*goal]+([] if yaw is None else [yaw])):raise ValueError('导航坐标必须为有限数值')
        began=time.monotonic();self._check_time()
        sx,sy=self.cell(start[0],start[1]);gx,gy=self.cell(goal[0],goal[1])
        if not 0<=gx<self.w or not 0<=gy<self.h:raise ValueError('目标位于地图范围外')
        if not self.free[gy,gx]:raise ValueError('目标落在障碍、未知区域或边界，请选择附近已知空地')
        if not self.pose_fits(*start):raise ValueError('机器人起点过于靠近障碍，无法规划当前姿态；请核对定位或移到空地')
        line_yaw=math.atan2(goal[1]-start[1],goal[0]-start[0]) if math.dist(start[:2],goal)>.001 else start[2]
        final_yaw=line_yaw if yaw is None else yaw
        soft_direct=self._segment_soft_cost(start,goal)>.01
        if (not soft_direct and self.segment_fits(start,goal,line_yaw) and self.world_rotation_fits(start,start[2],line_yaw)
                and self.world_rotation_fits(goal,line_yaw,final_yaw)):
            self._check_time()
            return self._result([tuple(start),(start[0],start[1],line_yaw),(goal[0],goal[1],line_yaw),
                                 (goal[0],goal[1],final_yaw)],goal,0,dict(method='continuous_direct',shortcuts=1,planning_ms=round((time.monotonic()-began)*1000,2)))
        gh=None if yaw is None else round(yaw/(math.pi/4))%8
        candidates=[h for h in range(8) if self.fits(gx,gy,h*4) and (gh is None or gh==h and self.rotation_fits(gx,gy,h*math.pi/4,yaw))]
        if not candidates:raise GoalSpaceError('目标周围空间不足以容纳机器人，请离墙或障碍远一些')
        sh=round(start[2]/(math.pi/4))%8
        original_cell=(sx,sy);start_bridge=0.
        centre=(*self.world(sx,sy),start[2])
        if not (self.segment_fits(start,centre,start[2]) and self.rotation_fits(sx,sy,start[2],sh*math.pi/4)):
            # A valid continuous pose need not fit at its rounded cell centre.
            # Join an adjacent centre through an explicitly checked swept-body
            # segment; the returned path still begins at the actual robot pose.
            choices=[]
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    nx,ny=original_cell[0]+dx,original_cell[1]+dy
                    if not (0<=nx<self.w and 0<=ny<self.h):continue
                    point=(*self.world(nx,ny),start[2]);length=math.dist(start[:2],point[:2])
                    if (length<=2*self.res and self.segment_fits(start,point,start[2])
                            and self.rotation_fits(nx,ny,start[2],sh*math.pi/4)):
                        choices.append((length,nx,ny))
            if not choices:raise ValueError('机器人起点过于靠近障碍，无法规划当前姿态；请核对定位或移到空地')
            start_bridge,sx,sy=min(choices)
        heuristic=self._potential(gx,gy);first=(sx,sy,sh);initial=heuristic(sx,sy)
        if not math.isfinite(initial):raise ValueError('目标与当前位置之间没有足够宽的已知通路；请检查门口、障碍或地图连通性')
        distance={first:0.};parent={};queue=[(initial,0.,first)];expanded=0;end=None
        directions=[(round(math.cos(h*math.pi/4)),-round(math.sin(h*math.pi/4))) for h in range(8)]
        while queue:
            _,cost,p=heapq.heappop(queue)
            if cost!=distance.get(p):continue
            x,y,h=p;expanded+=1
            if (x,y)==(gx,gy) and h in candidates:end=p;break
            if expanded%128==0:self._check_time()
            if expanded>250000:raise ValueError('地图搜索范围较大，本次未找到路径；可选更近的目标再规划')
            moves=[]
            for offset,penalty in ((0,1.),(4,1.25),(-2,1.4),(2,1.4)):
                dx,dy=directions[(h+offset)%8];nx,ny=x+dx,y+dy
                if self.fits(nx,ny,h*4) and (not dx or not dy or self.fits(x+dx,y,h*4) and self.fits(x,y+dy,h*4)):
                    moves.append(((nx,ny,h),penalty*math.hypot(dx,dy)*self.res*(1+.18/max(.1,self.clearance[ny,nx])+self.soft_cost[ny,nx])))
            for turn in [-1,1]:
                if self.rotation_fits(x,y,h*math.pi/4,(h+turn)*math.pi/4):moves.append(((x,y,(h+turn)%8),.25))
            for nxt,step in moves:
                total=cost+step
                if total<distance.get(nxt,math.inf):
                    distance[nxt]=total;parent[nxt]=p
                    heapq.heappush(queue,(total+heuristic(nxt[0],nxt[1]),total,nxt))
        if end is None:raise ValueError('目标与当前位置之间没有足够宽的已知通路；请检查门口、障碍或地图连通性')
        states=[end]
        while states[-1]!=first:states.append(parent[states[-1]])
        states.reverse();poses=[(*self.world(x,y),h*math.pi/4) for x,y,h in states]
        # Bridge subcell endpoints only with a checked swept body. No snapping
        # through a wall, including at the very first/last centimetres.
        if not self.segment_fits(start,poses[0],start[2]):raise ValueError('起点到规划栅格中心的空间不足，请核对定位')
        poses=[tuple(start),(poses[0][0],poses[0][1],start[2])]+poses
        final_yaw=poses[-1][2] if yaw is None else yaw
        if not self.segment_fits(poses[-1],goal,poses[-1][2]) or not self.world_rotation_fits(goal,poses[-1][2],final_yaw):
            raise GoalSpaceError('目标精确位置或朝向空间不足，请选择附近已知空地')
        poses.extend([(goal[0],goal[1],poses[-1][2]),(goal[0],goal[1],final_yaw)])
        raw_length=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))
        # Budget includes smoothing; if little budget remains return the already
        # validated lattice path rather than throw away the solved route.
        shortcuts=0
        if self._deadline-time.monotonic()>.15:
            try:poses,shortcuts=self._smooth(poses)
            except ValueError:pass
        return self._result(poses,goal,expanded,dict(method='obstacle_potential_lattice_shortcuts',shortcuts=shortcuts,
                            raw_length_m=round(raw_length,3),start_cell_adjusted=(sx,sy)!=original_cell,start_bridge_m=round(start_bridge,4),planning_ms=round((time.monotonic()-began)*1000,2)))


def live_obstacles(packet,start,round_trip=0):
    """Use only fresh, matched localization observations; never clear static cells."""
    import base64
    import numpy as np
    from map_fusion import pose_matrix,transform
    from autonomy_sensing import supported_floor
    age=float(packet.get('age',99))+round_trip
    if packet.get('protocol')!=1 or packet.get('frame_id')!='map' or not math.isfinite(age) or not 0<=age<=.7:raise ValueError('实时障碍点云过期')
    pose=np.asarray(packet['pose'],float);quality=packet.get('quality',[])
    if len(quality)!=2 or not np.isfinite(quality).all() or quality[0]>.25 or quality[1]<.85:raise ValueError('实时定位质量不足')
    if pose.shape!=(7,) or not np.isfinite(pose).all() or not .5<np.linalg.norm(pose[3:])<1.5 or math.dist(pose[:2],start[:2])>.4:raise ValueError('障碍点云与当前位姿不匹配')
    points=np.frombuffer(base64.b64decode(packet['points'],validate=True),dtype='<f4').reshape(-1,3)
    if len(points)>60000:raise ValueError('障碍点云过大')
    T=pose_matrix(pose);body=transform(points,np.linalg.inv(T));scan=supported_floor(body)
    cells=[]
    for x,y in scan['obstacles']:
        for dx in [-.1,0,.1]:
            for dy in [-.1,0,.1]:
                cells.append([x*.2+dx,y*.2+dy,0])
    if not cells:return []
    return transform(np.array(cells),T)[:,:2].tolist()

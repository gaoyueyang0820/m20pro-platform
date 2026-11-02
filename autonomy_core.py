"""Offline-testable exploration and visual servoing. Commands are normalized axes.

Only measured, supported floor is traversable. This deliberately targets level
indoor floors; neither a camera bounding box nor a laser ray proves floor support.
"""
import math
import time
from collections import deque
from robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH
from exploration_frontiers import FrontierSelector

RESOLUTION = .2
RADIUS = .65  # Conservative circular envelope for the complete wheel-legged body.
SUPPORT_RADIUS = .50  # Encloses the .84 x .506 m contact/body projection; clearance is separate.


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cell(x, y):
    return round(x / RESOLUTION), round(y / RESOLUTION)


def neighbours(p):
    x, y = p
    return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]


def disk(radius):
    n = math.ceil(radius / RESOLUTION)
    return [(x,y) for x in range(-n,n+1) for y in range(-n,n+1)
            if math.hypot(x,y)*RESOLUTION <= radius]


FOOTPRINT = disk(RADIUS)
SUPPORT = disk(SUPPORT_RADIUS)


def body_offsets(angle=0.,support=False):
    """Cell/rectangle intersections for the installed M20 planner footprint.

    Straight clearance includes 3 cm per side. Rotations are checked using
    swept orientations, not by forcing every corridor to fit a full disk.
    """
    hx,hy=(BODY_HALF_LENGTH,BODY_HALF_WIDTH) if support else (CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH)
    c,s=math.cos(angle),math.sin(angle);a,b=abs(c),abs(s);half=RESOLUTION/2
    n=math.ceil((math.hypot(hx,hy)+half*math.sqrt(2))/RESOLUTION);result=[]
    for ix in range(-n,n+1):
        for iy in range(-n,n+1):
            x,y=ix*RESOLUTION,iy*RESOLUTION
            if (abs(x)<hx*a+hy*b+half and abs(y)<hx*b+hy*a+half
                    and abs(c*x+s*y)<hx+half*(a+b) and abs(-s*x+c*y)<hy+half*(a+b)):
                result.append((ix,iy))
    return result


def supported_cells(scan):
    """Fill isolated scan gaps enclosed on four sides by measured floor.

    No cascading interpolation, no boundary expansion, no filling obstacles or
    measured depressions. A connected missing area remains unknown.
    """
    measured={tuple(p) for p in scan['floor']};blocked={tuple(p) for p in scan['obstacles']}
    candidates={q for p in measured for q in neighbours(p)}-measured-blocked
    return measured | {p for p in candidates if all(q in measured for q in neighbours(p))}


def clearance(scan, forward, yaw, side=0.):
    """Check the swept footprint in a CURRENT body-frame scan, including support."""
    if not forward and not yaw and not side:
        return True, ''
    if not all(math.isfinite(float(v)) for v in [forward,side,yaw,scan.get('_speed_mps',0)]):return False,'运动参数或速度反馈无效，已停车'
    free = supported_cells(scan)
    blocked = {tuple(p) for p in scan['obstacles']}
    if not free:return False,'尚未收到有效地面观测，已停车'
    # Standing footprint is already occupied by the robot, but do not infer any
    # free space beyond it. A sparse/missing floor ahead never becomes driveable.
    standing = set(disk(.55))
    speed=max(0.,float(scan.get('_speed_mps',0)))
    # Keep more space at measured speed; stationary creep need not reserve
    # 1.45 m in front or 1.30 m of corridor width.
    magnitude=math.hypot(forward,side)
    ahead=max(.3,speed*.9+.1) if magnitude else 0.
    collision=set();support=set()
    angles=[math.copysign(i*math.pi/12,yaw) for i in range(7)] if yaw else [0.]
    for angle in angles:
        collision.update(body_offsets(angle));support.update(body_offsets(angle,True))
    for step in range(math.ceil(ahead/RESOLUTION)+1):
        cx=step*forward/magnitude if magnitude else 0.;cy=step*side/magnitude if magnitude else 0.
        if abs(cx-round(cx))<1e-9:cx=round(cx)
        if abs(cy-round(cy))<1e-9:cy=round(cy)
        # Supercover fractional cells instead of cutting a diagonal corner.
        centers={(ix,iy) for ix in [math.floor(cx),math.ceil(cx)] for iy in [math.floor(cy),math.ceil(cy)]}
        for ix,iy in centers:
            for dx,dy in collision:
                if (ix+dx,iy+dy) in blocked:return False,'行进方向或转身范围有障碍，已停车'
            for dx,dy in support:
                p=(ix+dx,iy+dy)
                if p not in free and p not in standing:return False,'行进方向地面尚未看清，已停车'
    return True, ''


class Explorer:
    def __init__(self, radius=8, forward_limit=.35, yaw_limit=.70):
        self.radius = radius
        self.forward_limit=forward_limit
        self.yaw_limit=yaw_limit
        self.free = set(); self.obstacles = set(); self.path = []
        self.origin = None; self.last_pose = None; self.goal = None
        self.last_stamp=None
        self.prior=None;self.target_kind='observe'
        self.visited = set(); self.distance = 0.; self.replans = 0
        self.recovery=None;self.recovery_attempts=0;self.motion_hint='等待规划';self.recovery_limit=.4
        self.plan_state='idle'
        self.distance_anchor=None
        self._current_yaw_rate=0.
        self.frontiers=FrontierSelector(RESOLUTION)
        self.frontier_metrics={}
        self._blocked_goal=None

    def ingest(self, scan):
        self.current_scan=scan
        x,y,yaw = scan['pose']
        if not all(math.isfinite(float(v)) for v in [x,y,yaw]):raise ValueError('建图位姿包含无效数值')
        if self.origin is None:
            self.origin = (x,y)
            self.initial_pose=(x,y,yaw)
        if self.last_pose:
            if self.last_stamp is not None and scan.get('stamp',self.last_stamp)<self.last_stamp:raise ValueError('建图时间戳回退，请停稳后重新开始')
            jump = math.hypot(x-self.last_pose[0],y-self.last_pose[1])
            dt=max(0,scan.get('stamp',0)-(self.last_stamp or scan.get('stamp',0)))
            # After a stopped sensing gap, a real turn spans more than one
            # sample. Compare angle with elapsed sensor time, not frame count.
            turn_limit=max(.6,min(2.2,2.5*dt+.15))
            if jump > .8 or abs(wrap(yaw-self.last_pose[2])) > turn_limit:
                raise ValueError('建图位姿发生跳变，请停稳后重新开始探索')
        if self.distance_anchor is None:self.distance_anchor=(x,y)
        # Do not present accumulated stationary SLAM jitter as walking distance.
        if scan.get('_speed_mps',1.)<.08:self.distance_anchor=(x,y)
        elif math.dist((x,y),self.distance_anchor)>=.06:
            self.distance+=math.dist((x,y),self.distance_anchor);self.distance_anchor=(x,y)
        self.last_pose = (x,y,yaw)
        self.frontiers.observe_pose(self.last_pose)
        self.visited.add(cell(x,y))
        self.last_stamp=scan.get('stamp')
        c,s = math.cos(yaw), math.sin(yaw)
        def world(p):
            px,py = p[0]*RESOLUTION,p[1]*RESOLUTION
            return cell(x+c*px-s*py,y+s*px+c*py)
        # Forward-rounding a rotated grid creates holes even in continuous floor.
        # The global grid is a coarse route hint. Sample cell centres so a tiny
        # SLAM translation does not erode a whole row of measured floor. Every
        # actual command still checks the full footprint in the fresh local scan.
        measured={tuple(p) for p in scan['floor']}
        observed=supported_cells(scan) | set(disk(.55));cleared=set()
        bounds=[world(p) for p in observed]
        if bounds:
            left,right=min(p[0] for p in bounds)-1,max(p[0] for p in bounds)+1
            bottom,top=min(p[1] for p in bounds)-1,max(p[1] for p in bounds)+1
            for gx in range(left,right+1):
                for gy in range(bottom,top+1):
                    if math.hypot(gx*RESOLUTION-self.origin[0],gy*RESOLUTION-self.origin[1])>self.radius+1:continue
                    wx,wy=gx*RESOLUTION-x,gy*RESOLUTION-y
                    local=cell(c*wx+s*wy,-s*wx+c*wy)
                    if local in observed:self.free.add((gx,gy))
                    if local in measured:cleared.add((gx,gy))
        start=cell(x,y)
        self.free.update((start[0]+dx,start[1]+dy) for dx,dy in SUPPORT)
        # A transient obstacle must not block the start forever after a later
        # scan observes ground there. Mere absence of returns does not clear it.
        self.obstacles.difference_update(cleared)
        self.obstacles.update(world(p) for p in scan['obstacles'])
        self.free.difference_update(self.obstacles)
        # Finite per-run exploration bounds keep memory and route search bounded.
        within = lambda p: math.hypot(p[0]*RESOLUTION-self.origin[0],p[1]*RESOLUTION-self.origin[1]) <= self.radius+1
        self.free = {p for p in self.free if within(p)}
        self.obstacles = {p for p in self.obstacles if within(p)}

    def plan(self):
        start = cell(*self.last_pose[:2])
        support=body_offsets(self.last_pose[2],True);footprint=body_offsets(self.last_pose[2])
        allowed = {p for p in self.free if all((p[0]+dx,p[1]+dy) in self.free for dx,dy in support)
                   and not any((p[0]+dx,p[1]+dy) in self.obstacles for dx,dy in footprint)
                   and math.hypot(p[0]*RESOLUTION-self.origin[0],p[1]*RESOLUTION-self.origin[1]) <= self.radius}
        if self.prior:
            self.prior.bind(self.initial_pose)
            # Old occupancy guides what to revisit, not whether a currently
            # measured corridor exists. Live floor/obstacles decide traversal;
            # permanent map changes still require the separate evidence policy.
        if start not in allowed:
            self.path=[]
            self.plan_state='recovering' if self._recovery_available() else 'blocked';return self.plan_state
        can_forward=clearance(self.current_scan,.1,0)[0]
        turn_left=clearance(self.current_scan,0,.1)[0];turn_right=clearance(self.current_scan,0,-.1)[0]
        def possible(heading):
            error=wrap(heading-self.last_pose[2])
            if abs(error)<.2:return can_forward
            if (turn_left if error>0 else turn_right):return True
            f,s=self._translation(error)
            return self._inside_bound(f,s) and clearance(self.current_scan,f,0,side=s)[0]
        existing_heading=math.atan2(self.path[min(2,len(self.path)-1)][1]*RESOLUTION-self.last_pose[1],self.path[min(2,len(self.path)-1)][0]*RESOLUTION-self.last_pose[0]) if self.path else 0
        if self.goal in allowed and self.goal not in self.visited and self.path and all(p in allowed for p in self.path) and possible(existing_heading):
            self.plan_state='planned';return 'planned'
        self.replans += 1
        within=lambda p: math.hypot(p[0]*RESOLUTION-self.origin[0],p[1]*RESOLUTION-self.origin[1])<=self.radius
        prior_kind=(lambda p:self.prior.kind(p[0]*RESOLUTION,p[1]*RESOLUTION)) if self.prior else None
        selected=self.frontiers.select(start,self.last_pose[2],allowed,self.free,self.obstacles,within,
                    self.last_stamp if self.last_stamp is not None else time.monotonic(),prior_kind,possible,self.visited)
        self.frontier_metrics=dict(self.frontiers.last)
        if selected is None:
            self.path=[];self.goal=None
            self.plan_state='blocked' if self.frontier_metrics.get('deferred_regions',0) else 'exhausted'
            return self.plan_state
        self.goal=selected['goal'];self.path=selected['path'];self.target_kind=selected['kind']
        self.plan_state='planned'
        return 'planned'

    def note_blocked(self,reason=''):
        """Defer a repeatedly blocked approach, not the entire exploration run.

        Runtime calls this only after measured lack of progress/blocked commands.
        A changed local obstacle set or a finite cooldown reopens the region.
        """
        self.frontiers.blocked(self.goal,self.last_stamp if self.last_stamp is not None else time.monotonic(),self.obstacles,reason)
        self.path=[];self.goal=None;self.recovery=None;self.recovery_attempts=0

    def command(self,yaw_rate=0.,observation_age=0.):
        x,y,yaw = self.last_pose
        while self.path and math.hypot(self.path[0][0]*RESOLUTION-x,self.path[0][1]*RESOLUTION-y) < .25:
            self.visited.add(self.path.pop(0))
        if not self.path:
            return 0.,0.
        gx,gy = self.path[min(2,len(self.path)-1)]
        error = wrap(math.atan2(gy*RESOLUTION-y,gx*RESOLUTION-x)-yaw-yaw_rate*min(.65,max(0,observation_age)))
        # Normalized joystick input has a substantial turning dead zone on M20.
        # Turn in place, then translate; do not combine minimum turn with forward.
        if abs(error)>=.20:
            if error*yaw_rate<0 and abs(yaw_rate)>.25:return 0.,0.
            return 0.,math.copysign(self.yaw_limit,error)
        if abs(yaw_rate)>.25:return 0.,0.
        return self.forward_limit,0.

    def _translation(self,error):
        # Lateral/reverse commands are deliberately smaller than open-space
        # forward travel. These are joystick amplitudes, not metres per second.
        f=max(-min(.25,self.forward_limit),self.forward_limit*math.cos(error));side=max(-.25,min(.25,self.forward_limit*math.sin(error)))
        return (0. if abs(f)<1e-9 else f),(0. if abs(side)<1e-9 else side)

    def _inside_bound(self,f,side,distance=.4):
        if self.origin is None or self.last_pose is None:return False
        n=math.hypot(f,side)
        if not n:return True
        x,y,yaw=self.last_pose;c,s=math.cos(yaw),math.sin(yaw)
        return math.hypot(x+(c*f-s*side)*distance/n-self.origin[0],y+(s*f+c*side)*distance/n-self.origin[1])<=self.radius

    def _direct_goal(self):
        if not self.path:return 1.,0.
        x,y,yaw=self.last_pose;gx,gy=self.path[min(2,len(self.path)-1)]
        dx,dy=gx*RESOLUTION-x,gy*RESOLUTION-y;c,s=math.cos(yaw),math.sin(yaw)
        return c*dx+s*dy,-s*dx+c*dy

    def _escape_candidates(self):
        if self.recovery_attempts>=3:return []
        gx,gy=self._direct_goal();n=max(.001,math.hypot(gx,gy));scored=[]
        for f,side in [(self.forward_limit,0.),(0.,min(.25,self.forward_limit)),(0.,-min(.25,self.forward_limit)),(-min(.25,self.forward_limit),0.)]:
            if not self._inside_bound(f,side) or not clearance(self.current_scan,f,0,side=side)[0]:continue
            magnitude=math.hypot(f,side);dx=round(f/magnitude*.4/RESOLUTION);dy=round(side/magnitude*.4/RESOLUTION)
            preview=dict(self.current_scan,floor=[[a-dx,b-dy] for a,b in self.current_scan['floor']],obstacles=[[a-dx,b-dy] for a,b in self.current_scan['obstacles']],_speed_mps=0.)
            opens_forward=clearance(preview,self.forward_limit,0)[0]
            score=(gx*f+gy*side)/(n*magnitude)+2*opens_forward-(.25 if f<0 else 0)
            scored.append((score,f,side))
        return sorted(scored,reverse=True)

    def _recovery_available(self):return self.recovery is not None or bool(self._escape_candidates())

    def _motion_ready(self,f,side):
        velocity=self.current_scan.get('_velocity_body')
        if velocity is None:return '_velocity_body' not in self.current_scan
        if len(velocity)!=2 or not all(math.isfinite(float(v)) for v in velocity):return False
        vx,vy=velocity;speed=math.hypot(vx,vy);desired=math.hypot(f,side)
        if speed<.08:return True
        if not desired:return False
        along=(vx*f+vy*side)/desired;across=abs(vx*side-vy*f)/desired
        return along>=0 and across<.08

    def _recover_axes(self):
        if self.recovery is None:
            options=self._escape_candidates()
            if not options:self.motion_hint='没有已确认的退让通路';return 0.,0.,0.
            _,f,side=options[0];self.recovery_attempts+=1
            self.recovery=dict(forward=f,side=side,origin=self.last_pose[:2],stamp=self.current_scan.get('stamp',time.monotonic()))
        move=self.recovery;elapsed=self.current_scan.get('stamp',time.monotonic())-move['stamp']
        travelled=math.dist(self.last_pose[:2],move['origin'])
        if travelled>=self.recovery_limit or elapsed>3.:
            self.recovery=None;self.path=[];self.motion_hint='退让已结束，重新检查通路';return 0.,0.,0.
        f,side=move['forward'],move['side']
        if abs(self._current_yaw_rate)>.25 or not self._motion_ready(f,side):
            self.motion_hint='先停稳再退让';return 0.,0.,0.
        if not self._inside_bound(f,side) or not clearance(self.current_scan,f,0,side=side)[0]:
            self.recovery=None;self.motion_hint='退让方向已被阻挡';return 0.,0.,0.
        self.motion_hint='向左侧移让开' if side>0 else '向右侧移让开' if side<0 else '短距离后退脱困' if f<0 else '短距离前移重新观察'
        return f,side,0.

    def command_axes(self,yaw_rate=0.,observation_age=0.):
        """Choose a checked translation or turn, with bounded local recovery.

        This does not extend a sensor lease or prove real hardware tracking. The
        runtime rechecks the same three-axis sweep before issuing each command.
        """
        self._current_yaw_rate=yaw_rate
        if '_velocity_body' in self.current_scan and self.current_scan['_velocity_body'] is None:
            self.motion_hint='等待最新运动反馈';return 0.,0.,0.
        if self.recovery is not None:return self._recover_axes()
        f,yaw=self.command(yaw_rate,observation_age)
        if not self.path:
            if self.goal in self.visited:
                self.recovery_attempts=0
                if self.plan()=='planned':f,yaw=self.command(yaw_rate,observation_age)
            if not self.path:return self._recover_axes() if self.plan_state=='recovering' else (0.,0.,0.)
        if abs(yaw_rate)>.25:
            if yaw and self._motion_ready(0,0) and clearance(self.current_scan,0,yaw)[0]:self.motion_hint='转向对准通路';return 0.,0.,yaw
            self.motion_hint='等待转向停稳';return 0.,0.,0.
        if f and self._inside_bound(f,0) and clearance(self.current_scan,f,0)[0]:
            if not self._motion_ready(f,0):self.motion_hint='先停稳再改变行进方向';return 0.,0.,0.
            self.motion_hint='沿通路前进';return f,0.,0.
        if yaw and clearance(self.current_scan,0,yaw)[0]:
            if not self._motion_ready(0,0):self.motion_hint='先停稳再转向';return 0.,0.,0.
            self.motion_hint='转向对准通路';return 0.,0.,yaw
        gx,gy=self._direct_goal();forward,side=self._translation(math.atan2(gy,gx))
        if self._inside_bound(forward,side) and clearance(self.current_scan,forward,0,side=side)[0]:
            if not self._motion_ready(forward,side):self.motion_hint='先停稳再改变行进方向';return 0.,0.,0.
            self.motion_hint='保持朝向侧移通过' if abs(side)>.1 else '短距离后退通过' if forward<0 else '沿通路前进'
            return forward,side,0.
        return self._recover_axes()

    def view(self):
        return dict(floor=list(self.free),obstacles=list(self.obstacles),path=self.path,
                    pose=self.last_pose,origin=self.origin,resolution=RESOLUTION,
                    area=round(len(self.free)*RESOLUTION**2,1),distance=round(self.distance,2),net_displacement=round(math.dist(self.last_pose[:2],self.origin),2) if self.last_pose and self.origin else 0,replans=self.replans,target_kind=self.target_kind,motion_hint=self.motion_hint,recovery_attempts=self.recovery_attempts,frontier_metrics=dict(self.frontier_metrics))


class PriorMap:
    """Old-map labels guide priority; they never substitute for live floor."""
    def __init__(self,grid,anchor):
        import base64
        from navigation_planner import pgm_pixels
        self.w,self.h,self.pixels=pgm_pixels(base64.b64decode(grid['pgm'],validate=True))
        self.res=grid['resolution'];self.origin=grid['origin'];self.negate=grid.get('negate',0)
        self.free=grid.get('free_thresh',.196);self.occupied=grid.get('occupied_thresh',.65)
        self.anchor=anchor;self.first=None

    def bind(self,first):
        if self.first is None:
            self.first=first;angle=self.anchor['yaw']-first[2];self.c=math.cos(angle);self.s=math.sin(angle)

    def kind(self,x,y):
        dx,dy=x-self.first[0],y-self.first[1]
        x=self.anchor['x']+self.c*dx-self.s*dy;y=self.anchor['y']+self.s*dx+self.c*dy
        gx=math.floor((x-self.origin[0])/self.res);gy=self.h-1-math.floor((y-self.origin[1])/self.res)
        if not 0<=gx<self.w or not 0<=gy<self.h:return 'unknown'
        p=self.pixels[gy*self.w+gx]/255;p=p if self.negate else 1-p
        return 'wall' if p>self.occupied else 'known' if p<self.free else 'unknown'


def iou(a,b):
    intersection = max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union > 0 else 0.


class PersonLock:
    """Deliberately no automatic re-identification after occlusion/crossing."""
    def __init__(self, box):
        self.box = list(box); self.reference_height = box[3]-box[1]; self.lost = False

    def update(self, detections):
        if self.lost:
            raise ValueError('目标已丢失，请重新点选要跟随的人')
        matches = [d['box'] for d in detections if d['confidence']>=.55 and iou(self.box,d['box'])>.25]
        if len(matches)!=1:
            self.lost = True
            raise ValueError('目标丢失或多人交叉，已停车；请重新点选')
        target = matches[0]
        if any(d['box']!=target and iou(target,d['box'])>.12 for d in detections):
            self.lost = True
            raise ValueError('人物重叠，已停车；请重新点选')
        height = target[3]-target[1]
        if height<=.12 or target[1]<.02 or target[3]>.98 or not .7<height/(self.box[3]-self.box[1])<1.4:
            self.lost = True
            raise ValueError('人物被截断或尺度突变，已停车；请重新点选')
        self.box = target
        horizontal = (target[0]+target[2])/2-.5
        # Image-size servo: selected distance is the reference, NOT metres.
        forward = min(.12,max(0.,(self.reference_height/height-1.08)*.25))
        if abs(horizontal)>.18:
            forward = 0.
        yaw = max(-.15,min(.15,-horizontal*.6)) if abs(horizontal)>.035 else 0.
        return forward,yaw

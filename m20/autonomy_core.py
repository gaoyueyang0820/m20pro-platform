"""Offline-testable exploration and visual servoing. Commands are normalized axes.

Only measured, supported floor is traversable. This deliberately targets level
indoor floors; neither a camera bounding box nor a laser ray proves floor support.
"""
import math
import time
from collections import deque
from m20.robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH,CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH
from m20.exploration_frontiers import FrontierSelector

RESOLUTION = .2
RADIUS = .65  # Conservative circular envelope for the complete wheel-legged body.
SUPPORT_RADIUS = .50  # Encloses the .84 x .506 m contact/body projection; clearance is separate.
DIAGONAL_DIRECTIONS = tuple((math.cos(i*math.pi/4), math.sin(i*math.pi/4)) for i in range(1,8,2))
OBLIQUE_DIRECTIONS = tuple((math.cos(i*math.pi/8), math.sin(i*math.pi/8)) for i in range(1,16,2))


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
    if scan.get('geometry_version')==2:
        # Share the follower's exact obstacle rectangles. A coarse occupied
        # cell may touch the body while the measured points inside it do not.
        from m20.local_trajectory import LocalSurface
        surface=LocalSurface.cached(scan)
        if surface.precise:
            if not scan.get('floor') and not scan.get('measured_floor'):return False,'尚未收到有效地面观测，已停车'
            magnitude=math.hypot(forward,side);speed=max(0.,float(scan.get('_speed_mps',0)))
            ahead=max(.3,speed*.9+.1) if magnitude else 0.
            # Retain the old rounded-up horizon, but check the continuous
            # swept rectangle rather than rounding obstacle positions to it.
            distance=math.ceil(ahead/RESOLUTION)*RESOLUTION
            dx=distance*forward/magnitude if magnitude else 0.;dy=distance*side/magnitude if magnitude else 0.
            if surface.sweep_valid(dx,dy,yaw):return True,''
            return False,('行进方向或转身范围有障碍，已停车' if surface.last_failure=='obstacle' else '行进方向地面尚未看清，已停车')
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
    def __init__(self, radius=8, forward_limit=.35, yaw_limit=.70, *, expand=False):
        self.radius = radius
        self.expand=expand
        self.forward_limit=forward_limit
        self.yaw_limit=yaw_limit
        self.free = set(); self.obstacles = set(); self.path = []
        self.origin = None; self.last_pose = None; self.goal = None
        self.last_stamp=None
        self.prior=None;self.target_kind='observe'
        self.visited = set(); self.distance = 0.; self.replans = 0
        self.recovery=None;self.recovery_attempts=0;self.motion_hint='等待规划';self.recovery_limit=.4
        self.recovery_episode=0;self.recovery_total_attempts=0;self.recovery_resets=0
        self.recovery_episode_anchor=None;self.recovery_history=set();self.recovery_new_cells=set()
        self.recovery_measured_distance=0.;self.recovery_last_translation=None
        self.recovery_reset_reason=None;self.recovery_block_reason=None;self.actual_visited=set()
        self.recovery_pending=False;self.recovery_stop_stamp=None;self.recovery_forced=False
        self.recovery_tried=set();self.recovery_tried_world=[];self.recovery_outcomes=[];self.recovery_request_reason=None
        self.recovery_candidate_failures=0
        self.plan_state='idle'
        self.distance_anchor=None
        self._traversed=deque(maxlen=12)
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
        self._observe_traversal(scan, (x,y,yaw))
        self._update_recovery_episode(scan)
        self.last_pose = (x,y,yaw)
        self.frontiers.observe_pose(self.last_pose)
        self.visited.add(cell(x,y))
        self.actual_visited.add(cell(x,y))
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
        # Map-based exploration follows newly observed passages instead of
        # treating the initial map extent as a permanent task radius.
        if self.expand and self.radius<100 and any(math.hypot(p[0]*RESOLUTION-self.origin[0],p[1]*RESOLUTION-self.origin[1])>=self.radius-.8 for p in self.free):
            self.radius=min(100,self.radius+6)

    def plan(self):
        start = cell(*self.last_pose[:2])
        support=body_offsets(self.last_pose[2],True);footprint=body_offsets(self.last_pose[2])
        within=lambda p: math.hypot(p[0]*RESOLUTION-self.origin[0],p[1]*RESOLUTION-self.origin[1])<=self.radius
        def is_allowed(p):
            return (p in self.free and within(p)
                    and all((p[0]+dx,p[1]+dy) in self.free for dx,dy in support)
                    and not any((p[0]+dx,p[1]+dy) in self.obstacles for dx,dy in footprint))
        if self.prior:
            self.prior.bind(self.initial_pose)
            # Old occupancy guides what to revisit, not whether a currently
            # measured corridor exists. Live floor/obstacles decide traversal;
            # permanent map changes still require the separate evidence policy.
        if not is_allowed(start):
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
        if self.path and self.goal is not None and self.goal not in self.visited and is_allowed(self.goal) and all(map(is_allowed,self.path)) and possible(existing_heading):
            self.plan_state='planned';return 'planned'
        self.replans += 1
        allowed={p for p in self.free if is_allowed(p)}
        prior_kind=(lambda p:self.prior.kind(p[0]*RESOLUTION,p[1]*RESOLUTION)) if self.prior else None
        selected=self.frontiers.select(start,self.last_pose[2],allowed,self.free,self.obstacles,within,
                    self.last_stamp if self.last_stamp is not None else time.monotonic(),prior_kind,possible,self.visited)
        self.frontier_metrics=dict(self.frontiers.last)
        if selected is None:
            self.path=[];self.goal=None
            status=self.frontier_metrics.get('search_status')
            self.plan_state=('searching' if status=='searching' else
                             'recovering' if status in ['maneuver','local_space'] and self._recovery_available() else
                             'blocked' if status in ['maneuver','local_space','deferred'] else 'exhausted')
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
        self.path=[];self.goal=None;self.recovery=None

    def _observe_traversal(self, scan, pose):
        """Keep the recent approach backed by paired pose and measured motion."""
        velocity = scan.get('_velocity_body')
        stamp = scan.get('stamp')
        if self.last_pose is None or self.last_stamp is None or stamp is None:
            return
        dx, dy = pose[0] - self.last_pose[0], pose[1] - self.last_pose[1]
        distance = math.hypot(dx, dy)
        dt = stamp - self.last_stamp
        if dt <= 0 or distance < .005:
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        valid_velocity = isinstance(velocity, (list, tuple)) and len(velocity) == 2 and all(
            isinstance(v, (float, int)) and math.isfinite(v) for v in velocity
        )
        if valid_velocity:
            speed = math.hypot(*velocity)
            along = (c*dx+s*dy)*velocity[0] + (-s*dx+c*dy)*velocity[1]
            if (dt <= .75 and speed >= .08 and distance <= 2*speed*dt+.04
                    and along >= .5*distance*speed):
                if self._traversed and math.dist(self.last_pose[:2], self._traversed[-1]) > .24:
                    self._traversed.clear()
                if not self._traversed:
                    self._traversed.append(self.last_pose[:2])
                if math.dist(pose[:2], self._traversed[-1]) >= .16:
                    self._traversed.append(pose[:2])
                return
        if distance > .12:
            # A pose correction, manual move without paired feedback, or gap
            # must not be recorded as part of the same measured approach.
            self._traversed.clear()

    def _update_recovery_episode(self,scan):
        """Earn a new escape budget by observed translation into NEW space.

        Path bookkeeping/goal changes do not prove motion: command() marks
        nearby path cells visited before the body centre actually reaches them.
        actual_visited is therefore maintained only by pose observations.
        """
        anchor=self.recovery_episode_anchor
        if anchor is None or not (self.recovery_attempts or self.recovery_candidate_failures):return
        pose=scan['pose'];stamp=scan.get('stamp');speed=scan.get('_speed_mps')
        known=isinstance(speed,(int,float)) and not isinstance(speed,bool) and math.isfinite(speed) and speed>=0
        has_velocity='_velocity_body' in scan;velocity=scan.get('_velocity_body')
        if has_velocity:
            known=known and isinstance(velocity,(list,tuple)) and len(velocity)==2 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in velocity)
        timed=isinstance(stamp,(int,float)) and math.isfinite(stamp) and isinstance(self.last_stamp,(int,float)) and stamp>self.last_stamp
        if known and timed and self.last_pose and speed>=.08:
            dt=stamp-self.last_stamp;step=math.dist(pose[:2],self.last_pose[:2])
            measured=not has_velocity or math.hypot(*velocity)>=.08
            # Pose drift or a sub-jump-sized localization correction cannot
            # buy a budget inconsistent with the measured translation speed.
            if measured and .005<=step<=min(.8,2*speed*dt+.08):
                self.recovery_measured_distance+=step
                self.recovery_last_translation=(tuple(pose[:2]),stamp)
                key=cell(*pose[:2])
                if key not in self.recovery_history:self.recovery_new_cells.add(key)
        last=self.recovery_last_translation
        recent=(known and timed and last is not None and 0<=stamp-last[1]<=.7
                and math.dist(pose[:2],last[0])<=.12)
        if (self.recovery is None and recent and math.dist(pose[:2],anchor)>=.6
                and self.recovery_measured_distance>=.6 and len(self.recovery_new_cells)>=3
                and cell(*pose[:2]) not in self.recovery_history):
            self.recovery_attempts=0;self.recovery_resets+=1;self.recovery_episode_anchor=None
            self.recovery_candidate_failures=0
            self.recovery_tried=set();self.recovery_tried_world=[]
            self.recovery_reset_reason='已实测平移至少0.6米并进入新区域，恢复下一处受阻的退让次数'
            self.recovery_block_reason=None

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

    def _traversed_directions(self, speed):
        x, y, yaw = self.last_pose
        c, s = math.cos(yaw), math.sin(yaw)
        directions = []
        for px, py in reversed(self._traversed):
            dx, dy = px-x, py-y
            distance = math.hypot(dx, dy)
            if not .30 <= distance <= 1.1:
                continue
            forward, side = (c*dx+s*dy)/distance, (-s*dx+c*dy)/distance
            if any(forward*f+side*d >= math.cos(math.pi/12) for f,d in directions):
                continue
            directions.append((forward, side))
            if len(directions) == 3:
                break
        return [(speed*f, speed*s) for f,s in directions]

    def _escape_candidates(self):
        if self.recovery_attempts>=3 or self.recovery_candidate_failures>=3:return []
        gx,gy=self._direct_goal();n=max(.001,math.hypot(gx,gy));scored=[]
        slow = min(.25, self.forward_limit)
        cardinal = [(self.forward_limit, 0.), (0., slow), (0., -slow), (-slow, 0.)]
        traversed = self._traversed_directions(slow)
        diagonals = [(slow*f, slow*s) for f,s in DIAGONAL_DIRECTIONS]
        oblique = [(slow*f, slow*s) for f,s in OBLIQUE_DIRECTIONS]
        forward_open = False
        for directions in (cardinal + traversed, diagonals, oblique):
            for f,side in dict.fromkeys(directions):
                if self._recovery_was_tried(f,side,0.):continue
                if not self._inside_bound(f,side) or not clearance(self.current_scan,f,0,side=side)[0]:continue
                magnitude=math.hypot(f,side);dx=round(f/magnitude*.4/RESOLUTION);dy=round(side/magnitude*.4/RESOLUTION)
                preview=dict(self.current_scan,floor=[[a-dx,b-dy] for a,b in self.current_scan['floor']],obstacles=[[a-dx,b-dy] for a,b in self.current_scan['obstacles']],_speed_mps=0.)
                if 'measured_floor' in self.current_scan:preview['measured_floor']=[[a-dx,b-dy] for a,b in self.current_scan['measured_floor']]
                if 'obstacle_bounds' in self.current_scan:
                    try:preview['obstacle_bounds']=[[a-dx*RESOLUTION,b-dy*RESOLUTION,c-dx*RESOLUTION,d-dy*RESOLUTION] for a,b,c,d in self.current_scan['obstacle_bounds']]
                    except (ValueError,TypeError):pass  # malformed bounds retain conservative legacy handling
                opens_forward=clearance(preview,self.forward_limit,0)[0]
                score=(gx*f+gy*side)/(n*magnitude)+2*opens_forward-(.25 if f<0 else 0)
                forward_open = forward_open or f > 0 and side == 0
                if (f, side) in traversed and not forward_open:
                    # An observed approach is a useful exit from a coarse-grid
                    # dead end. Its entire motion is still checked above.
                    score += 4
                scored.append((score,f,side,0.))
            if scored:break
        for yaw in [self.yaw_limit,-self.yaw_limit]:
            if not self._recovery_was_tried(0.,0.,yaw) and clearance(self.current_scan,0,yaw)[0]:
                scored.append((-.2,0.,0.,yaw))
        return sorted(scored,reverse=True)

    @staticmethod
    def _recovery_direction(forward,side,yaw):
        if abs(yaw)>max(abs(forward),abs(side)):return 'turn_left' if yaw>0 else 'turn_right'
        if abs(side)>abs(forward):return 'left' if side>0 else 'right'
        return 'forward' if forward>=0 else 'reverse'

    def _recovery_world_direction(self,forward,side,heading=None):
        length=math.hypot(forward,side)
        if length<1e-9:return None
        heading=self.last_pose[2] if heading is None else heading;c,s=math.cos(heading),math.sin(heading)
        return ((c*forward-s*side)/length,(s*forward+c*side)/length)

    def _recovery_was_tried(self,forward,side,yaw):
        direction=self._recovery_world_direction(forward,side)
        if direction is None:return self._recovery_direction(forward,side,yaw) in self.recovery_tried
        # Suppress small bearing changes without hiding the adjacent 22.5°
        # heading from the newly measured corridor search.
        same_heading = math.cos(math.pi/12)
        return any(direction[0]*old[0]+direction[1]*old[1]>=same_heading for old in self.recovery_tried_world)

    def _remember_recovery_direction(self,forward,side,yaw,heading=None):
        self.recovery_tried.add(self._recovery_direction(forward,side,yaw))
        direction=self._recovery_world_direction(forward,side,heading)
        if direction is not None:self.recovery_tried_world.append(direction)

    def _begin_recovery_episode(self):
        if self.recovery_episode_anchor is not None:return
        self.recovery_episode+=1;self.recovery_episode_anchor=tuple(self.last_pose[:2])
        self.recovery_history=set(self.actual_visited);self.recovery_new_cells=set()
        self.recovery_measured_distance=0.;self.recovery_last_translation=None;self.recovery_reset_reason=None
        self.recovery_tried=set();self.recovery_tried_world=[]
        self.recovery_candidate_failures=0

    def request_recovery(self,failed_axes=(0.,0.,0.),reason='有指令但没有实际进展',failed_yaw=None):
        """Start a bounded stuck episode; always brake before a new direction."""
        deadline=self.current_scan.get('_valid_until')
        if deadline is not None and (not isinstance(deadline,(int,float)) or not math.isfinite(deadline) or time.monotonic()>=deadline):
            self.recovery_block_reason='expired_observation';self.motion_hint='感知已过期，不能启动脱困';return False
        if self.recovery is not None or self.recovery_pending:return True
        if self.recovery_attempts>=3:self._no_recovery();return False
        velocity=self.current_scan.get('_velocity_body')
        if velocity is None or len(velocity)!=2 or not all(math.isfinite(float(v)) for v in velocity):
            self.recovery_block_reason='missing_motion_feedback';self.motion_hint='缺少新鲜运动反馈，不能启动脱困';return False
        self._begin_recovery_episode()
        forward, side, yaw = failed_axes
        # A blocked arc does not rule out either component in isolation.
        # Recovery only issues fixed-heading translations or in-place turns.
        if any(abs(v)>.1 for v in failed_axes) and not (yaw and (forward or side)):
            self._remember_recovery_direction(forward, side, yaw, heading=failed_yaw)
        if not self._escape_candidates():self._no_recovery();return False
        self.frontiers.blocked(self.goal,self.last_stamp if self.last_stamp is not None else time.monotonic(),self.obstacles,reason)
        self.goal=None;self.path=[];self.plan_state='recovering'
        self.recovery_pending=True;self.recovery_forced=True;self.recovery_stop_stamp=self.current_scan.get('stamp')
        self.recovery_request_reason=reason;self.motion_hint='先请求停车，等新一帧停稳反馈后更换脱困方向'
        return True

    def _no_recovery(self):
        if self.recovery_candidate_failures>=3:
            self.recovery_block_reason='candidates_not_issued';self.motion_hint='连续3次退让候选未能下发，等待稳定反馈或重新规划';return
        self.recovery_block_reason='attempts_exhausted' if self.recovery_attempts>=3 else 'no_verified_corridor'
        self.motion_hint=('本处退让已尝试3次，需实际走出到新区域后再试' if self.recovery_attempts>=3
                          else '正在等待通路恢复')

    def _recovery_available(self):
        if self.recovery is not None or self.recovery_pending:return True
        possible=bool(self._escape_candidates())
        if not possible:self._no_recovery()
        return possible

    def _motion_ready(self,f,side):
        velocity=self.current_scan.get('_velocity_body')
        if velocity is None:return '_velocity_body' not in self.current_scan
        if len(velocity)!=2 or not all(math.isfinite(float(v)) for v in velocity):return False
        vx,vy=velocity;speed=math.hypot(vx,vy);desired=math.hypot(f,side)
        if speed<.08:return True
        if not desired:return False
        along=(vx*f+vy*side)/desired;across=abs(vx*side-vy*f)/desired
        return along>=0 and across<.08

    def note_recovery_command(self,axes,now=None):
        """Confirm a nonzero command actually accepted by the drive path.

        This proves issuance, not mechanical execution. Only later feedback
        proves motion. Selecting/waiting on a candidate consumes no action try.
        """
        move=self.recovery
        if move is None or move.get('issued') or len(axes)!=3 or not any(axes):return False
        if any(abs(float(v)-move[k])>1e-8 for v,k in zip(axes,['forward','side','yaw'])):return False
        now=time.monotonic() if now is None else now
        if self.recovery_attempts==0:
            self.recovery_episode_anchor=tuple(self.last_pose[:2]);self.recovery_history=set(self.actual_visited)
            self.recovery_measured_distance=0.;self.recovery_new_cells=set();self.recovery_last_translation=None
        move.update(issued=True,issued_at=now,origin=tuple(self.last_pose[:2]),origin_yaw=self.last_pose[2],
                    stamp=self.current_scan.get('stamp',now),deadline_monotonic=now+3.,
                    world_direction=self._recovery_world_direction(move['forward'],move['side']))
        self.recovery_attempts+=1;self.recovery_total_attempts+=1;self.recovery_candidate_failures=0
        self._remember_recovery_direction(move['forward'],move['side'],move['yaw'])
        return True

    def _finish_recovery(self,reason):
        move=self.recovery
        if move is None:return
        issued=bool(move.get('issued'));elapsed=max(0.,self.current_scan.get('stamp',move['stamp'])-move['stamp'])
        travelled=math.dist(self.last_pose[:2],move['origin']) if issued else 0.
        turned=abs(wrap(self.last_pose[2]-move['origin_yaw'])) if issued else 0.
        progressed=issued and (travelled>=.08 or turned>=.12)
        if not issued:self.recovery_candidate_failures+=1
        self.recovery_outcomes.append(dict(direction=self._recovery_direction(move['forward'],move['side'],move['yaw']),
            world_direction=move.get('world_direction'),issued=issued,reason=reason,translation=round(travelled,3),
            rotation=round(turned,3),elapsed=round(elapsed,2),progressed=progressed))
        self.recovery_outcomes=self.recovery_outcomes[-12:]
        if not progressed:
            self.frontiers.blocked(self.goal,self.last_stamp if self.last_stamp is not None else time.monotonic(),self.obstacles,reason)
            self.goal=None
        self.recovery=None;self.path=[]
        self.recovery_pending=bool(self.recovery_forced and not progressed and self.recovery_attempts<3 and self.recovery_candidate_failures<3)
        self.recovery_stop_stamp=self.current_scan.get('stamp')

    def reset_recovery_context(self,verified_translation_m=0.,reason='暂停后重新核对局部动作'):
        """Lifecycle-only reset after capture/frame identity has been verified.

        A caller may credit externally observed manual translation only after
        fresh paired poses prove it. Turning alone never refunds issued tries.
        """
        if self.recovery is not None:self._finish_recovery('context_reset')
        self.recovery=None;self.recovery_pending=False;self.recovery_stop_stamp=None;self.recovery_forced=False
        self.recovery_candidate_failures=0;self.recovery_request_reason=None
        if (isinstance(verified_translation_m,(int,float)) and math.isfinite(verified_translation_m)
                and verified_translation_m>=.6):
            self.recovery_attempts=0;self.recovery_resets+=1;self.recovery_episode_anchor=None
            self.recovery_history=set();self.recovery_new_cells=set();self.recovery_measured_distance=0.;self.recovery_last_translation=None
            self.recovery_tried=set();self.recovery_tried_world=[];self.recovery_block_reason=None
            self.recovery_reset_reason=reason
            self._traversed.clear()

    def _recover_axes(self):
        if self.recovery is None:
            if self.recovery_pending:
                velocity=self.current_scan.get('_velocity_body');stamp=self.current_scan.get('stamp')
                if (velocity is None or len(velocity)!=2 or not all(math.isfinite(float(v)) for v in velocity)
                        or stamp is None or self.recovery_stop_stamp is None or stamp<=self.recovery_stop_stamp
                        or math.hypot(*velocity)>=.08 or abs(self._current_yaw_rate)>=.12):
                    self.motion_hint='先停稳再更换脱困方向';return 0.,0.,0.
            options=self._escape_candidates()
            if not options:self.recovery_pending=False;self._no_recovery();return 0.,0.,0.
            self._begin_recovery_episode()
            _,f,side,yaw=options[0]
            self.recovery_pending=False;self.recovery_block_reason=None
            self.recovery=dict(forward=f,side=side,yaw=yaw,origin=self.last_pose[:2],origin_yaw=self.last_pose[2],
                               issued=False,world_direction=self._recovery_world_direction(f,side),stamp=self.current_scan.get('stamp',time.monotonic()),deadline_monotonic=time.monotonic()+2.)
        move=self.recovery;elapsed=self.current_scan.get('stamp',time.monotonic())-move['stamp']
        travelled=math.dist(self.last_pose[:2],move['origin'])
        turned=abs(wrap(self.last_pose[2]-move['origin_yaw']))
        if not move.get('yaw') and turned>.35:
            # A body-axis command would now point somewhere else in the world.
            # End this action without refunding its attempt or old direction.
            self._finish_recovery('heading_changed');self.motion_hint='朝向已变化，停稳后重新选择脱困方向'
            return 0.,0.,0.
        limit=3. if move.get('issued') else 2.
        if (move.get('issued') and (travelled>=self.recovery_limit or move.get('yaw') and turned>=.45) or elapsed>=limit-1e-9
                or time.monotonic()>=move['deadline_monotonic']):
            self._finish_recovery('motion_limit' if move.get('issued') and (travelled>=self.recovery_limit or turned>=.45) else 'action_timeout' if move.get('issued') else 'candidate_not_issued_timeout')
            self.motion_hint='本次脱困没有实际进展，停稳后换方向' if self.recovery_pending else '退让已结束，重新检查通路'
            return 0.,0.,0.
        f,side,yaw=move['forward'],move['side'],move.get('yaw',0.)
        if (not yaw and abs(self._current_yaw_rate)>.25 or yaw*self._current_yaw_rate<0 and abs(self._current_yaw_rate)>.12
                or not self._motion_ready(f,side)):
            self.motion_hint='先停稳再退让';return 0.,0.,0.
        if not self._inside_bound(f,side) or not clearance(self.current_scan,f,yaw,side=side)[0]:
            self._finish_recovery('path_blocked');self.motion_hint='退让方向已被阻挡，已记录本次结果并重新检查通路';return 0.,0.,0.
        self.motion_hint='小角度转向重新观察' if yaw else '沿斜向通路短移观察' if f and side else '向左侧移让开' if side>0 else '向右侧移让开' if side<0 else '短距离后退脱困' if f<0 else '短距离前移重新观察'
        return f,side,yaw

    def command_axes(self,yaw_rate=0.,observation_age=0.):
        """Choose a checked translation or turn, with bounded local recovery.

        This does not extend a sensor lease or prove real hardware tracking. The
        runtime rechecks the same three-axis sweep before issuing each command.
        """
        self._current_yaw_rate=yaw_rate
        if self.recovery is not None or self.recovery_pending:return self._recover_axes()
        if '_velocity_body' in self.current_scan and self.current_scan['_velocity_body'] is None:
            self.motion_hint='等待最新运动反馈';return 0.,0.,0.
        f,yaw=self.command(yaw_rate,observation_age)
        if not self.path:
            if self.goal in self.visited:
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
                    area=round(len(self.free)*RESOLUTION**2,1),distance=round(self.distance,2),net_displacement=round(math.dist(self.last_pose[:2],self.origin),2) if self.last_pose and self.origin else 0,replans=self.replans,target_kind=self.target_kind,motion_hint=self.motion_hint,recovery_attempts=self.recovery_attempts,frontier_metrics=dict(self.frontier_metrics),
                    recovery_episode=dict(number=self.recovery_episode,total_attempts=self.recovery_total_attempts,resets=self.recovery_resets,
                        anchor=self.recovery_episode_anchor,measured_distance=round(self.recovery_measured_distance,2),new_cells=len(self.recovery_new_cells),
                        block_reason=self.recovery_block_reason,reset_reason=self.recovery_reset_reason,pending=self.recovery_pending,
                        candidate_failures=self.recovery_candidate_failures,active_issued=bool(self.recovery and self.recovery.get('issued')),
                        tried=sorted(self.recovery_tried),tried_world=[list(v) for v in self.recovery_tried_world],
                        outcomes=list(self.recovery_outcomes),request_reason=self.recovery_request_reason))


class PriorMap:
    """Old-map labels guide priority; they never substitute for live floor."""
    def __init__(self,grid,anchor):
        import base64
        from m20.navigation_planner import pgm_pixels
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

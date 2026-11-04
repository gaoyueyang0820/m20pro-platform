"""Next-best-view exploration on measured free space, without actuator I/O.

Independent Python implementation informed by frontier exploration (see
reference/exploration_sources.md). Unknown cells contribute information value;
they never become traversable. Costs use graph distance, not a straight line
through walls. All coordinates here are integer world-grid cells.
"""
from collections import deque
import heapq
import math


CARDINAL = ((1,0),(-1,0),(0,1),(0,-1))
EIGHT = CARDINAL + ((1,1),(1,-1),(-1,1),(-1,-1))


def ray_cells(a,b):
    """Integer supercover: a ray cannot see diagonally through touching walls."""
    x,y=a;dx=b[0]-x;dy=b[1]-y;nx,ny=abs(dx),abs(dy)
    sx=1 if dx>0 else -1;sy=1 if dy>0 else -1;ix=iy=0
    yield (x,y)
    while ix<nx or iy<ny:
        left=(1+2*ix)*ny;right=(1+2*iy)*nx
        if left==right:
            yield (x+sx,y);yield (x,y+sy)
            x+=sx;y+=sy;ix+=1;iy+=1
        elif left<right:x+=sx;ix+=1
        else:y+=sy;iy+=1
        yield (x,y)


def reachable(start,allowed,resolution):
    costs={start:0.};parent={start:None};queue=[(0.,start)]
    while queue:
        cost,p=heapq.heappop(queue)
        if cost>costs[p]+1e-9:continue
        for dx,dy in EIGHT:
            q=(p[0]+dx,p[1]+dy)
            if q not in allowed:continue
            if dx and dy and ((p[0]+dx,p[1]) not in allowed or (p[0],p[1]+dy) not in allowed):continue
            value=cost+resolution*(math.sqrt(2) if dx and dy else 1.)
            if value+1e-9<costs.get(q,float('inf')):
                costs[q]=value;parent[q]=p;heapq.heappush(queue,(value,q))
    return costs,parent


def clusters(cells):
    remaining=set(cells);groups=[]
    while remaining:
        seed=min(remaining);remaining.remove(seed);group=[seed];queue=deque([seed])
        while queue:
            x,y=queue.popleft()
            for dx,dy in EIGHT:
                q=(x+dx,y+dy)
                if q in remaining:remaining.remove(q);group.append(q);queue.append(q)
        groups.append(group)
    return groups


class FrontierSelector:
    def __init__(self,resolution=.2):
        self.res=resolution;self.failed=[];self.viewpoints=[];self.last={}
        self.goal_switches=0;self.previous_goal=None
        self._search_key=None;self._search_index=0;self._maneuver_blocked=0

    def observe_pose(self,pose):
        # Record useful spatial baselines, not thousands of nearly identical
        # scans or stationary pose noise. These are coverage hints, not map edits.
        p=(pose[0]/self.res,pose[1]/self.res)
        if not self.viewpoints or min(math.dist(p,q) for q in self.viewpoints)*self.res>=.9:
            self.viewpoints.append(p)

    def blocked(self,goal,now,obstacles,reason=''):
        if goal is None:return
        count=1+sum(math.dist(goal,r['goal'])*self.res<1. for r in self.failed)
        nearby=frozenset(p for p in obstacles if math.dist(p,goal)*self.res<1.2)
        self.failed.append(dict(goal=goal,until=now+min(45.,10.*count),obstacles=nearby,reason=reason))
        self.failed=self.failed[-32:]

    def _deferred(self,p,now,obstacles):
        for row in self.failed:
            if now>=row['until'] or math.dist(p,row['goal'])*self.res>.9:continue
            # A changed, newly measured local obstacle set can reopen a region.
            old=row['obstacles']
            if old and not old.issubset(obstacles):continue
            return True
        return False

    def information(self,p,free,obstacles,within,prior_kind=None):
        """Bounded occlusion-aware ray estimate in m² (not wall-clock scans)."""
        unknown=set();new_old_map=set();visible=set();reach=round(2.4/self.res)
        for i in range(48):
            angle=i*2*math.pi/48
            end=(p[0]+round(reach*math.cos(angle)),p[1]+round(reach*math.sin(angle)))
            for q in ray_cells(p,end):
                if not within(q) or q in obstacles:break
                if q in free:
                    visible.add(q)
                    if prior_kind and prior_kind(q)=='unknown':new_old_map.add(q)
                else:
                    unknown.add(q)
                    if prior_kind and prior_kind(q)=='unknown':new_old_map.add(q)
        return len(unknown)*self.res**2,len(new_old_map)*self.res**2,visible

    def select(self,start,yaw,allowed,free,obstacles,within,now,prior_kind=None,possible=None,visited=()):
        costs,parent=reachable(start,allowed,self.res)
        visited=set(visited);observed_baselines=set();near_viewpoint=set();near_short_view=set()
        r=1.4/self.res;n=math.ceil(r);near2=(.85/self.res)**2
        for vx,vy in self.viewpoints:
            for x in range(math.floor(vx)-n,math.ceil(vx)+n+1):
                for y in range(math.floor(vy)-n,math.ceil(vy)+n+1):
                    d2=(x-vx)**2+(y-vy)**2
                    if d2<=r*r:observed_baselines.add((x,y))
                    if d2<near2:near_viewpoint.add((x,y))
                    if d2<(.35/self.res)**2:near_short_view.add((x,y))
        eligible_cache={};deferred=set()
        def eligible(p,short=False):
            key=(p,short)
            if key not in eligible_cache:
                usable=p not in visited and p not in (near_short_view if short else near_viewpoint)
                if usable and self._deferred(p,now,obstacles):deferred.add(p);usable=False
                eligible_cache[key]=usable
            return eligible_cache[key]
        known=free|obstacles
        boundary={p for p in free if within(p) and any((p[0]+dx,p[1]+dy) not in known and within((p[0]+dx,p[1]+dy)) for dx,dy in CARDINAL)}
        groups=clusters(boundary)
        # Approach points are on supported free space, behind each frontier.
        # Several candidates per cluster avoid sending the body to a centroid
        # inside an obstacle or overlooking a doorway at a long frontier's end.
        candidates=set();near=math.ceil(.9/self.res)
        for group in groups:
            options=set()
            for x,y in group:
                for dx in range(-near,near+1):
                    for dy in range(-near,near+1):
                        q=(x+dx,y+dy)
                        if dx*dx+dy*dy<=near*near and q in costs and costs[q]>=.7 and eligible(q):options.add(q)
            if not options:continue
            centre=(sum(p[0] for p in group)/len(group),sum(p[1] for p in group)/len(group))
            chosen=sorted(options,key=lambda p:(math.dist(p,centre)+costs[p]/self.res*.15,p))
            for p in chosen:
                if all(math.dist(p,q)*self.res>=.8 for q in candidates):candidates.add(p)
                if len(candidates)>=48:break
            if len(candidates)>=48:break
        # Add coarse viewpoints for revisiting old areas and for an already
        # fully visible room. Do not drive to every 20 cm cell in the room.
        coarse=max(1,round(.8/self.res))
        fallback=[p for p in costs if costs[p]>=.8 and p[0]%coarse==0 and p[1]%coarse==0 and eligible(p)]
        fallback.sort(key=lambda p:(costs[p],p))
        if not fallback:fallback=sorted((p for p in costs if costs[p]>=.8 and eligible(p)),key=lambda p:(costs[p],p))
        # A coarse lattice must not force a 45-degree turn merely because the
        # forward 0.8 m point was too near and the next axis point was 1.6 m.
        for angle in (yaw+i*math.pi/4 for i in range(8)):
            for length in (1.,1.6,2.4):
                p=(start[0]+round(length/self.res*math.cos(angle)),start[1]+round(length/self.res*math.sin(angle)))
                if p in costs and eligible(p):candidates.add(p)
        # Keep every remaining coarse viewpoint. A fixed prefix can contain
        # only visited/blocked locations while open space lies further away.
        ordered=sorted(candidates,key=lambda p:(costs[p],p))+[p for p in fallback if p not in candidates]
        normal_set=set(ordered)
        # An 85 cm viewpoint baseline is useful for coverage, but must not
        # forbid a shorter clear approach that reveals the next scan section.
        short_points={p for p in costs if .4<=costs[p]<=1.6 and .35<=math.dist(p,start)*self.res<=1.2 and p not in normal_set and eligible(p,True)}
        ordered+=sorted(short_points,key=lambda p:(costs[p],p))
        key=(start,round(yaw/.25),tuple(ordered),frozenset(free),frozenset(obstacles))
        if key!=self._search_key or self._search_index>=len(ordered) and self.last.get('search_status') in ['maneuver','local_space','deferred']:
            self._search_key=key;self._search_index=0;self._maneuver_blocked=0
        # Score bounded batches so large maps cannot stall the control loop.
        # Unchecked candidates mean search is pending, never exploration done.
        batch=ordered[self._search_index:self._search_index+192]
        self._search_index+=len(batch);ranked=[]
        for p in batch:
            if p in short_points and ranked:break
            gain,expansion,visible=self.information(p,free,obstacles,within,prior_kind)
            new_views=len(visible-observed_baselines)*self.res**2
            if gain<.08 and expansion<.08 and new_views<.2:continue
            if p in short_points and gain<.08 and expansion<.08:continue
            # Check the actual first path direction, not a line through walls.
            chain=[];q=p
            while q is not None:chain.append(q);q=parent[q]
            chain.reverse();lead=chain[min(3,len(chain)-1)]
            heading=math.atan2(lead[1]-start[1],lead[0]-start[0]);turn=abs(math.atan2(math.sin(heading-yaw),math.cos(heading-yaw)))
            if possible and not possible(heading):self._maneuver_blocked+=1;continue
            # Coverage gain means genuinely distinct nearby observation poses.
            # It is not a claim of improved map accuracy or metric SLAM quality.
            kind=('expand' if expansion>.08 else 'refine') if prior_kind else ('expand' if gain>.08 else 'observe')
            # Once an old-map unknown area is reobserved, moving within it has
            # diminishing utility. Frontier gain is the dominant global driver.
            utility=3.0*gain+1.4*min(expansion,4.)+0.45*min(new_views,8.)
            cost=1.+costs[p]+.85*turn
            score=utility/cost
            ranked.append((score,-turn,-costs[p],p,chain,kind,gain,expansion,new_views))
        if not ranked:
            remaining=len(ordered)-self._search_index
            status=('searching' if remaining else 'maneuver' if self._maneuver_blocked else 'deferred' if deferred
                    else 'local_space' if groups and max(costs.values(),default=0.)<.8 else 'complete')
            reasons=dict(searching='正在选择其他探索方向',maneuver='正在寻找转向或退让通路',deferred='等待暂时受阻通路重新开放',local_space='正在重新观察周边通路',complete='可达区域已完成观察')
            self.last=dict(frontier_clusters=len(groups),reachable_cells=len(costs),candidates=0,
                candidate_pool=len(ordered),search_remaining=remaining,search_status=status,
                maneuver_blocked=self._maneuver_blocked,deferred_regions=len(deferred),reason=reasons[status])
            return None
        best=max(ranked);_,_,_,goal,path,kind,gain,expansion,new_views=best
        self._search_index=0
        if prior_kind and kind=='observe':kind='refine'
        if goal!=self.previous_goal:self.goal_switches+=1;self.previous_goal=goal
        self.last=dict(frontier_clusters=len(groups),reachable_cells=len(costs),candidates=len(ranked),candidate_pool=len(ordered),search_status='planned',information_gain_m2=round(gain,2),old_unknown_gain_m2=round(expansion,2),revisit_gain_m2=round(new_views,2),path_length_m=round(costs[goal],2),deferred_regions=sum(now<r['until'] for r in self.failed),goal_switches=self.goal_switches,reason='前往信息收益较高的边界观察点' if gain>.08 else '前往具有新观察角度的复查点')
        return dict(goal=goal,path=path,kind=kind,metrics=self.last)

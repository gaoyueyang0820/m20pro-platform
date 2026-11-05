"""Short navigation bypasses over the current measured local surface only.

The original destination and occupancy map are untouched. This pure search
connects nearby full-body poses with continuous fixed-heading translations;
TrajectoryFollower validates each actual command again on newer observations.
"""
import heapq
import math
import time

from m20.local_trajectory import LocalSurface,RES


def plan_local_detour(scan,goal,*,tried=(),avoid_axes=None,radius=1.4,max_length=2.4,budget=.10):
    now=time.monotonic();expiry=scan.get('_valid_until',now+.65-float(scan.get('age',0.)))
    if now>=expiry:return None
    surface=LocalSurface.cached(scan)
    if not surface.valid(0.,0.,0.):return None
    pose=scan['pose'];c,s=math.cos(pose[2]),math.sin(pose[2])
    dx,dy=goal[0]-pose[0],goal[1]-pose[1];target=(c*dx+s*dy,-s*dx+c*dy);distance=math.hypot(*target)
    def world(p):return (pose[0]+c*p[0]-s*p[1],pose[1]+s*p[0]+c*p[1])
    def edge(a,b):
        return surface.valid((a[0]+b[0])/2,(a[1]+b[1])/2,0.,padding=(abs(b[0]-a[0])/2,abs(b[1]-a[1])/2))
    avoid=avoid_axes[:2] if avoid_axes is not None else (0.,0.);avoid_length=math.hypot(*avoid)
    queue=[(0.,(0,0))];cost={(0,0):0.};parent={};best=None;finish=time.perf_counter()+budget
    while queue:
        length,node=heapq.heappop(queue)
        if length!=cost.get(node):continue
        if time.perf_counter()>=finish or time.monotonic()>=expiry:break
        p=(node[0]*RES,node[1]*RES);world_p=world(p)
        if math.hypot(*p)>=min(.35,max(.16,distance-.1)) and not any(math.dist(world_p,q)<.35 for q in tried):
            delta=(target[0]-p[0],target[1]-p[1]);remaining=math.hypot(*delta)
            step=min(.4,remaining);future=(p[0]+delta[0]*step/max(.001,remaining),p[1]+delta[1]*step/max(.001,remaining))
            opening=edge(p,future)
            score=3*(distance-remaining)+.6*opening-.18*length-.08*abs(p[1])
            candidate=(score,-length,node)
            if best is None or candidate>best:best=candidate
        for ox,oy in [(1,0),(0,1),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            neighbor=(node[0]+ox,node[1]+oy);q=(neighbor[0]*RES,neighbor[1]*RES)
            if math.hypot(*q)>radius+1e-8:continue
            new_length=length+math.hypot(ox,oy)*RES
            if new_length>max_length or new_length>=cost.get(neighbor,math.inf)-1e-10:continue
            if node==(0,0) and avoid_length>.1 and (ox*avoid[0]+oy*avoid[1])/(math.hypot(ox,oy)*avoid_length)>.8:continue
            if not edge(p,q):continue
            cost[neighbor]=new_length;parent[neighbor]=node;heapq.heappush(queue,(new_length,neighbor))
    if best is None:return None
    node=best[2];cells=[node]
    while node!=(0,0):node=parent[node];cells.append(node)
    points=[world((node[0]*RES,node[1]*RES)) for node in reversed(cells)]
    return dict(path=[dict(x=x,y=y) for x,y in points],goal=dict(x=points[-1][0],y=points[-1][1]),
                length_m=cost[best[2]],original_goal=list(goal),source_stamp=scan.get('stamp'),kind='local_bypass')

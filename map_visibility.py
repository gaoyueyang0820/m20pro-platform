"""Visibility evidence for map surfaces, separate from navigable ground.

Requires an explicitly trusted sensor origin for each single-origin cloud. A
body pose or a merged front/rear scan is NOT a valid ray origin. Missing returns
are unknown; nearer returns are occlusion; only measured rays behind a surface
can contradict it. Original implementation, inspired by range-image methods.
"""
import math
import numpy as np


class RangeVisibility:
    def __init__(self,points,origin,angular_resolution=math.radians(.5)):
        points=np.asarray(points,dtype=float)[:,:3];self.origin=np.asarray(origin,dtype=float)
        if self.origin.shape!=(3,) or not np.isfinite(self.origin).all():raise ValueError('需要已标定的单雷达世界原点')
        self.resolution=angular_resolution;self.width=math.ceil(2*math.pi/angular_resolution)
        offset=points-self.origin;distance=np.linalg.norm(offset,axis=1)
        valid=np.isfinite(offset).all(axis=1)&(distance>.3)&(distance<8.)
        offset=offset[valid];distance=distance[valid]
        az=np.floor((np.arctan2(offset[:,1],offset[:,0])+math.pi)/angular_resolution).astype(int)%self.width
        el=np.floor((np.arctan2(offset[:,2],np.linalg.norm(offset[:,:2],axis=1))+math.pi/2)/angular_resolution).astype(int)
        self.depth={}
        for x,y,d in zip(az,el,distance):self.depth[x,y]=min(float(d),self.depth.get((x,y),math.inf))

    def classify(self,points,margin=.15):
        """0 unknown/occluded, 1 observed surface, -1 measured free ray.

        A fine-bin return is required. The adjacent coarse window vetoes removal
        near occlusion edges and close surfaces; it never invents new returns.
        """
        points=np.asarray(points,dtype=float)[:,:3];delta=points-self.origin
        distance=np.linalg.norm(delta,axis=1);answer=np.zeros(len(points),np.int8)
        for i,(p,d) in enumerate(zip(delta,distance)):
            if not .55<d<6 or not np.isfinite(p).all():continue
            x=int(math.floor((math.atan2(p[1],p[0])+math.pi)/self.resolution))%self.width
            y=int(math.floor((math.atan2(p[2],math.hypot(p[0],p[1]))+math.pi/2)/self.resolution))
            same=self.depth.get((x,y))
            nearby=[self.depth.get(((x+dx)%self.width,y+dy),math.inf) for dx in [-1,0,1] for dy in [-1,0,1]]
            nearest=min(nearby)
            if same is not None and abs(same-d)<=margin:answer[i]=1
            elif same is not None and same>d+margin and nearest>d+margin:answer[i]=-1
        return answer


def confirmed_surface_removals(points,frames,policy=None,now=None):
    """Temporal visibility votes for existing points; never ray-clear a grid.

    frames contain points, stamp, pose and optional sensor_origin_world plus
    sensor_origin_trusted=True. Ground support must separately clear occupancy.
    """
    from map_change_policy import classify_cell,ChangePolicy
    policy=policy or ChangePolicy();points=np.asarray(points)
    evidence={};trusted=0;missing=0;counts={'observed':0,'free_ray':0,'unknown_or_occluded':0}
    if now is None:frames=list(frames)
    latest=max((float(f['stamp']) for f in frames),default=0) if now is None else float(now)
    for frame in frames:
        stamp=float(frame['stamp'])
        if stamp<latest-policy.window_s:continue
        if frame.get('sensor_origin_trusted') is not True or frame.get('sensor_origin_world') is None:
            missing+=1;continue
        origin=np.asarray(frame['sensor_origin_world'],float);trusted+=1
        local=np.linalg.norm(points[:,:3]-origin,axis=1)<6
        # Surface contradiction must not remove ground/depressions. Existing
        # files carry a body pose; the validated indoor floor is ~0.55 m below.
        pose=frame.get('pose')
        if pose is not None:local&=(points[:,2]>float(pose[2])-.30)&(points[:,2]<float(pose[2])+1.5)
        indices=np.flatnonzero(local);votes=RangeVisibility(frame['points'],origin).classify(points[indices])
        counts['observed']+=int((votes==1).sum());counts['free_ray']+=int((votes==-1).sum());counts['unknown_or_occluded']+=int((votes==0).sum())
        for index,vote in zip(indices,votes):
            if vote:evidence.setdefault(int(index),[]).append(dict(stamp=stamp,vote=int(vote),origin=origin[:2].tolist()))
    removed=np.zeros(len(points),bool)
    for index,observations in evidence.items():
        removed[index]=classify_cell(0,observations,registration_ok=True,now=latest,policy=policy)['action']=='clear'
    return removed,dict(method='multi_resolution_range_visibility',trusted_origin_frames=trusted,
        missing_origin_frames=missing,removed_points=int(removed.sum()),votes=counts,
        state='evaluated' if trusted else 'no_calibrated_single_sensor_origin',occupancy_cleared_by_rays=False)

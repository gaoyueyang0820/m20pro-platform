"""Check an unnamed SLAM child frame against paired base/map point clouds."""
import math
import numpy as np
from m20.autonomy_sensing import xyz
from m20.map_fusion import pose_matrix,transform,nearest_distances


def verify_slam_body_pose(body,world,pose):
    if body.header.frame_id!='base_link' or world.header.frame_id!=pose['frame_id'] or pose['frame_id']!='map':
        raise ValueError('SLAM 对照点云坐标系不一致')
    stamp=lambda msg:msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    times=[stamp(body),stamp(world),pose['stamp']]
    if max(times)-min(times)>.02:raise ValueError('等待同一时刻的 SLAM 坐标对照帧')
    T=pose_matrix([pose['PosX'],pose['PosY'],pose['PosZ']]+pose['quaternion'])
    target=xyz(body);local=transform(xyz(world),np.linalg.inv(T))
    lo,hi=target.min(axis=0),target.max(axis=0)
    local=local[((local>=lo+.05)&(local<=hi-.05)).all(axis=1)]
    if len(local)<100 or np.ptp(local[:,0])<1 or np.ptp(local[:,1])<1 or np.ptp(local[:,2])<.5:
        raise ValueError('SLAM 坐标对照需要足够的空间结构')
    # Include structure above the floor, so a flat floor alone cannot validate XY/yaw.
    structure=local[local[:,2]>np.quantile(local[:,2],.3)+.15]
    if len(structure)<30:raise ValueError('SLAM 坐标对照缺少非地面结构')
    sample=np.vstack([local[::max(1,math.ceil(len(local)/100))],structure[::max(1,math.ceil(len(structure)/100))]])
    distances=nearest_distances(sample,target)
    median,p95=np.quantile(distances,[.5,.95])
    if median>.04 or p95>.10:raise ValueError('SLAM 位姿与机身点云变换不一致')
    return dict(verified=True,method='paired_base_link_and_map_clouds',samples=len(sample),median_m=round(float(median),4),p95_m=round(float(p95),4),stamp=pose['stamp'])

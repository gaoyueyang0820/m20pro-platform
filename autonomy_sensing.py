"""Compact supported-floor observations from a full (not strided) local scan."""
import math
import numpy as np
from autonomy_core import RESOLUTION


def xyz(message):
    fields = {f.name:f for f in message.fields}
    total = int(message.width)*int(message.height)
    if not 1 <= total <= 400000:
        raise ValueError('雷达点数不在自主控制支持范围内')
    rows = []
    for name in ['x','y','z']:
        f = fields.get(name)
        if not f or f.datatype not in [7,8] or f.count!=1:
            raise ValueError('雷达缺少浮点 XYZ 字段')
        dtype = np.dtype(('>' if message.is_bigendian else '<')+('f4' if f.datatype==7 else 'f8'))
        if f.offset<0 or f.offset+dtype.itemsize>message.point_step:
            raise ValueError('点云字段偏移无效')
        try:
            rows.append(np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                                   offset=f.offset,strides=(message.row_step,message.point_step)).ravel())
        except (ValueError,TypeError):
            raise ValueError('点云长度无效')
    points = np.column_stack(rows)
    return points[np.isfinite(points).all(axis=1)]


def supported_floor(points):
    """Prefer the explicitly built, equivalent full-point native reducer."""
    try:
        from native_sensing import try_reduce
        result=try_reduce(points) if RESOLUTION==.2 else None
        if result is not None:return result
    except ImportError:pass
    return _numpy_supported_floor(points)


def _numpy_supported_floor(points):
    points = np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100:
        raise ValueError('当前点云不足以判断可行走地面')
    x,y,z=points.T
    # Filter finite values and radius with one Nx3 copy. The 2-component sum
    # retains float64 arithmetic; only threshold-neighbouring values need sqrt
    # to reproduce the former norm comparison's last-bit rounding exactly.
    finite=np.isfinite(x)&np.isfinite(y)&np.isfinite(z)
    with np.errstate(over='ignore',invalid='ignore'):
        radius2=x*x+y*y
    inner=.55*.55;outer=36.
    inside=(radius2>inner)&(radius2<outer)
    epsilon=np.finfo(np.float64).eps
    edge=finite&((abs(radius2-inner)<=8*epsilon*inner)|(abs(radius2-outer)<=8*epsilon*outer))
    if np.any(edge):
        boundary_radius=np.sqrt(radius2[edge])
        inside[edge]=(boundary_radius>.55)&(boundary_radius<6)
    points=points[finite&inside]
    z=points[:,2];ground_mask=(z<-.15)&(z>-.9)
    ground_z=z[ground_mask]
    if len(ground_z)<80:
        raise ValueError('尚未看到连续地面，请停稳等待雷达')
    bins=np.round(ground_z/.05).astype(int)
    # Height bins are bounded by the unchanged (-.9, -.15) ground interval.
    # bincount preserves unique's ascending-bin tie break without sorting.
    first=int(bins.min());counts=np.bincount(bins-first)
    level=(first+int(np.argmax(counts)))*.05
    samples=points[ground_mask&(abs(z-level)<.08)]
    if len(samples)<80 or np.ptp(samples[:,0])<1.2 or np.ptp(samples[:,1])<1.2:
        raise ValueError('地面覆盖太窄，暂不能自主移动')
    # Solve the centered 2x2 normal system instead of constructing and QR-
    # factorizing an N x 3 matrix for every 150k-point scan on the robot CPU.
    # Samples is an owned filtered copy. In-place centering avoids another Nx3
    # allocation while retaining the original mean/subtraction/reduction order.
    # Raw second moments would introduce cancellation near height thresholds.
    mean=samples.mean(axis=0);samples-=mean
    x,y,z=samples.T
    xx=np.sum(x*x);xy=np.sum(x*y);yy=np.sum(y*y);xz=np.sum(x*z);yz=np.sum(y*z)
    determinant=xx*yy-xy*xy
    if determinant<=1e-8*max(1.,xx*yy):raise ValueError('地面观测退化为细线，等待更完整地面')
    ax=(xz*yy-yz*xy)/determinant;ay=(yz*xx-xz*xy)/determinant
    coeff=np.array([ax,ay,mean[2]-ax*mean[0]-ay*mean[1]])
    if math.hypot(*coeff[:2])>math.tan(math.radians(6)):
        raise ValueError('当前地面倾斜，自动探索仅支持平整室内地面')
    height = points[:,2]-(points[:,0]*coeff[0]+points[:,1]*coeff[1]+coeff[2])
    grid = np.rint(points[:,:2]/RESOLUTION).astype(int)
    # Pack the bounded grid into scalar IDs. Building tens of thousands of Python
    # tuples took >300 ms on NOS, making a freshly processed scan already stale.
    extent=math.ceil(6/RESOLUTION)+2;side=2*extent+1
    ids=(grid[:,0]+extent)*side+(grid[:,1]+extent)
    floor=np.flatnonzero(np.bincount(ids[abs(height)<.055],minlength=side*side))
    # Include low obstacles and deep depressions. Unknown floor stays unknown.
    obstacle_mask=((height>.12)&(height<1.5))|(height<-.12)
    obstacle_ids=ids[obstacle_mask]
    obstacles=np.flatnonzero(np.bincount(obstacle_ids,minlength=side*side))
    bounds=np.column_stack([np.full(side*side,np.inf),np.full(side*side,np.inf),np.full(side*side,-np.inf),np.full(side*side,-np.inf)])
    xy=points[obstacle_mask,:2]
    for axis in [0,1]:
        np.minimum.at(bounds[:,axis],obstacle_ids,xy[:,axis]);np.maximum.at(bounds[:,axis+2],obstacle_ids,xy[:,axis])
    bounds=bounds[obstacles]
    measured_floor=floor.copy()
    floor=np.setdiff1d(floor,obstacles,assume_unique=True)
    unpack=lambda values:np.column_stack([values//side-extent,values%side-extent]).astype(int).tolist()
    return dict(floor=unpack(floor),obstacles=unpack(obstacles),
                obstacle_bounds=np.round(bounds,4).tolist(),measured_floor=unpack(measured_floor),geometry_version=2,
                resolution=RESOLUTION,floor_height=round(float(coeff[2]),3),input_points=len(points))

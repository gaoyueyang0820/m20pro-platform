"""Exact output comparison against the saved pre-optimization producer algorithm.

Synthetic full point sets only. The legacy implementation is test-local so this
regression does not depend on private robot recordings or a data/ backup file.
"""
import math
import unittest
import numpy as np
from m20.autonomy_core import RESOLUTION
from m20.autonomy_sensing import supported_floor

def legacy_supported_floor(points):
    points = np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<100:
        raise ValueError('当前点云不足以判断可行走地面')
    points = points[np.isfinite(points).all(axis=1)]
    radius = np.linalg.norm(points[:,:2],axis=1)
    points = points[(radius>.55)&(radius<6)]
    ground = points[(points[:,2]<-.15)&(points[:,2]>-.9)]
    if len(ground)<80:
        raise ValueError('尚未看到连续地面，请停稳等待雷达')
    bins = np.round(ground[:,2]/.05).astype(int)
    values,counts = np.unique(bins,return_counts=True)
    level = values[np.argmax(counts)]*.05
    samples = ground[abs(ground[:,2]-level)<.08]
    if len(samples)<80 or np.ptp(samples[:,0])<1.2 or np.ptp(samples[:,1])<1.2:
        raise ValueError('地面覆盖太窄，暂不能自主移动')
    # Solve the centered 2x2 normal system instead of constructing and QR-
    # factorizing an N x 3 matrix for every 150k-point scan on the robot CPU.
    mean=samples.mean(axis=0);centered=samples-mean
    x,y,z=centered.T
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


def generated(seed,count=2500):
    rng=np.random.default_rng(seed)
    xy=rng.uniform(-6.2,6.2,(count,2));slope=rng.uniform(-.04,.04,2)
    z=-.55+xy@slope+rng.normal(0,.013,count)
    n=count//5;z[:n]+=rng.choice([.14,.3,.8,1.4,-.18],n)
    return np.column_stack([xy,z])


class SensingEquivalenceTests(unittest.TestCase):
    def assert_equivalent(self,points):
        before=np.asarray(points).copy()
        try:expected=legacy_supported_floor(points)
        except (ValueError,TypeError) as exc:
            with self.assertRaises(type(exc)) as actual:supported_floor(points)
            self.assertEqual(str(actual.exception),str(exc))
        else:
            actual=supported_floor(points)
            self.assertEqual(actual,expected)
        np.testing.assert_array_equal(np.asarray(points),before)

    def test_random_full_point_clouds_preserve_every_output_field(self):
        for seed in range(120):
            with self.subTest(seed=seed):
                points=generated(seed)
                if seed%3==0:points=points.astype(np.float32)
                if seed%4==0:points=np.vstack([points,[[np.nan,1,-.55],[2,np.inf,-.55],[1,2,np.nan]]])
                self.assert_equivalent(points)

    def test_150k_point_cloud_preserves_obstacle_cells_and_exact_bounds(self):
        for seed in [713,2718]:
            with self.subTest(seed=seed):self.assert_equivalent(generated(seed,150000))

    def test_radius_threshold_ulps_at_many_bearings(self):
        base=generated(98,5000);edges=[]
        for radius in [.55,6.]:
            for angle in np.linspace(0,2*math.pi,97):
                xy=np.array([math.cos(angle),math.sin(angle)])*radius
                for direction in [-np.inf,np.inf]:
                    for axis in [0,1]:
                        moved=xy.copy()
                        for _ in range(5):
                            edges.append([*moved,-.2]);moved[axis]=np.nextafter(moved[axis],direction)
        self.assert_equivalent(np.vstack([base,edges]))

    def test_ground_bins_sample_and_obstacle_height_threshold_ulps(self):
        x,y=np.meshgrid(np.linspace(-4,4,80),np.linspace(-4,4,75))
        base=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55)])
        probes=[]
        for level in [-.9,-.15,-.55-.08,-.55+.08,-.55-.055,-.55+.055,-.55-.12,-.55+.12,-.55+1.5]:
            for direction in [-np.inf,np.inf]:
                z=level
                for n in range(8):
                    probes.append([1.8+n*.2,(-1 if direction<0 else 1)*1.2,z]);z=np.nextafter(z,direction)
        self.assert_equivalent(np.vstack([base,probes]))

    def test_height_bin_tie_uses_lowest_bin_as_before(self):
        x,y=np.meshgrid(np.linspace(-2,2,12),np.linspace(-2,2,12))
        xy=np.column_stack([x.ravel(),y.ravel()])
        points=np.vstack([np.column_stack([xy,np.full(len(xy),z)]) for z in [-.55,-.65]])
        self.assert_equivalent(points)
        self.assertEqual(supported_floor(points)['floor_height'],-.65)

    def test_slope_threshold_and_invalid_geometry_exceptions_match(self):
        x,y=np.meshgrid(np.linspace(-3,3,40),np.linspace(-3,3,40))
        for slope in [np.nextafter(math.tan(math.radians(6)),0),math.tan(math.radians(6)),np.nextafter(math.tan(math.radians(6)),np.inf)]:
            self.assert_equivalent(np.column_stack([x.ravel(),y.ravel(),-.55+x.ravel()*slope]))
        for points in [np.zeros((100,3)),np.full((120,3),np.nan),generated(5)[:75],np.column_stack([np.linspace(1,3,100),np.zeros(100),np.full(100,-.55)])]:
            self.assert_equivalent(points)


if __name__=='__main__':unittest.main()

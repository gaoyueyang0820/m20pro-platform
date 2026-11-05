"""Full-point native/NumPy parity. Build explicitly before required execution.

M20_REQUIRE_NATIVE_SENSING=1 makes missing/invalid binaries fail instead of skip.
No test compiles, downloads, opens cameras, connects to a robot or samples points.
"""
import ctypes
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch
import numpy as np
import m20.native_sensing as native_sensing
from m20.autonomy_sensing import xyz
from test_sensing_equivalence import legacy_supported_floor,generated


def flat_cloud(count=6000,z=-.55):
    rng=np.random.default_rng(81);xy=rng.uniform(-4.,4.,(count,2))
    return np.column_stack([xy,np.full(count,z)])


def benchmark():
    """Explicit benchmark: every timed result must really be native and equal."""
    status=native_sensing.backend_status()
    if not status['available']:raise RuntimeError('native benchmark requires an explicitly built ABI 1 library')
    before=native_sensing.backend_status();rows=[]
    for count in [100000,150000,300000]:
        points=generated(713,count).astype(np.float32);expected=legacy_supported_floor(points)
        times={'native':[],'numpy':[]}
        for i in range(7):
            pairs=[('native',native_sensing.try_reduce),('numpy',legacy_supported_floor)]
            if i%2:pairs.reverse()
            for name,fn in pairs:
                start=time.perf_counter();result=fn(points);times[name].append((time.perf_counter()-start)*1000)
                if result is None:raise AssertionError('benchmark silently fell back: '+str(native_sensing.backend_status()))
                if result!=expected:raise AssertionError('benchmark output mismatch for '+name)
        rows.append(dict(points=count,input_dtype='float32',samples_ms=times,median_ms={k:statistics.median(v) for k,v in times.items()}))
    after=native_sensing.backend_status()
    return dict(scope='complete synthetic PointCloud2-style XYZ arrays, no physical or NOS timing claim',
                native_calls=after['native_frames']-before['native_frames'],fallback_calls=after['fallback_frames']-before['fallback_frames'],rows=rows,backend=after)


class NativeSensingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        status=native_sensing.backend_status()
        if not status['available']:
            message='native sensing library unavailable; explicitly run tools/build_native_sensing.py'
            if os.environ.get('M20_REQUIRE_NATIVE_SENSING')=='1':raise RuntimeError(message+': '+str(status['load_error']))
            raise unittest.SkipTest(message)

    def native_equal(self,points):
        original=points.copy();result=native_sensing.try_reduce(points)
        self.assertIsNotNone(result,native_sensing.backend_status())
        self.assertEqual(result,legacy_supported_floor(points))
        np.testing.assert_array_equal(points,original)
        return result

    def test_random_frames_require_native_hit_and_exact_all_field_equality(self):
        before=native_sensing.backend_status()
        for seed in range(120):
            with self.subTest(seed=seed):
                points=generated(seed)
                if seed%3==0:points=points.astype(np.float32)
                if seed%4==0:points=np.asfortranarray(points)
                self.native_equal(points)
        after=native_sensing.backend_status()
        self.assertEqual(after['native_frames']-before['native_frames'],120)
        self.assertEqual(after['fallback_frames']-before['fallback_frames'],0)

    def test_full_100k_150k_300k_inputs_are_native_not_hidden_fallback(self):
        for count in [100000,150000,300000]:
            with self.subTest(count=count):self.native_equal(generated(713,count).astype(np.float32))

    def test_pointcloud2_float32_full_150k_and_last_obstacle_are_preserved(self):
        # All ordinary points have |x|<4, so the last obstacle's x=5.8 cell is
        # unique: dropping that final return cannot be hidden by another point.
        points=flat_cloud(150000).astype('<f4');points[-1]=[5.8,0.,-.2]
        message=Obj(width=len(points),height=1,row_step=len(points)*12,point_step=12,data=points.tobytes(),is_bigendian=False,
                    fields=[Obj(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate('xyz')])
        decoded=xyz(message);self.assertEqual(len(decoded),150000)
        result=self.native_equal(decoded)
        self.assertIn([29,0],result['obstacles'])
        self.assertTrue(any(a<=float(points[-1,0])+.0001<=c+.0002 and b<=0<=d for a,b,c,d in result['obstacle_bounds']))

    def test_nan_inf_rows_are_excluded_without_losing_any_finite_obstacle(self):
        points=np.vstack([generated(713,4000),[[np.nan,0,-.55],[0,np.inf,-.55],[1,1,-np.inf],[5.8,0,-.2]]])
        result=self.native_equal(points);self.assertIn([29,0],result['obstacles'])

    def test_grid_bankers_rounding_and_height_bin_ties_match_numpy(self):
        base=flat_cloud(1000)
        points=np.vstack([base,[[x,.7,-.2] for x in [.1,.3,.5,.7,.9,1.1,1.3]]])
        self.native_equal(points)
        xy=np.random.default_rng(29).uniform(-3,3,(500,2))
        tie=np.vstack([np.column_stack([xy,np.full(len(xy),z)]) for z in [-.55,-.65]])
        self.assertEqual(self.native_equal(tie)['floor_height'],-.65)

    def test_radius_and_height_boundaries_explicitly_use_numpy_authority(self):
        base=flat_cloud();cases={
            'radius_boundary':np.vstack([base,[.55,0.,-.55]]),
            'ground_height_boundary':np.vstack([base,[2.,0.,-.15]]),
            'height_bin_boundary':np.vstack([base,[2.,0.,-.525]]),
            'sample_height_boundary':np.vstack([base,[2.,0.,-.47]]),
            'classification_boundary':np.vstack([base,[2.,0.,-.43]]),
            'rounded_height_boundary':flat_cloud(z=-.5505),
        }
        for reason,points in cases.items():
            with self.subTest(reason=reason):
                self.assertIsNone(native_sensing.try_reduce(points))
                self.assertEqual(native_sensing.backend_status()['last_reason'],reason)
                expected=legacy_supported_floor(points)
                # The caller executes its unchanged NumPy implementation when
                # None is returned; native never publishes partial boundaries.
                self.assertIsInstance(expected,dict)

    def test_degenerate_and_insufficient_ground_do_not_publish_native_result(self):
        for points in [np.zeros((100,3)),np.full((150,3),np.nan),
                       np.column_stack([np.linspace(1,3,100),np.linspace(1,3,100),np.full(100,-.55)])]:
            self.assertIsNone(native_sensing.try_reduce(points))
            with self.assertRaises(ValueError):legacy_supported_floor(points)

    def test_missing_library_returns_none_with_diagnostic_and_does_not_compile(self):
        with tempfile.TemporaryDirectory() as directory:
            missing=str(Path(directory)/'not-installed.so')
            with patch.dict(os.environ,{'M20_SENSING_LIBRARY':missing}):
                self.assertIsNone(native_sensing.try_reduce(flat_cloud()))
                status=native_sensing.backend_status();self.assertFalse(status['available']);self.assertTrue(status['load_error'])
                self.assertFalse(Path(missing).exists())
        self.assertTrue(native_sensing.backend_status()['available'])

    def test_incompatible_abi_is_diagnosable_and_falls_back(self):
        with tempfile.NamedTemporaryFile(suffix='.so') as file:
            fake=Obj(m20_sensing_abi_version=lambda:99,m20_sensing_result_size=lambda:ctypes.sizeof(native_sensing._Result))
            with patch.dict(os.environ,{'M20_SENSING_LIBRARY':file.name}),patch('m20.native_sensing.ctypes.CDLL',return_value=fake):
                self.assertIsNone(native_sensing.try_reduce(flat_cloud()))
                self.assertIn('ABI mismatch',native_sensing.backend_status()['load_error'])
        self.assertTrue(native_sensing.backend_status()['available'])


if __name__=='__main__':unittest.main()

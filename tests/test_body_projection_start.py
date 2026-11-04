"""Synthetic body-residue starts and departure; no physical device or file I/O."""
import copy
import math
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from test_live_surface_localization import planner,packet
from test_start_pose_recovery import room,body_oracle
from start_pose_recovery import recover_start
from start_recovery_runtime import plan_with_recovery
from robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH


class BodyProjectionStartTests(unittest.TestCase):
    def scene(self,yaw=0):
        pose=[3.,3.,yaw];truth=planner(room());old_grid=room()
        inside=[];c,s=math.cos(yaw),math.sin(yaw)
        for iy in range(45,55):
            for ix in range(25,35):
                x,y=truth.world(ix,iy)
                if abs(c*(x-3)+s*(y-3))+.05*(abs(c)+abs(s))<=BODY_HALF_LENGTH-1e-9 and abs(-s*(x-3)+c*(y-3))+.05*(abs(c)+abs(s))<=BODY_HALF_WIDTH-1e-9:
                    inside.append((ix,iy))
        inside.sort(key=lambda q:math.dist(truth.world(*q),pose[:2]))
        cells=inside[:5];self.assertEqual(len(cells),5)
        for ix,iy in cells:old_grid[iy,ix]=0
        return planner(old_grid,True),truth,pose,set(cells)

    def scan(self,truth,pose,stamp=100):
        scan,loc=packet(truth,actual=pose,stamp=stamp)
        # Real sensor body blind zone: no measured floor under the body.
        for key in ('floor','measured_floor'):
            scan[key]=[q for q in scan[key] if math.hypot(*q)*.2>.55]
        return scan,loc

    def test_five_unobserved_body_cells_plan_immediately_from_one_scan(self):
        p,truth,start,cells=self.scene();base=p.base_free.copy();original=p.original.copy()
        scan,loc=self.scan(truth,start)
        self.assertFalse(p.pose_fits(*start))
        report=p.update_live_surface(scan,loc)
        self.assertEqual(report['body_projection_cells'],5)
        self.assertEqual(p.live_surface.body_projection_cells,cells)
        self.assertFalse(p.live_surface.confirmed_cells)
        self.assertTrue(p.pose_fits(*start))
        route=p.plan(start,[5.,3.],0.)
        self.assertTrue(route['ok']);self.assertTrue(body_oracle(truth,route['dense_path']))
        self.assertEqual(route['dense_path'][0]['x'],start[0]);self.assertEqual(route['goal']['x'],5.)
        np.testing.assert_array_equal(p.base_free,base);np.testing.assert_array_equal(p.original,original)

    def test_recovery_entry_applies_body_seed_without_operator_validation(self):
        p,truth,start,_=self.scene();scan,loc=self.scan(truth,start)
        result=recover_start(p,start,[5.,3.],0.,scan,loc,standing=True)
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['classification'],'body_projection_start')
        self.assertFalse(result['operator_confirmation_required'])
        self.assertEqual(result['recovery_distance_m'],0.)
        self.assertTrue(body_oracle(truth,result['global_plan']['dense_path']))

    def test_public_adapter_keeps_original_start_and_goal(self):
        p,truth,start,_=self.scene();scan,loc=self.scan(truth,start)
        result=plan_with_recovery(p,start,[5.,3.],0.,scan,loc,True)
        self.assertFalse(result['recovery']['required'])
        self.assertEqual([result['dense_path'][0][k] for k in ['x','y','yaw']],start)
        self.assertEqual([result['goal'][k] for k in ['x','y']],[5.,3.])

    def test_rotated_body_cells_are_wholly_contained_not_axis_aligned_clearing(self):
        p,truth,start,_=self.scene(yaw=.63);scan,loc=self.scan(truth,start)
        p.update_live_surface(scan,loc)
        self.assertEqual(len(p.live_surface.body_projection_cells),5)
        c,s=math.cos(start[2]),math.sin(start[2])
        for ix,iy in p.live_surface.body_projection_cells:
            x,y=p.world(ix,iy)
            for dx,dy in ((-.05,-.05),(-.05,.05),(.05,-.05),(.05,.05)):
                a,b=x+dx-start[0],y+dy-start[1]
                self.assertLessEqual(abs(c*a+s*b),BODY_HALF_LENGTH+1e-8)
                self.assertLessEqual(abs(-s*a+c*b),BODY_HALF_WIDTH+1e-8)
        self.assertTrue(p.pose_fits(*start))

    def test_continuous_departure_does_not_reintroduce_residue_under_rear_body(self):
        p,truth,start,cells=self.scene();clock=[500.];carried=False
        with patch('live_traversability.time.monotonic',side_effect=lambda:clock[0]):
            for i in range(20):
                pose=[3.+i*.06,3.,0.];clock[0]=500.+i*.15
                scan,loc=self.scan(truth,pose,stamp=100.+i*.15)
                report=p.update_live_surface(scan,loc)
                carried|=bool(report.get('departed_body_projection_cells'))
                self.assertTrue(p.pose_fits(*pose),('rear body got stuck leaving residue',i,report))
                if i<12:self.assertTrue(p.plan(pose,[5.,3.],0.)['ok'])
            self.assertTrue(carried)
            # No permanent carve or permanent breadcrumb trail.
            clock[0]+=2.1;p.live_surface_status()
            self.assertFalse(p.live_surface.body_projection_cells)
            for ix,iy in cells:self.assertFalse(p.free[iy,ix])

    def test_slow_departure_keeps_only_previous_seed_until_tail_has_left(self):
        p,truth,start,cells=self.scene();clock=[500.]
        with patch('live_traversability.time.monotonic',side_effect=lambda:clock[0]):
            for i in range(81):
                pose=[3.+i*.01,3.,0.];clock[0]=500.+i*.2
                scan,loc=self.scan(truth,pose,stamp=100.+i*.2)
                p.update_live_surface(scan,loc)
                self.assertTrue(p.pose_fits(*pose),('very slow rear departure',i))
                self.assertTrue(p.live_surface.body_projection_cells<=cells)
            clock[0]+=2.1;p.live_surface_status()
            self.assertFalse(p.live_surface.body_projection_cells)

    def test_current_lidar_obstacle_inside_body_always_wins(self):
        p,truth,start,cells=self.scene();scan,loc=self.scan(truth,start)
        scan['obstacles']=[[0,0]];scan['obstacle_bounds']=[[-.03,-.03,.03,.03]]
        report=p.update_live_surface(scan,loc)
        self.assertGreater(report['blocked_cells'],0)
        self.assertFalse(p.pose_fits(*start))
        result=recover_start(p,start,[5.,3.],0.,scan,loc,standing=True)
        self.assertFalse(result['ok']);self.assertEqual(result['classification'],'live_body_collision')

    def test_new_obstacle_revokes_even_previously_cleared_body_cell(self):
        p,truth,start,_=self.scene();scan,loc=self.scan(truth,start);p.update_live_surface(scan,loc)
        self.assertTrue(p.pose_fits(*start))
        scan,loc=self.scan(truth,start,stamp=100.2)
        scan['obstacles']=[[0,0]];scan['obstacle_bounds']=[[-.03,-.03,.03,.03]]
        p.update_live_surface(scan,loc)
        self.assertFalse(p.pose_fits(*start))
        self.assertFalse(p.live_surface.body_projection_cells & p.live_surface.blocked_cells)

    def test_body_exception_never_clears_unknown_or_front_wall(self):
        p,truth,start,cells=self.scene();grid=room()
        for ix,iy in cells:grid[iy,ix]=0
        # 3 cm clearance and body-blind-radius are NOT extra clearing regions.
        for x,y,value in [(3.45,3.05,0),(3.55,3.05,0),(3.85,3.05,205),(3.05,3.05,205)]:
            ix,iy=truth.cell(x,y);grid[iy,ix]=value
        p=planner(grid,True);scan,loc=self.scan(truth,start)
        scan['floor']=[];scan['measured_floor']=[]
        p.update_live_surface(scan,loc)
        for x,y in [(3.45,3.05),(3.55,3.05),(3.85,3.05),(3.05,3.05)]:
            ix,iy=p.cell(x,y);self.assertFalse(p.free[iy,ix],(x,y))

    def test_expired_or_jumping_pose_cannot_keep_body_seed(self):
        for mode in ('expired','jump'):
            with self.subTest(mode=mode):
                p,truth,start,_=self.scene();clock=[500.]
                with patch('live_traversability.time.monotonic',side_effect=lambda:clock[0]):
                    scan,loc=self.scan(truth,start);p.update_live_surface(scan,loc)
                    self.assertTrue(p.pose_fits(*start))
                    if mode=='expired':clock[0]+=.6;p.live_surface_status()
                    else:
                        scan,loc=self.scan(truth,[4.5,3.,0.],stamp=100.2);p.update_live_surface(scan,loc)
                    self.assertFalse(p.live_surface.body_projection_cells)
                    self.assertFalse(p.pose_fits(*start))


class BodyProjectionPublicPreviewTests(unittest.TestCase):
    def setUp(self):
        from test_platform import PlatformTests
        PlatformTests.setUp(self)
    def tearDown(self):
        from test_point_navigation_runtime import PointNavigationRuntimeTests
        PointNavigationRuntimeTests.tearDown(self)

    def test_platform_plan_goal_accepts_five_body_residue_cells_without_manual_step(self):
        from test_point_navigation_runtime import PointNavigationRuntimeTests
        self.ready=PointNavigationRuntimeTests.ready.__get__(self)
        clock=[600.]
        with patch('time.monotonic',side_effect=lambda:clock[0]),patch('time.sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            app=PointNavigationRuntimeTests.setup_live(self);self.ready(x=3.,y=3.)
            truth=planner(room(120,100));grid=room(120,100)
            for x,y in [(2.95,2.95),(3.05,2.95),(2.95,3.05),(3.05,3.05),(3.15,3.05)]:
                ix,iy=truth.cell(x,y);grid[iy,ix]=0
            source=app.store.path.parent/'maps'/(self.map['id']+'.pgm')
            source.write_bytes(b'P5\n120 100\n255\n'+grid.tobytes());before=source.read_bytes()
            def observe(mapping=False):
                clock[0]+=.2;self.ready(x=3.,y=3.)
                scan,_=packet(truth,actual=(3.,3.,0.),stamp=clock[0])
                for key in ('floor','measured_floor'):scan[key]=[q for q in scan[key] if math.hypot(*q)*.2>.55]
                return scan
            app.autonomy.scan_now=observe
            result=app.plan_goal(dict(map_id=self.map['id'],x=5.,y=3.,yaw=0.,engine='platform'))
        self.assertTrue(result['ok']);self.assertTrue(result['plan_id'])
        self.assertEqual(result['live_surface']['body_projection_cells'],5)
        self.assertEqual(result['dense_path'][0]['x'],3.);self.assertEqual(result['goal']['x'],5.)
        self.assertEqual(source.read_bytes(),before)
        self.assertEqual(app.nav['status'],'idle');self.assertFalse(any(app.velocity))
        self.assertFalse(any(any(row['v']) for row in app.link.velocities))


if __name__=='__main__':unittest.main()

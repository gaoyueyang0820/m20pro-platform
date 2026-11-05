"""Exact obstacle bounds shared by trajectory/recovery/preflight, offline."""
import copy
import math
import unittest
from types import SimpleNamespace as Obj
from unittest.mock import patch,Mock
from m20.autonomy_core import Explorer,clearance,disk,cell
from m20.autonomy_runtime import Autonomy
from m20.local_trajectory import LocalSurface
import test_platform as platform_cases


def scene(points=((.49,.29),(.75,0.)),stamp=1.):
    bounds=[[x-.003,y-.003,x+.003,y+.003] for x,y in points]
    obstacles=[list(cell(x,y)) for x,y in points];blocked={tuple(p) for p in obstacles}
    measured=[list(p) for p in disk(3.)]
    return dict(pose=[0.,0.,0.],stamp=stamp,floor=[p for p in measured if tuple(p) not in blocked],
                measured_floor=measured,obstacles=obstacles,obstacle_bounds=bounds,geometry_version=2,_speed_mps=0.,_velocity_body=[0.,0.])


class PreciseClearanceTests(unittest.TestCase):
    def test_outside_front_corner_does_not_block_a_real_reverse_corridor(self):
        s=scene();self.assertTrue(LocalSurface(s).valid(0,0,0))
        self.assertFalse(clearance(s,.1,0)[0]);self.assertTrue(clearance(s,-.1,0)[0])
        legacy=copy.deepcopy(s);legacy.pop('obstacle_bounds')
        self.assertFalse(clearance(legacy,-.1,0)[0])

    def test_actual_body_contact_and_original_three_cm_margin_still_reject(self):
        for point in [(.40,.20),(.44,0.)]:
            s=scene(points=[point]);self.assertFalse(LocalSurface(s).valid(0,0,0))
            for forward,side,yaw in [(.1,0,0),(-.1,0,0),(0,.1,0),(0,-.1,0),(0,0,.1),(0,0,-.1)]:
                self.assertFalse(clearance(s,forward,yaw,side=side)[0])

    def test_unknown_rear_support_is_not_filled_by_precise_obstacle_bounds(self):
        s=scene()
        for key in ['floor','measured_floor']:s[key]=[p for p in s[key] if p[0]>-3]
        clear,reason=clearance(s,-.1,0)
        self.assertFalse(clear);self.assertIn('地面',reason)

    def test_full_turn_sweep_catches_intermediate_contact_with_clear_end_poses(self):
        s=scene(points=[(.50,0.)]);surface=LocalSurface(s)
        self.assertTrue(surface.valid(0,0,0));self.assertTrue(surface.valid(0,0,math.pi/2))
        self.assertFalse(clearance(s,0,.1)[0]);self.assertFalse(clearance(s,0,-.1)[0])

    def test_continuous_translation_cannot_jump_over_thin_obstacle(self):
        s=scene(points=[(.70,0.)]);s['_speed_mps']=1.3;surface=LocalSurface(s)
        self.assertTrue(surface.valid(0,0,0));self.assertTrue(surface.valid(1.4,0,0))
        self.assertFalse(clearance(s,.1,0)[0])

    def test_malformed_bounds_retain_conservative_cell_collision(self):
        s=scene();s['obstacle_bounds']=[[.49,.29,.50]]
        self.assertFalse(LocalSurface(s).precise);self.assertFalse(clearance(s,-.1,0)[0])

    def test_escape_preview_translates_all_geometry_including_bounds_and_floor_evidence(self):
        s=scene();e=Explorer(3);e.ingest(s);seen=[];actual=clearance
        def observe(candidate,*args,**kwargs):
            if candidate is not s:
                dx=s['obstacles'][0][0]-candidate['obstacles'][0][0]
                dy=s['obstacles'][0][1]-candidate['obstacles'][0][1]
                self.assertEqual(candidate['measured_floor'][0],[s['measured_floor'][0][0]-dx,s['measured_floor'][0][1]-dy])
                for old,new in zip(s['obstacle_bounds'],candidate['obstacle_bounds']):
                    expected=[old[0]-dx*.2,old[1]-dy*.2,old[2]-dx*.2,old[3]-dy*.2]
                    for a,b in zip(new,expected):self.assertAlmostEqual(a,b)
                seen.append((dx,dy))
            return actual(candidate,*args,**kwargs)
        with patch('m20.autonomy_core.clearance',side_effect=observe):options=e._escape_candidates()
        self.assertTrue(options);self.assertIn((-2,0),seen)

    def test_preflight_accepts_two_complete_frames_with_only_reverse_start(self):
        a=Autonomy.__new__(Autonomy);a.app=Obj(demo=False,mapping=False);a.scan_timing={}
        a.scan_now=Mock(side_effect=[scene(stamp=1),scene(stamp=2)])
        with patch('m20.autonomy_runtime.time.sleep'):a._check_observation()
        self.assertEqual(a.scan_timing['preflight_direction'],'reverse');self.assertEqual(a.scan_now.call_count,2)

    def test_preflight_accepts_verified_side_start_without_requiring_a_turn(self):
        first=scene(points=[(.75,0.),(-.75,0.)]);second=copy.deepcopy(first);second['stamp']=2.
        self.assertFalse(clearance(first,.1,0)[0]);self.assertFalse(clearance(first,-.1,0)[0])
        a=Autonomy.__new__(Autonomy);a.app=Obj(demo=False,mapping=False);a.scan_timing={};a.scan_now=Mock(side_effect=[first,second])
        with patch('m20.autonomy_runtime.time.sleep'):a._check_observation()
        self.assertEqual(a.scan_timing['preflight_direction'],'left')



class AutonomyDirectionPreflightTests(unittest.TestCase):
    setUp=platform_cases.PlatformTests.setUp
    tearDown=platform_cases.PlatformTests.tearDown
    def direction(self,value,at=None):
        self.app.demo=False;self.app.link=platform_cases.FakeLink()
        basic=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=0)
        if value is not None:basic['Direction']=value
        self.app.link.put(1002,6,dict(BasicStatus=basic),at=at)
        self.app.basic.update(MotionState=17,ControlUsageMode=0,Sleep=0)
        return self.app.autonomy
    def test_tail_first_is_rejected_before_mapping_or_worker_start(self):
        a=self.direction(1)
        with patch.object(a,'_prepare_and_run') as worker,patch.object(self.app.hub,'post') as post:
            with self.assertRaisesRegex(ValueError,'手柄.*头朝前'):a.start(dict(mode='explore'))
        worker.assert_not_called();post.assert_not_called();self.assertFalse(a.active)
    def test_fresh_head_first_is_accepted_without_any_mode_switch(self):
        a=self.direction(0);a._check_start();self.assertEqual(self.app.link.sent,[])
    def test_missing_or_stale_direction_never_assumes_head_first(self):
        import time
        for value,at in [(None,None),(0,time.time()-3)]:
            with self.subTest(value=value):
                a=self.direction(value,at)
                with self.assertRaisesRegex(ValueError,'头朝前|新鲜'):a._check_start()


if __name__=='__main__':unittest.main()

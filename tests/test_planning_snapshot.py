"""Computation may outlast a scan, but never extends execution permission."""
import copy
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from test_live_surface_localization import planner,door_world,packet
from m20.start_pose_recovery import recover_start,check_recovery_step
import m20.start_pose_recovery as start_pose_recovery


class PlanningSnapshotTests(unittest.TestCase):
    def setUp(self):
        self.clock=[100.08]
        self.timer=patch('time.monotonic',side_effect=lambda:self.clock[0]);self.timer.start();self.addCleanup(self.timer.stop)
    def passage(self):
        p=planner(door_world(),True);truth=planner(door_world(True))
        for i in range(4):
            stamp=100+i*.2;self.clock[0]=stamp+.08
            scan,loc=packet(truth,stamp=stamp);scan['_valid_until']=self.clock[0]+.57
            p.update_live_surface(scan,loc)
        self.assertTrue(p.pose_fits(4.,4.,0.))
        return p,truth,scan,loc

    def test_slow_global_plan_uses_snapshot_then_expiry_resumes(self):
        p,_,scan,_=self.passage();until=scan['_valid_until'];real_deadline=p.live_surface.real_expires_at
        solve=p._plan
        def slow(*args):self.clock[0]+=2.3;return solve(*args)
        with patch.object(p,'_plan',side_effect=slow):result=p.plan([2.8,4.,0.],[5.2,4.],0.,budget=3.)
        self.assertTrue(result['ok']);self.assertTrue(result['requires_live_revalidation'])
        self.assertEqual(scan['_valid_until'],until);self.assertEqual(p.live_surface.real_expires_at,real_deadline)
        self.assertIsNone(p._planning_revision);self.assertIsNone(p._deadline)
        self.assertFalse(p.pose_fits(4.,4.,0.))
        self.assertEqual(p.live_surface_status()['state'],'expired')

    def test_recovery_cpu_delay_does_not_expire_its_working_map_mid_search(self):
        p,_,scan,loc=self.passage();until=scan['_valid_until'];geometry=start_pose_recovery._Geometry
        def delayed(*args,**kwargs):self.clock[0]+=.8;return geometry(*args,**kwargs)
        with patch.object(start_pose_recovery,'_Geometry',side_effect=delayed):
            result=recover_start(p,[2.8,4.,0.],[5.2,4.],0.,scan,loc,standing=True,budget=2.)
        self.assertTrue(result['ok'],result);self.assertTrue(result['requires_live_revalidation'])
        self.assertEqual(scan['_valid_until'],until);self.assertFalse(result['motion_sent'])
        self.assertIsNone(p._planning_revision);self.assertFalse(p.pose_fits(4.,4.,0.))
        step=check_recovery_step(p,[2.8,4.,0.],[2.85,4.,0.],scan,loc,[2.8,4.,0.],standing=True)
        self.assertFalse(step['ok']);self.assertIn('期限',step['reason'])

    def test_projection_cpu_delay_is_part_of_same_planning_snapshot(self):
        p,truth,_,_=self.passage();self.clock[0]+=.2
        scan,loc=packet(truth,stamp=100.8);scan['_valid_until']=self.clock[0]+.15;scan['age']=.5
        update=p.update_live_surface
        def delayed(*args,**kwargs):
            report=update(*args,**kwargs);self.clock[0]+=.2;return report
        with patch.object(p,'update_live_surface',side_effect=delayed):
            result=recover_start(p,[2.8,4.,0.],[5.2,4.],0.,scan,loc,standing=True,budget=2.)
        self.assertTrue(result['ok'],result)
        self.assertLess(scan['_valid_until'],self.clock[0])
        self.assertFalse(p.pose_fits(4.,4.,0.))

    def test_nested_plan_does_not_release_outer_recovery_snapshot(self):
        p,_,scan,_=self.passage();revision=p.grid_revision
        with p.planning_snapshot():
            self.clock[0]+=.8
            first=p.plan([2.8,4.,0.],[5.2,4.],0.)
            self.assertTrue(first['ok']);self.assertEqual(p._planning_revision,revision)
            self.assertTrue(p.pose_fits(4.,4.,0.))
            second=p.plan([5.2,4.,0.],[2.8,4.],0.)
            self.assertTrue(second['ok']);self.assertEqual(p._planning_revision,revision)
        self.assertIsNone(p._planning_revision);self.assertFalse(p.pose_fits(4.,4.,0.))

    def test_input_expired_before_recovery_admission_is_still_rejected(self):
        p,_,scan,loc=self.passage();self.clock[0]=scan['_valid_until']+.001
        with patch.object(p,'update_live_surface',side_effect=AssertionError('stale input must not be projected')):
            result=recover_start(p,[2.8,4.,0.],[5.2,4.],0.,scan,loc,standing=True)
        self.assertFalse(result['ok']);self.assertEqual(result['classification'],'observation_unavailable')
        self.assertIsNone(p._planning_revision)

    def test_already_expired_layer_not_revived_by_entering_plan_snapshot(self):
        p,_,scan,_=self.passage();self.clock[0]=scan['_valid_until']+.001
        with self.assertRaises(ValueError):p.plan([2.8,4.,0.],[5.2,4.],0.)
        self.assertFalse(p.pose_fits(4.,4.,0.));self.assertIsNone(p._planning_revision)

    def test_exception_releases_snapshot_and_new_scan_can_recover(self):
        p,truth,scan,loc=self.passage()
        def failure(*args,**kwargs):self.clock[0]+=.8;raise ValueError('synthetic calculation failure')
        with patch.object(start_pose_recovery,'_Geometry',side_effect=failure):
            result=recover_start(p,[2.8,4.,0.],[5.2,4.],0.,scan,loc,standing=True,budget=2.)
        self.assertFalse(result['ok']);self.assertIsNone(p._planning_revision)
        self.assertFalse(p.pose_fits(4.,4.,0.))
        new,where=packet(truth,stamp=101.6);new['_valid_until']=self.clock[0]+.57
        p.update_live_surface(new,where)
        self.assertTrue(p.pose_fits(4.,4.,0.))

    def test_external_layer_change_aborts_result_instead_of_mixing_revisions(self):
        p,_,_,_=self.passage();solve=p._plan
        def changed(*args):
            p.update_obstacles([[2.,2.]])
            return solve(*args)
        with patch.object(p,'_plan',side_effect=changed),self.assertRaisesRegex(ValueError,'变化'):
            p.plan([2.8,4.,0.],[5.2,4.],0.)
        self.assertIsNone(p._planning_revision);self.assertIsNone(p._deadline)
        self.assertIn(p.cell(2.,2.),p.overlay)


if __name__=='__main__':unittest.main()

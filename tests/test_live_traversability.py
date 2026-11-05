"""Positive door/object-change use cases and temporary layer expiration.

All geometry is synthetic unless a test explicitly loads the saved local replay.
No robot API, sockets, persistent map writes, or browser calls.
"""
import copy,math,time,unittest
from unittest.mock import patch
import numpy as np
from test_live_surface_localization import door_world,planner,packet,feed
import m20.live_traversability as live_traversability


class LiveTraversabilityTests(unittest.TestCase):
    def test_readonly_planning_snapshot_does_not_extend_execution_permission(self):
        old=planner(door_world(),True);truth=planner(door_world(True));clock=[500.]
        with patch.object(live_traversability.time,'monotonic',side_effect=lambda:clock[0]):
            for i in range(3):
                clock[0]=500+i*.2;scan,loc=packet(truth,stamp=100+i*.2);scan['age']=.5
                old.update_live_surface(scan,loc)
            solve=old._plan
            def slow(*args):clock[0]+=.3;return solve(*args)
            with patch.object(old,'_plan',side_effect=slow):result=old.plan([2.8,4.,0.],[5.2,4.],0.)
            self.assertTrue(result['ok']);self.assertTrue(result['requires_live_revalidation'])
            self.assertFalse(old.pose_fits(4.,4.,0.),'a completed preview cannot renew an expired execution layer')

    def test_default_is_off_and_does_not_change_existing_free_map(self):
        old=planner(door_world());truth=planner(door_world(True));scan,loc=packet(truth)
        result=old.update_live_surface(scan,loc,now=100.08)
        self.assertFalse(result['enabled']);self.assertFalse(old.pose_fits(4,4,0))

    def test_scan_age_half_second_does_not_erase_short_gap_votes(self):
        old=planner(door_world(),True);truth=planner(door_world(True));clock=[500.]
        with patch.object(live_traversability.time,'monotonic',side_effect=lambda:clock[0]):
            for i in range(3):
                clock[0]=500+i*.2;scan,loc=packet(truth,stamp=100+i*.2);scan['age']=.5
                result=old.update_live_surface(scan,loc)
                if i<2:self.assertFalse(old.pose_fits(4,4,0))
            self.assertGreater(result['temporary_free_cells'],0);self.assertTrue(old.pose_fits(4,4,0))
            clock[0]+=.16
            self.assertFalse(old.pose_fits(4,4,0),'permission expires without requiring another scan call')
            self.assertTrue(old.live_surface.evidence,'the short-gap observation history is retained')
            clock[0]+=.04;scan,loc=packet(truth,stamp=100.6);scan['age']=.5
            result=old.update_live_surface(scan,loc)
            self.assertTrue(old.pose_fits(4,4,0));self.assertGreater(result['occupied_overrides'],0)

    def test_long_scan_loss_requires_new_independent_evidence(self):
        old=planner(door_world(),True);truth=planner(door_world(True));clock=[600.]
        with patch.object(live_traversability.time,'monotonic',side_effect=lambda:clock[0]):
            for i in range(3):
                clock[0]=600+i*.2;scan,loc=packet(truth,stamp=200+i*.2);old.update_live_surface(scan,loc)
            self.assertTrue(old.pose_fits(4,4,0));clock[0]+=2.1
            self.assertFalse(old.pose_fits(4,4,0));self.assertFalse(old.live_surface.evidence)
            scan,loc=packet(truth,stamp=202.6);old.update_live_surface(scan,loc)
            self.assertFalse(old.pose_fits(4,4,0))

    def test_confirmed_ground_is_carried_through_real_sensor_body_blind_spot(self):
        old=planner(door_world(),True);truth=planner(door_world(True));clock=[700.]
        with patch.object(live_traversability.time,'monotonic',side_effect=lambda:clock[0]):
            feed(old,truth,start=300.)
            self.assertTrue(old.pose_fits(4,4,0));carried=0
            for i,x in enumerate(np.arange(3.,4.61,.1)):
                clock[0]+=.2;stamp=300.8+i*.2
                scan,loc=packet(truth,actual=(float(x),4.,0.),stamp=stamp)
                for key in ['floor','measured_floor']:
                    scan[key]=[p for p in scan[key] if math.hypot(*p)*.2>.55]
                result=old.update_live_surface(scan,loc,now=stamp+.08)
                carried=max(carried,result.get('carried_under_body',0))
                self.assertTrue(old.pose_fits(float(x),4,0),f'body got stuck at x={x} while crossing an already confirmed opening')
            self.assertGreater(carried,0)
            self.assertFalse(old.base_free[40,39])

    def test_never_observed_ground_under_initial_body_is_not_invented(self):
        old=planner(door_world(),True);truth=planner(door_world(True))
        for i in range(4):
            scan,loc=packet(truth,actual=(4.,4.,0.),stamp=400+i*.2)
            for key in ['floor','measured_floor']:scan[key]=[p for p in scan[key] if math.hypot(*p)*.2>.55]
            old.update_live_surface(scan,loc,now=400.08+i*.2)
        self.assertFalse(old.pose_fits(4,4,0))

    def test_expiry_does_not_allow_the_same_old_source_frame_to_vote_again(self):
        old=planner(door_world(),True);truth=planner(door_world(True));scan,loc=packet(truth,stamp=500.)
        for now in [500.08,500.8,501.5,502.2,502.9]:old.update_live_surface(scan,loc,now=now)
        self.assertFalse(old.pose_fits(4,4,0));self.assertLessEqual(max([len(q) for q in old.live_surface.evidence.values()] or [0]),1)

    def test_new_obstacle_layer_always_wins_over_confirmed_floor(self):
        old=planner(door_world(),True);truth=planner(door_world(True));feed(old,truth)
        self.assertTrue(old.pose_fits(4,4,0));old.update_obstacles([[4,4]])
        self.assertFalse(old.pose_fits(4,4,0))
        scan,loc=packet(truth,stamp=100.8);old.update_live_surface(scan,loc,now=100.88)
        self.assertFalse(old.pose_fits(4,4,0))

    def test_no_return_cannot_clear_old_obstacle_or_unknown(self):
        old=planner(door_world(),True);truth=planner(door_world(True))
        for i in range(4):
            scan,loc=packet(truth,stamp=600+i*.2);scan['floor']=[];scan['measured_floor']=[]
            old.update_live_surface(scan,loc,now=600.08+i*.2)
        self.assertFalse(old.pose_fits(4,4,0));self.assertEqual(old.live_surface.free_cells,set())

if __name__=='__main__':unittest.main()

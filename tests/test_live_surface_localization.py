"""Independent temporary-map projection checks in a door-opening world.

Synthetic geometry and exact labeled poses only; no robot, socket or deployment.
Positive traversal is required alongside rejection cases. Static map bytes must
remain unchanged even when a temporary live passage becomes navigable.
"""
import copy
import math
import unittest
import numpy as np
from m20.navigation_planner import GridPlanner
from m20.point_navigation import simulated_scan


def door_world(open_door=False,unknown=False):
    grid=np.full((80,80),254,np.uint8)
    grid[:2]=0;grid[-2:]=0;grid[:,:2]=0;grid[:,-2:]=0
    grid[:,39:41]=0
    if open_door:grid[34:46,39:41]=254
    if unknown:grid[34:46,39:41]=205
    return grid


def planner(grid,live=False):
    h,w=grid.shape
    raw=('P5\n%d %d\n255\n'%(w,h)).encode()+grid.tobytes()
    return GridPlanner(raw,.1,[0.,0.,0.],enable_live_surface=live)


def packet(truth,actual=(2.8,4.,0.),reported=None,stamp=100.):
    scan=simulated_scan(truth,list(actual),stamp=stamp)
    reported=list(actual if reported is None else reported)
    scan.update(protocol=1,demo=False,frame_id='base_link',pose_frame_id='map',
                pose_stamp=stamp,stamp=stamp,age=.08,pose=reported,
                pose_source='/ODOM',_velocity_body=[0.,0.],_speed_mps=0.)
    loc=dict(normal=True,confidence=.96,pose=reported,age=.08)
    return scan,loc


def feed(target,truth,actual=(2.8,4.,0.),reported=None,start=100.,count=4):
    result=None
    for i in range(count):
        stamp=start+i*.2;scan,loc=packet(truth,actual,reported,stamp)
        result=target.update_live_surface(scan,loc,now=stamp+.08)
    return result


class LiveSurfaceLocalizationTests(unittest.TestCase):
    def test_open_door_with_parallel_wall_anchors_must_become_traversable(self):
        # Only the front wall and door jambs are in the 2 m projection radius.
        # A mandatory second wall normal would reject this ordinary open door.
        old=planner(door_world(),True);truth=planner(door_world(True))
        original=old.original.copy();base=old.base_free.copy()
        self.assertFalse(old.pose_fits(4.,4.,0.))
        feed(old,truth)
        self.assertTrue(old.pose_fits(4.,4.,0.))
        route=old.plan([2.8,4.,0.],[5.2,4.],0.)
        self.assertTrue(route['ok'])
        self.assertLess(route['length_m'],3.)
        np.testing.assert_array_equal(old.original,original)
        np.testing.assert_array_equal(old.base_free,base)

    def test_stable_wrong_pose_cannot_project_other_side_floor_through_wall(self):
        old=planner(door_world(),True);truth=planner(door_world())
        # Both pose channels agree with the SAME one-metre error. Repeated
        # normal/high-confidence messages are not independent proof of location.
        feed(old,truth,reported=(3.8,4.,0.))
        self.assertFalse(old.pose_fits(4.,4.,0.))
        with self.assertRaises(ValueError):old.plan([2.8,4.,0.],[5.2,4.],0.)

    def test_duplicate_source_stamp_does_not_accumulate_three_observations(self):
        old=planner(door_world(),True);truth=planner(door_world(True))
        scan,loc=packet(truth)
        for now in [100.08,100.28,100.48,100.68]:old.update_live_surface(scan,loc,now=now)
        self.assertFalse(old.pose_fits(4.,4.,0.))

    def test_closed_door_after_opening_blocks_on_first_actual_observation(self):
        old=planner(door_world(),True);open_truth=planner(door_world(True));closed_truth=planner(door_world())
        feed(old,open_truth);self.assertTrue(old.pose_fits(4.,4.,0.))
        scan,loc=packet(closed_truth,stamp=100.8)
        old.update_live_surface(scan,loc,now=100.88)
        self.assertFalse(old.pose_fits(4.,4.,0.))

    def test_global_location_loss_discards_map_projection_but_not_local_scan(self):
        old=planner(door_world(),True);truth=planner(door_world(True))
        feed(old,truth);self.assertTrue(old.pose_fits(4.,4.,0.))
        scan,loc=packet(truth,stamp=100.8);original_floor=copy.deepcopy(scan['floor']);loc['normal']=False
        old.update_live_surface(scan,loc,now=100.88)
        self.assertFalse(old.pose_fits(4.,4.,0.))
        self.assertEqual(scan['floor'],original_floor)

    def test_low_quality_stale_or_desynchronized_pose_cannot_clear_old_wall(self):
        truth=planner(door_world(True))
        for failure in ['quality','scan_age','loc_age','pair_time','pose_mismatch']:
            with self.subTest(failure=failure):
                old=planner(door_world(),True)
                for i in range(4):
                    stamp=100+i*.2;scan,loc=packet(truth,stamp=stamp)
                    if failure=='quality':loc['confidence']=.4
                    elif failure=='scan_age':scan['age']=.8
                    elif failure=='loc_age':loc['age']=1.5
                    elif failure=='pair_time':scan['pose_stamp']=stamp-.25
                    else:loc['pose']=[2.8,4.6,0.]
                    old.update_live_surface(scan,loc,now=stamp+.08)
                self.assertFalse(old.pose_fits(4.,4.,0.))

    def test_pose_jump_clears_previous_temporary_passage(self):
        old=planner(door_world(),True);truth=planner(door_world(True))
        feed(old,truth);self.assertTrue(old.pose_fits(4.,4.,0.))
        scan,loc=packet(truth,reported=(4.1,4.,0.),stamp=100.8)
        old.update_live_surface(scan,loc,now=100.88)
        self.assertFalse(old.pose_fits(4.,4.,0.))

    def test_missing_ground_strip_is_not_invented_as_unknown_map_free_space(self):
        old=planner(door_world(unknown=True),True);truth=planner(door_world(True))
        for i in range(4):
            stamp=100+i*.2;scan,loc=packet(truth,stamp=stamp)
            # A connected unsupported strip through the doorway, not an
            # isolated cell that a measured-neighbour interpolator might fill.
            for key in ['floor','measured_floor']:
                scan[key]=[p for p in scan[key] if not 5<=p[0]<=7]
            old.update_live_surface(scan,loc,now=stamp+.08)
        self.assertFalse(old.pose_fits(4.,4.,0.))


if __name__=='__main__':unittest.main()

"""Timestamp alignment uses measured motion without rewriting the scan pose."""
import math
import unittest
from m20.navigation_validation import align_observed_pose,poses_consistent
from test_live_surface_localization import planner,packet
from test_start_pose_recovery import room


class MotionAlignmentTests(unittest.TestCase):
    def test_normal_turn_is_compared_at_the_udp_pose_time(self):
        p=planner(room(),True);scan,loc=packet(p,actual=(3.,3.,0.))
        scan['age']=.5;loc.update(age=.02,velocity_body=[.4,0.],yaw_rate=.9,motion_age=.02)
        original=list(scan['pose']);loc['pose']=align_observed_pose(original,scan['age'],loc)
        self.assertGreater(loc['pose'][2],.25)
        report=p.update_live_surface(scan,loc)
        self.assertTrue(report['accepted'],report);self.assertEqual(scan['pose'],original)
    def test_static_large_jump_is_still_rejected(self):
        p=planner(room(),True);scan,loc=packet(p,actual=(3.,3.,0.))
        loc.update(velocity_body=[0.,0.],yaw_rate=0.,motion_age=.01);loc['pose']=[4.,3.,0.]
        self.assertFalse(p.update_live_surface(scan,loc)['accepted'])
    def test_missing_or_old_motion_does_not_invent_a_correction(self):
        pose=[1.,2.,.3]
        for loc in [dict(age=0),dict(age=0,motion_age=2,velocity_body=[1.,0.],yaw_rate=2.)]:
            self.assertEqual(align_observed_pose(pose,.4,loc),pose)
    def test_older_udp_pose_can_be_explained_by_measured_turn(self):
        pose=[3.,3.,.4];loc=dict(age=.65,motion_age=.01,velocity_body=[0.,0.],yaw_rate=.9)
        loc['pose']=align_observed_pose(pose,.15,loc)
        self.assertLess(loc['pose'][2],0);self.assertTrue(poses_consistent(pose,.15,loc))
    def test_matching_original_pose_is_not_rejected_by_receive_age_uncertainty(self):
        pose=[3.,3.,0.];loc=dict(pose=pose.copy(),age=.01,motion_age=.01,velocity_body=[0.,0.],yaw_rate=1.)
        self.assertGreater(align_observed_pose(pose,.6,loc)[2],.25)
        self.assertTrue(poses_consistent(pose,.6,loc))


if __name__=='__main__':unittest.main()

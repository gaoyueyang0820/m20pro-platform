"""Mapping-frame exploration must not start on a drifted or frozen pose."""
import time
import unittest

from m20.autonomy_runtime import exploration_pose_consistency, exploration_pose_gate_required


def odom(x=0., y=0., age=0.2):
    return dict(PosX=x, PosY=y, received_at=time.time() - age)


class ExplorationPoseConsistencyTests(unittest.TestCase):
    def test_matching_frames_are_accepted(self):
        self.assertIsNone(exploration_pose_consistency(odom(1.0, 2.0), odom(1.05, 2.02)))

    def test_a_missing_map_pose_is_refused(self):
        message = exploration_pose_consistency(None, odom())
        self.assertIn('定位输出', message)

    def test_a_frozen_map_pose_is_refused(self):
        message = exploration_pose_consistency(odom(age=54.), odom())
        self.assertIn('未更新', message)

    def test_a_drifted_slam_frame_is_refused_with_its_distance(self):
        message = exploration_pose_consistency(odom(-0.89, 0.09), odom(0.232, -1.173))
        self.assertIn('相差', message)
        self.assertIn('1.7', message)

    def test_a_missing_or_stale_slam_pose_is_not_judged_before_capture(self):
        self.assertIsNone(exploration_pose_consistency(odom(), None))
        self.assertIsNone(exploration_pose_consistency(odom(), odom(age=30.)))

    def test_unusable_values_are_refused_not_crashed(self):
        message = exploration_pose_consistency(dict(received_at=time.time(), PosX='bad', PosY=0.), odom())
        self.assertIn('无法解析', message)


    def test_only_extending_a_saved_map_needs_the_map_frame(self):
        self.assertTrue(exploration_pose_gate_required('explore', dict(extend=True)))
        self.assertFalse(exploration_pose_gate_required('explore', dict(extend=False)))
        self.assertFalse(exploration_pose_gate_required('explore', {}))
        self.assertFalse(exploration_pose_gate_required('follow', dict(extend=True)))
        self.assertFalse(exploration_pose_gate_required('explore', None))

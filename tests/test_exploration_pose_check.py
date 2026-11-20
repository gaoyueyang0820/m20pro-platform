"""Mapping-frame exploration must not start on a drifted or frozen pose."""
import time
import unittest

from m20.autonomy_runtime import exploration_pose_consistency, exploration_pose_gate_required


def odom(x=0., y=0., age=0.2):
    return dict(PosX=x, PosY=y, received_at=time.time() - age)


class ExplorationPoseConsistencyTests(unittest.TestCase):
    def test_a_fresh_good_match_is_accepted(self):
        self.assertIsNone(exploration_pose_consistency(odom(1.0, 2.0), dict(error=.05, inlier_ratio=.99)))

    def test_a_missing_map_pose_is_refused(self):
        message = exploration_pose_consistency(None, odom())
        self.assertIn('定位输出', message)

    def test_freeze_detection_belongs_to_the_caller(self):
        # The source stamps a clock that can differ by minutes, so an absolute
        # age here would be meaningless; the gate watches repeated local reads.
        self.assertIsNone(exploration_pose_consistency(odom(age=54.), dict(error=.05, inlier_ratio=.99)))

    def test_a_poor_match_is_refused_with_its_measured_error(self):
        message = exploration_pose_consistency(odom(), dict(error=.42, inlier_ratio=.6))
        self.assertIn('匹配误差', message)
        self.assertIn('.42', message)

    def test_missing_quality_is_not_judged(self):
        self.assertIsNone(exploration_pose_consistency(odom(), None))
        self.assertIsNone(exploration_pose_consistency(odom(), {}))

    def test_unusable_values_are_refused_not_crashed(self):
        message = exploration_pose_consistency({}, dict(error=.1))
        self.assertIn('缺少地图定位输出', message)

    def test_a_judged_pose_is_not_compared_by_raw_coordinates(self):
        # A correctly anchored session can start its SLAM frame anywhere.
        self.assertIsNone(exploration_pose_consistency(odom(10., 0.), dict(error=.05, inlier_ratio=.99)))


    def test_only_extending_a_saved_map_needs_the_map_frame(self):
        self.assertTrue(exploration_pose_gate_required('explore', dict(extend=True)))
        self.assertFalse(exploration_pose_gate_required('explore', dict(extend=False)))
        self.assertFalse(exploration_pose_gate_required('explore', {}))
        self.assertFalse(exploration_pose_gate_required('follow', dict(extend=True)))
        self.assertFalse(exploration_pose_gate_required('explore', None))

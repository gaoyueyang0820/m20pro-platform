"""Synthetic paired LiDAR frames. No robot, camera, network, or map writes."""
import math
import unittest

from m20.autonomy_core import Explorer, clearance, disk
from m20.moving_obstacles import MovingObstacles


def scan(stamp, points, pose=(0., 0., 0.), floor=()):
    """Place precisely measured obstacle clusters in a moving body frame."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    boxes = {}
    for wx, wy in points:
        dx, dy = wx-x, wy-y
        px, py = c*dx+s*dy, -s*dx+c*dy
        cell = (round(px/.2), round(py/.2))
        box = (px-.02, py-.02, px+.02, py+.02)
        if cell in boxes:
            old = boxes[cell]
            box = (min(old[0], box[0]), min(old[1], box[1]), max(old[2], box[2]), max(old[3], box[3]))
        boxes[cell] = box
    return dict(stamp=stamp, pose=pose, obstacles=list(boxes), obstacle_bounds=list(boxes.values()),
                floor=[cell for cell in floor if cell not in boxes])


class MovingObstaclesTests(unittest.TestCase):
    def test_coherent_motion_requires_distinct_timed_scans_and_drops_trail(self):
        tracker = MovingObstacles()
        first = tracker.observe(scan(10., [(1., 0.), (1., .2)]))
        self.assertEqual(first.confirmed, set())
        self.assertEqual(first.temporary, first.current)
        duplicate = tracker.observe(scan(10., [(1.4, 0.), (1.4, .2)]))
        self.assertEqual(duplicate.moving_tracks, 0)
        second = tracker.observe(scan(10.2, [(1.2, 0.), (1.2, .2)]))
        self.assertEqual(second.moving_tracks, 0)
        third = tracker.observe(scan(10.4, [(1.4, 0.), (1.4, .2)]))
        self.assertEqual(third.moving_tracks, 1)
        self.assertEqual(third.confirmed, set())
        self.assertEqual(third.temporary, third.current)
        self.assertTrue(first.current.isdisjoint(third.temporary))
        left = tracker.observe(scan(11.2, []))
        self.assertEqual(left.moving_tracks, 0)
        self.assertEqual(left.temporary | left.confirmed, set())

    def test_robot_motion_does_not_move_a_stationary_world_cluster(self):
        tracker = MovingObstacles()
        points = [(1.8, .0), (1.8, .2)]
        for i in range(5):
            result = tracker.observe(scan(20.+i*.2, points, pose=(i*.07, i*.03, i*.08)))
            self.assertEqual(result.moving_tracks, 0)
        self.assertEqual(result.confirmed, result.current)

    def test_pose_epoch_change_cannot_complete_a_track(self):
        tracker = MovingObstacles()
        tracker.observe(scan(30., [(1., 0.)]), epoch='a')
        tracker.observe(scan(30.2, [(1.2, 0.)]), epoch='a')
        changed = tracker.observe(scan(30.4, [(1.4, 0.)]), epoch='b')
        self.assertEqual(changed.moving_tracks, 0)
        self.assertEqual(changed.temporary, changed.current)

    def test_occluded_single_observation_is_not_promoted_to_static(self):
        tracker = MovingObstacles()
        first = tracker.observe(scan(35., [(1., 0.)]))
        self.assertEqual(first.temporary, {(5,0)})
        occluded = tracker.observe(scan(35.2, []))
        self.assertEqual(occluded.temporary, first.temporary)
        expired = tracker.observe(scan(35.8, []))
        self.assertEqual(expired.confirmed | expired.temporary, set())

    def test_exploration_keeps_wall_and_full_collision_scan(self):
        explorer = Explorer(3)
        wall = [(1.8, y*.2) for y in range(-7, 8)]
        floor = disk(2)
        initial = scan(40., wall+[(.6,-.4), (.8,-.4)], floor=floor)
        explorer.ingest(initial)
        explorer.ingest(scan(40.2, wall+[(.6,-.2), (.8,-.2)], floor=floor))
        current = scan(40.4, wall+[(.6,0.), (.8,0.)], floor=floor)
        explorer.ingest(current)
        self.assertIs(explorer.current_scan, current)
        self.assertIn((3,0), explorer.obstacles)
        self.assertFalse(clearance(current,.35,0)[0])
        self.assertIn((9,0), explorer.obstacles)
        self.assertNotIn((3,-2), explorer.obstacles)
        # No returns beyond the robot alone do not erase a confirmed wall.
        explorer.ingest(scan(41.2, [], floor=disk(.55)))
        self.assertIn((9,0), explorer.obstacles)
        self.assertNotIn((3,0), explorer.obstacles)
        self.assertNotIn((3,0), explorer.free)


if __name__=='__main__':
    unittest.main()

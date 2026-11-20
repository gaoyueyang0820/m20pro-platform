"""The mapping follower may reuse floor it has already scanned."""
import unittest

from m20.autonomy_core import RESOLUTION, merge_exploration_floor


def scan(floor=(), pose=(0., 0., 0.)):
    return dict(pose=list(pose), floor=[list(p) for p in floor], obstacles=[],
                geometry_version=2, resolution=RESOLUTION)


class ExplorationMemoryTests(unittest.TestCase):
    def test_scanned_floor_is_added_around_the_body(self):
        free = {(x, y) for x in range(-5, 6) for y in range(-3, 4)}
        merged = merge_exploration_floor(scan(), free)
        self.assertGreater(len(merged['floor']), 0)
        self.assertGreater(merged['memory_floor_cells'], 0)

    def test_known_floor_is_not_duplicated(self):
        free = {(0, 0), (1, 0)}
        merged = merge_exploration_floor(scan(floor=[(0, 0)]), free)
        self.assertEqual(merged['memory_floor_cells'], 1)

    def test_far_floor_is_not_merged(self):
        merged = merge_exploration_floor(scan(), {(60, 60)})
        self.assertIs(merged, merged)
        self.assertEqual(len(merged['floor']), 0)

    def test_moving_memory_does_not_add_obstacles(self):
        merged = merge_exploration_floor(scan(), {(1, 1)})
        self.assertEqual(merged['obstacles'], [])


if __name__ == '__main__':
    unittest.main()

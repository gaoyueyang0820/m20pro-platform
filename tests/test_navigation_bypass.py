"""Stopping-margin gate used to keep tracking while a bypass is prepared."""
import unittest

from m20.point_navigation import bypass_margin


class Follower:
    def __init__(self, stopping):
        self.last = dict(stopping_distance_m=stopping)


class NavigationBypassMarginTests(unittest.TestCase):
    def scan(self, obstacles=((6, 0),), floor=None):
        return dict(pose=[0., 0., 0.],
                    floor=floor if floor is not None else [[x, 0] for x in range(10)],
                    obstacles=[list(cell) for cell in obstacles], obstacle_bounds=[],
                    measured_floor=[], geometry_version=1, resolution=.2, age=0.)

    def test_margin_is_free_distance_minus_stopping_distance(self):
        margin = bypass_margin(self.scan(), Follower(.3))
        self.assertAlmostEqual(margin, 1.1 - .3, places=2)

    def test_a_near_obstacle_leaves_no_margin(self):
        margin = bypass_margin(self.scan(obstacles=((1, 0),)), Follower(.3))
        self.assertAlmostEqual(margin, .1 - .3, places=2)
        self.assertLess(margin, 0.)

    def test_no_forward_obstacle_means_the_margin_is_unknown(self):
        self.assertIsNone(bypass_margin(self.scan(obstacles=()), Follower(.3)))

    def test_missing_stopping_distance_is_treated_as_zero(self):
        margin = bypass_margin(self.scan(), Follower(None))
        self.assertAlmostEqual(margin, 1.1, places=2)

    def test_a_scan_without_floor_geometry_cannot_certify_a_margin(self):
        scan = self.scan()
        scan.pop('floor')
        scan.pop('pose')
        self.assertIsNone(bypass_margin(scan, Follower(.3)))

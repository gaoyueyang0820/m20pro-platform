"""A doorway that sits off the heading is entered by centring, not by turning."""
import unittest

from m20.local_trajectory import TrajectoryFollower


def doorway_scan(gap_y=(1, 4)):
    floor = [(x, y) for x in range(-20, 21) for y in range(-20, 21)]
    # A wall at 0.8 m with a gap on the robot's left; heading straight would hit it.
    wall = [(4, y) for y in range(-20, 21) if not (gap_y[0] <= y <= gap_y[1])]
    return dict(floor=floor, obstacles=wall, pose=[0, 0, 0],
                geometry_version=2, resolution=.2, age=0., stamp=1., _velocity_body=[0., 0.], _speed_mps=0.)


class PassageCentringTests(unittest.TestCase):
    def test_an_offset_doorway_gets_a_side_step_towards_the_gap(self):
        follower = TrajectoryFollower(.35, .7)
        follower.set_path([(0., 0.), (3., 0.)])
        result = follower.command(doorway_scan(), 0., None)
        self.assertGreater(result['axes'][1], 0., 'expected a leftward step towards the gap')

    def test_an_aligned_doorway_does_not_add_a_side_step(self):
        follower = TrajectoryFollower(.35, .7)
        follower.set_path([(0., 0.), (3., 0.)])
        # Gap centred on the heading: no centring step is needed.
        result = follower.command(doorway_scan(gap_y=(-2, 0)), 0., None)
        self.assertLessEqual(abs(result['axes'][1]), .25)

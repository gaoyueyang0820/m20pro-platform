"""Regressions for the review findings (offline, synthetic only)."""
import json
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from m20.autonomy_core import disk
from m20.bypass_planner import _snapshot
from m20.local_trajectory import TrajectoryFollower
from m20.map_fusion import multi_frame_registration, pose_matrix, transform, write_pcd
from m20.navigation_planner import GridPlanner
from m20.onboard_exploration import OnboardExploration
from m20.point_navigation import simulated_scan


def grid_planner(grid):
    h, w = grid.shape
    return GridPlanner(('P5\n%d %d\n255\n' % (w, h)).encode() + grid.tobytes(), .1, [0, 0, 0])


def world_grid():
    grid = np.full((90, 110), 254, np.uint8)
    grid[:2] = 0; grid[-2:] = 0; grid[:, :2] = 0; grid[:, -2:] = 0
    return grid


class SlowBrakeModel:
    """Measured latency and deceleration that are worse than the defaults."""

    def __init__(self, latency, decel):
        self.latency_s = latency
        self.decel = decel

    def latency(self, mode, default):
        return self.latency_s

    def braking_decel(self, mode, default):
        return self.decel

    def deadband(self, mode, axis, default):
        return .2


class BrakingRegressionTests(unittest.TestCase):
    def test_a_slow_measured_brake_extends_the_stopping_distance(self):
        planner = grid_planner(world_grid())
        pose = [1., 1., 0.]
        scan = simulated_scan(planner, pose, 1)
        scan['_velocity_body'] = [.4, 0.]
        scan['_speed_mps'] = .4
        follower = TrajectoryFollower(motion_model=SlowBrakeModel(.5, .2), motion_mode=0)
        follower.set_path([(1., 1.), (6., 1.)])
        result = follower.command(scan, 0., None)
        # 0.4*0.5 + 0.4^2/(2*0.2) = 0.60 m; clipping to .35/.35 gave 0.369 m.
        self.assertGreaterEqual(result['stopping_distance_m'], .55)
        self.assertLessEqual(result['stopping_model']['reaction_s'], .5 + .65)


class BypassSnapshotTests(unittest.TestCase):
    def test_the_snapshot_keeps_geometry_precision(self):
        scan = dict(pose=[0., 0., 0.], floor=[[1, 0]], obstacles=[[5, 0]],
                    obstacle_bounds=[[.9, -.05, 1.1, .05]], measured_floor=[[1, 0]],
                    geometry_version=2, resolution=.2, age=0., stamp=1.)
        snapshot = _snapshot(scan)
        self.assertEqual(snapshot['geometry_version'], 2)
        self.assertEqual(snapshot['obstacle_bounds'], [[.9, -.05, 1.1, .05]])


class CaptureConfirmationTests(unittest.TestCase):
    def test_a_capture_that_stays_unconfirmed_stops_renewing(self):
        task = OnboardExploration(SimpleNamespace(done=False), 'a' * 32, 'b' * 32, 'fixture', ('synthetic', 'token'))
        task.last_confirmed = time.monotonic() - 9.
        with self.assertRaisesRegex(ValueError, '长时间无法确认'):
            task._beat()

    def test_a_confirmed_capture_renews_and_refreshes_the_window(self):
        task = OnboardExploration(SimpleNamespace(done=False), 'a' * 32, 'b' * 32, 'fixture', ('synthetic', 'token'))
        status = dict(phase='collecting', mapping=dict(request_id='b' * 32))
        task._read_status(lambda: status)
        self.assertGreater(task.last_confirmed, time.monotonic() - 5.)


class GosExplorationContextTests(unittest.TestCase):
    def test_the_context_exposes_the_shared_motion_model(self):
        from bridge.gos_exploration import _ExplorationContext
        model = object()
        service = SimpleNamespace(lock=threading.RLock(), link=None, motion_response=model)
        context = _ExplorationContext(service, dict(id='a' * 32, capture_id='b' * 32, anchor=None))
        self.assertIs(context.motion_response, model)


class MultiFrameRegistrationTests(unittest.TestCase):
    def fixture(self, folder, body, frames=4, first_x=1.):
        folder.mkdir(parents=True)
        session = folder / '.sessions/session_0'
        (session / 'lidar_cloud').mkdir(parents=True)
        for index in range(frames):
            write_pcd(session / 'lidar_cloud' / ('%d.pcd' % index), body)
        rows = ['%d %.3f 0 0 0 0 0 1' % (index + 1, first_x) for index in range(frames)]
        (session / 'poses.txt').write_text('\n'.join(rows) + '\n')
        return folder

    def test_later_frames_are_projected_with_the_first_frame_guess(self):
        height = np.linspace(-.2, 1.35, 19)
        wall = np.array([[x, y, z] for x in np.linspace(-2.5, 2.5, 60) for z in height
                         for y in (-1.1, 1.1)]).astype(np.float32)
        wall = np.column_stack([wall, np.ones(len(wall), np.float32)])
        with tempfile.TemporaryDirectory() as temp:
            folder = self.fixture(Path(temp) / 'session', wall)
            poses = np.loadtxt(folder / '.sessions/session_0/poses.txt', ndmin=2)
            first = pose_matrix(poses[0, 1:])
            guess = np.linalg.inv(first)  # anchor_T = I, so guess = inv(first)
            target = wall[:, :3]
            delta, quality, merged = multi_frame_registration(
                folder, poses, guess, target, lambda source, target: np.eye(4),
                ValueError('occluded start'))
            np.testing.assert_allclose(delta, np.eye(4), atol=1e-6)
            self.assertGreaterEqual(quality['multi_frame']['frames'], 2)
            self.assertGreater(len(merged), 0)

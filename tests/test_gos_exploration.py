"""Exercise the GOS exploration mission with synthetic SLAM and feedback."""
import base64
import hashlib
from pathlib import Path
import tempfile
import time
import unittest

from bridge.gos_navigation import GOSNavigation, pack_grid
from test_gos_patrol import LocalLink, LocalScans


class MappingLink(LocalLink):
    def __init__(self):
        super().__init__()
        self.located = False
        self.report_moving = False

    def record(self, kind, command):
        result = super().record(kind, command)
        if (kind, command) == (1002, 4) and self.report_moving:
            result['items']['MotionStatus']['LinearX'] = .18
        return result


class MappingScans(LocalScans):
    pose_source = '/SLAM_ODOM'
    age = 0.

    def call(self, path, request, timeout):
        if path != '/sensors/autonomy' or request.get('mapping') is not True:
            raise AssertionError('GOS exploration must request a mapping observation')
        return dict(super().call(path, request, timeout), pose_source=self.pose_source, age=self.age)


class GOSExplorationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.link = MappingLink()
        self.scans = MappingScans(self.link)
        self.links_opened = 0

        def link_factory(_):
            self.links_opened += 1
            return self.link

        self.service = GOSNavigation(Path(temporary.name), lambda: self.scans,
                                     aos='10.21.31.103', enabled=True, link_factory=link_factory)
        self.addCleanup(self.service.close)
        self.identity = dict(id='1'*32, owner='2'*32)
        self.capture = '3'*32
        self.request = dict(self.identity, capture_id=self.capture, duration_s=20., radius=8.,
                            forward_limit=.35, yaw_limit=.70, expand=False)

    def wait(self, predicate, timeout=4.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(.02)
        status = self.service.status()
        self.fail(f"Condition did not become true: {status['phase']} · {status.get('message')}")

    def start(self, request=None):
        return self.service.dispatch('/missions/exploration/start', request or self.request)['mission']

    def wait_motion(self):
        self.wait(lambda: any(any(axes) for _, axes in self.link.events))
        return self.wait(lambda: self.service.status()['exploration']['map'])

    def wait_stop(self, phase):
        return self.wait(lambda: (self.service.status()['phase'] == phase
                                  and self.service.status()['stop_confirmed']))

    def test_exploration_runs_without_navigation_pose_and_excludes_patrol(self):
        self.assertEqual(self.service.capabilities()['exploration_protocol'], 1)
        first = self.start()
        self.assertEqual(first['kind'], 'exploration')
        self.assertEqual(first['capture_id'], self.capture)
        self.assertEqual(self.start()['id'], first['id'])
        view = self.wait_motion()
        self.assertTrue(view['floor'])
        self.assertTrue(view['observation']['live'])
        self.assertEqual(view['capture_id'], self.capture)
        raw = b'P5\n32 32\n255\n' + bytes([254])*32*32
        map_ = dict(name='synthetic', grid_id=0, sha256=hashlib.sha256(raw).hexdigest(),
                    yaml_sha256='a'*64, resolution=.1, origin=[0., 0., 0.])
        patrol = dict(id='4'*32, owner='5'*32, map=map_, grid=pack_grid(raw, map_),
                      points=[dict(x=1., y=1., yaw=0.)])
        with self.assertRaisesRegex(ValueError, '仍在执行'):
            self.service.dispatch('/missions/start', patrol)
        self.assertEqual(self.links_opened, 1)

    def test_expired_lease_parks_and_late_pulse_does_not_resume(self):
        self.start()
        self.wait_motion()
        with self.service.lock:
            self.service.lease_until = time.monotonic() - .01
        self.wait_stop('cancelled')
        expired = self.service.lease_until
        reply = self.service.dispatch('/missions/pulse', dict(self.identity, capture_id=self.capture))
        self.assertEqual(reply['mission']['phase'], 'cancelled')
        self.assertEqual(self.service.lease_until, expired)
        self.assertEqual(self.link.events[-1][1], (0., 0., 0.))

    def test_pulse_rejects_wrong_owner_and_parks_on_changed_capture(self):
        self.start()
        self.wait_motion()
        with self.assertRaisesRegex(ValueError, '身份不匹配'):
            self.service.dispatch('/missions/pulse', dict(self.identity, owner='4'*32, capture_id=self.capture))
        self.assertEqual(self.service.status()['phase'], 'running')
        previous = self.service.lease_until
        self.service.dispatch('/missions/pulse', dict(self.identity, capture_id=self.capture))
        self.assertGreater(self.service.lease_until, previous)
        with self.assertRaisesRegex(ValueError, '采集会话不匹配'):
            self.service.dispatch('/missions/pulse', dict(self.identity, capture_id='4'*32))
        self.wait_stop('cancelled')
        self.assertEqual(self.link.events[-1][1], (0., 0., 0.))

    def test_deadline_completion_waits_for_actual_stationary_feedback(self):
        self.start()
        self.wait_motion()
        self.link.report_moving = True
        with self.service.lock:
            self.service.navigator.deadline = time.monotonic() - .01
        self.wait(lambda: self.service.status()['phase'] == 'cancelling')
        self.assertFalse(self.service.status()['stop_confirmed'])
        self.link.report_moving = False
        self.wait_stop('completed')

    def test_navigation_pose_or_expired_scan_cannot_authorize_motion(self):
        self.scans.pose_source = '/ODOM'
        self.start()
        self.wait(lambda: len(self.scans.stamps) >= 2)
        self.scans.pose_source = '/SLAM_ODOM'
        self.scans.age = .8
        self.wait(lambda: len(self.scans.stamps) >= 4)
        self.assertFalse(any(any(axes) for _, axes in self.link.events))
        with self.service.lock:
            self.service.navigator.deadline = time.monotonic() - .01
        self.wait_stop('failed')

    def test_same_capture_resume_preserves_observations_and_origin(self):
        self.start()
        self.wait_motion()
        original = self.service.navigator.explorer
        origin = original.origin
        self.service.dispatch('/missions/cancel', dict(self.identity, after='paused'))
        self.wait_stop('paused')
        self.service.watchdog.join(timeout=.6)
        self.service.navigator.worker.join(timeout=.6)
        self.start(dict(self.request, id='4'*32))
        self.assertIs(self.service.navigator.explorer, original)
        self.wait(lambda: self.service.navigator.scan is not None)
        self.assertEqual(self.service.navigator.explorer.origin, origin)

    def test_prior_grid_requires_capture_anchor(self):
        raw = b'P5\n2 2\n255\n' + bytes([254])*4
        grid = dict(pgm=base64.b64encode(raw).decode(), resolution=.1, origin=[0., 0., 0.])
        with self.assertRaisesRegex(ValueError, '定位锚点'):
            self.start(dict(self.request, prior_grid=grid))
        self.assertEqual(self.links_opened, 0)

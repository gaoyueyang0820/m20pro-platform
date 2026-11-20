"""A momentary localization gap must wait, not end the operator's task."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from m20.autonomy_runtime import Autonomy


def odom(age=0.2, x=0., y=0.):
    return dict(PosX=x, PosY=y, received_at=time.time() - age)


class App:
    demo = False

    def __init__(self, status):
        self.lock = threading.RLock()
        self.store = SimpleNamespace(value={})
        self.hub = SimpleNamespace(connections=SimpleNamespace(value=dict(sensor_source='nos')))
        self._status = status
        self.proxy = lambda *args, **kwargs: self._status

    def set_status(self, status):
        self._status = status


class PoseGateWaitTests(unittest.TestCase):
    def task(self, status):
        app = App(status)
        autonomy = Autonomy(app)
        autonomy.run_id = 'run-1'
        autonomy.message = ''
        return app, autonomy

    def test_a_recovered_pose_starts_without_ending_the_task(self):
        app, autonomy = self.task(dict(odom=odom(age=54.)))
        app.set_status(dict(odom=odom(), matching_quality=dict(error=.05, inlier_ratio=.99), time=time.time()))
        autonomy._wait_pose_gate('地图定位输出已 54 秒未更新，请先恢复定位',
                                 'run-1', lambda: None, timeout=2.)
        self.assertIn('定位已恢复', autonomy.message)

    def test_a_pose_that_stays_unusable_pauses_instead_of_pretending(self):
        app, autonomy = self.task(dict(odom=odom(age=54.)))
        autonomy._pose_gate_key = (0.0, 0.0, None)
        autonomy._pose_gate_changed = -10.
        clock = {'t': 0.}
        def tick(*_):
            clock['t'] += 1.
        with patch('m20.autonomy_runtime.time.monotonic', side_effect=lambda: clock['t']), \
             patch('m20.autonomy_runtime.time.sleep', side_effect=tick):
            with self.assertRaisesRegex(ValueError, '等待 30 秒仍未恢复'):
                autonomy._wait_pose_gate('地图定位输出已 54 秒未更新，请先恢复定位',
                                         'run-1', lambda: None, timeout=.05)

    def test_a_poor_match_is_waited_out_too(self):
        app, autonomy = self.task(dict(odom=odom(), matching_quality=dict(error=.42, inlier_ratio=.6)))
        with patch('m20.autonomy_runtime.time.sleep', lambda *_: None):
            with self.assertRaisesRegex(ValueError, '匹配误差'):
                autonomy._wait_pose_gate('定位匹配误差 0.42 米，请先恢复定位',
                                         'run-1', lambda: None, timeout=.05)

    def test_reading_an_unavailable_service_is_unknown_not_recovered(self):
        app, autonomy = self.task(None)
        state, reason = autonomy._pose_gate_refusal()
        self.assertEqual(state, 'unknown')
        self.assertIsNone(reason)

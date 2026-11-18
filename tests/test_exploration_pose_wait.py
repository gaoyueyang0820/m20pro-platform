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
        app.set_status(dict(odom=odom(), slam_odom=odom(), time=time.time()))
        autonomy._wait_pose_gate('地图定位输出已 54 秒未更新，请先恢复定位',
                                 'run-1', lambda: None, timeout=2.)
        self.assertIn('定位已恢复', autonomy.message)

    def test_a_pose_that_stays_unusable_pauses_instead_of_pretending(self):
        app, autonomy = self.task(dict(odom=odom(age=54.)))
        with patch('m20.autonomy_runtime.time.sleep', lambda *_: None):
            with self.assertRaisesRegex(ValueError, '等待 30 秒仍未恢复'):
                autonomy._wait_pose_gate('地图定位输出已 54 秒未更新，请先恢复定位',
                                         'run-1', lambda: None, timeout=.05)

    def test_a_drifted_pose_is_waited_out_too(self):
        app, autonomy = self.task(dict(odom=odom(-0.89, 0.09), slam_odom=odom(0.232, -1.173)))
        with patch('m20.autonomy_runtime.time.sleep', lambda *_: None):
            with self.assertRaisesRegex(ValueError, '相差'):
                autonomy._wait_pose_gate('建图坐标系与地图定位相差 1.7 米，请先重置建图坐标系或恢复定位后再开始',
                                         'run-1', lambda: None, timeout=.05)

    def test_reading_an_unavailable_service_is_not_a_pose_refusal(self):
        app, autonomy = self.task(None)
        self.assertIsNone(autonomy._pose_gate_refusal())

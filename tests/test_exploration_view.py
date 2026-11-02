"""Synthetic display tests: no sockets, robot, planner or motion outputs."""
import copy
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from exploration_view import PausedExplorationView


class PausedExplorationViewTests(unittest.TestCase):
    def setUp(self):
        self.status = dict(phase='collecting', in_progress=True, mapping=dict(request_id='capture-1'))
        hub = SimpleNamespace(_operation_context=threading.local(),
                              remote_maps=Mock(side_effect=lambda *a, **k: copy.deepcopy(self.status)),
                              same_context=lambda c: c == {'epoch': 1},
                              connections=SimpleNamespace(value={'sensor_source': 'nos'}))
        self.app = SimpleNamespace(lock=threading.RLock(), done=False, mapping=True, hub=hub)
        self.packet = dict(pose=[1., 2., 1.57], stamp=11., _valid_until=time.monotonic()+.5)
        self.a = SimpleNamespace(app=self.app, active=False, state='paused', mode='explore',
                                 generation=1, capture_id='capture-1', context={'epoch': 1},
                                 scan=dict(stamp=10., _valid_until=time.monotonic()-.7),
                                 scan_now=Mock(side_effect=lambda *a, **k: copy.deepcopy(self.packet)))
        self.observer = PausedExplorationView(self.a)
        self.view = dict(pose=[0., 0., 0.], floor=[[0, 0]], obstacles=[[4, 0]], path=[[0, 0], [1, 0]])

    def decorate(self):
        with patch('exploration_view.threading.Thread.start'):
            with self.app.lock:
                return self.observer.decorate(self.view)

    def test_manual_translation_and_rotation_are_shown_in_same_slam_frame(self):
        self.decorate()
        self.assertTrue(self.observer._collect(self.observer.token))
        result = self.decorate()
        self.assertEqual(result['pose'], [1., 2., 1.57])
        self.assertTrue(result['observation']['live'])
        self.assertEqual(result['observation']['frame'], 'slam_session')
        self.assertEqual(result['path'], [])
        self.assertEqual(self.view['pose'], [0., 0., 0.])
        self.assertEqual(self.view['obstacles'], result['obstacles'])
        self.a.scan_now.assert_called_once_with(True, role='nos')
        self.assertEqual(self.app.hub.remote_maps.call_count, 2)

    def test_capture_switch_during_scan_discards_the_new_pose(self):
        self.decorate()
        def scan(*a, **k):
            self.status['mapping']['request_id'] = 'another-capture'
            return self.packet
        self.a.scan_now.side_effect = scan
        with self.assertRaisesRegex(ValueError, '切换'):
            self.observer._collect(self.observer.token)
        self.assertIsNone(self.observer.sample)

    def test_restarted_or_reconnected_task_ignores_late_response(self):
        self.decorate()
        def scan(*a, **k):
            self.a.generation += 1
            return self.packet
        self.a.scan_now.side_effect = scan
        self.assertFalse(self.observer._collect(self.observer.token))
        self.assertIsNone(self.observer.sample)

    def test_duplicate_or_reordered_stamp_never_freshens_pose(self):
        self.decorate()
        self.assertTrue(self.observer._collect(self.observer.token))
        original = copy.deepcopy(self.observer.sample)
        for stamp in [11., 10.5]:
            self.packet.update(stamp=stamp, pose=[4., 0., 0.], _valid_until=time.monotonic()+.6)
            self.assertFalse(self.observer._collect(self.observer.token))
            self.assertEqual(self.observer.sample, original)

    def test_source_staleness_does_not_become_live_by_http_polling(self):
        self.decorate()
        self.packet['_valid_until'] = time.monotonic()-2
        with self.assertRaisesRegex(ValueError, '尚未更新'):
            self.observer._collect(self.observer.token)
        self.assertFalse(self.decorate()['observation']['live'])

    def test_finished_map_is_historical_and_never_uses_navigation_pose(self):
        self.a.state = 'completed'
        self.app.mapping = False
        self.app.pose = dict(PosX=100., PosY=100., Yaw=3.14)
        result = self.decorate()
        self.assertFalse(result['observation']['live'])
        self.assertEqual(result['pose'], self.view['pose'])
        self.assertIsNone(self.observer.token)
        self.a.scan_now.assert_not_called()

    def test_active_view_does_not_start_another_sensor_reader(self):
        self.a.active = True
        self.a.scan['_valid_until'] = time.monotonic()+.4
        result = self.decorate()
        self.assertTrue(result['observation']['live'])
        self.assertIsNone(self.observer.token)
        self.a.scan_now.assert_not_called()

    def test_real_connection_context_keeps_link_identity_without_copying_locks(self):
        link = SimpleNamespace(lock=threading.RLock())
        context = (False, link, 3)
        self.a.context = context
        self.app.hub.same_context = lambda c: c is context
        self.decorate()
        self.assertIs(self.observer.token['context'], context)
        self.assertTrue(self.observer._collect(self.observer.token))
        self.assertTrue(self.decorate()['observation']['live'])

    def test_resumed_task_does_not_start_a_second_reader_after_pending_status(self):
        self.decorate()
        def status(*args, **kwargs):
            self.a.active = True
            return self.status
        self.app.hub.remote_maps.side_effect = status
        self.assertFalse(self.observer._collect(self.observer.token))
        self.a.scan_now.assert_not_called()
        self.assertEqual(self.app.hub.remote_maps.call_count, 1)

    def test_known_capture_change_immediately_marks_even_recent_pose_historical(self):
        self.decorate()
        self.assertTrue(self.observer._collect(self.observer.token))
        self.status['mapping']['request_id'] = 'another-capture'
        with self.assertRaisesRegex(ValueError, '会话已变化'):
            self.observer._collect(self.observer.token)
        result = self.decorate()
        self.assertFalse(result['observation']['live'])
        self.assertEqual(result['pose'], [1., 2., 1.57])


if __name__ == '__main__':
    unittest.main()

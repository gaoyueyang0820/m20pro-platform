"""One operator-facing stage per navigation run, independent of internal phases."""
import threading
import unittest
from types import SimpleNamespace

from m20.point_navigation import PointNavigator, STAGE_OF_PHASE, STAGE_TEXT


class NavigationStageTests(unittest.TestCase):
    def navigator(self, phase, message=''):
        nav = dict(status='running', phase=phase, message=message)
        app = SimpleNamespace(nav=nav, lock=threading.RLock(), demo=True)
        navigator = PointNavigator(app)
        navigator.nav = nav
        return navigator, nav

    def test_each_internal_phase_maps_to_one_stage(self):
        for phase in ('preparing', 'tracking', 'finding_bypass', 'local_bypass',
                      'waiting_space', 'waiting_observation', 'station_action',
                      'completed', 'paused', 'failed'):
            with self.subTest(phase=phase):
                navigator, nav = self.navigator(phase)
                navigator._refresh_stage()
                self.assertIn(nav['stage'], STAGE_TEXT)
                self.assertTrue(nav['stage_text'])

    def test_the_detail_is_the_operator_message(self):
        navigator, nav = self.navigator('waiting_space', '正在等待可通行空间，通路恢复后继续')
        navigator._refresh_stage()
        self.assertEqual(nav['stage'], 'waiting_for_space')
        self.assertEqual(nav['detail'], '正在等待可通行空间，通路恢复后继续')
        self.assertIn('等待', nav['next_action'])

    def test_an_unknown_phase_is_passed_through_not_invented(self):
        navigator, nav = self.navigator('some_new_phase', '测试')
        navigator._refresh_stage()
        self.assertEqual(nav['stage'], 'some_new_phase')
        self.assertEqual(nav['stage_text'], '')
        self.assertEqual(nav['next_action'], '')

    def test_every_phase_carries_a_stage_after_tick(self):
        navigator, nav = self.navigator('finding_bypass', '正在更换行进方向，继续前往原目标')
        navigator.tick()
        self.assertEqual(nav['stage'], 'planning_bypass')
        self.assertEqual(nav['detail'], '正在更换行进方向，继续前往原目标')
        self.assertIn('tracking', STAGE_OF_PHASE)

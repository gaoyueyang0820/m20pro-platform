"""Background bypass planning contracts. All searches are synthetic."""
import time
import unittest
from unittest.mock import patch

from m20.bypass_planner import BypassPlanner


PLAN = dict(path=[dict(x=0., y=0.), dict(x=.2, y=0.)], goal=dict(x=.2, y=0.))


class BypassPlannerTests(unittest.TestCase):
    def scan(self, stamp=1.):
        return dict(pose=[0., 0., 0.], floor=[[1, 1], [2, 1]], obstacles=[[3, 1]],
                    _valid_until=time.monotonic() + .65, stamp=stamp, age=0.)

    def wait_ready(self, planner, timeout=2.):
        deadline = time.monotonic() + timeout
        while planner._result is None and time.monotonic() < deadline:
            time.sleep(.005)

    def test_a_result_is_returned_once_with_its_request_age(self):
        planner = BypassPlanner()
        with patch('m20.bypass_planner.plan_local_detour', return_value=PLAN) as search:
            planner.submit(self.scan(), (.5, 0.))
            self.wait_ready(planner)
            result = planner.poll(pose=[0., 0., 0.])
        self.assertIsNotNone(result)
        self.assertEqual(search.call_count, 1)
        self.assertEqual(result['goal']['x'], .2)
        self.assertGreaterEqual(result['request_age_s'], 0.)
        self.assertIsNone(planner.poll(pose=[0., 0., 0.]))
        self.assertEqual(planner.status()['completed'], 1)
        self.assertEqual(planner.status()['submitted'], 1)

    def test_a_result_older_than_the_scan_window_is_rejected(self):
        clock = [10.]
        planner = BypassPlanner(clock=lambda: clock[0])
        with patch('m20.bypass_planner.plan_local_detour', return_value=PLAN):
            planner.submit(self.scan(), (.5, 0.), pose=[0., 0., 0.], now=10.)
            self.wait_ready(planner)
        clock[0] = 10.8
        self.assertIsNone(planner.poll(pose=[0., 0., 0.]))
        self.assertEqual(planner.status()['rejected'], 1)
        self.assertEqual(planner.last, 'stale')

    def test_a_result_for_a_moved_pose_is_rejected(self):
        clock = [10.]
        planner = BypassPlanner(clock=lambda: clock[0])
        with patch('m20.bypass_planner.plan_local_detour', return_value=PLAN):
            planner.submit(self.scan(), (.5, 0.), pose=[0., 0., 0.], now=10.)
            self.wait_ready(planner)
        self.assertIsNone(planner.poll(pose=[1.2, 0., 0.]))
        self.assertEqual(planner.status()['rejected'], 1)
        self.assertEqual(planner.last, 'pose_moved')

    def test_a_failed_search_is_counted_and_returns_nothing(self):
        planner = BypassPlanner()
        with patch('m20.bypass_planner.plan_local_detour', return_value=None):
            planner.submit(self.scan(), (.5, 0.))
            self.wait_ready(planner)
        self.assertIsNone(planner.poll(pose=[0., 0., 0.]))
        self.assertEqual(planner.status()['failed'], 1)
        self.assertEqual(planner.status()['completed'], 0)

    def test_cancel_drops_a_pending_request(self):
        planner = BypassPlanner()
        with patch('m20.bypass_planner.plan_local_detour', return_value=PLAN):
            planner.submit(self.scan(), (.5, 0.))
            planner.cancel()
            time.sleep(.05)
            self.assertIsNone(planner.poll(pose=[0., 0., 0.]))

    def test_the_snapshot_isolates_the_search_from_later_scan_mutation(self):
        planner = BypassPlanner()
        scan = self.scan()
        seen = {}

        def capture(packet, goal, **kwargs):
            time.sleep(.02)
            seen['floor'] = [list(cell) for cell in packet['floor']]
            return PLAN

        with patch('m20.bypass_planner.plan_local_detour', side_effect=capture):
            planner.submit(scan, (.5, 0.))
            scan['floor'].append([9, 9])
            scan['obstacles'].clear()
            self.wait_ready(planner)
            planner.poll(pose=[0., 0., 0.])
        self.assertEqual(seen['floor'], [[1, 1], [2, 1]])

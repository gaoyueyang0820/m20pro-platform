"""Synthetic journal RPCs; bounded background transfer never contacts hardware."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from m20.collection_store import CollectionCache, CollectionJournal
from m20.collection_sync import CollectionSynchronizer


RUN, OTHER = '1'*32, '2'*32
ROBOT, ROBOT_B = 'a'*64, 'b'*64


class Clock:
    def __init__(self): self.value = 0.
    def __call__(self): return self.value
    def advance(self, seconds=2.): self.value += seconds


class CollectionSyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.remote = CollectionJournal(self.root/'robot')
        self.current = ROBOT
        self.clock = Clock()
        self.calls = []
        self.supported = True
        self.online = True
        self.run = dict(id=RUN, robot=ROBOT, executor='gos')
        self.sync = self.make_sync()
        self.remote.ensure_run(RUN, robot=ROBOT)

    def make_sync(self, **options):
        sync = CollectionSynchronizer(self.root/'desktop', lambda:self.current, self.rpc,
                                      autostart=False, clock=self.clock, **options)
        self.addCleanup(sync.close)
        return sync

    def rpc(self, run, path, body, timeout):
        self.calls.append((copy.deepcopy(run), path, copy.deepcopy(body), timeout))
        if not self.online:
            raise OSError('synthetic link unavailable')
        if path == '/missions/capabilities':
            return dict(protocol=1, collection_sync_protocol=1) if self.supported else dict(protocol=1, collection_protocol=1)
        self.assertEqual(path, '/missions/collection')
        return self.remote.page(body['id'], cursor=body['cursor'], robot=body['robot'],
                                page_size=body['limit'], journal_id=body.get('journal_id'),
                                executor=run['executor'])

    def append(self, value=1, run=RUN, robot=ROBOT):
        return self.remote.append(run, dict(kind='iot_readings', sampled_at=100.,
                                  waypoint_index=0, values=dict(temperature=value)), robot=robot)

    def next(self):
        self.clock.advance()
        self.sync.run_once()

    def test_no_continuous_download_until_a_new_waypoint_event(self):
        self.append(1)
        self.sync.notify(self.run)
        self.sync.run_once()
        first_calls = len(self.calls)
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 1)
        self.assertFalse(self.sync.snapshot(self.run)['pending'])
        for value in range(2, 7):
            self.append(value)
            self.next()
        self.assertEqual(len(self.calls), first_calls)
        self.sync.notify(self.run, target=6)
        self.next()
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 6)
        self.assertEqual(len(self.calls), first_calls+1)
        page = self.sync.cache.page(RUN, robot=ROBOT)
        self.assertTrue(page['stale'])
        self.assertFalse(page['live'])
        self.assertFalse(page['complete'])

    def test_pages_are_bounded_and_cursor_resumes_after_each_batch(self):
        self.sync = self.make_sync(page_size=2, max_pages=2)
        for value in range(9): self.append(value)
        self.remote.finish(RUN, robot=ROBOT)
        self.sync.notify(self.run, final=True)
        self.sync.run_once()
        state = self.sync.snapshot(self.run)
        self.assertEqual(state['cursor'], 4)
        self.assertTrue(state['pending'])
        self.assertEqual(len([p for _,p,_,_ in self.calls if p.endswith('/collection')]), 2)
        self.next()
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 8)
        self.next()
        state = self.sync.snapshot(self.run)
        self.assertEqual(state['cursor'], 9)
        self.assertTrue(state['complete'])
        self.assertFalse(state['pending'])
        self.assertEqual([body['cursor'] for _,path,body,_ in self.calls if path.endswith('/collection')], [0,2,4,6,8])

    def test_wall_clock_budget_counts_negotiation_and_limits_request_timeout(self):
        self.sync = self.make_sync(page_size=1, max_pages=20, budget_s=1.)
        for value in range(3): self.append(value)
        original = self.rpc
        def slow(run, path, body, timeout):
            self.clock.advance(.2 if path.endswith('capabilities') else .8)
            return original(run, path, body, timeout)
        self.sync.request = slow
        self.sync.notify(self.run)
        self.sync.run_once()
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 1)
        self.assertEqual(len(self.calls), 2)
        self.assertLessEqual(self.calls[-1][3], .8)

    def test_terminal_offline_intent_survives_restart_without_navigation_heartbeat(self):
        self.append(1)
        self.online = False
        self.sync.notify(self.run, final=True)
        self.sync.run_once()
        self.assertTrue(self.sync.snapshot(self.run)['pending'])
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 0)
        self.append(2)
        self.remote.finish(RUN, robot=ROBOT)
        self.sync.close()
        self.online = True
        restored = self.make_sync()
        restored.run_once()  # No notify(), busy mission or pulse is needed.
        self.assertTrue(restored.snapshot(self.run)['complete'])
        self.assertEqual(restored.snapshot(self.run)['cursor'], 2)
        self.assertEqual(len(restored.cache.page(RUN, robot=ROBOT)['rows']), 2)

    def test_late_response_after_identity_switch_is_not_committed(self):
        self.append(7)
        original = self.rpc
        def changed(run, path, body, timeout):
            result = original(run, path, body, timeout)
            if path.endswith('/collection'):
                self.current = ROBOT_B
            return result
        self.sync.request = changed
        self.sync.notify(self.run, final=True)
        self.sync.run_once()
        state = self.sync.snapshot(self.run)
        self.assertEqual(state['cursor'], 0)
        self.assertEqual(state['state'], 'offline')
        self.assertFalse(self.sync.cache.metadata(RUN, robot=ROBOT)['available'])
        self.assertFalse(self.sync.cache.metadata(RUN, robot=ROBOT_B)['available'])
        self.current = ROBOT
        self.sync.request = self.rpc
        self.remote.finish(RUN, robot=ROBOT)
        self.next()
        self.assertTrue(self.sync.snapshot(self.run)['complete'])

    def test_multiple_runs_and_robot_identity_reuse_remain_separate(self):
        self.append(10)
        self.append(20, run=OTHER)
        self.append(30, robot=ROBOT_B)
        runs = [self.run, dict(self.run,id=OTHER), dict(self.run,robot=ROBOT_B)]
        for run in runs:
            self.remote.finish(run['id'], robot=run['robot'])
            self.sync.notify(run, final=True)
        self.sync.run_once(); self.next()
        self.assertTrue(all(run['robot']==ROBOT for run,_,_,_ in self.calls))
        self.current = ROBOT_B
        self.next()
        for run, value in zip(runs, (10,20,30)):
            page = self.sync.cache.page(run['id'], robot=run['robot'])
            self.assertEqual([r['values']['temperature'] for r in page['rows']], [value])
            self.assertTrue(page['complete'])

    def test_legacy_capabilities_never_imply_incremental_support(self):
        self.supported = False
        self.sync.notify(self.run, final=True)
        self.sync.run_once()
        state = self.sync.snapshot(self.run)
        self.assertFalse(state['supported'])
        self.assertEqual(state['state'], 'unsupported')
        self.assertFalse(state['complete'])
        self.assertEqual([p for _,p,_,_ in self.calls], ['/missions/capabilities'])
        self.next()
        self.assertEqual(len(self.calls), 1)
        self.supported = True
        self.remote.finish(RUN, robot=ROBOT)
        self.sync.notify(self.run, final=True)
        self.next()
        self.assertTrue(self.sync.snapshot(self.run)['complete'])

    def test_missing_remote_page_retries_instead_of_completing_empty(self):
        run = dict(self.run, id=OTHER)
        self.sync.notify(run, final=True)
        self.sync.run_once()
        state = self.sync.snapshot(run)
        self.assertEqual(state['state'], 'unavailable')
        self.assertTrue(state['pending'])
        self.assertFalse(state['complete'])
        self.append(3, run=OTHER)
        self.remote.finish(OTHER, robot=ROBOT)
        self.next()
        self.assertTrue(self.sync.snapshot(run)['complete'])

    def test_final_request_waits_for_durable_seal_without_empty_page_spin(self):
        self.append(2)
        self.sync.notify(self.run, final=True)
        self.sync.run_once()
        self.assertTrue(self.sync.snapshot(self.run)['pending'])
        self.assertFalse(self.sync.snapshot(self.run)['complete'])
        self.assertEqual(len(self.calls), 2)
        self.remote.finish(RUN, robot=ROBOT)
        self.next()
        self.assertTrue(self.sync.snapshot(self.run)['complete'])
        self.assertEqual(self.calls[-1][2]['cursor'], 1)

    def test_notify_and_snapshot_never_do_disk_or_transport_work(self):
        with patch.object(self.sync.cache,'schedule',side_effect=AssertionError('disk in heartbeat')):
            self.sync.notify(self.run)
            state = self.sync.snapshot(self.run)
            self.assertTrue(state['pending'])
            self.assertEqual(self.calls, [])
        self.sync.run_once()
        self.assertEqual(len(self.calls), 2)

    def test_source_identity_error_and_full_cache_never_ack_a_page(self):
        self.append(3)
        self.sync = self.make_sync(cache=CollectionCache(self.root/'desktop', limits=dict(run_bytes=1)))
        self.sync.notify(self.run)
        self.sync.run_once()
        self.assertEqual(self.sync.snapshot(self.run)['cursor'], 0)
        self.assertEqual(self.sync.snapshot(self.run)['state'], 'error')
        self.assertTrue(self.sync.snapshot(self.run)['pending'])
        self.assertFalse(self.sync.cache.metadata(RUN, robot=ROBOT)['available'])


if __name__ == '__main__':
    unittest.main()

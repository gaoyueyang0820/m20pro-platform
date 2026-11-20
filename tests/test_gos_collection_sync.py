"""GOS journal wiring with synthetic maps/links; no real transports or motion."""
import copy
import threading
import time
import unittest
from unittest.mock import Mock, patch

from bridge.gos_navigation import GOSNavigation
from m20.collection_store import CollectionJournal
import test_gos_patrol as patrol_fixtures


ROBOT = 'a'*64


class GOSCollectionSyncTests(unittest.TestCase):
    def setUp(self):
        patrol_fixtures.GOSPatrolTests.setUp(self)
        self.request['collection_robot'] = ROBOT
        self.request['points'][0]['station_action'] = dict(camera='front', pan='none')
        self.actions.run.return_value = dict(camera='front', pan='none', people=2, sampled_at=100.)
        self.addCleanup(lambda:self.service.nav.update(status='completed'))

    def start_without_motion(self, request=None):
        with patch.object(self.service, '_start_watchdog') as watch, patch('bridge.gos_navigation.GOSPointNavigator._launch') as launch:
            result = self.service.start(request or self.request)
        self.factory.assert_not_called()
        return result, watch, launch

    def station(self):
        self.service.link = self.link  # Pure in-memory synthetic feedback.
        self.service.station_action(0, self.request['points'][0]['station_action'], self.link.pose, lambda:None)

    def test_station_only_job_has_journal_and_compact_pulses_only_send_metadata(self):
        result, _, _ = self.start_without_motion()
        self.assertEqual(self.service.collection.specs, [])
        self.assertTrue(result['collection_journal']['available'])
        self.assertEqual(result['collection_journal']['robot'], ROBOT)
        self.assertEqual(self.service.capabilities()['collection_sync_protocol'], 1)
        self.station()
        full = self.service.status()
        self.assertEqual(full['station_results'][0]['people'], 2)
        for _ in range(3):
            compact = self.service.dispatch('/missions/pulse', dict(self.identity, collection_sync_protocol=1))['mission']
            self.assertNotIn('station_results', compact)
            self.assertEqual(compact['station_results_count'], 1)
            self.assertEqual(compact['collection_journal']['high_watermark'], 1)
            self.assertNotIn('people', str(compact))
        page = self.service.dispatch('/missions/collection', dict(id=self.identity['id'], robot=ROBOT, cursor=0, limit=1))
        self.assertEqual(page['rows'][0]['kind'], 'station_result')
        self.assertEqual(page['rows'][0]['people'], 2)
        self.assertEqual(page['next_cursor'], 1)
        self.assertEqual(self.service.nav['completed'], 1)

    def test_old_requests_keep_offset_reads_without_falsely_supporting_cursor(self):
        request = copy.deepcopy(self.request)
        request.pop('collection_robot')
        self.start_without_motion(request)
        self.station()
        self.assertEqual(self.service.status()['collection_journal']['collection_sync_protocol'], 0)
        offset = self.service.dispatch('/missions/collection', dict(id=self.identity['id'], offset=0))
        self.assertEqual(len(offset['rows']), 1)
        unsupported = self.service.dispatch('/missions/collection', dict(id=self.identity['id'], robot=ROBOT, cursor=0))
        self.assertFalse(unsupported['ok'])
        self.assertEqual(unsupported['collection_sync_protocol'], 0)
        envelope = self.service.dispatch('/missions/collection', dict(id=self.identity['id'], robot=ROBOT, cursor=0, response_envelope=True))
        self.assertTrue(envelope['ok'])
        self.assertFalse(envelope['page']['ok'])
        self.assertEqual(envelope['page']['collection_sync_protocol'], 0)
        missing = self.service.dispatch('/missions/collection', dict(id='9'*32, offset=0))
        self.assertTrue(missing['ok'])
        self.assertEqual(missing['rows'], [])

    def test_collection_robot_is_validated_and_bound_into_task_digest(self):
        bad = dict(self.request, collection_robot='../unsafe')
        with self.assertRaisesRegex(ValueError, '身份'):
            self.start_without_motion(bad)
        self.start_without_motion()
        self.assertEqual(self.service.job['collection_robot'], ROBOT)
        with self.assertRaisesRegex(ValueError, '重复任务'):
            self.service.start(dict(self.request, collection_robot='b'*64))
        self.assertFalse(CollectionJournal(self.data).metadata(self.identity['id'], robot='b'*64)['available'])
        self.factory.assert_not_called()

    def test_gos_never_extends_its_five_second_safety_lease(self):
        with self.assertRaisesRegex(ValueError, '固定为 5 秒'):
            self.start_without_motion(dict(self.request, disconnect_timeout_s=30))
        self.factory.assert_not_called()

    def test_station_write_failure_does_not_advance_waypoint(self):
        self.start_without_motion()
        with patch.object(self.service.collection.journal, 'append', side_effect=ValueError('synthetic disk full')):
            with self.assertRaisesRegex(ValueError, 'disk full'):
                self.station()
        self.assertEqual(self.service.nav['completed'], 0)
        self.assertEqual(self.service.nav.get('station_results', []), [])
        self.assertIn('disk full', self.service.status(compact=True)['collection_journal']['error'])
        self.assertEqual(self.service.collection.metadata()['high_watermark'], 0)

    def test_slow_station_journal_does_not_block_safety_pulse(self):
        self.start_without_motion()
        entered, release = threading.Event(), threading.Event()
        errors = []
        original = self.service.collection.journal.append
        self.addCleanup(release.set)
        def slow(*args, **kwargs):
            entered.set(); release.wait(3)
            return original(*args, **kwargs)
        def station():
            try:self.station()
            except Exception as error:errors.append(error)
        with patch.object(self.service.collection.journal, 'append', side_effect=slow):
            worker = threading.Thread(target=station)
            worker.start()
            self.assertTrue(entered.wait(1))
            began = time.monotonic()
            pulse = self.service.dispatch('/missions/pulse', dict(self.identity, collection_sync_protocol=1))
            self.assertLess(time.monotonic()-began, 1.)
            self.assertEqual(pulse['mission']['collection_journal']['high_watermark'], 0)
            release.set(); worker.join(2)
        self.assertEqual(errors, [])
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.service.nav['completed'], 1)

    def test_journal_start_failure_never_launches_navigation(self):
        with patch('bridge.gos_navigation.GOSPointNavigator._launch') as launch, \
                patch('bridge.task_collection.CollectionJournal.ensure_run', side_effect=ValueError('synthetic quota')):
            with self.assertRaisesRegex(ValueError, 'synthetic quota'):
                self.service.start(self.request)
        launch.assert_not_called()
        self.factory.assert_not_called()
        self.assertEqual(self.service.nav['status'], 'failed')
        self.assertFalse(self.service.collection.metadata()['complete'])

    def test_restart_recovers_rows_without_sampling_or_resuming_motion(self):
        self.start_without_motion()
        self.station()
        self.service._persist()
        # No sampler/trajectory worker is running in this synthetic crash fixture.
        restored_factory = Mock(side_effect=AssertionError('restart must not connect'))
        restored = GOSNavigation(self.data, lambda:self.sensors, aos='10.1.2.3', enabled=True,
                                 link_factory=restored_factory, actions=self.actions)
        self.addCleanup(restored.close)
        self.assertIsNone(restored.collection)
        self.assertEqual(restored.status()['phase'], 'interrupted')
        self.assertFalse(restored.status()['stop_confirmed'])
        journal = restored.status(compact=True)['collection_journal']
        self.assertTrue(journal['sealed'])
        self.assertFalse(journal['complete'])
        self.assertTrue(journal['truncated'])
        page = restored.dispatch('/missions/collection', dict(id=self.identity['id'], robot=ROBOT, cursor=0))
        self.assertEqual([r['people'] for r in page['rows']], [2])
        self.assertFalse(page['complete'])
        self.assertIn('中断', page['error'])
        restored_factory.assert_not_called()

    def test_compact_saved_snapshot_never_replays_legacy_latest_values(self):
        self.service.job = dict(self.identity)
        self.service.nav.update(status='completed', phase='completed', cancel_confirmed=True,
            station_results=[dict(people=100, index=0)],
            collection=[dict(spec=dict(kind='people_count'), samples=1, latest=dict(values=dict(people=100)), peak_people=100)])
        result = self.service.dispatch('/missions/status', dict(collection_sync_protocol=1))['mission']
        self.assertNotIn('station_results', result)
        self.assertNotIn('latest', result['collection'][0])
        self.assertNotIn('peak_people', result['collection'][0])
        self.assertIn('station_results', self.service.dispatch('/missions/status', None)['mission'])


if __name__ == '__main__':
    unittest.main()

"""Synthetic bounded journal/cache fixtures only; no cameras, sockets or robots."""
from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from bridge.task_collection import RouteCollection, collection_page, collection_path
from m20.collection_store import CollectionCache, CollectionJournal, CollectionStorageError, MAX_PAGE_BYTES


RUN = '1'*32
OTHER = '2'*32
ROBOT = 'a'*64
ROBOT_B = 'b'*64


def sample(value=2):
    return dict(kind='people_count', waypoint_index=0, sampled_at=100., values=dict(people=value))


class CollectionStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.store = CollectionJournal(self.root)

    def append(self, value=2, run=RUN, robot=ROBOT):
        return self.store.append(run, sample(value), robot=robot)

    def test_committed_rows_and_cursor_survive_restart(self):
        first = self.store.ensure_run(RUN, robot=ROBOT)
        self.append(2); self.append(3)
        restored = CollectionJournal(self.root)
        restored.append(RUN, sample(4), robot=ROBOT)
        meta = restored.finish(RUN, robot=ROBOT)
        self.assertEqual(first['journal_id'], meta['journal_id'])
        page = restored.page(RUN, cursor=1, robot=ROBOT, page_size=1)
        self.assertEqual([r['seq'] for r in page['rows']], [2])
        self.assertEqual(page['next_cursor'], 2)
        self.assertTrue(page['has_more'])
        page = restored.page(RUN, cursor=2, robot=ROBOT)
        self.assertEqual([r['values']['people'] for r in page['rows']], [4])
        self.assertTrue(page['complete'])
        self.assertFalse(page['has_more'])
        self.assertEqual(restored.page(RUN, cursor=3, robot=ROBOT)['next_cursor'], 3)

    def test_atomic_transaction_recovers_after_abrupt_process_exit(self):
        self.append()
        script = """import os,sqlite3,sys
c=sqlite3.connect(sys.argv[1]);c.execute('BEGIN IMMEDIATE')
c.execute('UPDATE runs SET seq=999')
os._exit(0)
"""
        result = subprocess.run([sys.executable, '-c', script, str(self.store.path)],
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        restored = CollectionJournal(self.root)
        self.assertEqual(restored.metadata(RUN, robot=ROBOT)['high_watermark'], 1)
        self.assertEqual(restored.append(RUN, sample(3), robot=ROBOT)['seq'], 2)

    def test_failed_commit_does_not_advance_sequence_or_claim_success(self):
        self.store.ensure_run(RUN, robot=ROBOT)
        open_db = self.store._db
        class BrokenCommit:
            def __init__(self, db): self.db = db
            def __getattr__(self, name): return getattr(self.db, name)
            def commit(self): raise sqlite3.OperationalError('synthetic disk full')
        @contextmanager
        def broken(*args, **kwargs):
            with open_db(*args, **kwargs) as db:
                yield BrokenCommit(db)
        with patch.object(self.store, '_db', broken):
            with self.assertRaisesRegex(CollectionStorageError, 'synthetic disk full'):
                self.append()
        page = self.store.page(RUN, cursor=0, robot=ROBOT)
        self.assertEqual((page['rows'], page['next_cursor']), ([], 0))
        self.assertFalse(page['complete'])

    def test_quota_keeps_existing_rows_and_reports_truncation(self):
        self.store = CollectionJournal(self.root, limits=dict(run_rows=2))
        self.append(1); self.append(2)
        with self.assertRaisesRegex(CollectionStorageError, '容量已满'):
            self.append(3)
        meta = self.store.finish(RUN, robot=ROBOT)
        self.assertTrue(meta['sealed'])
        self.assertFalse(meta['complete'])
        self.assertTrue(meta['truncated'])
        page = self.store.page(RUN, cursor=0, robot=ROBOT)
        self.assertEqual([r['values']['people'] for r in page['rows']], [1, 2])
        self.assertEqual(page['high_watermark'], 2)
        self.assertIn('容量已满', page['error'])

    def test_run_count_byte_and_global_limits_are_enforced(self):
        for limits in (dict(run_bytes=1), dict(total_bytes=1)):
            with tempfile.TemporaryDirectory() as folder:
                journal = CollectionJournal(folder, limits=limits)
                with self.assertRaises(CollectionStorageError):
                    journal.append(RUN, sample(), robot=ROBOT)
                self.assertEqual(journal.metadata(RUN, robot=ROBOT)['high_watermark'], 0)
        with tempfile.TemporaryDirectory() as folder:
            journal = CollectionJournal(folder, limits=dict(total_rows=1, runs=2))
            journal.append(RUN, sample(), robot=ROBOT)
            with self.assertRaises(CollectionStorageError):
                journal.append(OTHER, sample(), robot=ROBOT)
            with self.assertRaises(CollectionStorageError):
                journal.ensure_run('3'*32, robot=ROBOT)

    def test_robot_task_and_executor_scopes_never_mix(self):
        self.append(2)
        self.append(5, robot=ROBOT_B)
        self.append(8, run=OTHER)
        self.store.append(RUN, sample(9), robot=ROBOT, executor='nos')
        for robot, run, executor, value in ((ROBOT, RUN, 'gos', 2), (ROBOT_B, RUN, 'gos', 5),
                                            (ROBOT, OTHER, 'gos', 8), (ROBOT, RUN, 'nos', 9)):
            page = self.store.page(run, cursor=0, robot=robot, executor=executor)
            self.assertEqual([r['values']['people'] for r in page['rows']], [value])
        with self.assertRaisesRegex(CollectionStorageError, '多个机器人'):
            self.store.page(RUN)
        self.assertFalse(self.store.page(RUN, cursor=0, robot='c'*64)['available'])

    def test_idempotent_event_conflict_and_sealed_append(self):
        row = dict(kind='station_result', index=0, sampled_at=1., people=2)
        first = self.store.append(RUN, row, robot=ROBOT, event_id='station:0')
        self.assertEqual(first, self.store.append(RUN, row, robot=ROBOT, event_id='station:0'))
        with self.assertRaisesRegex(CollectionStorageError, '内容不一致'):
            self.store.append(RUN, dict(row, people=4), robot=ROBOT, event_id='station:0')
        self.store.finish(RUN, robot=ROBOT)
        self.assertEqual(first, self.store.append(RUN, row, robot=ROBOT, event_id='station:0'))
        with self.assertRaisesRegex(CollectionStorageError, '收尾'):
            self.append()

    def test_page_size_bytes_cursor_validation_and_reset_detection(self):
        for number in range(4):
            self.store.append(RUN, dict(kind='synthetic', text='x'*90_000, n=number), robot=ROBOT)
        page = self.store.page(RUN, cursor=0, robot=ROBOT)
        self.assertEqual(len(page['rows']), 2)
        self.assertLess(sum(len(json.dumps(r).encode()) for r in page['rows']), MAX_PAGE_BYTES)
        self.assertTrue(page['has_more'])
        for bad in (-1, True, 2**64, '1'):
            with self.assertRaises(ValueError): self.store.page(RUN, cursor=bad, robot=ROBOT)
        for bad in (0, 101, True):
            with self.assertRaises(ValueError): self.store.page(RUN, page_size=bad)
        self.assertFalse(self.store.page(RUN, cursor=5, robot=ROBOT)['available'])
        self.assertFalse(self.store.page(RUN, cursor=0, robot=ROBOT, journal_id='f'*32)['available'])
        with self.assertRaises(ValueError): self.store.page(RUN, cursor=0)

    def test_missing_new_page_is_not_a_committed_empty_journal(self):
        legacy = collection_page(self.root, RUN, 0)
        self.assertTrue(legacy['ok'])
        self.assertEqual(legacy['rows'], [])
        missing = collection_page(self.root, RUN, cursor=0, robot=ROBOT)
        self.assertFalse(missing['ok'])
        self.assertTrue(missing['unavailable'])
        self.store.ensure_run(RUN, robot=ROBOT)
        empty = collection_page(self.root, RUN, cursor=0, robot=ROBOT)
        self.assertTrue(empty['ok'])
        self.assertFalse(empty['complete'])
        self.assertEqual(empty['next_cursor'], 0)

    def test_legacy_partial_tail_is_read_only_and_never_counted(self):
        path = collection_path(self.root, RUN)
        path.parent.mkdir()
        raw = json.dumps(sample()).encode()+b'\n'+b'{"sampled_at":12'
        path.write_bytes(raw)
        page = collection_page(self.root, RUN, 0)
        self.assertEqual(page['rows'], [sample()])
        self.assertTrue(page['truncated'])
        self.assertFalse(page['complete'])
        self.assertEqual(collection_page(self.root, RUN, 1)['rows'], [])
        self.assertEqual(path.read_bytes(), raw)
        modern = collection_page(self.root, RUN, cursor=0, robot=ROBOT)
        self.assertEqual(modern['collection_sync_protocol'], 0)
        self.assertTrue(modern['unavailable'])
        runtime = RouteCollection(self.root, RUN, 'synthetic', [], lambda:None)
        with self.assertRaisesRegex(ValueError, '只读'): runtime.start()

    def test_corrupt_legacy_line_and_database_do_not_become_empty_success(self):
        path = collection_path(self.root, RUN)
        path.parent.mkdir()
        path.write_bytes(b'not-json\n')
        with self.assertRaises(ValueError): collection_page(self.root, RUN, 0)
        self.store.path.write_bytes(b'synthetic broken sqlite')
        with self.assertRaises(CollectionStorageError):
            self.store.page(RUN, cursor=0, robot=ROBOT)

    def test_runtime_station_and_sampler_share_one_durable_sequence(self):
        spec = dict(kind='people_count', interval_s=1, camera='front')
        runtime = RouteCollection(self.root, RUN, 'synthetic', [spec], lambda:None, robot=ROBOT)
        runtime.start()
        self.addCleanup(runtime.stop)
        runtime._write(spec, dict(index=0), dict(sampled_at=100., values=dict(people=2)))
        station = runtime.record_station(dict(index=0, sampled_at=101., people=3))
        self.assertEqual(station['seq'], 2)
        runtime.record_station(dict(index=0, sampled_at=101., people=3))
        self.assertEqual(runtime.metadata()['high_watermark'], 2)
        self.assertIn('latest', runtime.snapshot()[0])
        self.assertNotIn('latest', runtime.snapshot(compact=True)[0])
        runtime.stop(); runtime.finisher.join(2)
        for worker in runtime.threads: worker.join(1)
        self.assertTrue(runtime.metadata()['complete'])
        self.assertEqual(len(collection_page(self.root, RUN, cursor=0, robot=ROBOT)['rows']), 2)

    def test_finisher_thread_failure_does_not_raise_into_safety_stop(self):
        runtime = RouteCollection(self.root, RUN, 'synthetic', [], lambda:None, robot=ROBOT)
        runtime.start()
        with patch('bridge.task_collection.threading.Thread.start', side_effect=RuntimeError('synthetic thread limit')):
            runtime.stop()
        self.assertTrue(runtime.done.is_set())
        self.assertIsNone(runtime.finisher)
        self.assertFalse(runtime.metadata()['complete'])
        self.assertIn('收尾线程', runtime.metadata()['error'])

    def test_stop_and_metadata_do_not_wait_for_disk_and_failed_flush_is_visible(self):
        runtime = RouteCollection(self.root, RUN, 'synthetic', [], lambda:None, robot=ROBOT)
        runtime.start()
        blocked, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def slow_finish(*args, **kwargs):
            blocked.set(); release.wait(2)
            raise CollectionStorageError('synthetic flush failure')
        with patch.object(runtime.journal, 'finish', side_effect=slow_finish):
            runtime.stop()
            self.assertTrue(blocked.wait(1))
            self.assertFalse(runtime.metadata()['complete'])
            self.assertIsNotNone(runtime.snapshot())
            release.set(); runtime.finisher.join(2)
        self.assertFalse(runtime.metadata()['complete'])
        self.assertIn('flush failure', runtime.metadata()['error'])


class CollectionCacheTests(unittest.TestCase):
    append = CollectionStoreTests.append
    def setUp(self):
        CollectionStoreTests.setUp(self)
        self.cache = CollectionCache(self.root)

    def test_duplicate_pages_are_idempotent_and_missing_pages_are_unavailable(self):
        for value in (1, 2, 3): self.append(value)
        self.store.finish(RUN, robot=ROBOT)
        self.cache.schedule(RUN, robot=ROBOT, final=True)
        first = self.store.page(RUN, cursor=0, robot=ROBOT, page_size=2)
        state = self.cache.ingest(RUN, first, robot=ROBOT)
        self.assertEqual(state['cursor'], 2)
        self.assertTrue(state['pending'])
        gap = self.cache.page(RUN, 2, robot=ROBOT)
        self.assertTrue(gap['unavailable'])
        self.assertFalse(gap['complete'])
        last = self.store.page(RUN, cursor=2, robot=ROBOT)
        state = self.cache.ingest(RUN, last, robot=ROBOT, requested_cursor=2)
        self.assertTrue(state['complete'])
        repeated = self.cache.ingest(RUN, first, robot=ROBOT)
        self.assertEqual(repeated['cursor'], 3)
        self.assertEqual(len(self.cache.page(RUN, robot=ROBOT)['rows']), 3)
        self.assertTrue(CollectionCache(self.root).state(RUN, robot=ROBOT)['complete'])

    def test_replayed_unsealed_page_cannot_regress_a_known_final_journal(self):
        for value in (1,2): self.append(value)
        old = self.store.page(RUN, cursor=1, robot=ROBOT)
        self.store.finish(RUN, robot=ROBOT)
        self.cache.schedule(RUN, robot=ROBOT, final=True)
        first = self.store.page(RUN, cursor=0, robot=ROBOT, page_size=1)
        self.cache.ingest(RUN, first, robot=ROBOT)
        state = self.cache.ingest(RUN, old, robot=ROBOT, requested_cursor=1)
        self.assertTrue(state['complete'])
        self.assertFalse(state['pending'])
        self.assertTrue(self.cache.metadata(RUN, robot=ROBOT)['sealed'])

    def test_scope_mismatch_and_changed_duplicate_cannot_modify_cache(self):
        self.append()
        self.cache.schedule(RUN, robot=ROBOT)
        first = self.store.page(RUN, cursor=0, robot=ROBOT)
        self.cache.ingest(RUN, first, robot=ROBOT)
        for key, value in (('run_id', OTHER), ('robot', ROBOT_B), ('journal_id', 'f'*32)):
            with self.assertRaises(ValueError):
                self.cache.ingest(RUN, dict(first, **{key:value}), robot=ROBOT)
        changed = copy.deepcopy(first)
        changed['rows'][0]['values']['people'] = 999
        with self.assertRaisesRegex(CollectionStorageError, '内容不一致'):
            self.cache.ingest(RUN, changed, robot=ROBOT)
        self.assertEqual(self.cache.state(RUN, robot=ROBOT)['cursor'], 1)
        self.assertEqual(self.cache.page(RUN, robot=ROBOT)['rows'][0]['values']['people'], 2)

    def test_quota_or_invalid_page_does_not_advance_cache_cursor(self):
        for value in (1, 2): self.append(value)
        cache = CollectionCache(self.root, limits=dict(run_rows=1))
        cache.schedule(RUN, robot=ROBOT)
        page = self.store.page(RUN, cursor=0, robot=ROBOT)
        with self.assertRaisesRegex(CollectionStorageError, '容量已满'):
            cache.ingest(RUN, page, robot=ROBOT)
        self.assertEqual(cache.state(RUN, robot=ROBOT)['cursor'], 0)
        self.assertFalse(cache.metadata(RUN, robot=ROBOT)['available'])
        bad = copy.deepcopy(page); bad['rows'][1]['seq'] = 4
        with self.assertRaises(CollectionStorageError): cache.ingest(RUN, bad, robot=ROBOT)
        self.assertEqual(cache.state(RUN, robot=ROBOT)['cursor'], 0)

    def test_cache_holes_and_source_rewind_are_never_skipped(self):
        for value in (1, 2): self.append(value)
        self.cache.schedule(RUN, robot=ROBOT)
        self.cache.ingest(RUN, self.store.page(RUN, cursor=0, robot=ROBOT), robot=ROBOT)
        db=sqlite3.connect(self.cache.path)
        try:
            db.execute('DELETE FROM records WHERE seq=1')
            db.commit()
        finally:
            db.close()  # Windows keeps the file locked while the handle is open.
        with self.assertRaisesRegex(CollectionStorageError, '缺页'):
            self.cache.page(RUN, robot=ROBOT)
        backwards = self.store.page(RUN, cursor=2, robot=ROBOT)
        backwards.update(cursor=0, high_watermark=0, next_cursor=0)
        with self.assertRaisesRegex(CollectionStorageError, '回退'):
            self.cache.ingest(RUN, backwards, robot=ROBOT)


if __name__ == '__main__':
    unittest.main()

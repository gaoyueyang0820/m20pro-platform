"""Isolated sampling and a bounded durable journal; legacy JSONL stays readable."""
import contextlib
import copy
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time

from m20.collection_store import (CollectionJournal, CollectionStorageError, DEFAULT_LIMITS,
                                  MAX_ROW_BYTES, SYNC_PROTOCOL, page_arguments, unavailable)
from m20.map_catalog import read_json
from m20.navigation_disconnect import identity
from m20.patrol_actions import StationActions
from m20.task_collectors import COLLECTORS, collector_specs


ROOT = Path(__file__).resolve().parents[1]
MAX_PACKET = 128_000


class CollectionCancelled(Exception):
    pass


class SamplerProcess:
    def __init__(self, data, aos, spec):
        env = dict(os.environ, M20_COLLECTOR_DATA=str(data), M20_COLLECTOR_AOS=aos,
                   OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
        env.pop('M20_BRIDGE_TOKEN', None)
        self.process = subprocess.Popen([sys.executable, '-m', 'bridge.task_collection', '--worker'],
                                        cwd=ROOT, env=env, start_new_session=True, bufsize=0,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.buffer = b''
        try:
            self.process.stdin.write(json.dumps(spec).encode() + b'\n')
        except OSError:
            self.close()
            raise

    def sample(self, cancelled):
        self.process.stdin.write(b'sample\n')
        deadline = time.monotonic() + 30.
        stream = self.process.stdout
        while time.monotonic() < deadline:
            if cancelled():
                raise CollectionCancelled
            line, sep, remaining = self.buffer.partition(b'\n')
            if sep:
                self.buffer = remaining
                return json.loads(line)
            if select.select([stream], [], [], .1)[0]:
                chunk = os.read(stream.fileno(), 4096)
                if not chunk:
                    raise ValueError('采集进程已退出')
                self.buffer += chunk
                if len(self.buffer) > MAX_PACKET:
                    raise ValueError('采集记录超出容量')
        raise ValueError('采集超时')

    def close(self):
        proc = self.process
        # FFmpeg belongs to the same private process group as its sampler.
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=.5)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=1)
        finally:
            proc.stdin.close()
            proc.stdout.close()


def collection_path(data, run_id):
    return Path(data)/'task-collections'/(identity(run_id)+'.jsonl')


def collection_page(data, run_id, offset=0, page_size=100, *, cursor=None, robot=None,
                    executor='gos', journal_id=None):
    """Versioned cursor pages, or bounded read-only legacy offset pages.

    Old callers keep their empty-page behavior for a nonexistent run. Cursor
    callers must distinguish unavailable data from a committed empty journal.
    """
    page_arguments(offset, page_size, cursor)
    path = collection_path(data, run_id)
    journal = CollectionJournal(data)
    page = journal.page(run_id, offset, page_size, cursor=cursor, robot=robot,
                        executor=executor, journal_id=journal_id)
    if page.get('available') or page.get('error') != '采集日志尚不存在':
        return page
    if path.parent.is_symlink() or path.is_symlink():
        raise ValueError('采集记录路径异常')
    if cursor is not None:
        unbound = journal.metadata(run_id, executor=executor)
        if path.is_file() or unbound.get('available'):
            return unavailable(run_id, '旧采集记录仅支持 offset 读取，不支持增量同步',
                               robot=robot, executor=executor, protocol=0)
        return page
    if not path.is_file():
        return dict(ok=True, run_id=run_id, rows=[], next_offset=None,
                    collection_sync_protocol=0, available=False, unavailable=True,
                    complete=False, stale=True, live=False, source='legacy_jsonl')
    if robot is not None:
        return unavailable(run_id, '旧采集记录没有机器人身份，不能冒充增量缓存',
                           robot=robot, executor=executor, protocol=0)
    rows, truncated, read_bytes = [], False, 0
    if offset > DEFAULT_LIMITS['run_rows']:
        raise ValueError('旧采集页偏移超出安全读取预算')
    with path.open('rb') as handle:
        for index in range(offset+page_size+1):
            line = handle.readline(MAX_ROW_BYTES+1)
            read_bytes += len(line)
            if read_bytes > DEFAULT_LIMITS['run_bytes'] or len(line) > MAX_ROW_BYTES:
                raise ValueError('旧采集记录超出安全读取预算')
            if not line:
                break
            if not line.endswith(b'\n'):
                truncated = True
                break  # An interrupted old append is not a committed sample.
            if index >= offset:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('旧采集记录格式无效')
                rows.append(row)
    more = len(rows) > page_size
    return dict(ok=True, run_id=run_id, rows=rows[:page_size],
                next_offset=offset+page_size if more else None,
                collection_sync_protocol=0, available=True, unavailable=False,
                complete=False, truncated=truncated, stale=True, live=False,
                error='旧采集日志末行未完整提交' if truncated else None,
                source='legacy_jsonl')


class RouteCollection:
    def __init__(self, data, run_id, aos, specs, context, worker_factory=SamplerProcess, *,
                 robot=None, executor='gos', journal=None):
        self.data, self.aos = Path(data), aos
        self.path = collection_path(data, run_id)  # Legacy path, never overwritten.
        self.run_id, self.robot, self.executor = run_id, robot, executor
        self.journal = journal or CollectionJournal(data)
        self.specs, self.context, self.worker_factory = specs, context, worker_factory
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.journal_lock = threading.Lock()
        self.status = {s['kind']: dict(spec=s, samples=0, failures=0, state='waiting') for s in specs}
        self.threads = []
        self.finisher = None
        self.storage_error = None
        self._meta = dict(collection_sync_protocol=SYNC_PROTOCOL, run_id=run_id, robot=robot,
                          executor=executor, available=False, complete=False, sealed=False,
                          high_watermark=0, truncated=False, error=None)

    def snapshot(self, compact=False):
        with self.lock:
            return [copy.deepcopy({k:v for k,v in row.items()
                                   if not compact or k not in ('latest', 'peak_people')})
                    for row in self.status.values()]

    def metadata(self):
        # Used by safety heartbeats: no file/SQLite access or sampler lock here.
        with self.lock:
            meta = dict(self._meta)
        if self.robot is None:
            meta['collection_sync_protocol'] = 0  # Old requests did not bind a robot identity.
        return meta

    def start(self):
        if self.path.exists() and not self.journal.metadata(self.run_id, robot=self.robot,
                                                           executor=self.executor).get('available'):
            raise ValueError('旧 JSONL 采集日志只读，不能作为新日志继续追加')
        meta = self.journal.ensure_run(self.run_id, robot=self.robot, executor=self.executor)
        if meta['sealed'] or meta.get('error'):
            raise ValueError(meta.get('error') or '采集日志已结束，不能重新启动采集')
        with self.lock:
            self._meta = meta
        for spec in self.specs:
            worker = threading.Thread(target=self._run, args=(spec,), daemon=True,
                                      name='route-collector-'+spec['kind'])
            worker.start()
            self.threads.append(worker)

    def stop(self, error=None):
        if error:
            self._failed(error)
        self.done.set()
        with self.lock:
            for row in self.status.values():
                if row['state'] != 'error':
                    row['state'] = 'ended'
            if self.finisher is not None:
                return
            worker = threading.Thread(target=self._finish, daemon=True, name='route-collection-finish')
            try:
                worker.start()
            except (RuntimeError, OSError) as error:
                # Storage cleanup must never prevent the caller's motion stop.
                self.storage_error = '采集收尾线程无法启动：'+str(error)[:160]
                self._meta.update(complete=False, truncated=True, error=self.storage_error)
                for row in self.status.values():
                    row.update(state='error', error=self.storage_error)
                return
            self.finisher = worker

    def _finish(self):
        try:
            # stop() never waits for a slow sampler, a disk flush, or SQLite.
            # _write checks done under this lock; no record can arrive after seal.
            with self.journal_lock:
                meta = self.journal.finish(self.run_id, robot=self.robot, executor=self.executor,
                                           error=self.storage_error)
            with self.lock:
                self._meta = meta
        except (OSError, ValueError) as error:
            self._failed(error)

    def _failed(self, error):
        with self.lock:
            self.storage_error = str(error) or '采集记录无法写入'
            self._meta.update(error=self.storage_error, complete=False, truncated=True)
            for row in self.status.values():
                row.update(state='error', error=self.storage_error)

    def _append(self, row, event_id=None):
        result = self.journal.append(self.run_id, row, robot=self.robot,
                                     executor=self.executor, event_id=event_id)
        with self.lock:
            self._meta.update(available=True, high_watermark=max(self._meta['high_watermark'], result['seq']))
        return result

    def record_station(self, row):
        if not isinstance(row, dict) or type(row.get('index')) is not int or row['index'] < 0:
            raise ValueError('到点采集记录索引无效')
        try:
            with self.journal_lock:
                if self.done.is_set():
                    raise CollectionStorageError('采集已结束，未保存到点读数')
                return self._append(dict(row, kind='station_result', waypoint_index=row['index']),
                                    event_id='station:'+str(row['index']))
        except (OSError, ValueError) as error:
            self._failed(error)
            raise

    def _write(self, spec, context, result):
        row = dict(kind=spec['kind'], waypoint_index=context['index'], **result)
        with self.journal_lock:
            if self.done.is_set():
                return
            row = self._append(row)
            with self.lock:
                state = self.status[spec['kind']]
                if row.get('error'):
                    state.update(failures=state['failures']+1, error=row['error'])
                    if row.get('devices'):
                        state['latest'] = row
                else:
                    state.update(samples=state['samples']+1, latest=row, error=None)
                    if spec['kind'] == 'people_count':
                        state['peak_people'] = max(state.get('peak_people', 0), row['values']['people'])

    def _run(self, spec):
        sampler = None
        due = 0.
        try:
            while not self.done.is_set():
                context = self.context()
                if context is None:
                    if sampler is not None:
                        sampler.close()
                        sampler = None
                    with self.lock:
                        if not self.done.is_set():
                            self.status[spec['kind']]['state'] = 'waiting'
                    self.done.wait(.1)
                    continue
                delay = due - time.monotonic()
                if delay > 0:
                    self.done.wait(min(.1, delay))
                    continue
                with self.lock:
                    if self.done.is_set():
                        return
                    self.status[spec['kind']]['state'] = 'collecting'
                try:
                    if sampler is None:
                        sampler = self.worker_factory(self.data, self.aos, spec)
                    result = sampler.sample(lambda: self.done.is_set() or self.context() is None)
                except CollectionCancelled:
                    sampler.close()
                    sampler = None
                    continue
                except (OSError, ValueError):
                    if sampler is not None:
                        sampler.close()
                        sampler = None
                    result = dict(sampled_at=time.time(), error='采集暂不可用，稍后重试')
                if self.context() is not None:
                    self._write(spec, context, result)
                due = time.monotonic() + spec['interval_s']
        except (OSError, ValueError) as error:
            self._failed(error)
            self.stop()
        finally:
            if sampler is not None:
                sampler.close()


def run_worker(stream, output, source, spec):
    sampler = None
    try:
        for line in stream:
            if line.strip() != 'sample':
                return
            try:
                if sampler is None:
                    sampler = COLLECTORS[spec['kind']].factory(source, spec)
                result = sampler.sample()
            except ValueError as error:
                result = dict(sampled_at=time.time(), error=str(error)[:200])
            except Exception:
                result = dict(sampled_at=time.time(), error='采集依赖或设备暂不可用')
            print(json.dumps(result, ensure_ascii=False), file=output, flush=True)
    finally:
        if sampler is not None:
            sampler.close()


if __name__ == '__main__' and sys.argv[1:] == ['--worker']:
    os.nice(10)
    # Reserve stdout for packets, including when native libraries log to fd 1.
    with os.fdopen(os.dup(sys.stdout.fileno()), 'w', encoding='utf-8') as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        data = Path(os.environ['M20_COLLECTOR_DATA'])
        spec, = collector_specs([json.loads(sys.stdin.readline())])
        source = StationActions(os.environ['M20_COLLECTOR_AOS'], read_json(data/'gos-patrol.local.json', {}))
        run_worker(sys.stdin, output, source, spec)

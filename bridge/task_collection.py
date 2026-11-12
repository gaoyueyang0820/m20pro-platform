"""One isolated sampler process per collection type and a per-run JSONL log."""
import contextlib
import itertools
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time

from m20.map_catalog import read_json
from m20.navigation_disconnect import identity
from m20.patrol_actions import StationActions
from m20.task_collectors import COLLECTORS, collector_specs


ROOT = Path(__file__).resolve().parents[1]
MAX_PACKET = 32_768


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


def collection_page(data, run_id, offset, page_size=100):
    if type(offset) is not int or offset < 0:
        raise ValueError('采集记录页码无效')
    path = collection_path(data, run_id)
    if not path.is_file():
        return dict(ok=True, run_id=run_id, rows=[], next_offset=None)
    if path.is_symlink():
        raise ValueError('采集记录路径异常')
    with path.open('rb') as handle:
        # An active writer may be committing the last line.
        lines = [line for line in itertools.islice(handle, offset, offset+page_size+1) if line.endswith(b'\n')]
    rows = [json.loads(line) for line in lines[:page_size]]
    return dict(ok=True, run_id=run_id, rows=rows,
                next_offset=offset+len(rows) if len(lines) > page_size else None)


class RouteCollection:
    def __init__(self, data, run_id, aos, specs, context, worker_factory=SamplerProcess):
        self.data, self.aos = Path(data), aos
        self.path = collection_path(data, run_id)
        self.specs, self.context, self.worker_factory = specs, context, worker_factory
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.journal_lock = threading.Lock()
        self.status = {s['kind']: dict(spec=s, samples=0, failures=0, state='waiting') for s in specs}
        self.threads = []

    def snapshot(self):
        with self.lock:
            return [dict(row) for row in self.status.values()]

    def start(self):
        for spec in self.specs:
            worker = threading.Thread(target=self._run, args=(spec,), daemon=True,
                                      name='route-collector-'+spec['kind'])
            worker.start()
            self.threads.append(worker)

    def stop(self):
        self.done.set()
        with self.lock:
            for row in self.status.values():
                row['state'] = 'ended'

    def _write(self, spec, context, result):
        row = dict(kind=spec['kind'], waypoint_index=context['index'], **result)
        payload = json.dumps(row, ensure_ascii=False).encode() + b'\n'
        if len(payload) > MAX_PACKET:
            raise ValueError('采集记录超出容量')
        with self.journal_lock:
            if self.done.is_set():
                return
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            if self.path.is_symlink():
                raise ValueError('采集记录路径异常')
            with self.path.open('ab') as handle:
                handle.write(payload)
        with self.lock:
            state = self.status[spec['kind']]
            if row.get('error'):
                state.update(failures=state['failures']+1, error=row['error'])
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
        except (OSError, ValueError):
            with self.lock:
                self.status[spec['kind']].update(state='error', error='采集记录无法写入')
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

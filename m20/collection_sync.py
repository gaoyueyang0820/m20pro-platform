"""Event-driven, budgeted journal mirroring, separate from motion heartbeats.

Only explicit boundary/reconnect/download notifications create transfer intents.
An ordinary navigation update must not notify on every changed sample watermark.
Pending intents survive restart and keep working after a mission is terminal.
The immutable (robot, executor, run_id) scope is checked around every RPC; no
application/control lock is held over transport or database operations.
"""
import copy
import threading
import time

from m20.collection_store import CollectionCache, CollectionStorageError, SYNC_PROTOCOL, scope


class _IdentityChanged(ValueError):
    pass


class CollectionSynchronizer:
    def __init__(self, data, current_robot, request, *, on_update=None, cache=None,
                 autostart=True, page_size=64, max_pages=3, budget_s=2., clock=time.monotonic):
        self.cache = cache or CollectionCache(data)
        self.current_robot, self.request, self.on_update = current_robot, request, on_update
        self.autostart, self.page_size, self.max_pages, self.budget_s = autostart, page_size, max_pages, budget_s
        self.clock = clock
        self.done, self.wake = threading.Event(), threading.Event()
        self.lock = threading.Lock()
        self.worker = None
        self.incoming, self.states, self.tasks = {}, {}, {}
        self.loaded = False
        self.last_robot = None
        self.error = None

    @staticmethod
    def _key(run):
        return scope(run.get('id') or run.get('run_id'), run.get('robot'), run.get('executor', 'gos'))

    def notify(self, run, *, final=False, target=None):
        """Nonblocking, coalesced notification; safe in a heartbeat observer."""
        key = self._key(run)
        if not key[0] or key[1] not in ('gos', 'nos') or self.done.is_set():
            return
        with self.lock:
            old = self.incoming.get(key, {})
            if old.get('target') is not None and target is not None:
                target = max(old['target'], target)
            self.incoming[key] = dict(run=dict(id=key[2], robot=key[0], executor=key[1]),
                                      final=bool(final or old.get('final')), target=target)
        self.wake.set()
        if self.autostart:
            self.start()

    def start(self):
        with self.lock:
            if not self.done.is_set() and (self.worker is None or not self.worker.is_alive()):
                worker = threading.Thread(target=self._run, daemon=True, name='patrol-cache-sync')
                try:
                    worker.start()
                except (RuntimeError, OSError) as error:
                    self.error = '采集补传线程无法启动：'+str(error)[:160]
                    return  # Cache failure cannot terminate a navigation heartbeat.
                self.worker = worker

    def snapshot(self, run):
        key = self._key(run)
        with self.lock:
            row = copy.deepcopy(self.states.get(key))
            pending = key in self.incoming
        if row is None:
            row = dict(protocol=SYNC_PROTOCOL, supported=None, run_id=key[2], robot=key[0], executor=key[1],
                       state='pending', pending=True, cursor=0, high_watermark=0, cached_rows=0,
                       complete=False, last_synced_at=None, stale=True, live=False, error=self.error)
        if pending:
            row.update(pending=True, state='pending')
        if self.current_robot() != key[0]:
            row.update(state='offline', stale=True)
        return row

    def snapshots(self):
        with self.lock:
            rows = [copy.deepcopy(row) for row in self.states.values()]
        robot = self.current_robot()
        for row in rows:
            if row['robot'] != robot:
                row.update(state='offline', stale=True)
        return rows

    def _publish(self, key, state, rows=()):
        with self.lock:
            self.states[key] = copy.deepcopy(state)
        if self.on_update is not None and self.current_robot() == key[0] and not self.done.is_set():
            self.on_update(dict(id=key[2], robot=key[0], executor=key[1]), copy.deepcopy(state), rows)

    def _load(self):
        if self.loaded:
            return
        for state in self.cache.states():
            key = self._key(state)
            self._publish(key, state)
            self.tasks[key] = dict(run=dict(id=key[2], robot=key[0], executor=key[1]), due=0., failures=0)
        self.loaded = True

    def _drain(self):
        # Large histories do not make an unbounded burst of disk or RPC work.
        with self.lock:
            keys = list(self.incoming)[:4]
            items = [(key, self.incoming.pop(key)) for key in keys]
        for key, item in items:
            try:
                state = self.cache.schedule(key[2], robot=key[0], executor=key[1],
                                            final=item['final'], target=item['target'])
                self.tasks[key] = dict(run=item['run'], due=0., failures=0)
                self._publish(key, state)
            except (OSError, ValueError) as error:
                # Keep the intent in memory too if the disk could not accept it.
                self.error = str(error)
                with self.lock:
                    self.incoming.setdefault(key, item)
                    previous = self.states.get(key, dict(protocol=SYNC_PROTOCOL, supported=None,
                        run_id=key[2], robot=key[0], executor=key[1], cursor=0,
                        high_watermark=0, cached_rows=0, complete=False, stale=True, live=False,
                        last_synced_at=None))
                    self.states[key] = dict(previous, state='error', pending=True, error=str(error))

    def _guard(self, key):
        if self.done.is_set() or self.current_robot() != key[0]:
            raise _IdentityChanged('采集连接身份已变化，本次响应未用于当前机器人')

    def _rpc(self, key, task, path, body, timeout):
        self._guard(key)
        result = self.request(task['run'], path, body, timeout)
        self._guard(key)
        if path == '/missions/collection' and isinstance(result, dict) and isinstance(result.get('page'), dict):
            return result['page']  # Preserve typed unavailable/legacy replies through generic RPC.
        return result

    def _failure(self, key, error, *, state='error', supported=None, retry=True):
        try:
            row = self.cache.failure(key[2], str(error), robot=key[0], executor=key[1],
                                     state=state, supported=supported, retry=retry)
        except (OSError, ValueError) as storage_error:
            with self.lock:
                row = copy.deepcopy(self.states.get(key, {}))
            row.update(state='error', error=str(storage_error), pending=True, complete=False,
                       run_id=key[2], robot=key[0], executor=key[1])
        self._publish(key, row)
        return row

    def _batch(self, key, task):
        deadline = self.clock()+self.budget_s
        with self.lock:
            meta = copy.deepcopy(self.states[key])
        if meta.get('supported') is not True:
            caps = self._rpc(key, task, '/missions/capabilities', None, min(1., self.budget_s))
            if not isinstance(caps, dict) or caps.get('collection_sync_protocol') != SYNC_PROTOCOL:
                self._failure(key, '当前代理仅支持旧采集协议，自动增量补传不可用；可手动读取旧记录',
                              state='unsupported', supported=False, retry=False)
                return
        for _ in range(self.max_pages):
            remaining = deadline-self.clock()
            if remaining <= 0 or self.done.is_set():
                break
            self._guard(key)
            with self.lock:
                meta = copy.deepcopy(self.states[key])
                self.states[key]['state'] = 'syncing'
            cursor = meta.get('cursor', 0)
            request = dict(id=key[2], robot=key[0], cursor=cursor, limit=self.page_size,
                           collection_sync_protocol=SYNC_PROTOCOL, response_envelope=True)
            if meta.get('journal_id'):
                request['journal_id'] = meta['journal_id']
            page = self._rpc(key, task, '/missions/collection', request, min(1., remaining))
            if isinstance(page, dict) and page.get('collection_sync_protocol') != SYNC_PROTOCOL:
                self._failure(key, '代理未确认采集增量协议，未推进本机游标',
                              state='unsupported', supported=False, retry=False)
                return
            if not isinstance(page, dict) or page.get('available') is not True or page.get('ok') is not True:
                message = page.get('error') or page.get('err') if isinstance(page, dict) else None
                self._failure(key, message or '机载采集页暂不可用，未推进本机游标', state='unavailable')
                task['failures'] += 1
                return
            self._guard(key)
            state = self.cache.ingest(key[2], page, robot=key[0], executor=key[1], requested_cursor=cursor)
            # A switch during fsync can only commit to the immutable old-robot
            # namespace. It must not update the new robot's navigation/UI.
            self._publish(key, state, page['rows'])
            task['failures'] = 0
            if not state['pending']:
                return
            if state['cursor'] >= state['high_watermark']:
                break  # Await final seal; no tight loop of empty pages.

    def run_once(self):
        """One bounded scheduler turn; also the deterministic offline test seam."""
        if self.done.is_set():
            return
        try:
            self._load()
            self._drain()
        except (OSError, ValueError) as error:
            self.error = str(error)
            return
        robot = self.current_robot()
        if robot != self.last_robot:
            for key, task in self.tasks.items():
                if key[0] == robot:
                    task['due'] = 0.
            self.last_robot = robot
        if not robot:
            return
        now = self.clock()
        with self.lock:
            candidates = [(key, task) for key, task in self.tasks.items()
                          if key[0] == robot and task['due'] <= now and self.states.get(key, {}).get('pending')]
        if not candidates:
            return
        key, task = min(candidates, key=lambda pair:pair[1]['due'])
        try:
            self._batch(key, task)
        except _IdentityChanged:
            # The persisted intent remains pending for its own robot.
            with self.lock:
                self.states[key].update(state='offline', pending=True)
        except (OSError, ValueError) as error:
            task['failures'] += 1
            self._failure(key, error, state='offline' if not isinstance(error, CollectionStorageError) else 'error')
        task['due'] = self.clock() + (min(60., 2.**min(task['failures'], 6)) if task['failures'] else 1.)

    def _run(self):
        while not self.done.is_set():
            self.run_once()
            self.wake.wait(1.)
            self.wake.clear()

    def close(self):
        self.done.set()
        self.wake.set()
        if self.worker is not None and self.worker is not threading.current_thread():
            self.worker.join(timeout=.2)

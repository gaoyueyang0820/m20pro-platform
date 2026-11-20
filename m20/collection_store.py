"""Bounded, transactional patrol journals and desktop mirrors (standard library).

A cursor is the last durably committed sequence, not a byte offset or a timestamp.
SQLite's FULL synchronous rollback journal makes records and cursors one commit;
failed writes never advance the public watermark. Quotas reject new measurements,
not silently evict untransferred ones. No hardware, network or background threads.
Legacy JSONL is read separately by bridge.task_collection, never imported wholesale.
"""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import time
import uuid

from m20.navigation_disconnect import identity

SYNC_PROTOCOL = 1
MAX_ROW_BYTES = 128_000
MAX_PAGE_BYTES = 256_000
MAX_PAGE_ROWS = 100
DEFAULT_LIMITS = dict(run_rows=20_000, run_bytes=32*1024*1024,
                      total_rows=100_000, total_bytes=128*1024*1024,
                      runs=128, database_bytes=256*1024*1024)


class CollectionStorageError(ValueError):
    """A journal/cache operation failed; its result must not be acknowledged."""


def robot_key(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-f0-9]{64}', value):
        raise ValueError('采集机器人身份无效')
    return value


def robot_digest(value):
    if not isinstance(value, str) or not value:
        raise ValueError('尚未核对采集机器人身份')
    return hashlib.sha256(value.encode()).hexdigest()


def scope(run_id, robot=None, executor='gos'):
    if executor not in ('gos', 'nos', 'local'):
        raise ValueError('采集执行器无效')
    return (robot_key(robot) if robot is not None else '', executor, identity(run_id))


def encoded(value):
    try:
        return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (TypeError, ValueError, RecursionError) as error:
        raise ValueError('采集记录格式无效') from error


def page_arguments(offset, page_size, cursor=None):
    if type(offset) is not int or offset < 0 or offset > 2**63-1:
        raise ValueError('采集记录页码无效')
    if type(page_size) is not int or not 1 <= page_size <= MAX_PAGE_ROWS:
        raise ValueError('采集分页容量无效')
    if cursor is not None and (type(cursor) is not int or not 0 <= cursor <= 2**63-1):
        raise ValueError('采集增量游标无效')


def unavailable(run_id, message, *, robot=None, executor='gos', protocol=SYNC_PROTOCOL):
    return dict(ok=False, collection_sync_protocol=protocol, run_id=run_id, robot=robot,
                executor=executor, rows=[], next_offset=None, next_cursor=None,
                available=False, unavailable=True, stale=True, live=False,
                complete=False, has_more=False, error=message, err=message)


SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    robot TEXT NOT NULL, executor TEXT NOT NULL, run_id TEXT NOT NULL,
    journal_id TEXT NOT NULL, seq INTEGER NOT NULL DEFAULT 0,
    bytes INTEGER NOT NULL DEFAULT 0, sealed INTEGER NOT NULL DEFAULT 0,
    error TEXT NOT NULL DEFAULT '', truncated INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (robot, executor, run_id)
);
CREATE TABLE IF NOT EXISTS records (
    robot TEXT NOT NULL, executor TEXT NOT NULL, run_id TEXT NOT NULL,
    seq INTEGER NOT NULL, event_id TEXT, payload TEXT NOT NULL,
    PRIMARY KEY (robot, executor, run_id, seq),
    UNIQUE (robot, executor, run_id, event_id),
    FOREIGN KEY (robot, executor, run_id) REFERENCES runs(robot, executor, run_id)
);
CREATE TABLE IF NOT EXISTS transfers (
    robot TEXT NOT NULL, executor TEXT NOT NULL, run_id TEXT NOT NULL,
    metadata TEXT NOT NULL, PRIMARY KEY (robot, executor, run_id)
);
"""


class CollectionJournal:
    def __init__(self, data, *, directory='task-collections', limits=None):
        self.root = Path(data)/directory
        self.path = self.root/'journal.sqlite3'
        self.limits = dict(DEFAULT_LIMITS, **(limits or {}))
        if any(type(v) is not int or v <= 0 for v in self.limits.values()):
            raise ValueError('采集容量配置无效')

    @contextmanager
    def _db(self, create=False):
        db = None
        try:
            # Private directories; refuse links including SQLite sidecars.
            if self.root.is_symlink() or any(p.is_symlink() for p in
                    (self.path, Path(str(self.path)+'-journal'), Path(str(self.path)+'-wal'), Path(str(self.path)+'-shm'))):
                raise CollectionStorageError('采集记录路径异常')
            if create:
                created_directory = not self.root.exists()
                self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
                try:
                    fd = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
                except FileExistsError:
                    pass
                else:
                    os.close(fd)
                    if os.name != 'nt':
                        # SQLite syncs record transactions; also persist the new
                        # database/directory entries before acknowledging them.
                        for folder in (self.root, self.root.parent) if created_directory else (self.root,):
                            directory_fd = os.open(folder, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
                            try:
                                os.fsync(directory_fd)
                            finally:
                                os.close(directory_fd)
            if not self.path.exists():
                yield None
                return
            if not self.path.is_file():
                raise CollectionStorageError('采集记录路径异常')
            db = sqlite3.connect(self.path.resolve().as_uri()+'?mode=rw', uri=True, timeout=.25)
            db.row_factory = sqlite3.Row
            db.execute('PRAGMA foreign_keys=ON')
            db.execute('PRAGMA synchronous=FULL')
            version = db.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1):
                raise CollectionStorageError('采集数据库版本不兼容')
            if version == 0:
                if not create:
                    raise CollectionStorageError('采集数据库尚未完整初始化')
                db.execute('PRAGMA journal_mode=DELETE')
                db.executescript('BEGIN IMMEDIATE;'+SCHEMA+'PRAGMA user_version=1;COMMIT;')
            pages = max(32, self.limits['database_bytes']//db.execute('PRAGMA page_size').fetchone()[0])
            db.execute('PRAGMA max_page_count=%d' % pages)
            yield db
        except (OSError, sqlite3.Error) as error:
            raise CollectionStorageError('采集记录无法持久化或读取：'+str(error)[:160]) from error
        finally:
            if db is not None:
                db.close()  # An uncommitted transaction is rolled back, never acknowledged.

    @staticmethod
    def _run(db, key):
        return db.execute('SELECT * FROM runs WHERE robot=? AND executor=? AND run_id=?', key).fetchone()

    def _ensure(self, db, key, journal_id=None):
        row = self._run(db, key)
        if row is not None:
            if journal_id is not None and row['journal_id'] != journal_id:
                raise CollectionStorageError('采集日志身份已变化，不能合并为原任务')
            return row
        if db.execute('SELECT count(*) FROM runs').fetchone()[0] >= self.limits['runs']:
            raise CollectionStorageError('采集任务缓存容量已满，未丢弃已有记录')
        db.execute('INSERT INTO runs(robot,executor,run_id,journal_id) VALUES(?,?,?,?)',
                   (*key, journal_id or uuid.uuid4().hex))
        return self._run(db, key)

    @staticmethod
    def _metadata(row):
        if row is None:
            return dict(collection_sync_protocol=SYNC_PROTOCOL, available=False, complete=False,
                        sealed=False, high_watermark=0, error='采集日志尚不存在')
        return dict(collection_sync_protocol=SYNC_PROTOCOL if row['robot'] else 0, available=True, run_id=row['run_id'],
                    robot=row['robot'] or None, executor=row['executor'], journal_id=row['journal_id'],
                    high_watermark=row['seq'], rows=row['seq'], bytes=row['bytes'],
                    sealed=bool(row['sealed']), complete=bool(row['sealed']) and not row['error'],
                    truncated=bool(row['truncated']), error=row['error'] or None)

    def ensure_run(self, run_id, *, robot=None, executor='gos'):
        key = scope(run_id, robot, executor)
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            result = self._metadata(self._ensure(db, key))
            db.commit()
        return result

    def metadata(self, run_id, *, robot=None, executor='gos'):
        key = scope(run_id, robot, executor)
        with self._db() as db:
            return self._metadata(self._run(db, key) if db is not None else None)

    def _capacity(self, db, row, extra_rows, extra_bytes):
        total = db.execute('SELECT coalesce(sum(seq),0),coalesce(sum(bytes),0) FROM runs').fetchone()
        return (row['seq']+extra_rows <= self.limits['run_rows']
                and row['bytes']+extra_bytes <= self.limits['run_bytes']
                and total[0]+extra_rows <= self.limits['total_rows']
                and total[1]+extra_bytes <= self.limits['total_bytes'])

    @staticmethod
    def _payload(row):
        if not isinstance(row, dict) or 'seq' in row:
            raise ValueError('采集记录格式无效')
        payload = encoded(row)
        size = len(payload.encode())
        if size > MAX_ROW_BYTES:
            raise ValueError('采集记录超出容量')
        return payload, size

    def append(self, run_id, row, *, robot=None, executor='gos', event_id=None):
        key = scope(run_id, robot, executor)
        payload, size = self._payload(row)
        if event_id is not None and (not isinstance(event_id, str) or not 1 <= len(event_id) <= 128):
            raise ValueError('采集事件身份无效')
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            stored = self._ensure(db, key)
            if event_id is not None:
                old = db.execute('SELECT seq,payload FROM records WHERE robot=? AND executor=? AND run_id=? AND event_id=?',
                                 (*key, event_id)).fetchone()
                if old is not None:
                    if old['payload'] != payload:
                        raise CollectionStorageError('重复采集事件内容不一致')
                    return dict(row, seq=old['seq'])
            if stored['sealed'] or stored['error']:
                raise CollectionStorageError(stored['error'] or '采集日志已收尾，不能追加')
            if not self._capacity(db, stored, 1, size):
                message = '采集缓存容量已满，未保存本次读数；已有记录保留'
                db.execute('UPDATE runs SET error=?,truncated=1 WHERE robot=? AND executor=? AND run_id=?', (message, *key))
                db.commit()
                raise CollectionStorageError(message)
            seq = stored['seq']+1
            db.execute('INSERT INTO records VALUES(?,?,?,?,?,?)', (*key, seq, event_id, payload))
            db.execute('UPDATE runs SET seq=?,bytes=bytes+? WHERE robot=? AND executor=? AND run_id=?', (seq, size, *key))
            db.commit()
        return dict(row, seq=seq)

    def finish(self, run_id, *, robot=None, executor='gos', error=None):
        key = scope(run_id, robot, executor)
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            self._ensure(db, key)
            if error:
                db.execute('UPDATE runs SET sealed=1,error=?,truncated=1 WHERE robot=? AND executor=? AND run_id=?',
                           (str(error)[:300], *key))
            else:
                db.execute('UPDATE runs SET sealed=1 WHERE robot=? AND executor=? AND run_id=?', key)
            result = self._metadata(self._run(db, key))
            db.commit()
        return result

    def _resolve(self, db, run_id, robot, executor):
        key = scope(run_id, robot, executor)
        if robot is not None:
            return self._run(db, key)
        # Old offset requests have no robot hash. Never choose between two robots.
        matches = db.execute('SELECT * FROM runs WHERE executor=? AND run_id=? LIMIT 2', (executor, run_id)).fetchall()
        if len(matches) > 1:
            raise CollectionStorageError('任务编号对应多个机器人，请指定采集机器人身份')
        return matches[0] if matches else None

    def page(self, run_id, offset=0, page_size=100, *, cursor=None, robot=None,
             executor='gos', journal_id=None):
        scope(run_id, robot, executor)
        page_arguments(offset, page_size, cursor)
        if cursor is not None and robot is None:
            raise ValueError('增量同步需要采集机器人身份')
        with self._db() as db:
            if db is None:
                return unavailable(run_id, '采集日志尚不存在', robot=robot, executor=executor)
            # Records and watermark belong to one consistent read snapshot.
            db.execute('BEGIN')
            run = self._resolve(db, run_id, robot, executor)
            if run is None:
                return unavailable(run_id, '采集日志尚不存在', robot=robot, executor=executor)
            if journal_id is not None and journal_id != run['journal_id']:
                return unavailable(run_id, '采集日志身份已变化', robot=robot, executor=executor)
            after = offset if cursor is None else cursor
            if after > run['seq']:
                return unavailable(run_id, '采集游标超过已保存记录，数据不可确认', robot=robot, executor=executor)
            key = (run['robot'], run['executor'], run['run_id'])
            raw = db.execute('SELECT seq,payload FROM records WHERE robot=? AND executor=? AND run_id=? AND seq>? ORDER BY seq LIMIT ?',
                             (*key, after, page_size+1)).fetchall()
            rows, size = [], 0
            for item in raw[:page_size]:
                length = len(item['payload'].encode())
                if size+length > MAX_PAGE_BYTES:
                    break
                if item['seq'] != after+len(rows)+1:
                    raise CollectionStorageError('采集日志缺页，不能跳过数据')
                rows.append(dict(json.loads(item['payload']), seq=item['seq']))
                size += length
            next_cursor = after+len(rows)
            if next_cursor < run['seq'] and not rows:
                raise CollectionStorageError('采集日志缺页，不能跳过数据')
            more = next_cursor < run['seq']
            return dict(self._metadata(run), ok=True, rows=rows, cursor=after,
                        next_cursor=next_cursor, next_offset=next_cursor if more else None,
                        has_more=more, unavailable=False, stale=True, live=False, source='onboard_journal')


class CollectionCache(CollectionJournal):
    """Mirror with durable transfer intents. No received cursor is saved alone."""
    def __init__(self, data, *, limits=None):
        super().__init__(data, directory='task-collection-cache', limits=limits)

    @staticmethod
    def _state(db, key):
        row = db.execute('SELECT metadata FROM transfers WHERE robot=? AND executor=? AND run_id=?', key).fetchone()
        return json.loads(row[0]) if row else dict(robot=key[0], executor=key[1], run_id=key[2],
                    protocol=SYNC_PROTOCOL, supported=None, state='pending', pending=True,
                    cursor=0, high_watermark=0, cached_rows=0, complete=False, final=False,
                    last_synced_at=None, stale=True, live=False, goal=None, error=None)

    def _put_state(self, db, key, state):
        if not db.execute('SELECT 1 FROM transfers WHERE robot=? AND executor=? AND run_id=?', key).fetchone():
            if db.execute('SELECT count(*) FROM transfers').fetchone()[0] >= self.limits['runs']:
                raise CollectionStorageError('本机采集缓存任务容量已满')
        db.execute('INSERT OR REPLACE INTO transfers VALUES(?,?,?,?)', (*key, encoded(state)))

    def state(self, run_id, *, robot, executor='gos'):
        key = scope(run_id, robot, executor)
        with self._db() as db:
            return self._state(db, key) if db is not None else None

    def states(self):
        with self._db() as db:
            return [json.loads(row[0]) for row in db.execute('SELECT metadata FROM transfers')] if db is not None else []

    def schedule(self, run_id, *, robot, executor='gos', final=False, target=None):
        key = scope(run_id, robot, executor)
        if target is not None and (type(target) is not int or not 0 <= target <= 2**63-1):
            raise ValueError('采集目标游标无效')
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            state = self._state(db, key)
            if final and state.get('complete') and (target is None or target <= state['cursor']):
                return state  # A sealed, fully mirrored run is immutable.
            # A boundary snapshots a finite target. Unknown targets are captured
            # by the first page, never continuously chased during navigation.
            goal = max(state.get('goal') or 0, target) if target is not None else None
            state.update(pending=True, state='pending', final=state['final'] or bool(final), goal=goal)
            self._put_state(db, key, state)
            db.commit()
        return state

    def failure(self, run_id, message, *, robot, executor='gos', state='error', supported=None, retry=True):
        key = scope(run_id, robot, executor)
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            meta = self._state(db, key)
            meta.update(state=state, error=str(message)[:300], pending=bool(retry))
            if supported is not None:
                meta['supported'] = supported
            self._put_state(db, key, meta)
            db.commit()
        return meta

    def ingest(self, run_id, page, *, robot, executor='gos', requested_cursor=0):
        key = scope(run_id, robot, executor)
        page_arguments(0, MAX_PAGE_ROWS, requested_cursor)
        if (not isinstance(page, dict) or page.get('collection_sync_protocol') != SYNC_PROTOCOL
                or page.get('run_id') != run_id or page.get('robot') != robot
                or page.get('executor', executor) != executor or page.get('available') is not True
                or page.get('ok') is not True):
            raise CollectionStorageError('采集页协议或任务身份不匹配')
        journal_id = identity(page.get('journal_id'))
        rows, high, next_cursor = page.get('rows'), page.get('high_watermark'), page.get('next_cursor')
        if (not isinstance(rows, list) or len(rows) > MAX_PAGE_ROWS or type(high) is not int
                or not requested_cursor <= high <= 2**63-1 or type(next_cursor) is not int
                or next_cursor != requested_cursor+len(rows) or next_cursor > high
                or page.get('cursor', requested_cursor) != requested_cursor
                or type(page.get('sealed')) is not bool or type(page.get('complete')) is not bool
                or page['complete'] and not page['sealed']):
            raise CollectionStorageError('采集分页游标或收尾状态无效')
        payloads, byte_count = [], 0
        for expected, row in enumerate(rows, requested_cursor+1):
            if not isinstance(row, dict) or type(row.get('seq')) is not int or row['seq'] != expected:
                raise CollectionStorageError('采集分页缺页或顺序无效')
            payload, size = self._payload({k:v for k,v in row.items() if k != 'seq'})
            payloads.append((expected, payload, size))
            byte_count += size
        if byte_count > MAX_PAGE_BYTES or not rows and requested_cursor < high:
            raise CollectionStorageError('采集分页超限或缺页')
        with self._db(create=True) as db:
            db.execute('BEGIN IMMEDIATE')
            run = self._ensure(db, key, journal_id)
            meta = self._state(db, key)
            if requested_cursor > run['seq'] or high < max(run['seq'], meta.get('high_watermark',0)):
                raise CollectionStorageError('采集日志回退或本机缓存缺页')
            if run['sealed'] and high != meta.get('high_watermark', run['seq']):
                raise CollectionStorageError('已收尾的采集日志不能继续增长')
            additions = []
            for seq, payload, size in payloads:
                if seq <= run['seq']:
                    old = db.execute('SELECT payload FROM records WHERE robot=? AND executor=? AND run_id=? AND seq=?', (*key, seq)).fetchone()
                    if old is None or old[0] != payload:
                        raise CollectionStorageError('重复采集页内容不一致')
                else:
                    additions.append((seq, payload, size))
            if additions and additions[0][0] != run['seq']+1:
                raise CollectionStorageError('采集日志缺页，未推进游标')
            added_bytes = sum(row[2] for row in additions)
            if not self._capacity(db, run, len(additions), added_bytes):
                raise CollectionStorageError('本机采集缓存容量已满，未推进游标')
            for seq, payload, _ in additions:
                db.execute('INSERT INTO records VALUES(?,?,?,?,NULL,?)', (*key, seq, payload))
            cursor = max(run['seq'], next_cursor)
            sealed = bool(run['sealed']) or page['sealed']
            error = str(run['error'] or page.get('error') or '')[:300] or None
            truncated = bool(run['truncated'] or page.get('truncated'))
            db.execute('UPDATE runs SET seq=?,bytes=bytes+?,sealed=?,error=?,truncated=? WHERE robot=? AND executor=? AND run_id=?',
                       (cursor, added_bytes, int(sealed), error or '', int(truncated), *key))
            goal = meta['goal'] if meta.get('goal') is not None else high
            # A final request includes every committed row and waits for an
            # explicit sealed marker. Empty/partial pages never prove completion.
            if meta['final']:
                goal = max(goal, high)
            pending = cursor < goal or meta['final'] and not sealed
            source_complete = bool(page['complete'] or meta.get('source_complete') or meta.get('complete'))
            complete = source_complete and sealed and cursor >= high and not error and not truncated
            meta.update(supported=True, source_complete=source_complete, cursor=cursor, cached_rows=cursor,
                        high_watermark=high, goal=goal, journal_id=journal_id,
                        pending=bool(pending), complete=bool(complete),
                        state='pending' if pending else 'partial' if error or truncated else 'synced',
                        error=error, last_synced_at=time.time(), source='local_cache')
            self._put_state(db, key, meta)
            db.commit()  # Rows and the transfer cursor are acknowledged together.
        return meta

    def page(self, run_id, offset=0, page_size=100, *, robot, executor='gos', **kwargs):
        meta = self.state(run_id, robot=robot, executor=executor)
        page = super().page(run_id, offset, page_size, robot=robot, executor=executor, **kwargs)
        page.update(source='local_cache', stale=True, live=False, collection_transfer=meta)
        if meta is None:
            return page
        page['complete'] = bool(meta['complete'])
        page['sync_supported'] = meta['supported']
        page['cached_rows'] = meta['cached_rows']
        page['high_watermark'] = meta['high_watermark']
        page['pending'] = meta['pending']
        if page.get('ok'):
            after = page['next_cursor']
            more = after < meta['high_watermark']
            page['has_more'] = more
            page['next_offset'] = after if more else None
            if not page['rows'] and (more or meta['pending']):
                message = meta.get('error') or '采集记录尚未同步到本机，不能将缺页作为空记录'
                page.update(ok=False, available=False, unavailable=True, error=message, err=message)
        return page

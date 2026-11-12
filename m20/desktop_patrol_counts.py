"""Read-only camera sampling for patrols using native or desktop navigation."""
import time
import uuid

from bridge.task_collection import CollectionCancelled, RouteCollection
from m20.patrol_actions import StationActions
from m20.platform_core import age_of
from m20.task_collectors import requires_gos_collection


class SharedPeopleSampler:
    def __init__(self, cameras, aos, spec):
        self.actions = StationActions(aos)
        self.feed = cameras.get(spec['camera'])
        self.sequence = -1
        self.feed.subscribe()

    def sample(self, cancelled):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if cancelled():
                raise CollectionCancelled
            packet = self.feed.read_raw(self.sequence, .2)
            if packet is None:
                continue
            self.sequence, frame, captured_at, _ = packet
            try:
                people = self.actions.count_people(frame, size=416)
            except Exception:
                return dict(sampled_at=captured_at, error='电脑端人数识别模型或运行环境未就绪')
            return dict(sampled_at=captured_at, values={'people': people})
        return dict(sampled_at=time.time(), error='暂未取得新鲜相机画面')

    def close(self):
        self.feed.unsubscribe()


class DesktopPatrolCounts:
    def __init__(self, app):
        self.app = app
        self.runtime = None
        self.owner = None
        self.link = None

    def _running(self, nav):
        if (self.app.demo or self.app.done or not self.app.link
                or nav.get('status') != 'running' or nav.get('executor') == 'gos'
                or age_of(self.app.link.record(1002, 6)) >= 2):
            return False
        if nav.get('engine') == 'resident':
            return nav.get('phase') in ('resident_forward', 'resident_returning')
        return nav.get('execution_state') not in ('preparing', 'starting')

    def _context(self, run_id):
        with self.app.lock:
            nav = self.app.nav
            if (self.app.link is self.link and nav.get('collection_run', {}).get('id') == run_id
                    and self._running(nav)):
                return dict(index=nav.get('index', 0))
            return None

    def tick(self):
        nav = self.app.nav
        if self.runtime:
            if self.app.link is self.link and nav.get('collection_run') == self.owner.get('collection_run'):
                self.owner = nav
                if nav.get('status') in ('running', 'paused', 'cancelling'):
                    nav['collection'] = self.runtime.snapshot()
                    return
                self.finish(nav)
                return
            self.finish()
        specs = (nav.get('route') or {}).get('options', {}).get('collectors', [])
        if not specs or requires_gos_collection(specs) or not self._running(nav):
            return
        previous = nav.get('collection_run', {})
        run_id = previous['id'] if previous.get('local') is True else uuid.uuid4().hex
        nav['collection_run'] = dict(id=run_id, local=True)
        self.link, self.owner = self.app.link, nav
        self.runtime = RouteCollection(
            self.app.store.path.parent, run_id, self.link.ip, specs,
            lambda: self._context(run_id),
            worker_factory=lambda data, aos, spec: SharedPeopleSampler(self.app.cameras, aos, spec),
        )
        for row in nav.get('collection', []):
            kind = row.get('spec', {}).get('kind')
            if previous.get('local') is True and kind in self.runtime.status:
                self.runtime.status[kind].update(row, state='waiting')
        nav['collection'] = self.runtime.snapshot()
        self.runtime.start()

    def finish(self, nav=None):
        if self.runtime and (nav is None or nav.get('collection_run') == self.owner.get('collection_run')):
            self.runtime.stop()
            target = nav if nav is not None else self.owner
            target['collection'] = self.runtime.snapshot()
            self.runtime = self.owner = self.link = None

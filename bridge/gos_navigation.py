"""GOS runtime for the platform path planner and waypoint actions."""
import base64
import copy
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import threading
import time
import zlib

from bridge.resident_mission import atomic
from bridge.task_collection import RouteCollection, collection_page
from m20.map_catalog import read_json
from m20.navigation_disconnect import finite, identity, specification
from m20.navigation_planner import GridPlanner
from m20.patrol_actions import StationActions, station_spec
from m20.platform_core import RobotLink, age_of, number
from m20.point_navigation import PointNavigator, navigation_observation, stationary_feedback
from m20.task_collectors import check_collectors, collector_specs


GOS_LEASE_SECONDS = 5.
STOPPED = frozenset(('idle', 'paused', 'completed', 'cancelled', 'failed'))
MAX_PGM = 12_000_000


def pack_grid(raw, source):
    grid = dict(pgm_zlib=base64.b64encode(zlib.compress(raw, 3)).decode('ascii'),
                sha256=hashlib.sha256(raw).hexdigest(), resolution=source['resolution'],
                origin=source['origin'], negate=source.get('negate', 0),
                free_thresh=source.get('free_thresh', .196),
                occupied_thresh=source.get('occupied_thresh', .65))
    if len(grid['pgm_zlib']) > 3_800_000:
        raise ValueError('导航栅格超出 GOS 单次传输容量')
    return grid


def unpack_grid(grid):
    if not isinstance(grid, dict):
        raise ValueError('缺少导航栅格')
    try:
        compressed = base64.b64decode(grid['pgm_zlib'], validate=True)
        decoder = zlib.decompressobj()
        raw = decoder.decompress(compressed, MAX_PGM+1)
    except (KeyError, TypeError, ValueError, zlib.error):
        raise ValueError('导航栅格传输损坏') from None
    if len(raw) > MAX_PGM or not decoder.eof or decoder.unused_data:
        raise ValueError('导航栅格超过大小限制或传输不完整')
    if hashlib.sha256(raw).hexdigest() != grid.get('sha256'):
        raise ValueError('导航栅格指纹不匹配')
    origin = grid.get('origin')
    if not isinstance(origin, list) or len(origin) != 3 or abs(finite(origin[2])) > 1e-6:
        raise ValueError('导航栅格坐标系无效')
    options = dict(resolution=number(grid.get('resolution'), .001, 1),
                   origin=[finite(v) for v in origin], negate=grid.get('negate', 0),
                   free_thresh=number(grid.get('free_thresh'), 0, 1),
                   occupied_thresh=number(grid.get('occupied_thresh'), 0, 1))
    if options['negate'] not in (0, 1) or options['free_thresh'] >= options['occupied_thresh']:
        raise ValueError('导航栅格阈值无效')
    return raw, options


class GOSPointNavigator(PointNavigator):
    async_observations = True

    def current(self, token):
        return (self.active and self.run_id == token and self.app.nav is self.nav
                and self.nav['status'] == 'running' and time.monotonic() < self.app.lease_until)

    def _prepare(self, token, map_):
        app = self.app
        self.check(token)
        link = app.link_factory(app.aos)
        with app.lock:
            if not self.current(token):
                link.close(stop=False)
                self.check(token)
            app.link = link
        began = time.monotonic()
        while time.monotonic() - began < 2:
            self.check(token)
            if stationary_feedback(link, began):
                break
            time.sleep(.04)
        else:
            raise ValueError('机内运控未确认停稳')
        while time.monotonic() < app.axis_quiet_until:
            self.check(token)
            time.sleep(.04)
        self._prepare_motion(token)

    def _fetch_scan(self, after_stamp):
        request = dict(mapping=False)
        if after_stamp is not None:
            request['after_stamp'] = after_stamp
        began = time.monotonic()
        scan = self.app.sensors().call('/sensors/autonomy', request, timeout=.45)
        if scan.get('protocol') != 1:
            raise ValueError('GOS 感知服务尚未就绪')
        received = time.monotonic()
        elapsed = received - began
        processing = number(scan.get('server_elapsed_s', 0), 0, 5)
        age = number(scan.get('age'), 0, 100) + max(0., elapsed-processing)
        if age >= .65:
            raise ValueError('机内雷达观测已过期')
        return dict(scan, _valid_until=received+.65-age)


class GOSNavigation:
    """One leased job. Status and preview do not create an AOS connection."""
    demo = False

    def __init__(self, data, sensors, *, aos=None, enabled=None, link_factory=RobotLink, actions=None):
        self.data = Path(data)
        self.path = self.data/'gos-navigation-job.json'
        self.sensors = sensors
        self.aos = aos or os.environ.get('M20_GOS_AOS_IP', '')
        try:
            address = ipaddress.IPv4Address(self.aos)
            configured = address.is_private and not address.is_loopback and not address.is_unspecified
        except ValueError:
            configured = False
        self.enabled = configured and (enabled if enabled is not None else os.environ.get('M20_GOS_NAVIGATION') == '1')
        config = read_json(self.data/'gos-patrol.local.json', {})
        self.actions = actions or StationActions(self.aos, config)
        self.link_factory = link_factory
        self.lock = threading.RLock()
        self.link = None
        self.done = False
        self.watchdog = None
        self.navigator = None
        self.collection = None
        self.lease_until = 0.
        self.axis_quiet_until = 0.
        self.velocity = [0., 0., 0.]
        self.nav = dict(status='idle', phase='idle', index=0, completed=0)
        receipt = read_json(self.path, {})
        self.job = receipt.get('job')
        self.revoked = set(receipt.get('revoked', []))
        if self.job:
            self.nav.update(receipt['nav'])
            if self.nav['status'] not in STOPPED:
                self.nav.update(status='interrupted', phase='interrupted', cancel_confirmed=False,
                                message='GOS 服务曾中断，正在等待停车核对')

    def capabilities(self):
        available = bool(self.enabled and importlib.util.find_spec('numpy') is not None)
        return dict(ok=True, protocol=1, available=available, executor='gos',
                    policies=['stop'], station_actions=self.actions.capabilities(), collection_protocol=1,
                    message='GOS 自主寻路已配置' if available else '请在连接中心更新并配置 GOS 导航服务')

    def _persist(self):
        saved = {key: copy.deepcopy(self.nav[key]) for key in
                 ('status', 'phase', 'index', 'completed', 'message', 'station_results', 'cancel_confirmed', 'collection')
                 if key in self.nav}
        if self.collection is not None:
            saved['collection'] = self.collection.snapshot()
        atomic(self.path, dict(job=self.job, nav=saved, revoked=sorted(self.revoked)))

    def status(self):
        with self.lock:
            if not self.job:
                return dict(protocol=1, phase='idle', active=False, executor='gos')
            nav = self.nav
            plan = nav.get('plan') or {}
            path = plan.get('path') or []
            stride = max(1, math.ceil(len(path)/160))
            visible = path[::stride]
            if path and visible[-1] != path[-1]:
                visible = [*visible, path[-1]]
            result = dict(protocol=1, id=self.job['id'], executor='gos', phase=nav['status'],
                        stage=nav['phase'], completed=nav['completed'], active=nav['status'] not in STOPPED,
                        stop_confirmed=bool(nav.get('cancel_confirmed')), message=nav.get('message', ''),
                        station_results=copy.deepcopy(nav.get('station_results', [])),
                        plan=dict(path=visible, goal=copy.deepcopy(plan.get('goal'))) if visible else None)
            collected = self.collection.snapshot() if self.collection is not None else copy.deepcopy(nav.get('collection', []))
            if collected:
                if nav['status'] in STOPPED or nav['status'] == 'interrupted':
                    for row in collected:
                        row['state'] = 'ended'
                result['collection'] = collected
            return result

    def _collection_context(self, nav):
        with self.lock:
            if (self.nav is nav and nav['status'] == 'running' and time.monotonic() < self.lease_until
                    and nav['phase'] in ('tracking', 'local_bypass', 'recovering_start', 'waiting_space', 'finding_bypass')):
                return dict(index=nav['index'])
            return None

    def navigation_surface(self, map_id, scan):
        return navigation_observation(self.link, scan, self.job['id'])

    def set_velocity(self, request, autonomous=False, valid_until=None):
        values = [request[k] for k in ('forward', 'side', 'yaw')]
        if any(values):
            if self.done or time.monotonic() >= self.lease_until or self.nav['status'] != 'running':
                raise ValueError('GOS 运动授权已到期')
            record = self.link.record(1002, 6)
            basic = record.get('items', {}).get('BasicStatus', {})
            if (age_of(record) >= 1 or basic.get('HES') != 0 or basic.get('MotionState') != 17
                    or basic.get('Sleep') or basic.get('ControlUsageMode') != 0
                    or basic.get('Direction') != 0 or basic.get('Gait') != 0x3002):
                raise ValueError('机内运控状态已变化')
            valid_until = min(valid_until, self.lease_until)
        self.link.set_velocity(*values, valid_until=valid_until)
        self.velocity = values

    def station_action(self, index, action, pose, guard):
        record = self.link.record(1007, 2)
        located = record.get('items', {})
        if age_of(record) >= .65 or located.get('Location') != 0:
            raise ValueError('到点定位已过期，请重新核对航点')
        pose = [finite(located.get(k)) for k in ('PosX', 'PosY', 'Yaw')]
        row = dict(index=index, pose=list(pose), **self.actions.run(action, guard))
        with self.lock:
            guard()
            self.nav.setdefault('station_results', []).append(row)
            self.nav.update(index=index+1,completed=index+1)
            self._persist()

    def _record_route_completion(self):
        self.nav['ended'] = time.time()

    def _owned(self, request):
        key = (identity(request.get('id')), identity(request.get('owner')))
        return self.job is not None and key == (self.job['id'], self.job['owner'])

    def preview(self, request):
        if not self.capabilities()['available']:
            raise ValueError('GOS 自主寻路尚未配置')
        with self.lock:
            if self.nav['status'] not in STOPPED:
                raise ValueError('请先结束当前 GOS 任务')
        raw, options = unpack_grid(request.get('grid'))
        start = [finite(v) for v in request['start']]
        goal = [finite(v) for v in request['goal']]
        yaw = request.get('yaw')
        if len(start) != 3 or len(goal) != 2:
            raise ValueError('起点或目标坐标无效')
        yaw = finite(yaw) if yaw is not None else None
        planner = GridPlanner(raw, **options, soft_static_speckles=True)
        try:
            plan = planner.plan(start, goal, yaw, goal_tolerance=.65)
        except ValueError:
            if planner.pose_fits(*start):
                raise
            # The preview has no local ground evidence. The onboard executor
            # will plan from a fresh scan before issuing any velocity.
            plan = dict(path=[], points=[], goal=dict(x=goal[0], y=goal[1], yaw=start[2] if yaw is None else yaw),
                        needs_local_plan=True, planner='gos_live_start')
        return dict(ok=True, plan=plan)

    def start(self, request):
        if not self.capabilities()['available']:
            raise ValueError('GOS 自主寻路尚未配置')
        spec = specification(request)
        if spec['disconnect_policy'] != 'stop':
            raise ValueError('GOS 自主寻路目前支持断联停车')
        start_index = request.get('start_index', 0)
        pause = request.get('pause_at_point', False)
        if type(start_index) is not int or not 0 <= start_index < len(spec['points']) or type(pause) is not bool:
            raise ValueError('巡检继续位置或到点选项无效')
        for point, source in zip(spec['points'], request['points']):
            action = station_spec(source.get('station_action'))
            if action is not None:
                point['station_action'] = action
        collectors = collector_specs(request.get('collectors'))
        if collectors:
            check_collectors(collectors, self.actions.capabilities())
        spec.update(start_index=start_index, pause_at_point=pause)
        if collectors:
            spec['collectors'] = collectors
        digest = hashlib.sha256(json.dumps(spec, sort_keys=True).encode()).hexdigest()
        raw, grid = unpack_grid(request.get('grid'))
        if (hashlib.sha256(raw).hexdigest() != spec['map']['sha256']
                or any(grid[k] != spec['map'][k] for k in ('resolution', 'origin'))):
            raise ValueError('GOS 收到的栅格与 NOS 活动地图不一致')
        self.actions.validate(spec['points'][start_index:])
        with self.lock:
            if spec['id'] in self.revoked:
                raise ValueError('本次启动已撤销')
            if self._owned(request):
                if self.job['digest'] != digest:
                    raise ValueError('重复任务的内容已变化')
                return self.status()
            if self.nav['status'] not in STOPPED or self.watchdog and self.watchdog.is_alive():
                raise ValueError('GOS 仍在执行任务或确认停车')
            if self.navigator and self.navigator.worker and self.navigator.worker.is_alive():
                raise ValueError('上一航点动作正在收尾')
            previous = self.job, self.nav, self.navigator, self.collection
            self.job = dict(id=spec['id'], owner=spec['owner'], digest=digest)
            self.lease_until = time.monotonic() + GOS_LEASE_SECONDS
            self.navigator = GOSPointNavigator(self)
            route = dict(points=spec['points'], options=dict(pause_at_point=pause))
            plan = dict(raw=raw, fingerprint=spec['map']['sha256'])
            token = self.navigator._new_run(route, plan, start_index)
            self.navigator.deadline = time.monotonic() + spec['max_duration_s']
            nav = self.nav
            self.collection = (RouteCollection(self.data, spec['id'], self.aos, collectors,
                                               lambda: self._collection_context(nav)) if collectors else None)
            try:
                self._persist()
            except (OSError, ValueError):
                self.job, self.nav, self.navigator, self.collection = previous
                self.lease_until = 0.
                raise ValueError('GOS 任务记录写入失败') from None
            if self.collection is not None:
                try:
                    self.collection.start()
                except RuntimeError:
                    self.collection.stop()
                    self.nav.update(status='failed', phase='failed', cancel_confirmed=True,
                                    message='沿途采集无法启动')
                    self._persist()
                    raise ValueError('沿途采集无法启动') from None
            self.watchdog = threading.Thread(target=self._watch, daemon=True, name='gos-navigation-lease')
            self.watchdog.start()
            self.navigator._launch(token, dict(grid, id=spec['map']['name']), plan)
            return self.status()

    def cancel(self, request):
        with self.lock:
            if not self._owned(request):
                key=identity(request.get('id'))
                if self.job and key==self.job['id']:
                    raise ValueError('任务或控制端身份不匹配')
                self.revoked.add(key)
                self._persist()
                return dict(ok=True, matched=False, revoked=True, mission=self.status())
            if self.nav['status'] == 'interrupted' and not (self.watchdog and self.watchdog.is_alive()):
                self.watchdog = threading.Thread(target=self._reconcile, daemon=True)
                self.watchdog.start()
            elif self.nav['status'] == 'running':
                inspecting = self.nav['phase'] == 'station_action'
                after = 'paused' if request.get('after') == 'paused' and not inspecting else 'cancelled'
                message = '到点作业已中止，等待停车确认' if inspecting else '操作者已请求停车'
                self.navigator.stop(after, message)
            return dict(ok=True, matched=True, mission=self.status())

    def dispatch(self, path, request):
        request = request or {}
        if path == '/missions/capabilities':
            return self.capabilities()
        if path == '/missions/status':
            return dict(ok=True, mission=self.status())
        if path == '/missions/collection':
            return collection_page(self.data, request.get('id'), request.get('offset', 0))
        if path == '/missions/preview':
            return self.preview(request)
        if path == '/missions/start':
            return dict(ok=True, mission=self.start(request))
        if path == '/missions/cancel':
            return self.cancel(request)
        if path == '/missions/pulse':
            with self.lock:
                if not self._owned(request):
                    raise ValueError('任务或控制端身份不匹配')
                if self.nav['status'] == 'running' and time.monotonic() < self.lease_until:
                    self.lease_until = time.monotonic() + GOS_LEASE_SECONDS
                return dict(ok=True, mission=self.status())
        raise ValueError('GOS 任务接口不存在')

    def _watch(self):
        while not self.done:
            with self.lock:
                if self.nav['status'] == 'running' and time.monotonic() >= self.lease_until:
                    self.navigator.stop('cancelled', '电脑连接中断，GOS 已请求停车')
                self.navigator.tick()
                if self.nav['status'] == 'cancelling' and self.link is None:
                    self.nav.update(status=self.navigator.after_stop, phase=self.navigator.after_stop,
                                    cancel_confirmed=True)
                if self.nav['status'] in STOPPED:
                    if self.collection is not None:
                        self.collection.stop()
                    if self.link is not None:
                        self.axis_quiet_until = getattr(self.link,'last_axis_send',0.) + 2.05
                        self.link.close(stop=False)
                        self.link = None
                    self._persist()
                    return
            time.sleep(.05)

    def _reconcile(self):
        if not self.enabled:
            return
        link = self.link_factory(self.aos)
        try:
            began = time.monotonic()
            while time.monotonic()-began < 3 and not self.done:
                if stationary_feedback(link, began):
                    with self.lock:
                        self.nav.update(status='cancelled', phase='cancelled', cancel_confirmed=True,
                                        message='已核对停稳，旧任务已结束')
                        self._persist()
                    return
                time.sleep(.05)
        finally:
            link.close(stop=False)

    def close(self):
        with self.lock:
            if self.collection is not None:
                self.collection.stop()
            if self.nav['status'] == 'running':
                self.navigator.stop('cancelled', 'GOS 服务关闭，已请求停车')
        if self.watchdog:
            self.watchdog.join(timeout=1)
        self.done = True
        with self.lock:
            if self.link is not None:
                self.link.close()
                self.link = None

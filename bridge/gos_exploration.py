"""Run the shared exploration controller under the GOS mission lease."""
import hashlib
import json
import logging
import math
import threading
import time
from types import SimpleNamespace

from m20.autonomy_core import Explorer, PriorMap
from m20.autonomy_runtime import Autonomy
from m20.localization_runtime import OperationCancelled
from m20.navigation_disconnect import identity
from m20.platform_core import number
from m20.point_navigation import PointNavigator, stationary_feedback


def exploration_spec(request):
    expand = request.get('expand', False)
    if type(expand) is not bool:
        raise ValueError('探索范围无效')
    spec = dict(id=identity(request.get('id')), owner=identity(request.get('owner')),
                capture_id=identity(request.get('capture_id')),
                duration_s=number(request.get('duration_s'), 6., 3600.),
                radius=number(request.get('radius'), 2., 100.),
                forward_limit=number(request.get('forward_limit'), .1, 1.),
                yaw_limit=number(request.get('yaw_limit'), .1, 1.), expand=expand)
    grid = request.get('prior_grid')
    prior = None
    if grid is not None:
        if not isinstance(grid, dict) or not isinstance(grid.get('pgm'), str) or not 0 < len(grid['pgm']) <= 3_800_000:
            raise ValueError('原地图栅格缺失或超过传输容量')
        origin = grid.get('origin')
        if not isinstance(origin, list) or len(origin) != 3 or abs(number(origin[2], -math.pi, math.pi)) > 1e-6:
            raise ValueError('原地图栅格坐标系无效')
        grid = dict(pgm=grid['pgm'], resolution=number(grid.get('resolution'), .001, 1.),
                    origin=[number(value, -10000., 10000.) for value in origin],
                    negate=grid.get('negate', 0), free_thresh=number(grid.get('free_thresh', .196), 0., 1.),
                    occupied_thresh=number(grid.get('occupied_thresh', .65), 0., 1.))
        if grid['negate'] not in (0, 1) or grid['free_thresh'] >= grid['occupied_thresh']:
            raise ValueError('原地图栅格阈值无效')
        anchor = request.get('anchor')
        if not isinstance(anchor, dict):
            raise ValueError('缺少本轮旧图定位锚点')
        anchor = dict(x=number(anchor.get('x'), -10000., 10000.),
                      y=number(anchor.get('y'), -10000., 10000.),
                      yaw=number(anchor.get('yaw'), -math.pi, math.pi))
        prior = PriorMap(grid, anchor)
        spec.update(prior_grid=grid, anchor=anchor)
    return spec, prior


class _ExplorationContext:
    """Adapt the controller to a capture verified by the desktop heartbeat."""
    demo = False
    resident_execution = True

    def __init__(self, service, spec):
        self.service = service
        self.lock = service.lock
        self.context = spec['id'], spec['capture_id']
        self.anchor = spec.get('anchor')
        self.mapping = True
        self.store = SimpleNamespace(value={})
        self.hub = SimpleNamespace(connections=SimpleNamespace(value=dict(sensor_source='gos')),
                                   _operation_context=threading.local(),
                                   same_context=self.same_context, remote_maps=self.capture_status)

    @property
    def link(self):
        return self.service.link

    @property
    def velocity(self):
        return self.service.velocity

    @property
    def done(self):
        return self.service.done

    def same_context(self, context):
        service = self.service
        job = service.job or {}
        return (context == self.context and (job.get('id'), job.get('capture_id')) == context
                and not service.done and service.nav['status'] == 'running'
                and time.monotonic() < service.lease_until)

    def capture_status(self, path, *, context, timeout):
        with self.lock:
            if path != '/maps/build/status' or not self.same_context(context):
                raise OperationCancelled('建图采集授权已失效')
            return dict(phase='collecting', in_progress=True,
                        mapping=dict(request_id=context[1], context=dict(anchor=self.anchor)))

    def proxy(self, role, path, request, timeout):
        return self.service.sensors().call(path, request, timeout=timeout)

    def set_velocity(self, *args, **kwargs):
        return self.service.set_velocity(*args, **kwargs)

    @staticmethod
    def add_log(kind, message):
        logging.getLogger(__name__).warning('%s: %s', kind, message)


class GOSExploration(Autonomy):
    def __init__(self, service, spec, prior, previous=None):
        super().__init__(_ExplorationContext(service, spec))
        self.service = service
        self.active = True
        self.run_id = spec['id']
        self.capture_id = spec['capture_id']
        self.context = self.app.context
        self.mode = 'explore'
        self.scope = 'map' if spec['expand'] else 'radius'
        self.state = 'preparing'
        self.message = 'GOS 正在核对站立状态与建图扫描'
        self.duration = spec['duration_s']
        self.started = time.monotonic()
        self.deadline = self.started + self.duration
        settings = {key: value for key, value in spec.items() if key not in ('id', 'owner', 'duration_s')}
        self.settings = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
        self.reusing = (isinstance(previous, GOSExploration) and previous.settings == self.settings
                        and previous.explorer.last_pose is not None)
        if self.reusing:
            self.explorer = previous.explorer
            self.elapsed_base = previous.finished_elapsed
        else:
            self.explorer = Explorer(spec['radius'], spec['forward_limit'], spec['yaw_limit'], expand=spec['expand'])
            self.explorer.prior = prior
        self.nav = dict(status='running', phase='preparing', index=0, completed=0, message=self.message)
        self.after_stop = 'cancelled'
        self.stop_at = None
        self.worker = None

    def launch(self):
        self.worker = threading.Thread(target=self._execute, args=(self.run_id,), daemon=True, name='gos-exploration')
        self.worker.start()

    def check(self, token):
        self._assert_run(token)

    def _assert_run(self, run_id):
        with self.app.lock:
            self.watchdog()
            if not self.active or self.run_id != run_id or not self.app.same_context(self.context):
                raise OperationCancelled('GOS 自主任务或建图采集授权已失效')

    def _execute(self, token):
        try:
            self.service.prepare_link(lambda: self.check(token))
            PointNavigator._prepare_motion(self, token, require_location=False)
            if self.reusing:
                self._resume_from_scan(token)
            self.check(token)
            self._run(token, None, self.duration)
        except Exception as exc:
            with self.app.lock:
                if self.run_id == token and self.app.same_context(self.context):
                    self.stop('failed', str(exc))

    def _resume_from_scan(self, token):
        explorer = self.explorer
        last_stamp = explorer.last_stamp
        deadline = min(self.deadline, time.monotonic() + 2.)
        while time.monotonic() < deadline:
            self.check(token)
            try:
                scan = self.scan_now(True, after_stamp=last_stamp)
            except ValueError:
                time.sleep(.04)
                continue
            if last_stamp is not None and scan['stamp'] <= last_stamp:
                time.sleep(.04)
                continue
            with self.app.lock:
                self.check(token)
                pose = tuple(scan['pose'])
                displacement = math.dist(pose[:2], explorer.last_pose[:2])
                explorer.reset_recovery_context(verified_translation_m=displacement, reason='explicit_same_capture_resume')
                explorer.last_pose = pose
                explorer.last_stamp = scan['stamp']
                explorer.distance_anchor = pose[:2]
                explorer.current_scan = scan
                explorer.path = []
                explorer.goal = None
                explorer.plan_state = 'idle'
                self.scan = scan
            return
        raise ValueError('等待原采集的新鲜 SLAM 位姿，已保留探索记录')

    def scan_now(self, mapping, role=None, after_stamp=None):
        scan = super().scan_now(True, role=role, after_stamp=after_stamp)
        pose = scan.get('pose', [])
        if (scan.get('pose_source') != '/SLAM_ODOM' or scan.get('pose_frame_id') != 'map'
                or scan.get('frame_id') != 'base_link' or len(pose) != 3
                or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in pose)):
            raise ValueError('等待雷达与本轮 SLAM 坐标系的同步扫描')
        return scan

    def watchdog(self):
        if self.active:
            now = time.monotonic()
            if now >= self.service.lease_until:
                self.stop('cancelled', '电脑连接中断，GOS 已请求停车')
            elif now >= self.deadline:
                self.finish_save(self.run_id, '本轮探索时间已到，正在停车')

    def finish_save(self, run_id=None, message='本轮探索已结束，正在停车'):
        with self.app.lock:
            if self.active and (run_id is None or self.run_id == run_id) and self.app.same_context(self.context):
                if self.map_view:
                    self.stop('completed', message.replace('正在保存地图', '正在停车'))
                else:
                    self.stop('failed', '本轮未取得可用的 SLAM 扫描，已请求停车并保留采集')

    def pause(self, message='已停止自主移动'):
        self.watchdog()
        if self.active:
            self.stop('paused', message)

    def stop(self, after='cancelled', message='已请求停止 GOS 自主探索'):
        if not self.active or self.service.nav is not self.nav:
            return
        self.active = False
        self.run_id = None
        self.after_stop = after
        self.stop_at = time.monotonic()
        self.finished_elapsed = round(self.elapsed_base + self.stop_at - self.started)
        self.nav.update(status='cancelling', phase='stopping', message=message)
        if self.observation_pump:
            self.observation_pump.cancel()
        self.service.velocity = [0., 0., 0.]
        if self.app.link:
            self.app.link.set_velocity(0., 0., 0.)

    def tick(self):
        self.watchdog()
        if self.stop_at is None:
            return
        if self.app.link is None or stationary_feedback(self.app.link, self.stop_at):
            self.nav.update(status=self.after_stop, phase=self.after_stop, cancel_confirmed=True)
            self.stop_at = None
        elif time.monotonic() - self.stop_at > 2.:
            self.nav['phase'] = 'cancel_unconfirmed'

    def report(self):
        view = self.map_view
        if view:
            view = dict(view, capture_id=self.capture_id)
            # Status and lease replies carry a bounded display sample, never
            # the resident map used for footprint checks and exploration.
            for key, limit in [('floor', 4000), ('obstacles', 2000)]:
                cells = view.get(key, [])
                stride = max(1, math.ceil(len(cells) / limit))
                view[key] = cells[::stride]
            path = view.get('path', [])
            if len(path) > 160:
                view['path'] = path[::math.ceil(len(path) / 160)]
                if view['path'][-1] != path[-1]:
                    view['path'] = [*view['path'], path[-1]]
            deadline = (self.scan or {}).get('_valid_until')
            age = max(0., time.monotonic() + .65 - deadline) if deadline is not None else None
            view['observation'] = dict(live=self.active and age is not None and age < 1.5,
                                       age=round(age, 2) if age is not None else None, frame='slam_session',
                                       capture_id=self.capture_id, stamp=(self.scan or {}).get('stamp'))
        return dict(active=self.active, state=self.state if self.active else self.nav['status'],
                    message=self.message if self.active else self.nav['message'],
                    map=view, elapsed=round(self.elapsed_base + time.monotonic() - self.started)
                    if self.active else self.finished_elapsed, scope=self.scope, capture_id=self.capture_id)

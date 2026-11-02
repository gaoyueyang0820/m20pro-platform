"""Read-only SLAM-session position for a paused exploration display.

The navigation /ODOM pose is intentionally never copied onto a SLAM-session
map. This observer has no planner or actuator calls and never alters Explorer.
"""
import math
import threading
import time


class PausedExplorationView:
    def __init__(self, autonomy):
        self.autonomy = autonomy
        self.app = autonomy.app
        self.thread = None
        self.token = None
        self.demand_until = 0.
        self.sample = None
        self.error = None

    def _eligible(self, token):
        a = self.autonomy
        return (self.token is token and not self.app.done and not a.active
                and a.mode == 'explore' and a.state == 'paused' and self.app.mapping
                and a.generation == token['generation'] and a.capture_id == token['capture_id']
                and self.app.hub.same_context(token['context']))

    def _session_matches(self, status, token):
        return (status.get('phase') == 'collecting' and status.get('in_progress')
                and status.get('mapping', {}).get('request_id') == token['capture_id'])

    def _invalidate(self, token, message):
        with self.app.lock:
            if self.token is token:
                self.error = message
        raise ValueError(message)

    def _collect(self, token):
        """Read both sides of the scan; a concurrent save/restart cannot relabel it."""
        with self.app.lock:
            if not self._eligible(token):
                return False
        before = self.app.hub.remote_maps('/maps/build/status', context=token['context'], timeout=2)
        with self.app.lock:
            if not self._eligible(token):
                return False
        if not self._session_matches(before, token):
            self._invalidate(token, '采集会话已变化，保留最后观测')
        previous = getattr(self.app.hub._operation_context, 'value', None)
        self.app.hub._operation_context.value = token['context']
        try:
            scan = self.autonomy.scan_now(True, role=token['role'])
        finally:
            self.app.hub._operation_context.value = previous
        with self.app.lock:
            if not self._eligible(token):
                return False
        after = self.app.hub.remote_maps('/maps/build/status', context=token['context'], timeout=2)
        with self.app.lock:
            if not self._eligible(token):
                return False
        if not self._session_matches(after, token):
            self._invalidate(token, '采集已结束或切换，保留最后观测')
        pose = scan.get('pose', [])
        stamp = scan.get('stamp')
        if (len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)
                or not isinstance(stamp, (int, float)) or not math.isfinite(stamp)):
            raise ValueError('等待同一采集坐标系的有效位姿')
        now = time.monotonic()
        deadline = scan.get('_valid_until')
        # Display may tolerate 1.5 s age; commands still use their original
        # <=650 ms deadline. No observation from this class drives the robot.
        captured = deadline - .65 if isinstance(deadline, (int, float)) else float('nan')
        if not math.isfinite(captured) or now - captured > 1.5:
            raise ValueError('位置反馈尚未更新，保留最后观测')
        with self.app.lock:
            if not self._eligible(token):
                return False
            previous_stamp = self.sample['stamp'] if self.sample else token.get('initial_stamp')
            if previous_stamp is not None and stamp <= previous_stamp:
                return False
            self.sample = dict(pose=list(pose), stamp=stamp, captured=captured, key=token['key'])
            self.error = None
        return True

    def _run(self, token):
        try:
            while time.monotonic() < self.demand_until:
                with self.app.lock:
                    if not self._eligible(token):
                        return
                try:
                    self._collect(token)
                except Exception as exc:
                    with self.app.lock:
                        if self.token is token:
                            self.error = str(exc)
                time.sleep(.25)
        finally:
            with self.app.lock:
                if self.token is token:
                    self.thread = None

    def decorate(self, view):
        """Called under app.lock; starts bounded read-only work off that lock."""
        if not view:
            return view
        a = self.autonomy
        now = time.monotonic()
        result = dict(view)
        result['capture_id'] = a.capture_id
        scan = a.scan or {}
        deadline = scan.get('_valid_until')
        captured = deadline - .65 if isinstance(deadline, (int, float)) and math.isfinite(deadline) else None
        age = max(0., now - captured) if captured is not None else None
        observation = dict(live=False, age=round(age, 2) if age is not None else None,
                           stamp=scan.get('stamp'), frame='slam_session', capture_id=a.capture_id,
                           message='显示最后观测；采集结束后的地图定位不混入本轮坐标系')
        if a.active:
            self.token = None
            observation.update(live=bool(age is not None and age < 1.5),
                               message='探索位置实时更新' if age is not None and age < 1.5 else '等待新观测，当前位置尚未更新')
        elif a.state == 'paused' and self.app.mapping and a.capture_id and a.context:
            key = (a.generation, a.capture_id)
            if not self.token or self.token['key'] != key:
                self.token = dict(key=key, generation=a.generation, capture_id=a.capture_id,
                                  context=a.context,
                                  role=self.app.hub.connections.value.get('sensor_source', 'nos'),
                                  initial_stamp=scan.get('stamp'))
                self.sample = None
                self.error = None
            self.demand_until = now + 2.5
            if self.thread is None or not self.thread.is_alive():
                self.thread = threading.Thread(target=self._run, args=(self.token,), daemon=True)
                self.thread.start()
            if self.sample:
                age = max(0., now - self.sample['captured'])
                result['pose'] = list(self.sample['pose'])
                observation.update(live=age < 1.5 and not self.error, age=round(age, 2), stamp=self.sample['stamp'])
            observation['message'] = (self.error or '手动接管中，位置实时更新；背景为最近探索观测') if observation['live'] else (self.error or '手动接管中，等待同一采集的位置反馈')
            # This is a historical path, never a preview of a resumed command.
            result['path'] = []
        else:
            self.token = None
            if self.sample and self.sample['key'] == (a.generation, a.capture_id):
                result['pose'] = list(self.sample['pose'])
                observation.update(age=round(max(0., now-self.sample['captured']), 2), stamp=self.sample['stamp'])
        result['observation'] = observation
        return result

"""Background short-bypass planning; never touches actuators.

The control loop keeps issuing its current validated command while a bounded
local search runs on one worker thread. A result is only a candidate: the
trajectory follower checks every edge again on a newer scan before any command
changes, so a stale result can be discarded without a stop.

Only the newest request is kept. Results carry their request age and pose so
the caller can reject one that no longer describes the robot's situation.
"""
import threading
import time

from m20.navigation_detour import plan_local_detour

SNAPSHOT_KEYS = ('pose', 'floor', 'obstacles', 'obstacle_bounds', 'measured_floor',
                 'geometry_version', '_valid_until', 'age', 'stamp', 'resolution', '_velocity_body')


def _snapshot(scan):
    """A private copy of the fields local search reads, not the whole frame."""
    snapshot = {key: scan[key] for key in SNAPSHOT_KEYS if key in scan}
    for key in ('floor', 'obstacles', 'obstacle_bounds', 'measured_floor'):
        if key in snapshot:
            snapshot[key] = list(snapshot[key])
    return snapshot


class BypassPlanner:
    def __init__(self, budget=.10, clock=time.monotonic):
        self.budget = float(budget)
        self.clock = clock
        self._lock = threading.Lock()
        self._request = None
        self._result = None
        self._thread = None
        self.submitted = 0
        self.completed = 0
        self.dropped = 0
        self.failed = 0
        self.rejected = 0
        self.last = None

    def submit(self, scan, goal, *, tried=(), avoid_axes=None, pose=None, now=None):
        request = dict(scan=_snapshot(scan), goal=(float(goal[0]), float(goal[1])),
                       tried=list(tried), avoid_axes=avoid_axes,
                       pose=tuple(pose[:3]) if pose is not None else None,
                       at=self.clock() if now is None else float(now))
        with self._lock:
            if self._request is not None:
                self.dropped += 1
            self._request = request
            self.submitted += 1
            if self._thread is None:
                self._thread = threading.Thread(target=self._work, daemon=True, name='bypass-planner')
                self._thread.start()
        return True

    def _work(self):
        while True:
            with self._lock:
                request = self._request
                self._request = None
            if request is None:
                with self._lock:
                    if self._request is None:
                        self._thread = None
                        return
                continue
            try:
                result = plan_local_detour(request['scan'], request['goal'],
                                           tried=request['tried'], avoid_axes=request['avoid_axes'],
                                           budget=self.budget)
            except (ValueError, TypeError):
                result = None
            with self._lock:
                if result is None:
                    self.failed += 1
                else:
                    self.completed += 1
                self._result = (request, result)

    def poll(self, *, pose=None, max_age=.55, max_pose_m=.45):
        """Return one candidate or None; never blocks and never repeats it."""
        with self._lock:
            if self._result is None:
                return None
            request, result = self._result
            self._result = None
        if result is None:
            return None
        age = self.clock() - request['at']
        if age > max_age:
            self.last = 'stale'
            self.rejected += 1
            return None
        if pose is not None and request.get('pose') is not None:
            moved = max(abs(a - b) for a, b in zip(pose[:2], request['pose'][:2]))
            if moved > max_pose_m:
                self.last = 'pose_moved'
                self.rejected += 1
                return None
        self.last = 'accepted'
        return dict(result, request_age_s=round(age, 3), request_pose=request.get('pose'))

    def cancel(self):
        with self._lock:
            self._request = None
            self._result = None

    def status(self):
        with self._lock:
            return dict(submitted=self.submitted, completed=self.completed, dropped=self.dropped,
                        failed=self.failed, rejected=self.rejected, pending=self._request is not None,
                        last=self.last)

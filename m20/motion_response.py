"""Learned per-axis motion response shared by navigation and exploration.

The model keeps a conservative default until enough measured samples exist and
every learned value is bounded. It never changes footprint checks, trajectory
validation or axis limits: it only adjusts prediction, braking distance and the
start command floor.

Version 2 adds three measured quantities next to the steady-state gain:

- start buckets: command magnitude -> did the body actually move
- latency: command start -> first measured motion
- brake: speed decay after the command returns to zero

Version 1 files still load; their rows are upgraded when saved again.
"""
import math


AXES = ('forward', 'backward', 'left', 'right', 'turn_left', 'turn_right')
MODES = ('0', '2')
MIN_SAMPLES = 12
MIN_START_TRIES = 10
MIN_LATENCY_SAMPLES = 5
MIN_BRAKE_SAMPLES = 6
START_LIMITS = (.12, .20, .28, .36, .50)
START_RATE = .6
MAX_DEADBAND = .35
MAX_LATENCY = .60
MIN_DECEL = .10
MAX_DECEL = 1.20


def _row(count=0, gain=1., error=0., nominal=0., start=None, latency=None, brake=None):
    return dict(count=int(count), gain=float(gain), gain_error=float(error),
                nominal_error=float(nominal), start=dict(start or {}),
                latency=list(latency or (0, 0., 0.)), brake=list(brake or (0, .5, 0.)))


def _number(value, low, high):
    return type(value) in (int, float) and math.isfinite(value) and low <= value <= high


class MotionResponse:
    def __init__(self, saved=None):
        self.profiles = {}
        self.last_sample = -math.inf
        self.dirty = False
        self._load(saved)

    def _load(self, saved):
        if not isinstance(saved, dict):
            return
        profiles = saved.get('profiles')
        if not isinstance(profiles, dict):
            return
        for mode in MODES:
            rows = profiles.get(mode)
            if not isinstance(rows, dict):
                continue
            for name in AXES:
                row = self._parse_row(rows.get(name))
                if row is not None:
                    self.profiles.setdefault(mode, {})[name] = row

    @staticmethod
    def _parse_row(value):
        if isinstance(value, list) and len(value) == 4:
            # Version 1: [count, gain, gain_error, nominal_error]
            if (type(value[0]) is not int or value[0] < 1
                    or not all(_number(v, -.001, 3.) for v in value[1:])
                    or not .3 <= value[1] <= 2.
                    or not 0 <= min(value[2:]) <= max(value[2:]) <= 1.7):
                return None
            return _row(*value)
        if not isinstance(value, dict):
            return None
        count = value.get('count')
        if type(count) is not int or count < 1:
            return None
        gain = value.get('gain')
        error = value.get('gain_error')
        nominal = value.get('nominal_error')
        if not (_number(gain, .3, 2.) and _number(error, 0., 1.7)
                and _number(nominal, 0., 1.7) and error <= 1.7 and nominal <= 1.7):
            return None
        row = _row(count, gain, error, nominal)
        start = value.get('start')
        if isinstance(start, dict):
            for key, bucket in start.items():
                if key not in [str(v) for v in START_LIMITS] or not isinstance(bucket, list):
                    continue
                if (len(bucket) == 2 and type(bucket[0]) is int and type(bucket[1]) is int
                        and 0 <= bucket[1] <= bucket[0] <= 1_000_000):
                    row['start'][key] = [bucket[0], bucket[1]]
        latency = value.get('latency')
        if (isinstance(latency, list) and len(latency) == 3 and type(latency[0]) is int
                and 0 <= latency[0] <= 1_000_000 and _number(latency[1], 0., MAX_LATENCY)
                and _number(latency[2], 0., MAX_LATENCY)):
            row['latency'] = [latency[0], latency[1], latency[2]]
        brake = value.get('brake')
        if (isinstance(brake, list) and len(brake) == 3 and type(brake[0]) is int
                and 0 <= brake[0] <= 1_000_000 and _number(brake[1], MIN_DECEL, MAX_DECEL)
                and _number(brake[2], 0., MAX_DECEL)):
            row['brake'] = [brake[0], brake[1], brake[2]]
        return row

    def _get(self, mode, name, create=False):
        key = str(mode)
        rows = self.profiles.get(key)
        if rows is None:
            if not create:
                return None
            rows = self.profiles[key] = {}
        row = rows.get(name)
        if row is None and create:
            row = rows[name] = _row()
        return row

    def observe(self, mode, command, stamp, feedback):
        """One steady, transmitted single-axis interval and its measured response."""
        if mode not in (0, 2) or command is None or stamp is None:
            return
        axes, since, sent = command
        if not since + .32 <= stamp <= sent + .10 or stamp - self.last_sample < .16:
            return
        moving = [i for i, value in enumerate(axes) if abs(value) >= .12]
        if len(moving) != 1:
            return
        i = moving[0]
        if (any(abs(value) > .015 for j, value in enumerate(axes) if j != i)
                or any(abs(value) > .10 for j, value in enumerate(feedback) if j != i)):
            return
        ratio = feedback[i] / axes[i]
        # Stalls, direction changes and implausible jumps cannot train a gain.
        if not .3 <= ratio <= 2. or feedback[i] * axes[i] <= .01:
            return
        self.last_sample = stamp
        name = AXES[2 * i + (axes[i] < 0)]
        row = self._get(mode, name, create=True)
        gain_error = abs(ratio - row['gain'])
        nominal_error = abs(ratio - 1.)
        row['count'] += 1
        row['gain'] = .85 * row['gain'] + .15 * ratio
        row['gain_error'] = .9 * row['gain_error'] + .1 * gain_error
        row['nominal_error'] = .9 * row['nominal_error'] + .1 * nominal_error
        self.dirty = True

    def observe_start(self, mode, axis, magnitude, moved):
        """One single-axis command attempt and whether the body actually moved."""
        if mode not in (0, 2) or axis not in AXES:
            return
        try:
            magnitude = float(magnitude)
        except (TypeError, ValueError):
            return
        if not math.isfinite(magnitude) or not .05 <= magnitude <= 1.:
            return
        limit = next((value for value in START_LIMITS if magnitude <= value + 1e-9), START_LIMITS[-1])
        row = self._get(mode, axis, create=True)
        bucket = row['start'].get(str(limit)) or [0, 0]
        row['start'][str(limit)] = [bucket[0] + 1, bucket[1] + int(bool(moved))]
        self.dirty = True

    def observe_latency(self, mode, axis, seconds):
        """Command start to the first measured motion on that axis."""
        if mode not in (0, 2) or axis not in AXES:
            return
        try:
            seconds = float(seconds)
        except (TypeError, ValueError):
            return
        if not math.isfinite(seconds) or not 0. <= seconds <= MAX_LATENCY:
            return
        row = self._get(mode, axis, create=True)
        count, mean, error = row['latency']
        if count == 0:
            row['latency'] = [1, seconds, 0.]
        else:
            row['latency'] = [count + 1, .85 * mean + .15 * seconds,
                              .9 * error + .1 * abs(seconds - mean)]
        self.dirty = True

    def observe_brake(self, mode, axis, v0, v1, dt):
        """Measured speed decay between two frames after the command was zero."""
        if mode not in (0, 2) or axis not in AXES:
            return
        try:
            v0, v1, dt = float(v0), float(v1), float(dt)
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (v0, v1, dt)) or not 0 < dt <= .5:
            return
        if v0 <= .15 or v1 < 0. or v1 >= v0:
            return
        decel = (v0 - v1) / dt
        if not MIN_DECEL <= decel <= MAX_DECEL:
            return
        row = self._get(mode, axis, create=True)
        count, mean, error = row['brake']
        if count == 0:
            row['brake'] = [1, decel, 0.]
        else:
            row['brake'] = [count + 1, .85 * mean + .15 * decel,
                            .9 * error + .1 * abs(decel - mean)]
        self.dirty = True

    @staticmethod
    def _ready(row):
        return row['count'] >= MIN_SAMPLES and row['gain_error'] + .025 < .85 * row['nominal_error']

    def gains(self, mode):
        """Freeze these scoring gains when a job starts."""
        return {name: row['gain'] for name, row in self.profiles.get(str(mode), {}).items()
                if self._ready(row)}

    def deadband(self, mode, axis, default=.20):
        """Smallest command magnitude that started motion reliably."""
        row = self._get(mode, axis)
        if row is None:
            return default
        for limit in START_LIMITS:
            bucket = row['start'].get(str(limit))
            if not bucket or bucket[0] < MIN_START_TRIES:
                continue
            if bucket[1] / bucket[0] >= START_RATE:
                return min(MAX_DEADBAND, max(default, limit))
        return default

    def braking_decel(self, mode, default=.5):
        rows = self.profiles.get(str(mode), {})
        weight = total = 0.
        for row in rows.values():
            count, decel, error = row['brake']
            if count >= MIN_BRAKE_SAMPLES and error < .12:
                weight += count
                total += count * decel
        return total / weight if weight >= MIN_BRAKE_SAMPLES else default

    def latency(self, mode, default=.15):
        rows = self.profiles.get(str(mode), {})
        weight = total = 0.
        for row in rows.values():
            count, mean, error = row['latency']
            if count >= MIN_LATENCY_SAMPLES and error < .08:
                weight += count
                total += count * mean
        return total / weight if weight >= MIN_LATENCY_SAMPLES else default

    def status(self):
        modes = {}
        for mode, rows in self.profiles.items():
            modes[mode] = {name: dict(samples=row['count'],
                                      gain=round(row['gain'], 3) if self._ready(row) else None,
                                      start_samples=sum(b[0] for b in row['start'].values()),
                                      deadband=round(self.deadband(mode, name), 3) if row['start'] else None,
                                      latency=round(row['latency'][1], 3) if row['latency'][0] >= MIN_LATENCY_SAMPLES else None,
                                      brake_decel=round(row['brake'][1], 3) if row['brake'][0] >= MIN_BRAKE_SAMPLES else None)
                           for name, row in rows.items()}
        return dict(version=2, modes=modes, deadband_default=.20,
                    braking_decel_default=.5, latency_default=.15)

    def record(self):
        return dict(version=2, profiles=self.profiles)


def _speed(values):
    return math.hypot(values[0], values[1])


class MotionTracker:
    """Pair transmitted single-axis intervals with measured feedback.

    Read-only observer: it never sends a command and never relaxes a safety
    check. One instance belongs to one executor, fed once per fresh scan.
    """
    def __init__(self, model, mode, *, min_motion=.05, start_window=.5):
        self.model = model
        self.mode = mode
        self.min_motion = float(min_motion)
        self.start_window = float(start_window)
        self.run = None
        self.start_pending = None
        self.braking = None
        self.last_stamp = None

    def update(self, command, stamp, feedback, pose=None):
        if self.model is None or self.mode not in (0, 2):
            return
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        try:
            values = [float(feedback[0]), float(feedback[1]), float(feedback[2])]
        except (TypeError, ValueError, IndexError):
            return
        if not all(math.isfinite(value) for value in values):
            return
        axes = tuple(command[0]) if command else (0., 0., 0.)
        if len(axes) != 3:
            return
        since = command[1] if command else stamp
        active = any(abs(value) > .02 for value in axes)
        index = max(range(3), key=lambda j: abs(axes[j]))
        name = AXES[2 * index + (axes[index] < 0)] if active else None
        if active and axes != self.run:
            # Track whether the body was still during this command interval.
            # A residual velocity from the previous command is not a start.
            self.start_pending = (name, since, abs(axes[index]), False)
        self.run = axes
        if active:
            self.model.observe(self.mode, command, stamp, values)
            # Start window: one attempt per command interval.
            if self.start_pending is not None:
                axis, began, magnitude, still = self.start_pending
                sign = 1. if axes[index] >= 0 else -1.
                moving = values[index] * sign >= self.min_motion
                if abs(values[index]) < self.min_motion:
                    still = True
                    self.start_pending = (axis, began, magnitude, still)
                if moving and still:
                    self.model.observe_latency(self.mode, axis, max(0., stamp - began))
                    self.model.observe_start(self.mode, axis, magnitude, True)
                    self.start_pending = None
                elif stamp - began >= self.start_window:
                    # Only a window that began from rest is a failed start.
                    if still:
                        self.model.observe_start(self.mode, axis, magnitude, False)
                    self.start_pending = None
            speed = _speed(values)
            if speed > .05:
                self.braking = [name, speed, stamp]
            else:
                self.braking = None
        else:
            self.start_pending = None
            if self.braking is not None:
                axis, previous, at = self.braking
                speed = _speed(values)
                dt = stamp - at
                if 0 < dt <= .5 and previous - speed > .01:
                    self.model.observe_brake(self.mode, axis, previous, speed, dt)
                self.braking = [axis, speed, stamp] if speed > .03 else None
        self.last_stamp = stamp


def mode_for(app, owner=None):
    """Controller mode used for learning; None when no valid mode is known."""
    mode = getattr(owner, 'motion_mode', None) if owner is not None else None
    if mode in (0, 2):
        return mode
    basic = getattr(app, 'basic', None)
    if isinstance(basic, dict):
        try:
            mode = int(basic.get('ControlUsageMode'))
        except (TypeError, ValueError):
            return None
        return mode if mode in (0, 2) else None
    return None


def track(owner, app, scan, yaw_rate=0., *, mode=None, ready=True):
    """Feed one fresh scan to the executor's shared motion tracker.

    Read-only: no command is changed and no safety check is relaxed. Returns
    the tracker when a sample was consumed, otherwise None.
    """
    if not ready:
        return None
    model = getattr(app, 'motion_response', None)
    link = getattr(app, 'link', None)
    if model is None or link is None:
        return None
    if mode is None:
        mode = mode_for(app, owner)
    if mode not in (0, 2):
        return None
    axis_run = getattr(link, 'axis_run', None)
    if not callable(axis_run):
        return None
    try:
        command = axis_run()
    except (AttributeError, TypeError):
        return None
    velocity = scan.get('_velocity_body')
    if not isinstance(velocity, (list, tuple)) or len(velocity) != 2:
        return None
    tracker = getattr(owner, 'motion_tracker', None)
    if tracker is None or tracker.mode != mode or tracker.model is not model:
        tracker = MotionTracker(model, mode)
        owner.motion_tracker = tracker
    tracker.update(command, scan.get('_motion_feedback_at'), [*velocity, yaw_rate])
    return tracker

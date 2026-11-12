"""Small per-GOS response model fitted from paired commands and odometry."""
import math


AXES = ('forward', 'backward', 'left', 'right', 'turn_left', 'turn_right')
MIN_SAMPLES = 12


class MotionResponse:
    def __init__(self, saved=None):
        self.profiles = {}
        self.last_sample = -math.inf
        self.dirty = False
        if not isinstance(saved, dict) or saved.get('version') != 1:
            return
        profiles = saved.get('profiles')
        if not isinstance(profiles, dict):
            return
        for mode in ('0', '2'):
            rows = profiles.get(mode)
            if not isinstance(rows, dict):
                continue
            for name in AXES:
                row = rows.get(name)
                if (isinstance(row, list) and len(row) == 4 and type(row[0]) is int and row[0] >= 1
                        and all(type(v) in (int, float) and math.isfinite(v) for v in row[1:])
                        and .3 <= row[1] <= 2. and 0 <= min(row[2:]) <= max(row[2:]) <= 1.7):
                    self.profiles.setdefault(mode, {})[name] = list(row)

    def observe(self, mode, command, stamp, feedback):
        """One steady, transmitted single-axis interval and its measured response."""
        if mode not in (0, 2) or command is None or stamp is None:
            return
        axes, since, sent = command
        if not since + .32 <= stamp <= sent + .10 or stamp-self.last_sample < .16:
            return
        moving = [i for i, value in enumerate(axes) if abs(value) >= .12]
        if len(moving) != 1:
            return
        i = moving[0]
        if (any(abs(value) > .015 for j, value in enumerate(axes) if j != i)
                or any(abs(value) > .10 for j, value in enumerate(feedback) if j != i)):
            return
        ratio = feedback[i]/axes[i]
        # Stalls, direction changes and implausible jumps cannot train a gain.
        if not .3 <= ratio <= 2. or feedback[i]*axes[i] <= .01:
            return
        self.last_sample = stamp
        name = AXES[2*i+(axes[i]<0)]
        rows = self.profiles.setdefault(str(mode), {})
        count, gain, error, nominal = rows.get(name, (0, 1., 0., 0.))
        gain_error = abs(ratio-gain)
        nominal_error = abs(ratio-1.)
        rows[name] = [count+1, .85*gain+.15*ratio,
                      .9*error+.1*gain_error, .9*nominal+.1*nominal_error]
        self.dirty = True

    @staticmethod
    def _ready(row):
        count, _, error, nominal = row
        return count >= MIN_SAMPLES and error+.025 < .85*nominal

    def gains(self, mode):
        """Freeze these scoring gains when a job starts."""
        return {name:row[1] for name,row in self.profiles.get(str(mode), {}).items() if self._ready(row)}

    def status(self):
        return dict(version=1, modes={mode:{name:dict(samples=row[0],
                    gain=round(row[1],3) if self._ready(row) else None)
                    for name,row in axes.items()} for mode,axes in self.profiles.items()})

    def record(self):
        return dict(version=1, profiles=self.profiles)

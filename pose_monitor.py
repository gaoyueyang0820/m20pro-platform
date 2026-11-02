"""Display-only pose stabilization and raw-pose diagnostics.

This module performs no I/O and never produces a control pose. The caller must
keep using its original raw localization for navigation/control. A stationary
candidate combines current zero commands with fresh measured motion; it is not
proof of physical immobility. Drift warnings therefore also cover slow movement
that cannot be distinguished from the measured velocity bias.

All ages and ``now`` are seconds from a monotonic clock. ``relocation_epoch`` is
a caller-owned identity/counter for the coordinate-system/localization session;
do not invent a new epoch every frame. Confidence is exposed unchanged. Its
optional threshold is a display eligibility setting, not a control interlock.
"""
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
from statistics import median
import threading
import time


@dataclass(frozen=True)
class PoseMonitorConfig:
    stationary_window_s: float = 1.2
    stationary_speed_m_s: float = .08
    stationary_yaw_rate_rad_s: float = .12
    command_zero_epsilon: float = .001
    pose_ttl_s: float = 2.
    motion_ttl_s: float = .5
    max_update_gap_s: float = 1.5
    recovery_window_s: float = .6
    deadband_m: float = .01
    noise_radius_m: float = .05
    smoothing_tau_s: float = .6
    yaw_deadband_rad: float = math.radians(.5)
    yaw_noise_radius_rad: float = math.radians(3)
    drift_distance_m: float = .08
    drift_yaw_rad: float = math.radians(5)
    drift_hold_s: float = 1.
    jump_distance_m: float = .35
    jump_yaw_rad: float = math.radians(20)
    warning_hold_s: float = 2.
    min_confidence: float = None


def _number(value):
    if isinstance(value,bool):return None
    try:value=float(value)
    except (TypeError,ValueError):return None
    return value if math.isfinite(value) else None


def _wrap(value):
    return math.atan2(math.sin(value),math.cos(value))


def _axes(value, names):
    if isinstance(value,dict):
        values=[next((value[k] for k in options if k in value),None) for options in names]
    elif isinstance(value,(list,tuple)) and len(value)==3:values=value
    else:return None
    result=[_number(v) for v in values]
    return result if all(v is not None for v in result) else None


class PoseMonitor:
    """Use one instance per robot/map context; safe for serialized snapshots.

    ``update`` accepts the unmodified ``PosX/PosY/Yaw/Location/Confidence`` map
    pose, normalized command axes, and measured LinearX/LinearY/OmegaZ (or
    vx/vy/wz). ``display_pose=None`` means no current usable marker. A historical
    marker, when available, is returned only in health.last_known with
    usable=False. Raw input is copied verbatim in raw_pose, never rewritten.
    """
    def __init__(self, **settings):
        self.config=PoseMonitorConfig(**settings)
        for key,value in vars(self.config).items():
            if key=='min_confidence':
                if value is not None and (_number(value) is None or not 0<=value<=1):raise ValueError('min_confidence must be 0..1 or None')
            elif _number(value) is None or value<=0:raise ValueError(key+' must be positive and finite')
        if self.config.deadband_m>self.config.noise_radius_m:raise ValueError('deadband exceeds display noise radius')
        self._lock=threading.RLock();self._last_update=None;self._quiet_since=None
        self._epoch_set=False;self._epoch=None;self._last_known=None;self._last_known_at=None
        self._reset_pose()

    def _reset_pose(self):
        self._recovering=True;self._recovery_since=None;self._recovery_samples=0;self._recovery_source=None
        self._last_raw=None;self._last_source=None;self._display=None
        self._anchor=None;self._history=deque(maxlen=128)
        self._drift_since=None;self._drift_active=False
        self._jump_until=0.;self._jump_detail=None

    def _result(self,raw,display,status,message,stationary,now,metrics,filtered=False,severity='info'):
        health=dict(status=status,message=message,severity=severity,display_usable=display is not None,
                    filtered=filtered,display_only=True,control_pose_source='raw_pose',
                    location=raw.get('Location') if isinstance(raw,dict) else None,
                    confidence=_number(raw.get('Confidence')) if isinstance(raw,dict) else None,
                    relocation_epoch=deepcopy(self._epoch),**metrics)
        if self._last_known is not None and (display is None or severity=='warning'):
            health['last_known']=dict(pose=deepcopy(self._last_known),age_s=max(0.,now-self._last_known_at),usable=False)
        return dict(display_pose=deepcopy(display),health=health,stationary=bool(stationary),raw_pose=deepcopy(raw))

    def update(self, raw_pose, command_velocity, motion, *, pose_age, motion_age, now=None, relocation_epoch=None):
        with self._lock:
            return self._update(raw_pose,command_velocity,motion,pose_age,motion_age,
                                time.monotonic() if now is None else now,relocation_epoch)

    def _update(self,raw_pose,command_velocity,motion,pose_age,motion_age,now,epoch):
        c=self.config;raw=deepcopy(raw_pose);now=_number(now)
        if now is None:raise ValueError('now must be a finite monotonic timestamp')
        pose_age=_number(pose_age);motion_age=_number(motion_age)
        metrics=dict(pose_age_s=pose_age,motion_age_s=motion_age)
        if self._last_update is not None and now<self._last_update:
            self._reset_pose();self._quiet_since=None;self._last_update=now
            return self._result(raw,None,'clock_reset','监测时间基准变化，等待新鲜定位',False,now,metrics,severity='unavailable')
        dt=0. if self._last_update is None else now-self._last_update
        gap=self._last_update is not None and dt>c.max_update_gap_s;self._last_update=now
        command=_axes(command_velocity,[('forward','f'),('side','s'),('yaw','y')])
        measured=_axes(motion,[('LinearX','vx'),('LinearY','vy'),('OmegaZ','wz')])
        motion_fresh=measured is not None and motion_age is not None and 0<=motion_age<=c.motion_ttl_s
        speed=math.hypot(*measured[:2]) if measured else None
        command_zero=command is not None and all(abs(v)<=c.command_zero_epsilon for v in command)
        quiet=bool(command_zero and motion_fresh and speed<c.stationary_speed_m_s and abs(measured[2])<c.stationary_yaw_rate_rad_s)
        if not quiet or gap:
            self._quiet_since=now if quiet else None;self._anchor=None;self._history.clear()
            self._drift_since=None;self._drift_active=False
        elif self._quiet_since is None:self._quiet_since=now
        epoch_changed=self._epoch_set and epoch!=self._epoch
        self._epoch_set=True;self._epoch=deepcopy(epoch)
        if epoch_changed:
            self._reset_pose();self._quiet_since=now if quiet else None
        stationary=quiet and now-self._quiet_since>=c.stationary_window_s
        metrics.update(command_zero=command_zero,motion_fresh=motion_fresh,measured_speed_m_s=speed,
                       measured_yaw_rate_rad_s=measured[2] if measured else None,
                       stationary_elapsed_s=now-self._quiet_since if quiet else 0.,epoch_changed=epoch_changed)
        values=[_number(raw.get(k)) for k in ['PosX','PosY','Yaw']] if isinstance(raw,dict) else [None]*3
        location=_number(raw.get('Location')) if isinstance(raw,dict) else None
        confidence=_number(raw.get('Confidence')) if isinstance(raw,dict) else None
        problem=None
        if not isinstance(raw,dict) or any(v is None for v in values) or ('PosZ' in raw and _number(raw['PosZ']) is None):
            problem=('invalid_pose','定位坐标无效，当前位置不可用')
        elif location!=0:problem=('localization_invalid','定位未就绪，当前位置不可用')
        elif pose_age is None or not 0<=pose_age<=c.pose_ttl_s:problem=('pose_stale','定位反馈过期或时间无效，当前位置不可用')
        elif ('Confidence' in raw and (confidence is None or not 0<=confidence<=1)) or (c.min_confidence is not None and (confidence is None or confidence<c.min_confidence)):
            problem=('confidence_unavailable','定位置信度不足或无效，当前位置不可用')
        if problem:
            self._reset_pose()
            return self._result(raw,None,*problem,stationary,now,metrics,severity='unavailable')
        x,y,yaw=values;yaw=_wrap(yaw);source_at=now-pose_age
        recovered=False
        if self._recovering:
            if self._recovery_since is None:
                self._recovery_since=now;self._quiet_since=now if quiet else None;stationary=False
            if self._recovery_source is None or source_at>self._recovery_source+1e-6:
                self._recovery_samples+=1;self._recovery_source=source_at
            if now-self._recovery_since<c.recovery_window_s or self._recovery_samples<2:
                metrics['recovery_samples']=self._recovery_samples
                return self._result(raw,None,'relocalizing','等待连续有效定位；旧位置不可作为当前位置',False,now,metrics,severity='unavailable')
            self._recovering=False;recovered=True
        new_sample=self._last_source is None or source_at>self._last_source+1e-6
        if self._last_source is not None and source_at<self._last_source-.05:
            self._reset_pose();self._quiet_since=now if quiet else None
            return self._result(raw,None,'pose_time_reversed','定位时间倒退，等待新鲜定位',False,now,metrics,severity='unavailable')
        if new_sample:
            if self._last_raw is not None and not gap:
                elapsed=max(0.,source_at-self._last_source)
                jump=math.hypot(x-self._last_raw[0],y-self._last_raw[1]);turn=abs(_wrap(yaw-self._last_raw[2]))
                expected_distance=(speed*elapsed*1.5) if motion_fresh else 0.
                expected_turn=abs(measured[2])*elapsed*1.5 if motion_fresh else 0.
                if jump>c.jump_distance_m+expected_distance or turn>c.jump_yaw_rad+expected_turn:
                    self._jump_until=now+c.warning_hold_s;self._jump_detail=dict(distance_m=jump,yaw_rad=turn)
            self._last_raw=(x,y,yaw);self._last_source=source_at
            if quiet:self._history.append((now,x,y,yaw))
        if stationary and self._anchor is None:
            samples=[v for v in self._history if v[0]>=self._quiet_since]
            if samples:
                reference=samples[0][3]
                self._anchor=(median(v[1] for v in samples),median(v[2] for v in samples),
                              _wrap(reference+median(_wrap(v[3]-reference) for v in samples)))
            else:self._anchor=(x,y,yaw)
        displacement=math.hypot(x-self._anchor[0],y-self._anchor[1]) if self._anchor else 0.
        yaw_change=abs(_wrap(yaw-self._anchor[2])) if self._anchor else 0.
        if stationary:
            drifting=displacement>c.drift_distance_m or yaw_change>c.drift_yaw_rad
            if drifting:
                if self._drift_since is None:self._drift_since=now
                if now-self._drift_since>=c.drift_hold_s:self._drift_active=True
            elif displacement<c.drift_distance_m/2 and yaw_change<c.drift_yaw_rad/2:
                self._drift_since=None;self._drift_active=False
        jump_warning=now<self._jump_until
        small_noise=stationary and displacement<=c.noise_radius_m and yaw_change<=c.yaw_noise_radius_rad
        display=deepcopy(raw);filtered=False
        if small_noise and not self._drift_active and not jump_warning:
            previous=self._display
            if previous is None or math.hypot(previous['PosX']-self._anchor[0],previous['PosY']-self._anchor[1])>c.noise_radius_m or abs(_wrap(previous['Yaw']-self._anchor[2]))>c.yaw_noise_radius_rad:
                previous=raw
            alpha=1-math.exp(-max(0.,dt)/c.smoothing_tau_s)
            if math.hypot(x-previous['PosX'],y-previous['PosY'])<=c.deadband_m:
                display['PosX']=previous['PosX'];display['PosY']=previous['PosY']
            else:
                display['PosX']=previous['PosX']+alpha*(x-previous['PosX'])
                display['PosY']=previous['PosY']+alpha*(y-previous['PosY'])
            yaw_delta=_wrap(yaw-previous['Yaw'])
            display['Yaw']=_wrap(previous['Yaw'] if abs(yaw_delta)<=c.yaw_deadband_rad else previous['Yaw']+alpha*yaw_delta)
            filtered=True
        self._display=deepcopy(display)
        metrics.update(anchor_displacement_m=displacement,anchor_yaw_change_rad=yaw_change,
                       display_offset_m=math.hypot(display['PosX']-x,display['PosY']-y),
                       jump=deepcopy(self._jump_detail) if jump_warning else None)
        if jump_warning:status,message,severity='pose_jump','定位反馈出现明显突变，显示原始位置；请核对定位或实际移动','warning'
        elif self._drift_active:status,message,severity='drift_suspected','停稳反馈下定位持续偏移，可能存在定位漂移或未检测到的慢速移动','warning'
        elif not motion_fresh or command is None:status,message,severity='motion_unavailable','运动反馈不可用，未判断停稳；显示原始定位','warning'
        elif stationary and not small_noise:status,message,severity='stationary_pose_change','位置变化超过小幅噪声范围，显示原始定位','notice'
        elif recovered:status,message,severity='recovered','已重新收到连续有效定位，使用新坐标显示','info'
        elif filtered:status,message,severity='stationary_filtered','停稳反馈一致，仅稳定显示标记；原始定位保留','info'
        else:status,message,severity='tracking','显示原始定位，尚未确认停稳或正在移动','info'
        if severity=='info':self._last_known=deepcopy(display);self._last_known_at=source_at
        return self._result(raw,display,status,message,stationary,now,metrics,filtered,severity)

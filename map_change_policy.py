"""Pure evidence policy for versioned map changes; no robot or file I/O.

This is an initial, configurable policy, NOT tunnel validation. Thresholds must
be calibrated with independent ground truth for the installed sensors and site.
Registration metrics refer to NON-FLOOR structure in the already mapped overlap
region, not to a whole scan containing large newly explored/unknown areas.

An observation of free space means a measured ground/surface observation with
visibility through that cell. A missing return, occlusion or unobserved cell is
not free evidence. Epochs and spatial views prevent frame rate from becoming
confidence. Candidates still require the platform's explicit activation flow.
"""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class ChangePolicy:
    epoch_s: float = 10.
    window_s: float = 120.
    realtime_ttl_s: float = 1.
    view_separation_m: float = .35
    add_epochs: int = 3
    add_span_s: float = 20.
    add_views: int = 2
    clear_epochs: int = 5
    clear_span_s: float = 40.
    clear_views: int = 3
    min_structural_overlap: float = .75
    max_structural_rmse_m: float = .12
    max_correction_m: float = .35
    max_correction_yaw_rad: float = math.radians(10)
    max_known_free_conflict_ratio: float = .08
    coherent_shift_fraction: float = .60
    coherent_shift_m: float = .15


def _policy(value):
    if value is None:return ChangePolicy()
    if isinstance(value, dict):value=ChangePolicy(**value)
    if not isinstance(value, ChangePolicy):raise ValueError('地图变化策略格式无效')
    for field in fields(value):
        number=getattr(value,field.name)
        if isinstance(number,bool) or not isinstance(number,(int,float)) or not math.isfinite(number) or number<=0:
            raise ValueError('地图变化策略阈值必须为有限正数')
    if value.window_s<max(value.add_span_s,value.clear_span_s):raise ValueError('证据窗口短于确认时长')
    return value


def _number(value, name):
    if isinstance(value,bool):raise ValueError(name+'必须为有限数值')
    try:value=float(value)
    except (TypeError,ValueError):raise ValueError(name+'必须为有限数值')
    if not math.isfinite(value):raise ValueError(name+'必须为有限数值')
    return value


def gate_alignment(metrics, policy=None):
    """Decide whether registration supports publishing a map candidate.

    Required metrics: structural_overlap (0..1), rmse_m (or rmse),
    effective_frames. Optional: localized, ambiguous, correction_m,
    correction_yaw_rad, known_free_conflict_ratio, coherent_shift_fraction,
    coherent_shift_m, temporal_change_confirmed.

    A coherent residual shift in *several* structures is evidence to recheck
    pose/registration, not proof that all those structures physically moved.
    Large local changes may proceed only after independent temporal evidence;
    this does not relax overlap, ambiguity or correction checks.
    """
    p=_policy(policy)
    def result(allowed,status,reason,review=False):
        return dict(allowed=allowed,status=status,reason=reason,requires_review=review,
                    threshold_status='initial_defaults_require_site_validation')
    if metrics.get('localized',True) is False or metrics.get('pose_jump',False):
        return result(False,'localization_untrusted','定位不可靠或位姿跳变，保留观测并重新定位')
    if metrics.get('ambiguous',False):
        return result(False,'ambiguous_registration','重复结构存在多个匹配，不能确定地图坐标')
    try:
        overlap=_number(metrics['structural_overlap'],'结构重叠')
        error=_number(metrics.get('rmse_m',metrics.get('rmse')),'结构误差')
        frames=_number(metrics['effective_frames'],'独立关键帧数')
        shift=abs(_number(metrics.get('correction_m',0),'配准平移'))
        yaw=abs(_number(metrics.get('correction_yaw_rad',0),'配准转角'))
        conflicts=_number(metrics.get('known_free_conflict_ratio',0),'旧空地冲突比例')
        coherent=_number(metrics.get('coherent_shift_fraction',0),'同向偏移比例')
        residual=abs(_number(metrics.get('coherent_shift_m',0),'同向残差'))
    except (KeyError,ValueError):
        return result(False,'insufficient_structure','缺少有效的非地面结构验证指标')
    if not 0<=overlap<=1 or not 0<=conflicts<=1 or not 0<=coherent<=1 or error<0 or frames<1 or frames!=int(frames):
        return result(False,'insufficient_structure','结构验证指标或独立关键帧数无效')
    if shift>p.max_correction_m or yaw>p.max_correction_yaw_rad:
        return result(False,'registration_correction_large','配准修正较大，不能直接解释为环境变化',True)
    if coherent>=p.coherent_shift_fraction and residual>p.coherent_shift_m:
        return result(False,'coherent_structure_shift','多处结构出现同向偏移，应先复核定位与配准',True)
    if overlap<p.min_structural_overlap or error>p.max_structural_rmse_m:
        return result(False,'structure_conflict','旧图覆盖区结构未对齐，不能以地面匹配率替代墙线核对',True)
    confirmed=metrics.get('temporal_change_confirmed',False) is True
    if conflicts>p.max_known_free_conflict_ratio and not confirmed:
        return result(False,'unconfirmed_environment_change','旧空地出现明显结构冲突，需多时段多视角确认',True)
    if frames<2 and conflicts>.01:
        return result(False,'single_view_conflict','单个关键帧不能确认旧区域的结构变化',True)
    if confirmed and conflicts>0:
        return result(True,'confirmed_change_candidate','已有独立时序证据，生成环境变化候选供检查',True)
    return result(True,'aligned_candidate','重叠区域结构一致，可生成候选版本')


def _base(value):
    kind='obstacle'
    if isinstance(value,dict):kind=value.get('kind',kind);value=value.get('value')
    if isinstance(value,str):
        aliases={'occupied':0,'free':254,'unknown':205}
        if value not in aliases:raise ValueError('旧栅格状态无效')
        value=aliases[value]
    value=_number(value,'旧栅格值')
    if not 0<=value<=255:raise ValueError('旧栅格状态无效')
    return ('occupied' if value<90 else 'free' if value>205 else 'unknown'),kind


def _views(rows, separation):
    # Pick actual observer origins, never invent a mean observer position.
    # Each counted view comes from a different temporal epoch.
    origins=[(row['epoch'],origin) for row in rows for origin in row['origins']]
    if not origins:return 0
    pair=None;distance=-1
    for i,(epoch,a) in enumerate(origins):
        for other,b in origins[i+1:]:
            d=math.dist(a,b)
            if epoch!=other and d>=separation and d>distance:pair=((epoch,a),(other,b));distance=d
    if pair is None:return 1
    chosen=list(pair)
    # Only 3 views are currently required; bounded search avoids per-cell cubes.
    for epoch,origin in origins:
        if epoch not in {x[0] for x in chosen} and all(math.dist(origin,x[1])>=separation for x in chosen):
            chosen.append((epoch,origin))
            if len(chosen)>=3:break
    return len(chosen)


def classify_cell(base_value, observations, *, registration_ok, now=None, policy=None):
    """Classify evidence for one world-grid cell without mutating the map.

    observations: iterable of {stamp, vote: 1|-1, origin: [x,y], dynamic: bool}.
    origin may be omitted; such evidence cannot establish independent views.
    Optional trusted=False excludes an observation from persistent updates.
    Repeated points/frames within a 10-second epoch count only once; an occupied
    return wins a mixed occupied/free epoch. `dynamic` marks a person/vehicle or
    other tracked temporary object: its uninterrupted event is never promoted.

    Returns action occupy/clear/keep, classification, realtime_occupied
    (True/False/None when unavailable), and auditable evidence counts.
    No observation or an unseen region never implies clearing an old obstacle.
    """
    p=_policy(policy);base,kind=_base(base_value);raw=[]
    for observation in observations:
        stamp=_number(observation.get('stamp'),'观测时间')
        vote=observation.get('vote')
        if isinstance(vote,bool) or vote not in (1,-1):raise ValueError('变化证据需要明确占据或可通行观测')
        origin=observation.get('origin')
        if origin is not None:
            if len(origin)<2:raise ValueError('观察位置需要 XY')
            origin=tuple(_number(x,'观察位置') for x in origin[:2])
        raw.append(dict(stamp=stamp,vote=vote,origin=origin,dynamic=bool(observation.get('dynamic',False)),trusted=observation.get('trusted',True) is True))
    now=max((r['stamp'] for r in raw),default=0.) if now is None else _number(now,'当前时间')
    if any(r['stamp']>now+.001 for r in raw):raise ValueError('观测来自未来，请核对时间基准')
    recent=sorted((r for r in raw if now-p.window_s<=r['stamp']<=now),key=lambda r:r['stamp'])
    latest=recent[-1] if recent else None
    realtime=bool(latest['vote']==1) if latest and now-latest['stamp']<=p.realtime_ttl_s else None
    epochs={}
    for r in recent:
        if not r['trusted']:continue
        epoch=math.floor(r['stamp']/p.epoch_s);row=epochs.get(epoch)
        if row is None or r['vote']>row['vote']:
            row=dict(epoch=epoch,vote=r['vote'],stamp=r['stamp'],origins=[],dynamic=False);epochs[epoch]=row
        if r['vote']==row['vote']:
            if r['origin'] is not None and r['origin'] not in row['origins']:row['origins'].append(r['origin'])
            row['dynamic']=row['dynamic'] or r['dynamic'];row['stamp']=max(row['stamp'],r['stamp'])
    rows=sorted(epochs.values(),key=lambda r:r['epoch'])
    tail=[]
    for row in reversed(rows):
        if tail and row['vote']!=tail[0]['vote']:break
        tail.append(row)
    tail.reverse()
    span=tail[-1]['stamp']-tail[0]['stamp'] if tail else 0.
    evidence=dict(epochs=len(rows),consecutive_epochs=len(tail),span_s=round(span,3),views=_views(tail,p.view_separation_m),
                  dynamic_event=any(r['dynamic'] for r in tail),latest_stamp=latest['stamp'] if latest else None,
                  window_s=p.window_s)
    def result(action,classification,reason):
        return dict(action=action,classification=classification,reason=reason,realtime_occupied=realtime,
                    requires_review=action!='keep',evidence=evidence)
    if not registration_ok:return result('keep','registration_untrusted','定位或配准不可靠，仅保留观测，不更新长期地图')
    if not tail:return result('keep','unobserved','没有可信的新观测，保留旧状态')
    vote=tail[-1]['vote']
    if latest and latest['vote']!=vote:
        return result('keep','contested_latest_observation','同一时段存在相反观测，等待新的独立证据')
    if vote==1:
        if base=='occupied':return result('keep','unchanged_occupied','旧障碍仍被观测到')
        if evidence['dynamic_event']:return result('keep','dynamic_obstacle','动态目标仅用于实时避障')
        ready=len(tail)>=p.add_epochs and span>=p.add_span_s and evidence['views']>=p.add_views
        if ready:return result('occupy','fixed_obstacle_added','多时段多视角持续存在，生成固定障碍候选')
        return result('keep','transient_obstacle','当前有障碍，但长期地图证据尚不足')
    if base=='free':return result('keep','unchanged_free','原空地仍可通行')
    clear_old=base=='occupied'
    required=(p.clear_epochs,p.clear_span_s,p.clear_views) if clear_old else (p.add_epochs,p.add_span_s,p.add_views)
    ready=len(tail)>=required[0] and span>=required[1] and evidence['views']>=required[2]
    if ready:
        classification=('wall_removed_candidate' if kind=='wall' else 'obstacle_removed') if clear_old else 'explored_free_space'
        return result('clear',classification,'最近占据观测之后，独立空地证据已充分确认')
    return result('keep','removal_unconfirmed' if clear_old else 'new_free_unconfirmed','空地证据尚不足，不能把未看到当作已移除')

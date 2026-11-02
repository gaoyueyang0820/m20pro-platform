"""Local, multi-heading pose recovery from structural point clouds.

No network, file access, motion or localization commands are performed here.
The caller supplies a recent reliable position in the active map and controls
freshness, robot identity, map identity and whether the robot is stationary.
An accepted search is only a hypothesis until verify_pose accepts a NEW scan.
"""
import math
import numpy as np


MIN_POINTS = 120
MATCH_DISTANCE = .20
MIN_OVERLAP = .90
MAX_RMSE = .08
MIN_SCORE_GAP = .12


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _points(values):
    points = np.asarray(values, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] not in (3, 4) or len(points) > 2_000_000:
        raise ValueError('定位点云格式或数量无效')
    points = points[:, :3]
    return points[np.isfinite(points).all(axis=1)]


def _voxel(points, size, limit):
    if not len(points):
        return points
    _, index = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    points = points[index]
    return points[::max(1, math.ceil(len(points) / limit))]


def _body(values, frame_id):
    if frame_id != 'base_link':
        raise ValueError('自动定位需要 base_link 机身点云')
    points = _points(values)
    radius = np.linalg.norm(points[:, :2], axis=1)
    # The floor is near -0.55 m in the body frame. Do not let a dominant floor
    # plane produce a high overlap score for an otherwise incorrect pose.
    points = points[(points[:, 2] > -.30) & (points[:, 2] < 1.45) &
                    (radius > .75) & (radius < 6)]
    points = _voxel(points, .12, 1500)
    if len(points) < MIN_POINTS or np.ptp(points[:, 2]) < .25:
        raise ValueError('非地面结构点不足，不能自动确定位置')
    spread = np.linalg.eigvalsh(np.cov(points[:, :2], rowvar=False))
    if np.min(np.ptp(points[:, :2], axis=0)) < 1.2 or spread[0] < .03:
        raise ValueError('可见结构过于单一，不能唯一确定位置')
    return points


def _hint(value):
    if isinstance(value, dict):
        result = [value.get(a, value.get(b)) for a, b in
                  [('PosX', 'x'), ('PosY', 'y'), ('PosZ', 'z')]]
    else:
        result = list(value)[:3]
    try:
        result = np.asarray(result, dtype=float)
    except (ValueError, TypeError):
        raise ValueError('缺少最近一次可靠定位的位置')
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('缺少最近一次可靠定位的位置')
    return result


def _stamp(value):
    if isinstance(value, bool):
        raise ValueError('定位观测时间戳无效')
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError('定位观测时间戳无效')
    if not math.isfinite(value) or value < 0:
        raise ValueError('定位观测时间戳无效')
    return value


def _reference(values, center):
    points = _points(values)
    points = points[(np.linalg.norm(points[:, :2] - center[:2], axis=1) < 8) &
                    (points[:, 2] > center[2] - .35) &
                    (points[:, 2] < center[2] + 1.7)]
    points = _voxel(points, .08, 60000)
    if len(points) < MIN_POINTS:
        raise ValueError('最近可靠位置附近没有足够原图结构')
    return points


class _Index:
    def __init__(self, points):
        import cv2
        self.points = np.ascontiguousarray(points, dtype=np.float32)
        self.tree = cv2.flann_Index(self.points, dict(algorithm=1, trees=4))

    def query(self, points):
        indices, squared = self.tree.knnSearch(
            np.ascontiguousarray(points, dtype=np.float32), 1, params=dict(checks=128))
        return np.sqrt(np.maximum(0, squared[:, 0])), indices[:, 0]


def _transform(points, position, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return points @ rotation.T + position


def _quality(points, index):
    distance, _ = index.query(points)
    good = distance < MATCH_DISTANCE
    overlap = float(good.mean())
    rmse = float(np.sqrt(np.mean(distance[good] ** 2))) if good.any() else 99.
    return dict(overlap=overlap, rmse_m=rmse, score=1-overlap+2*rmse)


def _solve(body, reference, index, position, yaw, hint, max_translation):
    position = np.array(position, dtype=float, copy=True)
    for _ in range(45):
        world = _transform(body, position, yaw)
        distance, nearest = index.query(world)
        good = distance < min(.65, float(np.quantile(distance, .75)))
        if good.sum() < 100:
            break
        a, b = world[good], reference[nearest[good]]
        ac = a[:, :2] - a[:, :2].mean(axis=0)
        bc = b[:, :2] - b[:, :2].mean(axis=0)
        u, _, vt = np.linalg.svd(ac.T @ bc)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        turn = math.atan2(rotation[1, 0], rotation[0, 0])
        shift = b[:, :2].mean(axis=0) - a[:, :2].mean(axis=0) @ rotation.T
        position[:2] = position[:2] @ rotation.T + shift
        position[2] += float(np.median(b[:, 2] - a[:, 2]))
        yaw += turn
        if abs(turn) < .0001 and np.linalg.norm(shift) < .001:
            break
    if np.linalg.norm(position[:2] - hint[:2]) > max_translation or abs(position[2] - hint[2]) > .25:
        return None
    pose = dict(PosX=float(position[0]), PosY=float(position[1]),
                PosZ=float(position[2]), Yaw=_wrap(yaw))
    return dict(pose=pose, quality=_quality(_transform(body, position, yaw), index))


def _same_place(a, b):
    a, b = a['pose'], b['pose']
    return (math.hypot(a['PosX']-b['PosX'], a['PosY']-b['PosY']) < .30 and
            abs(_wrap(a['Yaw']-b['Yaw'])) < .15)


def match_pose(body_points, reference_points, hint, *, stamp, frame_id='base_link',
               max_translation=1.3):
    """Return a unique structural pose hypothesis, or raise ValueError.

    hint accepts {PosX, PosY, PosZ}, {x, y, z} or an xyz sequence. It is a
    search prior, not a successful localization. Input point arrays are Nx3
    (Nx4 with an unused intensity column is also accepted).
    """
    stamp = _stamp(stamp)
    if not math.isfinite(max_translation) or not .1 <= max_translation <= 3:
        raise ValueError('局部定位搜索范围无效')
    center = _hint(hint)
    body = _body(body_points, frame_id)
    reference = _reference(reference_points, center)
    index = _Index(reference)
    rows = []
    for yaw in np.arange(-math.pi, math.pi, math.pi/12):
        result = _solve(body, reference, index, center, float(yaw), center, max_translation)
        if result is not None:
            rows.append(result)
    if not rows:
        raise ValueError('当前位置附近未找到可信的地图匹配')
    rows.sort(key=lambda row: row['quality']['score'])
    # Check nearby translations too. A corridor/repeated bay can have several
    # equally good positions even if the initial yaw search finds only one.
    first = rows[0]['pose']
    for dx, dy in [(0, .6), (0, -.6), (.6, 0), (-.6, 0)]:
        position = [first['PosX']+dx, first['PosY']+dy, first['PosZ']]
        result = _solve(body, reference, index, position, first['Yaw'], center, max_translation)
        if result is not None:
            rows.append(result)
    rows.sort(key=lambda row: row['quality']['score'])
    alternatives = []
    for row in rows:
        if not any(_same_place(row, previous) for previous in alternatives):
            alternatives.append(row)
    best = alternatives[0]
    quality = best['quality']
    if quality['overlap'] < MIN_OVERLAP or quality['rmse_m'] > MAX_RMSE:
        raise ValueError('结构匹配不足：吻合 %.1f%%，误差 %.3f m' %
                         (quality['overlap']*100, quality['rmse_m']))
    gap = alternatives[1]['quality']['score']-quality['score'] if len(alternatives)>1 else None
    if gap is not None and gap < MIN_SCORE_GAP:
        raise ValueError('有多个相近的地图匹配，请在地图上指定实际位置')
    return dict(accepted=True, pose=best['pose'], quality=dict(quality, distinct_score_gap=gap),
                alternatives=alternatives[1:5], source_stamp=stamp, frame_id=frame_id,
                body_points=len(body), reference_points=len(reference),
                requires_independent_validation=True, method='local_structure_multi_heading_icp')


def verify_pose(match, new_body_points, reference_points, *, stamp, frame_id='base_link'):
    """Check the hypothesis against a later scan; does not mark a robot localized."""
    stamp = _stamp(stamp)
    if not match.get('accepted') or stamp <= _stamp(match.get('source_stamp')):
        raise ValueError('必须用时间戳更新的独立新帧验证定位')
    pose = match['pose']
    center = _hint(pose)
    yaw = float(pose['Yaw'])
    if not math.isfinite(yaw):
        raise ValueError('定位朝向无效')
    body = _body(new_body_points, frame_id)
    reference = _reference(reference_points, center)
    quality = _quality(_transform(body, center, yaw), _Index(reference))
    if quality['overlap'] < MIN_OVERLAP or quality['rmse_m'] > MAX_RMSE:
        raise ValueError('新帧没有确认定位：吻合 %.1f%%，误差 %.3f m' %
                         (quality['overlap']*100, quality['rmse_m']))
    return dict(verified=True, pose=dict(pose), quality=quality,
                source_stamp=match['source_stamp'], verification_stamp=stamp,
                body_points=len(body), reference_points=len(reference))


def _rigid_transform(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)):
        raise ValueError('保存轨迹变换无效')
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=.005) or
            not .995 < np.linalg.det(rotation) < 1.005):
        raise ValueError('保存轨迹变换不是刚体变换')
    return matrix


def saved_slam_pose(build, sensors, active_map):
    """Bind the last SLAM pose to the exact completed extension session.

    All journal/receive times are NOS times; no comparison with the Mac clock.
    This is only a pose hypothesis. The current robot may have moved since
    saving, so fresh structural verification is mandatory before using it.
    """
    from map_fusion import pose_matrix
    from map_change_policy import gate_alignment
    start = build.get('mapping') or {}
    stop = build.get('stop_mapping') or {}
    context = start.get('context') or {}
    final_context = stop.get('context') or {}
    fusion = stop.get('fusion') or {}
    slam = sensors.get('slam_odom') or {}
    if (build.get('phase') != 'saved' or build.get('in_progress') or
            stop.get('state') != 'command_finished' or stop.get('returncode') != 0):
        raise ValueError('尚无已完成的原图续建记录')
    if (not active_map or context.get('workflow') != 'extension' or
            context.get('base') != active_map or final_context.get('base') != active_map or
            final_context.get('workflow') != 'extension' or fusion.get('base_map') != active_map or
            final_context.get('grid_id', 0) != context.get('grid_id', 0) or
            final_context.get('anchor') != context.get('anchor')):
        raise ValueError('保存轨迹不属于当前活动地图或同一采集会话')
    source = fusion.get('source_map', '')
    name = start.get('name')
    candidate = fusion.get('map')
    if (not name or not isinstance(source, str) or not source.startswith(name+'-') or
            not candidate or candidate not in stop.get('result_maps', []) or
            candidate not in build.get('result_maps', [])):
        raise ValueError('保存轨迹与融合结果的采集会话不一致')
    structural = fusion.get('structural_alignment') or {}
    if ((structural.get('decision') or {}).get('allowed') is not True or
            not gate_alignment(structural)['allowed']):
        raise ValueError('保存地图没有通过独立结构配准检查')
    try:
        began, stopping, ended, created, received, stamp = map(_stamp, [
            start['started'], stop['started'], stop['ended'], fusion['created'],
            slam['received_at'], slam['stamp']])
    except (KeyError, TypeError):
        raise ValueError('保存轨迹缺少本轮时间记录')
    if (not began <= stopping <= created <= ended or not began <= received <= ended or
            not began <= stamp <= ended or received < stopping-2 or
            not -.1 <= received-stamp <= .75):
        raise ValueError('SLAM 缓存不属于本次停止保存时刻')
    if slam.get('frame_id') != 'map':
        raise ValueError('保存 SLAM 位姿的地图坐标系无效')
    child = slam.get('child_frame_id')
    if child != 'base_link':
        proof = sensors.get('slam_pose_proof') or {}
        if (child != '' or proof.get('verified') is not True or
                proof.get('method') != 'paired_base_link_and_map_clouds' or
                not began <= _stamp(proof.get('stamp')) <= stamp):
            raise ValueError('保存 SLAM 机身坐标系未在本轮核对')
    try:
        values = [slam[k] for k in ['PosX', 'PosY', 'PosZ']] + list(slam['quaternion'])
        q = np.asarray(values[3:], dtype=float)
        if len(values) != 7 or not np.isfinite(values).all() or not .5 < np.linalg.norm(q) < 1.5:
            raise ValueError('保存 SLAM 位姿无效')
        matrix = _rigid_transform(fusion['transform']) @ pose_matrix(values)
    except (KeyError, TypeError):
        raise ValueError('保存 SLAM 位姿不完整')
    matrix = _rigid_transform(matrix)
    pose = dict(PosX=float(matrix[0, 3]), PosY=float(matrix[1, 3]),
                PosZ=float(matrix[2, 3]), Yaw=math.atan2(matrix[1, 0], matrix[0, 0]))
    return dict(pose=pose, transform=matrix.tolist(), source_stamp=stamp,
                received_at=received, base_map=active_map, candidate_map=candidate,
                source_map=source, session_key=[name, began, stopping, ended, source, candidate],
                method='saved_slam_full_transform', requires_independent_validation=True)


def verify_transform(transform, new_body_points, reference_points, *, stamp,
                     after_stamp, frame_id='base_link'):
    """Verify a KNOWN full 3-D transform, retaining roll and pitch.

    No ICP or correction is fitted to this verification scan. The returned
    initialization pose contains only fields supported by the vendor's 2101
    interface; it is deliberately distinct from validation_transform.
    """
    stamp, after_stamp = _stamp(stamp), _stamp(after_stamp)
    if stamp <= after_stamp:
        raise ValueError('必须用时间戳更新的独立新帧验证保存轨迹')
    matrix = _rigid_transform(transform)
    body = _body(new_body_points, frame_id)
    world = body @ matrix[:3, :3].T + matrix[:3, 3]
    reference = _points(reference_points)
    # Crop in the transformed frame, rather than silently truncating tilted
    # structural points to the yaw-only matcher's vertical bounds.
    lower, upper = world.min(axis=0)-.3, world.max(axis=0)+.3
    reference = _voxel(reference[((reference >= lower) & (reference <= upper)).all(axis=1)], .08, 60000)
    if len(reference) < MIN_POINTS:
        raise ValueError('保存轨迹附近的原图结构不足')
    quality = _quality(world, _Index(reference))
    if quality['overlap'] < MIN_OVERLAP or quality['rmse_m'] > MAX_RMSE:
        raise ValueError('新帧没有确认保存轨迹：吻合 %.1f%%，误差 %.3f m' %
                         (quality['overlap']*100, quality['rmse_m']))
    pose = dict(PosX=float(matrix[0, 3]), PosY=float(matrix[1, 3]),
                PosZ=float(matrix[2, 3]), Yaw=math.atan2(matrix[1, 0], matrix[0, 0]))
    return dict(verified=True, initialization_pose=pose,
                validation_transform=matrix.tolist(), quality=quality,
                source_stamp=after_stamp, verification_stamp=stamp,
                body_points=len(body), reference_points=len(reference),
                method='independent_full_3d_structural_validation')

"""Navigation preview evidence and bounded error summaries; no robot I/O."""
import math


def align_observed_pose(pose,scan_age,localization):
    """Compare timestamped poses at one time without changing scan geometry."""
    try:
        velocity=localization.get('velocity_body');omega=float(localization.get('yaw_rate',0))
        motion_age=float(localization.get('motion_age',math.inf))
        age=float(localization.get('age',0));dt=max(-.65,min(.65,float(scan_age)-age))
        if (not isinstance(velocity,(list,tuple)) or len(velocity)!=2 or not 0<=motion_age<=.25
                or not all(math.isfinite(float(v)) for v in [*velocity,omega,age,scan_age])):return list(pose)
        heading=pose[2]+omega*dt/2;c,s=math.cos(heading),math.sin(heading)
        return [pose[0]+(c*velocity[0]-s*velocity[1])*dt,
                pose[1]+(s*velocity[0]+c*velocity[1])*dt,
                math.atan2(math.sin(pose[2]+omega*dt),math.cos(pose[2]+omega*dt))]
    except (TypeError,ValueError,KeyError):return list(pose)


def poses_consistent(pose,scan_age,localization):
    reported=localization['pose']
    def close(candidate):
        return math.dist(candidate[:2],reported[:2])<=.35 and abs(math.atan2(math.sin(candidate[2]-reported[2]),math.cos(candidate[2]-reported[2])))<=.25
    return close(pose) or close(align_observed_pose(pose,scan_age,localization))


class NavigationEvidenceError(ValueError):
    def __init__(self,message,**details):
        super().__init__(message);self.navigation=details


def validate_map_identity(selected,library):
    name=selected.get('robot_map_name');active=library.get('active');grid=selected.get('grid_id',0)
    details=dict(classification='map_identity_mismatch',reason_code='active_map_mismatch',
                 recommended_action='sync_active_map',selected_map=name,active_map=active,grid_id=grid)
    if not name or not active or name!=active:
        raise NavigationEvidenceError('所选导航地图与机器人活动地图不同；请先同步当前活动地图，再重新选点',**details)
    rows=library.get('maps')
    item=next((row for row in rows if isinstance(row,dict) and row.get('name')==active),None) if isinstance(rows,list) else None
    grids=item.get('grids') if item else None
    if type(grid) is not int or not isinstance(grids,list) or grid not in grids:
        details.update(reason_code='active_grid_unverified',available_grids=grids if isinstance(grids,list) else [])
        raise NavigationEvidenceError('当前活动地图的导航栅格尚未核对；请同步活动地图后再规划',**details)
    return dict(map_name=active,grid_id=grid)


def error_summary(error):
    direct=getattr(error,'navigation',None)
    if isinstance(direct,dict):return dict(direct)
    recovery=getattr(error,'recovery',None)
    if not isinstance(recovery,dict):return None
    code=recovery.get('reason_code') or recovery.get('classification')
    action=('revalidate_observation' if code in ['live_evidence_expired','insufficient_ground_votes','observation_unavailable']
            else 'relocalize_and_replan' if code in ['map_surface_disagreement','local_structure_unconfirmed','unobserved_body_overlap']
            else None)
    result={key:recovery[key] for key in ['classification','reason_code','initial_clearance_lower_bound_m'] if key in recovery}
    diagnostics=recovery.get('overlap_diagnostics')
    if isinstance(diagnostics,dict):
        # No raw floor/support arrays or complete map cells in HTTP errors.
        result['overlap_diagnostics']={k:diagnostics[k] for k in ['overlap_cells','unobserved_body_cells','insufficient_vote_cells',
            'live_blocked_cells','surface_state','structure','standing_is_not_map_clearing_evidence','body_projection_cells','body_projection_policy'] if k in diagnostics}
    result['recommended_action']=action
    return result

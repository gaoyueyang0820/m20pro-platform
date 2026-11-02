"""Navigation preview evidence and bounded error summaries; no robot I/O."""


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
            'live_blocked_cells','surface_state','structure','standing_is_not_map_clearing_evidence'] if k in diagnostics}
    result['recommended_action']=action
    return result

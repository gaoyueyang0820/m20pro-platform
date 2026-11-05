"""Structured temporary SLAM-sync and incomplete-ground observation errors.

Human-readable messages never determine cache policy. Only this protocol and
explicit reasons on mapping /sensors/autonomy requests permit retaining an
already complete paired observation until its ORIGINAL deadline. Ground-pending
errors instead invalidate old geometry immediately and permit waiting for a NEW
ground observation; neither the error text nor a caller-supplied flag grants it.
"""
ERROR_TYPE='observation_sync_pending'
REASONS=frozenset(['slam_pose_sync','slam_reference_sync'])
GROUND_ERROR_TYPE='ground_observation_pending'
GROUND_REASONS=frozenset(['points_insufficient','floor_insufficient','floor_coverage_narrow','floor_plane_degenerate'])


class BridgeTransportError(ValueError):
    """A failed network read, distinct from invalid geometry or device state."""


class ObservationSyncPending(ValueError):
    def __init__(self,message,reason):
        if reason not in REASONS:raise ValueError('unsupported observation synchronization reason')
        super().__init__(message);self.reason=reason


class GroundObservationPending(ValueError):
    """A complete ground decision is unavailable due to sparse/occluded returns.

    This never means an observed obstacle or slope is safe. Consumers must clear
    their previous scene and issue zero while continuing to request new scans.
    """
    def __init__(self,message,reason_code):
        if reason_code not in GROUND_REASONS:raise ValueError('unsupported pending ground reason')
        super().__init__(message);self.reason_code=reason_code;self.reason=reason_code


def error_payload(error,path=None,request=None):
    payload=dict(ok=False,err=str(error))
    if (isinstance(error,GroundObservationPending) and path=='/sensors/autonomy'
            and isinstance(request,dict) and type(request.get('mapping',False)) is bool):
        payload.update(error_protocol=1,error_type=GROUND_ERROR_TYPE,reason_code=error.reason_code)
    elif (isinstance(error,ObservationSyncPending) and path=='/sensors/autonomy'
            and isinstance(request,dict) and request.get('mapping') is True):
        payload.update(error_protocol=1,error_type=ERROR_TYPE,reason=error.reason)
    return payload


def sync_error(payload,path,request=None):
    if (path=='/sensors/autonomy' and isinstance(request,dict) and type(request.get('mapping',False)) is bool
            and isinstance(payload,dict) and payload.get('ok') is False
            and type(payload.get('error_protocol')) is int and payload['error_protocol']==1
            and payload.get('error_type')==GROUND_ERROR_TYPE
            and isinstance(payload.get('reason_code'),str) and payload['reason_code'] in GROUND_REASONS):
        return GroundObservationPending(str(payload.get('err','等待新的地面观测')),payload['reason_code'])
    if (path!='/sensors/autonomy' or not isinstance(request,dict) or request.get('mapping') is not True
            or not isinstance(payload,dict) or payload.get('ok') is not False
            or type(payload.get('error_protocol')) is not int or payload['error_protocol']!=1
            or payload.get('error_type')!=ERROR_TYPE or not isinstance(payload.get('reason'),str)
            or payload.get('reason') not in REASONS):return None
    return ObservationSyncPending(str(payload.get('err','等待 SLAM 观测同步')),payload['reason'])

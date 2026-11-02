"""Narrow structured contract for temporary SLAM observation synchronization.

Human-readable messages never determine cache policy. Only this protocol and
explicit reasons on mapping /sensors/autonomy requests permit retaining an
already complete paired observation until its ORIGINAL deadline.
"""
ERROR_TYPE='observation_sync_pending'
REASONS=frozenset(['slam_pose_sync','slam_reference_sync'])


class ObservationSyncPending(ValueError):
    def __init__(self,message,reason):
        if reason not in REASONS:raise ValueError('unsupported observation synchronization reason')
        super().__init__(message);self.reason=reason


def error_payload(error,path=None,request=None):
    payload=dict(ok=False,err=str(error))
    if (isinstance(error,ObservationSyncPending) and path=='/sensors/autonomy'
            and isinstance(request,dict) and request.get('mapping') is True):
        payload.update(error_protocol=1,error_type=ERROR_TYPE,reason=error.reason)
    return payload


def sync_error(payload,path,request=None):
    if (path!='/sensors/autonomy' or not isinstance(request,dict) or request.get('mapping') is not True
            or not isinstance(payload,dict) or payload.get('ok') is not False
            or type(payload.get('error_protocol')) is not int or payload['error_protocol']!=1
            or payload.get('error_type')!=ERROR_TYPE or not isinstance(payload.get('reason'),str)
            or payload.get('reason') not in REASONS):return None
    return ObservationSyncPending(str(payload.get('err','等待 SLAM 观测同步')),payload['reason'])

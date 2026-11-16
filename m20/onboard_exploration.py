"""Desktop lease and status transport; all exploration motion runs on GOS.

The lease heartbeat is the safety-critical signal: GOS stops itself when the
lease expires. Reading NOS mapping status is diagnostic bookkeeping by
comparison, so the two must never share a blocking path. This loop sends the
pulse first and reads status afterwards on a slower cadence; a slow or failed
status read never delays or skips a heartbeat.
"""
import threading
import time
import uuid

from m20.platform_core import rpc

STATUS_INTERVAL = 1.5
HEARTBEAT_INTERVAL = .6


class OnboardExploration:
    def __init__(self, app, run_id, capture_id, context, endpoint):
        self.app=app
        self.context=context
        self.endpoint=endpoint
        self.key=dict(id=run_id,owner=uuid.uuid4().hex,capture_id=capture_id)
        self.cancelled=threading.Event()
        self.stopped=threading.Event()
        self.lease_end=0.
        self.heartbeat=dict(sent=0,failed=0,stale=0,status_reads=0,status_failures=0,
                            last_sent=None,last_gap_s=None)

    @classmethod
    def probe(cls, app):
        targets=app.hub.connections.value.get('targets',{})
        configured=targets.get('gos',{}).get('enabled')
        base=getattr(app,'config',{}).get('gos')
        if not configured and not base:return None
        try:
            base,token=app.hub.connections.endpoint('gos') if configured else (base,app.bridge_token)
            result=rpc(base,'/missions/capabilities',None,2,token)
        except ValueError:
            return None
        return (base,token) if result.get('available') and result.get('exploration_protocol')==1 else None

    def _call(self, path, payload, timeout=2):
        base,token=self.endpoint
        lease=path in ('/missions/exploration/start','/missions/pulse')
        if lease:self.lease_end=time.monotonic()+timeout+5.5
        result=rpc(base,path,payload,timeout,token)
        if lease:self.lease_end=time.monotonic()+5.5
        return result

    def _owned(self, status):
        if not isinstance(status, dict):return False
        if status.get('phase')!='collecting':return False
        mapping=status.get('mapping') or {}
        return mapping.get('request_id')==self.key['capture_id']

    def _beat(self):
        began=time.monotonic()
        try:reply=self._call('/missions/pulse',self.key)
        except ValueError:
            self.heartbeat['failed']+=1
            raise
        self.heartbeat['sent']+=1
        previous=self.heartbeat['last_sent']
        if previous is not None:self.heartbeat['last_gap_s']=round(began-previous,2)
        self.heartbeat['last_sent']=began
        mission=reply.get('mission') or {}
        mission['heartbeat']=dict(self.heartbeat)
        return mission

    def _read_status(self, map_status):
        self.heartbeat['status_reads']+=1
        try:status=map_status()
        except ValueError:
            # An unavailable read is not evidence that the capture ended.
            self.heartbeat['stale']+=1
            self.heartbeat['status_failures']+=1
            return None
        return status

    def run(self, spec, map_status, report, current):
        """An uncertain start is cancelled on the same endpoint, never replayed."""
        started=False;mission={}
        try:
            if not current() or self.cancelled.is_set():return {}
            started=True
            mission=self._call('/missions/exploration/start',dict(self.key,**spec),timeout=4)['mission']
            report(mission)
            # Establish the capture baseline before any renewal. A capture that
            # already belongs to another request must not receive a new lease.
            baseline=self._read_status(map_status)
            if baseline is not None and not self._owned(baseline):
                raise RuntimeError('建图采集已结束或已变化')
            last_status_at=time.monotonic()
            while current() and not self.cancelled.is_set():
                if mission.get('stop_confirmed'):
                    self.stopped.set();return mission
                self.cancelled.wait(HEARTBEAT_INTERVAL)
                if self.cancelled.is_set() or not current():break
                # Heartbeat first: the GOS lease expires on its own clock no
                # matter how slow the NOS status read is.
                try:mission=self._beat()
                except ValueError:
                    # Let the onboard lease handle a genuine disconnect. A
                    # delayed read does not stop the control loop on the Mac.
                    continue
                report(mission)
                if time.monotonic()-last_status_at<STATUS_INTERVAL:continue
                last_status_at=time.monotonic()
                status=self._read_status(map_status)
                if status is None:continue
                if not self._owned(status):
                    raise RuntimeError('建图采集已结束或已变化')
            return mission
        finally:
            if not started:self.stopped.set()
            while not self.stopped.is_set() and not self.app.done:
                self.cancelled.set()
                try:
                    reply=self._call('/missions/cancel',self.key)
                    mission=reply.get('mission') or {}
                    if mission.get('stop_confirmed') or reply.get('revoked') and reply.get('matched') is False:
                        self.stopped.set();break
                except ValueError:
                    if time.monotonic()>=self.lease_end:
                        from m20.point_navigation import stationary_feedback
                        with self.app.lock:
                            same=self.app.hub.same_context(self.context)
                            if not same or self.app.link and stationary_feedback(self.app.link,self.lease_end):
                                self.stopped.set();break
                time.sleep(.4)

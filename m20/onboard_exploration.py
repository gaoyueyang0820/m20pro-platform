"""Desktop lease and status transport; all exploration motion runs on GOS."""
import threading
import time
import uuid

from m20.platform_core import rpc


class OnboardExploration:
    def __init__(self, app, run_id, capture_id, context, endpoint):
        self.app=app
        self.context=context
        self.endpoint=endpoint
        self.key=dict(id=run_id,owner=uuid.uuid4().hex,capture_id=capture_id)
        self.cancelled=threading.Event()
        self.stopped=threading.Event()
        self.lease_end=0.

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

    def run(self, spec, map_status, report, current):
        """An uncertain start is cancelled on the same endpoint, never replayed."""
        started=False;mission={}
        try:
            if not current() or self.cancelled.is_set():return {}
            started=True
            mission=self._call('/missions/exploration/start',dict(self.key,**spec),timeout=4)['mission']
            report(mission)
            while current() and not self.cancelled.is_set():
                if mission.get('stop_confirmed'):
                    self.stopped.set();return mission
                self.cancelled.wait(.6)
                if self.cancelled.is_set() or not current():break
                try:
                    status=map_status()
                    if status.get('phase')!='collecting' or status.get('mapping',{}).get('request_id')!=self.key['capture_id']:
                        raise RuntimeError('建图采集已结束或已变化')
                    mission=self._call('/missions/pulse',self.key)['mission']
                    report(mission)
                except ValueError:
                    # Let the onboard lease handle a genuine disconnect. A
                    # delayed read does not stop the control loop on the Mac.
                    continue
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

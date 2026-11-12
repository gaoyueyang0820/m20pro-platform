"""Desktop delegation/reconnect tests; only synthetic RPC and local fixtures."""
import base64
import copy
import hashlib
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import Mock,patch
from m20.platform_core import RobotLink
from m20.resident_navigation_client import ResidentNavigationClient


class ResidentClientTests(unittest.TestCase):
    def setUp(self):
        self.acl=patch('m20.platform_host.private_file',lambda path:None);self.acl.start();self.addCleanup(self.acl.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.raw=b'P5\n3 3\n255\n'+bytes([254])*9
        (self.root/'maps').mkdir();(self.root/'maps/map.pgm').write_bytes(self.raw)
        self.map=dict(id='map',robot_map_name='synthetic',grid_id=0,resolution=.1,origin=[0.,0.,0.])
        connections=Obj(value=dict(aos_identity_alias='synthetic-robot',targets={'nos':{'enabled':True}}),endpoint=lambda _:('http://synthetic.invalid','synthetic-token'))
        self.app=Obj(store=Obj(path=self.root/'workspace.json'),lock=threading.RLock(),hub=Obj(connections=connections),demo=False,
            maps=lambda:[self.map],_record_route_completion=Mock(),nav=dict(status='idle'),link=None)
        self.c=ResidentNavigationClient(self.app);self.c.tick=Mock()
        self.nav=dict(status='running',disconnect_policy='finish',options=dict(NavMode=1,Gait=0x3002,Speed=1,ObsMode=0,Manner=0),route=dict(map_id='map',points=[dict(x=.15,y=.15,yaw=0,type=1)]))
        self.c.prepared(self.nav);self.app.nav=self.nav;self.c.wake=Mock()
    def grid(self):return dict(robot_map_name='synthetic',pgm=base64.b64encode(self.raw).decode(),resolution=.1,origin=[0.,0.,0.],yaml_sha256='b'*64)
    def test_only_complete_route_is_uploaded_after_capability_and_hash_checks(self):
        calls=[]
        def rpc(path,body=None):
            calls.append((path,body))
            if path=='/missions/capabilities':return dict(protocol=1,available=True,policies=['finish'])
            if path=='/maps/grid':return self.grid()
            return dict(ok=True)
        self.c._request=rpc;self.c._submit()
        self.assertEqual([p for p,_ in calls],['/missions/capabilities','/maps/grid','/missions/start'])
        self.assertEqual(calls[-1][1]['points'],self.nav['route']['points'])
        self.assertEqual(calls[-1][1]['disconnect_policy'],'finish')
    def test_capability_missing_or_changed_map_never_uploads_task(self):
        for kind in ['unavailable','policy','map']:
            self.c.ref['submitted']=False;calls=[]
            def rpc(path,body=None):
                calls.append(path)
                if path=='/missions/capabilities':
                    return dict(protocol=1,available=kind!='unavailable',policies=['stop'] if kind=='policy' else ['finish'])
                return dict(self.grid(),origin=[1,1,0])
            self.c._request=rpc
            with self.assertRaises(ValueError):self.c._submit()
            self.assertNotIn('/missions/start',calls);self.assertFalse(self.c.ref['submitted'])
    def test_cancel_during_map_read_wins_before_upload(self):
        calls=[]
        def rpc(path,body=None):
            calls.append(path)
            if path=='/missions/capabilities':return dict(protocol=1,available=True,policies=['finish'])
            self.c.cancel();return self.grid()
        self.c._request=rpc;self.c._submit();self.assertNotIn('/missions/start',calls)
    def test_restart_reconciles_only_and_never_replays_an_uncertain_start(self):
        self.c.ref['submitted']=False;self.c._save()
        restored=ResidentNavigationClient(self.app);restored.wake=Mock();calls=[]
        def rpc(path,body=None):
            calls.append(path)
            return dict(mission=dict(id=restored.ref['id'],phase='completed',completed=1,stop_confirmed=True))
        restored._request=rpc;restored._run()
        self.assertNotIn('/missions/start',calls);self.assertEqual(restored.nav['status'],'completed')
    def test_cancel_receipt_for_not_started_request_is_final_without_motion(self):
        self.c.cancel();self.c._request=Mock(return_value=dict(matched=False,revoked=True,mission=dict(phase='idle')))
        self.c._run();self.assertFalse(self.c.busy());self.assertEqual(self.nav['status'],'cancelled')
    def test_sent_stop_or_failed_link_is_never_treated_as_confirmed_parking(self):
        self.c.cancel();self.c._observe(dict(mission=dict(id=self.c.ref['id'],phase='stopping',completed=0,stop_confirmed=False)))
        self.assertTrue(self.c.busy());self.assertEqual(self.nav['status'],'cancelling')
    def test_changed_robot_identity_prevents_old_rpc_and_pulses(self):
        self.app.hub.connections.value['aos_identity_alias']='another-robot'
        self.c._request=Mock();self.c._run();self.c._request.assert_not_called()
    def test_closing_observer_link_does_not_send_zero_or_cancel(self):
        link=RobotLink.__new__(RobotLink);link.set_velocity=Mock();link._send_axis=Mock();link.sock=Mock();link._stop=False
        link.close(stop=False);link.set_velocity.assert_not_called();link._send_axis.assert_not_called();self.assertTrue(link._stop)


if __name__=='__main__':unittest.main()

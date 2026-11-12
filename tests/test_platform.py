"""Offline behavior tests: no robot, ROS2, GPU or external API requests."""
import base64
import copy
import json
import math
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
import m20_platform as web
from m20.platform_core import JOINTS, NEUTRAL, ROOT, RobotLink, Store, Training, explain_error, leg_target, number
from sdk_bridge import JointController


class FakeLink:
    def __init__(self):
        self.ip='127.0.0.1';self.sent=[];self.records={};self.inhibit=False
        self.put(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=0,Direction=0)))
        self.put(1007,2,dict(Location=0,PosX=0,PosY=0,Yaw=0))
        self.put(1007,1,dict(Value=0,Status=0,ErrorCode=0))
    def put(self,t,c,it,at=None,seq=0):
        self.records[t,c]=dict(items=it,at=at or time.time(),seq=seq)
    def record(self,t,c):return copy.deepcopy(self.records.get((t,c),{}))
    def command(self,t,c,it,label):
        r=dict(seq=len(self.sent)+1,sent_at=time.time(),type=t,command=c,items=it)
        self.sent.append(r);return r
    def set_velocity(self,*v,valid_until=None):self.v=v;self.valid_until=valid_until
    def set_motion(self,key):self.sent.append(dict(motion=key))
    def set_mode(self,mode):self.sent.append(dict(mode=mode))
    def close(self):pass


class PlatformTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.app=web.Platform(Store(Path(self.temp.name)/'workspace.json'))
    def tearDown(self):self.temp.cleanup()
    def live(self):
        a=self.app;a.demo=False;a.link=FakeLink();a.basic=dict(HES=0,MotionState=17,ControlUsageMode=0)
        a.store.value['maps']=[dict(id='test',name='test',robot_map_name='test',verified_for_robot=True,robot_ip='127.0.0.1',origin=[-10,-10,0],width=200,height=200,resolution=.1)]
        a.proxy=lambda *args,**kwargs:dict(active='test',phase='idle',in_progress=False)
        a.hub.remote_maps=lambda path,req=None,timeout=120,context=None:a.proxy('nos',path,req,timeout=timeout)
        return a
    def route(self,live=False):
        if live:
            self.live()
            self.app.link.put(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=1,Sleep=0,Gait=0x3002)))
            self.app.link.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
        return self.app.route_save(dict(name='test',map_id='test' if live else 'demo',points=[dict(x=0,y=0),dict(x=1,y=0)]))['route']
    def test_reject_nonfinite(self):
        for n in [float('nan'),float('inf'),True,'NaN']:
            with self.assertRaises(ValueError):number(n)
    def test_bounded_relative_control_expires_without_next_http_request(self):
        self.live();self.app.set_velocity(dict(forward=.2,hold_seconds=.05))
        self.assertIsNotNone(self.app.link.valid_until)
        self.app.velocity_deadline=time.monotonic()-1;self.app.tick(.01)
        self.assertEqual(self.app.velocity,[0.,0.,0.]);self.assertEqual(self.app.link.v,(0,0,0))
        for invalid in [-1,1,float('nan')]:
            with self.assertRaises(ValueError):self.app.set_velocity(dict(forward=.2,hold_seconds=invalid))
    def test_ik_changes_only_requested_leg(self):
        for leg in ['FL','FR','HL','HR']:
            q=leg_target(NEUTRAL,leg,.04);i=['FL','FR','HL','HR'].index(leg)*4
            unchanged=[j for j in range(16) if j not in [i+1,i+2]]
            self.assertEqual([q[j] for j in unchanged],[NEUTRAL[j] for j in unchanged])
            def foot(v):return -.25*(math.cos(v[i+1])+math.cos(v[i+1]+v[i+2]))
            self.assertAlmostEqual(foot(q)-foot(NEUTRAL),.04,places=8)
    def test_joint_requires_session_and_confirmation(self):
        with self.assertRaises(ValueError):self.app.joint_control('set',dict(index=0,value=.1))
        with self.assertRaises(ValueError):self.app.joint_control('arm',{})
        self.assertEqual(self.app.positions,NEUTRAL)
    def test_joint_lease_expiry_stops(self):
        a=self.app;r=a.joint_control('arm',dict(rig_confirmed=True))
        a.joint_control('set',dict(session=r['session'],index=0,value=.1))
        a.lease_at=time.time()-1;a.tick(.01)
        self.assertFalse(a.sdk['armed']);self.assertIsNone(a.session);self.assertEqual(a.target,a.positions)
    def test_leg_lower_returns_to_anchor(self):
        a=self.app;s=a.joint_control('arm',dict(rig_confirmed=True))['session']
        a.joint_control('leg',dict(session=s,leg='FL',height=.04))
        self.assertNotEqual(a.target,NEUTRAL)
        a.positions=list(a.target)
        a.joint_control('lower',dict(session=s))
        self.assertEqual(a.target,NEUTRAL)
    def test_estop_latches_motion(self):
        a=self.app;a.command('estop')
        with self.assertRaises(ValueError):a.set_velocity(dict(forward=.1))
        a.command('stand');a.set_velocity(dict(forward=.1));self.assertEqual(a.velocity,[.1,0,0])
    def test_demo_velocity_watchdog(self):
        a=self.app;a.set_velocity(dict(forward=.1));a.velocity_at=time.time()-1;a.tick(.1)
        self.assertEqual(a.velocity,[0,0,0])
    def test_no_movement_with_stale_or_hard_estop(self):
        a=self.live();a.link.put(1002,6,dict(BasicStatus=dict(HES=0)),at=time.time()-4)
        with self.assertRaises(ValueError):a.set_velocity(dict(forward=.1))
        a.link.put(1002,6,dict(BasicStatus=dict(HES=1)))
        with self.assertRaises(ValueError):a.set_velocity(dict(forward=.1))
    def test_autonomous_output_requires_fresh_head_forward_and_no_software_reverse(self):
        a=self.live();a.set_velocity(dict(forward=.35),autonomous=True)
        self.assertEqual(a.link.v,(.35,0,0))
        for direction,age,reverse in [(1,0,False),(None,0,False),(0,3,False),(0,0,True)]:
            with self.subTest(direction=direction,age=age,reverse=reverse):
                basic=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=0)
                if direction is not None:basic['Direction']=direction
                a.link.put(1002,6,dict(BasicStatus=basic),at=time.time()-age);a.reverse=reverse
                a.velocity=[.35,0,0];a.velocity_deadline=time.monotonic()+1
                with self.assertRaisesRegex(ValueError,'头朝前'):a.set_velocity(dict(forward=.35,yaw=.7),autonomous=True)
                self.assertEqual(a.velocity,[0.,0.,0.]);self.assertEqual(a.link.v,(0,0,0));self.assertIsNone(a.velocity_deadline)
        # An explicit zero always remains available even with unknown direction.
        a.set_velocity({},autonomous=True);self.assertEqual(a.link.v,(0,0,0))
    def test_route_ack_is_not_completion(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        a.link.put(1003,1,dict(ErrorCode=0,Value=a.nav['value']))
        a.tick(.01);self.assertEqual(a.nav['index'],0)
        a.link.put(1007,1,dict(ErrorCode=0,Status=4,Value=a.nav['value']+9))
        a.tick(.01);self.assertEqual(a.nav['index'],0)
        a.link.put(1007,1,dict(ErrorCode=0,Status=4,Value=a.nav['value']),at=a.nav['sent_at']-1)
        a.tick(.01);self.assertEqual(a.nav['index'],0)
        a.link.put(1007,1,dict(ErrorCode=0x2300,Status=4,Value=a.nav['value']))
        a.tick(.01);self.assertEqual(a.nav['index'],1)
        self.assertEqual(a.link.sent[-1]['items']['PosX'],1)
    def test_idle_with_point_success_advances_once_and_completes_full_route(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        first=a.nav['value']
        # Captured firmware response: Status=0, ErrorCode=8960 (0x2300).
        a.link.put(1007,1,dict(Value=first,Status=0,ErrorCode=8960,LoopCnt=1,RemainingCnt=0))
        a.tick(.01)
        self.assertEqual(a.nav['completed'],1);self.assertEqual(a.nav['index'],1)
        self.assertEqual(a.link.sent[-1]['items']['PosX'],1)
        second=a.nav['value'];sent=len(a.link.sent)
        a.tick(.01);self.assertEqual(a.nav['value'],second);self.assertEqual(len(a.link.sent),sent)
        a.link.put(1007,1,dict(Value=second,Status=0,ErrorCode=8960))
        a.tick(.01);self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.nav['completed'],2)
        self.assertEqual(a.link.sent[-1]['type'],1004)
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'completed')
        a.tick(.01);self.assertEqual(len(a.link.sent),sent+1)
    def test_idle_ack_cancel_and_active_status_do_not_complete_a_point(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        value=a.nav['value']
        for status,code in [(0,0),(3,0),(3,0x2300)]:
            a.link.put(1007,1,dict(Value=value,Status=status,ErrorCode=code));a.tick(.01)
            self.assertEqual(a.nav['index'],0);self.assertEqual(a.nav['status'],'running')
        a.link.put(1007,1,dict(Value=value,Status=0,ErrorCode=0x2302));a.tick(.01)
        self.assertEqual(a.nav['index'],0);self.assertEqual(a.nav['status'],'cancelled')
    def test_success_for_wrong_value_or_before_dispatch_cannot_advance(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        a.link.put(1007,1,dict(Value=a.nav['value']-1,Status=0,ErrorCode=8960));a.tick(.01)
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=0,ErrorCode=8960),at=a.nav['sent_at']-1);a.tick(.01)
        self.assertEqual(a.nav['index'],0);self.assertEqual(len(a.link.sent),1)
    def test_terminal_event_survives_a_later_idle_poll(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        a.link.lock=threading.RLock()
        a.link.events=[dict(type=1007,command=1,items=dict(Value=a.nav['value'],Status=4,ErrorCode=0),at=time.time())]
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=0,ErrorCode=0))
        a.tick(.01);self.assertEqual(a.nav['index'],1);self.assertEqual(a.link.events,[])
        a.tick(.01);self.assertEqual(a.nav['index'],1)
    def test_arrival_during_localization_loss_waits_for_navigation_mode_to_recover(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=0,ErrorCode=8960))
        a.link.put(1007,2,dict(Location=1))
        a.link.records[1002,6]['items']['BasicStatus']['ControlUsageMode']=0
        a.tick(.01)
        self.assertEqual(a.nav['index'],1);self.assertEqual(a.nav['phase'],'preparing')
        self.assertEqual([r['type'] for r in a.link.sent],[1003])
    def test_task_point_pause_requires_continue_before_next_target(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id'],pause_at_point=True))
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=0,ErrorCode=8960));a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.nav['completed'],1)
        self.assertEqual([r['type'] for r in a.link.sent],[1003,1004])
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'paused')
        a.tick(.01);self.assertEqual(len(a.link.sent),2)
        a.resume_route();self.assertEqual(a.link.sent[-1]['items']['PosX'],1)
        self.assertEqual(a.nav['status'],'running')
    def test_saved_route_keeps_navigation_options_and_pause_behavior(self):
        a=self.live();r=a.route_save(dict(name='persisted options',map_id='test',points=[dict(x=0,y=0)],
            options=dict(nav_mode=1,speed=1,gait='flat',manner=0,pause_at_point=True)))['route']
        a.start_route(dict(id=r['id']))
        self.assertTrue(a.nav['pause_at_point']);self.assertEqual(a.nav['options']['NavMode'],1)
        self.assertEqual(Store(a.store.path).value['routes'][0]['options']['pause_at_point'],True)
    def test_cancel_requires_matching_ack(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']));a.stop()
        self.assertEqual(a.nav['status'],'cancelling')
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']+1);a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling')
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelled')
    def test_loss_of_location_and_original_task_feedback_pauses_route(self):
        a=self.app;r=self.route(True);a.start_route(dict(id=r['id']))
        a.link.put(1007,2,dict(Location=1));a.tick(.01)
        self.assertEqual(a.nav['execution_state'],'waiting_feedback')
        a.nav['missing_execution_since']=time.monotonic()-4
        a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.link.sent[-1]['type'],1004)
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'paused')
    def test_map_change_prevents_navigation(self):
        r=self.route(True);self.app.proxy=lambda *args,**kwargs:dict(active='different')
        with self.assertRaises(ValueError):self.app.start_route(dict(id=r['id']))
    def test_matching_active_map_is_resynced_before_navigation_when_cache_unverified(self):
        r=self.route(True);a=self.app;a.store.value['maps'][0]['verified_for_robot']=False
        grid=dict(ok=True,name='test',robot_map_name='test',grid_id=0,active=True,resolution=.1,origin=[-10,-10,0],
                  pgm=base64.b64encode(b'P5\n200 200\n255\n'+b'\xff'*40000).decode())
        a.hub.remote_maps=lambda path,*args,**kwargs:dict(phase='idle',in_progress=False) if path=='/maps/build/status' else grid
        a.start_route(dict(id=r['id']))
        self.assertTrue(a.store.value['maps'][0]['verified_for_robot']);self.assertEqual(a.nav['status'],'running')
        self.assertEqual(a.nav['route']['map_id'],'test')
    def test_out_of_map_rejected(self):
        with self.assertRaises(ValueError):self.app.route_save(dict(name='bad',map_id='demo',points=[dict(x=100,y=0)]))
    def test_autonomous_navigation_constraints(self):
        r=self.route()
        with self.assertRaises(ValueError):self.app.start_route(dict(id=r['id'],nav_mode=1,gait='stair'))
    def test_store_persists_routes(self):
        self.route();restored=Store(self.app.store.path)
        self.assertEqual(restored.value['routes'][0]['name'],'test')
    def test_component_bitmap(self):
        e=explain_error(0x8001,0x21);self.assertEqual(e['affected'],['FL.HipX','FR.HipY'])
        self.assertEqual(explain_error(0x8102,3)['affected'],['后侧电池','前侧电池'])
    def test_pgm_preserves_whitespace_pixel(self):
        for pixels in [b'\x20\xff',b'\n\xff',b'\x00\xff']:
            w,h,png=web.import_pgm(base64.b64encode(b'P5\n2 1\n255\n'+pixels).decode())
            self.assertEqual((w,h),(2,1));self.assertTrue(png.startswith(b'\x89PNG'))
        with self.assertRaises(ValueError):web.import_pgm(base64.b64encode(b'P5\n2 1\n255\n\xff').decode())
    def test_training_fixed_argv_and_path(self):
        t=Training(demo=True);p=t.preview(dict(task='flat',envs=32,iterations=10))
        self.assertIn('Flat-Deeprobotics-M20-v0',p['argv'])
        with self.assertRaises(ValueError):t.preview(dict(task='flat; curl evil'))
        with self.assertRaises(ValueError):t.preview(dict(kind='export',checkpoint='../../evil.pt'))
        with self.assertRaises(ValueError):t.preview(dict(envs=1.1))


class ProtocolTests(unittest.TestCase):
    def test_frames_and_status_namespaces(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(('127.0.0.1',0));server.settimeout(1)
        link=RobotLink('127.0.0.1',server.getsockname()[1])
        try:
            frame=link.apdu(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=0)))
            self.assertIsNotNone(link._parse(frame))
            self.assertIsNone(link._parse(b'xxxx'+frame[4:]))
            self.assertIsNone(link._parse(frame+b'x'))
            _,addr=server.recvfrom(65535)
            for t,c,items in [(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=0))),
                              (1007,1,dict(Status=3)),(2002,1,dict(Location=0))]:
                server.sendto(link.apdu(t,c,items),addr)
            deadline=time.time()+1
            while time.time()<deadline and not link.record(2002,1):time.sleep(.01)
            self.assertEqual(link.record(1007,1)['items']['Status'],3)
            self.assertEqual(link.record(2002,1)['items']['Location'],0)
            link.set_velocity(.1,0,0)
            link._vel_monotonic=time.monotonic()-1
            time.sleep(.1)
            self.assertEqual(link.payload()['cmd_vel'],dict(f=0,s=0,y=0))
            link.set_velocity(.1,0,0,valid_until=time.monotonic()+.02)
            time.sleep(.12)
            self.assertEqual(link.payload()['cmd_vel'],dict(f=0,s=0,y=0))
        finally:link.close();server.close()


class SDKTests(unittest.TestCase):
    def setup_controller(self):
        cal=json.loads((ROOT/'bridge/calibration.example.json').read_text());cal.update(verified=True,sdk_mode_verified=True)
        c=JointController(cal)
        raw=[SimpleNamespace(position=(NEUTRAL[i]-cal['joints'][i]['offset_rad'])*cal['joints'][i]['direction'],velocity=0,torque=0,motion_temp=30,driver_temp=30,status_word=1) for i in range(16)]
        c.feedback(raw);c.imu_feedback(SimpleNamespace(**{k:0 for k in ['roll','pitch','yaw','acc_x','acc_y','acc_z','omega_x','omega_y','omega_z']}));return c
    def test_unverified_calibration_refused(self):
        with self.assertRaises(ValueError):JointController(json.loads((ROOT/'bridge/calibration.example.json').read_text()))
    def test_no_publication_before_arm_and_damping_after_timeout(self):
        c=self.setup_controller();self.assertIsNone(c.command())
        s='test-session-with-20-chars';c.dispatch('/arm',dict(session=s,rig_confirmed=True))
        rows=c.command();self.assertEqual(rows[0]['kp'],20);self.assertEqual(rows[3]['kp'],0)
        rows=c.command(time.monotonic()+1)
        self.assertFalse(c.armed);self.assertTrue(all(r['kp']==0 and r['velocity']==0 for r in rows))
        self.assertTrue(all(r['control_word']==4 for r in rows))
    def test_publisher_conflict_and_nonfinite(self):
        c=self.setup_controller();c.publisher_conflict=True
        with self.assertRaises(ValueError):c.dispatch('/arm',dict(session='long-session-with-20-chars',rig_confirmed=True))
        c.publisher_conflict=False;s='long-session-with-20-chars';c.dispatch('/arm',dict(session=s,rig_confirmed=True))
        q=list(NEUTRAL);q[0]=float('nan')
        with self.assertRaises(ValueError):c.dispatch('/trajectory',dict(session=s,positions=q,wheels=[0]*4,duration=2))
        c.publisher_conflict=True;self.assertIsNone(c.command());self.assertFalse(c.armed)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();web.APP=web.Platform(Store(Path(self.tmp.name)/'workspace.json'));web.TOKEN='offline-test-token'
        self.server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
        self.base='http://127.0.0.1:'+str(self.server.server_address[1])
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.tmp.cleanup();web.TOKEN=None
    def request(self,path,data=None,headers=None):
        req=Request(self.base+path,data=json.dumps(data).encode() if data is not None else None,headers=headers or {})
        with urlopen(req,timeout=2) as res:return json.load(res)
    def test_auth_and_origin(self):
        with self.assertRaises(HTTPError) as e:self.request('/api/platform')
        self.assertEqual(e.exception.code,401)
        self.assertEqual(self.request('/api/platform',headers={'X-API-Token':'offline-test-token'})['mode'],'demo')
        with self.assertRaises(HTTPError) as e:self.request('/api/cmd',dict(action='stand'),{'Content-Type':'application/json','X-API-Token':'offline-test-token','Origin':'https://example.com'})
        self.assertEqual(e.exception.code,403)
    def test_json_shape_and_finite_velocity(self):
        headers={'Content-Type':'application/json','X-API-Token':'offline-test-token'}
        for data in [[],dict(forward=float('nan')),dict(forward=2)]:
            with self.assertRaises(HTTPError) as e:self.request('/api/velocity',data,headers)
            self.assertEqual(e.exception.code,400)


if __name__=='__main__':unittest.main()

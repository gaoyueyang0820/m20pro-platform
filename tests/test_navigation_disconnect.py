"""Synthetic onboard mission receipts, link loss and measured-path return."""
import copy
from contextlib import nullcontext
import math
import json
import tempfile
from pathlib import Path
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import Mock,patch
from m20.navigation_disconnect import ResidentMission,MeasuredTrace,specification
from bridge.resident_mission import ResidentMissionService,NativeMissionDriver
import m20.platform_core as platform_core  # Warm imports before taking synthetic receipt timestamps.

MAP=dict(name='synthetic',grid_id=0,sha256='a'*64,yaml_sha256='b'*64,resolution=.1,origin=[-3.,-3.,0.])
def request(policy='finish'):
    return dict(id='1'*32,owner='2'*32,disconnect_policy=policy,map=copy.deepcopy(MAP),
                points=[dict(x=1.,y=0.,yaw=0.),dict(x=1.,y=1.,yaw=math.pi/2)])
def frame(pose=(0.,0.,0.),stamp=1.,**extra):
    return dict(ok=True,map=copy.deepcopy(MAP),safe=True,normal=True,age=0.,pose=list(pose),stamp=stamp,**extra)


class DisconnectMissionTests(unittest.TestCase):
    def start(self,policy='finish'):
        self.saved=[];self.m=ResidentMission(lambda r:self.saved.append(copy.deepcopy(r)))
        self.m.start(request(policy),frame(),1.)
        action=self.m.step(frame(),1.);self.assertEqual(action['kind'],'goal');self.m.dispatched(action,{'value':10},1.)
        return self.m
    def test_finish_advances_all_points_offline_and_requires_stop_confirmation(self):
        m=self.start();self.assertIsNone(m.step(frame(),7.));self.assertTrue(m.job['lost_once'])
        a=m.step(frame((1,0,0),2,goal_complete=True),8);self.assertEqual(a['point']['y'],1);m.dispatched(a,{'value':11},8)
        a=m.step(frame((1,1,math.pi/2),3,goal_complete=True),9);self.assertEqual(a['kind'],'cancel');m.dispatched(a,{'seq':12},9)
        self.assertEqual(m.phase,'stopping');self.assertFalse(m.public()['stop_confirmed'])
        m.step(frame((1,1,0),4,cancel_confirmed=True,still=False),10);self.assertEqual(m.phase,'stopping')
        m.step(frame((1,1,0),5,cancel_confirmed=True,still=True),11)
        self.assertEqual(m.phase,'completed');self.assertTrue(m.public()['stop_confirmed'])
    def test_default_stop_policy_does_not_continue_on_disconnect(self):
        m=self.start('stop');a=m.step(frame(),7)
        self.assertEqual(a['kind'],'cancel');self.assertEqual(m.job['after_stop'],'cancelled')
    def test_return_waits_for_stop_and_uses_reversed_measured_route(self):
        m=self.start('return')
        m.step(frame((.5,0,0),2),2);m.pulse('1'*32,'2'*32,2)
        m.step(frame((1,0,0),3),3);m.step(frame((1,.5,math.pi/2),4),4)
        a=m.step(frame((1,.5,math.pi/2),5),8);self.assertEqual(a['kind'],'cancel');m.dispatched(a,{'seq':1},8)
        self.assertIsNone(m.step(frame((1,.5,math.pi/2),6),9));self.assertEqual(m.phase,'stopping')
        m.step(frame((1,.5,math.pi/2),7,cancel_confirmed=True,still=True),10)
        self.assertEqual(m.phase,'returning');points=m.job['return_points']
        self.assertEqual([(p['x'],p['y']) for p in points],[(1,0),(.5,0),(0,0)])
        m.pulse('1'*32,'2'*32,11);self.assertEqual(m.phase,'returning')
    def test_finish_then_return_applies_only_after_real_link_loss(self):
        m=self.start('finish_return');m.step(frame(),7)
        a=m.step(frame((1,0,0),2,goal_complete=True),8);m.dispatched(a,{},8)
        a=m.step(frame((1,1,0),3,goal_complete=True),9)
        self.assertEqual(a['kind'],'cancel');self.assertEqual(m.job['after_stop'],'return')
    def test_reconnect_does_not_change_a_latched_return_to_forward(self):
        m=self.start('return');a=m.step(frame((.5,0,0),2),7);m.dispatched(a,{},7)
        m.pulse('1'*32,'2'*32,7.1);m.step(frame((.5,0,0),3,cancel_confirmed=True,still=True),8)
        self.assertEqual(m.phase,'returning')
    def test_local_faults_stop_even_when_policy_is_finish(self):
        for changes in [dict(age=.66),dict(age=-1),dict(safe=False),dict(normal=False),dict(map=dict(MAP,sha256='c'*64)),dict(manual_override=True)]:
            m=self.start();f=frame();f.update(changes);a=m.step(f,2)
            self.assertEqual(a['kind'],'cancel');self.assertEqual(m.job['after_stop'],'failed')
    def test_transport_ack_never_advances_a_point(self):
        m=self.start();m.step(frame((1,0,0),2,ack=True),2)
        self.assertEqual(m.job['completed'],0);self.assertEqual(m.phase,'forward')
    def test_measured_arrival_must_agree_with_terminal_result(self):
        m=self.start();a=m.step(frame(goal_complete=True),2)
        self.assertEqual(a['kind'],'cancel');self.assertEqual(m.job['after_stop'],'failed')
    def test_cancel_before_start_and_late_dispatched_goal_are_revoked(self):
        m=ResidentMission();m.cancel('1'*32,'2'*32,0)
        with self.assertRaisesRegex(ValueError,'撤销'):m.start(request(),frame(),1)
        m=self.start();m.goal=None;a=m.step(frame(),2);m.cancel('1'*32,'2'*32,2.1)
        self.assertFalse(m.accepts(a));self.assertFalse(m.dispatched(a,{},2.2))
    def test_owner_and_idempotency_and_private_status(self):
        m=self.start();count=len(self.saved);m.start(request(),{},2)
        self.assertEqual(len(self.saved),count);self.assertNotIn('owner',m.public())
        with self.assertRaises(ValueError):m.pulse('1'*32,'3'*32,2)
        bad=request();bad['points'][0]['x']=2
        with self.assertRaises(ValueError):m.start(bad,frame(),2)
    def test_service_restart_never_resumes_saved_motion(self):
        self.start();saved=self.saved[-1];m=ResidentMission();m.restore(saved)
        self.assertEqual(m.phase,'interrupted');self.assertIsNone(m.step(frame(),3))
        with self.assertRaises(ValueError):m.start(dict(request(),id='4'*32),frame(),4)
    def test_trace_rejects_pose_jump_and_keeps_loop_order(self):
        t=MeasuredTrace()
        poses=[(0,0,0),(.5,0,0),(.5,.5,math.pi/2),(0,.5,math.pi),(0,0,-math.pi/2)]
        for i,p in enumerate(poses):t.add(p,i)
        points=t.reverse(poses[-1]);self.assertEqual((points[0]['x'],points[0]['y']),(0,.5))
        self.assertEqual((points[1]['x'],points[1]['y']),(.5,.5))
        with self.assertRaises(ValueError):t.add((3,3,0),9)
    def test_invalid_policies_coordinates_and_map_frames_are_rejected(self):
        for change in [dict(disconnect_policy='ignore_sensors'),dict(disconnect_policy=[]),dict(disconnect_timeout_s=0),dict(map={}),dict(points=[{'x':float('nan'),'y':0,'yaw':0}])]:
            with self.assertRaises((ValueError,TypeError)):specification(dict(request(),**change))


class ResidentAdapterTests(unittest.TestCase):
    def setUp(self):
        self.acl=patch('m20.platform_host.private_file',lambda path:None);self.acl.start();self.addCleanup(self.acl.stop)
    def test_capability_and_import_do_not_open_robot_link(self):
        with tempfile.TemporaryDirectory() as folder:
            driver=Mock();service=ResidentMissionService(None,None,folder,driver=driver,enabled=False)
            self.assertFalse(service.dispatch('/missions/capabilities')['available'])
            self.assertFalse(service.dispatch('/missions/status')['mission']['active'])
            with self.assertRaises(ValueError):service.dispatch('/missions/start',request())
            driver.open.assert_not_called()
    def test_unknown_cancel_persists_tombstone_without_starting_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            driver=Mock();service=ResidentMissionService(None,None,folder,driver=driver,enabled=False)
            with patch.object(service,'_worker'):
                result=service.dispatch('/missions/cancel',dict(id='1'*32,owner='2'*32))
            self.assertTrue(result['revoked']);self.assertFalse(result['matched']);driver.open.assert_not_called()
            self.assertTrue((Path(folder)/'resident-mission.json').is_file())
    def test_actual_receipt_decoder_ignores_acks_wrong_values_and_old_frames(self):
        now=time.monotonic()
        def record(t,c,items,stamp=None):return dict(type=t,command=c,items=items,monotonic_at=now if stamp is None else stamp,at=time.time(),seq=3)
        rows={(1007,2):record(1007,2,dict(PosX=0,PosY=0,Yaw=0,Location=0,Confidence=.9)),
              (1002,6):record(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,Sleep=0,ControlUsageMode=1,Gait=0x3002))),
              (1002,4):record(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))}
        link=Obj(record=lambda t,c:rows.get((t,c),{}),events=[],lock=threading.RLock())
        sensor=Obj(call=lambda *args,**kwargs:dict(protocol=1,age=0,pose=[0,0,0],stamp=1,floor=[[0,0]],obstacles=[]))
        driver=NativeMissionDriver(None,lambda:sensor,'.','192.0.2.1');driver.link=link;driver._map=lambda:MAP
        driver.goal=dict(value=42,sent_monotonic=now-.1)
        for item,stamp,expected in [(dict(Value=42,ErrorCode=0),now,False),
                 (dict(Value=43,Status=4,ErrorCode=0x2300),now,False),
                 (dict(Value=42,Status=4,ErrorCode=0x2300),now-.2,False),
                 (dict(Value=42,Status=4,ErrorCode=0x2300),now,True)]:
            link.events=[record(1003,1,item,stamp)]
            self.assertEqual(driver.frame()['goal_complete'],expected)
    def test_complete_worker_finishes_and_returns_without_pc_pulses(self):
        clock=[1.];pose=[0.,0.,0.];goal=[None];cancelled=[False];commands=[];values=[100];steps=[0]
        class Driver:
            def open(self,expected):pass
            def frame(self):
                if goal[0]:
                    target=goal[0];dx,dy=target['x']-pose[0],target['y']-pose[1];d=math.hypot(dx,dy)
                    if d>.001:
                        fraction=min(1.,.25/d);pose[0]+=dx*fraction;pose[1]+=dy*fraction
                    pose[2]=target['yaw']
                arrived=bool(goal[0] and math.dist(pose[:2],[goal[0]['x'],goal[0]['y']])<.01)
                return frame(pose,clock[0],goal_complete=arrived,still=True)
            def prepare(self,action,packet):
                values[0]+=1;return {'Value':values[0]} if action['kind']=='goal' else None
            def send(self,action,packet,prepared):
                commands.append(copy.deepcopy(action))
                if action['kind']=='goal':goal[0]=action['point'];cancelled[0]=False
                else:goal[0]=None;cancelled[0]=True
                return {'seq':values[0]}
            def stop_frame(self):return dict(cancel_confirmed=cancelled[0],still=cancelled[0])
            def close(self):pass
        with tempfile.TemporaryDirectory() as folder:
            service=ResidentMissionService(None,None,folder,driver=Driver(),clock=lambda:clock[0],enabled=True)
            with patch('bridge.resident_mission.activation_lock',return_value=nullcontext()),patch.object(service,'_worker'):
                service.dispatch('/missions/start',dict(request('finish_return'),disconnect_timeout_s=3))
            def wait(_):
                clock[0]+=.5;steps[0]+=1
                if steps[0]>100:service.done.set()
                return False
            with patch.object(service.done,'wait',side_effect=wait):service._run()
            self.assertTrue(service.engine.job['lost_once']);self.assertEqual(service.engine.phase,'completed');self.assertLess(math.dist(pose[:2],[0,0]),.01)
            self.assertTrue(any(a.get('returning') for a in commands));self.assertEqual(service.engine.job['completed'],2)
            self.assertEqual(commands[-1]['kind'],'cancel')
    def test_restored_terminal_receipt_does_not_require_another_stop(self):
        m=ResidentMission();m.restore(dict(job=dict(request(),digest='x'),phase='completed',trace=[]))
        self.assertEqual(m.phase,'completed');self.assertNotIn('reconciliation_required',m.job)
    def test_map_activation_mapping_and_relocalization_are_blocked_by_resident_task(self):
        from bridge.root_actions import _execute
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);data=root/'data';data.mkdir();maps=root/'maps';maps.mkdir()
            (data/'resident-mission.json').write_text(json.dumps(dict(job={'id':'1'*32},phase='forward')))
            for action in ['apply','mapping','extend_mapping','localize_auto']:
                with patch('bridge.root_actions.run') as run:
                    with self.assertRaisesRegex(ValueError,'驻机'):_execute(dict(action=action,name='synthetic'),maps,data)
                    run.assert_not_called()
    def test_stop_decoder_is_independent_of_failed_sensor_rpc(self):
        now=time.monotonic();rows={
            (1004,1):dict(seq=7,monotonic_at=now,items={'ErrorCode':0}),
            (1002,4):dict(monotonic_at=now,items={'MotionStatus':dict(LinearX=0,LinearY=0,OmegaZ=0)})}
        driver=NativeMissionDriver(None,None,'.','192.0.2.1')
        driver.link=Obj(record=lambda t,c:rows.get((t,c),{}));driver.cancel_receipt=dict(seq=7,sent_monotonic=now-.1)
        self.assertEqual(driver.stop_frame(),dict(cancel_confirmed=True,still=True))
        rows[1004,1]['seq']=6;self.assertFalse(driver.stop_frame()['cancel_confirmed'])
    def test_cancellation_never_targets_another_operators_goal(self):
        driver=NativeMissionDriver(None,None,'.','192.0.2.1')
        driver.link=Obj(record=lambda *args:dict(monotonic_at=time.monotonic(),items=dict(Value=999,Status=1)),command=Mock())
        driver.owned_values={42}
        with self.assertRaisesRegex(ValueError,'其他控制端'):driver.send(dict(kind='cancel'),{})
        driver.link.command.assert_not_called()


if __name__=='__main__':unittest.main()

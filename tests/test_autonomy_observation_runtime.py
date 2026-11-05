"""Actual autonomous _run + scan_now + Hub.proxy with bounded offline transport.

The sensor RPC body and robot feedback are synthetic; the scan runtime, thread,
Explorer, follower, ownership and actual drive arbitration execute unchanged.
No socket, ROS, camera, 8080 or robot connection is made.
"""
import copy
import threading
import time
import unittest
from unittest.mock import patch
from m20.observation_errors import GroundObservationPending
import test_platform as cases
from m20.autonomy_core import Explorer,disk


class RecordingExplorer(Explorer):
    def __init__(self):super().__init__(3);self.ingested=[];self.planned=[]
    def ingest(self,scan):self.ingested.append(scan['stamp']);return super().ingest(scan)
    def plan(self):self.planned.append(self.last_stamp);return super().plan()


class Link(cases.FakeLink):
    def __init__(self,app):self.app=app;self.output=[];super().__init__()
    def record(self,t,c):
        if (t,c)==(1002,4):return dict(at=time.time(),monotonic_at=time.monotonic(),items=dict(MotionStatus=dict(LinearX=0.,LinearY=0.,OmegaZ=0.)))
        row=super().record(t,c);row['at']=time.time();row['monotonic_at']=time.monotonic();return row
    def set_velocity(self,*v,valid_until=None):
        super().set_velocity(*v,valid_until=valid_until)
        self.output.append(dict(at=time.monotonic(),axes=list(v),deadline=valid_until,stamp=self.app.autonomy.scan_timing.get('stamp')))


class AutonomyObservationRuntimeTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):
        self.app.done=True
        with self.app.lock:self.app.autonomy.pause('offline test ended')
        if hasattr(self,'release'):self.release.set()
        if hasattr(self,'worker'):self.worker.join(1.)
        p=self.app.autonomy.observation_pump
        if p:p.cancel();p.join(1.)
        self.app.hub.close();cases.PlatformTests.tearDown(self)

    def configure(self):
        app=self.app;app.demo=False;app.link=Link(app);app.mapping=True
        app.basic.update(MotionState=17,ControlUsageMode=0,Sleep=0,HES=0)
        app.config['nos']='offline-old-context';app.hub.connections.value['targets']['nos']['enabled']=False
        app.hub.connections.value['sensor_source']='nos';app.hub.sdk_mode=dict(state='normal')
        a=app.autonomy;a.mode='explore';a.active=True;a.run_id='offline-run';a.state='preparing';a.explorer=RecordingExplorer()
        a.started=a.lease=time.monotonic();a.deadline=a.started+30;a.context=app.hub.context();a.capture_id='capture-one'
        app.hub.remote_maps=lambda *args,**kwargs:dict(phase='collecting',in_progress=True,mapping=dict(request_id=a.capture_id))
        self.release=threading.Event();self.calls=[]
        return a

    def packet(self,stamp,pose=(0.,0.,0.),age=.08):
        return dict(protocol=1,stamp=float(stamp),age=age,server_elapsed_s=0.,pose=list(pose),
                    floor=[list(p) for p in disk(3.)],obstacles=[],measured_floor=[list(p) for p in disk(3.)])

    def run_worker(self):
        a=self.app.autonomy;self.worker=threading.Thread(target=a._run,args=(a.run_id,None,30),daemon=True);self.worker.start()

    def wait_for(self,predicate,timeout=1.8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return True
            with self.app.lock:self.app.autonomy.lease=time.monotonic()
            time.sleep(.005)
        return bool(predicate())

    def test_reordered_and_duplicate_rpc_frames_never_reach_explorer(self):
        a=self.configure();stamps=[10.,10.1,10.1,10.05,10.2,10.3]
        def transport(base,path,request,timeout,token):
            i=len(self.calls);self.calls.append((base,path));time.sleep(.035)
            return self.packet(stamps[min(i,len(stamps)-1)])
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(self.wait_for(lambda:a.observation_pump and a.observation_pump.status()['counters']['reordered']>=1))
            self.assertTrue(self.wait_for(lambda:len(a.explorer.ingested)>=3),a.message)
            self.assertTrue(a.active,a.message);self.assertNotIn('时间戳回退',a.message)
            self.assertEqual(a.explorer.ingested,sorted(set(a.explorer.ingested)))
            counts=a.observation_pump.status()['counters'];self.assertGreaterEqual(counts['duplicates'],1)
            self.assertNotIn(10.05,a.explorer.ingested)
            with self.app.lock:a.pause('done')
        self.assertTrue(all(base=='offline-old-context' for base,path in self.calls))

    def test_ground_occlusion_survives_long_wait_and_resumes_same_task(self):
        self.occlusion_resume(typed=True)

    def test_sparse_valid_surface_recovers_without_a_passive_wait_becoming_actuator_failure(self):
        self.occlusion_resume(typed=False)

    def occlusion_resume(self,typed):
        import m20.autonomy_runtime as runtime
        a=self.configure();a.deadline=a.started+120
        phase={'blocked':False,'stamp':1.};requests=[];progress_windows=[]
        def transport(base,path,request,timeout,token):
            requests.append(dict(request))
            if phase['blocked'] and typed:raise GroundObservationPending('temporary foreground occlusion','floor_coverage_narrow')
            phase['stamp']+=.1;packet=self.packet(phase['stamp'])
            if phase['blocked']:packet['floor']=[];packet['measured_floor']=[]
            return packet
        progress_type=runtime.MotionProgress
        def progress():
            window=progress_type();progress_windows.append(window);return window
        with patch('m20.platform_hub.rpc',side_effect=transport),patch.object(runtime,'MotionProgress',side_effect=progress):
            self.worker=threading.Thread(target=a._run,args=(a.run_id,None,120),daemon=True);self.worker.start()
            self.assertTrue(self.wait_for(lambda:any(any(row['axes']) for row in self.app.link.output)),a.message)
            run_id=a.run_id;phase['blocked']=True
            self.assertTrue(self.wait_for(lambda:'恢复' in a.message),a.message)
            before=len(self.app.link.output);time.sleep(5.2)
            self.assertTrue(a.active,a.message);self.assertEqual(a.run_id,run_id)
            self.assertTrue(all(not any(row['axes']) for row in self.app.link.output[before:]))
            progress_windows[0].last_at=time.monotonic()-30
            phase['blocked']=False;before=len(self.app.link.output)
            self.assertTrue(self.wait_for(lambda:any(any(row['axes']) for row in self.app.link.output[before:])),a.message)
            self.assertEqual(a.run_id,run_id);self.assertTrue(a.active,a.message)
            self.assertGreaterEqual(len(progress_windows),2)
            self.assertNotIn('after_stamp',requests[0]);self.assertTrue(any('after_stamp' in req for req in requests[1:]))
            with self.app.lock:a.pause('done')

    def test_slow_rpc_does_not_block_cached_control_but_original_deadline_still_stops(self):
        a=self.configure();entered=threading.Event()
        def transport(base,path,request,timeout,token):
            i=len(self.calls);self.calls.append(i)
            if i>=2:entered.set();self.release.wait(.7)
            else:time.sleep(.035)
            return self.packet(10.+i*.1)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(entered.wait(1.))
            self.assertTrue(self.wait_for(lambda:sum(any(row['axes']) for row in self.app.link.output)>=3),a.message)
            moving=[r for r in self.app.link.output if any(r['axes'])];old_deadline=moving[0]['deadline']
            self.assertTrue(all(r['deadline']==old_deadline for r in moving))
            self.assertEqual(len(a.explorer.ingested),1,'the same latest scan must not be re-ingested')
            self.assertTrue(self.wait_for(lambda:any(not any(r['axes']) and r['at']>=old_deadline for r in self.app.link.output)),a.message)
            self.assertFalse(any(any(r['axes']) and r['at']>=r['deadline'] for r in self.app.link.output if r['deadline']))
            with self.app.lock:a.pause('done')
            self.release.set()

    def test_cancel_during_real_scan_rpc_ignores_late_result(self):
        a=self.configure();entered=threading.Event()
        def transport(*args,**kwargs):entered.set();self.release.wait(.7);return self.packet(20.)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(entered.wait(1.))
            with self.app.lock:a.pause('operator cancelled')
            self.worker.join(.15);self.assertFalse(self.worker.is_alive())
            self.release.set();a.observation_pump.join(.5)
            self.assertEqual(a.observation_pump.status()['counters']['published'],0)
            self.assertFalse(any(any(r['axes']) for r in self.app.link.output))

    def test_connection_change_cannot_retarget_old_sensor_request(self):
        a=self.configure();entered=threading.Event()
        def transport(base,*args,**kwargs):self.calls.append(base);entered.set();self.release.wait(.7);return self.packet(20.)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(entered.wait(1.))
            with self.app.lock:self.app.hub.detach_context();self.app.config['nos']='offline-new-context'
            self.release.set();self.assertTrue(self.wait_for(lambda:not a.active),a.message)
            self.assertEqual(self.calls,['offline-old-context'])
            self.assertFalse(any(any(r['axes']) for r in self.app.link.output))

    def test_collection_generation_change_discards_inflight_scan(self):
        a=self.configure();entered=threading.Event()
        def transport(*args,**kwargs):entered.set();self.release.wait(.7);return self.packet(20.)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(entered.wait(1.))
            with self.app.lock:a.capture_id='another-capture'
            self.release.set();self.assertTrue(self.wait_for(lambda:not a.active),a.message)
            self.assertFalse(any(any(r['axes']) for r in self.app.link.output))
            self.assertIn('会话',a.message)

    def test_late_old_run_response_cannot_replace_new_session_cache(self):
        a=self.configure();old_entered=threading.Event()
        def transport(*args,**kwargs):
            i=len(self.calls);self.calls.append(i)
            if i==0:old_entered.set();self.release.wait(.7);return self.packet(9000.)
            time.sleep(.04);return self.packet(100.+i*.1)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();old_worker=self.worker;self.assertTrue(old_entered.wait(1.))
            old_pump=a.observation_pump
            with self.app.lock:
                a.pause('cancel old run');a.active=True;a.run_id='new-run';a.capture_id='capture-two'
                a.generation+=1;a.state='preparing';a.explorer=RecordingExplorer();a.context=self.app.hub.context()
                a.started=a.lease=time.monotonic();a.deadline=a.started+30
            self.run_worker();self.assertTrue(self.wait_for(lambda:len(a.explorer.ingested)>=1))
            self.release.set();old_pump.join(.5);old_worker.join(.5)
            self.assertTrue(a.active,a.message)
            self.assertNotIn(9000.,a.explorer.ingested)
            self.assertLess(a.observation_pump.read()['observation']['stamp'],9000.)
            self.assertEqual(old_pump.status()['counters']['published'],0)
            with self.app.lock:a.pause('done')

    def test_newer_stamp_with_real_pose_jump_still_stops(self):
        a=self.configure()
        def transport(*args,**kwargs):
            i=len(self.calls);self.calls.append(i);time.sleep(.06)
            return self.packet(20.+i*.2,pose=(1.6,0,0) if i>=3 else (0,0,0))
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(self.wait_for(lambda:not a.active,2.))
            self.assertIn('跳变',a.message)
            self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_continuous_packets_overlap_rpc_and_control_without_reingesting_cache(self):
        a=self.configure()
        def transport(*args,**kwargs):
            i=len(self.calls);self.calls.append(i);time.sleep(.07)
            return self.packet(20.+i*.1)
        with patch('m20.platform_hub.rpc',side_effect=transport):
            self.run_worker();self.assertTrue(self.wait_for(lambda:len(a.explorer.ingested)>=6,2.))
            moving=[r for r in self.app.link.output if any(r['axes'])]
            self.assertGreater(len(moving),len(a.explorer.ingested))
            self.assertEqual(a.explorer.ingested,sorted(set(a.explorer.ingested)))
            self.assertFalse(any(r['at']>=r['deadline'] for r in moving))
            with self.app.lock:a.pause('done')

    def test_sustained_real_runtime_forward_nonresponse_brakes_then_changes_direction(self):
        a=self.configure();started=time.monotonic();requested=[];original=a.explorer.request_recovery
        def transport(*args,**kwargs):
            time.sleep(.06);return self.packet(100.+time.monotonic()-started)
        def recover(*args,**kwargs):
            requested.append((time.monotonic(),a.explorer.current_scan['stamp']))
            return original(*args,**kwargs)
        with patch('m20.platform_hub.rpc',side_effect=transport),patch.object(a.explorer,'request_recovery',side_effect=recover):
            self.run_worker()
            self.assertTrue(self.wait_for(lambda:any(abs(r['axes'][1])>.1 for r in self.app.link.output),6.),a.message)
            self.assertTrue(requested)
            self.assertTrue(self.wait_for(lambda:a.explorer.recovery is not None and a.explorer.recovery.get('issued')))
            side=next(r for r in self.app.link.output if abs(r['axes'][1])>.1)
            self.assertGreater(side['stamp'],requested[0][1],'direction change needs a newer stop-feedback frame')
            self.assertTrue(any(not any(r['axes']) and requested[0][0]<=r['at']<side['at'] for r in self.app.link.output))
            self.assertLessEqual(side['deadline'],a.explorer.recovery['deadline_monotonic'])
            self.assertNotIn('持续没有有效位移',a.message)
            with self.app.lock:a.pause('done')

    def test_expired_drive_does_not_charge_an_unissued_recovery_candidate(self):
        from test_precise_clearance import scene
        a=self.configure();late=threading.Event();original=a._drive;start=time.monotonic()
        def transport(*args,**kwargs):
            time.sleep(.04);stamp=100.+time.monotonic()-start
            packet=self.packet(stamp);packet.update(scene(stamp=stamp));return packet
        def drive(run_id,forward,yaw,valid_until=None,side=0.):
            if a.explorer.recovery is not None and any([forward,side,yaw]):
                late.set();valid_until=time.monotonic()-.001
            return original(run_id,forward,yaw,valid_until,side=side)
        with patch('m20.platform_hub.rpc',side_effect=transport),patch.object(a,'_drive',side_effect=drive):
            self.run_worker();self.assertTrue(late.wait(1.5),a.message)
            self.assertEqual(a.explorer.recovery_attempts,0);self.assertEqual(a.explorer.recovery_total_attempts,0)
            self.assertFalse(any(any(row['axes']) for row in self.app.link.output))
            with self.app.lock:a.pause('done')


if __name__=='__main__':unittest.main()

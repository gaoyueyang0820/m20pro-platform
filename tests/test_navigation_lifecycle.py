"""Offline navigation lifecycle regressions; no sockets or robot processes."""
import json,math,sys,time,threading,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_platform as cases
from platform_core import Store
import m20_platform as web


class NavigationLifecycleTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown

    def ready(self,app=None,mode=1,gait=0x3002,x=0,y=0,yaw=0):
        app=app or self.app
        app.link.put(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=mode,Sleep=0,Gait=gait)))
        app.link.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
        app.link.put(1007,2,dict(Location=0,PosX=x,PosY=y,Yaw=yaw))

    def route(self,mode=1,gait=0x3002):
        cases.PlatformTests.live(self);self.ready(mode=mode,gait=gait)
        return self.app.route_save(dict(name='offline lifecycle',map_id='test',points=[dict(x=4,y=0,yaw=0,type=1)]))['route']

    def test_goal_waits_for_mode_and_gait_feedback_in_order(self):
        r=self.route(mode=0,gait=0x1001);a=self.app;a.start_route(dict(id=r['id'],nav_mode=1))
        self.assertEqual(a.nav['phase'],'preparing');self.assertEqual(a.link.sent,[dict(mode=1)])
        for _ in range(3):self.ready(mode=0,gait=0x1001);a.tick(.01)
        self.assertEqual(a.link.sent,[dict(mode=1)])
        self.ready(mode=1,gait=0x1001);a.tick(.01)
        self.assertEqual(a.link.sent[-1]['type'],2);self.assertEqual(a.link.sent[-1]['command'],23)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        # A stale readback with the desired value is not confirmation.
        sent=a.nav['preparation']['gait_sent_at']
        self.ready();a.link.records[1002,6]['at']=sent-.1;a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        self.ready();a.tick(.01)
        goals=[p for p in a.link.sent if p.get('type')==1003]
        self.assertEqual(len(goals),1);self.assertEqual(goals[0]['items']['ObsMode'],0)
        self.assertEqual(a.nav['phase'],'running')

    def test_cancel_during_preparation_cannot_restart_on_late_feedback(self):
        for mode,gait in [(0,0x1001),(1,0x1001)]:
            r=self.route(mode,gait);a=self.app;a.start_route(dict(id=r['id']))
            a.stop();self.assertEqual(a.nav['status'],'cancelled')
            self.ready();a.tick(.01);a.tick(.01)
            self.assertFalse(any(p.get('type') in [1003,1004] for p in a.link.sent))

    def test_preparation_timeout_and_connection_change_never_dispatch_goal(self):
        r=self.route(mode=0,gait=0x1001);a=self.app;a.start_route(dict(id=r['id']))
        a.nav['preparation']['deadline']=time.monotonic()-1;a.tick(.01)
        self.assertEqual(a.nav['status'],'failed');self.ready();a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        self.ready(mode=0,gait=0x1001);a.start_route(dict(id=r['id']))
        a.link=cases.FakeLink();self.ready();a.tick(.01)
        self.assertEqual(a.nav['status'],'failed');self.assertEqual(a.link.sent,[])

    def test_mode_or_gait_ack_without_stationary_feedback_cannot_start(self):
        r=self.route(mode=1,gait=0x1001);a=self.app;a.start_route(dict(id=r['id']))
        self.ready();a.link.put(1002,4,dict(MotionStatus=dict(LinearX=.2,LinearY=0,OmegaZ=0)));a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        a.stop();self.ready();a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
    def test_standing_sleep_is_woken_without_changing_auto_sleep_configuration(self):
        r=self.route();a=self.app
        a.link.records[1002,6]['items']['BasicStatus']['Sleep']=1
        a.link.put(1101,7,dict(Auto=False,Time=15,Sleep=True))
        a.start_route(dict(id=r['id']))
        self.assertEqual(len(a.link.sent),1);request=a.link.sent[0]
        self.assertEqual((request['type'],request['command']),(1101,6))
        self.assertEqual(request['items'],dict(Auto=False,Time=15,Sleep=False))
        self.ready();a.tick(.01)
        self.assertEqual(len([p for p in a.link.sent if p.get('type')==1003]),1)
    def test_sleep_without_configuration_uses_sleep_only_and_never_stands(self):
        r=self.route();a=self.app;a.link.records[1002,6]['items']['BasicStatus']['Sleep']=1
        a.start_route(dict(id=r['id']))
        self.assertEqual((a.link.sent[0]['type'],a.link.sent[0]['command']),(1101,6))
        self.assertEqual(a.link.sent[0]['items'],dict(Sleep=False))
        a.stop();self.ready();a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        self.ready();a.link.records[1002,6]['items']['BasicStatus'].update(Sleep=1,MotionState=4)
        before=len(a.link.sent)
        with self.assertRaisesRegex(ValueError,'基础状态'):a.start_route(dict(id=r['id']))
        self.assertEqual(len(a.link.sent),before)

    def test_real_zero_progress_trace_waits_then_cancels_once(self):
        fixture=Path(__file__).parent/'fixtures/runtime_replay/native_zero_progress.json'
        if not fixture.is_file():self.skipTest('现场回放未随分享包提供')
        rows=[r for r in json.loads(fixture.read_text())['rows'] if r['t']>=10]
        self.assertGreater(len(rows),80)
        r=self.route();a=self.app;t=[0.];wall=time.time()
        with patch.object(web.time,'time',side_effect=lambda:wall+t[0]),patch.object(web.time,'monotonic',side_effect=lambda:t[0]):
            self.ready(x=rows[0]['x'],y=rows[0]['y'],yaw=rows[0]['yaw']);a.start_route(dict(id=r['id']))
            waiting=False
            for row in rows:
                t[0]=row['t']-rows[0]['t'];self.ready(x=row['x'],y=row['y'],yaw=row['yaw'])
                a.link.put(1007,1,dict(Value=a.nav['value'],Status=3,ErrorCode=0))
                a.tick(.01)
                waiting|=a.nav.get('execution_state')=='waiting_progress'
            self.assertTrue(waiting);self.assertEqual(a.nav['status'],'cancelling')
            self.assertEqual(a.nav['phase'],'cancel_unconfirmed')
            self.assertEqual(a.nav['stop_reason'],'no_progress');self.assertEqual(a.nav['completed'],0)
            self.assertEqual([p['type'] for p in a.link.sent],[1003,1004])
            with self.assertRaisesRegex(ValueError,'已有任务'):a.start_route(dict(id=r['id']))
            # Only a matching stop receipt makes a safe paused state.
            t[0]+=.1;a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
            self.assertEqual(a.nav['status'],'paused')
            self.assertEqual(a.nav['recovery']['automatic_resends'],0)

    def test_real_heading_progress_counts_but_oscillation_does_not(self):
        r=self.route();a=self.app;t=[0.];wall=time.time()
        with patch.object(web.time,'time',side_effect=lambda:wall+t[0]),patch.object(web.time,'monotonic',side_effect=lambda:t[0]):
            self.ready(yaw=1.2);a.start_route(dict(id=r['id']));a.tick(.01)
            for seconds,yaw in [(4,1.0),(8,.8),(12,.6)]:
                t[0]=seconds;self.ready(yaw=yaw);a.tick(.01)
                self.assertEqual(a.nav['execution_state'],'moving')
            for seconds,yaw in [(16,.64),(20,.58),(24,.63),(28,.57),(38,.60)]:
                t[0]=seconds;self.ready(yaw=yaw);a.tick(.01)
            self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.nav['stop_reason'],'no_progress')

    def test_stop_reuses_cancel_receipt_and_operator_cancel_wins(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']))
        a._cancel_navigation('no_progress','paused');seq=a.cancel_request['seq']
        a.stop();a.stop();self.assertEqual(a.cancel_request['seq'],seq)
        self.assertEqual([p['type'] for p in a.link.sent],[1003,1004])
        a.link.put(1004,1,dict(ErrorCode=0),seq=seq);a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelled')
    def test_robot_pause_wins_over_a_queued_completion_and_never_advances(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']))
        a.link.lock=threading.RLock()
        a.link.events=[dict(type=1007,command=1,at=time.time(),items=dict(Value=a.nav['value'],Status=4,ErrorCode=0))]
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=255,ErrorCode=0))
        a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.nav['completed'],0)
        self.assertEqual(a.nav['stop_reason'],'robot_paused')
        self.assertEqual([p['type'] for p in a.link.sent],[1003,1004])
    def test_vendor_docking_is_not_misclassified_as_ordinary_no_progress(self):
        self.route();a=self.app
        r=a.route_save(dict(name='dock',map_id='test',points=[dict(x=4,y=0,yaw=0,type=3)]))['route']
        a.start_route(dict(id=r['id']));a.nav['progress_monotonic']=time.monotonic()-40
        self.ready(x=4);a.link.records[1002,6]['items']['BasicStatus'].update(MotionState=4,Charge=2)
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=5,ErrorCode=0));a.tick(.01)
        self.assertEqual(a.nav['status'],'running');self.assertEqual(a.nav['execution_state'],'docking')
        self.assertFalse(any(p.get('type')==1004 for p in a.link.sent))
        a.link.put(1007,1,dict(Value=a.nav['value'],Status=4,ErrorCode=0));a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.nav['completed'],1)

    def test_restarted_platform_does_not_reuse_value_or_accept_old_success(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']));old=a.nav['value']
        b=web.Platform(Store(a.store.path));b.demo=False;b.link=cases.FakeLink();b.basic=dict(HES=0,MotionState=17,ControlUsageMode=1)
        b.proxy=lambda *args,**kwargs:dict(active='test',phase='idle',in_progress=False)
        b.hub.remote_maps=lambda path,req=None,timeout=120,context=None:b.proxy('nos',path,req,timeout=timeout)
        self.ready(b);b.start_route(dict(id=r['id']))
        self.assertGreater(b.nav['value'],old)
        b.link.put(1007,1,dict(Value=old,Status=0,ErrorCode=0x2300));b.tick(.01)
        self.assertEqual(b.nav['completed'],0);self.assertEqual(b.nav['status'],'running')
        b.link.put(1007,1,dict(Value=b.nav['value'],Status=0,ErrorCode=0x2300));b.tick(.01)
        self.assertEqual(b.nav['completed'],1);self.assertEqual(b.nav['status'],'cancelling')

    def test_counter_is_durable_before_goal_emission(self):
        r=self.route();a=self.app
        with patch.object(a.store,'save',side_effect=OSError('disk full')):
            with self.assertRaisesRegex(ValueError,'编号'):a.start_route(dict(id=r['id']))
        self.assertEqual(a.nav['status'],'failed')
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
    def test_preparation_send_error_cannot_later_send_a_goal(self):
        r=self.route(mode=0,gait=0x1001);a=self.app
        with patch.object(a.link,'set_mode',side_effect=OSError('offline')):
            with self.assertRaisesRegex(ValueError,'发送失败'):a.start_route(dict(id=r['id']))
        self.assertEqual(a.nav['status'],'failed');self.ready();a.tick(.01)
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
    def test_uncertain_goal_and_failed_cancel_remain_blocked_until_explicit_stop(self):
        r=self.route();a=self.app;original=a.link.command
        with patch.object(a.link,'command',side_effect=OSError('offline')):
            with self.assertRaisesRegex(ValueError,'发送结果未确认'):a.start_route(dict(id=r['id']))
        self.assertEqual(a.nav['status'],'cancelling');self.assertIsNone(a.cancel_request)
        with self.assertRaisesRegex(ValueError,'已有任务'):a.start_route(dict(id=r['id']))
        a.stop();self.assertIsNotNone(a.cancel_request)
        self.assertEqual(a.link.sent[-1]['type'],1004)
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelled')
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
    def test_monotonic_feedback_confirms_completion_and_cancel_after_wall_rollback(self):
        r=self.route();a=self.app;wall=[10000.];mono=[100.];original=a.link.command
        def command(*args):
            reply=original(*args);reply['sent_monotonic']=time.monotonic();return reply
        def fresh():
            self.ready()
            for record in a.link.records.values():record['monotonic_at']=mono[0]
        with patch.object(web.time,'time',side_effect=lambda:wall[0]),patch.object(web.time,'monotonic',side_effect=lambda:mono[0]),patch.object(a.link,'command',side_effect=command):
            fresh();a.start_route(dict(id=r['id']))
            wall[0]-=1000;mono[0]+=.2;fresh()
            a.link.put(1007,1,dict(Value=a.nav['value'],Status=4,ErrorCode=0));a.link.records[1007,1]['monotonic_at']=mono[0]
            a.tick(.01);self.assertEqual(a.nav['status'],'cancelling')
            wall[0]-=1000;mono[0]+=.2;fresh()
            a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.link.records[1004,1]['monotonic_at']=mono[0]
            a.tick(.01);self.assertEqual(a.nav['status'],'completed')
            self.assertTrue(a.snapshot()['online'])
    def test_disconnect_records_unconfirmed_stop_and_reconnect_does_not_resume(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']))
        counter=[time.monotonic()]
        def mono():counter[0]+=.2;return counter[0]
        with patch.object(web.time,'monotonic',side_effect=mono),patch.object(web.time,'sleep'):
            result=a.connect(dict(mode='demo'))
        self.assertTrue(a.demo);self.assertIsNone(a.link);self.assertTrue(result['warnings'])
        self.assertEqual(Store(a.store.path).value['previous_connection_stop']['state'],'unconfirmed')
        a.demo=False;a.link=cases.FakeLink();self.ready();a.link.put(1007,1,dict(Value=101,Status=3,ErrorCode=0));a.tick(.01)
        self.assertEqual(a.store.value['previous_connection_stop']['state'],'active_after_reconnect')
        self.assertEqual(a.link.sent,[])
        with self.assertRaisesRegex(ValueError,'机器人端'):a.start_route(dict(id=r['id']))
        a.link.put(1007,1,dict(Value=101,Status=0,ErrorCode=0));self.ready();a.tick(.01)
        self.assertEqual(a.store.value['previous_connection_stop']['state'],'idle_observed_after_reconnect')
        self.assertEqual(a.nav['status'],'idle');self.assertEqual(a.link.sent,[])
    def test_disconnect_can_confirm_stop_or_report_disk_failure_without_blocking_exit(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']));original=a.link.command
        def confirmed(*args):
            reply=original(*args)
            if args[:2]==(1004,1):a.link.put(1004,1,dict(ErrorCode=0),seq=reply['seq'])
            return reply
        with patch.object(a.link,'command',side_effect=confirmed),patch.object(a.store,'save',side_effect=OSError('full')):
            result=a.connect(dict(mode='demo'))
        self.assertTrue(a.demo);self.assertEqual(result['previous_connection_stop']['state'],'request_confirmed')
        self.assertTrue(any('磁盘' in text for text in result['warnings']))
    def test_existing_robot_task_is_not_overwritten_after_local_restart(self):
        r=self.route();a=self.app;a.link.put(1007,1,dict(Value=101,Status=3,ErrorCode=0))
        with self.assertRaisesRegex(ValueError,'机器人端'):a.start_route(dict(id=r['id']))
        self.assertEqual(a.link.sent,[])
        a.stop();self.assertEqual(a.nav['status'],'cancelling');self.assertEqual(a.link.sent[-1]['type'],1004)
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelled')
        # Accepted cancellation does not let a still-running status get ignored.
        a.start_route(dict(id=r['id']));self.assertEqual(a.nav['phase'],'preparing')
        self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
        a.link.put(1007,1,dict(Value=101,Status=0,ErrorCode=0));self.ready();a.tick(.01)
        self.assertEqual(len([p for p in a.link.sent if p.get('type')==1003]),1)
    def test_slow_map_validation_is_cancellable_before_any_mode_or_goal(self):
        r=self.route(mode=0,gait=0x1001);a=self.app;entered=threading.Event();release=threading.Event();errors=[]
        def proxy(role,path,*args,**kwargs):
            if path=='/maps/build/status':return dict(phase='saved',in_progress=False)
            if path=='/status':entered.set();release.wait(2);return dict(active='test')
            raise AssertionError(path)
        a.proxy=proxy
        def start():
            try:a.start_route(dict(id=r['id']))
            except Exception as exc:errors.append(str(exc))
        thread=threading.Thread(target=start);thread.start()
        try:
            self.assertTrue(entered.wait(1));self.assertEqual(a.nav['phase'],'validating')
            with a.lock:a.stop()
            self.assertEqual(a.nav['status'],'cancelled');release.set();thread.join(2)
            self.assertFalse(thread.is_alive());self.assertEqual(a.link.sent,[])
            self.assertTrue(any('取消' in error for error in errors))
        finally:release.set();thread.join(2)
    def test_unknown_or_running_mapping_cannot_start_navigation(self):
        r=self.route();a=self.app
        for phase,in_progress in [('collecting',True),('saving',True),('unconfirmed',True)]:
            a.proxy=lambda *args,**kwargs:dict(active='test',phase=phase,in_progress=in_progress)
            with self.assertRaisesRegex(ValueError,'建图'):a.start_route(dict(id=r['id']))
            self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
    def test_slow_resume_cannot_restart_after_operator_cancel(self):
        r=self.route();a=self.app;a.start_route(dict(id=r['id']))
        a.stop();a.nav['after_cancel']='paused'
        a.link.put(1004,1,dict(ErrorCode=0),seq=a.cancel_request['seq']);a.tick(.01)
        original_goals=len([p for p in a.link.sent if p.get('type')==1003])
        entered=threading.Event();release=threading.Event();errors=[]
        def proxy(role,path,*args,**kwargs):
            if path=='/maps/build/status':return dict(phase='saved',in_progress=False)
            if path=='/status':entered.set();release.wait(2);return dict(active='test')
            raise AssertionError(path)
        a.proxy=proxy
        def resume():
            try:a.resume_route()
            except Exception as exc:errors.append(str(exc))
        thread=threading.Thread(target=resume);thread.start()
        try:
            self.assertTrue(entered.wait(1))
            with a.lock:a.stop()
            release.set();thread.join(2)
            self.assertEqual(a.nav['status'],'cancelled')
            self.assertEqual(len([p for p in a.link.sent if p.get('type')==1003]),original_goals)
            self.assertTrue(any('取消' in error for error in errors))
        finally:release.set();thread.join(2)


if __name__=='__main__':unittest.main()

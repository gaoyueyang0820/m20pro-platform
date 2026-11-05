"""Production greeting coordination with synthetic feedback and deterministic waits.

No robot, socket, local web server, SDK process or real actuator is used. The
test replaces the module's thread/wait dependency, not the greeting worker.
"""
import copy
import importlib
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
import test_platform as cases
import m20.greeting as greeting


class RecordingLink(cases.FakeLink):
    def __init__(self):
        super().__init__();self.drive_lock=threading.RLock();self.rows=[];self.closed=False
        self.stale_basic=False;self.stale_motion=False;self.fail=None
        self.put(1002,6,dict(BasicStatus=dict(HES=0,MotionState=17,ControlUsageMode=0,
                                             Sleep=0,Gait=0x3002,Direction=0)))
        self.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
    def record(self,t,c):
        record=super().record(t,c)
        if record:
            stale=self.stale_basic if (t,c)==(1002,6) else self.stale_motion if (t,c)==(1002,4) else False
            record['at']=time.time()-(5 if stale else 0)
        return record
    def command(self,t,c,items,label):
        row=dict(type=t,command=c,items=copy.deepcopy(items),label=label,closed=self.closed)
        self.rows.append(row)
        if self.fail:self.fail(row)
        return super().command(t,c,items,label)
    def set_velocity(self,*values,valid_until=None):
        with self.drive_lock:
            self.rows.append(dict(velocity=list(values),closed=self.closed))
            return super().set_velocity(*values,valid_until=valid_until)
    def close(self):
        self.set_velocity(0,0,0);self.closed=True


def nonzero_axis(row):
    return (row.get('type'),row.get('command'))==(2,21) and any(row.get('items',{}).get(k,0) for k in ['X','Y','Z','Roll','Pitch','Yaw'])
def restore(row):return (row.get('type'),row.get('command')) in [(1101,5),(2,23)]


class GreetingRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime=importlib.import_module('m20.greeting_runtime')
        self.gate=threading.Event();self.release=threading.Event();self.phase=None;self.threads=[];self.start_fails=False;self.link=None
        outer=self
        class ControlledEvent(threading.Event):
            def wait(self,timeout=None):
                rows=outer.link.rows if outer.link else []
                is_worker=threading.current_thread().name.startswith('greeting-test-worker')
                matches=(outer.phase=='light' and any(r.get('items',{}).get('CustomMode') is True for r in rows) or
                         outer.phase=='motion' and any(nonzero_axis(r) for r in rows))
                if is_worker and matches and not self.is_set():
                    outer.gate.set();outer.release.wait(2)
                return super().wait(min(timeout,.005) if timeout is not None else timeout)
        def thread_factory(target,args=(),kwargs=None,**options):
            if outer.start_fails:
                def fail():raise RuntimeError('synthetic thread start failure')
                return SimpleNamespace(start=fail)
            worker=threading.Thread(target=target,args=args,kwargs=kwargs or {},daemon=True,
                                    name='greeting-test-worker-%d'%len(outer.threads))
            outer.threads.append(worker);return worker
        namespace=SimpleNamespace(Thread=thread_factory,Event=ControlledEvent,Lock=threading.Lock,
                                  RLock=threading.RLock,current_thread=threading.current_thread)
        self.patches=[patch.object(self.runtime,'threading',namespace),
                      patch.object(self.runtime,'LIGHT_SECONDS',.03),
                      patch.object(self.runtime,'ZERO_SECONDS',.02)]
        original=greeting.nod_plan
        self.patches.append(patch.object(greeting,'nod_plan',side_effect=lambda *a,**k:original(amplitude=.1,periods=1,period_s=.5,hz=20)))
        for item in self.patches:item.start()
        self.temp=tempfile.TemporaryDirectory()
        self.app=cases.web.Platform(cases.Store(Path(self.temp.name)/'workspace.json'))
        self.app.demo=False;self.link=RecordingLink();self.app.link=self.link
        self.app.basic=self.link.record(1002,6)['items']['BasicStatus']
        self.app.hub.sdk_mode=dict(state='normal')
    def tearDown(self):
        self.release.set();self.app.done=True
        self.app.cancel_greeting('offline test finished')
        for worker in self.threads:worker.join(2)
        self.app.hub.close();self.temp.cleanup()
        for item in reversed(self.patches):item.stop()
        self.assertFalse(any(worker.is_alive() for worker in self.threads),'greeting worker leaked')
    def start_at(self,phase,motion='nod'):
        self.phase=phase;result=self.app.greet(style='breathe',motion=motion)
        self.assertTrue(result['ok']);self.assertTrue(self.gate.wait(1),'worker did not reach requested wait')
        return result
    def finish_worker(self):
        self.release.set()
        for worker in self.threads:worker.join(2)
        self.assertFalse(any(worker.is_alive() for worker in self.threads))
        self.assertFalse(self.app._greet_busy)
    def quick_action(self,operation):
        finished=threading.Event();errors=[]
        def run():
            try:operation()
            except Exception as exc:errors.append(exc)
            finally:finished.set()
        worker=threading.Thread(target=run,daemon=True);worker.start()
        try:self.assertTrue(finished.wait(.25),'stop/takeover blocked behind greeting sequence')
        finally:
            if not finished.is_set():self.release.set()
            worker.join(2)
        if errors:raise errors[0]

    def test_stop_during_light_wait_cannot_late_start_body_motion(self):
        self.start_at('light');self.quick_action(self.app.stop);cut=len(self.link.rows)
        self.finish_worker();self.assertFalse(any(nonzero_axis(r) or restore(r) for r in self.link.rows[cut:]))

    def test_stop_during_motion_wait_does_not_wait_for_whole_drive_lock_sequence(self):
        self.start_at('motion');self.quick_action(self.app.stop);cut=len(self.link.rows)
        self.finish_worker();self.assertFalse(any(nonzero_axis(r) or restore(r) for r in self.link.rows[cut:]))
        self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_manual_takeover_is_not_overwritten_by_late_cleanup_or_restore(self):
        self.start_at('motion');self.quick_action(lambda:self.app.set_velocity(dict(forward=.2)));cut=len(self.link.rows)
        self.finish_worker();self.assertEqual(self.link.v,(.2,0,0))
        self.assertFalse(any((r.get('type'),r.get('command'))==(2,21) or restore(r) or 'velocity' in r for r in self.link.rows[cut:]))

    def test_reconnect_discards_old_worker_without_old_link_cleanup_or_new_link_output(self):
        self.start_at('light');old=self.link;self.quick_action(lambda:self.app.connect(dict(mode='demo')));cut=len(old.rows)
        self.finish_worker();self.assertTrue(self.app.demo);self.assertIsNone(self.app.link)
        self.assertEqual(old.rows[cut:],[])

    def test_stale_basic_or_motion_feedback_cannot_admit_body_greeting(self):
        for field in ['stale_basic','stale_motion']:
            with self.subTest(field=field):
                setattr(self.link,field,True)
                with self.assertRaises(ValueError):self.app.greet(motion='nod')
                self.assertFalse(self.app._greet_busy);self.assertFalse(any(nonzero_axis(r) for r in self.link.rows))
                setattr(self.link,field,False)

    def test_feedback_expiry_during_sequence_stops_nonzero_frames(self):
        self.start_at('motion');self.link.stale_motion=True;cut=len(self.link.rows)
        self.finish_worker();self.assertFalse(any(nonzero_axis(r) or restore(r) for r in self.link.rows[cut:]))

    def test_delayed_sender_stops_instead_of_replaying_a_burst(self):
        self.start_at('motion');cut=len(self.link.rows)
        # The fixture shortens earlier waits, so allow for its <=100 ms
        # head-start as well as the production 150 ms lateness threshold.
        time.sleep(.35)
        self.finish_worker()
        self.assertFalse(any(nonzero_axis(r) for r in self.link.rows[cut:]))
        self.assertEqual(self.app.greeting.snapshot()['state'],'cancelled')

    def test_demo_stop_removes_preview_and_does_not_keep_motion_owner(self):
        self.app.demo=True;self.app.link=None
        self.app.greet(motion='nod')
        self.app.stop();self.finish_worker()
        self.assertIsNone(self.app._greet_demo)
        self.assertFalse(self.app.greeting.snapshot()['active'])
        self.assertEqual(self.link.rows,[])

    def test_hard_estop_and_changed_mode_never_trigger_old_state_restore(self):
        for changed in ['estop','mode']:
            with self.subTest(changed=changed):
                self.gate.clear();self.release.clear();self.link.rows.clear()
                basic=self.link.records[1002,6]['items']['BasicStatus'];basic.update(HES=0,ControlUsageMode=0)
                self.link.inhibit=False;self.app.estopped=False
                self.start_at('motion');cut=len(self.link.rows)
                if changed=='estop':basic['HES']=1;self.link.inhibit=True
                else:basic['ControlUsageMode']=2
                self.finish_worker();self.assertFalse(any(nonzero_axis(r) or restore(r) for r in self.link.rows[cut:]))

    def test_exception_after_operator_estop_does_not_restore_mode_or_gait(self):
        def fail(row):
            if nonzero_axis(row):self.app.estopped=True;self.link.inhibit=True;raise OSError('synthetic send failure')
        self.link.fail=fail;self.app.greet(motion='nod');self.finish_worker()
        self.assertFalse(any(restore(r) for r in self.link.rows));self.assertTrue(self.app.estopped)

    def test_thread_start_failure_releases_busy_for_a_later_request(self):
        self.start_fails=True
        with self.assertRaises(RuntimeError):self.app.greet(motion='nod')
        self.assertFalse(self.app._greet_busy)
        self.start_fails=False;self.app.greet();self.finish_worker()

    def test_light_only_failure_never_emits_axes_or_restoration(self):
        def fail(row):
            if row.get('items',{}).get('CustomMode') is True:raise OSError('synthetic light failure')
        self.link.fail=fail;self.app.greet();self.finish_worker()
        self.assertFalse(any((r.get('type'),r.get('command'))==(2,21) or restore(r) for r in self.link.rows))

    def test_motion_greeting_rejects_sdk_navigation_or_mapping_owner(self):
        for owner in ['sdk','navigation','mapping','autonomy']:
            with self.subTest(owner=owner):
                self.app.hub.sdk_mode=dict(state='sdk' if owner=='sdk' else 'normal')
                self.app.nav['status']='running' if owner=='navigation' else 'idle'
                self.app.mapping=owner=='mapping';self.app.autonomy.active=owner=='autonomy'
                with self.assertRaises(ValueError):self.app.greet(motion='nod')
                self.assertFalse(self.app._greet_busy)
        self.app.nav['status']='idle';self.app.mapping=False;self.app.autonomy.active=False;self.app.hub.sdk_mode=dict(state='normal')

    def test_invalid_motion_name_is_not_silently_treated_as_bob(self):
        with self.assertRaises(ValueError):self.app.greet(motion='unknown-motion')
        self.assertFalse(self.app._greet_busy)

    def test_completed_sequence_releases_owner_without_changing_mode_or_gait(self):
        result=self.app.greet(motion='nod');self.assertTrue(result['ok']);self.finish_worker()
        self.assertTrue(any(nonzero_axis(r) for r in self.link.rows))
        self.assertFalse(any(restore(r) for r in self.link.rows))
        self.assertEqual(self.link.v,(0,0,0));self.assertFalse(self.app.greeting.snapshot()['active'])

    def test_navigation_sdk_joint_and_autonomy_starts_cannot_enter_body_session(self):
        self.start_at('light')
        with self.assertRaisesRegex(ValueError,'问候'):self.app.start_route(dict(id='not-read-before-owner-check'))
        with self.assertRaisesRegex(ValueError,'问候'):self.app.joint_control('arm',dict(rig_confirmed=True))
        with self.assertRaisesRegex(ValueError,'问候'):self.app.hub.post('/api/sdk/connect',{})
        with self.assertRaisesRegex(ValueError,'问候'):self.app.autonomy.start(dict(mode='explore'))
        self.app.stop();self.finish_worker()

    def test_late_old_worker_cannot_clear_the_new_greeting_owner(self):
        self.start_at('light');old_release=self.release;old_worker=self.threads[-1]
        self.app.cancel_greeting('operator cancelled first greeting')
        self.release=threading.Event();self.gate.clear()
        newer=self.app.greet(motion='nod');self.assertTrue(self.gate.wait(1))
        old_release.set();old_worker.join(1);self.assertFalse(old_worker.is_alive())
        self.assertTrue(self.app._greet_busy)
        self.assertEqual(self.app.greeting.snapshot()['run_id'],newer['run_id'])
        self.assertTrue(self.app.greeting.snapshot()['active'])
        self.app.stop();self.finish_worker()


if __name__=='__main__':unittest.main()

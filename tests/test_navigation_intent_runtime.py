"""Real start/cancel coordinators with mock transport and bounded thread races."""
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
import test_platform as cases
import test_navigation_lifecycle as native_cases
import test_point_navigation_runtime as point_cases
import m20.point_navigation as runtime


class ScopedNativeStartTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    ready=native_cases.NavigationLifecycleTests.ready
    route=native_cases.NavigationLifecycleTests.route
    def request(self,value='synthetic-intent-0001',**fields):
        return dict(client_intent=value,server_instance=self.app.instance_id,
                    connection_generation=self.app._plan_generation,**fields)
    def test_cancel_before_start_rejects_the_late_request_without_commands(self):
        route=self.route();req=self.request(id=route['id'])
        result=self.app.cancel_navigation_intent(req)
        self.assertTrue(result['revoked']);self.assertFalse(result['matched'])
        with self.assertRaisesRegex(ValueError,'撤销'):self.app.start_route(req)
        self.assertEqual(self.app.link.sent,[])
    def test_cancellation_after_native_goal_only_stops_its_own_task_once(self):
        route=self.route();req=self.request(id=route['id']);self.app.start_route(req)
        self.assertEqual(self.app.nav['client_intent'],req['client_intent'])
        result=self.app.cancel_navigation_intent(req);self.app.cancel_navigation_intent(req)
        self.assertTrue(result['matched']);self.assertEqual(self.app.nav['status'],'cancelling')
        self.assertEqual(len([r for r in self.app.link.sent if r.get('type')==1004]),1)
    def test_revoked_other_intent_does_not_stop_active_native_task(self):
        route=self.route();old=self.request();self.app.cancel_navigation_intent(old)
        current=self.request('synthetic-intent-0002',id=route['id']);self.app.start_route(current)
        before=list(self.app.link.sent);nav=self.app.nav
        with patch.object(self.app.link,'set_velocity',wraps=self.app.link.set_velocity) as velocity:
            result=self.app.cancel_navigation_intent(old);velocity.assert_not_called()
        self.assertFalse(result['matched']);self.assertIs(self.app.nav,nav)
        self.assertEqual(self.app.nav['status'],'running');self.assertEqual(self.app.link.sent,before)
    def test_slow_map_validation_cannot_start_after_matching_revoke(self):
        route=self.route(mode=0,gait=0x1001);req=self.request(id=route['id'])
        entered=threading.Event();release=threading.Event();errors=[]
        def proxy(role,path,*args,**kwargs):
            if path=='/maps/build/status':return dict(phase='saved',in_progress=False)
            if path=='/status':entered.set();release.wait(2);return dict(active='test')
            raise AssertionError(path)
        self.app.proxy=proxy
        def start():
            try:self.app.start_route(req)
            except ValueError as exc:errors.append(str(exc))
        worker=threading.Thread(target=start);worker.start()
        try:
            self.assertTrue(entered.wait(1));self.app.cancel_navigation_intent(req)
            release.set();worker.join(2);self.assertFalse(worker.is_alive())
            self.assertTrue(errors);self.assertEqual(self.app.link.sent,[])
            self.assertEqual(self.app.nav['status'],'cancelled')
        finally:release.set();worker.join(2)
    def test_late_goal_planning_cancellation_does_not_invalidate_another_start(self):
        route=self.route();old=self.request(map_id='test',x=1,y=0)
        entered=threading.Event();release=threading.Event();errors=[]
        def plan(req):entered.set();release.wait(2);return dict(plan_id='late-plan')
        def start():
            try:self.app.start_goal(old)
            except ValueError as exc:errors.append(str(exc))
        with patch.object(self.app,'plan_goal',side_effect=plan):
            worker=threading.Thread(target=start);worker.start()
            try:
                self.assertTrue(entered.wait(1));self.app.cancel_navigation_intent(old)
                other=self.request('synthetic-intent-0002',id=route['id']);self.app.start_route(other)
                sent=list(self.app.link.sent);self.app.cancel_navigation_intent(old)
                release.set();worker.join(2)
                self.assertFalse(worker.is_alive());self.assertTrue(errors)
                self.assertEqual(self.app.nav['client_intent'],other['client_intent'])
                self.assertEqual(self.app.nav['status'],'running');self.assertEqual(self.app.link.sent,sent)
            finally:release.set();worker.join(2)
    def test_old_connection_scope_cannot_start_or_cancel_a_new_connection_task(self):
        route=self.route();req=self.request(id=route['id']);self.app._plan_generation+=1
        with self.assertRaisesRegex(ValueError,'旧服务或旧连接'):self.app.start_route(req)
        with self.assertRaisesRegex(ValueError,'旧服务或旧连接'):self.app.cancel_navigation_intent(req)
        self.assertEqual(self.app.link.sent,[])
    def test_snapshot_advertises_scoped_cancellation_protocol(self):
        self.assertEqual(self.app.snapshot()['navigation_preparation']['client_intent_protocol'],1)


class ScopedPlatformStartTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    ready=point_cases.PointNavigationRuntimeTests.ready
    setup_live=point_cases.PointNavigationRuntimeTests.setup_live
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def test_platform_worker_never_drives_after_its_start_is_revoked(self):
        app=self.setup_live();workers=point_cases.DeferredWorkers()
        req=dict(map_id=self.map['id'],x=8.05,y=3.05,engine='platform',client_intent='synthetic-platform-0001',
                 server_instance=app.instance_id,connection_generation=app._plan_generation)
        with patch.object(runtime.threading,'Thread',side_effect=workers.thread):app.start_goal(req)
        self.assertEqual(app.nav['client_intent'],req['client_intent'])
        result=app.cancel_navigation_intent(req);self.assertTrue(result['matched']);workers.run()
        self.assertFalse(app.point_navigation.active)
        self.assertFalse(any(any(row['v']) for row in app.link.velocities))
    def test_unknown_intent_cannot_stop_platform_navigation(self):
        app=self.setup_live();workers=point_cases.DeferredWorkers()
        req=dict(map_id=self.map['id'],x=8.05,y=3.05,engine='platform',client_intent='synthetic-platform-0001',
                 server_instance=app.instance_id,connection_generation=app._plan_generation)
        with patch.object(runtime.threading,'Thread',side_effect=workers.thread):app.start_goal(req)
        old=dict(req,client_intent='synthetic-platform-old1');before=len(app.link.velocities)
        result=app.cancel_navigation_intent(old)
        self.assertFalse(result['matched']);self.assertTrue(app.point_navigation.active)
        self.assertEqual(len(app.link.velocities),before)


if __name__=='__main__':unittest.main()

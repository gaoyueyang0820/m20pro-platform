"""Platform patrol startup/resume dispatch, entirely synthetic."""
import copy
import hashlib
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import test_platform as cases


class PlatformRouteDispatchTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def prepare(self):
        a=self.app
        route=a.route_save(dict(name='inspection',map_id='demo',points=[dict(x=-5.,y=-3.,yaw=.6,type=0),dict(x=-4.,y=-3.,yaw=1.2,type=1)]))['route']
        source=a.store.path.parent/'synthetic-map';source.write_bytes(b'synthetic')
        self.planning=[]
        def plan(req):
            self.planning.append(dict(req));goal={k:req[k] for k in ['x','y','yaw']};key='plan-'+str(len(self.planning))
            a.goal_plans[key]=dict(at=time.monotonic(),map_id='demo',execution_engine='platform',
                connection_generation=a._plan_generation,link=a.link,demo=a.demo,
                start=[a.pose['PosX'],a.pose['PosY'],a.pose['Yaw']],source=str(source),fingerprint=hashlib.sha256(source.read_bytes()).hexdigest(),
                result=dict(goal=goal,requested_goal=copy.deepcopy(goal),path=[goal],planner='synthetic'))
            return dict(plan_id=key)
        a.plan_goal=Mock(side_effect=plan);a.point_navigation.start=Mock(return_value=dict(ok=True,execution='platform_trajectory'))
        return a,route
    def test_platform_patrol_dispatch_preserves_all_waypoints(self):
        a,route=self.prepare();reply=a.start_route(dict(id=route['id'],engine='platform'))
        self.assertEqual(reply['execution'],'platform_trajectory')
        submitted=a.point_navigation.start.call_args.args[0]
        self.assertEqual(submitted['id'],route['id']);self.assertEqual(submitted['points'],route['points'])
        self.assertIsNone(self.planning[0]['yaw']);self.assertEqual(self.planning[0]['x'],-5.)
    def test_operator_patrol_uses_gos_with_legacy_route_settings(self):
        a,route=self.prepare();a.demo=False
        route['options']=dict(engine='native',disconnect_policy='finish',pause_at_point=True)
        a._start_gos_route=Mock(return_value={'executor':'gos'})
        reply=a.navigation_request('start',dict(id=route['id'],engine='platform'))
        self.assertEqual(reply['executor'],'gos')
        self.assertEqual(a._start_gos_route.call_args.args[0]['disconnect_policy'],'stop')
        self.assertEqual(a._start_gos_route.call_args.kwargs['route'],route)
        a.plan_goal.assert_not_called()
    def test_operator_dock_route_uses_charging_workflow(self):
        a,route=self.prepare();a.demo=False;route['points'][-1]['type']=3
        a._start_gos_route=Mock();a.start_route=Mock(return_value={'ok':True})
        a.navigation_request('start',dict(id=route['id'],engine='gos'))
        a._start_gos_route.assert_not_called()
        self.assertEqual(a.start_route.call_args.kwargs['goal'],route)
    def test_explicit_native_continuation_checks_service_before_dispatch(self):
        a,route=self.prepare();a.demo=False
        route['options']=dict(explicit_execution=True,engine='native',disconnect_policy='return',nav_mode=1)
        a.start_route=Mock(return_value={'ok':True})
        a.resident_navigation.capabilities=Mock(return_value={'available':False,'policies':['return']})
        with self.assertRaisesRegex(ValueError,'NOS 驻机导航服务尚未就绪'):
            a.navigation_request('start',dict(id=route['id']))
        a.start_route.assert_not_called()
        a.resident_navigation.capabilities.return_value={'available':True,'policies':['stop']}
        with self.assertRaisesRegex(ValueError,'尚不支持所选断联策略'):
            a.navigation_request('start',dict(id=route['id']))
        a.start_route.assert_not_called()
        a.resident_navigation.capabilities.return_value={'available':True,'policies':['return']}
        a.navigation_request('start',dict(id=route['id']))
        submitted=a.start_route.call_args.args[0]
        self.assertEqual((submitted['engine'],submitted['disconnect_policy']),('native','return'))
    def test_native_continuation_route_can_be_saved_offline(self):
        a,route=self.prepare()
        a.resident_navigation.capabilities=Mock(side_effect=AssertionError('save must not probe robot'))
        options=dict(explicit_execution=True,engine='native',disconnect_policy='finish_return',
                     collectors=[dict(kind='people_count',camera='front',interval_s=2)])
        result=a.route_save(dict(route,options=options))['route']
        self.assertEqual(result['options']['engine'],'native')
        self.assertEqual(result['options']['disconnect_policy'],'finish_return')
        self.assertEqual(result['options']['collectors'],options['collectors'])
        self.assertEqual(a.store.value['routes'],[result])
        a.resident_navigation.capabilities.assert_not_called()
    def test_native_patrol_records_people_without_gos_or_motion(self):
        a,route=self.prepare()
        route['options']=dict(collectors=[dict(kind='people_count',camera='rear',interval_s=2)])
        a.demo=False
        a.link=SimpleNamespace(ip='192.0.2.1',record=lambda *_:dict(monotonic_at=time.monotonic()))
        a.nav=dict(status='running',phase='running',execution_state='moving',index=1,route=route)
        sampler=Mock()
        sampler.sample.return_value=dict(sampled_at=time.time(),values=dict(people=2))
        with patch('m20.desktop_patrol_counts.SharedPeopleSampler',return_value=sampler):
            try:
                a.desktop_patrol_counts.tick()
                run_id=a.nav['collection_run']['id']
                deadline=time.monotonic()+2
                rows=[]
                while not rows and time.monotonic()<deadline:
                    rows=a.resident_navigation.collection_page(run_id,0)['rows']
                    time.sleep(.01)
                self.assertEqual([(r['waypoint_index'],r['values']['people']) for r in rows],[(1,2)])
                a.nav['status']='paused'
                self.assertIsNone(a.desktop_patrol_counts._context(run_id))
                a.nav['status']='completed'
                a._record_route_completion()
                self.assertEqual(a.store.value['history'][-1]['collection'][0]['state'],'ended')
            finally:
                a.desktop_patrol_counts.finish()
    def test_operator_patrol_honors_explicit_platform_engine(self):
        a,route=self.prepare();a.demo=False
        a._start_platform_route=Mock(return_value={'ok':True})
        a.navigation_request('start',dict(id=route['id'],engine='platform',explicit_execution=True))
        self.assertEqual(a._start_platform_route.call_args.kwargs['route'],route)
    def test_draft_dispatch_leaves_saved_routes_untouched(self):
        a,route=self.prepare()
        a.start_route=Mock(return_value={'ok':True})
        a.navigation_request('start',dict(route=dict(route,name='临时巡检')))
        temporary=a.start_route.call_args.kwargs['goal']
        self.assertTrue(temporary['temporary'])
        self.assertNotEqual(temporary['id'],route['id'])
        self.assertEqual(a.store.value['routes'],[route])
    def test_current_matching_preview_is_reused_but_wrong_preview_is_replanned(self):
        a,route=self.prepare();query=dict(map_id='demo',x=-5.,y=-3.,yaw=None,engine='platform');plan=a.plan_goal(query)
        a.start_route(dict(id=route['id'],engine='platform',plan_id=plan['plan_id']))
        self.assertEqual(a.plan_goal.call_count,1)
        a.goal_plans[plan['plan_id']]['result']['requested_goal']['x']=30.
        a.start_route(dict(id=route['id'],engine='platform',plan_id=plan['plan_id']))
        self.assertEqual(a.plan_goal.call_count,2)
    def test_resume_uses_current_waypoint_and_keeps_the_remaining_route(self):
        a,route=self.prepare();a.nav=dict(status='paused',engine='platform',route=route,index=1)
        a.resume_route();call=a.point_navigation.start.call_args
        self.assertEqual(call.kwargs['start_index'],1);self.assertEqual(call.args[0]['points'],route['points'])
        self.assertEqual(self.planning[0]['x'],-4.);self.assertEqual(self.planning[0]['yaw'],1.2)


if __name__=='__main__':unittest.main()

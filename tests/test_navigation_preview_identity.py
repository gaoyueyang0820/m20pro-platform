"""Preview identity and diagnostic contracts; synthetic maps, no robot I/O."""
import copy
import io
import json
import time
import unittest
from unittest.mock import Mock,patch
import test_platform as cases
from navigation_validation import NavigationEvidenceError,error_summary


class PreviewIdentityTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    def ready(self):
        cases.PlatformTests.live(self);a=self.app;a.pose.update(PosX=0,PosY=0,Yaw=0,Location=0)
        p=a.store.path.parent/'maps';p.mkdir(exist_ok=True)
        (p/'test.pgm').write_bytes(b'P5\n200 200\n255\n'+bytes([254])*40000)
        self.library=dict(active='test',maps=[dict(name='test',grids=[0])]);self.calls=[]
        def proxy(role,path,*args,**kwargs):
            self.calls.append(path)
            if path=='/maps/list':return copy.deepcopy(self.library)
            raise ValueError('Synthetic live snapshot unavailable')
        a.proxy=proxy
        return a
    def test_wrong_map_fails_before_projecting_or_planning(self):
        a=self.ready();self.library['active']='other-map'
        with self.assertRaises(NavigationEvidenceError) as caught:a.plan_goal(dict(map_id='test',x=1,y=0,engine='platform'))
        self.assertEqual(error_summary(caught.exception)['recommended_action'],'sync_active_map')
        self.assertEqual(self.calls,['/maps/list']);self.assertEqual(a.goal_plans,{})
        self.assertEqual(a.link.sent,[])
    def test_unsupported_grid_is_not_assumed_to_be_current_floor(self):
        a=self.ready();a.store.value['maps'][0]['grid_id']=7
        with self.assertRaises(NavigationEvidenceError) as caught:a.plan_goal(dict(map_id='test',x=1,y=0))
        self.assertEqual(caught.exception.navigation['reason_code'],'active_grid_unverified')
    def test_valid_map_is_bound_to_the_returned_preview(self):
        a=self.ready();p=a.plan_goal(dict(map_id='test',x=1,y=0))
        self.assertEqual(p['map_identity'],dict(map_name='test',grid_id=0))
        self.assertEqual(self.calls.count('/maps/list'),2);self.assertEqual(a.link.sent,[])
    def test_map_switch_during_preview_does_not_publish_old_plan(self):
        a=self.ready();proxy=a.proxy
        def changed(role,path,*args,**kwargs):
            if path=='/sensors/observation':self.library['active']='other-map'
            return proxy(role,path,*args,**kwargs)
        a.proxy=changed
        with self.assertRaises(NavigationEvidenceError):a.plan_goal(dict(map_id='test',x=1,y=0))
        self.assertEqual(a.goal_plans,{})
    def test_connection_change_during_identity_read_does_not_retarget(self):
        a=self.ready();proxy=a.proxy
        def changed(*args,**kwargs):a._plan_generation+=1;return proxy(*args,**kwargs)
        a.proxy=changed
        with self.assertRaisesRegex(ValueError,'连接'):a.plan_goal(dict(map_id='test',x=1,y=0))
        self.assertEqual(a.goal_plans,{})
    def test_http_error_keeps_bounded_diagnostics_without_raw_support_arrays(self):
        error=ValueError('map conflict');error.recovery=dict(classification='map_surface_disagreement',reason_code='unobserved_body_overlap',
            support_seed={'floor':[[1,2]]},overlap_diagnostics=dict(overlap_cells=3,unobserved_body_cells=3,cells=[[1,2]],surface_state='ready'))
        self.app.plan_goal=Mock(side_effect=error);responses=[]
        handler=object.__new__(cases.web.Handler);handler.path='/api/nav/plan'
        payload=json.dumps(dict(map_id='demo',x=0,y=0)).encode();handler.headers={'Content-Type':'application/json','Content-Length':str(len(payload))}
        handler.rfile=io.BytesIO(payload);handler._auth=lambda:True;handler._admitted=lambda:True
        handler._json=lambda data,status=200:responses.append((data,status))
        with patch.object(cases.web,'APP',self.app):handler.do_POST()
        data,status=responses[-1];self.assertEqual(status,400)
        self.assertEqual(data['navigation']['recommended_action'],'relocalize_and_replan')
        self.assertEqual(data['navigation']['overlap_diagnostics']['unobserved_body_cells'],3)
        self.assertNotIn('support_seed',data['navigation']);self.assertNotIn('cells',data['navigation']['overlap_diagnostics'])
    def test_explicit_verify_does_not_accept_normal_flag_without_structure_check(self):
        a=self.ready();localizer=a.localizer
        localizer.status=Mock(return_value={});localizer.good=Mock(return_value=True);localizer.remember=Mock()
        localizer.saved_slam_hint=Mock(return_value=dict(ok=True,localized=True,method='independent_geometry'))
        ordinary=localizer.recover(Mock());self.assertEqual(ordinary['method'],'already_localized')
        localizer.saved_slam_hint.assert_not_called()
        forced=localizer.recover(Mock(),verify_current=True)
        self.assertEqual(forced['method'],'independent_geometry');localizer.saved_slam_hint.assert_called_once()
        self.assertEqual(a.link.sent,[])
    def test_hub_explicit_verify_is_forwarded_without_starting_navigation(self):
        self.app.localizer.recover=Mock(return_value=dict(ok=True,localized=True))
        request=self.app.hub.post('/api/maps/localize-auto',dict(verify_current=True))
        until=time.monotonic()+2
        while time.monotonic()<until:
            job=self.app.hub.operations.get(request['job_id'])
            if job['status'] in ['completed','failed']:break
            time.sleep(.01)
        self.assertEqual(job['status'],'completed',job)
        self.assertTrue(self.app.localizer.recover.call_args.kwargs['verify_current'])
        self.assertEqual(self.app.nav['status'],'idle');self.assertEqual(self.app.velocity,[0,0,0])

    def test_temporary_vote_shortfall_gets_only_one_read_only_recheck(self):
        from navigation_planner import GridPlanner
        from autonomy_core import disk
        a=self.ready();counter=[0]
        def surface(*args,**kwargs):
            counter[0]+=1;stamp=100+counter[0]*.2
            scan=dict(protocol=1,frame_id='base_link',pose_frame_id='map',pose=[0.,0.,0.],pose_stamp=stamp,
                      stamp=stamp,age=.05,resolution=.2,geometry_version=2,measured_floor=[list(p) for p in disk(2.)],
                      floor=[list(p) for p in disk(2.)],obstacles=[],obstacle_bounds=[],_valid_until=time.monotonic()+.6)
            return scan,dict(normal=True,confidence=.99,age=.05,pose=[0.,0.,0.],epoch='same-context')
        a.navigation_surface=surface
        temporary=ValueError('need independent frames');temporary.recovery=dict(classification='map_surface_disagreement',reason_code='insufficient_ground_votes')
        with patch.object(GridPlanner,'plan',side_effect=ValueError('起点余量不足')),patch('start_recovery_runtime.plan_with_recovery',side_effect=temporary) as recovery:
            with self.assertRaises(ValueError) as caught:a.plan_goal(dict(map_id='test',x=1,y=0,engine='platform'))
        self.assertIs(caught.exception,temporary)
        self.assertEqual(recovery.call_count,2,'the second shortfall must leave, not loop')
        self.assertLessEqual(counter[0],12);self.assertEqual(a.link.sent,[]);self.assertFalse(any(a.velocity))


if __name__=='__main__':unittest.main()

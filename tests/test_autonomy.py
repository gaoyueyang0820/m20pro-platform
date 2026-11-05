"""Autonomy algorithms + platform arbitration, entirely offline."""
import math
import json
from pathlib import Path
import sys
import time
import threading
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from m20.autonomy_core import Explorer,PriorMap,PersonLock,clearance,disk,body_offsets,RESOLUTION
from m20.autonomy_sensing import supported_floor,xyz
import test_platform as cases


def person(box=(.4,.2,.6,.7),confidence=.9):
    return dict(box=list(box),confidence=confidence)


def scan(pose=(0,0,0),radius=4):
    return dict(floor=[list(p) for p in disk(radius)],obstacles=[],pose=pose)


class CoreTests(unittest.TestCase):
    def prior(self,unknown=False,wall=False):
        import base64
        image=np.full((80,80),254,np.uint8)
        if unknown:image[:,44:]=205
        if wall:image[:,34]=0
        return PriorMap(dict(pgm=base64.b64encode(b'P5\n80 80\n255\n'+image.tobytes()).decode(),resolution=.2,origin=[-8,-8,0]),dict(x=0,y=0,yaw=0))
    def test_old_map_unknown_regions_have_priority_and_known_regions_are_revisited(self):
        e=Explorer(3);e.prior=self.prior(unknown=True,wall=True);e.ingest(scan(radius=4));e.plan()
        self.assertEqual(e.target_kind,'expand');self.assertGreater(e.goal[0],0)
        self.assertTrue(clearance(e.current_scan,*e.command())[0])
        e=Explorer(3);e.prior=self.prior();e.ingest(scan(radius=4));e.plan()
        self.assertEqual(e.target_kind,'refine');self.assertGreater(e.command()[0],0)
    def test_old_map_free_space_does_not_replace_missing_live_floor(self):
        e=Explorer(3);e.prior=self.prior();e.ingest(dict(floor=[],obstacles=[],pose=[0,0,0]))
        self.assertEqual(e.plan(),'blocked')
    def test_fully_observed_room_still_selects_unvisited_vantage(self):
        e=Explorer(2);e.ingest(scan(radius=6));self.assertEqual(e.plan(),'planned')
        self.assertGreater(e.command()[0],0);self.assertEqual(e.command()[1],0)
    def test_small_slam_offset_does_not_erase_forward_floor(self):
        e=Explorer(2);s=scan((-.004,-.003,-.001),2)
        # Match a sensor's unobserved standing disk plus measured forward floor.
        s['floor']=[p for p in s['floor'] if math.hypot(*p)*.2>.55]
        e.ingest(s);self.assertEqual(e.plan(),'planned')
        self.assertGreater(e.command()[0],0)
        self.assertTrue(clearance(s,*e.command())[0])
    def test_only_measured_floor_clears_old_obstacle(self):
        e=Explorer(2);s=scan();s['obstacles']=[[-2,-1]];s['floor'].remove([-2,-1]);e.ingest(s)
        self.assertEqual(e.plan(),'blocked')
        s['obstacles']=[];e.ingest(s);self.assertIn((-2,-1),e.obstacles)
        s['floor'].append([-2,-1]);e.ingest(s)
        self.assertNotIn((-2,-1),e.obstacles);self.assertEqual(e.plan(),'planned')
    def test_plan_only_crosses_supported_floor_and_avoids_wall(self):
        e=Explorer();s=scan();s['obstacles']=[[5,y] for y in range(-12,13)]
        e.ingest(s);self.assertEqual(e.plan(),'planned')
        self.assertTrue(e.path)
        for p in e.path:
            self.assertNotIn(p,e.obstacles)
            self.assertTrue(all((p[0]+x,p[1]+y) in e.free for x,y in body_offsets(0,True)))
    def test_narrow_corridor_allows_straight_passage_but_not_turning(self):
        s=scan();s['obstacles']=[[x,y] for x in range(-12,13) for y in [-2,2]]
        self.assertTrue(clearance(s,.35,0)[0])
        self.assertFalse(clearance(s,0,.7)[0])
        e=Explorer(2);e.ingest(s);self.assertEqual(e.plan(),'planned');self.assertGreater(e.command()[0],0)
    def test_front_goal_selected_when_rear_obstacle_prevents_turning(self):
        e=Explorer(2);s=scan();s['obstacles']=[[-3,0]];e.ingest(s)
        self.assertEqual(e.plan(),'planned');self.assertGreater(e.command()[0],0);self.assertEqual(e.command()[1],0)
    def test_old_occupancy_does_not_override_fresh_open_corridor(self):
        e=Explorer(2);e.prior=self.prior(wall=True);s=scan();s['pose']=[0,0,math.pi]
        e.ingest(s);self.assertEqual(e.plan(),'planned')
        # The old obstacle can only be traversed while CURRENT complete scans
        # support the path. A measured obstacle still stops the actual command.
        s['obstacles']=[[3,0]];self.assertFalse(clearance(s,.35,0)[0])
    def test_obstacle_inside_body_still_blocks_and_speed_extends_lookahead(self):
        s=scan();s['obstacles']=[[1,1]];self.assertFalse(clearance(s,.35,0)[0])
        s['obstacles']=[[6,0]];self.assertTrue(clearance(s,.35,0)[0])
        s['_speed_mps']=.9;self.assertFalse(clearance(s,.35,0)[0])
    def test_closed_room_exploration_discovers_new_area_without_waypoints(self):
        x=y=yaw=0.;e=Explorer(radius=4);first=None
        for step in range(1800):
            c,s=math.cos(yaw),math.sin(yaw);floor=[];obstacles=[]
            for ix,iy in disk(2.8):
                wx=x+c*ix*.2-s*iy*.2;wy=y+s*ix*.2+c*iy*.2
                if abs(wx)<4 and abs(wy)<3:floor.append([ix,iy])
                else:obstacles.append([ix,iy])
            data=dict(floor=floor,obstacles=obstacles,pose=[x,y,yaw]);e.ingest(data)
            if first is None:first=len(e.free)
            if step%10==0 or not e.path:
                if e.plan()!='planned':break
            f,turn=e.command();clear,_=clearance(data,f,turn)
            if not clear:break
            x+=math.cos(yaw)*f*.2;y+=math.sin(yaw)*f*.2;yaw+=turn*.2
        self.assertGreater(e.distance,1.)
        self.assertGreater(len(e.free),first*1.3)
    def test_unknown_floor_and_obstacle_each_stop_forward(self):
        s=scan();self.assertTrue(clearance(s,.1,0)[0])
        s['obstacles']=[[4,0]];self.assertFalse(clearance(s,.1,0)[0])
        s['obstacles']=[];s['floor']=[p for p in s['floor'] if p[0]<4]
        self.assertFalse(clearance(s,.1,0)[0])

    def test_ground_support_is_separate_from_body_clearance_margin(self):
        s=scan();s['floor']=[p for p in s['floor'] if not(abs(p[0])<=1 and abs(p[1])==3)]
        self.assertTrue(clearance(s,0,.1)[0]);self.assertTrue(clearance(s,.1,0)[0])
        s['obstacles']=[[0,3]];self.assertFalse(clearance(s,0,.1)[0])
        s['obstacles']=[];s['floor']=[p for p in s['floor'] if p!=[3,1]]
        self.assertTrue(clearance(s,.1,0)[0])  # One enclosed missing return.
        s['floor']=[p for p in s['floor'] if p!=[4,1]]
        self.assertFalse(clearance(s,.1,0)[0])  # Connected gap stays unknown.
    def test_scan_gap_interpolation_never_clears_measured_obstacle(self):
        s=scan();s['floor'].remove([3,1]);s['obstacles']=[[3,1]]
        self.assertFalse(clearance(s,.35,0)[0])
    def test_exploration_preserves_reachable_goal_during_turn(self):
        e=Explorer(2);e.ingest(scan(radius=2));e.plan();goal=e.goal
        e.ingest(scan((0,0,.4),radius=2));e.plan();self.assertEqual(e.goal,goal)
    def test_heading_prediction_brakes_before_forward_or_reversing_turn(self):
        e=Explorer(2);e.ingest(scan(radius=2));e.plan()
        self.assertEqual(e.command(yaw_rate=1,observation_age=.4),(0.,0.))
    def test_real_turn_over_sensing_gap_is_not_a_pose_reset(self):
        e=Explorer(2);e.ingest(dict(scan(),stamp=100));e.ingest(dict(scan((0,0,.7)),stamp=100.7))
        with self.assertRaises(ValueError):e.ingest(dict(scan((0,0,2.0)),stamp=100.8))
    def test_pose_reset_cannot_reuse_old_grid(self):
        e=Explorer();e.ingest(scan())
        with self.assertRaisesRegex(ValueError,'跳变'):e.ingest(scan((10,0,0)))
    def test_follow_does_not_switch_people_or_reacquire_after_loss(self):
        lock=PersonLock(person()['box'])
        f,y=lock.update([person((.48,.25,.64,.65))]);self.assertGreater(f,0);self.assertLess(y,0)
        with self.assertRaises(ValueError):lock.update([person((.05,.2,.20,.7))])
        with self.assertRaises(ValueError):lock.update([person()])
    def test_crossing_truncation_and_large_scale_jump_stop(self):
        for detections in [[person(),person((.45,.2,.65,.7))],[person((.4,0,.6,.7))],[person((.4,.2,.6,.9))]]:
            with self.assertRaises(ValueError):PersonLock(person()['box']).update(detections)
    def test_follow_waits_when_person_too_close_and_never_reverses(self):
        f,y=PersonLock(person()['box']).update([person((.4,.17,.6,.75))])
        self.assertEqual(f,0);self.assertEqual(y,0)


class SensingTests(unittest.TestCase):
    def ground(self):
        x,y=np.meshgrid(np.arange(-4,4,.1),np.arange(-4,4,.1))
        return np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55)])
    def test_thin_obstacle_not_dropped_by_preview_sampling(self):
        p=self.ground();p=np.vstack([p,[1,0,-.2]])
        s=supported_floor(p)
        self.assertIn([5,0],s['obstacles']);self.assertNotIn([5,0],s['floor'])
        self.assertEqual(json.loads(json.dumps(s)),s)
    def test_hole_is_not_inferred_free_from_distant_ray(self):
        p=self.ground();p=p[~((p[:,0]>.6)&(p[:,0]<1.4)&(abs(p[:,1])<.5))]
        s=supported_floor(p)
        self.assertNotIn([5,0],s['floor']);self.assertFalse(clearance(s,.1,0)[0])
    def test_nonlevel_or_unobserved_ground_rejected(self):
        for p in [np.zeros((100,3)),self.ground()+[0,0,2]]:
            with self.assertRaises(ValueError):supported_floor(p)
    def test_xyz_uses_every_return_including_last_point(self):
        from types import SimpleNamespace as NS
        data=np.zeros((10000,3),dtype='<f4');data[-1]=[1,2,3]
        msg=NS(width=10000,height=1,row_step=120000,point_step=12,data=data.tobytes(),is_bigendian=False,
               fields=[NS(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate('xyz')])
        out=xyz(msg);self.assertEqual(len(out),10000);np.testing.assert_array_equal(out[-1],[1,2,3])


class PlatformAutonomyTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    def test_general_ranges_normalize_legacy_requests_and_accept_configured_radius(self):
        a=self.app.autonomy
        for requested,expected in [('room','radius'),('floor','map'),('radius','radius'),('map','map')]:
            with self.subTest(scope=requested),patch('m20.autonomy_runtime.threading.Thread'):
                result=a.start(dict(mode='explore',scope=requested,extend=True,radius=40,minutes=45))
                self.assertEqual(a.snapshot()['scope'],expected);self.assertEqual(a.explorer.radius,40)
                self.assertAlmostEqual(a.deadline-a.started,45*60);self.assertTrue(result['ok'])
                a.pause()
        with patch('m20.autonomy_runtime.threading.Thread'):
            a.start(dict(mode='explore',scope='map'))
        self.assertEqual(a.scope,'radius');a.pause()

    def test_map_scope_derives_search_extent_from_existing_map_not_scene_name(self):
        import base64
        a=self.app.autonomy
        grid=dict(pgm=base64.b64encode(b'P5\n100 80\n255\n'+bytes([254])*8000).decode(),resolution=.2)
        with patch('m20.autonomy_runtime.threading.Thread'):
            result=a.start(dict(mode='explore',scope='map',extend=True,base='site-map',radius=6))
        with patch.object(a,'_check_observation'),patch.object(self.app.hub,'remote_maps',return_value=grid),\
             patch.object(self.app.hub,'post',return_value=dict(job_id='prepared')),patch.object(a,'_run') as run:
            a._prepare_and_run(result['run_id'],dict(mode='explore',scope='map',extend=True,base='site-map'),300)
        self.assertAlmostEqual(a.explorer.radius,math.hypot(100,80)*.2+6)
        self.assertEqual(run.call_args.args,(result['run_id'],'prepared',300));a.pause()

    def test_automatic_relocation_waits_for_valid_status_and_never_drives(self):
        cases.PlatformTests.live(self)
        with patch.object(self.app.hub,'remote_maps',return_value=dict(accepted=True,localized=False)),patch.object(self.app,'proxy',return_value=dict(localization_quality=dict(status=0,normal_status=0,age=.1),matching_quality=dict(error=.04,inlier_ratio=.99,age=.1))):
            result=self.app.hub.post('/api/maps/localize-auto',{});deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                job=self.app.hub.operations.get(result['job_id'])
                if job['status'] in ['completed','failed']:break
                time.sleep(.01)
        self.assertEqual(job['status'],'completed',job);self.assertTrue(job['result']['localized'])
        self.assertEqual(self.app.velocity,[0.,0.,0.]);self.assertEqual(self.app.link.sent,[])
    def test_automatic_relocation_refused_during_mapping(self):
        self.app.mapping=True
        with self.assertRaisesRegex(ValueError,'保存'):self.app.hub.post('/api/maps/localize-auto',{})
    def reserve(self):
        a=self.app.autonomy;a.active=True;a.run_id='old';a.state='following';a.lease=time.monotonic()
        self.app.velocity=[.1,0,0];return a
    def test_manual_takeover_invalidates_pending_worker_output(self):
        a=self.reserve();self.app.set_velocity(dict(yaw=.1))
        self.assertFalse(a.active);self.assertFalse(a._drive('old',.1,0));self.assertEqual(self.app.velocity,[0.,0.,.1])
    def test_browser_inactivity_does_not_pause_owned_background_task(self):
        a=self.reserve();a.context=self.app.hub.context();a.lease-=30;self.app.tick(.01)
        self.assertTrue(a.active);a._assert_run('old')
    def test_expired_sensor_command_stops_without_browser_or_worker_callbacks(self):
        cases.PlatformTests.live(self);a=self.reserve();a.context=self.app.hub.context();a.lease-=30
        self.app.velocity_deadline=time.monotonic()-.01;self.app.tick(.01)
        self.assertTrue(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.]);self.assertEqual(self.app.link.v,(0,0,0))
    def test_old_lease_cannot_extend_new_session(self):
        a=self.reserve();before=a.lease;a.post('/api/autonomy/lease',dict(run_id='different'))
        self.assertEqual(a.lease,before)
    def test_nav_joint_and_map_mutations_exclude_autonomy(self):
        self.reserve()
        with self.assertRaises(ValueError):self.app.start_goal(dict(map_id='demo',x=0,y=0))
        with self.assertRaises(ValueError):self.app.joint_control('arm',dict(rig_confirmed=True))
        with self.assertRaises(ValueError):self.app.hub.idle()
    def test_slow_scan_response_never_yields_motion(self):
        a=self.app.autonomy;self.app.demo=False
        self.app.proxy=lambda *args,**kwargs:dict(protocol=1,age=.9)
        with self.assertRaisesRegex(ValueError,'延迟'):a.scan_now(True)
    def test_scan_expiring_during_planning_cannot_refresh_motion(self):
        cases.PlatformTests.live(self);a=self.reserve()
        self.assertIsNone(a._drive('old',.12,0,time.monotonic()-.01))
        self.assertTrue(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.])
        self.assertEqual(a.timing[-1]['reason'],'decision_expired')
    def test_autonomous_command_retains_observation_expiry_in_transport(self):
        cases.PlatformTests.live(self);a=self.reserve();deadline=time.monotonic()+.1
        self.assertTrue(a._drive('old',.12,0,deadline))
        self.assertEqual(self.app.link.valid_until,deadline)
    def test_session_deadline_stops_background_task(self):
        a=self.reserve();a.deadline=time.monotonic()-.01
        self.app.tick(.01)
        self.assertFalse(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_follow_rejects_missing_or_stale_vision_before_any_movement(self):
        with self.assertRaises(ValueError):self.app.autonomy.start(dict(mode='follow',person=0))
        self.assertFalse(self.app.autonomy.active);self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_demo_follow_runs_and_pauses_without_robot_link(self):
        a=self.app.autonomy;a.vision.start()
        result=a.start(dict(mode='follow',person=0));deadline=time.monotonic()+2
        while a.state=='preparing' and time.monotonic()<deadline:time.sleep(.02)
        self.assertEqual(a.state,'following',a.message);self.assertIsNone(self.app.link)
        with self.app.lock:a.pause()
        self.assertFalse(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_demo_map_completion_waits_for_save_result(self):
        a=self.app.autonomy;self.app.mapping=True
        a.finish_save();deadline=time.monotonic()+4
        while a.state=='saving' and time.monotonic()<deadline:time.sleep(.05)
        self.assertEqual(a.state,'completed',a.message)
        self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_demo_explore_starts_collection_drives_and_saves_at_time_limit(self):
        a=self.app.autonomy
        a.start(dict(mode='explore',minutes=.1,radius=3))
        deadline=time.monotonic()+10;walked=False
        while time.monotonic()<deadline:
            with self.app.lock:
                if a.state in ['completed','paused','failed']:break
            a.post('/api/autonomy/lease',dict(run_id=a.run_id))
            self.app.tick(.05);walked=walked or any(self.app.velocity)
            time.sleep(.05)
        self.assertTrue(walked,a.message);self.assertEqual(a.state,'completed',a.message)
        self.assertFalse(self.app.mapping);self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_save_dispatch_failure_is_visible_after_motion_stops(self):
        from unittest.mock import patch
        a=self.reserve();self.app.mapping=True
        with patch.object(self.app.hub,'post',side_effect=ValueError('NOS unavailable')):
            with self.assertRaises(ValueError):a.finish_save()
        self.assertFalse(a.active);self.assertEqual(a.state,'failed')
        self.assertIn('NOS unavailable',a.message);self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_existing_mapping_requires_explicit_continue(self):
        self.app.mapping=True
        with self.assertRaisesRegex(ValueError,'继续当前采集'):self.app.autonomy.start(dict(mode='explore'))
    def test_inference_completion_does_not_refresh_source_timestamp(self):
        vision=self.app.autonomy.vision;self.app.demo=False
        vision.row.update(ready=True,enabled=True);vision.frame_at=time.time()-1
        self.assertFalse(vision.snapshot()['fresh'])
        vision.selection_frames[1]=(time.time()-1,[person()])
        with self.assertRaises(ValueError):vision.selected(1)
    def test_robot_service_without_new_sensor_contract_rejected_before_mapping(self):
        cases.PlatformTests.live(self)
        a=self.app.autonomy;a.start(dict(mode='explore'));deadline=time.monotonic()+2
        while a.active and time.monotonic()<deadline:time.sleep(.01)
        self.assertIn('自主感知接口',a.message);self.assertFalse(self.app.mapping);self.assertFalse(a.active)

    def test_missing_ground_does_not_start_native_mapping(self):
        from unittest.mock import patch
        cases.PlatformTests.live(self)
        a=self.app.autonomy
        self.app.proxy=lambda *args,**kw:dict(autonomy_protocol=1)
        with patch.object(a,'scan_now',return_value=dict(floor=[],obstacles=[])),patch.object(self.app.hub,'post') as post:
            a.start(dict(mode='explore',extend=True,base='existing'));deadline=time.monotonic()+2
            while a.active and time.monotonic()<deadline:time.sleep(.01)
        self.assertIn('已收到雷达',a.message);post.assert_not_called();self.assertFalse(self.app.mapping);self.assertFalse(a.active)

    def test_sensor_warmup_retries_without_map_or_motion_commands(self):
        from unittest.mock import patch
        cases.PlatformTests.live(self);a=self.app.autonomy
        with patch.object(a,'scan_now',side_effect=[ValueError('正在启动'),dict(scan(),stamp=1),dict(scan(),stamp=1),dict(scan(),stamp=2)]) as reader,patch('m20.autonomy_runtime.time.sleep'):
            a._check_observation()
        self.assertEqual(reader.call_count,4)
        self.assertFalse(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.])
    def test_start_can_use_straight_passage_without_in_place_turn_space(self):
        cases.PlatformTests.live(self);a=self.app.autonomy
        s=scan();s['obstacles']=[[-3,0]]
        self.assertTrue(clearance(s,.35,0)[0]);self.assertFalse(clearance(s,0,.7)[0])
        with patch.object(a,'scan_now',side_effect=[dict(s,stamp=1),dict(s,stamp=2)]),patch('m20.autonomy_runtime.time.sleep'):
            a._check_observation()
        self.assertEqual(self.app.link.sent,[]);self.assertFalse(a.active)

    def test_yaw_oscillation_does_not_keep_stationary_exploration_alive(self):
        from m20.autonomy_runtime import MotionProgress
        progress=MotionProgress()
        for i in range(70):progress.update((0,0,.3 if i%2 else -.3),(2,0),10,i,moving=False)
        self.assertTrue(progress.stalled(70))
        self.assertLessEqual(progress.last_at,10)

    def test_changing_goals_and_noisy_coverage_cannot_keep_turning_alive(self):
        from m20.autonomy_runtime import MotionProgress
        progress=MotionProgress()
        for i in range(70):progress.update((.03*(i%2),0,.4*(i%3)),(2,0) if i%2 else (-2,0),10+i*.6,i,moving=False)
        self.assertTrue(progress.stalled(70));self.assertLess(progress.last_at,20)

    def test_real_translation_and_bounded_alignment_count_as_progress(self):
        from m20.autonomy_runtime import MotionProgress
        progress=MotionProgress();progress.update((0,0,1.5),(3,0),10,0)
        progress.update((0,0,.8),(3,0),10,5);self.assertFalse(progress.stalled(29))
        progress.update((.3,0,.1),(3,0),10,30,moving=True)
        self.assertFalse(progress.stalled(50));self.assertEqual(progress.moved_at,30)

    def test_static_feedback_does_not_turn_slam_drift_into_movement(self):
        from m20.autonomy_runtime import MotionProgress
        progress=MotionProgress()
        for i in range(60):progress.update((i*.02,0,0),None,10,i,moving=False)
        self.assertTrue(progress.stalled(60));self.assertEqual(progress.moved_at,0)

    def await_job(self,result,limit=2):
        deadline=time.monotonic()+limit
        while time.monotonic()<deadline:
            job=self.app.hub.operations.get(result['job_id'])
            if job['status'] in ['completed','failed']:return job
            time.sleep(.01)
        self.fail('job did not finish: '+str(job))

    def test_cancel_before_mapping_dispatch_never_starts_collection(self):
        entered=threading.Event();release=threading.Event();calls=[]
        def remote(path,req=None,**kw):
            calls.append((path,req))
            if path=='/maps/build/status':entered.set();release.wait(2);return dict(request_protocol=1,phase='idle',in_progress=False)
            raise AssertionError('must not dispatch mapping after stop')
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='cancelled'))
            self.assertTrue(entered.wait(1))
            with self.app.lock:self.app.stop()
            release.set();job=self.await_job(result)
        self.assertEqual(job['status'],'completed');self.assertTrue(job['result']['cancelled'])
        self.assertFalse(any(path=='/maps/build' for path,req in calls));self.assertFalse(self.app.mapping)

    def test_cancel_during_mapping_dispatch_stops_and_saves_once(self):
        entered=threading.Event();release=threading.Event();calls=[];current={'id':None,'stopped':False}
        def remote(path,req=None,**kw):
            if path=='/maps/build/status':
                return dict(request_protocol=1,phase='saved' if current['stopped'] else 'collecting' if current['id'] else 'idle',in_progress=bool(current['id'] and not current['stopped']),mapping=dict(request_id=current['id']),result_maps=['saved'] if current['stopped'] else [])
            if path=='/maps/build':
                calls.append(dict(req))
                if req['action']=='start':
                    current['id']=req['request_id'];entered.set();release.wait(2);return dict(ok=True,request_id=current['id'])
                self.assertEqual(req['expected_start_request_id'],current['id']);current['stopped']=True;return dict(ok=True)
            raise AssertionError(path)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='cancelled'))
            self.assertTrue(entered.wait(1))
            with self.app.lock:self.app.stop();self.app.stop()
            release.set();job=self.await_job(result)
        self.assertEqual(job['status'],'completed',job);self.assertEqual(job['result']['cleanup'],'saved')
        self.assertEqual([c['action'] for c in calls],['start','stop']);self.assertFalse(self.app.mapping)
        self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_prepare_expiry_cannot_start_slam_after_watchdog(self):
        a=self.app.autonomy;entered=threading.Event();release=threading.Event()
        def warm(check=None):entered.set();release.wait(2)
        with patch.object(a,'_check_observation',side_effect=warm),patch.object(self.app.hub,'post') as post:
            result=a.start(dict(mode='explore'));self.assertEqual(result['state'],'preparing')
            self.assertTrue(entered.wait(1));a.deadline=time.monotonic()-.01
            with self.app.lock:a.watchdog()
            release.set();time.sleep(.05)
            post.assert_not_called()
        self.assertFalse(a.active);self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_pause_is_not_blocked_behind_sensor_warmup(self):
        a=self.app.autonomy;entered=threading.Event();release=threading.Event();stopped=threading.Event()
        def warm(check=None):entered.set();release.wait(2)
        with patch.object(a,'_check_observation',side_effect=warm),patch.object(self.app.hub,'post') as post:
            a.start(dict(mode='explore'));self.assertTrue(entered.wait(1))
            def stop():
                with self.app.lock:self.app.stop()
                stopped.set()
            t=threading.Thread(target=stop);t.start()
            try:self.assertTrue(stopped.wait(.3),'stop waited for slow preparation')
            finally:release.set();t.join(1)
            time.sleep(.05);post.assert_not_called()
        self.assertFalse(a.active)

    def test_connection_switch_cannot_retarget_old_preparation(self):
        entered=threading.Event();release=threading.Event();calls=[]
        def remote(path,req=None,**kw):
            calls.append((path,req))
            if path=='/maps/build/status':return dict(request_protocol=1,phase='idle',in_progress=False)
            entered.set();release.wait(2);return dict(ok=True)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='old-robot'))
            self.assertTrue(entered.wait(1))
            with self.app.lock:
                self.app.autonomy.reset_context();self.app.demo=False;self.app.link=cases.FakeLink()
            release.set();job=self.await_job(result)
        self.assertEqual(job['status'],'failed');self.assertIn('待核对',job['error'])
        self.assertEqual(sum(path=='/maps/build' for path,req in calls),1)
        self.assertFalse(self.app.mapping);self.assertIsNotNone(self.app.hub.mapping_uncertain)

    def test_old_save_completion_cannot_overwrite_new_session(self):
        a=self.app.autonomy;self.app.mapping=True;entered=threading.Event();release=threading.Event()
        result=dict(ok=True,job_id='save-old')
        def remote(*args,**kw):entered.set();release.wait(2);return dict(request_protocol=1,phase='saved',in_progress=False,result_maps=['old-result'])
        with patch.object(self.app.hub,'post',return_value=result),patch.object(self.app.hub.operations,'get',return_value=dict(status='completed',result={})),patch.object(self.app.hub,'remote_maps',side_effect=remote):
            a.finish_save();self.assertTrue(entered.wait(1))
            with self.app.lock:
                a.reset_context();self.app.mapping=False;a.state='preparing';a.message='new session';a.result_maps=['new-result']
            release.set();time.sleep(.05)
        self.assertEqual(a.state,'preparing');self.assertEqual(a.message,'new session');self.assertEqual(a.result_maps,['new-result'])

    def test_repeat_save_reuses_the_pending_operation(self):
        a=self.app.autonomy;self.app.mapping=True;result=dict(ok=True,job_id='same-save')
        with patch.object(self.app.hub,'post',return_value=result) as post,patch.object(self.app.hub.operations,'get',return_value=dict(status='running')):
            first=a.finish_save();second=a.finish_save();a.reset_context()
        self.assertEqual(first,second);post.assert_called_once()

    def test_preparation_waits_for_collection_not_only_launch_ack(self):
        collecting=threading.Event();current={'id':None}
        def remote(path,req=None,**kw):
            if path=='/maps/build':current['id']=req['request_id'];return dict(ok=True)
            return dict(request_protocol=1,mapping=dict(request_id=current['id']),phase='collecting' if collecting.is_set() else 'starting' if current['id'] else 'idle',in_progress=bool(current['id']))
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='ack-is-not-ready'))
            deadline=time.monotonic()+1
            while current['id'] is None and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(self.app.hub.operations.get(result['job_id'])['status'],'running')
            self.assertEqual(self.app.hub.busy,'准备建图')
            collecting.set();job=self.await_job(result)
        self.assertEqual(job['status'],'completed');self.assertEqual(job['result']['phase'],'collecting')

    def test_prepare_deadline_after_dispatch_stops_and_saves(self):
        entered=threading.Event();release=threading.Event();current={'id':None,'stopped':False};calls=[]
        def remote(path,req=None,**kw):
            if path=='/maps/build':
                calls.append(req['action'])
                if req['action']=='start':current['id']=req['request_id'];entered.set();release.wait(2)
                else:self.assertEqual(req['expected_start_request_id'],current['id']);current['stopped']=True
                return dict(ok=True)
            return dict(request_protocol=1,mapping=dict(request_id=current['id']),phase='saved' if current['stopped'] else 'collecting' if current['id'] else 'idle',in_progress=bool(current['id'] and not current['stopped']),result_maps=['owned-save'] if current['stopped'] else [])
        a=self.app.autonomy
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            a.start(dict(mode='explore'));self.assertTrue(entered.wait(1));a.deadline=time.monotonic()-.01
            with self.app.lock:a.watchdog()
            self.assertFalse(a.active);self.assertEqual(a.state,'saving')
            release.set();deadline=time.monotonic()+2
            while a.state=='saving' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(calls,['start','stop']);self.assertEqual(a.state,'completed',a.message)
        self.assertEqual(a.result_maps,['owned-save']);self.assertFalse(self.app.mapping)

    def test_wrong_mapping_request_is_never_stopped_as_cleanup(self):
        entered=threading.Event();release=threading.Event();calls=[];started=False
        def remote(path,req=None,**kw):
            nonlocal started
            if path=='/maps/build':
                calls.append(req['action']);started=True;entered.set();release.wait(2);return dict(ok=True)
            return dict(request_protocol=1,mapping=dict(request_id='f'*32),phase='collecting' if started else 'idle',in_progress=started)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='owned'))
            self.assertTrue(entered.wait(1))
            with self.app.lock:self.app.stop()
            release.set();job=self.await_job(result)
        self.assertEqual(calls,['start']);self.assertEqual(job['status'],'failed');self.assertIn('不属于',job['error'])

    def test_late_build_status_cannot_overwrite_changed_connection(self):
        def remote(*args,**kwargs):
            self.app.demo=False;self.app.link=cases.FakeLink()
            return dict(request_protocol=1,phase='collecting',in_progress=True)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            with self.assertRaisesRegex(ValueError,'连接已变化'):self.app.hub.get('/api/maps/build/status',{})
        self.assertFalse(self.app.mapping)

    def test_legacy_mapping_service_is_rejected_before_start_dispatch(self):
        calls=[]
        def remote(path,req=None,**kw):
            calls.append((path,req));return dict(phase='idle',in_progress=False)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            result=self.app.hub.post('/api/maps/start-here',dict(name='old-service'))
            job=self.await_job(result)
        self.assertEqual(job['status'],'failed');self.assertIn('收尾功能',job['error'])
        self.assertFalse(any(path=='/maps/build' for path,req in calls));self.assertFalse(self.app.mapping)

    def test_mapping_preparation_wakes_standing_robot_and_preserves_sleep_settings(self):
        cases.PlatformTests.live(self);link=self.app.link;basic=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=1,Gait=12290)
        self.app.basic=dict(basic);link.put(1002,6,dict(BasicStatus=basic));link.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
        link.put(1101,7,dict(Auto=False,Time=12));sent=link.command;current={'id':None}
        def command(t,c,it,label):
            result=sent(t,c,it,label)
            if (t,c)==(1101,6):basic['Sleep']=0;link.put(1002,6,dict(BasicStatus=dict(basic)))
            return result
        link.command=command
        def remote(path,req=None,**kw):
            if path=='/maps/build':current['id']=req['request_id'];return dict(ok=True)
            return dict(request_protocol=1,mapping=dict(request_id=current['id']),phase='collecting' if current['id'] else 'idle',in_progress=bool(current['id']))
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            job=self.await_job(self.app.hub.post('/api/maps/start-here',dict(name='wake',autonomy_gait=12290)))
        self.assertEqual(job['status'],'completed',job)
        wake=next(c for c in link.sent if (c.get('type'),c.get('command'))==(1101,6))
        self.assertEqual(wake['items'],dict(Sleep=False,Auto=False,Time=12))
        self.assertFalse(any(c.get('motion')=='stand' for c in link.sent))

    def test_mapping_does_not_start_without_wake_readback(self):
        from types import SimpleNamespace
        cases.PlatformTests.live(self);link=self.app.link;basic=dict(HES=0,MotionState=17,ControlUsageMode=0,Sleep=1,Gait=12290)
        self.app.basic=dict(basic);link.put(1002,6,dict(BasicStatus=basic));link.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
        calls=[];clock={'now':0.}
        fake_time=SimpleNamespace(monotonic=lambda:clock['now'],time=time.time,sleep=lambda s:clock.update(now=clock['now']+1))
        def remote(path,req=None,**kw):calls.append(path);return dict(request_protocol=1,phase='idle',in_progress=False)
        with patch.object(self.app.hub,'remote_maps',side_effect=remote),patch('m20.platform_hub.time',fake_time):
            job=self.await_job(self.app.hub.post('/api/maps/start-here',dict(name='not-awake')))
        self.assertEqual(job['status'],'failed');self.assertIn('休眠',job['error'])
        self.assertNotIn('/maps/build',calls)

    def test_continue_current_capture_checks_identity_without_starting_it_again(self):
        a=self.app.autonomy;self.app.mapping=True;calls=[];identity='a'*32
        def remote(path,req=None,**kw):
            calls.append((path,req))
            if path!='/maps/build/status':raise AssertionError(path)
            return dict(request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id=identity))
        with patch.object(self.app.hub,'remote_maps',side_effect=remote),patch.object(a,'_run') as worker:
            a.start(dict(mode='explore',use_current=True));deadline=time.monotonic()+1
            while not worker.called and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(worker.called,a.message)
            with self.app.lock:a.pause()
        self.assertTrue(self.app.mapping);self.assertTrue(all(path=='/maps/build/status' for path,req in calls))

    def test_continue_legacy_capture_without_identity_does_not_move_or_stop_it(self):
        a=self.app.autonomy;self.app.mapping=True;calls=[]
        def remote(path,req=None,**kw):
            calls.append(path);return dict(request_protocol=1,phase='collecting',in_progress=True,mapping={})
        with patch.object(self.app.hub,'remote_maps',side_effect=remote),patch.object(a,'_drive') as drive:
            a.start(dict(mode='explore',use_current=True));deadline=time.monotonic()+1
            while a.active and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(a.active);self.assertIn('归属',a.message);drive.assert_not_called()
        self.assertTrue(self.app.mapping);self.assertNotIn('/maps/build',calls)

    def test_cancel_continue_preparation_keeps_existing_capture(self):
        a=self.app.autonomy;self.app.mapping=True;entered=threading.Event();release=threading.Event();calls=[]
        def remote(path,req=None,**kw):
            calls.append((path,req));entered.set();release.wait(2)
            return dict(request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id='b'*32))
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            a.start(dict(mode='explore',use_current=True));self.assertTrue(entered.wait(1))
            pending=self.app.hub.preparation_job()
            with self.app.lock:a.pause()
            release.set();job=self.await_job(pending)
        self.assertTrue(job['result']['cancelled']);self.assertEqual(job['result']['cleanup'],'kept_existing')
        self.assertTrue(self.app.mapping);self.assertFalse(any(path=='/maps/build' for path,req in calls))

    def test_explicit_save_during_continue_preparation_uses_existing_owner(self):
        a=self.app.autonomy;self.app.mapping=True;entered=threading.Event();release=threading.Event();calls=[];stopped=False;identity='c'*32
        def remote(path,req=None,**kw):
            nonlocal stopped
            if path=='/maps/build':
                calls.append(req);self.assertEqual(req['action'],'stop');self.assertEqual(req['expected_start_request_id'],identity)
                stopped=True;return dict(ok=True)
            entered.set();release.wait(2)
            return dict(request_protocol=1,phase='saved' if stopped else 'collecting',in_progress=not stopped,mapping=dict(request_id=identity),result_maps=['continued-save'] if stopped else [])
        with patch.object(self.app.hub,'remote_maps',side_effect=remote):
            a.start(dict(mode='explore',use_current=True));self.assertTrue(entered.wait(1))
            a.finish_save();release.set();deadline=time.monotonic()+2
            while a.state=='saving' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(len(calls),1);self.assertEqual(a.state,'completed',a.message)
        self.assertEqual(a.result_maps,['continued-save']);self.assertFalse(self.app.mapping)

    def test_external_navigation_is_not_claimed_by_autonomy_after_restart(self):
        cases.PlatformTests.live(self)
        self.app.link.put(1007,1,dict(Status=3,Value=57,ErrorCode=0))
        with self.assertRaisesRegex(ValueError,'外部或上次导航'):self.app.autonomy.start(dict(mode='explore'))
        self.assertFalse(self.app.autonomy.active);self.assertEqual(self.app.link.sent,[])

    def test_worker_start_failure_does_not_leave_preparing_task(self):
        with patch('m20.autonomy_runtime.threading.Thread.start',side_effect=RuntimeError('thread unavailable')):
            with self.assertRaises(RuntimeError):self.app.autonomy.start(dict(mode='explore'))
        self.assertFalse(self.app.autonomy.active);self.assertIsNone(self.app.autonomy.run_id)
        self.assertFalse(self.app.mapping);self.assertEqual(self.app.velocity,[0.,0.,0.])

    def test_unknown_preparation_readback_unlocks_start_without_reissuing(self):
        hub=self.app.hub;a=self.app.autonomy
        hub.mapping_uncertain=dict(request_id='d'*32,context=hub.context(),message='unknown')
        self.assertFalse(a.snapshot()['can_start']);self.assertTrue(a.snapshot()['preparation_unresolved'])
        with patch.object(hub,'remote_maps',return_value=dict(phase='saved',in_progress=False)) as read:
            hub.get('/api/maps/build/status',{})
        self.assertTrue(a.snapshot()['can_start']);self.assertFalse(a.snapshot()['preparation_unresolved'])
        self.assertEqual(read.call_args.args[0],'/maps/build/status')


if __name__=='__main__':unittest.main()

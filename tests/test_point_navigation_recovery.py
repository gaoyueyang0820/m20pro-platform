"""Synthetic live-surface bypass and continuous multi-waypoint navigation.

Mock transport + ideal feedback plant only. No real maps, networks or robots.
"""
import copy
import math
import unittest
from unittest.mock import patch
import numpy as np
from m20.autonomy_core import disk
from m20.local_trajectory import LocalSurface,TrajectoryFollower
from m20.navigation_detour import plan_local_detour
from m20.observation_errors import GroundObservationPending
from m20.navigation_planner import GridPlanner
import m20.point_navigation as runtime
import test_platform as cases
import test_point_navigation_runtime as fixtures
DeferredWorkers=fixtures.DeferredWorkers
replay_clock=fixtures.replay_clock


class LocalNavigationSearchTests(unittest.TestCase):
    def scene(self):
        return dict(pose=[0.,0.,0.],stamp=1.,floor=list(disk(3)),obstacles=[[3,y] for y in range(-2,3)],_velocity_body=[0.,0.],_speed_mps=0.)
    def test_measured_short_path_goes_sideways_around_point_obstacle(self):
        scan=self.scene();plan=plan_local_detour(scan,[5.,0.]);self.assertIsNotNone(plan)
        self.assertEqual(plan['original_goal'],[5.,0.]);self.assertGreater(plan['goal']['x'],.6)
        self.assertTrue(any(abs(p['y'])>.5 for p in plan['path']))
        # Independent dense body/interior oracle against the real occupied cells.
        for a,b in zip(plan['path'],plan['path'][1:]):
            for t in np.linspace(0,1,15):
                x=a['x']+(b['x']-a['x'])*t;y=a['y']+(b['y']-a['y'])*t
                for ox in np.linspace(-.42,.42,11):
                    for oy in np.linspace(-.225,.225,7):
                        self.assertNotIn((round((x+ox)/.2),round((y+oy)/.2)),set(map(tuple,scan['obstacles'])))
    def test_failed_forward_uses_other_initial_direction_without_widening_unknown_ground(self):
        scan=self.scene();plan=plan_local_detour(scan,[5.,0.],avoid_axes=[.35,0,0]);self.assertIsNotNone(plan)
        first=plan['path'][1];self.assertTrue(first['x']<=0 or abs(first['y'])>.1)
        scan['floor']=[];self.assertIsNone(plan_local_detour(scan,[5.,0.]))
        scan=self.scene();scan['_valid_until']=-1;self.assertIsNone(plan_local_detour(scan,[5.,0.]))
    def test_fixed_heading_bypass_uses_pure_side_axis_and_checks_new_obstructions(self):
        for side in (1,-1):
            with self.subTest(side=side):
                scan=self.scene();scan['obstacles']=[[4,side*y] for y in range(-4,2)]
                plan=plan_local_detour(scan,[5.,0.]);self.assertIsNotNone(plan)
                follower=TrajectoryFollower();follower.set_path(plan['path'])
                rotating=follower.command(scan,yaw_rate=.3,fixed_heading=True)
                self.assertEqual(rotating['state'],'braking');self.assertEqual(rotating['axes'],[0,0,0])
                result=follower.command(scan,fixed_heading=True)
                self.assertEqual(result['state'],'tracking')
                self.assertEqual((result['axes'][0],result['axes'][2]),(0.,0.))
                self.assertGreater(side*result['axes'][1],.30)
                self.assertTrue(LocalSurface.cached(scan).sweep_valid(0.,result['trajectory'][-1][1]))
                changed=dict(scan,obstacles=scan['obstacles']+[[0,2*side]])
                self.assertNotEqual(follower.command(changed,fixed_heading=True)['axes'],result['axes'])
    def test_fixed_heading_bypass_still_allows_requested_final_yaw(self):
        scan=self.scene();scan['obstacles']=[]
        follower=TrajectoryFollower();follower.set_path([[0,0]])
        result=follower.command(scan,goal_yaw=.5,fixed_heading=True)
        self.assertEqual(result['axes'][:2],[0.,0.]);self.assertGreater(result['axes'][2],.1)
    def test_new_contact_or_missing_support_has_no_moving_local_path(self):
        scan=self.scene();scan['obstacles'].append([0,0]);self.assertIsNone(plan_local_detour(scan,[5.,0.]))
        scan=self.scene();scan['floor']=[p for p in scan['floor'] if abs(p[0])<2 and abs(p[1])<2]
        self.assertIsNone(plan_local_detour(scan,[5.,0.]))


class PointNavigationBypassTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    ready=fixtures.PointNavigationRuntimeTests.ready
    setup_live=fixtures.PointNavigationRuntimeTests.setup_live
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def exercise(self,points=None,obstacle=False,fail_global=False,action=None,ground_delay=0.,sealed=False,max_seconds=100):
        points=points or [dict(x=5.05,y=3.05,yaw=0.,type=1)]
        history=[];event=[None];resumed=[False];pose=np.array([2.05,3.05,0.]);velocity=np.zeros(3)
        with replay_clock() as clock:
            a=self.setup_live();source=a.store.path.parent/'maps'/(self.map['id']+'.pgm')
            raw_before=source.read_bytes();grid=np.full((100,120),254,np.uint8);grid[:2]=0;grid[-2:]=0;grid[:,:2]=0;grid[:,-2:]=0
            if obstacle:grid[68:71,27:30]=0
            if sealed:
                grid[65:74,14]=0;grid[65:74,26]=0;grid[65,14:27]=0;grid[73,14:27]=0
            truth=GridPlanner(b'P5\n120 100\n255\n'+grid.tobytes(),.1,[0,0,0])
            route=a.route_save(dict(name='synthetic route',map_id=self.map['id'],points=points),persist=False)['route']
            preview=a.plan_goal(dict(map_id=self.map['id'],x=points[0]['x'],y=points[0]['y'],yaw=None if len(points)>1 else points[0].get('yaw'),engine='platform'))
            workers=DeferredWorkers()
            with patch.object(runtime.threading,'Thread',side_effect=workers.thread):
                a.point_navigation.start(route,a.goal_plans[preview['plan_id']],a._nav_start_generation)
            old=a.link;started=runtime.time.monotonic()
            def advance(duration):
                remaining=duration
                while remaining>1e-8:
                    dt=min(.01,remaining);remaining-=dt;axes=np.array(getattr(old,'v',(0,0,0)),float)
                    if getattr(old,'valid_until',None) is not None and runtime.time.monotonic()>=old.valid_until:axes[:]=0
                    desired=axes.copy();c,s=math.cos(pose[2]),math.sin(pose[2]);desired[:2]=[c*axes[0]-s*axes[1],s*axes[0]+c*axes[1]]
                    velocity[:]+=np.clip(desired-velocity,[-.7*dt,-.7*dt,-1.8*dt],[.7*dt,.7*dt,1.8*dt]);pose[:]+=velocity*dt;clock(dt)
                    if a.link is old:
                        c,s=math.cos(pose[2]),math.sin(pose[2]);body=[c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1]]
                        self.ready(x=float(pose[0]),y=float(pose[1]),yaw=float(pose[2]),velocity=(*body,float(velocity[2])));a.tick(dt)
                    for x in np.linspace(-.42,.42,9):
                        for y in np.linspace(-.225,.225,5):
                            wx=pose[0]+math.cos(pose[2])*x-math.sin(pose[2])*y;wy=pose[1]+math.sin(pose[2])*x+math.cos(pose[2])*y
                            ix,iy=truth.cell(wx,wy);self.assertTrue(truth.free[iy,ix],('body collision',pose.copy(),ix,iy))
                    history.append(dict(pose=pose.copy(),velocity=velocity.copy(),axes=axes.copy(),phase=a.nav.get('phase'),index=a.nav.get('index'),status=a.nav.get('status')))
                    if event[0] is None and action=='pause_resume' and a.nav.get('index')==1:
                        event[0]=len(old.velocities);a.point_navigation.stop('paused')
                    if event[0] is None and action in ['cancel','disconnect'] and a.nav.get('phase')=='local_bypass' and np.linalg.norm(velocity[:2])>.08:
                        event[0]=len(old.velocities)
                        if action=='cancel':a.stop()
                        else:a.connect(dict(mode='demo'))
                    if runtime.time.monotonic()-started>max_seconds:a.done=True
            def sense(mapping=False,**kwargs):
                if runtime.time.monotonic()-started<ground_delay:raise GroundObservationPending('synthetic occlusion','floor_insufficient')
                scan=runtime.simulated_scan(truth,pose,runtime.time.monotonic());c,s=math.cos(pose[2]),math.sin(pose[2])
                scan.update(protocol=1,frame_id='base_link',pose_frame_id='map',pose_stamp=scan['stamp'],age=.1,_valid_until=runtime.time.monotonic()+.5,
                            _velocity_body=[c*velocity[0]+s*velocity[1],-s*velocity[0]+c*velocity[1]],_speed_mps=float(np.linalg.norm(velocity[:2])))
                return scan
            a.autonomy.scan_now=sense
            original=runtime.plan_with_recovery
            def replan(*args,**kwargs):
                if fail_global:raise ValueError('synthetic old-map disconnection')
                return original(*args,**kwargs)
            with patch.object(runtime.time,'sleep',side_effect=advance),patch.object(runtime,'plan_with_recovery',side_effect=replan):
                workers.run();advance(.6)
                if action=='pause_resume':
                    self.assertEqual(a.nav['status'],'paused');self.assertEqual(a.nav['index'],1)
                    point=route['points'][1];preview=a.plan_goal(dict(map_id=self.map['id'],x=point['x'],y=point['y'],yaw=None,engine='platform'))
                    next_workers=DeferredWorkers()
                    with patch.object(runtime.threading,'Thread',side_effect=next_workers.thread):
                        a.point_navigation.start(route,a.goal_plans[preview['plan_id']],a._nav_start_generation,start_index=1)
                    self.assertEqual(a.nav['index'],1);resumed[0]=True;next_workers.run();advance(.6)
            self.assertEqual(source.read_bytes(),raw_before,'bypass never edits the saved map')
            return a,old,pose,velocity,history,event[0],resumed[0]

    def test_paired_scan_turn_is_compared_at_its_own_time_without_cloud_reprojection(self):
        with replay_clock():
            a=self.setup_live();self.ready(yaw=.40,velocity=(.2,0.,.8))
            packet=a.autonomy.scan_now(False);packet.update(pose=[2.05,3.05,0.],age=.5,_valid_until=runtime.time.monotonic()+.15)
            # Measured .8 rad/s accounts for .4 rad since this paired scan.
            a.link.put(1007,2,dict(Location=0,Confidence=.99,PosX=2.05+.2*.5*math.cos(.2),PosY=3.05+.2*.5*math.sin(.2),Yaw=.4))
            original=copy.deepcopy(packet)
            with patch.object(a.autonomy,'scan_now',return_value=packet):result,yaw=a.point_navigation._scan()
            self.assertEqual(result['pose'],original['pose']);self.assertEqual(result['_valid_until'],original['_valid_until']);self.assertEqual(yaw,.8)
            self.ready(x=2.65,y=3.05,yaw=0.,velocity=(0.,0.,0.))
            with patch.object(a.autonomy,'scan_now',return_value=copy.deepcopy(original)):
                with self.assertRaisesRegex(ValueError,'不同步'):a.point_navigation._scan()

    def test_navigation_requests_the_next_sensor_stamp_instead_of_full_frame_wait(self):
        with replay_clock():
            a=self.setup_live();reader=a.autonomy.scan_now
            with patch.object(a.autonomy,'scan_now',wraps=reader) as scan:
                first,_=a.point_navigation._scan();a.point_navigation._scan()
            self.assertIsNone(scan.call_args_list[0].kwargs['after_stamp'])
            self.assertEqual(scan.call_args_list[1].kwargs['after_stamp'],first['stamp'])

    def test_occluded_ground_waits_beyond_old_five_second_timeout_then_continues(self):
        a,link,pose,v,history,event,resumed=self.exercise(ground_delay=6.)
        self.assertEqual(a.nav['status'],'completed',a.nav.get('error'));self.assertLess(math.dist(pose[:2],[5.05,3.05]),.2)
        self.assertTrue(all(not any(r['axes']) for r in history[:500]))

    def test_unavailable_space_keeps_destination_and_waits_beyond_old_progress_timeout(self):
        a,link,pose,v,history,event,resumed=self.exercise(sealed=True,fail_global=True,max_seconds=32)
        self.assertTrue(a.point_navigation.active);self.assertEqual(a.nav['status'],'running');self.assertEqual(a.nav['phase'],'waiting_space')
        self.assertTrue(all(not any(r['v']) for r in link.velocities));self.assertEqual(a.nav['route']['points'][-1]['x'],5.05)

    def test_global_route_failure_uses_live_side_path_and_keeps_original_destination(self):
        a,link,pose,v,history,event,resumed=self.exercise(obstacle=True,fail_global=True)
        self.assertEqual(a.nav['status'],'completed',a.nav.get('error') or a.nav.get('message'))
        self.assertLess(math.dist(pose[:2],[5.05,3.05]),.2);self.assertTrue(any(r['phase']=='local_bypass' for r in history))
        bypass=[row['axes'] for row in history if row['phase']=='local_bypass']
        self.assertTrue(any(abs(axes[1])>.1 and axes[0]==0. for axes in bypass))
        self.assertTrue(all(axes[2]==0. for axes in bypass))
        self.assertEqual(a.nav['route']['points'][-1]['x'],5.05)
    def test_continuous_intermediate_points_only_stop_at_final_destination(self):
        points=[dict(x=x,y=3.05,yaw=0.,type=i%2) for i,x in enumerate([3.05,4.05,5.05])]
        a,link,pose,v,history,event,resumed=self.exercise(points=points)
        self.assertEqual(a.nav['status'],'completed',a.nav.get('error'));self.assertEqual(a.nav['completed'],3)
        for x in [3.05,4.05]:
            passing=[r for r in history if math.dist(r['pose'][:2],[x,3.05])<.20]
            self.assertTrue(passing);self.assertTrue(all(np.linalg.norm(r['velocity'][:2])>.08 for r in passing))
        self.assertLess(np.linalg.norm(v[:2]),.08)
    def test_pause_resume_keeps_next_waypoint_and_final_route_count(self):
        points=[dict(x=x,y=3.05,yaw=0.,type=1) for x in [3.05,4.05,5.05]]
        a,link,pose,v,history,event,resumed=self.exercise(points=points,action='pause_resume')
        self.assertTrue(resumed);self.assertEqual(a.nav['status'],'completed',a.nav.get('error'));self.assertEqual(a.nav['completed'],3)
    def test_cancel_during_local_motion_cannot_emit_late_nonzero_frames(self):
        a,link,pose,v,history,index,resumed=self.exercise(obstacle=True,fail_global=True,action='cancel')
        self.assertIsNotNone(index);self.assertEqual(a.nav['status'],'cancelled');self.assertTrue(all(not any(row['v']) for row in link.velocities[index:]))
    def test_connection_change_during_local_motion_cannot_reclaim_control(self):
        a,link,pose,v,history,index,resumed=self.exercise(obstacle=True,fail_global=True,action='disconnect')
        self.assertIsNotNone(index);self.assertTrue(a.demo);self.assertEqual(a.nav['status'],'idle');self.assertTrue(all(not any(row['v']) for row in link.velocities[index:]))


if __name__=='__main__':unittest.main()

import math
import unittest
import numpy as np
from local_trajectory import TrajectoryFollower,LocalSurface
from navigation_planner import GridPlanner
from point_navigation import simulated_scan


def grid_planner(grid):
    h,w=grid.shape
    return GridPlanner(('P5\n%d %d\n255\n'%(w,h)).encode()+grid.tobytes(),.1,[0,0,0])


def world_grid():
    g=np.full((90,110),254,np.uint8);g[:2]=0;g[-2:]=0;g[:,:2]=0;g[:,-2:]=0
    return g


def run_closed_loop(grid,start,goal,steps=450,gain=1.,dynamic=False):
    planner=grid_planner(grid);route=planner.plan(start,goal[:2],goal[2])
    follow=TrajectoryFollower();follow.set_path(route['dense_path']);pose=np.array(start,float);velocity=np.zeros(3)
    trace=[];replans=0;blocked=0;collision=False;arrived=False
    for k in range(steps):
        if dynamic and k==30:
            # Appears ahead with braking room, not teleported inside the body.
            grid[51:58,60:64]=0;planner=grid_planner(grid)
        scan=simulated_scan(planner,pose,k+1);scan['_velocity_body']=velocity[:2].tolist()
        scan['_speed_mps']=float(np.linalg.norm(velocity[:2]))
        out=follow.command(scan,velocity[2],goal[2],allowed_pose=planner.pose_fits)
        if out['state']=='arrived':arrived=True;break
        if out['state']=='blocked':blocked+=1
        else:blocked=0
        if blocked==5:
            route=planner.plan(pose,goal[:2],goal[2]);follow.set_path(route['dense_path']);replans+=1;blocked=0
        wanted=np.asarray(out['axes'])*gain
        velocity+=np.clip(wanted-velocity,[-.7*.15,-.7*.15,-1.8*.15],[.7*.15,.7*.15,1.8*.15])
        c,s=math.cos(pose[2]+velocity[2]*.075),math.sin(pose[2]+velocity[2]*.075)
        pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*.15
        # Separate dense body-interior point oracle, not the sampled planner's
        # own boolean, against the simulated world's actual occupied cells.
        for x in np.linspace(-.42,.42,11):
            for y in np.linspace(-.225,.225,7):
                wx=pose[0]+math.cos(pose[2])*x-math.sin(pose[2])*y
                wy=pose[1]+math.sin(pose[2])*x+math.cos(pose[2])*y
                ix,iy=math.floor(wx/.1),grid.shape[0]-1-math.floor(wy/.1)
                if not(0<=iy<grid.shape[0] and 0<=ix<grid.shape[1]) or grid[iy,ix]==0:collision=True
        trace.append(dict(pose=pose.tolist(),velocity=velocity.tolist(),axes=out['axes'],ms=out['compute_ms'],state=out['state'],remaining_path_m=out.get('remaining_path_m'),stopping_distance_m=out.get('stopping_distance_m')))
        if collision:break
    return dict(arrived=arrived,collision=collision,pose=pose.tolist(),steps=k+1,replans=replans,
                p95_ms=round(float(np.percentile([t['ms'] for t in trace],95)),2) if trace else 0,arrival_velocity=velocity.tolist() if arrived else None,trace=trace)


class TrajectoryTests(unittest.TestCase):
    def test_turning_path_completes_without_stop_at_each_reference_point(self):
        r=run_closed_loop(world_grid(),[2,3,0],[5,5,.5],gain=.8)
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])
        self.assertTrue(any(abs(t['axes'][0])>.1 and abs(t['axes'][2])>.1 for t in r['trace']) or any(abs(t['axes'][1])>.1 for t in r['trace']))

    def test_narrow_doorway_with_full_body(self):
        g=world_grid();g[:,54:56]=0;g[40:50,54:56]=254
        r=run_closed_loop(g,[3,4.5,0],[8,4.5,0])
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])

    def test_static_wall_requires_a_detour(self):
        g=world_grid();g[35:75,54:56]=0
        r=run_closed_loop(g,[3,3.5,0],[8,3.5,0],steps=650)
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])

    def test_new_obstacle_is_avoided_without_changing_static_walls(self):
        r=run_closed_loop(world_grid(),[3,3.5,0],[8,3.5,0],steps=550,dynamic=True)
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])

    def test_reverse_in_narrow_corridor(self):
        g=world_grid();g[:40]=0;g[50:]=0
        r=run_closed_loop(g,[7,4.5,0],[3,4.5,0],steps=300)
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])
        self.assertTrue(any(t['axes'][0]<0 for t in r['trace']))

    def test_entering_goal_tolerance_at_speed_brakes_before_reporting_arrived(self):
        floor=[(x,y) for x in range(-20,21) for y in range(-20,21)]
        f=TrajectoryFollower();f.set_path([[0,0],[1,0]])
        for x in [.2,.4,.6,.8,.85]:
            out=f.command(dict(pose=[x,0,0],floor=floor,obstacles=[],_velocity_body=[.7,0]),0.,0.)
        self.assertEqual(out['state'],'braking');self.assertEqual(out['axes'],[0,0,0])
        out=f.command(dict(pose=[.85,0,0],floor=floor,obstacles=[],_velocity_body=[0,0]),0.,0.)
        self.assertEqual(out['state'],'arrived')

    def test_nearby_goal_on_another_path_branch_does_not_skip_remaining_arc(self):
        floor=[(x,y) for x in range(-20,21) for y in range(-20,21)]
        f=TrajectoryFollower();f.set_path([[0,0],[0,1],[1,1],[1,0],[.12,0]])
        out=f.command(dict(pose=[.11,0,0],floor=floor,obstacles=[],_velocity_body=[0,0]),0.,0.)
        self.assertNotEqual(out['state'],'arrived');self.assertGreater(out['remaining_path_m'],3.)
        self.assertLess(f.index,4)

    def test_two_times_input_speed_brakes_before_near_wall_goal_and_waits_for_stillness(self):
        g=world_grid();g[:,60:]=0;goal=[5.54,4.5,0.]
        r=run_closed_loop(g,[2.,4.5,0.],goal,steps=550,gain=2.)
        self.assertTrue(r['arrived'],str({k:v for k,v in r.items() if k!='trace'}));self.assertFalse(r['collision'])
        self.assertLess(math.hypot(*r['arrival_velocity'][:2]),.08);self.assertLess(abs(r['arrival_velocity'][2]),.12)
        self.assertTrue(any(t['state']=='braking' and math.dist(t['pose'][:2],goal[:2])>.18 for t in r['trace']))
        self.assertLess(math.dist(r['pose'][:2],goal[:2]),.18)

    def test_goal_yaw_motion_must_settle_before_arrival(self):
        f=TrajectoryFollower();f.set_path([[0,0]])
        scan=dict(pose=[0,0,0],floor=[],obstacles=[],_velocity_body=[0,0])
        out=f.command(scan,.8,0.)
        self.assertEqual(out['state'],'braking');self.assertEqual(out['axes'],[0,0,0])
        self.assertEqual(f.command(scan,0.,0.)['state'],'arrived')

    def test_short_final_approach_retains_useful_input(self):
        f=TrajectoryFollower();f.set_path([[0,0],[.22,0]])
        scan=dict(pose=[0,0,0],floor=[(x,y) for x in range(-20,21) for y in range(-20,21)],obstacles=[],_velocity_body=[0,0])
        out=f.command(scan,0.,0.)
        self.assertEqual(out['state'],'tracking');self.assertGreaterEqual(abs(out['axes'][0]),.20)

    def test_autonomy_keeps_braking_zero_without_falling_back_to_forward(self):
        import tempfile,threading,time
        from pathlib import Path
        from unittest.mock import patch
        from platform_core import Store
        from m20_platform import Platform
        from autonomy_core import Explorer
        with tempfile.TemporaryDirectory() as directory:
            app=Platform(Store(Path(directory)/'workspace.json'));a=app.autonomy
            reached=threading.Event();driven=threading.Event();requests=[];original=a._drive
            def braking(*args,**kwargs):
                reached.set();return dict(state='braking',axes=[0.,0.,0.],reason='offline terminal braking')
            def drive(run_id,forward,yaw,valid_until=None,side=0.):
                result=original(run_id,forward,yaw,valid_until,side=side)
                if reached.is_set():requests.append((forward,side,yaw));driven.set()
                return result
            try:
                with patch.object(TrajectoryFollower,'command',side_effect=braking),patch.object(Explorer,'command_axes',side_effect=AssertionError('braking must not fall back')) as fallback,patch.object(a,'_drive',side_effect=drive):
                    a.start(dict(mode='explore',minutes=.1,radius=3))
                    limit=time.monotonic()+3
                    while not driven.is_set() and time.monotonic()<limit:
                        a.post('/api/autonomy/lease',dict(run_id=a.run_id));app.tick(.02);time.sleep(.02)
                    self.assertTrue(driven.is_set(),a.message);self.assertEqual(fallback.call_count,0)
                    self.assertTrue(requests);self.assertTrue(all(not any(v) for v in requests))
                    with app.lock:a.pause();app.done=True
            finally:
                with app.lock:a.pause();app.done=True
                if getattr(a,'observation_pump',None):a.observation_pump.cancel()
                app.hub.close()

    def test_missing_floor_not_invented_by_path(self):
        f=TrajectoryFollower();f.set_path([[0,0],[3,0]])
        out=f.command(dict(pose=[0,0,0],floor=[],obstacles=[],_velocity_body=[0,0]))
        self.assertEqual(out['axes'],[0,0,0])

    def test_missing_feedback_cannot_generate_motion(self):
        f=TrajectoryFollower();f.set_path([[0,0],[3,0]])
        out=f.command(dict(pose=[0,0,0],floor=[],obstacles=[],_velocity_body=None))
        self.assertEqual(out['state'],'waiting_feedback')

    def test_body_sweep_rejects_obstacle_and_unknown_support(self):
        floor=[(x,y) for x in range(-20,21) for y in range(-20,21)]
        s=LocalSurface(dict(floor=floor,obstacles=[(4,0)]))
        self.assertTrue(s.valid(0,0,0));self.assertFalse(s.valid(.5,0,0))
        s=LocalSurface(dict(floor=[(x,0) for x in range(-20,21)],obstacles=[]))
        self.assertFalse(s.valid(1,0,0))

    def test_precise_obstacle_geometry_avoids_grid_rounding_collision_but_keeps_contact(self):
        floor=[(x,y) for x in range(-20,21) for y in range(-20,21)]
        coarse=dict(floor=floor,obstacles=[(0,1)])
        self.assertFalse(LocalSurface(coarse).valid(0,0,0))
        precise=dict(coarse,geometry_version=2,obstacle_bounds=[[-.04,.29,.04,.31]])
        self.assertTrue(LocalSurface(precise).valid(0,0,0))
        self.assertFalse(LocalSurface(precise).valid(0,.1,0))

    def test_sensor_reduction_retains_all_obstacle_extents(self):
        from autonomy_sensing import supported_floor
        floor=np.array([[x,y,-.55] for x in np.arange(-2,2,.08) for y in np.arange(-2,2,.08)])
        obstacles=np.array([[1.02,.21,-.3],[1.08,.25,-.3],[1.01,.29,-.3]])
        s=supported_floor(np.vstack([floor,obstacles]));self.assertEqual(s['geometry_version'],2)
        for x,y,_ in obstacles:
            self.assertTrue(any(a-.0001<=x<=c+.0001 and b-.0001<=y<=d+.0001 for a,b,c,d in s['obstacle_bounds']))


if __name__=='__main__':unittest.main()

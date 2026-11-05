"""Functional geometric routes and replanning, with a sampled-body oracle.

These are planar simulations; no robot sockets or locomotion physics.
"""
import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.navigation_planner import GridPlanner
from m20.robot_geometry import CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_WIDTH


def image_room(width=120,height=100):
    a=np.full((height,width),254,np.uint8);a[:2]=0;a[-2:]=0;a[:,:2]=0;a[:,-2:]=0;return a


def planner(image):
    h,w=image.shape
    return GridPlanner(('P5\n%d %d\n255\n'%(w,h)).encode()+image.tobytes(),.1,[0,0,0])


def validate_sampled_body(p,result):
    # An independent dense body-point oracle over every returned SE2 segment.
    xx,yy=np.meshgrid(np.linspace(-CLEARANCE_HALF_LENGTH,CLEARANCE_HALF_LENGTH,47),
                      np.linspace(-CLEARANCE_HALF_WIDTH,CLEARANCE_HALF_WIDTH,29))
    points=np.column_stack((xx.ravel(),yy.ravel()))
    dense=result['dense_path']
    for a,b in zip(dense,dense[1:]):
        delta=math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw']))
        for t in np.linspace(0,1,5):
            yaw=a['yaw']+delta*t;c,s=math.cos(yaw),math.sin(yaw)
            x=a['x']+(b['x']-a['x'])*t+c*points[:,0]-s*points[:,1]
            y=a['y']+(b['y']-a['y'])*t+s*points[:,0]+c*points[:,1]
            ix=np.floor((x-p.origin[0])/p.res).astype(int);iy=p.h-1-np.floor((y-p.origin[1])/p.res).astype(int)
            if not ((ix>=0)&(ix<p.w)&(iy>=0)&(iy<p.h)).all():return False
            if not p.free[iy,ix].all():return False
    return True


class GlobalPlanningV3Tests(unittest.TestCase):
    def test_open_space_uses_arbitrary_angle_and_exact_endpoints(self):
        p=planner(image_room());start=[2.01,2.02,.1];goal=[9.73,6.81]
        r=p.plan(start,goal)
        self.assertEqual(r['diagnostics']['method'],'continuous_direct');self.assertEqual(r['expanded'],0)
        self.assertLess(abs(r['length_m']-math.dist(start[:2],goal)),.006)
        self.assertEqual((r['goal']['x'],r['goal']['y']),tuple(goal))
        self.assertTrue(validate_sampled_body(p,r))

    def test_slanted_narrow_corridor_avoids_forced_heading_quantization(self):
        h=w=140;yy,xx=np.mgrid[:h,:w];wx=(xx+.5)*.1;wy=(h-yy-.5)*.1
        angle=.29;c,s=math.cos(angle),math.sin(angle);x=wx-2.;y=wy-3.
        along=c*x+s*y;side=-s*x+c*y
        a=np.where((along>-.8)&(along<10.5)&(abs(side)<.43),254,0).astype(np.uint8)
        p=planner(a);start=[2,3,angle];goal=[2+8*c,3+8*s]
        r=p.plan(start,goal);self.assertLess(r['length_m'],8.01)
        self.assertTrue(validate_sampled_body(p,r))

    def test_exact_subcell_pose_preserves_original_six_tenths_corridor_case(self):
        # Old planner moved y=2.5 to cell centre 2.55 and then rejected it.
        # Continuous SAT must accept the original feasible pose, not remove walls.
        a=np.zeros((50,100),np.uint8);a[22:28,2:98]=254;p=planner(a)
        r=p.plan([2.05,2.5,.03],[8.05,2.5])
        self.assertTrue(validate_sampled_body(p,r))
        self.assertEqual(r['goal']['y'],2.5)
        a[:]=0;a[23:28,2:98]=254
        with self.assertRaises(ValueError):planner(a).plan([2.05,2.5,.03],[8.05,2.5])

    def test_local_collision_queries_do_not_inherit_expired_search_budget(self):
        p=planner(image_room());p._deadline=-1
        self.assertTrue(p.pose_fits(3,3,.2))
        self.assertTrue(p.segment_fits([3,3],[4,3],0))

    def test_reverse_out_of_narrow_corridor_without_impossible_turn(self):
        a=np.zeros((60,120),np.uint8);a[26:34,2:118]=254;p=planner(a)
        r=p.plan([8.05,3.05,0],[2.05,3.05],0)
        self.assertLess(r['length_m'],6.01)
        self.assertTrue(all(abs(q['yaw'])<1e-6 for q in r['dense_path']))
        self.assertTrue(validate_sampled_body(p,r))

    def test_wall_route_is_shortened_without_crossing_wall_or_turning_through_it(self):
        a=image_room(100,80);a[15:70,49:51]=0;p=planner(a)
        r=p.plan([2,3,0],[8,3],budget=5)
        self.assertLess(r['length_m'],9.)
        self.assertLess(r['expanded'],5000)
        self.assertTrue(validate_sampled_body(p,r))

    def test_dynamic_obstacle_replans_then_removal_restores_direct_route(self):
        p=planner(image_room());start=[2.05,5.05,0];goal=[10.05,5.05]
        direct=p.plan(start,goal);self.assertEqual(direct['expanded'],0)
        block=[(float(x),float(y)) for x in np.arange(5.5,6.5,.08) for y in np.arange(4.3,5.8,.08)]
        change=p.update_obstacles(block);self.assertGreater(change['changed_cells'],0)
        reroute=p.plan(start,goal,budget=5)
        self.assertGreater(reroute['length_m'],direct['length_m']);self.assertTrue(validate_sampled_body(p,reroute))
        same=p.update_obstacles(block);self.assertEqual(same['changed_cells'],0)
        self.assertTrue(p._potential_cache)
        p.update_obstacles([]);r=p.plan(start,goal)
        self.assertEqual(r['diagnostics']['method'],'continuous_direct')
        self.assertFalse(p.free[0,0])

    def test_closed_loop_replan_from_progress_after_new_obstacle(self):
        p=planner(image_room());goal=[10.05,5.05];route=p.plan([2.05,5.05,0],goal)
        # Perfect-pose kinematic progress along the initial route; an object then
        # enters ahead. Replan from the observed pose, not the original start.
        q=route['dense_path'][20];start=[q['x'],q['y'],q['yaw']]
        p.update_obstacles([(x,y) for x in np.arange(6,7,.09) for y in np.arange(4.4,5.8,.09)])
        revised=p.plan(start,goal,budget=5)
        self.assertAlmostEqual(revised['path'][0]['x'],start[0])
        self.assertTrue(validate_sampled_body(p,revised))
        final=revised['dense_path'][-1]
        self.assertLess(math.dist([final['x'],final['y']],goal),.001)
        self.assertGreater(max(abs(q['y']-5.05) for q in revised['path']),1.)

    def test_unknown_divider_does_not_become_any_angle_shortcut(self):
        a=image_room();a[:,59:61]=205;p=planner(a)
        with self.assertRaisesRegex(ValueError,'没有足够宽'):p.plan([2,5,0],[10,5])

if __name__=='__main__':unittest.main()

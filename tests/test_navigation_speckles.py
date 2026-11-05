"""Synthetic saved-map residue scenarios; no private map, robot or network I/O."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from m20.navigation_planner import GridPlanner,_static_speckles
from test_navigation_planning_v3 import validate_sampled_body
from test_live_surface_localization import packet


def planner(image,soft=True,live=False,res=.1,origin=None,**kwargs):
    h,w=image.shape
    return GridPlanner(('P5\n%d %d\n255\n'%(w,h)).encode()+image.tobytes(),res,origin or [0,0,0],
                       soft_static_speckles=soft,enable_live_surface=live,**kwargs)


def room(w=160,h=100):
    grid=np.full((h,w),254,np.uint8);grid[:2]=0;grid[-2:]=0;grid[:,:2]=0;grid[:,-2:]=0
    return grid


def corridor():
    grid=np.zeros((90,160),np.uint8);grid[39:51,2:158]=254
    return grid


class NavigationSpeckleTests(unittest.TestCase):
    def test_dotted_people_trail_no_longer_closes_entire_corridor(self):
        grid=corridor();truth=planner(grid,False)
        for column in range(23,140,7):grid[45,column]=0
        start=[1.55,4.45,0.];goal=[14.55,4.45]
        with self.assertRaises(ValueError):planner(grid,False).plan(start,goal,budget=5)
        p=planner(grid);raw=p.original.copy();base=p.base_free.copy()
        route=p.plan(start,goal,budget=5)
        self.assertTrue(route['ok']);self.assertGreater(route['static_speckles']['planning_cells'],10)
        self.assertEqual([route['goal'][k] for k in ('x','y')],goal)
        self.assertEqual([route['dense_path'][0][k] for k in ('x','y','yaw')],start)
        self.assertTrue(validate_sampled_body(truth,route))
        np.testing.assert_array_equal(p.original,raw);np.testing.assert_array_equal(p.base_free,base)

    def test_low_confidence_is_a_positive_cost_and_prefers_clean_space(self):
        grid=room();grid[49:51,78:80]=0;p=planner(grid)
        start=[2.05,5.05,0.];goal=[13.05,5.05]
        self.assertGreater(p._segment_soft_cost(start,goal),0.)
        route=p.plan(start,goal,budget=5)
        self.assertTrue(route['ok'])
        actual=sum(p._segment_soft_cost([a['x'],a['y']],[b['x'],b['y']]) for a,b in zip(route['path'],route['path'][1:]))
        self.assertLess(actual,p._segment_soft_cost(start,goal))
        self.assertGreater(max(abs(q['y']-5.05) for q in route['path']),.3)

    def test_single_continuous_thin_wall_cannot_be_softened(self):
        grid=room();grid[:,79]=0;p=planner(grid)
        self.assertFalse(p.soft_cells[:,79].any())
        with self.assertRaisesRegex(ValueError,'没有足够宽'):p.plan([2,5,0],[13,5],budget=5)

    def test_long_diagonal_structure_is_eight_connected_and_stays_hard(self):
        grid=room()
        for offset in range(45):grid[20+offset,65+offset]=0
        p=planner(grid)
        self.assertFalse(any(p.soft_cells[20+i,65+i] for i in range(45)))

    def test_door_jambs_attached_to_walls_are_not_removed(self):
        grid=room();grid[:,79:81]=0;grid[45:55,79:81]=254
        grid[44,77:79]=0;grid[55,81:83]=0;p=planner(grid)
        self.assertFalse(p.soft_cells[44,77]);self.assertFalse(p.soft_cells[55,82])
        self.assertFalse(p.free[44,77]);self.assertFalse(p.free[55,82])

    def test_unknown_gap_and_unknown_adjacent_fragment_stay_hard(self):
        grid=room();grid[:,79]=205;grid[40,78]=0;p=planner(grid)
        self.assertFalse(p.free[:,79].any());self.assertFalse(p.free[40,78])
        with self.assertRaises(ValueError):p.plan([2,5,0],[13,5],budget=5)

    def test_larger_furniture_is_not_classified_as_speckle(self):
        grid=room();grid[42:51,75:85]=0;p=planner(grid)
        self.assertFalse(p.soft_cells[42:51,75:85].any())
        self.assertFalse(p.pose_fits(8,5,0))

    def test_realtime_point_overrides_soft_map_even_inside_robot_pose(self):
        grid=room();grid[49,80]=0
        p=planner(grid,obstacles=[[8.05,5.05]],robot_pose=[8.05,5.05,0.])
        self.assertTrue(p.soft_cells[49,80]);self.assertFalse(p.free[49,80])
        self.assertFalse(p.pose_fits(8.05,5.05,0.));self.assertEqual(p.self_projection_ignored,0)

    def test_current_lidar_returns_override_and_replan_around_saved_residue(self):
        grid=room();grid[49,80]=0;p=planner(grid,live=True)
        truth=planner(room(),False)
        scan,loc=packet(truth,actual=(7.05,5.05,0.),stamp=100.)
        scan['obstacles']=[[5,0]];scan['obstacle_bounds']=[[.98,-.06,1.08,.06]]
        p.update_live_surface(scan,loc)
        self.assertFalse(p.free[49,80]);self.assertFalse(p.pose_fits(8.05,5.05,0.))
        route=p.plan([7.05,5.05,0.],[11.05,5.05],budget=5)
        self.assertTrue(validate_sampled_body(p,route))
        self.assertGreater(max(abs(q['y']-5.05) for q in route['path']),.3)

    def test_dynamic_obstacle_removal_restores_soft_cost_not_original_hard_noise(self):
        grid=room();grid[49,80]=0;p=planner(grid)
        p.update_obstacles([[8.05,5.05]]);self.assertFalse(p.free[49,80])
        p.update_obstacles([]);self.assertTrue(p.free[49,80]);self.assertGreater(p.soft_cost[49,80],0)
        self.assertFalse(p.base_free[49,80])

    def test_live_obstacle_can_close_the_whole_narrow_corridor(self):
        grid=corridor();grid[45,80]=0;p=planner(grid)
        wall=[(8.05,y) for y in np.arange(3.9,5.1,.05)]
        p.update_obstacles(wall)
        with self.assertRaises(ValueError):p.plan([2.05,4.45,0.],[14.05,4.45],budget=5)

    def test_subcell_start_uses_checked_neighbour_bridge_without_teleporting(self):
        grid=room();grid[47:90,43:45]=0;p=planner(grid,False)
        # Synthetic wall ends at y=5.3. The real y=5.58 pose fits, whereas
        # its nearest y=5.55 centre infringes the normal 3 cm margin.
        start=[4.02,5.58,.002];goal=[2.05,7.05]
        self.assertTrue(p.pose_fits(*start))
        route=p.plan(start,goal,budget=5)
        self.assertEqual([route['dense_path'][0][k] for k in ('x','y','yaw')],start)
        self.assertTrue(route['diagnostics'].get('start_cell_adjusted'))
        self.assertTrue(validate_sampled_body(p,route))

    def test_downsampling_never_softens_mixed_unknown_cell(self):
        grid=room(120,120);grid[60,60]=0;grid[61,61]=205
        p=planner(grid,res=.05)
        ix,iy=p.cell(3.025,2.975)
        self.assertFalse(p.free[iy,ix])
        self.assertFalse(p.soft_cells[iy,ix])

    def test_numpy_fallback_matches_component_fast_path(self):
        grid=room(60,60);grid[25,25]=0;grid[28:30,28:30]=0;grid[2:40,40]=0;grid[40,10]=205
        probability=1-grid/255.
        expected,report=_static_speckles(probability,.1,.196,.65)
        with patch.dict(sys.modules,{'cv2':None}):actual,fallback=_static_speckles(probability,.1,.196,.65)
        np.testing.assert_array_equal(actual,expected);self.assertEqual(fallback,report)

    def test_disabled_mode_keeps_original_geometric_oracle_behavior(self):
        grid=room();grid[49,80]=0;p=planner(grid,False)
        self.assertFalse(p.free[49,80]);self.assertFalse(p.static_speckles['enabled'])
        self.assertFalse(p.soft_cost.any())


if __name__=='__main__':unittest.main()

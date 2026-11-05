"""A requested tight goal may become a nearby explicit, body-checked endpoint."""
import math
import unittest
from test_navigation_planning_v3 import image_room, planner, validate_sampled_body
from m20.navigation_planner import GoalSpaceError


class GoalApproachTests(unittest.TestCase):
    def test_known_free_goal_too_close_to_wall_gets_nearby_reachable_endpoint(self):
        p=planner(image_room(100,80));goal=[.4,3.]
        with self.assertRaises(GoalSpaceError):p.plan([2.,3.,0.],goal)
        r=p.plan([2.,3.,0.],goal,goal_tolerance=.65,budget=5.)
        self.assertTrue(r['goal_adjustment']['applied'])
        self.assertEqual(r['requested_goal'],dict(x=.4,y=3.))
        self.assertGreater(r['goal']['x'],goal[0])
        self.assertLessEqual(math.dist([r['goal']['x'],r['goal']['y']],goal),.65)
        self.assertTrue(validate_sampled_body(p,r))
        self.assertEqual(r['goal_adjustment']['actual'],r['goal'])

    def test_explicit_final_heading_is_preserved_at_adjusted_goal(self):
        p=planner(image_room(100,80));r=p.plan([2.,3.,0.],[.4,3.],0.,goal_tolerance=.65)
        self.assertAlmostEqual(r['goal']['yaw'],0.)
        self.assertTrue(r['goal_adjustment']['applied'])
        self.assertTrue(validate_sampled_body(p,r))

    def test_already_feasible_goal_stays_exact(self):
        p=planner(image_room());goal=[8.23,4.17]
        r=p.plan([2.,3.,0.],goal,goal_tolerance=.65)
        self.assertFalse(r['goal_adjustment']['applied'])
        self.assertEqual([r['goal']['x'],r['goal']['y']],goal)

    def test_near_goal_does_not_jump_to_the_other_side_of_a_wall(self):
        a=image_room(100,80);a[:,49:51]=0;p=planner(a)
        with self.assertRaises(ValueError):p.plan([2.,3.,0.],[5.25,3.],goal_tolerance=.65,budget=5.)

    def test_unknown_goal_cannot_be_invented_as_known_floor(self):
        a=image_room(100,80);p=planner(a);gx,gy=p.cell(8.,3.);a[gy,gx]=205;p=planner(a)
        with self.assertRaisesRegex(ValueError,'未知'):p.plan([2.,3.,0.],[8.,3.],goal_tolerance=.65)

    def test_no_valid_endpoint_inside_requested_limit_is_reported(self):
        p=planner(image_room(100,80))
        with self.assertRaisesRegex(ValueError,'附近'):p.plan([2.,3.,0.],[.3,3.],0.,goal_tolerance=.15)

    def test_unsafe_start_is_not_relabelled_as_a_goal_adjustment(self):
        p=planner(image_room(100,80))
        with self.assertRaisesRegex(ValueError,'起点'):p.plan([.3,3.,0.],[.4,3.],goal_tolerance=.65)
        self.assertEqual(p.goal_tolerance,.65,'start recovery keeps this explicit goal policy')

    def test_replanning_to_actual_endpoint_preserves_original_click_for_display(self):
        from m20.point_navigation import PointNavigator
        from types import SimpleNamespace
        p=planner(image_room(100,80));original=p.plan([2.,3.,0.],[.4,3.],0.,goal_tolerance=.65)
        runtime=PointNavigator(SimpleNamespace());runtime.goal_intent=original
        fresh=dict(goal=dict(original['goal']),requested_goal=dict(x=original['goal']['x'],y=original['goal']['y']),goal_adjustment=dict(applied=False))
        shown=runtime._display_plan(fresh)
        self.assertTrue(shown['goal_adjustment']['applied']);self.assertEqual(shown['requested_goal'],dict(x=.4,y=3.))
        self.assertFalse(fresh['goal_adjustment']['applied'])
        self.assertFalse(runtime._display_plan(dict(fresh,goal=dict(x=5.,y=3.)))['goal_adjustment']['applied'])


if __name__=='__main__':unittest.main()

"""Occluded-room next-best-view tests; no sockets, cameras or robot commands."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from exploration_frontiers import FrontierSelector,reachable,ray_cells
from autonomy_core import Explorer,disk


class FrontierTests(unittest.TestCase):
    def test_information_does_not_see_through_wall_or_diagonal_gap(self):
        f=FrontierSelector(.2);free={(x,y) for x in range(-12,1) for y in range(-12,13)}
        wall={(1,y) for y in range(-14,15)}
        within=lambda p:-12<=p[0]<=12 and -12<=p[1]<=12
        gain,_,_=f.information((0,0),free,wall,within)
        self.assertEqual(gain,0.)
        self.assertIn((1,0),list(ray_cells((0,0),(2,2))))
        self.assertIn((0,1),list(ray_cells((0,0),(2,2))))

    def test_graph_cost_goes_around_wall_instead_of_euclidean_shortcut(self):
        free={(x,y) for x in range(16) for y in range(12)}-{(7,y) for y in range(9)}
        costs,parent=reachable((3,3),free,.2)
        self.assertGreater(costs[(11,3)],3.)
        path=[];p=(11,3)
        while p is not None:path.append(p);p=parent[p]
        self.assertTrue(any(y>=9 for _,y in path));self.assertTrue(all(p in free for p in path))

    def test_failed_approach_deferred_then_reopens_after_cooldown(self):
        f=FrontierSelector(.2);f.blocked((5,0),10,{(6,0)},'local route blocked')
        self.assertTrue(f._deferred((5,0),12,{(6,0)}))
        self.assertFalse(f._deferred((5,0),21,{(6,0)}))
        self.assertFalse(f._deferred((5,0),12,set()))

    def test_target_distance_is_viewpoint_scale_not_dense_cell_visiting(self):
        e=Explorer(8);e.ingest(dict(pose=[0,0,0],floor=list(disk(3.)),obstacles=[],stamp=1))
        self.assertEqual(e.plan(),'planned')
        self.assertGreaterEqual(math.dist(e.goal,(0,0))*.2,1.)
        self.assertGreater(e.frontier_metrics['information_gain_m2'],1.)
        self.assertGreater(e.frontier_metrics['frontier_clusters'],0)

    def test_static_goal_not_changed_by_small_pose_noise(self):
        e=Explorer(8);e.ingest(dict(pose=[0,0,0],floor=list(disk(3)),obstacles=[],stamp=1));e.plan();goal=e.goal
        for i in range(8):
            e.ingest(dict(pose=[.005*i,0,0],floor=list(disk(3)),obstacles=[],stamp=2+i));e.plan()
            self.assertEqual(e.goal,goal)

    def test_coarse_revisit_does_not_depend_on_full_interior_rescan(self):
        e=Explorer(3);e.ingest(dict(pose=[0,0,0],floor=list(disk(5)),obstacles=[],stamp=1));e.plan()
        self.assertEqual(e.target_kind,'observe');self.assertGreater(e.frontier_metrics['revisit_gain_m2'],0)
        self.assertGreaterEqual(math.dist(e.goal,(0,0))*.2,1.)

    def test_blocked_callback_preserves_map_and_defers_current_goal(self):
        e=Explorer(8);e.ingest(dict(pose=[0,0,0],floor=list(disk(3)),obstacles=[],stamp=1));e.plan()
        old=e.goal;observed=set(e.free);e.note_blocked('no progress');e.plan()
        self.assertEqual(e.free,observed);self.assertNotEqual(e.goal,old)
        self.assertGreater(e.frontier_metrics['deferred_regions'],0)

    def test_two_rooms_with_occluded_doorway_reach_98_percent_coverage(self):
        from tools.benchmark_frontier_exploration import run
        result=run(Explorer,2,max_steps=250)
        self.assertGreaterEqual(result['coverage'],.98)
        self.assertEqual(result['rooms_entered'],2)
        self.assertLess(result['distance_m'],45)
        self.assertLess(result['goal_changes'],25)

    def test_three_rooms_with_offset_doors_complete_without_dense_rescanning(self):
        from tools.benchmark_frontier_exploration import run
        result=run(Explorer,3,max_steps=350)
        self.assertGreaterEqual(result['coverage'],.98)
        self.assertEqual(result['rooms_entered'],3)
        self.assertLess(result['distance_m'],75)
        self.assertLess(result['goal_changes'],35)


if __name__=='__main__':unittest.main()

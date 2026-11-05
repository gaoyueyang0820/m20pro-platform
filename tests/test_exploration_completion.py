"""Synthetic open-space selection and completion; no robot/network I/O."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from m20.autonomy_core import Explorer,disk
from m20.autonomy_runtime import ExplorationCompletion
from m20.exploration_frontiers import FrontierSelector


class ExplorationCompletionTests(unittest.TestCase):
    def test_visited_prefix_does_not_hide_remaining_open_space_in_any_direction(self):
        for angle in [0,math.pi/2,-math.pi/2,math.pi]:
            with self.subTest(angle=angle):
                c,s=math.cos(angle),math.sin(angle)
                rotate=lambda p:(round(c*p[0]-s*p[1]),round(s*p[0]+c*p[1]))
                free={rotate((x,y)) for x in range(-10,161) for y in range(-10,11)}
                f=FrontierSelector(.2);f.viewpoints=[rotate((x,y)) for x in range(-12,137,4) for y in range(-12,13,4)]
                result=f.select((0,0),angle,free,free,set(),lambda p:True,1)
                self.assertIsNotNone(result,f.last);self.assertIn(result['goal'],free)
                self.assertTrue(all(p in free for p in result['path']))
                self.assertGreater(result['goal'][0]*c+result['goal'][1]*s,135)

    def test_candidate_budget_continues_search_instead_of_reporting_complete(self):
        free={(x,y) for x in range(-50,51) for y in range(-50,51)};f=FrontierSelector(.2)
        goal=(48,48);pending=0
        def information(p,*args):return (1.,0.,set()) if p==goal else (0.,0.,set())
        with patch.object(f,'information',side_effect=information):
            for stamp in range(1,12):
                result=f.select((0,0),0,free,free,set(),lambda p:True,stamp)
                if result is not None:break
                pending+=1;self.assertEqual(f.last['search_status'],'searching',f.last)
                self.assertGreater(f.last['search_remaining'],0)
        self.assertGreater(pending,0);self.assertIsNotNone(result,f.last);self.assertEqual(result['goal'],goal)

    def test_heading_constraint_starts_available_retreat_instead_of_completing(self):
        e=Explorer(8);e.ingest(dict(pose=[0,0,0],floor=list(disk(3)),obstacles=[],stamp=1,_velocity_body=[0.,0.]))
        e.visited.update(p for p in e.free if p[0]<=0)
        with patch('m20.autonomy_core.clearance',side_effect=lambda scan,f,yaw,side=0.:(f<0,'')):
            self.assertEqual(e.plan(),'recovering',e.frontier_metrics)
            self.assertEqual(e.frontier_metrics['search_status'],'maneuver')
            self.assertLess(e.command_axes()[0],0)

    def test_clear_short_approach_is_not_rejected_by_viewpoint_spacing(self):
        free=set(disk(3));allowed={(i,0) for i in range(5)}
        f=FrontierSelector(.2);f.observe_pose((0,0,0))
        result=f.select((0,0),0,allowed,free,set(),lambda p:True,1,visited={(0,0)})
        self.assertIsNotNone(result,f.last);self.assertTrue(all(p in allowed for p in result['path']))
        self.assertLess(math.dist(result['goal'],(0,0))*.2,.85)
        self.assertGreaterEqual(f.last['information_gain_m2'],.08)

    def test_maneuver_rechecks_fresh_conditions_without_map_cell_changes(self):
        free=set(disk(3));f=FrontierSelector(.2)
        self.assertIsNone(f.select((0,0),0,free,free,set(),lambda p:True,1,possible=lambda heading:False))
        self.assertEqual(f.last['search_status'],'maneuver')
        self.assertIsNotNone(f.select((0,0),0,free,free,set(),lambda p:True,2,possible=lambda heading:True))

    def test_missing_observations_are_not_completed_coverage(self):
        e=Explorer(5);e.ingest(dict(pose=[0,0,0],floor=[],obstacles=[],stamp=1))
        self.assertEqual(e.plan(),'blocked');self.assertEqual(e.command_axes(),(0.,0.,0.))

    def test_map_scope_follows_newly_observed_passages_while_radius_scope_stays_configured(self):
        fixed=Explorer(6);expanding=Explorer(6,expand=True)
        for i in range(8):
            scan=dict(pose=[i*.5,0,0],floor=list(disk(3.)),obstacles=[],stamp=i+1)
            fixed.ingest(scan);expanding.ingest(scan)
        self.assertEqual(fixed.radius,6);self.assertGreater(expanding.radius,6)
        self.assertEqual(expanding.origin,(0.,0.));self.assertTrue(fixed.free.issubset(expanding.free))

    def test_closed_observed_area_can_complete(self):
        free={(x,y) for x in range(-10,11) for y in range(-10,11)}
        walls={(x,y) for x in range(-11,12) for y in range(-11,12) if abs(x)==11 or abs(y)==11}
        f=FrontierSelector(.2);f.viewpoints=[(x,y) for x in range(-10,11,4) for y in range(-10,11,4)]
        self.assertIsNone(f.select((0,0),0,free,free,walls,lambda p:True,1))
        self.assertEqual(f.last['search_status'],'complete')

    def completion(self):
        return ExplorationCompletion(),dict(stamp=1.,pose=[0.,0.,0.]),SimpleNamespace(free={(0,0)},obstacles=set())
    def test_repeated_source_frame_never_completes(self):
        done,scan,e=self.completion()
        for now in range(20):self.assertFalse(done.observe('exhausted',scan,e,now))
    def test_five_fast_empty_results_do_not_finish_but_stable_new_observations_do(self):
        done,scan,e=self.completion()
        for i in range(5):
            scan['stamp']=i+1;self.assertFalse(done.observe('exhausted',scan,e,i*.2))
        scan['stamp']=6;self.assertTrue(done.observe('exhausted',scan,e,3.))
    def test_new_area_or_new_plan_cancels_pending_completion(self):
        done,scan,e=self.completion()
        for i in range(5):
            scan['stamp']=i+1;done.observe('exhausted',scan,e,i*.2)
        e.free.add((1,0));scan['stamp']=6;self.assertFalse(done.observe('exhausted',scan,e,3.))
        self.assertFalse(done.observe('searching',scan,e,4.));self.assertEqual(done.frames,0)
        self.assertFalse(done.observe('planned',scan,e,5.));self.assertIsNone(done.since)


if __name__=='__main__':unittest.main()

"""Continuous observation-point handoff and observed stuck recovery, offline."""
import copy
import math
import time
import unittest
from unittest.mock import patch
import numpy as np
from autonomy_core import Explorer,disk
from autonomy_runtime import MotionProgress
from local_trajectory import TrajectoryFollower,exploration_step
from tools.benchmark_exploration_flow import stuck_loop,multi_vantage


def open_scan(x=0.,stamp=1.,velocity=.35):
    return dict(pose=[x,0.,0.],floor=[list(p) for p in disk(3.)],obstacles=[],stamp=stamp,age=.3,
                _velocity_body=[velocity,0.],_speed_mps=velocity)


def near_vantage():
    e=Explorer(4);e.ingest(open_scan(0,1));e.path=[(i,0) for i in range(6)];e.goal=(5,0);e.plan_state='planned'
    f=TrajectoryFollower();f.set_path([(x*.2,y*.2) for x,y in e.path],terminal=False);f.plan_key=tuple(e.path)
    for i,x in enumerate([0.,.2,.4]):
        s=open_scan(x,2+i);e.ingest(s);f.command(s)
    scan=open_scan(.6,6);e.ingest(scan)
    return e,f,scan


class ExplorationFlowTests(unittest.TestCase):
    def test_valid_successor_is_joined_before_viewpoint_terminal_stop(self):
        e,f,scan=near_vantage();old=e.goal;actual=set(e.actual_visited)
        result=exploration_step(e,f,scan,new_scan=True)
        self.assertEqual(result.get('continuation_state'),'joined',result)
        self.assertNotEqual(e.goal,old);self.assertTrue(any(result['axes']),result)
        self.assertEqual(e.actual_visited,actual,'retiring a viewpoint must not fabricate actual travel')

    def test_cached_scan_cannot_select_a_new_exploration_goal(self):
        e,f,scan=near_vantage();old=e.goal
        with patch.object(e,'plan',side_effect=AssertionError('cached frame cannot plan')):
            result=exploration_step(e,f,scan,new_scan=False)
        self.assertEqual(e.goal,old);self.assertNotEqual(result.get('continuation_state'),'joined')

    def test_failure_after_new_path_install_restores_full_follower_and_explorer(self):
        e,f,scan=near_vantage();old=e.goal;old_path=list(e.path)
        expected=copy.deepcopy(f);expected.command(scan,allowed_pose=lambda x,y,a:True)
        original=f.command;calls=[]
        def command(*args,**kwargs):
            calls.append(1)
            if len(calls)==2:raise ValueError('new path validation failed')
            return original(*args,**kwargs)
        with patch.object(f,'command',side_effect=command):result=exploration_step(e,f,scan,new_scan=True)
        self.assertEqual(len(calls),2);self.assertEqual(result['continuation_state'],'no_valid_successor')
        self.assertEqual(e.goal,old);self.assertEqual(e.path,old_path);self.assertNotIn(old,e.visited)
        for key in ['path','path_arc','previous','gain','progress_pose']:np.testing.assert_array_equal(getattr(f,key),getattr(expected,key))
        for key in ['index','plan_key','goal_braking','terminal','last_handoff_pose']:self.assertEqual(getattr(f,key),getattr(expected,key))

    def test_no_successor_keeps_braking_and_default_point_goal_semantics(self):
        floor=[list(p) for p in disk(3.)]
        for terminal in [True,False]:
            f=TrajectoryFollower();f.set_path([[0,0],[1,0]],terminal=terminal)
            for x in [.2,.4,.6,.8,.85]:out=f.command(dict(pose=[x,0,0],floor=floor,obstacles=[],_velocity_body=[.7,0],age=.3))
            self.assertEqual(out['state'],'braking');self.assertEqual(out['axes'],[0,0,0])
        self.assertTrue(TrajectoryFollower().terminal)

    def test_commanded_forward_without_displacement_escapes_sideways_in_closed_loop(self):
        result=stuck_loop()
        self.assertEqual(result['reason'],'escaped_to_new_space',result)
        self.assertTrue(result['requests']);self.assertLess(result['requests'][0],6.)
        self.assertGreater(abs(result['pose'][1]),.3)
        self.assertTrue(any(abs(row['axes'][1])>.1 for row in result['trace']))

    def test_repeated_actuator_nonresponse_tries_different_directions_then_stops(self):
        result=stuck_loop(all_axes_blocked=True)
        self.assertEqual(result['reason'],'bounded_recovery_exhausted',result)
        self.assertEqual(result['total_attempts'],3);self.assertLess(result['simulated_s'],18.)
        directions=[p['direction'] for p in result['outcomes']]
        self.assertEqual(len(directions),len(set(directions)))
        self.assertFalse(any(p['progressed'] for p in result['outcomes']))

    def test_missing_feedback_never_escapes_and_reaches_finite_stop(self):
        result=stuck_loop(feedback=False)
        self.assertEqual(result['reason'],'finite_no_progress_stop',result)
        self.assertEqual(result['total_attempts'],0)
        self.assertTrue(all(not any(row['axes']) for row in result['trace']))
        self.assertLessEqual(result['simulated_s'],25.4)

    def test_recovery_without_goal_waits_for_new_stopped_feedback(self):
        e=Explorer(4);e.ingest(open_scan(0,1,0));self.assertIsNone(e.goal)
        self.assertTrue(e.request_recovery((.35,0,0)))
        self.assertEqual(e.command_axes(),(0.,0.,0.))
        e.ingest(open_scan(0,2,0));axes=e.command_axes()
        self.assertTrue(any(axes));self.assertEqual(axes[0],0.)

    def test_expired_observation_cannot_start_a_stuck_escape(self):
        e=Explorer(4);s=open_scan(0,1,0);s['_valid_until']=time.monotonic()-.01;e.ingest(s)
        self.assertFalse(e.request_recovery((.35,0,0)));self.assertEqual(e.recovery_total_attempts,0)

    def test_goal_changes_do_not_erase_sustained_no_motion_window(self):
        p=MotionProgress()
        for k in range(22):
            now=k*.2;p.observe_command([.35,0,0],now);p.update([0,0,0],[1 if k%2 else -1,0],k,now,moving=False)
        self.assertIsNotNone(p.stuck_reason(4.2));p.recovery_started()
        self.assertIsNone(p.stuck_reason(4.2));self.assertEqual(p.moved_at,0.)

    def test_translation_failures_follow_world_direction_after_90_and_180_degree_turns(self):
        e=Explorer(4);e.ingest(open_scan(0,1,0));e._begin_recovery_episode()
        e._remember_recovery_direction(.35,0,0);e.recovery_attempts=2
        for stamp,yaw in [(2,math.pi/2),(3,math.pi),(4,math.pi/2),(5,0.)]:
            s=open_scan(0,stamp,0);s['pose'][2]=yaw;e.ingest(s)
            self.assertEqual(e.recovery_attempts,2,'turning is not a new recovery budget')
            self.assertEqual(e._recovery_was_tried(.35,0,0),yaw==0.)
            # The old world-east direction stays excluded even when its body
            # name becomes right or reverse; a new world-north/west can be tried.
            f=.35*math.cos(yaw);side=-.35*math.sin(yaw)
            self.assertTrue(e._recovery_was_tried(f,side,0))
        e.recovery_attempts=3
        s=open_scan(0,6,0);s['pose'][2]=math.pi/2;e.ingest(s)
        self.assertEqual(e._escape_candidates(),[])

    def test_failed_command_can_carry_its_original_heading(self):
        e=Explorer(4);s=open_scan(0,1,0);s['pose'][2]=math.pi/2;e.ingest(s)
        self.assertTrue(e.request_recovery((.35,0,0),failed_yaw=0.))
        self.assertFalse(e._recovery_was_tried(.35,0,0))
        self.assertTrue(e._recovery_was_tried(0,-.25,0))

    def test_external_turn_ends_inflight_translation_without_refunding_attempt(self):
        e=Explorer(4);e.ingest(open_scan(0,1,0));e._recover_axes();self.assertIsNotNone(e.recovery)
        attempts=e.recovery_attempts
        s=open_scan(0,2,0);s['pose'][2]=math.pi/2;e.ingest(s)
        self.assertEqual(e._recover_axes(),(0.,0.,0.));self.assertIsNone(e.recovery)
        self.assertEqual(e.recovery_attempts,attempts);self.assertIn('朝向已变化',e.motion_hint)


if __name__=='__main__':unittest.main()

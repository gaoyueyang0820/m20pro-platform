"""Recovery candidates are not action attempts until a successful drive issue."""
import unittest
from unittest.mock import patch
from autonomy_core import Explorer,cell
from local_trajectory import TrajectoryFollower,exploration_step
from test_autonomy_recovery import scan


def blocked_scene(stamp=1.,velocity=(0.,0.),pose=(0.,0.,0.),obstacles=((3,0),)):
    return dict(scan(pose,obstacles=obstacles,stamp=stamp),_velocity_body=list(velocity),_speed_mps=(velocity[0]**2+velocity[1]**2)**.5)


class RecoveryIssuanceTests(unittest.TestCase):
    def setup_candidate(self,velocity=(0.,0.)):
        e=Explorer(3);e.ingest(blocked_scene(velocity=velocity));e.path=[(2,0),(5,0)];e.goal=(5,0);e.plan_state='planned'
        return e,e.command_axes()

    def test_candidate_wait_does_not_spend_attempt_and_actual_issue_starts_action_clock(self):
        e,axes=self.setup_candidate((.5,0.));self.assertEqual(axes,(0,0,0))
        self.assertEqual(e.recovery_attempts,0);self.assertFalse(e.recovery['issued'])
        e.ingest(blocked_scene(stamp=1.4));axes=e.command_axes();self.assertTrue(any(axes))
        self.assertEqual(e.recovery_attempts,0)
        self.assertTrue(e.note_recovery_command(axes,now=100.))
        self.assertEqual(e.recovery_attempts,1);self.assertEqual(e.recovery['stamp'],1.4)
        self.assertEqual(e.recovery['deadline_monotonic'],103.)
        self.assertFalse(e.note_recovery_command(axes,now=100.1));self.assertEqual(e.recovery_attempts,1)

    def test_geometry_interrupt_before_issue_records_unissued_result_without_spending_action(self):
        e,axes=self.setup_candidate();self.assertTrue(any(axes));self.assertEqual(e.recovery_attempts,0)
        e.ingest(blocked_scene(stamp=1.2,obstacles=((3,0),(0,3))))
        self.assertEqual(e.command_axes(),(0,0,0));row=e.recovery_outcomes[-1]
        self.assertFalse(row['issued']);self.assertEqual(row['reason'],'path_blocked')
        self.assertEqual(e.recovery_attempts,0);self.assertEqual(e.recovery_candidate_failures,1)

    def test_issued_motion_interruption_records_measured_result_and_keeps_budget_spent(self):
        e,axes=self.setup_candidate();e.note_recovery_command(axes)
        e.ingest(blocked_scene(stamp=1.2,pose=(0,.1,0),velocity=(0,.25),obstacles=((3,0),(0,3))))
        self.assertEqual(e.command_axes(),(0,0,0));row=e.recovery_outcomes[-1]
        self.assertTrue(row['issued']);self.assertEqual(row['reason'],'path_blocked')
        self.assertAlmostEqual(row['translation'],.1);self.assertEqual(e.recovery_attempts,1)
        self.assertEqual(e.path,[])

    def test_unissued_waits_are_separately_bounded_without_refunding_sent_actions(self):
        e=Explorer(3)
        for i in range(3):
            stamp=1+i*3.;e.ingest(blocked_scene(stamp=stamp,velocity=(.5,0.)));e.plan_state='recovering'
            self.assertEqual(e.command_axes(),(0,0,0))
            e.ingest(blocked_scene(stamp=stamp+2.1,velocity=(.5,0.)))
            self.assertEqual(e.command_axes(),(0,0,0))
        self.assertEqual(e.recovery_attempts,0);self.assertEqual(e.recovery_candidate_failures,3)
        e.plan_state='recovering';self.assertEqual(e.command_axes(),(0,0,0))
        self.assertEqual(e.recovery_block_reason,'candidates_not_issued')

    def test_disconnected_initial_path_stops_replans_and_resumes_on_new_frame(self):
        e=Explorer(3);data=dict(scan(stamp=1),_speed_mps=0.,_velocity_body=[0.,0.]);e.ingest(data)
        e.path=[(8,0),(9,0)];e.goal=(9,0);e.plan_state='planned';f=TrajectoryFollower()
        result=exploration_step(e,f,data,new_scan=True)
        self.assertEqual(result['path_repair_state'],'replanned');self.assertEqual(result['axes'],[0,0,0])
        self.assertLessEqual(((e.path[0][0]*.2)**2+(e.path[0][1]*.2)**2)**.5,.35)
        with patch.object(e,'plan',side_effect=AssertionError('same source frame cannot replan')):
            repeated=exploration_step(e,f,data,new_scan=True)
        self.assertEqual(repeated['axes'],[0,0,0]);self.assertEqual(f.path_repair_attempts,1)
        newer=dict(data,stamp=2);e.ingest(newer);result=exploration_step(e,f,newer,new_scan=True)
        self.assertTrue(any(result['axes']),result)

    def test_irreparable_path_has_three_attempt_cap_without_internal_exception(self):
        e=Explorer(3);f=TrajectoryFollower();calls=[]
        def disconnected():calls.append(1);e.path=[(30,30),(31,30)];return 'planned'
        with patch.object(e,'plan',side_effect=disconnected):
            for stamp in range(1,6):
                data=dict(scan(stamp=stamp),_speed_mps=0.,_velocity_body=[0.,0.]);e.ingest(data)
                e.path=[(8,0),(9,0)];e.goal=(9,0)
                result=exploration_step(e,f,data,new_scan=True);self.assertEqual(result['axes'],[0,0,0])
        self.assertEqual(len(calls),3);self.assertEqual(result['path_repair_state'],'exhausted')

    def test_consumed_prefix_keeps_existing_continuous_path_instead_of_false_disconnect(self):
        e=Explorer(3);e.ingest(dict(scan(stamp=1),_speed_mps=.2,_velocity_body=[.2,0.]))
        full=[(i,0) for i in range(11)];e.path=full;e.goal=full[-1];f=TrajectoryFollower()
        f.set_path([(x*.2,y*.2) for x,y in full],terminal=False);f.plan_key=tuple(full)
        for i,x in enumerate([.2,.4,.6]):
            data=dict(scan((x,0,0),stamp=2+i),_speed_mps=.2,_velocity_body=[.2,0.]);e.ingest(data);f.command(data)
        e.path=full[5:]
        result=exploration_step(e,f,data,new_scan=True)
        self.assertNotIn('path_repair_state',result);self.assertEqual(f.path_repair_attempts,0)
        self.assertTrue(any(result['axes']))

    def test_lifecycle_reset_preserves_action_history_and_turning_does_not_refund(self):
        e,axes=self.setup_candidate();e.note_recovery_command(axes);e._finish_recovery('test_stop')
        total=e.recovery_total_attempts;history=list(e.recovery_outcomes)
        e.reset_recovery_context(.1)
        self.assertEqual(e.recovery_attempts,1);self.assertEqual(e.recovery_total_attempts,total)
        self.assertEqual(e.recovery_outcomes,history)
        e.reset_recovery_context(.65,'fresh same-capture manual displacement')
        self.assertEqual(e.recovery_attempts,0);self.assertEqual(e.recovery_total_attempts,total)
        self.assertEqual(e.recovery_outcomes,history)


if __name__=='__main__':unittest.main()

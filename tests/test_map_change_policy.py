"""Synthetic change scenarios, not sensor/tunnel field acceptance."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.map_change_policy import ChangePolicy,classify_cell,gate_alignment


def evidence(vote,count,start=0,dynamic=False,stationary=False):
    return [dict(stamp=start+i*10,vote=vote,origin=[0 if stationary else i*.4,0],dynamic=dynamic)
            for i in range(count)]


def aligned(**changes):
    return dict(dict(structural_overlap=.96,rmse_m=.05,effective_frames=3,
                     correction_m=.06,correction_yaw_rad=.01,
                     known_free_conflict_ratio=.01),**changes)


class ChangeEvidenceTests(unittest.TestCase):
    def test_person_crossing_uses_realtime_layer_without_changing_long_term(self):
        observations=evidence(1,1)
        current=classify_cell(254,observations,registration_ok=True)
        self.assertTrue(current['realtime_occupied']);self.assertEqual(current['action'],'keep')
        self.assertEqual(current['classification'],'transient_obstacle')
        observations+=evidence(-1,1,start=10)
        left=classify_cell(254,observations,registration_ok=True)
        self.assertFalse(left['realtime_occupied']);self.assertEqual(left['action'],'keep')

    def test_fixed_new_object_requires_time_and_observer_baseline(self):
        rows=evidence(1,3)
        self.assertEqual(classify_cell(254,rows[:2],registration_ok=True)['action'],'keep')
        result=classify_cell(254,rows,registration_ok=True)
        self.assertEqual(result['action'],'occupy');self.assertEqual(result['classification'],'fixed_obstacle_added')
        self.assertGreaterEqual(result['evidence']['views'],2)

    def test_more_frames_at_same_place_do_not_create_independent_evidence(self):
        rows=[dict(stamp=i*.1,vote=1,origin=[i*.01,0]) for i in range(95)]
        result=classify_cell(254,rows,registration_ok=True)
        self.assertEqual(result['evidence']['epochs'],1);self.assertEqual(result['action'],'keep')
        stationary=classify_cell(254,evidence(1,12,stationary=True),registration_ok=True)
        self.assertEqual(stationary['evidence']['views'],1);self.assertEqual(stationary['action'],'keep')

    def test_known_dynamic_object_never_becomes_wall_during_its_event(self):
        rows=evidence(1,8);rows[0]['dynamic']=True
        result=classify_cell(254,rows,registration_ok=True)
        self.assertEqual(result['classification'],'dynamic_obstacle');self.assertEqual(result['action'],'keep')
        self.assertTrue(result['realtime_occupied'])

    def test_removed_object_can_be_cleared_despite_older_occupied_observations(self):
        rows=evidence(1,3)+evidence(-1,5,start=30)
        early=classify_cell(0,rows[:-1],registration_ok=True)
        self.assertEqual(early['action'],'keep')
        result=classify_cell(0,rows,registration_ok=True)
        self.assertEqual(result['action'],'clear');self.assertEqual(result['classification'],'obstacle_removed')
        self.assertEqual(result['evidence']['consecutive_epochs'],5)
        self.assertEqual(result['evidence']['span_s'],40)

    def test_reappearing_object_resets_removal_confirmation(self):
        rows=evidence(-1,5)+evidence(1,1,start=50)+evidence(-1,4,start=60)
        self.assertEqual(classify_cell(0,rows,registration_ok=True)['action'],'keep')

    def test_no_return_and_expired_observation_never_clear_old_obstacle(self):
        self.assertEqual(classify_cell(0,[],registration_ok=True)['action'],'keep')
        result=classify_cell(0,evidence(-1,5),registration_ok=True,now=300)
        self.assertEqual(result['action'],'keep');self.assertIsNone(result['realtime_occupied'])
        # Presence of an old obstacle must not masquerade as current live sensing.
        result=classify_cell(0,evidence(1,1),registration_ok=True,now=5)
        self.assertIsNone(result['realtime_occupied']);self.assertEqual(result['action'],'keep')

    def test_single_mixed_epoch_cannot_publish_an_already_departed_obstacle(self):
        rows=evidence(1,3)+[dict(stamp=20.5,vote=-1,origin=[.8,0])]
        result=classify_cell(254,rows,registration_ok=True)
        self.assertFalse(result['realtime_occupied']);self.assertEqual(result['action'],'keep')

    def test_untrusted_alignment_blocks_persistent_updates_but_reports_live_obstacle(self):
        result=classify_cell(254,evidence(1,5),registration_ok=False)
        self.assertTrue(result['realtime_occupied']);self.assertEqual(result['action'],'keep')
        result=classify_cell(0,evidence(-1,5),registration_ok=False)
        self.assertEqual(result['action'],'keep')

    def test_unknown_area_needs_measured_views_before_marking_free(self):
        result=classify_cell(205,evidence(-1,3),registration_ok=True)
        self.assertEqual(result['action'],'clear');self.assertEqual(result['classification'],'explored_free_space')

    def test_local_wall_change_can_produce_reviewable_clear_and_add_candidates(self):
        gate=gate_alignment(aligned(known_free_conflict_ratio=.16,temporal_change_confirmed=True))
        self.assertTrue(gate['allowed']);self.assertTrue(gate['requires_review'])
        removed=classify_cell(dict(value=0,kind='wall'),evidence(-1,5),registration_ok=gate['allowed'])
        added=classify_cell(254,evidence(1,3),registration_ok=gate['allowed'])
        self.assertEqual(removed['classification'],'wall_removed_candidate')
        self.assertEqual(removed['action'],'clear');self.assertEqual(added['action'],'occupy')


class AlignmentPolicyTests(unittest.TestCase):
    def test_coherent_map_shift_is_not_a_collection_of_new_walls(self):
        decision=gate_alignment(aligned(coherent_shift_fraction=.8,coherent_shift_m=.25))
        self.assertFalse(decision['allowed']);self.assertEqual(decision['status'],'coherent_structure_shift')
        self.assertEqual(classify_cell(254,evidence(1,8),registration_ok=decision['allowed'])['action'],'keep')

    def test_large_pose_correction_and_known_bad_candidate_are_not_published(self):
        decision=gate_alignment(aligned(structural_overlap=.55,rmse_m=.14,correction_m=1.4775))
        self.assertFalse(decision['allowed']);self.assertEqual(decision['status'],'registration_correction_large')
        self.assertFalse(gate_alignment(aligned(correction_yaw_rad=math.radians(18)))['allowed'])

    def test_floor_overlap_cannot_substitute_for_structural_metrics(self):
        decision=gate_alignment(dict(overall_overlap=.99,rmse_m=.02,effective_frames=5))
        self.assertFalse(decision['allowed'])

    def test_new_obstacle_conflicts_need_temporal_evidence_not_one_good_alignment(self):
        decision=gate_alignment(aligned(known_free_conflict_ratio=.15))
        self.assertFalse(decision['allowed']);self.assertEqual(decision['status'],'unconfirmed_environment_change')
        decision=gate_alignment(aligned(known_free_conflict_ratio=.15,temporal_change_confirmed=True))
        self.assertTrue(decision['allowed']);self.assertTrue(decision['requires_review'])

    def test_tunnel_like_ambiguity_blocks_updates_even_with_high_overlap(self):
        result=gate_alignment(aligned(structural_overlap=.99,rmse_m=.02,ambiguous=True))
        self.assertFalse(result['allowed']);self.assertEqual(result['status'],'ambiguous_registration')

    def test_single_unchanged_structure_can_pass_but_not_single_frame_change(self):
        self.assertTrue(gate_alignment(aligned(effective_frames=1,known_free_conflict_ratio=0))['allowed'])
        self.assertFalse(gate_alignment(aligned(effective_frames=1,known_free_conflict_ratio=.02))['allowed'])

    def test_invalid_or_missing_geometry_is_not_accepted(self):
        for kwargs in [dict(structural_overlap=float('nan')),dict(rmse_m=-1),dict(effective_frames=0),dict(localized=False)]:
            self.assertFalse(gate_alignment(aligned(**kwargs))['allowed'])
        with self.assertRaises(ValueError):
            classify_cell(0,[dict(stamp=100,vote=-1,origin=[0,0])],registration_ok=True,now=0)


if __name__=='__main__':unittest.main()

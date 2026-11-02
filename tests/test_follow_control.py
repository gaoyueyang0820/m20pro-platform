"""Offline follow gate: small real sweeps, no inferred human depth or target swap."""
import copy,math,unittest
from follow_control import FollowControl,_Surface
from autonomy_core import clearance,PersonLock


def vision(sequence=1,age=.1):return dict(locked=True,fresh=True,age=age,sequence=sequence,target_id='selected-person')
def scan(floor=None,obstacles=None,bounds=None,stamp=1.,velocity=(0.,0.)):
 floor=[(x,y) for x in range(-20,21) for y in range(-20,21)] if floor is None else list(floor)
 obstacles=[] if obstacles is None else obstacles
 return dict(protocol=1,frame_id='base_link',resolution=.2,floor=floor,measured_floor=list(floor),obstacles=obstacles,
             obstacle_bounds=[] if bounds is None else bounds,geometry_version=2,stamp=stamp,age=.1,_velocity_body=list(velocity))


class FollowControlTests(unittest.TestCase):
 def test_small_yaw_needs_its_short_sweep_not_ninety_degrees(self):
  s=scan([(x,y) for x in range(-20,21) for y in [-1,0,1]])
  self.assertFalse(clearance(s,0.,-.029)[0])
  out=FollowControl().choose(s,(0.,-.029),vision=vision(),now=100.)
  self.assertEqual(out['state'],'tracking',out);self.assertEqual(out['axes'],[0.,0.,-.029])
  self.assertLess(out['candidate_diagnostics'][0]['max_angle_deg'],5.)

 def test_forward_small_arc_is_checked_continuously_in_supported_strip(self):
  s=scan([(x,y) for x in range(-20,21) for y in [-1,0,1]])
  out=FollowControl().choose(s,(.12,.029),vision=vision(),now=100.)
  self.assertEqual(out['state'],'tracking',out);self.assertEqual(out['axes'],[.12,0.,.029])
  self.assertTrue(out['trajectory_verified']);self.assertTrue(out['trajectory'])

 def test_unobserved_forward_ground_waits_without_reversing_or_growing_floor(self):
  s=scan([(x,y) for x in range(-2,3) for y in [-1,0,1]]);original=copy.deepcopy(s)
  out=FollowControl().choose(s,(.12,0.),vision=vision(),now=100.)
  self.assertEqual(out['state'],'waiting_ground');self.assertEqual(out['axes'],[0.,0.,0.])
  self.assertEqual(out['reason_code'],'unobserved_support');self.assertGreater(out['missing']['count'],0)
  self.assertIn('front',out['missing']['directions']);self.assertEqual(s,original)
  self.assertTrue(all(p['axes'][0]>=0 and p['axes'][1]==0 for p in out['candidate_diagnostics']))

 def test_occluded_forward_support_can_align_but_is_not_labelled_confirmed_person(self):
  s=scan([(x,y) for x in range(-2,3) for y in [-1,0,1]],[(3,0)],[[.55,-.35,.65,.35]])
  out=FollowControl().choose(s,(.12,.06),vision=vision(),now=100.)
  self.assertEqual(out['state'],'tracking',out);self.assertEqual(out['mode'],'aligning')
  self.assertEqual(out['axes'][0],0.);self.assertGreater(out['axes'][2],0.)
  self.assertTrue(out['missing']['occlusion_possible_cells']);self.assertEqual(out['missing']['person_association'],'unverified')
  self.assertIsNone(out['distance_m'])

 def test_actual_obstacle_in_body_sweep_keeps_zero(self):
  s=scan(obstacles=[(2,0)],bounds=[[.4,-.1,.44,.1]])
  out=FollowControl().choose(s,(.12,.05),vision=vision(),now=100.)
  self.assertEqual(out['axes'],[0.,0.,0.]);self.assertGreater(out['obstacles']['count'],0)
  self.assertEqual(out['reason_code'],'obstacle_in_sweep')

 def test_same_vision_does_not_renew_its_expiry_but_new_scan_can_recheck(self):
  f=FollowControl();first=f.choose(scan(),(.12,0.),vision=vision(),now=100.)
  second=f.choose(scan(stamp=2),(.12,0.),vision=vision(),now=100.3)
  self.assertEqual(second['state'],'tracking');self.assertLessEqual(second['valid_until'],100.55)
  expired=f.choose(scan(stamp=3),(.12,0.),vision=vision(),now=100.6)
  self.assertEqual(expired['state'],'stale');self.assertEqual(expired['axes'],[0,0,0])
  new=f.choose(scan(stamp=4),(.12,0.),vision=vision(2),now=100.6)
  self.assertEqual(new['state'],'tracking')

 def test_same_scan_cannot_be_refreshed_by_new_visual_frame(self):
  f=FollowControl();s=scan();f.choose(s,(.12,0),vision=vision(),now=100.)
  out=f.choose(s,(.12,0),vision=vision(2),now=100.6)
  self.assertEqual(out['axes'],[0,0,0]);self.assertEqual(out['state'],'stale')

 def test_lock_loss_and_target_change_never_reacquire_another_person(self):
  f=FollowControl();f.choose(scan(),(.12,0),vision=vision(),now=100.)
  lost=f.choose(scan(stamp=2),(.12,0),vision=dict(vision(2),locked=False),now=100.1)
  self.assertTrue(lost['reselection_required']);self.assertEqual(lost['axes'],[0,0,0])
  changed=f.choose(scan(stamp=3),(.12,0),vision=dict(vision(3),target_id='other'),now=100.2)
  self.assertTrue(changed['reselection_required']);self.assertEqual(changed['axes'],[0,0,0])
  # Person association remains the existing lock, not this gate.
  lock=PersonLock([.4,.2,.6,.7])
  with self.assertRaises(ValueError):lock.update([dict(box=[.1,.2,.3,.7],confidence=.9)])

 def test_missing_feedback_does_not_generate_motion(self):
  s=scan();s['_velocity_body']=None
  out=FollowControl().choose(s,(.12,.02),vision=vision(),now=100.)
  self.assertEqual(out['state'],'waiting_feedback');self.assertEqual(out['axes'],[0,0,0])

 def test_zero_request_models_a_single_braking_deceleration(self):
  import numpy as np
  s=scan(velocity=(.3,0));control=FollowControl()
  result=control._rollout(_Surface(s),[0.,0.,0.],np.array([.3,0.,0.]),.1,0.)
  self.assertTrue(result['clear'])
  # 0.3 * (0.1 + 0.08) reaction + 0.3^2/(2*0.5) stopping,
  # with bounded discretization. Applying deceleration twice would be unsafe.
  self.assertGreater(result['max_translation_m'],.13)
  self.assertLess(result['max_translation_m'],.17)

 def test_existing_momentum_and_missing_stopping_support_requests_zero(self):
  s=scan([(x,y) for x in range(-2,3) for y in [-1,0,1]],velocity=(.3,0))
  out=FollowControl().choose(s,(.12,.03),vision=vision(),now=100.)
  self.assertEqual(out['axes'],[0,0,0]);self.assertEqual(out['state'],'braking')
  self.assertTrue(out['missing']['cells'])

if __name__=='__main__':unittest.main()

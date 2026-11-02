"""Offline bounded escape + global route continuation; no motion/network I/O."""
import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from navigation_planner import GridPlanner
from point_navigation import simulated_scan
from robot_geometry import BODY_HALF_LENGTH,BODY_HALF_WIDTH
from start_pose_recovery import recover_start,check_recovery_step
from test_navigation_planning_v3 import validate_sampled_body


def planner(a):
 h,w=a.shape;return GridPlanner(('P5\n%d %d\n255\n'%(w,h)).encode()+a.tobytes(),.1,[0,0,0])
def room(w=100,h=80):
 a=np.full((h,w),254,np.uint8);a[:2]=0;a[-2:]=0;a[:,:2]=0;a[:,-2:]=0;return a

def observe(truth,pose,stamp=100.):
 scan=simulated_scan(truth,pose,stamp=stamp)
 scan.update(protocol=1,frame_id='base_link',pose_frame_id='map',pose_stamp=stamp,age=.05,resolution=.2,demo=False)
 loc=dict(normal=True,confidence=.95,pose=list(pose),age=.05)
 return scan,loc

def body_oracle(p,path):
 # Dense points over the actual body, independent of the production SAT test.
 xx,yy=np.meshgrid(np.linspace(-BODY_HALF_LENGTH,BODY_HALF_LENGTH,43),np.linspace(-BODY_HALF_WIDTH,BODY_HALF_WIDTH,25))
 for q in path:
  c,s=math.cos(q['yaw']),math.sin(q['yaw']);x=q['x']+c*xx-s*yy;y=q['y']+s*xx+c*yy
  ix=np.floor(x/.1).astype(int);iy=p.h-1-np.floor(y/.1).astype(int)
  if not ((ix>=0)&(ix<p.w)&(iy>=0)&(iy<p.h)).all() or not p.free[iy,ix].all():return False
 return True

class StartRecoveryTests(unittest.TestCase):
 def run_recovery(self,p,start,goal,yaw=0.,budget=1.2):
  scan,loc=observe(p,start)
  return recover_start(p,start,goal,yaw,scan,loc,standing=True,budget=budget)
 def assert_prefix_closed_loop(self,p,start,r):
  self.assertTrue(r['ok'],r)
  path=r['recovery_dense_path'];self.assertEqual([path[0][k] for k in ['x','y','yaw']],start)
  self.assertTrue(body_oracle(p,path))
  for i,(a,b) in enumerate(zip(path,path[1:])):
   current=[a[k] for k in ['x','y','yaw']];next_pose=[b[k] for k in ['x','y','yaw']]
   scan,loc=observe(p,current,100+i*.1)
   check=check_recovery_step(p,current,next_pose,scan,loc,start,standing=True,support_seed=r.get('support_seed'))
   self.assertTrue(check['ok'],check)
  self.assertTrue(body_oracle(p,r['global_plan']['dense_path']))
  self.assertLess(math.dist([r['global_plan']['goal'][k] for k in ['x','y']],r['requested_goal']),1e-8)

 def test_near_wall_can_back_out_of_margin_then_resume_goal(self):
  a=room();a[:,40:42]=0;p=planner(a);start=[3.56,4.,0.];goal=[2.05,4.]
  self.assertFalse(p.pose_fits(*start));r=self.run_recovery(p,start,goal)
  self.assertEqual(r['classification'],'clearance_only');self.assert_prefix_closed_loop(p,start,r)
  self.assertTrue(any(q['kind']=='backward' for q in r['recovery_dense_path']))
  self.assertLessEqual(r['recovery_distance_m'],.2)
  clear=[q['clearance_lower_bound_m'] for q in r['recovery_dense_path']]
  self.assertTrue(all(b>=a-.001 for a,b in zip(clear,clear[1:])))

 def test_narrow_door_can_go_straight_without_an_in_place_turn(self):
  a=np.zeros((80,100),np.uint8);a[36:44,2:98]=254;p=planner(a);start=[2.05,4.05,0.]
  self.assertFalse(p.rotation_fits(*p.cell(*start[:2]),0,math.pi/2))
  r=self.run_recovery(p,start,[7.05,4.05]);self.assertTrue(r['ok'],r)
  self.assertEqual(r['recovery_distance_m'],0);self.assertTrue(validate_sampled_body(p,r['global_plan']))

 def test_fully_pinched_pose_refuses_invented_escape(self):
  a=np.zeros((80,100),np.uint8);a[38:43,35:44]=254;p=planner(a);start=[3.95,3.95,0.]
  self.assertTrue(p._rectangle_free(*start,BODY_HALF_LENGTH,BODY_HALF_WIDTH))
  r=self.run_recovery(p,start,[3.95,3.95]);self.assertFalse(r['ok']);self.assertIn(r['classification'],['clearance_only','live_body_collision'])
  self.assertEqual(r['recovery_dense_path'],[]);self.assertFalse(r['motion_sent'])

 def test_recovery_and_global_continuation_do_not_cross_the_wall(self):
  a=room();a[15:70,49:51]=0;p=planner(a);start=[4.46,3.05,0.];goal=[8.05,3.05]
  r=self.run_recovery(p,start,goal,budget=2.)
  self.assert_prefix_closed_loop(p,start,r);self.assertGreater(r['global_plan']['length_m'],5.)
  self.assertTrue(any(q['y']<1 or q['y']>6.5 for q in r['global_plan']['path']))

 def test_turn_quantization_can_be_escaped_by_forward_prefix(self):
  h=w=120;yy,xx=np.mgrid[:h,:w];wx=(xx+.5)*.1;wy=(h-yy-.5)*.1
  start=[2.,3.,.29];c,s=math.cos(start[2]),math.sin(start[2]);dx,dy=wx-start[0],wy-start[1]
  u,v=c*dx+s*dy,-s*dx+c*dy
  free=((u>-.8)&(u<.8)&(abs(v)<.36))|((u>.45)&(u<4.5)&(abs(v)<1.7))
  p=planner(np.where(free,254,0).astype(np.uint8));goal=[2+c*3-s,3+s*3+c]
  r=self.run_recovery(p,start,goal,yaw=0.,budget=2.)
  self.assertTrue(r['ok'],r);self.assertEqual(r['classification'],'start_quantization')
  self.assertTrue(any(q['kind']=='forward' for q in r['recovery_dense_path']));self.assert_prefix_closed_loop(p,start,r)

 def test_actual_live_body_collision_is_not_solved_by_shrinking_body(self):
  p=planner(room());start=[2,3,0];scan,loc=observe(p,start)
  scan['obstacles']=[[1,0]];scan['obstacle_bounds']=[[.19,-.02,.21,.02]]
  r=recover_start(p,start,[4,3],0,scan,loc,standing=True)
  self.assertFalse(r['ok']);self.assertEqual(r['classification'],'live_body_collision');self.assertEqual(r['recovery_dense_path'],[])

 def test_map_wall_disagreement_is_not_cleared_from_a_single_scan(self):
  a=room();a[39:41,19:21]=0;p=planner(a);truth=planner(room());start=[2,4,0];scan,loc=observe(truth,start)
  before=p.free.copy();r=recover_start(p,start,[4,4],0,scan,loc,standing=True)
  self.assertFalse(r['ok']);self.assertEqual(r['classification'],'map_surface_disagreement');np.testing.assert_array_equal(p.free,before)

 def test_missing_rear_ground_does_not_become_a_retreat_route(self):
  a=room();a[:,40:42]=0;p=planner(a);start=[3.56,4.,0.];scan,loc=observe(p,start)
  for key in ['floor','measured_floor']:scan[key]=[q for q in scan[key] if q[0]>=0]
  # Explicit step beyond the standing patch has no measured rear floor.
  check=check_recovery_step(p,start,[3.51,4,0],scan,loc,start,standing=True)
  # A short first step may remain entirely within the confirmed stance patch;
  # a longer plan cannot invent rear support after that patch ends.
  r=recover_start(p,start,[2.05,4],0,scan,loc,standing=True,max_distance=.8)
  if r['ok']:
   for q in r['recovery_dense_path']:self.assertGreaterEqual(q['x'],3.41)
  new_scan,new_loc=observe(p,[3.3,4,0])
  for key in ['floor','measured_floor']:new_scan[key]=[q for q in new_scan[key] if q[0]>=0]
  missing=check_recovery_step(p,[3.3,4,0],[3.25,4,0],new_scan,new_loc,start,standing=True)
  self.assertFalse(missing['ok']);self.assertIn('地面',missing['reason'])

 def test_revalidation_carries_only_previously_measured_ground_in_body_blind_spot(self):
  p=planner(room());start=[3.,3.,0.];scan,loc=observe(p,start)
  r=recover_start(p,start,[5,3],0,scan,loc,standing=True);self.assertTrue(r['ok'],r)
  current=[3.3,3,0];fresh,where=observe(p,current,101.)
  for key in ['floor','measured_floor']:fresh[key]=[q for q in fresh[key] if math.hypot(*q)*.2>.55]
  without=check_recovery_step(p,current,[3.325,3,0],fresh,where,start,standing=True)
  self.assertFalse(without['ok'])
  with_seed=check_recovery_step(p,current,[3.325,3,0],fresh,where,start,standing=True,support_seed=r['support_seed'])
  self.assertTrue(with_seed['ok'],with_seed)
  empty=dict(r['support_seed'],floor=[])
  self.assertFalse(check_recovery_step(p,current,[3.325,3,0],fresh,where,start,standing=True,support_seed=empty)['ok'])

 def test_step_revalidation_stops_if_new_obstacle_enters_escape(self):
  a=room();a[:,40:42]=0;p=planner(a);start=[3.56,4,0];r=self.run_recovery(p,start,[2.05,4]);self.assertTrue(r['ok'],r)
  scan,loc=observe(p,start);scan['obstacles'].append([-2,0]);scan['obstacle_bounds'].append([-.49,-.05,-.43,.05])
  next_pose=[r['recovery_dense_path'][1][k] for k in ['x','y','yaw']]
  self.assertFalse(check_recovery_step(p,start,next_pose,scan,loc,start,standing=True)['ok'])

if __name__=='__main__':unittest.main()

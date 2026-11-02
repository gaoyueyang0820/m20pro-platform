import math,sys,unittest
from pathlib import Path
import numpy as np
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation_planner import GridPlanner

class PlanningTests(unittest.TestCase):
 def plan(self,image,start,goal,yaw=None):
  h,w=image.shape;raw=('P5\n%d %d\n255\n'%(w,h)).encode()+image.tobytes()
  return GridPlanner(raw,.1,[0,0,0]).plan(start,goal,yaw,budget=5)
 def room(self):
  a=np.full((80,100),254,np.uint8);a[:2]=0;a[-2:]=0;a[:,:2]=0;a[:,-2:]=0;return a
 def test_wall_detour_and_guides_keep_native_goal_semantics(self):
  a=self.room();a[15:70,49:51]=0
  r=self.plan(a,[2,3,0],[8,3]);self.assertGreater(r['length_m'],6)
  self.assertTrue(any(p['y']>6.5 or p['y']<1. for p in r['path']));self.assertGreater(len(r['points']),1)
  self.assertTrue(all(p['type']==0 for p in r['points'][:-1]));self.assertEqual(r['points'][-1]['type'],1)
 def test_unknown_or_disconnected_goal_is_not_silently_snapped(self):
  a=self.room();a[:,49:51]=0
  with self.assertRaisesRegex(ValueError,'没有足够宽'):self.plan(a,[2,3,0],[8,3])
  a=self.room();a[49,80]=205
  with self.assertRaisesRegex(ValueError,'未知'):self.plan(a,[2,3,0],[8,3])
 def test_straight_corridor_allows_body_width_but_rejects_too_narrow(self):
  a=np.zeros((50,100),np.uint8);a[20:30,2:98]=254
  r=self.plan(a,[2,2.5,0],[8,2.5]);self.assertLess(r['length_m'],6.2)
  a[:]=0;a[23:27,2:98]=254
  with self.assertRaises(ValueError):self.plan(a,[2,2.5,0],[8,2.5])
 def test_small_initial_heading_error_does_not_add_fixed_five_cm_margin(self):
  a=np.zeros((50,100),np.uint8);a[21:28,2:98]=254
  r=self.plan(a,[2.05,2.55,.03],[8.05,2.55]);self.assertLess(r['length_m'],6.2)
  # Exact continuous poses can fit the 0.6 m corridor that old centre snapping rejected.
  a[:]=0;a[23:28,2:98]=254
  with self.assertRaises(ValueError):self.plan(a,[2.05,2.5,.03],[8.05,2.5])
 def test_timeout_in_footprint_expansion_does_not_poison_next_preview(self):
  a=self.room();h,w=a.shape;raw=('P5\n%d %d\n255\n'%(w,h)).encode()+a.tobytes()
  p=GridPlanner(raw,.1,[0,0,0]);clock=[0.]
  def now():clock[0]+=.05;return clock[0]
  with patch('navigation_planner.time.monotonic',side_effect=now):
   with self.assertRaisesRegex(ValueError,'搜索超时'):p.plan([2,3,0],[8,3],budget=.1)
  self.assertIsNone(p._deadline)
  self.assertTrue(p.plan([2,3,0],[8,3])['ok'])
 def test_terminal_heading_is_automatic_or_explicit(self):
  a=self.room();r=self.plan(a,[3,2,math.pi/2],[3,6]);self.assertAlmostEqual(r['goal']['yaw'],math.pi/2)
  r=self.plan(a,[3,2,math.pi/2],[3,6],0);self.assertAlmostEqual(r['goal']['yaw'],0)
 def test_live_obstacles_only_add_blocks_and_require_fresh_aligned_pose(self):
  import base64
  from navigation_planner import live_obstacles
  axis=np.arange(-3,3,.1);xx,yy=np.meshgrid(axis,axis);floor=np.column_stack([xx.ravel(),yy.ravel(),np.zeros(xx.size)])
  pts=np.vstack([floor,[[1,0,.5],[1,.1,.5]]]).astype('<f4')
  packet=dict(protocol=1,frame_id='map',age=.1,quality=[.02,.98],pose=[0,0,.55,0,0,0,1],points=base64.b64encode(pts.tobytes()).decode())
  obstacles=live_obstacles(packet,[0,0,0],.1);self.assertTrue(any(abs(x-1)<.21 and abs(y)<.21 for x,y in obstacles))
  with self.assertRaises(ValueError):live_obstacles(packet,[2,0,0])
  with self.assertRaises(ValueError):live_obstacles(dict(packet,age=2),[0,0,0])
 def test_live_self_projection_is_ignored_only_inside_current_body(self):
  a=self.room();h,w=a.shape;raw=('P5\n%d %d\n255\n'%(w,h)).encode()+a.tobytes()
  start=[2,3,0]
  p=GridPlanner(raw,.1,[0,0,0],obstacles=[[2,3]],robot_pose=start)
  r=p.plan(start,[8,3]);self.assertEqual(r['self_projection_ignored'],1)
  # A point ahead of the occupied body is not removed; route must detour it.
  p=GridPlanner(raw,.1,[0,0,0],obstacles=[[4,3]],robot_pose=start)
  self.assertFalse(p.free[p.cell(4,3)[1],p.cell(4,3)[0]])
  # Original walls/unknown cells can never be cleared by the self mask.
  a[49,20]=0;raw=('P5\n%d %d\n255\n'%(w,h)).encode()+a.tobytes()
  p=GridPlanner(raw,.1,[0,0,0],obstacles=[[2.05,3.05]],robot_pose=start)
  self.assertFalse(p.free[49,20]);self.assertEqual(p.self_projection_ignored,0)
  with self.assertRaises(ValueError):p.plan(start,[8,3])

if __name__=='__main__':unittest.main()

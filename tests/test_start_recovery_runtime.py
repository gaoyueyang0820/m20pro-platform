"""Offline recovery policy and PointNavigator execution, no device sockets."""
import math,time,unittest,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from test_start_pose_recovery import planner,room,observe,body_oracle
from start_recovery_runtime import plan_with_recovery,TranslationRecovery


def recovery_scene():
 a=room();a[:,40:42]=0;p=planner(a);start=[3.56,4.,0.];goal=[2.05,4.]
 scan,loc=observe(p,start);r=plan_with_recovery(p,start,goal,0.,scan,loc,True)
 return p,start,goal,r


def scan_at(p,pose,velocity,now,stamp=None):
 s,l=observe(p,list(pose),stamp=1000+now if stamp is None else stamp)
 c,sn=math.cos(pose[2]),math.sin(pose[2]);s['_velocity_body']=[c*velocity[0]+sn*velocity[1],-sn*velocity[0]+c*velocity[1]]
 s.update(_valid_until=now+.6,age=.05)
 return s,l


class RecoveryControlTests(unittest.TestCase):
 def test_adapter_keeps_original_goal_and_explicit_preview_prefix(self):
  p,start,goal,r=recovery_scene()
  self.assertTrue(r['recovery']['required']);self.assertEqual(r['recovery']['mode'],'translation_only')
  self.assertEqual([r['dense_path'][0][k] for k in ['x','y','yaw']],start)
  self.assertLess(math.dist([r['goal']['x'],r['goal']['y']],goal),1e-8)
  self.assertTrue(r['recovery']['preview_only'])

 def test_normal_straight_route_has_no_recovery(self):
  p=planner(room());start=[2,3,0];s,l=observe(p,start)
  r=plan_with_recovery(p,start,[5,3],0,s,l,True)
  self.assertFalse(r['recovery']['required'])

 def test_closed_loop_reads_two_times_input_velocity_and_brakes_before_handoff(self):
  p,start,goal,r=recovery_scene();clock=[50.];pose=np.array(start,float);v=np.zeros(2);dt=.1;rows=[]
  with patch('time.monotonic',side_effect=lambda:clock[0]):
   controller=TranslationRecovery(r['recovery'],now=clock[0]);finished=False
   for _ in range(90):
    scan,loc=scan_at(p,pose,v,clock[0]);command=controller.command(p,scan,loc,standing=True,now=clock[0]);rows.append((clock[0],command,pose.copy(),v.copy()))
    self.assertNotEqual(command['state'],'failed',command)
    if command['state']=='complete':finished=True;break
    axes=np.asarray(command['axes'][:2]);c,s=math.cos(pose[2]),math.sin(pose[2]);target=np.array([c*axes[0]-s*axes[1],s*axes[0]+c*axes[1]])*2.
    for _ in range(10):
     delta=target-v;rate=1.2 if np.linalg.norm(target)>np.linalg.norm(v) else .7
     length=np.linalg.norm(delta)
     if length:v+=delta*min(1.,rate*.01/length)
     pose[:2]+=v*.01;clock[0]+=.01
     self.assertTrue(body_oracle(p,[dict(x=pose[0],y=pose[1],yaw=pose[2])]))
   self.assertTrue(finished,rows[-1][1]);self.assertLess(np.linalg.norm(v),.08)
   self.assertGreater(start[0]-pose[0],.06);self.assertLess(start[0]-pose[0],.8)
   self.assertTrue(p.pose_fits(*pose));self.assertTrue(any(row[1]['state']=='braking' for row in rows))
   self.assertTrue(any(np.linalg.norm(row[3])>.08 for row in rows))
   self.assertTrue(all(row[1]['valid_until']-row[0]<=.250001 for row in rows))
   self.assertTrue(p.plan(list(pose),goal,0)['ok'])

 def test_new_obstacle_prevents_a_command_whose_stopping_envelope_would_cross_it(self):
  p,start,goal,r=recovery_scene();clock=[100.];controller=TranslationRecovery(r['recovery'],now=100.)
  with patch('time.monotonic',side_effect=lambda:clock[0]):
   for _ in range(5):
    scan,loc=scan_at(p,start,[0,0],clock[0]);clock[0]+=.1
    controller.command(p,scan,loc,standing=True,now=clock[0]-.1)
   scan,loc=scan_at(p,start,[0,0],clock[0]);scan['obstacles'].append([-3,0]);scan['obstacle_bounds'].append([-.62,-.1,-.58,.1])
   result=controller.command(p,scan,loc,standing=True,now=clock[0])
   self.assertEqual(result['axes'],[0.,0.,0.]);self.assertEqual(result['state'],'failed')
   self.assertIn('制动',result['reason'])

 def test_expired_scan_and_out_of_model_velocity_do_not_move(self):
  p,start,goal,r=recovery_scene()
  for kind in ['expired','too_fast']:
   controller=TranslationRecovery(r['recovery'],now=100.);scan,loc=scan_at(p,start,[.9,0] if kind=='too_fast' else [0,0],100.)
   if kind=='expired':scan['_valid_until']=99.
   with patch('time.monotonic',return_value=100.):result=controller.command(p,scan,loc,standing=True,now=100.)
   self.assertEqual(result['axes'],[0,0,0]);self.assertEqual(result['state'],'failed')

 def test_no_physical_displacement_is_not_assumed_complete_after_pulses(self):
  p,start,goal,r=recovery_scene();clock=[100.];controller=TranslationRecovery(r['recovery'],now=100.)
  with patch('time.monotonic',side_effect=lambda:clock[0]):
   for _ in range(40):
    scan,loc=scan_at(p,start,[0,0],clock[0]);result=controller.command(p,scan,loc,standing=True,now=clock[0])
    if result['state']=='failed':break
    clock[0]+=.1
   self.assertEqual(result['state'],'failed');self.assertIn('位移',result['reason']);self.assertEqual(result['axes'],[0,0,0])



class PointRecoveryLifecycleTests(unittest.TestCase):
 def setUp(self):
  from test_platform import PlatformTests
  PlatformTests.setUp(self)
 def tearDown(self):
  from test_point_navigation_runtime import PointNavigationRuntimeTests
  PointNavigationRuntimeTests.tearDown(self)
 def test_public_preview_entry_offers_recovery_without_sending_motion(self):
  from test_point_navigation_runtime import PointNavigationRuntimeTests
  self.ready=PointNavigationRuntimeTests.ready.__get__(self);clock=[300.]
  with patch('time.monotonic',side_effect=lambda:clock[0]),patch('time.sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
   a=PointNavigationRuntimeTests.setup_live(self);self.ready(x=3.56,y=4.)
   grid=room(120,100);grid[:,40:42]=0
   (a.store.path.parent/'maps'/(self.map['id']+'.pgm')).write_bytes(b'P5\n120 100\n255\n'+grid.tobytes())
   truth=planner(grid)
   def fresh(mapping=False):
    clock[0]+=.2;self.ready(x=3.56,y=4.)
    scan,loc=scan_at(truth,[3.56,4.,0.],[0.,0.],clock[0]);return scan
   a.autonomy.scan_now=fresh
   result=a.plan_goal(dict(map_id=self.map['id'],x=2.05,y=4.,yaw=0.,engine='platform'))
  self.assertTrue(result['recovery']['required']);self.assertEqual(result['goal']['x'],2.05)
  self.assertEqual(result['path'][0],dict(x=3.56,y=4.))
  self.assertEqual(a.nav['status'],'idle');self.assertFalse(any(a.velocity))
  self.assertFalse(any(any(r['v']) for r in a.link.velocities))
 def _exercise(self,action=None):
  import hashlib
  import point_navigation as runtime
  from test_point_navigation_runtime import PointNavigationRuntimeTests,DeferredWorkers
  self.ready=PointNavigationRuntimeTests.ready.__get__(self)
  clock=[200.];initial_time=clock[0];pose=np.array([3.56,4.,0.]);velocity=np.zeros(2);history=[];event=[None]
  with patch('time.monotonic',side_effect=lambda:clock[0]):
   a=PointNavigationRuntimeTests.setup_live(self);self.ready(x=pose[0],y=pose[1])
   grid=room(120,100);grid[:,40:42]=0;raw=b'P5\n120 100\n255\n'+grid.tobytes()
   source=a.store.path.parent/'maps'/(self.map['id']+'.pgm');source.write_bytes(raw)
   truth=planner(grid);scan,loc=scan_at(truth,pose,velocity,clock[0]);preview=plan_with_recovery(truth,list(pose),[2.05,4.],0.,scan,loc,True)
   # The execution phase must ignore this deliberately wrong preview prefix.
   preview['recovery']['prefix']=[dict(x=3.56,y=4.,yaw=0.),dict(x=3.96,y=4.,yaw=0.)]
   preview['recovery']['support_seed']['floor']=[]
   route=a.route_save(dict(name='recovery lifecycle',map_id=self.map['id'],points=[preview['goal']]),persist=False)['route']
   record=dict(source=str(source),fingerprint=hashlib.sha256(raw).hexdigest(),result=preview)
   workers=DeferredWorkers()
   with patch.object(runtime.threading,'Thread',side_effect=workers.thread):r=a.point_navigation.start(route,record,a._nav_start_generation)
   oldlink=a.link
   def advance(duration):
    remaining=duration
    while remaining>1e-8:
     dt=min(.01,remaining);remaining-=dt
     axes=np.array(getattr(oldlink,'v',(0,0,0))[:2],float)
     if getattr(oldlink,'valid_until',None) is not None and clock[0]>=oldlink.valid_until:axes[:]=0
     c,s=math.cos(pose[2]),math.sin(pose[2]);target=2*np.array([c*axes[0]-s*axes[1],s*axes[0]+c*axes[1]])
     delta=target-velocity;norm=np.linalg.norm(delta);rate=1.2 if np.linalg.norm(target)>np.linalg.norm(velocity) else .7
     if norm:velocity[:]+=delta*min(1,rate*dt/norm)
     pose[:2]+=velocity*dt;clock[0]+=dt
     self.assertTrue(body_oracle(truth,[dict(x=pose[0],y=pose[1],yaw=pose[2])]))
     if a.link is oldlink:
      self.ready(x=float(pose[0]),y=float(pose[1]),yaw=float(pose[2]),velocity=(float(velocity[0]),float(velocity[1]),0.))
      a.point_navigation.renew(r['run_id']);a.tick(dt)
     if event[0] is None and a.nav.get('phase')=='recovering_start' and np.linalg.norm(velocity)>.08 and action:
      event[0]=len(oldlink.velocities)
      if action=='cancel':a.stop()
      elif action=='disconnect':a.connect(dict(mode='demo'))
     if clock[0]-initial_time>25:a.done=True
   def actual_scan(mapping=False):
    advance(.06)
    s,l=scan_at(truth,pose,velocity,clock[0]);return s
   a.autonomy.scan_now=actual_scan
   original_update=a.point_navigation._update
   def observe_update(*args,**kwargs):
    original_update(*args,**kwargs);history.append(dict(t=clock[0],phase=a.nav.get('phase'),recovery=dict(a.nav.get('recovery',{})),pose=pose.copy(),velocity=velocity.copy()))
   original_plan=runtime.plan_with_recovery
   def during_plan(*args,**kwargs):
    fresh=original_plan(*args,**kwargs)
    if action=='cancel_planning' and event[0] is None:event[0]=len(oldlink.velocities);a.stop()
    return fresh
   with patch.object(runtime.time,'sleep',side_effect=advance),patch.object(a.point_navigation,'_update',side_effect=observe_update),patch.object(runtime,'plan_with_recovery',side_effect=during_plan):workers.run()
   if action!='disconnect':advance(1.5)
   return a,oldlink,pose,velocity,history,event[0]

 def test_full_point_navigation_recomputes_preview_recovers_then_arrives_at_original_goal(self):
  a,link,pose,velocity,history,event=self._exercise()
  self.assertEqual(a.nav['status'],'completed',a.nav)
  self.assertTrue(a.nav['history_saved']);self.assertLess(math.dist(pose[:2],[2.05,4.]),.2)
  self.assertTrue(any(r['phase']=='recovering_start' for r in history))
  self.assertTrue(any(r['recovery'].get('state')=='completed' for r in history))
  self.assertFalse(any(v['v'][0]>0 for v in link.velocities),'tampered forward preview must never become motion')
  self.assertTrue(any(v['v'][0]<0 and v['valid_until'] is not None for v in link.velocities))

 def test_cancel_during_actual_recovery_zeros_and_late_worker_cannot_resume(self):
  a,link,pose,velocity,history,index=self._exercise('cancel')
  self.assertIsNotNone(index);self.assertEqual(a.nav['status'],'cancelled')
  self.assertTrue(all(not any(v['v']) for v in link.velocities[index:]))
  self.assertFalse(a.point_navigation.active)

 def test_connection_switch_during_recovery_cannot_retarget_old_commands(self):
  a,link,pose,velocity,history,index=self._exercise('disconnect')
  self.assertIsNotNone(index);self.assertTrue(a.demo);self.assertEqual(a.nav['status'],'idle')
  self.assertTrue(all(not any(v['v']) for v in link.velocities[index:]))
  self.assertFalse(a.point_navigation.active)

 def test_cancel_during_fresh_recovery_planning_cannot_start_on_late_result(self):
  a,link,pose,velocity,history,index=self._exercise('cancel_planning')
  self.assertIsNotNone(index);self.assertEqual(a.nav['status'],'cancelled')
  self.assertTrue(all(not any(v['v']) for v in link.velocities))
  self.assertFalse(a.point_navigation.active)

if __name__=='__main__':unittest.main()

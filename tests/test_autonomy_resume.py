"""Same-capture resume is an explicit new control lease, not a new map."""
import copy,threading,time,unittest
from unittest.mock import patch
import test_platform as cases
from autonomy_core import Explorer,disk

A='a'*32
class ResumeTests(unittest.TestCase):
 setUp=cases.PlatformTests.setUp
 def tearDown(self):
  self.app.done=True
  with self.app.lock:self.app.autonomy.pause('test end')
  self.app.hub.close();cases.PlatformTests.tearDown(self)
 def paused(self):
  a=self.app.autonomy;e=Explorer(6);old=dict(pose=[0.,0.,0.],floor=[list(q) for q in disk(3)],obstacles=[],stamp=10.)
  e.ingest(old);e.distance=.4;e.path=[(0,0),(1,0)];e.goal=(1,0)
  a.mode='explore';a.state='paused';a.active=False;a.explorer=e;a.capture_id=A;a.context=self.app.hub.context();a.generation=2;a.scan=old
  a.finished_elapsed=12;a._session_request=dict(minutes=1,mode='explore',extend=True,base='old-map');a.prior_grid={'kept':'reference'};a.scope='room';self.app.mapping=True
  self.app.hub.demo_maps.dispatch('/maps/build',dict(action='start',name='original-capture',request_id=A))
  return a,e
 def wait(self,predicate):
  end=time.monotonic()+2
  while not predicate() and time.monotonic()<end:time.sleep(.01)
  self.assertTrue(predicate(),self.app.autonomy.message)
 def test_resume_preserves_history_origin_capture_and_rebases_manual_displacement(self):
  a,e=self.paused();origin=e.origin;known=e.free.copy();visited=e.visited.copy();frontiers=e.frontiers
  packets=[dict(pose=[1.4,0.,0.],floor=[list(q) for q in disk(3)],obstacles=[],stamp=11.,_valid_until=time.monotonic()+.6),dict(pose=[1.42,0.,0.],floor=[list(q) for q in disk(3)],obstacles=[],stamp=12.,_valid_until=time.monotonic()+.6)]
  with patch.object(a,'scan_now',side_effect=packets),patch.object(a,'_run') as run:
   r=a.resume(dict(request_id='b'*32,capture_id=A));self.wait(lambda:run.called)
   self.assertIs(a.explorer,e);self.assertIs(e.frontiers,frontiers);self.assertEqual(e.origin,origin);self.assertEqual(e.free,known);self.assertEqual(e.visited,visited)
   self.assertEqual(a.capture_id,A);self.assertEqual(e.path,[]);self.assertEqual(e.distance,.4);self.assertEqual(e.last_pose,(1.42,0.,0.));self.assertEqual(a.elapsed_base,12)
   e.ingest(dict(packets[-1],pose=[1.44,0,0],stamp=13.))
   self.assertEqual(a.prior_grid,{'kept':'reference'});self.assertEqual(run.call_args.args,(r['run_id'],None,60.))
   again=a.resume(dict(request_id='b'*32,capture_id=A));self.assertEqual(again['delivery'],'already_requested');self.assertEqual(run.call_count,1)
  self.assertEqual(self.app.hub.demo_maps.demo_mapping['request_id'],A)
 def test_cancel_during_resume_read_keeps_capture_and_never_starts_worker(self):
  a,e=self.paused();entered=threading.Event();release=threading.Event()
  def read(*args,**kwargs):entered.set();release.wait(2);return dict(request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id=A))
  with patch.object(self.app.hub,'remote_maps',side_effect=read),patch.object(a,'_run') as run:
   a.resume();self.assertTrue(entered.wait(1))
   with self.app.lock:a.pause('operator paused again')
   release.set();self.wait(lambda:a._resume_token is None)
   run.assert_not_called();self.assertTrue(self.app.mapping);self.assertEqual(a.state,'paused')
 def test_changed_capture_is_not_relabelled_as_original_history(self):
  a,e=self.paused()
  status=dict(request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id='c'*32))
  with patch.object(self.app.hub,'remote_maps',return_value=status),patch.object(a,'_run') as run:
   a.resume();self.wait(lambda:not a.active)
   run.assert_not_called();self.assertIs(a.explorer,e);self.assertEqual(a.capture_id,A);self.assertIn('采集',a.message)
 def test_structured_sync_pending_retries_but_coordinate_error_does_not(self):
  from observation_errors import ObservationSyncPending
  a,e=self.paused();sample=dict(pose=[1.,0.,0.],floor=[list(q) for q in disk(3)],obstacles=[],stamp=11.,_valid_until=time.monotonic()+.6)
  with patch.object(a,'scan_now',side_effect=[ObservationSyncPending('wait','slam_pose_sync'),sample,dict(sample,stamp=12.)]) as read,patch.object(a,'_run') as run:
   a.resume();self.wait(lambda:run.called);self.assertEqual(read.call_count,3)
   with self.app.lock:a.pause()
  self.wait(lambda:a._worker_thread is None or not a._worker_thread.is_alive())
  with patch.object(a,'scan_now',side_effect=ValueError('机身倾斜')) as read,patch.object(a,'_run') as run:
   a.resume();self.wait(lambda:not a.active);self.assertEqual(read.call_count,1);run.assert_not_called()
 def test_stop_before_late_resume_request_invalidates_its_control_generation(self):
  a,e=self.paused();generation=a.snapshot()['control_generation']
  with self.app.lock:a.pause('stop arrived first')
  with self.assertRaisesRegex(ValueError,'取消'):a.resume(dict(control_generation=generation,capture_id=A))
  self.assertFalse(a.active);self.assertTrue(self.app.mapping)

 def test_ui_flags_offer_resume_instead_of_a_new_map(self):
  a,e=self.paused()
  with patch.object(a.display_observer,'decorate',side_effect=lambda view:view):status=a.snapshot()
  self.assertTrue(status['can_resume']);self.assertFalse(status['can_start']);self.assertTrue(status['discard_supported']);self.assertTrue(status['can_discard'])

if __name__=='__main__':unittest.main()

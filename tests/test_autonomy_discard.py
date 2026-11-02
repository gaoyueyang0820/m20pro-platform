"""Capture-owned stop/discard without publishing a fusion candidate; all offline."""
import json,time,tempfile,unittest,threading,hashlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import test_platform as cases
import test_autonomy_resume as resume_cases
A=resume_cases.A
from bridge import mapping_job
from bridge.map_service import MapService

B='b'*32
class DiscardWorkerTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.logs=self.root/'logs';self.logs.mkdir();self.maps=self.root/'maps';self.maps.mkdir()
  self.base=self.maps/'base';self.base.mkdir();(self.base/'full_cloud.pcd').write_bytes(b'original');(self.maps/'active').symlink_to('base',target_is_directory=True)
  self.context=dict(workflow='extension',base='base',anchor=dict(x=0,y=0,z=0,yaw=0),grid_id=0,maps=str(self.maps))
  self.capture=dict(state='command_finished',request_id=A,name='m20-session-'+A,started=time.time()-10,context=self.context,mapping_processes=[])
  (self.logs/'mapping.json').write_text(json.dumps(self.capture));self.folder=self.maps/(self.capture['name']+'-20261102');self.folder.mkdir();(self.folder/'full_cloud.pcd').write_bytes(b'raw observation')
 def tearDown(self):self.tmp.cleanup()
 def run_worker(self):
  with patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=0)) as command,patch.object(mapping_job,'slam_processes',return_value={}),patch('map_fusion.fuse_maps',side_effect=AssertionError('discard cannot fuse')) as fuse:
   mapping_job.worker('stop_mapping',self.logs,context=dict(self.context,discard=True),request_id=B,expected_start_request_id=A)
  command.assert_called_once();self.assertEqual(command.call_args.args[0],['drmap','stop_mapping']);fuse.assert_not_called()
  return mapping_job.read_job('stop_mapping',self.logs)
 def test_discard_retains_hidden_vendor_observations_without_fusing_or_applying(self):
  before=(self.base/'full_cloud.pcd').read_bytes();result=self.run_worker()
  self.assertTrue(result['discarded']);self.assertFalse(result['candidate_created']);self.assertTrue(result['observations_saved']);self.assertEqual(result['result_maps'],[])
  self.assertTrue((self.folder/'.m20-session-source.json').is_file());self.assertEqual((self.maps/'active').resolve(),self.base.resolve());self.assertEqual((self.base/'full_cloud.pcd').read_bytes(),before)
  self.assertNotIn(self.folder.name,[m['name'] for m in MapService(self.maps,self.root/'data',False).library()['maps']])
  phase=mapping_job.mapping_phase(self.capture,result,[]);self.assertEqual(phase['phase'],'discarded');self.assertFalse(phase['in_progress'])
 def test_ambiguous_directories_are_not_hidden_or_deleted(self):
  other=self.maps/(self.capture['name']+'-another');other.mkdir();(other/'full_cloud.pcd').write_bytes(b'other')
  result=self.run_worker();self.assertEqual(result['observation_ownership'],'unconfirmed')
  self.assertFalse((self.folder/'.m20-session-source.json').exists());self.assertFalse((other/'.m20-session-source.json').exists());self.assertTrue(result['discarded'])
 def test_active_directory_prevents_all_hiding_before_any_marker_is_written(self):
  (self.maps/'active').unlink();(self.maps/'active').symlink_to(self.folder.name,target_is_directory=True)
  result=self.run_worker();self.assertEqual(result['observation_ownership'],'unconfirmed');self.assertFalse((self.folder/'.m20-session-source.json').exists())
 def test_stop_identity_cannot_be_replayed_with_opposite_disposition(self):
  self.run_worker()
  with patch.object(mapping_job.subprocess,'Popen') as process:
   with self.assertRaisesRegex(ValueError,'意图'):mapping_job.launch('stop_mapping',logs=self.logs,request_id=B,expected_start_request_id=A)
  process.assert_not_called()
 def test_changed_capture_never_calls_vendor_stop(self):
  capture=dict(self.capture,request_id='c'*32);(self.logs/'mapping.json').write_text(json.dumps(capture))
  with patch.object(mapping_job.subprocess,'run') as command,patch.object(mapping_job,'slam_processes',return_value={}):mapping_job.worker('stop_mapping',self.logs,context=dict(self.context,discard=True),request_id=B,expected_start_request_id=A)
  command.assert_not_called();self.assertEqual(mapping_job.read_job('stop_mapping',self.logs)['state'],'failed')


class DiscardLifecycleTests(unittest.TestCase):
 setUp=cases.PlatformTests.setUp
 tearDown=resume_cases.ResumeTests.tearDown
 paused=resume_cases.ResumeTests.paused
 wait=resume_cases.ResumeTests.wait
 def job_done(self,request):
  self.wait(lambda:self.app.hub.operations.get(request['job_id'])['status'] in ['completed','failed'])
  return self.app.hub.operations.get(request['job_id'])
 def test_real_hub_status_updates_live_capability_and_snapshot(self):
  a,e=self.paused();self.app.demo=False;self.app.link=cases.FakeLink();a.context=self.app.hub.context();self.app.proxy=lambda *args,**kwargs:dict(discard_protocol=1,request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id=A))
  self.assertFalse(a.snapshot()['discard_supported'])
  with patch('platform_hub.rpc',return_value=dict(discard_protocol=1,request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id=A))):self.app.hub.remote_maps('/maps/build/status')
  self.assertTrue(a.snapshot()['discard_supported']);self.assertTrue(a.snapshot()['can_discard'])
 def test_late_old_connection_capability_does_not_enable_new_robot(self):
  a,e=self.paused();self.app.demo=False;self.app.link=cases.FakeLink();a.context=self.app.hub.context()
  def read(*args,**kwargs):self.app.hub._context_generation+=1;return dict(discard_protocol=1)
  with patch('platform_hub.rpc',side_effect=read):self.app.hub.remote_maps('/maps/build/status')
  self.assertFalse(a.snapshot()['discard_supported'])
 def test_active_capture_can_discard_directly_without_a_separate_pause(self):
  a,e=self.paused();a.active=True;a.state='exploring';a.run_id='active-run';a.started=time.monotonic();a.lease=a.started;self.app.velocity=[.2,0,0]
  request=a.post('/api/autonomy/discard',dict(capture_id=A));job=self.job_done(request)
  self.assertEqual(job['status'],'completed',job);self.assertEqual(a.state,'discarded');self.assertFalse(a.active);self.assertFalse(self.app.mapping);self.assertEqual(a.result_maps,[]);self.assertEqual(self.app.velocity,[0,0,0]);self.assertFalse(a.snapshot()['can_resume'])
  again=a.finish_discard(dict(capture_id=A));self.assertEqual(again['job_id'],request['job_id'])
 def test_unavailable_robot_protocol_never_falls_back_to_normal_save(self):
  a,e=self.paused();calls=[]
  def remote(path,req=None,**kwargs):calls.append((path,req));return dict(request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id=A))
  with patch.object(self.app.hub,'remote_maps',side_effect=remote):job=self.job_done(a.finish_discard())
  self.assertEqual(job['status'],'failed');self.assertTrue(self.app.mapping);self.assertEqual(a.state,'paused');self.assertFalse(any(path=='/maps/build' for path,_ in calls))
 def test_duplicate_discard_returns_same_operation_and_save_cannot_replace_it(self):
  a,e=self.paused();entered=threading.Event();release=threading.Event();remote=self.app.hub.remote_maps
  def delayed(*args,**kwargs):entered.set();release.wait(2);return remote(*args,**kwargs)
  with patch.object(self.app.hub,'remote_maps',side_effect=delayed):
   first=a.finish_discard();self.assertTrue(entered.wait(1));second=a.finish_discard();self.assertEqual(first['job_id'],second['job_id'])
   with self.assertRaisesRegex(ValueError,'同时保存'):a.finish_save()
   release.set();self.assertEqual(self.job_done(first)['status'],'completed')
 def test_lost_discard_reply_reconciles_same_request_without_saving(self):
  a,e=self.paused();real=self.app.hub.remote_maps;sent=[]
  def transport(path,req=None,**kwargs):
   result=real(path,req,**kwargs)
   if path=='/maps/build':sent.append(dict(req));raise ValueError('simulated lost reply')
   return result
  with patch.object(self.app.hub,'remote_maps',side_effect=transport):job=self.job_done(a.finish_discard())
  self.assertEqual(job['status'],'completed',job);self.assertEqual(len(sent),1);self.assertEqual(sent[0]['action'],'discard')
  self.assertEqual(a.state,'discarded');self.assertFalse(self.app.mapping)

 def test_discard_rechecks_capture_before_sending_mutation(self):
  a,e=self.paused();calls=[]
  def remote(path,req=None,**kwargs):calls.append(path);return dict(discard_protocol=1,request_protocol=1,phase='collecting',in_progress=True,mapping=dict(request_id='c'*32))
  with patch.object(self.app.hub,'remote_maps',side_effect=remote):job=self.job_done(a.finish_discard())
  self.assertEqual(job['status'],'failed');self.assertNotIn('/maps/build',calls);self.assertTrue(self.app.mapping)

if __name__=='__main__':unittest.main()

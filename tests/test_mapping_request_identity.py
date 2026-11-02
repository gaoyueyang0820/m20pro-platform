"""Lost replies and ownership races; no vendor process or robot is started."""
import json,tempfile,time,unittest,sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge import mapping_job
from bridge.map_service import MapService

A='a'*32
B='b'*32

class MappingRequestIdentityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.logs=Path(self.tmp.name)/'logs';self.logs.mkdir()
    def tearDown(self):self.tmp.cleanup()
    def write(self,kind,record):
        (self.logs/(kind+'.json')).write_text(json.dumps(record))
    def test_lost_start_reply_retry_does_not_spawn_again(self):
        with patch.object(mapping_job.subprocess,'Popen',return_value=SimpleNamespace(pid=12345)) as process:
            first=mapping_job.launch('mapping','survey',self.logs,request_id=A)
            second=mapping_job.launch('mapping','survey',self.logs,request_id=A)
        self.assertEqual(process.call_count,1);self.assertEqual(first['request_id'],A)
        self.assertEqual(second['delivery'],'already_launched')
        self.assertEqual(mapping_job.read_job('mapping',self.logs)['request_id'],A)
    def test_same_identity_cannot_silently_change_payload(self):
        self.write('mapping',dict(state='command_finished',name='first',context={},request_id=A))
        with patch.object(mapping_job.subprocess,'Popen') as process:
            with self.assertRaisesRegex(ValueError,'参数不一致'):mapping_job.launch('mapping','second',self.logs,request_id=A)
        process.assert_not_called()
    def test_cleanup_cannot_stop_another_computers_capture(self):
        self.write('mapping',dict(state='command_finished',name='new-owner',context={},request_id=B))
        with patch.object(mapping_job.subprocess,'Popen') as process:
            with self.assertRaisesRegex(ValueError,'不属于'):mapping_job.launch('stop_mapping',logs=self.logs,request_id=A,expected_start_request_id=A)
        process.assert_not_called()
    def test_no_start_while_previous_map_is_still_saving(self):
        self.write('stop_mapping',dict(state='starting',started=time.time(),request_id=A))
        with patch.object(mapping_job.subprocess,'Popen') as process:
            with self.assertRaisesRegex(ValueError,'停止或保存'):mapping_job.launch('mapping','next',self.logs,request_id=B)
        process.assert_not_called()
    def test_stop_worker_rechecks_owner_before_any_vendor_call(self):
        self.write('mapping',dict(state='command_finished',name='other',context={},request_id=B))
        with patch.object(mapping_job.subprocess,'run') as run:
            mapping_job.worker('stop_mapping',self.logs,request_id=A,expected_start_request_id=A)
        run.assert_not_called();r=mapping_job.read_job('stop_mapping',self.logs)
        self.assertEqual(r['state'],'failed');self.assertEqual(r['request_id'],A)
    def test_stop_waits_for_start_then_saves_same_capture(self):
        states=[dict(state='running',request_id=A),dict(state='running',request_id=A),dict(state='command_finished',request_id=A)]
        with patch.object(mapping_job,'read_job',side_effect=states),patch.object(mapping_job.time,'sleep'),patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=0)) as run:
            mapping_job.worker('stop_mapping',self.logs,request_id=B,expected_start_request_id=A)
        run.assert_called_once();self.assertEqual(run.call_args.args[0],['drmap','stop_mapping'])
    def test_worker_preserves_identity_for_status_reconciliation(self):
        with patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=0)),patch.object(mapping_job,'slam_processes',return_value={}):
            mapping_job.worker('mapping',self.logs,'survey',request_id=A)
        self.assertEqual(mapping_job.read_job('mapping',self.logs)['request_id'],A)
    def test_demo_has_same_ownership_and_idempotency_contract(self):
        service=MapService(Path(self.tmp.name)/'maps',Path(self.tmp.name)/'data',True)
        service.dispatch('/maps/build',dict(action='start',name='demo',request_id=A))
        again=service.dispatch('/maps/build',dict(action='start',name='demo',request_id=A))
        self.assertEqual(again['delivery'],'already_launched')
        with self.assertRaisesRegex(ValueError,'不属于'):service.dispatch('/maps/build',dict(action='stop',expected_start_request_id=B))
        self.assertTrue(service.dispatch('/maps/build/status',{})['in_progress'])
        service.dispatch('/maps/build',dict(action='stop',expected_start_request_id=A,request_id=B))
        self.assertFalse(service.dispatch('/maps/build/status',{})['in_progress'])
        self.assertEqual(service.dispatch('/maps/build/status',{})['mapping']['request_id'],A)
    def test_manual_start_also_returns_identity_for_later_continue(self):
        service=MapService(Path(self.tmp.name)/'maps',Path(self.tmp.name)/'data',True)
        result=service.dispatch('/maps/build',dict(action='start',name='manual'))
        self.assertRegex(result['request_id'],r'^[0-9a-f]{32}$')
        status=service.dispatch('/maps/build/status',{})
        self.assertEqual(status['request_protocol'],1);self.assertEqual(status['mapping']['request_id'],result['request_id'])
    def test_old_request_replay_after_another_completed_session_is_not_a_new_start(self):
        with patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=0)),patch.object(mapping_job,'slam_processes',return_value={}):
            mapping_job.worker('mapping',self.logs,'first',request_id=A)
            mapping_job.worker('mapping',self.logs,'second',request_id=B)
        with patch.object(mapping_job.subprocess,'Popen') as process:
            replay=mapping_job.launch('mapping','first',self.logs,request_id=A)
        process.assert_not_called();self.assertEqual(replay['delivery'],'already_launched')
        self.assertEqual(mapping_job.read_job('mapping',self.logs)['request_id'],B)
    def test_external_slam_without_journal_prevents_new_launch(self):
        external={99:dict(pid=99,start='1',command='/opt/robot/share/slam/bin/slam')}
        with patch.object(mapping_job,'slam_processes',return_value=external),patch.object(mapping_job.subprocess,'Popen') as process:
            with self.assertRaisesRegex(ValueError,'已有SLAM'):mapping_job.launch('mapping','survey',self.logs,request_id=A)
        process.assert_not_called()
    def test_owned_old_journal_does_not_authorize_stopping_new_external_slam(self):
        self.write('mapping',dict(state='command_finished',name='old',request_id=A,context={}))
        external={99:dict(pid=99,start='1',command='/opt/robot/share/slam/bin/slam')}
        with patch.object(mapping_job,'slam_processes',return_value=external),patch.object(mapping_job.subprocess,'Popen') as process:
            with self.assertRaisesRegex(ValueError,'外部建图'):mapping_job.launch('stop_mapping',logs=self.logs,request_id=B,expected_start_request_id=A)
        process.assert_not_called()
    def test_no_start_record_does_not_claim_all_history_as_new_results(self):
        result=mapping_job.mapping_phase(dict(state='none'),dict(state='command_finished',started=100),[dict(name='old',modified=200,has_cloud=True,grids=[0])])
        self.assertEqual(result['phase'],'unconfirmed');self.assertEqual(result['result_maps'],[])
    def test_process_detection_ignores_python_diagnostic_text(self):
        proc=Path(self.tmp.name)/'proc';(proc/'7').mkdir(parents=True);(proc/'8').mkdir()
        (proc/'7/cmdline').write_bytes(b'python3\0-c\0print("/opt/robot/share/slam/config")\0')
        (proc/'8/cmdline').write_bytes(b'/opt/robot/share/slam/bin/slam\0--config\0params.yaml\0')
        (proc/'8/stat').write_text('8 (slam) R '+'0 '*18+'123')
        self.assertEqual(set(mapping_job.slam_processes(proc)),{8})
    def test_status_marks_unowned_slam_as_external_work(self):
        service=MapService(Path(self.tmp.name)/'maps',Path(self.tmp.name)/'data',False)
        with patch('bridge.map_service.read_job',return_value=dict(state='none')),patch('bridge.map_service.unowned_processes',return_value=[dict(pid=9)]):
            result=service.dispatch('/maps/build/status',{})
        self.assertEqual(result['phase'],'external_collecting');self.assertTrue(result['in_progress']);self.assertEqual(result['result_maps'],[])

if __name__=='__main__':unittest.main()

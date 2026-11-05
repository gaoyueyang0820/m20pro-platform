"""Saved-scan outcomes: portable synthetic cases and a separate optional private replay.

Only temporary files and mocked vendor commands; no robot or service access.
"""
import copy,hashlib,json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge import mapping_job
from m20.map_change_policy import gate_alignment
from m20.map_fusion import FusionDecisionError,capture_continuation,fuse_maps,write_pcd

PRIVATE_FIXTURE=Path(__file__).parent/'fixtures/runtime_replay/fusion_deferred_20261102.json'


class SavedObservationTests(unittest.TestCase):
    fixture=None
    def setUp(self):
        if self.fixture is None:
            if not PRIVATE_FIXTURE.is_file():self.skipTest('Private recorded fusion fixture is not distributed')
            self.fixture=json.loads(PRIVATE_FIXTURE.read_text())
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.maps=self.root/'maps';self.maps.mkdir();self.logs=self.root/'logs';self.logs.mkdir()
        self.base=self.maps/'base';self.base.mkdir();(self.maps/'active').symlink_to('base')
        self.scan=self.maps/'capture-20261102';self.scan.mkdir()
        points=np.random.default_rng(11).normal(size=(500,4)).astype(np.float32)
        for folder in [self.base,self.scan]:
            write_pcd(folder/'full_cloud.pcd',points)
            native=folder/'.sessions/session_0';(native/'lidar_cloud').mkdir(parents=True)
            write_pcd(native/'lidar_cloud/0.pcd',points[:300])
            (native/'poses.txt').write_text(self.fixture['poses_text'])
            (folder/'occ_grid.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([254]*4))
            (folder/'occ_grid.yaml').write_text('resolution: 1\norigin: [0,0,0]\n')
        self.start=dict(state='command_finished',started=100,name='capture',request_id='a'*32,
                        context=dict(workflow='extension',base='base',maps=str(self.maps),
                                     anchor=dict(x=0,y=0,z=0,yaw=0),grid_id=0))
        (self.logs/'mapping.json').write_text(json.dumps(self.start))
    def tearDown(self):self.tmp.cleanup()
    def rejection(self,aligned=True,context=True):
        report=copy.deepcopy(self.fixture['alignment']['metrics'])
        if not aligned:report['structural_overlap']=.4
        decision=gate_alignment(report);report.update(frames=copy.deepcopy(self.fixture['alignment']['frames']),decision=decision)
        if context:
            poses=np.loadtxt(self.scan/'.sessions/session_0/poses.txt',ndmin=2)
            report.update(capture_continuation(self.base,self.scan,poses,0,self.fixture['evidence']))
        return FusionDecisionError(report)
    def finish(self,rejection):
        with patch.object(mapping_job,'slam_processes',return_value={}),patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as run,patch('m20.map_fusion.fuse_maps',side_effect=rejection):
            mapping_job.worker('stop_mapping',self.logs,request_id='b'*32,expected_start_request_id='a'*32)
        self.assertEqual(run.call_count,1)
        return mapping_job.read_job('stop_mapping',self.logs)

    def test_one_frame_fixture_remains_rejected_for_publication_without_lowering_gate(self):
        error=self.rejection();self.assertIsInstance(error,ValueError);self.assertTrue(error.evidence_deferred)
        self.assertEqual(error.report['decision'],self.fixture['alignment']['decision'])
        self.assertEqual(error.report['known_free_conflict_ratio'],self.fixture['alignment']['metrics']['known_free_conflict_ratio'])
        capture=error.report['observation_session']
        self.assertEqual(capture['saved_keyframes'],1);self.assertEqual(capture['minimum_additional_keyframes'],2)
        self.assertEqual(capture['minimum_additional_epochs'],2);self.assertEqual(capture['minimum_additional_span_s'],20)
        self.assertEqual(error.report['requirements']['min_view_separation_m'],.35)
        self.assertFalse(error.report['next_capture']['append_supported'])
        self.assertFalse(error.report['next_capture']['automatic_cross_session_fusion'])

    def test_fusion_attaches_saved_session_identity_to_structured_rejection(self):
        error=self.rejection(context=False)
        with patch('m20.map_fusion.structural_alignment',side_effect=error):
            with self.assertRaises(FusionDecisionError) as caught:
                fuse_maps(self.base,self.scan,self.maps/'candidate',dict(x=0,y=0,z=0,yaw=0),registration=lambda source,target:np.eye(4))
        self.assertEqual(caught.exception.report['observation_session']['path'],str(self.scan))
        self.assertEqual(caught.exception.report['observation_session']['saved_keyframes'],1)
        self.assertEqual(caught.exception.report['temporal_evidence']['reason'],'fewer_than_three_keyframes')
        self.assertFalse((self.maps/'candidate').exists())

    def test_native_save_is_completed_but_candidate_remains_deferred(self):
        before=hashlib.sha256((self.base/'full_cloud.pcd').read_bytes()).hexdigest();row=self.finish(self.rejection())
        self.assertEqual(row['state'],'command_finished');self.assertEqual(row['returncode'],0)
        self.assertEqual(row['save_status'],'observations_saved');self.assertEqual(row['fusion_status'],'awaiting_evidence')
        self.assertTrue(row['observations_saved']);self.assertFalse(row['candidate_created']);self.assertFalse(row['activated'])
        self.assertEqual(row['result_maps'],[]);self.assertEqual(row['observation_session']['path'],str(self.scan))
        self.assertIn('本轮 1 个关键帧',row['message']);self.assertNotIn('可用于导航',row['message'])
        self.assertEqual((self.maps/'active').resolve(),self.base.resolve())
        self.assertEqual(hashlib.sha256((self.base/'full_cloud.pcd').read_bytes()).hexdigest(),before)
        self.assertEqual((self.scan/'.sessions/session_0/poses.txt').read_text(),self.fixture['poses_text'])
        self.assertEqual(list(self.maps.glob('*-r2-*')),[])
        receipt=json.loads((self.logs/'requests'/('stop_mapping-'+'b'*32+'.json')).read_text())
        self.assertEqual(receipt['fusion_status'],'awaiting_evidence')

    def test_saved_phase_has_no_fused_map_and_repeated_stop_never_restarts_save(self):
        row=self.finish(self.rejection());phase=mapping_job.mapping_phase(self.start,row,[])
        self.assertEqual(phase['phase'],'saved');self.assertFalse(phase['in_progress'])
        self.assertEqual(phase['fusion_status'],'awaiting_evidence');self.assertEqual(phase['result_maps'],[])
        self.assertEqual(phase['requirements']['min_saved_keyframes'],3)
        with patch.object(mapping_job,'slam_processes',return_value={}),patch.object(mapping_job.subprocess,'Popen') as fork:
            repeated=mapping_job.launch('stop_mapping',logs=self.logs,expected_start_request_id='a'*32)
            same_request=mapping_job.launch('stop_mapping',logs=self.logs,request_id='b'*32,expected_start_request_id='a'*32)
        fork.assert_not_called();self.assertEqual(repeated['delivery'],'already_saved')
        self.assertEqual(same_request['delivery'],'already_launched')
        for result in [repeated,same_request]:
            self.assertEqual(result['fusion_status'],'awaiting_evidence');self.assertEqual(result['result_maps'],[])
            self.assertIn('观测已保存',result['message'])

    def test_bad_structural_alignment_stays_failed_but_saved_scan_is_not_lost(self):
        error=self.rejection(aligned=False);self.assertFalse(error.evidence_deferred)
        row=self.finish(error);self.assertEqual(row['state'],'failed');self.assertEqual(row['fusion_status'],'alignment_failed')
        self.assertTrue(row['observations_saved']);self.assertIn('原始观测已保存',row['message'])
        self.assertEqual(row['result_maps'],[]);self.assertFalse(row['candidate_created'])
        phase=mapping_job.mapping_phase(self.start,row,[])
        self.assertEqual(phase['phase'],'error');self.assertTrue(phase['observations_saved']);self.assertFalse(phase['in_progress'])

    def test_vendor_stop_failure_is_not_relabelled_as_saved_observations(self):
        with patch.object(mapping_job,'slam_processes',return_value={}),patch.object(mapping_job.subprocess,'run',return_value=SimpleNamespace(returncode=2,stdout='',stderr='save failed')),patch('m20.map_fusion.fuse_maps') as fuse:
            mapping_job.worker('stop_mapping',self.logs)
        row=mapping_job.read_job('stop_mapping',self.logs);fuse.assert_not_called()
        self.assertEqual(row['state'],'failed');self.assertFalse(row.get('observations_saved',False))
        self.assertEqual(mapping_job.mapping_phase(self.start,row,[])['phase'],'error')


# A separate synthetic contract suite; it never impersonates the private replay.
_synthetic_metrics=dict(structural_overlap=.8,rmse_m=.08,effective_frames=1,known_free_conflict_ratio=.2,
    correction_m=.1,correction_yaw_rad=.01,temporal_change_confirmed=False,confirmed_conflict_fraction=0.)
class SyntheticSavedObservationTests(SavedObservationTests):
    fixture=dict(source='synthetic contract fixture',poses_text='0 0 0 0 0 0 0 1\n',
        evidence=dict(state='deferred',reason='fewer_than_three_keyframes',frames=1),
        alignment=dict(metrics=_synthetic_metrics,decision=gate_alignment(_synthetic_metrics),
                       frames=[dict(frame=0,points=500,structural_overlap=.8,retained_old_overlap=.2,rmse_m=.08)]))


if __name__=='__main__':unittest.main()

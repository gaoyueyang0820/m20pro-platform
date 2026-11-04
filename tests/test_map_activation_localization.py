"""Synthetic map lineage + real activation coordinator; no robot/network I/O."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock,patch
import sys
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from bridge.map_service import MapService
from bridge import root_actions
import test_platform as cases


def candidate_report(kind='sessions'):
    report=dict(map='candidate',base_map='base',created=10.,includes_complete_base=True,
        structural_alignment=dict(structural_overlap=.96,rmse_m=.05,effective_frames=3,
            correction_m=.03,correction_yaw_rad=.01,known_free_conflict_ratio=.01,decision=dict(allowed=True)))
    if kind=='sessions':report.update(session=1,source_map='capture',known_region_change_policy='saved_keyframe_temporal_evidence',
        transform=[[0,-1,0,8],[1,0,0,9],[0,0,1,.2],[0,0,0,1]])
    else:report['method']='structure_aligned_temporal_visibility'
    return report


class ActivationFrameEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name);self.maps=root/'maps';self.maps.mkdir()
        for name in ['base','candidate','unrelated']:(self.maps/name).mkdir()
        (self.maps/'active').symlink_to('base',target_is_directory=True)
        self.service=MapService(self.maps,root/'data',True)
    def manifest(self,kind='sessions',report=None):
        (self.maps/'candidate/.m20-project.json').write_text(json.dumps(dict(root_map='base',**{kind:[report or candidate_report(kind)]})))
    def test_fusion_and_refinement_preserve_base_pose_not_session_transform(self):
        for kind in ['sessions','refinements']:
            self.manifest(kind);result=self.service.activation_context('candidate')
            self.assertTrue(result['same_frame']);self.assertEqual(result['pose_transform'],'identity')
            self.assertEqual(result['source_map'],'base');self.assertEqual(result['target_map'],'candidate')
    def test_root_name_or_complete_flag_without_structure_is_not_frame_evidence(self):
        for change in [lambda r:r.update(base_map='other'),lambda r:r.update(includes_complete_base=False),
                       lambda r:r.pop('structural_alignment'),lambda r:r['structural_alignment'].update(structural_overlap=.1)]:
            report=candidate_report();change(report);self.manifest(report=report)
            self.assertFalse(self.service.activation_context('candidate')['same_frame'])
        self.assertFalse(self.service.activation_context('unrelated')['same_frame'])
    def test_apply_rejects_changed_active_or_candidate_manifest_before_side_effect(self):
        self.manifest();proof=self.service.activation_context('candidate')
        with self.assertRaisesRegex(ValueError,'活动地图'):self.service.dispatch('/maps/apply',dict(name='candidate',expected_active='unrelated'))
        self.assertEqual((self.maps/'active').resolve().name,'base')
        self.manifest(report=dict(candidate_report(),created=11.))
        with self.assertRaisesRegex(ValueError,'版本'):self.service.dispatch('/maps/apply',dict(name='candidate',expected_active='base',expected_project_sha256=proof['project_sha256']))
        self.assertEqual((self.maps/'active').resolve().name,'base')
    def test_privileged_apply_rechecks_context_under_catalog_lock(self):
        self.manifest();proof=self.service.activation_context('candidate')
        with patch.object(root_actions,'run',return_value='') as run:
            with self.assertRaisesRegex(ValueError,'活动地图'):root_actions.execute(dict(action='apply',name='candidate',expected_active='other'),self.maps,Path(self.temp.name)/'data')
            run.assert_not_called()
            result=root_actions.execute(dict(action='apply',name='candidate',expected_active='base',expected_project_sha256=proof['project_sha256']),self.maps,Path(self.temp.name)/'data')
            self.assertEqual(run.call_count,2);self.assertGreater(result['localization_restart_completed_at'],0)


class ActivationLocalizationTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def live(self,same_frame=True,valid_pose=True):
        app=self.app;app.demo=False;app.link=cases.FakeLink();self.link=app.link
        self.pose=dict(PosX=1.25,PosY=2.5,PosZ=.1,Yaw=.3,Location=0 if valid_pose else 1)
        self.link.put(1007,2,self.pose.copy());self.link.put(1002,4,dict(MotionStatus=dict(LinearX=0,LinearY=0,OmegaZ=0)))
        self.active='base';self.calls=[];self.restart=None;self.after_apply=None
        self.proof=dict(protocol=1,source_map='base',target_map='candidate',same_frame=same_frame,project_sha256='synthetic-manifest',pose_transform='identity')
        self.recovered=False
        original=self.link.command
        def command(t,c,items,label):
            self.assertEqual((t,c),(2101,1),'activation may only initialize localization')
            result=original(t,c,items,label)
            self.link.put(1007,2,dict(items,Location=0));self.link.records[1007,2]['monotonic_at']=time.monotonic()
            self.recovered=True;return result
        self.link.command=command
        def status():return dict(localization_quality=dict(status=0,normal_status=0,age=0.),
            matching_quality=dict(error=.03,inlier_ratio=.98,age=0.),odom=dict(received_at=time.time()))
        app.localizer.status=Mock(side_effect=status)
        def recover(progress,check=None,verify_current=False):
            if check:check()
            self.link.put(1007,2,dict(self.pose,Location=0));self.link.records[1007,2]['monotonic_at']=time.monotonic()
            self.recovered=True;return dict(localized=True,method='synthetic_geometric_recovery')
        app.localizer.recover=Mock(side_effect=recover)
        def remote(path,req=None,timeout=120,context=None):
            self.calls.append(path)
            if path=='/maps/list':return dict(active=self.active,catalog_integrity_protocol=1,maps=[])
            if path=='/maps/activation-context':return copy.deepcopy(self.proof)
            if path=='/maps/apply':
                self.assertEqual(req['expected_active'],'base');self.active=req['name'];self.restart=time.time()
                self.link.put(1007,2,dict(self.pose,Location=1))
                if self.after_apply:self.after_apply()
                return dict(ok=True,activation_context=copy.deepcopy(self.proof),localization_restart_completed_at=self.restart)
            if path=='/maps/cleanup-previous':return dict(ok=True,name='base')
            raise AssertionError(path)
        app.hub.remote_maps=Mock(side_effect=remote)
        app.hub.sync_map=Mock(return_value=dict(map=dict(id='candidate-cache',robot_map_name='candidate',verified_for_robot=True)))
        return app
    def wait(self,response):
        for _ in range(200):
            row=self.app.hub.operations.get(response['job_id'])
            if row['status'] in ['completed','failed']:return row
            time.sleep(.01)
        self.fail('activation job did not finish')
    def apply(self,**kwargs):return self.wait(self.app.hub.post('/api/maps/library/apply',dict(name='candidate',**kwargs)))
    def test_same_frame_candidate_initializes_unchanged_pose_and_waits_before_cleanup(self):
        app=self.live();result=self.apply(cleanup_previous=True,cleanup_base='base')
        self.assertEqual(result['status'],'completed',result);result=result['result']
        self.assertTrue(result['navigation_ready']);self.assertEqual(result['localization']['method'],'inherited_map_pose')
        self.assertEqual(len(self.link.sent),1);self.assertEqual(self.link.sent[0]['items'],{k:self.pose[k] for k in ['PosX','PosY','PosZ','Yaw']})
        app.localizer.recover.assert_not_called();self.assertEqual(self.calls[-1],'/maps/cleanup-previous')
        saved=json.loads((app.hub.data/'map-activation.json').read_text());self.assertEqual(saved['state'],'ready');self.assertEqual(saved['prior_pose']['source_map'],'base')
    def test_unrelated_map_or_missing_good_pose_uses_automatic_recovery_not_old_coordinates(self):
        for same,valid in [(False,True),(True,False)]:
            with self.subTest(same=same,valid=valid):
                app=self.live(same,valid);result=self.apply()
                self.assertEqual(result['status'],'completed',result);self.assertTrue(result['result']['navigation_ready'])
                self.assertEqual(self.link.sent,[]);app.localizer.recover.assert_called_once()
                self.assertTrue(app.localizer.recover.call_args.kwargs['verify_current'])
    def test_stop_after_apply_prevents_late_pose_initialization_and_retains_old_map(self):
        app=self.live();self.after_apply=app.localizer.cancel;result=self.apply(cleanup_previous=True,cleanup_base='base')
        self.assertEqual(result['status'],'completed',result);self.assertFalse(result['result']['navigation_ready'])
        self.assertEqual(self.link.sent,[]);app.localizer.recover.assert_not_called();self.assertNotIn('/maps/cleanup-previous',self.calls)
    def test_connection_change_after_apply_never_initializes_new_connection(self):
        app=self.live();old=self.link;self.after_apply=lambda:setattr(app,'link',cases.FakeLink())
        result=self.apply();self.assertEqual(result['status'],'failed');self.assertEqual(old.sent,[]);self.assertEqual(app.link.sent,[])
    def test_failed_localization_reports_applied_but_not_ready_and_never_cleans_base(self):
        app=self.live();app.localizer.recover.side_effect=ValueError('synthetic geometry unavailable')
        with patch.object(app.hub,'_wait_activation_localization',side_effect=ValueError('not converged')):
            result=self.apply(cleanup_previous=True,cleanup_base='base')
        self.assertEqual(result['status'],'completed',result);self.assertFalse(result['result']['navigation_ready'])
        self.assertTrue(result['result']['cleanup']['retained']);self.assertNotIn('/maps/cleanup-previous',self.calls)
        self.assertIn('尚未就绪',result['result']['message']);self.assertEqual(len(self.link.sent),2)
        app.localizer.recover.assert_not_called()
    def test_pre_restart_cached_normal_is_not_accepted_as_readiness(self):
        app=self.live();marker=dict(wall=time.time()-.01,monotonic=time.monotonic()-.01)
        self.link.records[1007,2]['at']=marker['wall']-.1
        with self.assertRaisesRegex(ValueError,'尚未就绪'):
            app.hub._wait_activation_localization('candidate',app.hub.context(),app.localizer.generation,marker,seconds=.01)
        self.assertEqual(self.link.sent,[])
    def test_new_aos_feedback_cannot_hide_old_nos_epoch(self):
        app=self.live();marker=dict(wall=time.time()-.01,monotonic=time.monotonic()-.01)
        self.link.put(1007,2,self.pose.copy());restart=time.time()
        app.localizer.status.return_value=None
        app.localizer.status.side_effect=lambda:dict(localization_quality=dict(status=0,normal_status=0,age=0.),
            matching_quality=dict(error=.02,inlier_ratio=.98,age=0.),odom=dict(received_at=restart-1))
        with self.assertRaisesRegex(ValueError,'尚未就绪'):
            app.hub._wait_activation_localization('candidate',app.hub.context(),app.localizer.generation,marker,restart_at=restart,seconds=.01)
        self.assertEqual(self.link.sent,[])


if __name__=='__main__':unittest.main()

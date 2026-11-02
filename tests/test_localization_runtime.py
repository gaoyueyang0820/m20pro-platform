"""Mock coordination tests: no real platform, network or robot requests."""
import base64,copy,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from localization_runtime import AutomaticLocalizer,OperationCancelled
from localization_recovery import saved_slam_pose
from test_localization_recovery import structure,saved_fixture


class SavedSlamCoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.build,self.sensor=saved_fixture()
        self.sensor.update(localization_quality=dict(status=5,normal_status=0,age=.1),
                           matching_quality=dict(error=.1,inlier_ratio=.8,age=.1))
        self.active='base';self.speed=0;self.sent=[];self.build_calls=0
        self.link=NS(ip='robot',record=self.record,command=lambda *a:self.sent.append(a))
        self.hub=NS(idle=Mock(),remote_maps=Mock(side_effect=self.remote))
        self.app=NS(demo=False,mapping=False,link=self.link,lock=threading.RLock(),
                    live_ready=Mock(),hub=self.hub,store=NS(path=Path(self.temp.name)/'workspace.json'),
                    proxy=Mock(side_effect=lambda *a,**kw:copy.deepcopy(self.sensor)))
        self.localizer=AutomaticLocalizer(self.app);self.localizer.wait_normal=Mock(return_value={})
        matrix=np.array(saved_slam_pose(self.build,self.sensor,self.active)['transform'])
        self.body=structure();reference=(self.body@matrix[:3,:3].T+matrix[:3,3]).astype('<f4')
        self.reference=dict(map_name='base',frame_id='map',points=base64.b64encode(reference.tobytes()).decode())
        self.clouds=[(dict(points=self.body,frame_id='base_link'),200.),
                     (dict(points=self.body+.001,frame_id='base_link'),201.)]
        self.localizer.body_cloud=Mock(side_effect=self.clouds)
    def tearDown(self):self.temp.cleanup()
    def record(self,t,c):
        if (t,c)==(1002,4):return dict(at=time.time(),items=dict(MotionStatus=dict(LinearX=self.speed,LinearY=0,OmegaZ=0)))
        return dict(at=time.time(),items=dict(Location=1))
    def remote(self,path,req=None,timeout=None,**kwargs):
        if path=='/maps/list':return dict(active=self.active)
        if path=='/maps/build/status':self.build_calls+=1;return copy.deepcopy(self.build)
        if path=='/maps/localization-reference':return self.reference
        if path=='/maps/localize-auto':return dict(accepted=True)
        raise AssertionError(path)
    def test_saved_trajectory_runs_before_local_search_and_only_initializes_pose(self):
        self.localizer.local_hint=Mock(side_effect=AssertionError('must prefer saved trajectory'))
        result=self.localizer.recover(Mock())
        self.assertEqual(result['method'],'saved_slam_full_transform')
        self.assertEqual(len(self.sent),1);self.assertEqual(self.sent[0][:2],(2101,1))
        self.assertEqual(set(self.sent[0][2]),{'PosX','PosY','PosZ','Yaw'})
        self.assertGreater(abs(result['verification']['validation_transform'][2][0]),.05)
        self.localizer.local_hint.assert_not_called();self.localizer.wait_normal.assert_called_once()
        self.assertEqual(self.localizer.body_cloud.call_args_list[0].kwargs['after'],110.)
        self.assertEqual(self.localizer.body_cloud.call_args_list[1].kwargs['after'],200.)
        self.assertFalse(any(c.args[0]=='/maps/localize-auto' for c in self.hub.remote_maps.call_args_list))
    def test_stale_saved_session_falls_back_without_sending_pose(self):
        self.sensor['slam_odom'].update(received_at=99,stamp=99)
        self.localizer.local_hint=Mock(return_value=dict(ok=True,method='fallback'))
        result=self.localizer.recover(Mock())
        self.assertEqual(result['method'],'fallback');self.localizer.local_hint.assert_called_once()
        self.assertEqual(self.sent,[]);self.localizer.body_cloud.assert_not_called()
    def test_changed_map_before_command_is_rejected(self):
        def clouds(after=None,**kwargs):
            packet=self.clouds.pop(0)
            if not self.clouds:self.active='different'
            return packet
        self.localizer.body_cloud.side_effect=clouds
        with self.assertRaises(ValueError):self.localizer.saved_slam_hint(Mock())
        self.assertEqual(self.sent,[]);self.localizer.wait_normal.assert_not_called()
    def test_new_session_before_command_is_rejected(self):
        original=self.remote
        def remote(path,req=None,timeout=None,**kwargs):
            if path=='/maps/build/status' and self.build_calls:
                result=copy.deepcopy(self.build);result['mapping']['name']='later-session';return result
            return original(path,req,timeout)
        self.hub.remote_maps.side_effect=remote
        with self.assertRaises(ValueError):self.localizer.saved_slam_hint(Mock())
        self.assertEqual(self.sent,[])
    def test_motion_or_failed_independent_frame_cannot_emit_initialization(self):
        self.speed=.2
        with self.assertRaisesRegex(ValueError,'停稳'):self.localizer.saved_slam_hint(Mock())
        self.assertEqual(self.sent,[]);self.localizer.body_cloud.assert_not_called()
        self.speed=0
        self.localizer.body_cloud.side_effect=[self.clouds[0],(dict(points=self.body+[1.2,0,0],frame_id='base_link'),201.)]
        with self.assertRaisesRegex(ValueError,'没有确认'):self.localizer.saved_slam_hint(Mock())
        self.assertEqual(self.sent,[])

    def test_relocation_cancel_does_not_initialize_or_start_vendor_fallback(self):
        def interrupted(*args,**kwargs):
            self.localizer.cancel();raise ValueError('a late geometry failure')
        self.localizer.saved_slam_hint=Mock(side_effect=interrupted)
        self.localizer.local_hint=Mock()
        with self.assertRaises(OperationCancelled):self.localizer.recover(Mock())
        self.localizer.local_hint.assert_not_called();self.assertEqual(self.sent,[])
        self.assertFalse(any(c.args[0]=='/maps/localize-auto' for c in self.hub.remote_maps.call_args_list))

    def test_late_success_after_cancel_is_not_reported_as_current(self):
        def interrupted(*args,**kwargs):
            self.localizer.cancel();return dict(ok=True,localized=True)
        self.localizer.saved_slam_hint=Mock(side_effect=interrupted)
        with self.assertRaises(OperationCancelled):self.localizer.recover(Mock())
        self.assertEqual(self.sent,[])

    def test_connection_switch_before_vendor_fallback_never_targets_new_robot(self):
        def switched(*args,**kwargs):
            self.app.link=NS(ip='different',record=self.record,command=Mock())
            raise ValueError('old scan not matched')
        self.localizer.saved_slam_hint=Mock(side_effect=switched)
        self.localizer.local_hint=Mock()
        with self.assertRaises(OperationCancelled):self.localizer.recover(Mock())
        self.localizer.local_hint.assert_not_called();self.assertEqual(self.sent,[])
        self.assertFalse(any(c.args[0]=='/maps/localize-auto' for c in self.hub.remote_maps.call_args_list))


if __name__=='__main__':unittest.main()

# coding: utf-8
"""Offline behavior tests: no robot, network fetch or physical output."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch,MagicMock
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_workflows import await_job
import m20_platform as web
from m20.platform_core import Store
from sdk_agent import SDKAgent,ros_input_source
from m20.motion_catalog import catalog

class UsabilityTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.app=web.Platform(Store(self.root/'workspace.json'))
    def tearDown(self):self.app.hub.close();self.tmp.cleanup()
    def test_all_documented_modes_and_gaits_dispatch(self):
        c=catalog();self.assertEqual({x['value'] for x in c['modes']},{0,1,2})
        self.assertEqual({x['value'] for x in c['gaits']},{4097,4098,4099,12290,12291})
        for row in c['modes']:
            self.app.command(row['action']);self.assertEqual(self.app.basic['ControlUsageMode'],row['value'])
        for row in c['gaits']:
            self.app.command(row['action']);self.assertEqual(self.app.basic['Gait'],row['value'])
    def test_start_here_stops_driving_before_starting_scan(self):
        self.app.set_velocity(dict(forward=.1));r=await_job(self.app,self.app.hub.post('/api/maps/start-here',dict(name='new-survey')))
        self.assertEqual(self.app.velocity,[0.,0.,0.]);self.assertTrue(self.app.mapping);self.assertIsNone(self.app.hub.busy)
    def test_bad_map_name_does_not_stop_current_drive(self):
        self.app.set_velocity(dict(forward=.1))
        with self.assertRaises(ValueError):self.app.hub.post('/api/maps/start-here',dict(name='../bad'))
        self.assertEqual(self.app.velocity[0],.1)
    def test_existing_map_extension_is_an_explicit_new_map_plan(self):
        hub=self.app.hub;before=hub.remote_maps('/maps/list')
        result=hub.post('/api/maps/extension-plan',dict(base='demo-yard',name='larger-room'))
        self.assertEqual(result['plan']['strategy'],'new_coverage');self.assertEqual([m['name'] for m in hub.remote_maps('/maps/list')['maps']],[m['name'] for m in before['maps']])
        for request in [dict(base='demo-yard',name='demo-yard'),dict(base='demo-yard',name='larger-room',strategy='incremental'),dict(base='missing',name='larger-room')]:
            with self.assertRaises(ValueError):hub.post('/api/maps/extension-plan',request)
    def test_takeover_preserves_route_and_resumes_next_target(self):
        a=self.app;m=a.maps()[0];r=a.route_save(dict(name='tour',map_id=m['id'],points=[dict(x=0,y=0),dict(x=1,y=1)]))['route']
        a.start_route(dict(id=r['id']));a._advance();self.assertEqual(a.nav['index'],1)
        await_job(a,a.hub.post('/api/nav/manual',{}));self.assertEqual(a.nav['status'],'paused');self.assertEqual(a.basic['ControlUsageMode'],0)
        a.resume_route();self.assertEqual(a.nav['index'],1);self.assertEqual(a.basic['ControlUsageMode'],1)
    def test_map_switch_waits_until_collection_and_fusion_finish(self):
        a=self.app;a.mapping=True;a.hub.demo_maps.demo_build=dict(phase='collecting',in_progress=True,message='collecting',result_maps=[])
        with self.assertRaisesRegex(ValueError,'保存完成'):a.hub.post('/api/maps/library/apply',dict(name='demo-yard'))

    def test_sdk_connect_no_joint_calibration_workflow(self):
        self.app.sdk['calibration_verified']=False
        self.app.hub.post('/api/sdk/connect',{})
        self.assertTrue(self.app.hub.demo_policy['vendor_running']);self.assertFalse(self.app.sdk['armed'])
        self.app.hub.post('/api/sdk/action',dict(action='exit'))
        self.assertFalse(self.app.hub.demo_policy['vendor_running'])
    def test_failed_sdk_shutdown_does_not_mark_handset_exit_ready(self):
        a=self.app;a.demo=False;a.hub.sdk_mode=dict(state='sdk',source='operator_confirmed')
        a.hub.connections.value['targets']['sdk'].update(deployed=True,enabled=True)
        with patch.object(a,'stop'),patch.object(a.hub,'proxy',side_effect=ValueError('still running')):
            with self.assertRaises(ValueError):a.hub.sdk_action(dict(action='exit'))
        self.assertEqual(a.hub.sdk_mode['state'],'sdk')

    def test_sdk_setup_preserves_unrecognized_user_source(self):
        workspace=self.root/'sdk';source=workspace/'src/M20_sdk_deploy/main.cpp';source.parent.mkdir(parents=True);source.write_text('custom project')
        agent=SDKAgent(workspace=workspace,data=self.root/'agent')
        with patch.object(agent,'prepare_source') as prepare,patch.object(agent,'build',return_value={'ok':True}):result=agent.setup()
        self.assertEqual(source.read_text(),'custom project');self.assertEqual(Path(result['workspace']),self.root/'agent/sdk-platform-ros2');prepare.assert_called_once()
        restored=SDKAgent(workspace=workspace,data=self.root/'agent')
        # In this mock build no directory is made, so an absent workspace is ignored.
        self.assertEqual(restored.workspace,workspace.resolve())
    def test_sdk_connect_attaches_fresh_existing_controller_once(self):
        agent=SDKAgent(workspace=self.root/'sdk',data=self.root/'agent')
        state=SimpleNamespace(status=lambda:dict(fresh=True,source='ros2'))
        with patch.object(agent,'ros',return_value=state),patch('sdk_agent.subprocess.Popen') as launch:
            self.assertTrue(agent.dispatch('/policy/connect',dict(mode_confirmed=True))['attached']);launch.assert_not_called()

    def test_attached_sdk_stop_waits_for_idle_without_killing_external_process(self):
        agent=SDKAgent(workspace=self.root/'sdk',data=self.root/'agent');agent.attached=True
        policy=MagicMock();policy.status.return_value=dict(fresh=True,current_state_id=0);agent.policy=policy
        with patch.object(agent,'stop') as stop:
            result=agent.dispatch('/policy/stop',{})
        stop.assert_called_once();policy.status.assert_called_once();self.assertFalse(agent.attached);self.assertIn('idle',result['message'])

class SDKInputTests(unittest.TestCase):
    def test_comments_spacing_namespaces_and_idempotency(self):
        comment='// RobotName::M20, RemoteCommandType::kKeyBoard\n'
        src=comment+'QwStateMachine(ns::RobotName :: M20, /* input */ ns::RemoteCommandType :: kKeyBoard);'
        updated,old=ros_input_source(src);self.assertEqual(old,'kKeyBoard');self.assertTrue(updated.startswith(comment));self.assertIn('ns::RemoteCommandType :: kRos2',updated)
        self.assertEqual(ros_input_source(updated),(updated,'kRos2'))
    def test_ambiguous_or_different_robot_is_not_rewritten(self):
        for src in ['RobotName::X30, RemoteCommandType::kKeyBoard','RobotName::M20,RemoteCommandType::kRos2;RobotName::M20,RemoteCommandType::kKeyBoard']:
            with self.assertRaises(ValueError):ros_input_source(src)

if __name__=='__main__':unittest.main()

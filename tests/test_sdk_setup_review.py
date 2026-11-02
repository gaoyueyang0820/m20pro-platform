"""Offline SDK preparation/connection tests; no ROS, network or real processes."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
import sdk_agent


class Clock:
    def __init__(self):self.now=100.
    def monotonic(self):return self.now
    def sleep(self,seconds):self.now+=seconds


class SDKSetupReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.workspace=self.root/'sdk'
        self.agent=sdk_agent.SDKAgent(workspace=self.workspace,data=self.root/'agent')
        self.clock=Clock()
        self.monotonic=patch.object(sdk_agent.time,'monotonic',self.clock.monotonic);self.monotonic.start();self.addCleanup(self.monotonic.stop)
        self.sleep=patch.object(sdk_agent.time,'sleep',self.clock.sleep);self.sleep.start();self.addCleanup(self.sleep.stop)
    def installed(self,mode='kRos2',merged=False):
        source=self.workspace/'src/M20_sdk_deploy/main.cpp';source.parent.mkdir(parents=True,exist_ok=True)
        source.write_text('QwStateMachine(RobotName::M20,RemoteCommandType::'+mode+');')
        setup=self.workspace/'install/setup.bash';setup.parent.mkdir(parents=True,exist_ok=True);setup.write_text('# synthetic')
        executable=self.workspace/('install/lib/m20_sdk_deploy/rl_deploy' if merged else 'install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy')
        executable.parent.mkdir(parents=True,exist_ok=True);executable.write_text('synthetic-not-executed');executable.chmod(0o755)
        os.utime(source,(10,10));os.utime(executable,(20,20))
        return source,executable
    def policy(self,status,publishers=0):
        return SimpleNamespace(status=status,node=SimpleNamespace(count_publishers=lambda topic:publishers))

    def test_ready_setup_is_noop_even_if_controller_running(self):
        self.installed();self.agent.vendor=SimpleNamespace(poll=lambda:None)
        with patch.object(self.agent,'build') as build,patch.object(self.agent,'prepare_source') as prepare:
            result=self.agent.dispatch('/policy/setup',{})
        self.assertFalse(result['changed']);self.assertFalse(result['reload_required'])
        build.assert_not_called();prepare.assert_not_called()
        self.assertFalse(self.agent.selection.exists())

    def test_merged_install_is_also_recognized(self):
        self.installed(merged=True)
        self.assertTrue(self.agent.environment_status()['ready'])

    def test_setup_script_without_executable_is_not_ready(self):
        _,executable=self.installed();executable.unlink()
        self.assertFalse(self.agent.environment_status()['ready'])
        with patch.object(self.agent,'build',return_value=dict(ok=True)) as build:
            result=self.agent.setup()
        build.assert_called_once_with(dict(confirmed=True))
        self.assertTrue(result['changed']);self.assertTrue(result['reload_required'])

    def test_old_binary_or_changed_input_requires_build(self):
        source,executable=self.installed();os.utime(source,(30,30))
        self.assertFalse(self.agent.environment_status()['ready'])
        os.utime(source,(10,10));source.write_text('QwStateMachine(RobotName::M20,RemoteCommandType::kKeyBoard);')
        os.utime(source,(10,10))
        self.assertFalse(self.agent.environment_status()['ready'])

    def test_nonexecutable_controller_is_not_ready(self):
        _,executable=self.installed();executable.chmod(0o644)
        self.assertFalse(self.agent.environment_status()['ready'])

    def test_failed_build_cannot_be_mistaken_for_ready_overlay(self):
        source,executable=self.installed(mode='kKeyBoard')
        before=executable.read_bytes()
        failure=SimpleNamespace(returncode=1,stderr='synthetic compile error',stdout='')
        with patch.object(sdk_agent.subprocess,'run',return_value=failure):
            with self.assertRaisesRegex(ValueError,'编译失败'):self.agent.setup()
        self.assertIn('kRos2',source.read_text())
        self.assertEqual(executable.read_bytes(),before)
        self.assertFalse(self.agent.environment_status()['ready'])
        self.assertFalse(self.agent.selection.exists())

    def test_existing_controller_first_frame_delayed_attaches_without_start(self):
        self.installed()
        status=Mock(side_effect=lambda:dict(fresh=self.clock.now>=100.2,source='ros2'))
        policy=self.policy(status,publishers=1)
        with patch.object(self.agent,'ros',return_value=policy),patch.object(sdk_agent.subprocess,'Popen') as launch:
            result=self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        self.assertTrue(result['attached']);launch.assert_not_called()
        self.assertGreaterEqual(status.call_count,2)
        self.assertLessEqual(self.clock.now,100.3)

    def test_already_fresh_controller_attaches_immediately(self):
        policy=self.policy(lambda:dict(fresh=True,source='ros2'),publishers=1)
        with patch.object(self.agent,'ros',return_value=policy),patch.object(sdk_agent.subprocess,'Popen') as launch:
            result=self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        self.assertTrue(result['attached']);self.assertEqual(self.clock.now,100.)
        launch.assert_not_called()

    def test_no_controller_starts_once_after_bounded_discovery(self):
        self.installed();policy=self.policy(lambda:dict(fresh=False),publishers=0)
        with patch.object(self.agent,'ros',return_value=policy),patch.object(sdk_agent.subprocess,'Popen') as launch:
            result=self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        self.assertTrue(result['ok']);launch.assert_called_once()
        self.assertGreaterEqual(self.clock.now,101.5);self.assertLess(self.clock.now,101.6)

    def test_unknown_joint_publisher_is_not_overridden_after_timeout(self):
        self.installed();policy=self.policy(lambda:dict(fresh=False),publishers=1)
        with patch.object(self.agent,'ros',return_value=policy),patch.object(sdk_agent.subprocess,'Popen') as launch:
            with self.assertRaisesRegex(ValueError,'JOINTS_CMD'):
                self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        launch.assert_not_called();self.assertFalse(self.agent.attached)

    def test_stop_during_discovery_cannot_later_attach_or_start(self):
        self.installed();policy=self.policy(lambda:dict(fresh=False),publishers=0)
        def stopping_sleep(seconds):
            self.clock.sleep(seconds);self.agent.stop()
        with patch.object(self.agent,'ros',return_value=policy),patch.object(sdk_agent.time,'sleep',stopping_sleep),patch.object(sdk_agent.subprocess,'Popen') as launch:
            with self.assertRaisesRegex(ValueError,'取消'):
                self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        launch.assert_not_called();self.assertFalse(self.agent.attached)

    def test_handset_confirmation_and_exclusive_direct_control_remain_required(self):
        with patch.object(self.agent,'ros') as ros:
            with self.assertRaises(ValueError):self.agent.dispatch('/policy/connect',{})
            self.agent.worker=SimpleNamespace(poll=lambda:None)
            with self.assertRaisesRegex(ValueError,'单关节'):
                self.agent.dispatch('/policy/connect',dict(mode_confirmed=True))
        ros.assert_not_called()


if __name__=='__main__':unittest.main()

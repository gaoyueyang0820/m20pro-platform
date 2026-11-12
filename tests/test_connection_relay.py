# coding: utf-8
"""SSH transport and state regressions, isolated from robot hardware."""
import json
from pathlib import Path
import sys,tempfile,time,unittest
from unittest.mock import MagicMock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.connection_manager import Connections,target_config
class RelayTests(unittest.TestCase):
    def setUp(self):
        # Connection-policy tests do not invoke Windows account/ACL tools.
        self.acl=patch('m20.platform_host.private_file',lambda path:None);self.acl.start();self.addCleanup(self.acl.stop)
        self.executables=patch('m20.platform_host.ssh_executable',side_effect=lambda name:name);self.executables.start();self.addCleanup(self.executables.stop)
        self.tmp=tempfile.TemporaryDirectory();self.c=Connections(self.tmp.name)
        self.c.save(dict(aos_ip='172.20.10.14',targets=dict(nos=dict(host='10.21.31.106',enabled=True,via_aos=True))))
        self.c.known.write_text('172.20.10.14 ssh-ed25519 AAAA\n10.21.31.106 ssh-ed25519 AAAA\n')
    def tearDown(self):self.c.close();self.tmp.cleanup()
    def test_same_relay_used_for_tunnel_and_service_calls(self):
        for remote in [None,['/usr/bin/python3','-c','print(1)']]:
            argv,_=self.c.ssh_command('nos',remote=remote)
            proxy=next(x for x in argv if x.startswith('ProxyCommand='))
            self.assertIn('-W 10.21.31.106:22',proxy);self.assertIn('user@172.20.10.14',proxy);self.assertIn('StrictHostKeyChecking=yes',proxy)
            self.assertIn('user@10.21.31.106',argv)
    def test_changing_aos_reconnects_relays_without_marking_target_undeployed(self):
        c=self.c;c.value['targets']['nos']['deployed']=True
        proc=MagicMock();proc.poll.return_value=None;c.processes['nos']=proc
        c.save(dict(aos_ip='172.20.10.9'))
        proc.terminate.assert_called_once();self.assertTrue(c.value['targets']['nos']['deployed']);self.assertTrue(c.value['targets']['nos']['via_aos'])
    def test_running_ssh_without_listener_is_not_reported_as_established(self):
        proc=MagicMock();proc.poll.return_value=None;self.c.processes['nos']=proc
        self.c.states['nos']=dict(status='connecting',attempted=time.time())
        with patch('m20.connection_manager.socket.create_connection',side_effect=ConnectionRefusedError),patch('m20.connection_manager.urlopen') as request:self.c.poll_once()
        self.assertEqual(self.c.states['nos']['status'],'connecting');request.assert_not_called()
    def test_exited_ssh_reports_network_error(self):
        proc=MagicMock();proc.poll.return_value=255;self.c.processes['nos']=proc
        (self.c.ssh/'nos.log').write_text('connect timeout')
        self.c.states['nos']=dict(attempted=time.time())
        self.c.poll_once();self.assertEqual(self.c.states['nos']['status'],'unreachable');self.assertIn('connect timeout',self.c.states['nos']['message'])
    def test_relay_requires_existing_aos_host_identity(self):
        self.c.known.write_text('10.21.31.106 ssh-ed25519 AAAA\n')
        with self.assertRaisesRegex(ValueError,'AOS'):self.c.ssh_command('nos')
    def test_first_setup_scans_internal_host_through_aos(self):
        with patch.object(self.c,'scan_via_aos',return_value='10.21.31.106 ssh-ed25519 cHVibGljLWtleQ==\n') as scan,patch('m20.connection_manager.socket.create_connection') as direct:
            result=self.c.scan('nos')
        scan.assert_called_once();direct.assert_not_called();self.assertEqual(result['host'],'10.21.31.106')
    def test_sdk_and_training_do_not_accidentally_relay(self):
        for role in ['sdk','train']:
            with self.assertRaises(ValueError):target_config(role,dict(host='robot',via_aos=True))
    def test_nos_resident_address_is_explicit_and_remains_independent_from_wifi(self):
        self.assertNotIn('resident_aos_ip',self.c.public()['targets']['nos'])
        self.c.save(dict(targets=dict(nos=dict(resident_aos_ip='10.21.31.103'))))
        self.c.save(dict(aos_ip='172.20.10.9'))
        self.assertEqual(self.c.public()['targets']['nos']['resident_aos_ip'],'10.21.31.103')
        for value in ['127.0.0.1','0.0.0.0','8.8.8.8','robot']:
            with self.assertRaisesRegex(ValueError,'NOS 驻机导航'):
                self.c.save(dict(targets=dict(nos=dict(resident_aos_ip=value))))
        self.c.save(dict(targets=dict(nos=dict(resident_aos_ip=''))))
        self.assertNotIn('resident_aos_ip',self.c.public()['targets']['nos'])
if __name__=='__main__':unittest.main()

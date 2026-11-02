"""Synthetic profiles/host keys only; no network, SSH daemon or robot required."""
import base64
import contextlib
import io
import json
from pathlib import Path
import shlex
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock,patch
from connection_manager import Connections,target_config
from ssh_identity import known_keys,write_alias
from windows_ssh import bind_host_identity


def public_key(seed=1):
    # SSH wire-format public key with deterministic synthetic bytes; no private key.
    algorithm=b'ssh-ed25519';raw=struct.pack('>I',len(algorithm))+algorithm+struct.pack('>I',32)+bytes([seed])*32
    return base64.b64encode(raw).decode()


class ConnectionRoamingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.acl=patch('platform_host.private_file',lambda path:None);self.acl.start()
        self.executables=patch('platform_host.ssh_executable',side_effect=lambda name:name);self.executables.start()
        c=Connections(self.root)
        c.value.update(aos_ip='192.0.2.10',aos_ssh_port=22,aos_ssh_user='user')
        c.value['targets']['sdk']=target_config('sdk',dict(host='192.0.2.10',enabled=True),{})
        c.value['targets']['sdk']['deployed']=True
        for role,host in [('nos','nos.fixture'),('gos','gos.fixture')]:
            c.value['targets'][role]=target_config(role,dict(host=host,enabled=True,via_aos=True),{})
            c.value['targets'][role]['deployed']=True
        c.known.write_text('192.0.2.10 ssh-ed25519 '+public_key()+'\nnos.fixture ssh-ed25519 '+public_key(2)+'\ngos.fixture ssh-ed25519 '+public_key(3)+'\n')
        c.persist();c.close();self.c=Connections(self.root)
    def tearDown(self):self.c.close();self.executables.stop();self.acl.stop();self.temp.cleanup()
    def test_network_move_keeps_identity_and_service_deployment(self):
        c=self.c;alias=c.value['aos_identity_alias'];before=c.known.read_text()
        with patch('connection_manager.subprocess.run',side_effect=AssertionError('save must not scan network')):
            c.save(dict(aos_ip='198.51.100.7'))
        self.assertEqual(c.value['aos_identity_alias'],alias);self.assertEqual(c.known.read_text(),before)
        self.assertEqual(c.value['targets']['sdk']['host'],'198.51.100.7');self.assertTrue(c.value['targets']['sdk']['deployed'])
        for role in ['nos','gos','sdk']:
            args,_=c.ssh_command(role);text=' '.join(args)
            self.assertIn('HostKeyAlias='+alias,text);self.assertIn('StrictHostKeyChecking=yes',text)
            self.assertIn('198.51.100.7',text);self.assertNotIn('192.0.2.10',text)
    def test_restart_keeps_pin_even_without_a_record_for_the_new_ip(self):
        self.c.save(dict(aos_ip='198.51.100.8'));expected=self.c.value['aos_identity_alias']
        restored=Connections(self.root)
        try:
            self.assertEqual(restored.value['aos_identity_alias'],expected)
            self.assertTrue(restored.public()['aos_identity_ready'])
            self.assertEqual(known_keys(restored.known,expected),[('ssh-ed25519',public_key())])
        finally:restored.close()
    def test_unrelated_new_address_key_cannot_replace_expected_identity(self):
        alias=self.c.value['aos_identity_alias']
        with self.c.known.open('a') as f:f.write('198.51.100.9 ssh-ed25519 '+public_key(9)+'\n')
        self.c.save(dict(aos_ip='198.51.100.9'));args,_=self.c.aos_ssh()
        self.assertIn('HostKeyAlias='+alias,args)
        self.assertEqual(known_keys(self.c.known,alias),[('ssh-ed25519',public_key())])
    def test_deleted_pin_never_silently_falls_back_to_new_address_trust(self):
        alias=self.c.value['aos_identity_alias'];lines=[s for s in self.c.known.read_text().splitlines() if not s.startswith(alias+' ')]
        self.c.known.write_text('\n'.join(lines)+'\n')
        with self.assertRaisesRegex(ValueError,'配对'):self.c.aos_ssh()
    def test_external_sdk_target_is_not_repointed(self):
        self.c.save(dict(targets={'sdk':dict(host='sdk.fixture',follows_aos=False)}))
        self.c.save(dict(aos_ip='198.51.100.7'))
        self.assertEqual(self.c.value['targets']['sdk']['host'],'sdk.fixture')
    def test_matching_legacy_endpoint_can_be_upgraded_after_a_previous_move(self):
        self.c.value.pop('aos_identity_alias');self.c.value['targets']['sdk']['follows_aos']=None
        self.c.value['aos_ip']='198.51.100.7'
        with self.c.known.open('a') as f:f.write('198.51.100.7 ssh-ed25519 '+public_key()+'\n')
        self.c.persist();restored=Connections(self.root)
        try:self.assertEqual(restored.value['targets']['sdk']['host'],'198.51.100.7');self.assertTrue(restored.value['targets']['sdk']['deployed'])
        finally:restored.close()
    def test_only_explicit_confirmation_can_rotate_pinned_identity(self):
        c=self.c;alias=c.value['aos_identity_alias'];c.scans['fixture']=dict(role='sdk',host='192.0.2.10',port=22,at=__import__('time').monotonic(),rows=[dict(line='192.0.2.10 ssh-ed25519 '+public_key(4))])
        with self.assertRaises(ValueError):c.trust(dict(scan_id='fixture',confirmed=False))
        self.assertEqual(known_keys(c.known,alias),[('ssh-ed25519',public_key())])
        c.trust(dict(scan_id='fixture',confirmed=True));self.assertEqual(known_keys(c.known,alias),[('ssh-ed25519',public_key(4))])
    def test_aos_remote_keyscan_program_is_self_contained(self):
        result=subprocess.CompletedProcess([],0,'fixture output','')
        with patch('connection_manager.subprocess.run',return_value=result) as run:
            self.c.scan_via_aos(self.c.value['targets']['nos'])
        command=shlex.split(run.call_args.args[0][-1]);program=command[command.index('-c')+1]
        stream=MagicMock();stream.recv.return_value=b'SSH-2.0-fixture'
        with patch('socket.create_connection',return_value=stream),patch('subprocess.run',return_value=result) as scan,patch('sys.stdin',io.StringIO(json.dumps({'host':'nos.fixture','port':22}))),contextlib.redirect_stdout(io.StringIO()):
            exec(program,{})
        self.assertEqual(scan.call_args.args[0][0],'ssh-keyscan')
    def test_missing_trust_does_not_bootstrap_unverified_keys(self):
        with tempfile.TemporaryDirectory() as path:
            c=Connections(path)
            try:
                with self.assertRaisesRegex(ValueError,'配对'):c.aos_ssh()
                self.assertFalse(c.known.exists())
            finally:c.close()
    def test_windows_password_path_binds_only_the_pinned_key(self):
        class Keys(dict):
            def lookup(self,name):return self.get(name)
            def add(self,name,kind,key):self.setdefault(name,{})[kind]=key
        keys=Keys({'pinned':{'ssh-ed25519':'expected'},'[198.51.100.7]:2222':{'ssh-rsa':'unrelated'}})
        client=MagicMock();client.get_host_keys.return_value=keys
        bind_host_identity(client,dict(host='198.51.100.7',ssh_port=2222,host_key_alias='pinned'))
        self.assertEqual(keys['[198.51.100.7]:2222'],{'ssh-ed25519':'expected'})
        with self.assertRaises(ValueError):bind_host_identity(client,dict(host='198.51.100.8',ssh_port=22,host_key_alias='missing'))


if __name__=='__main__':unittest.main()

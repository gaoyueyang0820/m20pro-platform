"""Synthetic review regressions. No robot, real credentials or motion."""
import ast
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import platform_host
from bridge.mapping_job import mapping_phase,empty_capture_confirmed


class PortableReviewTests(unittest.TestCase):
    def test_empty_capture_needs_positive_evidence_and_known_stopped_processes(self):
        start=dict(state='command_finished',started=100,name='survey',processes_alive=[])
        stop=dict(state='command_finished',started=130,ended=160,output='No poses loaded')
        self.assertEqual(mapping_phase(start,stop,[])['phase'],'empty')
        for bad_start,bad_stop in [(dict(start,processes_alive=[123]),stop),
                ({k:v for k,v in start.items() if k!='processes_alive'},stop),
                (start,dict(stop,output='saving')), (start,dict(stop,expected_start_request_id='different')),
                (start,dict(stop,started=90)),(dict(start,context={'workflow':'extension'}),stop)]:
            self.assertFalse(empty_capture_confirmed(bad_start,bad_stop))
        self.assertEqual(mapping_phase(start,stop,[dict(name='survey-123',modified=140,has_cloud=True,grids=[0])])['phase'],'saved')

    def test_unknown_or_missing_capture_does_not_become_empty(self):
        start=dict(state='command_finished',started=100,name='survey')
        stop=dict(state='command_finished',started=130,ended=160)
        self.assertEqual(mapping_phase(start,stop,[])['phase'],'saving')

    def test_acl_failure_blocks_writing_credentials(self):
        sid='S-1-5-21-100-200-300-400'
        identity=subprocess.CompletedProcess([],0,'"test","'+sid+'"','')
        for failure in [subprocess.CalledProcessError(1,['icacls']),subprocess.TimeoutExpired('icacls',5),OSError('unavailable')]:
            with patch.object(platform_host,'IS_WINDOWS',True),patch('platform_host.subprocess.run',side_effect=[identity,failure]):
                with self.assertRaisesRegex(ValueError,'私有权限'):platform_host.private_file(Path('synthetic.key'))

    def test_missing_sid_is_not_silently_accepted(self):
        with patch.object(platform_host,'IS_WINDOWS',True),patch('platform_host.subprocess.run',return_value=subprocess.CompletedProcess([],0,'"missing"','')):
            with self.assertRaises(ValueError):platform_host.private_file(Path('synthetic.key'))

    def test_proxy_keeps_the_selected_executable_and_quoted_paths(self):
        with patch.object(platform_host,'IS_WINDOWS',True):
            command=platform_host.local_command_line([r'C:\Program Files\OpenSSH\ssh.exe','-i',r'C:\User Data\test.key'])
            self.assertIn('C:/Program Files/OpenSSH/ssh.exe',command)
            self.assertIn('C:/User Data/test.key',command)
            self.assertIn('"',command)

    def test_diagnostic_context_never_records_command_secrets(self):
        root=Path(__file__).resolve().parents[1]
        node=next(n for n in ast.parse((root/'m20_platform.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='exc_location')
        scope={'Path':Path};exec(compile(ast.Module(body=[node],type_ignores=[]),'<diagnostic>','exec'),scope)
        for error in [ValueError('password=synthetic-secret'),subprocess.CalledProcessError(1,['ssh','token=synthetic-secret'])]:
            try:raise error
            except Exception as caught:message=scope['exc_location'](caught)
            self.assertNotIn('synthetic-secret',message);self.assertIn(' @ ',message)

    @unittest.skipUnless(os.name=='nt','Actual Windows OpenSSH parsing only')
    def test_windows_openssh_path_option_round_trip_without_network(self):
        ssh=platform_host.ssh_executable('ssh')
        with tempfile.TemporaryDirectory(prefix='m20 SSH 空格 ') as folder:
            known=Path(folder)/'known hosts'
            option=platform_host.ssh_path_option('UserKnownHostsFile',known)
            result=subprocess.run([ssh,'-G','-F','none','-o',option,'synthetic.invalid'],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            line=next(line for line in result.stdout.splitlines() if line.startswith('userknownhostsfile '))
            self.assertIn(str(known).replace('\\','/'),line)

    @unittest.skipUnless(os.name=='nt','Actual Windows ProxyCommand execution only')
    def test_windows_proxy_executes_absolute_helper_and_preserves_arguments(self):
        with tempfile.TemporaryDirectory(prefix='m20 proxy 空格 ') as folder:
            helper=Path(folder)/'synthetic helper.py'
            helper.write_text('import json,sys\nsys.stderr.write("M20_PROXY_ARGS:"+json.dumps(sys.argv[1:])+"\\n")\n',encoding='utf-8')
            option=platform_host.ssh_path_option('UserKnownHostsFile',Path(folder)/'known hosts')
            proxy=platform_host.local_command_line([sys.executable,str(helper),option])
            result=subprocess.run([platform_host.ssh_executable('ssh'),'-F','none','-o','ProxyCommand='+proxy,
                '-o','BatchMode=yes','-o','PubkeyAuthentication=no','-o','IdentityAgent=none','-o','ConnectTimeout=3','127.0.0.1'],
                capture_output=True,text=True,timeout=10)
            self.assertNotEqual(result.returncode,0)  # The local helper is not an SSH server.
            line=next((x for x in result.stderr.splitlines() if x.startswith('M20_PROXY_ARGS:')),None)
            self.assertIsNotNone(line,result.stderr)
            self.assertEqual(json.loads(line.split(':',1)[1]),[option])


if __name__=='__main__':unittest.main()

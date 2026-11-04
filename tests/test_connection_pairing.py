"""First-pairing and deployment flow with synthetic keys and fully mocked I/O."""
import base64
import json
import io
import contextlib
import shlex
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock,patch
from urllib.error import HTTPError,URLError

from connection_manager import Connections,target_config
from ssh_identity import known_keys,endpoint_name


def synthetic_key(seed):
    kind=b'ssh-ed25519'
    raw=struct.pack('>I',len(kind))+kind+struct.pack('>I',32)+bytes([seed])*32
    return base64.b64encode(raw).decode()


class PairingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.events=[]
        self.patches=[patch('platform_host.private_file',lambda path:None),
                      patch('platform_host.ssh_executable',side_effect=lambda name:name),
                      patch('connection_manager.platform_host.IS_WINDOWS',False),
                      patch('connection_manager.socket.create_connection'),
                      patch('connection_manager.subprocess.run',side_effect=self.fake_run),
                      patch('connection_manager.subprocess.Popen',side_effect=AssertionError('No real process allowed')),
                      patch('connection_manager.urlopen',side_effect=AssertionError('No HTTP allowed'))]
        self.mocks=[p.start() for p in self.patches]
        for p in self.patches:self.addCleanup(p.stop)
        stream=self.mocks[3].return_value.__enter__.return_value
        stream.recv.return_value=b'SSH-2.0-synthetic-test'
        self.c=Connections(self.root,log=lambda *args:self.events.append(('log',args)))
        self.addCleanup(self.c.close)
        self.c.save(dict(aos_ip='192.0.2.10',targets=dict(
            sdk=dict(host='192.0.2.10',sdk_workspace='/home/user/M20_sdk_deploy',follows_aos=True),
            nos=dict(host='10.21.31.106',via_aos=True),gos=dict(host='10.21.31.104',via_aos=True))))
        self.keys={'192.0.2.10':synthetic_key(1),'198.51.100.7':synthetic_key(1),
                   '10.21.31.106':synthetic_key(2),'10.21.31.104':synthetic_key(3)}
        self.c.key.write_text('synthetic private key, not usable')
        self.c.key.with_suffix('.pub').write_text('ssh-ed25519 '+synthetic_key(4)+' synthetic-client')
        self.require_bootstrap_password=False
        self.secret='synthetic-password-do-not-store'
        self.admin='synthetic-admin-do-not-store'
    def key_line(self,host,port=22):return endpoint_name(host,port)+' ssh-ed25519 '+self.keys[host]+'\n'
    def fake_run(self,argv,**kwargs):
        tool=Path(argv[0]).name
        if tool=='ssh-keyscan':
            host=argv[-1];port=int(argv[argv.index('-p')+1]);self.events.append(('scan',host))
            return subprocess.CompletedProcess(argv,0,self.key_line(host,port),'')
        if tool!='ssh':raise AssertionError('Unexpected subprocess: '+tool)
        body=json.loads(kwargs.get('input') or '{}')
        password=kwargs.get('env',{}).get('M20_SSH_PASSWORD')
        self.assertNotIn(self.secret,' '.join(argv));self.assertNotIn(self.admin,' '.join(argv))
        if set(body)=={'host','port'}:
            self.events.append(('scan-via-aos',body['host']))
            self.assertIsNone(password)
            return subprocess.CompletedProcess(argv,0,self.key_line(body['host'],body['port']),'')
        if set(body)=={'key'}:
            self.events.append(('bootstrap-password' if password else 'bootstrap-key',argv[-2]))
            if self.require_bootstrap_password and not password:
                return subprocess.CompletedProcess(argv,255,'','Permission denied (publickey).')
            return subprocess.CompletedProcess(argv,0,json.dumps({'ok':True}),'')
        if 'bundle' in body:
            self.events.append(('install',body['role'],argv[-2],body['target']['host']))
            return subprocess.CompletedProcess(argv,0,json.dumps({'ok':True,'installed':True}),'')
        raise AssertionError('Unexpected remote payload')
    def deploy(self,role='nos',**kwargs):
        req=dict(role=role,confirmed=True,**kwargs)
        with patch.object(self.c,'bundle',return_value='synthetic-bundle'),patch.object(self.c,'start',side_effect=lambda role:self.events.append(('start',role))):
            return self.c.deploy(req,lambda *args:None)
    def assert_no_saved_password(self):
        for path in self.root.rglob('*'):
            if path.is_file():
                text=path.read_text(errors='replace')
                self.assertNotIn(self.secret,text,str(path));self.assertNotIn(self.admin,text,str(path))
        self.assertNotIn(self.secret,json.dumps(self.c.public()))
        self.assertNotIn(self.admin,json.dumps(self.c.public()))

    def test_first_aos_pair_does_not_require_manual_fingerprint_step(self):
        result=self.c.pair('aos')
        self.assertTrue(result['paired']);self.assertTrue(self.c.public()['aos_identity_ready'])
        alias=self.c.value['aos_identity_alias']
        self.assertEqual(known_keys(self.c.known,alias),[('ssh-ed25519',synthetic_key(1))])
        self.assertEqual(self.events,[('scan','192.0.2.10')])
        self.assertEqual(self.c.scans,{})

    def test_saved_pairing_reused_without_network_or_key_rotation(self):
        self.c.pair('aos');alias=self.c.value['aos_identity_alias'];before=self.c.known.read_text();self.events=[]
        self.keys['192.0.2.10']=synthetic_key(9)
        result=self.c.pair('aos')
        self.assertTrue(result['already_paired']);self.assertEqual(self.events,[])
        self.assertEqual(self.c.known.read_text(),before)
        args,_=self.c.aos_ssh();self.assertIn('HostKeyAlias='+alias,args);self.assertIn('StrictHostKeyChecking=yes',args)

    def test_explicit_replace_rotates_the_selected_device_pin(self):
        self.c.pair('aos');alias=self.c.value['aos_identity_alias']
        self.keys['192.0.2.10']=synthetic_key(9)
        self.c.pair('aos',replace=True)
        self.assertEqual(known_keys(self.c.known,alias),[('ssh-ed25519',synthetic_key(9))])

    def test_missing_existing_pin_requires_explicit_replace(self):
        self.c.pair('aos');alias=self.c.value['aos_identity_alias']
        self.c.known.write_text('\n'.join(line for line in self.c.known.read_text().splitlines() if not line.startswith(alias+' '))+'\n')
        self.keys['192.0.2.10']=synthetic_key(9);before=self.c.known.read_text()
        with self.assertRaisesRegex(ValueError,'配对|身份|记录'):
            self.c.pair('aos')
        self.assertEqual(self.c.known.read_text(),before)

    def test_aos_remote_uses_the_aos_target_python_without_a_role_entry(self):
        self.c.pair('aos')
        self.assertTrue(self.c.remote('aos','synthetic program',{'key':'synthetic-client-key'})['ok'])
        self.assertEqual(self.events[-1][0],'bootstrap-key')
        self.assertNotIn('aos',self.c.value['targets'])

    def test_nos_first_deploy_bootstraps_aos_before_internal_scan(self):
        self.require_bootstrap_password=True
        result=self.deploy(password=self.secret,admin_password=self.admin)
        self.assertTrue(result['installed'])
        self.assertEqual([event[0] for event in self.events if event[0]!='log'],
                         ['scan','bootstrap-key','bootstrap-password','scan-via-aos','install','start'])
        self.assertTrue(self.c.value['targets']['nos']['deployed']);self.assertTrue(self.c.value['targets']['nos']['enabled'])
        self.assert_no_saved_password()

    def test_known_aos_key_is_tried_before_falling_back_to_password(self):
        self.c.pair('aos');self.events=[];self.require_bootstrap_password=False
        self.deploy(password=self.secret)
        self.assertIn('bootstrap-key',[event[0] for event in self.events])
        self.assertNotIn('bootstrap-password',[event[0] for event in self.events])
        self.assert_no_saved_password()

    def test_separate_aos_password_is_used_only_for_bootstrap(self):
        self.require_bootstrap_password=True
        aos_secret='synthetic-aos-only'
        original=self.fake_run;passwords=[]
        def inspect(argv,**kwargs):
            if Path(argv[0]).name=='ssh':passwords.append((json.loads(kwargs.get('input') or '{}'),kwargs.get('env',{}).get('M20_SSH_PASSWORD')))
            return original(argv,**kwargs)
        with patch('connection_manager.subprocess.run',side_effect=inspect):self.deploy(password=self.secret,aos_password=aos_secret)
        self.assertEqual([password for body,password in passwords if set(body)=={'key'}],[None,aos_secret])
        self.assertEqual([password for body,password in passwords if 'bundle' in body],[self.secret])
        self.assertNotIn(aos_secret,self.c.path.read_text())

    def test_missing_password_stops_before_internal_host_scan_or_install(self):
        self.require_bootstrap_password=True
        with self.assertRaisesRegex(ValueError,'密码'):self.deploy()
        self.assertEqual([event[0] for event in self.events],['scan','bootstrap-key'])
        self.assertFalse(self.c.value['targets']['nos']['deployed'])

    def test_direct_sdk_deploy_does_not_bootstrap_unrelated_hosts(self):
        self.deploy(role='sdk',password=self.secret)
        self.assertEqual([event[0] for event in self.events if event[0]!='log'],['scan','install','start'])
        self.assertTrue(self.c.public()['aos_identity_ready'])
        self.assert_no_saved_password()

    def test_network_move_keeps_three_deployments_and_pins(self):
        self.c.pair('aos');self.c.pair('nos');self.c.pair('gos')
        for role in ['sdk','nos','gos']:self.c.value['targets'][role].update(deployed=True,enabled=True)
        self.c.persist();before=self.c.known.read_text();self.events=[]
        self.c.save(dict(aos_ip='198.51.100.7'))
        self.assertEqual(self.c.known.read_text(),before);self.assertEqual(self.events,[])
        for role in ['sdk','nos','gos']:
            target=self.c.value['targets'][role];self.assertTrue(target['deployed']);self.assertTrue(target['enabled'])
            args,_=self.c.ssh_command(role);self.assertIn('198.51.100.7',' '.join(args))
        self.assertEqual(self.c.value['targets']['sdk']['host'],'198.51.100.7')
        restored=Connections(self.root)
        try:
            for role in ['sdk','nos','gos']:self.assertTrue(restored.value['targets'][role]['deployed'])
            self.assertTrue(restored.public()['aos_identity_ready'])
        finally:restored.close()

    def test_pair_rejects_address_change_during_scan(self):
        real_scan=self.c.scan
        def changed(role,progress):
            result=real_scan(role,progress);self.c.save(dict(aos_ip='198.51.100.7'));return result
        with patch.object(self.c,'scan',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'地址|变化'):self.c.pair('aos')
        self.assertFalse(self.c.public()['aos_identity_ready'])

    def test_deploy_does_not_install_using_new_target_after_configuration_changes(self):
        original=self.fake_run
        def changed(argv,**kwargs):
            result=original(argv,**kwargs)
            body=json.loads(kwargs.get('input') or '{}')
            if Path(argv[0]).name=='ssh' and set(body)=={'key'}:
                self.c.save(dict(aos_ip='198.51.100.7'))
            return result
        with patch('connection_manager.subprocess.run',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'变化|取消|配置'):
                self.deploy()
        self.assertNotIn('install',[event[0] for event in self.events])
        self.assertFalse(self.c.value['targets']['nos']['deployed'])

    def test_finished_install_does_not_mark_new_network_profile_deployed(self):
        original=self.fake_run
        def changed(argv,**kwargs):
            result=original(argv,**kwargs)
            if Path(argv[0]).name=='ssh' and 'bundle' in json.loads(kwargs.get('input') or '{}'):
                self.c.save(dict(aos_ip='198.51.100.7'))
            return result
        with patch('connection_manager.subprocess.run',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'更新|变化|状态'):self.deploy(role='sdk')
        self.assertFalse(self.c.value['targets']['sdk']['deployed'])
        self.assertNotIn('start',[event[0] for event in self.events])

    def test_bootstrap_program_preserves_existing_keys_and_is_idempotent(self):
        programs=[];original=self.fake_run
        def capture(argv,**kwargs):
            body=json.loads(kwargs.get('input') or '{}')
            if Path(argv[0]).name=='ssh' and set(body)=={'key'}:
                words=shlex.split(argv[-1]);programs.append(words[words.index('-c')+1])
            return original(argv,**kwargs)
        with patch('connection_manager.subprocess.run',side_effect=capture):self.deploy()
        home=self.root/'synthetic-remote-home';ssh=home/'.ssh';ssh.mkdir(parents=True)
        authorized=ssh/'authorized_keys';authorized.write_text('existing-synthetic-key')
        payload=json.dumps(dict(key='new-synthetic-key'))
        for _ in range(2):
            with patch('pathlib.Path.home',return_value=home),patch('sys.stdin',io.StringIO(payload)),contextlib.redirect_stdout(io.StringIO()):
                exec(compile(programs[0],'<bootstrap-fixture>','exec'),{})
        self.assertEqual(authorized.read_text(),'existing-synthetic-key\nnew-synthetic-key\n')

    def online_tunnel(self,deployed=True,roles=('nos',)):
        for role in roles:
            self.c.value['targets'][role].update(enabled=True,deployed=deployed)
            process=MagicMock();process.poll.return_value=None;self.c.processes[role]=process
        self.c.persist()

    def test_poll_recovers_installed_service_once_per_minute_without_redeploy(self):
        self.online_tunnel();clock=[100.]
        def immediate_thread(*args,**kwargs):
            return type('ThreadFixture',(),{'start':lambda _:kwargs['target']()})()
        with patch('connection_manager.time.monotonic',side_effect=lambda:clock[0]),patch('connection_manager.threading.Thread',side_effect=immediate_thread),patch.object(self.c,'service',return_value=dict(ok=True)) as service,patch.object(self.c,'deploy') as deploy:
            self.c.poll_once();self.c.poll_once()
            self.assertEqual(service.call_count,1);self.assertEqual(service.call_args.args[0],dict(role='nos',action='start'))
            clock[0]=159.9;self.c.poll_once();self.assertEqual(service.call_count,1)
            clock[0]=160.1;self.c.poll_once();self.assertEqual(service.call_count,2)
            deploy.assert_not_called()
        self.assertEqual(self.c.states['nos']['status'],'waiting_service')

    def test_uninstalled_service_is_never_automatically_deployed_or_started(self):
        self.online_tunnel(deployed=False)
        with patch.object(self.c,'service') as service,patch.object(self.c,'deploy') as deploy,patch('connection_manager.threading.Thread') as thread:
            self.c.poll_once()
        service.assert_not_called();deploy.assert_not_called();thread.assert_not_called()

    def test_background_service_recovery_cannot_follow_connection_change(self):
        self.online_tunnel();pending=[]
        def queued_thread(*args,**kwargs):
            return type('ThreadFixture',(),{'start':lambda _:pending.append(kwargs['target'])})()
        with patch('connection_manager.threading.Thread',side_effect=queued_thread),patch.object(self.c,'service') as service:
            self.c.poll_once();self.assertEqual(len(pending),1)
            self.c.save(dict(aos_ip='198.51.100.7'))
            pending.pop()()
        service.assert_not_called()
        self.assertFalse(self.c.service_attempts['nos']['running'])

    def test_background_recovery_is_independent_for_nos_and_gos(self):
        self.online_tunnel(roles=('nos','gos'));pending=[]
        def queued_thread(*args,**kwargs):
            return type('ThreadFixture',(),{'start':lambda _:pending.append(kwargs['target'])})()
        with patch('connection_manager.threading.Thread',side_effect=queued_thread),patch.object(self.c,'service',return_value=dict(ok=True)) as service:
            self.c.poll_once();self.c.poll_once()
            self.assertEqual(len(pending),2)
            self.assertTrue(all(self.c.service_attempts[role]['running'] for role in ('nos','gos')))
            for run in pending:run()
        self.assertEqual({call.args[0]['role'] for call in service.call_args_list},{'nos','gos'})

    def test_recovery_failure_retains_backoff_without_claiming_ready(self):
        self.online_tunnel();clock=[100.]
        def immediate_thread(*args,**kwargs):
            return type('ThreadFixture',(),{'start':lambda _:kwargs['target']()})()
        with patch('connection_manager.time.monotonic',side_effect=lambda:clock[0]),patch('connection_manager.threading.Thread',side_effect=immediate_thread),patch.object(self.c,'service',side_effect=ValueError('synthetic service error')) as service:
            self.c.poll_once();clock[0]=105.;self.c.poll_once()
            self.assertEqual(service.call_count,1)
        self.assertFalse(self.c.service_attempts['nos']['running'])
        self.assertEqual(self.c.service_attempts['nos']['detail'],'synthetic service error')
        self.assertEqual(self.c.states['nos']['status'],'waiting_service')

    def test_health_probe_avoids_business_status(self):
        self.online_tunnel()
        def response(req,**kwargs):
            self.assertTrue(req.full_url.endswith('/health'))
            self.assertEqual(req.get_header('X-api-token'),self.c.value['targets']['nos']['token'])
            return io.BytesIO(b'{"ok":true,"health_protocol":1}')
        with patch('connection_manager.urlopen',side_effect=response) as request,patch.object(self.c,'recover_service') as recover:
            self.c.poll_once()
        self.assertEqual(request.call_count,1);recover.assert_not_called()
        self.assertEqual(self.c.states['nos']['status'],'ready')

    def test_old_agents_fall_back_only_on_explicit_unsupported_endpoint(self):
        for code,message in [(404,''),(405,''),(400,'NOS 接口或操作不存在'),(400,'GOS 只读代理不支持此操作'),(400,'SDK 接口不存在'),(400,'接口不存在')]:
            error=HTTPError('http://synthetic/health',code,'unsupported',{},io.BytesIO(json.dumps({'err':message}).encode()))
            with patch('connection_manager.urlopen',side_effect=[error,io.BytesIO(b'{"ok":true}')]) as request:
                self.c.probe_service('nos')
            self.assertTrue(request.call_args.args[0].full_url.endswith('/status'))

    def test_auth_server_and_transport_errors_never_fall_back(self):
        for error in [HTTPError('http://synthetic/health',code,'error',{},io.BytesIO(b'{"err":"failure"}')) for code in (400,401,403,500)]+[TimeoutError('timeout')]:
            with patch('connection_manager.urlopen',side_effect=error) as request:
                with self.assertRaises(Exception):self.c.probe_service('nos')
            self.assertEqual(request.call_count,1)

    def test_invalid_heartbeat_does_not_claim_online(self):
        for body in [b'{}',b'[]',b'{"ok":true}',b'{"ok":false,"health_protocol":1}']:
            with patch('connection_manager.urlopen',return_value=io.BytesIO(body)):
                with self.assertRaises(ValueError):self.c.probe_service('nos')

    def test_one_timeout_retries_without_starting_service_or_claiming_ready(self):
        self.online_tunnel();self.c.states['nos']={'status':'ready','checked':10}
        with patch('connection_manager.urlopen',side_effect=URLError(TimeoutError('synthetic timeout'))),patch.object(self.c,'recover_service') as recover:
            self.c.poll_once()
            self.assertEqual(self.c.states['nos']['status'],'connecting');recover.assert_not_called()
            self.assertNotIn('checked',self.c.states['nos'])
            self.c.poll_once()
            self.assertEqual(self.c.states['nos']['status'],'waiting_service');recover.assert_called_once_with('nos')
        with patch('connection_manager.urlopen',return_value=io.BytesIO(b'{"ok":true,"health_protocol":1}')):self.c.poll_once()
        self.assertEqual(self.c.states['nos']['status'],'ready');self.assertNotIn('timeouts',self.c.states['nos'])


class AgentHealthTests(unittest.TestCase):
    def request(self,headers=None,post=False):
        import bridge.http_agent as agent
        captured=[];dispatch=MagicMock(return_value={'business':True})
        class Server:
            def __init__(self,address,handler):self.handler=handler
            def serve_forever(self):
                h=self.handler.__new__(self.handler);h.path='/health';h.headers=headers or {}
                h.rfile=io.BytesIO(b'{}');h.reply=lambda obj,status=200:captured.append((obj,status))
                h.run_request(post=post)
            def server_close(self):pass
        with patch.dict('os.environ',{'M20_BRIDGE_TOKEN':'synthetic-token'}),patch.object(agent,'ThreadingHTTPServer',Server),patch.object(agent,'threading_is_main',return_value=False),patch('builtins.print'):
            agent.serve(0,dispatch)
        return captured[0],dispatch

    def test_authenticated_health_does_not_call_business_dispatch(self):
        (body,code),dispatch=self.request({'X-API-Token':'synthetic-token'})
        self.assertEqual((body,code),({'ok':True,'health_protocol':1},200));dispatch.assert_not_called()

    def test_health_still_requires_token_and_rejects_browser_origin(self):
        for headers,expected in [({},401),({'X-API-Token':'wrong'},401),({'X-API-Token':'synthetic-token','Origin':'http://example.invalid'},403)]:
            (body,code),dispatch=self.request(headers)
            self.assertEqual(code,expected);self.assertFalse(body['ok']);dispatch.assert_not_called()

    def test_post_does_not_bypass_business_dispatch(self):
        (body,code),dispatch=self.request({'X-API-Token':'synthetic-token','Content-Type':'application/json','Content-Length':'2'},post=True)
        self.assertEqual(body,{'business':True});dispatch.assert_called_once_with('/health',{})


if __name__=='__main__':unittest.main()

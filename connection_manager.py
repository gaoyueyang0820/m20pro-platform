"""Saved robot profiles, verified SSH setup, managed forwarding and reconnection."""
import base64
import copy
import hashlib
import io
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import socket
import subprocess
import sys
import threading
import time
import uuid
import zipfile
import platform_host
import robot_discovery
from ssh_identity import endpoint_name,known_keys,valid_alias,write_alias
from urllib.request import Request,urlopen

ROOT=Path(__file__).resolve().parent
ROLES=dict(nos=8093,gos=8090,sdk=8091,train=8092)


def host(value):
    if not isinstance(value,str) or not value or value.startswith('-') or len(value)>253:raise ValueError('主机地址无效')
    try:ipaddress.ip_address(value);return value
    except ValueError:
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]*',value):raise ValueError('主机地址只能是 IP 或域名')
        return value


def target_config(role,value,old=None):
    if role not in ROLES or not isinstance(value,dict):raise ValueError('连接目标无效')
    old=old or {}
    name=str(value.get('host',old.get('host',{'gos':'10.21.31.104','nos':'10.21.31.106'}.get(role,'')))).strip()
    if name:host(name)
    user=str(value.get('user',old.get('user','user'))).strip()
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]{0,63}',user):raise ValueError('SSH 用户名无效')
    via_aos=value.get('via_aos',old.get('via_aos',role in ['nos','gos'] and name.startswith('10.21.31.')))
    if not isinstance(via_aos,bool) or via_aos and role not in ['nos','gos']:raise ValueError('AOS 中转仅用于 NOS / GOS')
    result=dict(host=name,user=user,via_aos=via_aos,enabled=bool(value.get('enabled',old.get('enabled',False))),
                token=old.get('token') or secrets.token_urlsafe(32),deployed=old.get('deployed',False),
                python=str(value.get('python',old.get('python','/usr/bin/python3'))),
                train_root=str(value.get('train_root',old.get('train_root',''))),
                sdk_workspace=str(value.get('sdk_workspace',old.get('sdk_workspace',''))),
                sdk_admin=bool(value.get('sdk_admin',old.get('sdk_admin',False))),
                domain_id=value.get('domain_id',old.get('domain_id',0)),
                ros_setup=value.get('ros_setup',old.get('ros_setup',[])))
    for key,default in [('ssh_port',22),('local_port',ROLES[role]),('remote_port',ROLES[role])]:
        v=value.get(key,old.get(key,default))
        if not isinstance(v,int) or isinstance(v,bool) or not 1<=v<=65535:raise ValueError('端口需为 1–65535 的整数')
        result[key]=v
    if not result['python'].startswith('/') or any(c in result['python'] for c in '\n\r\x00'):raise ValueError('Python 路径需为绝对路径')
    for key in ['train_root','sdk_workspace']:
        if result[key] and (not result[key].startswith('/') or any(c in result[key] for c in '\n\r\x00')):
            raise ValueError('工程路径需为有效绝对路径')
    if not isinstance(result['domain_id'],int) or isinstance(result['domain_id'],bool) or not 0<=result['domain_id']<=232:
        raise ValueError('DDS domain 需为 0–232 的整数')
    if not isinstance(result['ros_setup'],list) or any(not isinstance(p,str) or not p.startswith('/') or '\n' in p for p in result['ros_setup']):
        raise ValueError('环境脚本需为绝对路径列表')
    if result['enabled'] and not name:raise ValueError('启用连接前请填写主机地址')
    if role=='sdk':
        follow=value.get('follows_aos',old.get('follows_aos'))
        if follow is not None and not isinstance(follow,bool):raise ValueError('SDK 地址跟随选项无效')
        result['follows_aos']=follow
    if (name,user,result['ssh_port'])!=(old.get('host'),old.get('user'),old.get('ssh_port')):result['deployed']=False
    return result


class Connections:
    def __init__(self,data,log=None):
        self.root=Path(data);self.root.mkdir(parents=True,exist_ok=True)
        self.path=self.root/'connections.json';self.ssh=self.root/'ssh';self.ssh.mkdir(mode=0o700,exist_ok=True)
        self.known=self.ssh/'known_hosts';self.key=self.ssh/'m20_ed25519'
        self.lock=threading.RLock();self.processes={};self.states={};self.scans={};self.service_attempts={};self.done=False
        self.log=log or (lambda *a:None)
        self.value=dict(aos_ip='10.21.31.103',aos_port=30000,aos_ssh_user='user',aos_ssh_port=22,auto_connect=False,sensor_source='nos',targets={})
        if self.path.exists():
            try:self.value.update(json.loads(self.path.read_text()))
            except (ValueError,OSError):raise ValueError('连接配置读取失败，请检查 data/connections.json')
        for role in ROLES:
            self.value['targets'][role]=target_config(role,self.value['targets'].get(role,{}),self.value['targets'].get(role,{}))
        changed=self._remember_aos_identity()
        sdk=self.value['targets']['sdk'];hop=self._aos_target()
        if sdk.get('follows_aos') is None and sdk.get('host'):
            same=(sdk['host'],sdk['ssh_port'],sdk['user'])==(hop['host'],hop['ssh_port'],hop['user'])
            # Upgrade an already-moved legacy profile only when both endpoints
            # have the same previously trusted key and account.
            shared=set(known_keys(self.known,endpoint_name(sdk['host'],sdk['ssh_port']))) & set(known_keys(self.known,hop.get('host_key_alias','')))
            sdk['follows_aos']=bool(same or shared and sdk['user']==hop['user'] and sdk['ssh_port']==hop['ssh_port'])
            if sdk['follows_aos']:sdk.update(host=hop['host'],user=hop['user'],ssh_port=hop['ssh_port'])
            changed=True
        if changed and self.path.exists():self.persist()
        self.askpass=self.ssh/'askpass.py'
        if not platform_host.IS_WINDOWS:
            self.askpass.write_text('#!'+sys.executable+'\nimport os,sys\nsys.stdout.write(os.environ.get("M20_SSH_PASSWORD", ""))\n')
            self.askpass.chmod(0o700)

    def persist(self):
        temp=self.path.with_suffix('.tmp');temp.write_text(json.dumps(self.value,ensure_ascii=False,indent=2));platform_host.private_file(temp);temp.replace(self.path)

    def save(self,req):
        with self.lock:
            ip=str(ipaddress.IPv4Address(req.get('aos_ip',self.value['aos_ip'])))
            port=req.get('aos_port',self.value['aos_port'])
            if not isinstance(port,int) or not 1<=port<=65535:raise ValueError('AOS UDP 端口无效')
            aos_user=req.get('aos_ssh_user',self.value.get('aos_ssh_user','user'))
            aos_ssh_port=req.get('aos_ssh_port',self.value.get('aos_ssh_port',22))
            if not isinstance(aos_user,str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]{0,63}',aos_user):raise ValueError('AOS SSH 用户名无效')
            if not isinstance(aos_ssh_port,int) or isinstance(aos_ssh_port,bool) or not 1<=aos_ssh_port<=65535:raise ValueError('AOS SSH 端口无效')
            relay_changed=(ip,aos_user,aos_ssh_port)!=(self.value['aos_ip'],self.value.get('aos_ssh_user','user'),self.value.get('aos_ssh_port',22))
            targets={role:target_config(role,req.get('targets',{}).get(role,self.value['targets'][role]),self.value['targets'][role]) for role in ROLES}
            ports=[t['local_port'] for t in targets.values() if t['enabled']]
            if len(ports)!=len(set(ports)):raise ValueError('不同目标不能使用相同的本地端口')
            source=req.get('sensor_source',self.value.get('sensor_source','nos'))
            if source not in ['gos','nos']:raise ValueError('ROS2 数据来源只能是 GOS 或 NOS')
            # Pin the old, explicitly trusted AOS before replacing its address.
            # This is local I/O only: no blocking keyscan while holding the lock.
            self._remember_aos_identity()
            sdk=targets['sdk'];old_sdk=self.value['targets']['sdk'];explicit=req.get('targets',{}).get('sdk',{})
            if sdk.get('follows_aos') is None:
                sdk['follows_aos']=(sdk['host'],sdk['ssh_port'],sdk['user']) in [
                    (self.value['aos_ip'],self.value.get('aos_ssh_port',22),self.value.get('aos_ssh_user','user')),
                    (ip,aos_ssh_port,aos_user)]
            if 'follows_aos' not in explicit and explicit.get('host') not in [None,old_sdk.get('host'),ip]:sdk['follows_aos']=False
            if sdk.get('follows_aos'):
                same_endpoint=(old_sdk['host'],old_sdk['ssh_port'],old_sdk['user'])==(ip,aos_ssh_port,aos_user)
                same_identity=same_endpoint or old_sdk.get('follows_aos') and self.value.get('aos_identity_alias')
                sdk.update(host=ip,user=aos_user,ssh_port=aos_ssh_port)
                sdk['deployed']=old_sdk.get('deployed',False) if same_identity else False
            for role,target in targets.items():
                if self.value['targets'][role]!=target or relay_changed and target.get('via_aos'):self._terminate(role)
            self.value.update(aos_ip=ip,aos_port=port,aos_ssh_user=aos_user,aos_ssh_port=aos_ssh_port,auto_connect=bool(req.get('auto_connect',False)),sensor_source=source,targets=targets)
            self.persist()
        return dict(ok=True,connections=self.public())

    def public(self):
        with self.lock:
            value=copy.deepcopy(self.value)
            value.pop('aos_identity_alias',None)
            value['aos_identity_ready']=bool(self._trusted(self._aos_target()))
            for role,target in value['targets'].items():
                target.pop('token',None)
                target['state']=copy.deepcopy(self.states.get(role,dict(status='idle',message='尚未连接')))
                target['trusted']=self._trusted(self._effective_target(role))
                target['url']='http://127.0.0.1:'+str(target['local_port'])
            value['key_ready']=self.key.is_file()
            return value

    def _trusted(self,target):
        if not target.get('host') or not self.known.exists():return False
        name=target.get('host_key_alias') or endpoint_name(target['host'],target['ssh_port'])
        return any(line.split(' ',1)[0]==name for line in self.known.read_text().splitlines() if line and not line.startswith('#'))

    def _aos_target(self):
        hop=dict(host=self.value['aos_ip'],ssh_port=self.value.get('aos_ssh_port',22),user=self.value.get('aos_ssh_user','user'))
        alias=self.value.get('aos_identity_alias')
        if alias:
            if not valid_alias(alias):raise ValueError('AOS 身份记录无效，请重新核对主机指纹')
            hop['host_key_alias']=alias
        return hop

    def _remember_aos_identity(self):
        if self.value.get('aos_identity_alias'):return False
        hop=self._aos_target();keys=known_keys(self.known,endpoint_name(hop['host'],hop['ssh_port']))
        if not keys:return False
        alias='m20-aos-'+uuid.uuid4().hex
        write_alias(self.known,alias,keys);self.value['aos_identity_alias']=alias
        return True

    def _effective_target(self,role):
        if role=='aos':return dict(self._aos_target(),python='/usr/bin/python3')
        target=dict(self.value['targets'][role])
        if role=='sdk' and target.get('follows_aos'):
            target.update(self._aos_target())
        return target

    def scan(self,role,progress=None):
        if role not in [*ROLES,'aos']:raise ValueError('无效连接目标')
        target=self._effective_target(role)
        name=host(target['host'])
        endpoint='%s:%s'%(name,target['ssh_port'])
        if progress:progress(.15,'检查 SSH 通信：'+endpoint)
        if target.get('via_aos'):
            output=self.scan_via_aos(target,progress)
        else:
            try:
                with socket.create_connection((name,target['ssh_port']),timeout=3) as connection:
                    connection.settimeout(2)
                    banner=connection.recv(256)
            except ConnectionRefusedError:
                raise ValueError(endpoint+' 拒绝 SSH 连接，请检查目标主机和端口是否正确、SSH 服务是否启动')
            except (TimeoutError,socket.timeout):
                raise ValueError('访问 '+endpoint+' 超时。请使用当前网络可达的目标主机地址；外部 Wi-Fi 或 DC-Client 联网不保证可直达 10.21.31.x 内网地址')
            except OSError as exc:
                raise ValueError('无法访问 '+endpoint+'：'+str(exc))
            if not banner.startswith(b'SSH-'):
                raise ValueError(endpoint+' 未返回 SSH 标识，请检查端口、代理或转发设置')
            if progress:progress(.5,'SSH 已响应，正在读取目标主机公钥指纹')
            try:
                p=subprocess.run([platform_host.ssh_executable('ssh-keyscan'),'-T','4','-p',str(target['ssh_port']),'-t','ed25519,ecdsa,rsa',name],capture_output=True,text=True,timeout=10)
            except subprocess.TimeoutExpired:
                raise ValueError(endpoint+' 的 SSH 公钥读取超时，请检查 SSH 服务或中间代理')
            output=p.stdout
        rows=[]
        for line in output.splitlines():
            parts=line.split()
            if len(parts)!=3:continue
            try:fingerprint='SHA256:'+base64.b64encode(hashlib.sha256(base64.b64decode(parts[2],validate=True)).digest()).decode().rstrip('=')
            except Exception:continue
            rows.append(dict(line=line,algorithm=parts[1],fingerprint=fingerprint))
        if not rows:raise ValueError(endpoint+' 已响应 SSH，但未取得有效主机公钥，请检查 SSH 服务或中间代理')
        key=uuid.uuid4().hex
        self.scans[key]=dict(role=role,host=name,port=target['ssh_port'],rows=rows,at=time.monotonic())
        return dict(ok=True,scan_id=key,host=name,port=target['ssh_port'],fingerprints=[{k:r[k] for k in ['algorithm','fingerprint']} for r in rows])

    def trust(self,req):
        scan=self.scans.get(req.get('scan_id'))
        if not scan or time.monotonic()-scan['at']>300:raise ValueError('指纹扫描已过期，请重新读取')
        target=self._effective_target(scan['role'])
        if (target['host'],target['ssh_port'])!=(scan['host'],scan['port']):raise ValueError('连接地址发生变化，请重新核对指纹')
        if req.get('confirmed') is not True:raise ValueError('需先核对并确认主机指纹')
        with self.lock:
            old=self.known.read_text() if self.known.exists() else ''
            names={r['line'].split(' ',1)[0] for r in scan['rows']}
            lines=[l for l in old.splitlines() if l.split(' ',1)[0] not in names]
            self.known.write_text('\n'.join(lines+[r['line'] for r in scan['rows']])+'\n');self.known.chmod(0o600)
            hop=self._aos_target()
            if scan['role']=='aos' or scan['role']=='sdk' and (scan['host'],scan['port'])==(hop['host'],hop['ssh_port']):
                keys=known_keys(self.known,endpoint_name(hop['host'],hop['ssh_port']))
                if keys:
                    alias=self.value.get('aos_identity_alias') or 'm20-aos-'+uuid.uuid4().hex
                    write_alias(self.known,alias,keys);self.value['aos_identity_alias']=alias;self.persist()
            self.scans.pop(req['scan_id'],None)
        return dict(ok=True)

    def pair(self,role,progress=None,replace=False):
        """Remember the selected device on first use; normal reconnects reuse it."""
        if role not in [*ROLES,'aos']:raise ValueError('连接目标无效')
        target=self._effective_target(role)
        if target.get('via_aos'):self.pair('aos',progress)
        if self._trusted(target) and not replace:return dict(ok=True,paired=True,already_paired=True)
        if role in ['aos','sdk'] and target.get('host_key_alias') and not replace:
            raise ValueError('设备配对记录不完整，请在高级设置重新配对')
        result=self.scan(role,progress)
        self.trust(dict(scan_id=result['scan_id'],confirmed=True))
        return dict(ok=True,paired=True,message='设备已配对')

    def ensure_key(self):
        if not self.key.exists():
            subprocess.run([platform_host.ssh_executable('ssh-keygen'),'-t','ed25519','-N','','-C','m20-platform-managed','-f',str(self.key)],capture_output=True,check=True,timeout=15)
            platform_host.private_file(self.key)
        return self.key.with_suffix('.pub').read_text().strip()

    def aos_ssh(self):
        with self.lock:
            if self._remember_aos_identity():self.persist()
            hop=self._aos_target()
        if not self._trusted(hop):raise ValueError('机器人 AOS 尚未配对，请点击完成首次连接')
        argv=[platform_host.ssh_executable('ssh'),'-T','-p',str(hop['ssh_port']),'-o','StrictHostKeyChecking=yes',
                '-o',platform_host.ssh_path_option('UserKnownHostsFile',self.known),'-o','ConnectTimeout=5','-o','BatchMode=yes',
                '-o','IdentitiesOnly=yes','-i',str(self.key),'-o','LogLevel=ERROR']
        if hop.get('host_key_alias'):argv+=['-o','HostKeyAlias='+hop['host_key_alias']]
        return argv,hop

    def scan_via_aos(self,target,progress=None):
        argv,hop=self.aos_ssh()
        if progress:progress(.3,'通过 AOS 读取内部主机 SSH 指纹')
        program="import json,sys,socket,subprocess;r=json.load(sys.stdin);s=socket.create_connection((r['host'],r['port']),timeout=3);s.settimeout(2);b=s.recv(256);s.close();assert b.startswith(b'SSH-'),'目标端口没有返回 SSH 标识';p=subprocess.run(['ssh-keyscan','-T','4','-p',str(r['port']),'-t','ed25519,ecdsa,rsa',r['host']],capture_output=True,text=True,timeout=10);print(p.stdout,end='')"
        argv += [hop['user']+'@'+hop['host'],shlex.join(['/usr/bin/python3','-c',program])]
        try:r=subprocess.run(argv,input=json.dumps(dict(host=target['host'],port=target['ssh_port'])),capture_output=True,text=True,timeout=18)
        except subprocess.TimeoutExpired:raise ValueError('经 AOS 读取内部主机指纹超时')
        if r.returncode:raise ValueError('AOS 中转读取失败：'+r.stderr[-1000:])
        return r.stdout

    def ssh_command(self,role,password=None,remote=None):
        target=self._effective_target(role)
        if not self._trusted(target):raise ValueError('设备尚未配对，请点击完成首次连接')
        argv=[platform_host.ssh_executable('ssh'),'-T','-p',str(target['ssh_port']),'-o','StrictHostKeyChecking=yes','-o',platform_host.ssh_path_option('UserKnownHostsFile',self.known),
              '-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','-o','LogLevel=ERROR']
        if target.get('host_key_alias'):argv+=['-o','HostKeyAlias='+target['host_key_alias']]
        if target.get('via_aos'):
            hop_argv,hop=self.aos_ssh()
            hop_argv+=['-W',target['host']+':'+str(target['ssh_port']),hop['user']+'@'+hop['host']]
            argv+=['-o','ProxyCommand='+platform_host.local_command_line(hop_argv)]
        if password:argv+=['-o','BatchMode=no','-o','PreferredAuthentications=password,keyboard-interactive','-o','PubkeyAuthentication=no','-o','NumberOfPasswordPrompts=1']
        else:
            argv+=['-o','BatchMode=yes']
            if self.key.exists():argv+=['-i',str(self.key),'-o','IdentitiesOnly=yes']
        if remote is None:
            argv+=['-N','-o','ExitOnForwardFailure=yes','-L','127.0.0.1:%d:127.0.0.1:%d'%(target['local_port'],target['remote_port'])]
        argv+=[target['user']+'@'+target['host']]
        if remote is not None:argv+=[shlex.join(remote)]
        env=os.environ.copy()
        if password and not platform_host.IS_WINDOWS:env.update(SSH_ASKPASS=str(self.askpass),SSH_ASKPASS_REQUIRE='force',DISPLAY='m20-platform:0',M20_SSH_PASSWORD=password)
        return argv,env

    def remote(self,role,program,payload,password=None,timeout=180,expected=None):
        with self.lock:
            configured=self._aos_target() if role=='aos' else self.value['targets'][role]
            if expected and (self.done or configured!=expected['target'] or self.value['aos_ip']!=expected['aos']):
                raise ValueError('连接已更新，旧服务操作已取消')
            target=self._effective_target(role);hop=self._aos_target() if target.get('via_aos') else None
            argv,env=self.ssh_command(role,password,[target['python'],'-c',program])
        if platform_host.IS_WINDOWS and password:
            from windows_ssh import run_remote
            p=run_remote(self.known,self.key,target,hop,argv[-1],json.dumps(payload),password,timeout)
        else:
            p=subprocess.run(argv,input=json.dumps(payload),capture_output=True,text=True,env=env,start_new_session=not platform_host.IS_WINDOWS,timeout=timeout)
        try:response=json.loads(p.stdout)
        except ValueError:
            message=p.stderr[-1000:] if p.stderr else '远端未返回有效结果，请检查 Python 环境与连接凭据'
            if password:message=message.replace(password,'[已隐藏]')
            raise ValueError(message)
        if p.returncode or response.get('ok') is False:raise ValueError(response.get('err','远端操作失败'))
        return response

    @staticmethod
    def bundle():
        names=['observation_errors.py','slam_pose_check.py','autonomy_core.py','local_trajectory.py','exploration_frontiers.py','autonomy_sensing.py','native_sensing.py','native/sensing.cpp','tools/build_native_sensing.py','robot_geometry.py','navigation_diagnostics.py','map_fusion.py','map_refinement.py','map_change_policy.py','map_registration.py','map_visibility.py','platform_core.py','m20_console.py','map_formats.py','operations.py','bridge/map_refinement_agent.py',
               'bridge/http_agent.py','bridge/sensor_agent.py','bridge/sensor_client.py','bridge/map_service.py','bridge/mapping_job.py','bridge/ros_sensors.py','bridge/nos_agent.py','bridge/gos_agent.py',
               'bridge/sdk_bridge.py','bridge/sdk_agent.py','bridge/training_agent.py','bridge/root_actions.py','map_catalog.py',
               'platform_web/assets/model/M20.urdf','reference/errors.md']
        buffer=io.BytesIO()
        with zipfile.ZipFile(buffer,'w',zipfile.ZIP_DEFLATED) as z:
            for name in names:z.write(ROOT/name,name)
        return base64.b64encode(buffer.getvalue()).decode()

    def deploy(self,req,progress):
        role=req.get('role')
        if role not in ROLES:raise ValueError('部署目标无效')
        if req.get('confirmed') is not True:raise ValueError('请确认首次部署、专用公钥与系统服务变更')
        target=copy.deepcopy(self.value['targets'][role])
        deployment=dict(target=target,aos=self.value['aos_ip'])
        if not target['host']:raise ValueError('请先填写目标主机')
        if role=='train' and not target['train_root'].startswith('/'):raise ValueError('请填写工作站训练工程的绝对路径')
        if role=='sdk' and not target['sdk_workspace'].startswith('/'):
            raise ValueError('请填写 SDK 工作区的绝对路径')
        public=self.ensure_key()
        if target.get('via_aos'):
            self.pair('aos',progress)
            program="import json,sys,os;from pathlib import Path;r=json.load(sys.stdin);d=Path.home()/'.ssh';d.mkdir(mode=0o700,exist_ok=True);p=d/'authorized_keys';s=p.read_text() if p.exists() else '';k=r['key'];p.write_text(s+('' if not s or s.endswith('\\n') else '\\n')+k+'\\n') if k not in s.splitlines() else None;p.chmod(0o600);print(json.dumps(dict(ok=True)))"
            relay=dict(target=self._aos_target(),aos=deployment['aos'])
            try:self.remote('aos',program,dict(key=public),timeout=20,expected=relay)
            except ValueError:
                password=req.get('aos_password') or req.get('password')
                if not password:raise ValueError('首次连接需要机器人账号密码，后续会自动连接')
                self.remote('aos',program,dict(key=public),password,timeout=30,expected=relay)
        self.pair(role,progress)
        progress(.1,'正在安装连接服务')
        program=(ROOT/'bridge/install_remote.py').read_text()
        payload=dict(role=role,target=target,user=target['user'],public_key=public,bundle=self.bundle(),
                     authorize_key=True,administrative_setup=True,admin_password=req.get('admin_password') or req.get('password',''),calibration=req.get('calibration'))
        result=self.remote(role,program,payload,req.get('password'),timeout=240,expected=deployment)
        with self.lock:
            if self.value['targets'][role]!=target or self.value['aos_ip']!=deployment['aos']:
                raise ValueError('连接已更新，请在当前连接中查看安装状态')
            self.value['targets'][role]['deployed']=bool(result.get('installed'))
            self.value['targets'][role]['enabled']=bool(result.get('installed'))
            self.persist()
        progress(.9,'安装完成，正在建立后台连接')
        self.start(role)
        self.log('connection',role+' 服务已部署，SDK 控制不会自动开启')
        return result

    def _terminate(self,role):
        proc=self.processes.pop(role,None)
        if proc and proc.poll() is None:
            proc.terminate()

    def start(self,role):
        with self.lock:
            target=self.value['targets'][role]
            if not target['enabled']:raise ValueError('请先启用并完成该目标的首次配置')
            proc=self.processes.get(role)
            if proc and proc.poll() is None:return dict(ok=True,message='后台连接已运行')
            argv,env=self.ssh_command(role)
            logfile=self.ssh/(role+'.log')
            with logfile.open('w') as log:
                proc=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=log,env=env,start_new_session=not platform_host.IS_WINDOWS)
            self.processes[role]=proc
            self.states[role]=dict(status='connecting',message='正在连接',attempted=time.time())
            return dict(ok=True)

    def stop(self,role):
        with self.lock:
            self._terminate(role)
            self.value['targets'][role]['enabled']=False;self.persist()
            self.states[role]=dict(status='stopped',message='后台连接已停止')
        return dict(ok=True)

    def endpoint(self,role):
        target=self.value['targets'][role]
        return 'http://127.0.0.1:'+str(target['local_port']),target['token']

    def service(self,req,expected=None):
        role=req.get('role');action=req.get('action')
        if role not in ROLES or action not in ['start','stop','restart','status','logs']:raise ValueError('服务操作无效')
        program="import json,subprocess,sys\nr=json.load(sys.stdin)\nname='m20-platform-'+r['role']+'.service'\na=r['action']\nargs=['systemctl',a,name] if a in ['start','stop','restart'] else ['systemctl','show',name,'--property=ActiveState,SubState,ExecMainStatus'] if a=='status' else ['journalctl','--no-pager','-n','100','-u',name]\nif a in ['start','stop','restart']:args=['sudo','-n','/usr/bin/python3','/opt/m20-platform/bridge/root_actions.py'];body=json.dumps(dict(action='service',role=r['role'],operation=a))\nelse:body=None\np=subprocess.run(args,input=body,capture_output=True,text=True,timeout=30)\nprint(json.dumps(dict(ok=p.returncode==0,output=p.stdout[-12000:],err=p.stderr[-2000:])))"
        return self.remote(role,program,dict(role=role,action=action),timeout=40,**(dict(expected=expected) if expected else {}))

    def recover_service(self,role):
        """Start an installed platform service once, without blocking other hosts."""
        with self.lock:
            previous=self.service_attempts.get(role,{})
            if previous.get('running') or time.monotonic()-previous.get('at',-60)<60:return
            target=copy.deepcopy(self.value['targets'][role]);aos=self.value['aos_ip']
            token=dict(at=time.monotonic(),running=True);self.service_attempts[role]=token
        def recover():
            try:
                with self.lock:
                    if self.done or self.value['targets'][role]!=target or self.value['aos_ip']!=aos:return
                self.service(dict(role=role,action='start'),expected=dict(target=target,aos=aos))
            except Exception as exc:token['detail']=str(exc)[:250]
            finally:token['running']=False
        threading.Thread(target=recover,daemon=True).start()

    def poll_once(self):
        for role,target in list(self.value['targets'].items()):
            if not target['enabled']:continue
            state=self.states.get(role,{})
            proc=self.processes.get(role)
            if proc is None or proc.poll() is not None:
                if proc is not None:
                    log=self.ssh/(role+'.log')
                    try:detail=log.read_text()[-800:].strip()
                    except OSError:detail='SSH 进程已退出'
                    if 'Host key verification failed' in detail or 'REMOTE HOST IDENTIFICATION HAS CHANGED' in detail:
                        detail='设备身份已变化，请确认机器人地址；更换机器人或重装系统后可在高级设置重新配对。'
                    elif 'Permission denied' in detail:detail='这台电脑尚未完成首次连接，请在连接设置中安装连接服务。'
                    self.states[role]=dict(status='unreachable',message='设备未连接：'+detail,attempted=state.get('attempted',0))
                # Internal hosts remain behind AOS after an external Wi-Fi move.
                # Prefer the existing relay over administrator-added PC routes.
                if (proc is not None and role in ['nos','gos'] and not target.get('via_aos')
                        and target['host'].startswith('10.21.31.') and not self.value['aos_ip'].startswith('10.21.31.')
                        and self._trusted(self._aos_target()) and self.key.exists()):
                    with self.lock:target['via_aos']=True;self.persist()
                if time.time()-state.get('attempted',0)<15:continue
                try:self.start(role)
                except ValueError as exc:self.states[role]=dict(status='needs_setup',message=str(exc),attempted=time.time())
                continue
            try:
                with socket.create_connection(('127.0.0.1',target['local_port']),timeout=.2):pass
            except OSError:
                self.states[role]=dict(status='connecting',message='正在连接设备',attempted=state.get('attempted',0))
                continue
            try:
                url,token=self.endpoint(role)
                request=Request(url+'/status',headers={'X-API-Token':token})
                with urlopen(request,timeout=2) as r:status=json.load(r)
                if status.get('ok') is False:raise ValueError('服务未就绪')
                self.states[role]=dict(status='ready',message='服务已连接'+(' · 经 AOS' if target.get('via_aos') else ''),attempted=state.get('attempted',0),checked=time.time())
            except Exception as exc:
                self.states[role]=dict(status='waiting_service',message='正在等待设备服务响应',detail=str(exc)[:250],attempted=state.get('attempted',0))
                if target.get('deployed'):self.recover_service(role)

    def discover(self,progress=None):
        with self.lock:
            alias=self.value.get('aos_identity_alias')
            expected=known_keys(self.known,alias) if valid_alias(alias) else []
            original=(self.value['aos_ip'],self.value.get('aos_ssh_port',22),alias)
        if not expected:
            return dict(ok=True,found=False,connections=self.public(),message='尚未配对机器人，请填写地址完成首次连接')
        def current():
            with self.lock:
                return not self.done and original==(self.value['aos_ip'],self.value.get('aos_ssh_port',22),self.value.get('aos_identity_alias'))
        found=robot_discovery.find_robot(original[0],original[1],expected,progress,cancel=lambda:not current())
        with self.lock:
            if not current() or known_keys(self.known,alias)!=expected:
                return dict(ok=True,found=False,connections=self.public(),message='连接设置已更新')
            if not found:
                return dict(ok=True,found=False,connections=self.public(),message='未找到已配对机器人，请确认连接到同一网络或填写机器人地址')
            if found!=self.value['aos_ip']:
                self.save(dict(aos_ip=found,auto_connect=self.value.get('auto_connect',False)))
            return dict(ok=True,found=True,connections=self.public(),message='已找到机器人')

    def close(self):
        self.done=True
        with self.lock:
            for role in list(self.processes):self._terminate(role)

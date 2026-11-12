"""Executed on the selected host by the console's first-time setup wizard.

The SSH host key is checked before this payload is sent. Nothing runs during a
normal page visit. Administrative setup is an explicit wizard action.
"""
import base64
import io
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import zipfile

ROOT_SETUP = r'''
import base64,io,ipaddress,json,os,pwd,re,shlex,subprocess,sys,zipfile
import xml.etree.ElementTree as ET
from pathlib import Path,PurePosixPath
r=json.load(sys.stdin)
user=r['user']
if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_.-]{0,63}',user):raise ValueError('Invalid user')
account=pwd.getpwnam(user)
root=Path('/opt/m20-platform');root.mkdir(parents=True,exist_ok=True)
data=Path('/var/lib/m20-platform');data.mkdir(parents=True,exist_ok=True)
maplock=Path('/var/opt/robot/data/maps/.m20-catalog.lock')
if maplock.parent.is_dir():
 if maplock.is_symlink():raise ValueError('Map lock path is a symlink')
 maplock.touch(exist_ok=True);maplock.chmod(0o644)
warnings=[]
os.chown(data,account.pw_uid,account.pw_gid)
with zipfile.ZipFile(io.BytesIO(base64.b64decode(r['bundle'],validate=True))) as z:
 for entry in z.infolist():
  p=PurePosixPath(entry.filename)
  if p.is_absolute() or '..' in p.parts or '\\' in entry.filename or entry.is_dir():raise ValueError('Invalid installation bundle')
  out=root/entry.filename;out.parent.mkdir(parents=True,exist_ok=True)
  if out.is_symlink():raise ValueError('Installation target is a symlink')
  out.write_bytes(z.read(entry));out.chmod(0o644)
native_builder=root/'tools/build_native_sensing.py'
if native_builder.is_file():
 build=subprocess.run(['/usr/bin/python3',str(native_builder)],capture_output=True,text=True,timeout=120)
 if build.returncode:warnings.append('原生点云加速未编译，暂用 NumPy 后端；检查 C++ 编译器后可重新部署')
envdir=Path('/etc/m20-platform');envdir.mkdir(mode=0o700,exist_ok=True)
for role,target in r['targets'].items():
 if role not in ['nos','gos','sdk','train']:raise ValueError('Invalid role')
 token=target['token']
 if not re.fullmatch(r'[A-Za-z0-9_-]{20,100}',token):raise ValueError('Invalid token')
 env='M20_BRIDGE_TOKEN='+token+'\nM20_DATA_DIR=/var/lib/m20-platform\n'
 onboard={'gos':('gos_aos_ip','M20_GOS'),'nos':('resident_aos_ip','M20_RESIDENT')}.get(role)
 if onboard and target.get(onboard[0]):
  address=ipaddress.IPv4Address(target[onboard[0]])
  if not address.is_private or address.is_loopback or address.is_unspecified:raise ValueError('Invalid onboard AOS address')
  prefix=onboard[1]
  env+=prefix+'_NAVIGATION=1\n'+prefix+'_AOS_IP='+str(address)+'\n'
 envfile=envdir/(role+'.env');envfile.write_text(env);envfile.chmod(0o600)
 py=target['python'];setups=list(target.get('ros_setup',[]))
 if role=='sdk' and target.get('sdk_workspace'):
  overlay=target['sdk_workspace'].rstrip('/')+'/install/setup.bash'
  if overlay not in setups:setups.append(overlay)
 if not py.startswith('/') or '\n' in py:raise ValueError('Python path must be absolute')
 sources=[]
 for path in setups:
  if not path.startswith('/') or '\n' in path:raise ValueError('Setup path must be absolute')
  sources.append('source '+shlex.quote(path))
 entry='training_agent.py' if role=='train' else role+'_agent.py'
 argv=[py,str(root/'bridge'/entry),'--port',str(target['remote_port'])]
 enabled=True
 if role=='train':
  if not target.get('train_root','').startswith('/'):raise ValueError('Training root must be absolute')
  argv+=['--root',target['train_root'],'--python',py]
 if role=='sdk':
  calibration=r.get('calibration')
  if calibration is not None:
   calfile=data/'calibration.json';calfile.write_text(json.dumps(calibration));calfile.chmod(0o600);os.chown(calfile,account.pw_uid,account.pw_gid)
  envfile.write_text(envfile.read_text()+'M20_SDK_WORKSPACE='+json.dumps(target.get('sdk_workspace','/home/'+user+'/M20_sdk_deploy'))+'\n')
 if role in ['nos','gos']:
  # Keep HTTP/map management unprivileged. The factory DDS reader uses its own
  # profile and participant; never rewrite the vendor XML or remove shared SHM.
  reader='m20-platform-sensors-'+role+'.service'
  profile=envdir/('sensors-'+role+'.xml')
  tree=ET.parse('/opt/robot/fastdds.xml')
  ns=tree.getroot().tag.partition('}')[0].lstrip('{')
  tag=lambda name:('{'+ns+'}' if ns else '')+name
  if ns:ET.register_namespace('',ns)
  participants=list(tree.iter(tag('participant')))
  default=next((p for p in participants if p.get('is_default_profile','').lower()=='true'),None)
  if default is None:raise ValueError('Vendor DDS default participant profile missing')
  rtps=default.find(tag('rtps'))
  if rtps is None:raise ValueError('Vendor DDS rtps profile missing')
  participant=rtps.find(tag('participantID'))
  if participant is None:participant=ET.SubElement(rtps,tag('participantID'))
  participant.text='30' if role=='nos' else '31'
  tree.write(str(profile),encoding='utf-8',xml_declaration=True);profile.chmod(0o600)
  reader_cmd='source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID='+str(target.get('domain_id',0))+' FASTRTPS_DEFAULT_PROFILES_FILE='+shlex.quote(str(profile))+'; exec '+shlex.join([py,'-u',str(root/'bridge/sensor_agent.py'),'--port','8094'])
  reader_exec=('/bin/bash -lc '+shlex.quote(reader_cmd)).replace('%','%%')
  reader_unit='[Unit]\nDescription=M20 read-only DDS sensor reader\nAfter=network-online.target\nPartOf=m20-platform-'+role+'.service\n\n[Service]\nType=simple\nUser=root\nWorkingDirectory=/opt/m20-platform\nEnvironmentFile='+str(envfile)+'\nExecStart='+reader_exec+'\nRestart=on-failure\nRestartSec=3\nUMask=0077\n'
  Path('/etc/systemd/system/'+reader).write_text(reader_unit)
  # The host agent now needs no ROS environment and never edits vendor DDS XML.
  sources=[]
 vision_python=root/'vision-runtime/python'
 if role=='gos' and (vision_python/'onnxruntime/__init__.py').is_file():
  sources.append('export PYTHONPATH='+shlex.quote(str(vision_python)))
 command='; '.join(sources+['export ROS_DOMAIN_ID='+str(target.get('domain_id',0)),'exec '+shlex.join(argv)])
 # % is a systemd specifier, including inside shell quotes.
 shell=('/bin/bash -lc '+shlex.quote(command)).replace('%','%%')
 unit='[Unit]\nDescription=M20 platform '+role+' agent\nAfter=network-online.target\nWants=network-online.target\n\n[Service]\nType=simple\nUser='+('root' if role=='sdk' and target.get('sdk_admin') else user)+'\nWorkingDirectory=/opt/m20-platform\nEnvironmentFile='+str(envfile)+'\nExecStart='+shell+'\nRestart=on-failure\nRestartSec=5\nTimeoutStopSec=15\nUMask=0077\n\n[Install]\nWantedBy=multi-user.target\n'
 name='m20-platform-'+role+'.service'
 if role in ['nos','gos']:unit=unit.replace('[Service]','Requires='+reader+'\nAfter='+reader+'\n\n[Service]',1)
 Path('/etc/systemd/system/'+name).write_text(unit)
 subprocess.run(['systemctl','daemon-reload'],check=True,capture_output=True)
 if enabled:
  subprocess.run(['systemctl','enable',name],check=True,capture_output=True)
  subprocess.run(['systemctl','restart',name],check=True,capture_output=True)
 else:subprocess.run(['systemctl','disable',name],capture_output=True)
sudoers=Path('/etc/sudoers.d/m20-platform-'+user)
body=user+' ALL=(root) NOPASSWD: /usr/bin/python3 /opt/m20-platform/bridge/root_actions.py\n'
tmp=sudoers.with_suffix('.tmp');tmp.write_text(body);tmp.chmod(0o440)
test=subprocess.run(['visudo','-cf',str(tmp)],capture_output=True,text=True)
if test.returncode:tmp.unlink();raise ValueError('Sudo rule validation failed')
tmp.replace(sudoers)
print(json.dumps(dict(ok=True,installed=list(r['targets']),sdk_autostart=False,warnings=warnings)))
'''


def install(req):
    public=req.get('public_key','')
    if not public.startswith('ssh-ed25519 ') or '\n' in public.strip(): raise ValueError('无效专用公钥')
    role=req.get('role')
    if role not in ['nos','gos','sdk','train']: raise ValueError('无效服务类型')
    if req.get('authorize_key'):
        ssh=Path.home()/'.ssh';ssh.mkdir(mode=0o700,exist_ok=True)
        keys=ssh/'authorized_keys'
        old=keys.read_text() if keys.exists() else ''
        if public.split()[1] not in old:
            with keys.open('a') as f:f.write(('\n' if old and not old.endswith('\n') else '')+public.strip()+'\n')
        keys.chmod(0o600)
    if not req.get('administrative_setup'):
        return dict(ok=True,key_authorized=bool(req.get('authorize_key')),installed=False,
                    message='已配置连接公钥；开机服务尚未安装，请选择安装系统服务')
    helper=Path.home()/'.local/share/m20-platform/setup-askpass.py'
    helper.parent.mkdir(parents=True,exist_ok=True)
    helper.write_text('#!/usr/bin/python3\nimport os,sys\nsys.stdout.write(os.environ.get("M20_SETUP_PASSWORD", ""))\n')
    helper.chmod(0o700)
    env=os.environ.copy()
    password=req.get('admin_password') or ''
    if password:env.update(SUDO_ASKPASS=str(helper),M20_SETUP_PASSWORD=password)
    target=req['target']
    payload=dict(bundle=req['bundle'],user=req['user'],targets={role:target})
    if req.get('calibration') is not None:payload['calibration']=req['calibration']
    argv=['sudo','-A' if password else '-n','--','/usr/bin/python3','-c',ROOT_SETUP]
    p=subprocess.run(argv,input=json.dumps(payload),capture_output=True,text=True,env=env,timeout=180)
    if p.returncode:raise ValueError('安装系统服务失败，请检查管理员权限和运行环境；'+(p.stderr or p.stdout)[-1600:])
    try:return json.loads(p.stdout)
    except ValueError:raise ValueError('安装程序返回格式异常')


if __name__=='__main__':
    try:print(json.dumps(install(json.load(sys.stdin)),ensure_ascii=False))
    except Exception as exc:
        print(json.dumps(dict(ok=False,err=str(exc)),ensure_ascii=False));sys.exit(1)

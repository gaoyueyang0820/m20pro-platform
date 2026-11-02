#!/usr/bin/env python3
"""SDK supervisor: read-only startup, vendor ROS2 control and exclusive joint debug."""
import argparse
import copy
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from platform_core import JOINTS,number,rpc
from http_agent import serve

SDK_SOURCE='https://github.com/DeepRoboticsLab/sdk_deploy.git'
SDK_REVISION='20279073bc43522aa06b0abf611bb2daaa53d87d'


def ros_input_source(body):
    # Ignore comments while preserving offsets into the original source.
    clean=re.sub(r'/\*.*?\*/|//[^\n]*',lambda m:' '*len(m.group()),body,flags=re.S)
    pattern=r'RobotName\s*::\s*M20\s*,\s*(?:\w+\s*::\s*)?RemoteCommandType\s*::\s*(k\w+)'
    matches=list(re.finditer(pattern,clean))
    if len(matches)!=1:
        raise ValueError('未找到唯一的 M20 输入配置。需要包含 RobotName::M20 和 RemoteCommandType 的 SDK 入口；请查看检测结果，而不是修改固件版本')
    token=matches[0].span(1)
    return body[:token[0]]+'kRos2'+body[token[1]:],matches[0].group(1)


class RosPolicy:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        from drdds.msg import RobotStatus
        self.rclpy=rclpy;self.String=String;self.Twist=Twist
        rclpy.init();self.node=Node('m20_platform_policy_client')
        self.states=self.node.create_publisher(String,'/rl_deploy/command_state',10)
        self.velocities=self.node.create_publisher(Twist,'/rl_deploy/cmd_vel',10)
        self.state={};self.rx=0.;self.velocity=[0.,0.,0.];self.until=0.;self.pending_zero=False;self.lock=threading.RLock()
        self.node.create_subscription(RobotStatus,'/rl_deploy/status',self.feedback,10)
        self.node.create_timer(.02,self.tick)
        threading.Thread(target=lambda:rclpy.spin(self.node),daemon=True).start()

    def feedback(self,msg):
        names=list(msg.joints.name);rows=[]
        for joint in JOINTS:
            if joint['name'] not in names:return
            i=names.index(joint['name'])
            try:rows.append(dict(position=number(msg.joints.position[i]),velocity=number(msg.joints.velocity[i]),
                                 torque=number(msg.joints.effort[i]) if i<len(msg.joints.effort) else None,fresh=bool(msg.telemetry_fresh)))
            except (ValueError,IndexError):return
        with self.lock:
            self.rx=time.monotonic()
            self.state=dict(current_state=msg.current_state,current_state_id=msg.current_state_id,
                            requested_state=msg.requested_state,source=msg.command_source,safety_mode=msg.safety_mode,
                            telemetry_fresh=bool(msg.telemetry_fresh),joints=rows,
                            command_timed_out=msg.command_timed_out,rejected_commands=msg.rejected_commands)

    def status(self):
        with self.lock:return dict(self.state,fresh=time.monotonic()-self.rx<.5,connected=bool(self.rx))

    def state_command(self,name):
        if name not in ['stand','rl_control','lie_down','damping']:raise ValueError('SDK 状态指令无效')
        status=self.status()
        if not status['fresh'] or status.get('source')!='ros2':raise ValueError('尚未收到 ROS2 输入版 SDK 的新鲜状态，请检查是否已编译 kRos2 并启动')
        if name=='rl_control' and status.get('current_state_id')!=1:raise ValueError('需先站立完成再进入策略控制')
        self.stop_velocity();msg=self.String();msg.data=name;self.states.publish(msg)
        return dict(ok=True,delivery='sent',message='SDK ROS2 状态指令已发送，请核对状态回报')

    def set_velocity(self,req):
        v=[number(req.get(k,0),-limit,limit) for k,limit in [('forward',.7),('side',.5),('yaw',.7)]]
        status=self.status()
        if any(v) and (not status['fresh'] or not status.get('telemetry_fresh') or status.get('source')!='ros2' or status.get('current_state_id')!=6 or status.get('safety_mode')):
            raise ValueError('SDK 未处于新鲜、无安全异常的策略控制状态')
        with self.lock:self.velocity=v;self.until=time.monotonic()+.25;self.pending_zero=True
        return dict(ok=True)

    def stop_velocity(self):
        with self.lock:self.velocity=[0.,0.,0.];self.until=0.;self.pending_zero=True

    def tick(self):
        with self.lock:
            fresh=time.monotonic()-self.rx<.5
            v=self.velocity if time.monotonic()<self.until and fresh else [0.,0.,0.]
            if not any(v) and not self.pending_zero:return
            msg=self.Twist();msg.linear.x=v[0];msg.linear.y=v[1];msg.angular.z=v[2]
            self.velocities.publish(msg)
            if not any(v):self.velocity=[0.,0.,0.];self.pending_zero=False

    def close(self):
        self.stop_velocity();self.tick();self.node.destroy_node();self.rclpy.shutdown()


class SDKAgent:
    def __init__(self,workspace=None,calibration=None,data=None):
        self.workspace=Path(workspace or os.environ.get('M20_SDK_WORKSPACE',Path.home()/'M20_sdk_deploy')).resolve()
        self.data=Path(data or os.environ.get('M20_DATA_DIR',Path.home()/'.local/share/m20-platform/data'));self.data.mkdir(parents=True,exist_ok=True)
        self.selection=self.data/'sdk-active-workspace.json'
        if self.selection.is_file():
            selected=Path(json.loads(self.selection.read_text())['workspace'])
            if selected.is_absolute() and selected.is_dir():self.workspace=selected.resolve()
        self.calibration=Path(calibration or self.data/'calibration.json')
        self.policy=None;self.ros_error=None;self.vendor=None;self.worker=None;self.attached=False;self.lock=threading.RLock()
        self.control_generation=0
        self.worker_url='http://127.0.0.1:8094';self.token=os.environ.get('M20_BRIDGE_TOKEN')

    def ros(self):
        with self.lock:
            if self.policy is None:
                try:
                    overlay=self.workspace/'install/setup.bash'
                    if overlay.is_file():
                        import shlex
                        loaded=subprocess.run(['/bin/bash','-c','source '+shlex.quote(str(overlay))+' >/dev/null && env -0'],capture_output=True,timeout=8)
                        for entry in loaded.stdout.split(b'\0'):
                            key,sep,value=entry.partition(b'=')
                            if sep and key.decode(errors='replace') in ['PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','PATH','COLCON_PREFIX_PATH']:
                                os.environ[key.decode()]=value.decode()
                        for path in reversed(os.environ.get('PYTHONPATH','').split(':')):
                            if path and path not in sys.path:sys.path.insert(0,path)
                    self.policy=RosPolicy();self.ros_error=None
                except Exception as exc:
                    self.ros_error=type(exc).__name__+': '+str(exc)
                    raise ValueError('SDK 运行环境未就绪：'+self.ros_error+'。点击“准备 SDK 环境”可编译并重新载入工作区')
            return self.policy

    def worker_alive(self):return self.worker is not None and self.worker.poll() is None
    def vendor_alive(self):return self.vendor is not None and self.vendor.poll() is None

    def worker_rpc(self,path,req=None):return rpc(self.worker_url,path,req,2,self.token)

    def status(self):
        result=dict(ok=True,connected=True,backend='hardware',armed=False,fresh=False,joints=[],
                    calibration_verified=False,supervisor=True,vendor_running=self.vendor_alive(),
                    direct_joint_running=self.worker_alive(),workspace=str(self.workspace),ros_error=self.ros_error,
                    effective_uid=os.geteuid() if hasattr(os,'geteuid') else None,
                    software_mode_switch_supported=False)
        if self.worker_alive():
            try:result.update(self.worker_rpc('/status'))
            except ValueError:pass
        elif self.policy is not None:
            state=self.policy.status();result['policy']=state
            if state.get('fresh') and state.get('telemetry_fresh'):
                result.update(joints=state.get('joints',[]),fresh=True)
        if self.calibration.is_file():
            try:
                from sdk_bridge import JointController
                JointController(json.loads(self.calibration.read_text()))
                result['calibration_verified']=True
            except ValueError:pass
        result['environment']=self.environment_status()
        return result

    def environment_status(self):
        source=self.workspace/'src/M20_sdk_deploy/main.cpp'
        if not source.is_file():
            return dict(ready=False,stage='source_missing',message='工作区中没有 M20 SDK 源码',workspace=str(self.workspace))
        try:_,mode=ros_input_source(source.read_text())
        except ValueError as exc:return dict(ready=False,stage='entry_unrecognized',message=str(exc),workspace=str(self.workspace))
        # setup.bash may exist after building drdds alone or a failed rebuild.
        # Require the actual installed controller, with input source no newer
        # than its binary. Support colcon's isolated and merged install layouts.
        candidates=[self.workspace/'install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy',
                    self.workspace/'install/lib/m20_sdk_deploy/rl_deploy']
        executable=next((path for path in candidates if path.is_file() and os.access(path,os.X_OK)),None)
        installed=(self.workspace/'install/setup.bash').is_file() and executable is not None
        built=installed and executable.stat().st_mtime_ns>=source.stat().st_mtime_ns
        ready=bool(built and mode=='kRos2')
        message=('SDK 工程已就绪' if ready else 'SDK 控制器尚未完整安装，需要准备环境' if not installed
                 else 'SDK 输入配置已更新，需要重新编译' if mode=='kRos2' else '需要一次编译以启用平台 ROS2 输入')
        return dict(ready=ready,stage='ready' if ready else 'needs_build',input=mode,
                    workspace=str(self.workspace),executable=str(executable) if executable else None,message=message)

    def stop(self):
        # Cancel a connection still waiting for DDS discovery before it can
        # attach or start a controller after the operator has pressed stop.
        with self.lock:self.control_generation+=1
        if self.worker_alive():
            try:self.worker_rpc('/stop',{})
            except ValueError:pass
        if self.policy:
            self.policy.stop_velocity()
            try:self.policy.state_command('damping')
            except ValueError:pass
        return dict(ok=True,message='已请求 SDK 归零与阻尼，未声称退出固件 SDK 模式')

    def build(self,req):
        if self.vendor_alive() or self.worker_alive():raise ValueError('请先停止 SDK 策略和关节调试再编译')
        if req.get('confirmed') is not True:raise ValueError('需确认修改 SDK 输入方式并重新编译')
        source=self.workspace/'src/M20_sdk_deploy/main.cpp'
        if not source.is_file():raise ValueError('SDK 工作区缺少 src/M20_sdk_deploy/main.cpp')
        body=source.read_text()
        updated,previous=ros_input_source(body)
        backup=source.with_suffix('.cpp.m20-platform.bak')
        if not backup.exists():backup.write_text(body)
        if updated!=body:source.write_text(updated)
        p=subprocess.run(['colcon','build','--packages-up-to','m20_sdk_deploy','--cmake-args','-DBUILD_PLATFORM=arm'],
                         cwd=self.workspace,capture_output=True,text=True,timeout=1200)
        if p.returncode:raise ValueError('SDK 编译失败：'+(p.stderr or p.stdout)[-10000:])
        return dict(ok=True,changed=True,reload_required=True,output=p.stdout[-12000:],message='已编译 ROS2 输入版 SDK，请重启 SDK 服务以载入新工作区消息')

    def setup(self):
        environment=self.environment_status()
        if environment.get('ready'):
            return dict(ok=True,changed=False,reload_required=False,workspace=str(self.workspace),
                        environment=environment,message='SDK 工程已就绪，无需重新编译或重启')
        if self.vendor_alive() or self.worker_alive():raise ValueError('请先结束正在运行的 SDK 控制，再更新工程')
        source=self.workspace/'src/M20_sdk_deploy/main.cpp'
        compatible=False
        if source.is_file():
            try:ros_input_source(source.read_text());compatible=True
            except ValueError:pass
        if source.is_file() and not compatible:
            self.workspace=self.data/'sdk-platform-ros2'
            source=self.workspace/'src/M20_sdk_deploy/main.cpp'
        if not source.is_file():self.prepare_source(dict(confirmed=True))
        result=self.build(dict(confirmed=True))
        self.selection.write_text(json.dumps(dict(workspace=str(self.workspace))));self.selection.chmod(0o600)
        return dict(result,changed=True,reload_required=True,workspace=str(self.workspace))

    def prepare_source(self,req):
        if req.get('confirmed') is not True:raise ValueError('需确认下载固定版本厂商源码')
        if self.vendor_alive() or self.worker_alive():raise ValueError('请先停止 SDK 控制进程')
        target=self.workspace/'src'
        if any((target/name).exists() or (target/name).is_symlink() for name in ['drdds','M20_sdk_deploy']):
            raise ValueError('目标工作区已含 SDK 源码，已拒绝覆盖；可直接编译现有版本')
        if not shutil.which('git'):raise ValueError('目标设备未安装 git，需先完成厂商 SDK 开发环境配置')
        with tempfile.TemporaryDirectory(prefix='sdk-source-',dir=self.data) as temp:
            checkout=Path(temp)
            commands=[['git','init','-q'],['git','remote','add','origin',SDK_SOURCE],
                      ['git','fetch','--depth','1','origin',SDK_REVISION],['git','checkout','--detach','-q','FETCH_HEAD']]
            for argv in commands:
                p=subprocess.run(argv,cwd=checkout,capture_output=True,text=True,timeout=360)
                if p.returncode:raise ValueError('厂商源码下载失败，请检查目标设备访问 GitHub 的网络：'+(p.stderr or p.stdout)[-1600:])
            required=['src/drdds/msg/RobotStatus.msg','src/M20_sdk_deploy/main.cpp',
                      'src/M20_sdk_deploy/third_party/onnxruntime/arm/lib/libonnxruntime.so']
            for relative in required:
                p=checkout/relative
                if not p.is_file():raise ValueError('固定版本源码缺少所需文件：'+relative)
                with p.open('rb') as f:
                    if f.read(80).startswith(b'version https://git-lfs'):raise ValueError('源码含未下载的 Git LFS 资源，请补齐厂商依赖')
            target.mkdir(parents=True,exist_ok=True)
            for name in ['drdds','M20_sdk_deploy']:shutil.copytree(checkout/'src'/name,target/name)
        return dict(ok=True,revision=SDK_REVISION,workspace=str(self.workspace),
                    message='已准备固定版本厂商 SDK 源码与 drdds；下一步编译 ROS2 输入版，不会自动启动控制器')

    def dispatch(self,path,req):
        req=req or {}
        if path=='/status':return self.status()
        if path=='/logs':
            rows=[]
            for name in ['sdk-policy.log','sdk-joints.log']:
                p=self.data/name
                if p.is_file():
                    with p.open('rb') as f:
                        f.seek(max(0,p.stat().st_size-18000))
                        rows.append(name+'\n'+f.read().decode('utf-8',errors='replace'))
            return dict(ok=True,output='\n\n'.join(rows) or '尚无 SDK 策略或关节进程日志')
        if path=='/stop':return self.stop()
        if path=='/policy/build':return self.build(req)
        if path=='/policy/prepare':return self.prepare_source(req)
        if path=='/policy/setup':return self.setup()
        if path=='/policy/connect':
            if not req.get('mode_confirmed'):raise ValueError('请先在手柄开启 SDK 模式')
            with self.lock:
                if self.worker_alive():raise ValueError('单关节调试正在运行，请先停止关节调试')
                self.control_generation+=1;generation=self.control_generation
            policy=self.ros();deadline=time.monotonic()+1.5
            # A newly created subscription needs time for DDS discovery. A
            # missing first frame is not evidence that no controller exists.
            while True:
                with self.lock:
                    if generation!=self.control_generation:raise ValueError('SDK 连接已取消，未启动其他控制器')
                    if self.worker_alive():raise ValueError('单关节调试正在运行，请先停止关节调试')
                    state=policy.status()
                    if state.get('fresh') and state.get('source')=='ros2':
                        self.attached=True
                        return dict(ok=True,attached=True,message='已连接现有 SDK 控制器，无需再次启动')
                if time.monotonic()>=deadline:break
                time.sleep(.05)
            with self.lock:
                if generation!=self.control_generation:raise ValueError('SDK 连接已取消，未启动其他控制器')
                # start retains the existing JOINTS_CMD publisher check. Do not
                # bypass an unidentified controller just because waiting ended.
                return self.dispatch('/policy/start',req)
        if path=='/policy/start':
            if not req.get('mode_confirmed'):raise ValueError('请先确认已通过手柄进入授权的 SDK 模式')
            if self.worker_alive():raise ValueError('单关节调试正在运行，请先停止关节调试')
            if self.vendor_alive():return dict(ok=True,message='SDK 策略进程已启动')
            env=self.environment_status()
            if not env['ready']:raise ValueError(env['message']+'；请点击“准备 SDK 环境”')
            ros=self.ros()
            if ros.node.count_publishers('/JOINTS_CMD'):
                raise ValueError('检测到已有 JOINTS_CMD 发布者，请先检查并结束已有控制器')
            if not (self.workspace/'install/setup.bash').is_file():raise ValueError('SDK 工作区尚未编译')
            with (self.data/'sdk-policy.log').open('a') as log:
                self.vendor=subprocess.Popen(['ros2','run','m20_sdk_deploy','rl_deploy'],cwd=self.workspace,
                                              stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            return dict(ok=True,message='SDK 策略进程已请求启动，等待状态回报')
        if path=='/policy/status':
            try:return dict(ok=True,policy=self.ros().status(),vendor_running=self.vendor_alive())
            except ValueError as exc:return dict(ok=True,policy={},vendor_running=self.vendor_alive(),error=str(exc))
        if path=='/policy/state':return self.ros().state_command(req.get('state'))
        if path=='/policy/velocity':return self.ros().set_velocity(req)
        if path=='/policy/stop':
            self.stop()
            if self.vendor_alive() or self.attached:
                deadline=time.monotonic()+5
                while self.policy is not None and time.monotonic()<deadline:
                    state=self.policy.status()
                    if state.get('fresh') and state.get('current_state_id')==0:break
                    time.sleep(.1)
                else:raise ValueError('尚未确认 SDK 已回到 idle，保留进程；请核对阻尼状态与日志')
                if self.vendor_alive():
                    self.vendor.terminate()
                    try:self.vendor.wait(timeout=8)
                    except subprocess.TimeoutExpired:self.vendor.kill()
            message='外部 SDK 已回到 idle，进程由原启动方管理' if self.attached else 'SDK 策略进程已停止'
            self.attached=False
            return dict(ok=True,message=message)
        if path=='/direct/stop':
            self.stop()
            if self.worker_alive():self.worker.terminate();self.worker.wait(timeout=8)
            return dict(ok=True)
        if path=='/arm':
            if self.vendor_alive():raise ValueError('厂商策略进程仍在运行，请先停止后再开启单关节调试')
            if not req.get('mode_confirmed'):raise ValueError('请确认已进入授权 SDK 模式')
            if not self.worker_alive():
                if not self.calibration.is_file():raise ValueError('缺少本机关节校准文件，请在首次配置中上传')
                # Validate before starting a child with any ROS publishers.
                from sdk_bridge import JointController
                JointController(json.loads(self.calibration.read_text()))
                with (self.data/'sdk-joints.log').open('a') as log:
                    self.worker=subprocess.Popen([sys.executable,str(Path(__file__).with_name('sdk_bridge.py')),
                        '--hardware','--sdk-authorized','--rig-confirmed','--calibration',str(self.calibration),'--port','8094'],
                        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+6
                while time.monotonic()<deadline:
                    if not self.worker_alive():raise ValueError('SDK 关节服务启动失败，请在页面查看服务日志')
                    try:
                        if self.worker_rpc('/status').get('healthy'):break
                    except ValueError:pass
                    time.sleep(.1)
            return self.worker_rpc('/arm',req)
        if path in ['/lease','/trajectory']:
            if not self.worker_alive():raise ValueError('单关节调试进程未运行')
            return self.worker_rpc(path,req)
        raise ValueError('SDK 接口不存在')

    def close(self):
        self.stop()
        for proc in [self.worker,self.vendor]:
            if proc and proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:proc.kill()
        if self.policy:self.policy.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8091);args=p.parse_args()
    agent=SDKAgent()
    try:serve(args.port,agent.dispatch)
    finally:agent.close()

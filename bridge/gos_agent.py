#!/usr/bin/env python3
"""GOS health, perception and explicitly leased navigation."""
import argparse
from contextlib import nullcontext
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import threading
try:
    from .http_agent import serve
except ImportError:
    from http_agent import serve
from bridge.resident_mission import atomic
from m20.map_catalog import read_json
from m20.patrol_actions import FastBee, IoTLoginChallenge, device_id


class GOSAgent:
    def __init__(self,data=None):
        self.data=Path(data or os.environ.get('M20_DATA_DIR',Path.home()/'.local/share/m20-platform/data'))
        self.data.mkdir(parents=True,exist_ok=True)
        self.sensors=None;self.lock=threading.RLock()
        self.navigation_sensors=None;self.navigation_sensor_host='gos'
        self.navigation=None
        self.patrol_config=self.data/'gos-patrol.local.json'
        self.iot=None

    def ros(self):
        with self.lock:
            if self.sensors is None:
                try:
                    try:from .sensor_client import SensorClient
                    except ImportError:from sensor_client import SensorClient
                    self.sensors=SensorClient()
                except Exception as exc:raise ValueError('GOS ROS2 通道未就绪，请核对环境、domain 和 multicast-relay：'+str(exc))
            return self.sensors

    def navigation_ros(self):
        with self.lock:
            if self.navigation_sensors is None:
                config=read_json(self.data/'gos-nav-sensors.local.json',{})
                if not config:
                    self.navigation_sensors=self.ros()
                else:
                    from bridge.sensor_client import SensorClient
                    token=config.get('token')
                    if not isinstance(token,str) or not token:
                        raise ValueError('NOS 只读感知转发缺少连接凭据')
                    self.navigation_sensors=SensorClient(port=config.get('port'),token=token)
                    self.navigation_sensor_host='nos'
            return self.navigation_sensors

    def dispatch(self,path,req):
        if path.startswith('/iot/'):
            return self.iot_request(path,req)
        if path.startswith('/missions/'):
            with self.lock:
                if self.navigation is None:
                    from bridge.gos_navigation import GOSNavigation
                    self.navigation = GOSNavigation(self.data, self.navigation_ros)
                    self.navigation.actions.fastbee = self.iot_client()
            return self.navigation.dispatch(path, req)
        if path=='/status':
            disk=shutil.disk_usage(self.data)
            try:load=list(os.getloadavg())
            except (OSError,AttributeError):load=None
            return dict(ok=True,role='gos',hostname=socket.gethostname(),load=load,
                        disk=dict(total=disk.total,free=disk.free),ros_started=self.sensors is not None,
                        note='GOS 主机代理在线；不会自动迁移电脑程序、启动 SDK 或发布运动指令')
        if path=='/logs':
            proc=subprocess.run(['journalctl','--no-pager','-n','100','-u','m20-platform-gos.service','-u','multicast-relay.service'],
                                capture_output=True,text=True,timeout=10)
            return dict(ok=True,output=(proc.stdout+proc.stderr)[-18000:])
        if path=='/sensors/autonomy':
            mapping=bool((req or {}).get('mapping'))
            sensors=self.ros() if mapping else self.navigation_ros()
            return sensors.autonomy(mapping,**(dict(after_stamp=req['after_stamp']) if req and 'after_stamp' in req else {}))
        if path=='/sensors/status':
            status=self.navigation_ros().status()
            return dict(status,host_role='gos',navigation_sensor_host=self.navigation_sensor_host)
        if path=='/sensors/cloud':return dict(self.ros().preview((req or {}).get('topic','/ALIGNED_POINTS')),host_role='gos')
        raise ValueError('GOS 只读代理不支持此操作')

    def iot_client(self):
        with self.lock:
            if self.iot is None:
                self.iot = FastBee(read_json(self.patrol_config, {}))
            return self.iot

    def iot_request(self, path, request):
        if path == '/iot/settings' and request is not None:
            return self.configure_iot(request)
        client = self.iot_client()
        if path == '/iot/settings':
            return dict(ok=True, url=client.base, user=client.user, configured=client.configured)
        if path == '/iot/devices':
            from m20.platform_core import integer
            request = request or {}
            page = integer(request.get('page', 1), 1, 10000)
            return dict(ok=True, **client.devices(page, str(request.get('name', ''))[:120]))
        if path == '/iot/read':
            device = device_id((request or {}).get('device_id'))
            return dict(ok=True, **client.read_devices([device])[0])
        raise ValueError('物联网接口不存在')

    def configure_iot(self, request):
        old = read_json(self.patrol_config, {})
        url = str(request.get('url', '')).strip().rstrip('/')
        user = str(request.get('user', '')).strip()
        password = str(request.get('password', ''))
        token = str(request.get('token', '')).strip()
        if max(len(url), len(user), len(password)) > 500 or len(token) > 8192:
            raise ValueError('物联网连接参数过长')
        if not (password or token) and url == str(old.get('iot_url', '')).rstrip('/') and user == old.get('iot_user', ''):
            password, token = old.get('iot_password', ''), old.get('iot_token', '')
        config = dict(old, iot_url=url, iot_user=user, iot_password=password, iot_token=token)
        client = FastBee(config)
        if not client.configured:
            raise ValueError('请填写物联网 API 地址和账号凭据，或授权令牌')
        code, uuid = request.get('code', ''), request.get('uuid', '')
        try:
            if code:
                if not (isinstance(code, str) and re.fullmatch(r'[A-Za-z0-9+-]{1,16}', code)
                        and isinstance(uuid, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,96}', uuid)):
                    raise ValueError('物联网验证码无效')
                client._login(code, uuid)
            listing = client.devices()
        except IoTLoginChallenge as error:
            return dict(ok=True, challenge=error.challenge)
        config['iot_token'] = client.token
        with self.lock:
            navigation = self.navigation
            with (navigation.lock if navigation else nullcontext()):
                if navigation:
                    from bridge.gos_navigation import STOPPED
                    if navigation.nav['status'] not in STOPPED:
                        raise ValueError('请结束当前巡检后更新物联网连接')
                if read_json(self.patrol_config, {}) != old:
                    raise ValueError('物联网配置已更新，请重新验证')
                atomic(self.patrol_config, config)
                self.iot = client
                if navigation:
                    navigation.actions.fastbee = client
        return dict(ok=True, configured=True, **listing)

    def close(self):
        if self.navigation:self.navigation.close()
        if self.sensors:self.sensors.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8090);args=parser.parse_args()
    agent=GOSAgent()
    try:serve(args.port,agent.dispatch)
    finally:agent.close()

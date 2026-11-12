#!/usr/bin/env python3
"""GOS health, perception and explicitly leased navigation."""
import argparse
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
try:
    from .http_agent import serve
except ImportError:
    from http_agent import serve


class GOSAgent:
    def __init__(self,data=None):
        self.data=Path(data or os.environ.get('M20_DATA_DIR',Path.home()/'.local/share/m20-platform/data'))
        self.data.mkdir(parents=True,exist_ok=True)
        self.sensors=None;self.lock=threading.RLock()
        self.navigation=None

    def ros(self):
        with self.lock:
            if self.sensors is None:
                try:
                    try:from .sensor_client import SensorClient
                    except ImportError:from sensor_client import SensorClient
                    self.sensors=SensorClient()
                except Exception as exc:raise ValueError('GOS ROS2 通道未就绪，请核对环境、domain 和 multicast-relay：'+str(exc))
            return self.sensors

    def dispatch(self,path,req):
        if path.startswith('/missions/'):
            with self.lock:
                if self.navigation is None:
                    from bridge.gos_navigation import GOSNavigation
                    self.navigation = GOSNavigation(self.data, self.ros)
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
        if path=='/sensors/autonomy':return self.ros().autonomy(bool((req or {}).get('mapping')),**(dict(after_stamp=req['after_stamp']) if req and 'after_stamp' in req else {}))
        if path=='/sensors/status':return dict(self.ros().status(),host_role='gos')
        if path=='/sensors/cloud':return dict(self.ros().preview((req or {}).get('topic','/ALIGNED_POINTS')),host_role='gos')
        raise ValueError('GOS 只读代理不支持此操作')

    def close(self):
        if self.navigation:self.navigation.close()
        if self.sensors:self.sensors.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8090);args=parser.parse_args()
    agent=GOSAgent()
    try:serve(args.port,agent.dispatch)
    finally:agent.close()

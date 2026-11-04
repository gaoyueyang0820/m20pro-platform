#!/usr/bin/env python3
"""Root DDS reader for factory SHM compatibility. No actuator or map-write APIs."""
import argparse
import threading
try:
    from .http_agent import serve
    from .ros_sensors import RosSensors
except ImportError:
    from http_agent import serve
    from ros_sensors import RosSensors


class SensorAgent:
    def __init__(self):
        # Initialize DDS from the process main thread, before HTTP worker threads.
        self.sensor=RosSensors();self.lock=threading.Lock()

    def dispatch(self,path,request):
        if path not in ['/sensors/status','/sensors/cloud','/sensors/autonomy','/sensors/observation']:
            raise ValueError('只读感知服务不支持此操作')
        with self.lock:
            if self.sensor is None:self.sensor=RosSensors()
        if path=='/sensors/status':return self.sensor.status()
        if path=='/sensors/cloud':return self.sensor.preview((request or {}).get('topic','/ALIGNED_POINTS'))
        if path=='/sensors/observation':return self.sensor.observation()
        return self.sensor.autonomy(bool((request or {}).get('mapping')),**(dict(after_stamp=request['after_stamp']) if request and 'after_stamp' in request else {}))

    def close(self):
        if self.sensor:self.sensor.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8094);args=parser.parse_args()
    agent=SensorAgent()
    try:serve(args.port,agent.dispatch)
    finally:agent.close()

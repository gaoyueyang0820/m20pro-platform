#!/usr/bin/env python3
"""NOS map adapter. Legacy routes share the same fixed vendor operations."""
import argparse
import shutil
import subprocess
import threading
try:
    from .http_agent import serve
    from .map_service import MapService
except ImportError:
    from http_agent import serve
    from map_service import MapService

MAP_SERVICE=MapService()
SENSORS=None
SENSOR_LOCK=threading.RLock()
REFINEMENT=None
RESIDENT=None


def sensors():
    global SENSORS
    with SENSOR_LOCK:
        if SENSORS is None:
            try:
                try:from .sensor_client import SensorClient
                except ImportError:from sensor_client import SensorClient
                SENSORS=SensorClient()
            except Exception as exc:raise ValueError('ROS2 只读数据通道未就绪：'+str(exc))
        return SENSORS


def dispatch(path,req):
    global REFINEMENT,RESIDENT
    if path.startswith('/missions/'):
        with SENSOR_LOCK:
            if RESIDENT is None:
                from bridge.resident_mission import ResidentMissionService
                RESIDENT=ResidentMissionService(MAP_SERVICE,sensors,MAP_SERVICE.data)
        return RESIDENT.dispatch(path,req)
    if path=='/diagnostics/navigation':
        from m20.navigation_diagnostics import read_navigation_diagnostics
        result=read_navigation_diagnostics()
        try:
            status=sensors().status()
            result['sensors']={k:status.get(k) for k in ['nav_publishers','localization_quality','matching_quality','scan_diagnostics']}
        except Exception as exc:result['sensor_error']=str(exc)
        return result
    if path=='/maps/refinement':
        with SENSOR_LOCK:
            if REFINEMENT is None:
                try:from .map_refinement_agent import RefinementAgent
                except ImportError:from map_refinement_agent import RefinementAgent
                REFINEMENT=RefinementAgent(MAP_SERVICE,sensors)
        return REFINEMENT.control(req or {})
    if path=='/sensors/observation':return dict(sensors().observation(),map_name=MAP_SERVICE.path('active').name)
    if path=='/sensors/autonomy':return sensors().autonomy(bool((req or {}).get('mapping')),**(dict(after_stamp=req['after_stamp']) if req and 'after_stamp' in req else {}))
    if path=='/sensors/status':return sensors().status()
    if path=='/sensors/cloud':return sensors().preview((req or {}).get('topic','/ALIGNED_POINTS'))
    if path.startswith(('/maps/','/files/')):return MAP_SERVICE.dispatch(path,req)
    if path=='/status':
        result=MAP_SERVICE.library();services={}
        for service in ['localization','planner','charge_manager']:
            p=subprocess.run(['systemctl','is-active',service+'.service'],capture_output=True,text=True,timeout=3)
            services[service]=p.stdout.strip()
        return dict(result,services=services,drmap=shutil.which('drmap'))
    if path=='/logs':
        p=subprocess.run(['journalctl','--no-pager','-n','100','-u','localization.service','-u','planner.service'],
                         capture_output=True,text=True,timeout=10)
        jobs=MAP_SERVICE.dispatch('/maps/build/status',{})
        output=p.stdout[-18000:]+p.stderr[-1000:]
        for kind in ['mapping','stop_mapping']:
            record=jobs[kind]
            output+='\n\n'+kind+' · '+record.get('message','')+'\n'+record.get('output','')
        return dict(ok=True,output=output)
    if path=='/active-map':return MAP_SERVICE.grid('active')
    if path=='/action' and req is not None:
        action=req.get('action')
        if action in ['mapping','stop_mapping']:
            return MAP_SERVICE.dispatch('/maps/build',dict(action='start' if action=='mapping' else 'stop',name=req.get('name','m20-map')))
        if action=='pack':return MAP_SERVICE.dispatch('/maps/package/export',dict(name='active'))
        if action=='apply':return MAP_SERVICE.dispatch('/maps/apply',dict(name=req.get('name')))
    raise ValueError('NOS 接口或操作不存在')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8093);args=parser.parse_args()
    from bridge.resident_mission import ResidentMissionService
    RESIDENT=ResidentMissionService(MAP_SERVICE,sensors,MAP_SERVICE.data)
    if RESIDENT.enabled:RESIDENT.recover()
    try:serve(args.port,dispatch)
    finally:
        if RESIDENT:RESIDENT.close()
        if SENSORS:SENSORS.close()

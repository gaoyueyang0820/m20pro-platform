"""Explicit supervised live test: 2 m / 60 s, then stop and save.

Run only after current-session field confirmation. Does not change active map.
Logs API evidence without credentials. An independent timer requests stopping.
"""
import json
import math
from pathlib import Path
import threading
import time
from urllib.request import Request,urlopen
from urllib.error import HTTPError

ROOT=Path(__file__).resolve().parents[1]
BASE='http://127.0.0.1:8080'
def call(path,data=None,timeout=3):
    req=Request(BASE+path,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json'})
    try:
        with urlopen(req,timeout=timeout) as response:return json.load(response)
    except HTTPError as exc:raise RuntimeError(exc.read().decode())

def main():
    stamp=time.strftime('%Y%m%d-%H%M%S');out=ROOT/'data/network'/('bounded-'+stamp+'.json')
    result=dict(started_at=stamp,limits=dict(radius_m=2,duration_s=60),rows=[])
    state=call('/api/status');build=call('/api/maps/build/status',timeout=6)
    assert state['mode']=='live' and state['online'] and not state['autonomy']['active']
    assert not any(state['velocity']) and state['nav']['status'] not in ['running','cancelling']
    assert not build['in_progress'] and state['basic']['MotionState']==17 and state['basic']['HES']==0
    result['initial_state']={k:state.get(k) for k in ['basic','pose','motion','velocity','mode','ip']}
    if state['basic']['Sleep']:
        call('/api/sleep',dict(sleep=False));time.sleep(.5)
    done=threading.Event()
    def stop_at_deadline():
        if not done.wait(60):
            try:call('/api/autonomy/stop',{})
            except Exception as exc:result['deadline_stop_error']=str(exc)
    threading.Thread(target=stop_at_deadline,daemon=True).start()
    began=time.monotonic();origin=None;previous=None;progress_at=began;distance=0
    try:
        start=call('/api/autonomy/start',dict(mode='explore',name='m20-short-'+stamp,radius=2,minutes=1,forward_limit=.35,yaw_limit=.70),timeout=10)
        result['run_id']=start['run_id'];print(json.dumps(dict(started=True,run_id=start['run_id'])),flush=True)
        while time.monotonic()-began<60:
            call('/api/autonomy/lease',dict(run_id=start['run_id']))
            state=call('/api/status');a=state['autonomy'];pose=(a.get('map') or {}).get('pose')
            row=dict(elapsed=round(time.monotonic()-began,2),state=a['state'],message=a['message'],command=state['velocity'],motion={k:state['motion'].get(k) for k in ['LinearX','LinearY','OmegaZ']},pose=pose)
            result['rows'].append(row)
            if pose:
                if origin is None:origin=pose[:2]
                distance=max(distance,math.dist(pose[:2],origin))
                if previous is None or math.dist(pose[:2],previous[:2])>.04 or abs(math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2])))>.08:
                    previous=pose;progress_at=time.monotonic()
            if len(result['rows'])%5==0:print(json.dumps(row,ensure_ascii=False),flush=True)
            if not a['active'] or a.get('run_id')!=start['run_id']:result['reason']=a['message'];break
            if distance>=1.8:result['reason']='reached supervised radius';break
            if any(state['velocity']) and time.monotonic()-progress_at>8:result['reason']='no measurable progress';break
            if not state['online'] or state['basic']['HES']!=0:result['reason']='robot not ready';break
            time.sleep(.15)
    except Exception as exc:result['error']=str(exc)
    finally:
        done.set()
        try:result['stop']=call('/api/autonomy/stop',{})
        except Exception as exc:result['stop_error']=str(exc)
        result['max_radius_m']=round(distance,3);result['elapsed']=round(time.monotonic()-began,2)
        try:
            result['final_state']=call('/api/status')
            build=call('/api/maps/build/status',timeout=6)
            if build['in_progress'] and build['phase']!='saving' and result['final_state']['autonomy']['state']!='saving':result['save']=call('/api/autonomy/save',{},timeout=8)
            result['build_at_stop']=build
        except Exception as exc:result['save_error']=str(exc)
        out.write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(dict(file=str(out),reason=result.get('reason'),error=result.get('error'),max_radius_m=result['max_radius_m'],elapsed=result['elapsed'],save=result.get('save'),save_error=result.get('save_error')),ensure_ascii=False),flush=True)

if __name__=='__main__':main()

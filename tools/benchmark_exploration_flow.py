#!/usr/bin/env python3
"""Occluded multi-vantage exploration and stuck recovery, ideal plant only.

No network/robot/8080. Zero-command duration is a model metric, not observed
hardware parking time. Original expiry, obstacle and ground rules are retained.
"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from autonomy_core import Explorer,cell,disk,clearance
from autonomy_runtime import MotionProgress
from local_trajectory import TrajectoryFollower,exploration_step
from tools.benchmark_exploration_control_v3 import sensed_world,body_collision
from tools.benchmark_frontier_exploration import make_world


def load_old():
    folder=ROOT/'data/reviews/exploration-flow-20261102/before'
    loaded=[]
    for filename in ['autonomy_core.py','local_trajectory.py']:
        spec=importlib.util.spec_from_file_location('flow_before_'+filename.replace('.','_'),folder/filename)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);loaded.append(module)
    return loaded[0].Explorer,loaded[1].TrajectoryFollower


def multi_vantage(modern=True,steps=1100,classes=None):
    cls,follow_cls=classes or (Explorer,TrajectoryFollower)
    free,walls=make_world(2)
    clutter={(x,y) for x in range(5,9) for y in range(4,8)}|{(x,y) for x in range(29,33) for y in range(-9,-5)}
    free-=clutter;walls|=clutter
    e=cls(14,.35,.7);f=follow_cls(.35,.7);pose=np.array([0.,0.,0.]);velocity=np.zeros(3)
    key=None;last_plan=-10.;seen=set();trace=[];dt=.2;path_length=0.;zero_s=0.;zero_run=0.;max_zero=0.;handoffs=0;reason='budget';collision=False
    start=time.perf_counter()
    for k in range(steps):
        scan,visible=sensed_world(pose,free,walls,1+k*dt,velocity);scan['age']=.3;seen|=visible;e.ingest(scan)
        if (k*dt-last_plan>1 or not e.path) and e.recovery is None and not getattr(e,'recovery_pending',False):
            outcome=e.plan();last_plan=k*dt
            if outcome=='exhausted':reason='exhausted';break
        axes=[0.,0.,0.];detail={};verified=False
        if e.path and e.recovery is None:
            if modern:detail=exploration_step(e,f,scan,float(velocity[2]),new_scan=True)
            else:
                current=tuple(e.path)
                if current!=key:f.set_path([(x*.2,y*.2) for x,y in current]);key=current
                detail=f.command(scan,float(velocity[2]),allowed_pose=lambda x,y,a:math.dist((x,y),e.origin)<=e.radius)
            if detail.get('continuation_state')=='joined':handoffs+=1
            if detail['state']=='arrived':e.visited.add(e.goal);e.path=[];last_plan=-10.;verified=True
            elif detail['state']=='tracking':axes=detail['axes'];verified=True
            elif detail['state']=='braking':verified=True
            else:axes=e.command_axes(float(velocity[2]),.3)
        else:axes=e.command_axes(float(velocity[2]),.3)
        if not verified and not clearance(scan,axes[0],axes[2],side=axes[1])[0]:axes=[0.,0.,0.]
        if modern:e.note_recovery_command(axes)
        if any(axes):zero_run=0.
        else:zero_s+=dt;zero_run+=dt;max_zero=max(max_zero,zero_run)
        old=pose.copy();velocity+=np.clip(np.asarray(axes)-velocity,[-.7*dt,-.7*dt,-1.8*dt],[.7*dt,.7*dt,1.8*dt])
        theta=pose[2]+velocity[2]*dt*.5;c,s=math.cos(theta),math.sin(theta)
        pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*dt
        path_length+=float(np.linalg.norm(pose[:2]-old[:2]));collision=body_collision(pose,free)
        trace.append(dict(t=round(k*dt,2),pose=pose.tolist(),axes=list(axes),coverage=len(seen)/len(free),controller=detail.get('state'),zero_reason=detail.get('zero_reason'),handoff=detail.get('continuation_state')))
        if collision:reason='collision';break
        if len(seen)/len(free)>=.98:reason='coverage_98_percent';break
    exploration_s=(k+1)*dt;stop_steps=0
    while np.linalg.norm(velocity[:2])>=.08 or abs(velocity[2])>=.12:
        old=pose.copy();velocity+=np.clip(-velocity,[-.5*dt,-.5*dt,-1.2*dt],[.5*dt,.5*dt,1.2*dt])
        theta=pose[2]+velocity[2]*dt*.5;c,s=math.cos(theta),math.sin(theta)
        pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*dt
        path_length+=float(np.linalg.norm(pose[:2]-old[:2]));collision=collision or body_collision(pose,free);stop_steps+=1
        if stop_steps>=10:break
    return dict(modern=modern,steps=k+1,simulated_s=round(exploration_s+stop_steps*dt,2),exploration_s=round(exploration_s,2),
                required_final_stop_s=round(stop_steps*dt,2),final_stopped=bool(np.linalg.norm(velocity[:2])<.08 and abs(velocity[2])<.12),
                coverage=round(len(seen)/len(free),4),distance_m=round(path_length,2),
                zero_command_s=round(zero_s,2),max_zero_run_s=round(max_zero,2),handoffs=handoffs,collision=collision,end_reason=reason,
                elapsed_s=round(time.perf_counter()-start,3),trace=trace)


def stuck_loop(feedback=True,all_axes_blocked=False,steps=160):
    e=Explorer(4);f=TrajectoryFollower();p=MotionProgress();pose=np.zeros(3);velocity=np.zeros(3);axes=np.zeros(3)
    requested=[];trace=[];reason='budget';dt=.2
    for k in range(steps):
        now=k*dt;floor=[list(v) for v in disk(3.)]
        scan=dict(pose=pose.tolist(),floor=floor,obstacles=[],stamp=now+1,age=.1,
                  _velocity_body=velocity[:2].tolist() if feedback else None,_speed_mps=float(np.linalg.norm(velocity[:2])))
        e.ingest(scan);p.observe_command(axes,now);p.update(pose,None,0.,now,moving=bool(np.linalg.norm(velocity[:2])>.08))
        why=p.stuck_reason(now)
        if why and e.recovery is None and not e.recovery_pending and feedback:
            if not e.request_recovery(p.failed_axes(),why):reason='bounded_recovery_exhausted';break
            requested.append(now);p.recovery_started();axes=np.zeros(3)
        elif p.stalled(now):reason='finite_no_progress_stop';axes=np.zeros(3);break
        elif e.recovery is not None or e.recovery_pending:
            e._current_yaw_rate=float(velocity[2]);axes=np.asarray(e._recover_axes())
        else:
            if not e.path:e.plan()
            if e.path:
                detail=exploration_step(e,f,scan,float(velocity[2]),new_scan=True)
                axes=np.asarray(detail['axes'])
            else:axes=np.zeros(3)
        if not feedback:axes=np.zeros(3)
        e.note_recovery_command(axes)
        # Forward input is ignored until a sideways escape genuinely changes
        # the position. Complete actuator nonresponse is a separate scenario.
        response=axes.copy()
        if all_axes_blocked:response[:]=0
        elif abs(pose[1])<.3 and response[0]>0:response[0]=0
        velocity=response
        c,s=math.cos(pose[2]),math.sin(pose[2]);pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*dt
        trace.append(dict(t=now,pose=pose.tolist(),axes=axes.tolist(),pending=e.recovery_pending,recovery=e.recovery is not None,episode=e.recovery_episode,attempts=e.recovery_attempts))
        if np.linalg.norm(pose[:2])>.85 and requested:reason='escaped_to_new_space';break
    return dict(feedback=feedback,all_axes_blocked=all_axes_blocked,reason=reason,simulated_s=round((k+1)*dt,2),
                pose=pose.tolist(),requests=requested,total_attempts=e.recovery_total_attempts,tried=sorted(e.recovery_tried),
                outcomes=e.recovery_outcomes,trace=trace)


def main():
    report=dict(scope='synthetic occluded 2D world and ideal kinematics; no hardware validation',
                baseline=multi_vantage(False,classes=load_old()),updated=multi_vantage(True),
                stuck_escape=stuck_loop(),no_actuator_response=stuck_loop(all_axes_blocked=True),missing_feedback=stuck_loop(feedback=False))
    output=ROOT/'data/reviews/exploration-flow-20261102/flow-benchmark.json';output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:({a:b for a,b in v.items() if a!='trace'} if isinstance(v,dict) else v) for k,v in report.items()},ensure_ascii=False,indent=2))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Offline Explorer + sampled-controller replay in an occluded two-room world.

Normalized axes drive an acceleration-bounded ideal kinematic plant. The plant
uses 1 m/s or rad/s per unit axis; this is NOT measured M20 calibration, physics,
real sensing, network timing, robot control or a doorway hardware acceptance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from autonomy_core import Explorer,clearance,cell,disk,RESOLUTION
from exploration_frontiers import ray_cells
from local_trajectory import TrajectoryFollower
from tools.benchmark_frontier_exploration import make_world


def sensed_world(pose,floor,walls,stamp,velocity,radius=2.4):
    c,s=math.cos(pose[2]),math.sin(pose[2]);floor_scan=[];obstacles=[];seen=set();p=cell(*pose[:2])
    for ix,iy in disk(radius):
        wx=pose[0]+c*ix*.2-s*iy*.2;wy=pose[1]+s*ix*.2+c*iy*.2;q=cell(wx,wy)
        if q not in floor and q not in walls:continue
        if any(v in walls for v in list(ray_cells(p,q))[1:-1]):continue
        if q in floor:floor_scan.append([ix,iy]);seen.add(q)
        else:obstacles.append([ix,iy])
    return dict(pose=pose.tolist(),floor=floor_scan,obstacles=obstacles,stamp=stamp,
                _velocity_body=velocity[:2].tolist(),_speed_mps=float(np.linalg.norm(velocity[:2]))),seen


def body_collision(pose,floor):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    for x in np.linspace(-.42,.42,13):
        for y in np.linspace(-.225,.225,9):
            if cell(pose[0]+c*x-s*y,pose[1]+s*x+c*y) not in floor:return True
    return False


def run(steps=750,dt=.2,stop_after_door=False):
    floor,walls=make_world(2);pose=np.array([0.,0.,0.]);velocity=np.zeros(3)
    explorer=Explorer(14,.35,.7);follower=TrajectoryFollower(.35,.7)
    key=None;last_plan=-100.;seen=set();trace=[];distance=0.;first_door=None;last_progress=0.;anchor=pose[:2].copy()
    goal_changes=0;old_goal=None;fallbacks=0;reason='budget';collision=False;began=time.perf_counter()
    for k in range(steps):
        now=k*dt;scan,visible=sensed_world(pose,floor,walls,now+1,velocity);seen|=visible;explorer.ingest(scan)
        plan_state=explorer.plan_state
        if now-last_plan>=1 or not explorer.path:
            plan_state=explorer.plan();last_plan=now
        if explorer.goal!=old_goal:goal_changes+=1;old_goal=explorer.goal
        detail={};axes=[0.,0.,0.];verified=False
        if explorer.path and explorer.recovery is None:
            new_key=tuple(explorer.path)
            if new_key!=key:follower.set_path([(x*.2,y*.2) for x,y in new_key]);key=new_key
            detail=follower.command(scan,velocity[2],allowed_pose=lambda x,y,a:math.dist((x,y),explorer.origin)<=explorer.radius)
            if detail['state']=='arrived':explorer.visited.add(explorer.goal);explorer.path=[];last_plan=-100.;verified=True
            elif detail['state']=='tracking':axes=detail['axes'];verified=True
            else:axes=explorer.command_axes(velocity[2],0.);fallbacks+=1
        elif plan_state not in ('exhausted','blocked'):
            axes=explorer.command_axes(velocity[2],0.);fallbacks+=1
        if not verified and not clearance(scan,axes[0],axes[2],side=axes[1])[0]:axes=[0.,0.,0.]
        explorer.note_recovery_command(axes)
        old_pose=pose.copy();wanted=np.array(axes)
        velocity+=np.clip(wanted-velocity,[-.7*dt,-.7*dt,-1.8*dt],[.7*dt,.7*dt,1.8*dt])
        theta=pose[2]+velocity[2]*dt*.5;c,s=math.cos(theta),math.sin(theta)
        pose+=np.array([c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1],velocity[2]])*dt
        distance+=float(np.linalg.norm(pose[:2]-old_pose[:2]));collision=body_collision(pose,floor)
        if np.linalg.norm(pose[:2]-anchor)>.2:anchor=pose[:2].copy();last_progress=now
        trace.append(dict(t=round(now,2),pose=pose.tolist(),axes=list(axes),velocity=velocity.tolist(),coverage=round(len(seen)/len(floor),4),goal=explorer.goal,plan=plan_state,controller=detail.get('state'),reason=detail.get('reason'),candidate_count=detail.get('valid_candidates'),path_index=detail.get('path_index'),path_points=detail.get('path_points'),controller_ms=detail.get('compute_ms'),fallback=not verified))
        if first_door is None and pose[0]>3.6:first_door=dict(time_s=round(now,2),distance_m=round(distance,2),pose=pose.tolist())
        if collision:reason='body_collision';break
        if first_door and stop_after_door:reason='first_door_crossed';break
        if len(seen)/len(floor)>=.98:reason='coverage_98_percent';break
        if now-last_progress>18:reason='no_translation_progress_18s';break
        if plan_state=='exhausted':reason='exploration_exhausted';break
    timing=[p['controller_ms'] for p in trace if p['controller_ms'] is not None]
    return dict(scope='offline continuous ideal kinematics with occluded 2.4m grid sensing; NOT physical or hardware validation',network_or_robot_io=False,dt_s=dt,plant_gain=1.,forward_limit=.35,yaw_limit=.7,steps=k+1,simulated_s=round((k+1)*dt,2),elapsed_s=round(time.perf_counter()-began,3),coverage=round(len(seen)/len(floor),4),distance_m=round(distance,2),goal_changes=goal_changes,first_door_crossed=first_door,collision=collision,end_reason=reason,fallback_count=fallbacks,controller_ms_p95=round(float(np.percentile(timing,95)),2) if timing else None,last_frontier_metrics=explorer.frontier_metrics,trace=trace)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--steps',type=int,default=750);parser.add_argument('--stop-after-door',action='store_true');args=parser.parse_args()
    names=['autonomy_core.py','exploration_frontiers.py','local_trajectory.py','tools/benchmark_exploration_control_v3.py']
    before={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in names}
    result=run(args.steps,stop_after_door=args.stop_after_door)
    result['sources']=before
    result['source_changed_during_run']=[p for p in names if before[p]!=hashlib.sha256((ROOT/p).read_bytes()).hexdigest()]
    out=ROOT/'data/reviews/autonomy-v3-20261101/exploration-control-benchmark.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['trace','sources']},ensure_ascii=False,indent=2));print(out)


if __name__=='__main__':main()

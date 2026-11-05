#!/usr/bin/env python3
"""Offline multi-room target-layer replay with occluded 360-degree grid scans.

The plant traverses one planned grid edge per step with a fixed body heading.
This isolates target selection from joystick calibration and contact dynamics;
it is NOT a real controller/robot acceptance. No I/O except local result files.
"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from m20.autonomy_core import Explorer
from m20.exploration_frontiers import ray_cells


def make_world(rooms):
    free=set();obstacles=set()
    for x in range(-15,rooms*30-15+1):
        for y in range(-15,16):
            wall=x in [-15,rooms*30-15] or y in [-15,15]
            for n in range(1,rooms):
                door=0 if n%2 else 5
                if x==n*30-15 and abs(y-door)>4:wall=True
            (obstacles if wall else free).add((x,y))
    return free,obstacles


def run(cls,rooms,max_steps=550):
    floor,walls=make_world(rooms);p=(0,0);e=cls(rooms*6+2);seen=set();distance=0.;switches=0;previous=None
    sensor=12;offsets=[(x,y) for x in range(-sensor,sensor+1) for y in range(-sensor,sensor+1) if x*x+y*y<=sensor*sensor]
    checkpoints={};trace=[];reached=set();began=time.perf_counter();times=[];reason='budget'
    for step in range(max_steps):
        local_free=[];local_obstacles=[]
        for dx,dy in offsets:
            q=(p[0]+dx,p[1]+dy)
            if q not in floor and q not in walls:continue
            occluded=any(v in walls for v in list(ray_cells(p,q))[1:-1])
            if occluded:continue
            if q in floor:local_free.append([dx,dy]);seen.add(q)
            else:local_obstacles.append([dx,dy])
        e.ingest(dict(pose=[p[0]*.2,p[1]*.2,0.],floor=local_free,obstacles=local_obstacles,stamp=step+1.,_speed_mps=.2))
        reached.add(min(rooms-1,max(0,(p[0]+15)//30)))
        coverage=len(seen)/len(floor)
        for milestone in [10,20,30,50]:
            if distance>=milestone and str(milestone) not in checkpoints:checkpoints[str(milestone)]=round(coverage,4)
        trace.append([round(p[0]*.2,2),round(p[1]*.2,2),round(coverage,4)])
        if coverage>=.98:reason='coverage_98_percent';break
        started=time.perf_counter();state=e.plan();times.append(time.perf_counter()-started)
        if state!='planned':reason=state;break
        if e.goal!=previous:switches+=1;previous=e.goal
        while e.path and e.path[0]==p:e.visited.add(e.path.pop(0))
        if not e.path:continue
        q=e.path[0]
        if q not in floor:raise AssertionError('planner left ground truth floor')
        distance+=math.dist(p,q)*.2;p=q
    return dict(rooms=rooms,steps=step+1,distance_m=round(distance,2),coverage=round(len(seen)/len(floor),4),rooms_entered=len(reached),goal_changes=switches,end_reason=reason,coverage_at_distance_m=checkpoints,plan_ms_p95=round(sorted(times)[int(.95*(len(times)-1))]*1000,2) if times else None,elapsed_s=round(time.perf_counter()-began,3),trace=trace)


def main():
    before=ROOT/'data/reviews/autonomy-v3-20261101/before/autonomy_core.py'
    spec=importlib.util.spec_from_file_location('frontier_baseline',before);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result={'scope':'offline occluded-grid target-selection replay, ideal tracking, not physical validation','network_or_robot_io':False,'scan_radius_m':2.4,'completion_coverage':.98,'scenarios':[]}
    for rooms in [2,3]:
        row={'name':f'{rooms}_rooms_offset_doors','baseline':run(module.Explorer,rooms),'frontier_v3':run(Explorer,rooms)}
        result['scenarios'].append(row)
        print(json.dumps({k:v if k=='name' else {a:b for a,b in v.items() if a!='trace'} for k,v in row.items()},ensure_ascii=False),flush=True)
    out=ROOT/'data/reviews/autonomy-v3-20261101/frontier-benchmark.json';out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(out)


if __name__=='__main__':main()

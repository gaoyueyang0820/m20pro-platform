#!/usr/bin/env python3
"""Offline geometric benchmark. Does not open sockets or contact the platform.

Compares the locally preserved previous planner with current source. Coordinates
and obstacles are synthetic, except the explicitly labelled recorded-map case.
No velocity controller, contact dynamics or robot success is inferred.
"""
import hashlib,importlib.util,json,math,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from navigation_planner import GridPlanner
from test_navigation_planning_v3 import image_room,validate_sampled_body
OUT=ROOT/'data/reviews/autonomy-v3-20261101'
old_path=OUT/'before/navigation_planner.py'
spec=importlib.util.spec_from_file_location('baseline_navigation',old_path);baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)


def raw(image):
    h,w=image.shape;return ('P5\n%d %d\n255\n'%(w,h)).encode()+image.tobytes()


def cases():
    yield dict(name='open_oblique',image=image_room(200,160),start=[2.01,2.02,.1],goal=[17.73,12.81])
    a=image_room(100,80);a[15:70,49:51]=0
    yield dict(name='wall_two_routes',image=a,start=[2,3,0],goal=[8,3])
    h=w=140;yy,xx=np.mgrid[:h,:w];wx=(xx+.5)*.1;wy=(h-yy-.5)*.1
    angle=.29;c,s=math.cos(angle),math.sin(angle);x=wx-2;y=wy-3
    a=np.where((c*x+s*y>-.8)&(c*x+s*y<10.5)&(abs(-s*x+c*y)<.43),254,0).astype(np.uint8)
    yield dict(name='oblique_narrow_passage',image=a,start=[2,3,angle],goal=[2+8*c,3+8*s])
    a=np.zeros((60,120),np.uint8);a[26:34,2:118]=254
    yield dict(name='reverse_narrow_corridor',image=a,start=[8.05,3.05,0],goal=[2.05,3.05],yaw=0)
    a=image_room(200,160);a[35:125,60:64]=0;a[35:125,130:134]=0;a[121:125,60:134]=0
    yield dict(name='u_shaped_detour',image=a,start=[9.5,5,math.pi/2],goal=[9.5,2])
    real=ROOT/'data/maps/8495c42ad3.pgm'
    if real.exists():yield dict(name='recorded_room_door_goal',raw=real.read_bytes(),origin=[-5.3,-29.9,0],start=[1.057126,.371620,-1.278329],goal=[4,-1.5],evidence='recorded PGM only; start from saved localization, not a live run',map_sha256=hashlib.sha256(real.read_bytes()).hexdigest())


def run_case(case,cls):
    timing=[];final=None;p=None
    for _ in range(3):
        begin=time.monotonic()
        try:
            p=cls(case.get('raw') or raw(case['image']),.1,case.get('origin',[0,0,0]))
            final=p.plan(case['start'],case['goal'],case.get('yaw'),budget=3)
            timing.append(time.monotonic()-begin)
        except ValueError as exc:return dict(ok=False,error=str(exc),elapsed_s=round(time.monotonic()-begin,4))
    result=dict(ok=True,median_elapsed_s=round(statistics.median(timing),4),length_m=final['length_m'],expanded=final['expanded'],guide_points=len(final['points']))
    if 'dense_path' in final:result.update(body_oracle_passed=validate_sampled_body(p,final),diagnostics=final['diagnostics'])
    return result


def main():
    rows=[]
    for case in cases():
        row=dict(name=case['name'],baseline=run_case(case,baseline.GridPlanner),current=run_case(case,GridPlanner))
        if 'map_sha256' in case:row.update(map_sha256=case['map_sha256'],evidence=case['evidence'])
        rows.append(row)
    a=image_room();p=GridPlanner(raw(a),.1,[0,0,0]);start=[2.05,5.05,0];goal=[10.05,5.05]
    initial=p.plan(start,goal);q=initial['dense_path'][20]
    block=[(float(x),float(y)) for x in np.arange(6,7,.09) for y in np.arange(4.4,5.8,.09)]
    changed=p.update_obstacles(block);begin=time.monotonic();new=p.plan([q['x'],q['y'],q['yaw']],goal,budget=3);replan_elapsed=time.monotonic()-begin
    same=p.update_obstacles(block);p.update_obstacles([]);cleared=p.plan(start,goal)
    result=dict(scope='OFFLINE_GEOMETRIC_PLANNING_NOT_PHYSICAL_VALIDATION',baseline_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),current_sha256=hashlib.sha256((ROOT/'navigation_planner.py').read_bytes()).hexdigest(),cases=rows,
      dynamic_replan=dict(progress_pose=q,new_obstacle_cells=changed['changed_cells'],elapsed_s=round(replan_elapsed,4),length_m=new['length_m'],body_oracle_passed=validate_sampled_body(GridPlanner(raw(a),.1,[0,0,0],obstacles=block),new),unchanged_layer_changed_cells=same['changed_cells'],removal_method=cleared['diagnostics']['method']))
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'navigation-benchmark.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if all(row['current']['ok'] and row['current'].get('body_oracle_passed') for row in rows if row['name']!='recorded_room_door_goal') and result['dynamic_replan']['body_oracle_passed'] else 1
if __name__=='__main__':raise SystemExit(main())

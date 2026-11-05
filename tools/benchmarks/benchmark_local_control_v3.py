"""Repeatable ideal-kinematic local controller scenarios, without robot I/O."""
from pathlib import Path
import hashlib
import json
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from test_local_trajectory import run_closed_loop,world_grid


def main():
    names=['m20/local_trajectory.py','m20/point_navigation.py','m20/navigation_planner.py','tests/test_local_trajectory.py']
    hashes={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names};cases=[]
    for name in ['turning','door','wall','new_obstacle','reverse']:
        grid=world_grid();start=[3,3.5,0];goal=[8,3.5,0]
        if name=='turning':start=[2,3,0];goal=[5,5,.5]
        if name=='door':grid[:,54:56]=0;grid[40:50,54:56]=254;start=[3,4.5,0];goal=[8,4.5,0]
        if name=='wall':grid[35:75,54:56]=0
        if name=='reverse':grid[:40]=0;grid[50:]=0;start=[7,4.5,0];goal=[3,4.5,0]
        began=time.monotonic();result=run_closed_loop(grid,start,goal,steps=650,dynamic=name=='new_obstacle',gain=.8 if name=='turning' else 1.)
        result.update(name=name,wall_seconds=round(time.monotonic()-began,3));cases.append(result)
    report=dict(scope='closed_loop_ideal_kinematics_with_acceleration_and_occluded_scan_not_hardware',source_hashes=hashes,
                source_changed_during_run=[n for n,h in hashes.items() if hashlib.sha256((ROOT/n).read_bytes()).hexdigest()!=h],cases=cases)
    path=ROOT/'data/reviews/autonomy-v3-20261101/local-control-benchmark.json';path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in c.items() if k!='trace'} for c in cases],ensure_ascii=False,indent=2))
    return 0 if all(c['arrived'] and not c['collision'] for c in cases) and not report['source_changed_during_run'] else 1


if __name__=='__main__':raise SystemExit(main())

"""Read-only interpretation of the installed planner's monitor log.

No controller commands, parameter changes or service operations are exposed.
A zero-output monitor is evidence of execution stagnation, not proof of why.
"""
import math,re,time
from pathlib import Path


def parse_navigation_monitor(text):
    numeric=r'([+-]?(?:\d+(?:\.\d*)?|\.\d+))'
    blocks=re.findall(r'\|\s*Odom\[[^\n]*\n.*?\|\s*Parameters[^\n]*(?:\n|$)',text,re.S)
    rows=[]
    for block in blocks:
        flags=dict(re.findall(r'(Odom|Map|Goal|Cancel)\[([^\]]+)\]',block))
        mode=re.search(r'\|\s*Odom[^\n]*Mode:\s*([^\n]+)',block)
        command=re.search(r'Cmd Velocity[^\n]*x_vel\s*=\s*'+numeric+r'[^\n]*y_vel\s*=\s*'+numeric+r'[^\n]*yaw_vel\s*=\s*'+numeric,block)
        code=re.search(r'Planner Status[^\n]*Code\s*=\s*(\d+)',block)
        path=re.search(r'Global Path:\s*(\d+)\s*waypoints',block)
        distance=re.search(r'与目标距离\s*=\s*'+numeric,block)
        stop=re.search(r'停障距离\s*=\s*'+numeric,block)
        if command is None or code is None:continue
        values=[float(v) for v in command.groups()]
        if not all(math.isfinite(v) for v in values):continue
        row=dict(inputs=flags,mode=mode[1].strip() if mode else None,code=int(code[1]),command=dict(zip(['forward','side','yaw'],values)),path_points=int(path[1]) if path else 0,
                 goal_distance_m=float(distance[1]) if distance else None,reported_stop_distance_m=float(stop[1]) if stop else None)
        row['zero_output_with_path']=bool(flags.get('Goal')=='YES' and flags.get('Cancel')=='NO' and row['path_points']>0 and max(abs(v) for v in values)<.0001 and (row['goal_distance_m'] or 0)>.15)
        rows.append(row)
    if not rows:return dict(available=False,message='当前日志格式中未找到完整的规划监控记录')
    current=rows[-1];streak=0
    for row in reversed(rows):
        if not row['zero_output_with_path']:break
        streak+=1
    recent=[r for r in rows if r['inputs'].get('Goal')=='YES']
    message='当前没有活动导航目标' if current['inputs'].get('Goal')!='YES' else '已读到原厂规划监控'
    if streak>=4:message='全局路径存在，但日志连续记录零速度输出；需核对局部规划、障碍投影及实际参数'
    return dict(available=True,current=current,consecutive_zero_output_records=streak,
                recent_active_zero_records=sum(r['zero_output_with_path'] for r in recent),records=len(rows),message=message,
                boundary='日志不能单独确定阻挡原因，也不表示可以关闭避障')


def read_navigation_diagnostics(log_root=Path('/var/opt/robot/log')):
    files=list(Path(log_root).glob('*/planner.*.log'))
    if not files:return dict(ok=True,available=False,message='没有找到原厂导航监控日志')
    path=max(files,key=lambda p:p.stat().st_mtime)
    with path.open('rb') as source:
        source.seek(max(0,path.stat().st_size-1024*1024));body=source.read().decode('utf-8',errors='replace')
    result=parse_navigation_monitor(body);age=time.time()-path.stat().st_mtime
    return dict(result,ok=True,log_name=path.name,log_age_s=round(age,2) if age>=0 else None,
                fresh=bool(0<=age<3),read_only=True)

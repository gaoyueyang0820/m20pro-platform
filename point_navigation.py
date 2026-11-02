"""Computer-side point navigation using global search + live trajectory control.

Uses the supported normal-mode joystick interface, not the vendor point-goal
executor. Never falls back between the two executors while a task is moving.
Robot locomotion/balance remain vendor controlled. Live acceptance is separate.
"""
import hashlib
import copy
import math
from pathlib import Path
import threading
import time
import uuid

from local_trajectory import TrajectoryFollower, wrap
from navigation_planner import GridPlanner
from platform_core import age_of
from start_recovery_runtime import plan_with_recovery, TranslationRecovery


def simulated_scan(planner,pose,stamp=None):
    """Occlusion-aware planar test sensor; not a physical lidar simulator."""
    c,s=math.cos(pose[2]),math.sin(pose[2]);floor=set();obstacles=set();bounds={}
    for k in range(180):
        angle=k*math.tau/180
        for j in range(1,45):
            d=j*.1;px,py=d*math.cos(angle),d*math.sin(angle)
            x,y=pose[0]+c*px-s*py,pose[1]+s*px+c*py;ix,iy=planner.cell(x,y)
            q=(round(px/.2),round(py/.2))
            if not(0<=ix<planner.w and 0<=iy<planner.h):break
            if not planner.free[iy,ix]:
                obstacles.add(q)
                wx,wy=planner.world(ix,iy);r=planner.res/2
                corners=[(c*(wx+dx-pose[0])+s*(wy+dy-pose[1]),-s*(wx+dx-pose[0])+c*(wy+dy-pose[1])) for dx in [-r,r] for dy in [-r,r]]
                box=[min(v[0] for v in corners),min(v[1] for v in corners),max(v[0] for v in corners),max(v[1] for v in corners)]
                if q in bounds:box=[min(box[0],bounds[q][0]),min(box[1],bounds[q][1]),max(box[2],bounds[q][2]),max(box[3],bounds[q][3])]
                bounds[q]=box;break
            floor.add(q)
    measured=list(floor);floor-=obstacles;obstacles=sorted(obstacles)
    return dict(pose=list(pose),floor=list(floor),obstacles=obstacles,obstacle_bounds=[bounds[q] for q in obstacles],geometry_version=2,measured_floor=measured,stamp=stamp or time.monotonic(),demo=True,age=0)


class PointNavigator:
    def __init__(self,app):
        self.app=app;self.active=False;self.run_id=None;self.generation=0
        self.context=None;self.nav=None;self.lease=0.;self.deadline=0.;self.stop_at=None
        self.client_intent=None
        self.after_stop='cancelled';self.planner=None;self.follower=None
        self.goal_intent=None

    def current(self,token):
        return (self.active and self.run_id==token and self.app.nav is self.nav
                and self.app.hub.same_context(self.context) and self.nav['status']=='running'
                and self.app.navigation_intents.valid(self.client_intent,self.app.instance_id,self.app._plan_generation))

    def check(self,token):
        with self.app.lock:
            if not self.current(token):raise ValueError('导航已取消或连接已变化')
            if time.monotonic()>self.deadline:
                self.stop('paused','本轮时间已到，已请求停车');raise ValueError('导航任务已结束')

    def start(self,route,plan,intent,client_intent=None):
        app=self.app
        with app.lock:
            if intent!=app._nav_start_generation:raise ValueError('导航启动已取消')
            app.navigation_intents.check(client_intent,app.instance_id,app._plan_generation)
            app.live_ready();app.hub.normal_mode();app.hub.idle()
            if app.autonomy.active or app.mapping or app.hub.busy or app.nav['status'] in ['running','cancelling']:
                raise ValueError('请先结束当前运动、导航或建图任务')
            if app.reverse or app.estopped or app.sdk.get('armed') or app.basic.get('MotionState')!=17:
                raise ValueError('请头部朝前、站稳并退出关节控制')
            if app._reported_active_navigation():raise ValueError('原厂导航任务仍在运行，请先停止')
            map_=next(m.copy() for m in app.maps() if m['id']==route['map_id'])
            self.generation+=1;self.run_id=uuid.uuid4().hex;self.active=True;self.context=app.hub.context()
            self.client_intent=client_intent
            self.lease=time.monotonic();self.deadline=self.lease+600;self.stop_at=None
            self.goal_intent=copy.deepcopy({k:plan['result'].get(k) for k in ['goal','requested_goal','goal_adjustment']})
            self.nav=dict(status='running',engine='platform',phase='preparing',execution_state='preparing',
                          route=route,index=0,completed=0,progress=0,started=time.time(),run_id=self.run_id,
                          message='正在准备平台自主寻路',plan=plan['result'],replans=0)
            self.nav.update(app.navigation_intents.nav_fields(client_intent))
            app.nav=self.nav;app.cancel_request=None;app.velocity=[0.,0.,0.]
            if app.link:app.link.set_velocity(0,0,0)
            token=self.run_id
        try:threading.Thread(target=self._run,args=(token,map_,plan),daemon=True).start()
        except Exception:
            with app.lock:self.stop('failed','导航线程未启动')
            raise
        return dict(ok=True,engine='platform',execution='platform_trajectory',run_id=token,phase='preparing',
                    message='平台正在准备全局寻路与实时绕障；可随时停止',demo=app.demo)

    def stop(self,after='cancelled',message='已请求停止平台导航'):
        if self.nav is not self.app.nav or self.nav is None:return
        if not self.active and self.nav.get('status') not in ['running','cancelling']:return
        self.active=False;self.generation+=1;self.after_stop=after;self.stop_at=time.monotonic()
        self.app.velocity=[0.,0.,0.];self.app.velocity_deadline=None
        if self.app.link:self.app.link.set_velocity(0,0,0)
        self.nav.update(status='cancelling',phase='stopping',execution_state='stopping',message=message)
        self.tick()

    def tick(self):
        if self.nav is not self.app.nav:return
        if self.active and time.monotonic()>self.deadline:
            self.stop('paused','本轮时间已到，已请求停车');return
        if self.stop_at is None:return
        app=self.app
        if app.demo:still=True
        elif app.link:
            record=app.link.record(1002,4);v=record.get('items',{}).get('MotionStatus',{})
            still=age_of(record)<.5 and app._navigation_record_after(record,0,self.stop_at) and all(abs(v.get(k,999))<limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)])
        else:still=False
        if still:
            self.nav.update(status=self.after_stop,phase=self.after_stop,execution_state=self.after_stop,cancel_confirmed=True)
            if self.after_stop=='completed':
                self.nav.update(completed=1,index=1,progress=1.,message='已到达目标并观察到停车');app._record_route_completion()
            self.stop_at=None
        elif time.monotonic()-self.stop_at>2:
            self.nav.update(phase='cancel_unconfirmed',execution_state='stop_unconfirmed',warning='零速度已发送，等待新鲜的实际停稳反馈')

    def renew(self,token):
        with self.app.lock:
            if not self.current(token):return dict(ok=False,active=False)
            self.lease=time.monotonic();return dict(ok=True,active=True)

    def _update(self,token,**fields):
        with self.app.lock:
            if 'plan' in fields:fields['plan']=self._display_plan(fields['plan'])
            self.check(token);self.nav.update(fields)

    def _display_plan(self,plan):
        original=self.goal_intent or {};adjustment=original.get('goal_adjustment') or {}
        goal=original.get('goal') or {};current=plan.get('goal') or {}
        if (adjustment.get('applied') and all(k in goal and k in current for k in ['x','y'])
                and math.hypot(goal['x']-current['x'],goal['y']-current['y'])<1e-6):
            return dict(plan,requested_goal=copy.deepcopy(original['requested_goal']),goal_adjustment=copy.deepcopy(adjustment))
        return plan

    def _prepare(self,token,map_):
        app=self.app;self.check(token)
        if app.demo:
            with app.lock:app.basic.update(ControlUsageMode=0,Gait=0x3002,Sleep=0)
            return
        build=app.hub.remote_maps('/maps/build/status',context=self.context,timeout=5);self.check(token)
        if build.get('in_progress',True):raise ValueError('建图尚未结束，不能开始地图导航')
        status=app.hub.remote_maps('/status',context=self.context,timeout=3);self.check(token)
        if status.get('active')!=map_.get('robot_map_name'):raise ValueError('活动地图已变化，请重新同步规划')
        if not map_.get('verified_for_robot') or map_.get('robot_ip')!=app.link.ip:raise ValueError('请先同步当前活动地图')
        commands={};limit=time.monotonic()+9
        while time.monotonic()<limit:
            self.check(token)
            with app.lock:
                self.check(token);link=app.link;b=link.record(1002,6);basic=b.get('items',{}).get('BasicStatus',{})
                loc=link.record(1007,2);native=link.record(1007,1)
                if age_of(b)>2 or basic.get('HES')!=0 or basic.get('MotionState')!=17:raise ValueError('站立或控制状态未就绪')
                if age_of(loc)>1 or loc.get('items',{}).get('Location')!=0:raise ValueError('定位未就绪')
                if age_of(native)>2:
                    if 'query' not in commands:commands['query']=time.monotonic();link.command(1007,1,{},'平台导航核对任务')
                elif native.get('items',{}).get('Status') not in [0,4]:raise ValueError('原厂仍有活动导航任务')
                elif basic.get('Sleep'):
                    if 'wake' not in commands:commands['wake']=time.monotonic();link.command(1101,6,{'Sleep':False},'平台导航唤醒')
                elif basic.get('ControlUsageMode')!=0:
                    if 'mode' not in commands:commands['mode']=time.monotonic();link.set_mode(0)
                elif basic.get('Gait')!=0x3002:
                    motion=link.record(1002,4);v=motion.get('items',{}).get('MotionStatus',{})
                    if age_of(motion)<.5 and all(abs(v.get(k,999))<.08 for k in ['LinearX','LinearY','OmegaZ']) and 'gait' not in commands:
                        commands['gait']=time.monotonic();link.command(2,23,{'GaitParam':0x3002},'平台导航敏捷平地')
                elif all(app._navigation_record_after(b,0,at) for name,at in commands.items() if name!='query'):return
            time.sleep(.06)
        raise ValueError('平台导航模式准备未获确认')

    def _scan(self):
        app=self.app
        if app.demo:
            pose=[app.pose[k] for k in ['PosX','PosY','Yaw']]
            result=simulated_scan(self.planner,pose);result['_velocity_body']=app.velocity[:2];result['_speed_mps']=math.hypot(*app.velocity[:2])
            result.update(protocol=1,frame_id='base_link',pose_frame_id='map',pose_stamp=result['stamp'],resolution=.2,_valid_until=time.monotonic()+.65)
            return result,app.velocity[2]
        result=app.autonomy.scan_now(False)
        rec=app.link.record(1002,4);v=rec.get('items',{}).get('MotionStatus',{})
        loc=app.link.record(1007,2)
        if age_of(loc)>1 or loc.get('items',{}).get('Location')!=0:raise ValueError('定位丢失或过期')
        pp=loc['items'];sp=result.get('pose',[])
        if len(sp)!=3 or math.hypot(sp[0]-pp.get('PosX',1e9),sp[1]-pp.get('PosY',1e9))>.35 or abs(wrap(sp[2]-pp.get('Yaw',1e9)))>.25:
            raise ValueError('雷达与地图定位坐标不同步')
        if age_of(rec)>.25:raise ValueError('等待新鲜运动反馈')
        result['_velocity_body']=[v.get('LinearX'),v.get('LinearY')];result['_speed_mps']=math.hypot(*result['_velocity_body'])
        return result,v.get('OmegaZ',0)

    def _drive(self,token,axes,scan):
        with self.app.lock:
            self.check(token)
            if not self.app.demo:
                native=self.app.link.record(1007,1)
                if age_of(native)>2 or native.get('items',{}).get('Status') not in [0,4]:raise ValueError('原厂导航状态变化，平台已让出控制')
                if time.monotonic()>=scan.get('_valid_until',0):axes=[0.,0.,0.]
            self.app.set_velocity(dict(zip(['forward','side','yaw'],axes)),autonomous=True,valid_until=scan.get('_valid_until'))

    def _run(self,token,map_,plan):
        app=self.app
        try:
            self._prepare(token,map_);self.check(token)
            raw=Path(plan['source']).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=plan['fingerprint']:raise ValueError('地图已更新，请重新规划')
            planner=GridPlanner(raw,map_['resolution'],map_['origin'],map_.get('negate',0),map_.get('free_thresh',.196),map_.get('occupied_thresh',.65),enable_live_surface=not app.demo)
            preview_recovery=plan['result'].get('recovery',{})
            initial_path=preview_recovery.get('global_plan',plan['result'])
            follower=TrajectoryFollower();follower.set_path(initial_path.get('dense_path') or initial_path['path'])
            with app.lock:self.check(token);self.planner=planner;self.follower=follower
            goal=plan['result']['goal'];last_replan=0.;blocked_at=None;sensor_lost=None;initial=None
            recovery=None;start_checked=False;start_wait=time.monotonic()
            from autonomy_runtime import MotionProgress
            progress=MotionProgress();dynamic={};last_stamp=None
            while not app.done:
                self.check(token);now=time.monotonic()
                try:scan,yaw_rate=self._scan()
                except (ValueError,TypeError) as exc:
                    sensor_lost=sensor_lost or now;self._drive(token,[0,0,0],{})
                    self._update(token,message='等待感知恢复：'+str(exc),execution_state='waiting_progress')
                    if now-sensor_lost>5:raise
                    time.sleep(.08);continue
                self.check(token);sensor_lost=None
                if scan.get('stamp')==last_stamp:time.sleep(.025);continue
                last_stamp=scan.get('stamp');pose=scan['pose'];distance=math.hypot(goal['x']-pose[0],goal['y']-pose[1]);initial=initial or max(.01,distance)
                if app.demo:
                    projected=scan;localization=dict(normal=True,confidence=1.,pose=list(pose),age=0.,epoch='demo')
                    standing=app.basic.get('MotionState')==17 and app.basic.get('HES')==0
                    surface=dict(state='demo',temporary_free_cells=0)
                else:
                    projected,localization=app.navigation_surface(map_['id'],scan)
                    surface=planner.update_live_surface(projected,localization)
                    basic_record=app.link.record(1002,6);basic=basic_record.get('items',{}).get('BasicStatus',{})
                    standing=age_of(basic_record)<.5 and basic.get('MotionState')==17 and basic.get('HES')==0 and not basic.get('Sleep')
                    self._update(token,live_surface=surface)
                motion_supported=math.hypot(*scan.get('_velocity_body',[0.,0.]))>.08
                progress_detail=progress.update(pose,(goal['x'],goal['y']),0.,now,motion_supported)
                if progress.stalled(now):raise ValueError('持续没有实际进展，当前目标暂停')
                if not start_checked:
                    # A preview seed is never a motion plan. Recompute recovery
                    # from the NEW scan and current working map while stopped.
                    if preview_recovery.get('required') or not planner.pose_fits(*pose):
                        self._drive(token,[0,0,0],scan)
                        self._update(token,phase='preparing_recovery',execution_state='preparing',message='正在按当前扫描核对起点退让空间')
                        try:fresh=plan_with_recovery(planner,pose,[goal['x'],goal['y']],goal['yaw'],projected,localization,standing)
                        except ValueError as exc:
                            detail=getattr(exc,'recovery',{})
                            if (detail.get('classification')=='map_surface_disagreement' and now-start_wait<2.5
                                    and surface.get('state') in ['waiting_evidence','projection_unconfirmed']):
                                self._update(token,message='等待独立地面帧核对起点旧图冲突',execution_state='waiting_progress')
                                time.sleep(.04);continue
                            raise
                        self.check(token)
                        if fresh.get('recovery',{}).get('required'):
                            recovery=TranslationRecovery(fresh['recovery'],now=time.monotonic())
                            self._update(token,plan=fresh,recovery=dict(state='settling',preview_recomputed=True),phase='recovering_start',message='起点仅余量不足，准备连续平移退让')
                        else:
                            follower.set_path(fresh.get('dense_path') or fresh['path']);self._update(token,plan=fresh)
                    start_checked=True
                if recovery is not None:
                    result=recovery.command(planner,projected,localization,yaw_rate,standing=standing)
                    self.check(token)
                    limited=dict(scan,_valid_until=min(scan.get('_valid_until',time.monotonic()),result['valid_until']))
                    self._drive(token,result['axes'],limited)
                    self._update(token,phase='recovering_start',execution_state=('moving' if motion_supported else 'awaiting_progress') if any(result['axes']) else 'waiting_progress',
                                 message=result['reason'],recovery=result,controller=result,progress_detail=progress_detail)
                    if result['state']=='failed':raise ValueError(result['reason'])
                    if result['state']=='complete':
                        # Handoff is based on observed position and stillness;
                        # do not teleport to the preview's recovered endpoint.
                        self._drive(token,[0,0,0],scan)
                        new=planner.plan(pose,[goal['x'],goal['y']],goal['yaw'],budget=1.2)
                        self.check(token);follower.set_path(new.get('dense_path') or new['path']);recovery=None
                        self._update(token,plan=new,phase='tracking',message='起点恢复已停稳，已从实际位置接续原目标',recovery=dict(result,state='completed'))
                    time.sleep(.04);continue
                result=follower.command(scan,yaw_rate,goal['yaw'],allowed_pose=planner.pose_fits)
                self.check(token)
                if result['state']=='arrived':
                    with app.lock:self.stop('completed','到达目标，正在确认停车')
                    return
                blocked_at=(blocked_at or now) if result['state'] in ['blocked','waiting_feedback'] else None
                self._drive(token,result['axes'],scan)
                with app.lock:
                    if not self.current(token):return
                    self.nav.update(phase='tracking',execution_state=('moving' if motion_supported or abs(yaw_rate)>.12 else 'awaiting_progress') if any(result['axes']) else 'waiting_progress',
                                    message=result['reason'],controller=result,progress=max(0,min(.99,1-distance/initial)),distance_remaining=round(distance,2),progress_detail=progress_detail)
                if blocked_at and now-blocked_at>.6 and now-last_replan>1.5:
                    self._drive(token,[0,0,0],scan);last_replan=now
                    c,s=math.cos(pose[2]),math.sin(pose[2])
                    def world(p):return (round((pose[0]+c*p[0]*.2-s*p[1]*.2)/.2),round((pose[1]+s*p[0]*.2+c*p[1]*.2)/.2))
                    for p in scan['floor']:dynamic.pop(world(p),None)
                    for p in scan['obstacles']:dynamic[world(p)]=now
                    dynamic={p:t for p,t in dynamic.items() if now-t<15}
                    planner.update_obstacles([(p[0]*.2,p[1]*.2) for p in dynamic],robot_pose=pose)
                    try:new=plan_with_recovery(planner,pose,[goal['x'],goal['y']],goal['yaw'],projected,localization,standing,budget=1.2)
                    except ValueError as exc:
                        self._update(token,message='通路暂时被占用，等待或绕行：'+str(exc));continue
                    self.check(token)
                    if new.get('recovery',{}).get('required'):
                        recovery=TranslationRecovery(new['recovery'],now=time.monotonic())
                        self._update(token,phase='recovering_start',recovery=dict(state='settling',preview_recomputed=True))
                    else:follower.set_path(new.get('dense_path') or new['path'])
                    with app.lock:
                        if self.current(token):self.nav.update(plan=self._display_plan(new),replans=self.nav['replans']+1,message='已根据当前障碍重规划')
                time.sleep(.04)
        except Exception as exc:
            with app.lock:
                if self.current(token):
                    self.nav['error']=str(exc);self.stop('paused',str(exc))

    def _map_pose_valid(self,x,y,yaw):
        # Independent static footprint test; a clear live scan does not erase
        # a map wall. The global planner owns cached orientation masks.
        return self.planner.pose_fits(x,y,yaw)

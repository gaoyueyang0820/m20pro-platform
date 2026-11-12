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

from m20.local_trajectory import TrajectoryFollower,LocalSurface,wrap
from m20.latest_observation import LatestObservationPump, ObservationContextChanged
from m20.moving_obstacles import MovingObstacles
from m20.navigation_detour import plan_local_detour
from m20.navigation_validation import poses_consistent
from m20.observation_errors import GroundObservationPending, ObservationSyncPending
from m20.navigation_planner import GridPlanner
from m20.platform_core import age_of
from m20.start_recovery_runtime import plan_with_recovery, TranslationRecovery


class NavigationObservationPending(ValueError):
    """Keep the goal while stopped and waiting for a fresh paired observation."""


def stationary_feedback(link, after=0.):
    record = link.record(1002, 4)
    motion = record.get('items', {}).get('MotionStatus', {})
    return (age_of(record) < .5 and record.get('monotonic_at', 0.) > after
            and all(isinstance(value, (int, float)) and abs(value) < limit for value, limit in
                    ((motion.get(key), limit) for key, limit in
                     [('LinearX', .08), ('LinearY', .08), ('OmegaZ', .12)])))


def navigation_observation(link, scan, epoch):
    scan = copy.deepcopy(scan)
    record = link.record(1007, 2)
    pose = record.get('items', {})
    if '_valid_until' in scan:
        scan['age'] = max(0., .65 + time.monotonic() - scan['_valid_until'])
    motion = link.record(1002, 4)
    values = motion.get('items', {}).get('MotionStatus', {})
    return scan, dict(normal=pose.get('Location') == 0, confidence=pose.get('Confidence', 0),
                      age=age_of(record), pose=[pose.get(k) for k in ('PosX', 'PosY', 'Yaw')],
                      epoch=epoch, velocity_body=[values.get('LinearX'), values.get('LinearY')],
                      yaw_rate=values.get('OmegaZ', 0), motion_age=age_of(motion))


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
    async_observations = False

    def __init__(self,app):
        self.app=app;self.active=False;self.run_id=None;self.generation=0
        self.context=None;self.nav=None;self.lease=0.;self.deadline=0.;self.stop_at=None
        self.client_intent=None
        self.after_stop='cancelled';self.planner=None;self.follower=None
        self.goal_intent=None
        self.last_sensor_stamp=None
        self.motion_mode=None
        self.observations=None
        self.last_observation_sequence=None
        self.worker=None

    def current(self,token):
        return (self.active and self.run_id==token and self.app.nav is self.nav
                and self.app.hub.same_context(self.context) and self.nav['status']=='running'
                and self.app.navigation_intents.valid(self.client_intent,self.app.instance_id,self.app._plan_generation))

    def check(self,token):
        with self.app.lock:
            if not self.current(token):raise ValueError('导航已取消或连接已变化')
            if time.monotonic()>self.deadline:
                self.stop('paused','本轮时间已到，已请求停车');raise ValueError('导航任务已结束')

    def start(self,route,plan,intent,client_intent=None,start_index=0):
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
            token=self._new_run(route,plan,start_index,client_intent,app.hub.context())
            self.nav.update(app.navigation_intents.nav_fields(client_intent))
            app.cancel_request=None;app.velocity=[0.,0.,0.]
            if app.link:app.link.set_velocity(0,0,0)
        self._launch(token,map_,plan)
        return dict(ok=True,engine='platform',execution='platform_trajectory',run_id=token,phase='preparing',
                    message='平台正在准备全局寻路与实时绕障；可随时停止',demo=app.demo)

    def _new_run(self, route, plan, start_index, client_intent=None, context=None):
        self.generation += 1
        self.run_id = uuid.uuid4().hex
        self.active = True
        self.context = context
        self.last_sensor_stamp = None
        self.last_observation_sequence = None
        self.motion_mode = None
        self.client_intent = client_intent
        self.lease = time.monotonic()
        self.deadline = self.lease + 600
        self.stop_at = None
        result = plan.get('result') or {}
        self.goal_intent = {key: copy.deepcopy(result.get(key)) for key in
                            ('goal', 'requested_goal', 'goal_adjustment')}
        self.nav = dict(status='running', engine='platform', phase='preparing', execution_state='preparing',
                        route=route, index=start_index, completed=start_index,
                        progress=start_index / len(route['points']), started=time.time(), run_id=self.run_id,
                        message='正在准备自主寻路', plan=plan.get('result'), replans=0)
        self.app.nav = self.nav
        return self.run_id

    def _launch(self, token, map_, plan):
        self.worker=threading.Thread(target=self._run,args=(token,map_,plan),daemon=True)
        try:self.worker.start()
        except Exception:
            with self.app.lock:self.stop('failed','导航线程未启动')
            raise

    def stop(self,after='cancelled',message='已请求停止平台导航'):
        if self.nav is not self.app.nav or self.nav is None:return
        if not self.active and self.nav.get('status') not in ['running','cancelling']:return
        self.active=False;self.generation+=1;self.after_stop=after;self.stop_at=time.monotonic()
        if self.observations is not None:self.observations.cancel()
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
        elif app.link:still=stationary_feedback(app.link,self.stop_at)
        else:still=False
        if still:
            self.nav.update(status=self.after_stop,phase=self.after_stop,execution_state=self.after_stop,cancel_confirmed=True)
            if self.after_stop=='completed':
                count=len(self.nav['route']['points'])
                self.nav.update(completed=count,index=count,progress=1.,message='已到达目标并观察到停车');app._record_route_completion()
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

    def _join_waypoints(self,planner,plan,index):
        points=self.nav['route']['points'];joined=copy.deepcopy(plan)
        dense=list(plan.get('dense_path') or plan['path']);goals={index:dict(plan['goal'])};covered=index
        # Bound speculative look-ahead as a whole. A long patrol starts on
        # its ready prefix and plans later legs when approaching them.
        deadline=time.monotonic()+.15
        for following in range(index+1,len(points)):
            budget=deadline-time.monotonic()
            if budget<=0 or self._stops_at(covered):break
            previous=goals[covered];point=points[following]
            try:segment=planner.plan([previous['x'],previous['y'],previous.get('yaw') or 0.],
                                     [point['x'],point['y']],point.get('yaw'),budget=budget,goal_tolerance=.65)
            except ValueError:break
            dense.extend((segment.get('dense_path') or segment['path'])[1:]);goals[following]=dict(segment['goal']);covered=following
        joined.update(dense_path=dense,path=[dict(x=p['x'],y=p['y']) for p in dense],route_through=covered)
        return joined,goals,covered

    def _stops_at(self, index):
        route = self.nav['route']
        return bool(route['points'][index].get('station_action') or
                    getattr(self.app, 'station_action', None) and route.get('options', {}).get('pause_at_point'))

    def _station(self, token, index, pose):
        """Hold zero and observe stopped motion before calling a station action."""
        point = self.nav['route']['points'][index]
        self._drive(token, [0., 0., 0.], {})
        began = time.monotonic()
        self._update(token, phase='station_parking', execution_state='stopping', message='正在确认航点停稳')
        while not self.app.demo and not stationary_feedback(self.app.link, began):
            self.check(token)
            if time.monotonic() - began >= 3:
                raise ValueError('未确认航点停稳')
            time.sleep(.04)
        self.check(token)
        if point.get('station_action'):
            def guard():
                self.check(token)
                if not self.app.demo and not stationary_feedback(self.app.link, began):
                    raise ValueError('航点作业时机身停稳状态已变化')
            self._update(token, phase='station_action', execution_state='inspecting', message='正在执行到点作业')
            self.app.station_action(index, point['station_action'], pose, guard)

    def _rejoin_live_route(self,planner,follower,scan,goal):
        pose=scan['pose']
        if not len(follower.path) or not planner.pose_fits(*pose):return False
        end=min(range(follower.index,len(follower.path)),key=lambda i:math.dist(follower.path[i],goal))
        candidates=[i for i in range(follower.index+1,end+1) if .25<math.dist(follower.path[i],pose[:2])<1.4]
        geometry=LocalSurface.cached(scan);c,s=math.cos(pose[2]),math.sin(pose[2])
        def connects(a,b):
            ax,ay=a[0]-pose[0],a[1]-pose[1]
            bx,by=b[0]-pose[0],b[1]-pose[1]
            x1,y1=c*ax+s*ay,-s*ax+c*ay
            x2,y2=c*bx+s*by,-s*bx+c*by
            return (planner.segment_fits(a,b,pose[2])
                    and geometry.valid((x1+x2)/2,(y1+y2)/2,0.,padding=(abs(x2-x1)/2,abs(y2-y1)/2)))
        for i in reversed(candidates):
            point=follower.path[i]
            ahead=follower.path[min(end,i+3)]
            if connects(pose[:2],point) and connects(point,ahead):
                follower.set_path([pose[:2],*follower.path[i:]],terminal=follower.terminal);return True
        return False

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
        self._prepare_motion(token)

    def _prepare_motion(self, token, *, require_location=True):
        app=self.app
        commands={};limit=time.monotonic()+9
        reason='平台导航模式准备未获确认'
        while time.monotonic()<limit:
            self.check(token)
            with app.lock:
                self.check(token);link=app.link;b=link.record(1002,6);basic=b.get('items',{}).get('BasicStatus',{})
                loc=link.record(1007,2) if require_location else {}
                native=link.record(1007,1)
                reason='平台导航模式准备未获确认'
                if age_of(b)>2:
                    reason='等待机内运控状态回报超时'
                elif basic.get('HES')!=0:
                    raise ValueError('机器人急停尚未解除')
                elif basic.get('MotionState')!=17:
                    raise ValueError('机器人尚未确认站立，请站稳后重试')
                elif require_location and age_of(loc)>1:
                    reason='等待机内地图定位回报超时'
                elif require_location and loc.get('items',{}).get('Location')!=0:
                    reason='等待机内地图定位恢复'
                elif age_of(native)>2:
                    if 'query' not in commands:commands['query']=time.monotonic();link.command(1007,1,{},'平台导航核对任务')
                elif native.get('items',{}).get('Status') not in [0,4]:raise ValueError('原厂仍有活动导航任务')
                elif basic.get('Sleep'):
                    if 'wake' not in commands:commands['wake']=time.monotonic();link.command(1101,6,{'Sleep':False},'平台导航唤醒')
                elif basic.get('ControlUsageMode') not in (0,2):
                    if 'mode' not in commands:commands['mode']=time.monotonic();link.set_mode(0)
                elif basic.get('Gait')!=0x3002:
                    motion=link.record(1002,4);v=motion.get('items',{}).get('MotionStatus',{})
                    if age_of(motion)<.5 and all(abs(v.get(k,999))<.08 for k in ['LinearX','LinearY','OmegaZ']) and 'gait' not in commands:
                        commands['gait']=time.monotonic();link.command(2,23,{'GaitParam':0x3002},'平台导航敏捷平地')
                elif all(b.get('monotonic_at',-math.inf)>at for name,at in commands.items() if name!='query'):
                    self.motion_mode=basic['ControlUsageMode']
                    return
            time.sleep(.06)
        raise ValueError(reason)

    def _fetch_scan(self, after_stamp):
        return self.app.autonomy.scan_now(False,after_stamp=after_stamp)

    def _start_observations(self, token):
        last_stamp = None
        def fetch(context, stop):
            nonlocal last_stamp
            if stop.is_set() or not self.current(token):
                raise ObservationContextChanged('导航任务已变更')
            scan = self._fetch_scan(last_stamp)
            if stop.is_set() or not self.current(token):
                raise ObservationContextChanged('导航任务已变更')
            stamp = scan.get('stamp')
            if isinstance(stamp, (int, float)) and (last_stamp is None or stamp > last_stamp):
                last_stamp = stamp
            return scan
        pump = LatestObservationPump(fetch, clock=time.monotonic)
        self.observations = pump
        pump.start()
        return pump

    def _scan(self):
        app=self.app
        if app.demo:
            pose=[app.pose[k] for k in ['PosX','PosY','Yaw']]
            result=simulated_scan(self.planner,pose);result['_velocity_body']=app.velocity[:2];result['_speed_mps']=math.hypot(*app.velocity[:2])
            result.update(protocol=1,frame_id='base_link',pose_frame_id='map',pose_stamp=result['stamp'],resolution=.2,_valid_until=time.monotonic()+.65)
            return result,app.velocity[2]
        if self.observations is None:
            result=self._fetch_scan(self.last_sensor_stamp)
        else:
            current = self.observations.read(after_sequence=self.last_observation_sequence,fresh_only=True)
            if current['state']=='cached':
                # Transport retains the last command only until the unchanged
                # observation deadline. Do not rescore or validate this scan.
                return None,None
            result = current['observation']
            if result is None:
                reason = current.get('error') or '等待新鲜雷达观测'
                if current.get('error_kind') == 'ground_pending':
                    raise GroundObservationPending(reason, current['reason_code'])
                if current.get('state') in ('waiting','expired') or current.get('error_kind') in ('sync_pending','transport'):
                    raise NavigationObservationPending(reason)
                raise ValueError(reason)
        rec=app.link.record(1002,4);v=rec.get('items',{}).get('MotionStatus',{})
        loc=app.link.record(1007,2)
        if age_of(loc)>1 or loc.get('items',{}).get('Location')!=0:raise NavigationObservationPending('等待地图定位恢复')
        if age_of(rec)>.25:raise NavigationObservationPending('等待新鲜运动反馈')
        vx,vy,omega=[v.get(k) for k in ['LinearX','LinearY','OmegaZ']]
        if not all(isinstance(value,(int,float)) and math.isfinite(value) for value in [vx,vy,omega]):raise ValueError('等待有效运动反馈')
        pp=loc['items'];sp=result.get('pose',[])
        age=max(0.,.65+time.monotonic()-result['_valid_until']) if '_valid_until' in result else float(result.get('age',0.))
        # Compare at a common time using measured motion. This is only a
        # residual check: never move the paired scan pose/cloud or its expiry.
        if len(sp)!=3:raise NavigationObservationPending(result.get('pose_error') or '等待与雷达匹配的地图位姿')
        evidence=dict(age=age_of(loc),pose=[pp.get('PosX'),pp.get('PosY'),pp.get('Yaw')],velocity_body=[vx,vy],yaw_rate=omega,motion_age=age_of(rec))
        if not poses_consistent(sp,age,evidence):
            raise NavigationObservationPending('雷达与地图定位不同步，正在等待新观测')
        result['_velocity_body']=[vx,vy];result['_speed_mps']=math.hypot(vx,vy)
        result['_motion_feedback_at']=rec.get('monotonic_at')
        avoidance=app.link.record(2002,1)
        result['_vendor_avoidance']=(age_of(avoidance)<1.35 and avoidance.get('items',{}).get('ObsState')==1)
        self.last_sensor_stamp=result.get('stamp')
        self.last_observation_sequence=result.get('_pump_sequence')
        return result,omega

    def _drive(self,token,axes,scan):
        with self.app.lock:
            self.check(token)
            if not self.app.demo:
                native=self.app.link.record(1007,1)
                if age_of(native)>2 or native.get('items',{}).get('Status') not in [0,4]:raise ValueError('原厂导航状态变化，平台已让出控制')
                if any(axes) and self.motion_mode is not None:
                    basic=self.app.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                    if basic.get('ControlUsageMode')!=self.motion_mode:raise ValueError('运控模式已切换')
                if time.monotonic()>=scan.get('_valid_until',0):axes=[0.,0.,0.]
            self.app.set_velocity(dict(zip(['forward','side','yaw'],axes)),autonomous=True,valid_until=scan.get('_valid_until'))
            return list(axes)

    def _trajectory_follower(self):
        return TrajectoryFollower()

    def _run(self,token,map_,plan):
        app=self.app
        pump=None
        try:
            self._prepare(token,map_);self.check(token)
            route=self.nav['route']
            raw=plan.get('raw')
            if raw is None:raw=Path(plan['source']).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=plan['fingerprint']:raise ValueError('地图已更新，请重新规划')
            planner=GridPlanner(raw,map_['resolution'],map_['origin'],map_.get('negate',0),map_.get('free_thresh',.196),map_.get('occupied_thresh',.65),enable_live_surface=not app.demo,soft_static_speckles=True)
            if self.async_observations:
                with app.lock:
                    self.check(token)
                    pump=self._start_observations(token)
            initial_result=plan.get('result')
            if initial_result is None:
                initial_result=self._plan_from_local_observation(token,map_,planner,route['points'][self.nav['index']])
            preview_recovery=initial_result.get('recovery',{})
            initial_path=preview_recovery.get('global_plan',initial_result)
            route_index=self.nav.get('index',0);route_goals={};covered=route_index;final_yaw=None;follower=self._trajectory_follower()
            follower.straight_clearance=planner.straight_clearance
            def install_path(new):
                nonlocal covered,final_yaw,route_goals
                joined,route_goals,covered=self._join_waypoints(planner,new,route_index)
                if covered==len(route['points'])-1 or self._stops_at(covered):follower.set_path(joined.get('dense_path') or joined['path'])
                else:follower.set_path(joined.get('dense_path') or joined['path'],terminal=False)
                final_yaw=route_goals[covered].get('yaw')
                return joined
            initial_path=install_path(initial_path)
            with app.lock:
                self.check(token);self.planner=planner;self.follower=follower;self.nav['plan']=self._display_plan(initial_path)
            goal=initial_result['goal'];last_replan=0.;blocked_at=None;sensor_lost=None;initial=None
            recovery=None;start_checked=False;start_wait=time.monotonic()
            from m20.autonomy_runtime import MotionProgress
            progress=MotionProgress();last_stamp=None
            obstacle_tracks=MovingObstacles();obstacle_memory={};applied_obstacles=set()
            local_follower=None;local_plan=None;local_pending=False;local_tried=[];avoid_axes=None;route_first=False
            vendor_rechecked=False;last_lookahead=-math.inf
            def apply_obstacles(cells,pose):
                nonlocal applied_obstacles
                if cells!=applied_obstacles:
                    resolution=obstacle_tracks.resolution
                    planner.update_obstacles([(x*resolution,y*resolution) for x,y in cells],robot_pose=pose)
                    applied_obstacles=cells
            def request_local(reason,failed=None,prefer_route=False):
                nonlocal local_follower,local_plan,local_pending,avoid_axes,blocked_at,route_first
                if local_plan:
                    local_tried.append((local_plan['goal']['x'],local_plan['goal']['y']))
                    del local_tried[:-16]
                local_follower=None;local_plan=None;local_pending=True;avoid_axes=failed;blocked_at=None
                route_first=prefer_route
                progress.recovery_started();self._drive(token,[0,0,0],scan)
                self._update(token,phase='finding_bypass',execution_state='waiting_progress',message=reason,
                             recovery=dict(state='choosing_local_path',original_goal=dict(goal)))
            while not app.done:
                self.check(token);now=time.monotonic()
                try:scan,yaw_rate=self._scan()
                except (ValueError,TypeError) as exc:
                    if sensor_lost is None:
                        sensor_lost=now;self._drive(token,[0,0,0],{})
                    progress.observe_command([0.,0.,0.],now)
                    self._update(token,message=str(exc)+'，恢复后继续原任务',execution_state='waiting_progress')
                    if now-sensor_lost>5 and not isinstance(exc,(GroundObservationPending,ObservationSyncPending,NavigationObservationPending)):raise
                    time.sleep(.08);continue
                if scan is None:time.sleep(.025);continue
                self.check(token)
                if sensor_lost is not None:
                    progress=MotionProgress();sensor_lost=None
                if scan.get('stamp')==last_stamp:time.sleep(.025);continue
                last_stamp=scan.get('stamp');pose=scan['pose']
                while route_index<len(route['points'])-1 and not self._stops_at(route_index) and math.dist(pose[:2],[goal['x'],goal['y']])<.25:
                    route_index+=1;goal=dict(route_goals.get(route_index,route['points'][route_index]));goal.setdefault('yaw',None)
                    self.goal_intent=dict(goal=dict(goal));initial=None
                    self._update(token,index=route_index,completed=route_index,message='已通过中间点，继续下一点',progress=route_index/len(route['points']))
                    if route_index>covered:local_pending=True;route_first=True
                distance=math.hypot(goal['x']-pose[0],goal['y']-pose[1]);initial=initial or max(.01,distance)
                vendor_avoidance=bool(scan.get('_vendor_avoidance'))
                if app.demo:
                    projected=scan;localization=dict(normal=True,confidence=1.,pose=list(pose),age=0.,epoch='demo')
                    standing=app.basic.get('MotionState')==17 and app.basic.get('HES')==0
                    surface=dict(state='demo',temporary_free_cells=0)
                else:
                    projected,localization=app.navigation_surface(map_['id'],scan)
                    surface=planner.update_live_surface(projected,localization)
                    basic_record=app.link.record(1002,6);basic=basic_record.get('items',{}).get('BasicStatus',{})
                    standing=age_of(basic_record)<.5 and basic.get('MotionState')==17 and basic.get('HES')==0 and not basic.get('Sleep')
                    vendor_avoidance |= age_of(basic_record)<.65 and basic.get('ControlUsageMode')==2 and basic.get('OOA')==3
                if app.demo or surface.get('accepted'):
                    observed=obstacle_tracks.observe(projected,epoch=localization.get('epoch'))
                    for p in observed.cleared:obstacle_memory.pop(p,None)
                    obstacle_memory.update((p,now) for p in observed.confirmed)
                    obstacle_memory={p:at for p,at in obstacle_memory.items() if now-at<15}
                    planning_obstacles=observed.current | observed.temporary | obstacle_memory.keys()
                    surface['moving_obstacle_tracks']=observed.moving_tracks
                else:
                    obstacle_tracks.reset()
                    obstacle_memory.clear()
                    planning_obstacles=set()
                # Defer adding a planning overlay to replans. Retire obsolete
                # cells immediately so a mover does not leave a 15 s trail.
                if applied_obstacles-planning_obstacles:
                    apply_obstacles(applied_obstacles & planning_obstacles,pose)
                if not app.demo:
                    self._update(token,live_surface=surface,vendor_avoidance=vendor_avoidance)
                if not vendor_avoidance:vendor_rechecked=False
                motion_supported=math.hypot(*scan.get('_velocity_body',[0.,0.]))>.08
                progress_detail=progress.update(pose,(goal['x'],goal['y']),0.,now,motion_supported)
                stuck=progress.stuck_reason(now)
                if progress.stalled(now) and not local_pending and any(any(v) for _,v in progress.command_window):
                    raise ValueError('多次换向仍未产生实际进展，已暂停导航')
                if stuck and recovery is None:
                    request_local('正在更换行进方向，继续前往原目标',progress.failed_axes());continue
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
                            request_local('正在从实时空地寻找可通行方向');start_checked=True;continue
                        self.check(token)
                        if fresh.get('recovery',{}).get('required'):
                            recovery=TranslationRecovery(fresh['recovery'],now=time.monotonic())
                            self._update(token,plan=fresh,recovery=dict(state='settling',preview_recomputed=True),phase='recovering_start',message='起点仅余量不足，准备连续平移退让')
                        else:
                            fresh=install_path(fresh);self._update(token,plan=fresh)
                    start_checked=True
                if local_pending and recovery is None:
                    candidate=None
                    # Rejoin a proven route if it opened up, then look for a
                    # short local bypass before repeating the full map search.
                    if route_first and route_index<=covered and self._rejoin_live_route(planner,follower,scan,[goal['x'],goal['y']]):
                        local_pending=False;route_first=False
                        self._update(token,phase='tracking',message='已找到通往原路线的安全接续段')
                        continue
                    try_local_first=standing and route_index<=covered and len(local_tried)<2
                    if try_local_first:
                        candidate=plan_local_detour(scan,[goal['x'],goal['y']],tried=local_tried,avoid_axes=avoid_axes)
                    if route_first:
                        route_first=False;last_replan=now
                        if candidate is None:
                            apply_obstacles(planning_obstacles,pose)
                            try:
                                new=plan_with_recovery(planner,pose,[goal['x'],goal['y']],goal['yaw'],
                                                       projected,localization,standing,budget=.55)
                            except ValueError:
                                new=None
                            self.check(token)
                            if new:
                                goal=dict(new['goal'])
                                if new.get('recovery',{}).get('required'):
                                    recovery=TranslationRecovery(new['recovery'],now=time.monotonic())
                                else:
                                    new=install_path(new)
                                local_pending=False
                                self._update(token,plan=new,phase='tracking',message='已按当前障碍调整通往目标的路线',
                                             replans=self.nav.get('replans',0)+1)
                                continue
                    if standing and not try_local_first:
                        candidate=plan_local_detour(scan,[goal['x'],goal['y']],tried=local_tried,avoid_axes=avoid_axes)
                    self.check(token)
                    if candidate:
                        local_plan=candidate;local_follower=self._trajectory_follower()
                        local_follower.set_path(candidate['path']);local_pending=False
                        self._update(token,phase='local_bypass',message='正在绕开当前障碍，目的地保持不变',
                                     recovery=dict(state='following_local_path',**candidate))
                    else:
                        self._drive(token,[0,0,0],scan);progress.observe_command([0.,0.,0.],now)
                        self._update(token,phase='waiting_space',execution_state='waiting_progress',message='正在等待可通行空间，通路恢复后继续',
                                     recovery=dict(state='waiting_for_space',original_goal=dict(goal)))
                        # Recheck the original route as well: a person may have
                        # moved away while the short local maneuver was pending.
                        if now-last_replan>1.5:
                            route_first=True
                        time.sleep(.04);continue
                if recovery is not None:
                    result=recovery.command(planner,projected,localization,yaw_rate,standing=standing)
                    self.check(token)
                    limited=dict(scan,_valid_until=min(scan.get('_valid_until',time.monotonic()),result['valid_until']))
                    self._drive(token,result['axes'],limited)
                    self._update(token,phase='recovering_start',execution_state=('moving' if motion_supported else 'awaiting_progress') if any(result['axes']) else 'waiting_progress',
                                 message=result['reason'],recovery=result,controller=result,progress_detail=progress_detail)
                    if result['state']=='failed':
                        recovery=None;request_local('正在选择其他绕行方向');continue
                    if result['state']=='complete':
                        # Handoff is based on observed position and stillness;
                        # do not teleport to the preview's recovered endpoint.
                        self._drive(token,[0,0,0],scan)
                        try:new=planner.plan(pose,[goal['x'],goal['y']],goal['yaw'],budget=.3)
                        except ValueError:
                            recovery=None;request_local('正在连接下一段通路，目的地保持不变');continue
                        self.check(token);new=install_path(new);recovery=None
                        self._update(token,plan=new,phase='tracking',message='起点恢复已停稳，已从实际位置接续原目标',recovery=dict(result,state='completed'))
                    time.sleep(.04);continue
                if local_follower is not None:
                    # The bypass edges were certified with the body held at its heading.
                    near_final=distance<.5 and math.dist([local_plan['goal']['x'],local_plan['goal']['y']],[goal['x'],goal['y']])<.25
                    result=local_follower.command(scan,yaw_rate,goal['yaw'] if near_final else None,fixed_heading=True)
                    if result.get('remaining_path_m',math.inf)<.65 and self._rejoin_live_route(planner,follower,scan,[goal['x'],goal['y']]):
                        local_follower=None;local_plan=None
                        result=follower.command(scan,yaw_rate,final_yaw,allowed_pose=planner.pose_fits)
                        self._update(token,message='已接回原路线，继续前往目标',recovery=dict(state='rejoined',original_goal=dict(goal)))
                else:result=follower.command(scan,yaw_rate,final_yaw,allowed_pose=planner.pose_fits)
                self.check(token)
                if result['state']=='arrived':
                    if local_follower is None or distance<.18 and (goal['yaw'] is None or abs(wrap(goal['yaw']-pose[2]))<.14):
                        if self._stops_at(route_index):
                            self._station(token,route_index,pose)
                            if route_index<len(route['points'])-1:
                                route_index+=1
                                goal=dict(route['points'][route_index])
                                self._update(token,index=route_index,completed=route_index,progress=route_index/len(route['points']))
                                if route.get('options',{}).get('pause_at_point'):
                                    with app.lock:self.stop('paused','航点任务完成，等待继续')
                                    return
                                self._update(token,phase='preparing',message='航点任务完成，正在规划下一段')
                                following=self._plan_from_local_observation(token,map_,planner,goal,after_stamp=last_stamp)
                                goal=following['goal']
                                self.goal_intent=dict(goal=dict(goal))
                                joined=install_path(following.get('recovery',{}).get('global_plan',following))
                                self._update(token,plan=joined,phase='tracking',message='正在前往下一航点')
                                recovery=(TranslationRecovery(following['recovery'],now=time.monotonic())
                                          if following.get('recovery',{}).get('required') else None)
                                local_follower=None;local_plan=None;local_pending=False
                                local_tried.clear();initial=None;blocked_at=None;progress=MotionProgress()
                                continue
                        if route_index<len(route['points'])-1:
                            request_local('正在连接下一段巡检路线',prefer_route=True);continue
                        with app.lock:self.stop('completed','到达目标，正在确认停车')
                        return
                    local_tried.append((local_plan['goal']['x'],local_plan['goal']['y']))
                    local_follower=None;local_plan=None;last_replan=now-2;blocked_at=now-.7
                    result=dict(state='blocked',axes=[0.,0.,0.],reason='已绕过局部障碍，正在接续原目标')
                blocked_at=(blocked_at or now) if result['state'] in ['blocked','waiting_feedback'] else None
                issued=self._drive(token,result['axes'],scan);progress.observe_command(issued,now)
                if (local_follower is None and not local_pending and standing and issued[0]>0
                        and result.get('ahead_obstacle_m') is not None and now-last_lookahead>=1.5):
                    last_lookahead=now
                    bypass=plan_local_detour(scan,[goal['x'],goal['y']],tried=local_tried,avoid_axes=issued)
                    self.check(token)
                    if bypass:
                        local_plan=bypass;local_follower=self._trajectory_follower()
                        local_follower.set_path(bypass['path'])
                        self._update(token,phase='local_bypass',message='正在提前绕开前方障碍',
                                     recovery=dict(state='following_local_path',**bypass))
                        continue
                with app.lock:
                    if not self.current(token):return
                    self.nav.update(phase='local_bypass' if local_follower is not None else 'tracking',execution_state=('moving' if motion_supported or abs(yaw_rate)>.12 else 'awaiting_progress') if any(result['axes']) else 'waiting_progress',
                                    message=result['reason'],controller=result,progress=max(0,min(.99,(route_index+1-distance/initial)/len(route['points']))),distance_remaining=round(distance,2),progress_detail=progress_detail)
                held_by_vendor=(vendor_avoidance and not vendor_rechecked and any(issued) and not motion_supported
                                and abs(yaw_rate)<.12 and progress_detail['seconds_without_progress']>.65)
                if (held_by_vendor or blocked_at and now-blocked_at>(.18 if vendor_avoidance else .6)) and now-last_replan>1.5:
                    vendor_rechecked |= held_by_vendor
                    reason='原厂避障已介入，正在复核并调整路线' if vendor_avoidance else '正在核对通路并调整路线'
                    request_local(reason,progress.failed_axes(),prefer_route=True);continue
                time.sleep(.04)
        except Exception as exc:
            with app.lock:
                if self.current(token):
                    self.nav['error']=str(exc)
                    after='failed' if self.nav.get('phase')=='station_action' else 'paused'
                    self.stop(after,str(exc))
        finally:
            if pump is not None:
                pump.cancel()
                with app.lock:
                    if self.observations is pump:self.observations=None

    def _plan_from_local_observation(self, token, map_, planner, goal, after_stamp=None):
        deadline = time.monotonic() + 5
        seen = set()
        while True:
            self.check(token)
            try:
                scan, _ = self._scan()
                if scan is None:
                    raise NavigationObservationPending('等待新的雷达观测')
                stamp=scan['stamp']
                if stamp in seen or after_stamp is not None and stamp <= after_stamp:
                    raise ValueError('等待航点作业后的新雷达观测')
                projected, localization = self.app.navigation_surface(map_['id'], scan)
                surface=planner.update_live_surface(projected, localization)
                seen.add(stamp)
            except (NavigationObservationPending,GroundObservationPending,ObservationSyncPending) as exc:
                if time.monotonic()>=deadline:
                    self._update(token,phase='waiting_observation',execution_state='waiting_progress',message=str(exc))
                time.sleep(.08)
                continue
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.04)
                continue
            with planner.planning_snapshot():
                try:
                    return planner.plan(scan['pose'], [goal['x'], goal['y']], goal['yaw'], goal_tolerance=.65)
                except ValueError as error:
                    if (surface.get('state') in ('waiting_evidence','projection_unconfirmed')
                            and len(seen)<3 and time.monotonic()<deadline):
                        time.sleep(.04)
                        continue
                    return plan_with_recovery(planner, scan['pose'], [goal['x'], goal['y']], goal['yaw'],
                                              projected, localization, True, standard_error=error)

    def _map_pose_valid(self,x,y,yaw):
        # Independent static footprint test; a clear live scan does not erase
        # a map wall. The global planner owns cached orientation masks.
        return self.planner.pose_fits(x,y,yaw)

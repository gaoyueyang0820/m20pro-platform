"""Opt-in NOS-resident missions. Import/status never creates a robot link.

Deployment must explicitly enable this adapter and configure the onboard AOS IP.
The default remains unavailable until the shared control integration is loaded.
"""
import copy
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from contextlib import contextmanager
from m20.map_catalog import read_json
from m20.navigation_disconnect import ResidentMission,identity,mission_map,specification,TERMINAL


def atomic(path,value):
    path=Path(path)
    if path.is_symlink():raise ValueError('任务记录路径异常')
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.'+secrets.token_hex(8)+'.tmp')
    try:
        with temp.open('x',encoding='utf-8') as handle:
            if os.name=='nt':
                from m20.platform_host import private_file
                private_file(temp)
            else:os.chmod(temp,0o600)
            json.dump(value,handle,ensure_ascii=False)
            handle.flush();os.fsync(handle.fileno())
        temp.replace(path)
    finally:temp.unlink(missing_ok=True)


@contextmanager
def activation_lock(maps):
    """Coordinate start with the root helper without granting map write access."""
    import fcntl
    path=maps.maps/'.m20-catalog.lock'
    if path.is_symlink() or not path.is_file():raise ValueError('地图保护锁尚未准备，请先完成驻机服务部署')
    with path.open('rb') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX)
        try:yield
        finally:fcntl.flock(handle,fcntl.LOCK_UN)


MAP_LEASE_FILE = 'gos-navigation-map.json'
MAP_LEASE_SECONDS = 20.


def held_map_lease(data, now=None):
    row = read_json(Path(data)/MAP_LEASE_FILE, {})
    remaining = row.get('expires', 0) - (time.monotonic() if now is None else now)
    return row if 0 < remaining <= MAP_LEASE_SECONDS else None


def navigation_grid_identity(grid, raw):
    return mission_map(dict(name=grid['robot_map_name'], grid_id=grid['grid_id'],
                            resolution=grid['resolution'], origin=grid['origin'],
                            sha256=hashlib.sha256(raw).hexdigest(), yaml_sha256=grid['yaml_sha256']))


def grid_file_signature(maps, expected):
    folder, pgm, yaml = maps.grid_files('active', expected['grid_id'])
    if folder.name != expected['name']:
        raise ValueError('NOS 活动地图已变化')
    return [[stat.st_ino, stat.st_size, stat.st_mtime_ns] for stat in (pgm.stat(), yaml.stat())]


def manage_map_lease(maps, request):
    """Hold the NOS map while the GOS controls motion. Renewal checks files only."""
    import base64
    key = dict(id=identity(request.get('id')), owner=identity(request.get('owner')))
    action = request.get('action')
    path = maps.data/MAP_LEASE_FILE
    with activation_lock(maps):
        now = time.monotonic()
        row = held_map_lease(maps.data, now)
        owned = row is not None and all(row[k] == value for k, value in key.items())
        if action == 'release':
            if owned:
                path.unlink(missing_ok=True)
            return dict(ok=True, released=owned)
        if action == 'pulse' or action == 'claim' and owned:
            if not owned or row['files'] != grid_file_signature(maps, row['map']):
                raise ValueError('GOS 任务的活动地图授权已变化')
            if action == 'claim' and mission_map(request.get('map')) != row['map']:
                raise ValueError('重复申请使用的地图已变化')
            if row['expires'] - now < 12:
                row['expires'] = now + MAP_LEASE_SECONDS
                atomic(path, row)
            return dict(ok=True)
        if action != 'claim':
            raise ValueError('未知地图使用操作')
        if row:
            raise ValueError('其他 GOS 任务正在使用活动地图')
        resident = read_json(maps.data/'resident-mission.json', {})
        if resident.get('job') and resident.get('phase') not in ('idle', 'completed', 'cancelled', 'failed'):
            raise ValueError('NOS 驻机任务仍在运行或等待停车')
        if maps.dispatch('/maps/build/status', {}).get('in_progress', True):
            raise ValueError('请先结束建图并保存结果')
        expected = mission_map(request.get('map'))
        signature = grid_file_signature(maps, expected)
        grid = maps.grid('active', expected['grid_id'])
        raw = base64.b64decode(grid['pgm'], validate=True)
        if navigation_grid_identity(grid, raw) != expected or signature != grid_file_signature(maps, expected):
            raise ValueError('活动地图内容已变化，请重新同步')
        atomic(path, dict(key, map=expected, files=signature, expires=time.monotonic()+MAP_LEASE_SECONDS))
        return dict(ok=True)


class NativeMissionDriver:
    def __init__(self,maps,sensors,data,aos,clock=time.monotonic):
        self.maps=maps;self.sensors=sensors;self.data=Path(data);self.aos=aos;self.clock=clock
        self.link=None;self.expected=None;self.grid_key=None;self.map=None;self.planner=None
        self.goal=None;self.cancel_receipt=None;self.progress=None
        self.owned_values=set()
    def open(self,expected):
        from m20.platform_core import RobotLink,age_of
        if self.link is None:self.link=RobotLink(self.aos)
        self.expected=expected;self.goal=None;self.cancel_receipt=None;self.progress=None
        deadline=time.monotonic()+1.25
        while time.monotonic()<deadline:
            if all(age_of(self.link.record(t,c))<.65 for t,c in [(1007,1),(1007,2),(1002,6),(1002,4)]):return
            time.sleep(.025)
        self.close();raise ValueError('机内机器人状态尚未就绪，未提交任务')
    def _map(self):
        from m20.map_fusion import read_grid
        from m20.navigation_planner import GridPlanner
        folder=self.maps.path('active');grid_id=self.expected['grid_id']
        stem='occ_grid'+('_'+str(grid_id) if grid_id else '')
        if grid_id==0 and not (folder/(stem+'.pgm')).is_file():stem='occ_grid_0'
        pgm=folder/(stem+'.pgm');yaml=folder/(stem+'.yaml')
        key=(str(folder),pgm.stat().st_mtime_ns,pgm.stat().st_size,yaml.stat().st_mtime_ns,yaml.stat().st_size)
        if key!=self.grid_key:
            raw=pgm.read_bytes();config=yaml.read_bytes();grid=read_grid(folder,grid_id)
            self.map=dict(name=folder.name,grid_id=grid_id,sha256=hashlib.sha256(raw).hexdigest(),
                          yaml_sha256=hashlib.sha256(config).hexdigest(),resolution=grid['resolution'],origin=grid['origin'])
            # read_grid normalizes negate/thresholds to the standard palette.
            image=grid['image'];normalized=('P5\n%d %d\n255\n'%(image.shape[1],image.shape[0])).encode()+image.tobytes()
            self.planner=GridPlanner(normalized,grid['resolution'],grid['origin']);self.grid_key=key
        return self.map
    def frame(self):
        from m20.platform_core import age_of
        if not self.link:raise ValueError('驻机机器人连接未建立')
        began=self.clock();error=None
        try:scan=self.sensors().call('/sensors/autonomy',dict(mapping=False),timeout=.25)
        except Exception as exc:scan=dict(age=99);error=str(exc)
        pose_record=self.link.record(1007,2);basic_record=self.link.record(1002,6);motion_record=self.link.record(1002,4)
        native_record=self.link.record(1007,1)
        pose=pose_record.get('items',{});basic=basic_record.get('items',{}).get('BasicStatus',{})
        motion=motion_record.get('items',{}).get('MotionStatus',{});native=native_record.get('items',{})
        coordinates=[pose.get(k) for k in ('PosX','PosY','Yaw')]
        age=max(age_of(pose_record),age_of(basic_record),age_of(motion_record),
                float(scan.get('age',99))+self.clock()-began)
        safe=(basic.get('HES')==0 and basic.get('MotionState')==17 and not basic.get('Sleep')
              and basic.get('ControlUsageMode')==1 and basic.get('Gait')==0x3002 and basic.get('Direction')==0)
        aligned=(scan.get('protocol')==1 and bool(scan.get('floor')) and len(scan.get('pose',[]))==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in coordinates)
                 and math.dist(coordinates[:2],scan['pose'][:2])<.25
                 and abs(math.atan2(math.sin(coordinates[2]-scan['pose'][2]),math.cos(coordinates[2]-scan['pose'][2])))<.35)
        still=(age_of(motion_record)<.5 and all(abs(motion.get(k,999))<v for k,v in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)]))
        active=age_of(native_record)<.65 and native.get('Status') in (1,2,3,5,255)
        external=active and (not self.goal or native.get('Value')!=self.goal['value'])
        try:map_=self._map()
        except Exception as exc:map_={};error=str(exc)
        result=dict(ok=error is None,error=error,map=map_,pose=coordinates,stamp=scan.get('stamp'),age=age,
                    safe=safe,normal=pose.get('Location')==0 and pose.get('Confidence',0)>=.5 and aligned,
                    still=still,external_task=external,manual_override=external,scan=scan,
                    cancel_confirmed=False,goal_complete=False)
        if self.goal:
            with self.link.lock:rows=copy.deepcopy(self.link.events)
            rows += [native_record,self.link.record(1003,1)]
            for record in rows:
                item=record.get('items',{})
                if (record.get('type'),record.get('command')) not in [(1003,1),(1007,1)]:continue
                if record.get('monotonic_at',0)<=self.goal['sent_monotonic'] or age_of(record)>.65 or item.get('Value')!=self.goal['value']:continue
                code=item.get('ErrorCode',0);status=item.get('Status')
                if ((code==0x2300 and status in (0,4)) or (status==4 and code==0)):result['goal_complete']=True
                elif status==255 or code==0x2302:result['manual_override']=True
                elif code not in (0,12,0x2300):result.update(ok=False,error='机器人导航错误 0x%04X'%code)
            if self.progress is None or math.dist(coordinates[:2],self.progress[1])>.08:
                self.progress=(self.clock(),coordinates[:2])
            if self.clock()-self.progress[0]>30 or self.clock()-self.goal['sent_monotonic']>180:
                result.update(ok=False,error='导航持续无进展或超时')
        if self.cancel_receipt:
            ack=self.link.record(1004,1)
            result['cancel_confirmed']=(age_of(ack)<1 and ack.get('monotonic_at',0)>self.cancel_receipt['sent_monotonic']
                and ack.get('seq')==self.cancel_receipt['seq'] and ack.get('items',{}).get('ErrorCode')==0)
            result['still']=still and motion_record.get('monotonic_at',0)>self.cancel_receipt['sent_monotonic']
        return result
    def prepare(self,action,frame):
        if action['kind']=='cancel':return None
        if frame['age']>.65 or self._map()!=self.expected:raise ValueError('下发前状态或地图已过期')
        goal=action['point'];pose=frame['pose']
        if action['returning']:
            if math.dist(pose[:2],[goal['x'],goal['y']])>1.:raise ValueError('返回轨迹不连续')
            self.planner.update_obstacles(self._obstacles(frame['scan'],pose))
            heading=math.atan2(goal['y']-pose[1],goal['x']-pose[0])
            if (not self.planner.segment_fits(pose,[goal['x'],goal['y']])
                    or not self.planner.world_rotation_fits(pose,pose[2],heading)
                    or not self.planner.world_rotation_fits([goal['x'],goal['y']],heading,goal['yaw'])):
                raise ValueError('原路被障碍、未知区域或转向空间阻断，已请求停车')
        cell=self.planner.cell(goal['x'],goal['y'])
        if not self.planner.fits(*cell,goal['yaw']*16/math.pi):raise ValueError('目标净空不足')
        path=self.data/'resident-navigation-value.json'
        previous=json.loads(path.read_text()) if path.is_file() else secrets.randbelow(1<<30)+1024
        if type(previous) is not int or not 0<=previous<2147483646:raise ValueError('目标编号记录无效')
        value=previous+1;atomic(path,value)
        return dict(Value=value,MapID=self.expected['grid_id'],PosX=goal['x'],PosY=goal['y'],PosZ=0.,
            AngleYaw=goal['yaw'],PointInfo=1,NavMode=0 if action['returning'] else 1,Gait=0x3002,Speed=1,ObsMode=0,Manner=0)
    def send(self,action,frame,prepared=None):
        if action['kind']=='cancel':
            from m20.platform_core import age_of
            native=self.link.record(1007,1);items=native.get('items',{})
            if age_of(native)>1:raise ValueError('尚无新鲜的本机导航状态，停止结果待核对')
            if items.get('Status') in (1,2,3,5,255) and items.get('Value') not in self.owned_values:
                raise ValueError('检测到其他控制端任务，未发送全局取消')
            receipt=self.link.command(1004,1,{},'驻机任务停止')
            self.cancel_receipt=receipt;self.goal=None;self.progress=None;return receipt
        # Observation collection itself may block. Its receipt time is rechecked
        # immediately before a command; stale input never authorizes movement.
        if self.clock()-frame['_received']+frame['age']>.65:raise ValueError('下发前本地感知已过期')
        if not prepared:raise ValueError('目标尚未核对')
        receipt=self.link.command(1003,1,prepared,'驻机导航目标')
        self.goal=dict(value=prepared['Value'],**receipt);self.progress=None;self.cancel_receipt=None;return self.goal.copy()
    def stop_frame(self):
        """Parking proof does not depend on a working map or sensor HTTP call."""
        from m20.platform_core import age_of
        if not self.link or not self.cancel_receipt:return dict(cancel_confirmed=False,still=False)
        ack=self.link.record(1004,1);motion=self.link.record(1002,4);v=motion.get('items',{}).get('MotionStatus',{})
        sent=self.cancel_receipt['sent_monotonic']
        return dict(cancel_confirmed=(age_of(ack)<1 and ack.get('monotonic_at',0)>sent
            and ack.get('seq')==self.cancel_receipt['seq'] and ack.get('items',{}).get('ErrorCode')==0),
            still=(age_of(motion)<.5 and motion.get('monotonic_at',0)>sent
                and all(abs(v.get(k,999))<limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)])))
    @staticmethod
    def _obstacles(scan,pose):
        c,s=math.cos(pose[2]),math.sin(pose[2])
        return [(pose[0]+c*x*.2-s*y*.2,pose[1]+s*x*.2+c*y*.2) for x,y in scan.get('obstacles',[])]
    def close(self):
        if self.link:self.link.close(stop=False);self.link=None


class ResidentMissionService:
    def __init__(self,maps,sensors,data,driver=None,clock=time.monotonic,enabled=None):
        self.clock=clock;self.path=Path(data)/'resident-mission.json';self.thread=None;self.done=threading.Event()
        address=os.environ.get('M20_RESIDENT_AOS_IP','')
        try:valid=ipaddress.ip_address(address).is_private and not ipaddress.ip_address(address).is_loopback
        except ValueError:valid=False
        self.enabled=(os.environ.get('M20_RESIDENT_NAVIGATION')=='1' and valid) if enabled is None else bool(enabled)
        self.maps=maps;self.driver=driver or NativeMissionDriver(maps,sensors,data,address,clock)
        self.trace_signature=None;self.trace_receipt={}
        self.engine=ResidentMission(self._save);self.start_lock=threading.Lock()
        if self.path.is_file():
            if self.path.is_symlink() or self.path.stat().st_size>4_000_000:raise ValueError('驻机任务记录异常')
            receipt=json.loads(self.path.read_text(encoding='utf-8'))
            if receipt.get('trace_file'):
                name=receipt.get('job',{}).get('id','')+'.json'
                if receipt['trace_file']!=name:raise ValueError('轨迹引用无效')
                trace=self.path.parent/'resident-traces'/name
                if trace.is_symlink() or trace.stat().st_size>4_000_000:raise ValueError('轨迹记录异常')
                raw=trace.read_bytes()
                if hashlib.sha256(raw).hexdigest()!=receipt.get('trace_sha256'):raise ValueError('轨迹记录不完整')
                receipt['trace']=json.loads(raw)
            self.engine.restore(receipt)
    def _save(self,receipt):
        row=copy.deepcopy(receipt);points=row.pop('trace',[]);job=row.get('job')
        if job:job.pop('return_points',None)  # Restarts only cancel; no path replay.
        if points and job:
            signature=(job['id'],len(points),tuple(points[-1]))
            if signature!=self.trace_signature:
                folder=self.path.parent/'resident-traces'
                if folder.is_symlink():raise ValueError('轨迹目录异常')
                name=job['id']+'.json';path=folder/name;atomic(path,points)
                self.trace_receipt=dict(trace_file=name,trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                self.trace_signature=signature
            row.update(self.trace_receipt)
        atomic(self.path,row)
    def dispatch(self,path,request=None):
        if path=='/missions/capabilities':return dict(ok=True,protocol=1,available=self.enabled,
            map_lease_protocol=1,
            policies=['stop','finish','return','finish_return'],execution='onboard_native',
            message='驻机导航已配置' if self.enabled else '需要统一加载驻机导航并核对机内AOS连接')
        if path=='/missions/status':return dict(ok=True,mission=self.engine.public())
        req=request or {}
        if path=='/missions/pulse':return dict(ok=True,mission=self.engine.pulse(req['id'],req['owner'],self.clock()))
        if path=='/missions/cancel':
            with self.engine.lock:
                if self.engine.phase=='interrupted' and self.engine.job and self.engine.job['id']==req.get('id'):
                    self.engine._owner(req['id'],req['owner'])
                    self.driver.open(self.engine.job['map'])
                    self.driver.owned_values=set(self.engine.job.get('owned_values',[]))
            row=self.engine.cancel(req['id'],req['owner'],self.clock());self._worker()
            return dict(ok=True,mission=row,matched=row.get('id')==req['id'],revoked=req['id'] in self.engine.revoked)
        if path!='/missions/start':raise ValueError('驻机任务接口不存在')
        if not self.enabled:raise ValueError('驻机执行服务尚未配置，不能启用断联继续或返回')
        with self.start_lock:
            spec=specification(req)
            with self.engine.lock:
                if self.engine.job and self.engine.job['id']==spec['id']:
                    # Idempotent retries neither create a link nor reissue a goal.
                    return dict(ok=True,mission=self.engine.start(req,{},self.clock()))
                if self.engine.phase not in TERMINAL:raise ValueError('驻机任务仍在执行或停止尚未确认')
            with activation_lock(self.maps):
                if held_map_lease(self.path.parent):
                    raise ValueError('GOS 任务仍在使用活动地图')
                try:
                    self.driver.open(spec['map'])
                    frame=self.driver.frame()
                    if frame.get('still') is not True:raise ValueError('请先停稳再提交驻机任务')
                    row=self.engine.start(req,frame,self.clock())
                except Exception:
                    self.driver.close();raise
                try:self._worker()
                except Exception:
                    self.engine.fail('驻机执行线程未启动，停车状态待核对',self.clock());self.driver.close();raise
                return dict(ok=True,mission=row)
    def _worker(self):
        with self.engine.lock:
            if self.thread and self.thread.is_alive():return
            self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def recover(self):
        """Explicit startup reconciliation sends cancellation only, never a goal."""
        if self.engine.phase!='interrupted':return
        self.driver.open(self.engine.job['map'])
        self.driver.owned_values=set(self.engine.job.get('owned_values',[]))
        self.engine.cancel(self.engine.job['id'],self.engine.job['owner'],self.clock());self._worker()
    def _run(self):
        while not self.done.is_set():
            if self.engine.phase in TERMINAL:return
            try:frame=self.driver.frame();frame['_received']=self.clock()
            except Exception as error:frame=dict(ok=False,error=str(error))
            if self.engine.phase=='stopping':
                try:frame.update(self.driver.stop_frame())
                except Exception:frame.update(cancel_confirmed=False,still=False)
            action=self.engine.step(frame,self.clock())
            if action:
                try:prepared=self.driver.prepare(action,frame)
                except Exception as error:
                    self.engine.dispatch_failed(action,str(error),self.clock());self.done.wait(.1);continue
                with self.engine.lock:
                    if self.engine.accepts(action):
                        try:
                            if action['kind']=='goal':
                                values=self.engine.job.setdefault('owned_values',[])
                                if len(values)>=30000:raise ValueError('目标记录容量已满')
                                values.append(prepared['Value']);self.engine._persist()
                                self.driver.owned_values=set(values)
                            receipt=self.driver.send(action,frame,prepared);self.engine.dispatched(action,receipt,self.clock())
                        except Exception as error:self.engine.dispatch_failed(action,str(error),self.clock())
            self.done.wait(.1)
    def close(self):
        if self.engine.job and self.engine.phase not in TERMINAL:
            self.engine.cancel(self.engine.job['id'],self.engine.job['owner'],self.clock());self._worker()
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and self.engine.phase not in TERMINAL:self.done.wait(.05)
        self.done.set()
        if self.thread:self.thread.join(timeout=.5)
        self.driver.close()

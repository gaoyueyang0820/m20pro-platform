"""Central console operations: connections, deployment, maps, files and SDK flow."""
import base64
import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
import zipfile

from connection_manager import Connections,ROLES
from map_formats import Files,inspect_package,preview_pcd,identifier
from operations import Operations
from platform_core import rpc,number,age_of
from bridge.map_service import MapService
from bridge.mapping_job import empty_capture_confirmed
from localization_runtime import OperationCancelled


class Hub:
    def __init__(self,app):
        self.app=app
        self.data=app.store.path.parent
        self.files=Files(self.data/'files')
        self.operations=Operations()
        self.connections=Connections(self.data,app.add_log)
        self.previews=self.data/'previews';self.previews.mkdir(parents=True,exist_ok=True)
        self.demo_maps=MapService(self.data/'demo_robot/maps',self.data/'demo_nos',True)
        self.sdk_mode=dict(state='normal',source='demo')
        self.demo_policy=dict(current_state='idle',current_state_id=0,source='ros2',fresh=True,telemetry_fresh=True,vendor_running=False)
        self.busy=None
        self._busy_owner=None;self.preparation=None;self.mapping_uncertain=None
        self._operation_context=threading.local()
        self._context_generation=0
        self._connection_lost_since=None;self._connection_search_at=-30.;self._connection_search=None
        self.done=False
        self._seed_demo()

    def _seed_demo(self):
        folder=self.demo_maps.maps/'demo-yard'
        if folder.exists() or self.demo_maps.maps.exists() and any(p.is_dir() and not p.name.startswith('.') for p in self.demo_maps.maps.iterdir()):return
        folder.mkdir(parents=True)
        w,h=320,220
        pixels=bytearray([245]*(w*h))
        cloud=[]
        rectangles=[(-13.5,-8.5,13.5,8.5),(-13,-8,-7,-4),(-4,-8,3,-4),(6,-8,13,-4),(-13,4,-7,8),(-4,4,3,8),(6,4,13,8)]
        for x0,y0,x1,y1 in rectangles:
            for n in range(120):
                t=n/119
                for x,y in [(x0+(x1-x0)*t,y0),(x0+(x1-x0)*t,y1),(x0,y0+(y1-y0)*t),(x1,y0+(y1-y0)*t)]:
                    px=round((x+16)/.1);py=h-1-round((y+11)/.1)
                    if 0<=px<w and 0<=py<h:pixels[py*w+px]=0
                    for z in [0,.4,.8,1.2,1.6,2.]:cloud.append((x,y,z))
        for x in range(-26,27):
            for y in range(-16,17):cloud.append((x*.5,y*.5,0))
        (folder/'occ_grid.pgm').write_bytes(b'P5\n320 220\n255\n'+pixels)
        (folder/'occ_grid.yaml').write_text('image: occ_grid.pgm\nresolution: 0.1\norigin: [-16, -11, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        header='VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA ascii\n'%(len(cloud),len(cloud))
        (folder/'full_cloud.pcd').write_text(header+''.join('%.3f %.3f %.3f\n'%p for p in cloud))
        active=self.demo_maps.maps/'active'
        if not active.exists():active.symlink_to(folder.name,target_is_directory=True)

    def proxy(self,role,path,req=None,timeout=4):
        with self.app.lock:
            context=getattr(self._operation_context,'value',None)
            if context is not None and not self.same_context(context):raise OperationCancelled('连接已变化，未向新连接发送旧操作')
            target=self.connections.value['targets'][role]
            if target.get('enabled'):
                base,token=self.connections.endpoint(role)
            else:base,token=self.app.config.get(role),self.app.bridge_token
        return rpc(base,path,req,timeout,token)

    def idle(self):
        with self.app.lock:
            resident=getattr(self.app,'resident_navigation',None)
            if resident and resident.busy():raise ValueError('驻机任务或停车状态尚未确认')
            if getattr(self.app,"_greet_busy",False):raise ValueError("请先停止问候动作")
            if getattr(self.app,'autonomy',None) and self.app.autonomy.active:raise ValueError('请先停止自主任务')
            if self.app.nav['status'] in ['running','cancelling'] or any(self.app.velocity) or self.app.sdk.get('armed'):
                raise ValueError('请先停止运动并结束导航/关节控制')

    def normal_mode(self):
        if self.sdk_mode.get('state') in ['sdk','awaiting_handset_exit'] or self.app.sdk.get('vendor_running'):
            raise ValueError('请先结束 SDK 输出并返回原厂模式')

    def mutate(self,kind,work):
        with self.app.lock:
            self.idle()
            if self.busy:raise ValueError('已有后台操作正在进行：'+self.busy)
            self.busy=kind;owner=uuid.uuid4().hex;self._busy_owner=owner
            if self.preparation and self.preparation['kind']=='localize':self.preparation['busy_owner']=owner
            context=self.context()
        def wrapped(progress):
            try:
                if not self.same_context(context):raise OperationCancelled('连接已变化，已取消后台操作')
                self._operation_context.value=context
                return work(progress)
            finally:
                self._operation_context.value=None
                with self.app.lock:
                    if self._busy_owner==owner:self.busy=None;self._busy_owner=None
        try:return self.operations.submit(kind,wrapped)
        except Exception:
            if self._busy_owner==owner:self.busy=None;self._busy_owner=None
            raise

    def context(self):return (self.app.demo,self.app.link,self._context_generation)

    def same_context(self,context):
        return context==self.context() and not self.done and not self.app.done

    def detach_context(self):
        """Release old local ownership; never route cleanup through a new link."""
        p=self.preparation;self._context_generation+=1;self.preparation=None
        if p:
            p['cancelled']=True
            if self._busy_owner in [p['id'],p.get('busy_owner')]:self.busy=None;self._busy_owner=None
            if p.get('dispatching'):
                self.mapping_uncertain=dict(request_id=p.get('request_id',p['id']),context=p['context'],message='旧连接建图启动结果需重新核对')

    def cancel_preparations(self):
        # Called from the local stop path. Never wait on a network request here.
        if self.preparation:self.preparation['cancelled']=True
        localizer=getattr(self.app,'localizer',None)
        if localizer:localizer.cancel()

    def has_preparation(self,run_id=None):
        p=self.preparation
        return bool(p and p['kind']=='mapping' and (run_id is None or p.get('run_id')==run_id))

    def preparation_job(self,save=False):
        p=self.preparation
        if p and p['kind']=='mapping' and save:p['save_requested']=True
        return dict(ok=True,job_id=p['job_id']) if p and p['kind']=='mapping' and p.get('job_id') else None

    def check_preparation(self,token):
        with self.app.lock:
            if token is not self.preparation or token['cancelled'] or not self.same_context(token['context']):
                raise OperationCancelled('准备已取消或连接已变化，不再启动任务')
            if token.get('run_id'):
                a=self.app.autonomy
                if not a.active or a.run_id!=token['run_id'] or a.deadline is not None and time.monotonic()>=a.deadline:
                    token['cancelled']=True
                    raise OperationCancelled('自主任务已停止或到期，不再启动建图')

    def _own_mapping_status(self,token):
        if not self.same_context(token['context']):raise OperationCancelled('连接已变化，旧连接建图结果需重新核对')
        status=self.remote_maps('/maps/build/status',timeout=8,context=token['context'])
        start=status.get('mapping',{})
        if start.get('request_id')!=token.get('request_id',token['id']):
            raise ValueError('建图记录不属于本次请求，未停止其他采集')
        with self.app.lock:
            if not self.same_context(token['context']):raise OperationCancelled('连接已变化，未采用旧建图回包')
            self.app.mapping=bool(status.get('in_progress'))
        return status

    def _report_cancelled_preparation(self,token,result=None,error=None):
        with self.app.lock:
            a=self.app.autonomy
            if (not token.get('run_id') or a.generation!=token.get('owner_generation') or
                    a.active or a.state!='paused' or a._save_token is not None or not self.same_context(token['context'])):return
            if error:a.message='已停止自主移动；'+str(error)
            elif result.get('cleanup')=='saved':
                a.message='准备已取消，采集已停止并保存';a.result_maps=result.get('result_maps',[])
            elif result.get('cleanup')=='kept_existing':a.message='接续准备已取消，原采集仍保留，可手动操作或结束保存'
            else:a.message='准备已取消，未启动采集或自主移动'

    def _close_cancelled_mapping(self,token,progress):
        """A dispatched start can finish late; wait, then stop only that ID."""
        progress(.65,'已取消移动，正在收尾已发出的建图请求')
        status=self._own_mapping_status(token)
        if status.get('phase')=='saved':return dict(ok=True,cancelled=True,result_maps=status.get('result_maps',[]),cleanup='saved')
        if not status.get('in_progress'):return dict(ok=True,cancelled=True,cleanup='not_collecting')
        if status.get('phase')!='saving':
            if not self.same_context(token['context']):raise OperationCancelled('连接已变化，未向新连接发送停止')
            self.remote_maps('/maps/build',dict(action='stop',request_id=token['stop_id'],expected_start_request_id=token.get('request_id',token['id'])),timeout=15,context=token['context'])
        deadline=time.monotonic()+300
        while time.monotonic()<deadline:
            status=self._own_mapping_status(token)
            if status.get('phase')=='saved':return dict(ok=True,cancelled=True,cleanup='saved',result_maps=status.get('result_maps',[]))
            if status.get('phase')=='error':raise ValueError(status.get('message','取消后的地图保存失败'))
            time.sleep(.3)
        raise ValueError('停止保存仍待确认，请查看建图状态；本次任务不会恢复运动')

    def remote_maps(self,path,req=None,timeout=120,context=None):
        # Resolve a channel while holding the local state lock, then release it
        # before I/O. A late operation may never resolve against a new robot.
        context=context if context is not None else getattr(self._operation_context,'value',None)
        with self.app.lock:
            if context is not None and not self.same_context(context):raise OperationCancelled('连接已变化，未向新连接发送旧操作')
            observed_context=self.context()
            demo=self.app.demo;service=self.demo_maps
            if not demo:
                target=self.connections.value['targets']['nos']
                base,token=self.connections.endpoint('nos') if target.get('enabled') else (self.app.config.get('nos'),self.app.bridge_token)
        result=service.dispatch(path,req) if demo else self.proxy('nos',path,req,timeout) if context is None else rpc(base,path,req,timeout,token)
        if path=='/maps/build/status':
            with self.app.lock:
                a=getattr(self.app,'autonomy',None)
                if a is not None and self.same_context(observed_context):
                    a._discard_capability=result.get('discard_protocol')==1
        return result

    def preview(self,kind,key,progress):
        progress(.1,'正在读取点云地图')
        demo=self.app.demo
        if kind=='map':
            data=self.remote_maps('/maps/cloud',dict(name=key,limit=60000),120)
            data.update(name=key,source='demo' if demo else 'nos')
        elif kind=='file':
            meta=self.files.get(key)
            if meta['kind']=='pcd':
                data=preview_pcd(self.files.path(key),progress=progress)
            else:
                info=inspect_package(self.files.path(key))
                with zipfile.ZipFile(self.files.path(key)) as z:
                    name=next(i.filename for i in z.infolist() if i.filename in [info['prefix']+'full_cloud.pcd',info['prefix']+'jueying.pcd'])
                    if z.getinfo(name).file_size>1024**3:raise ValueError('包内点云超过 1 GiB，请先导入机器人后按地图预览，或使用降采样文件')
                    temp=self.previews/(uuid.uuid4().hex+'.pcd')
                    try:
                        import shutil
                        with z.open(name) as source,temp.open('wb') as out:shutil.copyfileobj(source,out,1024**2)
                        data=preview_pcd(temp,progress=progress)
                    finally:temp.unlink(missing_ok=True)
            data.update(name=meta['name'],source='local')
        else:raise ValueError('点云来源无效')
        key=uuid.uuid4().hex
        (self.previews/(key+'.json')).write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')))
        progress(1,'点云已就绪')
        return dict(ok=True,cloud_id=key,total_points=data['total_points'],shown_points=data['shown_points'])

    def export_map(self,name,progress):
        progress(.1,'正在打包完整地图文件')
        result=self.remote_maps('/maps/package/export',dict(name=name),180)
        meta=result['file']
        started=self.files.start(dict(name=meta['name'],size=meta['size'],kind='map-package'))['file']
        offset=0
        while offset<meta['size']:
            chunk=self.remote_maps('/files/read',dict(id=meta['id'],offset=offset),40)
            self.files.chunk(dict(id=started['id'],offset=offset,data=chunk['data']))
            if chunk['next']<=offset:raise ValueError('地图包传输中断')
            offset=chunk['next'];progress(.1+.8*offset/meta['size'],'正在下载地图包至控制电脑')
        final=self.files.finish(dict(id=started['id'],sha256=meta['sha256']))
        m=final['file'];m['map_format']=result.get('format','unknown')
        self.files._meta_path(m['id']).write_text(json.dumps(m,ensure_ascii=False))
        return dict(ok=True,file=m,format=m['map_format'])

    def sync_map(self,req):
        context=getattr(self._operation_context,'value',None) or self.context()
        grid=self.remote_maps('/maps/grid',req,context=context)
        with self.app.lock:
            if not self.same_context(context):raise OperationCancelled('地图同步期间连接已变化，旧栅格未导入当前机器人')
            if (req.get('name') not in [None,'active',grid.get('robot_map_name')]
                    or req.get('grid_id',0)!=grid.get('grid_id',0)):
                raise ValueError('返回地图或栅格与同步请求不一致，请重新同步')
            ip=self.app.link.ip if self.app.link else None
            matches=[m for m in self.app.store.value['maps'] if m.get('robot_map_name')==grid['robot_map_name'] and
                     m.get('grid_id',0)==grid.get('grid_id',0) and m.get('robot_ip')==ip]
            result=self.app.import_map(grid,existing_id=matches[0]['id'] if matches else None)
            selected=result['map']
            duplicate_ids={m['id'] for m in matches if m['id']!=selected['id'] and all(m.get(k)==selected.get(k) for k in ['origin','resolution','width','height'])}
            if grid.get('active'):
                for m in self.app.store.value['maps']:m['verified_for_robot']=False
            selected.update(grid_id=grid.get('grid_id',0),verified_for_robot=grid.get('active',False),robot_ip=ip,demo=self.app.demo,archived=False)
            self.app.store.value['maps']=[m for m in self.app.store.value['maps'] if m['id'] not in duplicate_ids]
            for route in self.app.store.value['routes']:
                if route['map_id'] in duplicate_ids:route['map_id']=selected['id']
            self.app.store.save()
            return result

    def _map_routes(self,name):
        """Saved definitions are not running jobs; retain data before hard loss."""
        ids={m['id'] for m in self.app.store.value['maps'] if m.get('robot_map_name')==name}
        if self.app.demo and name=='demo-yard':ids.add('demo')
        return [r for r in self.app.store.value['routes'] if r.get('map_id') in ids]

    def _require_unreferenced_map(self,name):
        routes=self._map_routes(name)
        if routes:
            labels='、'.join(str(r.get('name') or r['id']) for r in routes[:5])
            raise ValueError('本机已保存路线仍引用此地图，请先调整路线：'+labels)

    def _cache_map_matches(self,cached,name,context):
        return (cached.get('robot_map_name')==name and bool(cached.get('demo'))==context[0]
                and (context[0] or cached.get('robot_ip')==(context[1].ip if context[1] else None)))

    def _activation_guard(self,context,generation):
        if not self.same_context(context):raise OperationCancelled('连接已变化，地图定位接续已停止')
        if self.app.localizer.generation!=generation:raise OperationCancelled('地图定位接续已取消')
        self.normal_mode();self.idle()
        if not context[0]:self.app.localizer._stationary(context[1])

    def _save_activation(self,record):
        file=self.data/'map-activation.json';temporary=file.with_suffix('.tmp')
        temporary.write_text(json.dumps(record,ensure_ascii=False,indent=2));temporary.replace(file)

    def _activation_pose(self,context,generation,source):
        if context[0]:return None
        try:
            self._activation_guard(context,generation)
            status=self.app.localizer.status()
            if not self.app.localizer.good(status):return None
            record=context[1].record(1007,2);values=record.get('items',{})
            pose={key:number(values.get(key)) for key in ['PosX','PosY','PosZ','Yaw']}
            self._activation_guard(context,generation)
            return dict(pose=pose,source_map=source,captured_at=time.time())
        except OperationCancelled:raise
        except (ValueError,KeyError):return None

    def _wait_activation_localization(self,target,context,generation,after,*,expected=None,restart_at=None,seconds=12):
        """Require new post-initialization feedback, not pre-restart cached good."""
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self._activation_guard(context,generation)
            began=time.monotonic()
            try:status=self.app.localizer.status()
            except OperationCancelled:raise
            except ValueError:time.sleep(.2);continue
            elapsed=time.monotonic()-after['monotonic'];round_trip=time.monotonic()-began
            record=context[1].record(1007,2);pose=record.get('items',{})
            stamp=record.get('monotonic_at')
            newer=stamp>=after['monotonic'] if isinstance(stamp,(int,float)) else record.get('at',0)>=after['wall']
            updated=all(0<=(status.get(key) or {}).get('age',99)+round_trip<=elapsed+.02 for key in ['localization_quality','matching_quality'])
            odom=status.get('odom') or {}
            if restart_at is not None:updated=updated and odom.get('received_at',0)>=restart_at
            close=expected is None or (all(isinstance(pose.get(k),(int,float)) for k in ['PosX','PosY','Yaw']) and
                math.hypot(pose['PosX']-expected['PosX'],pose['PosY']-expected['PosY'])<.35 and
                abs(math.atan2(math.sin(pose['Yaw']-expected['Yaw']),math.cos(pose['Yaw']-expected['Yaw'])))<.25)
            if newer and updated and close and self.app.localizer.good(status):
                self._activation_guard(context,generation)
                if self.remote_maps('/maps/list',context=context).get('active')!=target:raise OperationCancelled('活动地图已变化，未采用旧图定位结果')
                self._activation_guard(context,generation)
                self.app.localizer.remember(status)
                self._activation_guard(context,generation)
                return dict(localized=True,ready=True,message='定位已就绪')
            time.sleep(.2)
        raise ValueError('新地图定位尚未就绪')

    def _restore_activation_localization(self,target,previous,proof,applied,seed,context,generation,progress):
        if context[0]:return dict(localized=True,ready=True,demo=True,method='simulated',message='演示地图已就绪')
        check=lambda:self._activation_guard(context,generation)
        after=dict(wall=time.time(),monotonic=time.monotonic())
        restart=applied.get('localization_restart_completed_at')
        confirmed=applied.get('activation_context') or {}
        inherited=(seed is not None and proof.get('protocol')==1 and proof.get('same_frame') is True and
            proof.get('source_map')==previous and proof.get('target_map')==target and
            confirmed.get('same_frame') is True and confirmed.get('source_map')==previous and
            confirmed.get('target_map')==target and confirmed.get('project_sha256')==proof.get('project_sha256'))
        seed_error=None
        if inherited:
            progress(.82,'正在接续当前位置')
            # No ICP or extra scan votes: a proved complete version keeps base
            # coordinates. Do NOT apply the new-session registration matrix.
            for attempt in range(2):
                try:
                    check()
                    if self.remote_maps('/maps/list',context=context).get('active')!=target:raise OperationCancelled('活动地图已变化，停止定位接续')
                    with self.app.lock:
                        check();after=dict(wall=time.time(),monotonic=time.monotonic())
                        context[1].command(2101,1,seed['pose'],'接续更新地图的当前位置')
                    result=self._wait_activation_localization(target,context,generation,after,expected=seed['pose'],restart_at=restart,seconds=3 if attempt==0 else 9)
                    return dict(result,method='inherited_map_pose',source_map=previous)
                except OperationCancelled:raise
                except ValueError as exc:seed_error=str(exc)
            # Do not turn the fast same-frame handoff into a long full-map
            # search. The applied map is retained and reported as not ready.
            raise ValueError('已接续原位置，定位尚未收敛：'+str(seed_error))
        else:progress(.82,'正在识别新地图中的当前位置')
        check()
        result=self.app.localizer.recover(lambda amount,message:progress(.82+.16*amount,message),check=check,verify_current=True)
        check()
        # Even an old service's accepted search is not enough: the readiness
        # readback must belong to the newly activated map's sensor epoch.
        ready=self._wait_activation_localization(target,context,generation,after,restart_at=restart)
        return dict(result,**ready,previous_pose_used=False,prior_initialization_error=seed_error)

    def import_package(self,file_id,name,progress):
        self.idle()
        meta=self.files.get(file_id)
        if meta['kind']!='map-package':raise ValueError('请选择已上传的 ZIP 地图包')
        inspect_package(self.files.path(file_id))
        remote=self.remote_maps('/files/start',dict(name=meta['name'],kind='map-package',size=meta['size']))['file']
        offset=0
        while offset<meta['size']:
            chunk=self.files.read_chunk(file_id,offset)
            self.remote_maps('/files/chunk',dict(id=remote['id'],offset=offset,data=chunk['data']),40)
            offset=chunk['next'];progress(.8*offset/meta['size'],'正在向机器人传输地图包')
        self.remote_maps('/files/finish',dict(id=remote['id'],sha256=meta['sha256']),120)
        progress(.9,'正在校验并导入地图；原活动地图保持不变')
        return self.remote_maps('/maps/package/import',dict(id=remote['id'],name=name),180)

    def ensure_sdk_service(self,progress):
        target=self.connections.value['targets']['sdk']
        if not target.get('deployed'):raise ValueError('SDK 连接服务尚未部署，请在连接中心完成一次配置')
        if not target.get('enabled'):self.connections.save(dict(targets=dict(sdk=dict(enabled=True))))
        progress(.05,'连接 SDK 后台服务')
        try:return self.proxy('sdk','/status',timeout=2)
        except (ValueError,OSError):pass
        self.connections.start('sdk')
        self.connections.service(dict(role='sdk',action='start'))
        deadline=time.monotonic()+10
        while True:
            try:return self.proxy('sdk','/status',timeout=2)
            except (ValueError,OSError) as exc:
                if time.monotonic()>=deadline:raise ValueError('SDK 服务未就绪：'+str(exc))
                time.sleep(.3)

    def sdk_status(self):
        a=self.app
        if a.demo:
            return dict(ok=True,**self.sdk_mode,demo=True,software_switch_supported=True,
                        environment=dict(ready=True,message='演示工程就绪'),agent_ready=True,calibration_verified=True,feedback_fresh=True,posture=a.basic.get('MotionState'),
                        note='SDK 演示环境')
        state=dict(self.sdk_mode)
        if state.get('source')=='demo':state=dict(state='unknown',source='unavailable')
        return dict(ok=True,**state,demo=False,software_switch_supported=False,
                    environment=a.sdk.get('environment') or dict(ready=False,stage='agent_update_required' if a.sdk.get('connected') else 'disconnected',message='连接服务需更新，请在连接中心更新 SDK 服务' if a.sdk.get('connected') else '尚未连接 SDK 后台服务'),agent_ready=bool(a.sdk.get('connected')),calibration_verified=bool(a.sdk.get('calibration_verified')),
                    feedback_fresh=bool(a.sdk.get('fresh')),posture=a.basic.get('MotionState'),
                    note='在手柄开启 SDK 后连接；结束平台控制后，在手柄退出 SDK。')

    def sdk_action(self,req):
        action=req.get('action')
        a=self.app
        if action in ['prepare','enter','confirm_handset'] and a.mapping:
            self.get('/api/maps/build/status',{})
            if a.mapping:raise ValueError('请先结束建图采集和后处理，再切换 SDK')
        if action=='prepare':
            self.normal_mode()
            a.stop()
            if a.nav['status']=='cancelling':raise ValueError('正在等待导航取消确认，确认后可再次准备')
            a.command('lie')
            self.sdk_mode=dict(state='preparing',source='demo' if a.demo else 'operator_flow')
            return dict(ok=True,message='已提交停止和趴下，请等待机器人姿态反馈后切换 SDK 模式')
        if action=='enter':
            self.idle()
            if a.basic.get('MotionState') not in [0,4]:raise ValueError('进入 SDK 前需趴下或空闲')
            if not a.demo:raise ValueError('当前按厂商公开流程使用手柄授权切换；完成后在平台记录并检查 SDK 通道')
            self.sdk_mode=dict(state='sdk',source='demo')
            return dict(ok=True,message='演示环境已进入 SDK 流程，尚未开启关节控制')
        if action=='confirm_handset':
            self.idle()
            if req.get('confirmed') is not True:raise ValueError('请先确认已在手柄上完成授权与模式切换')
            self.sdk_mode=dict(state='sdk',source='operator_confirmed',confirmed_at=time.time())
            return dict(ok=True,message='已记录手柄开启 SDK；正在连接策略服务')
        if action=='exit':
            a.stop()
            if a.demo:
                self.sdk_mode=dict(state='normal',source='demo');a.basic['MotionState']=4
                self.demo_policy.update(current_state='idle',current_state_id=0,vendor_running=False)
                return dict(ok=True,message='演示环境已结束 SDK 流程')
            target=self.connections.value['targets']['sdk']
            if target.get('deployed') and target.get('enabled'):
                self.proxy('sdk','/policy/stop',{},timeout=12)
                self.proxy('sdk','/direct/stop',{},timeout=12)
                self.connections.service(dict(role='sdk',action='stop'))
            self.sdk_mode=dict(state='awaiting_handset_exit',source='operator_flow')
            return dict(ok=True,message='SDK 输出已停止。请在手柄退出 SDK，并确认已返回原厂模式')
        if action=='confirm_exit':
            if req.get('confirmed') is not True:raise ValueError('需确认手柄已退出 SDK')
            if a.sdk.get('armed'):raise ValueError('需先结束关节控制')
            self.sdk_mode=dict(state='normal',source='operator_confirmed',confirmed_at=time.time())
            return dict(ok=True)
        raise ValueError('未知 SDK 模式操作')

    def get(self,path,query):
        if path=='/api/nos/navigation-diagnostics':
            if self.app.demo:return dict(ok=True,demo=True,available=False,read_only=True,message='离线演示不读取机器人日志；本机回放结果见调试记录')
            return self.proxy('nos','/diagnostics/navigation',timeout=5)
        if path=="/api/autonomy/vision":return dict(ok=True,**self.app.autonomy.vision.snapshot(image=True))
        if path=="/api/autonomy/status":return dict(ok=True,**self.app.autonomy.snapshot())
        if path=='/api/motion/catalog':
            from motion_catalog import catalog
            return catalog()
        if path=='/api/maps/extensions':return dict(ok=True,plans=self.app.store.value.get('map_extensions',[]),native_extension_supported=False,platform_extension_supported=True,message='平台保留旧图，采集新轮次并配准融合为新版本；不调用会覆盖历史记录的隐藏 -c 参数')
        if path=='/api/maps/project':return self.remote_maps('/maps/project',dict(name=query.get('name',['active'])[0]))
        if path=='/api/nav/resident/capabilities':return self.app.resident_navigation.capabilities()
        if path=='/api/connections':return dict(ok=True,connections=self.connections.public())
        if path=='/api/operations':return dict(ok=True,jobs=self.operations.get(query.get('id',[None])[0]))
        if path=='/api/files':return dict(ok=True,files=self.files.list())
        if path=='/api/maps/library':return dict(self.remote_maps('/maps/list'),desktop_management_version=1,desktop_catalog_integrity_protocol=1,desktop_permanent_delete_supported=True)
        if path=='/api/maps/library/trash':return self.remote_maps('/maps/trash')
        if path=='/api/nos/status':
            return dict(self.demo_maps.library(),services=dict(localization='demo',planner='demo')) if self.app.demo else self.proxy('nos','/status',timeout=12)
        if path=='/api/nos/logs':
            return dict(ok=True,output='演示环境无真实定位、规划与建图日志') if self.app.demo else self.proxy('nos','/logs',timeout=12)
        if path=='/api/gos/status':
            if self.app.demo:return dict(ok=True,demo=True,role='gos',note='当前为演示，尚未连接真实 GOS')
            return self.proxy('gos','/status',timeout=8)
        if path=='/api/gos/logs':
            if self.app.demo:return dict(ok=True,demo=True,output='演示环境无真实 GOS 日志')
            return self.proxy('gos','/logs',timeout=12)
        if path=='/api/maps/build/status':
            context=self.context();result=self.remote_maps('/maps/build/status',timeout=10,context=context)
            with self.app.lock:
                if not self.same_context(context):raise OperationCancelled('连接已变化，未采用旧建图回包')
                start=result.get('mapping',{});stop=result.get('stop_mapping',{})
                # Legacy agents require the same positive vendor/process evidence.
                if (result.get('phase')=='saving' and not result.get('result_maps')
                        and empty_capture_confirmed(start,stop)):
                    result=dict(result,phase='empty',in_progress=False,
                                message='采集已停止，厂商报告没有有效位姿，未生成地图；请核对定位和雷达后重新建图')
                if 'in_progress' in result:self.app.mapping=bool(result['in_progress'])
                elif result.get('demo') or (stop.get('state')=='command_finished' and stop.get('ended',0)>start.get('started',0)) or start.get('state')=='failed':
                    self.app.mapping=False
                elif start.get('state')=='running' or stop.get('state')=='running':self.app.mapping=True
                if result.get('phase') in ['idle','saved','discarded','interrupted','empty'] and not result.get('in_progress'):
                    self.mapping_uncertain=None
                elif (self.mapping_uncertain and result.get('phase')=='collecting'
                      and start.get('request_id')==self.mapping_uncertain['request_id']):
                    self.mapping_uncertain=None
            return result
        if path=='/api/sdk/status':return self.sdk_status()
        if path=='/api/sdk/logs':return dict(ok=True,output='演示环境没有实机 SDK 进程日志') if self.app.demo else self.proxy('sdk','/logs',timeout=8)
        if path=='/api/sensors/status':
            if self.app.demo:return dict(ok=True,demo=True,imu=None,odom=None,topics=[])
            role=self.connections.value.get('sensor_source','nos')
            return dict(self.proxy(role,'/sensors/status',timeout=8),host_role=role)
        if path=='/api/sdk/policy/status':
            return dict(ok=True,policy=self.demo_policy,demo=True) if self.app.demo else self.proxy('sdk','/policy/status',timeout=8)
        return None

    def post(self,path,req):
        if path=='/api/sdk/setup':
            self.idle()
            if self.app.demo:return dict(ok=True,delivery='simulated',message='演示 SDK 环境已就绪')
            def setup(progress):
                self.ensure_sdk_service(progress)
                progress(.1,'检查工程并准备平台所需的 ROS2 输入')
                result=self.proxy('sdk','/policy/setup',{},timeout=1500)
                if result.get('reload_required',True):
                    progress(.9,'载入 SDK 环境')
                    self.connections.service(dict(role='sdk',action='restart'))
                return dict(result,message='SDK 环境已准备好，手柄开启 SDK 后点击连接')
            return self.mutate('sdk-setup',setup)
        if path=='/api/maps/extension-plan':
            from map_formats import map_name
            name=map_name(req.get('name'));base=req.get('base')
            library=self.remote_maps('/maps/list')
            if not any(m['name']==base for m in library['maps']):raise ValueError('请先选择已有地图作为参考')
            if name==base:raise ValueError('请使用新的地图名称，保留参考地图')
            if req.get('strategy','new_coverage')!='new_coverage':raise ValueError('旧版计划接口仅用于重新建图；原图续建请使用 /api/maps/extend-here')
            plan=dict(id=uuid.uuid4().hex,base=base,name=name,strategy='new_coverage',created=time.time(),status='prepared')
            with self.app.lock:
                self.app.store.value.setdefault('map_extensions',[]).append(plan);self.app.store.save()
            return dict(ok=True,plan=plan)
        if path=='/api/maps/localize-auto':
            verify_current=req.get('verify_current',False)
            if not isinstance(verify_current,bool):raise ValueError('静态识别参数无效')
            self.normal_mode();self.idle();self.app.live_ready()
            if self.app.mapping:raise ValueError('请先结束并保存建图，再恢复旧图定位')
            with self.app.lock:
                if self.busy:raise ValueError('正在处理上一项任务，请稍候')
                token=dict(id=uuid.uuid4().hex,kind='localize',context=self.context(),cancelled=False)
                self.preparation=token
            def localize(progress):
                try:
                    check=lambda:self.check_preparation(token)
                    return self.app.localizer.recover(progress,check=check,verify_current=True) if verify_current else self.app.localizer.recover(progress,check=check)
                finally:
                    with self.app.lock:
                        if self.preparation is token:self.preparation=None
            try:return self.mutate('自动恢复定位',localize)
            except Exception:
                if self.preparation is token:self.preparation=None
                raise
        if path in ['/api/maps/start-here','/api/maps/extend-here','/api/maps/continue-here']:
            from map_formats import map_name
            name=map_name(req.get('name','m20-map'))
            extending=path.endswith('extend-here');base=map_name(req.get('base')) if extending else None
            continuing=path.endswith('continue-here')
            gait=req.get('autonomy_gait');owner=req.get('_autonomy_run_id')
            if gait is not None and gait not in [0x1001,0x3002]:raise ValueError('当前自主平地流程不支持这个步态')
            self.normal_mode()
            with self.app.lock:
                if self.busy:raise ValueError('正在处理上一项任务，请稍候')
                if self.app.mapping and not continuing:raise ValueError('建图已经在进行，可直接查看扫描')
                if continuing and not owner:raise ValueError('请通过自主作业接续当前采集')
                if not owner and self.app.autonomy._save_token is not None:raise ValueError('上一张地图保存结果正在确认，请等待完成')
                self.app.live_ready()
                if self.app.basic.get('MotionState') not in [1,17] or self.app.estopped:raise ValueError('建图需要站立状态，请先站稳')
                if self.app.sdk.get('armed'):raise ValueError('请先结束关节控制，再开始建图')
                if owner:
                    a=self.app.autonomy
                    if not a.active or a.run_id!=owner or a.state!='preparing':raise OperationCancelled('自主准备已取消')
                    if any(self.app.velocity) or self.app.nav['status'] in ['preparing','running','cancelling'] or self.app.sdk.get('armed'):raise ValueError('请先停止其他控制任务')
                else:
                    self.app.stop();self.app.autonomy.capture_id=None;self.app.autonomy.generation+=1
                token=dict(id=uuid.uuid4().hex,stop_id=uuid.uuid4().hex,kind='mapping',run_id=owner,
                           context=self.context(),cancelled=False,dispatching=False,existing=continuing,save_requested=False,
                           owner_generation=self.app.autonomy.generation if owner else None)
                token['request_id']=token['id']
                self.preparation=token;self.busy='准备建图';self._busy_owner=token['id']
            def check():self.check_preparation(token)
            def begin(progress):
                try:
                    check()
                    # Read the persistent device state, not only a local cache.
                    prior=self.remote_maps('/maps/build/status',timeout=8,context=token['context'])
                    # Capture an existing session's identity even when Stop was
                    # pressed while this read was in flight; never use None as
                    # ownership for an automatic save.
                    existing_id=prior.get('mapping',{}).get('request_id')
                    if (continuing and self.same_context(token['context']) and prior.get('request_protocol')==1
                            and isinstance(existing_id,str) and len(existing_id)==32 and all(c in '0123456789abcdef' for c in existing_id)):
                        token['request_id']=existing_id
                    check()
                    protocol=prior.get('request_protocol')
                    if isinstance(protocol,bool) or not isinstance(protocol,int) or protocol<1:
                        raise ValueError('机器人连接服务尚未同步本次任务收尾功能，请在连接中心更新服务')
                    if continuing:
                        identity=prior.get('mapping',{}).get('request_id')
                        if prior.get('phase')!='collecting' or not prior.get('in_progress'):raise ValueError('当前采集未就绪，请核对状态后接续')
                        if not isinstance(identity,str) or len(identity)!=32 or any(c not in '0123456789abcdef' for c in identity):raise ValueError('当前采集缺少请求归属，先停止保存并核对后再开始')
                        token['request_id']=identity
                    elif prior.get('in_progress'):raise ValueError('机器人仍有采集或保存任务，请先完成收尾')
                    self.mapping_uncertain=None
                    anchor=None
                    if extending:
                        library=self.remote_maps('/maps/list',context=token['context']);check()
                        if library.get('active')!=base:raise ValueError('先启用要续建的地图并完成定位')
                        if not self.app.demo:
                            location=self.app.link.record(1007,2)
                            if time.time()-location.get('at',0)>2 or location.get('items',{}).get('Location')!=0:raise ValueError('请先在旧地图上完成定位，再开始续建')
                        pose=self.app.pose;anchor=dict(x=pose.get('PosX',0),y=pose.get('PosY',0),z=pose.get('PosZ',0),yaw=pose.get('Yaw',0))
                    deadline=time.monotonic()+6
                    while self.app.nav['status']=='cancelling' and time.monotonic()<deadline:check();time.sleep(.1)
                    check()
                    if self.app.nav['status']=='cancelling':raise ValueError('尚未收到导航停止确认，请查看任务状态')
                    if not self.app.demo:
                        with self.app.lock:
                            check();self.app.live_ready()
                            basic=self.app.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                            if basic.get('Sleep'):
                                config=self.app.link.record(1101,7).get('items',{})
                                wake=dict(Sleep=False)
                                if isinstance(config.get('Auto'),bool):wake['Auto']=config['Auto']
                                if isinstance(config.get('Time'),int) and not isinstance(config['Time'],bool) and 5<=config['Time']<=30:wake['Time']=config['Time']
                                self.app.link.command(1101,6,wake,'开始建图前唤醒')
                            if self.app.basic.get('ControlUsageMode')!=0:self.app.link.set_mode(0)
                        deadline=time.monotonic()+6
                        while time.monotonic()<deadline:
                            check();basic=self.app.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                            motion=self.app.link.record(1002,4);values=motion.get('items',{}).get('MotionStatus',{})
                            still=time.time()-motion.get('at',0)<2 and all(abs(values.get(k,999))<.08 for k in ['LinearX','LinearY','OmegaZ'])
                            if basic.get('ControlUsageMode')==0 and basic.get('MotionState')==17 and not basic.get('Sleep') and still:break
                            time.sleep(.1)
                        else:raise ValueError('机器人仍休眠或未站立停稳，未启动建图')
                    if self.app.demo:
                        check();self.app.basic['ControlUsageMode']=0
                        if gait is not None:self.app.basic['Gait']=gait
                    elif gait is not None:
                        with self.app.lock:
                            check();basic=self.app.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                            if basic.get('Gait')!=gait:self.app.link.command(2,23,{'GaitParam':gait},'自主建图切换敏捷平地步态')
                        deadline=time.monotonic()+6
                        while time.monotonic()<deadline:
                            check();basic=self.app.link.record(1002,6).get('items',{}).get('BasicStatus',{})
                            motion=self.app.link.record(1002,4);values=motion.get('items',{}).get('MotionStatus',{})
                            if basic.get('Gait')==gait and time.time()-motion.get('at',0)<.5 and all(abs(values.get(k,999))<.08 for k in ['LinearX','LinearY','OmegaZ']):break
                            time.sleep(.1)
                        else:raise ValueError('敏捷平地步态尚未确认或未停稳，暂不开始建图')
                    progress(.5,'已停稳，正在启动建图')
                    with self.app.lock:
                        check()
                        if continuing:progress(.9,'控制模式已就绪，正在确认原采集归属')
                        if extending:
                            if not self.app.demo:
                                location=self.app.link.record(1007,2)
                                if time.time()-location.get('at',0)>2 or location.get('items',{}).get('Location')!=0:raise ValueError('停稳后定位尚未就绪，请重新定位再续建')
                            pose=self.app.pose;anchor.update(x=pose.get('PosX',0),y=pose.get('PosY',0),z=pose.get('PosZ',0),yaw=pose.get('Yaw',0))
                        payload=dict(action='extend',base=base,anchor=anchor,grid_id=req.get('grid_id',0)) if extending else dict(action='start',name=name)
                        payload['request_id']=token['request_id']
                        if not continuing:token['dispatching']=True
                    if continuing:
                        status=self._own_mapping_status(token);check()
                        if status.get('phase')!='collecting':raise ValueError('原采集状态已变化，未接续运动')
                        return dict(ok=True,phase='collecting',request_id=token['request_id'],continuing=True)
                    result=self.remote_maps('/maps/build',payload,timeout=15,context=token['context'])
                    with self.app.lock:
                        if self.same_context(token['context']):self.app.mapping=True
                    check();deadline=time.monotonic()+45
                    while True:
                        status=self._own_mapping_status(token);check()
                        if status.get('phase')=='collecting':return dict(result,phase='collecting',request_id=token['id'])
                        if status.get('phase') in ['error','interrupted'] or not status.get('in_progress'):raise ValueError(status.get('message','建图未进入采集状态'))
                        if time.monotonic()>=deadline:raise ValueError('建图启动仍未就绪，正在停止保存本次请求')
                        time.sleep(.2)
                except Exception as original_error:
                    if token['dispatching'] or token['existing'] and token['save_requested']:
                        try:
                            result=self._close_cancelled_mapping(token,progress)
                            if token['cancelled']:
                                self._report_cancelled_preparation(token,result);return result
                        except Exception as cleanup_error:
                            if self.same_context(token['context']) or self.mapping_uncertain is None:
                                self.mapping_uncertain=dict(request_id=token['request_id'],context=token['context'],message=str(cleanup_error))
                            self._report_cancelled_preparation(token,error='建图启动/收尾结果待核对：'+str(cleanup_error))
                            raise ValueError('本次建图启动/收尾结果待核对：'+str(cleanup_error))
                    elif isinstance(original_error,OperationCancelled):
                        result=dict(ok=True,cancelled=True,cleanup='kept_existing' if token['existing'] else 'not_collecting')
                        self._report_cancelled_preparation(token,result);return result
                    raise
                finally:
                    with self.app.lock:
                        if self.preparation is token:self.preparation=None
                        if self._busy_owner==token['id']:self.busy=None;self._busy_owner=None
            try:
                result=self.operations.submit('mapping-extend' if extending else 'mapping-start',begin)
                with self.app.lock:token['job_id']=result['job_id']
                return result
            except Exception:
                with self.app.lock:
                    if self.preparation is token:self.preparation=None
                    if self._busy_owner==token['id']:self.busy=None;self._busy_owner=None
                raise
        if path=='/api/sdk/connect':
            self.idle()
            self.sdk_action(dict(action='confirm_handset',confirmed=True))
            if self.app.demo:
                self.demo_policy['vendor_running']=True
                return dict(ok=True,delivery='simulated',message='SDK 演示已连接，可直接使用下方操作')
            def connect(progress):
                self.ensure_sdk_service(progress)
                return self.proxy('sdk','/policy/connect',dict(mode_confirmed=True),timeout=20)
            return self.mutate('sdk-connect',connect)
        if path=='/api/nav/manual':
            self.normal_mode()
            with self.app.lock:
                if self.busy:raise ValueError('请等待当前后台任务结束')
                running=self.app.nav['status'] in ['running','paused','cancelling']
                self.app.stop()
                if running:
                    self.app.nav['after_cancel']='paused'
                    if self.app.demo:self.app.nav['status']='paused'
                self.busy='切换手动'
            def takeover(progress):
                try:
                    deadline=time.monotonic()+6
                    while self.app.nav['status']=='cancelling' and time.monotonic()<deadline:time.sleep(.1)
                    if self.app.nav['status']=='cancelling':raise ValueError('仍在等待导航暂停确认')
                    with self.app.lock:
                        self.busy=None
                        return self.app.command('mode_normal')
                finally:self.busy=None
            return self.operations.submit('nav-manual',takeover)
        if path=='/api/connections/save':return self.connections.save(req)
        if path=='/api/connections/scan':return self.operations.submit('host-key-scan',lambda progress:self.connections.scan(req.get('role'),progress))
        if path=='/api/connections/trust':return self.connections.trust(req)
        if path=='/api/connections/pair':return self.operations.submit('device-pair',lambda progress:self.connections.pair(req.get('role'),progress,replace=req.get('replace') is True))
        if path=='/api/connections/discover':return self.operations.submit('find-robot',self.connections.discover)
        if path=='/api/connections/deploy':
            return self.mutate('deploy-'+str(req.get('role')),lambda progress:self.connections.deploy(req,progress))
        if path=='/api/connections/target':
            role=req.get('role')
            if role not in ROLES:raise ValueError('无效连接目标')
            if req.get('action')=='stop':
                if role=='sdk':self.app.stop()
                return self.connections.stop(role)
            return self.connections.start(role)
        if path=='/api/connections/service':
            if req.get('action') in ['status','logs']:
                return self.operations.submit('service-read',lambda progress:self.connections.service(req))
            if req.get('role')=='sdk' and req.get('action') in ['stop','restart'] and not self.app.demo:
                self.app.stop()
                self.proxy('sdk','/policy/stop',{},timeout=15)
                self.proxy('sdk','/direct/stop',{},timeout=12)
            return self.mutate('service-'+str(req.get('role')),lambda progress:self.connections.service(req))
        if path=='/api/connections/connect':
            saved=self.connections.value
            return self.app.connect(dict(mode=req.get('mode','live'),ip=saved['aos_ip'],port=saved['aos_port']))
        if path=='/api/files/start':return self.files.start(req)
        if path=='/api/files/chunk':return self.files.chunk(req)
        if path=='/api/files/finish':return self.operations.submit('file-verify',lambda progress:self.files.finish(req))
        if path=='/api/files/inspect':return dict(ok=True,package=inspect_package(self.files.path(req.get('id'))))
        if path=='/api/cloud/preview':return self.operations.submit('cloud-preview',lambda progress:self.preview(req.get('kind'),req.get('key'),progress))
        if path=='/api/maps/package/export':return self.operations.submit('map-export',lambda progress:self.export_map(req.get('name'),progress))
        if path=='/api/maps/package/import':return self.mutate('map-import',lambda progress:self.import_package(req.get('id'),req.get('name'),progress))
        if path=='/api/maps/details':return self.remote_maps('/maps/details',req)
        if path in ['/api/maps/library/rename','/api/maps/library/delete','/api/maps/library/restore','/api/maps/library/purge']:
            action=path.rsplit('/',1)[-1]
            context=self.context()
            def edit(progress):
                if not self.same_context(context):raise OperationCancelled('地图编辑连接已变化，未向新连接提交操作')
                if action!='rename':
                    if self.app.mapping or self.remote_maps('/maps/build/status',context=context).get('in_progress'):raise ValueError('正在保存地图，完成后即可编辑文件')
                    learning=self.app.map_learning.snapshot()
                    if learning.get('enabled') or learning.get('state')=='generating':raise ValueError('地图更新正在使用文件，结束更新后即可编辑文件')
                if action=='purge':
                    capabilities=self.remote_maps('/maps/list',context=context)
                    if capabilities.get('catalog_integrity_protocol')!=1:raise ValueError('请先更新 NOS 地图管理服务，再彻底删除；未发送删除命令')
                    with self.app.lock:
                        if not self.same_context(context):raise OperationCancelled('地图编辑连接已变化')
                result=self.remote_maps('/maps/'+action,req,context=context)
                with self.app.lock:
                    if not self.same_context(context):raise OperationCancelled('连接已变化，旧机器人编辑结果未写入当前地图缓存')
                    for cached in self.app.store.value['maps']:
                        if self._cache_map_matches(cached,result.get('name'),context):
                            if action=='rename':cached['name']=result['display_name']
                            if action in ['delete','purge']:cached.update(verified_for_robot=False,archived=True)
                            if action=='restore':cached.update(verified_for_robot=False,archived=False)
                    self.app.store.save()
                return result
            # A display label does not touch map geometry or running jobs.
            if action=='rename':return self.operations.submit('map-rename',edit)
            return self.mutate('map-'+action,edit)
        if path=='/api/sensors/cloud':
            if self.app.demo:raise ValueError('演示环境不提供虚构的实时 ROS2 点云；请选择保存的演示地图')
            role=self.connections.value.get('sensor_source','nos')
            return dict(self.proxy(role,'/sensors/cloud',req,timeout=8),host_role=role)
        if path=='/api/maps/help':return self.operations.submit('drmap-help',lambda progress:self.remote_maps('/maps/help',{},65))
        if path=='/api/maps/library/apply':
            self.normal_mode()
            if self.app.mapping:
                self.get('/api/maps/build/status',{})
                if self.app.mapping:raise ValueError('正在采集或融合地图，保存完成后再切换地图')
            self.idle()
            context=self.context()
            localization_generation=self.app.localizer.generation
            def apply(progress):
                previous_library=self.remote_maps('/maps/list',context=context)
                previous=previous_library.get('active')
                target=req.get('name')
                try:proof=self.remote_maps('/maps/activation-context',dict(name=target),context=context)
                except ValueError:proof={}
                seed=self._activation_pose(context,localization_generation,previous)
                record=dict(state='preparing',requested_map=target,previous_map=previous,
                            robot=context[1].ip if context[1] else None,created=time.time(),prior_pose=seed,coordinate_evidence=proof)
                with self.app.lock:
                    if not self.same_context(context):raise OperationCancelled('地图启用连接已变化，未向新连接提交操作')
                    self._activation_guard(context,localization_generation)
                    self._save_activation(record)
                    for m in self.app.store.value['maps']:m['verified_for_robot']=False
                    self.app.store.save()
                payload=dict(req,expected_active=previous)
                if proof.get('protocol')==1:payload['expected_project_sha256']=proof['project_sha256']
                result=self.remote_maps('/maps/apply',payload,context=context)
                progress(.75,'活动地图已切换，正在同步导航栅格')
                try:synced=self.sync_map(dict(name=req.get('name'),grid_id=req.get('grid_id',0)))
                except ValueError as exc:raise ValueError('活动地图已切换，但导航栅格同步未完成：'+str(exc)+'。请点击“同步用于导航”重试')
                if not synced['map']['verified_for_robot']:raise ValueError('活动地图在同步期间发生变化，请重新核对')
                try:
                    localization=self._restore_activation_localization(target,previous,proof,result,seed,context,localization_generation,progress)
                except (ValueError,OperationCancelled) as exc:
                    if not self.same_context(context):raise
                    localization=dict(ready=False,localized=False,state='cancelled' if isinstance(exc,OperationCancelled) else 'waiting',message=str(exc))
                ready=localization.get('ready') is True
                with self.app.lock:
                    if not self.same_context(context):raise OperationCancelled('连接已变化，未保存旧地图的定位结果')
                    record.update(state='ready' if ready else 'localization_pending',applied=True,localization=localization,finished=time.time())
                    self._save_activation(record)
                if req.get('cleanup_previous') is True and previous and previous!=target:
                    cleaned=False
                    try:
                        if not ready:raise ValueError('新地图定位尚未就绪，原地图保留')
                        if previous_library.get('catalog_integrity_protocol')!=1:raise ValueError('机器人地图服务尚未支持完整性核对，未自动清理')
                        if req.get('cleanup_base')!=previous:raise ValueError('确认的旧版本与当前活动版本不一致，未自动清理')
                        with self.app.lock:
                            if not self.same_context(context):raise OperationCancelled('连接已变化，未清理旧图')
                            self._require_unreferenced_map(previous)
                        learning=self.app.map_learning.snapshot()
                        if learning.get('enabled') or learning.get('state')=='generating':raise ValueError('行走地图更新仍在运行')
                        result['cleanup']=self.remote_maps('/maps/cleanup-previous',dict(name=req['name'],base=previous),context=context)
                        cleaned=True
                    except Exception as exc:
                        result['cleanup']=dict(ok=False,retained=True,message='新图已启用，旧图保留：'+str(exc))
                    if cleaned:
                        with self.app.lock:
                            if not self.same_context(context):raise OperationCancelled('旧机器人已完成旧图归档，当前连接已变化；旧结果未写入当前缓存')
                            for cached in self.app.store.value['maps']:
                                if self._cache_map_matches(cached,previous,context):cached.update(verified_for_robot=False,archived=True)
                            self.app.store.save()
                message='地图已启用，定位已就绪' if ready else '地图已启用，定位尚未就绪：'+localization.get('message','等待定位反馈')
                return dict(result,map=synced['map'],navigation_ready=ready,localization=localization,message=message)
            return self.mutate('map-apply',apply)
        if path=='/api/maps/library/sync':
            return self.sync_map(req)
        if path=='/api/maps/sync':return self.post('/api/maps/library/sync',dict(name='active',grid_id=0))
        if path=='/api/nos/action':
            action=req.get('action')
            if action in ['mapping','stop_mapping']:
                return self.post('/api/maps/library/build',dict(action='start' if action=='mapping' else 'stop',name=req.get('name','m20-map')))
            if action=='pack':return self.post('/api/maps/package/export',dict(name='active'))
            if action=='apply':return self.post('/api/maps/library/apply',dict(name=req.get('name')))
            raise ValueError('NOS 操作无效')
        if path=='/api/maps/library/build':
            if req.get('action')=='start':return self.post('/api/maps/start-here',req)
            self.normal_mode()
            if req.get('action')=='stop':
                self.app.stop()
                pending=self.preparation_job()
                if pending:return pending
            self.idle()
            if req.get('action')=='start':
                self.app.live_ready()
                if self.app.basic.get('MotionState') not in [1,17] or self.app.estopped:
                    raise ValueError('建图启动需要机器人静态站立，请先站立并停止移动')
            context=self.context()
            def build(progress):
                result=self.remote_maps('/maps/build',req,context=context)
                if req.get('action')=='start':self.app.mapping=not self.app.demo
                return result
            return self.mutate('mapping',build)
        if path=='/api/sdk/action':return self.sdk_action(req)
        if path.startswith('/api/sdk/policy/'):
            operation=path.rsplit('/',1)[-1]
            if operation not in ['start','stop','state','velocity','build','prepare','direct-stop']:raise ValueError('SDK 软件操作无效')
            if operation in ['start','state','velocity'] and (self.busy or self.app.sdk.get('armed') or self.app.nav['status'] in ['running','cancelling']):
                raise ValueError('请先结束后台操作、关节控制或导航任务')
            if operation not in ['stop','direct-stop','build','prepare'] and self.sdk_mode.get('state')!='sdk':
                raise ValueError('请先完成 SDK 模式确认')
            if self.app.demo:
                p=self.demo_policy
                if operation=='start':p['vendor_running']=True
                if operation=='stop':p.update(vendor_running=False,current_state='idle',current_state_id=0);self.app.velocity=[0.,0.,0.]
                if operation=='state':
                    state=req.get('state');ids=dict(stand=1,rl_control=6,lie_down=4,damping=2)
                    if state not in ids:raise ValueError('SDK 状态无效')
                    if not p['vendor_running']:raise ValueError('请先启动演示 SDK 程序')
                    p.update(current_state=state,current_state_id=ids[state])
                if operation=='velocity':
                    v=[number(req.get(k,0),-v,v) for k,v in [('forward',.7),('side',.5),('yaw',.7)]]
                    if any(v) and (not p['vendor_running'] or p['current_state_id']!=6):raise ValueError('请先进入 SDK 策略控制')
                    self.app.velocity=v;self.app.velocity_at=time.time()
                return dict(ok=True,delivery='simulated',message='SDK 演示操作完成')
            if operation=='build':
                def build_sdk(progress):
                    result=self.proxy('sdk','/policy/build',dict(confirmed=True),timeout=1250)
                    progress(.9,'编译完成，正在重新载入 SDK 工作区')
                    self.connections.service(dict(role='sdk',action='restart'))
                    return dict(result,message='SDK 已编译并重新载入，可直接连接')
                return self.mutate('sdk-build',build_sdk)
            if operation=='prepare':return self.mutate('sdk-source',lambda progress:self.proxy('sdk','/policy/prepare',dict(confirmed=req.get('confirmed')),timeout=1500))
            if operation=='direct-stop':return self.proxy('sdk','/direct/stop',{},timeout=12)
            if operation in ['start','stop']:
                if operation=='start':self.idle()
                return self.proxy('sdk','/policy/'+operation,dict(req,mode_confirmed=True),timeout=15)
            return self.proxy('sdk','/policy/'+operation,req,timeout=8)
        return None

    def maintain_connection(self):
        """Find the paired robot after a network move without resuming old jobs."""
        with self.app.lock:
            if self.done or self.app.demo or not self.app.link:
                self._connection_lost_since=None;return
            if age_of(self.app.link.record(1002,6))<2:
                self._connection_lost_since=None;return
            now=time.monotonic()
            if self._connection_lost_since is None:self._connection_lost_since=now
            if now-self._connection_lost_since<5 or now-self._connection_search_at<30:return
            if self._connection_search and self._connection_search.is_alive():return
            context=self.context();self._connection_search_at=now
        def recover():
            try:
                result=self.connections.discover()
                with self.app.lock:
                    if result.get('found') and self.same_context(context) and not self.app.demo and age_of(self.app.link.record(1002,6))>=2:
                        saved=self.connections.value
                        if self.app.link.ip!=saved['aos_ip']:
                            self.app.connect(dict(mode='live',ip=saved['aos_ip'],port=saved['aos_port']))
                            self.app.add_log('connection','已在当前网络恢复机器人连接')
            except Exception:pass  # Next scheduled attempt retries; the UI retains link status.
        self._connection_search=threading.Thread(target=recover,daemon=True)
        self._connection_search.start()

    def loop(self):
        while not self.done:
            if not self.app.demo and not self.busy and self.mapping_uncertain and self.same_context(self.mapping_uncertain['context']):
                try:self.get('/api/maps/build/status',{})
                except Exception:pass
            try:self.app.localizer.poll()
            except Exception:pass
            try:self.app.map_learning.poll()
            except Exception:pass
            if not self.app.demo:
                self.maintain_connection()
                try:self.connections.poll_once()
                except Exception:pass
            for _ in range(20):
                if self.done:return
                time.sleep(.1)

    def close(self):
        self.done=True;self.connections.close()
        if hasattr(self.app,'map_learning'):self.app.map_learning.close()

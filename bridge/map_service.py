"""Map library API used by NOS and the explicitly labelled local demo."""
import base64
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import time
import tempfile
import uuid
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.map_formats import Files,map_name,pack_map,unpack_package,inspect_package,preview_pcd,digest
from m20.map_catalog import MapCatalog,catalog_lock
try:
    from .mapping_job import read_job,mapping_phase,request_identity,unowned_processes
except ImportError:
    from mapping_job import read_job,mapping_phase,request_identity,unowned_processes


class MapService:
    def __init__(self,maps=None,data=None,demo=False,runner=None):
        self.maps=Path(maps or '/var/opt/robot/data/maps')
        self.data=Path(data or os.environ.get('M20_DATA_DIR',Path.home()/'.local/share/m20-platform/data'))
        self.files=Files(self.data/'files')
        self.demo=demo
        self.runner=runner or self.run
        self.demo_build=dict(phase='idle',in_progress=False,message='离线演示 · 尚未开始建图',result_maps=[])
        self.demo_mapping=dict(state='none');self.demo_stop=dict(state='none')

    @staticmethod
    def run(argv,timeout=120):
        try:r=subprocess.run(argv,capture_output=True,text=True,timeout=timeout)
        except subprocess.TimeoutExpired:raise ValueError('机器人端操作超时，请查看服务日志核对结果')
        if r.returncode:raise ValueError((r.stderr or r.stdout or '机器人端操作失败')[-3000:])
        return r.stdout[-12000:]

    def path(self,name):
        if name=='active':
            p=(self.maps/'active').resolve()
        else:p=(self.maps/map_name(name)).resolve()
        if p.parent!=self.maps.resolve() or not p.is_dir():raise ValueError('地图不存在或路径无效')
        return p

    def activation_context(self,name):
        """Read evidence for carrying a pose from the CURRENT map to a version.

        Fusion's transform maps the NEW SLAM session into the base. It does not
        transform the base coordinate system, so a base pose stays unchanged.
        Similar names or root_map alone never establish this relationship.
        """
        from m20.map_change_policy import gate_alignment
        folder=self.path(name);active=(self.maps/'active').resolve()
        previous=active.name if active.is_dir() and active.parent==self.maps.resolve() else None
        meta=folder/'.m20-project.json';raw=meta.read_bytes() if meta.is_file() else b''
        proof=dict(protocol=1,source_map=previous,target_map=folder.name,same_frame=False,
                   frame_id='map',project_sha256=hashlib.sha256(raw).hexdigest(),reason='map_lineage_unproven')
        if previous==folder.name:return dict(ok=True,active=previous,**dict(proof,same_frame=True,method='same_active_map',reason='already_active'))
        try:project=json.loads(raw) if raw else {}
        except (ValueError,UnicodeError):return dict(ok=True,active=previous,**proof)
        if not isinstance(project,dict):return dict(ok=True,active=previous,**proof)
        rows=[(kind,row) for kind in ['sessions','refinements'] for row in (project.get(kind) or [])
              if isinstance(row,dict) and row.get('map')==folder.name and isinstance(row.get('created'),(int,float))
              and not isinstance(row['created'],bool) and math.isfinite(row['created'])]
        if not rows:return dict(ok=True,active=previous,**proof)
        kind,report=max(rows,key=lambda item:item[1].get('created',0))
        structure=report.get('structural_alignment') or {}
        known=(kind=='sessions' and report.get('known_region_change_policy')=='saved_keyframe_temporal_evidence' and isinstance(report.get('transform'),list)
               or kind=='refinements' and report.get('method')=='structure_aligned_temporal_visibility')
        if (not known or report.get('base_map')!=previous or report.get('includes_complete_base') is not True or
                (structure.get('decision') or {}).get('allowed') is not True or not gate_alignment(structure)['allowed']):
            return dict(ok=True,active=previous,**proof)
        proof.update(same_frame=True,method='complete_base_'+kind,reason='base_coordinates_preserved',
                     report_created=report.get('created'),includes_complete_base=True,
                     pose_transform='identity',structural_alignment=structure)
        return dict(ok=True,active=previous,**proof)

    def library(self):
        active=(self.maps/'active').resolve()
        catalog=MapCatalog(self.maps);labels=catalog.labels()
        result=[]
        if self.maps.is_dir():
            for p in sorted(self.maps.iterdir(),key=lambda p:p.name):
                if p.name.startswith('.') or p.name=='active' or p.is_symlink() or not p.is_dir() or (p/'.m20-session-source.json').exists():continue
                grids=sorted({int(m.group(1) or 0) for f in p.glob('occ_grid*.pgm') for m in [re.fullmatch(r'occ_grid(?:_(\d+))?\.pgm',f.name)] if m})
                cloud=p/'full_cloud.pcd'
                review=p/'.m20-alignment-review.json'
                try:review_state=json.loads(review.read_text()).get('status','pending') if review.is_file() else None
                except (OSError,ValueError):review_state='pending'
                try:cleanup_base=catalog.cleanup_base(p.name)
                except (OSError,ValueError,TypeError):cleanup_base=None
                result.append(dict(name=p.name,display_name=labels.get(p.name,p.name),cleanup_base=cleanup_base,active=p.resolve()==active,grids=grids,has_cloud=cloud.is_file(),alignment_review_status=review_state,
                                   requires_alignment_review=(p/'.m20-alignment-review.json').is_file(),
                                   cloud_bytes=cloud.stat().st_size if cloud.is_file() else 0,
                                   modified=p.stat().st_mtime,source='demo' if self.demo else 'nos'))
        usage=shutil.disk_usage(self.maps if self.maps.exists() else self.data)
        return dict(ok=True,maps=result,active=active.name if active.is_dir() else None,management_version=1,catalog_integrity_protocol=1,activation_handoff_protocol=1,permanent_delete_supported=True,trash_count=len(catalog.trash_list()),
                    disk=dict(total=usage.total,free=usage.free),demo=self.demo)

    def grid(self,name='active',grid_id=0):
        if not isinstance(grid_id,int) or isinstance(grid_id,bool) or not 0<=grid_id<=999:raise ValueError('栅格编号无效')
        folder=self.path(name)
        stem='occ_grid'+('_'+str(grid_id) if grid_id else '')
        if grid_id==0 and not (folder/(stem+'.pgm')).is_file():stem='occ_grid_0'
        yaml=(folder/(stem+'.yaml')).read_text()
        resolution=re.search(r'^resolution:\s*([\d.eE+-]+)',yaml,re.M)
        origin=re.search(r'^origin:\s*\[([^\]]+)\]',yaml,re.M)
        if not resolution or not origin:raise ValueError('地图配置缺少分辨率或原点')
        pgm=folder/(stem+'.pgm')
        if pgm.stat().st_size>12_000_000:raise ValueError('栅格超过 12 MB，请先降采样后编辑')
        return dict(ok=True,name=MapCatalog(self.maps).labels().get(folder.name,folder.name),robot_map_name=folder.name,grid_id=grid_id,
                    negate=int(re.search(r'^negate:\s*([01])',yaml,re.M)[1]) if re.search(r'^negate:\s*([01])',yaml,re.M) else 0,
                    free_thresh=float(re.search(r'^free_thresh:\s*([\d.]+)',yaml,re.M)[1]) if re.search(r'^free_thresh:\s*([\d.]+)',yaml,re.M) else .196,
                    occupied_thresh=float(re.search(r'^occupied_thresh:\s*([\d.]+)',yaml,re.M)[1]) if re.search(r'^occupied_thresh:\s*([\d.]+)',yaml,re.M) else .65,
                    resolution=float(resolution.group(1)),origin=[float(x) for x in origin.group(1).split(',')],
                    pgm=base64.b64encode(pgm.read_bytes()).decode(),yaml_sha256=hashlib.sha256((folder/(stem+'.yaml')).read_bytes()).hexdigest(),active=folder==(self.maps/'active').resolve())

    def privileged(self,action,req):
        payload=dict(action=action,**req)
        helper=Path(__file__).with_name('root_actions.py')
        if not helper.is_file():raise ValueError('未安装地图管理助手，请在连接中心重新部署 NOS 服务')
        argv=['sudo','-n','/usr/bin/python3',str(helper)]
        try:r=subprocess.run(argv,input=json.dumps(payload),capture_output=True,text=True,timeout=180)
        except subprocess.TimeoutExpired:raise ValueError('地图管理操作超时，请核对日志')
        try:result=json.loads(r.stdout)
        except ValueError:
            if r.returncode:raise ValueError('地图管理助手执行失败，请核对服务权限：'+(r.stderr or r.stdout)[-1000:])
            raise ValueError('地图管理助手返回格式异常')
        if not isinstance(result,dict):raise ValueError('地图管理助手返回格式异常')
        if not result.get('ok'):raise ValueError(result.get('err','地图管理失败'))
        if r.returncode:raise ValueError('地图管理助手异常退出，请查看服务日志')
        return result

    def dispatch(self,path,req):
        req=req or {}
        if path=='/maps/list':return self.library()
        if path=='/maps/trash':return dict(ok=True,maps=MapCatalog(self.maps).trash_list())
        if path in ['/maps/rename','/maps/delete','/maps/restore','/maps/purge','/maps/cleanup-previous']:
            action=path.rsplit('/',1)[-1]
            if self.demo:
                def busy():
                    if self.demo_build.get('in_progress'):raise ValueError('正在采集或保存地图，请完成后再编辑')
                return MapCatalog(self.maps,busy).execute(action,req)
            return self.privileged('catalog-'+action,req)
        if path=='/maps/project':
            folder=self.path(req.get('name','active'));meta=folder/'.m20-project.json'
            project=json.loads(meta.read_text()) if meta.is_file() else dict(root_map=folder.name,revision=1,sessions=[])
            capabilities=dict(ready=True,pipeline_version=2,message='离线演示') if self.demo else self.privileged('mapping_capabilities',{})
            return dict(ok=True,project=project,active=folder==(self.maps/'active').resolve(),extension_method='new_session_registration',original_preserved=True,extension_ready=capabilities['ready'],pipeline_version=capabilities['pipeline_version'],capability_message=capabilities['message'])
        if path=='/maps/activation-context':return self.activation_context(req.get('name'))
        if path=='/maps/details':
            folder=self.path(req.get('name','active'))
            mapping=folder/'occ_grid_id_map.toml'
            body=mapping.read_text() if mapping.is_file() and mapping.stat().st_size<128000 else None
            return dict(ok=True,name=folder.name,grid_mapping=body,
                        files=[dict(name=p.relative_to(folder).as_posix(),size=p.stat().st_size)
                               for p in sorted(folder.rglob('*')) if p.is_file() and not p.is_symlink()][:1000])
        if path=='/maps/build/status':
            if self.demo:return dict(ok=True,demo=True,request_protocol=1,discard_protocol=1,mapping=dict(self.demo_mapping),stop_mapping=dict(self.demo_stop),**self.demo_build)
            start=read_job('mapping');stop=read_job('stop_mapping')
            if start.get('state') not in ['starting','running'] and stop.get('state') not in ['starting','running']:
                external=unowned_processes(start)
                if external:return dict(ok=True,request_protocol=1,discard_protocol=1,mapping=start,stop_mapping=stop,phase='external_collecting',in_progress=True,message='检测到平台记录之外的SLAM/后处理进程，请先核对当前任务',result_maps=[],external_processes=external)
            return dict(ok=True,request_protocol=1,discard_protocol=1,mapping=start,stop_mapping=stop,**mapping_phase(start,stop,self.library()['maps']))
        if path=='/maps/localize-auto':
            if self.demo:return dict(ok=True,demo=True,accepted=True,localized=True)
            return self.privileged('localize_auto',{})
        if path=='/maps/localization-reference':
            import numpy as np
            import hashlib,math
            from m20.map_fusion import read_pcd,voxel
            center=req.get('center',[])
            if len(center)!=3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in center):raise ValueError('参考位置无效')
            folder=self.path(req.get('name','active'))
            if folder!=self.path('active'):raise ValueError('活动地图已变化，请重新恢复定位')
            source=folder/'full_cloud.pcd';points=read_pcd(source)
            local=points[(np.linalg.norm(points[:,:2]-center[:2],axis=1)<8)&(points[:,2]>center[2]-1.5)&(points[:,2]<center[2]+3)]
            local=voxel(local,.08,60000)[:,:3].astype('<f4')
            if len(local)<200:raise ValueError('最近位置附近的地图结构不足')
            return dict(ok=True,protocol=1,frame_id='map',map_name=folder.name,count=len(local),points=base64.b64encode(local.tobytes()).decode(),cloud_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        if path=='/maps/help':
            if self.demo:return dict(ok=True,demo=True,output='演示环境不安装 drmap。实机连接后读取目标版本的帮助。\n-s：不打开 RViz\n-b：完成后不自动激活地图')
            return dict(ok=True,output='\n\n'.join('drmap '+args+'\n'+self.runner(['drmap']+args.split(),timeout=8)
                        for args in ['-h','mapping -h','stop_mapping -h','pack -h','unpack -h','apply -h']))
        if path=='/maps/grid':return self.grid(req.get('name','active'),req.get('grid_id',0))
        if path=='/maps/cloud':return preview_pcd(self.path(req.get('name','active'))/'full_cloud.pcd',req.get('limit',60000))
        if path=='/files/start':return self.files.start(req)
        if path=='/files/chunk':return self.files.chunk(req)
        if path=='/files/finish':return self.files.finish(req)
        if path=='/files/read':return self.files.read_chunk(req.get('id'),req.get('offset',0))
        if path=='/maps/package/inspect':return dict(ok=True,package=inspect_package(self.files.path(req.get('id'))))
        if path=='/maps/package/export':
            folder=self.path(req.get('name','active'))
            if not self.demo and folder==(self.maps/'active').resolve():
                with tempfile.TemporaryDirectory(prefix='drmap-export-',dir=str(self.data)) as staging:
                    self.runner(['drmap','pack','-d',staging,str(folder)],timeout=180)
                    generated=Path(staging)/(folder.name+'.zip')
                    if not generated.is_file() or generated.is_symlink():raise ValueError('drmap 未生成预期地图包：'+generated.name+'，请查看厂商打包日志')
                    file=self.files.register(generated)
                return dict(ok=True,file=file,format='drmap')
            out=self.data/('map-'+uuid.uuid4().hex+'.zip')
            try:
                pack_map(folder,out)
                return dict(ok=True,file=self.files.register(out,folder.name+'.zip'),format='directory-backup')
            finally:out.unlink(missing_ok=True)
        if path=='/maps/package/import':
            meta=self.files.get(req.get('id'))
            if meta['kind']!='map-package':raise ValueError('请选择地图包')
            name=map_name(req.get('name'))
            if self.demo:return unpack_package(self.files.path(meta['id']),self.maps,name)
            return self.privileged('import',dict(id=meta['id'],name=name,sha256=meta['sha256']))
        if path=='/maps/apply':
            folder=self.path(req.get('name'))
            context=self.activation_context(folder.name)
            if 'expected_active' in req and req['expected_active']!=context['active']:raise ValueError('活动地图已变化，未继续切换')
            if req.get('expected_project_sha256') and req['expected_project_sha256']!=context['project_sha256']:raise ValueError('地图版本已变化，请重新启用')
            review=folder/'.m20-alignment-review.json'
            if review.is_file() and json.loads(review.read_text()).get('status')=='rejected':raise ValueError('这份候选已确认存在墙线冲突，请保留作诊断，不用于导航；请选择原图或重新生成的版本')
            if self.demo:
                with catalog_lock(self.maps):
                    active=self.maps/'active'
                    if active.is_symlink():active.unlink()
                    elif active.exists():raise ValueError('active 必须为软链接')
                    active.symlink_to(folder.name,target_is_directory=True)
                return dict(ok=True,name=folder.name,delivery='simulated',activation_context=context)
            result=self.privileged('apply',dict(name=folder.name,expected_active=context['active'],expected_project_sha256=context['project_sha256']))
            return dict(result,activation_context=context)
        if path=='/maps/build':
            action=req.get('action')
            if action not in ['start','extend','stop','discard']:raise ValueError('建图操作无效')
            identity=request_identity(req.get('request_id'));expected=request_identity(req.get('expected_start_request_id'))
            if action=='discard' and expected is None:raise ValueError('放弃本轮需要明确的采集归属')
            if action in ['start','extend'] and identity is None:identity=uuid.uuid4().hex
            if self.demo:
                closing=action in ['stop','discard']
                if closing and expected is not None and self.demo_mapping.get('request_id')!=expected:raise ValueError('当前采集不属于本次请求，未发送停止命令')
                previous=self.demo_stop if closing else self.demo_mapping
                if identity is not None and previous.get('request_id')==identity:
                    if closing and (bool(previous.get('context',{}).get('discard'))!=(action=='discard') or previous.get('expected_start_request_id')!=expected):raise ValueError('重复请求的保存/放弃意图或采集归属不一致')
                    return dict(ok=True,delivery='already_launched',request_id=identity,message='本次请求已接收')
                if closing and not self.demo_build.get('in_progress') and previous.get('started',0)>self.demo_mapping.get('started',0):
                    if bool(previous.get('context',{}).get('discard'))!=(action=='discard'):raise ValueError('本轮已经结束，不能改写保存/放弃结果')
                    return dict(ok=True,delivery='already_finished',request_id=identity,message=self.demo_build.get('message'))
                record=dict(state='command_finished',started=time.time(),request_id=identity,name=req.get('name'),context=dict(workflow='extension',base=req.get('base'),anchor=req.get('anchor')) if action=='extend' else {})
                if closing:record['expected_start_request_id']=expected
                if action=='discard':record.update(context=dict(discard=True),discarded=True,expected_start_request_id=expected)
                if closing:self.demo_stop=record
                else:self.demo_mapping=record
                self.demo_build=dict(phase='collecting' if action in ['start','extend'] else 'discarded' if action=='discard' else 'saved',in_progress=action in ['start','extend'],
                    message='演示采集中 · 不连接真实雷达' if action in ['start','extend'] else '演示已放弃本轮 · 未生成候选，实机厂商可能保留临时观测' if action=='discard' else '演示保存已完成 · 示例地图可用于体验操作',
                    result_maps=[] if action in ['start','extend','discard'] else ['demo-yard'])
                if action=='discard':self.demo_build.update(discarded=True,candidate_created=False,observations_saved=False,temporary_files_retained=False,temporary_observations=[],activated=False)
                return dict(ok=True,delivery='simulated',request_id=identity,message=self.demo_build['message'])
            if action=='extend':return self.privileged('extend_mapping',dict(base=map_name(req.get('base')),anchor=req.get('anchor'),grid_id=req.get('grid_id',0),request_id=identity))
            if action=='discard':return self.privileged('discard_mapping',dict(request_id=identity,expected_start_request_id=expected))
            return self.privileged('mapping' if action=='start' else 'stop_mapping',dict(name=map_name(req.get('name','m20-map')),request_id=identity,expected_start_request_id=expected))
        raise ValueError('地图接口不存在')

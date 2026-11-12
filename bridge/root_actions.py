#!/usr/bin/env python3
"""Root-owned fixed map operations. Installed outside the writable data directory."""
import json
import os
import subprocess
import sys
import zipfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.map_formats import Files,map_name,inspect_package,digest
from m20.map_catalog import MapCatalog,catalog_lock,read_json
try:
    from .mapping_job import launch,request_identity
except ImportError:
    from mapping_job import launch,request_identity


def run(argv):
    p=subprocess.run(argv,capture_output=True,text=True,timeout=150)
    if p.returncode:raise ValueError((p.stderr or p.stdout or '系统操作失败')[-3000:])
    return p.stdout[-12000:]


def _execute(req,maps,data):
    action=req.get('action')
    if action in ['apply','mapping','extend_mapping','localize_auto']:
        from bridge.resident_mission import held_map_lease
        if held_map_lease(data):
            raise ValueError('GOS 导航正在使用活动地图，请先结束任务')
        resident=read_json(data/'resident-mission.json',{})
        if resident.get('job') and resident.get('phase') not in ['completed','cancelled','failed','idle']:
            raise ValueError('驻机导航任务或停车状态尚未确认，请先结束任务再切图、建图或重定位')
    if action in ['catalog-rename','catalog-delete','catalog-restore','catalog-purge','catalog-cleanup-previous']:
        def busy():
            try:from .mapping_job import read_job,mapping_phase,unowned_processes
            except ImportError:from mapping_job import read_job,mapping_phase,unowned_processes
            start=read_job('mapping');stop=read_job('stop_mapping')
            available=[dict(name=p.name,modified=p.stat().st_mtime,has_cloud=(p/'full_cloud.pcd').is_file(),grids=bool(list(p.glob('occ_grid*.pgm'))))
                       for p in maps.iterdir() if p.is_dir() and not p.is_symlink() and not p.name.startswith('.')]
            if mapping_phase(start,stop,available).get('in_progress') or unowned_processes(start):raise ValueError('机器人正在采集或处理地图，请完成后再删除')
            refinement=read_json(data/'refinement/state.json',{})
            if refinement.get('enabled') or refinement.get('state')=='generating':raise ValueError('请先结束行走地图更新，再编辑地图')
        return MapCatalog(maps,busy).execute(action[len('catalog-'):],req)
    if action=='localize_auto':
        client=Path('/opt/robot/share/localization/bin/test_reloc_client')
        if not client.is_file():raise ValueError('当前设备没有已适配的厂商全局重定位客户端')
        output=run([str(client),'0','1'])
        if '请求已接受' not in output:raise ValueError('厂商未确认接受重定位请求：'+output[-1000:])
        return dict(ok=True,accepted=True,localized=False,message='全局重定位搜索已开始，等待定位状态收敛')
    if action=='refine_map':
        import re
        session=req.get('session','')
        if not re.fullmatch('[a-f0-9]{32}',session):raise ValueError('采集会话编号无效')
        folder=(data/'refinement/sessions'/session).resolve()
        if folder.parent!=(data/'refinement/sessions').resolve():raise ValueError('采集路径无效')
        info=json.loads((folder/'session.json').read_text());base=(maps/map_name(info['base'])).resolve()
        if base.parent!=maps.resolve():raise ValueError('原地图路径无效')
        from m20.map_refinement import refine_map
        return refine_map(base,folder,maps/map_name(req.get('name')),int(info['grid_id']),info['fingerprint'])
    if action=='mapping_capabilities':
        import shutil
        try:
            import numpy
            numpy_ready=True
        except Exception:numpy_ready=False
        pcl=shutil.which('pcl_icp');ready=bool(numpy_ready and pcl)
        return dict(ok=True,pipeline_version=2,ready=ready,numpy=numpy_ready,pcl=bool(pcl),message='续建组件已就绪' if ready else 'NOS 需要 NumPy 与 PCL 配准工具')
    if action=='service':
        role=req.get('role');operation=req.get('operation')
        if role not in ['nos','gos','sdk','train'] or operation not in ['start','stop','restart']:
            raise ValueError('服务操作不在允许列表')
        return dict(ok=True,output=run(['systemctl',operation,'m20-platform-'+role+'.service']))
    if action=='import':
        files=Files(data/'files')
        meta=files.get(req.get('id'));p=files.path(meta['id'])
        name=map_name(req.get('name'))
        # Copy to root-owned staging before validating/extracting: the unprivileged
        # uploader cannot replace bytes after the hash check.
        import tempfile,shutil
        with tempfile.TemporaryDirectory(prefix='.m20-package-',dir=str(maps)) as stage:
            target=Path(stage)/(name+'.zip')
            with p.open('rb') as source,target.open('xb') as out:shutil.copyfileobj(source,out,1024**2)
            if digest(target)!=req.get('sha256'):raise ValueError('地图包在导入前发生变化')
            info=inspect_package(target)
            if info['prefix']!=name+'/':raise ValueError('实机导入需保留包内原地图名称与目录结构，请使用：'+str(info.get('map_name')))
            with zipfile.ZipFile(target) as archive:
                if any(not entry.is_dir() and not entry.filename.startswith(info['prefix']) for entry in archive.infolist()):
                    raise ValueError('地图包包含目标地图之外的文件，已拒绝导入，避免覆盖其他地图')
            if (maps/name).exists() or (maps/name).is_symlink():raise ValueError('目标地图已存在，已拒绝覆盖')
            output=run(['drmap','unpack','-n',str(target)])
            if not (maps/name).is_dir():raise ValueError('drmap 已结束，但未发现预期地图目录，请查看日志核对结果')
            return dict(ok=True,name=name,output=output,activated=False)
    if action=='apply':
        import hashlib,time
        name=map_name(req.get('name'));p=(maps/name).resolve()
        if p.parent!=maps.resolve() or not p.is_dir():raise ValueError('地图不存在')
        active=(maps/'active').resolve();previous=active.name if active.is_dir() and active.parent==maps.resolve() else None
        if 'expected_active' in req and req['expected_active']!=previous:raise ValueError('活动地图已变化，未继续切换')
        meta=p/'.m20-project.json';raw=meta.read_bytes() if meta.is_file() else b''
        if req.get('expected_project_sha256') and hashlib.sha256(raw).hexdigest()!=req['expected_project_sha256']:raise ValueError('地图版本已变化，未继续切换')
        output=run(['drmap','apply',str(p)])+run(['systemctl','restart','localization.service'])
        return dict(ok=True,name=name,output=output,localization_restart_completed_at=time.time(),
                    message='地图已切换，正在恢复定位')
    if action=='extend_mapping':
        import math,uuid
        base=map_name(req.get('base'));folder=(maps/base).resolve()
        if folder.parent!=maps.resolve() or not (folder/'full_cloud.pcd').is_file():raise ValueError('续建原图不存在')
        anchor=req.get('anchor',{})
        if set(anchor)!={'x','y','z','yaw'} or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in anchor.values()):raise ValueError('续建需要有效初始定位')
        if (maps/'active').resolve()!=folder:raise ValueError('先将要续建的地图设为活动地图并完成定位')
        grid=req.get('grid_id',0)
        if not isinstance(grid,int) or isinstance(grid,bool) or not 0<=grid<=999:raise ValueError('栅格编号无效')
        from m20.map_fusion import read_grid
        read_grid(folder,grid)
        identity=req.get('request_id')
        request_identity(identity)
        return launch('mapping','m20-session-'+(identity or uuid.uuid4().hex[:10]),context=dict(workflow='extension',base=base,anchor=anchor,grid_id=grid,maps=str(maps)),request_id=identity)
    if action=='mapping':
        return launch('mapping',map_name(req.get('name')),request_id=req.get('request_id'))
    if action in ['stop_mapping','discard_mapping']:
        context=None
        if action=='discard_mapping':
            try:from .mapping_job import read_job
            except ImportError:from mapping_job import read_job
            expected=request_identity(req.get('expected_start_request_id'))
            if expected is None:raise ValueError('放弃本轮需要明确的采集归属')
            capture=read_job('mapping')
            if capture.get('request_id')!=expected:raise ValueError('采集已变化，未放弃其他会话')
            context=dict(capture.get('context',{}),discard=True,maps=str(maps))
        return launch('stop_mapping',context=context,request_id=req.get('request_id'),expected_start_request_id=req.get('expected_start_request_id'))
    raise ValueError('不允许此系统操作')


def execute(req,maps=Path('/var/opt/robot/data/maps'),data=Path('/var/lib/m20-platform')):
    # Serialize activation/producer starts with catalog edits across helper processes.
    if req.get('action') in ['apply','mapping','extend_mapping','refine_map','import','localize_auto']:
        with catalog_lock(maps):return _execute(req,maps,data)
    return _execute(req,maps,data)


if __name__=='__main__':
    try:
        raw=sys.stdin.read(8193)
        if len(raw)>8192:raise ValueError('请求过大')
        print(json.dumps(execute(json.loads(raw)),ensure_ascii=False))
    except Exception as exc:
        print(json.dumps(dict(ok=False,err=str(exc)),ensure_ascii=False))
        sys.exit(1)

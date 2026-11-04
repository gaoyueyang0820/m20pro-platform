"""Persistent drmap jobs; command progress and vendor logs remain distinguishable."""
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_fusion import FusionDecisionError


LOGS = Path('/var/log/m20-platform')


def completion_metadata(record):
    """Additive fields; old callers retain phase/state/result_maps semantics."""
    keys=['save_status','fusion_status','observations_saved','candidate_created',
          'observation_session','requirements','next_capture','discarded','temporary_files_retained','temporary_observations','observation_ownership']
    return {key:record[key] for key in keys if key in record}


def completion_message(record, fallback):
    return record.get('message',fallback) if record.get('fusion_status') in ['awaiting_evidence','discarded'] else fallback


def slam_processes(proc=Path('/proc')):
    """Identify SLAM children, excluding the platform's own command wrapper."""
    result={}
    for path in proc.glob('[0-9]*'):
        try:
            argv=[s.decode(errors='replace') for s in (path/'cmdline').read_bytes().split(b'\0') if s]
            if not argv:continue
            target=argv[0]
            if Path(target).name.startswith(('python','bash','sh')):
                if len(argv)<2 or argv[1].startswith('-'):continue
                target=argv[1]
            if Path(target).name=='mapping_job.py' or not ('/slam/' in target or Path(target).name in ['fastlio_mapping','lio_mapping','laserMapping','lio_sam_mapping','mapping_node']):continue
            command=' '.join(argv)
            start=(path/'stat').read_text().rsplit(')',1)[1].split()[19]
            result[int(path.name)]=dict(pid=int(path.name),start=start,command=command[:500])
        except (OSError,ValueError,IndexError):continue
    return result


def empty_capture_confirmed(start,stop):
    """A missing field or an elapsed PC timeout is not an empty-map receipt."""
    if start.get('context',{}).get('workflow')=='extension' or stop.get('context',{}).get('discard'):return False
    expected=stop.get('expected_start_request_id')
    if expected is not None and expected!=start.get('request_id'):return False
    return (stop.get('state')=='command_finished' and stop.get('ended',0)>0
            and stop.get('started',0)>start.get('started',0)>0
            and 'processes_alive' in start and start['processes_alive']==[]
            and not stop.get('processes_alive')
            and re.search(r'\bNo poses loaded\b',stop.get('output',''),re.I) is not None)


def mapping_phase(start,stop,maps):
    since=start.get('started',0)
    if start.get('state')=='none' or not since:
        if stop.get('state') in ['starting','running']:return dict(phase='saving',in_progress=True,message='正在停止保存，尚无平台采集起始记录',result_maps=[])
        if stop.get('state')=='command_finished':return dict(phase='unconfirmed',in_progress=False,message='停止命令已结束，但没有本轮采集记录；请核对地图库中的实际结果',result_maps=[])
        return dict(phase='idle',in_progress=False,message='尚未开始建图',result_maps=[])
    newer_stop=stop.get('started',0)>since and stop.get('state') not in ['none']
    if newer_stop and stop.get('context',{}).get('discard') is True:
        metadata=completion_metadata(stop)
        if stop.get('state')=='failed':return dict(phase='error',in_progress=bool(start.get('processes_alive')),message=stop.get('message','放弃本轮尚未完成'),result_maps=[],**metadata)
        if stop.get('state')=='command_finished' and not start.get('processes_alive'):
            return dict(phase='discarded',in_progress=False,message=stop.get('message','本轮已放弃，未发布候选'),result_maps=[],activated=False,**metadata)
        return dict(phase='discarding',in_progress=True,message='正在停止采集并放弃本轮更新；厂商可能仍生成临时观测文件',result_maps=[],**metadata)
    candidates=[m['name'] for m in maps if m.get('modified',0)>=since and m.get('has_cloud') and m.get('grids') and (not start.get('name') or m['name'].startswith(start['name']+'-'))]
    if newer_stop and start.get('context',{}).get('workflow')=='extension':
        if stop.get('state')=='failed':return dict(phase='error',in_progress=bool(start.get('processes_alive')),message=stop.get('message','融合失败，原图与本轮扫描均已保留'),result_maps=stop.get('result_maps',[]),fusion=stop.get('fusion'),activated=False,**completion_metadata(stop))
        if stop.get('state')=='command_finished' and stop.get('fusion_status')=='awaiting_evidence' and stop.get('observations_saved'):
            return dict(phase='saved',in_progress=False,message=stop['message'],result_maps=[],
                        fusion=stop.get('fusion'),activated=False,**completion_metadata(stop))
        if stop.get('state')=='command_finished' and stop.get('result_maps'):
            return dict(phase='saved',in_progress=False,message='旧图与本轮扫描已融合，候选版本已保存，等待你点击启用',result_maps=stop['result_maps'],fusion=stop.get('fusion'),revision=stop.get('revision'),activated=False,**completion_metadata(stop))
        return dict(phase='saving',in_progress=True,message=stop.get('message','正在完成回环优化并融合新旧地图'),result_maps=[])
    if newer_stop:
        if stop.get('state')=='failed':return dict(phase='error',in_progress=True,message='停止或保存失败，请查看日志后重试停止保存',result_maps=[])
        if stop.get('state')=='command_finished' and candidates:
            return dict(phase='saved',in_progress=False,message='采集已停止，地图文件已生成',result_maps=candidates)
        if empty_capture_confirmed(start,stop):
            return dict(phase='empty',in_progress=False,
                        message='采集已停止，厂商报告没有有效位姿，未生成地图；原始目录已保留，请核对定位和雷达后重新建图',result_maps=[])
        return dict(phase='saving',in_progress=True,message='正在停止采集、生成栅格与点云，请等待保存完成',result_maps=[])
    if start.get('state')=='none' or not since:return dict(phase='idle',in_progress=False,message='尚未开始建图',result_maps=[])
    if start.get('state')=='failed':return dict(phase='error',in_progress=bool(start.get('processes_alive')),message='建图启动失败，请查看日志',result_maps=[])
    if start.get('processes_alive') or start.get('state')=='running' and 'Building map' in start.get('output',''):
        return dict(phase='collecting',in_progress=True,message='正在采集，请低速移动覆盖房间；结束后点击停止并保存',result_maps=[])
    if start.get('state') in ['starting','running']:
        return dict(phase='starting',in_progress=True,message='正在启动建图，请等待雷达数据',result_maps=[])
    if 'processes_alive' in start and not start['processes_alive']:
        return dict(phase='interrupted',in_progress=False,message='上次采集进程已结束，尚无完整地图；原始扫描保留，请勿覆盖该目录',result_maps=[])
    return dict(phase='unconfirmed',in_progress=True,message='启动命令已结束，采集状态待核对；请停止并保存后确认结果',result_maps=[])


def read_job(kind, logs=LOGS):
    path = Path(logs)/(kind+'.json')
    try:
        result = json.loads(path.read_text())
    except (OSError, ValueError):
        return dict(state='none', message='尚无平台建图记录')
    logfile = Path(logs)/(kind+'.log')
    try:
        with logfile.open('rb') as f:
            f.seek(max(0, logfile.stat().st_size-16000))
            result['output'] = f.read().decode('utf-8', errors='replace')
    except OSError:
        result['output'] = ''
    if result.get('state') == 'running':
        try:
            cmdline = Path('/proc/%s/cmdline'%result['pid']).read_bytes()
            alive = b'mapping_job.py' in cmdline
        except OSError:
            alive = False
        if not alive:
            result.update(state='unknown', message='后台进程已结束，尚无结束记录，请核对地图与日志')
        elif kind == 'mapping' and 'Building map' in result['output']:
            result['message'] = '厂商日志已报告 Building map，正在采集地图'
    if result.get('state')=='starting' and time.time()-result.get('started',0)>10:
        result.update(state='unknown',message='后台启动尚未确认，请核对服务日志')
    if kind=='mapping' and result.get('mapping_processes'):
        current=slam_processes()
        result['processes_alive']=[p['pid'] for p in result['mapping_processes'] if current.get(p['pid'],{}).get('start')==p['start']]
    return result


def request_identity(value):
    if value is not None and (not isinstance(value,str) or not re.fullmatch(r'[0-9a-f]{32}',value)):
        raise ValueError('建图请求编号无效')
    return value


def write_receipt(logs,kind,state):
    identity=state.get('request_id')
    if identity is None:return
    request_identity(identity)
    folder=Path(logs)/'requests';folder.mkdir(mode=0o755,exist_ok=True)
    if folder.is_symlink():raise ValueError('请求记录目录异常')
    path=folder/(kind+'-'+identity+'.json');temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(state,ensure_ascii=False));temp.chmod(0o644);temp.replace(path)


def unowned_processes(capture):
    owned=set(capture.get('processes_alive',[]))
    return [p for pid,p in slam_processes().items() if pid not in owned]


def launch(kind, name=None, logs=LOGS, context=None, request_id=None, expected_start_request_id=None):
    import fcntl  # Root launcher runs on Linux; local preview also supports Windows.
    if kind not in ['mapping', 'stop_mapping']:
        raise ValueError('建图操作无效')
    request_identity(request_id);request_identity(expected_start_request_id)
    def check_intent(record):
        if kind=='stop_mapping' and (bool(record.get('context',{}).get('discard'))!=bool((context or {}).get('discard'))
                or record.get('expected_start_request_id')!=expected_start_request_id):
            raise ValueError('重复停止请求的保存/放弃意图或采集归属不一致')
    logs = Path(logs)
    logs.mkdir(mode=0o755, parents=True, exist_ok=True)
    lockfile=logs/'.launch.lock'
    if lockfile.is_symlink():raise ValueError('建图锁路径异常')
    with lockfile.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        previous = read_job(kind, logs)
        capture=read_job('mapping',logs) if kind=='stop_mapping' else previous
        if expected_start_request_id is not None and capture.get('request_id')!=expected_start_request_id:
            raise ValueError('当前采集不属于本次请求，未发送停止命令')
        if expected_start_request_id is not None and capture.get('state') not in ['starting','running'] and unowned_processes(capture):
            raise ValueError('存在未确认归属的外部建图进程，未发送自动停止')
        if request_id is not None:
            receipt=logs/'requests'/(kind+'-'+request_id+'.json')
            try:history=json.loads(receipt.read_text())
            except (OSError,ValueError):history=None
            if history:
                check_intent(history)
                if kind=='mapping' and (history.get('name')!=name or history.get('context',{})!=(context or {})):raise ValueError('重复请求的建图参数不一致')
                if history.get('state')=='failed':raise ValueError(history.get('message','该请求已失败'))
                return dict(ok=True,request_id=request_id,delivery='already_launched',state=history.get('state'),result_maps=history.get('result_maps',[]),message=completion_message(history,'已从请求记录找到原操作，未重复启动'),**completion_metadata(history))
        if request_id is not None and previous.get('request_id')==request_id:
            check_intent(previous)
            if kind=='mapping' and (previous.get('name')!=name or previous.get('context',{})!=(context or {})):
                raise ValueError('重复请求的建图参数不一致')
            if previous.get('state')=='failed':raise ValueError(previous.get('message','本次请求已失败，请核对状态'))
            return dict(ok=True,request_id=request_id,delivery='already_launched',state=previous.get('state'),result_maps=previous.get('result_maps',[]),message=completion_message(previous,'本次请求已接收，未重复启动'),**completion_metadata(previous))
        if kind=='mapping' and read_job('stop_mapping',logs).get('state') in ['starting','running']:
            raise ValueError('上一轮地图仍在停止或保存，请等待完成')
        if kind=='stop_mapping' and previous.get('state')=='command_finished':
            if previous.get('started',0)>capture.get('started',0) and not capture.get('processes_alive'):
                if bool(previous.get('context',{}).get('discard'))!=bool((context or {}).get('discard')):
                    raise ValueError('本轮已经结束，不能将已保存和已放弃的结果相互改写')
                return dict(ok=True,request_id=request_id,delivery='already_saved',result_maps=previous.get('result_maps',[]),message=completion_message(previous,'本轮地图已经保存'),**completion_metadata(previous))
        if previous['state'] in ['running','starting'] or kind=='mapping' and previous.get('processes_alive'):
            raise ValueError('同一建图操作仍在运行，请查看日志与状态')
        if kind=='mapping' and slam_processes():raise ValueError('检测到已有SLAM/后处理进程，请先核对外部任务，不重复启动')
        # Reserve before forking, so a double click cannot start two drmap jobs.
        journal=logs/(kind+'.json')
        if journal.is_symlink():raise ValueError('建图状态路径异常')
        reservation=dict(state='starting',started=time.time(),name=name,message='正在启动后台进程',context=context or {},request_id=request_id,expected_start_request_id=expected_start_request_id)
        journal.write_text(json.dumps(reservation))
        journal.chmod(0o644)
        write_receipt(logs,kind,reservation)
        argv = [sys.executable, str(Path(__file__).resolve()), kind, str(logs)]
        argv.extend([name or '',json.dumps(context or {}),request_id or '',expected_start_request_id or ''])
        try:
            proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    start_new_session=True)
        except Exception:
            failed=dict(reservation,state='failed',message='后台进程启动失败');journal.write_text(json.dumps(failed));write_receipt(logs,kind,failed)
            raise
    # The child owns and updates its journal. Do not call this a completed map.
    return dict(ok=True, pid=proc.pid, delivery='launched',request_id=request_id,
                message='已提交建图命令，请查看采集/后处理状态与厂商日志')


def worker(kind, logs, name=None, context=None, request_id=None, expected_start_request_id=None):
    request_identity(request_id);request_identity(expected_start_request_id)
    logs = Path(logs)
    path = logs/(kind+'.json')
    logfile = logs/(kind+'.log')
    if path.is_symlink() or logfile.is_symlink():
        raise ValueError('建图日志路径异常')
    previous=read_job('mapping',logs) if kind=='stop_mapping' else {}
    context=context or previous.get('context',{})
    state = dict(context=context,state='running', pid=os.getpid(), started=time.time(), name=name,request_id=request_id,expected_start_request_id=expected_start_request_id,
                 message='正在启动建图' if kind == 'mapping' else '正在停止采集并等待厂商后处理')
    def persist():
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(state, ensure_ascii=False))
        temp.chmod(0o644)
        temp.replace(path)
        write_receipt(logs,kind,state)
    persist()
    command = ['drmap', 'mapping', '-s', '-b', '-n', name] if kind == 'mapping' else ['drmap', 'stop_mapping']
    before=slam_processes() if kind=='mapping' else {}
    try:
        if kind=='mapping' and before:raise ValueError('已有外部SLAM/后处理进程，未启动第二份采集')
        if kind=='stop_mapping':
            deadline=time.monotonic()+60
            while True:
                capture=read_job('mapping',logs)
                if expected_start_request_id is not None and capture.get('request_id')!=expected_start_request_id:
                    raise ValueError('采集请求已变化，未停止其他会话')
                if capture.get('state') not in ['starting','running']:
                    if expected_start_request_id is not None and unowned_processes(capture):raise ValueError('存在未确认归属的外部建图进程，未停止其他会话')
                    break
                if time.monotonic()>=deadline:raise ValueError('建图启动尚未结束，停止结果待核对；请稍后重试停止保存')
                state['message']='等待本次建图启动结束后停止保存';persist();time.sleep(.2)
        with logfile.open('w') as log:
            logfile.chmod(0o644)
            result = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode==0 and kind=='stop_mapping' and context.get('discard') is True:
            maps=Path(context['maps']);temporary=[];active=(maps/'active').resolve()
            # drmap stop_mapping always performs vendor post-processing. Keep
            # its own files as hidden observations; do not fuse or activate.
            candidates=[folder for folder in maps.glob((previous.get('name') or '__no_capture__')+'-*')
                        if not folder.is_symlink() and folder.is_dir() and folder.stat().st_mtime>=previous.get('started',0)]
            unique=len(candidates)==1 and candidates[0].resolve()!=active
            if unique:
                marker=candidates[0]/'.m20-session-source.json'
                if marker.is_symlink():unique=False
                elif marker.exists():
                    try:
                        existing=json.loads(marker.read_text())
                        unique=(existing.get('capture_id') in [None,expected_start_request_id]
                                and all(existing.get(key)==previous.get('context',{}).get(key) for key in ['workflow','base','anchor','grid_id'] if key in existing))
                    except (OSError,ValueError):unique=False
            for folder in candidates:
                temporary.append(dict(name=folder.name,path=str(folder),has_cloud=(folder/'full_cloud.pcd').is_file(),ownership_verified=unique))
            for folder in candidates if unique else []:
                marker=folder/'.m20-session-source.json'
                if marker.is_symlink():raise ValueError('临时观测标记路径异常')
                marker.write_text(json.dumps(dict(context,discarded=True,capture_id=expected_start_request_id),ensure_ascii=False))
            state.update(discarded=True,save_status='discarded',fusion_status='discarded',candidate_created=False,
                         observations_saved=any(p['has_cloud'] for p in temporary) if unique else None,temporary_files_retained=True if temporary else None,
                         observation_ownership='unique' if unique else 'unconfirmed',
                         temporary_observations=temporary,result_maps=[],activated=False,
                         message=('本轮已停止并放弃，未生成融合候选、未切换活动地图；厂商临时观测保留，可另行清理' if unique else
                                  '本轮已停止并放弃，未生成融合候选；临时文件归属未能唯一确认，未隐藏或删除任何目录，原活动图不改动'))
        elif result.returncode==0 and context.get('workflow')=='extension':
            maps=Path(context['maps'])
            if kind=='mapping':
                for folder in maps.glob(name+'-*'):
                    if folder.is_dir():(folder/'.m20-session-source.json').write_text(json.dumps(context))
            else:
                from map_fusion import fuse_maps
                from datetime import datetime,timezone,timedelta
                from map_formats import map_name
                base=maps/map_name(context['base'])
                candidates=[p for p in maps.glob(previous['name']+'-*') if p.is_dir() and (p/'full_cloud.pcd').is_file() and p.stat().st_mtime>=previous.get('started',0)]
                if len(candidates)!=1:raise ValueError('未找到本轮完整扫描，原图保持不变，请查看采集日志')
                state.update(vendor_save_succeeded=True,save_status='vendor_save_finished',
                    observation_session=dict(name=candidates[0].name,path=str(candidates[0]),native_session='session_0'),
                    message='厂商已结束采集保存，正在核对本轮扫描并与原图配准');persist()
                project=json.loads((base/'.m20-project.json').read_text()) if (base/'.m20-project.json').exists() else dict(root_map=base.name,revision=1)
                stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d-%H%M%S')
                output=maps/(project['root_map'][:45]+'-r'+str(project['revision']+1)+'-'+stamp)
                fused=fuse_maps(base,candidates[0],output,context['anchor'],context.get('grid_id',0))
                state.update(result_maps=[output.name],fusion=fused['report'],revision=fused['revision'],
                             save_status='candidate_saved',fusion_status='candidate_created',observations_saved=True,candidate_created=True)
                # Candidates are reviewed and activated explicitly in the map UI.
                state['activated']=False
        state.update(state='command_finished' if result.returncode == 0 else 'failed',
                     returncode=result.returncode,
                     message=(state.get('message') if state.get('discarded') else '厂商命令已结束，请核对新地图与后处理日志') if result.returncode == 0 else '厂商命令失败，请查看日志')
        if kind=='mapping':state['mapping_processes']=[p for pid,p in slam_processes().items() if pid not in before or p['start']!=before[pid]['start']]
    except FusionDecisionError as exc:
        report=exc.report;capture=report.get('observation_session') or {}
        saved=bool(state.get('vendor_save_succeeded') and capture.get('saved_keyframes',0)>0)
        deferred=exc.evidence_deferred and saved
        state.update(state='command_finished' if deferred else 'failed',fusion=report,
                     fusion_status='awaiting_evidence' if deferred else 'alignment_failed',
                     observations_saved=saved,candidate_created=False,result_maps=[],activated=False)
        for key in ['observation_session','requirements','next_capture']:
            if key in report:state[key]=report[key]
        if saved:state['save_status']='observations_saved'
        if deferred:
            count=capture['saved_keyframes'];required=report['requirements']['min_saved_keyframes']
            state.update(returncode=0,message='观测已保存（本轮 %d 个关键帧，确认变化至少需要 %d 个）；环境变化证据待补充，未生成融合候选，原图未改。%s'%(count,required,report['decision']['reason']))
        else:
            prefix='原始观测已保存，但结构配准未通过，原图未改。' if saved else ''
            state['message']=prefix+str(exc)
    except Exception as exc:
        state.update(state='failed', message=str(exc))
        if state.get('vendor_save_succeeded'):
            state.update(fusion_status='processing_failed',candidate_created=False,result_maps=[],activated=False,
                         message='厂商保存命令已完成，但融合处理失败；原始扫描路径已保留，请核对：'+str(exc))
    finally:
        state['ended'] = time.time()
        persist()


if __name__ == '__main__':
    worker(sys.argv[1], sys.argv[2], (sys.argv[3] or None) if len(sys.argv)>3 else None,json.loads(sys.argv[4]) if len(sys.argv)>4 else None,(sys.argv[5] or None) if len(sys.argv)>5 else None,(sys.argv[6] or None) if len(sys.argv)>6 else None)

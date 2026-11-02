"""Automatic relocation coordination; never emits a locomotion command."""
import base64,json,math,time,threading
from pathlib import Path


class OperationCancelled(ValueError):
    """A superseded operation must never fall through to another action."""


class AutomaticLocalizer:
    def __init__(self,app):
        self.app=app;self.file=app.store.path.parent/'localization-hint.json';self.last_poll=0
        self.generation=0
        self._hint_lock=threading.Lock()
        try:self.hint=json.loads(self.file.read_text())
        except (OSError,ValueError):self.hint=None

    def cancel(self):self.generation+=1

    def status(self):return self.app.proxy('nos','/sensors/status',timeout=3)

    def good(self,status):
        loc=status.get('localization_quality') or {};q=status.get('matching_quality') or {}
        record=self.app.link.record(1007,2) if self.app.link else {}
        return (loc.get('status')==loc.get('normal_status',1) and loc.get('age',99)<1.5
                and q.get('age',99)<1.5 and q.get('error',99)<.15 and q.get('inlier_ratio',0)>.90
                and time.time()-record.get('at',0)<2 and record.get('items',{}).get('Location')==0)

    def remember(self,status):
        if not self.good(status) or self.app.mapping:return False
        link=self.app.link;pose=link.record(1007,2)['items']
        name=self.app.hub.remote_maps('/maps/list',timeout=3).get('active')
        if not name or link is not self.app.link or self.app.mapping:return False
        hint=dict(schema_version=1,robot=link.ip,map_name=name,pose={k:pose[k] for k in ['PosX','PosY','PosZ','Yaw']},saved_at=time.time())
        with self._hint_lock:
            temp=self.file.with_suffix('.tmp');temp.write_text(json.dumps(hint,ensure_ascii=False,indent=2));temp.replace(self.file);self.hint=hint
        return True

    def poll(self):
        a=self.app
        if a.demo or not a.link or a.mapping or a.hub.busy or time.monotonic()-self.last_poll<5:return
        self.last_poll=time.monotonic()
        record=a.link.record(1007,2)
        if time.time()-record.get('at',0)<2 and record.get('items',{}).get('Location')==0:self.remember(self.status())

    def wait_normal(self,seconds,expected=None,check=None):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if check:check()
            try:status=self.status()
            except OperationCancelled:raise
            except ValueError:
                if check:check()
                time.sleep(.3);continue
            if check:check()
            if self.good(status):
                p=self.app.link.record(1007,2)['items']
                close=expected is None or math.hypot(p['PosX']-expected['PosX'],p['PosY']-expected['PosY'])<.35 and abs(math.atan2(math.sin(p['Yaw']-expected['Yaw']),math.cos(p['Yaw']-expected['Yaw'])))<.25
                if close:self.remember(status);return status
            time.sleep(.3)
        raise ValueError('定位尚未恢复正常')

    def body_cloud(self,after=None,check=None):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if check:check()
            try:
                began=time.monotonic();p=self.app.proxy('nos','/sensors/cloud',dict(topic='/LOC_BODY_POINTS'),timeout=2)
                if check:check()
                stamp=p['stamp']['sec']+p['stamp']['nanosec']/1e9
                if p['frame_id']=='base_link' and p.get('age',99)+time.monotonic()-began<.75 and (after is None or stamp>after):return p,stamp
            except OperationCancelled:raise
            except (ValueError,KeyError):pass
            time.sleep(.1)
        raise ValueError('等待新鲜机身点云')

    def _stationary(self,link):
        with self.app.lock:
            if link is not self.app.link:raise ValueError('机器人连接已变化，停止恢复定位')
            self.app.hub.idle();self.app.live_ready()
            if self.app.mapping:raise ValueError('建图状态已变化，停止恢复定位')
            motion=link.record(1002,4);values=motion.get('items',{}).get('MotionStatus',{})
            if time.time()-motion.get('at',0)>.5 or any(abs(values.get(k,999))>limit for k,limit in [('LinearX',.08),('LinearY',.08),('OmegaZ',.12)]):
                raise ValueError('请停稳后再恢复定位')

    def saved_slam_hint(self,progress,status=None,check=None):
        """Recover from this completed mapping session, not an old pose cache."""
        import numpy as np
        from localization_recovery import saved_slam_pose,verify_transform,match_pose,verify_pose
        def hypothesis_for(build,sensors,active):
            context=(build.get('mapping') or {}).get('context') or {}
            fusion=(build.get('stop_mapping') or {}).get('fusion') or {}
            project=None
            if active!=context.get('base') and active==fusion.get('map'):
                manifest=self.app.hub.remote_maps('/maps/project',dict(name=active),timeout=5)
                if check:check()
                if manifest.get('active') is not True:raise ValueError('活动地图已变化，停止恢复定位')
                project=manifest.get('project')
            return saved_slam_pose(build,sensors,active,project=project)
        if check:check()
        link=self.app.link
        if not link:raise ValueError('机器人未连接')
        self._stationary(link)
        library=self.app.hub.remote_maps('/maps/list',timeout=3)
        build=self.app.hub.remote_maps('/maps/build/status',timeout=4)
        if check:check()
        hypothesis=hypothesis_for(build,status or self.status(),library.get('active'))
        p=hypothesis['pose'];name=hypothesis['reference_map']
        progress(.15,'正在用本轮已保存轨迹核对当前位置')
        ref=self.app.hub.remote_maps('/maps/localization-reference',dict(name=name,center=[p['PosX'],p['PosY'],p['PosZ']]),timeout=8)
        if ref.get('map_name')!=name or ref.get('frame_id')!='map':raise ValueError('定位参考地图不一致')
        points=np.frombuffer(base64.b64decode(ref['points'],validate=True),dtype='<f4').reshape(-1,3)
        first,stamp=self.body_cloud(after=hypothesis['source_stamp'],check=check)
        method='saved_slam_full_transform';match=None;fixed_error=None
        try:
            first_check=verify_transform(hypothesis['transform'],np.asarray(first['points']),points,
                stamp=stamp,after_stamp=hypothesis['source_stamp'],frame_id=first['frame_id'])
        except ValueError as exc:
            # Manual turning/translation after saving invalidates the exact
            # ending pose, not the session's independently registered prior.
            # Fit this fresh frame, then require a separate later observation.
            if check:check()
            self._stationary(link)
            fixed_error=str(exc)
            progress(.35,'保存后位置或朝向已变化，正在附近重新识别当前位置')
            match=match_pose(np.asarray(first['points']),points,p,stamp=stamp,frame_id=first['frame_id'])
            first_check=dict(verified=False,message=fixed_error)
            method='saved_session_local_match'
        if check:check()
        second,next_stamp=self.body_cloud(after=stamp,check=check)
        if match is None:
            verification=verify_transform(hypothesis['transform'],np.asarray(second['points']),points,
                stamp=next_stamp,after_stamp=stamp,frame_id=second['frame_id'])
            pose=verification['initialization_pose']
        else:
            verification=verify_pose(match,np.asarray(second['points']),points,
                stamp=next_stamp,frame_id=second['frame_id'])
            pose=match['pose']
        if check:check()
        current=self.app.hub.remote_maps('/maps/list',timeout=3)
        latest=self.app.hub.remote_maps('/maps/build/status',timeout=4)
        current_hint=hypothesis_for(latest,self.status(),current.get('active'))
        with self.app.lock:
            if check:check()
            self._stationary(link)
            if (current_hint['reference_map']!=hypothesis['reference_map'] or
                    current_hint['session_key']!=hypothesis['session_key'] or
                    current_hint['source_stamp']!=hypothesis['source_stamp'] or
                    not np.allclose(current_hint['transform'],hypothesis['transform'],atol=1e-8)):
                raise ValueError('保存轨迹已变化，停止恢复定位')
            # Validation retains roll/pitch. The vendor initialization protocol
            # only accepts xyz/yaw; its running estimator supplies IMU attitude.
            link.command(2101,1,pose,'由本轮保存轨迹与新鲜结构恢复定位')
        progress(.8,'新鲜结构已确认当前位置，等待定位收敛')
        self.wait_normal(12,pose,check=check)
        return dict(ok=True,localized=True,method=method,pose=pose,
                    source_map=hypothesis['source_map'],session_key=hypothesis['session_key'],
                    reference_map=name,original_base_map=hypothesis['original_base_map'],
                    match=match,fixed_pose_error=fixed_error,
                    first_verification=first_check,verification=verification,
                    message='已根据本轮保存轨迹自动恢复定位')

    def local_hint(self,progress,check=None):
        import numpy as np
        from localization_recovery import match_pose,verify_pose
        if check:check()
        link=self.app.link;hint=self.hint
        if not link or not hint or hint.get('robot')!=link.ip:raise ValueError('尚无这台机器的可靠位置记录')
        library=self.app.hub.remote_maps('/maps/list',timeout=3)
        if library.get('active')!=hint['map_name']:raise ValueError('当前地图与最近位置记录不同')
        p=hint['pose'];progress(.15,'正在最近可靠位置附近识别当前位置与朝向')
        ref=self.app.hub.remote_maps('/maps/localization-reference',dict(name=hint['map_name'],center=[p['PosX'],p['PosY'],p['PosZ']]),timeout=8)
        if ref.get('map_name')!=hint['map_name'] or ref.get('frame_id')!='map':raise ValueError('定位参考地图不一致')
        points=np.frombuffer(base64.b64decode(ref['points'],validate=True),dtype='<f4').reshape(-1,3)
        first,stamp=self.body_cloud(check=check);match=match_pose(np.asarray(first['points']),points,p,stamp=stamp,frame_id=first['frame_id'])
        second,next_stamp=self.body_cloud(after=stamp,check=check)
        verification=verify_pose(match,np.asarray(second['points']),points,stamp=next_stamp,frame_id=second['frame_id'])
        if check:check()
        current=self.app.hub.remote_maps('/maps/list',timeout=3)
        with self.app.lock:
            if check:check()
            self._stationary(link)
            if current.get('active')!=hint['map_name']:raise ValueError('地图已经切换，停止恢复定位')
            link.command(2101,1,match['pose'],'自动匹配最近位置并恢复定位')
        progress(.8,'已识别当前位置，等待机器人定位收敛')
        self.wait_normal(12,match['pose'],check=check)
        return dict(ok=True,localized=True,method='recent_pose_cloud_match',pose=match['pose'],verification=verification,message='已自动识别位置与朝向，定位恢复正常')

    def recover(self,progress,check=None,verify_current=False):
        if check:check()
        if self.app.demo:return dict(ok=True,demo=True,localized=True)
        generation=self.generation;link=self.app.link;demo=self.app.demo
        source_context=self.app.hub.context() if hasattr(self.app.hub,'context') else (demo,link)
        def guard():
            if check:check()
            if generation!=self.generation or self.app.link is not link or self.app.demo!=demo or getattr(self.app,'done',False):
                raise OperationCancelled('恢复定位已取消；不会继续初始化或启动其他操作')
        guard();status=self.status();guard()
        if self.good(status) and not verify_current:
            self.remember(status);guard();return dict(ok=True,localized=True,method='already_localized',message='当前定位正常')
        saved_error=None
        try:
            result=self.saved_slam_hint(progress,status,check=guard);guard();return result
        except OperationCancelled:raise
        except (ValueError,KeyError,ImportError) as exc:saved_error=str(exc)
        guard();hint_error=None
        try:
            result=self.local_hint(progress,check=guard);guard();return result
        except OperationCancelled:raise
        except (ValueError,KeyError,ImportError) as exc:hint_error=str(exc)
        guard();progress(.3,'正在全图搜索当前位置')
        self.app.hub.remote_maps('/maps/localize-auto',{},75,context=source_context)
        guard()
        try:self.wait_normal(65,check=guard)
        except OperationCancelled:raise
        except ValueError:raise ValueError('自动识别尚未确认当前位置，请在地图上指定大致位置；保存轨迹：'+str(saved_error)+'；附近匹配：'+str(hint_error))
        guard()
        return dict(ok=True,localized=True,method='vendor_global_search',message='自动重定位已恢复正常')

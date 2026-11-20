"""Desktop observer/heartbeat for explicitly delegated onboard missions.

No robot commands here. Transport loss never becomes proof of completion or a
reason to replay a saved start. The onboard service owns execution and stopping.
"""
import base64
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
from bridge.resident_mission import atomic
from m20.collection_store import SYNC_PROTOCOL, robot_digest, robot_key
from m20.collection_sync import CollectionSynchronizer
from m20.navigation_disconnect import POLICIES,TERMINAL,finite,identity as mission_identity,specification
from m20.platform_core import age_of,rpc


class ResidentNavigationClient:
    def __init__(self,app):
        self.app=app;self.path=app.store.path.parent/'resident-client.json';self.ref=None;self.nav=None
        self.worker=None;self.done=threading.Event();self.wake=threading.Event();self.caps={}
        if self.path.is_file():
            if self.path.is_symlink() or self.path.stat().st_size>4_000_000:raise ValueError('驻机任务引用异常')
            self.ref=json.loads(self.path.read_text(encoding='utf-8'))
            # Startup is reconciliation only, including an uncertain old upload.
            self.ref['submitted']=True
            self.nav=(copy.deepcopy(self.ref['nav']) if self.ref.get('terminal') else
                      dict(self.ref['nav'],status='running',phase='resident_unknown',message='正在核对机器人端原任务，不会重发启动'))
        self.collection_sync=CollectionSynchronizer(app.store.path.parent,self._collection_robot,
                                                    self._collection_request,on_update=self._collection_updated)
        self._collection_restored=False
        self._collection_link_lost=False
    def robot_identity(self):return self.app.hub.connections.value.get('aos_identity_alias')
    def _collection_robot(self):
        value=self.robot_identity()
        return robot_digest(value) if value and not self.app.demo else None
    def _collection_rows(self):
        history=getattr(self.app.store,'value',{}).get('history',[])
        return [self.app.nav,self.nav or {},*history]
    def _collection_tick(self):
        # This path also runs for terminal missions. It only queues work; disk
        # reads, negotiation and page transport belong to the sync worker.
        if self.done.is_set():return
        self.collection_sync.start()
        if not self._collection_restored:
            self._collection_restored=True
            with self.app.lock:
                candidates=[(copy.deepcopy(row.get('collection_run')),row.get('status')) for row in self._collection_rows()]
            seen=set()
            for run,status in candidates:
                if not run or run.get('local') or not run.get('robot'):continue
                key=(run['robot'],run.get('executor','gos'),run.get('id'))
                if key in seen:continue
                seen.add(key)
                try:self.collection_sync.notify(run,final=status in ('completed','cancelled','failed','paused'))
                except ValueError:pass  # A malformed old reference cannot authorize a network request.
    def _collection_request(self,run,path,body,timeout):
        with self.app.lock:
            if self._collection_robot()!=run['robot']:raise ValueError('当前连接不是采集任务所属机器人')
            role=run.get('executor','gos')
            if role not in ('gos','nos'):raise ValueError('采集执行器无效')
            base,token=self._endpoint(role)
        result=rpc(base,path,body,timeout,token)
        with self.app.lock:
            if self._collection_robot()!=run['robot']:raise ValueError('采集响应返回时机器人身份已变化')
        return result
    def _collection_updated(self,run,state,records):
        # Cache rows are history even if received just now. Never refresh a
        # sensor timestamp or publish them as live control observations.
        with self.app.lock:
            if self._collection_robot()!=run['robot']:return
            seen=set()
            for target in self._collection_rows():
                ref=target.get('collection_run',{})
                if (id(target) in seen or ref.get('id')!=run['id'] or ref.get('robot')!=run['robot']
                        or ref.get('executor','gos')!=run.get('executor','gos')):continue
                seen.add(id(target));target['collection_transfer']=copy.deepcopy(state)
                for record in records:
                    sample=dict(copy.deepcopy(record),stale=True,fresh=False,source='local_cache')
                    if record.get('kind')=='station_result':
                        rows=target.setdefault('station_results',[])
                        if not any(r.get('collection_run_id')==run['id'] and r.get('collection_seq')==record['seq'] for r in rows):
                            sample.update(collection_run_id=run['id'],collection_seq=record['seq'])
                            rows.append(sample)
                        continue
                    rows=target.setdefault('collection',[])
                    item=next((r for r in rows if r.get('spec',{}).get('kind')==record.get('kind')),None)
                    if item is None:
                        item=dict(spec=dict(kind=record.get('kind')),samples=0,failures=0,state='cached')
                        rows.append(item)
                    if record['seq']>item.get('cached_seq',0):
                        item.update(latest=sample,cached_seq=record['seq'],stale=True,fresh=False)
                        values=record.get('values')
                        people=values.get('people') if isinstance(values,dict) else None
                        if record.get('kind')=='people_count' and type(people) is int:
                            item['peak_people']=max(item.get('peak_people',0),people)
    @staticmethod
    def _resident_limits(nav):
        options=nav.get('route',{}).get('options',{})
        result={key:finite(nav.get(key,options.get(key,default))) for key,default in
                (('disconnect_timeout_s',5),('max_duration_s',1800))}
        if not 3<=result['disconnect_timeout_s']<=30 or not 30<=result['max_duration_s']<=3600:
            raise ValueError('断联等待或任务时限超出范围')
        return result
    def busy(self):return bool(self.ref and not self.ref.get('terminal'))
    def owns(self):return self.busy() and self.app.nav is self.nav
    def gos_busy(self):return self.busy() and self.ref.get('executor')=='gos'
    def preserve(self):return self.busy() and (self.gos_busy() or self.ref.get('policy')!='stop')
    def _save(self):
        if self.ref:self.ref['nav']=copy.deepcopy(self.nav);atomic(self.path,self.ref)
    def _endpoint(self,role='nos'):
        c=self.app.hub.connections;t=c.value['targets'].get(role,{})
        if not t.get('enabled'):raise ValueError('请先连接 '+role.upper()+' 服务')
        return c.endpoint(role)
    def _capabilities(self,role):
        try:
            base,token=self._endpoint(role)
            return rpc(base,'/missions/capabilities',None,2,token)
        except ValueError:
            return dict(ok=True,protocol=1,available=False,message=role.upper()+' 尚未加载导航任务服务')
    def capabilities(self):
        if self.app.demo:return dict(ok=True,protocol=1,available=False,message='断联执行需要机器人端驻机服务；演示不代表实机能力')
        nos=self._capabilities('nos')
        gos=self._capabilities('gos')
        if gos.get('available') and nos.get('map_lease_protocol')!=1:
            gos=dict(gos,available=False,message='请更新 NOS 地图服务后使用 GOS 导航')
        self.caps=dict(nos,gos=gos)
        return self.caps
    def snapshot(self):
        run=(self.nav or {}).get('collection_run')
        transfer=self.collection_sync.snapshot(run) if run and not run.get('local') and run.get('robot') else None
        return dict(protocol=1,active=self.busy(),mission=copy.deepcopy(self.ref.get('last',{})) if self.ref else None,
                    collection_transfer=transfer,collection_transfers=self.collection_sync.snapshots())
    def prepared(self,nav,executor='nos',axis_quiet_until=0.):
        """Called once by the existing navigation preparation, before dispatch."""
        app=self.app
        if self.busy():raise ValueError('驻机任务或停止状态尚未确认')
        policy=nav.get('disconnect_policy','stop')
        if executor=='gos':
            if policy!='stop':raise ValueError('GOS 自主寻路目前支持断联停车')
        elif policy not in POLICIES or policy=='stop':raise ValueError('驻机任务策略无效')
        if app.demo:raise ValueError('驻机断联任务需要实机执行服务')
        route=nav['route'];map_=copy.deepcopy(next((m for m in app.maps() if m['id']==route['map_id']),{}))
        if executor!='gos':
            if route.get('options',{}).get('collectors') or any(p.get('station_action') for p in route['points']):
                raise ValueError('原厂驻机断联任务暂不支持到点或沿途采集')
            limits=self._resident_limits(nav)
        if not map_.get('robot_map_name'):raise ValueError('需要已同步的机器人地图')
        expected=dict(NavMode=1,Gait=0x3002,Speed=1,ObsMode=0,Manner=0)
        if nav.get('options')!=expected:raise ValueError('断联任务仅支持低速敏捷平地、机器人自主导航与正向行走')
        if nav.get('pause_at_point') and executor=='nos':raise ValueError('断联任务的中间航点需自动接续，请关闭到点暂停')
        if not self.robot_identity():raise ValueError('尚未核对机器人身份')
        self._endpoint(executor);identity=self.robot_identity()
        if executor=='gos':self._endpoint('nos')
        source=app.store.path.parent/'maps'/(map_['id']+'.pgm')
        fingerprint=hashlib.sha256(source.read_bytes()).hexdigest()
        self.nav=nav;nav.update(engine='resident',executor=executor,phase='resident_starting',execution_state='preparing',
                               message='正在检查 GOS 定位与地图' if executor=='gos' else '正在向机器人提交完整任务与断联策略')
        self.ref=dict(id=uuid.uuid4().hex,owner=uuid.uuid4().hex,robot_identity=identity,
                      policy=policy,map=map_,fingerprint=fingerprint,
                      submitted=False,cancel=False,terminal=False,last={},executor=executor,
                      axis_quiet_until=axis_quiet_until,start_index=nav.get('index',0),
                      prior_station_results=copy.deepcopy(nav.get('station_results',[])))
        if executor=='gos' and (route.get('options',{}).get('collectors') or any(p.get('station_action') for p in route['points'])):
            self.ref['collection_robot']=robot_digest(identity)
            nav['collection_run']=dict(id=self.ref['id'],robot=self.ref['collection_robot'],executor='gos')
        elif executor!='gos':
            self.ref.update(limits)
        self._collection_link_lost=False
        try:self._save()
        except Exception:
            self.ref=None;nav.update(status='failed',phase='failed',error='任务引用无法保存，未提交驻机任务');raise
        self.tick()
    def cancel(self,after='cancelled'):
        if not self.busy():return
        self.ref['cancel']=True;self.ref['after_cancel']=after
        self.nav.update(status='cancelling',phase='resident_stopping',message='正在请求机器人结束驻机任务，停车尚待确认')
        self._save();self.wake.set();self.tick()
    def tick(self):
        self._collection_tick()
        if not self.busy() or self.done.is_set():return
        if self.app.demo or self.robot_identity()!=self.ref['robot_identity']:return
        if self.app.nav.get('engine')!='resident' and self.app.nav.get('status') not in ('idle','completed','failed','cancelled'):return
        if self.app.nav is not self.nav:self.app.nav=self.nav
        if not self.worker or not self.worker.is_alive():
            self.worker=threading.Thread(target=self._run,daemon=True);self.worker.start()
    def _request(self,path,request=None):
        ref=self.ref
        if self.robot_identity()!=ref['robot_identity'] or self.app.demo:raise ValueError('当前连接不是任务所属机器人')
        # Current authenticated tunnel, only after the stable robot identity
        # matched. A changed local port/token may not be replayed blindly.
        role='nos' if path.startswith('/maps/') else ref.get('executor','nos')
        base,token=self._endpoint(role)
        return rpc(base,path,request,4 if path in ('/maps/grid','/missions/start') else 2,token)

    def _map_lease(self,action,expected=None):
        req={key:self.ref[key] for key in ('id','owner')}
        req['action']=action
        if expected is not None:req['map']=expected
        return self._request('/maps/navigation-lease',req)

    def _release_map(self):
        if not self.ref.get('map_lease'):
            return
        try:self._map_lease('release')
        except ValueError:pass
        self.ref['map_lease']=False
    def _submit(self):
        ref=self.ref;map_=ref['map'];nav=self.nav
        caps=self._request('/missions/capabilities')
        if caps.get('protocol')!=1 or caps.get('available') is not True:raise ValueError('驻机执行服务尚未配置，未启动任务')
        if ref['policy']!='stop' and ref['policy'] not in caps.get('policies', []):
            raise ValueError('当前 NOS 服务尚不支持所选断联策略')
        if ref.get('executor')=='gos':
            sensors=self._request('/sensors/status')
            if 'odom' in sensors and sensors['odom'] is None:
                self.wake.wait(.4)
                if ref['cancel'] or self.done.is_set():return
                sensors=self._request('/sensors/status')
                if 'odom' in sensors and sensors['odom'] is None:
                    raise ValueError('GOS 未收到 /ODOM 地图定位，请检查 NOS 到 GOS 的定位数据连接')
        grid=self._request('/maps/grid',dict(name=map_['robot_map_name'],grid_id=map_.get('grid_id',0)))
        raw=base64.b64decode(grid['pgm'],validate=True)
        if (grid.get('robot_map_name')!=map_['robot_map_name'] or hashlib.sha256(raw).hexdigest()!=ref['fingerprint']
                or grid.get('resolution')!=map_['resolution'] or grid.get('origin')!=map_['origin']):
            raise ValueError('驻机任务地图内容或坐标系已变化，请重新同步规划')
        body=dict(id=ref['id'],owner=ref['owner'],disconnect_policy=ref['policy'],
                  points=copy.deepcopy(nav['route']['points']),
                  map=dict(name=map_['robot_map_name'],grid_id=map_.get('grid_id',0),sha256=ref['fingerprint'],
                           yaml_sha256=grid.get('yaml_sha256'),resolution=map_['resolution'],origin=map_['origin']))
        if ref.get('executor')!='gos':
            body.update(self._resident_limits(nav))
        spec=specification(body)
        if ref.get('executor')=='gos':
            if nav.get('collection_run'):
                ref['collection_robot']=robot_digest(ref['robot_identity'])
                body['collection_robot']=ref['collection_robot']
                protocol=SYNC_PROTOCOL if caps.get('collection_sync_protocol')==SYNC_PROTOCOL else 0
                ref['collection_sync_protocol']=protocol
                nav['collection_run'].update(sync_protocol=protocol,executor='gos')
            from bridge.gos_navigation import pack_grid
            if caps.get('executor')!='gos' or grid.get('active') is not True:
                raise ValueError('GOS 未就绪或航点地图已变化')
            body.update(grid=pack_grid(raw,grid), start_index=ref['start_index'],
                        pause_at_point=bool(nav.get('pause_at_point')))
            collectors=nav['route'].get('options', {}).get('collectors')
            uses_iot_picker = (any((p.get('station_action') or {}).get('iot_device_ids') for p in body['points'])
                               or any(item.get('device_ids') for item in collectors or []))
            if uses_iot_picker and caps.get('iot_task_protocol')!=1:
                raise ValueError('请在连接中心更新 GOS 物联网任务服务')
            if collectors:
                if caps.get('collection_protocol')!=1:
                    raise ValueError('请更新 GOS 沿途采集服务')
                body['collectors']=collectors
            wait=max(0.,ref['axis_quiet_until']-time.monotonic())
            if wait:self.wake.wait(wait)
            if ref['cancel'] or self.done.is_set():return
            self._map_lease('claim',spec['map'])
            ref['map_lease']=True
        else:body=spec
        with self.app.lock:
            token=nav.get('client_intent')
            if token:
                context=nav.get('client_intent_context',[])
                self.app.navigation_intents.check(tuple(context+[token]),self.app.instance_id,self.app._plan_generation)
            if ref['cancel'] or self.done.is_set():return
            if ref.get('executor')=='gos':nav['message']='正在向 GOS 提交导航任务'
            # Persist before transmission: after an uncertain response/restart,
            # reconcile the identity, never automatically repeat this POST.
            ref['submitted']=True;self._save()
        try:
            self._request('/missions/start',body)
        except ValueError as error:
            if ref.get('executor')=='gos':
                ref['start_error']=str(error)
                self.cancel()
            raise
    def _observe(self,result):
        row=result.get('mission',{})
        if row.get('id')!=self.ref['id']:raise ValueError('机器人端任务记录尚未匹配，启动或停车结果待核对')
        gos=self.ref.get('executor')=='gos'
        phase=row.get('phase')
        terminal=phase in (('completed','cancelled','failed','paused') if gos else
                           ('completed','cancelled','failed')) and row.get('stop_confirmed') is True
        if gos and terminal:self._release_map()
        previous=self.ref.get('last',{})
        notify=False;collection_run=None;collection_target=None
        with self.app.lock:
            self.ref['last']=copy.deepcopy(row)
            status=phase if terminal else 'cancelling' if self.ref['cancel'] else 'running'
            self.nav.update(status=status,phase='resident_'+str(phase),execution_state='resident',
                index=row.get('completed',0),completed=row.get('completed',0),
                progress=row.get('completed',0)/len(self.nav['route']['points']),message=row.get('message','机器人端任务状态已同步'),
                resident_mission=copy.deepcopy(row))
            if gos:
                self.nav.update(executor='gos',phase=row.get('stage',phase),
                                plan=row.get('plan') or self.nav.get('plan'),
                                cancel_confirmed=terminal)
                prior=self.ref.get('prior_station_results',[])
                if 'station_results' in row:
                    self.nav['station_results']=prior+[dict(copy.deepcopy(r),stale=True,fresh=False) for r in row['station_results']]
                if self.nav.get('collection_run'):
                    old={r.get('spec',{}).get('kind'):r for r in self.nav.get('collection',[])}
                    collected=copy.deepcopy(row.get('collection',[]))
                    for item in collected:
                        cached=old.get(item.get('spec',{}).get('kind'),{})
                        if 'latest' not in item:
                            for key in ('latest','cached_seq','peak_people'):
                                if key in cached:item[key]=copy.deepcopy(cached[key])
                        if item.get('latest'):
                            item['latest'].update(stale=True,fresh=False)
                        item.update(stale=True,fresh=False)
                    self.nav['collection']=collected
                    collection_run=copy.deepcopy(self.nav['collection_run'])
                    journal=row.get('collection_journal') or {}
                    if (journal.get('collection_sync_protocol')==SYNC_PROTOCOL
                            and journal.get('robot')==collection_run.get('robot')):
                        self.ref['collection_sync_protocol']=SYNC_PROTOCOL
                        self.nav['collection_run']['sync_protocol']=SYNC_PROTOCOL
                        collection_target=journal.get('high_watermark')
                    notify=(not previous or self._collection_link_lost or
                            row.get('completed',0)>previous.get('completed',0) or
                            terminal and not self.ref.get('terminal'))
                    self.nav['collection_transfer']=self.collection_sync.snapshot(collection_run)
            if phase=='interrupted':
                self.nav.update(status='cancelling',error='驻机服务曾中断，停车状态待核对')
                if gos:self.ref['cancel']=True
            if terminal:
                if gos and (phase=='completed' or self.nav.get('station_results') or self.nav.get('collection')
                            or self.nav.get('collection_run')):
                    self.app._record_route_completion(self.nav)
                elif phase=='completed':
                    self.app._record_route_completion()
            self.ref['terminal']=terminal;self._save()
        self._collection_link_lost=False
        if notify and collection_run:
            self.collection_sync.notify(collection_run,final=terminal,target=collection_target)

    def collection_page(self, run_id, offset):
        run_id=mission_identity(run_id)
        if type(offset) is not int or offset<0:raise ValueError('采集记录页码无效')
        with self.app.lock:
            candidates=[r['collection_run'] for r in self._collection_rows()
                        if r.get('collection_run',{}).get('id')==run_id]
            if not candidates:raise ValueError('采集任务记录不存在')
            keys={(r.get('robot'),r.get('executor','gos'),bool(r.get('local'))) for r in candidates}
            if len(keys)!=1:raise ValueError('采集任务编号对应多个机器人，不能混用记录')
            run=copy.deepcopy(candidates[0])
            final=any(r.get('status') in ('completed','cancelled','failed','paused') for r in self._collection_rows()
                      if r.get('collection_run',{}).get('id')==run_id)
        if run.get('local') is True:
            from bridge.task_collection import collection_page
            return collection_page(self.app.store.path.parent,run_id,offset)
        robot_key(run.get('robot'))
        role=run.get('executor','gos')
        page=self.collection_sync.cache.page(run_id,offset,robot=run['robot'],executor=role)
        if page.get('available') and page.get('rows'):
            return page  # Cached history works offline, even after switching robots.
        state=self.collection_sync.snapshot(run)
        if page.get('available') and not state.get('pending'):
            return page
        self.collection_sync.notify(run,final=final)
        if self._collection_robot()!=run['robot']:
            page.update(stale=True,live=False)
            return page
        # Explicit legacy downloads retain the old endpoint behavior, but are
        # never labelled durable incremental synchronization or realtime data.
        supported=state.get('supported')
        if supported is None and run.get('sync_protocol') in (0,SYNC_PROTOCOL):
            supported=run['sync_protocol']==SYNC_PROTOCOL
        if supported is None:
            try:
                caps=self._collection_request(run,'/missions/capabilities',None,1.)
                supported=caps.get('collection_sync_protocol')==SYNC_PROTOCOL
            except ValueError:
                return page  # A transport failure is not proof of old protocol.
        if supported is False:
            legacy=self._collection_request(run,'/missions/collection',dict(id=run_id,offset=offset),2.)
            if legacy.get('run_id')!=run_id or not isinstance(legacy.get('rows'),list):
                raise ValueError('旧采集页任务身份不匹配')
            legacy.update(sync_supported=False,collection_sync_protocol=0,source='legacy_remote',
                          cached=False,stale=True,live=False,complete=False)
            return legacy
        return page
    def _run(self):
        reference=self.ref
        while not self.done.is_set() and self.ref is reference and self.busy():
            try:
                if self.app.demo or self.robot_identity()!=self.ref['robot_identity']:return
                identity={k:self.ref[k] for k in ('id','owner')}
                gos=self.ref.get('executor')=='gos'
                if gos and self.nav.get('collection_run'):
                    identity['collection_sync_protocol']=SYNC_PROTOCOL
                if self.ref['cancel']:
                    cancel=dict(identity,after=self.ref.get('after_cancel')) if gos else identity
                    result=self._request('/missions/cancel',cancel)
                    if result.get('matched') is False and result.get('revoked') is True:
                        if gos:self._release_map()
                        with self.app.lock:
                            if self.ref is not reference:return
                            self.ref['terminal']=True
                            start_error=self.ref.get('start_error')
                            status='failed' if start_error else 'cancelled'
                            self.nav.update(status=status,phase=status,message=start_error or '本次驻机任务启动已撤销')
                            self._save()
                        return
                elif not self.ref['submitted']:
                    try:self._submit()
                    except Exception as error:
                        if not self.ref['submitted']:
                            if gos:self._release_map()
                            with self.app.lock:
                                message=str(error) if isinstance(error,ValueError) and str(error) else '驻机任务未提交，请检查执行服务与地图'
                                self.ref['terminal']=True;self.nav.update(status='failed',phase='failed',message=message)
                                self._save()
                        raise
                if not self.ref['submitted'] and not self.ref['cancel']:continue
                if gos and not self.ref['cancel']:
                    link=self.app.link
                    if link and age_of(link.record(1002,6))<2 and self.ref.get('map_lease'):
                        from m20.observation_errors import BridgeTransportError
                        try:self._map_lease('pulse')
                        except BridgeTransportError:raise
                        except ValueError:
                            self.cancel()
                            raise
                        self._observe(self._request('/missions/pulse',identity))
                        self.wake.wait(1);self.wake.clear()
                        continue
                result=(self._request('/missions/status',dict(collection_sync_protocol=SYNC_PROTOCOL))
                        if gos and self.nav.get('collection_run') else self._request('/missions/status'))
                if self.ref is not reference:return
                self._observe(result)
                link=self.app.link
                if gos and result.get('mission',{}).get('phase')=='interrupted':
                    self.cancel()
                if not gos and self.busy() and not self.ref['cancel'] and link and age_of(link.record(1002,6))<2:
                    self._request('/missions/pulse',identity)
            except Exception as error:
                self._collection_link_lost=True
                with self.app.lock:
                    if self.ref is not reference:return
                    if self.busy():self.nav.update(phase='resident_unknown',message='电脑未确认机器人状态；按已选断联策略执行，结果待同步',connection_error=str(error))
                    else:self.nav['error']=str(error)
            self.wake.wait(1);self.wake.clear()
    def close(self):
        # Closing the PC is loss of its heartbeat, not an operator Stop command.
        self.done.set();self.wake.set()
        self.collection_sync.close()
        if self.worker:self.worker.join(timeout=.2)

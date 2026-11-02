"""NOS-side passive recorder. The active map and navigation services are untouched."""
import base64,copy,json,math,threading,time,uuid
from pathlib import Path
import numpy as np
from map_refinement import base_fingerprint


class RefinementAgent:
    def __init__(self,maps,sensors,clock=time.monotonic):
        self.maps=maps;self.sensors=sensors;self.clock=clock;self.lock=threading.RLock();self.thread=None;self.done=False;self.context={};self.lease=0
        self.root=maps.data/'refinement';self.root.mkdir(parents=True,exist_ok=True);self.file=self.root/'state.json'
        self.row=json.loads(self.file.read_text()) if self.file.exists() else dict(frames=0,revisions=[],message='开启后，在正常定位下积累行走观测')
        self.row.update(enabled=False,state='off',protocol=1);self.last_pose=None;self.last_stamp=0;self.published_frames=self.row.get('published_frames',0);self.last_publish=self.clock();self.flush=False

    def save(self):
        temp=self.file.with_suffix('.tmp');temp.write_text(json.dumps(self.row,ensure_ascii=False));temp.replace(self.file)

    def status(self):
        with self.lock:return dict(ok=True,**copy.deepcopy(self.row))

    def control(self,req):
        action=req.get('action','status')
        with self.lock:
            if action=='status':return self.status()
            if action=='lease':self.context=dict(req.get('context',{}));self.lease=self.clock();return self.status()
            if action=='enable':
                if self.row.get('state')=='generating':raise ValueError('正在生成新版本，请稍候')
                base=self.maps.path('active');grid=int(req.get('grid_id',0));fingerprint=base_fingerprint(base,grid)
                if self.row.get('active_fingerprint',self.row.get('fingerprint'))!=fingerprint or self.row.get('base')!=base.name or self.row.get('grid_id')!=grid:
                    key=uuid.uuid4().hex;folder=self.root/'sessions'/key;folder.mkdir(parents=True)
                    self.row=dict(protocol=1,session=key,base=base.name,working_base=base.name,grid_id=grid,fingerprint=fingerprint,active_fingerprint=fingerprint,frames=0,total_frames=0,revisions=[],bytes=0,published_frames=0)
                    self.published_frames=0
                    (folder/'session.json').write_text(json.dumps(dict(base=base.name,grid_id=grid,fingerprint=fingerprint)))
                self.last_pose=None;self.last_stamp=0;self.row.update(enabled=True,state='waiting',message='等待行走与新鲜定位观测');self.context=dict(req.get('context',{}));self.lease=self.clock()
            elif action=='disable':
                self.row.update(enabled=False,message='已关闭，已采集观测保留')
                if self.row.get('state')!='generating':self.row['state']='off'
                self.flush=self.row.get('frames',0)>self.published_frames
            elif action=='generate':self.flush=True
            else:raise ValueError('未知地图优化操作')
            if not self.thread or not self.thread.is_alive():
                self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
            self.save();return self.status()

    def ingest(self,packet):
        if packet.get('protocol')!=1 or packet.get('frame_id')!='map' or not math.isfinite(packet.get('age',99)) or not 0<=packet.get('age',99)<=.7:raise ValueError('完整定位点云已过期或接口尚未更新')
        stamp=float(packet['stamp']);pose=np.asarray(packet['pose'],float);points=np.frombuffer(base64.b64decode(packet['points'],validate=True),dtype='<f4').reshape(-1,3)
        quality=packet.get('quality',[])
        if len(quality)!=2 or not np.isfinite(quality).all() or quality[0]>.25 or quality[1]<.85:raise ValueError('当前匹配质量不足')
        if pose.shape!=(7,) or not np.isfinite(pose).all() or not .5<np.linalg.norm(pose[3:])<1.5 or not math.isfinite(stamp) or not np.isfinite(points).all() or not 100<=len(points)<=60000:raise ValueError('完整点云或位姿格式无效')
        if 'anchor_z' in self.row and abs(pose[2]-self.row['anchor_z'])>.35:raise ValueError('当前高度已离开本层，暂停更新此栅格；请切换正确地图/楼层')
        if stamp<=self.last_stamp:return False
        if self.last_pose is not None:
            displacement=float(np.linalg.norm(pose[:2]-self.last_pose[:2]));elapsed=stamp-self.last_stamp
            if elapsed<.5 and displacement>1 or elapsed>0 and displacement/elapsed>2.5:raise ValueError('定位位置跳变，暂停积累地图观测')
            if elapsed<3 or displacement<.25:
                self.row.update(state='waiting_motion',message='已接通定位观测，等待行走带来新的视角');return False
        from map_fusion import pose_matrix,transform
        from autonomy_sensing import supported_floor
        supported_floor(transform(points,np.linalg.inv(pose_matrix(pose))))
        if self.row['frames']>=500 or self.row.get('bytes',0)>256*1024**2:raise ValueError('本轮采集已达到容量，请生成版本后启用新图继续')
        folder=self.root/'sessions'/self.row['session'];name='frame-%06d.npz'%self.row['frames'];path=folder/name
        with path.with_suffix('.tmp').open('wb') as f:np.savez_compressed(f,points=points,pose=pose,stamp=stamp)
        path.with_suffix('.tmp').replace(path)
        self.row['frames']+=1;self.row['bytes']=self.row.get('bytes',0)+path.stat().st_size
        self.row['total_frames']=self.row.get('total_frames',self.row['frames']-1)+1
        self.row.setdefault('anchor_z',float(pose[2]))
        self.last_pose=pose;self.last_stamp=stamp;self.row.update(state='collecting',message='正在积累行走观测，导航与当前地图继续使用',last_frame=time.time());self.save();return True

    def publish(self):
        with self.lock:
            if self.row.get('frames',0)<=self.published_frames:self.flush=False;return
            if self.row.get('frames',0)<3:
                self.flush=False;self.row.update(state='waiting' if self.row['enabled'] else 'off',message='观测已保留，至少需要多次行走观测才能生成更新');self.save();return
            count=self.row['frames'];session=self.row['session'];base=self.row.get('working_base',self.row['base']);self.row.update(state='generating',message='正在合并完整旧图与行走更新，当前地图继续使用');self.flush=False
        try:
            name=self.row['base'][:55]+'-walk-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:4]
            if self.maps.demo:
                from map_refinement import refine_map
                r=refine_map(self.maps.path(base),self.root/'sessions'/session,self.maps.maps/name,self.row['grid_id'],self.row['fingerprint'])
            else:r=self.maps.privileged('refine_map',dict(session=session,name=name))
            with self.lock:
                self.published_frames=count;self.row['published_frames']=count
                if r.get('map'):
                    self.row['revisions'].append(dict(name=r['map'],created=time.time(),report=r['report']));self.row['revisions']=self.row['revisions'][-20:]
                # Further updates build on the latest complete candidate, while
                # navigation keeps using the explicitly selected active map.
                if r.get('map') or count>=120:
                    working=r.get('map') or base;fingerprint=base_fingerprint(self.maps.path(working),self.row['grid_id'])
                    key=uuid.uuid4().hex;folder=self.root/'sessions'/key;folder.mkdir(parents=True)
                    (folder/'session.json').write_text(json.dumps(dict(base=working,grid_id=self.row['grid_id'],fingerprint=fingerprint)))
                    self.row.update(session=key,working_base=working,fingerprint=fingerprint,frames=0,bytes=0,published_frames=0);self.published_frames=0
                self.row.update(state='collecting' if self.row['enabled'] else 'off',message=r['message']);self.last_publish=self.clock();self.save()
        except Exception as exc:
            with self.lock:self.row.update(state='error',message='生成失败，观测已保留：'+str(exc));self.last_publish=self.clock();self.save()

    def run(self):
        while not self.done:
            with self.lock:
                enabled=self.row.get('enabled');flush=self.flush
                context=dict(self.context);age=self.clock()-self.lease
            if enabled:
                try:
                    if age>12 or not context.get('connected'):raise ValueError('平台连接已暂停，已保留观测')
                    if context.get('mapping') or context.get('sdk'):raise ValueError('当前处于建图或 SDK 操作，暂停后台积累')
                    if not context.get('localized'):raise ValueError('等待定位恢复正常')
                    if self.maps.path('active').name!=self.row['base']:
                        with self.lock:self.row['enabled']=False;self.save()
                        raise ValueError('活动地图已改变，采集已关闭；重新开启即可绑定新地图')
                    packet=self.sensors().observation()
                    with self.lock:
                        if self.row.get('enabled'):self.ingest(packet)
                except Exception as exc:
                    with self.lock:self.row.update(state='waiting',message=str(exc))
            if flush or enabled and self.row.get('frames',0)-self.published_frames>=12 and self.clock()-self.last_publish>60:self.publish()
            if not enabled and not flush:
                with self.lock:
                    if not self.row.get('enabled') and not self.flush:self.thread=None;return
            time.sleep(1)

    def close(self):self.done=True

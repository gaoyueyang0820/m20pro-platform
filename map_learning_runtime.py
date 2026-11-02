"""Console coordination for passive observations; this module never drives."""
import base64,copy,json,math,time,threading

class MapLearning:
    def __init__(self,app):
        self.app=app;self.lock=threading.RLock();self.enabled=False;self.demo_agent=None;self.target=None
        self.row=dict(protocol=1,enabled=False,state='off',frames=0,revisions=[],message='开启后，在行走中积累观测；新版本由你点击启用')
        self.pending_disable=False
        saved=app.hub.demo_maps.data/'refinement/state.json'
        if saved.exists():
            self.row=json.loads(saved.read_text());self.row.update(enabled=False,state='off',message='已保留上次观测与新版本');self.target='demo'

    def context(self):
        a=self.app
        return dict(connected=a.demo or bool(a.link and time.time()-a.link.record(1002,6).get('at',0)<2),localized=a.demo or bool(a.link and time.time()-a.link.record(1007,2).get('at',0)<2 and a.pose.get('Location')==0),
                    mapping=a.mapping,sdk=a.sdk.get('armed') or a.sdk.get('vendor_running') or a.hub.sdk_mode.get('state') in ['sdk','awaiting_handset_exit'])

    def call(self,request):
        if self.app.demo:
            if self.demo_agent is None:
                from bridge.map_refinement_agent import RefinementAgent
                self.demo_agent=RefinementAgent(self.app.hub.demo_maps,lambda:self)
            return self.demo_agent.control(request)
        return self.app.proxy('nos','/maps/refinement',request,timeout=3)

    def observation(self):
        """Explicit offline fixture, using demo pose only; never used in live mode."""
        import numpy as np
        x,y,z=self.app.pose['PosX'],self.app.pose['PosY'],.55;angle=self.app.pose['Yaw']
        axis=np.arange(-3,3,.12);xx,yy=np.meshgrid(axis,axis);pts=np.column_stack([xx.ravel()+x,yy.ravel()+y,np.zeros(xx.size)])
        # Stable demonstration pillar, present in the generated observations.
        a,b=np.meshgrid(np.arange(-.15,.16,.06),np.arange(.15,1.1,.1));wall=np.column_stack([np.full(a.size,1.),a.ravel(),b.ravel()]);pts=np.vstack([pts,wall])
        return dict(ok=True,protocol=1,demo=True,frame_id='map',age=0,stamp=time.time(),quality=[.03,.97],pose=[x,y,z,0,0,math.sin(angle/2),math.cos(angle/2)],points=base64.b64encode(pts.astype('<f4').tobytes()).decode())

    def control(self,req):
        action=req.get('action')
        if action=='enable':
            self.app.hub.normal_mode()
            if self.app.mapping:raise ValueError('请先结束专门的建图采集，再开启日常行走优化')
            self.app.live_ready();ctx=self.context()
            if not ctx['localized']:raise ValueError('先在活动地图上完成定位，再开启行走优化')
            active=next((m for m in self.app.maps() if m.get('verified_for_robot')),None)
            req=dict(req,grid_id=(active or {}).get('grid_id',0),context=ctx)
        if action not in ['enable','disable','generate']:raise ValueError('未知地图优化操作')
        with self.lock:
            if action=='disable':self.enabled=False;self.pending_disable=True
            try:self.row=self.call(req)
            except Exception:
                if action!='disable':raise
                self.row=dict(self.row,enabled=False,state='waiting',message='已停止本机采集续约，等待设备确认关闭；失联时设备端会自动暂停')
                return dict(self.row,ok=True,demo=self.app.demo)
            self.pending_disable=False;self.enabled=bool(self.row.get('enabled'));self.target=('demo' if self.app.demo else self.app.link.ip if self.app.link else None)
            if self.enabled:self.row=self.call(dict(action='lease',context=self.context()))
            return dict(self.row,demo=self.app.demo)

    def poll(self):
        with self.lock:
            # Do not transfer a recording session between robots or demo/live modes.
            target='demo' if self.app.demo else self.app.link.ip if self.app.link else None
            if self.target is None:
                if not self.app.demo and self.context()['connected']:
                    try:
                        self.row=self.call(dict(action='status'));self.target=target;self.enabled=bool(self.row.get('enabled'))
                    except Exception:pass
                return
            if target!=self.target:
                self.enabled=False;self.target=None
                if self.demo_agent:self.demo_agent.close();self.demo_agent=None
                self.row=dict(enabled=False,state='off',frames=0,revisions=[],message='连接已切换，请按当前地图重新开启');return
            try:
                if self.pending_disable:
                    self.row=self.call(dict(action='disable'));self.pending_disable=False
                else:self.row=self.call(dict(action='lease',context=self.context())) if self.enabled else self.call(dict(action='status'))
                self.enabled=bool(self.row.get('enabled'))
            except Exception as exc:self.row=dict(self.row,state='waiting',message='等待地图优化服务：'+str(exc))

    def snapshot(self):
        # Called from the app state lock: never wait for network or the polling lock.
        return dict(copy.deepcopy(self.row),demo=self.app.demo)

    def reset(self):
        with self.lock:
            if self.demo_agent:self.demo_agent.close();self.demo_agent=None
            self.enabled=False;self.pending_disable=False;self.target=None
            self.row=dict(protocol=1,enabled=False,state='off',frames=0,revisions=[],message='连接已切换，等待当前地图；开启后积累行走观测')

    def close(self):
        if self.demo_agent:self.demo_agent.close()

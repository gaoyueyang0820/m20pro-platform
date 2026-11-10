"""Offline PointNavigator integration: real start/prepare/drive/stop lifecycle.

Only transport, sensor input, and trajectory choices are mocked; no sockets,
8080, robot processes, or hardware commands. Worker race cases use barriers.
"""
import base64,copy,threading,time,unittest,sys
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_platform as cases
import m20.point_navigation as runtime
from m20.navigation_planner import GridPlanner

REAL_THREAD=threading.Thread

@contextmanager
def replay_clock():
    # CPU scheduling must not age synthetic telemetry. Only explicit simulated
    # time advances it; production freshness and stop feedback checks still run.
    now=[100.]
    def advance(seconds):now[0]+=seconds
    with patch.object(runtime.time,'monotonic',side_effect=lambda:now[0]),\
         patch.object(runtime.time,'time',side_effect=lambda:1700000000.+now[0]),\
         patch.object(runtime.time,'sleep',side_effect=advance):
        yield advance

class RecordedLink(cases.FakeLink):
    def __init__(self):
        self.velocities=[];self.on_mode=None
        super().__init__()
    def put(self,t,c,it,at=None,seq=0):
        super().put(t,c,it,at,seq);self.records[t,c]['monotonic_at']=time.monotonic()
    def set_velocity(self,*v,valid_until=None):
        super().set_velocity(*v,valid_until=valid_until)
        self.velocities.append(dict(v=list(v),valid_until=valid_until))
    def command(self,*args):
        r=super().command(*args);r['sent_monotonic']=time.monotonic();return r
    def set_mode(self,mode):
        super().set_mode(mode)
        if self.on_mode:self.on_mode(mode)


class DeferredWorkers:
    def __init__(self):self.jobs=[]
    def thread(self,target,args=(),daemon=None,**kwargs):
        outer=self
        class Deferred:
            def start(self):outer.jobs.append((target,args))
        return Deferred()
    def run(self,index=0):
        target,args=self.jobs[index];target(*args)


class PointNavigationRuntimeTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    def tearDown(self):
        self.app.done=True;self.app.hub.close();cases.PlatformTests.tearDown(self)
    def ready(self,x=2.05,y=3.05,yaw=0.,mode=0,velocity=(0,0,0)):
        a=self.app
        basic=dict(HES=0,MotionState=17,ControlUsageMode=mode,Sleep=0,Gait=0x3002,Direction=0)
        a.link.put(1002,6,dict(BasicStatus=basic));a.basic=basic.copy()
        a.link.put(1002,4,dict(MotionStatus=dict(zip(['LinearX','LinearY','OmegaZ'],velocity))))
        a.link.put(1007,2,dict(Location=0,Confidence=.99,PosX=x,PosY=y,Yaw=yaw));a.pose=dict(Location=0,Confidence=.99,PosX=x,PosY=y,Yaw=yaw)
        a.link.put(1007,1,dict(Value=0,Status=0,ErrorCode=0))
    def setup_live(self):
        a=self.app;grid=np.full((100,120),254,np.uint8);grid[:2]=0;grid[-2:]=0;grid[:,:2]=0;grid[:,-2:]=0
        raw=b'P5\n120 100\n255\n'+grid.tobytes()
        map_=a.import_map(dict(name='runtime-test',pgm=base64.b64encode(raw).decode(),resolution=.1,origin=[0,0,0]))['map']
        map_.update(robot_map_name='runtime-test',verified_for_robot=True,robot_ip='127.0.0.1')
        a.demo=False;a.link=RecordedLink();self.ready();a.hub.sdk_mode=dict(state='normal')
        a.proxy=lambda *args,**kwargs:dict(active='runtime-test',maps=[dict(name='runtime-test',grids=[0])],phase='idle',in_progress=False)
        a.hub.remote_maps=lambda *args,**kwargs:dict(active='runtime-test',maps=[dict(name='runtime-test',grids=[0])],phase='idle',in_progress=False)
        self.map=map_;self.scans=0
        def scan(mapping=False,**kwargs):
            self.scans+=1
            return dict(pose=[a.pose[k] for k in ['PosX','PosY','Yaw']],floor=[[x,y] for x in range(-20,21) for y in range(-20,21)],
                measured_floor=[[x,y] for x in range(-20,21) for y in range(-20,21)],obstacles=[],obstacle_bounds=[],geometry_version=2,protocol=1,
                frame_id='base_link',pose_frame_id='map',pose_stamp=float(self.scans),age=.15,stamp=float(self.scans),_valid_until=time.monotonic()+.5)
        a.autonomy.scan_now=scan
        return a
    def start(self,workers,x=8.05,y=3.05):
        with patch.object(runtime.threading,'Thread',side_effect=workers.thread):
            return self.app.start_goal(dict(map_id=self.map['id'],x=x,y=y,engine='platform'))
    @staticmethod
    def follower(sequence):
        class Stub:
            def __init__(self):self.calls=0
            def set_path(self,path):self.path=path
            def command(self,scan,yaw_rate,goal_yaw,allowed_pose=None,*,fixed_heading=False):
                if not fixed_heading:assert allowed_pose(*scan['pose'])
                row=sequence[min(self.calls,len(sequence)-1)];self.calls+=1
                return dict(state=row[0],axes=row[1],reason='offline trajectory choice')
        return Stub

    def test_real_start_prepare_drive_arrival_waits_for_actual_stop(self):
        with replay_clock() as advance:
            a=self.setup_live();workers=DeferredWorkers();result=self.start(workers)
            self.assertEqual(result['execution'],'platform_trajectory');self.assertEqual(a.nav['phase'],'preparing')
            with patch.object(runtime,'TrajectoryFollower',self.follower([('tracking',[.35,0,0]),('arrived',[0,0,0])])):
                workers.run()
            self.assertTrue(any(v['v']==[.35,0,0] and v['valid_until'] for v in a.link.velocities))
            self.assertEqual(a.nav['status'],'cancelling',a.nav)
            self.assertEqual(a.point_navigation.after_stop,'completed',a.nav)
            self.assertNotIn('error',a.nav);self.assertFalse(a.nav.get('cancel_confirmed',False))
            self.assertFalse(any(p.get('type')==1003 for p in a.link.sent))
            advance(.01);self.ready(x=8.05);a.tick(.01)
            self.assertEqual(a.nav['status'],'completed',a.nav);self.assertTrue(a.nav['history_saved'])

    def test_cancel_before_preparation_does_not_late_start_or_drive(self):
        a=self.setup_live();workers=DeferredWorkers();self.start(workers)
        a.stop();self.ready();a.tick(.01);count=len(a.link.sent)
        workers.run()
        self.assertEqual(a.nav['status'],'cancelled');self.assertEqual(len(a.link.sent),count)
        self.assertFalse(any(any(v['v']) for v in a.link.velocities))

    def test_mode_change_is_read_back_before_first_drive(self):
        with replay_clock() as advance:
            a=self.setup_live();self.ready(mode=1);workers=DeferredWorkers();self.start(workers)
            def mode_changed(mode):
                advance(.001);self.ready(mode=mode)
            a.link.on_mode=mode_changed
            with patch.object(runtime,'TrajectoryFollower',self.follower([('tracking',[.35,0,0]),('arrived',[0,0,0])])):
                workers.run()
            self.assertEqual([p for p in a.link.sent if 'mode' in p],[dict(mode=0)])
            self.assertTrue(any(v['v']==[.35,0,0] for v in a.link.velocities))

    def test_manual_takeover_stops_platform_and_cannot_be_reclaimed(self):
        a=self.setup_live();workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        scan=dict(_valid_until=time.monotonic()+.5)
        a.point_navigation._drive(r['run_id'],[.35,0,0],scan)
        self.assertEqual(a.link.v,(.35,0,0))
        with self.assertRaises(ValueError):a.set_velocity(dict(forward=.2))
        self.ready();a.tick(.01);a.set_velocity(dict(forward=.2))
        self.assertEqual(a.link.v,(.2,0,0));self.assertFalse(a.point_navigation.active)
        with self.assertRaises(ValueError):a.point_navigation._drive(r['run_id'],[.35,0,0],scan)
        self.assertEqual(a.link.v,(.2,0,0))

    def test_platform_tick_does_not_consume_native_goal_completion(self):
        a=self.setup_live();workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        a.link.put(1007,1,dict(Value=101,Status=4,ErrorCode=0x2300));a.tick(.01)
        self.assertEqual(a.nav['status'],'running');self.assertEqual(a.nav['completed'],0)

    def test_old_native_cancel_ack_cannot_confirm_new_platform_stop(self):
        a=self.setup_live();request=a.link.command(1004,1,{},'previous native cancel')
        a.cancel_request=request;a.link.put(1004,1,dict(ErrorCode=0),seq=request['seq'])
        workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        a.point_navigation._drive(r['run_id'],[.35,0,0],dict(_valid_until=time.monotonic()+.5))
        self.ready(velocity=(.35,0,0));a.stop();a.tick(.01)
        self.assertEqual(a.nav['status'],'cancelling','old native ACK is not evidence of this platform task stopping')
        self.assertFalse(a.nav.get('cancel_confirmed',False))
        self.ready();a.tick(.01);self.assertEqual(a.nav['status'],'cancelled')

    def test_late_old_preparation_cannot_replace_new_task_planner(self):
        a=self.setup_live();workers=DeferredWorkers();self.start(workers)
        old_entered=threading.Event();release_old=threading.Event();new_scanning=threading.Event();release_new=threading.Event()
        instances=[];old_scan=a.point_navigation._scan
        def delayed_planner(*args,**kwargs):
            obj=GridPlanner(*args,**kwargs);instances.append(obj)
            if len(instances)==1:old_entered.set();release_old.wait(3)
            return obj
        def delayed_scan():new_scanning.set();release_new.wait(3);return old_scan()
        with patch.object(runtime,'GridPlanner',side_effect=delayed_planner),patch.object(a.point_navigation,'_scan',side_effect=delayed_scan):
            old=REAL_THREAD(target=lambda:workers.run(0));old.start()
            new=None
            try:
                self.assertTrue(old_entered.wait(1));a.stop();self.ready();a.tick(.01)
                self.start(workers,x=8.05,y=4.05)
                new=REAL_THREAD(target=lambda:workers.run(1));new.start();self.assertTrue(new_scanning.wait(1))
                expected=instances[1];release_old.set();old.join(1)
                self.assertFalse(old.is_alive());self.assertIs(a.point_navigation.planner,expected)
            finally:
                a.stop();release_old.set();release_new.set();old.join(2)
                if new:new.join(2)

    def test_observation_expiry_cannot_emit_moving_axes(self):
        a=self.setup_live();workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        a.point_navigation._drive(r['run_id'],[.35,0,.7],dict(_valid_until=time.monotonic()-.1))
        self.assertEqual(a.link.v,(0,0,0));self.assertFalse(any(any(v['v']) for v in a.link.velocities))

    def test_deadline_stops_and_blocks_restart_until_fresh_stillness(self):
        a=self.setup_live();workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        a.point_navigation._drive(r['run_id'],[.35,0,0],dict(_valid_until=time.monotonic()+.5))
        future=a.point_navigation.deadline+.01
        with patch.object(runtime.time,'monotonic',return_value=future):
            a.point_navigation.tick()
            self.assertFalse(a.point_navigation.active);self.assertEqual(a.link.v,(0,0,0));self.assertEqual(a.nav['status'],'cancelling')
        with self.assertRaises(ValueError):self.start(workers)
        with patch.object(runtime.time,'monotonic',return_value=future+.1):
            self.ready();a.point_navigation.tick()
        self.assertEqual(a.nav['status'],'paused')

    def test_browser_inactivity_does_not_pause_navigation_but_expired_sensing_stops_axes(self):
        a=self.setup_live();workers=DeferredWorkers();r=self.start(workers)
        a.point_navigation._prepare(r['run_id'],self.map)
        a.point_navigation.lease-=30
        a.point_navigation.check(r['run_id']);a.point_navigation.tick()
        self.assertTrue(a.point_navigation.active)
        a.point_navigation._drive(r['run_id'],[.35,0,0],dict(_valid_until=time.monotonic()+.1))
        self.assertEqual(a.link.v,(.35,0,0))
        a.velocity_deadline=time.monotonic()-.01;a.tick(.01)
        self.assertTrue(a.point_navigation.active);self.assertEqual(a.link.v,(0,0,0))

    def test_new_native_task_during_control_takes_away_platform_ownership(self):
        a=self.setup_live();workers=DeferredWorkers();self.start(workers)
        Base=self.follower([('tracking',[.35,0,0])])
        class NativeTakeover(Base):
            def command(inner,*args,**kwargs):
                a.link.put(1007,1,dict(Value=777,Status=3,ErrorCode=0))
                return super().command(*args,**kwargs)
        with patch.object(runtime,'TrajectoryFollower',NativeTakeover):workers.run()
        self.assertFalse(any(any(v['v']) for v in a.link.velocities))
        self.assertFalse(a.point_navigation.active);self.assertEqual(a.nav['status'],'cancelling')
        self.assertIn('让出控制',a.nav['error'])

    def test_repeated_heading_oscillation_cannot_keep_zero_translation_alive(self):
        clock=[100.];cycles=[0]
        with patch.object(runtime.time,'monotonic',side_effect=lambda:clock[0]):
            a=self.setup_live();workers=DeferredWorkers();self.start(workers)
            original_scan=a.point_navigation._scan
            def oscillating_scan():
                cycles[0]+=1;sign=1 if cycles[0]%2 else -1
                self.ready(yaw=sign*.25,velocity=(0,0,sign*.4))
                if cycles[0]>50:a.done=True  # Bound a broken replay; no real sleep.
                return original_scan()
            def advance(_):
                clock[0]+=1;a.point_navigation.lease=clock[0]
            with patch.object(a.point_navigation,'_scan',side_effect=oscillating_scan),patch.object(runtime.time,'sleep',side_effect=advance),\
                 patch.object(runtime,'TrajectoryFollower',self.follower([('tracking',[0,0,.7]),('tracking',[0,0,-.7])])):
                workers.run()
            self.assertLess(cycles[0],40,'alternating heading without translation must not renew useful progress forever')
            self.assertIn(a.nav['status'],['cancelling','paused'])
            self.assertIn('进展',a.nav.get('error','')+a.nav.get('message',''))

    def test_connection_switch_during_prepare_never_drives_new_connection(self):
        a=self.setup_live();workers=DeferredWorkers();self.start(workers)
        entered=threading.Event();release=threading.Event();oldlink=a.link
        def status(*args,**kwargs):entered.set();release.wait(3);return dict(phase='idle',in_progress=False)
        a.hub.remote_maps=status
        worker=REAL_THREAD(target=lambda:workers.run(0));worker.start()
        try:
            self.assertTrue(entered.wait(1));a.connect(dict(mode='demo'));release.set();worker.join(2)
            self.assertFalse(worker.is_alive());self.assertTrue(a.demo);self.assertEqual(a.nav['status'],'idle')
            self.assertFalse(any(any(v['v']) for v in oldlink.velocities));self.assertEqual(a.velocity,[0,0,0])
        finally:release.set();worker.join(2)

if __name__=='__main__':unittest.main()

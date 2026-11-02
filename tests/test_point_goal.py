# coding: utf-8
import sys,threading,unittest
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_platform as cases
class PointGoalTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    def test_one_goal_uses_robot_planner_without_saving_a_route(self):
        a=self.app;before=list(a.store.value['routes']);result=a.start_goal(dict(map_id='demo',x=0,y=0,yaw=.2))
        self.assertTrue(result['ok']);self.assertEqual(a.store.value['routes'],before)
        self.assertEqual(len(a.nav['route']['points']),1);self.assertTrue(a.nav['route']['temporary'])
        self.assertEqual(a.nav['route']['points'][0]['type'],1)
        self.assertEqual(result['execution'],'native_single_goal')
        self.assertEqual(a.nav['options']['NavMode'],1);self.assertEqual(a.nav['options']['ObsMode'],0)
        self.assertFalse(a.nav['pause_at_point'])
        while a.nav['status']=='running':a._advance()
        self.assertEqual(a.nav['status'],'completed')
        self.assertEqual(a.velocity,[0.,0.,0.])
    def test_goal_validation_happens_before_motion(self):
        for p in [dict(map_id='missing',x=0,y=0),dict(map_id='demo',x=9999,y=0),dict(map_id='demo',x=float('nan'),y=0)]:
            with self.assertRaises(ValueError):self.app.start_goal(p)
        self.assertEqual(self.app.nav['status'],'idle')
    def test_demo_single_goal_follows_preview_without_crossing_known_wall(self):
        import numpy as np,base64,math
        a=self.app;grid=np.full((80,100),254,np.uint8);grid[:2]=0;grid[-2:]=0;grid[:,:2]=0;grid[:,-2:]=0;grid[15:70,49:51]=0
        raw=b'P5\n100 80\n255\n'+grid.tobytes()
        map_=a.import_map(dict(name='detour',pgm=base64.b64encode(raw).decode(),resolution=.1,origin=[0,0,0]))['map']
        a.pose.update(PosX=2,PosY=3,Yaw=0);a.start_goal(dict(map_id=map_['id'],x=8,y=3))
        for _ in range(500):
            a.tick(.1);ix=math.floor(a.pose['PosX']/.1);iy=79-math.floor(a.pose['PosY']/.1)
            self.assertNotEqual(grid[iy,ix],0)
            if a.nav['status']=='completed':break
        self.assertEqual(a.nav['status'],'completed')
        self.assertEqual(len(a.nav['route']['points']),1)

class PlannedGoalTests(unittest.TestCase):
    setUp=cases.PlatformTests.setUp
    tearDown=cases.PlatformTests.tearDown
    def test_preview_never_moves_and_rejects_stale_pose_at_execution(self):
        a=self.app;p=a.plan_goal(dict(map_id='demo',x=0,y=0))
        self.assertEqual(a.nav['status'],'idle');self.assertEqual(a.velocity,[0.,0.,0.])
        a.pose['PosX']+=1
        with self.assertRaisesRegex(ValueError,'位置或朝向'):a.start_goal(dict(map_id='demo',plan_id=p['plan_id']))
        self.assertEqual(a.nav['status'],'idle')
    def test_preview_cannot_execute_against_changed_grid(self):
        a=self.app;p=a.plan_goal(dict(map_id='demo',x=0,y=0))
        source=Path(a.goal_plans[p['plan_id']]['source']);source.write_bytes(source.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError,'地图内容'):a.start_goal(dict(map_id='demo',plan_id=p['plan_id']))
    def test_completed_preview_cannot_cross_a_connection_epoch(self):
        a=self.app;p=a.plan_goal(dict(map_id='demo',x=0,y=0));a.connect(dict(mode='demo'))
        with self.assertRaises(ValueError):a.start_goal(dict(map_id='demo',plan_id=p['plan_id']))
        self.assertEqual(a.nav['status'],'idle')
    def test_late_plan_is_not_cached_after_connection_changes(self):
        from navigation_planner import GridPlanner
        a=self.app;entered=threading.Event();release=threading.Event();errors=[];original=GridPlanner.plan
        def slow(planner,*args,**kwargs):entered.set();release.wait(2);return original(planner,*args,**kwargs)
        def preview():
            try:a.plan_goal(dict(map_id='demo',x=0,y=0))
            except Exception as exc:errors.append(str(exc))
        with patch.object(GridPlanner,'plan',slow):
            worker=threading.Thread(target=preview);worker.start()
            try:
                self.assertTrue(entered.wait(1));a.connect(dict(mode='demo'));release.set();worker.join(3)
                self.assertFalse(worker.is_alive());self.assertEqual(a.goal_plans,{})
                self.assertTrue(any('连接' in error for error in errors))
            finally:release.set();worker.join(3)
    def test_stop_or_manual_input_during_implicit_plan_cannot_start_later(self):
        from navigation_planner import GridPlanner
        original=GridPlanner.plan
        for action in ['stop','velocity','mode']:
            a=self.app;entered=threading.Event();release=threading.Event();errors=[]
            def slow(planner,*args,**kwargs):entered.set();release.wait(2);return original(planner,*args,**kwargs)
            def start():
                try:a.start_goal(dict(map_id='demo',x=0,y=0))
                except Exception as exc:errors.append(str(exc))
            with patch.object(GridPlanner,'plan',slow):
                worker=threading.Thread(target=start);worker.start()
                try:
                    self.assertTrue(entered.wait(1))
                    if action=='stop':a.stop()
                    elif action=='velocity':a.set_velocity(dict(forward=.1))
                    else:a.command('mode_normal')
                    release.set();worker.join(3)
                    self.assertFalse(worker.is_alive());self.assertNotEqual(a.nav['status'],'running')
                    self.assertTrue(any('取消' in error for error in errors))
                finally:release.set();worker.join(3);a.stop()

if __name__=='__main__':unittest.main()

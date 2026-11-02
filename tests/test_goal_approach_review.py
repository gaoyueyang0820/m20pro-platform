"""Synthetic end-to-end geometry review: no private map, transport or robot."""
import math
import unittest
from unittest.mock import patch
import numpy as np
from test_start_pose_recovery import planner,room,observe,body_oracle
from test_start_recovery_runtime import scan_at
from start_pose_recovery import _Geometry
from start_recovery_runtime import plan_with_recovery,TranslationRecovery


def two_sided_scene():
    grid=room();grid[41:43,40:42]=0;grid[45,34:36]=0
    p=planner(grid);start=[3.56,3.745,0.];goal=[2.05,5.]
    scan,loc=observe(p,start)
    return p,start,goal,scan,loc


class GoalApproachReviewTests(unittest.TestCase):
    def test_endpoint_tolerance_and_adjustment_survive_start_recovery(self):
        p=planner(room());start=[.64,4.,0.];goal=[9.6,4.]
        with self.assertRaisesRegex(ValueError,'起点'):p.plan(start,goal,0.,goal_tolerance=.65)
        scan,loc=observe(p,start)
        result=plan_with_recovery(p,start,goal,0.,scan,loc,True,budget=3.)
        self.assertTrue(result['recovery']['required']);self.assertEqual(p.goal_tolerance,.65)
        self.assertTrue(result['goal_adjustment']['applied'])
        self.assertEqual(result['requested_goal'],dict(x=goal[0],y=goal[1]))
        self.assertEqual(result['goal_adjustment']['actual'],result['goal'])
        self.assertEqual(result['goal'],result['recovery']['global_plan']['goal'])
        self.assertLessEqual(math.dist([result['goal']['x'],result['goal']['y']],goal),.65)
        self.assertTrue(body_oracle(p,result['dense_path']))

    def test_two_sided_margin_deficit_uses_verified_diagonal_instead_of_rejected_l_prefix(self):
        p,start,goal,scan,loc=two_sided_scene();geometry=_Geometry(p,start,scan,.8,True)
        self.assertTrue(geometry.physical(start));self.assertFalse(geometry.standard(start))
        result=plan_with_recovery(p,start,goal,0.,scan,loc,True,budget=3.)
        prefix=result['recovery']['prefix'];anchor=np.array(start[:2]);end=np.array([prefix[-1]['x'],prefix[-1]['y']]);direction=end-anchor
        self.assertLess(direction[0],0);self.assertGreater(direction[1],0)
        self.assertIsNotNone(geometry.edge(start,[*end,start[2]],relaxed=True))
        for item in prefix:
            delta=np.array([item['x'],item['y']])-anchor
            self.assertLess(abs(direction[0]*delta[1]-direction[1]*delta[0]),1e-9)
            self.assertAlmostEqual(item['yaw'],start[2])
        self.assertTrue(body_oracle(p,prefix));self.assertTrue(body_oracle(p,result['recovery']['global_plan']['dense_path']))

    def test_unverified_shortcut_does_not_bypass_single_translation_runtime_limit(self):
        p,start,goal,scan,loc=two_sided_scene();original=_Geometry.edge
        def veto_diagonal(self,a,b,relaxed=False):
            if math.dist(a,start)<1e-8 and abs(a[0]-b[0])>1e-8 and abs(a[1]-b[1])>1e-8:return None
            return original(self,a,b,relaxed)
        with patch.object(_Geometry,'edge',veto_diagonal):
            with self.assertRaisesRegex(ValueError,'多段变向'):plan_with_recovery(p,start,goal,0.,scan,loc,True,budget=3.)

    def test_same_runtime_reads_diagonal_movement_and_brakes_before_handoff(self):
        p,start,goal,scan,loc=two_sided_scene();result=plan_with_recovery(p,start,goal,0.,scan,loc,True,budget=3.)
        clock=[50.];pose=np.array(start,float);velocity=np.zeros(2);rows=[];completed=False
        with patch('time.monotonic',side_effect=lambda:clock[0]):
            controller=TranslationRecovery(result['recovery'],now=clock[0])
            for _ in range(90):
                fresh,where=scan_at(p,pose,velocity,clock[0]);command=controller.command(p,fresh,where,standing=True,now=clock[0]);rows.append(command)
                self.assertNotEqual(command['state'],'failed',command)
                if command['state']=='complete':completed=True;break
                target=np.asarray(command['axes'][:2])*2.
                for _ in range(10):
                    delta=target-velocity;norm=np.linalg.norm(delta);rate=1.2 if np.linalg.norm(target)>np.linalg.norm(velocity) else .7
                    if norm:velocity+=delta*min(1.,rate*.01/norm)
                    pose[:2]+=velocity*.01;clock[0]+=.01
                    self.assertTrue(body_oracle(p,[dict(x=pose[0],y=pose[1],yaw=pose[2])]))
            self.assertTrue(completed,rows[-1]);self.assertLess(np.linalg.norm(velocity),.08)
            self.assertLess(pose[0],start[0]);self.assertGreater(pose[1],start[1]);self.assertTrue(p.pose_fits(*pose))
            self.assertTrue(any(row['axes'][0]<0 and row['axes'][1]>0 for row in rows))
            self.assertTrue(any(row['state']=='braking' for row in rows))


if __name__=='__main__':unittest.main()

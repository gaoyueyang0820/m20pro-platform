"""Geometry and feedback replays; no robot, API, ROS or actuator requests.

The closed-loop case uses an ideal kinematic plant with an explicitly chosen
1 m/s per joystick unit gain. It checks geometric decisions and state changes;
it is not hardware speed calibration, contact dynamics or a terrain simulation.
"""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from autonomy_core import Explorer,clearance,disk,cell,RESOLUTION

def scan(pose=(0.,0.,0.),obstacles=(),stamp=1.,floor=None):
    return dict(pose=list(pose),floor=[list(p) for p in disk(4)] if floor is None else floor,
                obstacles=[list(p) for p in obstacles],stamp=stamp)

class RecoveryTests(unittest.TestCase):
    def test_reverse_and_lateral_sweeps_check_the_correct_direction(self):
        s=scan(obstacles=[(-3,0)])
        self.assertTrue(clearance(s,.25,0)[0]);self.assertFalse(clearance(s,-.25,0)[0])
        s=scan(obstacles=[(0,3)])
        self.assertFalse(clearance(s,0,0,side=.25)[0]);self.assertTrue(clearance(s,0,0,side=-.25)[0])
    def test_unknown_side_is_not_a_recovery_corridor(self):
        s=scan(floor=[list(p) for p in disk(4) if p[1]<2])
        self.assertFalse(clearance(s,0,0,side=.25)[0])
        s['floor']=[];self.assertFalse(clearance(s,-.25,0)[0])
    def test_reverse_can_reach_a_goal_in_corridor_without_turning(self):
        e=Explorer(3);s=scan(obstacles=[(x,y) for x in range(-14,15) for y in [-2,2]])
        e.ingest(s);e.path=[(-2,0),(-3,0),(-4,0),(-5,0)];e.goal=(-5,0);e.plan_state='planned'
        f,side,yaw=e.command_axes()
        self.assertLess(f,0);self.assertAlmostEqual(side,0,places=10);self.assertEqual(yaw,0)
        self.assertTrue(clearance(s,f,yaw,side=side)[0])
        self.assertIsNone(e.recovery)
    def test_front_obstacle_uses_clear_side_instead_of_spinning_or_stomping(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)])
        e.ingest(s);e.path=[(2,0),(3,0),(4,0),(5,0)];e.goal=(5,0);e.plan_state='planned'
        f,side,yaw=e.command_axes()
        self.assertEqual(f,0);self.assertNotEqual(side,0);self.assertEqual(yaw,0)
        self.assertTrue(clearance(s,f,yaw,side=side)[0]);self.assertEqual(e.recovery_attempts,1)
    def test_recovery_stops_if_person_enters_selected_side(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)]);e.ingest(s)
        e.path=[(2,0),(3,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        _,side,_=e.command_axes();self.assertNotEqual(side,0)
        e.ingest(scan(obstacles=[(3,0),(0,3 if side>0 else -3)],stamp=1.1))
        self.assertEqual(e.command_axes(),(0.,0.,0.));self.assertIsNone(e.recovery)
    def test_no_feedback_progress_expires_attempt_and_is_bounded(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)]);e.ingest(s)
        e.path=[(2,0),(3,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        for attempt in range(3):
            e.current_scan['stamp']=1+attempt*4
            self.assertNotEqual(e.command_axes(),(0.,0.,0.))
            e.current_scan['stamp']+=3.1
            self.assertEqual(e.command_axes(),(0.,0.,0.))
            e.path=[(2,0),(3,0),(4,0)];e.plan_state='planned'
        self.assertEqual(e.command_axes(),(0.,0.,0.));self.assertEqual(e.recovery_attempts,3)
    def test_recovery_does_not_cross_session_radius(self):
        e=Explorer(2);e.ingest(scan());e.ingest(scan((.7,0,0),stamp=2));e.ingest(scan((1.4,0,0),stamp=3));e.ingest(scan((1.9,0,0),stamp=4))
        e.path=[(12,0),(13,0)];e.goal=(13,0);e.plan_state='planned'
        f,side,yaw=e.command_axes()
        self.assertTrue(e._inside_bound(f,side));self.assertFalse(f>0 and not side)
    def test_complete_coverage_does_not_trigger_unrequested_escape(self):
        e=Explorer(2);e.ingest(scan());e.path=[];e.goal=None;e.plan_state='exhausted'
        self.assertEqual(e.command_axes(),(0.,0.,0.));self.assertEqual(e.recovery_attempts,0)
    def test_reached_intermediate_vantage_can_continue_in_same_decision(self):
        e=Explorer(3);e.ingest(scan((0,0,0)));e.goal=(1,0);e.path=[(1,0)];e.plan_state='planned'
        command=e.command_axes()
        self.assertNotEqual(command,(0.,0.,0.));self.assertIn((1,0),e.visited)
        self.assertTrue(clearance(e.current_scan,command[0],command[2],side=command[1])[0])
    def test_turning_must_settle_before_side_translation(self):
        e=Explorer(3);e.ingest(scan(obstacles=[(3,0)]));e.path=[(2,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        self.assertEqual(e.command_axes(yaw_rate=.8,observation_age=.3),(0.,0.,0.))
    def test_forward_momentum_is_braked_before_side_escape(self):
        e=Explorer(3);s=dict(scan(obstacles=[(3,0)]),_velocity_body=[.5,0],_speed_mps=.5);e.ingest(s)
        e.path=[(2,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        self.assertEqual(e.command_axes(),(0.,0.,0.));self.assertIsNotNone(e.recovery)
        e.ingest(dict(scan(obstacles=[(3,0)],stamp=1.2),_velocity_body=[0,0],_speed_mps=0))
        f,side,yaw=e.command_axes();self.assertEqual(f,0);self.assertNotEqual(side,0);self.assertEqual(yaw,0)
    def test_missing_motion_feedback_does_not_start_a_new_escape(self):
        e=Explorer(3);e.ingest(dict(scan(obstacles=[(3,0)]),_velocity_body=None));e.path=[(2,0),(4,0)];e.goal=(4,0)
        self.assertEqual(e.command_axes(),(0.,0.,0.));self.assertEqual(e.recovery_attempts,0)
    def test_timestamp_reversal_and_nonfinite_feedback_cannot_move(self):
        e=Explorer(3);e.ingest(scan(stamp=10))
        with self.assertRaisesRegex(ValueError,'回退'):e.ingest(scan(stamp=9))
        self.assertFalse(clearance(dict(scan(),_speed_mps=float('nan')),.3,0)[0])
    def test_stationary_pose_noise_is_not_reported_as_walked_distance(self):
        e=Explorer(3)
        for i in range(40):e.ingest(dict(scan((.03*math.sin(i),.03*math.cos(i),0),stamp=i+1),_speed_mps=.04))
        self.assertEqual(e.distance,0.)
        for i in range(1,10):e.ingest(dict(scan((i*.08,0,0),stamp=40+i),_speed_mps=.2))
        self.assertGreater(e.distance,.6)
    def test_closed_loop_sidesteps_then_resumes_progress(self):
        e=Explorer(3);x=y=yaw=previous_turn=0.;seen_side=False;seen_forward=False
        for i in range(70):
            c,sn=math.cos(yaw),math.sin(yaw);ox,oy=.6-x,-y
            obstacle=cell(c*ox+sn*oy,-sn*ox+c*oy)
            s=scan((x,y,yaw),obstacles=[obstacle],stamp=1+i*.2)
            e.ingest(s)
            # A fixed target beyond the obstruction; an escape must not count
            # as reaching it. Subsequent decisions consume actual moved poses.
            if not e.path:e.path=[cell(1.3,0),cell(1.6,0)];e.goal=cell(1.6,0);e.plan_state='planned'
            f,side,turn=e.command_axes(yaw_rate=previous_turn)
            self.assertTrue(clearance(s,f,turn,side=side)[0])
            seen_side=seen_side or abs(side)>.1;seen_forward=seen_forward or f>.1
            x+=(c*f-sn*side)*.2;y+=(sn*f+c*side)*.2;yaw+=turn*.2;previous_turn=turn
            if x>.9:break
        self.assertTrue(seen_side);self.assertTrue(seen_forward);self.assertGreater(x,.9)

if __name__=='__main__':unittest.main()

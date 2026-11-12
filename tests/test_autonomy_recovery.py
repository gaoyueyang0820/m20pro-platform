"""Geometry and feedback replays; no robot, API, ROS or actuator requests.

The closed-loop case uses an ideal kinematic plant with an explicitly chosen
1 m/s per joystick unit gain. It checks geometric decisions and state changes;
it is not hardware speed calibration, contact dynamics or a terrain simulation.
"""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.autonomy_core import Explorer,clearance,disk,cell,RESOLUTION
from m20.robot_geometry import CLEARANCE_HALF_LENGTH, CLEARANCE_HALF_WIDTH

def scan(pose=(0.,0.,0.),obstacles=(),stamp=1.,floor=None):
    return dict(pose=list(pose),floor=[list(p) for p in disk(4)] if floor is None else floor,
                obstacles=[list(p) for p in obstacles],stamp=stamp)


def oblique_passage(stamp, trail_blocked=False):
    hx, hy = CLEARANCE_HALF_LENGTH, CLEARANCE_HALF_WIDTH
    points = [(hx+.38, 0), (-hx-.175, 0), (0, hy+.175), (0, -hy-(.10 if trail_blocked else .175))]
    return dict(scan(obstacles=[cell(x,y) for x,y in points], stamp=stamp),
                geometry_version=2, obstacle_bounds=[[x,y,x,y] for x,y in points],
                _speed_mps=0., _velocity_body=[0.,0.])


def applied(e,**kwargs):
    axes=e.command_axes(**kwargs)
    e.note_recovery_command(axes)
    return axes

class RecoveryTests(unittest.TestCase):
    def exhaust_episode(self,e):
        pose=e.last_pose or (0.,0.,0.);start=(e.last_stamp or 0)+1
        for attempt in range(3):
            e.ingest(dict(scan(pose,obstacles=[(3,0)],stamp=start+attempt*4),_speed_mps=0.,_velocity_body=[0.,0.]))
            e.path=[cell(pose[0]+.5,pose[1]),cell(pose[0]+1.2,pose[1])];e.goal=e.path[-1];e.plan_state='planned'
            self.assertNotEqual(applied(e,),(0.,0.,0.))
            e.ingest(dict(scan(pose,obstacles=[(3,0)],stamp=start+attempt*4+3.1),_speed_mps=0.,_velocity_body=[0.,0.]))
            self.assertEqual(applied(e,),(0.,0.,0.))
        return start+12

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
        f,side,yaw=applied(e,)
        self.assertLess(f,0);self.assertAlmostEqual(side,0,places=10);self.assertEqual(yaw,0)
        self.assertTrue(clearance(s,f,yaw,side=side)[0])
        self.assertIsNone(e.recovery)
    def test_front_obstacle_uses_clear_side_instead_of_spinning_or_stomping(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)])
        e.ingest(s);e.path=[(2,0),(3,0),(4,0),(5,0)];e.goal=(5,0);e.plan_state='planned'
        f,side,yaw=applied(e,)
        self.assertEqual(f,0);self.assertNotEqual(side,0);self.assertEqual(yaw,0)
        self.assertTrue(clearance(s,f,yaw,side=side)[0]);self.assertEqual(e.recovery_attempts,1)
    def test_oblique_passage_survives_disconnected_coarse_footprints(self):
        e = Explorer(8)
        frame = oblique_passage(1.)
        e.ingest(frame)
        for i in range(8):
            heading = i * math.pi/4
            self.assertFalse(clearance(frame, .25*math.cos(heading), 0, side=.25*math.sin(heading))[0])
        self.assertEqual(e.plan(), 'recovering')
        f, side, yaw = applied(e)
        self.assertGreater(f, 0)
        self.assertAlmostEqual(abs(math.atan2(side, f)), math.pi/8)
        self.assertEqual(yaw, 0)
        self.assertTrue(clearance(frame, f, yaw, side=side)[0])

    def test_failed_straight_recovery_leaves_adjacent_oblique_passage_available(self):
        e = Explorer(8)
        e.ingest(scan(stamp=1.))
        e.plan_state = 'recovering'
        self.assertEqual(applied(e), (e.forward_limit, 0., 0.))

        frame = oblique_passage(1.2)
        e.ingest(frame)
        self.assertEqual(applied(e), (0.,0.,0.))
        self.assertEqual(e.recovery_outcomes[-1]['reason'], 'path_blocked')
        self.assertTrue(e._recovery_was_tried(e.forward_limit, 0., 0.))
        self.assertTrue(e._recovery_was_tried(.25, .25*math.tan(math.pi/18), 0.))

        self.assertEqual(e.plan(), 'recovering')
        f, side, yaw = applied(e)
        self.assertAlmostEqual(abs(math.atan2(side, f)), math.pi/8)
        self.assertEqual(yaw, 0.)
        self.assertTrue(clearance(frame, f, yaw, side=side)[0])
        self.assertEqual(e.recovery_attempts, 2)

    def test_measured_approach_is_preferred_until_a_fresh_obstacle_blocks_it(self):
        e = Explorer(8)
        slope = math.tan(math.pi/8)
        for i in range(5):
            x = (4-i)*.16
            e.ingest(dict(scan((x, -x*slope, 0.), stamp=1+i*.4),
                          _speed_mps=math.hypot(.4, .4*slope), _velocity_body=[-.4, .4*slope]))
        e.ingest(oblique_passage(3.))
        self.assertEqual(e.plan(), 'recovering')
        f, side, yaw = applied(e)
        self.assertAlmostEqual(math.atan2(side, f), -math.pi/8)
        self.assertEqual(yaw, 0)

        e.ingest(oblique_passage(3.2, trail_blocked=True))
        self.assertEqual(applied(e), (0.,0.,0.))
        self.assertEqual(e.recovery_outcomes[-1]['reason'], 'path_blocked')
        self.assertEqual(e.plan(), 'recovering')
        f, side, yaw = applied(e)
        self.assertAlmostEqual(math.atan2(side, f), math.pi/8)
        self.assertTrue(clearance(e.current_scan, f, yaw, side=side)[0])

    def test_pose_corrections_without_measured_motion_do_not_create_an_approach(self):
        e = Explorer(3)
        for i in range(8):
            e.ingest(dict(scan(((7-i)*.08, i*.02, 0.), stamp=1+i*.2),
                          _speed_mps=0., _velocity_body=[0.,0.]))
        self.assertEqual(e._traversed_directions(.25), [])

    def test_blocked_rotating_arc_can_recover_with_fixed_heading_side_motion(self):
        e = Explorer(8)
        obstacles = [(3,0),(-3,0),(0,-3)]
        frame = dict(scan(obstacles=obstacles), _speed_mps=0., _velocity_body=[0.,0.])
        e.ingest(frame)
        for f, side, yaw in [(.35,0.,0.),(-.25,0.,0.),(0.,-.25,0.),(0.,0.,.7),(0.,0.,-.7)]:
            self.assertFalse(clearance(frame, f, yaw, side=side)[0])
        self.assertTrue(clearance(frame, 0.,0.,side=.25)[0])
        self.assertTrue(e.request_recovery((0.,.25,.7), '原转弯轨迹受阻'))
        self.assertEqual(applied(e), (0.,0.,0.))  # Wait for a fresh stopped frame.

        e.ingest(dict(frame, stamp=1.2))
        forward,side,yaw=applied(e)
        self.assertEqual((forward,yaw),(0.,0.))
        self.assertGreater(side,.30)
        self.assertEqual(e.recovery_attempts, 1)

        e.ingest(dict(frame, stamp=1.4, obstacles=[*frame['obstacles'], [0,3]]))
        self.assertEqual(applied(e), (0.,0.,0.))
        self.assertEqual(e.recovery_outcomes[-1]['reason'], 'path_blocked')

    def test_recovery_stops_if_person_enters_selected_side(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)]);e.ingest(s)
        e.path=[(2,0),(3,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        _,side,_=applied(e,);self.assertNotEqual(side,0)
        e.ingest(scan(obstacles=[(3,0),(0,3 if side>0 else -3)],stamp=1.1))
        self.assertEqual(applied(e,),(0.,0.,0.));self.assertIsNone(e.recovery)
    def test_no_feedback_progress_expires_attempt_and_is_bounded(self):
        e=Explorer(3);s=scan(obstacles=[(3,0)]);e.ingest(s)
        e.path=[(2,0),(3,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        for attempt in range(3):
            e.current_scan['stamp']=1+attempt*4
            self.assertNotEqual(applied(e,),(0.,0.,0.))
            e.current_scan['stamp']+=3.1
            self.assertEqual(applied(e,),(0.,0.,0.))
            e.path=[(2,0),(3,0),(4,0)];e.plan_state='planned'
        self.assertEqual(applied(e,),(0.,0.,0.));self.assertEqual(e.recovery_attempts,3)
    def test_recovery_does_not_cross_session_radius(self):
        e=Explorer(2);e.ingest(scan());e.ingest(scan((.7,0,0),stamp=2));e.ingest(scan((1.4,0,0),stamp=3));e.ingest(scan((1.9,0,0),stamp=4))
        e.path=[(12,0),(13,0)];e.goal=(13,0);e.plan_state='planned'
        f,side,yaw=applied(e,)
        self.assertTrue(e._inside_bound(f,side));self.assertFalse(f>0 and not side)
    def test_complete_coverage_does_not_trigger_unrequested_escape(self):
        e=Explorer(2);e.ingest(scan());e.path=[];e.goal=None;e.plan_state='exhausted'
        self.assertEqual(applied(e,),(0.,0.,0.));self.assertEqual(e.recovery_attempts,0)
    def test_reached_intermediate_vantage_can_continue_in_same_decision(self):
        e=Explorer(3);e.ingest(scan((0,0,0)));e.goal=(1,0);e.path=[(1,0)];e.plan_state='planned'
        command=applied(e,)
        self.assertNotEqual(command,(0.,0.,0.));self.assertIn((1,0),e.visited)
        self.assertTrue(clearance(e.current_scan,command[0],command[2],side=command[1])[0])
    def test_turning_must_settle_before_side_translation(self):
        e=Explorer(3);e.ingest(scan(obstacles=[(3,0)]));e.path=[(2,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        self.assertEqual(applied(e,yaw_rate=.8,observation_age=.3),(0.,0.,0.))
    def test_forward_momentum_is_braked_before_side_escape(self):
        e=Explorer(3);s=dict(scan(obstacles=[(3,0)]),_velocity_body=[.5,0],_speed_mps=.5);e.ingest(s)
        e.path=[(2,0),(4,0)];e.goal=(4,0);e.plan_state='planned'
        self.assertEqual(applied(e,),(0.,0.,0.));self.assertIsNotNone(e.recovery)
        e.ingest(dict(scan(obstacles=[(3,0)],stamp=1.2),_velocity_body=[0,0],_speed_mps=0))
        f,side,yaw=applied(e,);self.assertEqual(f,0);self.assertNotEqual(side,0);self.assertEqual(yaw,0)
    def test_missing_motion_feedback_does_not_start_a_new_escape(self):
        e=Explorer(3);e.ingest(dict(scan(obstacles=[(3,0)]),_velocity_body=None));e.path=[(2,0),(4,0)];e.goal=(4,0)
        self.assertEqual(applied(e,),(0.,0.,0.));self.assertEqual(e.recovery_attempts,0)
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
            f,side,turn=applied(e,yaw_rate=previous_turn)
            self.assertTrue(clearance(s,f,turn,side=side)[0])
            seen_side=seen_side or abs(side)>.1;seen_forward=seen_forward or f>.1
            x+=(c*f-sn*side)*.2;y+=(sn*f+c*side)*.2;yaw+=turn*.2;previous_turn=turn
            if x>.9:break
        self.assertTrue(seen_side);self.assertTrue(seen_forward);self.assertGreater(x,.9)

    def test_new_observed_translation_rearms_next_obstacle_without_goal_reached(self):
        e=Explorer(4);self.exhaust_episode(e);self.assertEqual(e.recovery_attempts,3)
        stamp=e.last_stamp
        # This is measured movement from an external trajectory follower; it
        # deliberately never calls the old goal-arrived command_axes branch.
        for i in range(1,9):
            e.ingest(dict(scan((i*.1,0,0),stamp=stamp+i*.5),_speed_mps=.2,_velocity_body=[.2,0.]))
        self.assertEqual(e.recovery_attempts,0);self.assertEqual(e.recovery_resets,1)
        self.assertGreaterEqual(e.recovery_measured_distance,.6)
        e.ingest(dict(scan((.8,0,0),obstacles=[(3,0)],stamp=stamp+5),_speed_mps=0.,_velocity_body=[0.,0.]))
        e.path=[cell(1.3,0),cell(2.,0)];e.goal=e.path[-1];e.plan_state='planned'
        self.assertNotEqual(applied(e,),(0.,0.,0.))
        self.assertEqual(e.recovery_attempts,1);self.assertEqual(e.recovery_episode,2)
        self.assertEqual(e.recovery_total_attempts,4)

    def test_stationary_drift_and_missing_feedback_cannot_rearm_episode(self):
        for speed,velocity in [(.03,[.03,0]),(.2,None)]:
            e=Explorer(4);self.exhaust_episode(e);stamp=e.last_stamp
            for i in range(1,16):
                e.ingest(dict(scan((i*.05,0,0),stamp=stamp+i*.2),_speed_mps=speed,_velocity_body=velocity))
            self.assertEqual(e.recovery_attempts,3);self.assertEqual(e.recovery_resets,0)

    def test_back_and_forth_in_previously_visited_space_does_not_rearm(self):
        e=Explorer(4);stamp=1.
        positions=[i*.1 for i in range(9)]+[i*.1 for i in range(7,-9,-1)]+[i*.1 for i in range(-7,1)]
        for x in positions:
            e.ingest(dict(scan((x,0,0),stamp=stamp),_speed_mps=.25,_velocity_body=[.25,0.]));stamp+=.5
        self.exhaust_episode(e);stamp=e.last_stamp
        for x in positions*3:
            stamp+=.5;e.ingest(dict(scan((x,0,0),stamp=stamp),_speed_mps=.25,_velocity_body=[.25,0.]))
        self.assertEqual(e.recovery_attempts,3);self.assertEqual(e.recovery_resets,0)

    def test_goal_changes_and_blocked_replanning_do_not_reset_attempt_budget(self):
        e=Explorer(4);self.exhaust_episode(e)
        for i in range(5):
            e.goal=(i,0);e.visited.add(e.goal);e.path=[]
            applied(e,);self.assertEqual(e.recovery_attempts,3)
            e.note_blocked('same obstruction, different goal');self.assertEqual(e.recovery_attempts,3)

    def test_exhaustion_and_no_corridor_have_different_explanations(self):
        e=Explorer(4);self.exhaust_episode(e);e._recover_axes()
        self.assertEqual(e.recovery_block_reason,'attempts_exhausted');self.assertIn('3次',e.motion_hint)
        blocked=Explorer(4);blocked.ingest(scan(obstacles=[(3,0),(-3,0),(0,3),(0,-3)]));blocked._recover_axes()
        self.assertEqual(blocked.recovery_block_reason,'no_verified_corridor');self.assertNotIn('3次',blocked.motion_hint)

    def test_closed_loop_two_obstacles_get_separate_recovery_episodes(self):
        e=Explorer(4);x=y=yaw=0.;previous=(0.,0.,0.);obstacles=[(.6,0.)];second_added=False
        episodes=set();translated_after_first=False
        for i in range(220):
            if not second_added and x>.95 and e.recovery_resets>=1:
                # A second obstruction is introduced in the new region, then
                # stays fixed. Reusing the old region cannot earn this budget.
                obstacles.append((x+.6*math.cos(yaw),y+.6*math.sin(yaw)));second_added=True
            c,sn=math.cos(yaw),math.sin(yaw)
            body=[cell(c*(ox-x)+sn*(oy-y),-sn*(ox-x)+c*(oy-y)) for ox,oy in obstacles]
            data=dict(scan((x,y,yaw),obstacles=body,stamp=1+i*.2),_speed_mps=math.hypot(*previous[:2]),_velocity_body=list(previous[:2]))
            e.ingest(data)
            if not e.path:e.path=[cell(3.3,0)];e.goal=e.path[-1];e.plan_state='planned'
            f,side,turn=applied(e,yaw_rate=previous[2])
            self.assertTrue(clearance(data,f,turn,side=side)[0])
            if e.recovery is not None:episodes.add(e.recovery_episode)
            x+=(c*f-sn*side)*.2;y+=(sn*f+c*side)*.2;yaw+=turn*.2;previous=(f,side,turn)
            if x>2.6:translated_after_first=True;break
        self.assertTrue(second_added)
        self.assertTrue(translated_after_first,(x,y,e.view()['recovery_episode'],e.motion_hint))
        self.assertGreaterEqual(len(episodes),2)
        self.assertGreaterEqual(e.recovery_resets,1)
        self.assertLessEqual(e.recovery_attempts,3)

if __name__=='__main__':unittest.main()

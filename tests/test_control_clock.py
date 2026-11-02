"""Pure control tick replay; no socket or transport is constructed."""
import sys,threading,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from platform_core import RobotLink,age_of

class ControlClockTests(unittest.TestCase):
    def link(self):
        link=RobotLink.__new__(RobotLink);link.lock=threading.RLock();link.drive_lock=threading.RLock()
        link.inhibit=False;link._vel=dict(f=0,s=0,y=0);link._vel_monotonic=0;link._vel_ts=0;link._zero_until=0;link._observation_deadline=None;link._vel_last_nz=0
        link.records={'1002:6':dict(at=1000,monotonic_at=100,items=dict(BasicStatus=dict(HES=0,MotionState=17,Sleep=0,ControlUsageMode=0)))}
        link.sent=[];link._send_axis=lambda *v:link.sent.append(v);link._log=lambda *a:None
        return link
    def test_wall_clock_rollback_cannot_extend_moving_lease(self):
        link=self.link()
        with patch('platform_core.time.time',return_value=1000),patch('platform_core.time.monotonic',return_value=100):link.set_velocity(.3,0,0)
        with patch('platform_core.time.time',return_value=400),patch('platform_core.time.monotonic',return_value=100.6):link._velocity_tick()
        self.assertEqual(link._vel,dict(f=0,s=0,y=0));self.assertEqual(link.sent[-1],(0,0,0))
    def test_expiry_repeats_zero_for_a_bounded_burst(self):
        link=self.link()
        with patch('platform_core.time.monotonic',return_value=100):link.set_velocity(.3,0,0)
        for now in [100.6,100.65,100.7]:
            with patch('platform_core.time.monotonic',return_value=now):link._velocity_tick()
        self.assertGreaterEqual(link.sent.count((0,0,0)),3)
        count=len(link.sent)
        with patch('platform_core.time.monotonic',return_value=101.1):link._velocity_tick()
        self.assertEqual(len(link.sent),count)
    def test_fresh_commands_cannot_override_stale_robot_feedback(self):
        link=self.link()
        with patch('platform_core.time.time',return_value=500),patch('platform_core.time.monotonic',return_value=102.1):
            link.set_velocity(.3,0,0);link._velocity_tick()
        self.assertEqual(link._vel,dict(f=0,s=0,y=0))
    def test_fresh_lease_is_not_lost_on_forward_wall_clock_jump(self):
        link=self.link()
        with patch('platform_core.time.monotonic',return_value=100):link.set_velocity(.3,0,0)
        with patch('platform_core.time.time',return_value=90000),patch('platform_core.time.monotonic',return_value=100.2):link._velocity_tick()
        self.assertEqual(link.sent[-1],(.3,0,0))
    def test_unknown_or_future_legacy_timestamp_is_not_fresh(self):
        with patch('platform_core.time.time',return_value=500):self.assertEqual(age_of(dict(at=1000)),float('inf'))
        self.assertEqual(age_of({}),float('inf'))
    def test_pose_polling_is_faster_than_housekeeping_without_motion_commands(self):
        link=self.link();link._stop=False;clock=[100.];queries=[]
        link.apdu=lambda t,c:(t,c);link.send=lambda q:queries.append(q)
        def sleep(dt):
            clock[0]+=dt
            if clock[0]>100.81:link._stop=True
        with patch('platform_core.time.monotonic',side_effect=lambda:clock[0]),patch('platform_core.time.sleep',side_effect=sleep):link._poll()
        self.assertEqual(queries.count((1007,2)),5)
        for request in [(1007,1),(2002,1),(1101,7)]:self.assertEqual(queries.count(request),1)
        self.assertTrue(all(t in [1007,2002,1101] for t,c in queries))

if __name__=='__main__':unittest.main()

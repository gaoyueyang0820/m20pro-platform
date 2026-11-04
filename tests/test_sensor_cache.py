import sys,threading,time,unittest
from pathlib import Path
from collections import deque
from types import SimpleNamespace as Obj
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge.ros_sensors import RosSensors

class SensorCacheTests(unittest.TestCase):
    def reader(self,age=.2):
        r=RosSensors.__new__(RosSensors);r.lock=threading.RLock()
        r.ground_ready=threading.Condition(r.lock)
        r.autonomy_scan=dict(received_monotonic=time.monotonic()-age,source_age=.1,stamp=123)
        return r
    def test_late_next_packet_retains_fresh_scan_without_renewing_it(self):
        r=self.reader();before=dict(r.autonomy_scan)
        r.ground_failure(TimeoutError('late packet'),temporal=True)
        self.assertEqual(r.autonomy_scan,before)
    def test_expired_cache_cannot_survive_late_next_packet(self):
        r=self.reader(.6);r.ground_failure(TimeoutError('late packet'),temporal=True)
        self.assertIsNone(r.autonomy_scan)
    def test_geometry_failure_invalidates_even_fresh_scan(self):
        r=self.reader();r.ground_failure(ValueError('nonlevel floor'))
        self.assertIsNone(r.autonomy_scan)

    def localized_reader(self):
        r=self.reader(.02);r.ground_event=threading.Event();r.odom_history=deque(maxlen=40)
        r.ensure_topic=lambda _:None;r.expire_clouds=lambda:None;r.normal_location_status=0
        r.location_quality=(0,time.monotonic());r.autonomy_scan.update(floor=[[4,0]],obstacles=[],geometry_version=2,stamp=123.)
        return r

    def add_odom(self,r,stamp,x):
        m=Obj(header=Obj(frame_id='map',stamp=Obj(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),child_frame_id='',
              pose=Obj(pose=Obj(position=Obj(x=x,y=2.,z=.1),orientation=Obj(x=0.,y=0.,z=0.,w=1.))))
        r.odom_update(m)

    def test_localized_scan_uses_timestamp_matched_real_odom_callback(self):
        r=self.localized_reader();self.add_odom(r,123.,1.);self.add_odom(r,123.3,9.)
        data=r.autonomy(False)
        self.assertEqual(data['pose'],[1.,2.,0.]);self.assertEqual(data['pose_source'],'/ODOM')
        self.assertEqual(data['floor'],[[4,0]]);self.assertEqual(data['geometry_version'],2)

    def test_unlocalized_follow_keeps_scan_but_never_fabricates_map_pose(self):
        r=self.localized_reader();self.add_odom(r,123.,1.);r.location_quality=(5,time.monotonic())
        data=r.autonomy(False);self.assertNotIn('pose',data);self.assertIn('pose_error',data)

    def test_newest_unmatched_odom_is_not_attached_to_older_scan(self):
        r=self.localized_reader();self.add_odom(r,123.5,1.)
        self.assertNotIn('pose',r.autonomy(False))

    def virtual_reader(self,age=.50,stamp=123.):
        """Exercise the actual API producer with scheduled completions, no ROS."""
        clock=Obj(value=100.,waits=[],events=[])
        clock.monotonic=lambda:clock.value
        r=RosSensors.__new__(RosSensors);r.lock=threading.RLock();r.ground_event=threading.Event()
        r.ensure_topic=lambda _:None;r.expire_clouds=lambda:None;r.odom_history=deque(maxlen=40)
        r.autonomy_error=None;r.location_quality=None
        def publish(value,new_age):
            r.autonomy_scan=dict(stamp=value,received_monotonic=clock.value,source_age=new_age,
                                 floor=[[4,0]],obstacles=[],processing_ms=180.)
        publish(stamp,age)
        def wait(timeout):
            clock.waits.append(timeout)
            if clock.events and clock.events[0][0]<=clock.value+timeout:
                at,callback=clock.events.pop(0);clock.value=at;callback()
            else:clock.value+=timeout
        r.ground_ready=Obj(wait=wait)
        return r,clock,publish

    def test_new_processed_scan_returns_at_completion_without_fixed_220ms_wait(self):
        r,clock,publish=self.virtual_reader()
        clock.events=[(100.03,lambda:publish(123.1,.48))]
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False)
        self.assertAlmostEqual(data['server_elapsed_s'],.03)
        self.assertAlmostEqual(data['age'],.48)
        self.assertEqual(data['stamp'],123.1);self.assertEqual(len(clock.waits),1)

    def test_unseen_valid_frame_returns_now_instead_of_waiting_until_expired(self):
        r,clock,publish=self.virtual_reader(age=.50,stamp=123.)
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False,after_stamp=122.)
        self.assertEqual(clock.waits,[]);self.assertEqual(data['server_elapsed_s'],0.)
        self.assertAlmostEqual(data['age'],.50);self.assertEqual(data['stamp'],123.)

    def test_seen_frame_waits_for_a_successor_instead_of_republishing_duplicates(self):
        r,clock,publish=self.virtual_reader(age=.1)
        clock.events=[(100.03,lambda:publish(123.1,.12))]
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False,after_stamp=123.)
        self.assertAlmostEqual(data['server_elapsed_s'],.03);self.assertEqual(data['stamp'],123.1)
        self.assertAlmostEqual(data['age'],.12)

    def test_no_successor_does_not_refresh_the_old_source_age(self):
        r,clock,publish=self.virtual_reader(age=.1)
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False,after_stamp=123.)
        self.assertAlmostEqual(data['server_elapsed_s'],.22);self.assertAlmostEqual(data['age'],.32)
        self.assertEqual(data['stamp'],123.)

    def test_duplicate_stamp_notification_does_not_claim_a_new_scan(self):
        r,clock,publish=self.virtual_reader(age=.425)
        clock.events=[(100.03,lambda:publish(123.,.455)),(100.07,lambda:publish(123.,.495))]
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False)
        self.assertAlmostEqual(data['server_elapsed_s'],.22)
        self.assertAlmostEqual(data['age'],.645)
        self.assertEqual(data['stamp'],123.);self.assertEqual(len(clock.waits),3)

    def test_reordered_stamp_does_not_end_wait_for_a_new_scan(self):
        r,clock,publish=self.virtual_reader(age=.43)
        clock.events=[(100.03,lambda:publish(122.9,.50)),(100.10,lambda:publish(123.1,.46))]
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False)
        self.assertAlmostEqual(data['server_elapsed_s'],.10)
        self.assertEqual(data['stamp'],123.1)

    def test_newer_but_expired_scan_is_still_rejected(self):
        r,clock,publish=self.virtual_reader()
        clock.events=[(100.03,lambda:publish(123.1,.66))]
        with patch('bridge.ros_sensors.time',clock):
            with self.assertRaisesRegex(ValueError,'雷达更新中断'):r.autonomy(False)
        self.assertAlmostEqual(clock.value,100.22)

    def test_cold_start_returns_first_usable_processed_scan_immediately(self):
        r,clock,publish=self.virtual_reader();r.autonomy_scan=None
        clock.events=[(100.06,lambda:publish(123.1,.49))]
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False)
        self.assertAlmostEqual(data['server_elapsed_s'],.06)
        self.assertAlmostEqual(data['age'],.49)

    def test_already_fresh_cache_keeps_original_age_and_does_not_wait(self):
        r,clock,publish=self.virtual_reader(age=.31)
        with patch('bridge.ros_sensors.time',clock):data=r.autonomy(False)
        self.assertEqual(clock.waits,[]);self.assertAlmostEqual(data['age'],.31)
        self.assertEqual(data['server_elapsed_s'],0.)

if __name__=='__main__':unittest.main()

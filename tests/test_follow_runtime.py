"""Real follow loop with synthetic transport, detections and robot feedback."""
import copy
import threading
import time
import unittest
from unittest.mock import patch
import test_platform as cases
from test_autonomy_observation_runtime import Link
from autonomy_core import PersonLock, disk


class FollowRuntimeTests(unittest.TestCase):
    setUp = cases.PlatformTests.setUp

    def tearDown(self):
        self.app.done = True
        with self.app.lock:
            self.app.autonomy.pause('offline test ended')
        if hasattr(self, 'worker'):
            self.worker.join(1.)
        p = self.app.autonomy.observation_pump
        if p:
            p.cancel(); p.join(1.)
        self.app.hub.close()
        cases.PlatformTests.tearDown(self)

    def configure(self):
        app = self.app; app.demo = False; app.link = Link(app); app.mapping = False
        app.basic.update(MotionState=17, ControlUsageMode=0, Sleep=0, HES=0)
        app.config['nos'] = 'offline-follow'
        app.hub.connections.value['targets']['nos']['enabled'] = False
        app.hub.connections.value['sensor_source'] = 'nos'; app.hub.sdk_mode = dict(state='normal')
        a = app.autonomy; a.mode = 'follow'; a.active = True; a.run_id = 'selected-person-session'
        a.state = 'preparing'; a.context = app.hub.context(); a.capture_id = None
        a.started = a.lease = time.monotonic(); a.deadline = a.started+10
        self.box = [.41, .25, .59, .65]
        a.target = PersonLock(self.box); a.target.reference_height = .8
        self.obstructed = True; self.lost = False; self.fixed_vision = False
        self.captured = time.monotonic(); self.stamp = 100.
        read_feedback = app.link.record
        def feedback(t, c):
            row = read_feedback(t, c)
            if (t, c) == (1002, 4):
                # With residual motion, missing support ahead must cause a
                # stop; a tiny stationary turn may fit in the current stance.
                row['items']['MotionStatus']['LinearX'] = .22 if self.obstructed else 0.
            return row
        app.link.record = feedback
        def vision():
            age = time.monotonic()-self.captured if self.fixed_vision else .01
            return dict(fresh=age<.65, age=age, sequence=1 if self.fixed_vision else int(time.monotonic()*20),
                        detections=[] if self.lost else [dict(box=self.box, confidence=.98)])
        a.vision.snapshot = vision
        return a

    def transport(self, *args, **kwargs):
        time.sleep(.035); self.stamp += .05
        return dict(protocol=1, stamp=self.stamp, frame_id='base_link', age=.05,
                    floor=[] if self.obstructed else [list(p) for p in disk(3)], obstacles=[])

    def start(self):
        a = self.app.autonomy
        self.worker = threading.Thread(target=a._run, args=(a.run_id, None, 10), daemon=True)
        self.worker.start()

    def wait_for(self, predicate, seconds=2.):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            with self.app.lock: self.app.autonomy.lease = time.monotonic()
            if predicate(): return True
            time.sleep(.01)
        return bool(predicate())

    def test_missing_ground_waits_and_new_scan_resumes_same_selected_person(self):
        a = self.configure()
        with patch('platform_hub.rpc', side_effect=self.transport):
            self.start()
            self.assertTrue(self.wait_for(lambda:a.follow_detail is not None), a.message)
            self.assertTrue(a.active)
            self.assertFalse(a.follow_detail['motion_allowed'])
            self.assertTrue(a.follow_detail['missing']['count'])
            self.assertFalse(any(any(r['axes']) for r in self.app.link.output))
            self.obstructed = False
            self.assertTrue(self.wait_for(lambda:any(any(r['axes']) for r in self.app.link.output)), a.message)
            self.assertEqual(a.run_id, 'selected-person-session')
            self.assertTrue(a.active)
            self.assertEqual(a.target.box, self.box)
            with self.app.lock: a.pause('done')

    def test_fresh_radar_does_not_renew_an_old_vision_frame(self):
        a = self.configure(); self.obstructed = False; self.fixed_vision = True
        with patch('platform_hub.rpc', side_effect=self.transport):
            self.start()
            self.assertTrue(self.wait_for(lambda:not a.active), a.message)
            moving = [r for r in self.app.link.output if any(r['axes'])]
            self.assertTrue(moving)
            self.assertTrue(all(r['deadline']<=self.captured+.65+.003 for r in moving))
            self.assertFalse(any(any(r['axes']) and r['at']>=r['deadline'] for r in moving))
            self.assertEqual(self.app.velocity, [0., 0., 0.])

    def test_identity_loss_still_stops_and_never_reselects(self):
        a = self.configure(); self.obstructed = False
        with patch('platform_hub.rpc', side_effect=self.transport):
            self.start()
            self.assertTrue(self.wait_for(lambda:any(any(r['axes']) for r in self.app.link.output)), a.message)
            self.lost = True
            self.assertTrue(self.wait_for(lambda:not a.active), a.message)
            self.assertEqual(self.app.velocity, [0., 0., 0.])
            self.assertEqual(a.state, 'paused')

    def test_wall_clock_rollback_does_not_freshen_old_motion_feedback(self):
        a = self.configure(); self.obstructed = False
        original = self.app.link.record
        def stale_record(t, c):
            row = original(t, c)
            if (t, c) == (1002, 4):
                row.update(at=time.time()+60, monotonic_at=time.monotonic()-2)
            return row
        self.app.link.record = stale_record
        with patch('platform_hub.rpc', side_effect=self.transport):
            self.start()
            self.assertTrue(self.wait_for(lambda:a.follow_detail is not None), a.message)
            self.assertEqual(a.follow_detail['state'], 'waiting_feedback')
            self.assertFalse(any(any(r['axes']) for r in self.app.link.output))
            with self.app.lock: a.pause('done')


if __name__ == '__main__':
    unittest.main()

import unittest

from m20.motion_response import MotionResponse, MotionTracker


class MotionResponseTests(unittest.TestCase):
    def test_repeated_measured_motion_improves_later_prediction(self):
        model=MotionResponse()
        for i in range(24):
            stamp=1.+i*.2
            model.observe(0,((.35,0.,0.),.5,stamp-.01),stamp,(.21,0.,0.))
        restored=MotionResponse(model.record())
        self.assertEqual(restored.status()['modes']['0']['forward']['samples'],24)
        learned=restored.gains(0)['forward']
        self.assertLess(abs(.35*learned-.21),.02)
        self.assertEqual(restored.gains(2),{})
        self.assertNotIn('backward',restored.gains(0))

    def test_stationary_repeated_mixed_or_old_feedback_does_not_train(self):
        model=MotionResponse()
        for i in range(20):
            stamp=1.+i*.2
            model.observe(0,((.35,0.,0.),stamp-.1,stamp),stamp,(.21,0.,0.))
            model.observe(0,((.35,0.,0.),.5,stamp-.2),stamp,(.21,0.,0.))
            model.observe(0,((.35,0.,0.),.5,stamp),stamp,(0.,0.,0.))
            model.observe(0,((.35,.25,0.),.5,stamp),stamp,(.21,.15,0.))
            model.observe(0,((.35,0.,0.),.5,stamp),stamp,(-.21,0.,0.))
        self.assertEqual(model.gains(0),{})
        self.assertFalse(model.dirty)
        for _ in range(24):
            model.observe(0,((0.,0.,.7),.5,1.),1.,(0.,0.,.42))
        self.assertEqual(model.status()['modes']['0']['turn_left']['samples'],1)
        self.assertEqual(model.gains(0),{})

    def test_version_one_record_upgrades_without_losing_the_gain(self):
        saved=dict(version=1,profiles=dict({'0':dict(forward=[5,1.1,.1,.2])}))
        model=MotionResponse(saved)
        record=model.record()
        self.assertEqual(record['version'],2)
        row=record['profiles']['0']['forward']
        self.assertEqual(row['count'],5)
        self.assertAlmostEqual(row['gain'],1.1)
        self.assertEqual(row['start'],{})
        self.assertEqual(row['latency'],[0,0.,0.])

    def test_start_buckets_only_raise_the_deadband_after_enough_attempts(self):
        model=MotionResponse()
        for _ in range(4):
            model.observe_start(0,'forward',.20,False)
        self.assertEqual(model.deadband(0,'forward'),.20)
        for _ in range(6):
            model.observe_start(0,'forward',.20,False)
        # .20 is now known to fail reliably, but no higher bucket has been
        # measured, so the conservative floor stays.
        self.assertEqual(model.deadband(0,'forward'),.20)
        for _ in range(10):
            model.observe_start(0,'forward',.28,True)
        self.assertEqual(model.deadband(0,'forward'),.28)
        self.assertEqual(model.deadband(0,'backward'),.20)

    def test_latency_and_brake_queries_need_enough_consistent_samples(self):
        model=MotionResponse()
        for _ in range(4):
            model.observe_latency(0,'forward',.20)
        self.assertEqual(model.latency(0),.15)
        model.observe_latency(0,'forward',.20)
        self.assertAlmostEqual(model.latency(0),.20,places=2)
        for _ in range(5):
            model.observe_brake(0,'forward',.40,.30,.10)
        self.assertEqual(model.braking_decel(0),.5)
        model.observe_brake(0,'forward',.40,.30,.10)
        self.assertAlmostEqual(model.braking_decel(0),1.0,places=2)
        # Implausible decay never trains.
        for _ in range(20):
            model.observe_brake(0,'forward',.10,.09,.10)
        self.assertAlmostEqual(model.braking_decel(0),1.0,places=2)
        self.assertEqual(model.latency(2),.15)

    def test_tracker_learns_a_start_latency_and_a_brake_sample(self):
        model=MotionResponse()
        tracker=MotionTracker(model,0)
        command=((.35,0.,0.),1.0,1.35)
        for stamp,feedback in [(1.05,(0.,0.,0.)),(1.10,(0.,0.,0.)),(1.20,(.20,0.,0.)),
                               (1.40,(.35,0.,0.)),(1.55,(.20,0.,0.)),(1.75,(0.,0.,0.))]:
            tracker.update(command if stamp<1.75 else None,stamp,feedback)
        row=model._get(0,'forward')
        self.assertEqual(row['latency'][0],1)
        self.assertAlmostEqual(row['latency'][1],.20,places=2)
        self.assertEqual(row['start']['0.36'],[1,1])
        self.assertEqual(row['brake'][0],1)
        self.assertAlmostEqual(row['brake'][1],1.0,places=2)

    def test_tracker_never_trains_on_stale_or_mixed_feedback(self):
        model=MotionResponse()
        tracker=MotionTracker(model,0)
        command=((.35,.20,0.),1.0,1.35)
        for stamp in (1.10,1.10,1.05):
            tracker.update(command,stamp,(.21,.12,0.))
        self.assertEqual(tracker.last_stamp,1.10)
        self.assertEqual(model.status()['modes'],{})
        # A command that never moves records exactly one failed start attempt
        # when its window closes; it never invents a gain sample.
        for stamp in (1.20,1.70,1.75):
            tracker.update(((0.,0.,.7),1.20,1.75),stamp,(0.,0.,0.))
        row=model._get(0,'turn_left')
        self.assertEqual(row['start']['0.5'],[1,0])
        self.assertEqual(row['count'],0)
        self.assertEqual(model.gains(0),{})

    def test_track_feeds_one_shared_tracker_from_a_paired_scan(self):
        from types import SimpleNamespace
        from m20.motion_response import track

        class Link:
            def axis_run(self):
                return ((.35, 0., 0.), 1.0, 1.35)

        model = MotionResponse()
        app = SimpleNamespace(motion_response=model, link=Link(),
                              basic=dict(ControlUsageMode=0))
        owner = SimpleNamespace()
        scan = dict(_velocity_body=[.21, 0.], _motion_feedback_at=1.40)
        first = track(owner, app, scan, 0.)
        self.assertIsNotNone(first)
        self.assertIs(owner.motion_tracker, first)
        self.assertIs(first.model, model)
        self.assertEqual(first.mode, 0)
        self.assertIsNone(track(owner, app, scan, 0., ready=False))
        self.assertIsNone(track(SimpleNamespace(), SimpleNamespace(), scan, 0.))

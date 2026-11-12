import unittest

from m20.motion_response import MotionResponse


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

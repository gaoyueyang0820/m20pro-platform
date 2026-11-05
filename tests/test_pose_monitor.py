"""Display-only regressions; no robot, server, network or control requests.

Synthetic cases are portable. One optional private replay loads recorded poses
from an ignored fixture only inside its test method; missing data is reported
as skipped, never replaced with invented field evidence.
"""
from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.pose_monitor import PoseMonitor

# Synthetic velocity cases; recorded poses are loaded only by the optional test.
REAL_STATIC_MOTION=(-.045,.001,0.)
MOVING_WITH_ZERO_COMMAND=[[.65,0.,.05],[.35,.09,-1.04]]
PRIVATE_POSE_FIXTURE=Path(__file__).parent/'fixtures/runtime_replay/pose_status_drift_20261102.json'


def pose(x=1.,y=2.,yaw=0.,location=0,confidence=.95):
    return dict(PosX=x,PosY=y,PosZ=.05,Yaw=yaw,Location=location,Confidence=confidence)


def motion(values=REAL_STATIC_MOTION):
    return dict(zip(['LinearX','LinearY','OmegaZ'],values))


class PoseMonitorTests(unittest.TestCase):
    def feed(self,monitor,p=None,now=100.,command=(0,0,0),measured=None,pose_age=.02,motion_age=.01,epoch='map-1'):
        return monitor.update(p or pose(),command,motion() if measured is None else measured,
                              pose_age=pose_age,motion_age=motion_age,now=now,relocation_epoch=epoch)
    def warm(self,monitor,p=None):
        result=None
        for i in range(17):result=self.feed(monitor,p,100.+i*.1)
        self.assertTrue(result['stationary']);return result

    def test_actual_unready_pose_series_is_never_frozen_into_a_trusted_marker(self):
        if not PRIVATE_POSE_FIXTURE.is_file():self.skipTest('Private recorded pose fixture is not distributed')
        import json
        REAL_POSE_ROWS=json.loads(PRIVATE_POSE_FIXTURE.read_text())['REAL_POSE_ROWS']
        self.assertEqual(len(REAL_POSE_ROWS),20)
        monitor=PoseMonitor()
        for i,row in enumerate(REAL_POSE_ROWS):
            x,y,z,yaw,location,confidence,vx,vy,wz=row
            raw=dict(PosX=x,PosY=y,PosZ=z,Yaw=yaw,Location=location,Confidence=confidence)
            out=self.feed(monitor,raw,100+i*.25,measured=motion((vx,vy,wz)))
            self.assertIsNone(out['display_pose']);self.assertFalse(out['health']['display_usable'])
            self.assertEqual(out['health']['status'],'localization_invalid');self.assertFalse(out['health']['filtered'])
            self.assertEqual(out['raw_pose'],raw)

    def test_velocity_bias_can_be_stationary_but_zero_command_alone_cannot(self):
        out=self.warm(PoseMonitor());self.assertLess(out['health']['measured_speed_m_s'],.08)
        self.assertTrue(out['stationary'])
        self.assertGreater(len(MOVING_WITH_ZERO_COMMAND),0)
        for values in MOVING_WITH_ZERO_COMMAND:
            monitor=PoseMonitor();self.warm(monitor)
            out=self.feed(monitor,pose(x=1.1),101.7,measured=motion(values))
            self.assertFalse(out['stationary']);self.assertFalse(out['health']['filtered'])
            self.assertEqual(out['display_pose']['PosX'],1.1)

    def test_centimetre_noise_is_stabilized_after_settling_without_changing_raw(self):
        monitor=PoseMonitor();outputs=[];inputs=[]
        for i in range(100):
            raw=pose(x=1+.018*math.sin(i*1.7),y=2+.017*math.cos(i*1.3),yaw=.003*math.sin(i))
            out=self.feed(monitor,raw,100+i*.1)
            self.assertEqual(out['raw_pose'],raw)
            if i>25:outputs.append(out['display_pose']['PosX']);inputs.append(raw['PosX'])
        self.assertLess(max(outputs)-min(outputs),(max(inputs)-min(inputs))*.6)
        self.assertTrue(out['stationary']);self.assertTrue(out['health']['filtered'])
        self.assertEqual(out['health']['control_pose_source'],'raw_pose')

    def test_cumulative_drift_is_visible_and_warned_instead_of_held_forever(self):
        monitor=PoseMonitor();self.warm(monitor);statuses=[]
        for i in range(1,36):
            raw=pose(x=1+i*.006);out=self.feed(monitor,raw,101.6+i*.2);statuses.append(out['health']['status'])
        self.assertIn('drift_suspected',statuses);self.assertEqual(out['health']['status'],'drift_suspected')
        self.assertEqual(out['display_pose'],raw);self.assertFalse(out['health']['filtered'])
        self.assertGreater(out['health']['anchor_displacement_m'],.15)

    def test_sudden_jump_is_not_hidden_or_silently_adopted_as_stationary_truth(self):
        monitor=PoseMonitor();self.warm(monitor);raw=pose(x=2.,y=2.4)
        out=self.feed(monitor,raw,101.7)
        self.assertEqual(out['health']['status'],'pose_jump');self.assertEqual(out['display_pose'],raw)
        self.assertGreater(out['health']['jump']['distance_m'],1.)
        self.assertFalse(out['health']['last_known']['usable'])
        for i in range(1,31):out=self.feed(monitor,raw,101.7+i*.1)
        self.assertEqual(out['health']['status'],'drift_suspected')
        self.assertEqual(out['display_pose'],raw)

    def test_starting_motion_immediately_bypasses_marker_filter(self):
        monitor=PoseMonitor();self.warm(monitor)
        raw=pose(x=1.06,y=2.02,yaw=.02)
        out=self.feed(monitor,raw,101.7,command=(.35,0,0),measured=motion((.3,0,.05)))
        self.assertFalse(out['stationary']);self.assertFalse(out['health']['filtered']);self.assertEqual(out['display_pose'],raw)
        # Even before measured speed responds, a new nonzero command ends hold.
        out=self.feed(monitor,pose(x=1.07),101.8,command=(.35,0,0))
        self.assertEqual(out['display_pose']['PosX'],1.07);self.assertFalse(out['stationary'])

    def test_zero_command_during_coasting_does_not_reenable_stationary_filter(self):
        monitor=PoseMonitor();self.warm(monitor)
        for i in range(20):
            out=self.feed(monitor,pose(x=1+i*.02),101.7+i*.1,measured=motion((.2,0,0)))
            self.assertFalse(out['stationary']);self.assertFalse(out['health']['filtered'])

    def test_real_movement_is_not_a_jump_when_consistent_with_measured_speed(self):
        monitor=PoseMonitor();self.warm(monitor)
        out=self.feed(monitor,pose(x=1.5),102.1,command=(.5,0,0),measured=motion((1.,0,0)))
        self.assertNotEqual(out['health']['status'],'pose_jump');self.assertEqual(out['display_pose']['PosX'],1.5)

    def test_invalid_location_exposes_only_unusable_last_known_until_recovered(self):
        monitor=PoseMonitor();previous=self.warm(monitor)['display_pose']
        out=self.feed(monitor,pose(x=4,location=1,confidence=.99),101.7)
        self.assertIsNone(out['display_pose']);self.assertEqual(out['health']['status'],'localization_invalid')
        self.assertEqual(out['health']['last_known']['pose'],previous);self.assertFalse(out['health']['last_known']['usable'])
        first=self.feed(monitor,pose(x=4),101.8);self.assertIsNone(first['display_pose'])
        states=[]
        for i in range(1,8):out=self.feed(monitor,pose(x=4),101.8+i*.2);states.append(out['health']['status'])
        self.assertIn('recovered',states);self.assertEqual(out['display_pose']['PosX'],4)
        self.assertNotIn('pose_jump',states)

    def test_epoch_change_never_reuses_old_display_even_if_location_reports_zero(self):
        monitor=PoseMonitor();self.warm(monitor)
        out=self.feed(monitor,pose(x=7,yaw=1),101.7,epoch='map-2')
        self.assertIsNone(out['display_pose']);self.assertTrue(out['health']['epoch_changed'])
        self.assertEqual(out['health']['status'],'relocalizing');self.assertFalse(out['health']['last_known']['usable'])
        for i in range(1,8):out=self.feed(monitor,pose(x=7,yaw=1),101.7+i*.2,epoch='map-2')
        self.assertEqual(out['display_pose']['PosX'],7);self.assertEqual(out['health']['relocation_epoch'],'map-2')

    def test_pose_and_motion_staleness_are_reported_separately(self):
        monitor=PoseMonitor();self.warm(monitor)
        raw=pose(x=1.03);out=self.feed(monitor,raw,101.7,motion_age=2.)
        self.assertFalse(out['stationary']);self.assertEqual(out['display_pose'],raw)
        self.assertEqual(out['health']['status'],'motion_unavailable')
        out=self.feed(monitor,pose(),102.2,pose_age=3.)
        self.assertIsNone(out['display_pose']);self.assertEqual(out['health']['status'],'pose_stale')
        self.assertFalse(out['health']['last_known']['usable'])

    def test_yaw_wrap_does_not_average_to_zero_or_raise_false_jump(self):
        monitor=PoseMonitor()
        for i in range(60):
            yaw=math.atan2(math.sin(math.pi+.003*math.sin(i)),math.cos(math.pi+.003*math.sin(i)))
            out=self.feed(monitor,pose(yaw=yaw),100+i*.1)
            if out['display_pose']:
                self.assertLess(abs(abs(out['display_pose']['Yaw'])-math.pi),.01)
                self.assertNotEqual(out['health']['status'],'pose_jump')

    def test_same_old_sample_cannot_be_renewed_forever_by_polling(self):
        monitor=PoseMonitor();self.warm(monitor)
        for i in range(1,27):out=self.feed(monitor,pose(),101.6+i*.1,pose_age=.02+i*.1)
        self.assertIsNone(out['display_pose']);self.assertEqual(out['health']['status'],'pose_stale')
        self.assertGreater(out['health']['last_known']['age_s'],2.5)

    def test_feedback_gap_resets_stationary_confirmation(self):
        monitor=PoseMonitor();self.warm(monitor)
        out=self.feed(monitor,pose(x=2),110.)
        self.assertFalse(out['stationary']);self.assertFalse(out['health']['filtered'])
        self.assertEqual(out['display_pose']['PosX'],2)

    def test_outputs_are_independent_copies_and_never_rewrite_input_pose(self):
        monitor=PoseMonitor();raw=pose();raw['diagnostic']={'source':'fixture'};before=deepcopy(raw)
        out=self.warm(monitor,raw)
        self.assertEqual(raw,before);self.assertEqual(out['raw_pose'],before)
        out['raw_pose']['PosX']=99;out['display_pose']['diagnostic']['source']='changed'
        self.assertEqual(raw,before)
        later=self.feed(monitor,pose(location=1),101.7)
        self.assertEqual(later['health']['last_known']['pose']['diagnostic']['source'],'fixture')

    def test_invalid_numbers_and_explicit_confidence_requirement_are_not_smoothed(self):
        for key,value in [('PosX',float('nan')),('Yaw',float('inf')),('Confidence',float('nan'))]:
            monitor=PoseMonitor();self.warm(monitor);raw=pose();raw[key]=value
            out=self.feed(monitor,raw,101.7);self.assertIsNone(out['display_pose']);self.assertFalse(out['health']['filtered'])
        monitor=PoseMonitor(min_confidence=.9)
        out=self.feed(monitor,pose(confidence=.8));self.assertEqual(out['health']['status'],'confidence_unavailable')
        self.assertEqual(out['raw_pose']['Confidence'],.8)


if __name__=='__main__':unittest.main()

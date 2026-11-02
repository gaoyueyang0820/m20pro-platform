"""Offline advice/transition checks. No sockets or actuator calls."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from terrain_profiles import recommend_profile,transition_step,GAITS


class TerrainProfilesTests(unittest.TestCase):
    def setUp(self):
        self.basic=dict(HES=0,Sleep=0,MotionState=17,ControlUsageMode=2,Gait=0x1003)
        self.motion=dict(LinearX=0,LinearY=0,OmegaZ=0)
    def test_narrow_profile_does_not_select_stairs_or_claim_path_clear(self):
        row=recommend_profile('narrow')
        self.assertEqual(row['recommended_gait'],0x3002)
        self.assertFalse(row['path_is_proven'])
        self.assertIn('swept_turn_clearance',row['required_perception'])
    def test_mode_change_precedes_gait_and_readback_is_required(self):
        kwargs=dict(desired_mode=0,desired_gait=0x3002)
        row=transition_step(self.basic,self.motion,**kwargs)
        self.assertEqual(row['command'],dict(type=1101,command=5,items=dict(Mode=0)))
        self.assertIsNone(transition_step(self.basic,self.motion,pending=True,**kwargs)['command'])
        self.basic.update(ControlUsageMode=0,Gait=0x1001)
        row=transition_step(self.basic,self.motion,**kwargs)
        self.assertEqual(row['command'],dict(type=2,command=23,items=dict(GaitParam=0x3002)))
        self.basic['Gait']=0x3002
        row=transition_step(self.basic,self.motion,**kwargs)
        self.assertTrue(row['ready']);self.assertIsNone(row['command'])
    def test_real_motion_and_sdk_state_cannot_trigger_mode_change(self):
        for update in [dict(MotionState=6),dict(Sleep=1),dict(HES=1)]:
            row=transition_step(dict(self.basic,**update),self.motion,desired_mode=0,desired_gait=0x3002)
            self.assertIsNone(row['command']);self.assertFalse(row['ready'])
        self.motion['LinearX']=.4
        row=transition_step(self.basic,self.motion,desired_mode=0,desired_gait=0x3002)
        self.assertEqual(row['action'],'stop_before_switch');self.assertIsNone(row['command'])
        for kw in [dict(sdk_active=True),dict(navigation_active=True),dict(basic_age=3),dict(motion_age=1)]:
            self.assertIsNone(transition_step(self.basic,self.motion,desired_mode=0,desired_gait=0x3002,**kw)['command'])
    def test_native_auto_stairs_is_not_silently_encoded_as_flat(self):
        row=recommend_profile('stairs',control='native_auto',slope_deg=20,step_height_m=.2)
        self.assertEqual(row['recommended_gait'],0x3003)
        self.assertFalse(row['control_path_compatible']);self.assertEqual(row['obs_mode'],0)
        direct=recommend_profile('stairs',control='native_direct',slope_deg=20,step_height_m=.2)
        self.assertTrue(direct['control_path_compatible']);self.assertEqual(direct['nav_mode'],0)
        self.assertFalse(direct['path_is_proven']);self.assertEqual(direct['obs_mode'],0)
    def test_hardware_45_degree_claim_does_not_widen_nav_limit(self):
        row=recommend_profile('slope',slope_deg=40)
        self.assertFalse(row['geometry_within_documented_range'])
        self.assertEqual(row['documented_limits']['documented_navigation_slope_deg'],30)
        self.assertTrue(recommend_profile('slope',slope_deg=-25)['geometry_within_documented_range'])
        self.assertIsNone(recommend_profile('stairs',step_height_m=.2)['geometry_within_documented_range'])
        self.assertFalse(recommend_profile('stairs',step_height_m=.4)['geometry_within_documented_range'])
    def test_low_tunnel_does_not_automatically_disable_assistance_and_continue(self):
        row=recommend_profile('narrow',prefer_assisted=True,native_assist_ready=True,ceiling_m=.8)
        self.assertFalse(row['control_path_compatible']);self.assertIsNone(row['use_mode'])
        ordinary=recommend_profile('narrow',prefer_assisted=True,native_assist_ready=True,ceiling_m=2)
        self.assertEqual(ordinary['use_mode'],2)
    def test_desert_name_does_not_invent_gait_or_authorize_movement(self):
        row=recommend_profile('soft_ground')
        self.assertIsNone(row['recommended_gait']);self.assertFalse(row['path_is_proven'])
        known=recommend_profile('soft_ground',validated_gait=0x1001)
        self.assertEqual(known['recommended_gait'],0x1001);self.assertFalse(known['path_is_proven'])
        with self.assertRaises(ValueError):recommend_profile('soft_ground',validated_gait=0x1002)
        with self.assertRaises(ValueError):transition_step(self.basic,self.motion,desired_mode=0,desired_gait=0x1002)
    def test_invalid_sensor_values_are_not_standstill(self):
        for value in [None,float('nan'),float('inf'),True]:
            row=transition_step(self.basic,dict(self.motion,OmegaZ=value),desired_mode=0,desired_gait=0x3002)
            self.assertIsNone(row['command'])
        for value in [float('nan'),True,91]:
            with self.assertRaises(ValueError):recommend_profile('slope',slope_deg=value)


if __name__=='__main__':unittest.main()

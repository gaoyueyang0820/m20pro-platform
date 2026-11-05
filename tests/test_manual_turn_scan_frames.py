"""Synthetic static-wall replay; no hardware, network or private observations."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.autonomy_sensing import _numpy_supported_floor
from m20.autonomy_core import Explorer,clearance


class ManualTurnFrameTests(unittest.TestCase):
    def scan(self,yaw):
        axis=np.arange(-2.8,2.81,.1)
        x,y=np.meshgrid(axis,axis)
        floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55)])
        wall=np.array([[.8,y,z] for y in [-.2,0,.2] for z in [0,.2,.4,.6]])
        # World -> measured base_link coordinates, not command integration.
        c,s=math.cos(yaw),math.sin(yaw)
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        scan=_numpy_supported_floor(np.concatenate([floor,wall])@rotation)
        scan.update(pose=[0,0,yaw],stamp=100.,age=.01)
        return scan

    def test_static_wall_rotates_in_body_frame_and_stays_fixed_in_map(self):
        expected={(4,-1),(4,0),(4,1)}
        for yaw,body_wall in [(0,expected),(math.pi/2,{(-1,-4),(0,-4),(1,-4)}),
                              (math.pi,{(-4,-1),(-4,0),(-4,1)})]:
            with self.subTest(yaw=yaw):
                scan=self.scan(yaw)
                self.assertEqual(set(map(tuple,scan['obstacles'])),body_wall)
                explorer=Explorer();explorer.ingest(scan)
                self.assertEqual(explorer.obstacles,expected)

    def test_new_body_observation_removes_old_front_wall_from_forward_clearance(self):
        self.assertFalse(clearance(self.scan(0),.35,0)[0])
        for yaw in [math.pi/2,math.pi]:
            with self.subTest(yaw=yaw):
                self.assertTrue(clearance(self.scan(yaw),.35,0)[0])


if __name__=='__main__':unittest.main()

import base64,threading,time,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge.ros_sensors import RosSensors

class MappingObservationTests(unittest.TestCase):
 def reader(self):
  r=RosSensors.__new__(RosSensors);r.ensure_topic=lambda topic:None;r.lock=threading.RLock();r.refinement_error=None;now=time.monotonic()
  x,y=np.meshgrid(np.arange(-3,3,.1),np.arange(-3,3,.1));pts=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]).astype('<f4')
  msg=NS(header=NS(frame_id='map',stamp=NS(sec=100,nanosec=0)),width=len(pts),height=1,row_step=pts.nbytes,point_step=12,data=pts.tobytes(),is_bigendian=False,fields=[NS(name=n,datatype=7,count=1,offset=i*4) for i,n in enumerate('xyz')])
  r.aligned_raw=(msg,now);r.odom=dict(frame_id='map',received_monotonic=now,stamp=100,tilt=0,PosX=0,PosY=0,PosZ=.55,quaternion=[0,0,0,1])
  r.location_quality=(1,now);r.match_quality=([.02,.98],now);r.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)))
  return r
 def test_full_localization_scan_has_matching_pose_and_quality(self):
  r=self.reader();p=r.observation();self.assertEqual(p['frame_id'],'map');self.assertEqual(p['pose'],[0,0,.55,0,0,0,1]);self.assertEqual(len(base64.b64decode(p['points'])),p['count']*12)
  self.assertGreater(p['count'],1000)
 def test_bad_quality_stale_and_mismatched_frames_are_not_recordable(self):
  for change in [lambda r:setattr(r,'location_quality',(2,time.monotonic())),lambda r:setattr(r,'match_quality',([.3,.7],time.monotonic())),lambda r:r.odom.update(stamp=99),lambda r:setattr(r.aligned_raw[0].header,'frame_id','odom'),lambda r:setattr(r,'aligned_raw',(r.aligned_raw[0],time.monotonic()-2))]:
   r=self.reader();change(r)
   with self.assertRaises(ValueError):r.observation()
 def test_verified_device_contract_accepts_zero_and_rejects_lost_five(self):
  r=self.reader();r.normal_location_status=0;r.location_quality=(0,time.monotonic())
  self.assertGreater(r.observation()['count'],1000)
  r.location_quality=(5,time.monotonic())
  with self.assertRaisesRegex(ValueError,'定位'):r.observation()
if __name__=='__main__':unittest.main()

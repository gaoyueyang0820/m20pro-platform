"""Typed occluded-ground recovery across real producer/RPC/cache, no sockets."""
import copy
import io
import json
import math
import threading
import unittest
from collections import deque
from types import SimpleNamespace as Obj
from unittest.mock import patch
from urllib.error import HTTPError
import numpy as np

from m20.autonomy_sensing import supported_floor,xyz
from m20.observation_errors import GroundObservationPending,ObservationSyncPending,error_payload,sync_error
from m20.latest_observation import LatestObservationPump
from bridge.ros_sensors import RosSensors
from bridge.sensor_client import SensorClient
import bridge.http_agent as http_agent
import m20.platform_core as platform_core


def floor_points():
    x,y=np.meshgrid(np.arange(-3,3,.075),np.arange(-3,3,.075))
    return np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.55)])


def message(points,stamp=123,frame='base_link'):
    points=np.asarray(points,dtype='<f4')
    return Obj(width=len(points),height=1,row_step=12*len(points),point_step=12,data=points.tobytes(),is_bigendian=False,
               fields=[Obj(name=name,datatype=7,count=1,offset=4*i) for i,name in enumerate('xyz')],
               header=Obj(frame_id=frame,stamp=Obj(sec=stamp,nanosec=0)))


def encode_handler(dispatch,request):
    captured=[]
    class Server:
        def __init__(self,address,handler):self.handler=handler
        def serve_forever(self):
            h=self.handler.__new__(self.handler);raw=json.dumps(request).encode()
            h.path='/sensors/autonomy';h.headers={'Content-Type':'application/json','Content-Length':str(len(raw))}
            h.rfile=io.BytesIO(raw);h.reply=lambda payload,status=200:captured.append((payload,status))
            h.run_request(post=True)
        def server_close(self):pass
    with patch.object(http_agent,'ThreadingHTTPServer',Server),patch.object(http_agent,'threading_is_main',return_value=False),patch('builtins.print'):
        http_agent.serve(0,dispatch)
    return captured[0]


def http_failure(payload):
    return HTTPError('http://offline.invalid/sensors/autonomy',400,'synthetic',{},io.BytesIO(json.dumps(payload).encode()))


def two_hops(error,mapping):
    request=dict(mapping=mapping,after_stamp=10.)
    def producer(path,req):raise error
    inner,status=encode_handler(producer,request)
    assert status==400
    with patch('bridge.sensor_client.urlopen',side_effect=lambda *a,**k:(_ for _ in ()).throw(http_failure(inner))):
        outer,status=encode_handler(lambda path,req:SensorClient().call(path,req),request)
    assert status==400
    return outer,request


def reader():
    clock=Obj(now=100.)
    r=RosSensors.__new__(RosSensors);r.lock=threading.RLock();r.ground_event=threading.Event()
    r.ground_ready=Obj(wait=lambda seconds:setattr(clock,'now',clock.now+max(0.,seconds)),notify_all=lambda:None)
    r.ensure_topic=lambda _:None;r.expire_clouds=lambda:None;r.autonomy_until=102.
    r.autonomy_error=None;r.autonomy_failure=None;r.autonomy_scan=None
    r.slam_pose_proof=None;r.slam_raw={};r.location_quality=None;r.odom_history=deque()
    r.slam_odom=dict(frame_id='map',child_frame_id='base_link',tilt=0.,stamp=123.,received_monotonic=clock.now,PosX=1.,PosY=2.,Yaw=0.)
    r.slam_history=deque([r.slam_odom])
    return r,clock


class GroundClassificationTests(unittest.TestCase):
    def test_only_four_observability_failures_are_pending(self):
        ground=floor_points();line=np.linspace(-3,3,300)
        cases=[(np.zeros((20,3)),'points_insufficient'),
               (ground+[0,0,2.],'floor_insufficient'),
               (np.column_stack([ground[:,0],ground[:,1]*.02,ground[:,2]]),'floor_coverage_narrow'),
               (np.column_stack([line,line,np.full(len(line),-.55)]),'floor_plane_degenerate')]
        for points,reason in cases:
            with self.subTest(reason=reason),self.assertRaises(GroundObservationPending) as caught:supported_floor(points)
            self.assertEqual(caught.exception.reason_code,reason)

    def test_bad_shape_and_observed_slope_remain_geometry_errors(self):
        steep=floor_points();steep[:,2]+=.11*steep[:,0]
        # Dense centre returns select an interior height bin, leaving enough
        # span to fit and actually observe the excessive slope (not sparse
        # coverage at an edge bin).
        steep=np.vstack([steep,np.repeat(steep[abs(steep[:,0])<.1],8,axis=0)])
        for points in (np.zeros((200,2)),np.zeros((100,3,1)),steep):
            with self.subTest(shape=points.shape),self.assertRaises(ValueError) as caught:supported_floor(points)
            self.assertNotIsInstance(caught.exception,GroundObservationPending)

    def test_zero_point_message_pending_but_invalid_xyz_fields_not_pending(self):
        with self.assertRaises(GroundObservationPending) as caught:xyz(message(np.empty((0,3))))
        self.assertEqual(caught.exception.reason_code,'points_insufficient')
        bad=message(floor_points());bad.fields=[]
        with self.assertRaises(ValueError) as caught:xyz(bad)
        self.assertNotIsInstance(caught.exception,GroundObservationPending)

    def test_visible_obstacle_and_missing_floor_remain_in_result(self):
        ground=floor_points();ground=ground[~((ground[:,0]>.7)&(ground[:,0]<1.3)&(abs(ground[:,1])<.4))]
        scan=supported_floor(np.vstack([ground,[1.6,0,-.2]]))
        self.assertNotIn([5,0],scan['floor'])
        self.assertIn([8,0],scan['obstacles'])
        self.assertNotIn([8,0],scan['floor'])


class GroundProducerTests(unittest.TestCase):
    def test_failure_drops_pre_occlusion_scene_and_rethrows_typed_in_both_modes(self):
        for mapping in (False,True):
            r,clock=reader();r.autonomy_scan=dict(received_monotonic=100.,source_age=.05,stamp=123.)
            error=GroundObservationPending('地面暂时不可见','floor_insufficient')
            with patch('bridge.ros_sensors.time.monotonic',side_effect=lambda:clock.now):
                r.ground_failure(error)
                self.assertIsNone(r.autonomy_scan)
                self.assertIsNot(r.autonomy_failure,error)
                with self.assertRaises(GroundObservationPending) as caught:r.autonomy(mapping,after_stamp=123.)
            self.assertEqual(caught.exception.reason_code,'floor_insufficient')
            self.assertGreater(r.autonomy_until,clock.now)

    def test_pending_always_clears_geometry_even_if_temporal_hint_is_passed(self):
        r,clock=reader();r.autonomy_scan=dict(received_monotonic=100.,source_age=.05,stamp=123.)
        with patch('bridge.ros_sensors.time.monotonic',side_effect=lambda:clock.now):
            r.ground_failure(GroundObservationPending('ground wait','points_insufficient'),temporal=True)
        self.assertIsNone(r.autonomy_scan)

    def test_hard_coordinate_or_tilt_error_not_reclassified_by_pending_ground(self):
        for field,value in [('frame_id','odom'),('tilt',.2),('child_frame_id','unverified')]:
            r,clock=reader();r.slam_odom[field]=value
            r.ground_failure(GroundObservationPending('ground wait','points_insufficient'))
            with self.assertRaises(ValueError) as caught:r.autonomy(True)
            self.assertNotIsInstance(caught.exception,GroundObservationPending)

    def test_hard_ground_failure_replaces_previous_pending_type(self):
        r,clock=reader();r.ground_failure(GroundObservationPending('wait','floor_insufficient'))
        r.ground_failure(ValueError('nonlevel surface'))
        with patch('bridge.ros_sensors.time.monotonic',side_effect=lambda:clock.now):
            with self.assertRaises(ValueError) as caught:r.autonomy(False)
        self.assertNotIsInstance(caught.exception,GroundObservationPending)
        self.assertEqual(str(caught.exception),'nonlevel surface')

    def test_actual_ground_loop_recovers_after_person_occlusion(self):
        r,clock=reader();r.ground_done=threading.Event();records=[]
        frames=[message(floor_points(),123),message(np.zeros((20,3)),124),message(floor_points(),125)]
        class Event:
            def wait(self,timeout):
                if r.autonomy_scan is not None or r.autonomy_failure is not None:
                    records.append((copy.deepcopy(r.autonomy_scan),r.autonomy_failure))
                if not frames:r.ground_done.set();r.lidar_raw=None;return
                clock.now+=.05;r.lidar_raw=(frames.pop(0),clock.now,.02)
            def clear(self):pass
            def set(self):pass
        r.ground_event=Event()
        with patch('bridge.ros_sensors.time.monotonic',side_effect=lambda:clock.now):r.ground_loop()
        self.assertEqual(len(records),3)
        self.assertEqual(records[0][0]['stamp'],123.)
        self.assertIsNone(records[1][0]);self.assertIsInstance(records[1][1],GroundObservationPending)
        self.assertEqual(records[2][0]['stamp'],125.);self.assertIsNone(records[2][1])
        self.assertEqual(records[0][0]['floor'],records[2][0]['floor'])


class GroundProtocolTests(unittest.TestCase):
    def test_two_actual_http_handlers_client_and_platform_rpc_preserve_pending(self):
        for mapping in (False,True):
            payload,request=two_hops(GroundObservationPending('等待地面恢复','floor_coverage_narrow'),mapping)
            self.assertEqual(payload['error_type'],'ground_observation_pending')
            self.assertEqual(payload['reason_code'],'floor_coverage_narrow')
            with patch('m20.platform_core.urlopen',side_effect=lambda *a,**k:(_ for _ in ()).throw(http_failure(payload))):
                with self.assertRaises(GroundObservationPending) as caught:platform_core.rpc('http://offline.invalid','/sensors/autonomy',request)
            self.assertEqual(caught.exception.reason_code,'floor_coverage_narrow')

    def test_success_status_with_error_body_uses_same_decoder(self):
        request={'mapping':False};payload=error_payload(GroundObservationPending('wait','floor_insufficient'),'/sensors/autonomy',request)
        for client in ('inner','outer'):
            response=io.BytesIO(json.dumps(payload).encode())
            with patch('bridge.sensor_client.urlopen' if client=='inner' else 'm20.platform_core.urlopen',return_value=response):
                with self.assertRaises(GroundObservationPending):
                    if client=='inner':SensorClient().call('/sensors/autonomy',request)
                    else:platform_core.rpc('http://offline.invalid','/sensors/autonomy',request)

    def test_plain_text_unknown_reason_and_wrong_endpoints_cannot_claim_pending(self):
        request={'mapping':False};plain=error_payload(ValueError('地面覆盖太窄，暂不能自主移动'),'/sensors/autonomy',request)
        self.assertIsNone(sync_error(plain,'/sensors/autonomy',request))
        good=error_payload(GroundObservationPending('wait','floor_insufficient'),'/sensors/autonomy',request)
        for payload,path,req in [(dict(good,reason_code='slope'),'/sensors/autonomy',request),
                                 (dict(good,reason_code=[]),'/sensors/autonomy',request),
                                 (dict(good,error_protocol=True),'/sensors/autonomy',request),
                                 (good,'/sensors/observation',request),(good,'/sensors/autonomy',{'mapping':'false'})]:
            self.assertIsNone(sync_error(payload,path,req))
        with self.assertRaises(ValueError):GroundObservationPending('slope','slope')

    def test_existing_mapping_only_sync_protocol_unchanged(self):
        pending=ObservationSyncPending('sync','slam_pose_sync')
        payload=error_payload(pending,'/sensors/autonomy',{'mapping':True})
        self.assertIsInstance(sync_error(payload,'/sensors/autonomy',{'mapping':True}),ObservationSyncPending)
        self.assertIsNone(sync_error(payload,'/sensors/autonomy',{'mapping':False}))
        self.assertNotIn('error_type',error_payload(pending,'/sensors/autonomy',{'mapping':False}))


class GroundPumpTests(unittest.TestCase):
    def pump(self):
        clock=[100.];p=LatestObservationPump(lambda c,s:None,clock=lambda:clock[0])
        p.running=True;p.generation=1;p.stop_event=threading.Event()
        p._publish(dict(stamp=10.,_valid_until=100.6,floor=[[4,0]],obstacles=[]),1)
        return p,clock
    def test_occlusion_immediately_drops_scene_and_waits_without_renewing_old_frame(self):
        p,clock=self.pump();p._error(GroundObservationPending('person occlusion','floor_insufficient'),1)
        for now in (100.01,106.,120.):
            clock[0]=now;r=p.read(1)
            self.assertEqual(r['error_kind'],'ground_pending');self.assertEqual(r['reason_code'],'floor_insufficient')
            self.assertEqual(r['state'],'waiting');self.assertIsNone(r['observation'])
        self.assertFalse(p._publish(dict(stamp=10.,_valid_until=120.5,floor=[[99,99]],obstacles=[]),1))
        self.assertIsNone(p.read(1)['observation'])
        self.assertTrue(p._publish(dict(stamp=11.,_valid_until=120.5,floor=[[4,0]],obstacles=[]),1))
        first=p.read(1);self.assertEqual(first['state'],'fresh');self.assertIsNone(first['error_kind']);self.assertIsNone(first['reason_code'])
        self.assertTrue(p._publish(dict(stamp=12.,_valid_until=120.5,floor=[[5,0]],obstacles=[]),1))
        self.assertGreater(p.read(1)['sequence'],first['sequence'])

    def test_ground_error_from_old_connection_and_post_cancel_results_are_ignored(self):
        p,clock=self.pump();p.generation=2
        p._error(GroundObservationPending('old','points_insufficient'),1)
        self.assertIsNotNone(p.read(2)['observation'])
        p.cancel();p._error(GroundObservationPending('late','points_insufficient'),2)
        self.assertEqual(p.read(2)['state'],'cancelled')
        self.assertFalse(p._publish(dict(stamp=20.,_valid_until=100.5),2))

    def test_real_geometry_failure_during_ground_wait_remains_invalidated(self):
        p,_=self.pump();p._error(GroundObservationPending('wait','floor_insufficient'),1)
        p._error(ValueError('slope too large'),1)
        r=p.read(1);self.assertEqual(r['error_kind'],'invalidated');self.assertIsNone(r['observation']);self.assertIsNone(r['reason_code'])


if __name__=='__main__':unittest.main()

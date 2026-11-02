"""Typed transient synchronization through the real HTTP/RPC/cache code.

HTTP handlers execute with fake in-memory server objects; no socket is bound.
Runtime cases execute actual scan_now, RPC decoding, pump and _run threads.
"""
import copy
import io
import json
import threading
import time
import unittest
from collections import deque
from urllib.error import HTTPError
from unittest.mock import patch
from observation_errors import ObservationSyncPending,error_payload,sync_error
from bridge.ros_sensors import RosSensors
from bridge.sensor_client import SensorClient
import bridge.http_agent as http_agent
import platform_core
from latest_observation import LatestObservationPump
from autonomy_runtime import Autonomy
from types import SimpleNamespace as Obj
import test_autonomy_observation_runtime as runtime_cases


def reader(stamp=100.):
    r=RosSensors.__new__(RosSensors);r.lock=threading.RLock();r.ground_ready=threading.Condition(r.lock);r.ground_event=threading.Event()
    r.ensure_topic=lambda _:None;r.expire_clouds=lambda:None;r.autonomy_error=None;r.slam_pose_proof=None;r.slam_raw={}
    now=time.monotonic()
    r.autonomy_scan=dict(stamp=stamp,received_monotonic=now,source_age=.1,pose=None,frame_id='base_link',source_frame='lidar_link',floor=[[4,0]],obstacles=[])
    r.slam_odom=dict(frame_id='map',child_frame_id='base_link',stamp=stamp,received_monotonic=now,tilt=0.,PosX=1.,PosY=2.,Yaw=.3)
    r.slam_history=deque([r.slam_odom],maxlen=40)
    return r


def encoded_by_actual_handler(dispatch,path='/sensors/autonomy',request=None):
    request={'mapping':True} if request is None else request;captured=[]
    class Server:
        def __init__(self,address,handler):self.handler=handler
        def serve_forever(self):
            h=self.handler.__new__(self.handler);raw=json.dumps(request).encode()
            h.path=path;h.headers={'Content-Type':'application/json','Content-Length':str(len(raw))};h.rfile=io.BytesIO(raw)
            h.reply=lambda payload,status=200:captured.append((payload,status))
            h.run_request(post=True)
        def server_close(self):pass
    with patch.object(http_agent,'ThreadingHTTPServer',Server),patch.object(http_agent,'threading_is_main',return_value=False),patch('builtins.print'):
        http_agent.serve(0,dispatch)
    return captured[0]


def raise_error(exc):
    def dispatch(path,request):raise exc
    return dispatch


def http_error(payload):return HTTPError('http://offline.invalid/sensors/autonomy',400,'offline',{},io.BytesIO(json.dumps(payload).encode()))


def two_hop_payload(exc):
    inner,status=encoded_by_actual_handler(raise_error(exc))
    with patch('bridge.sensor_client.urlopen',side_effect=lambda *a,**k:(_ for _ in ()).throw(http_error(inner))):
        outer,status=encoded_by_actual_handler(lambda path,req:SensorClient().call(path,req))
    return outer


class ObservationSyncProtocolTests(unittest.TestCase):
    def test_only_explicit_slampose_wait_is_typed_by_the_producer(self):
        r=reader();r.slam_history.clear()
        with self.assertRaises(ObservationSyncPending) as caught:r.autonomy(True)
        self.assertEqual(caught.exception.reason,'slam_pose_sync')
        r=reader();r.slam_odom['child_frame_id']='';r.slam_pose_proof=None
        with self.assertRaises(ObservationSyncPending) as caught:r.autonomy(True)
        self.assertEqual(caught.exception.reason,'slam_reference_sync')

    def test_real_slam_pose_metadata_and_lidar_coordinates_are_preserved(self):
        r=reader();original=copy.deepcopy(r.autonomy_scan);packet=r.autonomy(True)
        self.assertEqual(packet['pose_stamp'],100.);self.assertEqual(packet['pose_source'],'/SLAM_ODOM')
        self.assertEqual(packet['pose_frame_id'],'map');self.assertEqual(packet['pose'],[1.,2.,.3])
        self.assertEqual(packet['floor'],original['floor']);self.assertEqual(packet['source_frame'],'lidar_link')
        self.assertEqual(packet['frame_id'],'base_link')

    def test_coordinates_and_tilt_remain_hard_errors_even_when_pair_is_not_ready(self):
        for field,value in [('frame_id','odom'),('child_frame_id','unverified'),('tilt',.2)]:
            with self.subTest(field=field):
                r=reader();r.slam_odom.update({field:value,'stamp':99.})
                with self.assertRaises(ValueError) as caught:r.autonomy(True)
                self.assertNotIsInstance(caught.exception,ObservationSyncPending)
                self.assertNotIn('error_type',error_payload(caught.exception,'/sensors/autonomy',{'mapping':True}))

    def test_http_inner_client_outer_rpc_and_scan_now_preserve_type(self):
        payload=two_hop_payload(ObservationSyncPending('等待同步','slam_pose_sync'))
        self.assertEqual(payload['error_type'],'observation_sync_pending')
        a=Autonomy.__new__(Autonomy)
        a.app=Obj(demo=False,hub=Obj(connections=Obj(value={'sensor_source':'nos'})),
                  proxy=lambda role,path,data,timeout:platform_core.rpc('http://offline.invalid',path,data,timeout))
        with patch('platform_core.urlopen',side_effect=lambda *a,**k:(_ for _ in ()).throw(http_error(payload))):
            with self.assertRaises(ObservationSyncPending) as caught:a.scan_now(True)
        self.assertEqual(caught.exception.reason,'slam_pose_sync')

    def test_chinese_text_unknown_codes_and_other_requests_cannot_enable_cache_retention(self):
        plain=error_payload(ValueError('等待与雷达同步的 SLAM 位姿'),'/sensors/autonomy',{'mapping':True})
        self.assertIsNone(sync_error(plain,'/sensors/autonomy',{'mapping':True}))
        good=error_payload(ObservationSyncPending('等待','slam_pose_sync'),'/sensors/autonomy',{'mapping':True})
        for candidate,path,request in [(dict(good,reason='coordinate_changed'),'/sensors/autonomy',{'mapping':True}),
                                       (dict(good,reason=[]),'/sensors/autonomy',{'mapping':True}),
                                       (good,'/maps/build/status',{'mapping':True}),
                                       (good,'/sensors/autonomy',{'mapping':False}),
                                       (dict(good,error_protocol=True),'/sensors/autonomy',{'mapping':True})]:
            self.assertIsNone(sync_error(candidate,path,request))

    def test_pending_keeps_exact_expiry_and_duplicates_cannot_revive_or_replace_pose(self):
        clock=[100.];pump=LatestObservationPump(lambda c,s:None,clock=lambda:clock[0]);pump.running=True;pump.generation=1;pump.stop_event=threading.Event()
        original=dict(stamp=10.,_valid_until=100.3,pose=[1,2,.3],floor=[[4,0]],obstacles=[])
        pump._publish(original,1)
        pending=ObservationSyncPending('等待同步','slam_pose_sync')
        for now in [100.05,100.15,100.25]:
            clock[0]=now;pump._error(pending,1)
            snapshot=pump.read(1);self.assertEqual(snapshot['error_kind'],'sync_pending')
            self.assertEqual(snapshot['observation']['_valid_until'],100.3)
        pump._publish(dict(original,_valid_until=100.8,pose=[9,9,0]),1)
        pump._publish(dict(original,stamp=9.9,_valid_until=100.7,pose=[8,8,0]),1)
        self.assertEqual(pump.read(1)['observation']['pose'],[1,2,.3]);self.assertEqual(pump.read(1)['sequence'],1)
        clock[0]=100.31;pump._error(pending,1);self.assertEqual(pump.read(1)['state'],'expired')
        self.assertIsNone(pump.read(1)['observation'])

    def test_hard_error_after_pending_immediately_invalidates_still_valid_pair(self):
        pump=LatestObservationPump(lambda c,s:None,clock=lambda:100.);pump.running=True;pump.generation=1;pump.stop_event=threading.Event()
        pump._publish(dict(stamp=10.,_valid_until=100.5,pose=[1,2,0]),1)
        pump._error(ObservationSyncPending('等待同步','slam_pose_sync'),1)
        self.assertIsNotNone(pump.read()['observation'])
        pump._error(ValueError('SLAM 位姿父坐标系尚未核对：odom'),1)
        self.assertEqual(pump.read()['state'],'invalidated');self.assertIsNone(pump.read()['observation'])


class ObservationSyncRuntimeTests(unittest.TestCase):
    setUp=runtime_cases.AutonomyObservationRuntimeTests.setUp
    tearDown=runtime_cases.AutonomyObservationRuntimeTests.tearDown
    configure=runtime_cases.AutonomyObservationRuntimeTests.configure
    packet=runtime_cases.AutonomyObservationRuntimeTests.packet
    run_worker=runtime_cases.AutonomyObservationRuntimeTests.run_worker
    wait_for=runtime_cases.AutonomyObservationRuntimeTests.wait_for

    def exercise_pending(self,hard_after=False):
        a=self.configure();self.app.config['nos']='http://offline.invalid'
        pending=two_hop_payload(ObservationSyncPending('等待与雷达同步的 SLAM 位姿','slam_pose_sync'))
        invalid=two_hop_payload(ValueError('SLAM 位姿父坐标系尚未核对：odom'))
        entered=threading.Event();hard=threading.Event()
        def response(request,timeout):
            i=len(self.calls);self.calls.append(i);time.sleep(.035)
            if i<3:return io.BytesIO(json.dumps(self.packet(10.+i*.1)).encode())
            entered.set();raise http_error(invalid if hard.is_set() else pending)
        with patch('platform_core.urlopen',side_effect=response):
            self.run_worker();self.assertTrue(entered.wait(1.))
            self.assertTrue(self.wait_for(lambda:sum(any(r['axes']) for r in self.app.link.output)>=3),a.message)
            output=[r for r in self.app.link.output if any(r['axes'])];deadline=output[-1]['deadline'];stamp=output[-1]['stamp']
            self.assertEqual(a.observation_pump.status()['error_kind'],'sync_pending')
            if hard_after:
                hard.set();before=time.monotonic()
                self.assertTrue(self.wait_for(lambda:a.observation_pump.status()['error_kind']=='invalidated'))
                self.assertTrue(self.wait_for(lambda:any(not any(r['axes']) and r['at']>=before for r in self.app.link.output)))
                self.assertLess(time.monotonic(),deadline)
            else:
                self.assertTrue(self.wait_for(lambda:any(not any(r['axes']) and r['at']>=deadline for r in self.app.link.output)),a.message)
                self.assertEqual(a.observation_pump.read()['state'],'expired')
                self.assertFalse(any(any(r['axes']) and r['stamp']==stamp and (r['deadline']!=deadline or r['at']>=deadline) for r in self.app.link.output))
            with self.app.lock:a.pause('done')

    def test_production_runtime_continues_valid_pair_during_pending_then_stops_at_original_expiry(self):self.exercise_pending()
    def test_production_runtime_hard_error_after_pending_stops_before_original_expiry(self):self.exercise_pending(True)


if __name__=='__main__':unittest.main()

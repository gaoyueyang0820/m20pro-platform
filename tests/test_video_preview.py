"""Pure mock/synthetic video policy tests. No sockets, ROS, or robot processes.

JPEG markers below are synthetic transport fixtures, not camera recordings. The
real codec fixture is exercised separately by test_ffmpeg_capture.py.
"""
import base64
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from bridge.video_preview import VideoPreviewAgent, internal_aos
from m20.camera_streams import CameraFeed, Cameras
from m20.video_preview import MAX_JPEG_BYTES, PreviewDenied, PreviewLeases
from m20.video_preview_client import VideoPreviewClient


JPEG=b'\xff\xd8synthetic-preview-only\xff\xd9'
A='preview-consumer-0001'
B='preview-consumer-0002'
C='preview-consumer-0003'


class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now
    def advance(self,seconds):self.now+=seconds


class Frame:
    def __init__(self,width=1280,height=720):self.shape=(height,width,3)


class Decoder:
    CAP_FFMPEG=1; CAP_PROP_OPEN_TIMEOUT_MSEC=2; CAP_PROP_READ_TIMEOUT_MSEC=3
    CAP_PROP_FRAME_TYPE=69; IMWRITE_JPEG_QUALITY=4
    def __init__(self):
        self.opens=0; self.releases=0; self.active=0; self.max_active=0; self.encodes=[]; self.reads=0
    def VideoCapture(self,*args):
        self.opens+=1; self.active+=1; self.max_active=max(self.max_active,self.active)
        owner=self
        class Capture:
            closed=False
            def isOpened(self):return not self.closed
            def read(self):
                time.sleep(.01); owner.reads+=1
                return True,Frame()
            def get(self,prop):return ord('I')
            def release(self):
                if not self.closed:
                    self.closed=True; owner.releases+=1; owner.active-=1
        return Capture()
    def resize(self,frame,size):return Frame(*size)
    def imencode(self,extension,frame,params):
        self.encodes.append((frame.shape,params[-1]))
        return True,SimpleNamespace(tobytes=lambda:JPEG)


class FeedStub:
    def __init__(self,viewers=0):
        self.viewers=viewers; self.immediate=[]; self.packet=(1,JPEG,.1)
        self.raw=(1,Frame(),time.time(),time.monotonic())
    def subscribe(self):self.viewers+=1
    def unsubscribe(self,*,immediate=False):self.viewers-=1; self.immediate.append(immediate)
    def read_packet(self,after=-1,timeout=1,*,cancelled=None):return self.packet
    def read_raw(self,after,timeout):return self.raw
    def status(self):return dict(width=800,height=450,error=None,retry_in=0.)


def application(remote=None,feed=None):
    source=['192.168.20.10']; calls=[]; feed=feed or FeedStub()
    decoder=Decoder()
    def get(camera):calls.append(camera); return feed
    app=SimpleNamespace(demo=False,cameras=SimpleNamespace(target=lambda:source[0],get=get,decoder=decoder),
                        proxy=remote or (lambda *args,**kwargs:None))
    return app,calls,feed,source


def remote_frame(sequence=10,stream='synthetic-agent-instance',**changes):
    return dict(ok=True,protocol=1,sequence=sequence,stream=stream,jpeg_base64=base64.b64encode(JPEG).decode(),
                age=.1,age_basis='decode_receive',width=480,height=270,profile='weak',transport='onboard_jpeg',**changes)


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.started=[]; self.stopped=[]
        self.pool=PreviewLeases(lambda cam,profile:self.started.append((cam,profile)) or object(),
                                lambda lease,reason:self.stopped.append((lease,reason)),clock=self.clock,start_worker=False)
        self.addCleanup(self.pool.close)
    def test_construction_and_status_are_passive(self):
        self.assertIsNone(self.pool.worker); self.assertEqual(self.pool.status()['leases'],0)
        self.assertFalse(self.started)
    def test_single_stream_profile_limits_camera_and_transport_not_viewer_count(self):
        first=self.pool.begin('front','weak',A); self.pool.finish(first)
        same=self.pool.begin('front','weak',B); self.pool.finish(same)
        with self.assertRaises(PreviewDenied) as caught:self.pool.begin('rear','weak',C)
        self.assertEqual(caught.exception.result['code'],'stream_limit')
        with self.assertRaises(PreviewDenied):self.pool.begin('front','standard',C)
        self.assertEqual(len(self.started),2)
    def test_standard_allows_two_camera_streams(self):
        for camera,consumer in [('front',A),('rear',B)]:self.pool.finish(self.pool.begin(camera,'standard',consumer))
        self.assertEqual(self.pool.status()['leases'],2)
    def test_busy_rate_limit_and_profile_change_do_not_duplicate_subscription(self):
        lease=self.pool.begin('front','weak',A)
        with self.assertRaises(PreviewDenied) as caught:self.pool.begin('front','weak',A)
        self.assertEqual(caught.exception.result['code'],'busy')
        self.pool.finish(lease)
        with self.assertRaises(PreviewDenied) as caught:self.pool.begin('front','weak',A)
        self.assertEqual(caught.exception.result['code'],'rate_limited')
        self.clock.advance(.5)
        self.assertIs(self.pool.begin('front','weak',A),lease)
        with self.assertRaises(PreviewDenied):self.pool.begin('front','snapshot',A)
        self.assertEqual(len(self.started),1)
    def test_release_before_late_frame_request_is_tombstoned_and_idempotent(self):
        self.assertFalse(self.pool.release(A)['released'])
        with self.assertRaises(PreviewDenied):self.pool.begin('front','weak',A)
        self.assertFalse(self.started)
        lease=self.pool.begin('front','weak',B)
        self.assertTrue(self.pool.release(B)['released']); self.assertFalse(self.pool.release(B)['released'])
        self.pool.finish(lease)
        self.assertTrue(lease.cancelled.is_set()); self.assertEqual(len(self.stopped),1)
        self.assertEqual(self.pool.status()['leases'],0)
    def test_request_deadline_and_idle_expiry_cannot_be_renewed_by_late_completion(self):
        lease=self.pool.begin('front','weak',A,timeout=5.)
        self.clock.advance(3.1); self.assertEqual(self.pool.expire(),0)
        self.clock.advance(2.5); self.assertEqual(self.pool.expire(),1)
        self.pool.finish(lease); self.assertEqual(self.pool.status()['leases'],0)
        self.assertTrue(lease.cancelled.is_set())
        second=self.pool.begin('front','weak',B); self.pool.finish(second)
        self.clock.advance(3.1); self.assertEqual(self.pool.expire(),1)
    def test_viewer_and_identifier_limits(self):
        for value in ('', 'short','x'*97, 'consumer/invalid-0000',None):
            with self.assertRaises(ValueError):self.pool.begin('front','weak',value)
        for i in range(16):
            lease=self.pool.begin('front','weak','preview-consumer-%04d'%i); self.pool.finish(lease)
        with self.assertRaises(PreviewDenied) as caught:self.pool.begin('front','weak','preview-consumer-9999')
        self.assertEqual(caught.exception.result['code'],'viewer_limit')


class ClientTests(unittest.TestCase):
    def client(self,app,clock=None):
        client=VideoPreviewClient(app,clock=clock or time.monotonic,start_worker=False)
        self.addCleanup(client.close); return client
    def test_status_and_reset_do_not_probe_or_open_streams(self):
        remote=SimpleNamespace(calls=[])
        app,calls,_,_=application(lambda *a,**k:remote.calls.append(a))
        client=self.client(app); client.status(); client.reset(); client.status()
        self.assertFalse(calls); self.assertFalse(remote.calls)
    def test_weak_uses_only_existing_gos_rpc_and_preserves_age(self):
        clock=Clock(); rpc=[]
        def remote(role,path,req,timeout):
            rpc.append((role,path,req,timeout)); clock.advance(.4); return remote_frame()
        app,opened,_,_=application(remote); client=self.client(app,clock)
        result=client.frame(profile='weak',consumer=A)
        self.assertTrue(result['ok']); self.assertEqual(result['jpeg'],JPEG)
        self.assertAlmostEqual(result['age'],.5); self.assertTrue(result['upstream_reduced'])
        self.assertFalse(result['source_bitrate_changed']); self.assertFalse(opened)
        self.assertEqual(rpc[0][0:2],('gos','/video/preview/frame'))
        self.assertEqual(set(rpc[0][2]),{'camera','profile','consumer','after','stream'})
        client.release(A)
        self.assertEqual(rpc[-1][1],'/video/preview/release')
    def test_unavailable_weak_never_falls_back_to_direct_rtsp_or_leaks_errors(self):
        def remote(*args,**kwargs):raise ValueError('rtsp://user:secret@fixture unsupported')
        app,opened,_,_=application(remote); client=self.client(app)
        result=client.frame(profile='weak',consumer=A)
        self.assertFalse(result['ok']); self.assertEqual(result['code'],'unavailable')
        self.assertIn('未改用原码率视频',result['err']); self.assertNotIn('secret',str(result)); self.assertFalse(opened)
    def test_old_agent_error_is_explicitly_unsupported(self):
        app,opened,_,_=application(lambda *a,**k:dict(ok=False,err='unknown route'))
        result=self.client(app).frame(profile='weak',consumer=A)
        self.assertEqual(result['code'],'unavailable'); self.assertIn('更新代理',result['err']); self.assertFalse(opened)
    def test_invalid_or_unbounded_remote_payloads_are_never_published(self):
        cases=[{'age':float('nan')},{'age':float('inf')},{'age':-1},{'sequence':True},{'sequence':-1},
               {'jpeg_base64':'!!!'},{'jpeg_base64':base64.b64encode(b'\xff\xd8partial').decode()},
               {'jpeg_base64':'A'*(MAX_JPEG_BYTES*2)},{'protocol':2},{'width':481},{'height':1081},
               {'age_basis':'exposure'},{'profile':'standard'},{'transport':'direct_rtsp'}]
        for change in cases:
            with self.subTest(change=change):
                packet=remote_frame(); packet.update(change)
                app,opened,_,_=application(lambda *a,**k:packet)
                result=self.client(app).frame(profile='weak',consumer=A)
                self.assertFalse(result['ok']); self.assertEqual(result['code'],'invalid_frame'); self.assertFalse(opened)
    def test_slow_transport_cannot_make_an_old_frame_fresh(self):
        clock=Clock()
        def remote(*a,**k):clock.advance(6.); return remote_frame()
        app,_,_,_=application(remote)
        result=self.client(app,clock).frame(profile='weak',consumer=A)
        self.assertEqual(result['code'],'stale_frame'); self.assertNotIn('jpeg',result)
    def test_duplicate_source_frame_and_agent_restart_keep_monotonic_client_sequence(self):
        clock=Clock(); packet=[remote_frame()]; rpc=[]
        def remote(*args,**kwargs):rpc.append(args[2]); return packet[0]
        app,_,_,_=application(remote); client=self.client(app,clock)
        first=client.frame(profile='weak',consumer=A); self.assertEqual(first['sequence'],1)
        clock.advance(.6)
        duplicate=client.frame(profile='weak',consumer=A,after=1)
        self.assertEqual(duplicate['code'],'waiting_frame'); self.assertEqual(rpc[-1]['after'],10)
        self.assertEqual(rpc[-1]['stream'],'synthetic-agent-instance')
        packet[0]=remote_frame(sequence=0,stream='replacement-agent')
        clock.advance(.6)
        fresh=client.frame(profile='weak',consumer=A,after=1); self.assertEqual(fresh['sequence'],2)
    def test_snapshot_finishes_its_demand_after_one_rpc(self):
        rpc=[]
        def remote(role,path,request,timeout):
            rpc.append(path); packet=remote_frame(); packet['profile']='snapshot'; return packet
        app,opened,_,_=application(remote); client=self.client(app)
        result=client.frame(profile='snapshot',consumer=A)
        self.assertTrue(result['ok']); self.assertEqual(client.status()['demand']['leases'],0)
        self.assertEqual(rpc,['/video/preview/frame','/video/preview/release']); self.assertFalse(opened)
    def test_direct_preview_release_keeps_an_independent_detection_subscription(self):
        app,opened,feed,_=application(feed=FeedStub(viewers=1)); client=self.client(app)
        result=client.frame(consumer=A)
        self.assertTrue(result['ok']); self.assertFalse(result['upstream_reduced']); self.assertEqual(feed.viewers,2)
        client.release(A); self.assertEqual(feed.viewers,1); self.assertEqual(feed.immediate,[True])
    def test_pc_economy_resizes_output_but_does_not_claim_upstream_reduction(self):
        app,_,_,_=application(); client=self.client(app)
        result=client.frame(profile='economy',consumer=A)
        self.assertTrue(result['ok']); self.assertEqual((result['width'],result['height']),(480,270))
        self.assertFalse(result['upstream_reduced']); self.assertEqual(app.cameras.decoder.encodes[-1][1],45)
    def test_slow_pc_encoding_retains_original_capture_age(self):
        app,_,feed,_=application(); feed.raw=(1,Frame(),time.time()-2,time.monotonic()-2)
        result=self.client(app).frame(profile='economy',consumer=A)
        self.assertEqual(result['code'],'stale_frame')
    def test_release_during_rpc_invalidates_late_response(self):
        entered=threading.Event(); finish=threading.Event(); result=[]
        def remote(role,path,request,timeout):
            if path.endswith('/release'):return dict(ok=True)
            entered.set(); finish.wait(1); return remote_frame()
        app,opened,_,_=application(remote); client=self.client(app)
        worker=threading.Thread(target=lambda:result.append(client.frame(profile='weak',consumer=A)))
        worker.start(); self.assertTrue(entered.wait(1)); client.release(A); finish.set(); worker.join(1)
        self.assertFalse(worker.is_alive()); self.assertEqual(result[0]['code'],'cancelled'); self.assertFalse(opened)
    def test_connection_reset_invalidates_late_rpc_without_contacting_new_target(self):
        entered=threading.Event(); finish=threading.Event(); results=[]; rpc=[]
        def remote(*args,**kwargs):rpc.append(args[1]); entered.set(); finish.wait(1); return remote_frame()
        app,_,_,source=application(remote); client=self.client(app)
        worker=threading.Thread(target=lambda:results.append(client.frame(profile='weak',consumer=A)))
        worker.start(); self.assertTrue(entered.wait(1)); source[0]='192.168.20.11'; client.reset()
        finish.set(); worker.join(1)
        self.assertEqual(results[0]['code'],'cancelled'); self.assertEqual(rpc,['/video/preview/frame'])
    def test_demo_never_probes_or_opens_a_camera(self):
        app,calls,_,_=application(lambda *a,**k:self.fail('RPC in demo')); app.demo=True
        self.assertEqual(self.client(app).frame(profile='weak',consumer=A)['code'],'offline'); self.assertFalse(calls)


class AgentTests(unittest.TestCase):
    def agent(self,source=None):
        decoder=Decoder(); agent=VideoPreviewAgent(aos=source or '192.168.20.10',decoder=decoder,
                                                  open_capture=lambda camera:decoder.VideoCapture(),start_worker=False)
        self.addCleanup(agent.close); return agent,decoder
    def test_only_configured_private_ipv4_is_accepted(self):
        for address in ('','localhost','127.0.0.1','0.0.0.0','8.8.8.8','224.0.0.1','::1',
                        '192.168.20.10/path','rtsp://192.168.20.10','192.168.20.10 ','192.168.20.10:8554',None):
            with self.subTest(address=address),self.assertRaises(ValueError):internal_aos(address)
        self.assertEqual(internal_aos('192.168.20.10'),'192.168.20.10')
    def test_construction_and_status_do_not_probe_decoder_or_network(self):
        with patch('bridge.video_preview.find_ffmpeg',side_effect=AssertionError('probe')):
            agent=VideoPreviewAgent(aos=lambda:'192.168.20.10',start_worker=False)
            self.addCleanup(agent.close); status=agent.dispatch('/video/preview/status',None)
        self.assertTrue(status['configured']); self.assertFalse(status['initialized']); self.assertIsNone(agent.pool)
        missing=VideoPreviewAgent(aos='',start_worker=False); self.addCleanup(missing.close)
        self.assertFalse(missing.status()['configured']); self.assertIsNone(missing.pool)
    def test_fixed_camera_names_and_no_user_supplied_urls(self):
        agent,decoder=self.agent()
        for request in (dict(camera='ball',consumer=A),dict(camera='front',consumer=A,url='http://fixture'),
                        dict(camera='front',consumer=A,aos='192.168.20.11'),dict(camera='front',consumer=A,profile='standard')):
            with self.assertRaises(ValueError):agent.frame(request)
        self.assertEqual(decoder.opens,0)
    def test_agent_reuses_one_onboard_decoder_and_limits_front_rear_concurrency(self):
        agent,decoder=self.agent()
        first=agent.frame(dict(camera='front',profile='weak',consumer=A))
        second=agent.frame(dict(camera='front',profile='weak',consumer=B))
        rejected=agent.frame(dict(camera='rear',profile='weak',consumer=C))
        self.assertTrue(first['ok']); self.assertTrue(second['ok']); self.assertEqual(decoder.opens,1)
        self.assertEqual(first['transport'],'onboard_jpeg'); self.assertFalse(first['source_bitrate_changed'])
        self.assertEqual((first['width'],first['height']),(480,270))
        self.assertEqual(base64.b64decode(first['jpeg_base64']),JPEG)
        self.assertEqual(rejected['code'],'stream_limit')
        agent.dispatch('/video/preview/release',dict(consumer=A)); agent.dispatch('/video/preview/release',dict(consumer=B))
        final=agent.frame(dict(camera='rear',profile='weak',consumer=C))
        self.assertTrue(final['ok']); self.assertEqual(decoder.max_active,1)
    def test_snapshot_and_lease_expiry_stop_only_the_preview_reader(self):
        agent,decoder=self.agent(); clock=Clock(); agent.leases.clock=clock
        shot=agent.frame(dict(camera='front',profile='snapshot',consumer=A))
        self.assertTrue(shot['ok']); self.assertEqual(agent.status()['demand']['leases'],0)
        front=agent.pool.get('front'); worker=front.thread
        if worker:worker.join(.5)
        self.assertEqual(decoder.active,0)
        result=agent.frame(dict(camera='front',profile='weak',consumer=B)); self.assertTrue(result['ok'])
        clock.advance(3.1); self.assertEqual(agent.leases.expire(),1)
        worker=front.thread
        if worker:worker.join(.5)
        self.assertEqual(decoder.active,0)
    def test_released_consumer_cannot_be_resurrected_by_out_of_order_http(self):
        agent,decoder=self.agent()
        agent.dispatch('/video/preview/release',dict(consumer=A))
        response=agent.frame(dict(camera='front',profile='weak',consumer=A))
        self.assertEqual(response['code'],'cancelled'); self.assertEqual(decoder.opens,0)
    def test_source_configuration_change_invalidates_old_feed_and_stream_identity(self):
        source=['192.168.20.10']; agent,_=self.agent(lambda:source[0])
        first=agent.frame(dict(camera='front',consumer=A)); old=agent.pool.get('front')
        source[0]='192.168.20.11'
        second=agent.frame(dict(camera='front',consumer=B,stream=first['stream'],after=first['sequence']))
        self.assertTrue(old.closed); self.assertTrue(second['ok']); self.assertNotEqual(first['stream'],second['stream'])


class CaptureDemandTests(unittest.TestCase):
    def test_no_viewer_opens_nothing_and_short_idle_timeout_stops_capture(self):
        decoder=Decoder(); feed=CameraFeed(decoder,'192.168.20.10','front',idle_timeout=.03)
        self.addCleanup(feed.close); self.assertEqual(decoder.opens,0)
        feed.subscribe(); self.assertIsNotNone(feed.read(timeout=1)); worker=feed.thread
        feed.unsubscribe(); worker.join(.3)
        self.assertFalse(worker.is_alive()); self.assertEqual(decoder.active,0); self.assertEqual(feed.status()['phase'],'idle')
        feed.subscribe(); self.assertIsNotNone(feed.read(timeout=1)); self.assertEqual(decoder.opens,2)
    def test_backoff_stops_promptly_after_last_demand_is_released(self):
        class Missing(Decoder):
            def VideoCapture(self,*args):
                self.opens+=1
                return SimpleNamespace(isOpened=lambda:False,release=lambda:None)
        decoder=Missing(); feed=CameraFeed(decoder,'192.168.20.10','front'); self.addCleanup(feed.close)
        feed.subscribe(); worker=feed.thread
        deadline=time.monotonic()+1
        while decoder.opens<1 and time.monotonic()<deadline:time.sleep(.005)
        feed.unsubscribe(immediate=True); worker.join(.3)
        self.assertFalse(worker.is_alive()); self.assertEqual(decoder.opens,1)
    def test_flapping_single_good_frames_do_not_reset_exponential_backoff(self):
        class Flapping(Decoder):
            def VideoCapture(self,*args):
                cap=super().VideoCapture(*args); count=[0]; read=cap.read
                def flap():
                    count[0]+=1
                    return read() if count[0]==1 else (False,None)
                cap.read=flap; return cap
        decoder=Flapping(); feed=CameraFeed(decoder,'192.168.20.10','front'); self.addCleanup(feed.close)
        feed.subscribe(); deadline=time.monotonic()+1.2
        while feed.consecutive_failures<2 and time.monotonic()<deadline:time.sleep(.005)
        self.assertEqual(feed.consecutive_failures,2); self.assertEqual(decoder.opens,2)
        self.assertGreater(feed.status()['retry_in'],.5)
    def test_low_fps_encoding_keeps_draining_input_and_publishes_latest_only(self):
        decoder=Decoder(); feed=CameraFeed(decoder,'192.168.20.10','front',jpeg_width=480,max_fps=2.)
        self.addCleanup(feed.close); feed.subscribe(); first=feed.read(timeout=1)
        newer=feed.read(after=first[0],timeout=2)
        # The feed keeps draining frames while publishing at 2 fps, so later
        # publications skip raw frames. Exact skip counts follow the scheduler.
        self.assertGreater(newer[0]-first[0],3); self.assertLess(len(decoder.encodes),decoder.reads)
        self.assertLessEqual(len(decoder.encodes),4)
        # Age stays within one throttle interval (0.5 s at 2 fps) plus margin.
        self.assertLess(feed.read_packet(timeout=.5)[2],.6)


if __name__=='__main__':unittest.main()

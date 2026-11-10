"""Local weights, PPE evidence logic, and shared-camera output. No robot/network."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.ppe_runtime import HelmetAlerts,SharedPPEDetector,PPESources
from m20.vision_models import ModelChecks,catalog,default_path
import m20.m20_console as original


def detection(name='NO-Hardhat',x=100):
    return dict(class_id=2 if name=='NO-Hardhat' else 0,class_name=name,confidence=.9,x1=x,y1=50,x2=x+40,y2=90)


class HelmetAlertTests(unittest.TestCase):
    def frames(self,checker,rows,start=0,count=23):
        events=[]
        for i in range(count):events+=checker.update(rows,start+i*.1)
        return events
    def test_hat_on_one_person_does_not_suppress_another_bare_head(self):
        events=self.frames(HelmetAlerts(),[detection('Hardhat',20),detection('NO-Hardhat',180)])
        self.assertEqual(len(events),1);self.assertEqual(events[0]['type'],'NO-Hardhat')
    def test_two_bare_heads_receive_independent_timers_and_cooldowns(self):
        checker=HelmetAlerts();rows=[detection(x=20),detection(x=180)]
        events=self.frames(checker,rows);self.assertEqual(len(events),2)
        self.assertEqual(len({e['track_id'] for e in events}),2)
        self.assertEqual(self.frames(checker,rows,2.3,30),[])
    def test_person_without_visible_head_is_unknown(self):
        self.assertEqual(self.frames(HelmetAlerts(),[detection('Person')],count=50),[])
    def test_missing_frame_or_changed_target_cannot_inherit_elapsed_time(self):
        checker=HelmetAlerts();self.frames(checker,[detection()],count=15)
        checker.update([],1.5)
        self.assertEqual(self.frames(checker,[detection()],1.6,15),[])
        self.assertEqual(self.frames(checker,[detection(x=300)],3.1,15),[])
    def test_same_head_conflicting_evidence_does_not_alert(self):
        self.assertEqual(self.frames(HelmetAlerts(),[detection(),detection('Hardhat')],count=50),[])
    def test_long_frame_gap_resets_persistence(self):
        checker=HelmetAlerts();self.frames(checker,[detection()],count=15)
        self.assertEqual(checker.update([detection()],5),[])


class PPEStreamTests(unittest.TestCase):
    def test_ppe_http_frame_exposes_latest_sequence_and_capture_age(self):
        from urllib.request import urlopen
        import m20_platform as web
        from m20.platform_core import Store
        old=web.APP;old_ppe=original.PPE
        with tempfile.TemporaryDirectory() as directory:
            web.APP=web.Platform(Store(Path(directory)/'workspace.json'));web.APP.demo=False
            web.TOKEN=None;web.ACCESS=None
            calls=[]
            def detector(camera):
                def frame(after,timeout):calls.append((camera,after));return 8,b'\xff\xd8frame',.04
                return NS(enabled=True,frame=frame)
            original.PPE=NS(enabled=True,detector=detector)
            server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                with urlopen('http://127.0.0.1:%s/api/ppe/frame?camera=rear&after=7'%server.server_address[1]) as response:
                    self.assertEqual(response.headers['Content-Type'],'image/jpeg')
                    self.assertEqual(response.headers['X-Frame-Sequence'],'8')
                    self.assertEqual(float(response.headers['X-Frame-Age']),.04)
                    self.assertEqual(response.read(),b'\xff\xd8frame')
                self.assertEqual(calls,[('rear',7)])
            finally:
                server.shutdown();server.server_close();thread.join(1);web.APP=old;original.PPE=old_ppe

    def test_dual_detectors_reuse_camera_feeds_and_share_one_model(self):
        import cv2,numpy as np
        class Feed:
            def __init__(self):
                self.condition=threading.RLock();self.sequence=1;self.at=time.time();self.users=0
                self.jpeg=cv2.imencode('.jpg',np.zeros((240,320,3),np.uint8))[1].tobytes()
            def subscribe(self):self.users+=1
            def unsubscribe(self):self.users-=1
            def read(self,after,timeout):
                if after!=1:return 1,self.jpeg
                time.sleep(.02);return None
            def status(self):return dict(error=None)
        feeds={key:Feed() for key in ('front','rear')};requested=[];loads=[]
        def get(camera):requested.append(camera);return feeds[camera]
        detectors=PPESources(NS(cameras=NS(get=get)))
        model=NS(names={0:'Hardhat',2:'NO-Hardhat',5:'Person'})
        def load(worker):loads.append(worker.cam);worker.model=model;return True
        with patch.object(original,'PPE_MODEL_PATH',str(default_path('ppe'))),patch.object(SharedPPEDetector,'_load_model',load),patch('m20.ppe_runtime.detections',return_value=[detection()]),patch.object(original.cv2,'VideoCapture',side_effect=AssertionError('duplicate RTSP')):
            ok,_=detectors.start('both');self.assertTrue(ok)
            for key in feeds:
                packet=detectors.detector(key).frame(-1,2)
                self.assertIsNotNone(packet);self.assertTrue(packet[1].startswith(b'\xff\xd8'))
            self.assertEqual(detectors.status()['camera'],'both')
            self.assertCountEqual(requested,['front','rear']);self.assertEqual(len(loads),1)
            detectors.stop()
            for key,feed in feeds.items():
                worker=detectors.detector(key);worker._thread.join(1)
                self.assertEqual(feed.users,0);self.assertIsNone(worker.frame(-1,.01))

    def test_restart_waits_for_a_new_annotation(self):
        worker=SharedPPEDetector(NS(cameras=NS(get=lambda _camera:None)))
        worker.sequence=4
        worker.last_frame_time=time.time()
        worker._annotated=b'previous frame'
        worker.last_dets=[detection()]
        with patch.object(original,'PPE_MODEL_PATH',str(default_path('ppe'))),patch.object(worker,'_run',side_effect=lambda _feed:worker.halt.wait(1)):
            self.assertTrue(worker.start('rear')[0])
            try:
                status=worker.status()
                self.assertFalse(status['fresh']);self.assertEqual(status['det_count'],0)
                self.assertIsNone(worker.frame(-1,.01));self.assertEqual(worker.sequence,4)
            finally:
                worker.stop();worker._thread.join(1)


class TrainedWeightsTests(unittest.TestCase):
    def test_bundled_model_paths_are_portable_and_present(self):
        self.assertTrue(all(row['installed'] for row in catalog()))
        self.assertEqual(default_path('ppe').name,'ppe.pt')
        self.assertEqual(default_path('person').name,'yolo11n.pt')
    def test_actual_models_run_locally_on_public_image(self):
        with tempfile.TemporaryDirectory() as directory:
            checks=ModelChecks(directory)
            for kind in ['person','ppe','counting']:
                result=checks.run(kind,lambda *args:None)
                self.assertTrue(result['counts']);self.assertTrue(result['image'].startswith('/api/vision/check-image/'))
                self.assertEqual(json.loads(json.dumps(result))['counts'],result['counts'])
                if kind in ['person','counting']:self.assertGreaterEqual(result['counts'].get('person',0),1)
                else:
                    self.assertTrue({'Hardhat','NO-Hardhat','Person'}<=set(result['labels'].values()))
                    self.assertGreaterEqual(result['counts'].get('NO-Hardhat',0),1)
    def test_check_rejects_arbitrary_model_identifier(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):ModelChecks(directory).run('../other',lambda *args:None)


if __name__=='__main__':unittest.main()

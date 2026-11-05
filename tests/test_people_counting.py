"""Counting and lifecycle tests. No real camera or robot connection."""
import sys,time,threading,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.people_counting import CountLedger,PeopleCounter,TRACKER

class LedgerTests(unittest.TestCase):
    def test_confirmation_stops_one_frame_false_positive(self):
        c=CountLedger();self.assertEqual(c.update([1]),set());c.update([]);c.update([1]);self.assertEqual(c.total,0)
        self.assertEqual(c.update([1]),{1});self.assertEqual(c.total,1)
    def test_current_total_and_peak_have_different_meanings(self):
        c=CountLedger();c.update([1,2]);c.update([1,2]);self.assertEqual((c.current,c.total,c.peak),(2,2,2))
        c.update([2]);c.update([2]);self.assertEqual((c.current,c.total,c.peak),(1,2,2))
        c.update([1,2,3]);c.update([1,2,3]);self.assertEqual((c.current,c.total,c.peak),(3,3,3))
    def test_duplicate_ids_do_not_count_twice_and_new_segment_may_recount(self):
        c=CountLedger();c.update([1,1]);c.update([1,1]);self.assertEqual(c.total,1)
        c.break_segment();c.update([1]);c.update([1]);self.assertEqual(c.total,2)

class Feed:
    def __init__(self,once=False):
        self.condition=threading.RLock();self.sequence=0;self.at=0;self.users=0;self.allowed=True;self.once=once
        self.jpeg=cv2.imencode('.jpg',np.zeros((240,320,3),np.uint8))[1].tobytes()
    def subscribe(self):self.users+=1
    def unsubscribe(self):self.users-=1
    def read(self,after,timeout):
        time.sleep(.025)
        if not self.allowed or self.once and self.sequence:return None
        with self.condition:self.sequence+=1;self.at=time.time();return self.sequence,self.jpeg
    def status(self):return dict(error=None)

class Boxes(list):
    id=np.array([1,2])

class FakeModel:
    names={0:'person',1:'car'}
    def __init__(self):
        self.calls=0;self.resets=0;self.predictor=NS(trackers=[NS(reset=self.reset)]);self.started=None;self.release=None
    def reset(self):self.resets+=1
    def track(self,*args,**kwargs):
        self.calls+=1
        if self.started:self.started.set();self.release.wait(2)
        return [NS(boxes=Boxes([NS(id=np.array([i]),cls=np.array([0]),conf=np.array([.9]),xyxy=np.array([[i*60,40,i*60+30,120]])) for i in [1,2]]))]

class RuntimeTests(unittest.TestCase):
    def wait(self,fn,timeout=3):
        end=time.monotonic()+timeout
        while not fn() and time.monotonic()<end:time.sleep(.02)
        self.assertTrue(fn())
    def make(self,feed=None,model=None):
        feed=feed or Feed();model=model or FakeModel();app=NS(demo=False,cameras=NS(get=lambda cam:feed))
        counter=PeopleCounter(app,model_factory=lambda:model)
        self.addCleanup(lambda:(counter.stop(),counter.thread.join(3) if counter.thread else None))
        return counter,feed,model
    def test_shared_feed_counts_pause_resume_and_cleanup(self):
        c,feed,model=self.make()
        with patch('cv2.VideoCapture',side_effect=AssertionError('second RTSP connection')):
            c.start();self.wait(lambda:c.status()['current']==2)
            self.assertEqual(feed.users,1);self.assertEqual(c.status()['total_tracks'],2)
            c.control({'action':'pause'});self.assertIsNone(c.status()['current']);self.assertEqual(c.status()['peak'],2)
            calls=model.calls;time.sleep(.1);self.assertLessEqual(model.calls,calls+1)
            c.control({'action':'resume'});self.wait(lambda:c.status()['current']==2)
            self.assertEqual(c.status()['total_tracks'],4)
            c.stop();c.thread.join(2);self.assertEqual(feed.users,0);self.assertIsNone(c.frame(timeout=.01))
    def test_reset_rejects_result_from_inflight_old_epoch(self):
        m=FakeModel();m.started=threading.Event();m.release=threading.Event();c,feed,m=self.make(Feed(once=True),m)
        c.start();self.assertTrue(m.started.wait(2));c.control({'action':'reset'});m.release.set();time.sleep(.1)
        self.assertEqual(c.status()['frames'],0);self.assertEqual(c.status()['total_tracks'],0);self.assertIsNone(c.frame(timeout=.01))
    def test_stale_frame_is_not_reported_as_zero_people_or_fresh_video(self):
        c,feed,m=self.make();c.start();self.wait(lambda:c.status()['current']==2);feed.allowed=False
        self.wait(lambda:not c.status()['fresh'],2)
        self.assertIsNone(c.status()['current']);self.assertEqual(c.status()['peak'],2);self.assertIsNone(c.frame(timeout=.01))
        feed.allowed=True;self.wait(lambda:c.status()['fresh']);self.assertGreaterEqual(m.resets,2)
    def test_confidence_validation_and_demo_do_not_open_capture(self):
        c,feed,m=self.make()
        for value in [True,float('nan'),0,.99,'bad']:
            with self.assertRaises(ValueError):c.control(dict(action='configure',conf=value))
        c.app.demo=True
        with self.assertRaisesRegex(ValueError,'实时人数'):c.start()
        self.assertEqual(feed.users,0)
    def test_different_camera_requires_a_new_session(self):
        c,feed,m=self.make();c.start('front');self.wait(lambda:c.status()['fresh'])
        with self.assertRaisesRegex(ValueError,'切换相机'):c.start('rear')
        self.assertEqual(c.status()['camera'],'front')

class ActualWeightsTests(unittest.TestCase):
    def test_supplied_yolo12n_tracks_public_sample_and_counts_once(self):
        from ultralytics import YOLO
        root=Path(__file__).resolve().parents[1];model=YOLO(str(root/'models/yolo12n.pt'));frame=cv2.imread(str(root/'models/samples/persons.jpg'));ledger=CountLedger()
        for _ in range(4):
            result=model.track(frame,persist=True,tracker=str(TRACKER),classes=[0],conf=.3,imgsz=480,verbose=False,device='cpu')[0]
            ids=[] if result.boxes.id is None else result.boxes.id.int().tolist();ledger.update(ids)
        self.assertGreaterEqual(ledger.current,1);self.assertEqual(ledger.current,ledger.total);self.assertEqual(ledger.current,ledger.peak)

class FrameHTTPTests(unittest.TestCase):
    def test_snapshot_has_filename_and_frame_age(self):
        import m20_platform as web
        from m20.platform_core import Store
        from urllib.request import urlopen
        old=web.APP
        with tempfile.TemporaryDirectory() as directory:
            web.APP=web.Platform(Store(Path(directory)/'workspace.json'));web.TOKEN=None;web.ACCESS=None
            web.APP.people=NS(frame=lambda **kw:(9,b'jpeg-bytes',.05),status=lambda:{})
            server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler);t=threading.Thread(target=server.serve_forever,daemon=True);t.start()
            try:
                with urlopen('http://127.0.0.1:%d/api/people/snapshot'%server.server_port,timeout=3) as response:
                    self.assertIn('attachment',response.headers['Content-Disposition']);self.assertEqual(response.headers['X-Frame-Sequence'],'9');self.assertEqual(response.read(),b'jpeg-bytes')
            finally:server.shutdown();server.server_close();web.APP.hub.close();web.APP=old

if __name__=='__main__':unittest.main()

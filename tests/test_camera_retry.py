"""Offline regressions for a stream that rejects RTSP open immediately."""
import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_streams import Cameras

class MissingStream:
    CAP_FFMPEG=1;CAP_PROP_OPEN_TIMEOUT_MSEC=2;CAP_PROP_READ_TIMEOUT_MSEC=3
    def __init__(self,error=''):self.opens=0;self.error=error
    def VideoCapture(self,*args):
        self.opens+=1
        class Capture:
            def isOpened(self):return False
            def release(self):pass
        capture=Capture();capture.error=self.error
        return capture

class CameraRetryTests(unittest.TestCase):
    def test_confirmed_rtsp_404_explains_missing_stream_without_exposing_url(self):
        for camera,path,view in [('rear','video2','后视'),('front','video1','前视')]:
            with self.subTest(camera=camera):
                decoder=MissingStream('method DESCRIBE failed: 404 (Not Found)\nError opening input file rtsp://user:secret@127.0.0.1/'+path)
                pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get(camera)
                try:
                    feed.subscribe();deadline=time.monotonic()+1
                    while not feed.status()['error'] and time.monotonic()<deadline:time.sleep(.01)
                    status=feed.status()
                    self.assertIn(view+' RTSP /'+path+' 返回 404',status['error'])
                    self.assertIn('机器人视频服务当前未提供这路视频流',status['error'])
                    self.assertNotIn('secret',status['error'])
                    self.assertFalse(status['fresh']);self.assertEqual(status['frames'],0)
                finally:pool.close()

    def test_missing_stream_backs_off_and_manual_retry_wakes_it(self):
        decoder=MissingStream();pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get('rear')
        try:
            feed.subscribe();deadline=time.monotonic()+2
            while decoder.opens<2 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(decoder.opens,2)
            status=feed.status();self.assertFalse(status['fresh']);self.assertEqual(status['frames'],0)
            self.assertEqual(status['phase'],'opening');self.assertIn('后视 RTSP /video2',status['error'])
            self.assertGreater(status['retry_in'],.5)
            time.sleep(.2);self.assertEqual(decoder.opens,2)
            feed.reconnect();deadline=time.monotonic()+.4
            while decoder.opens<3 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(decoder.opens,3)
        finally:pool.close()

    def test_close_interrupts_retry_wait(self):
        decoder=MissingStream();pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get('front')
        feed.subscribe();thread=feed.thread;pool.close();thread.join(.5)
        self.assertFalse(thread.is_alive())

if __name__=='__main__':unittest.main()

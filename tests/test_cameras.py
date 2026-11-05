"""Camera fan-out tests use a synthetic decoder and never open RTSP."""
import sys
from pathlib import Path
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.camera_streams import Cameras


class Decoder:
    CAP_FFMPEG=1;CAP_PROP_OPEN_TIMEOUT_MSEC=2;CAP_PROP_READ_TIMEOUT_MSEC=3;IMWRITE_JPEG_QUALITY=4
    def __init__(self):self.opens=0
    def VideoCapture(self,*args):
        self.opens+=1
        class Capture:
            def isOpened(self):return True
            def read(self):
                time.sleep(.02)
                class Frame:shape=(2,4,3)
                return True,Frame()
            def release(self):pass
        return Capture()
    def imencode(self,*args):
        class Encoded:
            def tobytes(self):return b'jpeg-fixture'
        return True,Encoded()


class CameraTests(unittest.TestCase):
    def test_multiple_viewers_share_decoder_and_ip_change_discards_old_feed(self):
        ip=['127.0.0.1'];decoder=Decoder();pool=Cameras(decoder,lambda:ip[0])
        try:
            first=pool.get('front');first.subscribe();second=pool.get('front');second.subscribe()
            frame=first.read(timeout=1)
            self.assertIs(first,second);self.assertEqual(decoder.opens,1);self.assertEqual(frame[1],b'jpeg-fixture')
            self.assertTrue(first.status()['fresh']);self.assertEqual(first.status()['viewers'],2)
            first.unsubscribe();self.assertEqual(second.status()['viewers'],1)
            ip[0]='127.0.0.2';new=pool.get('front')
            self.assertTrue(first.closed);self.assertIsNot(first,new);self.assertFalse(new.status()['fresh'])
        finally:pool.close()
    def test_missing_decoder_and_demo_cannot_claim_live_video(self):
        pool=Cameras(None,lambda:'127.0.0.1')
        with self.assertRaisesRegex(ValueError,'OpenCV'):pool.get('rear').subscribe()
        self.assertFalse(pool.status()['available']);self.assertFalse(pool.status()['rear']['fresh'])
        pool.close()
        with self.assertRaises(ValueError):Cameras(Decoder(),lambda:None).get('front')



class CameraFrameHTTPTests(unittest.TestCase):
    def test_latest_frame_endpoint_shares_capture_and_sends_sequence(self):
        import tempfile,threading
        from urllib.request import urlopen
        from types import SimpleNamespace
        import m20_platform as web
        from m20.platform_core import Store
        with tempfile.TemporaryDirectory() as temp:
            web.APP=web.Platform(Store(Path(temp)/'workspace.json'));web.TOKEN=None;web.ACCESS=None
            web.APP.demo=False;web.APP.link=SimpleNamespace(ip='127.0.0.1')
            decoder=Decoder();web.APP.cameras=Cameras(decoder,lambda:'127.0.0.1')
            server=web.ThreadingHTTPServer(('127.0.0.1',0),web.Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
            url='http://127.0.0.1:'+str(server.server_port)+'/api/camera/frame?camera=front'
            try:
                with urlopen(url,timeout=3) as response:
                    seq=int(response.headers['X-Frame-Sequence']);self.assertEqual(response.headers['Content-Type'],'image/jpeg');self.assertEqual(response.read(),b'jpeg-fixture')
                with urlopen(url+'&after='+str(seq),timeout=3) as response:self.assertGreater(int(response.headers['X-Frame-Sequence']),seq)
                self.assertEqual(decoder.opens,1)
            finally:
                server.shutdown();server.server_close();web.APP.cameras.close();web.APP.hub.close();web.APP.done=True


class LatestFrameTests(unittest.TestCase):
    def wait_for(self, predicate, seconds=2):
        end=time.monotonic()+seconds
        while not predicate() and time.monotonic()<end:time.sleep(.01)
        self.assertTrue(predicate())

    def test_slow_jpeg_encoding_skips_to_latest_capture_instead_of_blocking_reader(self):
        class Slow(Decoder):
            def imencode(self,*args):
                time.sleep(.12)
                return super().imencode(*args)
        pool=Cameras(Slow(),lambda:'127.0.0.1');f=pool.get('front')
        try:
            f.subscribe();self.wait_for(lambda:f.sequence>=12)
            self.assertGreater(f.raw_sequence,f.sequence+2)
            self.assertGreater(f.status()['skipped'],4)
            self.assertLess(f.read_packet(timeout=.1)[2],.3)
        finally:pool.close()

    def test_encoder_completion_does_not_make_old_input_fresh(self):
        class Slow(Decoder):
            def imencode(self,*args):
                time.sleep(1.1)
                return super().imencode(*args)
        pool=Cameras(Slow(),lambda:'127.0.0.1');f=pool.get('front')
        try:
            f.subscribe();time.sleep(1.2)
            self.assertIsNone(f.read(timeout=.01));self.assertFalse(f.status()['fresh'])
        finally:pool.close()

    def test_reconnect_keeps_subscribers_and_uses_a_new_decoder(self):
        decoder=Decoder();pool=Cameras(decoder,lambda:'127.0.0.1');f=pool.get('front')
        try:
            f.subscribe();f.subscribe();old=f.read(timeout=1)[0]
            f.reconnect();self.wait_for(lambda:decoder.opens>=2)
            self.assertGreater(f.read(after=old,timeout=1)[0],old)
            self.assertEqual(f.viewers,2);self.assertIs(pool.get('front'),f)
        finally:pool.close()

    def test_no_newer_frame_returns_none_and_expired_frame_is_not_repeated(self):
        from m20.camera_streams import CameraFeed
        f=CameraFeed(Decoder(),'127.0.0.1','front')
        f.jpeg=b'frame';f.at=time.time();f.frame_mono=time.monotonic();f.sequence=12
        self.assertIsNone(f.read(after=12,timeout=.01))
        f.frame_mono-=1.2
        self.assertIsNone(f.read(timeout=.01));self.assertFalse(f.status()['fresh'])

    def test_single_decode_thread_selected_at_open(self):
        from m20.camera_streams import capture_params,CAPTURE_OPTIONS
        cv=Decoder();cv.CAP_PROP_N_THREADS=70
        options=capture_params(cv)
        self.assertEqual(options[options.index(70)+1],1)
        self.assertNotIn('low_delay',CAPTURE_OPTIONS)
        self.assertNotIn('nobuffer',CAPTURE_OPTIONS)

if __name__=='__main__':unittest.main()

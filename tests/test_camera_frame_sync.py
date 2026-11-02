"""No RTSP: simulate concealed predictive frames during decoder startup."""
import sys,threading,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_streams import Cameras,decoded_frame_type

class Decoder:
    CAP_FFMPEG=1;CAP_PROP_OPEN_TIMEOUT_MSEC=2;CAP_PROP_READ_TIMEOUT_MSEC=3
    CAP_PROP_FRAME_TYPE=69;IMWRITE_JPEG_QUALITY=4
    def __init__(self,metadata=True):self.ready=threading.Event();self.opens=0;self.reads=0;self.metadata=metadata
    def VideoCapture(self,*args):
        self.opens+=1;self.ready.clear();decoder=self
        class Capture:
            emitted_i=False;kind='P'
            def isOpened(self):return True
            def read(self):
                time.sleep(.01);decoder.reads+=1
                self.kind=('P' if self.emitted_i else 'I') if decoder.ready.is_set() else ('P' if decoder.reads%2 else 'B')
                if self.kind=='I':self.emitted_i=True
                class Frame:shape=(2,4,3)
                frame=Frame();frame.content=b'valid' if self.emitted_i else b'concealed-gray'
                return True,frame
            def get(self,prop):return ord(self.kind) if decoder.metadata else 0
            def release(self):pass
        return Capture()
    def imencode(self,extension,frame,params):
        class Encoded:
            def tobytes(self):return frame.content
        return True,Encoded()

class FrameSyncTests(unittest.TestCase):
    def wait_for(self,test):
        end=time.monotonic()+1
        while not test() and time.monotonic()<end:time.sleep(.01)
        self.assertTrue(test())
    def test_predictive_gray_frames_are_not_published_until_intra_frame(self):
        decoder=Decoder();pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get('front')
        try:
            feed.subscribe();self.wait_for(lambda:decoder.reads>=4)
            self.assertIsNone(feed.read(timeout=.03));s=feed.status()
            self.assertFalse(s['fresh']);self.assertTrue(s['awaiting_keyframe']);self.assertEqual(s['state'],'waiting_keyframe')
            self.assertGreaterEqual(s['startup_frames_discarded'],4);self.assertIn('等待完整视频帧',s['error'])
            decoder.ready.set();self.assertEqual(feed.read(timeout=1)[1],b'valid')
            self.assertTrue(feed.status()['fresh']);self.assertFalse(feed.status()['awaiting_keyframe'])
            self.assertIsNone(feed.status()['error'])
        finally:pool.close()
    def test_every_reconnect_requires_its_own_intra_frame(self):
        decoder=Decoder();pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get('front')
        try:
            feed.subscribe();self.wait_for(lambda:decoder.reads>0);decoder.ready.set();old=feed.read(timeout=1)[0]
            feed.reconnect();self.wait_for(lambda:decoder.opens==2 and feed.status()['awaiting_keyframe'])
            self.assertIsNone(feed.read(after=old,timeout=.04));decoder.ready.set()
            new=feed.read(after=old,timeout=1);self.assertGreater(new[0],old);self.assertEqual(new[1],b'valid')
        finally:pool.close()
    def test_unknown_frame_metadata_keeps_legacy_decoder_usable(self):
        decoder=Decoder(metadata=False);pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get('front')
        try:
            feed.subscribe();self.assertIsNotNone(feed.read(timeout=1));self.assertFalse(feed.status()['frame_type_supported'])
            self.assertFalse(feed.status()['awaiting_keyframe'])
        finally:pool.close()
    def test_predictive_only_decoder_automatically_reopens_and_recovers_both_views(self):
        from unittest.mock import patch
        class RecoveringDecoder(Decoder):
            def VideoCapture(self,*args):
                capture=super().VideoCapture(*args)
                if self.opens>=2:self.ready.set()
                return capture
        for view in ['front','rear']:
            with self.subTest(view=view),patch('camera_streams.MAX_KEYFRAME_WAIT',.06):
                decoder=RecoveringDecoder();pool=Cameras(decoder,lambda:'127.0.0.1');feed=pool.get(view)
                try:
                    feed.subscribe();packet=feed.read(timeout=2)
                    self.assertIsNotNone(packet);self.assertEqual(packet[1],b'valid')
                    self.assertEqual(decoder.opens,2);self.assertGreater(feed.status()['startup_frames_discarded'],0)
                    self.assertEqual(feed.status()['reconnects'],1);self.assertFalse(feed.status()['awaiting_keyframe'])
                finally:pool.close()

    def test_property_is_read_only_and_only_known_ascii_types_are_used(self):
        class Capture:
            def get(self,prop):return 63
        self.assertIsNone(decoded_frame_type(Decoder(),Capture()))
        self.assertIsNone(decoded_frame_type(object(),Capture()))

if __name__=='__main__':unittest.main()

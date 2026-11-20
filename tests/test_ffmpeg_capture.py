"""Strict capture regression using a local generated clip or fake child processes."""
from pathlib import Path
import io,sys,time,unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.ffmpeg_capture import FFmpegCapture,capture_command,find_ffmpeg,read_exact
try:import cv2
except ImportError:cv2=None
ROOT=Path(__file__).resolve().parents[1]

class CaptureContractTests(unittest.TestCase):
    def test_strict_options_and_raw_color_output_do_not_add_a_jpeg_decoder(self):
        args=capture_command('/fixture/ffmpeg','rtsp://127.0.0.1:8554/video1')
        self.assertEqual(args[args.index('-err_detect')+1],'explode')
        self.assertEqual(args[args.index('-c:v')+1],'ppm')
        self.assertEqual(args[args.index('-rtsp_transport')+1],'tcp')
        self.assertEqual(args[args.index('-fps_mode')+1],'passthrough')
        self.assertEqual(args[args.index('-fflags')+1],'+discardcorrupt')
        self.assertNotIn('ignore_err',args)
    def test_preview_sizing_and_legacy_transport_do_not_claim_source_rate_control(self):
        args=capture_command('/fixture/ffmpeg','rtsp://192.168.20.10:8554/video1',4,max_width=480)
        self.assertIn('-stimeout',args); self.assertIn('-vsync',args)
        self.assertEqual(args[args.index('-vf')+1],"scale=w='min(480,iw)':h=-2")
        self.assertEqual(args[args.index('-i')+1],'rtsp://192.168.20.10:8554/video1')
        self.assertNotIn('-b:v',args); self.assertNotIn('-r',args)
        for width in (0,801,True,'480'):
            with self.assertRaises(ValueError):capture_command('/fixture/ffmpeg','fixture',max_width=width)
    def test_raw_ppm_never_invents_original_ipb_metadata(self):
        cap=FFmpegCapture.__new__(FFmpegCapture)
        self.assertEqual(cap.get(69),0)
    def test_short_payload_is_rejected(self):
        with self.assertRaises(EOFError):read_exact(io.BytesIO(b'abc'),4)
        self.assertEqual(read_exact(io.BytesIO(b'abc'),3),b'abc')

@unittest.skipUnless(cv2,'OpenCV not installed')
class CaptureLifecycleTests(unittest.TestCase):
    def test_open_timeout_terminates_child_and_both_pipe_threads(self):
        with patch('m20.ffmpeg_capture.capture_command',return_value=[sys.executable,'-c','import time;time.sleep(30)']):
            cap=FFmpegCapture(cv2,'fixture','unused',open_timeout=.1)
        self.assertFalse(cap.isOpened());self.assertIsNotNone(cap.process.poll())
        self.assertFalse(cap.reader.is_alive());self.assertFalse(cap.errors.is_alive())
        cap.release()  # Idempotent cleanup.
    def test_invalid_image_header_is_bounded_and_releases_process(self):
        child="import sys,time;sys.stdout.buffer.write(b'P6\\n99999999 99999999\\n255\\n');sys.stdout.flush();time.sleep(30)"
        with patch('m20.ffmpeg_capture.capture_command',return_value=[sys.executable,'-c',child]):cap=FFmpegCapture(cv2,'fixture','unused',open_timeout=.5)
        self.assertFalse(cap.isOpened());self.assertIsNotNone(cap.process.poll());self.assertIn('dimensions',cap.error)
    def test_partial_ppm_payload_is_never_a_complete_frame(self):
        child="import sys;sys.stdout.buffer.write(b'P6\\n2 2\\n255\\n'+b'x'*11);sys.stdout.flush()"
        with patch('m20.ffmpeg_capture.capture_command',return_value=[sys.executable,'-c',child]):
            cap=FFmpegCapture(cv2,'fixture','unused',open_timeout=.5)
        self.assertFalse(cap.isOpened()); self.assertIn('ended early',cap.error)
        self.assertEqual(cap.frames.qsize(),0); self.assertIsNotNone(cap.process.poll())
    def test_demand_cancellation_interrupts_initial_frame_wait(self):
        import threading
        cancelled=threading.Event();timer=threading.Timer(.08,cancelled.set);timer.start()
        started=time.monotonic()
        try:
            with patch('m20.ffmpeg_capture.capture_command',return_value=[sys.executable,'-c','import time;time.sleep(30)']):
                cap=FFmpegCapture(cv2,'fixture','unused',open_timeout=5.,cancelled=cancelled.is_set)
            self.assertFalse(cap.isOpened()); self.assertLess(time.monotonic()-started,1.)
            self.assertIsNotNone(cap.process.poll()); self.assertFalse(cap.reader.is_alive()); self.assertFalse(cap.errors.is_alive())
        finally:timer.cancel()
    @unittest.skipUnless(find_ffmpeg(),'FFmpeg 6+ not installed')
    def test_local_hevc_is_color_and_slow_consumers_receive_latest_only(self):
        cap=FFmpegCapture(cv2,str(ROOT/'tests/fixtures/video/camera-codec-fixture.mp4'),find_ffmpeg())
        try:
            self.assertTrue(cap.isOpened(),cap.error);ok,frame=cap.read();self.assertTrue(ok)
            self.assertEqual(frame.shape,(600,800,3));self.assertGreater(float(frame.std()),10)
            self.assertTrue(cap.isOpened());time.sleep(.35);ok,frame=cap.read()
            self.assertTrue(ok);self.assertLess(time.monotonic()-cap.last_frame_mono,.2);self.assertLessEqual(cap.frames.qsize(),1)
        finally:cap.release()
        self.assertIsNotNone(cap.process.poll());self.assertFalse(cap.reader.is_alive());self.assertFalse(cap.errors.is_alive())

if __name__=='__main__':unittest.main()

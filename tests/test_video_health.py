import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bridge.video_health import matching_pids,probe,publisher_command


class VideoHealthTests(unittest.TestCase):
    def test_cleanup_never_matches_diagnostic_text_or_other_cameras(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            commands={1:['gst-launch-1.0','device=/dev/video2','location=rtsp://127.0.0.1:8554/video2'],
                      2:['python3','-c','gst-launch-1.0 device=/dev/video2'],
                      3:['gst-launch-1.0','device=/dev/video20','location=rtsp://127.0.0.1:8554/video2'],
                      4:['gst-launch-1.0','device=/dev/video2','location=rtsp://127.0.0.1:9000/other'],
                      5:['python3',str(Path(__file__).resolve().parents[1]/'bridge/video_publisher.py'),'device=/dev/video2','location=rtsp://127.0.0.1:8554/video2'],
                      6:['python3','-c','video_publisher.py device=/dev/video2']}
            for pid,args in commands.items():
                (p/str(pid)).mkdir();(p/str(pid)/'cmdline').write_bytes(('\0'.join(args)+'\0').encode())
            self.assertEqual(sorted(matching_pids('/dev/video2',p)),[1,5])

    def test_publisher_keeps_cpu_affinity_device_and_codec_arguments(self):
        args=['taskset','-c','4,5','gst-launch-1.0','v4l2src','device=/dev/video2','!','mpph265enc','gop=2']
        result=publisher_command(args)
        self.assertEqual(result[:3],args[:3]);self.assertEqual(result[5:],args[4:])
        self.assertEqual(Path(result[4]).name,'video_publisher.py')

    def test_probe_requires_two_decoded_frames_not_successful_handshake(self):
        for code,payload,good in [(0,b'',False),(0,b'x'*256,False),(0,b'x'*512,True),(1,b'x'*512,False)]:
            with self.subTest(code=code,length=len(payload)),patch('bridge.video_health.subprocess.run',return_value=subprocess.CompletedProcess([],code,payload,b'')):
                self.assertEqual(probe('video2')[0],good)

    def test_hung_decoder_is_bounded(self):
        with patch('bridge.video_health.subprocess.run',side_effect=subprocess.TimeoutExpired('ffmpeg',4)):
            self.assertEqual(probe('video2'),(False,'decode deadline exceeded'))


if __name__=='__main__':unittest.main()

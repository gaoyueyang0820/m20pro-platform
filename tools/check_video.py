#!/usr/bin/env python3
"""Decode the repository's synthetic HEVC fixture; never opens a camera/network."""
import argparse
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def check(bundled=False):
    import cv2
    import numpy as np
    from ffmpeg_capture import FFmpegCapture,find_ffmpeg
    if bundled:
        import imageio_ffmpeg
        binary=imageio_ffmpeg.get_ffmpeg_exe()
    else:binary=find_ffmpeg()
    if not binary:raise ValueError('FFmpeg 6+ is missing; install requirements-video.txt')
    version=subprocess.check_output([binary,'-version'],text=True,timeout=3).splitlines()[0]
    match=re.search(r'ffmpeg version (\d+)\.',version)
    if not match or int(match.group(1))<6:raise ValueError('FFmpeg 6+ required: '+version)
    fixture=ROOT/'data/network/camera-codec-fixture.mp4'
    if not fixture.is_file():raise ValueError('Synthetic video fixture is missing; download the complete source')
    capture=FFmpegCapture(cv2,str(fixture),binary)
    try:
        if not capture.isOpened():raise ValueError('Synthetic HEVC open failed: '+capture.error)
        for _ in range(3):
            ok,frame=capture.read()
            if not ok or frame is None or float(frame.std())<10:raise ValueError('Synthetic HEVC frame did not decode correctly')
        ok,jpeg=cv2.imencode('.jpg',frame)
        if not ok or not jpeg.size:raise ValueError('JPEG conversion failed')
        return dict(ok=True,source='synthetic_fixture_no_robot',os=platform.system(),machine=platform.machine(),python=platform.python_version(),
                    numpy=np.__version__,opencv=cv2.__version__,ffmpeg=version,frames=3,shape=list(frame.shape),jpeg_bytes=int(jpeg.size))
    finally:capture.release()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--bundled',action='store_true',help='Verify FFmpeg installed with imageio-ffmpeg')
    args=parser.parse_args()
    try:print(json.dumps(check(args.bundled),ensure_ascii=False,indent=2))
    except Exception as exc:print('Video dependency check failed: '+str(exc),file=sys.stderr);sys.exit(1)

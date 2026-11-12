"""Bounded, latest-frame-only capture using an existing FFmpeg 6+ binary.

Strict decoding rejects missing-reference HEVC packets instead of constructing
concealed gray frames. PPM carries raw RGB, avoiding a second JPEG codec pass.
"""
from functools import lru_cache
import os
import queue
import re
import shutil
import subprocess
import threading
import time


@lru_cache(maxsize=8)
def ffmpeg_major(binary):
    try:
        result=subprocess.run([binary,'-version'],capture_output=True,text=True,timeout=2)
        match=re.search(r'ffmpeg version [nN]?(\d+)\.',result.stdout)
        if result.returncode==0 and match:
            return int(match.group(1))
    except (OSError,subprocess.TimeoutExpired):
        pass
    return 0


@lru_cache(maxsize=2)
def find_ffmpeg(minimum=6):
    candidates=[shutil.which('ffmpeg'),'/opt/homebrew/bin/ffmpeg','/usr/local/bin/ffmpeg']
    try:
        import imageio_ffmpeg
        candidates.append(imageio_ffmpeg.get_ffmpeg_exe())
    except (ImportError,RuntimeError):pass
    for candidate in dict.fromkeys(p for p in candidates if p):
        if not os.path.isfile(candidate) or not os.access(candidate,os.X_OK):continue
        if ffmpeg_major(candidate)>=minimum:
            return candidate
    return None


def capture_command(binary,source,legacy_version=None):
    if source.startswith('rtsp://'):
        timeout = '-stimeout' if legacy_version==4 else '-timeout'
        inputs = ['-rtsp_transport', 'tcp', timeout, '3500000']
    elif source.startswith(('http://', 'https://')):
        inputs = ['-rw_timeout', '3500000']
    else:
        inputs = ['-re', '-stream_loop', '-1']
    timing = ['-vsync','0'] if legacy_version else ['-fps_mode','passthrough']
    return [binary,'-hide_banner','-loglevel','error','-nostdin','-filter_threads','1',
            *inputs,'-err_detect','explode','-threads','1','-i',source,
            '-an','-sn','-dn','-vf',"scale=w='min(800,iw)':h=-2",'-pix_fmt','rgb24',
            '-c:v','ppm','-threads','1',*timing,
            '-f','image2pipe','-flush_packets','1','pipe:1']


def read_exact(stream,size):
    parts=[];remaining=size
    while remaining:
        chunk=stream.read(remaining)
        if not chunk:raise EOFError('FFmpeg frame ended early')
        parts.append(chunk);remaining-=len(chunk)
    return b''.join(parts)


class FFmpegCapture:
    def __init__(self,cv,source,binary,open_timeout=5.,read_timeout=1.2,legacy_version=None):
        self.cv=cv;self.closed=False;self.frames=queue.Queue(maxsize=1)
        self.done=threading.Event();self.error='';self.read_timeout=read_timeout
        self.first=None;self.last_frame_mono=0.
        command=capture_command(binary,source,legacy_version) if legacy_version else capture_command(binary,source)
        self.process=subprocess.Popen(command,stdin=subprocess.DEVNULL,
                                      stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0,
                                      creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0) if os.name=='nt' else 0)
        self.reader=threading.Thread(target=self._read_frames,daemon=True)
        self.errors=threading.Thread(target=self._read_errors,daemon=True)
        self.reader.start();self.errors.start()
        self.first=self._next(open_timeout);self.opened=self.first is not None
        if not self.opened:self.release()

    def _read_errors(self):
        try:
            while not self.closed:
                chunk=self.process.stderr.read(1024)
                if not chunk:return
                self.error=(self.error+chunk.decode('utf8','replace'))[-2048:]
        except (OSError,ValueError):pass

    def _read_frames(self):
        try:
            while not self.closed:
                magic=self.process.stdout.readline(64)
                if not magic:break
                if magic.strip()!=b'P6':raise ValueError('Unexpected FFmpeg image header')
                width,height=map(int,self.process.stdout.readline(64).split())
                peak=int(self.process.stdout.readline(64))
                if not (0<width<=800 and 0<height and width*height<=4_000_000 and peak==255):
                    raise ValueError('Invalid FFmpeg image dimensions')
                packet=(read_exact(self.process.stdout,width*height*3),width,height,time.monotonic())
                try:self.frames.put_nowait(packet)
                except queue.Full:
                    try:self.frames.get_nowait()
                    except queue.Empty:pass
                    self.frames.put_nowait(packet)
        except (OSError,ValueError,EOFError) as exc:
            if not self.closed:self.error=(self.error+'\n'+str(exc))[-2048:]
        finally:self.done.set()

    def _next(self,timeout):
        deadline=time.monotonic()+timeout
        while not self.closed:
            try:return self.frames.get(timeout=min(.05,max(0.,deadline-time.monotonic())))
            except queue.Empty:
                if self.done.is_set() or time.monotonic()>=deadline:return None
        return None

    def isOpened(self):return not self.closed and self.opened

    def read(self):
        import numpy as np
        if self.closed:return False,None
        packet=self.first;self.first=None
        try:packet=self.frames.get_nowait()
        except queue.Empty:pass
        if packet is None:packet=self._next(self.read_timeout)
        if packet is None:return False,None
        raw,width,height,self.last_frame_mono=packet
        rgb=np.frombuffer(raw,dtype=np.uint8).reshape(height,width,3)
        return True,self.cv.cvtColor(rgb,self.cv.COLOR_RGB2BGR)

    def get(self,prop):
        # Raw RGB output carries no original I/P/B type. Do not invent metadata.
        return 0

    def release(self):
        if self.closed:return
        self.closed=True
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=.7)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1)
        self.reader.join(timeout=1);self.errors.join(timeout=1)
        self.process.stdout.close();self.process.stderr.close()

#!/usr/bin/env python3
"""AOS video-only health tools; called by the existing camera supervisor.

No control API, credentials, extra daemon, or camera power switching.
"""
import argparse
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time


def matching_pids(device=None, proc=Path('/proc')):
    """Match actual executable and exact device token, never diagnostic text."""
    result=[]
    for entry in proc.iterdir():
        if not entry.name.isdigit():continue
        try:
            args=entry.joinpath('cmdline').read_bytes().split(b'\0')
            gst=Path(os.fsdecode(args[0])).name=='gst-launch-1.0'
            worker=(len(args)>1 and Path(os.fsdecode(args[0])).name.startswith('python')
                    and Path(os.fsdecode(args[1]))==Path(__file__).resolve().with_name('video_publisher.py'))
            if not (gst or worker):continue
            if device and os.fsencode('device='+device) not in args:continue
            if not any(a in args for a in (b'location=rtsp://127.0.0.1:8554/video1',b'location=rtsp://127.0.0.1:8554/video2')):continue
            result.append(int(entry.name))
        except (OSError,ValueError):continue
    return result


def probe(stream, timeout=4):
    """Decode two frames locally; a listening port, SDP or live PID is insufficient."""
    if stream not in ('video1','video2'):raise ValueError('invalid stream')
    args=['ffmpeg','-hide_banner','-loglevel','error','-nostdin',
          '-rtsp_transport','tcp','-stimeout','2000000','-err_detect','explode',
          '-threads','1','-i','rtsp://127.0.0.1:8554/'+stream,
          '-an','-sn','-frames:v','2','-vf','scale=16:16','-pix_fmt','gray',
          '-threads','1','-f','rawvideo','pipe:1']
    try:
        p=subprocess.run(args,capture_output=True,timeout=timeout)
    except subprocess.TimeoutExpired:return False,'decode deadline exceeded'
    except OSError as exc:return False,'probe unavailable: '+str(exc)
    if p.returncode==0 and len(p.stdout)==512:return True,'two decoded frames'
    reason=p.stderr.decode('utf8','replace').strip()[-600:]
    return False,reason or 'no complete decoded frames'


def publisher_command(args):
    # Keep taskset and every existing pipeline argument; replace only the runner.
    command=list(args);index=command.index('gst-launch-1.0')
    return command[:index]+[sys.executable,str(Path(__file__).resolve().with_name('video_publisher.py'))]+command[index+1:]


def run_logged(stream,args, directory=Path('/run/m20-video')):
    """Keep bounded startup diagnostics and forward supervisor shutdown to GST."""
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/(stream+'.log');previous=directory/(stream+'.previous.log')
    if path.exists():path.replace(previous)
    env=os.environ.copy()
    # Per-packet DEBUG output previously rotated away the handshake in seconds.
    # The worker emits explicit, low-volume state/progress events instead.
    env.update(GST_DEBUG='2',GST_DEBUG_NO_COLOR='1')
    env.pop('GST_DEBUG_FILE',None)
    child=subprocess.Popen(publisher_command(args),stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env)
    stopping=[None]
    def stop(signum,frame):
        if stopping[0] is None:stopping[0]=time.monotonic()
        if child.poll() is None:
            child.send_signal(signal.SIGCONT)
            child.terminate()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    selector=selectors.DefaultSelector();selector.register(child.stdout,selectors.EVENT_READ)
    log=path.open('wb');size=0
    startup=directory/(stream+'.startup.log');prior_startup=directory/(stream+'.previous-startup.log')
    if startup.exists():startup.replace(prior_startup)
    startup_log=startup.open('wb');startup_size=0
    try:
        log.write(('started pid=%d stream=%s\n'%(child.pid,stream)).encode());log.flush()
        while selector.get_map():
            if stopping[0] is not None and time.monotonic()-stopping[0]>1 and child.poll() is None:child.kill()
            for key,_ in selector.select(.2):
                chunk=os.read(key.fd,4096)
                if not chunk:selector.unregister(key.fileobj);break
                # Preserve the beginning separately from the rotating tail.
                if startup_size<65536:
                    prefix=chunk[:65536-startup_size];startup_log.write(prefix);startup_log.flush();startup_size+=len(prefix)
                if size+len(chunk)>131072:
                    log.close();path.replace(previous);log=path.open('wb');size=0
                log.write(chunk);log.flush();size+=len(chunk)
        return child.wait(timeout=2)
    finally:
        selector.close();log.close();startup_log.close();child.stdout.close()
        if child.poll() is None:child.kill();child.wait()


if __name__=='__main__':
    parser=argparse.ArgumentParser();sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('probe');p.add_argument('stream',choices=['video1','video2'])
    p=sub.add_parser('pids');p.add_argument('device',nargs='?')
    p=sub.add_parser('run');p.add_argument('stream',choices=['video1','video2']);p.add_argument('args',nargs=argparse.REMAINDER)
    a=parser.parse_args()
    if a.command=='pids':print(' '.join(map(str,matching_pids(a.device))))
    elif a.command=='probe':
        ok,message=probe(a.stream);print(message);sys.exit(0 if ok else 1)
    else:sys.exit(run_logged(a.stream,a.args))

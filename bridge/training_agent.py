#!/usr/bin/env python3
"""Run on the NVIDIA workstation with its Isaac Lab Python environment."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.platform_core import Training
from bridge.http_agent import serve


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',required=True,help='rl_training 工程绝对路径')
    parser.add_argument('--python',default=sys.executable,help='已安装 Isaac Lab 的 Python 路径')
    parser.add_argument('--port',type=int,default=8092)
    args=parser.parse_args()
    manager=Training(root=args.root,python=args.python)
    def dispatch(path,req):
        if req is None:
            if path=='/status': return manager.status()
            if path=='/checkpoints': return dict(ok=True,checkpoints=manager.checkpoints())
        else:
            if path=='/preview': return manager.preview(req)
            if path=='/start': return manager.start(req)
            if path=='/stop': return manager.stop(req.get('id'))
        raise ValueError('接口不存在')
    try: serve(args.port,dispatch)
    finally:
        for job in manager.status()['jobs']:
            if job['status']=='running': manager.stop(job['id'])


if __name__=='__main__': main()

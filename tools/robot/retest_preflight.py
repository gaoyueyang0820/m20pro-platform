#!/usr/bin/env python3
"""Default: offline inventory. --live-readonly: GET-only checks via local platform.

Never starts a connection, task, mapping session or motion command. No secrets
are loaded or printed. Requires only the Python standard library.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[2]

def offline():
    manifest=json.loads((ROOT/'release-manifest.json').read_text())
    changed=[]
    for item in manifest['files']:
        file=ROOT/item['path']
        if not file.exists() or hashlib.sha256(file.read_bytes()).hexdigest()!=item['sha256']:
            changed.append(item['path'])
    checkpoint=ROOT/'调试交接/2026-10-30/CHECKPOINT.json'
    saved=json.loads(checkpoint.read_text()) if checkpoint.exists() else {}
    return dict(mode='offline_only',release_baseline=manifest['version'],changed_files=changed,
                handoff='调试交接/2026-10-30/交接与计划.md',
                priorities=saved.get('pending',saved.get('next_priorities',[])),candidate=saved.get('candidate'),
                unfinished_map=saved.get('test_map',{}),robot_requests_sent=0)

def get(path):
    with urlopen('http://127.0.0.1:8080'+path,timeout=5) as response:return json.load(response)

def live_readonly():
    state=get('/api/status')
    result=dict(mode='live_readonly',platform_mode=state.get('mode'),online=state.get('online'),ip=state.get('ip'),
                basic=state.get('basic'),velocity=state.get('velocity'),navigation=state.get('nav',{}).get('status'),
                autonomous_active=state.get('autonomy',{}).get('active'),
                autonomous_message=state.get('autonomy',{}).get('message'),mapping_cached=state.get('mapping'))
    if state.get('mode')!='live' or not state.get('online'):
        result['message']='平台未在线连接实机；未启动连接，也未读取机器人代理。'
        return result
    for key,path in [('mapping','/api/maps/build/status'),('sensors','/api/sensors/status'),('library','/api/maps/library')]:
        try:
            data=get(path)
            if key=='mapping':data={k:data.get(k) for k in ['phase','in_progress','message','result_maps']}
            elif key=='sensors':data={k:data.get(k) for k in ['reader_pid','autonomy_protocol','refinement_protocol','scan_diagnostics','slam_pose_proof','localization_quality','matching_quality','autonomy_error','spin_error']}
            else:data=dict(active=data.get('active'),maps=[{k:m.get(k) for k in ['name','active','grids','has_cloud']} for m in data.get('maps',[])])
            result[key]=data
        except Exception as exc:result[key]={'error':str(exc)}
    result['message']='只读检查完成。此输出不代表允许运动；先按交接计划确认现场与保存状态。'
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-readonly',action='store_true',help='Only GET existing local-platform status; no control operations')
    args=parser.parse_args()
    try:result=live_readonly() if args.live_readonly else offline()
    except Exception as exc:print(json.dumps({'ok':False,'error':str(exc)},ensure_ascii=False));return 1
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0

if __name__=='__main__':sys.exit(main())

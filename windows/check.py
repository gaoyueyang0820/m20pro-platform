"""Offline environment checks. Does not connect to a robot or execute actions."""
from pathlib import Path
import importlib,json,platform,struct,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    results=[]
    def add(name,ok,detail):results.append(dict(check=name,ok=bool(ok),detail=str(detail)))
    add('64-bit Python 3.11-3.13',struct.calcsize('P')==8 and (3,11)<=sys.version_info[:2]<=(3,13),sys.version.split()[0])
    for module in ['numpy','cv2','torch','torchvision','ultralytics','lap','paramiko','imageio_ffmpeg']:
        try:m=importlib.import_module(module);add(module,True,getattr(m,'__version__','loaded'))
        except Exception as exc:add(module,False,str(exc))
    try:
        from ffmpeg_capture import find_ffmpeg
        binary=find_ffmpeg();add('FFmpeg 6+ strict decoder',bool(binary),binary or 'not found')
    except Exception as exc:add('FFmpeg',False,str(exc))
    for name in ['yolo11n.pt','ppe.pt','yolo12n.pt']:add('model '+name,(ROOT/'models'/name).is_file(),name)
    for name in ['platform_web/index.html','platform_web/vendor/viewer.js','reference/errors.md','platform_web/assets/model/M20.urdf']:
        add('asset', (ROOT/name).is_file(),name)
    ssh=[]
    try:
        from platform_host import ssh_executable
        for tool in ['ssh','ssh-keygen','ssh-keyscan']:ssh.append(ssh_executable(tool))
        print('OpenSSH tools: '+', '.join(ssh))
    except ValueError as exc:print('Optional for demo/video; required for robot services: '+str(exc))
    for result in results:print(('PASS ' if result['ok'] else 'FAIL ')+result['check']+': '+result['detail'])
    if not all(r['ok'] for r in results):
        print('For a torch DLL error, install Microsoft Visual C++ x64 Redistributable (see README-Windows.md).')
        raise SystemExit(1)
    print('Offline checks passed. This does not verify Windows camera/SSH/robot operation.')

if __name__=='__main__':main()

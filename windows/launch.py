"""Start the original platform with per-user data; never stop an existing server."""
from pathlib import Path
import json,os,socket,sys,threading,time,webbrowser
from urllib.request import urlopen
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def existing_platform(port):
    try:
        with urlopen('http://127.0.0.1:%d/api/platform'%port,timeout=2) as response:value=json.load(response)
        if value.get('mode') in ('demo','live') and isinstance(value.get('nav'),dict) and isinstance(value.get('joints'),list):return value
    except Exception:pass
    return None

def main():
    if os.name!='nt':raise SystemExit('Use the existing launcher for your operating system.')
    mode=sys.argv[1] if len(sys.argv)>1 else 'start'
    if mode not in ['start','demo','share']:raise SystemExit('Unknown launcher mode')
    port=int(os.environ.get('M20_HTTP_PORT','8080'));url='http://127.0.0.1:%d/'%port
    existing=existing_platform(port)
    if existing:
        if mode=='demo' and existing['mode']!='demo':raise SystemExit('The live platform is already running on this port. It was not changed or stopped.')
        if mode=='share':raise SystemExit('A platform is already running. Close it after tasks finish before starting shared mode.')
        print('Opening the existing platform: '+url);webbrowser.open(url);return
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=.5):raise SystemExit('Port %d is used by another service. No process was stopped.'%port)
    except OSError:pass
    data=Path(os.environ.get('LOCALAPPDATA',str(Path.home()/'AppData/Local')))/'M20ProPlatform'
    args=['m20_platform.py','--http',str(port),'--data-dir',str(data)]
    if mode=='demo':args+=['--demo']
    if mode=='share':
        subnet=os.environ.get('M20_SHARE_SUBNET') or input('Private IPv4 subnet/CIDR (example 172.20.10.0/28): ').strip()
        if not subnet:raise SystemExit('No subnet provided; shared mode was not started.')
        args+=['--share',subnet]
    def open_when_ready():
        for _ in range(80):
            if existing_platform(port):webbrowser.open(url);return
            time.sleep(.25)
    threading.Thread(target=open_when_ready,daemon=True).start()
    os.chdir(ROOT);sys.argv=args
    import m20_platform
    print('User data: '+str(data)+'\nPlatform: '+url+'\nKeep this console open while using the platform.')
    m20_platform.main()

if __name__=='__main__':main()

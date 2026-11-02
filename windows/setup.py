"""Run with a supported Windows Python; installs only this project's venv."""
from pathlib import Path
import os,platform,struct,subprocess,sys,venv
ROOT=Path(__file__).resolve().parents[1]
RUNTIME=ROOT/'data/runtime-venv'

def main():
    if os.name!='nt':raise SystemExit('This installer is for Windows. Use the existing Mac launchers on macOS.')
    if struct.calcsize('P')!=8 or not (3,11)<=sys.version_info[:2]<=(3,13) or sysconfig_free_threaded():
        raise SystemExit('Install standard 64-bit Python 3.13 (or 3.12 / 3.11), then run setup again.')
    print('Creating the project environment. The first installation needs Internet access.')
    if not (RUNTIME/'Scripts/python.exe').is_file():venv.EnvBuilder(with_pip=True).create(RUNTIME)
    python=str(RUNTIME/'Scripts/python.exe');env=os.environ.copy();env['PYTHONUTF8']='1'
    def pip(*args):subprocess.run([python,'-X','utf8','-m','pip','--disable-pip-version-check',*args],env=env,check=True)
    pip('install','--upgrade','pip')
    pip('install','torch==2.8.0','torchvision==0.23.0','--index-url','https://download.pytorch.org/whl/cpu')
    pip('install','--only-binary=:all:','-r',str(ROOT/'requirements-windows.txt'))
    subprocess.run([python,'-X','utf8',str(ROOT/'windows/check.py')],env=env,check=True)
    print('\nSetup complete. Run Start-Windows.cmd. Models are already included.')

def sysconfig_free_threaded():
    import sysconfig
    return bool(sysconfig.get_config_var('Py_GIL_DISABLED'))

if __name__=='__main__':
    try:main()
    except subprocess.CalledProcessError as exc:
        print('Installation/check failed. Read the error above, fix the cause and run setup again.');sys.exit(exc.returncode)

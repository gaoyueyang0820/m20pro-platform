"""Local host helpers; remote robot commands continue to use POSIX syntax."""
import csv
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

IS_WINDOWS=os.name=='nt'


def lock_workspace(directory):
    """One backend owns a workspace, regardless of loopback/shared bind address.

    The OS releases the lock on exit/crash. Separate --data-dir workspaces can
    still be used for isolated demos and tests; no stale PID is ever killed.
    """
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    handle=(directory/'.platform-instance.lock').open('a+b')
    try:
        if IS_WINDOWS:
            import msvcrt
            if handle.seek(0,2)==0:handle.write(b'0');handle.flush()
            handle.seek(0);msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
        else:
            import fcntl
            fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except OSError:
        handle.close()
        raise ValueError('这个工作区的平台后台已经在运行，请使用现有网页；独立演示请指定其他 --data-dir')
    return handle


def ssh_executable(name):
    if name not in ('ssh','ssh-keygen','ssh-keyscan'):raise ValueError('Unsupported SSH tool')
    if not IS_WINDOWS:return name
    found=shutil.which(name)
    if found:return found
    path=Path(os.environ.get('SystemRoot',r'C:\Windows'))/'System32/OpenSSH'/(name+'.exe')
    if path.is_file():return str(path)
    raise ValueError('Windows 缺少 OpenSSH 客户端；请在“可选功能”安装 OpenSSH Client 后重开平台')


def ssh_path_option(name,path):
    value=str(path)
    if IS_WINDOWS:value=value.replace('\\','/')
    return name+'="'+value.replace('"','\\"')+'"'


def local_command_line(argv):
    if not IS_WINDOWS:return shlex.join(argv)
    # Keep the selected SSH executable: it may not be available on PATH.
    parts=[str(a).replace('\\','/') for a in argv]
    return subprocess.list2cmdline(parts)


def private_file(path):
    """New platform-owned keys/config only; never touch arbitrary user SSH keys."""
    path=Path(path)
    if not IS_WINDOWS:path.chmod(0o600);return
    system32=Path(os.environ.get('SystemRoot',r'C:\Windows'))/'System32'
    try:
        result=subprocess.run([str(system32/'whoami.exe'),'/user','/fo','csv','/nh'],capture_output=True,text=True,check=True,timeout=5)
        row=next(csv.reader(result.stdout.strip().splitlines()))
        sid=next((v for v in row if re.fullmatch(r'S-1-\d+(?:-\d+)+',v)),None)
        if not sid:raise ValueError('Windows 用户 SID 无效')
        subprocess.run([str(system32/'icacls.exe'),str(path),'/inheritance:r','/grant:r','*'+sid+':F'],capture_output=True,check=True,timeout=5)
    except (OSError,subprocess.SubprocessError,ValueError,StopIteration) as exc:
        raise ValueError('无法设置平台文件的 Windows 私有权限，请检查账号权限与系统工具后重试') from exc

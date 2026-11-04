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
    if IS_WINDOWS:
        # Windows OpenSSH 自己解析 -o 的值：带引号会报 "command-line line 0: invalid quotes"。
        # 参数是以 argv 整串传给进程的，不需要再包引号，只把反斜杠换成正斜杠。
        return name+'='+value.replace('\\','/')
    return name+'="'+value.replace('"','\\"')+'"'


def local_command_line(argv):
    if not IS_WINDOWS:
        return shlex.join(argv)
    # 这条命令行作为 ProxyCommand 由 OpenSSH 启动内层 ssh。
    # - 原生 Windows OpenSSH 通过 CreateProcessW 拉起，可执行文件必须是能被解析的
    #   路径；把整条命令都换成正斜杠会让 CreateProcessW 找不到 exe（error:2）。
    # - Git/MSYS 版 OpenSSH 把命令交给 /bin/sh，sh 会把反斜杠当转义符。
    # 因此：可执行文件用裸名（两类 OpenSSH 都能靠 PATH 解析），其余参数统一正斜杠
    # （原生 OpenSSH 接受正斜杠参数，MSYS sh 需要正斜杠避免反斜杠转义）。
    parts=[str(a) for a in argv]
    if parts:
        parts[0]=os.path.basename(parts[0])
    parts[1:]=[p.replace('\\','/') for p in parts[1:]]
    return subprocess.list2cmdline(parts)


def private_file(path):
    """New platform-owned keys/config only; never touch arbitrary user SSH keys."""
    path=Path(path)
    if not IS_WINDOWS:path.chmod(0o600);return
    # Windows 上通过 icacls 把私钥 ACL 收拢到当前用户，避免 OpenSSH 因权限过宽拒绝。
    # 在便携/受限环境下 whoami 可能不稳定（PATH 缺 System32 时偶发 rc=1），用绝对路径重试；
    # 即便收拢失败也不阻断机器人连接——当前用户本就是文件所有者，OpenSSH 默认即可接受。
    try:
        whoami=os.path.join(os.environ.get('SystemRoot','C:\\Windows'),'System32','whoami.exe')
        result=subprocess.run([whoami,'/user','/fo','csv','/nh'],capture_output=True,text=True,timeout=5)
        result.check_returncode()
        row=next(csv.reader(result.stdout.strip().splitlines()))
        sid=next((v for v in row if re.fullmatch(r'S-1-\d+(?:-\d+)+',v)),None)
        if sid:
            subprocess.run(['icacls',str(path),'/inheritance:r','/grant:r','*'+sid+':F'],capture_output=True,timeout=5)
    except Exception as exc:
        print('警告: 无法为 %s 收拢 ACL（%s: %s），跳过；连接流程继续'%(path,type(exc).__name__,exc))

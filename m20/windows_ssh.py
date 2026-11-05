"""Password bootstrap on Windows without POSIX shebang/SSH_ASKPASS assumptions.

Host keys must already have been explicitly confirmed in the existing UI.
Long-lived key-authenticated tunnels still use the native OpenSSH client.
"""
import subprocess
import time


def bind_host_identity(client,config):
    """Paramiko equivalent of OpenSSH HostKeyAlias; keep RejectPolicy enabled."""
    alias=config.get('host_key_alias')
    if not alias:return
    keys=client.get_host_keys();pinned=keys.lookup(alias)
    if not pinned:raise ValueError('缺少已确认的 AOS 身份记录，请重新核对指纹')
    address=config['host'] if config['ssh_port']==22 else '[%s]:%s'%(config['host'],config['ssh_port'])
    # Do not let an unrelated key already associated with this DHCP address pass.
    if address in keys:del keys[address]
    for algorithm,key in list(pinned.items()):keys.add(address,algorithm,key)


def run_remote(known,key,target,hop,command,payload,password,timeout=180):
    try:import paramiko
    except ImportError:raise ValueError('请先运行 Windows 依赖安装脚本，补齐 Paramiko')
    clients=[];relay=None;channel=None
    def connect(config,secret=None,sock=None):
        client=paramiko.SSHClient();clients.append(client)
        client.load_host_keys(str(known));client.set_missing_host_key_policy(paramiko.RejectPolicy())
        bind_host_identity(client,config)
        client.connect(config['host'],port=config['ssh_port'],username=config['user'],password=secret,
                       key_filename=None if secret else str(key),allow_agent=False,look_for_keys=False,
                       timeout=5,banner_timeout=5,auth_timeout=10,sock=sock)
        return client
    try:
        if hop:
            upstream=connect(hop)
            relay=upstream.get_transport().open_channel('direct-tcpip',(target['host'],target['ssh_port']),('127.0.0.1',0),timeout=5)
        client=connect(target,password,relay)
        channel=client.get_transport().open_session(timeout=10);channel.settimeout(10)
        channel.exec_command(command);channel.sendall(payload.encode('utf-8'));channel.shutdown_write()
        stdout=bytearray();stderr=bytearray();deadline=time.monotonic()+timeout
        while True:
            if channel.recv_ready():stdout.extend(channel.recv(65536))
            if channel.recv_stderr_ready():stderr.extend(channel.recv_stderr(65536))
            if len(stdout)+len(stderr)>8*1024*1024:raise ValueError('远端响应过大，已停止读取')
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():break
            if time.monotonic()>=deadline:raise ValueError('远端操作等待超时，请核对服务状态后再决定是否重试')
            time.sleep(.01)
        return subprocess.CompletedProcess(['paramiko'],channel.recv_exit_status(),stdout.decode('utf-8','replace'),stderr.decode('utf-8','replace'))
    except paramiko.BadHostKeyException:raise ValueError('SSH 主机密钥已变化，请重新核对指纹；连接已拒绝')
    except paramiko.AuthenticationException:raise ValueError('SSH 认证失败，请核对用户名和密码')
    except (paramiko.SSHException,OSError) as exc:
        message=str(exc).replace(password,'[已隐藏]') if password else str(exc)
        raise ValueError('Windows SSH 连接失败：'+message)
    finally:
        if channel:channel.close()
        if relay:relay.close()
        for client in reversed(clients):client.close()

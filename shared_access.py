"""Trusted private LAN access without browser login or control leases."""
import ipaddress
import json
import re
import subprocess
import platform_host


PRIVATE = [ipaddress.ip_network(n) for n in ['10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16']]


def lan_addresses(network):
    """Find assigned addresses in the explicitly selected LAN, without probing peers."""
    if platform_host.IS_WINDOWS:
        command='[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false); Get-NetIPAddress -AddressFamily IPv4 -AddressState Preferred | Select-Object -ExpandProperty IPAddress | ConvertTo-Json -Compress'
        try:
            output=subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-Command',command],capture_output=True,text=True,encoding='utf-8',timeout=5,check=True).stdout
            values=json.loads(output or '[]');values=[values] if isinstance(values,str) else values
            return sorted({v for v in values if ipaddress.ip_address(v) in network})
        except (OSError,ValueError,TypeError,subprocess.SubprocessError):return []
    try:
        output = subprocess.run(['/sbin/ifconfig'], capture_output=True, text=True, timeout=3, check=True).stdout
        values = re.findall(r'\binet (\d+\.\d+\.\d+\.\d+)\b', output)
    except (OSError, subprocess.SubprocessError):
        try:
            output = subprocess.run(['ip', '-j', 'address'], capture_output=True, text=True, timeout=3, check=True).stdout
            values = [a['local'] for row in json.loads(output) for a in row.get('addr_info', []) if a.get('family') == 'inet']
        except (OSError, ValueError, subprocess.SubprocessError):
            values = []
    return sorted({value for value in values if ipaddress.ip_address(value) in network})


class SharedAccess:
    """Trusted private Wi-Fi: no login, access password or control-claim workflow."""
    def __init__(self, data, subnet, password=None, clock=None):
        self.network=ipaddress.ip_network(subnet,strict=True)
        if self.network.version!=4 or not any(self.network.subnet_of(n) for n in PRIVATE):
            raise ValueError('请选择实际使用的私人 IPv4 网段')
        self.urls=[]

    @staticmethod
    def local(peer):
        try:return ipaddress.ip_address(peer).is_loopback
        except ValueError:return False

    def admitted(self,peer):
        try:return self.local(peer) or ipaddress.ip_address(peer) in self.network
        except ValueError:return False

    def status(self,key='',peer=''):
        return dict(ok=True,enabled=True,subnet=str(self.network),urls=self.urls,
                    local=self.local(peer),authenticated=True,authentication_required=False,
                    controlling=True,owner=None,stopping=False)


def shared_listener(data,subnet,port,password=None):
    """Keep localhost usable after leaving the explicitly configured LAN."""
    access=SharedAccess(data,subnet,password)
    addresses=lan_addresses(access.network)
    if not addresses:
        return None,'127.0.0.1','当前网络不在配置的共享网段内，已仅启动本机入口；共享网段配置保留'
    access.urls=['http://%s:%s/'%(address,port) for address in addresses]
    return access,'0.0.0.0',None

"""Find an already paired robot on the computer's directly attached network.

This reads adapter/neighbor inventories, never the default route. A TCP listener
is only a candidate: its complete SSH public key must match the saved AOS key.
No trust records, routes, adapters or remote hosts are changed here.
"""
from concurrent.futures import ThreadPoolExecutor,as_completed,TimeoutError as FuturesTimeout
import ipaddress
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import threading
import time

import platform_host

MAX_CANDIDATES=512
WORKERS=32
PRIVATE_NETWORKS=tuple(ipaddress.IPv4Network(v) for v in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16','100.64.0.0/10'))
VIRTUAL_NAME=re.compile(r'^(?:lo\d*|utun\d*|tun\d*|tap\d*|wg\d*|ppp\d*|ipsec\d*|docker\d*|veth.*|virbr\d*|br[-\d].*|bridge\d*|tailscale\d*|zt[a-z0-9]*|vmnet\d*|vboxnet\d*)$',re.I)


def _capture(argv):
    try:
        result=subprocess.run(argv,capture_output=True,text=True,timeout=3)
        return result.stdout if result.returncode==0 else ''
    except (OSError,subprocess.SubprocessError):return ''


def _interface(value):
    try:return ipaddress.IPv4Interface(value)
    except (ValueError,TypeError):return None


def _local(value):
    return any(value in net for net in PRIVATE_NETWORKS)


def _linux_physical(name):
    device=Path('/sys/class/net')/name
    return (device/'device').exists() or (device/'phy80211').exists()


def _mac_inventory():
    hardware=_capture(['networksetup','-listallhardwareports'])
    devices={name for name in re.findall(r'^Device:\s*(\S+)',hardware,re.M) if not VIRTUAL_NAME.fullmatch(name)}
    interfaces=[]
    for block in re.split(r'(?=^\S[^\n]*:\s+flags=)',_capture(['ifconfig','-a']),flags=re.M):
        header=re.match(r'(\S+):\s+flags=[^\n]*<([^>]*)>',block)
        if not header or header[1] not in devices or 'UP' not in header[2].split(',') or re.search(r'status:\s*inactive',block):continue
        for address,mask in re.findall(r'\binet\s+(\d+\.\d+\.\d+\.\d+)\s+netmask\s+(0x[0-9a-fA-F]+|[\d.]+)',block):
            try:mask=str(ipaddress.IPv4Address(int(mask,16))) if mask.startswith('0x') else mask
            except ValueError:continue
            parsed=_interface(address+'/'+mask)
            if parsed:interfaces.append(dict(name=header[1],address=parsed))
    neighbors=[]
    for address,name in re.findall(r'\((\d+\.\d+\.\d+\.\d+)\)\s+at\s+(?!\(incomplete\))\S+.*?\bon\s+(\S+)',_capture(['arp','-an'])):
        if name in devices:neighbors.append(dict(name=name,address=address))
    return interfaces,neighbors


def _linux_inventory():
    def rows(command):
        try:value=json.loads(_capture(command));return value if isinstance(value,list) else []
        except ValueError:return []
    devices=set()
    for row in rows(['ip','-j','link','show']):
        name=row.get('ifname','')
        if (name and not VIRTUAL_NAME.fullmatch(name) and row.get('link_type')=='ether'
                and not row.get('linkinfo',{}).get('info_kind') and 'UP' in row.get('flags',[]) and _linux_physical(name)):
            devices.add(name)
    interfaces=[]
    for row in rows(['ip','-j','-4','address','show']):
        if row.get('ifname') not in devices:continue
        for addr in row.get('addr_info',[]):
            if addr.get('family')!='inet':continue
            parsed=_interface(str(addr.get('local',''))+'/'+str(addr.get('prefixlen','')))
            if parsed:interfaces.append(dict(name=row['ifname'],address=parsed))
    neighbors=[]
    for row in rows(['ip','-j','-4','neigh','show']):
        state=row.get('state',[]);state=[state] if isinstance(state,str) else state
        if row.get('dev') in devices and not set(state)&{'FAILED','INCOMPLETE'}:
            neighbors.append(dict(name=row['dev'],address=row.get('dst')))
    return interfaces,neighbors


def _windows_inventory():
    command=("$ErrorActionPreference='Stop';"
        "$a=@(Get-NetAdapter -Physical | Where-Object {$_.Status -eq 'Up'} | Select-Object Name,InterfaceIndex);"
        "$i=@(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object InterfaceIndex,IPAddress,PrefixLength);"
        "$n=@(Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object InterfaceIndex,IPAddress,State);"
        "@{adapters=$a;addresses=$i;neighbors=$n} | ConvertTo-Json -Depth 4 -Compress")
    try:inventory=json.loads(_capture(['powershell.exe','-NoProfile','-NonInteractive','-Command',command]))
    except ValueError:return [],[]
    if not isinstance(inventory,dict):return [],[]
    def rows(key):
        value=inventory.get(key,[]);return value if isinstance(value,list) else [value] if isinstance(value,dict) else []
    devices={str(row['InterfaceIndex']) for row in rows('adapters') if row.get('InterfaceIndex') is not None and not VIRTUAL_NAME.fullmatch(row.get('Name',''))}
    interfaces=[]
    for row in rows('addresses'):
        name=str(row.get('InterfaceIndex'))
        if name not in devices:continue
        parsed=_interface(str(row.get('IPAddress',''))+'/'+str(row.get('PrefixLength','')))
        if parsed:interfaces.append(dict(name=name,address=parsed))
    neighbors=[dict(name=str(row.get('InterfaceIndex')),address=row.get('IPAddress')) for row in rows('neighbors')
               if str(row.get('InterfaceIndex')) in devices and row.get('State') not in ['Unreachable','Incomplete']]
    return interfaces,neighbors


def network_inventory(system=None):
    system=system or sys.platform
    if system=='darwin':return _mac_inventory()
    if system.startswith('linux'):return _linux_inventory()
    if system in ['win32','cygwin']:return _windows_inventory()
    return [],[]


def candidate_addresses(saved,interfaces,neighbors,limit=MAX_CANDIDATES):
    """Prioritize saved address/neighbors, then each physical adapter's /24.

    A wider configured LAN never turns into a /8 or /16 address sweep. Neighbor
    entries may cover the wider attached subnet without enumerating it.
    """
    result=[];seen=set();own={row['address'].ip for row in interfaces}
    attached=[row for row in interfaces if _local(row['address'].ip)]
    def add(value):
        try:value=ipaddress.IPv4Address(value)
        except (ValueError,TypeError,ipaddress.AddressValueError):return
        if value in seen or value in own or value.is_multicast or value.is_unspecified or value.is_loopback or len(result)>=limit:return
        seen.add(value);result.append(str(value))
    add(saved)
    for row in neighbors:
        try:address=ipaddress.IPv4Address(row.get('address'))
        except (ValueError,TypeError):continue
        for interface in attached:
            network=interface['address'].network
            if (row.get('name')==interface['name'] and _local(address) and address in network
                    and address not in [network.network_address,network.broadcast_address]):add(address);break
    # Round-robin avoids letting a wired adapter exhaust the budget before Wi-Fi.
    spans=[]
    for row in attached:
        interface=row['address'];network=interface.network
        if network.prefixlen<24:network=ipaddress.IPv4Network(str(interface.ip)+'/24',strict=False)
        spans.append(iter(network.hosts()))
    while spans and len(result)<limit:
        remaining=[]
        for span in spans:
            try:add(next(span));remaining.append(span)
            except StopIteration:pass
        spans=remaining
    return result


def _matches(address,port,expected,stop):
    if stop.is_set():return False
    try:
        with socket.create_connection((address,port),timeout=.22):pass
        if stop.is_set():return False
        result=subprocess.run([platform_host.ssh_executable('ssh-keyscan'),'-T','1','-p',str(port),'-t','ed25519,ecdsa,rsa',address],
                              capture_output=True,text=True,timeout=2)
    except (OSError,subprocess.SubprocessError):return False
    for line in result.stdout.splitlines():
        fields=line.split()
        if len(fields)>=3 and (fields[1],fields[2]) in expected:return True
    return False


def find_robot(saved,port,expected,progress=None,cancel=None):
    """Return a matching address or None. Expected keys come from pairing."""
    if not expected:return None
    if progress:progress(.05,'正在搜索当前网络中的机器人')
    stop=threading.Event();expected=set(tuple(key) for key in expected)
    if cancel and cancel():return None
    saved_candidates=candidate_addresses(saved,[],[])
    if saved_candidates and _matches(saved_candidates[0],port,expected,stop):
        return None if cancel and cancel() else saved_candidates[0]
    interfaces,neighbors=network_inventory()
    candidates=candidate_addresses(saved,interfaces,neighbors)
    candidates=[address for address in candidates if address not in saved_candidates]
    if not candidates:return None
    pool=ThreadPoolExecutor(max_workers=min(WORKERS,len(candidates)))
    futures={};found=None;started=time.monotonic()
    try:
        futures={pool.submit(_matches,address,port,expected,stop):address for address in candidates}
        for done in as_completed(futures,timeout=8):
            if cancel and cancel():break
            try:matched=done.result()
            except (OSError,subprocess.SubprocessError):matched=False
            if matched:found=futures[done];break
            if progress and time.monotonic()-started>.4:
                progress(min(.9,.15+.7*sum(f.done() for f in futures)/len(futures)),'正在查找已连接的机器人…');started=time.monotonic()
    except FuturesTimeout:pass
    finally:
        stop.set();pool.shutdown(wait=True,cancel_futures=True)
    if cancel and cancel():return None
    return found

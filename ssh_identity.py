"""Keep an explicitly trusted robot identity independent of its network address."""
import base64
import hashlib
import hmac
from pathlib import Path
import re
import uuid
import platform_host


def endpoint_name(host,port=22):return host if port==22 else '[%s]:%s'%(host,port)


def valid_alias(alias):return isinstance(alias,str) and re.fullmatch(r'm20-aos-[a-f0-9]{32}',alias) is not None


def _matches(names,name):
    for candidate in names.split(','):
        if candidate==name:return True
        if candidate.startswith('|1|'):
            try:
                _,_,salt,value=candidate.split('|')
                actual=hmac.new(base64.b64decode(salt),name.encode(),hashlib.sha1).digest()
                if hmac.compare_digest(actual,base64.b64decode(value)):return True
            except (ValueError,TypeError):pass
    return False


def known_keys(path,name):
    path=Path(path)
    if not path.exists():return []
    result=[]
    for line in path.read_text().splitlines():
        fields=line.split()
        if len(fields)<3 or fields[0].startswith(('#','@')) or not _matches(fields[0],name):continue
        algorithm,key=fields[1:3]
        if algorithm not in ['ssh-ed25519','ssh-rsa','ecdsa-sha2-nistp256','ecdsa-sha2-nistp384','ecdsa-sha2-nistp521']:continue
        try:
            raw=base64.b64decode(key,validate=True);size=int.from_bytes(raw[:4],'big')
            if len(raw)<size+8 or raw[4:4+size].decode()!=algorithm:continue
        except (ValueError,UnicodeError):continue
        if (algorithm,key) not in result:result.append((algorithm,key))
    return result


def write_alias(path,alias,keys):
    """Only caller-confirmed keys; never derive trust from an unverified scan."""
    if not valid_alias(alias) or not keys:raise ValueError('AOS 身份记录无效')
    path=Path(path)
    if path.is_symlink():raise ValueError('SSH 信任文件路径异常')
    old=path.read_text() if path.exists() else '';lines=[]
    for line in old.splitlines():
        parts=line.split(None,1)
        if len(parts)==2 and alias in parts[0].split(','):
            others=[v for v in parts[0].split(',') if v!=alias]
            if others:lines.append(','.join(others)+' '+parts[1])
        else:lines.append(line)
    lines.extend(alias+' '+algorithm+' '+key for algorithm,key in keys)
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temp.write_text('\n'.join(lines)+'\n');platform_host.private_file(temp)
        if (path.read_text() if path.exists() else '')!=old:raise ValueError('SSH 信任记录已变化，请重试')
        temp.replace(path)
    finally:temp.unlink(missing_ok=True)

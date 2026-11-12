"""PCD previews, resumable files and validated map archives. Standard library only."""
import base64
import hashlib
import io
import json
import math
import mmap
import os
import re
import shutil
import stat
import struct
import threading
import time
import uuid
import zipfile
from pathlib import Path, PurePosixPath

MAX_FILE = 4 * 1024**3
CHUNK = 2 * 1024**2


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024**2), b''): h.update(chunk)
    return h.hexdigest()


def identifier(value):
    if not isinstance(value,str) or not re.fullmatch(r'[a-f0-9]{32}',value):
        raise ValueError('无效文件编号')
    return value


def map_name(value):
    if not isinstance(value,str) or not re.fullmatch(r'[\w][\w.-]{0,99}',value) or value in ['active','.','..']:
        raise ValueError('地图名需由字母、数字、下划线、点或连字符组成，且不能是 active')
    return value


class Files:
    def __init__(self,root):
        self.root=Path(root)
        self.root.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock()

    def _meta_path(self,key): return self.root/(identifier(key)+'.json')

    def get(self,key,complete=True):
        try: m=json.loads(self._meta_path(key).read_text())
        except (ValueError,OSError): raise ValueError('文件不存在')
        if complete and m['status']!='complete': raise ValueError('文件尚未上传完成')
        return m

    def path(self,key,complete=True):
        self.get(key,complete)
        return self.root/(key+'.bin')

    def start(self,req):
        name=Path(str(req.get('name','')).replace('\\','/')).name[:160]
        kind=req.get('kind')
        size=req.get('size')
        if kind not in ['pcd','map-package'] or not isinstance(size,int) or isinstance(size,bool) or not 0<size<=MAX_FILE:
            raise ValueError('文件类型或大小无效（最大 4 GiB）')
        if not name.lower().endswith('.pcd' if kind=='pcd' else '.zip'):
            raise ValueError('请选择 .pcd 点云或 .zip 地图包')
        if req.get('resume_id'):
            m=self.get(req['resume_id'],False)
            if (m['name'],m['size'],m['kind'])!=(name,size,kind):raise ValueError('续传文件与原文件不匹配')
            return dict(ok=True,file=m,chunk_size=CHUNK)
        key=uuid.uuid4().hex
        m=dict(id=key,name=name,kind=kind,size=size,offset=0,status='uploading',created=time.time())
        with self.lock:
            (self.root/(key+'.bin')).touch(mode=0o600)
            self._meta_path(key).write_text(json.dumps(m,ensure_ascii=False))
        return dict(ok=True,file=m,chunk_size=CHUNK)

    def chunk(self,req):
        key=identifier(req.get('id'))
        try: chunk=base64.b64decode(req.get('data',''),validate=True)
        except Exception: raise ValueError('文件块编码无效')
        if not 0<len(chunk)<=CHUNK: raise ValueError('文件块大小无效')
        with self.lock:
            m=self.get(key,False)
            if m['status']!='uploading' or req.get('offset')!=m['offset']: raise ValueError('文件块顺序不匹配，请查询进度后重试')
            if m['offset']+len(chunk)>m['size']: raise ValueError('超过声明文件大小')
            p=self.path(key,False)
            with p.open('r+b') as f:
                f.seek(m['offset']);f.write(chunk);f.truncate()
            m['offset']+=len(chunk)
            self._meta_path(key).write_text(json.dumps(m,ensure_ascii=False))
        return dict(ok=True,offset=m['offset'])

    def finish(self,req):
        key=identifier(req.get('id'))
        with self.lock:
            m=self.get(key,False)
            if m['offset']!=m['size']: raise ValueError('文件尚未传完')
            actual=digest(self.path(key,False))
            if req.get('sha256') and req['sha256']!=actual: raise ValueError('文件校验失败')
            if m['kind']=='map-package': inspect_package(self.path(key,False))
            m.update(status='complete',sha256=actual)
            self._meta_path(key).write_text(json.dumps(m,ensure_ascii=False))
            return dict(ok=True,file=m)

    def register(self,path,name=None,kind='map-package'):
        p=Path(path)
        m=self.start(dict(name=name or p.name,kind=kind,size=p.stat().st_size))['file']
        target=self.path(m['id'],False)
        shutil.copyfile(p,target)
        m['offset']=m['size'];self._meta_path(m['id']).write_text(json.dumps(m))
        return self.finish(dict(id=m['id']))['file']

    def list(self):
        result=[]
        for p in self.root.glob('*.json'):
            try:
                m=json.loads(p.read_text())
                if m.get('status')=='complete': result.append(m)
            except (ValueError,OSError): pass
        return sorted(result,key=lambda m:m['created'],reverse=True)

    def read_chunk(self,key,offset):
        m=self.get(key)
        if not isinstance(offset,int) or not 0<=offset<=m['size']: raise ValueError('文件偏移无效')
        with self.path(key).open('rb') as f:f.seek(offset);chunk=f.read(CHUNK)
        return dict(ok=True,file=m,offset=offset,data=base64.b64encode(chunk).decode(),next=offset+len(chunk))


def inspect_package(path):
    try:
        with zipfile.ZipFile(path) as z:
            entries=z.infolist()
            if not entries or len(entries)>100000: raise ValueError('地图包文件数无效')
            total=0;names=set()
            for it in entries:
                name=it.filename
                p=PurePosixPath(name)
                if '\\' in name or p.is_absolute() or '..' in p.parts or ':' in name or len(name)>512:
                    raise ValueError('地图包含有不安全的路径')
                mode=it.external_attr>>16
                if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in [0,stat.S_IFREG,stat.S_IFDIR]):
                    raise ValueError('地图包不能包含链接或特殊文件')
                if name in names: raise ValueError('地图包存在重复路径')
                names.add(name);total+=it.file_size
                if total>16*1024**3 or it.file_size>8*1024**3 or (it.file_size>1024**2 and it.file_size/max(1,it.compress_size)>1500):
                    raise ValueError('地图包解压大小或压缩率超限')
            candidates=set()
            for name in names:
                if PurePosixPath(name).name in ['full_cloud.pcd','jueying.pcd']:
                    folder=str(PurePosixPath(name).parent)
                    prefix='' if folder=='.' else folder+'/'
                    grids=[n for n in names if n.startswith(prefix) and re.fullmatch(r'occ_grid(?:_\d+)?\.pgm|jueying\.pgm',n[len(prefix):])]
                    if any(n[:-4]+'.yaml' in names for n in grids):candidates.add(prefix)
            if len(candidates)!=1: raise ValueError('地图包需含有一份完整的 PCD、PGM 和 YAML 地图')
            prefix=next(iter(candidates))
            return dict(files=len(entries),unpacked_size=total,prefix=prefix,map_name=PurePosixPath(prefix.rstrip('/')).name if prefix else None,
                        grid_ids=sorted([int(m.group(1) or 0) for n in names for m in [re.fullmatch(re.escape(prefix)+r'occ_grid(?:_(\d+))?\.pgm',n)] if m]),
                        contents=[dict(name=e.filename,size=e.file_size) for e in entries if not e.is_dir()][:120])
    except (zipfile.BadZipFile,RuntimeError,NotImplementedError): raise ValueError('无效或不支持的 ZIP 地图包')


def unpack_package(path,destination,name):
    """Exclusive new destination; active and existing maps are never overwritten."""
    name=map_name(name)
    info=inspect_package(path)
    root=Path(destination).resolve();root.mkdir(parents=True,exist_ok=True)
    target=root/name
    if target.exists() or target.is_symlink(): raise ValueError('地图已存在，请使用新的地图名')
    staging=root/('.m20-import-'+uuid.uuid4().hex)
    staging.mkdir(mode=0o700)
    try:
        with zipfile.ZipFile(path) as z:
            for it in z.infolist():
                if it.is_dir() or not it.filename.startswith(info['prefix']):continue
                relative=it.filename[len(info['prefix']):]
                if not relative:continue
                out=staging/relative
                out.parent.mkdir(parents=True,exist_ok=True)
                with z.open(it) as source,out.open('xb') as f:shutil.copyfileobj(source,f,1024**2)
        for old,new in [('jueying.pcd','full_cloud.pcd'),('jueying.pgm','occ_grid.pgm'),('jueying.yaml','occ_grid.yaml')]:
            if (staging/old).exists() and not (staging/new).exists():(staging/old).rename(staging/new)
        if (staging/'loc_map').is_dir() and not (staging/'blocks').exists():(staging/'loc_map').rename(staging/'blocks')
        for yaml in staging.glob('occ_grid*.yaml'):
            body=yaml.read_text()
            image=str(target/(yaml.stem+'.pgm'))
            if not re.search(r'^image\s*:',body,re.M):raise ValueError('地图 YAML 缺少 image 字段')
            body=re.sub(r'^image\s*:.*$',lambda m:'image: '+json.dumps(image),body,flags=re.M)
            yaml.write_text(body)
        staging.rename(target)
    except Exception:
        shutil.rmtree(staging,ignore_errors=True)
        raise
    return dict(ok=True,name=name,activated=False,files=info['files'])


def make_map_readable(folder):
    """Give the NOS service access to a newly staged map, without following links."""
    for directory,_,files in os.walk(folder,followlinks=False):
        path=Path(directory)
        path.chmod(stat.S_IMODE(path.stat().st_mode)|0o055)
        for name in files:
            file=path/name;mode=file.lstat().st_mode
            if stat.S_ISREG(mode):file.chmod(stat.S_IMODE(mode)|0o044)


def pack_map(folder,output):
    folder=Path(folder).resolve()
    if not (folder/'full_cloud.pcd').is_file() or not (folder/'occ_grid.pgm').is_file():
        raise ValueError('地图目录缺少完整点云或栅格')
    output=Path(output)
    if folder==output.resolve() or folder in output.resolve().parents:raise ValueError('打包输出不能位于地图目录内')
    planned=[];links=[];total=0
    def walk(path,relative,ancestors):
        nonlocal total
        try:resolved=path.resolve(strict=True)
        except (OSError,RuntimeError):raise ValueError('地图链接失效或循环：'+relative)
        if resolved!=folder and folder not in resolved.parents:raise ValueError('地图链接超出当前地图目录：'+relative)
        info=resolved.stat();identity=(info.st_dev,info.st_ino)
        if path.is_symlink():links.append(dict(path=relative,target=resolved.relative_to(folder).as_posix()))
        if identity in ancestors:raise ValueError('地图链接存在循环：'+relative)
        if relative and ('\\' in relative or ':' in relative or len(folder.name+'/'+relative)>510):raise ValueError('地图文件名不适合打包：'+relative)
        if stat.S_ISDIR(info.st_mode):
            if relative:planned.append((relative,resolved,info,True))
            for child in sorted(resolved.iterdir()):walk(child,relative+'/'+child.name if relative else child.name,ancestors|{identity})
        elif stat.S_ISREG(info.st_mode):
            total+=info.st_size
            if total>16*1024**3 or info.st_size>8*1024**3:raise ValueError('地图展开大小超限')
            planned.append((relative,resolved,info,False))
        else:raise ValueError('地图内存在特殊文件：'+relative)
        if len(planned)>=99999:raise ValueError('地图文件数超限')
    walk(folder,'',set())
    temp=output.with_name('.'+output.name+'.'+uuid.uuid4().hex+'.tmp')
    def entry(name,directory=False):
        info=zipfile.ZipInfo(name)
        info.create_system=3
        info.compress_type=zipfile.ZIP_DEFLATED
        info._compresslevel=1
        mode=(stat.S_IFDIR|0o755) if directory else (stat.S_IFREG|0o644)
        info.external_attr=(mode<<16)|(0x10 if directory else 0)
        return info
    root_fd=os.open(folder,os.O_RDONLY|os.O_DIRECTORY)
    try:
        with zipfile.ZipFile(temp,'w',zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
            for relative,resolved,info,is_dir in planned:
                arc=folder.name+'/'+relative
                if is_dir:z.writestr(entry(arc+'/',True),b'');continue
                # Open canonical components below the map root without following
                # links again if a file/directory is replaced during packing.
                descriptors=[];parent=root_fd
                try:
                    parts=resolved.relative_to(folder).parts
                    for component in parts[:-1]:
                        parent=os.open(component,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,dir_fd=parent);descriptors.append(parent)
                    fd=os.open(parts[-1],os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK,dir_fd=parent)
                    with os.fdopen(fd,'rb') as source:
                        current=os.fstat(source.fileno())
                        if not stat.S_ISREG(current.st_mode) or (current.st_dev,current.st_ino,current.st_size,current.st_mtime_ns)!=(info.st_dev,info.st_ino,info.st_size,info.st_mtime_ns):
                            raise ValueError('地图文件在打包时发生变化，请结束建图后重试：'+relative)
                        with z.open(entry(arc),'w',force_zip64=True) as target:shutil.copyfileobj(source,target,1024**2)
                finally:
                    for descriptor in reversed(descriptors):os.close(descriptor)
            z.writestr(entry(folder.name+'/.m20-platform-manifest.json'),json.dumps(dict(format='m20-platform-map-v1',name=folder.name,
                files=[row[0] for row in planned if not row[3]],materialized_links=links,created=time.time())))
        inspect_package(temp)
        temp.replace(output)
    finally:
        os.close(root_fd);temp.unlink(missing_ok=True)
    return dict(path=str(output),size=output.stat().st_size,materialized_links=len(links))


def lzf_decode(data,expected):
    if not 0<expected<=512*1024**2: raise ValueError('压缩点云展开后超过 512 MiB，请先降采样')
    out=bytearray();i=0
    while i<len(data):
        control=data[i];i+=1
        if control<32:
            length=control+1
            if i+length>len(data):raise ValueError('损坏的 LZF 点云')
            out.extend(data[i:i+length]);i+=length
        else:
            length=control>>5;offset=(control&31)<<8
            if length==7:
                if i>=len(data):raise ValueError('损坏的 LZF 长度')
                length+=data[i];i+=1
            if i>=len(data):raise ValueError('损坏的 LZF 偏移')
            offset+=data[i]+1;i+=1;length+=2
            pos=len(out)-offset
            if pos<0:raise ValueError('无效的 LZF 引用')
            for _ in range(length):out.append(out[pos]);pos+=1
        if len(out)>expected:raise ValueError('点云解压长度异常')
    if len(out)!=expected:raise ValueError('点云解压不完整')
    return out


def preview_pcd(path,limit=60000,progress=None):
    limit=max(100,min(100000,int(limit)))
    with Path(path).open('rb') as f:
        header={};count=0
        while True:
            line=f.readline(65536);count+=len(line)
            if not line or count>1024**2:raise ValueError('PCD 头部无效')
            try:parts=line.decode('ascii').strip().split()
            except UnicodeDecodeError:raise ValueError('PCD 头部不是 ASCII')
            if not parts or parts[0].startswith('#'):continue
            header[parts[0].upper()]=parts[1:]
            if parts[0].upper()=='DATA':break
        names=header.get('FIELDS',header.get('FIELD',[]));sizes=list(map(int,header.get('SIZE',[])));types=header.get('TYPE',[])
        counts=list(map(int,header.get('COUNT',['1']*len(names))))
        total=int(header.get('POINTS',[int(header.get('WIDTH',['0'])[0])*int(header.get('HEIGHT',['1'])[0])])[0])
        if not 0<total<=200000000 or len(names)>100 or not (len(names)==len(sizes)==len(types)==len(counts)):
            raise ValueError('PCD 字段或点数无效')
        if any(c<1 or c>10000 for c in counts):raise ValueError('PCD COUNT 无效')
        if not all(axis in names for axis in ['x','y','z']):raise ValueError('PCD 缺少 x/y/z 坐标')
        encoding=header['DATA'][0].lower();step=max(1,math.ceil(total/limit));offset=f.tell()
        points=[];bounds=[[math.inf]*3,[-math.inf]*3]
        def add(values):
            if not all(math.isfinite(v) and abs(v)<1e7 for v in values):return
            points.append([round(v,4) for v in values])
            for j,v in enumerate(values):bounds[0][j]=min(bounds[0][j],v);bounds[1][j]=max(bounds[1][j],v)
        scalar_offsets=[];byte_offsets=[];scalars=0;stride=0
        formats={('F',4):'f',('F',8):'d',('I',1):'b',('I',2):'h',('I',4):'i',('I',8):'q',('U',1):'B',('U',2):'H',('U',4):'I',('U',8):'Q'}
        for size,typ,n in zip(sizes,types,counts):
            if (typ.upper(),size) not in formats:raise ValueError('PCD 数据类型不支持')
            scalar_offsets.append(scalars);byte_offsets.append(stride);scalars+=n;stride+=n*size
        axes=[names.index(a) for a in ['x','y','z']]
        if encoding=='ascii':
            rows=0
            for index,line in enumerate(f):
                if not line.strip():continue
                rows+=1
                if index%step:continue
                values=line.split()
                if len(values)!=scalars:raise ValueError('PCD ASCII 字段数不匹配')
                try:add([float(values[scalar_offsets[a]]) for a in axes])
                except ValueError:raise ValueError('PCD ASCII 坐标无效')
                if len(points)>=limit:break
            if rows==0:raise ValueError('PCD 没有点数据')
        elif encoding in ['binary','binary_compressed']:
            if encoding=='binary':
                if Path(path).stat().st_size-offset<stride*total:raise ValueError('PCD 二进制数据不完整')
                with mmap.mmap(f.fileno(),0,access=mmap.ACCESS_READ) as data:
                    for index in range(0,total,step):add([struct.unpack_from('<'+formats[(types[a].upper(),sizes[a])],data,offset+index*stride+byte_offsets[a])[0] for a in axes])
            else:
                raw=f.read(8)
                if len(raw)!=8:raise ValueError('PCD 压缩头不完整')
                compressed,expanded=struct.unpack('<II',raw)
                if expanded!=stride*total or compressed>512*1024**2:raise ValueError('PCD 压缩数据大小无效')
                if progress:progress(.2,'正在解压 PCD 点云')
                body=f.read(compressed)
                if len(body)!=compressed:raise ValueError('PCD 压缩数据不完整')
                data=lzf_decode(body,expanded)
                for index in range(0,total,step):add([struct.unpack_from('<'+formats[(types[a].upper(),sizes[a])],data,byte_offsets[a]*total+index*sizes[a]*counts[a])[0] for a in axes])
        else:raise ValueError('仅支持 PCD ASCII / binary / binary_compressed')
    if not points:raise ValueError('点云没有有效坐标')
    return dict(ok=True,points=points,total_points=total,shown_points=len(points),encoding=encoding,
                bounds=bounds,fields=names,units='m',sampled=len(points)<total)

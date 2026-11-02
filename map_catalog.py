"""Recoverable map-library edits; map identities and geometry stay unchanged."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import shutil
import stat
import threading
import time
import uuid
from map_formats import map_name

_lock=threading.RLock()
_KEY=re.compile('[a-f0-9]{32}')


@contextmanager
def catalog_lock(maps):
    with _lock:
        path=Path(maps)/'.m20-catalog.lock'
        if path.is_symlink():raise ValueError('地图管理锁路径异常')
        flags=os.O_WRONLY|os.O_CREAT|os.O_APPEND|getattr(os,'O_NOFOLLOW',0)
        with os.fdopen(os.open(path,flags,0o644),'a') as handle:
            if os.name!='nt':
                import fcntl
                fcntl.flock(handle,fcntl.LOCK_EX)
            try:yield
            finally:
                if os.name!='nt':fcntl.flock(handle,fcntl.LOCK_UN)


def read_json(path,default):
    if path.is_symlink():raise ValueError('地图管理记录异常')
    if not path.exists():return default
    if not path.is_file() or path.stat().st_size>4*1024*1024:raise ValueError('地图管理记录异常')
    return json.loads(path.read_text())


def atomic_json(path,value):
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with temp.open('x') as handle:
            json.dump(value,handle,ensure_ascii=False,indent=2)
            handle.flush();os.fsync(handle.fileno())
        temp.replace(path)
        if os.name!='nt':
            fd=os.open(path.parent,os.O_RDONLY)
            try:os.fsync(fd)
            finally:os.close(fd)
    finally:temp.unlink(missing_ok=True)


def _within(path,folder):return path==folder or folder in path.parents


def _scalar(value):
    """Known YAML image/TOML path string forms; no YAML object construction."""
    value=value.strip()
    if value.startswith('"'):
        try:return json.JSONDecoder().raw_decode(value)[0]
        except ValueError:raise ValueError('地图文件引用格式无法核对')
    if value.startswith("'"):
        match=re.match(r"'((?:[^']|'')*)'",value)
        if not match:raise ValueError('地图文件引用格式无法核对')
        return match.group(1).replace("''", "'")
    return value.split(' #',1)[0].strip()


def _paths(folder):
    """Walk real directories only, but include links for reference checking."""
    def unreadable(error):raise ValueError('地图目录无法读取，未确认引用或完整性：'+str(error))
    for directory,dirs,files in os.walk(folder,followlinks=False,onerror=unreadable):
        for name in dirs+files:yield Path(directory)/name


class MapCatalog:
    def __init__(self,maps,busy=None):
        self.maps=Path(maps).resolve();self.busy=busy or (lambda:None)
        self.meta=self.maps/'.m20-library.json';self.trash=self.maps/'.m20-trash'

    def folder(self,name):
        name=map_name(name);path=self.maps/name
        if path.is_symlink() or not path.is_dir() or path.resolve().parent!=self.maps:raise ValueError('地图不存在或路径无效')
        return path

    def labels(self):
        labels=read_json(self.meta,{})
        if not isinstance(labels,dict):raise ValueError('地图名称记录异常')
        return labels

    def cleanup_base(self,name):
        project=read_json(self.folder(name)/'.m20-project.json',{})
        if not isinstance(project,dict):raise ValueError('地图合并记录异常')
        reports=project.get('sessions',[])+project.get('refinements',[])
        own=[r for r in reports if isinstance(r,dict) and r.get('map')==name and r.get('includes_complete_base') is True]
        if not own:return None
        base=max(own,key=lambda r:r.get('created',0)).get('base_map')
        try:return map_name(base) if base!=name else None
        except (ValueError,TypeError):return None

    def _trash_root(self):
        if self.trash.is_symlink() or self.trash.exists() and not self.trash.is_dir():raise ValueError('回收站路径异常')

    def _record(self,key):
        if not isinstance(key,str) or not _KEY.fullmatch(key):raise ValueError('回收记录无效')
        self._trash_root();source=self.trash/key
        if source.is_symlink() or source.exists() and not source.is_dir():raise ValueError('回收记录路径异常')
        meta=read_json(source/'meta.json',{})
        if not isinstance(meta,dict):raise ValueError('回收记录无效')
        name=map_name(meta.get('name'))
        return source,meta,name

    def _payload(self,source,meta,allow_missing=False):
        folder=source/'map'
        if folder.is_symlink() or folder.exists() and not folder.is_dir():raise ValueError('回收地图路径异常')
        if not folder.exists():
            if allow_missing:return folder
            raise ValueError('地图已恢复或回收记录无效')
        identity=meta.get('map_identity')
        if identity is not None and identity!=[folder.stat().st_dev,folder.stat().st_ino]:raise ValueError('回收地图归属已变化，未操作替换后的文件')
        return folder

    def trash_list(self):
        with _lock:
            self._trash_root();result=[]
            if self.trash.exists():
                for source in self.trash.iterdir():
                    if source.is_symlink() or not source.is_dir() or not _KEY.fullmatch(source.name):continue
                    if not (source/'map').exists() and not (source/'meta.json').exists():continue
                    _,meta,_=self._record(source.name)
                    state=meta.get('state','archived')
                    if state in ('restored','purged') or read_json(self.trash/('.'+source.name+'.purged.json'),{}):continue
                    folder=self._payload(source,meta,allow_missing=True)
                    if folder.exists() or state in ('purging','purge_failed'):
                        result.append(dict(meta,id=source.name,state=state,recoverable=state=='archived',purge_retryable=state in ('purging','purge_failed')))
            return sorted(result,key=lambda row:row.get('deleted_at',0),reverse=True)

    def _map_roots(self):
        for other in self.maps.iterdir():
            if other.name=='active' or other.name.startswith('.'):continue
            if other.is_dir() or other.is_symlink():yield other,other,other.name
        self._trash_root()
        if self.trash.exists():
            for source in self.trash.iterdir():
                if source.is_symlink() or not source.is_dir() or not _KEY.fullmatch(source.name):continue
                if not (source/'map').exists() and not (source/'meta.json').exists():continue
                _,meta,name=self._record(source.name)
                folder=self._payload(source,meta,allow_missing=True)
                if folder.exists():yield folder,self.maps/name,'回收站/'+name

    def ensure_unused(self,folder):
        self.busy()
        active=self.maps/'active'
        if not active.is_symlink() or not active.resolve().is_dir() or active.resolve().parent!=self.maps:raise ValueError('活动地图状态无法核对，未删除地图')
        if active.resolve()==folder:raise ValueError('当前活动地图不能删除；请先启用其他地图')
        # Archived maps are still recoverable consumers. Relative references in
        # them must also be interpreted at their original map location.
        for other,logical,label in self._map_roots():
            if other==folder:continue
            paths=[other] if other.is_symlink() else _paths(other)
            for path in paths:
                targets=[]
                if path.is_symlink():
                    try:
                        targets.append(path.resolve())
                        raw=Path(os.readlink(path))
                        if not raw.is_absolute():targets.append((logical/path.relative_to(other).parent/raw).resolve())
                    except (OSError,RuntimeError):raise ValueError('地图链接无法核对，未删除：'+label)
                elif path.is_file() and (re.fullmatch(r'occ_grid(?:_\d+)?\.ya?ml',path.name) or path.name=='occ_grid_id_map.toml'):
                    if path.stat().st_size>4*1024*1024:raise ValueError('地图配置过大，引用无法核对：'+label)
                    pattern=r'^\s*image\s*:\s*(.+)$' if path.suffix in ('.yaml','.yml') else r'^\s*path\s*=\s*(.+)$'
                    for match in re.finditer(pattern,path.read_text(),re.M):
                        raw=_scalar(match.group(1))
                        if not isinstance(raw,str) or not raw:raise ValueError('地图文件引用格式无法核对：'+label)
                        ref=Path(raw)
                        targets.append((path.parent/ref).resolve())
                        if not ref.is_absolute():targets.append((logical/path.relative_to(other).parent/ref).resolve())
                if any(_within(target,folder) for target in targets):raise ValueError('其他地图仍引用这份地图，未删除：'+label)

    def archive(self,name,reason='manual'):
        folder=self.folder(name);self.ensure_unused(folder)
        self._trash_root();labels=self.labels();info=folder.stat()
        self.trash.mkdir(exist_ok=True)
        key=uuid.uuid4().hex;destination=self.trash/key;destination.mkdir()
        meta=dict(name=name,display_name=labels.get(name,name),deleted_at=time.time(),reason=reason,state='archived',map_identity=[info.st_dev,info.st_ino])
        try:
            atomic_json(destination/'meta.json',meta)
            folder.rename(destination/'map')
        except Exception:
            # No payload moved: discard only our empty bookkeeping, never maps.
            if not (destination/'map').exists():
                (destination/'meta.json').unlink(missing_ok=True);destination.rmdir()
            raise
        return dict(ok=True,name=name,trash_id=key,recoverable=True)

    def _purge(self,req):
        key=req.get('id','')
        if not isinstance(key,str) or not _KEY.fullmatch(key):raise ValueError('回收记录无效')
        self._trash_root();receipt=self.trash/('.'+key+'.purged.json')
        completed=read_json(receipt,{})
        if completed:
            if not isinstance(completed,dict) or completed.get('name')!=req.get('confirmed_name'):raise ValueError('请明确确认要彻底删除的地图')
            return dict(ok=True,name=completed['name'],permanently_deleted=True,already_completed=True)
        source,meta,name=self._record(key)
        if req.get('confirmed_name')!=name:raise ValueError('请明确确认要彻底删除的地图')
        state=meta.get('state','archived')
        if state not in ('archived','purging','purge_failed'):raise ValueError('该地图已恢复或回收记录不可删除')
        folder=self._payload(source,meta,allow_missing=state in ('purging','purge_failed'))
        self.ensure_unused(folder)
        # Keep identity outside rmtree and checkpoint intent BEFORE removing any
        # bytes. An interrupted purge may be retried, never offered as a full map.
        meta.update(state='purging',recoverable=False)
        atomic_json(source/'meta.json',meta)
        try:
            if folder.exists():shutil.rmtree(folder)
        except Exception as exc:
            meta.update(state='purge_failed',purge_error=str(exc)[-1000:])
            atomic_json(source/'meta.json',meta)
            raise ValueError('永久删除未完成，剩余文件保留，可重试；本记录不能再完整恢复：'+str(exc)) from exc
        atomic_json(receipt,dict(name=name,purged_at=time.time()))
        # Failure removing tiny bookkeeping after the durable receipt cannot
        # change the successful payload result or cause a same-name retry delete.
        try:(source/'meta.json').unlink();source.rmdir()
        except OSError:pass
        return dict(ok=True,name=name,permanently_deleted=True)

    @staticmethod
    def _unchanged_copy(original,copy):
        before=original.stat();after=copy.stat()
        if before.st_size!=after.st_size:return False
        with original.open('rb') as left,copy.open('rb') as right:
            while True:
                a=left.read(1024*1024);b=right.read(1024*1024)
                if a!=b:return False
                if not a:break
        return (original.stat().st_size,original.stat().st_mtime_ns)==(before.st_size,before.st_mtime_ns) and (copy.stat().st_size,copy.stat().st_mtime_ns)==(after.st_size,after.st_mtime_ns)

    def _complete_candidate(self,candidate,old):
        review=read_json(candidate/'.m20-alignment-review.json',{})
        if review.get('status')=='rejected':raise ValueError('候选对齐检查未通过，旧图保留')
        for path in _paths(candidate):
            if path.is_symlink():
                try:target=path.resolve(strict=True)
                except (OSError,RuntimeError):raise ValueError('候选链接失效，旧图保留')
                if not _within(target,candidate):raise ValueError('候选仍引用外部文件，旧图保留')
        cloud=candidate/'full_cloud.pcd'
        if not cloud.is_file() or not cloud.stat().st_size:raise ValueError('候选点云不完整，旧图保留')
        for path in _paths(old):
            relative=path.relative_to(old)
            if path.name.startswith('.m20-') or '.nav_maps' in relative.parts or relative.parts[0] in ('.blocks','blocks'):continue
            if path.is_symlink() and path.is_dir():
                # Original internal directory links are expected to remain links
                # into the candidate; do not silently skip an entire raw subtree.
                target=(candidate/relative)
                if not target.is_dir():raise ValueError('候选缺少原图目录，旧图保留：'+str(relative))
                continue
            if not path.is_file():continue
            target=candidate/relative
            if not target.is_file():raise ValueError('候选缺少原图文件，旧图保留：'+str(relative))
            # These are intentionally regenerated by fusion. Raw sessions,
            # optimizers and other retained original files must match bytewise.
            derived=len(relative.parts)==1 and (path.name in ('full_cloud.pcd','occ_grid_id_map.toml') or re.fullmatch(r'occ_grid(?:_\d+)?\.(pgm|yaml)',path.name))
            if derived:
                if path.stat().st_size and not target.stat().st_size:raise ValueError('候选地图文件不完整，旧图保留：'+str(relative))
            elif not self._unchanged_copy(path,target):raise ValueError('候选原始文件不完整或内容不一致，旧图保留：'+str(relative))

    def execute(self,action,req):
        with catalog_lock(self.maps):
            if action=='rename':
                name=self.folder(req.get('name')).name;label=req.get('display_name')
                if not isinstance(label,str) or not 1<=len(label.strip())<=80 or any(ord(c)<32 for c in label):raise ValueError('名称需为 1–80 个字符，不能包含换行或控制字符')
                labels=self.labels();labels[name]=label.strip();atomic_json(self.meta,labels)
                return dict(ok=True,name=name,display_name=labels[name])
            if action=='delete':return self.archive(req.get('name'))
            if action=='purge':return self._purge(req)
            if action=='restore':
                self.busy();source,meta,name=self._record(req.get('id',''));target=self.maps/name
                state=meta.get('state','archived')
                if state not in ('archived','restored'):raise ValueError('永久删除已开始，本记录无法完整恢复；可重试删除剩余文件')
                folder=self._payload(source,meta,allow_missing=True)
                if not folder.exists() and not target.is_symlink() and target.is_dir() and meta.get('map_identity')==[target.stat().st_dev,target.stat().st_ino]:
                    return dict(ok=True,name=name,activated=False,already_completed=True)
                if state=='restored':raise ValueError('原恢复操作已完成；同编号地图已变化，未覆盖')
                if target.exists() or target.is_symlink():raise ValueError('同编号地图已存在，未覆盖')
                if not folder.exists():raise ValueError('地图已恢复或回收记录无效')
                folder.rename(target)
                # Keep the receipt so a later retry can never restore a replaced
                # trash payload over a new map with the same identifier.
                meta.update(state='restored',restored_at=time.time())
                try:atomic_json(source/'meta.json',meta)
                except OSError:pass  # Move succeeded; folder identity remains gone.
                return dict(ok=True,name=name,activated=False)
            if action=='cleanup-previous':
                name=self.folder(req.get('name')).name;base=req.get('base')
                if (self.maps/'active').resolve()!=self.maps/name:raise ValueError('新图尚未成为活动地图，未清理旧图')
                if self.cleanup_base(name)!=base:raise ValueError('未找到对应的完整合并记录，旧图保留')
                candidate=self.folder(name);old=self.folder(base)
                self._complete_candidate(candidate,old)
                return self.archive(base,'merged-into:'+name)
            raise ValueError('地图编辑操作无效')

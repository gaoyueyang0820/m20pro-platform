"""Recoverable map-library edits; map identities and geometry stay unchanged."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import threading
import time
import uuid
from map_formats import map_name

_lock=threading.RLock()


@contextmanager
def catalog_lock(maps):
    with _lock:
        path=Path(maps)/'.m20-catalog.lock'
        if path.is_symlink():raise ValueError('地图管理锁路径异常')
        with path.open('a') as handle:
            if os.name!='nt':
                import fcntl
                fcntl.flock(handle,fcntl.LOCK_EX)
            try:yield
            finally:
                if os.name!='nt':fcntl.flock(handle,fcntl.LOCK_UN)


def read_json(path,default):
    if not path.exists():return default
    if path.is_symlink() or path.stat().st_size>4*1024*1024:raise ValueError('地图管理记录异常')
    return json.loads(path.read_text())


def atomic_json(path,value):
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temp.write_text(json.dumps(value,ensure_ascii=False,indent=2));temp.replace(path)
    finally:temp.unlink(missing_ok=True)


class MapCatalog:
    def __init__(self,maps,busy=None):
        self.maps=Path(maps).resolve();self.busy=busy or (lambda:None)
        self.meta=self.maps/'.m20-library.json';self.trash=self.maps/'.m20-trash'

    def folder(self,name):
        name=map_name(name);path=self.maps/name
        if path.is_symlink() or not path.is_dir() or path.resolve().parent!=self.maps:raise ValueError('地图不存在或路径无效')
        return path

    def labels(self):return read_json(self.meta,{})

    def cleanup_base(self,name):
        folder=self.folder(name);project=read_json(folder/'.m20-project.json',{})
        reports=project.get('sessions',[])+project.get('refinements',[])
        own=[r for r in reports if r.get('map')==name and r.get('includes_complete_base') is True]
        if not own:return None
        base=max(own,key=lambda r:r.get('created',0)).get('base_map')
        try:return map_name(base) if base!=name else None
        except (ValueError,TypeError):return None

    def trash_list(self):
        if self.trash.is_symlink():raise ValueError('回收站路径异常')
        result=[]
        if self.trash.exists():
            for folder in self.trash.iterdir():
                if folder.is_symlink() or not re.fullmatch('[a-f0-9]{32}',folder.name):continue
                meta=read_json(folder/'meta.json',{})
                if (folder/'map').is_dir() and not (folder/'map').is_symlink():result.append(dict(meta,id=folder.name))
        return sorted(result,key=lambda row:row.get('deleted_at',0),reverse=True)

    def ensure_unused(self,folder):
        self.busy()
        active=self.maps/'active'
        if not active.is_symlink() or not active.resolve().is_dir() or active.resolve().parent!=self.maps:raise ValueError('活动地图状态无法核对，未删除地图')
        if active.resolve()==folder:raise ValueError('当前活动地图不能删除；请先启用其他地图')
        # Moving a directory must not break another map's file references.
        for other in self.maps.iterdir():
            if other==folder or other.is_symlink() or not other.is_dir() or other.name.startswith('.'):continue
            for path in other.rglob('*'):
                if path.is_symlink():
                    target=path.resolve()
                    if target==folder or folder in target.parents:raise ValueError('其他地图仍引用这份地图，未删除：'+other.name)

    def archive(self,name,reason='manual'):
        folder=self.folder(name);self.ensure_unused(folder)
        if self.trash.is_symlink():raise ValueError('回收站路径异常')
        self.trash.mkdir(exist_ok=True)
        key=uuid.uuid4().hex;destination=self.trash/key;destination.mkdir()
        meta=dict(name=name,display_name=self.labels().get(name,name),deleted_at=time.time(),reason=reason)
        atomic_json(destination/'meta.json',meta)
        folder.rename(destination/'map')
        return dict(ok=True,name=name,trash_id=key,recoverable=True)

    def execute(self,action,req):
        with catalog_lock(self.maps):
            if action=='rename':
                name=self.folder(req.get('name')).name;label=req.get('display_name')
                if not isinstance(label,str) or not 1<=len(label.strip())<=80 or any(ord(c)<32 for c in label):raise ValueError('名称需为 1–80 个字符，不能包含换行或控制字符')
                labels=self.labels();labels[name]=label.strip();atomic_json(self.meta,labels)
                return dict(ok=True,name=name,display_name=labels[name])
            if action=='delete':return self.archive(req.get('name'))
            if action=='restore':
                self.busy();key=req.get('id','')
                if not re.fullmatch('[a-f0-9]{32}',key):raise ValueError('回收记录无效')
                if self.trash.is_symlink():raise ValueError('回收站路径异常')
                source=self.trash/key
                if source.is_symlink():raise ValueError('回收记录路径异常')
                meta=read_json(source/'meta.json',{});name=map_name(meta.get('name'));target=self.maps/name
                if target.exists() or target.is_symlink():raise ValueError('同编号地图已存在，未覆盖')
                if not (source/'map').is_dir() or (source/'map').is_symlink():raise ValueError('地图已恢复或回收记录无效')
                (source/'map').rename(target)
                return dict(ok=True,name=name,activated=False)
            if action=='cleanup-previous':
                name=self.folder(req.get('name')).name;base=req.get('base')
                if (self.maps/'active').resolve()!=self.maps/name:raise ValueError('新图尚未成为活动地图，未清理旧图')
                if self.cleanup_base(name)!=base:raise ValueError('未找到对应的完整合并记录，旧图保留')
                candidate=self.folder(name);old=self.folder(base)
                review=read_json(candidate/'.m20-alignment-review.json',{})
                if review.get('status')=='rejected':raise ValueError('候选对齐检查未通过，旧图保留')
                if not (candidate/'full_cloud.pcd').is_file() or not (candidate/'full_cloud.pcd').stat().st_size:raise ValueError('候选点云不完整，旧图保留')
                # Complete candidates retain original scans and all old grid files.
                for path in old.rglob('*'):
                    if path.is_file() and not path.name.startswith('.m20-') and '.nav_maps' not in path.parts:
                        if not (candidate/path.relative_to(old)).is_file():raise ValueError('候选缺少原图文件，旧图保留：'+path.name)
                for path in candidate.rglob('*'):
                    if path.is_symlink() and candidate not in path.resolve().parents:raise ValueError('候选仍引用外部文件，旧图保留')
                return self.archive(base,'merged-into:'+name)
            raise ValueError('地图编辑操作无效')

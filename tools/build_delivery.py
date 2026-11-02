#!/usr/bin/env python3
"""Build a source delivery from an allowlist; never export runtime credentials."""
from pathlib import Path
import argparse,datetime,hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]


def source_files(root=ROOT):
    result=set(root.glob('*.py'))|set(root.glob('*.cmd'))|set(root.glob('*.command'))
    for name in ['README.md','README-Windows.md','API.md','自主作业说明.md','package.json','package-lock.json','viewer-src.js','.gitignore','requirements-video.txt','requirements-windows.txt']:
        p=root/name
        if p.is_file():result.add(p)
    for directory in ['bridge','platform_web','m20_web','models','reference','tests','windows','upstream','tools','native']:
        for p in (root/directory).rglob('*'):
            rel=p.relative_to(root)
            # Real site geometry/trajectory replays remain on the development
            # machine; sanitized coordinates are still operational data.
            if rel.parts[:3]==('tests','fixtures','runtime_replay') and p.suffix=='.json':continue
            if directory=='native' and p.suffix in ['.so','.dylib','.dll','.o']:continue
            if p.is_file() and not p.is_symlink() and '__pycache__' not in rel.parts and not p.name.startswith('.') and p.suffix not in ['.pyc','.pyo','.log']:
                result.add(p)
    # The one permitted data file is a generated HEVC test pattern, not camera footage.
    fixture=root/'data/network/camera-codec-fixture.mp4'
    if fixture.is_file():result.add(fixture)
    return sorted(p for p in result if p.is_file() and not p.is_symlink())


def build(destination):
    destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    files=source_files();snapshot=[]
    for path in files:
        content=path.read_bytes();name=path.relative_to(ROOT).as_posix()
        if path.suffix in ['.py','.json','.md','.txt','.cmd','.command'] and (b'-----BEGIN '+b'OPENSSH PRIVATE KEY-----') in content:
            raise ValueError('Refusing private key material: '+name)
        snapshot.append((path,name,content,hashlib.sha256(content).hexdigest()))
    manifest=dict(version='2026.10.30-workspace-windows',created_at=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
                  verification=dict(source_syntax='checked separately',unpacked_demo='checked separately',windows_native='NOT_TESTED'),
                  exclusions=['runtime profiles/tokens/private keys','real maps','logs/camera recordings','real site replay fixtures','venvs','node_modules','debug handoffs'],
                  files=[dict(path=name,bytes=len(content),sha256=digest) for _,name,content,digest in snapshot])
    prefix='M20Pro-Platform/'
    with zipfile.ZipFile(destination,'x',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for path,name,content,digest in snapshot:
            if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise RuntimeError('Source changed during packaging: '+name)
            z.writestr(prefix+name,content)
        z.writestr(prefix+'release-manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    digest=hashlib.sha256(destination.read_bytes()).hexdigest()
    destination.with_suffix(destination.suffix+'.sha256').write_text(digest+'  '+destination.name+'\n')
    print(json.dumps(dict(zip=str(destination.resolve()),files=len(snapshot)+1,bytes=destination.stat().st_size,sha256=digest),ensure_ascii=False))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('destination');build(ap.parse_args().destination)

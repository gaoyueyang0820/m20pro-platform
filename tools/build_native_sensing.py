#!/usr/bin/env python3
"""Explicit local C++ build; no dependency installation, downloads or robot I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def build(output=None,compiler=None):
    source=ROOT/'native/sensing.cpp'
    is_windows=os.name=='nt'
    default_name='libsensing.pyd' if is_windows else 'libsensing.so'
    output=Path(output or ROOT/'native'/default_name).absolute()
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='m20-native-sensing-',dir=str(output.parent)) as directory:
        candidate=Path(directory)/default_name
        if is_windows:
            # MSVC (cl.exe). Locate it through vswhere if it is not already on PATH.
            if not compiler:
                vswhere=r'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
                if os.path.exists(vswhere):
                    try:
                        out=subprocess.run([vswhere,'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],capture_output=True,text=True,timeout=30).stdout.strip()
                        if out:
                            for candidate_cl in (Path(out)/'VC'/'Tools'/'MSVC').rglob('bin/Hostx64/x64/cl.exe'):
                                compiler=str(candidate_cl);break
                    except (OSError,subprocess.SubprocessError):pass
                compiler=compiler or shutil.which('cl.exe')
            if not compiler:raise ValueError('Windows 上未找到 MSVC (cl.exe)；请安装「Visual Studio 生成工具」后重试，或继续使用 NumPy 感知后端')
            flags=['/O2','/std:c++17','/EHsc','/nologo','/LD','/DNDEBUG']
            command=[compiler,*flags,str(source),'/Fe:'+str(candidate)]
        else:
            compiler=compiler or os.environ.get('CXX') or shutil.which('c++') or shutil.which('clang++')
            if not compiler:raise ValueError('没有可用 C++ 编译器；继续使用 NumPy 感知后端')
            flags=['-O3','-std=c++17','-shared','-fPIC','-fno-fast-math','-ffp-contract=off']
            command=[compiler,*flags,str(source),'-o',str(candidate)]
        result=subprocess.run(command,text=True,capture_output=True,timeout=180,shell=is_windows)
        if result.returncode:raise ValueError('原生感知编译失败：'+result.stderr[-5000:])
        # cl.exe names the DLL after the source basename unless /Fe: points elsewhere.
        built=candidate if candidate.exists() else Path(directory)/(source.stem+'.dll')
        if not built.exists():raise ValueError('原生感知编译完成但未找到产物：'+str(built))
        os.replace(built,output)
    return dict(ok=True,abi=1,compiler=compiler,flags=flags,output=str(output),platform='windows' if is_windows else 'posix',
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                scope='local_build_only_no_robot_deployment')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output');parser.add_argument('--compiler');args=parser.parse_args()
    try:result=build(args.output,args.compiler)
    except (ValueError,subprocess.TimeoutExpired,OSError) as exc:print(json.dumps(dict(ok=False,error=str(exc)),ensure_ascii=False));return 1
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0


if __name__=='__main__':sys.exit(main())

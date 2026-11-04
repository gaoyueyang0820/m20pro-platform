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


def windows_compiler(compiler=None):
    env=os.environ.copy()
    if env.get('INCLUDE') and env.get('LIB'):
        found=compiler or env.get('CXX') or shutil.which('cl.exe')
        if found:return found,env
    vswhere=Path(env.get('ProgramFiles(x86)',r'C:\Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    if not vswhere.is_file():raise ValueError('请安装 Visual Studio C++ 生成工具，或从 x64 开发者命令行执行构建')
    probe=subprocess.run([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],capture_output=True,text=True,check=True,timeout=30)
    installation=probe.stdout.strip()
    setup=Path(installation)/'VC/Auxiliary/Build/vcvarsall.bat'
    if not installation or not setup.is_file():raise ValueError('未找到 x64 C++ 编译环境')
    cmd=Path(env.get('SystemRoot',r'C:\Windows'))/'System32/cmd.exe'
    # Only the installed, fixed vcvarsall path is evaluated by cmd. The compiler
    # itself receives an argv list and runs in the temporary output directory.
    command=subprocess.list2cmdline([str(cmd)])+' /d /u /s /c ""'+str(setup)+'" x64 >nul && set"'
    probe=subprocess.run(command,executable=str(cmd),capture_output=True,encoding='utf-16le',errors='replace',check=True,timeout=45)
    for line in probe.stdout.splitlines():
        key,separator,value=line.partition('=')
        if separator and key and not key.startswith('='):env[key.upper()]=value
    found=compiler or shutil.which('cl.exe',path=env.get('PATH',''))
    if not found or not env.get('INCLUDE') or not env.get('LIB'):raise ValueError('C++ 编译环境未就绪')
    return found,env


def build(output=None,compiler=None):
    source=ROOT/'native/sensing.cpp'
    is_windows=os.name=='nt'
    default_name='libsensing.pyd' if is_windows else 'libsensing.so'
    output=Path(output or ROOT/'native'/default_name).absolute()
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='m20-native-sensing-',dir=str(output.parent)) as directory:
        candidate=Path(directory)/default_name
        env=None
        if is_windows:
            compiler,env=windows_compiler(compiler)
            flags=['/O2','/std:c++17','/EHsc','/nologo','/LD','/DNDEBUG','/fp:strict']
            command=[compiler,*flags,str(source),'/Fe:'+str(candidate),'/Fo:'+str(Path(directory)/'sensing.obj')]
        else:
            compiler=compiler or os.environ.get('CXX') or shutil.which('c++') or shutil.which('clang++')
            if not compiler:raise ValueError('没有可用 C++ 编译器；继续使用 NumPy 感知后端')
            flags=['-O3','-std=c++17','-shared','-fPIC','-fno-fast-math','-ffp-contract=off']
            command=[compiler,*flags,str(source),'-o',str(candidate)]
        result=subprocess.run(command,text=True,capture_output=True,timeout=180,cwd=directory,env=env)
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
    except (ValueError,subprocess.SubprocessError,OSError) as exc:print(json.dumps(dict(ok=False,error=str(exc)),ensure_ascii=False));return 1
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0


if __name__=='__main__':sys.exit(main())

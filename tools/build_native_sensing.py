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
    source=ROOT/'native/sensing.cpp';output=Path(output or ROOT/'native/libsensing.so').absolute()
    compiler=compiler or os.environ.get('CXX') or shutil.which('c++') or shutil.which('clang++')
    if not compiler:raise ValueError('没有可用 C++ 编译器；继续使用 NumPy 感知后端')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='m20-native-sensing-',dir=str(output.parent)) as directory:
        candidate=Path(directory)/'libsensing.so'
        flags=['-O3','-std=c++17','-shared','-fPIC','-fno-fast-math','-ffp-contract=off']
        command=[compiler,*flags,str(source),'-o',str(candidate)]
        result=subprocess.run(command,text=True,capture_output=True,timeout=120)
        if result.returncode:raise ValueError('原生感知编译失败：'+result.stderr[-5000:])
        os.replace(candidate,output)
    return dict(ok=True,abi=1,compiler=compiler,flags=flags,output=str(output),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                scope='local_build_only_no_robot_deployment')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output');parser.add_argument('--compiler');args=parser.parse_args()
    try:result=build(args.output,args.compiler)
    except (ValueError,subprocess.TimeoutExpired,OSError) as exc:print(json.dumps(dict(ok=False,error=str(exc)),ensure_ascii=False));return 1
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0


if __name__=='__main__':sys.exit(main())

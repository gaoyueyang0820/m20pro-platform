#!/bin/zsh
set -e
m20_dir="$(cd -- "$(dirname -- "$0")/../.." && pwd)"
cd "$m20_dir"
python3 -m venv data/runtime-venv
data/runtime-venv/bin/python -m pip install --disable-pip-version-check --upgrade pip
data/runtime-venv/bin/python -m pip uninstall -y opencv-python-headless
data/runtime-venv/bin/python -m pip install --disable-pip-version-check --only-binary=:all: -r requirements-video.txt 'ultralytics==8.4.190' 'lap==0.5.12'
print '人员识别与跟踪依赖已安装。请重启平台，在“视觉识别”开启人数统计，或在“自主作业”开启跟随。'
print '此脚本不会下载模型、连接机器人或启动跟随。'

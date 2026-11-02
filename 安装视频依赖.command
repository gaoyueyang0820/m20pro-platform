#!/bin/zsh
set -e
m20_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$m20_dir"
python3 -m venv data/runtime-venv
data/runtime-venv/bin/python -m pip install --disable-pip-version-check --upgrade pip
data/runtime-venv/bin/python -m pip uninstall -y opencv-python-headless
data/runtime-venv/bin/python -m pip install --disable-pip-version-check --only-binary=:all: -r requirements-video.txt
print '视频依赖已安装。请使用“启动平台”或“共享平台”启动。'

#!/bin/zsh
m20_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$m20_dir" || exit 1
print '本机打开 http://127.0.0.1:8080/，在连接中心查看共享地址（无需口令）。'
m20_python="$m20_dir/data/runtime-venv/bin/python"
[[ -x "$m20_python" ]] || m20_python=python3
exec "$m20_python" m20_platform.py --http "${M20_HTTP_PORT:-8080}" --share "${M20_SHARE_SUBNET:-172.20.10.0/28}"

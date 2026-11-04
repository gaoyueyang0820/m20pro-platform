@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 正在为内置 Python 安装/更新项目依赖...
runtime\python.exe -m pip install -r requirements-windows.txt --index-url https://pypi.org/simple --retries 10 --timeout 120
echo.
echo 完成，按任意键关闭。
pause >nul

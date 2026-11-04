@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo   具身智能机器狗 · 演示模式（无需连接机器人）
echo   访问地址: http://127.0.0.1:8081/
echo ============================================
echo.
start "" /min cmd /c "timeout /t 3 >nul & explorer http://127.0.0.1:8081/"
runtime\python.exe -X utf8 m20_platform.py --demo --http 8081 --data-dir "%~dp0userdata-demo"
echo.
echo 平台已退出，按任意键关闭窗口。
pause >nul

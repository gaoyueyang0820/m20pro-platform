@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo   具身智能机器狗 · 机器人操作平台
echo   访问地址: http://127.0.0.1:8080/
echo   关闭本窗口即停止平台
echo ============================================
echo.
start "" /min cmd /c "timeout /t 3 >nul & explorer http://127.0.0.1:8080/"
runtime\python.exe -X utf8 m20_platform.py --http 8080 --data-dir "%~dp0userdata"
echo.
echo 平台已退出，按任意键关闭窗口。
pause >nul

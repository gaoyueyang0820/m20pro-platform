@echo off
setlocal EnableExtensions DisableDelayedExpansion
chcp 65001 >nul
cd /d "%~dp0"
if not exist "runtime\python.exe" (
  call "windows\dispatch.cmd" setup
  exit /b
)
"runtime\python.exe" -m pip uninstall -y opencv-python-headless
if errorlevel 1 goto failed
"runtime\python.exe" -m pip install -r requirements-windows.txt --retries 3 --timeout 30
if errorlevel 1 goto failed
"runtime\python.exe" -X utf8 tools\check_video.py --bundled
if errorlevel 1 goto failed
echo Dependencies installed and video decoder verified.
pause
exit /b 0
:failed
echo Setup did not complete. See the error above.
pause
exit /b 1

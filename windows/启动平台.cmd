@echo off
setlocal EnableExtensions DisableDelayedExpansion
chcp 65001 >nul
cd /d "%~dp0.."
if not exist "runtime\python.exe" (
  call "windows\dispatch.cmd" start
  exit /b
)
set "M20_PORTABLE=1"
set "PYTHONUTF8=1"
"runtime\python.exe" -X utf8 "windows\launch.py" start
set "M20_RESULT=%ERRORLEVEL%"
if not "%M20_RESULT%"=="0" echo Platform did not start. See the error above.
pause
exit /b %M20_RESULT%

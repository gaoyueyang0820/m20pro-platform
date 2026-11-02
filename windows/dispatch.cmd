@echo off
setlocal EnableExtensions DisableDelayedExpansion
chcp 65001 >nul
set "PYTHONUTF8=1"
cd /d "%~dp0.."
set "M20_PY="
if /i not "%~1"=="setup" if exist "data\runtime-venv\Scripts\python.exe" (
  set "M20_PY=data\runtime-venv\Scripts\python.exe"
  goto run
)
where py >nul 2>nul
if not errorlevel 1 (
  for %%V in (3.13 3.12 3.11) do (
    py -%%V -c "import struct,sys;sys.exit(0 if struct.calcsize('P')==8 else 1)" >nul 2>nul
    if not errorlevel 1 (
      set "M20_PY=py -%%V"
      goto run
    )
  )
)
where python >nul 2>nul
if not errorlevel 1 (
  python -c "import sys,struct;sys.exit(0 if (3,11)<=sys.version_info[:2]<=(3,13) and struct.calcsize('P')==8 else 1)" >nul 2>nul
  if not errorlevel 1 (
    set "M20_PY=python"
    goto run
  )
)
echo Install standard 64-bit Python 3.13 from https://www.python.org/downloads/windows/
echo Select "Add python.exe to PATH", then run Setup-Windows.cmd again.
pause
exit /b 1
:run
if /i "%~1"=="setup" (
  %M20_PY% -X utf8 "windows\setup.py"
) else if /i "%~1"=="video" (
  %M20_PY% -X utf8 "windows\setup.py" --video-only
) else if /i "%~1"=="check" (
  %M20_PY% -X utf8 "windows\check.py"
) else (
  %M20_PY% -X utf8 "windows\launch.py" "%~1"
)
set "M20_RESULT=%ERRORLEVEL%"
if not "%M20_RESULT%"=="0" echo The operation did not complete. See the error above.
pause
exit /b %M20_RESULT%

@echo off
call "%~dp0windows\dispatch.cmd" check
exit /b %ERRORLEVEL%

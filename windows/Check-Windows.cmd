@echo off
call "%~dp0dispatch.cmd" check
exit /b %ERRORLEVEL%

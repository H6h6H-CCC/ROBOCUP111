@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\Invoke-CMake.ps1" %*
exit /b %ERRORLEVEL%

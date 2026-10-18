@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup\windows\run.ps1" -Mode Baseline
set "course_exit=%ERRORLEVEL%"
if not "%course_exit%"=="0" pause
exit /b %course_exit%

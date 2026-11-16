@echo off
setlocal DisableDelayedExpansion
pushd "%~dp0"
if errorlevel 1 goto failed
set "COFLY_PUSHED=1"
echo [CoFly Autopilot] Working directory: "%CD%"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ".\prepare_python.ps1" -ToolDirectory "%CD%" -Modules numpy,pandas,matplotlib,tkinterdnd2
if errorlevel 1 goto failed
".venv\Scripts\python.exe" main.py %*
if errorlevel 1 goto failed
popd
exit /b 0
:failed
echo.
echo Unable to start. See the error above for details.
if defined COFLY_PUSHED popd
pause
exit /b 1

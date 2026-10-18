@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  python -m venv .venv
  if errorlevel 1 goto fail
)
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto fail
".venv\Scripts\python.exe" g1_soccer.py --demo --hold --verify %*
if errorlevel 1 goto fail
pause
exit /b 0
:fail
echo 自动演示启动失败，请保留本窗口中的完整报错。
pause
exit /b 1

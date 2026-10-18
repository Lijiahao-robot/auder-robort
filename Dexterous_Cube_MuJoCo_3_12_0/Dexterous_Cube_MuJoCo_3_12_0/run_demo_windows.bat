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
".venv\Scripts\python.exe" dexterous_cube.py --hold --verify %*
if errorlevel 1 goto fail
pause
exit /b 0
:fail
echo 演示未完成或运行失败，请查看上方报告并保留本窗口信息。
pause
exit /b 1

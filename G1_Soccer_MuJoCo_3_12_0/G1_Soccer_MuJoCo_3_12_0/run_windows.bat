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
".venv\Scripts\python.exe" start_web.py %*
if errorlevel 1 goto fail
pause
exit /b 0
:fail
echo.
echo 启动失败，请保留本窗口中的完整报错。
echo 若提示找不到 python，请安装 Python 3.11 并勾选 Add Python to PATH。
pause
exit /b 1

@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" goto setup
".venv\Scripts\python.exe" -c "import mujoco, numpy, PIL; assert mujoco.__version__ == '3.12.0'" >nul 2>&1
if not errorlevel 1 goto run
:setup
call "1_安装依赖.bat"
if errorlevel 1 exit /b 1
:run
".venv\Scripts\python.exe" verify_simulation.py
pause

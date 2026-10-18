@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto install
py -3.11 --version >nul 2>&1
if errorlevel 1 goto fallback
py -3.11 -m venv .venv
if errorlevel 1 goto failed
goto install
:fallback
python -c "import sys; assert sys.version_info >= (3,10)" >nul 2>&1
if errorlevel 1 goto needpython
python -m venv .venv
if errorlevel 1 goto failed
:install
".venv\Scripts\python.exe" -m pip --version >nul 2>&1
if not errorlevel 1 goto packages
echo Repairing missing pip in the project virtual environment...
".venv\Scripts\python.exe" -m ensurepip --upgrade --default-pip
".venv\Scripts\python.exe" -m pip --version >nul 2>&1
if not errorlevel 1 goto packages
echo ensurepip did not repair pip. Trying the base Python pip...
py -3.11 -m pip --python ".venv\Scripts\python.exe" install --upgrade --force-reinstall pip
".venv\Scripts\python.exe" -m pip --version >nul 2>&1
if not errorlevel 1 goto packages
python -m pip --python ".venv\Scripts\python.exe" install --upgrade --force-reinstall pip
".venv\Scripts\python.exe" -m pip --version >nul 2>&1
if errorlevel 1 goto pipfailed
:packages
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
".venv\Scripts\python.exe" -c "import mujoco; print('MuJoCo:', mujoco.__version__); assert mujoco.__version__ == '3.12.0'"
if errorlevel 1 goto failed
echo Installation complete. Run 2_启动Atlas.bat
exit /b 0
:needpython
echo Python 3.10 or newer is required. Python 3.11 x64 is recommended.
echo Install Python and enable Add Python to PATH, then retry.
pause
exit /b 1
:pipfailed
echo Could not install pip. Repair your full Python installation and enable pip.
echo Your existing project files and virtual environment have been kept.
pause
exit /b 1
:failed
echo Installation failed. See the error above.
pause
exit /b 1

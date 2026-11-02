@echo off
setlocal
cd /d "%~dp0"
".venv\Scripts\python.exe" app.py --calibracion
if errorlevel 1 pause

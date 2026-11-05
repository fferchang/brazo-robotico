@echo off
setlocal
cd /d "%~dp0.."
".venv\Scripts\python.exe" simulacion_mujoco.py %*
if errorlevel 1 pause

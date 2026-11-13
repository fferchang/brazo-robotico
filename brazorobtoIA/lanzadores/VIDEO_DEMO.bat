@echo off
setlocal
cd /d "%~dp0.."
".venv\Scripts\python.exe" video_demo.py --abrir %*
if errorlevel 1 pause

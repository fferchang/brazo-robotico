@echo off
setlocal
cd /d "%~dp0.."
".venv\Scripts\python.exe" simulador_app.py
if errorlevel 1 pause

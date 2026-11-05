@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo Primero ejecuta INSTALAR.bat.
  pause
  exit /b 1
)
rem El brazo virtual hace solo la tarea completa. No usa el ESP32 ni las camaras.
".venv\Scripts\python.exe" gemelo_digital.py --visor %*
if errorlevel 1 pause

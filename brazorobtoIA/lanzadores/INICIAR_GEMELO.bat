@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo Primero ejecuta INSTALAR.bat.
  pause
  exit /b 1
)
rem La aplicacion de siempre, conectada al brazo y las camaras virtuales.
".venv\Scripts\python.exe" app.py --gemelo
if errorlevel 1 pause

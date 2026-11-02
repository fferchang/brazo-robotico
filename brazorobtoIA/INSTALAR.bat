@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  rem Python 3.12 si esta instalado; si no, el Python 3 disponible.
  py -3.12 -m venv .venv 2>nul
  if errorlevel 1 py -3 -m venv .venv
  if errorlevel 1 goto :error
)
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 goto :error
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto :error
echo.
echo Instalacion terminada. Ya podes ejecutar INICIAR.bat.
pause
exit /b 0
:error
echo.
echo ERROR: no se pudo completar la instalacion.
pause
exit /b 1


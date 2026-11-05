@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo Primero ejecuta INSTALAR.bat.
  pause
  exit /b 1
)
echo Prueba de hardware de solo lectura: no mueve el brazo ni cambia el torque.
echo Antes de seguir:
echo   - cerra la aplicacion principal: camaras y puerto COM tienen que estar libres;
echo   - conecta las dos camaras y el ESP32, con los servos alimentados;
echo   - deja el brazo quieto y el cubo rojo y el recipiente a la vista.
echo.
pause
".venv\Scripts\python.exe" prueba_hardware.py %*
echo.
pause

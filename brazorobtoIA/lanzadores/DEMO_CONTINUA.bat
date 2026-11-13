@echo off
setlocal
cd /d "%~dp0.."
title Brazo IA - Demo continua (MuJoCo, solo virtual)
rem Evita que la PC entre en suspension o apague la pantalla mientras corre la demo (con corriente).
powercfg /change standby-timeout-ac 0 >nul 2>&1
powercfg /change monitor-timeout-ac 0 >nul 2>&1
:loop
".venv\Scripts\python.exe" simulacion_mujoco.py --episodios 0 --cerrar-al-terminar %*
rem Codigo 0 = se cerro la ventana a mano: termina. Otro codigo = error inesperado: reinicia.
if not errorlevel 1 goto fin
echo.
echo La simulacion se detuvo por un error. Reinicio en 5 segundos (Ctrl+C para cancelar)...
timeout /t 5 >nul
goto loop
:fin
endlocal

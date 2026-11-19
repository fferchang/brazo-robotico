@echo off
setlocal
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo Primero ejecuta INSTALAR.bat.
  pause
  exit /b 1
)
title Brazo IA - Demo con el brazo (no cerrar)
rem Evita que la PC entre en suspension o apague la pantalla mientras corre la demo (con corriente).
rem El cambio queda despues de la demo: se vuelve atras en Opciones de energia.
powercfg /change standby-timeout-ac 0 >nul 2>&1
powercfg /change monitor-timeout-ac 0 >nul 2>&1
echo Demo con el brazo: conecta el robot y las camaras, y en "Tarea automatica" toca DEMO con Vueltas = 0.
echo Con la casilla de reintento marcada, si una vuelta falla suelta el cubo, vuelve a Inicio y lo busca de nuevo.
rem Pasar --gemelo para probarla con el brazo virtual.
".venv\Scripts\python.exe" app.py %*
if errorlevel 1 pause

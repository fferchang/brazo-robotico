# Simulador manual de Rerun

Abrir `lanzadores/SIMULADOR_RERUN.bat`, o usar **Simulador manual · Rerun** en la sección
de entrenamiento de la aplicación principal. Se abre una ventana de controles
y una grabación de Rerun identificada como `SIMULADOR MANUAL · SOLO VIRTUAL`.

- Arrastrar M1–M6 actualiza el modelo 3D. También se puede escribir un ángulo
  y pulsar **Aplicar** o Enter; se acepta coma decimal.
- **Pose inicial** coloca M1–M5 en 180°, 180°, 90°, 180°, 263°, y M6 en 0°.
- **Abrir/Cerrar garra virtual** cambia únicamente M6 en el modelo.
- **Copiar ángulos** copia los seis valores virtuales para compartir una pose.
- El indicador muestra la menor altura de la garra sobre el piso: rojo a
  5 mm o menos; contacto/intersección a 0 mm o menos.

Los límites de los controles vienen de `config.json`. Dentro de esos límites,
la simulación permite posiciones que las reglas de movimiento físico rechazan.
No recorta los ángulos a los límites internos del URDF; avisa si los supera,
para que el movimiento virtual refleje exactamente el valor solicitado.

Esta ventana no importa el controlador del robot, no abre puertos serie,
no usa cámaras, no modifica muestras ni modelos entrenados y no envía órdenes
al ESP32 al abrirse, mover controles o cerrarse. No representa presión,
contactos dinámicos ni ejecución de trayectorias. La medición geométrica es
diagnóstica; consultar `docs/DIAGNOSTICO_PISO_3D.md` para la calibración pendiente.

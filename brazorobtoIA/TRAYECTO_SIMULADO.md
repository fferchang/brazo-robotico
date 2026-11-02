# Prueba de inicio a final, solamente en Rerun

1. Abrir `TRAYECTO_SIMULADO.bat`. Inicia la aplicación principal y una ventana
   **TRAYECTO SIMULADO**. No conecta automáticamente el ESP32 ni mueve motores.
2. Conectar el ESP32 desde la aplicación principal. Para acomodar M1–M5 a
   mano, usar el botón habitual de torque OFF, que conserva el torque de M6.
3. Colocar el brazo en el inicio, dejarlo quieto y pulsar **Capturar del brazo**
   en la columna **Inicio** de la ventana de simulación.
4. Acomodar el brazo en el final, dejarlo quieto y capturar la columna **Final**.
5. Pulsar **Calcular y simular**. Si encuentra un recorrido verificado, lo
   reproduce en Rerun. El brazo físico permanece donde se lo dejó a mano.
6. Usar **Detener**, la barra de recorrido y **Reproducir otra vez** para examinar
   la animación. La velocidad es virtual, configurable entre 1 y 60 grados/s.

También se pueden escribir los doce ángulos, sin ESP32, ejecutando
`trayecto_app.py`. Desde el simulador manual, **Planificar trayecto Inicio →
Final** permite copiar las poses virtuales con **Usar pose virtual**.

La captura desde el brazo solo lee una copia de la telemetría ya recibida por
la aplicación principal. Rechaza datos incompletos, antiguos o no finitos.
No ordena lecturas adicionales, cambios de torque, apertura/cierre de garra ni
movimientos. Cerrar esta ventana de trayectoria tampoco envía comandos. La
aplicación principal mantiene sus funciones y su comportamiento de cierre.

## Qué comprueba

Usa las mallas y la calibración del modelo 3D, incluyendo ambos dedos de la
garra y M5/M6. Exige más de **5 mm** de separación respecto al piso virtual.
La altura del eje M2 se mantiene en 100 mm. No usa el modelo de aprendizaje.

Primero intenta interpolar los seis ángulos directamente. Si no pasa, busca
rutas elevando M2/M3/M4 hasta la pose inicial estándar, probando el movimiento
conjunto y los seis órdenes individuales. Conecta las dos poses elevadas y
baja al destino. M1, M5 y M6 también se verifican cuando cambian.

Cada tramo se subdivide adaptativamente. La suma de radios máximos de los
vértices por desplazamiento angular acota cuánto puede bajar cualquier punto
de la garra entre muestras. Solo se acepta un intervalo cuando su cota mínima
supera el margen. La altura mínima informada es una cota conservadora del
modelo continuo, no simplemente el mínimo de las fotos o de los cuadros de
animación. Esto supone la interpolación articular sincronizada que se anima.

Si una pose toca el piso virtual, si un tramo no puede verificarse o si la
búsqueda agota 15 segundos, no se reproduce una ruta parcial. La búsqueda no
es exhaustiva: no encontrar una ruta no demuestra que no exista.

Editar o recapturar una pose invalida el plan anterior y descarta resultados
de cálculos pendientes. No se corrigen ángulos ni alturas silenciosamente.

## Alcance

Solo verifica garra contra plano del piso. No comprueba autocollisiones,
obstáculos, cubo, recipiente, cargas ni dinámica del servo. No valida la
ejecución real ni modifica reglas del ESP32. Las diferencias de calibración
documentadas en `DIAGNOSTICO_PISO_3D.md` siguen pendientes. No se generan
órdenes físicas, no se graban demostraciones ni se reentrena ningún modelo.

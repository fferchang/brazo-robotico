# Brazo Robot IA — versión final

Aplicación local para enseñar y ejecutar, de forma supervisada, la tarea:

1. detectar un cubo rojo;
2. aproximarse y agarrarlo;
3. trasladarlo;
4. soltarlo dentro del recipiente verde lima rectangular;
5. volver a la pose guardada.

## Principios

### Reintentar la última foto

Durante una demostración activa, acomodá de nuevo el brazo y pulsá **Reintentar
última foto**, junto a **Tomar foto**. Confirmá para sustituir las imágenes de ambas
cámaras, los ángulos y las observaciones de la última etapa. No mueve el brazo ni
avanza el contador. También funciona en 12/12, antes de guardar como exitosa.

Las tomas anteriores se conservan como respaldo (`replaced_captures` en el
metadata y sus imágenes); el entrenamiento utiliza únicamente las capturas
vigentes de `stages`. Si falla el guardado, la toma anterior sigue vigente.
No modifica demostraciones ya finalizadas ni modelos ya entrenados.

- Funciona localmente y no necesita Internet durante el uso.
- Dos cámaras fijas: `USB CAMERA` frontal y `GENERAL WEBCAM` lateral.
- Panel independiente por cámara para ajustar en vivo brillo, contraste,
  saturación y zoom digital, con guardado persistente y restauración a valores
  neutros. La configuración guardada es frontal `brillo −25%`, `contraste +50%`,
  `saturación +10%`, `zoom 1×`; lateral `brillo −25%`, `contraste +50%`,
  `saturación 0%`, `zoom 1×`. El panel inicia bloqueado y atenuado; **Editar
  ajustes** lo habilita temporalmente. Se aplican por igual a la vista previa,
  las imágenes guardadas y la detección.
- Doce poses clave por demostración, con una imagen de cada cámara y los seis
  ángulos reales en cada pose.
- El torque y la captura son acciones separadas. Tomar una foto nunca cambia el
  torque ni ordena movimiento.
- Debajo de la telemetría hay un botón de pose fija que lleva M1–M5 a
  `180°, 90°, 270°, 180°, 270°`. Pide confirmación, usa solamente
  `arm torque on` si hace falta y nunca envía comandos a M6.
- A su derecha, **Pose inicial** lleva M1–M5 a
  `180°, 180°, 90°, 180°, 263°`, con el mismo mecanismo y sin afectar M6.
  Usar esa pose para las fotos de Inicio y Guardado; cerrar la garra por separado.
- La ejecución automática pide confirmación antes de cada etapa.
- El firmware del ESP32 vuelve a validar límites, piso y colisiones.

## Doce poses de una demostración

1. `inicio`: garra cerrada y brazo en la posición inicial estándar.
2. `aproximacion_1`: primer punto entre inicio y cubo, todavía con la garra cerrada.
3. `aproximacion_2`: más cerca del cubo; abrir la garra antes de la foto.
4. `aproximacion_3`: garra cerrada y alineada con el cubo.
5. `apertura_sobre_cubo`: abrir la garra sobre el cubo.
6. `descenso_abierto`: bajar la garra abierta al lado del cubo.
7. `agarre`: cerrar por presión, esperar `GRIPPER_HOLD` y fotografiar el cubo agarrado.
8. `traslado_1`: primer punto de traslado; M6 permanece sosteniendo el cubo.
9. `traslado_2`: continuar hacia el recipiente sosteniendo el cubo.
10. `traslado_3`: ubicar el cubo sostenido sobre el centro del recipiente.
11. `soltado`: abrir la garra, comprobar que el cubo quedó dentro y tomar la foto.
12. `guardado`: volver a la pose de `inicio`, cerrar la garra y tomar la última foto.

Pulsar **Tomar foto** en cada una de las doce etapas, también en apertura,
descenso y cierre por presión. Cada pose genera dos imágenes PNG y un registro JSON con telemetría, detecciones,
orientación de cámaras y estado del robot.

Cada prueba crea su propia carpeta en el momento de iniciarse:

- `datos/demostraciones/prueba_AAAAMMDD_HHMMSS_microsegundos/` contiene las 24
  imágenes y su `metadata.json`.
- `datos/ejecuciones/prueba_AAAAMMDD_HHMMSS_microsegundos/` contiene el
  `metadata.json` de la ejecución automática.

Así, los archivos de dos pruebas distintas nunca se mezclan.

## Instalación

Ejecutar `INSTALAR.bat` una sola vez. Después, `INICIAR.bat` abre la aplicación.
La instalación puede necesitar Internet para descargar dependencias; la ejecución
posterior es completamente local.

## Flujo de enseñanza

1. Abrir las dos cámaras y comprobar su orientación. Cada botón de cámara la gira
   180° de forma independiente.
2. Conectar el ESP32. Conectar solo solicita `posall`: no activa torque ni mueve.
3. Pulsar **Nueva demostración**.
4. Para cada pose: apagar torque, acomodar físicamente el brazo, activar torque,
   esperar que los seis ángulos estén estables y pulsar **Tomar foto**.
   El operador decide si la toma es apta. Torque, estabilidad, límites y estado
   de la garra sólo generan avisos no bloqueantes y quedan registrados en el
   `metadata.json`. La captura sólo se rechaza si faltan alguna de las dos
   imágenes o alguno de los seis ángulos necesarios para entrenar.
5. Respetar el estado de la garra indicado en cada etapa. La aplicación avisa si
   M6 no coincide, pero permite guardar la foto porque la evaluación final de la
   toma corresponde al operador.
6. Después de las doce poses, guardar la demostración como exitosa. Una toma
   interrumpida se conserva para auditoría, pero no entra al entrenamiento.

Desde 5 demostraciones exitosas se permite entrenar un **modelo piloto** para
comprobar el circuito completo bajo supervisión. Con menos de 20 muestras no se
debe considerar confiable ni generalizar fuera de posiciones muy parecidas a las
enseñadas. Para la versión final se recomiendan aproximadamente 300, variando las
posiciones del cubo y del recipiente sobre la misma mesa; el 20% se reserva
automáticamente para validación.

Aunque haya cinco muestras, el modelo no se guarda si la validación supera
`20°` de error medio o `45°` en alguna etapa. Esto evita cargar automáticamente
una política incoherente en un brazo físico.

Los avisos de captura no excluyen muestras completas marcadas como exitosas del
entrenamiento. Para análisis offline, `train_policy(..., export_unvalidated=True)`
permite exportar resultados aunque no superen la validación. Se identifican como
experimentales y la aplicación rechaza cargarlos para mover el brazo. Esto no
desactiva reglas geométricas, límites de motores ni protecciones del firmware.

## Ejecución supervisada

### Probar un modelo sin hardware

Abrí `PRUEBA_OFFLINE.bat`, o **Probar modelo OFFLINE** junto al botón de entrenar.
Es una ventana/proceso separado: no abre COM, no captura cámaras en vivo y no
envía órdenes al brazo. Puede abrir modelos experimentales sin habilitarlos en
la aplicación de control físico.

1. Carga automáticamente el modelo más reciente de `modelos/no_validados` o el
   modelo activo; también permite elegir otro `.npz`.
2. Elegí una demostración guardada. Si el modelo registra la partición, se
   selecciona inicialmente una de validación y se identifica como tal.
3. Recorré las etapas con Anterior/Siguiente, el selector o Reproducir 12 etapas.
4. Compará las fotos, los ángulos enseñados, la predicción cruda y el error M1–M5.
5. Pulsá **Ver predicción en Rerun** para visualizar las poses en una grabación
   identificada como OFFLINE. Pausar detiene únicamente el avance visual.

Los valores de la vista se recortan a los límites y el URDF puede recortarlos
adicionalmente; la tabla no oculta la predicción cruda ni su error. Inicio y
guardado usan la pose canónica del modelo en la vista, como el flujo supervisado.
M6 copia el ángulo grabado: no es una predicción ni simula presión. Las fotos y
características son las guardadas, no hay interacción simulada con los objetos.
Esta prueba no verifica colisiones, trayectorias ni éxito físico.

La aplicación primero propone una pose. **No mueve el brazo** hasta pulsar
**CONFIRMAR Y MOVER** y aceptar el cuadro de confirmación de esa etapa. Si la
pose inicial se desvió, la primera propuesta es una recuperación a la pose de
inicio aprendida. Al terminar, se realiza una comprobación visual automática y
el operador registra el resultado real.

## Simulación física en MuJoCo

`SIMULACION_MUJOCO.bat` ejecuta la tarea completa en un entorno virtual con
física: el brazo, la mesa, el cubo y el recipiente. Un experto por cinemática
inversa calcula las doce poses a partir de la posición del cubo y del recipiente,
sin demostraciones. No abre puertos serie ni usa las cámaras reales. Ver
`SIMULACION_MUJOCO.md`.

## Calibración modelo-brazo

`CALIBRACION.bat`, o **Calibración · toques de mesa** en la sección de
entrenamiento, abre una ventana de solo lectura. Captura toques de la garra
sobre la mesa y la garra cerrada, y estima la altura real de la mesa y los ceros
de M2–M4 y M6. No modifica `calibracion.json`. Ver `CALIBRACION.md`.

## Hardware esperado

- ESP32 preferido en `COM4`, 115200 baud. Si cambia el número y hay un solo
  adaptador USB serie, la interfaz lo selecciona automáticamente.
- Firmware compatible con `posall`, `arm torque on/off`, `torque on/off`, `pair`,
  `gripper open/close/stop`.
- Dos cámaras DirectShow.
- Seis servos STS3215.

## Seguridad

Los límites configurados son M1 `60..300`, M2 `90..270`, M3 `90..276`,
M4 `75..278`, M5 `0..340` y M6 lógico `0..140`. La geometría de software es
una aproximación y no sustituye la supervisión humana ni un paro físico.

El planificador (`brazoia/planner.py`) y el firmware usan la misma regla de piso:
el contorno de la garra, según la apertura de M6 y para cualquier giro de M5,
tiene que quedar a más de 5 mm de la mesa. Su modelo plano sale del modelo 3D
calibrado (eje M2 a 121,9 mm de la mesa). Si la pose actual ya está bajo ese
margen, se permite salir sin acercarse más a la mesa. Detalle en
`REGLAS_GEOMETRIA.md`. Tras cambiar el firmware hay que volver a cargarlo en
el ESP32.

El botón **Torque ON** usa la orden global compatible con ambas versiones del
firmware. El botón **Torque brazo OFF** usa `arm torque off` y libera únicamente
M1–M5; M6 conserva su estado para no soltar el cubo durante la enseñanza. La
emergencia y el cierre usan `torque off` global y sí desenergizan M1–M6.


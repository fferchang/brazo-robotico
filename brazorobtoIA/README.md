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
  `metadata.json` de la ejecución automática y, desde el 6/11/2026, sus fotos:
  la de Inicio con la que se planificó (y dónde ubicó el cubo y el recipiente,
  con la distancia del cubo a la base), una al terminar cada etapa con los
  ángulos y el estado de la garra, una si una etapa se bloquea y la de la
  verificación final. También anota los avisos del ejecutor («Llegada con
  desvío», «Garra abierta a …»). Sirve para entender después, sin el brazo, por
  qué una corrida falló.

Así, los archivos de dos pruebas distintas nunca se mezclan.

## Instalación

Ejecutar `lanzadores/INSTALAR.bat` una sola vez. Después, `lanzadores/INICIAR.bat` abre la aplicación.
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
posiciones del cubo y del recipiente sobre la misma mesa.

El modelo predice las doce poses a partir de la escena de **Inicio**: la posición
del cubo y del recipiente en las dos cámaras, antes de mover el brazo. Es lo mismo
que ve la ejecución automática; las fotos posteriores muestran el brazo tapando o
llevando el cubo. Por eso, en la foto de Inicio el cubo y el recipiente tienen que
detectarse en ambas cámaras; las demostraciones que no lo cumplen se excluyen y
quedan listadas en el modelo. Por etapa usa una regresión lineal con
regularización elegida automáticamente: con pocas demostraciones tiende a la pose
media en vez de extrapolar. Inicio y Guardado usan una pose canónica.

La validación es cruzada: cada demostración se predice con un modelo entrenado
sin ella. Aunque haya cinco muestras, el modelo no se guarda si ese error supera
`20°` de media o `45°` en alguna etapa. Esto evita cargar automáticamente una
política incoherente en un brazo físico. Los modelos entrenados con la versión
anterior se rechazan: hay que volver a entrenar con las demostraciones guardadas.

Los avisos de captura no excluyen muestras completas marcadas como exitosas del
entrenamiento. Para análisis offline, `train_policy(..., export_unvalidated=True)`
permite exportar resultados aunque no superen la validación. Se identifican como
experimentales y la aplicación rechaza cargarlos para mover el brazo. Esto no
desactiva reglas geométricas, límites de motores ni protecciones del firmware.

## Ejecución supervisada

### Gemelo digital: el brazo virtual hace la tarea solo

`lanzadores/GEMELO_DIGITAL.bat` muestra el brazo de MuJoCo haciendo la tarea
completa sin ayuda, con el mismo código de ejecución que el brazo real:
`RobotSerial` habla con un firmware virtual y las cámaras virtuales reemplazan a
las reales. `lanzadores/INICIAR_GEMELO.bat` abre esta aplicación conectada al
gemelo (puerto `SIMULADO`), con un botón para hacer las doce etapas solo. No usa
el ESP32 ni las cámaras, y no mezcla datos con los reales. Ver
`docs/GEMELO_DIGITAL.md`.

### Probar un modelo sin hardware

Abrí `lanzadores/PRUEBA_OFFLINE.bat`, o **Probar modelo OFFLINE** junto al botón de entrenar.
Es una ventana/proceso separado: no abre COM, no captura cámaras en vivo y no
envía órdenes al brazo. Puede abrir modelos experimentales sin habilitarlos en
la aplicación de control físico.

1. Carga automáticamente el modelo más reciente de `modelos/no_validados` o el
   modelo activo; también permite elegir otro `.npz`.
2. Elegí una demostración guardada. En los modelos nuevos, cada demostración
   muestra la predicción de validación cruzada: la de un modelo entrenado sin
   ella. En los anteriores se selecciona una de validación, si está registrada.
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

La comprobación visual busca rojo **dentro** del recipiente en cada cámara: da
«posible éxito» si alguna lo ve adentro y ninguna ve el recipiente vacío (una
cámara que no encuentra el recipiente no decide). Antes miraba si el rojo más
grande de la imagen caía en el recipiente, y fallaba: el cubo ya dentro asoma
poco por arriba de las paredes y, en la cámara lateral, quedaba por debajo del
área mínima, o algo rojo detrás de la mesa pasaba por el cubo. Con las fotos de
las 15 demostraciones de octubre aprueba 14 de 15 finales (antes 8) y ninguna de
las 90 fotos con el cubo todavía en la mesa. En las ejecuciones reales del
5/11 había dado «no confirmado» en 2 de los 4 éxitos.

## Simulación física en MuJoCo

`lanzadores/SIMULACION_MUJOCO.bat` ejecuta la tarea completa en un entorno virtual con
física: el brazo, la mesa, el cubo y el recipiente. Un experto por cinemática
inversa calcula las doce poses a partir de la posición del cubo y del recipiente,
sin demostraciones. No abre puertos serie ni usa las cámaras reales. Ver
`docs/SIMULACION_MUJOCO.md`.

## Calibración modelo-brazo

`lanzadores/CALIBRACION.bat`, o **Calibración · toques de mesa** en la sección de
entrenamiento, abre una ventana de solo lectura. Captura toques de la garra
sobre la mesa y la garra cerrada, y estima la altura real de la mesa y los ceros
de M2–M4 y M6. No modifica `calibracion.json`. Ver `docs/CALIBRACION.md`.

## Modo geométrico: cámaras calibradas

Alternativa al modelo aprendido para agarrar el cubo en cualquier lugar de la
zona calibrada. Con la foto de Inicio calcula dónde están el cubo y el
recipiente, y la cinemática inversa del modelo 3D calcula las doce poses, con
descenso y retiro verticales y un empuje que deja el cubo apoyado contra el dedo
fijo antes de cerrar. Se elige en **Ejecución automática supervisada**
y sigue pidiendo confirmación en cada etapa. Necesita una calibración hecha con
**Calibración · cámaras**; `lanzadores/SIMULACION_GEOMETRICA.bat` lo prueba con cámaras
virtuales. Ver `docs/CALIBRACION_CAMARAS.md`.

## Hardware esperado

- ESP32 preferido en `COM4`, 115200 baud. Si cambia el número y hay un solo
  adaptador USB serie, la interfaz lo selecciona automáticamente.
- Firmware compatible con `posall`, `arm torque on/off`, `torque on/off`, `pair`,
  `gripper open/close/stop`.
- Dos cámaras DirectShow.
- Seis servos STS3215.

## Prueba de hardware

`lanzadores/PRUEBA_HARDWARE.bat` revisa las cámaras y el ESP32 en modo solo lectura: no
mueve el brazo ni cambia el torque, y al ESP32 solo le envía `help`, `posall` y
`scan`. Antes de ejecutarla, cerrá la aplicación principal, dejá el brazo quieto
y poné el cubo y el recipiente a la vista de las dos cámaras.

- Cámaras: identificación por nombre, apertura, resolución, cuadros por segundo,
  imagen negra o congelada, desfase entre cámaras y detección del cubo y del
  recipiente con los mismos parámetros que la aplicación.
- ESP32: puerto, reinicio al conectar, versión del firmware (texto de `help`
  comparado con `firmware/firmware_secuencias.ino`), lectura de M1–M6, voltaje,
  temperatura, telemetría, límites y estabilidad en reposo.
- Integración: las condiciones que comprueba **Tomar foto**, salvo el torque.

Los resultados quedan en `datos/prueba_hardware/prueba_AAAAMMDD_HHMMSS/`:
`reporte.txt`, `reporte.json`, `serial.log` y las fotos de cada cámara, con y sin
la detección dibujada. Opciones: `--puerto COM5`, `--segundos 10`,
`--solo-camaras`, `--solo-esp32`. No prueba movimiento, torque ni garra.

## Seguridad

Los límites configurados son M1 `60..300`, M2 `90..270`, M3 `90..276`,
M4 `75..278`, M5 `0..340` y M6 lógico `0..140`. La geometría de software es
una aproximación y no sustituye la supervisión humana ni un paro físico.

El planificador (`brazoia/planner.py`) y el firmware usan la misma regla de piso:
el contorno de la garra, según la apertura de M6 y para cualquier giro de M5,
tiene que quedar a más de 5 mm de la mesa. Su modelo plano sale del modelo 3D
calibrado (eje M2 a 121,9 mm de la mesa). Si la pose actual ya está bajo ese
margen, se permite salir sin acercarse más a la mesa. Detalle en
`docs/REGLAS_GEOMETRIA.md`. Tras cambiar el firmware hay que volver a cargarlo en
el ESP32.

El botón **Torque ON** usa la orden global compatible con ambas versiones del
firmware. El botón **Torque brazo OFF** usa `arm torque off` y libera únicamente
M1–M5; M6 conserva su estado para no soltar el cubo durante la enseñanza. La
emergencia y el cierre usan `torque off` global y sí desenergizan M1–M6.


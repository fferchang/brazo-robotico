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

Abrí `lanzadores/PRUEBA_OFFLINE.bat`, o **Probar modelo OFFLINE** en la pestaña **Enseñar y entrenar**.
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

La interfaz tiene cámaras y telemetría siempre a la vista y cuatro pestañas:
**Tarea automática**, **Mover a mano** (poses de referencia, ángulos, torque y
garra), **Enseñar y entrenar** y **Cámaras y calibración**. La ventana abre
maximizada y **EMERGENCIA · TORQUE OFF** queda siempre abajo a la derecha.

En **Tarea automática** hay dos formas:

- **HACER LA TAREA SOLA**: las etapas seguidas, sin confirmar cada una. Enciende
  el torque de M1–M5 si estaba apagado, espera a que el brazo quede quieto entre
  etapas y pasa por el mismo planificador y las mismas reglas del firmware.
  **Detener** o **EMERGENCIA** la cortan en cualquier momento, también entre dos
  etapas y antes de abrir o cerrar la garra. Cada etapa se planifica desde la
  pose medida en ese momento: una etapa que quedó preparada paso a paso se
  vuelve a planificar. No arranca (y no mueve nada) si las cámaras no tienen
  imagen actual, si el cubo ya está dentro del recipiente (la escena de la tarea
  anterior) o si el modo geométrico lo ubica a más de 30 cm de la base (ahí el
  agarre falla en el gemelo). Si después de dos recuperaciones el brazo no queda
  en Inicio, se frena. Si una etapa se bloquea, la tarea queda ahí: se puede
  seguir paso a paso, volver a pulsar el botón o marcar el resultado. Los
  rechazos y bloqueos quedan en el registro de la ejecución. Si una tarea terminada no se marcó, al empezar la
  siguiente se guarda como `sin_marcar`.
- **Paso a paso**: la aplicación primero propone una pose y **no mueve el
  brazo** hasta pulsar **CONFIRMAR Y MOVER** y aceptar el cuadro de esa etapa.
  Con el cubo a más de 30 cm solo avisa.

Si la pose inicial se desvió, la primera etapa es una recuperación a la pose de
inicio. Al terminar, se realiza una comprobación visual automática y el operador
registra el resultado real.

La comprobación visual busca rojo **dentro** del recipiente en cada cámara: da
«posible éxito» si alguna lo ve adentro y ninguna ve el recipiente vacío (una
cámara que no encuentra el recipiente no decide). Antes miraba si el rojo más
grande de la imagen caía en el recipiente, y fallaba: el cubo ya dentro asoma
poco por arriba de las paredes y, en la cámara lateral, quedaba por debajo del
área mínima, o algo rojo detrás de la mesa pasaba por el cubo. Con las fotos de
las 15 demostraciones de octubre aprueba 14 de 15 finales (antes 8) y ninguna de
las 90 fotos con el cubo todavía en la mesa. En las ejecuciones reales del
5/11 había dado «no confirmado» en 2 de los 4 éxitos.

### Movimiento más rápido en el modo geométrico (etapa 1)

En el modo geométrico (la tarea y la demo) la garra ya no copia los gestos de las
demostraciones a mano. Abre **una sola vez**, a 90° por ángulo, mientras el brazo va
hacia Aproximación 2: ya no cierra y reabre sobre el cubo. Suelta abriendo a 90° y en
Guardado cierra por presión mientras vuelve a Inicio, después del retiro vertical.
Con 90°, en el agarre el dedo móvil queda 35 mm por encima del cubo (con 137°, 43 mm).
El modelo aprendido y el modo paso a paso sin modo geométrico siguen como antes.

Menos paradas:
- Los tramos chicos de un mismo punto de paso (hasta 10° por motor) salen seguidos y
  los motores se mueven a la vez. Solo pasa si todas las combinaciones intermedias
  pasan las reglas 2D del firmware y los choques del modelo 3D.
- En los puntos de paso no se espera a que el brazo quede quieto: alcanza con pasar
  a 1,5°.
- Cada orden espera el acuse del firmware (`Moviendo ID…` / `PAIR_MOVE_OK`) antes de
  la siguiente: un rechazo o un AJUSTE SEGURO corta en el acto, y el enfriamiento de
  50 ms se cuenta desde ese acuse.

Abrir mientras el brazo se mueve se revisa contra la mesa en la pose de partida; si
no pasa, la garra abre recién al llegar. En Agarre, una garra que cierra hasta el tope
sin tocar el cubo (`closed_limit`) frena la tarea. En «Mover a mano», abrir M6 por
ángulo también se revisa contra la mesa (el firmware no lo hace).

En el gemelo, cada vuelta de la demo bajó de ~88 s a ~38 s. Lo que queda lento son
los movimientos grandes, como salir de Inicio y volver, que todavía van de a dos
motores: es la etapa 2 (los 5 motores a la vez desde el firmware).

### Etapa 2: cinco motores por orden (`sync`)

El firmware nuevo agrega `sync <ms> <id> <grados> ...`: hasta cinco motores de M1–M5
en un solo `SyncWritePosEx`, con velocidades proporcionales para que lleguen juntos.
Revisa piso, autocolisiones y la regla de M4 en 20 puntos de la recta, como `pair`, y
rechaza la orden entera (`SYNC_REJECTED`) sin ajustes automáticos. El techo de
velocidad sigue en 450 (~40°/s). M6 no va por `sync`.

- Al conectar, la app pregunta `caps`. Si el firmware responde `FIRMWARE_CAPS sync=1`,
  cada punto de paso «junto» sale en una sola orden y el siguiente se manda cuando el
  motor más atrasado está a un paso del destino, si la caja de esos puntos pasa las
  revisiones 2D y 3D de siempre. Con el firmware de la exposición no hay respuesta y
  todo sigue con `pair`, como antes.
- `app.py --sin-sync` y `gemelo_digital.py --sin-sync` fuerzan `pair` aunque el
  firmware tenga `sync`, para comparar.
- Gemelo (19/11): la vuelta de la demo bajó de 42,7–44,1 s a 33,1–35,9 s y las frenadas
  de los servos antes de la orden siguiente, de 97–98% a 43%. La tarea con recipiente,
  de 44–47 s a 36–39 s (3/3 en ambos casos). Falta medirlo en el brazo real.
- `firmware/prueba_pc/` compila el `.ino` en la PC con stubs de Arduino y del bus y
  prueba `sync`; `tests/test_orden_sync.py` lo corre si hay `g++`.

### Guardado preparado de antemano

Preparar Guardado (la vuelta larga a Inicio, con la garra cerrándose en camino) tardaba
3–7 s después de soltar, con el brazo quieto. Ahora se prepara en segundo plano apenas
se conoce la escena, desde el destino de Soltado y para cualquier M6 entre 84° y 96°
(la apertura de Soltado ± la tolerancia con que se acepta). Al terminar Soltado se usa
solo si M1–M5 quedaron a 1,2° o menos de ese destino y M6 dentro de la banda; si no,
se replanifica como siempre. El hilo usa su propia copia del modelo 3D.
`--sin-anticipar` vuelve a prepararlo al final. En el gemelo la espera pasó a 0 s.

### Supervisor de la fuente y velocidad máxima

Con el firmware nuevo, la app manda al conectar la sección `movimiento` de
`config.json`:

- `velocidad_max` (pasos/s, 100–1000; 450 ≈ 40°/s) y `aceleracion` (10–150) de M1–M5.
- `guardia_fuente`: cada 100 ms el firmware lee la corriente y el voltaje de los seis
  servos. Con el voltaje más bajo bajo `lento_v` o la corriente total sobre `lento_ma`,
  M1–M5 van a la mitad de velocidad, todos por igual y con los mismos destinos (la ruta
  no cambia), hasta 25%; vuelven de a 25% tras 1 s normal. Bajo `corte_v` o sobre
  `corte_ma` durante 200 ms el brazo se frena donde está (sin cortar el torque) y la
  tarea se detiene con `POWER_GUARD_HOLD`. La garra no se frena: tiene su protección.
- Valores iniciales: 9,5 V / 3000 mA para ir más lento y 8,0 V / 4500 mA para frenar.
  Son puntos de partida: hay que ajustarlos con «Medir corriente» en el brazo real.
- En el gemelo, subir `velocidad_max` casi no acelera la vuelta (450: 29,0 s; 1000 con
  aceleración 90: 26,4 s) y aumenta las frenadas, porque cada paso de 6° termina antes
  de que llegue la orden siguiente. Por eso queda en 450.

### Garra más rápida

Con el firmware nuevo (`caps` con `gripper_fast=1`):

- **Abrir**: `velocidad_garra_abrir` (900 pasos/s ≈ 79°/s; antes 450 por ángulo y 300 con
  «gripper open»). Solo al abrir, que es alejarse del objeto; cerrar por ángulo no cambia.
- **Cerrar por presión**: hasta 55° va a ~53°/s y después sigue como el final del cierre
  de siempre (consigna cerrada, servo a ~19°/s): llega al cubo a la misma velocidad que
  antes, con la detección de contacto y el corte por sobrecorriente activos todo el
  tiempo. En Guardado, sin cubo entre los dedos, va rápido hasta 10°.
  `"cierre_rapido": false` en `movimiento` vuelve al cierre de siempre.
- Gemelo: la vuelta de la demo bajó de 29,0 / 27,8 s a 26,8 / 25,7 s. La tarea con
  recipiente con cubos girados hasta 5°, 10/10 igual que antes. Con cubos girados ~15°
  los agarres quedan al límite con los dos cierres y con el rápido fallaron más: si en
  el brazo real se resbalan cubos, apagalo.
- **Apriete al agarrar** (`apriete_grados`, 5°): al detectar el contacto, la garra queda
  en la posición de la primera lectura de contacto menos esos grados, siempre igual.
  Antes quedaba donde estaba al confirmar el contacto, y el apriete dependía de en qué
  lectura de corriente (cada 40 ms) caía. Gemelo: 15/15 escenas, también los cubos
  girados ~14° que antes se resbalaban. `0` vuelve a lo de antes. En el brazo real hay
  que mirar con «Medir corriente» que M6 sosteniendo quede bien por debajo de ~1,1 A:
  si no, la protección de la garra afloja de a 1,5° (`GRIPPER_HOLD_RELIEF`); en ese
  caso, bajá `apriete_grados`.
- **Espera después de agarrar**: el firmware avisa `GRIPPER_SETTLED` cuando la corriente
  de M6 queda estable (dos lecturas seguidas, cada 100 ms, a 6 o menos de diferencia,
  pasados 200 ms; como máximo 2 s), y recién ahí se levanta el cubo. Antes eran 0,8 s
  fijos o, con «Medir corriente», hasta que M6 bajara de 300 mA, que con el apriete no
  pasaría nunca. Gemelo: se asienta a los 200–300 ms y Agarre bajó ~0,5 s.

### Demo: el cubo de un lado a otro, sin recipiente

**🔁 DEMO: llevar el cubo de un lado a otro** (pestaña **Tarea automática**)
saca el recipiente de la tarea y deja el brazo en un loop: pose inicial → agarra
el cubo → lo lleva al otro punto de la mesa → lo baja hasta casi apoyarlo y lo
suelta → vuelve a Inicio → repite. Los dos puntos son (−60, −210) y (130, −210) mm
del marco del modelo (`DEMO_SPOTS` en `brazoia/demo.py`), a 22 y 25 cm de la base y
dentro de la zona calibrada de las cámaras. En cada vuelta las cámaras buscan el
cubo y lo llevan al punto más lejano: si alguien lo mueve, la vuelta siguiente lo
agarra donde esté.

Cada vuelta usa lo mismo que **HACER LA TAREA SOLA** (la calibración de cámaras,
el planificador, el ejecutor y las reglas del firmware), sin confirmar cada
etapa. **Vueltas: 0** sigue hasta **Detener** o **EMERGENCIA**. Entre vueltas
espera 2 s en Inicio y mira dónde quedó el cubo: si quedó a más de 40 mm del
destino lo anota y sigue, y si no lo ve, se frena. Si una etapa se bloquea, la
demo se frena y el brazo queda donde estaba. Cada vuelta se registra en
`datos/demo/` (en el gemelo, `datos/gemelo/demo/`) con la foto de Inicio, la final
y las de los bloqueos. En el gemelo, la demo saca el recipiente de la mesa: para
volver a la tarea normal, **Nueva escena**.

## Simulación física en MuJoCo

`lanzadores/SIMULACION_MUJOCO.bat` ejecuta la tarea completa en un entorno virtual con
física: el brazo, la mesa, el cubo y el recipiente. Un experto por cinemática
inversa calcula las doce poses a partir de la posición del cubo y del recipiente,
sin demostraciones. No abre puertos serie ni usa las cámaras reales. Ver
`docs/SIMULACION_MUJOCO.md`.

`lanzadores/VIDEO_DEMO.bat` graba un video para exponer: la misma simulación hace la
tarea seis veces con el cubo y el recipiente en lugares distintos, con la etapa en
pantalla y un contador. Ver `docs/VIDEO_DEMO.md`.

## Calibración modelo-brazo

`lanzadores/CALIBRACION.bat`, o **Calibración · toques de mesa** en la pestaña
**Cámaras y calibración**, abre una ventana de solo lectura. Captura toques de la garra
sobre la mesa y la garra cerrada, y estima la altura real de la mesa y los ceros
de M2–M4 y M6. No modifica `calibracion.json`. Ver `docs/CALIBRACION.md`.

## Modo geométrico: cámaras calibradas

Alternativa al modelo aprendido para agarrar el cubo en cualquier lugar de la
zona calibrada. Con la foto de Inicio calcula dónde están el cubo y el
recipiente, y la cinemática inversa del modelo 3D calcula las doce poses, con
descenso y retiro verticales y un empuje que deja el cubo apoyado contra el dedo
fijo antes de cerrar. Se elige en la pestaña **Tarea automática**, paso a paso
o con **HACER LA TAREA SOLA**. Necesita una calibración hecha con
**Calibración · cámaras**; `lanzadores/SIMULACION_GEOMETRICA.bat` lo prueba con cámaras
virtuales. Ver `docs/CALIBRACION_CAMARAS.md`.

## Hardware esperado

- ESP32 preferido en `COM4`, 115200 baud. Si cambia el número y hay un solo
  adaptador USB serie, la interfaz lo selecciona automáticamente.
- Firmware compatible con `posall`, `arm torque on/off`, `torque on/off`, `pair`,
  `gripper open/close/stop`. Opcional: `caps` y `sync` (etapa 2).
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


## Ideas para una v2

`docs/IDEAS_V2.md` junta lo que se podría agregar o mejorar en una próxima versión
(cámaras, motores y alimentación, garra, montaje, seguridad y aprendizaje), con el
motivo y el esfuerzo de cada idea.

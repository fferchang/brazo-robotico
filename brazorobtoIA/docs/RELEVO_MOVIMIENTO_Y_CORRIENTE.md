# Relevo interno de desarrollo del brazo

## Punto de reanudación

**Leé esta sección primero.** Lo que sigue después es el historial de
decisiones y pruebas del 6/11. Sus propuestas y comandos de carga describen
el contexto de entonces. La próxima tarea vigente está al final de esta
sección.

El repositorio Git es la carpeta `brazo robotico` y la raíz de la app es
`brazorobtoIA`. La exposición del 14/11 corrió la rama
`claude/camera-esp32-testing-wgp4qh` (`0793b5c`) con el firmware cargado el 6/11.
La etapa 2 (orden `sync`, 19/11) está en `claude/wizardly-johnson-jazaal`, sobre
`0793b5c`: ver [Orden sync](#orden-sync-cinco-motores-por-mensaje-1911).

### Estado por entorno

| Entorno | Estado verificado |
|---|---|
| ESP32 real (COM11) | La última carga autorizada registrada fue el 6/11. Es la versión con `power telem` y protecciones de M6 de `313e3e6`. La orden `waypoint` pertenece al candidato posterior. Al conectar, la PC fija el umbral de contacto en 60. |
| App normal y gemelo de referencia | La planificación geométrica usa cadenas de 6° y caja de hasta 12° para los pasos anticipados. Guardado se prepara al terminar Soltado. Cada orden clásica lleva hasta dos motores. Los grupos clásicos pueden solaparse. |
| Firmware con `sync` (19/11) | `caps` y `sync <ms> <id> <grados> ...`: hasta cinco motores M1–M5 en un `SyncWritePosEx`. Compila en la PC con stubs (`firmware/prueba_pc/`) y pasa sus pruebas; réplica en el gemelo. **No compilado con Arduino CLI ni cargado en el ESP32.** |
| Firmware candidato y réplica virtual | `waypoint` agrupa los destinos de un punto en un mensaje. Ejecuta grupos de uno o dos motores de forma secuencial, espera quietud, excluye M6 y permite cancelar pendientes. Compiló para ESP32 con core 3.3.2 y SCServo. Se validó en el gemelo y sigue sin cargar en el equipo. |
| Ensayos locales | Los pasos selectivos de 10° y Guardado anticipado viven en un banco de prueba del gemelo. La variante logró 147 órdenes frente a 159 con 8° en la misma escena, y 0,15–0,26 s de pausa tras soltar en dos escenas. La app del gemelo ofrece una opción experimental para la salida y el regreso por poses directas. |
| Última verificación integral | 271 tests de Python pasaron. La demo virtual con el protocolo candidato completó una vuelta. En Traslado 2 bajó de 12 a 6 mensajes de movimiento desde la PC, pero pasó de 1,85 a 5,26 s por la espera entre grupos. Hubo 24 latidos adicionales y 12 grupos internos. |

### Límites y forma de trabajar

- Pedí confirmación antes de empezar una tarea nueva. Al terminar, decí qué
  harías a continuación y esperá el OK. El usuario prefiere español
  rioplatense.
- Mover el brazo y cargar el ESP32 requieren autorización explícita.
  Para cargar, el usuario cierra antes la app que ocupa COM11. El gemelo
  usa exclusivamente `SIMULADO`.
- Conservá los límites articulares, controles 2D y 3D, acuses, parada y
  protecciones de M6. La fuente de 12 V y 5 A es compartida. Diferenciá
  motores **por mensaje** de motores **en movimiento a la vez**.
- La telemetría de corriente real lee los servos en secuencia cada 100 ms.
  Con umbral 60 el pico de M6 fue 578 mA y el mínimo durante Agarre 10,7 V.
  Antes se registraron 994–1020 mA y mínimos de 6,9–8,2 V. La espera tras
  Agarre evitó que el pico se superpusiera con Traslado 1. Estos registros
  no caracterizan los picos rápidos ni validan un presupuesto de cinco
  motores.

### Mapa del código y del protocolo

| Archivo | Dónde intervenir |
|---|---|
| [app.py](../app.py) | `run_demo_loop`, `_drive_stages` y `_fresh_pose` coordinan la demo y sus registros. `stop_automatic` y `emergency` gestionan la parada. `--puntos-firmware` solo se habilita con `--gemelo`. |
| [execution.py](../brazoia/execution.py) | `SupervisedExecution` prepara los grupos y revisa las cajas de movimiento. `_chain` y `_blend` gestionan los puntos de paso. `MotionExecutor._send_waypoint` espera acuses por ID y cancela al detenerse. `_wait_grip_settle` conserva la espera eléctrica tras Agarre. |
| [planner.py](../brazoia/planner.py), [politica_geometrica.py](../brazoia/politica_geometrica.py), [mujoco_sim.py](../brazoia/mujoco_sim.py), [demo.py](../brazoia/demo.py) | Plan de grupos y geometría 2D, revisión 3D, física y movimientos de la tarea y de la demo. |
| [robot.py](../brazoia/robot.py), [runlog.py](../brazoia/runlog.py) | Puerto serie, correlación de acuses de `waypoint`, CSV de corriente y eventos `completed` con `timings_s` por etapa. |
| [firmware_secuencias.ino](../firmware/firmware_secuencias.ino), [gemelo.py](../brazoia/gemelo.py) | Firmware candidato y réplica virtual. Un solo punto activo, espera de quietud, chequeo de grupos, latido y cancelación. |

Formato de prueba: `waypoint ID N MOTOR GRADOS ... [N MOTOR GRADOS ...]`.
Cada N es 1 o 2. Un motor aparece una sola vez por punto, dentro de M1–M5.
`WAYPOINT_ACCEPTED` confirma recepción y `WAYPOINT_DONE` confirma quietud al
final. El ejecutor renueva con `waypoint keepalive ID` cada 200 ms y el
firmware invalida el punto a los 750 ms sin latido. `waypoint cancel ID`
descarta los grupos pendientes. Un grupo ya despachado puede terminar su
movimiento. El detalle de tiempos, fallos y pruebas está en
[Firmware de prueba](#firmware-de-prueba-un-mensaje-por-punto-con-grupos-de-dos-611).

### Arranque y verificación en otra máquina

Desde `brazorobtoIA`, recreá el entorno desde `requirements.txt`. La
última verificación usó Python 3.14.3. Ejemplo en PowerShell:

```powershell
py -3.14 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m unittest discover -s tests -t .
.\.venv\Scripts\python.exe -m unittest tests.test_puntos_firmware -v
```

Para repetir una vuelta, abrir la app con una de estas líneas, conectar
cámaras virtuales y el puerto `SIMULADO`, y elegir **Vueltas: 1 → DEMO**.

```powershell
.\.venv\Scripts\python.exe app.py --gemelo
.\.venv\Scripts\python.exe app.py --gemelo --puntos-firmware
```

La segunda línea cambia solo Traslado 2 si todos sus segmentos admiten el
protocolo. Para comparar, revisar `metadata.json`, usar la misma escena y
ejecutar las variantes por separado. Para ensayos deterministas, usar los
bancos locales indicados abajo. Ver también [Gemelo digital](GEMELO_DIGITAL.md).

Para **compilar**, usar Arduino CLI con core `esp32:esp32` 3.3.2, biblioteca
SCServo y FQBN `esp32:esp32:esp32`. El sketch debe estar en una carpeta
`firmware_secuencias` con el archivo del mismo nombre. Ejemplo en Windows
con Arduino IDE instalado y compilación en una carpeta temporal:

```powershell
$cli = Join-Path $env:ProgramFiles 'Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
$env:ARDUINO_DIRECTORIES_DATA = Join-Path $env:LOCALAPPDATA 'Arduino15'
$root = Join-Path $env:TEMP ('brazo-build-' + [guid]::NewGuid().ToString('N'))
$sketch = Join-Path $root 'firmware_secuencias'
New-Item -ItemType Directory -Path $sketch -Force | Out-Null
Copy-Item -LiteralPath 'firmware/firmware_secuencias.ino' -Destination $sketch
$libraries = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Arduino/libraries'
& $cli compile --fqbn esp32:esp32:esp32 --build-path (Join-Path $root 'build') --libraries $libraries $sketch
```

### Qué trasladar

Antes de mover el trabajo a otro checkout, revisá `git status --short` y
`git diff HEAD` en origen y destino. Si exportás un parche de los cambios
rastreados, aplicalo desde la raíz Git y verificá primero con
`git apply --check`. Un parche de `git diff HEAD` no incluye archivos sin
seguimiento ni ignorados.

- **Cambios rastreados de desarrollo:** `app.py`,
  `brazoia/{execution,gemelo,mujoco_sim,politica_geometrica,robot,runlog}.py`,
  `firmware/firmware_secuencias.ino`,
  `tests/{test_politica_geometrica,test_registro_ejecucion}.py` y las guías
  internas `docs/{GEMELO_DIGITAL,DIAGNOSTICO_GARRA}.md`. `README.md` ya
  tiene cambios locales previos. Revisalos aparte si necesitás trasladarlos.
- **Sin seguimiento:** `brazoia/demo.py`,
  `tests/{test_demo,test_medicion_corriente,test_movimiento_fluido,test_puntos_firmware}.py`,
  este relevo y, para el backlog, `docs/IDEAS_V2.md`.
- **Ignorados en `datos/gemelo/`:** copiá juntos
  `ensayo_cola/prototipo.py` y `ensayo_guardado/prototipo.py` para reproducir
  esos ensayos. Guardado importa `clone` del banco de cola. Comandos desde
  `brazorobtoIA`:

  ```powershell
  .\.venv\Scripts\python.exe -m datos.gemelo.ensayo_cola.prototipo
  .\.venv\Scripts\python.exe -m datos.gemelo.ensayo_guardado.prototipo selectivo10_temprano 4
  ```

  La calibración virtual `datos/gemelo/camaras_virtuales.json` se regenera
  si hace falta. Las calibraciones reales `datos/calibracion/camaras.json`
  y `assets/calibracion.json` están rastreadas.
- **Registros:** `datos/corriente/` y `datos/demo/` están sin seguimiento.
  Para el diagnóstico eléctrico, seleccioná los CSV `191436`, `192608`,
  `192716` y `194625` del 6/11, y el `metadata.json` de la vuelta real
  `prueba_20261106_194645_306768`. Para el protocolo, seleccioná
  `metadata.json` y `ensayo_puntos_firmware.json` de las vueltas virtuales
  `prueba_20261106_214128_286840` y `prueba_20261106_214629_792222`.
  El historial identifica los demás registros. Acordá por separado el
  traslado de imágenes de las cámaras reales.
- **Cambios de otra sesión:** conservar separados `video_demo.py`,
  `lanzadores/VIDEO_DEMO.bat`, `docs/VIDEO_DEMO.md`,
  `tests/test_video_demo.py` y la línea local de `.gitignore` para
  `datos/videos/`. Recreá `.venv` y dejá fuera `__pycache__`.

### Próxima tarea vigente

**19/11, pendiente del usuario:** compilar con Arduino CLI y cargar el firmware
con `sync` (cerrar antes la app que ocupa el COM), y correr una vuelta de DEMO
en el brazo real con «Medir corriente». Comparar con la vuelta de referencia del
6/11 (38,4 s, pico de M6 578 mA, mínimo 10,0 V) y observar las pausas cortas.
Para volver a `pair` sin recargar: `app.py --sin-sync`. Después siguen, a
confirmar: anticipar la preparación de Guardado (5,5–7 s en el gemelo), el
supervisor de tensión y corriente que baja la velocidad de todos los motores
por igual, y subir `SYNC_MAX_SPEED`.

**Anterior (6/11), reemplazada por la orden `sync`:** probar primero en el gemelo
un límite dinámico. Usar el mismo Traslado 2 y comparar contra el límite
fijo. Partir de dos motores activos y ensayar sumar de a uno, con un
máximo de cinco solo en el gemelo. Inyectar picos de corriente, caídas
de tensión y lecturas vencidas para verificar que no se incorporen
grupos en esas condiciones. Probar la prioridad de M6, la cancelación
y medir órdenes, frenadas, tiempo y resultado. Los 2 A que propuso el
usuario son un parámetro experimental. El resultado virtual sirve
para verificar la lógica. La capacidad eléctrica requiere medición
en el equipo y una autorización aparte.

### Orden sync: cinco motores por mensaje (19/11)

Objetivo del usuario: más rápido y sin micromovimientos, partiendo del modo
geométrico automático de la exposición. Causa medida el 6/11: cada servo
terminaba su paso de 6° y frenaba antes de recibir el siguiente, porque cada
punto necesitaba hasta tres órdenes de dos motores con acuse y 65 ms entre ellas.

Cambios:
- **Firmware:** `caps` responde `FIRMWARE_CAPS sync=1 sync_max_motors=5
  sync_max_speed=450`. `sync <ms> <id> <grados> ...` acepta de 1 a 5 motores de
  M1–M5, cada uno una vez, y los manda en un `SyncWritePosEx`. Rechaza la orden
  entera (`SYNC_REJECTED reason=formato|motores|fuera_de_rango|cooldown|
  servo_sin_respuesta|trayectoria_insegura`) sin ajustes. Revisa piso,
  autocolisión y regla de M4 en 20 puntos de la recta desde la pose medida, como
  `pair`. Comparte el enfriamiento de 50 ms. Velocidad del que más recorre:
  `SYNC_MAX_SPEED` (= `MOVE_SPEED`, 450); los demás, proporcionales, con piso de
  20 (0 sería la máxima en el STS3215). Con `ms` > 0 baja la velocidad para
  tardar ese tiempo. Con un `waypoint` activo responde `WAYPOINT_BUSY`.
- **Gemelo:** réplica de `caps` y `sync`. `DigitalTwin.sync_firmware = False`
  se comporta como el firmware del 6/11.
- **PC:** `RobotSerial` manda `caps` al conectar y marca `sync_supported`.
  `MotionExecutor` manda un `sync` por punto «junto» (o de un solo grupo) si
  cada motor aparece una vez; si no, o sin `sync` en el firmware, todo sigue
  como antes. `SupervisedExecution(sync_orders=True)` marca «juntos» también los
  pasos de cadena de un solo grupo (ya revisados) y manda el punto siguiente con
  el motor más atrasado a un paso del destino (`SYNC_BLEND_FRACTION = 1.0`). La
  caja de ese anticipo sigue limitada a 12° y revisada en 2D y 3D. En los puntos
  de paso la telemetría se mira cada 10 ms; la quietud cuenta lecturas nuevas.
- `--sin-sync` en `app.py` y `gemelo_digital.py` fuerza `pair`.

Resultados en el gemelo (una vuelta por fila, DEMO, `App(gemelo=True)` oculta).
«Frenadas»: intervalos entre órdenes de un motor (destinos a ≥0,5° y pico del
perfil ≥5°/s) en los que el perfil llegó a 0 antes de la orden siguiente.

| Semilla | Modo | Paso / anticipo | Vuelta | Dentro del ejecutor | Frenadas | Preparar Guardado |
|---|---|---|---:|---:|---:|---:|
| 4 | pair (exposición) | 6° / 0,5 | 44,06 s | 33,56 s | 303/308 | — |
| 4 | sync | 6° / 0,5 | 35,51 s | 25,52 s | 178/298 | — |
| 4 | sync | 6° / 1,0 | 34,01 s | 24,33 s | 127/296 | — |
| 4 | sync | 6° / 1,0 (repetida) | 35,91 s | 24,64 s | 127/295 | 7,15 s |
| 4 | sync | 4° / 1,0 | 40,03 s | 23,82 s | 76/402 | — |
| 1 | pair (exposición) | 6° / 0,5 | 42,72 s | 32,75 s | 293/301 | 6,19 s |
| 1 | sync | 6° / 1,0 | 33,12 s | 23,89 s | 124/286 | 5,56 s |
| 1 | sync | 4° / 1,0 | 39,82 s | 23,22 s | 68/398 | 11,46 s |

- Todas `result=success`. Se eligió 6° / 1,0: con 4° frena menos, pero preparar
  Guardado casi se duplica y la vuelta queda más lenta. La preparación es casi
  toda revisión 3D (0,37 ms por pose, hasta 32 esquinas por caja y ~12 muestras
  de la garra cerrándose en camino).
- Tarea con recipiente (`gemelo_digital.py --episodios 3 --semilla 4`): 3/3 con
  `--sin-sync` (46,9 / 44,1 / 46,9 s) y 3/3 con `sync` (39,0 / 36,5 / 38,4 s),
  sin avisos de llegada con desvío.
- En el gemelo el cálculo compite con la física por el GIL: las pausas de
  preparación son mayores que en la PC del usuario (3,1–3,5 s el 6/11).
- El perfil del gemelo supone que el STS3215 sigue con su velocidad al recibir
  un destino nuevo en movimiento. **No está verificado en el servo real.**
- Pruebas: `tests/test_orden_sync.py` (18) y la suite completa (285, salvo las
  dos que necesitan `rerun`). No se movió el brazo real ni se cargó firmware.

### Cronómetro y métricas de ejecución

La pestaña **Tarea automática** muestra el tiempo del último intento.
Cada vuelta de DEMO y cada intento autónomo con recipiente agrega
`attempt_metrics` al `metadata.json` del registro correspondiente:
`datos/demo/`, `datos/ejecuciones/` o sus equivalentes bajo `datos/gemelo/`.

- `duration_seconds` mide desde el inicio del intento hasta su resultado.
  En recipiente se guarda al terminar la ejecución y la verificación
  automática, antes de esperar la evaluación humana. Una reanudación
  genera un intento nuevo en el mismo registro.
- `transfer_seconds` estima el intervalo desde la confirmación de
  Agarre hasta el final de Soltado, a partir de las marcas del ejecutor.
  La estimación descuenta la espera posterior al agarre de la marca
  de cierre de esa etapa. Queda vacía si el intento no contiene ambos hitos.
- `stages` mide de propuesta a confirmación, bloqueo o interrupción
  de cada etapa. El detalle anterior de órdenes, llegada y quietud
  sigue en los eventos `completed.timings_s`.
- `result` guarda el resultado observado de cada intento. En el brazo
  real, la verificación visual es provisional. El resultado humano
  conserva su campo en el registro de ejecución.

### Ensayo de pose directa y rollback (11/11)

Copia previa: `datos/gemelo/ensayo_pose_20261111_174518/rollback.zip`.
Contiene 202 archivos rastreados o locales visibles para Git y los
prototipos ignorados seleccionados. Se verificó el SHA-256 de cada
entrada contra el manifiesto interno. Para recuperar el estado, extraer
primero en una carpeta aparte y usar las rutas relativas a la raíz Git
y el commit del manifiesto para revisar y restaurar los archivos
necesarios. El ZIP queda en la carpeta local ignorada del gemelo.

El banco `comparar.py` en la misma carpeta ensayó solo Aproximación 1.
Busca la menor cantidad de puntos que pasa el planificador 2D, la caja
de movimiento y un muestreo 3D adicional de su interior. Una única pose
objetivo pasó para la escena de semilla 4. El ejecutor clásico la mandó
en tres grupos, sin cambiar el firmware. Es un ensayo virtual de una
escena por variante. Los chequeos muestreados y ese resultado no
habilitan el salto en el brazo real.

| Variante | Órdenes en Aproximación 1 | Tiempo de esa etapa | Vuelta |
|---|---:|---:|---:|
| Referencia 6° | 48 | 6,437 s | 42,176 s |
| Pose directa | 3 | 2,721 s | 38,281 s |

Ambas vueltas terminaron con el cubo a 12 mm del destino. En los
intervalos entre destinos sucesivos del mismo motor, Aproximación 1
registró 75 frenadas en referencia. La pose directa no generó intervalos
intermedios de ese tipo. Registros bajo `datos/gemelo/demo/` en
`prueba_20261111_174623_015864/` y `prueba_20261111_174710_980109/`.
El banco corre desde `brazorobtoIA` con
`.venv/Scripts/python.exe -m datos.gemelo.ensayo_pose_20261111_174518.comparar directo 4`.
El modo `referencia` repite la base. Próximo paso sujeto a confirmación:
probar más escenas del gemelo y evaluar el regreso después de la
retirada vertical.

### Ensayo ampliado de salida y regreso directos (11/11)

Con el respaldo anterior conservado, `ampliar.py` en la misma carpeta
probó ambas direcciones de la demo (semillas 1 y 4). En Guardado se
conservaron los puntos de retirada y el cierre de la garra en el lugar
previsto. Solo se simplificó el regreso largo después de la retirada.
Cada escena aceptó una pose objetivo de salida y otra de regreso con
los chequeos 2D, las cajas 3D y el muestreo adicional. Cada pose larga
se ejecutó en tres grupos con el protocolo clásico.

| Semilla | Órdenes de salida | Órdenes de Guardado completo | Órdenes por vuelta | Vuelta |
|---|---:|---:|---:|---:|
| 1 | 48 → 3 | 61 → 16 | 189 → 99 | 40,08 → 32,74 s |
| 4 | 48 → 3 | 66 → 15 | 194 → 98 | 42,18 → 33,20 s |

Las cuatro vueltas de la comparación completaron la demo. El cubo
quedó a 5 mm con poses directas en semilla 1 y a 12 mm en semilla 4.
La pausa antes de Guardado quedó en 1,23–1,27 s con el plan directo.
Se midió una vuelta por variante y escena. El ensayo vive en el banco
del gemelo y el código de ejecución y el firmware coinciden con el ZIP.

Registros bajo `datos/gemelo/demo/`, con prefijo `prueba_20261111_`:
semilla 1 base `174942_437612`, directa `175028_068668`. Semilla 4
base `174623_015864`, directa `175106_337910`. Reproducción desde
`brazorobtoIA`: `.venv/Scripts/python.exe -m
datos.gemelo.ensayo_pose_20261111_174518.ampliar directo 4`.
Próximo paso sujeto a confirmación: integrar una opción experimental
en la app del gemelo con regreso al plan habitual si el salto falla.

### Opción de poses directas en la app del gemelo (11/11)

Abrí la app con `.venv/Scripts/python.exe app.py --gemelo`, conectá las
cámaras virtuales y el puerto `SIMULADO`. En **Tarea automática**, marcá
**Poses directas (experimental)** y arrancá **DEMO**. La casilla empieza
desactivada y se lee al iniciar la serie de vueltas.

`SupervisedExecution` intenta una pose directa al salir de Inicio,
para llegar a Traslado 3 sin pasar por Traslado 2 y para el último
tramo de Guardado. Si no se aprueba el salto, mantiene Traslado 2
con su propia opción de pose directa. En Traslado 2
exige que el plan original tenga un solo destino y que M6 quede
sosteniendo, sin una acción de garra en ese tramo. Conserva la retirada vertical y
confirma su llegada antes de empezar el regreso largo. El salto pasa
el planificador, las comprobaciones de trayectoria y el muestreo 3D de
la caja. Si se rechaza, ese tramo sigue el plan habitual. La app ofrece
la casilla solo para la demo virtual. El registro guarda la selección
en `configuration.poses_directas`, los grupos efectivamente ordenados
y el cronómetro de cada vuelta. Sigue disponible el ZIP de rollback
anterior a los ensayos.

Pasaron 37 pruebas de movimiento, demo y tarea autónoma, incluida una
prueba que fuerza el rechazo del salto y compara el respaldo con el plan
habitual. La app completó una vuelta en la versión final con semilla 1,
3 órdenes de salida y 16 en Guardado, en 32,78 s. El registro
`datos/gemelo/demo/prueba_20261111_175625_000921/metadata.json` confirma
que el último tramo de retirada exige quietud antes de iniciar el regreso.

### Comparación de la casilla en tres escenas del gemelo (11/11)

Se corrió una vuelta por modo y semilla (1, 4 y 7) desde la app con
puerto `SIMULADO`, en procesos separados. En semilla 4 se invirtió el
orden de prueba. Los tiempos salen de `attempt_metrics`; las órdenes
son los grupos registrados en las propuestas.

| Semilla | Vuelta normal → directa | Órdenes normal → directa | Distancia final en ambos |
|---|---:|---:|---:|
| 1 | 45,90 → 34,39 s | 189 → 99 | 5 mm |
| 4 | 48,88 → 34,02 s | 194 → 98 | 12 mm |
| 7 | 46,96 → 35,80 s | 188 → 95 | 12 mm |

Las seis vueltas terminaron con éxito. El promedio bajó de 47,25 a
34,74 s (12,51 s, 26,5%) y de 190,3 a 97,3 órdenes (48,9%). El tiempo
estimado de Agarre a Soltado pasó de 10,68 a 10,46 s. La casilla
simplifica Aproximación 1 y Guardado. El tramo con el cubo se midió
como referencia para una próxima optimización. La muestra tiene una
sola vuelta por modo en cada escena, con tiempos de pared del gemelo.

Registros bajo `datos/gemelo/demo/`, prefijo `prueba_20261111_`, en
orden normal/directa por semilla: 1, `181537_272354` y `181628_567961`;
4, `181748_056797` y `181708_342384`; 7, `181842_260032` y
`181934_902681`. Próximo paso sujeto a confirmación: evaluar si
Traslado 2 admite menos puntos conservando los controles y el agarre.

### Ensayo de pose directa en Traslado 2 (11/11)

Se conservó activada la casilla para salida y Guardado. Solo el banco
temporal `datos/gemelo/ensayo_pose_20261111_174518/traslado2.py` intentó
simplificar Traslado 2. Pasó el planificador 2D, las cajas y el muestreo
3D del brazo para las tres escenas, con una pose objetivo y los grupos
clásicos. M6 permaneció en estado `holding` al terminar el tramo. El
resultado del cubo se verificó al final de la vuelta, sin un chequeo
geométrico independiente de su volumen durante todo el traslado.

| Semilla | Órdenes de Traslado 2 | Ejecución | Incluye preparar Traslado 2 | Vuelta completa |
|---|---:|---:|---:|---:|
| 1 | 11 → 3 | 1,49 → 1,05 s | 1,55 → 1,30 s | 34,39 → 35,94 s |
| 4 | 12 → 2 | 2,03 → 1,07 s | 2,10 → 1,10 s | 34,02 → 32,66 s |
| 7 | 10 → 2 | 1,74 → 1,14 s | 1,87 → 1,18 s | 35,80 → 32,90 s |

Las tres vueltas completaron la demo, con el cubo a 5, 11 y 10 mm del
destino. En Traslado 2 desaparecieron los intervalos entre destinos
sucesivos del mismo motor. Las referencias son las vueltas de la
comparación anterior, no repeticiones intercaladas. Los tiempos de vuelta
variaron también por la preparación de otras etapas.

Registros nuevos bajo `datos/gemelo/demo/` con prefijo `prueba_20261111_`:
`182541_453611`, `182623_037504` y `182701_523719`, respectivamente.
Desde `brazorobtoIA`: `.venv/Scripts/python.exe -m
datos.gemelo.ensayo_pose_20261111_174518.traslado2 directo 4`.
Traslado 2 quedó incorporado a la casilla experimental de la demo virtual
con el plan habitual como respaldo. Pasaron 37 pruebas y se verificó una
vuelta de la app con semilla 4: una pose y dos órdenes en Traslado 2,
M6 en estado `holding` al completar, 1,07 s para el tramo y 32,36 s para
la vuelta. Registro `datos/gemelo/demo/prueba_20261111_183707_659386/metadata.json`.
El próximo ensayo, sujeto a confirmación, es revisar la trayectoria con
el volumen del cubo agarrado, incluida su distancia a la mesa.

### Ensayo de salto de Traslado 2 (11/11)

En dos escenas de la demo con poses directas, se comparó el recorrido
habitual con un solo destino desde el fin de Traslado 1 hasta el objetivo
de Traslado 3. El banco temporal `salto_traslado2.py` usa la caja 2D/3D
y el muestreo interior. Durante la revisión proyecta el cubo como sólido
rígido con la pose relativa a la pinza medida en el gemelo y exige al
menos 5 mm de separación en cada muestra. En ejecución mide el punto
más bajo de la caja física de MuJoCo en cada tick durante Traslados 2 y
3. En ambas escenas el salto se aprobó con una pose y tres órdenes.

| Semilla | Órdenes del tramo | Fin de Traslado 1 a fin de Traslado 3 (s) | Agarre a Soltado (s) | Vuelta (s) | Despeje físico mínimo (mm) |
|---|---:|---:|---:|---:|---:|
| 1 | 18 → 3 | 3,50 → 1,92 | 10,26 → 8,85 | 35,74 → 34,46 | 72,31 → 73,19 |
| 4 | 16 → 3 | 3,55 → 2,15 | 10,29 → 8,95 | 36,96 → 34,96 | 73,60 → 73,63 |

Las cuatro vueltas terminaron bien. La variante directa dejó el cubo a
6 mm del destino en ambas escenas y mantuvo M6 en `holding` al final
de Traslado 3. El despeje medido pertenece a estas escenas del gemelo.
El ensayo no modificó la app ni el firmware.

El banco y los archivos `salto_traslado2_*.json` están en
`datos/gemelo/ensayo_pose_20261111_174518/`. Desde `brazorobtoIA`:
`.venv/Scripts/python.exe -m datos.gemelo.ensayo_pose_20261111_174518.salto_traslado2 salto 4`.
Los JSON enlazan por nombre los registros de la demo. El salto quedó
integrado en la casilla experimental de la demo virtual. Se intenta
con M6 en `holding` y ambas etapas libres de acciones de garra y puntos
previos. Si faltan las lecturas o la revisión 2D/3D o del cubo falla,
la app conserva Traslado 2. El chequeo de despeje usa la pose del cubo
relativa a la pinza, capturada de la física virtual, y exige 5 mm
en todas las poses muestreadas. Los bancos históricos de Traslado 2
desactivan el salto integrado para conservar sus variantes originales.

Pasaron 38 pruebas de demo, movimiento y tarea autónoma. Una prueba
fuerza el rechazo por despeje y verifica que Traslado 2 se conserve.
La app completó una vuelta con la opción activa y semilla 4. El salto
usó tres órdenes, mantuvo un despeje físico mínimo de 73,69 mm y el
cronómetro registró 9,05 s de agarre a soltado y 36,06 s de vuelta.
Registro `datos/gemelo/demo/prueba_20261111_184951_569675/metadata.json`.

### Posiciones iniciales variadas y respaldo (11/11)

Cinco vueltas de la app con **Poses directas (experimental)** activada,
en procesos separados con puerto `SIMULADO`. Se fijó la pose inicial
reemplazando `twin_demo_placement` en memoria. Se midió por tick la
distancia del punto más bajo de la caja física del cubo al plano de
mesa durante Traslados 2 y 3.

| X, Y (mm) | Giro | Salto | Vuelta | Llegada | Despeje mínimo durante el salto |
|---|---:|---|---:|---|---:|
| −95, −230 | +10° | Sí | 34,65 s | 4 mm | 73,25 mm |
| +165, −230 | −10° | Sí | 34,99 s | 3 mm | 74,97 mm |
| −35, −180 | −15° | Respaldo | 26,50 s hasta bloqueo | Bloqueada en Soltado | — |
| +100, −180 | +15° | Sí | 34,86 s | 4 mm | 73,68 mm |
| +20, −250 | 0° | Sí | 33,64 s | 7 mm | 74,09 mm |

En las cuatro vueltas con salto se emitieron tres órdenes en ese tramo.
El caso (−35, −180, −15°) se repitió con diagnóstico y la casilla activa:
la caja 2D/3D del brazo pasó y el chequeo del cubo rechazó el salto.
La app conservó Traslado 2 y volvió a bloquearse en Soltado. Durante
Traslado 3, el centro del cubo quedó a unos 100 mm de la pinza en la
muestra de menor despeje, aunque el estado informado era `holding`.
Con la casilla desactivada, el cubo también se separó de la pinza y
la vuelta terminó `fuera_de_lugar`. En los tres intentos de esa
posición, la pérdida física de agarre se observó en Traslado 3.
El diagnóstico dejó de medir antes de cerrar la app.

Registros bajo `datos/gemelo/demo/` con prefijo `prueba_20261111_`,
en el orden de la tabla: `185339_905697`, `185420_300499`,
`185501_033895`, `185533_640334` y `185614_458973`. Cada carpeta tiene
`ensayo_posicion.json`. Las repeticiones con y sin casilla quedaron
en `185724_919986` y `185757_156965`, con `diagnostico_posicion.json`.
La app y el firmware se conservaron. Próximo paso sujeto a confirmación:
evaluar cómo detectar que el cubo se soltó físicamente aun cuando M6
sigue informando `holding`, antes de continuar hasta Soltado.

### Ensayo de detección de pérdida física del cubo (11/11)

Se corrieron tres vueltas de la app del gemelo con poses directas.
Un monitor temporal midió en cada tick el centro del cubo en
coordenadas de la pinza. Tomó como referencia la primera lectura
de Traslado 1 y observó hasta el fin de Traslado 3. Emitió una alerta
si el desplazamiento relativo alcanzó 20 mm durante al menos 80 ms
simulados. El monitor registró la alerta sin intervenir en la marcha.

| Posición X, Y, giro | Desplazamiento máximo | Alerta | Resultado de la demo |
|---|---:|---|---|
| −35 mm, −180 mm, −15° | 96,42 mm | Traslado 3, 0,40 s antes de Soltado | Bloqueada |
| −95 mm, −230 mm, +10° | 3,62 mm | Ninguna | Éxito |
| +100 mm, −180 mm, +15° | 3,63 mm | Ninguna | Éxito |

En el primer caso la garra aún informaba `holding` al dispararse la
alerta. Se trata de un umbral experimental, con una caída y dos
controles observados. Usa la pose física interna de MuJoCo.
Registros bajo `datos/gemelo/demo/`, con `ensayo_agarre.json`, en
`prueba_20261111_190118_636221`, `prueba_20261111_190148_277515` y
`prueba_20261111_190225_419358`, en el orden de la tabla.
El monitor quedó integrado en la demo del gemelo, con la casilla de
poses directas activa o desactivada. Usa el umbral de 20 mm por 80 ms
durante los traslados. Si queda una separación pendiente de confirmar,
espera antes de iniciar Soltado. Al confirmar la pérdida, señala la
parada al ejecutor, cancela los grupos pendientes del punto virtual si
hay uno activo y registra la etapa, la separación y `cubo_perdido`.
Las órdenes ya emitidas pueden completar su movimiento. El aviso del
gemelo se basa en la pose interna del cubo en MuJoCo.

La escena de caída espontánea completó bien una repetición. Para
verificar la parada de forma reproducible se forzó una separación en
Traslado 3 de una escena estable. El registro
`datos/gemelo/demo/prueba_20261111_190907_576306/metadata.json` guardó
`cubo_perdido` en el resultado y el cronómetro, con una alerta y sin
propuesta de Soltado. En la misma escena sin inyección, la vuelta
completó Soltado y terminó bien, sin alerta
(`datos/gemelo/demo/prueba_20261111_190930_367831/metadata.json`).
Pasaron las pruebas dirigidas de demo, movimiento, tarea autónoma y
protocolo de puntos. Las regresiones verifican la parada durante el
espaciado, la espera de confirmación antes de Soltado y la cancelación
de grupos pendientes.

Próximo paso sujeto a confirmación: estudiar si las cámaras pueden
detectar la pérdida del cubo en el brazo real cuando se oculta
parcialmente detrás de la garra.

### Revisión de cámaras para detectar una caída (11/11)

Se volvió a correr `detect_scene` con la configuración real sobre
las capturas de 12 tareas con recipiente de `datos/ejecuciones/`.
Las dos cámaras detectaron el cubo en las 36 parejas de fotos tomadas
al terminar Traslado 1, 2 y 3. En Agarre la frontal dio 12/12 y la
lateral 11/12. En Soltado la frontal dio 12/12 y la lateral 7/12.
En la imagen lateral de Agarre de `prueba_20261106_202824_467931`
se ve solo parte del cubo junto a la garra y el recipiente. Estos
registros son fotos al terminar etapas y no permiten medir latencia
ni oclusiones durante el movimiento real.

En el gemelo se tomó una muestra cada 100 ms simulados durante los
traslados de dos vueltas de la misma escena, una normal y otra con
el cubo desplazado 75 mm hacia abajo al comenzar Traslado 3. El
ensayo dejó continuar la vuelta virtual para obtener imágenes de la
caída. Se renderizaron ambos puntos de vista a partir de cada estado y
se corrió el mismo detector que sobre las fotos reales. La posición
esperada del cubo se proyectó usando la posición inicial relativa a
la pinza y la cámara exacta de MuJoCo. La física etiquetó el cubo
como separado cuando se alejó 20 mm de esa referencia.

| Estado físico | Parejas de cuadros | Detección en ambas cámaras | Distancia frontal al centro esperado | Distancia lateral al centro esperado |
|---|---:|---:|---:|---:|
| Junto a la pinza | 73 | 73 | 1,89–11,98 px | 1,51–3,91 px |
| Separado | 21 | 21 | 88,19–261,94 px | 66,54–97,25 px |

El detector siguió viendo el cubo tras la caída. La separación respecto
de la posición esperada distinguió ambos estados en esta muestra
virtual. Se necesita una referencia de la pinza en la imagen real y
ejemplos reales de oclusiones y caídas. Los píxeles de MuJoCo no
constituyen umbrales para el equipo. Registros `ensayo_camaras.json` en
`datos/gemelo/demo/prueba_20261111_191508_041193/` (normal) y
`prueba_20261111_191553_376059/` (caída simulada).

Próximo paso sujeto a confirmación: probar fuera de línea una
referencia visual de la pinza en las fotos reales y medir cuánto
varía la posición del cubo cuando el agarre se mantiene.

## Historial de desarrollo y pruebas

Las secciones siguientes conservan la cronología, incluidos los planes
que estaban pendientes cuando se anotaron. Para actuar hoy, seguí el
[punto de reanudación](#punto-de-reanudación).

## Roadmap original de optimización (histórico)

Objetivo del usuario: que la vuelta sea **más rápida** y **más fluida**, sin los
micromovimientos ni la «escalera» de los movimientos grandes.

### Dónde se va el tiempo hoy

Vuelta real del 6/11 a las 19:46, `datos/demo/prueba_20261106_194645_306768`: 38,4 s.
**Es de antes del encadenado de la sección 4**, que en el brazo real todavía no se
probó.

| Etapa | Tiempo | Qué pasa |
|---|---|---|
| Aproximación 1 (salir de Inicio) | 6,0 s | movimiento grande en escalera: 3 órdenes de a 2 motores, cada una espera a llegar |
| Aproximación 2 | 2,5 s | la garra abre a 90° mientras va |
| Aproximación 3 | 1,5 s | |
| Abrir sobre el cubo | 0,2 s | etapa vacía (heredada de las demos a mano) |
| Bajar con garra abierta | 2,5 s | 9 puntos de paso, 7 con motores juntos |
| **Agarre** | **3,6 s** | cierre por presión desde 90° + espera de 0,8 s a que afloje |
| Traslado 1 | 2,7 s | 4 puntos (subida vertical) |
| Traslado 2 | 2,0 s | |
| Traslado 3 | 2,5 s | |
| Soltado | 3,3 s | 5 puntos de bajada + abrir a 90° (~1,3 s) |
| **Guardado (volver)** | **8,9 s** | retiro vertical + movimiento grande en escalera + cerrar |
| Antes de empezar | 1,2 s | espera a que el brazo quede quieto y foto |

Preparar cada etapa (planificar) tarda menos de 0,1 s: no es un problema.

### Límites que no se pueden pasar
- El firmware acepta **1 o 2 motores por orden** y pide **50 ms entre órdenes**: la PC
  espera el acuse y deja 65 ms (`MOVE_SPACING_SECONDS`).
- La velocidad de cada servo la fija el firmware: `MOVE_SPEED = 450` (~40°/s) y
  `MOVE_ACC = 45`. Desde la PC no se puede subir.
- Fuente (medido): el brazo nunca pasó de ~300 mA, aun con varios motores juntos. El
  pico problemático es la garra al cerrar (578 mA con el umbral 60; el voltaje bajó a
  10,7 V).
- Todo movimiento nuevo tiene que pasar las revisiones 2D (`classify_pose`, `edge_safe`)
  y 3D (`policy.motion_problems`). No saltearlas nunca.

### Problemas a resolver, en orden de prioridad

**P1. Medir antes de cambiar (hacer primero).**
- Agregar a `ExecutionLog` (`brazoia/runlog.py`), por etapa, el tiempo en: mandar órdenes
  y esperar acuses, esperar llegada, esperar quietud, garra, y la espera tras el agarre.
- Instrumentar `MotionExecutor.execute` en `brazoia/execution.py`.
- Listo cuando una vuelta en el gemelo deja ese desglose en el `metadata.json`.

**P2. Probar en el brazo real el encadenado que ya está hecho (sección 4).**
- Aproximación 1 y Guardado ya van en pasos de 6° con todos los motores juntos.
- En el gemelo no aceleró la vuelta (~39 s), pero tendría que verse sin escalera.
- Hacerlo con el paso 4 del roadmap de abajo. Preguntarle al usuario si se ve fluido.

**P3. Las paradas al final de cada etapa.**
- Cada etapa termina con el brazo quieto (`MotionSegment.final=True`: 3 lecturas dentro
  de 1,2° + 0,08 s). Son 11 frenadas por vuelta, aunque la etapa siguiente siga en la
  misma dirección. Pasa entre Aproximación 1→2→3 y entre Traslado 1→2→3.
- Qué hacer: en las etapas **sin acción de garra** y sin foto (la demo no saca fotos
  por etapa), no frenar al final.
  - Preparar la etapa siguiente desde el destino ordenado, no desde la pose medida.
  - Pasar a ella con el mismo paso a mitad de camino (`handoff`) que ya usa `_blend`,
    si la caja de los tres puntos pasa las revisiones.
  - Al llegar a una etapa con acción de garra, frenar como hoy.
- Dónde: `App._drive_stages` (`app.py`) prepara y ejecuta etapa por etapa. Hay que dejar
  que el último segmento de una etapa no sea `final` cuando la siguiente se puede encadenar.
- Ganancia estimada: menos frenadas visibles y algunos segundos. Medir con P1.

**P4. El agarre tarda 3,6 s.**
- La garra cierra por presión de a 1,5° cada 40 ms (~37°/s) desde 90° hasta tocar el cubo,
  a ~40°: ~1,3 s. Después espera hasta 0,8 s a que afloje.
- Qué hacer (sin firmware): abrir menos para agarrar. Con 90° el dedo móvil queda 35 mm
  sobre el cubo.
  - Probar 75–80° en `GRIPPER_GRASP_OPEN` (`brazoia/mujoco_sim.py`).
  - Correr la física con error de ubicación. El script no está en el repo; rehacerlo así:
    `PickPlaceExpert(arm, open_degrees=X)`, planificar con la ubicación estimada y correr
    `run_episode` con el cubo corrido 10 mm en 8 direcciones (como la tabla de la sección 5).
  - Aceptar solo si sigue 24/24 con 10 mm de error.
- Qué hacer (con firmware, OK del usuario): cerrar rápido hasta ~10° antes del contacto
  esperado y después de a 1,5°.

**P5. Muchos puntos de paso en las bajadas, subidas y el empuje.**
- La bajada vertical tiene un punto cada 15 mm (`PickPlaceExpert._vertical`, `step=0.015`)
  y el empuje cada 4 mm (`push_step=0.004`). Cada punto son hasta 3 órdenes.
- Qué hacer:
  - probar `step` 25 mm y `push_step` 8 mm;
  - revisar en la física que el dedo no se desvíe de la recta más de ~2 mm (como el test
    `test_the_push_runs_level_toward_the_cube`);
  - revisar que la demo y la tarea sigan 12/12 en `run_episode`.
- Mantener cada salto dentro de `TOGETHER_MAX_DEGREES` (10°), para que siga yendo con los
  motores juntos.

**P6. Velocidad de los servos (necesita firmware y OK del usuario).**
- Subir `MOVE_SPEED` de 450 a ~700 en `firmware/firmware_secuencias.ino`, y lo mismo en
  `brazoia/gemelo.py` (`MOVE_SPEED`) y `brazoia/execution.py` (`M6_ANGLE_SPEED_DEG_S`).
- Medir una vuelta con «Medir corriente». Volver atrás si el brazo pasa ~1 A o el voltaje
  baja de 10 V.
- Cómo cargar: paso 5 del roadmap de abajo.

**P7. Etapas que sobran.**
- «Abrir sobre el cubo» quedó vacía.
- Traslado 2 es un punto intermedio a 13–17 cm de altura.
- Con P3 hecho, medir si conviene unir etapas. **No borrarlas de `KEYFRAME_STAGES`**: el
  modelo aprendido y las demostraciones usan las 12. Solo saltearlas en el modo
  geométrico (`GeometricPolicy` / `DemoPolicy`).

### Cómo comparar cada cambio
1. `.venv\Scripts\python.exe -m unittest discover -s tests -t .`: tiene que dar `OK`.
2. Física: demo y tarea 12/12. `run_episode` con 12 escenas de `sample_placement`
   (tarea) y 12 con el cubo cerca de `DEMO_SPOTS` y `TablePlaceExpert` (demo).
3. Gemelo, sin nada más corriendo: 3 vueltas de la demo con el script del paso 2 de
   abajo. Anotar el tiempo por vuelta (`duration_seconds`) y, con P1, el desglose.
4. Recién después, brazo real con «Medir corriente» y el OK del usuario.
5. Anotar en este archivo: qué se cambió, tiempos antes y después, y lo que dijo el usuario.

## Procedimiento histórico de la etapa 4

Este era el orden de trabajo antes de los ensayos de optimización y del
firmware candidato. Se conserva para interpretar la evidencia de esa etapa.

Todos los comandos se corren desde `C:\Users\Fer\proyectos\brazo robotico\brazorobtoIA`.
Python: `.venv\Scripts\python.exe`. Más abajo en este archivo hay detalles de todo, pero
para avanzar no hace falta leerlos.

### Paso 0 — Antes de empezar (siempre)
- Para usar COM11, confirmá que ninguna otra instancia de la app tenga abierto el
  puerto. El gemelo usa un puerto virtual y registra por separado.
- No toques `video_demo.py`, `lanzadores/VIDEO_DEMO.bat`, `docs/VIDEO_DEMO.md` ni
  `tests/test_video_demo.py`: son de otra sesión.
- Para mover el brazo real o cargar firmware necesitás el OK explícito del usuario en esa
  misma conversación. El firmware que está cargado alcanza para todo hasta el paso 5.

### Paso 1 — Comprobar que todo pasa
```
.venv\Scripts\python.exe -m unittest discover -s tests -t .
```
**Listo cuando:** dice `OK` (263 tests o más). Si falla algo, arreglalo antes de seguir.

### Paso 2 — Vuelta de referencia en el gemelo (sin el brazo)
Corré este script (abre la app en modo gemelo sin ventana y hace 3 vueltas de la demo):
```python
# guardalo fuera del repo, por ejemplo en %TEMP%\demo_gemelo.py, y correlo desde brazorobtoIA
import sys, time
sys.path.insert(0, '.')
import tkinter as tk
from app import App
root = tk.Tk(); root.withdraw()
ui = App(root, gemelo=True)
ui.open_cameras(); ui.connect_robot()
state = {'fase': 'preparando', 't': time.monotonic()}
def poll():
    if state['fase'] == 'preparando':
        listo = (ui.robot is not None and ui.robot.connected and len(ui.cameras) == 2
                 and ui.twin_calibration_ready.is_set()
                 and all(c.snapshot().frame is not None for c in ui.cameras)
                 and 6 in ui.robot.snapshot().positions)
        if listo and time.monotonic() - state['t'] > 3:
            ui.demo_cycles.set(sys.argv[1] if len(sys.argv) > 1 else '3')
            ui.run_demo_loop(); state['fase'] = 'corriendo'
    elif state['fase'] == 'corriendo' and not ui.execution_busy:
        print('FIN:', ui.auto_status.get(), flush=True); ui.close(); return
    root.after(500, poll)
root.after(500, poll); root.mainloop()
```
**No corras nada más en paralelo**, porque el gemelo va en tiempo real y los tiempos se
distorsionan. Anotá el tiempo de cada vuelta: cada carpeta nueva en `datos\gemelo\demo\`
tiene `metadata.json` con `duration_seconds`.

**Listo cuando:** dice `FIN: Demo terminada: 3 vueltas completas` y las 3 vueltas dan
`"result": "success"`. Referencia: ~39 s por vuelta.

### Paso 3 — Revisión del encadenado (sin el brazo)
Leé `brazoia/execution.py`: `_chain`, `_blend`, `_together_problem`, `MotionExecutor.execute`.
Buscá un solo tipo de problema: **algún camino donde el brazo pueda pasar por una pose
que nadie revisó** (2D con `classify_pose`/`edge_safe`, 3D con `policy.motion_problems`).
Si encontrás uno, escribí un test que lo muestre, arreglalo y volvé al paso 1.

**Listo cuando:** no encontraste nada o lo arreglaste, y el paso 1 da `OK`.

### Paso 4 — Vuelta en el brazo real (necesita OK del usuario)
1. Pedile al usuario: «¿Puedo pedirte que corras una vuelta de la demo en el brazo real?».
2. Con el OK, pedile que haga él, en la app:
   - abrir la app, cámaras y robot;
   - prender **Medir corriente**;
   - sacar el recipiente y poner el cubo cerca de uno de los dos puntos;
   - **Vueltas: 1** → **🔁 DEMO**;
   - al terminar, apagar **Medir corriente** y contarte si se vio más fluido, sobre todo
     al salir de Inicio y al volver.
3. Analizá el CSV más nuevo de `datos\corriente\` y la carpeta más nueva de `datos\demo\`:
   - duración y resultado de la vuelta;
   - pico de `I6_mA` en la etapa `agarre`;
   - `total_mA` máximo y `v_min_V` mínimo por etapa.

**Listo cuando:** la vuelta dio `success`, el pico de la garra quedó por debajo de
~700 mA y el voltaje mínimo por encima de ~10 V (referencia: 578 mA y 10,0 V). Si el
voltaje bajó de 9 V o la vuelta falló, **parás y se lo contás al usuario**.

### Paso 5 — Según lo que diga el usuario
- **Si dice que se ve fluido y está conforme:** pasá al paso 6.
- **Si quiere que sea más rápido:** hay que subir `MOVE_SPEED` (hoy 450, en
  `firmware/firmware_secuencias.ino`, línea ~8). Hacé esto:
  1. Proponele un valor, por ejemplo 700, y explicale que hay que cargar firmware.
  2. Con su OK y la app cerrada **por él**, compilá y cargá:
     ```
     mkdir %TEMP%\firmware_secuencias
     copy firmware\firmware_secuencias.ino %TEMP%\firmware_secuencias\firmware_secuencias.ino
     "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 %TEMP%\firmware_secuencias
     "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload -p COM11 --fqbn esp32:esp32:esp32 %TEMP%\firmware_secuencias
     ```
  3. Cambiá también `MOVE_SPEED` en `brazoia/gemelo.py` y `M6_ANGLE_SPEED_DEG_S` en
     `brazoia/execution.py` (ambos usan 450), y corré el paso 1.
  4. Repetí el paso 4 y compará la corriente con la medición anterior.
- **Si sigue viendo tironeos:** probá en el gemelo `CHAIN_STEP_DEGREES` 8 (hoy 6, en
  `brazoia/execution.py`) y repetí los pasos 1 a 4. No subas `BLEND_MAX_DEGREES` por
  encima de `2 * CHAIN_STEP_DEGREES`.

### Paso 6 — Commit (necesita OK del usuario)
Preguntale si quiere commitear. Con el OK:
- commit en la rama `claude/camera-esp32-testing-wgp4qh`;
- **sin** los archivos del video de demo (paso 0), salvo que él diga lo contrario;
- mensaje en español, con un resumen: demo, garra a 90°, tramos juntos, `power telem`,
  espera tras el agarre, umbral 60, empuje 10/6, encadenado.
- Antes de `git pull`, mirá si hay commits locales sin subir: otra sesión sube a la misma
  rama.

### Pendientes que no son urgentes
- Firmware: que la garra afloje 1–2° al detectar el contacto, para bajar más el pico.
  Necesita carga de firmware y OK del usuario.
- Hardware, para sugerir al usuario: ~1 A no debería hundir una fuente de 5 A. Que revise
  la fuente y los cables, y agregue un capacitor de 1000–2200 µF (16 V o más) cerca de los
  servos.
- Las secciones «4b» y «5» de abajo tienen partes viejas: el encadenado de movimientos
  grandes ya está hecho, y el empuje ya es 10/6 (no 18/15).

## Hecho (sin commit)

### 1. Demo sin recipiente (funciona en el brazo real)

Botón **🔁 DEMO** en la pestaña «Tarea automática» (`App.run_demo_loop` en `app.py`,
módulo `brazoia/demo.py`). Loop: Inicio → agarra el cubo → lo lleva al otro punto de
la mesa → lo baja hasta ~8 mm y lo suelta → vuelve a Inicio. Los puntos son
(−60, −210) y (130, −210) mm (`DEMO_SPOTS`). Las cámaras ubican el cubo en cada vuelta
y lo lleva al punto más lejano. Registro por vuelta en `datos/demo/`. Tests:
`tests/test_demo.py`. Sección en el README.

### 2. Etapa 1 «más rápido y fluido» (probada en el brazo real: 38,8 s, salió bien)

Cambios en `brazoia/execution.py`, `brazoia/mujoco_sim.py`, `brazoia/politica_geometrica.py`,
`brazoia/demo.py`, `brazoia/robot.py`, `app.py`, `brazoia/runlog.py`. Tests:
`tests/test_movimiento_fluido.py`. Sección «Movimiento más rápido…» en el README.

- **Garra del modo geométrico** (`EXPERT_GRIPPER_STEPS` en `mujoco_sim.py`):
  - abre una vez a 90° por ángulo (`6 90.00`) al empezar Aproximación 2, mientras el brazo va;
  - ya no cierra ni reabre sobre el cubo;
  - Agarre sigue con cierre por presión;
  - Soltado abre a 90°;
  - Guardado cierra por presión en camino, después del retiro vertical.
  - El modelo aprendido usa `LEGACY_GRIPPER_ACTIONS`, como antes.
- **Tramos «juntos»**: los grupos de un mismo punto de paso (≤10° por motor) salen
  seguidos si todas las esquinas y aristas de la caja pasan las reglas 2D y los choques
  3D (`SupervisedExecution._together_problem`, `GeometricPolicy.motion_problems`).
  - En puntos de paso no se espera quietud (`PASS_TOLERANCE` 1,5°).
  - Cada orden espera el acuse del firmware (`Moviendo ID` / `PAIR_MOVE_OK`) y corta
    ante un fallo.
  - El espaciado de 65 ms se cuenta desde el acuse.
- **Arreglos de una revisión adversaria**:
  - abrir en camino se revisa contra la mesa en la pose de partida (si no pasa, abre al llegar);
  - Agarre espera un `GRIPPER_HOLD` nuevo y frena si es `closed_limit`;
  - «Mover a mano» revisa la mesa al abrir M6.
- Gemelo: ~88 s → ~38 s por vuelta. Física: 12/12 demo, 12/12 tarea con recipiente.

**Problema abierto de la etapa 1:** los tramos «juntos» mueven hasta 5 motores a la
vez (M1+M5, M2+M3 y M4 a 65 ms), y la garra se mueve junto con el brazo en
Aproximación 2 y Guardado. Eso **va contra el límite de la fuente**. En la prueba real
anduvo, pero no hay que darlo por seguro. El usuario eligió **medir antes de decidir**
cuántos motores pueden ir a la vez (ver la parte 3).

El usuario dijo que quedó «mucho más fluido, pero todavía se notan los
micromovimientos». Vienen de unas 28 paradas cortas por vuelta: cada punto de paso
es una orden aparte, con su arranque y su frenado. Él eligió seguir con un ajuste
**solo en la PC**, no con firmware.

## En curso: 3. Medición de corriente y voltaje de la fuente

Objetivo: medir, en una vuelta de la demo en el brazo real, la corriente de los 6
motores y el voltaje mínimo del bus. Con eso se decide el límite de motores
simultáneos.

Hecho:
- **Firmware** (`firmware/firmware_secuencias.ino`): orden nueva `power telem on|off`.
  - Cada 100 ms imprime `POWER_TELEM ms=<millis> I=<i1>,…,<i6> total_ma=<mA> v_min_dv=<décimas de V> v_id=<id>`.
  - Las corrientes son crudas: 6,5 mA por unidad, −1 si no hay lectura.
  - `processPowerTelemetry()` se llama en `loop()`.
  - Compiló con Arduino CLI. En ese punto todavía no estaba cargado. La carga de
    esta versión se registró más tarde el 6/11.
- **Gemelo** (`brazoia/gemelo.py`): imita `power telem on|off` y las líneas. Las
  corrientes salen del torque del actuador y el voltaje de una fuente ideal de 12 V
  con 0,2 Ω: sirve para probar el camino, no como medida.

También hecho, y probado en el gemelo (una vuelta de la demo con la medición prendida
grabó 354 muestras, cada una con su etapa):
- `brazoia/robot.py`:
  - `parse_power`, `PowerSample`;
  - `start_power_log(path)` / `stop_power_log()`, que graban un CSV con estas columnas
    (`POWER_LOG_COLUMNS`): hora, t, ms del firmware, corrientes en mA de M1..M6,
    total, voltaje mínimo, `v_id`, ángulos M1..M6 y `etapa`;
  - `robot.power_label` lo pone la app.
- `app.py`:
  - casilla **Medir corriente** en la barra de lecturas, que manda `power telem on/off`
    y graba en `datos/corriente/medicion_<fecha>.csv` (en el gemelo,
    `datos/gemelo/corriente/`);
  - muestra «Fuente: X A · mín Y V» y avisa si no llegan datos (firmware viejo);
  - las líneas `POWER_TELEM` no van a la caja de registro;
  - `_drive_stages` pone la etapa en `robot.power_label`.
- Tests: `tests/test_medicion_corriente.py`. Pasaban 257 en total.
- Los números del gemelo **no son una medida**: escala de la garra aplicada a todos los
  motores, con picos falsos de ~6 A.

Plan previo a la carga del 6/11, conservado para el historial:
5. Se pidió autorización antes de cargar. Cargar esa versión también
   carga lo del commit 313e3e6, todavía no cargado:
   - la protección de corriente de la garra: 200 crudo → torque OFF; 170 sostenido
     2 s → afloja 1,5°, hasta 3 veces;
   - la revisión de mesa al abrir con `gripper open`.
   - El umbral de 170 no está medido en el brazo real; esta misma medición lo mide
     (columna de M6 al sostener el cubo). Si el agarre real supera 170, la garra puede
     aflojarse en el traslado.
6. Con el OK: cerrar la app y cargar:
   ```
   "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload -p COM11 --fqbn esp32:esp32:esp32 <carpeta firmware_secuencias>
   ```
   Para compilar, copiar el `.ino` a una carpeta llamada `firmware_secuencias`
   (`compile --fqbn esp32:esp32:esp32 <carpeta>`). Core esp32:esp32 3.3.2.
7. El usuario corre una vuelta de la demo con la medición activa (Vueltas: 1).
   Analizar el CSV:
   - corriente total y por motor en cada etapa;
   - picos en los tramos «juntos» contra los secuenciales;
   - caída de voltaje;
   - corriente de M6 al sostener.
   - Mostrar el resultado con un gráfico (al usuario le gustan; no hay matplotlib en
     el venv: usar un widget o artifact, o instalarlo con permiso).
8. Con los datos, decidir con el usuario el límite (2 contando la garra, 2 del brazo +
   la garra, 3 en saltos chicos…) y aplicarlo en `_together_problem` /
   `_plan_waypoints` (contar los motores que se mueven a la vez, incluida la garra si
   va en movimiento).

### Resultado de la primera medición (6/11, 19:14)

Archivos: `datos/corriente/medicion_20261106_191436.csv` y la vuelta
`datos/demo/prueba_20261106_191446_431843` (salió bien: 38,3 s, cubo a 7 mm del destino).
Una sola vuelta, con muestras cada 100 ms: los picos más cortos pueden no verse.

- **Brazo (M1–M5):** el total nunca pasó de 274 mA, incluso en los tramos «juntos».
  El voltaje se mantuvo en 11,5–11,7 V. Con 3 motores del brazo moviéndose a la vez hubo
  1 sola muestra. Moverlos juntos, a estas velocidades, no parece ser el problema.
- **Garra (M6) al cerrar sobre el cubo:** 900–980 mA durante ~0,5 s, y el voltaje cayó
  de **11,7 a 7,7 V** (t ≈ 16 s de la vuelta). La caída pasa con la garra sola, sin
  motores del brazo moviéndose, y se suma al arranque de Traslado 1 (M3/M4). Con la
  etapa 1 ya no se espera ~1 s entre Agarre y Traslado 1: `_fresh_pose` en lugar de
  la ventana de quietud.
- **M6 sosteniendo el cubo en los traslados:** media 8 crudo (~50 mA), máximo 151 crudo
  (en el pico del cierre). No llegó al umbral de alivio de 170 ni al duro de 200.

Propuesta hecha al usuario:
1. **Hecho (6/11, sin commit)**: `MotionExecutor._wait_grip_settle` en `execution.py`.
   - Después de Agarre espera dos muestras seguidas de M6 < 300 mA, si hay `power telem`
     reciente, con un máximo de 2 s y el aviso «La garra sigue consumiendo…»; si no, 0,8 s.
   - El gemelo siempre llega a los 2 s, porque su corriente estimada no baja: no es una falla.
   - Tests: `test_movimiento_fluido.py`; pasaban 259 en total. Falta medir en el brazo real.
   - **Medido en el brazo real (6/11, 19:26 y 19:27; dos vueltas, ambas bien):** Traslado 1
     ahora empieza con M6 en 20–32 mA y el voltaje en 11,6 V. En Traslado 1 el voltaje no
     baja de 11,2 V. La superposición se fue.
2. Bajar el pico del cierre. Sigue siendo el problema:
   - la garra sola llega a ~1 A (994 y 1020 mA) y el bus cae a **6,9 V** y 8,2 V;
   - no lo marca solo M6: también M4 y M5, o sea que cae todo el bus;
   - ~1 A no debería hundir una fuente de 5 A en 4–5 V, así que la fuente o los cables
     están flojos.
   - Cómo sube el pico: el cierre avanza 1,5° cada 40 ms y declara contacto con 3 lecturas
     seguidas ≥ 92 crudo (~600 mA). En la medición sigue subiendo después de cruzar 600 mA
     (546 → 767 → 1020 mA) y tarda ~0,5 s en bajar.
   - Opciones:
     - (a) sin firmware: `gripper current 60` (~390 mA), que ya existe en el firmware
       cargado, para que detecte el contacto antes. **Elegida y hecha (6/11, sin commit):**
       - `GRIPPER_CONTACT_RAW = 60` en `execution.py`;
       - el ejecutor manda `gripper current 60` antes de cada `gripper close`, y la app al
         conectar. El botón manual sigue mandando solo `gripper close`, porque
         `test_gripper_diagnostic` exige una sola línea, pero usa el umbral que se fijó al
         conectar;
       - física: 40/40 con 92 y con 60, también con 8 mm de error; M6 sosteniendo a
         ~38,5° en vez de ~38°;
       - gemelo: 2/2 vueltas;
       - 260 tests pasan;
       - **falta medir el pico en el brazo real.**
     - (b) firmware: aflojar 1–2° al detectar el contacto, o menos muestras;
     - (c) hardware: capacitor grande cerca de los servos, cables más gruesos, revisar
       la fuente.
3. Mantener los tramos «juntos» del brazo y seguir con los micromovimientos.

### Medición con el umbral 60 (6/11, 19:46): los números dieron bien

`datos/corriente/medicion_20261106_194625.csv`, vuelta `datos/demo/prueba_20261106_194645_306768`
(bien: 38,4 s, cubo a 12 mm del destino).
- Pico de la garra: **578 mA** (antes 994–1020).
- Voltaje al cerrar: **10,7 V** (antes 6,9–8,2).
- Brazo: ≤ 215 mA.
- Voltaje mínimo de toda la vuelta: 10,0 V, en una sola muestra de Traslado 1.

El usuario dijo: «si dan bien los números, procedé a acelerar los movimientos o mover
más motores a la vez».

## En curso: 4. Más motores a la vez y sin frenar (PC, sin firmware)

Hecho en `execution.py`, sin commit:
- **Movimientos grandes en cadena**: con política geométrica, un salto de más de
  `TOGETHER_MAX_DEGREES` (10°) se parte en pasos de `CHAIN_STEP_DEGREES` (6°) sobre la
  línea recta (`_chain`). Cada paso va con todos los motores juntos si su caja pasa las
  revisiones 2D y 3D. Si algún paso no pasa, o el planificador 2D se desvía de la recta,
  el salto va como antes.
- **Encadenar sin frenar** (`_blend`, `MotionSegment.handoff`): en un punto de paso se
  manda el siguiente cuando falta la mitad del paso, si la caja de los tres puntos
  (mínimo y máximo de cada motor) pasa las revisiones, hasta `BLEND_MAX_DEGREES` (12°).
- `_together_problem` acepta `poses` y `limit`. `_plan_waypoints` devuelve también el
  segmento donde empieza a moverse la garra, porque con cadenas ya no hay un segmento
  por punto.
- Resultado: Aproximación 1 en 15 pasos juntos (14 encadenados) y Guardado en 22
  (21 encadenados). Preparar las 11 etapas: 3,2 s.
- Tests nuevos en `test_movimiento_fluido.py`.
- **Gemelo (6/11, 19:52):** demo 3/3, a 5–12 mm del destino; `gemelo_digital.py
  --episodios 3 --semilla 4` (tarea con recipiente) 3/3.
  - **La vuelta no se aceleró**: ~39 s, igual que antes de encadenar.
  - Lo más probable es que cada paso de 6° necesite 3 órdenes (acuse + 65 ms cada una),
    ~0,2 s por paso, lo que da ~30–40°/s, más o menos la velocidad del servo.
  - Debería verse más continuo (sin la escalera), no más rápido.
  - Para acelerar de verdad: subir `MOVE_SPEED` en el firmware (medir con `power telem`).
    Otra opción: mandar el par (2,3) antes, o pasos más largos.
- La velocidad máxima de cada servo la fija el firmware (`MOVE_SPEED` 450 ≈ 40°/s).
  Subirla es otra carga de firmware, que habría que medir con `power telem`.

### Revisión y validación de la etapa 4 (6/11, sin commit)

Cambios en `brazoia/execution.py`:
- `_chain` aplica la revisión de la caja también a los pasos de un solo grupo.
  Si algún paso queda vacío o falla la revisión, usa el recorrido anterior.
- `_gripper_sweep_samples` conserva los extremos y muestras intermedias separadas
  como máximo 8° para revisar la garra en movimiento. La usan la planificación normal,
  la cadena y `_blend`. Antes la cadena reducía el barrido a cuatro valores y
  `_blend` miraba únicamente los extremos.
- `_blend` incluye en la caja el posible rezago de órdenes previas, incluso en
  motores que no reciben una orden nueva. Solo habilita el paso anticipado si pasa
  la revisión de esa caja. El chequeo 3D sigue usando las esquinas de la caja y
  las muestras de la garra.
- `MotionExecutor.execute` conserva los destinos pendientes mientras hay paso
  anticipado. Al cortar la cadena espera todos esos destinos con las tolerancias
  habituales, incluso si el último tramo ordenó solo un subconjunto de motores.
- Se conservaron el tamaño de paso, la velocidad y el espaciado de órdenes.

Verificación desde la raíz del repo:
- `tests/test_movimiento_fluido.py`: dos regresiones nuevas, una para un posible
  choque con M1 rezagado y M6 a medio abrir, otra para la espera de motores omitidos
  del último tramo.
- `.venv/Scripts/python.exe -m unittest discover -s tests -t .`: 263 tests, todos pasaron.
- Demo por `App(root, gemelo=True)`, con puerto `SIMULADO` y «Vueltas: 1»:
  éxito, cubo a 6 mm del destino. Registro
  `datos/gemelo/demo/prueba_20261106_201647_916574/metadata.json`. Duró 46,0 s con
  otras pruebas en paralelo, sin una comparación controlada de velocidad.
- `.venv/Scripts/python.exe gemelo_digital.py --episodios 1 --semilla 4`:
  éxito con recipiente tras ajustar la espera al cortar la cadena. Registro
  `datos/gemelo/gemelo_20261106_201726.json`. Antes de ese ajuste final pasaron
  las tres escenas con la misma semilla (`datos/gemelo/gemelo_20261106_201754.json`).

Para retomar:
1. Pedir autorización explícita antes de mover el brazo. Para esta prueba se usa
   el firmware que ya está cargado. Esta sesión trabajó solo con el gemelo y no
   tocó los archivos de la otra sesión (`video_demo.py` y compañía).
2. Con el OK, correr una sola vuelta de DEMO en el brazo real con «Medir corriente».
   Guardar el CSV y la carpeta de la vuelta. Pedir al usuario que observe las
   pausas cortas, en particular en Aproximación 1 y Guardado.
3. Comparar duración, pico de corriente de la garra y voltaje mínimo por etapa
   con la vuelta del umbral 60 (38,4 s, pico M6 578 mA, mínimo de la vuelta 10,0 V).
   Usar la observación del usuario para evaluar la continuidad. Decidir luego si
   conviene ajustar los pasos en la PC. Cargar firmware requiere autorización aparte.

### Etapa 2 de optimización: qué sigue (6/11)

«Etapa 2» en el README es la segunda etapa del trabajo de optimización. El
«4» de este relevo es el número de sección. El encadenado en la PC ya es
parte de esta segunda etapa. El README describía como idea original una orden
de firmware para cinco motores. Para retomar, mantener el límite vigente de
dos motores por orden y trabajar primero con el firmware instalado.

Lectura de los registros existentes, sin ejecutar movimientos:
- Brazo real, vuelta de referencia con umbral 60:
  `datos/demo/prueba_20261106_194645_306768/metadata.json`. De propuesta a
  completado, Aproximación 1 tardó 6,01 s y Guardado 8,89 s. Sumaron 14,90 s
  de los 38,37 s de la vuelta (39%).
- Gemelo con el encadenado revisado:
  `datos/gemelo/demo/prueba_20261106_201647_916574/metadata.json`. Esas etapas
  tuvieron 48 y 61 grupos, respectivamente. Solo el espaciado de 65 ms exige
  al menos 3,1 y 3,9 s entre la primera y la última orden de cada etapa,
  antes de contar el tiempo de acuses. Parte de ese tiempo coincide con
  movimiento del brazo. Estas dos vueltas son de entornos distintos y sirven
  para elegir qué medir, no para calcular una mejora.

La comparación en el gemelo se hizo en la subsección siguiente. Subir
`MOVE_SPEED` sería una prueba aparte que requiere autorización para cargar
firmware. En la revisión de los registros previos solo se actualizó el relevo.

### Comparación controlada de pasos en el gemelo (6/11)

Se hicieron cuatro vueltas individuales de DEMO con la app oculta en modo gemelo
(puerto `SIMULADO`), en procesos separados y de a una. Semilla 4 (cubo a la
derecha): primero 6°, luego 8°. Semilla 1 (cubo a la izquierda): primero 8°,
luego 6°. Reiniciar `app.twin.rng = np.random.default_rng(semilla)` justo antes
de `run_demo_loop()` para que cada par arranque con el mismo cubo y orientación.
Cambiar `brazoia.execution.CHAIN_STEP_DEGREES` solo en memoria tras importar.
Mantener el límite de 12° para el paso anticipado, los chequeos 2D y 3D, y la
espera de acuses con 65 ms de espaciado. Registrar tiempo total, tiempo por
etapa, grupos, pasos anticipados y resultado. El objetivo es ahorrar tiempo
manteniendo el agarre y la llegada al destino en las dos escenas. La cantidad
de pasos anticipados ayuda a interpretar el cambio de continuidad.

Resultados de los cuatro registros (tiempos de propuesta a completado para
las dos etapas indicadas, y duración completa según `metadata.json`):

| Semilla | Paso | Total | Aproximación 1 | Guardado | Órdenes del brazo | Pasos anticipados | Distancia al destino |
|---|---:|---:|---:|---:|---:|---:|---:|
| 4 | 6° | 42,20 s | 6,42 s | 8,76 s | 194 | 35 | 11 mm |
| 4 | 8° | 37,53 s | 5,47 s | 7,77 s | 159 | 8 | 12 mm |
| 1 | 8° | 36,50 s | 5,58 s | 7,44 s | 154 | 11 | 6 mm |
| 1 | 6° | 40,53 s | 6,32 s | 8,40 s | 189 | 37 | 5 mm |

- Las cuatro vueltas terminaron con `result=success`. Con 8° se ahorraron
  4,67 s y 4,03 s (aproximadamente 10–11%), con 35 órdenes menos en cada escena.
  Las órdenes del brazo se contaron como grupos de la propuesta. Los pasos
  anticipados se contaron en memoria en `ExecutionLog.proposal` como segmentos
  con `handoff > 0`. Hubo menos con 8° y el límite de 12° siguió vigente.
  Ese recuento no mide la fluidez física.
- Registros, en orden de ejecución, todos bajo `datos/gemelo/demo/`:
  `prueba_20261106_204306_911110/metadata.json`,
  `prueba_20261106_204354_943633/metadata.json`,
  `prueba_20261106_204438_062357/metadata.json`,
  `prueba_20261106_204520_384960/metadata.json`.
- El ajuste de 8° se probó solo en memoria. El código sigue con 6°. No se
  movió el brazo real, no se cargó firmware y no se editaron archivos del video.

Siguiente paso según el roadmap: P1. Instrumentar `MotionExecutor.execute`
y registrar en `ExecutionLog` cuánto tiempo lleva, por etapa, enviar órdenes y
esperar acuses, esperar llegada y quietud, mover la garra y esperar tras el
agarre. Validarlo en el gemelo con la versión de 6°. Estos cuatro registros
miden tiempos de etapa y órdenes; todavía no incluyen ese desglose.

Después sigue P2: pedir autorización para que el usuario haga una vuelta
real con el encadenado actual de 6°, «Vueltas: 1» y «Medir corriente».
Guardar el CSV y el registro de la vuelta y pedirle que observe Aproximación 1
y Guardado. Los 8° quedan como candidato si la prioridad es reducir tiempo
y el movimiento se ve bien. Para validarlos en el brazo real, pedir
autorización y comparar con 6° en las mismas condiciones. Revisar tiempos,
resultado, fluidez, pico de M6 y voltaje mínimo antes de adoptar el ajuste.

### P1: desglose de tiempos y diagnóstico de los micromovimientos

El usuario reportó que la demo todavía tiembla y tiene muchos micromovimientos
a lo largo del recorrido. El candidato de 8° del ensayo anterior queda sin
aplicar: aceleró las dos vueltas virtuales, pero redujo los pasos anticipados.

Cambios implementados en la PC:
- `MotionExecutor.execute` mide con reloj monotónico cada etapa completada.
  `orders` incluye espaciado, envío y acuses de las órdenes del brazo.
  `arrival` cubre la espera hasta el punto de paso o la primera lectura de la
  ventana aceptada. `settling` cubre esa ventana de confirmación y la pausa
  final. `gripper` suma el envío y la espera explícita de M6, y
  `after_grasp` mide por separado la espera después del agarre.
- `total` es el tiempo dentro del ejecutor, incluyendo su sobrecarga.
  Las categorías no cuentan dos veces el tiempo de la PC. Si M6 se mueve
  en paralelo, parte de su movimiento físico ocurre durante las órdenes o
  esperas del brazo. El desglose tampoco incluye planificación, fotos ni
  esperas entre etapas; esas se pueden ver con las marcas `t` de los eventos.
- `ExecutionLog.completed` agrega `timings_s` (segundos) al evento
  `completed`, en la tarea automática, la demo y el modo supervisado.
  Los registros viejos y las llamadas que no pasan tiempos conservan su
  formato. Una etapa bloqueada conserva su evento de bloqueo sin desglose.
- Los movimientos, tolerancias, acuses, velocidad y firmware se mantienen.
  No se modificaron los archivos de la otra sesión del video.

Verificación: `.venv/Scripts/python.exe -m unittest discover -s tests -t .`
dio 263/263. Se comprobó el desglose de llegada, quietud, garra y espera
posterior al agarre, y que el registro persiste los tiempos.

Validación en el gemelo con la app cerrada por el usuario antes de la prueba:
- Se abrió `App(root, gemelo=True)` oculta, se comprobó el puerto
  `SIMULADO`, y se corrió DEMO con una vuelta, semilla 4, pasos de 6° y sin
  telemetría de corriente. Resultado `success`, cubo a 12 mm del destino.
  Registro `datos/gemelo/demo/prueba_20261106_205323_164180/metadata.json`.
- Los 11 eventos `completed` tienen las seis claves de `timings_s` y
  valores no negativos. Por redondeo, se admitieron 0,4 ms de diferencia
  al verificar que la suma por fase no supera al total.
- De 46,31 s de vuelta, 35,22 s estuvieron dentro del ejecutor: 9,57 s en
  órdenes y acuses, 16,43 s en llegada, 2,25 s en confirmar quietud, 6,16 s
  de espera explícita y órdenes de garra, y 0,80 s tras agarrar. Parte del
  movimiento físico transcurre durante órdenes y acuses.
- Entre el `completed` de Soltado y el `proposal` de Guardado hubo 7,72 s
  fuera del ejecutor. El mismo intervalo en las cuatro vueltas del ensayo
  anterior fue de 1,89–3,89 s. Medir por separado lectura de pose y
  preparación de Guardado antes de atribuir esa pausa a una sola causa.
- P1 quedó verificado. Esta sesión no movió el brazo real ni cargó firmware.

Qué mirar para corregir el temblor:
1. Separar los tirones mientras cambia la consigna de una oscilación con
   consigna fija. Los tiempos de P1 explican pausas de ejecución y no
   miden vibración física.
2. Si las pausas coinciden con los límites de etapa, ensayar en el gemelo
   la continuidad de P3 entre Traslado 1→2→3, manteniendo los controles de
   colisión y la quietud donde la garra o una foto la exige.
   Si predominan dentro de una etapa, revisar cuántos pasos anticipados
   permite la cadena actual antes de cambiar tamaño o espaciado.
3. Luego validar en el brazo real con autorización explícita. Si también
   oscila quieto, registrar lecturas de ángulo con consigna fija para
   distinguir ese caso de las pausas del planificador.

### P3: ensayo de continuidad entre Traslado 1, 2 y 3

El usuario autorizó probarlo solo en el gemelo. Se ensaya con una modificación
en memoria de `SupervisedExecution.prepare`, limitada a `DemoPolicy` y al
puerto `SIMULADO`. El ensayo reúne los puntos originales de los tres traslados
en una planificación, verifica que no haya acciones de garra en esas etapas
y usa `_plan_waypoints` y `_blend` con 6°, límite de caja de 12° y los controles
2D y 3D existentes. En cada unión busca el punto ordenado a no más de 0,5°
por motor del objetivo de la etapa. Conserva la espera de quietud si la unión
no recibe un `handoff` y usa la ejecución original si ninguna unión lo recibe.
El final de Traslado 3 siempre espera quietud antes de Soltado.

Comparación realizada: cuatro vueltas de una en una, en procesos separados.
Semilla 4, referencia y ensayo. Semilla 1, ensayo y referencia. Registrar
uniones habilitadas, grupos, tiempos del ejecutor en el traslado y tiempo
desde fin de Agarre a fin de Traslado 3 (incluye preparación), además de
resultado y distancia final del cubo. El ensayo agrupado registra el conjunto
como `traslado_3`. Se conserva la configuración del repositorio para la app.

Resultados (tiempos en segundos):

| Semilla | Modo | Uniones anticipadas | Desde fin de Agarre hasta fin de Traslado 3 | Quietud en traslados | Vuelta | Órdenes de traslado | Distancia final |
|---|---|---|---:|---:|---:|---:|---:|
| 4 | Referencia | ninguna | 6,358 | 0,624 | 41,96 | 42 | 12 mm |
| 4 | Ensayo | 1→2 | 5,993 | 0,415 | 41,53 | 41 | 11 mm |
| 1 | Ensayo | 1→2 y 2→3 | 5,686 | 0,207 | 39,76 | 43 | 4 mm |
| 1 | Referencia | ninguna | 6,017 | 0,625 | 40,34 | 41 | 4 mm |

- Las cuatro vueltas terminaron con `result=success`. El traslado, contando
  su preparación, se acortó 0,365 s en la semilla 4 y 0,331 s en la semilla 1.
  El resultado confirma que se quitaron esperas de quietud. La fluidez física
  y la ganancia reproducible de la vuelta completa siguen sin medirse.
- Repetición de `_blend` sin movimiento sobre los grupos registrados, usando
  la pose de fin de Agarre y `DemoPolicy`: en semilla 4 la caja de la unión
  1→2 abarcó como máximo 11,218° y la de 2→3 abarcó 12,993° al incluir el
  rezago de las órdenes anteriores. La segunda devolvió `salto grande` y
  conservó la espera. En semilla 1 las dos cajas pasaron (9,987° y 9,296°).
- Registros bajo `datos/gemelo/demo/`, en el orden de la tabla:
  `prueba_20261106_205819_988659/metadata.json`,
  `prueba_20261106_205907_635285/metadata.json`,
  `prueba_20261106_205954_690586/metadata.json`,
  `prueba_20261106_210039_966337/metadata.json`.
- Solo se ejecutó el ensayo en memoria con el gemelo. En el repositorio se
  actualizó este relevo. No se movió el brazo, no se cargó firmware ni se
  tocaron los archivos del video.

Próximo paso propuesto, sujeto a confirmación: convertir el ensayo en una
opción de prueba del gemelo que mantenga identificadas las etapas y registre
cada unión aceptada o rechazada. Validar también el camino que conserva la
parada y comparar el movimiento en las dos uniones antes de decidir si se
lleva al brazo real.

### Trazas del gemelo: por qué tironean los motores juntos (6/11)

El usuario autorizó medir la velocidad y las órdenes en el gemelo. Una vuelta
de DEMO con `App(root, gemelo=True)`, puerto `SIMULADO`, semilla 4 y pasos de
6°, sin el ensayo P3. Se envolvieron en memoria `DigitalTwin._write_pos` y
`DigitalTwin.tick`. Se registraron Aproximación 1 y Traslados 1–3. Para cada
motor M1–M5 se guardaron el tiempo simulado, destino y velocidad ordenados,
velocidad del perfil justo antes de cada orden y, cada 10 ms simulados, la
consigna, la velocidad de ese perfil y la magnitud de la velocidad física
de MuJoCo (`qvel`). No se modificó el ejecutor ni el firmware.

Resultado: éxito en 41,65 s, cubo a 12 mm del destino. En
`datos/gemelo/demo/prueba_20261106_210451_052957/` quedaron `metadata.json`,
`ordenes_gemelo.csv` (153 cambios de destino por motor) y
`velocidades_gemelo.csv` (6395 muestras por motor). Los motores agrupados
por tick coincidieron con los grupos de las cuatro propuestas del registro.

- En 35 puntos con más de un grupo «junto» transcurrieron 70–160 ms entre
  el primer y el último grupo (mediana 150 ms). Ejemplo al salir de Inicio:
  M1/M5 en t=0, M2/M3 en t=80 ms y M4 en t=160 ms.
- Para cada motor y etapa se tomaron las parejas de órdenes consecutivas
  con destinos al menos a 0,5° y pico del perfil de al menos 5°/s entre
  ambas. En los 135 intervalos elegibles el perfil llegó a su destino y
  se detuvo antes de recibir otro: M1 25/25, M2 31/31, M3 31/31, M4 22/22,
  M5 26/26. Entre la primera muestra detenida y la orden siguiente pasaron
  30–770 ms (mediana 170 ms). En la física del gemelo la mediana de los
  picos fue 33,54°/s y la mediana justo antes de la siguiente orden fue
  0,41°/s. Cada intervalo bajó a menos de la mitad de su pico físico.
- En la simulación de M1–M5 la fuente estimada se usa para telemetría y
  no decide la cadencia. El desfase y las frenadas aparecen con el perfil
  de movimiento y las órdenes espaciadas. La protección por corriente de M6
  es otro mecanismo. Esta vuelta no mide corriente ni vibración del brazo real.

Próximo paso propuesto, sujeto a confirmación: probar en el gemelo un
adelanto del siguiente destino en un tramo concreto, solo si la caja de
movimiento pasa los controles, conservando el límite de dos motores por
orden, los acuses y sus 65 ms. Repetir la misma traza y comparar cuántas
veces cae a cero antes de la nueva orden, además del tiempo y la llegada
del cubo. No llevar el cambio al brazo real sin autorización explícita.

### Diseño de una cola para un solo traslado (6/11)

El usuario autorizó preparar el diseño. Queda como propuesta. No se programó
ni se cargó firmware y no se movió el brazo.

**Alcance.** Empezar con Traslado 2 de la demo, que sostiene el cubo y no
ordena movimiento de M6. Usar los destinos de 6° y las revisiones actuales.
La PC cargaría todos los grupos del traslado antes de arrancar. El ESP32
marcaría cuándo despacharlos al bus, sin esperar una respuesta de la PC
entre cada grupo. El firmware instalado solo acepta órdenes inmediatas,
con hasta dos motores y un mínimo de 50 ms. La PC hoy espera cada acuse y
deja 65 ms. Mandarle todo seguido al firmware instalado produce rechazos.

**Carga y ejecución.** Protocolo propuesto para el gemelo, todavía sin
definir sintaxis definitiva:

1. **Cargar.** La PC manda un ID de recorrido, la cantidad de puntos y los
   grupos de cada punto. Para cada punto indica si es de paso (con la
   tolerancia de `handoff` y la caja aprobada) o si exige quietud. El buffer
   tiene capacidad fija y se rechazan cargas incompletas, fuera de rango o
   más grandes que esa capacidad. La confirmación de carga significa que
   quedó listo para empezar.
2. **Arrancar.** Con el mismo ID, verificar torque, ausencia de fallo,
   lecturas actuales, pose inicial dentro del margen aprobado y estado
   de M6 sosteniendo. La PC conserva las revisiones 2D y 3D del recorrido,
   incluyendo el posible rezago. El ESP32 conserva el chequeo 2D desde la
   pose medida antes de emitir cada grupo y cancela si necesita un ajuste
   o no puede validar la orden. Mantener la cadencia mínima del firmware
   entre grupos, y probar primero con 65 ms en ambos modos para aislar el
   efecto de la cola. En la prueba, cada orden tiene hasta dos motores.
3. **Anticipar.** Cuando todos los grupos del punto se hayan emitido, el
   ESP32 revisa la posición de los motores con lecturas frescas. Solo pasa
   al destino siguiente con movimiento pendiente si el plan aprobó esa
   unión, se alcanzó el umbral de `handoff` y la pose sigue dentro de la
   caja revisada. Si la unión exige parada, espera llegada y quietud.
   No avanzar solo por reloj. Para la prueba inicial, usar un presupuesto
   conservador de hasta dos motores del brazo en movimiento: una orden
   al mismo motor no consume otro cupo, y un motor que cambia de grupo
   libera el cupo al confirmar quietud. La garra solo sostiene. Si la
   protección de M6 interviene, se cancela la cola. Separar el límite de
   motores por orden del límite de motores que se mueven simultáneamente.
   El segundo no debe aumentarse para esta prueba.
4. **Terminar.** El último punto exige quietud, incluyendo los motores
   que quedaron con una orden pendiente. Informar ID y estado final
   (terminó, cancelado o fallo). Registrar en el gemelo cuándo se despacha
   cada grupo y cuándo la trayectoria realmente se frena. Las órdenes
   manuales de movimiento se rechazan mientras la cola está activa.

**Detener.** Hoy el botón cancela el ejecutor de la PC. Para la cola también
debe mandar una cancelación prioritaria al ESP32. Al recibirla, invalidar el
ID, vaciar pendientes y confirmar la cancelación antes de despachar otro
grupo. No encolar la cancelación detrás de los movimientos. Los motores
que ya recibieron un destino pueden terminar ese paso verificado. La
confirmación de cancelación significa «sin más órdenes», no «brazo quieto».
La emergencia vacía la cola antes de ejecutar el corte de torque existente.
Un fallo, lectura vencida o pérdida del enlace cancela la cola; en el
prototipo se prueba un latido de la PC con plazo acotado. Probar pérdida de
comunicación y cancelación justo entre grupos. No reanudar automáticamente
ni aceptar un arranque atrasado para un ID cancelado.

**Cómo medir.** Repetir el mismo Traslado 2, desde la misma pose y escena en
el gemelo. Comparar el envío normal y la cola con igual presupuesto de
motores activos, cadencia mínima, controles y destinos. Registrar tiempos de
recepción y despacho, velocidad por motor, caídas a cero antes de una nueva
orden y resultado final. Probar además un punto que exige quietud, una pose
fuera de caja y Detener con órdenes pendientes. Si la cola ahorra tiempo de
comunicación pero los perfiles siguen llegando a cero, ajustar el criterio
de paso o la trayectoria dentro de las cajas verificadas. El presupuesto
eléctrico y cualquier cambio al firmware real requieren una prueba aparte
con autorización.

**Próximo paso, sujeto a confirmación.** Prototipar esta cola solo en el
gemelo para un Traslado 2. Comparar la traza con la referencia y verificar
que Detener cancele los grupos pendientes antes de decidir si conviene
llevar el diseño al ESP32.

### Prototipo de cola en el gemelo: resultado de Traslado 2

El usuario autorizó el prototipo virtual. Se guardó el banco en
`datos/gemelo/ensayo_cola/prototipo.py` y su salida en `resultado.json`
(misma carpeta, ignorada por Git). Desde la raíz de `brazorobtoIA` se
reproduce con `.venv/Scripts/python.exe -m datos.gemelo.ensayo_cola.prototipo`.

- La app se ejecutó oculta en modo gemelo, puerto `SIMULADO`, una vuelta,
  semilla 4, pasos de 6°. Justo al iniciar Traslado 2 se capturaron el
  plan y el estado físico de MuJoCo (incluido el cubo ya agarrado),
  perfiles, torque y estado de M6. Se clonó ese estado para cada ensayo.
- El banco despacha los grupos existentes vía `DigitalTwin.handle_line`,
  con sus chequeos y respuestas, y corre la física en pasos de 10 ms.
  Usa hasta dos motores con destinos pendientes y 65 ms mínimos entre
  órdenes. Un motor libera su cupo al quedar cerca del destino, con
  perfil detenido y velocidad física ≤2°/s durante tres muestras.
  Esta es una condición del banco, no una medición de corriente.
- El modo «PC» se modeló con 10 ms adicionales antes de cada despacho.
  Ambos modos usan el mismo plan y condiciones de llegada. El banco
  prueba la cadencia de una cola precargada y la prioridad de cancelar
  en el despachador. Todavía no tiene protocolo por USB, latido de enlace
  ni conexión con el botón real de la app.

| Variante | Traslado 2 | Grupos emitidos | Reórdenes con perfil detenido | Anticipos |
|---|---:|---:|---:|---:|
| Cola local | 3,26 s | 12 | 20 | 0 |
| PC con demora modelada | 3,38 s | 12 | 20 | 0 |

Las dos ejecuciones completaron el tramo. El error final máximo fue 0,633°
y la variación máxima de la posición del cubo relativa a la pinza fue
4,36 mm con cola y 4,41 mm con demora. Los 0,12 s de diferencia coinciden
con los 12 × 10 ms modelados. No se observó una reducción de frenadas.
En este recorrido se alternan los pares M1/M5 y M2/M3; con dos motores
activos como máximo, el siguiente par espera que el anterior libere cupo.
En ambos modos hubo 127 ticks bloqueados por ese presupuesto mientras
había un grupo listo para enviar.

Cancelación: se pidió al despachador a los 0,28 s, en el tick donde la
referencia iba a enviar el segundo grupo. Quedó un grupo emitido y se
cancelaron los otros 11. Se avanzaron 0,60 s de física sin despachar otra
orden. También se ensayó una pose de arranque fuera del margen y se
rechazó con cero órdenes. Las verificaciones del banco terminaron bien.
No se movió el brazo real ni se cargó firmware.

Próximo paso propuesto, sujeto a confirmación: probar en el gemelo una
trayectoria que permita dar más de un destino consecutivo al mismo par
antes de ceder el turno. Recalcular las cajas 2D/3D de esa trayectoria y
mantener el presupuesto de dos motores. Medir si baja la cantidad de
frenadas antes de invertir en el protocolo del ESP32.

### Diagnóstico de la pausa entre Soltado y Guardado (6/11)

El usuario reportó una pausa larga después de soltar el cubo y autorizó
separar la lectura de pose del cálculo de Guardado. Se hicieron dos vueltas
individuales, una con el cubo de cada lado (semillas 4 y 1), en la app
oculta con `gemelo=True`, puerto `SIMULADO` y pasos de 6°. Ambas dieron
`result=success`.

Se pusieron cronómetros temporales en memoria con `perf_counter`.
`ExecutionLog.completed` marca fin de Soltado antes de escribir el evento.
Se midió `App._fresh_pose`, `SupervisedExecution.prepare` solo en Guardado
y sus llamadas a `_plan_waypoints`, `_chain`, `_blend`,
`_together_problem` y `GeometricPolicy.motion_problems`. Se marcó también
la propuesta y el primer envío de `RobotSerial.send` en Guardado.
Los tiempos internos se incluyen en el de preparación, por lo que
no se suman entre sí.

| Semilla | Leer pose | Preparar Guardado | Controles 3D dentro de preparar | Fin de Soltado a primer envío |
|---|---:|---:|---:|---:|
| 4 | 0,055 ms | 3,490 s | 3,251 s (35 llamadas) | 3,496 s |
| 1 | 0,058 ms | 3,075 s | 2,831 s (34 llamadas) | 3,081 s |

- La pose ya estaba disponible en la telemetría. En estas vueltas la pausa
  fue casi toda preparación, y 92–93% de esa preparación fue revisión 3D.
  `_chain` ocupó 2,213 y 2,003 s; `_blend`, 1,253 y 1,031 s. La pausa de
  7,72 s de la vuelta anterior no se reprodujo en estas dos.
- Registros con `metadata.json` y `diagnostico_guardado.json` en
  `datos/gemelo/demo/prueba_20261106_211822_223073/` (semilla 4) y
  `datos/gemelo/demo/prueba_20261106_211909_638259/` (semilla 1). No se
  cambió el movimiento, no se abrió un puerto real ni se cargó firmware.

**Próximo paso propuesto, sujeto a confirmación.** Ensayar en el gemelo
preparar Guardado antes de que termine Soltado, usando su pose final
prevista y manteniendo la revisión 2D y 3D. Al soltar, conservar la
confirmación de la garra y comparar la pose medida con la región que
se revisó. Si queda fuera, o se cancela o cambia la ejecución, descartar
el plan anticipado y usar la planificación actual si corresponde.
Medir la pausa y el tiempo total, ya que adelantar el cálculo puede
mover la espera a otro punto de la vuelta.

### Ensayo de Guardado anticipado y pasos de 8° (6/11)

El usuario autorizó anticipar el cálculo y pidió probar órdenes de mayor
recorrido. Se hizo solo en el gemelo, en procesos separados, sin cambiar
el ejecutor de la app ni el firmware. El banco está en
`datos/gemelo/ensayo_guardado/prototipo.py` y usa el clon de
`datos/gemelo/ensayo_cola/prototipo.py`. Ambos directorios quedan ignorados
por Git. Desde `brazorobtoIA` se ejecuta
`.venv/Scripts/python.exe -m datos.gemelo.ensayo_guardado.prototipo MODO SEMILLA`.

- Modos: `referencia6`, `anticipado6`, `largo8` y `combinado8`.
  Cada uno usa la app oculta en modo gemelo y verifica el puerto
  `SIMULADO`. En los modos de 8° cambia `CHAIN_STEP_DEGREES` solo en
  memoria. Se mantienen el límite de caja de 12° para el paso anticipado,
  las revisiones 2D y 3D, y el espaciado y acuses originales. Si un tramo
  no admite la cadena se usa el recorrido anterior.
- Al empezar a abrir M6 en Soltado, se clona el estado de MuJoCo y se
  pronostica la pose final en la copia, sin emitir órdenes adicionales
  al gemelo de la demo. Otro hilo prepara Guardado en un modelo de
  planificación independiente, usando la misma escena y chequeos.
  Se reutiliza solo si las seis lecturas recibidas al terminar Soltado
  coinciden exactamente a la resolución de la telemetría con las usadas
  para planificar. Si no coinciden se planifica normalmente. El cálculo
  anticipado puede seguir en curso al soltar: solo se espera lo que falta.
  La predicción por clonación es específica del gemelo.
- Se mide desde `ExecutionLog.completed` de Soltado hasta el primer
  `RobotSerial.send` de Guardado. Se muestrea cada 10 ms y se cuentan,
  por motor, los intervalos entre órdenes con destinos separados al menos
  0,5° y pico del perfil de 5°/s o más. Una frenada completa se cuenta
  si el perfil ya estaba quieto en su destino al llegar la próxima orden.
  El recuento abarca Aproximación 1, Traslados 1–3 y Guardado.

Resultados. Las cinco vueltas dieron `result=success`:

| Semilla | Modo | Pausa antes de Guardado | Vuelta | Órdenes del brazo | Frenadas completas / intervalos | Distancia final |
|---|---|---:|---:|---:|---:|---:|
| 4 | Referencia 6° | 3,568 s | 42,164 s | 194 | 239/239 | 12 mm |
| 4 | Anticipado 6° | 2,279 s | 40,697 s | 194 | 239/239 | 12 mm |
| 4 | Solo 8° | 1,862 s | 37,732 s | 159 | 175/176 | 12 mm |
| 1 | Combinado 8° | 0,613 s | 35,095 s | 154 | 171/171 | 4 mm |
| 4 | Combinado 8° | 0,902 s | 36,509 s | 159 | 176/176 | 12 mm |

- Comparación en la misma escena (semilla 4): anticipar con 6° acortó la
  pausa 1,289 s. Los pasos de 8° redujeron las órdenes en 35 y la cantidad
  de frenadas completas en 63–64 en los tramos medidos. Los motores aún
  frenaron antes de casi todos los destinos siguientes.
- Combinando ambos, en la semilla 4 la pausa bajó 2,666 s y la vuelta
  5,655 s. En las tres vueltas anticipadas se reutilizó el plan y se
  solaparon 1,47 s de cálculo con la apertura. La semilla 1 confirma que
  completó el recorrido con el cubo del otro lado. Se hizo una vuelta
  por configuración y escena, sin estimar variabilidad.
- Registros con `metadata.json` y `ensayo_guardado.json` bajo
  `datos/gemelo/demo/`, en orden de la tabla:
  `prueba_20261106_212329_143596/`, `prueba_20261106_212417_842162/`,
  `prueba_20261106_212504_130772/`, `prueba_20261106_212547_414156/` y
  `prueba_20261106_212631_926933/`.
- Se conservó la configuración de la app. No se movió el brazo real,
  no se cargó firmware ni se tocaron los archivos del video.

**Próximo paso, sujeto a confirmación.** Habilitar una opción visible
solo en el gemelo para comparar ambos modos lado a lado o en vueltas
repetidas. Medir si el usuario percibe menos tirones y verificar el
camino de descarte del plan anticipado con una discrepancia provocada
entre la pose pronosticada y la medida.

### Menos órdenes con pasos selectivos de hasta 10° (6/11)

El usuario autorizó buscar ahorros en Aproximación 1 y Guardado y preguntó
por la pausa antes de volver. La app normal sigue en 6° y prepara Guardado
al terminar Soltado. Los ensayos anteriores de anticipación eran separados
de la app. Incluso en ellos quedaban 0,61–0,90 s: el cálculo empezaba al
abrir la garra, unos 1,47 s antes del fin, y todavía no había terminado.

Cambios en el banco local ignorado por Git,
`datos/gemelo/ensayo_guardado/prototipo.py`:
- `selectivo10` y `selectivo10_temprano` parten de 8°. Solo para las
  cadenas largas de Aproximación 1 y Guardado se intenta el máximo
  vigente de 10°, con los chequeos normales. Si falla se reintenta con
  8° y queda disponible el recorrido anterior. Se conservan los puntos
  de retirada de Guardado, los acuses, su espaciado y la caja de 12°
  para el paso anticipado.
- Dentro de una sola preparación de Guardado, se reutiliza el resultado
  de revisión 3D únicamente si coinciden los seis ángulos exactos y el
  modelo de revisión es el mismo sin recipiente. No se redondea, ni se
  omiten poses nuevas. Hubo 239 aciertos de 5024 llamadas en semilla 4 y
  128 de 4216 en semilla 1. No se aisló el ahorro de tiempo del cache.
- En `selectivo10_temprano` se conserva el pronóstico por clon del gemelo
  al abrir M6. Se reutilizó el plan en ambas escenas al coincidir las
  seis lecturas. Sigue prevista la planificación normal si difieren.
- Desde `brazorobtoIA`: `.venv/Scripts/python.exe -m
  datos.gemelo.ensayo_guardado.prototipo selectivo10_temprano 4`.
  Ejecutar una configuración por proceso, puerto `SIMULADO`.

| Semilla | Modo | Aproximación 1 | Guardado | Órdenes totales | Frenadas medidas | Pausa antes de Guardado | Vuelta |
|---|---|---:|---:|---:|---:|---:|---:|
| 4 | Combinado 8° previo | 36 | 49 | 159 | 176 | 0,902 s | 36,509 s |
| 4 | Selectivo 10° sin anticipar | 30 | 43 | 147 | 156 | 1,529 s | 35,777 s |
| 4 | Selectivo 10° anticipado | 30 | 43 | 147 | 156 | 0,263 s | 34,592 s |
| 1 | Combinado 8° previo | 36 | 47 | 154 | 171 | 0,613 s | 35,095 s |
| 1 | Selectivo 10° anticipado | 30 | 41 | 142 | 151 | 0,153 s | 33,837 s |

- Los tres ensayos nuevos dieron `success`, cubo a 12 mm del destino
  en semilla 4 y 6 mm en semilla 1. Cada escena ahorró seis órdenes en
  Aproximación 1 y seis en Guardado, sin cambiar los grupos de las demás
  etapas. El ahorro proviene de dos puntos menos por cadena larga, con
  tres órdenes por punto (M1/M5, M2/M3 y M4). En los 156 y 151 intervalos
  elegibles, respectivamente, el perfil siguió frenando completamente.
- En semilla 4 los modos selectivos emitieron los mismos 147 grupos.
  Preparar antes de que termine Soltado bajó de 1,529 a 0,263 s la
  espera hasta el primer envío de Guardado. Con ambos cambios quedó
  en 0,153–0,263 s. Son vueltas individuales por modo y escena.
- Nuevos registros `metadata.json` y `ensayo_guardado.json` en
  `datos/gemelo/demo/prueba_20261106_213109_639409/`,
  `datos/gemelo/demo/prueba_20261106_213150_980690/` y
  `datos/gemelo/demo/prueba_20261106_213231_117248/`.
- Siguiente lugar para buscar ahorro: M1/M5 recibe 10 órdenes al salir
  y 11 en el regreso largo de semilla 4. Se puede ensayar acumular
  cambios pequeños de ese par en tramos despejados, manteniendo destinos
  finales y recalculando las cajas de toda la secuencia. Pendiente de prueba.
- Se mantuvo el código de la app. No se movió el brazo real, no se cargó
  firmware ni se tocaron los archivos del video.

**Próximo paso, sujeto a confirmación.** Hacer que el modo selectivo
anticipado quede disponible como prueba visible solo en el gemelo para
comparar con la demo normal y verificar también el descarte ante una
discrepancia de pose. Mantener la configuración del brazo real.

### Firmware de prueba: un mensaje por punto, con grupos de dos (6/11)

El usuario autorizó implementar, compilar y ensayar en el gemelo.
Cargar en el ESP32 y mover el brazo real requieren una autorización aparte.
Se revisó el trabajo previo, en especial el commit `313e3e6` y
`docs/DIAGNOSTICO_GARRA.md`, y los CSV reales de corriente.

- Equipo documentado: ESP32, seis STS3215, fuente compartida de 12 V y 5 A.
  La garra M6 es la variante de 12 V y 30 kg·cm. Para la variante de
  12 V, Waveshare publica 0,9 A nominales y 2,7 A con rotor bloqueado.
  Dos servos equivalentes bloqueados sumarían 5,4 A antes del resto de
  la carga. No se verificó la variante individual de M1–M5.
  [Ficha primaria de Waveshare](https://docs.waveshare.com/ST3215_Servo).
- CSV `medicion_20261106_191436.csv`: M6 llegó a ~982 mA en
  Traslado 1, con mínimo de 7,7 V informado por M5. Tras agregar la
  espera después del agarre, `medicion_20261106_192608.csv` y
  `medicion_20261106_192716.csv` dieron picos de 994 y 1020 mA en
  Agarre y mínimos de 6,9 y 8,2 V. En Traslado 1 M6 quedó en hasta
  26–32 mA y el mínimo fue 11,2 V.
- Con umbral de contacto 60, `medicion_20261106_194625.csv` dio
  578 mA de pico M6 y 10,7 V de mínimo en Agarre. La lectura aislada de
  10,0 V en Traslado 1 fue de M6 con apenas 59 mA totales; no alcanza
  para atribuirla a una sobrecorriente. La telemetría son lecturas
  consecutivas de los servos, cada 100 ms. Puede perder picos y no
  equivale a un límite de corriente instantáneo.
- Se preservaron umbral de contacto 60 desde la PC, vigilancia de la
  garra y espera posterior al agarre. La orden nueva excluye M6 y
  cancela ante intervención de su vigilancia. Los grupos de la orden
  nueva se ejecutan sin superponerse. El límite es preventivo, no una
  garantía de que la fuente mantenga tensión ante un servo trabado.

Protocolo y archivos:
- `firmware/firmware_secuencias.ino` acepta
  `waypoint ID N MOTOR GRADOS ... [N MOTOR GRADOS ...]`. Cada grupo N es
  de uno o dos motores, con M1–M5 únicos. Hay un punto pendiente a la vez.
  `WAYPOINT_ACCEPTED` indica recepción, `WAYPOINT_DONE` indica llegada
  y quietud. El ESP32 valida la trayectoria de cada grupo desde la pose
  medida, comprueba torque y lee los destinos ordenados. Para avanzar
  confirma quietud de los seis motores en tres lecturas y que el par
  anterior quedó dentro de 3°. El espaciado mínimo es de 65 ms.
- `waypoint keepalive ID` mantiene vigente el punto (750 ms). La PC lo
  manda cada 200 ms. `waypoint cancel ID` descarta los grupos pendientes.
  Detener hace que el ejecutor mande esa cancelación; un grupo ya
  despachado puede terminar su movimiento. Emergencia conserva su corte
  de torque y cancela antes de actuar. El host hace coincidir los acuses
  por ID e ignora los atrasados.
- `brazoia/gemelo.py` replica el protocolo. En `app.py`, la bandera
  `--puntos-firmware` solo se acepta con `--gemelo` y habilita la prueba
  en Traslado 2 de la demo. Desde `brazorobtoIA`:
  `.venv/Scripts/python.exe app.py --gemelo --puntos-firmware`. Los demás
  modos siguen usando sus órdenes existentes. En el registro de esta
  prueba, `arrival` incluye la espera y quietud dentro del firmware.

Verificación en el gemelo, semilla 4, dos modos, misma planificación de 6°:

| Modo | Órdenes de movimiento PC | Latidos PC | Tiempo dentro de Traslado 2 | Vuelta |
|---|---:|---:|---:|---:|
| Referencia | 12 | 0 | 1,849 s | 41,959 s |
| Puntos de firmware | 6 | 24 | 5,263 s | 44,938 s |

La referencia permitió el solapamiento original. La prueba nueva mantuvo
los mismos 12 grupos internos con quietud entre grupos. Ahorró mensajes
de movimiento de la PC, pero no aceleró el tramo. Con los latidos
también hubo más mensajes totales. Ambas vueltas terminaron con éxito,
cubo a 11 y 13 mm del destino. Registros `metadata.json` y
`ensayo_puntos_firmware.json` en
`datos/gemelo/demo/prueba_20261106_214128_286840/` y
`datos/gemelo/demo/prueba_20261106_214629_792222/`. Una primera repetición
de la variante también terminó bien (Traslado 2 en 5,222 s, carpeta
`prueba_20261106_214216_008872`).

Pruebas en `tests/test_puntos_firmware.py`: despacho por grupos y
quietud, cancelación con pendientes, pérdida de latido, rechazo de M6 y
duplicados, veto por vigilancia de garra, acuses por ID y cancelación
enviada por el ejecutor al detener. La suite completa dio 271/271.
El firmware compiló con Arduino CLI, core ESP32 3.3.2, FQBN
`esp32:esp32:esp32` y la biblioteca local SCServo (329 007 bytes de
programa y 21 048 bytes de memoria dinámica). La compilación usó la
instalación local de Arduino con acceso fuera del entorno aislado;
solo se ejecutó `compile`. La cancelación física no fue probada.

Sobre mover los cinco con un umbral de 2 A: se puede diseñar un presupuesto
dinámico, pero reaccionar al muestreo existente después del arranque no
evita el pico inicial. El voltaje ya llegó a 6,9 V con ~1 A registrado en
la garra. Para estudiar concurrencia variable, primero diseñar medición
más rápida de corriente total y tensión, un presupuesto previo al despacho
y una respuesta ante caída de tensión. Los 2 A serían un parámetro a
validar, no una garantía. No se cambió el límite ni se probó el brazo real.

**Próximo paso, sujeto a confirmación.** El usuario pidió probar primero
en el gemelo un límite variable. Probar un supervisor virtual que parta
de dos motores y agregue como máximo uno por vez, con techo de cinco
solo en simulación. Inyectar muestras de corriente y tensión con picos,
caídas y lecturas vencidas. Verificar que ante un pico no se incorpore
otro motor, que el agarre conserve su prioridad y que Detener invalide
las órdenes pendientes. Comparar tiempos y frenadas con la referencia.
La fuente simulada no valida el presupuesto eléctrico real. El firmware
de dos motores queda como base hasta decidir el siguiente cambio.

## Después: 4b. Otros ajustes posibles para los micromovimientos

Ideas, todavía sin programar, a respetar con el límite que salga de la medición:
- **Pasar al punto siguiente antes de llegar**, por ejemplo a mitad de camino del
  actual, si la caja que forman los tres puntos consecutivos (mínimo y máximo de cada
  motor) pasa las reglas 2D y 3D. En el gemelo, `ServoProfile.command` dice cómo
  reacciona un servo a un destino nuevo en marcha. Revisarlo antes, y medir en el real
  si el STS3215 conserva la velocidad.
- **Menos puntos de paso**: bajada vertical cada 15 mm y empuje cada 4 mm, hoy en
  `PickPlaceExpert._vertical` / `push_step`. Medir el salto por motor y el desvío de la
  línea recta con pasos más largos, y mantener los saltos «juntos» dentro del límite.
- **Movimientos grandes** (salir de Inicio, Traslado 2, volver): hoy van «en escalera»
  de a dos motores. Partirlos en pasos chicos encadenados solo si el límite de
  corriente lo permite.

## Pendiente: 5. La garra empuja el cubo en vez de agarrarlo donde está

El usuario dijo que «suele pasar» que la garra empuja el cubo y no lo agarra en su
lugar. Es el agarre empujando del experto (`PickPlaceExpert(push_clearance=0.018,
push_extra=0.015)`): el dedo fijo baja a 18 mm del costado del cubo y avanza
33 mm, arrastrándolo ~15 mm antes de cerrar, para aguantar 20 mm de error de
ubicación.

Medido en la física (3 escenas, error en 8 direcciones):

| Variante | Mueve el cubo | 5 mm | 10 mm | 15 mm |
|---|---|---|---|---|
| 18/15 (hoy) | ~14 mm | 24/24 | 24/24 | 24/24 |
| 10/6 | ~5 mm | 24/24 | 24/24 | 20/24 |
| 8/3 | ~3 mm | 24/24 | 24/24 | 13/24 |
| sin empuje | ~2 mm | 24/24 | 8/24 | 6/24 |

La calibración real de cámaras erra al cubo ~12 mm de media (p90 21 mm) en
validación cruzada, y ~7 mm de mediana sobre las demos de octubre. **El usuario eligió
10/6 mm y ya está aplicado** como valor por defecto de `PickPlaceExpert` (física 12/12
demo y 12/12 tarea). Falta probarlo en el brazo real.

La versión de `power telem` se cargó el 6/11 por COM11, con autorización
y la app cerrada. En reposo reportó 0 mA y 11,7 V. Las vueltas reales
posteriores están en la parte 3. Esa carga precedió al candidato
`waypoint`, que se verificó solo en el gemelo.

## Cómo medir sin el brazo

- Gemelo con la app: `.venv/Scripts/python.exe app.py --gemelo`.
- Script de prueba que se usó: abre la app en modo gemelo sin ventana, conecta, pone
  «Vueltas: 3» y aprieta DEMO. Estaba en el scratchpad de la sesión; si falta, se
  rehace con `App(root, gemelo=True)`, `open_cameras()`, `connect_robot()`,
  `demo_cycles.set('3')`, `run_demo_loop()`, y se espera a que `execution_busy` vuelva
  a False.
- Física pura: `brazoia.mujoco_sim.run_episode` con `PickPlaceExpert` /
  `brazoia.demo.TablePlaceExpert`.

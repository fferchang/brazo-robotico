# Gemelo digital

El brazo de MuJoCo detrás del mismo puerto serie y de las mismas cámaras que el
brazo real. La aplicación, `RobotSerial`, `MotionExecutor`, `SupervisedExecution`
y la política geométrica corren el mismo código que con el ESP32. Solo cambia lo
que hay del otro lado del cable: un firmware virtual (`brazoia/gemelo.py`) que
responde como `firmware/firmware_secuencias.ino`, con la física de
`docs/SIMULACION_MUJOCO.md`.

No abre puertos COM ni cámaras DirectShow, no graba demostraciones y no toca
`config.json`, `datos/calibracion/camaras.json` ni `assets/calibracion.json`.
Ningún resultado habilita movimientos del brazo real.

## Dos formas de usarlo

- **`lanzadores/GEMELO_DIGITAL.bat`**: el brazo virtual hace la tarea completa
  solo, escena tras escena, con el visor de MuJoCo abierto. La consola muestra
  cada etapa; al final queda un informe en `datos/gemelo/gemelo_*.json`. Acepta
  `--episodios N`, `--semilla S`, `--giro-max G` (grados de giro del cubo y del
  recipiente, 5 por defecto), `--recalibrar` y `--ritmo X`. Sin `--visor`
  (llamando a `gemelo_digital.py` directamente) corre sin ventana.
- **`lanzadores/INICIAR_GEMELO.bat`**: la aplicación de siempre (`app.py --gemelo`).
  «Abrir cámaras» abre las virtuales y el único puerto es `SIMULADO`. Arriba
  aparece el recuadro del gemelo:
  - **Nueva escena**: cubo y recipiente en otro lugar; el brazo vuelve a Inicio.
  - **Hacer la tarea solo (12 etapas)**: prepara y ejecuta cada etapa sin pedir
    confirmación y anota el resultado según la física. Solo existe en el gemelo.
  - **Ver en 3D (MuJoCo)**: el visor 3D, con la física en marcha.

  El flujo supervisado (Iniciar tarea → Preparar → CONFIRMAR Y MOVER) también
  funciona, igual que con el brazo real. Las ejecuciones se guardan en
  `datos/gemelo/ejecuciones/`, separadas de las reales. Demostraciones,
  calibraciones y «Guardar ajustes» de cámara están deshabilitados.

## Qué imita el firmware virtual

- Comandos: `<id> <grados>`, `motorN`, `pair`, `bpair`, `bmove`, `read`, `scan`,
  `posall`, `arm torque on/off`, `torque on/off`, `gripper open/close/stop/release`,
  `gripper current N`, `help`. Respuestas con el mismo texto.
- Telemetría `POS ID n: pos (~grados deg)` a 20 Hz; M6 informa real = lógico + 10.
- Límites por motor (`AJUSTE LIMITE`), regla dinámica de M4, piso y autocolisión
  2D (`firmware_pose_reason`, con la geometría de `brazoia/planner.py` y solo las
  revisiones de `classifyPoseInvalid`), corrección cercana
  (`AJUSTE SEGURO`), revisión de 20 puntos intermedios en `pair` y la pausa de
  50 ms entre movimientos.
- Servos: perfil trapezoidal con la velocidad y aceleración de `WritePosEx`
  (450 pasos/s ≈ 39,6°/s y 4500 pasos/s² para el brazo; 300 pasos/s para abrir
  la garra). En `pair`, el motor que recorre menos va más lento, como el firmware.
- Garra: cierre por presión con pasos de 1,5° cada 40 ms, 250 ms de arranque
  ignorados, tres lecturas sobre el umbral, tope cerrado, sobrecorriente y 15 s
  de límite. La «corriente» sale del torque del actuador: 0,30 N·m equivale a 92.
- Torque OFF deja el motor sin fuerza: el brazo cae con la gravedad.

## Diferencias que quedan

- Las de `SIMULACION_MUJOCO.md`: medidas estimadas, ceros de M3 y M4 sin
  calibrar, sin juego mecánico, flexión ni error de encoder.
- Al congelar la garra (`GRIPPER_HOLD`), el firmware ordena la posición leída y
  la elasticidad del cubo y de los dedos sigue apretando. MuJoCo no tiene esa
  elasticidad: el gemelo conserva la consigna del momento.
- Las cámaras virtuales están ubicadas a ojo y se calibran solas (25 escenas,
  `datos/gemelo/camaras_virtuales.json`); se recalibran si cambia el modelo 3D.
  Usan umbrales de detección más permisivos para el recipiente.
- La unidad de velocidad del servo (pasos/s) sale del manual de Feetech; no se
  midió en el brazo.

## Lo que encontró

- **La espera de apertura de la garra era corta.** Abrir de 0° a 140° a 300
  pasos/s lleva ~5,3 s, y `MotionExecutor` esperaba 5 s: la tarea se cortaba en
  Aproximación 2/3 con «M6 no confirmó 140°». Ahora la espera depende de cuánto
  falta abrir (`gripper_open_timeout`, ~8 s desde cerrada). Conviene medir en el
  brazo real cuánto tarda `gripper open` desde cerrada para confirmarlo.
- **Una orden al límite podía trabar la etapa siguiente.** El experto llevó M4 a
  278° (su límite), el servo quedó unas décimas más allá con la carga y la etapa
  siguiente no se podía planificar («Pose inicial insegura: LIMITE»). En el brazo
  real pasa lo mismo con M3, que descansa en 89,8° con límite 90°. Ahora
  `SupervisedExecution` planifica desde `planner.planning_start`, que tolera hasta
  5° fuera del límite como ya hacía la pose de referencia.
- **Ordenar justo al límite traba los `pair`.** Con M4 un poco pasado de 278°,
  el firmware (también el real) rechaza todo `pair` de M2/M3 por la regla de M4.
  La cinemática inversa del experto ahora deja 1° de margen en M1–M5
  (`IK_LIMIT_MARGIN_DEGREES`); la simulación sigue en 100 de 100.
- **El hombro cede con el brazo estirado.** Con el cubo lejos (x ≈ 12–13 cm,
  y ≈ −28 a −30 cm) y agarrado, M2 queda 1,1–1,4° por debajo de la orden por la
  gravedad, y `MotionExecutor` exigía ±1,2°: Traslado 1/3 terminaba en «Sin
  confirmación de llegada». En el SO-100 real es común que el hombro ceda algunos
  grados. Ahora, si no llega a ±1,2°, el ejecutor acepta un motor que quedó quieto
  (variación ≤ 0,3° durante ~0,6 s) a menos de 3° de la orden, y lo anota como
  «Llegada con desvío». Decidido por el usuario el 5/11.

Lotes de 10 escenas (semilla 11): 6/10 con el primer gemelo; 7/10 tras corregir
la tolerancia de partida y el lanzador; **10/10** con el margen de la cinemática
inversa y la llegada con carga (en 2 escenas, M2 llegó a 1,2–1,4°).

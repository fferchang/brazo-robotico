# Simulación física en MuJoCo

Abrir `SIMULACION_MUJOCO.bat`. Se abre el visor de MuJoCo con el brazo, la
mesa, un cubo rojo y el recipiente verde lima, y se ejecutan tres episodios de
la tarea completa con posiciones al azar del cubo y del recipiente. La consola
informa el resultado de cada episodio. Al terminar, la ventana queda abierta
con la física en marcha: se puede girar la cámara y empujar el cubo (doble clic
sobre él y Ctrl + arrastrar con el botón derecho). Cerrarla termina el programa.

No importa el controlador del robot, no abre puertos serie, no usa las cámaras
reales y no modifica demostraciones ni modelos entrenados.

## Opciones

Se agregan al `.bat` o a `simulacion_mujoco.py`:

- `--episodios N`: cantidad de episodios (3 por defecto).
- `--semilla S`: repite las mismas posiciones.
- `--ritmo X`: velocidad del visor respecto del tiempo real (2 = el doble).
- `--sin-visor`: sin ventana, lo más rápido posible (~1 s por episodio).
- `--rerun`: muestra también brazo, cubo y recipiente en Rerun.
- `--fotos`: guarda en `datos/simulacion/` una foto de cada cámara virtual por
  etapa, con los mismos nombres que las demostraciones reales, y un `metadata.json`.
- `--cerrar-al-terminar`: cierra el visor al final en lugar de esperar.

Ejemplo: `SIMULACION_MUJOCO.bat --sin-visor --episodios 100`.

## Qué hace

Cada episodio recorre las doce etapas de las demostraciones, de `inicio` a
`guardado`, con las mismas acciones de garra que la ejecución supervisada. Un
experto calcula cada pose con cinemática inversa (IK) a partir de la posición del
cubo y del recipiente; no usa el modelo de aprendizaje ni las fotos.

- El agarre es desde arriba, como en las 15 demostraciones reales. La IK prefiere
  la garra vertical; si no alcanza, la inclina hacia afuera dentro del plano del
  brazo: hasta 30° cerca del cubo y del recipiente, hasta 70° en tránsito.
- El dedo fijo baja a 3 mm del costado del cubo, con la punta ~22 mm sobre la
  mesa; al cerrar, el dedo móvil empuja el cubo contra el fijo. El dedo móvil
  cierra en arco y toca primero cerca de la arista superior del cubo: con estas
  medidas toca el costado ~2 mm por debajo de la arista. Con 6 mm de holgura la
  tocaba justo en la arista y a veces el "contacto" se detectaba sin agarrar el
  cubo, como podría pasar con el firmware real.
- El cierre imita el firmware: pasos de 1,5° cada 40 ms y contacto con tres
  lecturas seguidas de esfuerzo alto, pasados 250 ms. El umbral de corriente
  real se reemplaza por un par de 0,3 N·m.
- Los motores son servos de posición con el par aproximado del STS3215
  (2,9 N·m). Cada tramo interpola los cinco motores a la vez, a 45°/s.
- Se agregan puntos intermedios para bajar al cubo y subir en línea casi recta,
  y un retiro sobre el recipiente antes de volver al inicio.

Un episodio es exitoso si la garra detecta el cubo, el cubo termina dentro del
recipiente y ninguna pieza del brazo atraviesa otra ni la mesa.

## Restricciones

Las únicas restricciones duras son físicas: que las piezas no se atraviesen entre
sí y que el brazo no atraviese la mesa.

- Autocolisión: cada pieza choca con su envolvente convexa; la garra, con la
  parte de dedo de sus mallas reales. MuJoCo no compara piezas contiguas: para
  ellas valen los topes de cada motor. Al planificar se exigen además 3 mm de
  separación entre piezas; durante la física solo cuenta la interpenetración.
- Topes: lo que permitan `config.json` o el URDF, dentro de 0–360°. Quedan
  M1 60–300°, M2 73,5–274°, M3 90–276°, M4 66,8–278,8°, M5 0–350° y M6 0–140°.
- Mesa: las poses planificadas dejan la garra a más de 5 mm; con la
  calibración de `CALIBRACION.md` el modelo le erra a la mesa por ~2 mm.
  Durante la física se cuenta cualquier contacto entre una pieza del brazo y la
  mesa, salvo el hombro, que se excluye junto con la base porque comparten el
  eje de M1.

Los límites de `config.json` y las reglas del planificador 2D se informan como
avisos ("el brazo real hoy rechazaría esta pose"), pero no bloquean la
simulación. Para ejecutar estos planes en el brazo real habrá que reajustarlas.

Validación con datos reales: en las 180 poses de las 15 demostraciones exitosas
ninguna pieza del modelo atraviesa otra ni queda a menos de 3 mm, y con la
calibración del 30/10 ninguna garra queda a menos de 5 mm de la mesa. Antes de
calibrar, 12 quedaban debajo de la mesa del modelo aunque en la realidad no tocaron.

## Resultados

Lote de 100 episodios (`--sin-visor --episodios 100 --semilla 11`), con las
medidas reales del cubo y de la caja y la calibración del 30/10: 100 exitosos.
La garra sostiene el cubo con M6 en 36,8–39,3°; en las demostraciones, 36–45°.
Cada episodio dura ~25 s simulados y ~1 s de cálculo sin visor.

Con las reglas reajustadas el 2/11 (`REGLAS_GEOMETRIA.md`), ninguna de las
1200 poses clave queda fuera de las reglas del brazo real, y los tramos de los
planes son ejecutables por su planificador. Antes rechazaban 968 (81 %), sobre
todo por M4 > 260° y por un modelo plano que suponía el eje M2 a 100 mm. La IK
planifica dentro de los límites de `config.json`; la física conserva los topes
físicos.

## Diferencias conocidas con el brazo real

- Medidas: cubo de 50 mm; caja de 100 × 100 mm por fuera, paredes de 5 mm y
  50 mm de alto. La masa del cubo (60 g) y el fondo de la caja (5 mm) son
  estimados. Se corrigen en `SceneDimensions` (`brazoia/mujoco_sim.py`).
- Piso y ceros: calibrados el 30/10 con toques de mesa (eje M2 a 121,9 mm,
  ~2 mm de error); faltan los ceros de M3 y M4. Ver `DIAGNOSTICO_PISO_3D.md`.
- Espejo en X corregido el 5/11: M1 estaba invertido en el modelo (las fotos
  reales mostraban el brazo del lado contrario). Ver «M1 invertido» en
  `CALIBRACION.md`. Los resultados anteriores de la simulación siguen valiendo:
  el mundo simulado era el espejo del real, con la misma física.
- Cámaras virtuales: ubicadas a ojo según las fotos reales. Falta calibrarlas
  antes de usar sus imágenes para entrenar.
- El ejecutor real mueve uno o dos motores por tramo (`pair`); la simulación
  interpola los cinco a la vez.
- No modela juego mecánico, flexión, error de encoders ni la protección de
  corriente pendiente de `DIAGNOSTICO_GARRA.md`.

## Alcance

Las fotos de `--fotos` son sintéticas: no entran al entrenamiento ni se mezclan
con `datos/demostraciones/`. Ningún resultado de esta simulación habilita
movimientos del brazo real.

# Calibración de cámaras y modo geométrico

El modo geométrico mueve el brazo sin modelo aprendido. Con la foto de Inicio
calcula dónde están el cubo y el recipiente sobre la mesa, y la cinemática
inversa del modelo 3D (la misma de la simulación MuJoCo) calcula las doce
poses. Cada etapa se sigue confirmando con **CONFIRMAR Y MOVER**, y el
planificador y el firmware siguen validando cada tramo. Pasa por puntos
intermedios en el descenso sobre el cubo, que es vertical, en el empuje del
cubo contra el dedo fijo, a la altura del agarre, y en el retiro final, también
vertical.

Para eso necesita una calibración que diga, para cada lugar de la imagen de
cada cámara, a qué posición de la mesa corresponde. Se arma agarrando el cubo
donde quedó: el modelo 3D dice dónde está la garra. Así la calibración absorbe
también errores suaves del modelo, porque después se piden ángulos parecidos a
los del agarre enseñado.

## Antes de calibrar

- Cámaras fijas. Si se mueve una cámara o cambia su rotación o zoom, hay que
  volver a calibrar. La aplicación rechaza una calibración hecha con otra
  cámara, rotación o zoom, pero no puede detectar que una cámara se movió.
- Nada rojo ni verde lima a la vista de las cámaras, además del cubo y el
  recipiente.
- Pruebas básicas de movimiento hechas: torque, Pose inicial, garra y
  emergencia.

## Captura

En la aplicación principal: abrir las cámaras, conectar el ESP32 y pulsar
**Calibración · cámaras**. La ventana solo lee la escena y la telemetría.

1. Con el brazo en la pose inicial, poner el cubo y el recipiente en lugares
   nuevos, con las caras paralelas al frente de la base, y pulsar **Foto de la
   escena**.
2. Con **Torque brazo OFF (M1–M5)**, llevar a mano la garra abierta alrededor
   del cubo sin moverlo. Cerrarla con **Cerrar garra por presión** y pulsar
   **Registrar agarre del cubo**.
3. Sin soltarlo, llevar el cubo sobre el centro del recipiente, como en
   Soltado, y pulsar **Registrar cubo sobre el recipiente**.
4. **Abrir garra**, volver a la pose inicial y repetir desde 1.

Hacen falta al menos 8 lugares de cada uno, y conviene 15 o más, repartidos
por toda la zona donde van a ir el cubo y el recipiente: cerca y lejos de la
base, a izquierda y derecha, y también a media distancia. Las 17 muestras del
4/11 quedaron en dos filas, a unos 16–20 cm y 27–30 cm hacia adelante de la
base: entre 21 y 26 cm no hay ninguna, y el error ahí no está medido. La
aplicación no ubica objetos a más de 3 cm de la zona calibrada. Cada muestra
se guarda sola en
`datos/calibracion/camaras_capturas_<fecha>.json`, con los ángulos del agarre:
si después cambia la calibración del modelo 3D (ceros de M3/M4, sentido de M1),
la posición en la mesa se puede recalcular sin repetir la captura. **Cargar
capturas…** permite retomar una sesión.

Sin muestras propias, el recipiente usa el mapeo del cubo. En la simulación eso
agrega 10–13 mm de error, frente a 1–5 mm con muestras propias.

## Calcular y guardar

**Calcular calibración** prueba varios mapeos (homografía de cada cámara,
polinomios, combinación de ambas) y elige el de menor error al dejar cada
muestra afuera. Informa la mediana, el 90 % y el máximo en milímetros.

Los mapeos de reproyección ajustan, para cada cámara, dónde se ve cada punto de
la mesa, y ubican el objeto donde las dos imágenes coinciden mejor. Así cada
cámara pesa en la dirección que resuelve bien: con las cámaras reales, la
frontal mueve 1,6 píxeles por milímetro de izquierda a derecha pero solo 0,26
de cerca a lejos (unos 4 mm por píxel), y la lateral, 0,8 píxeles por
milímetro de cerca a lejos. El mapeo compartido suma las muestras del otro
objeto, corridas en la mesa por la diferencia media entre los dos (unos 7 mm
el 4/11). Con las capturas del 4/11, la homografía frontal daba una mediana de
18 mm (90 % ≤ 35 mm); el mapeo compartido da 8 mm (90 % ≤ 16 mm) para el cubo y
8 mm (90 % ≤ 15 mm) para el recipiente.

Como referencia, en la simulación el agarre empujando aguanta unos 20 mm de
error en el cubo y el recipiente al menos 25 mm (ver `--tolerancia` más abajo).
**Guardar como calibración activa** escribe `datos/calibracion/camaras.json`.

## Si cambia el modelo 3D

La posición de cada muestra en la mesa sale del modelo 3D. Si cambian
`assets/calibracion.json`, el URDF o la altura del eje M2 (por ejemplo, al
calibrar los ceros de M3/M4), las posiciones viejas ya no valen: con M1
invertido, el brazo habría ido al lado espejado. Por eso `camaras.json` y cada
archivo de capturas guardan la huella del modelo (`modelo_3d`), y el modo
geométrico rechaza una calibración de otro modelo o sin huella. **Cargar
capturas…** recalcula sola la posición de las capturas con ángulos y descarta
las que no los tienen; después hay que volver a calcular y guardar.

Las capturas del 4/11 se hicieron sin guardar ángulos y con M1 todavía sin
invertir. Se reconstruyeron (`camaras_capturas_20261104_200930_m1_corregido.json`;
las originales quedan en `camaras_capturas_20261104_200930.json`): el modelo
viejo y el nuevo difieren solo en M1, que gira el brazo rígido alrededor de su
eje, y lo que falta saber se completó con lo que pide el procedimiento (cubo
alineado con la base, garra vertical y el giro de M5 más chico desde la pose
inicial). Esa suposición explica mejor las demostraciones de octubre que la de
dejar la muñeca sin girar (12 contra 14 mm de mediana para el cubo). La
calibración activa sale de esas capturas: cubo 6 mm de mediana (90 % ≤ 17 mm)
y recipiente 8 mm (90 % ≤ 15 mm).

El 5/11 se hicieron 16 capturas nuevas con ángulos medidos
(`camaras_capturas_20261105_202522.json`, de 16 a 31 cm de la base). Solas dan
12 mm de mediana dejando cada una afuera: el 6 mm del 4/11 era optimista,
porque salía de ángulos reconstruidos. Las cámaras no se movieron: el mapeo
del 4/11 les erra a las capturas nuevas lo mismo, de 3 a 36 mm. La calibración
activa usa **las dos tandas juntas**
(`camaras_capturas_20261105_juntas.json`; para recalcularla, cargar ese
archivo). Con las demostraciones de octubre como prueba independiente:

| Capturas | Escenas ubicadas | Cubo, mediana (90 %) | Recipiente, mediana (90 %) |
|---|---|---|---|
| Solo 4/11 | 11 de 15 | 10 mm (16 mm) | 7 mm (21 mm) |
| Solo 5/11 | 6 de 15 | 15 mm (20 mm) | 14 mm (26 mm) |
| Juntas | 13 de 15 | 7 mm (18 mm) | 8 mm (27 mm) |

En la simulación, el agarre empujando aguanta 17 mm de error en el cubo (20 de
20) y 22 mm en 15 de 20; el recipiente aguanta 22 mm (20 de 20). Los errores
más grandes del cubo (22–24 mm) aparecen a más de 30 cm de la base, donde se
hicieron las capturas 1 y 3 del 5/11: para las primeras pruebas conviene dejar
el cubo entre 18 y 29 cm de la base.

## Ejecutar

En **Ejecución automática supervisada**, elegir **Geométrico (cámaras
calibradas)**, pulsar **Iniciar tarea** y avanzar etapa por etapa:
**Preparar siguiente etapa**, revisar la propuesta en Rerun y **CONFIRMAR Y
MOVER**. Al preparar la primera etapa, el registro muestra dónde ubicó el cubo
y el recipiente. Tener la mano cerca de **EMERGENCIA · TORQUE OFF**.

## Simulación con cámaras virtuales

`lanzadores/SIMULACION_GEOMETRICA.bat` (`simulacion_geometrica.py`) corre el circuito
completo en MuJoCo: calibra las cámaras virtuales con escenas al azar y prueba
escenas nuevas con física. Opciones: `--muestras`, `--episodios`, `--semilla`,
`--con-recipiente`, `--giro-max`, `--comparar-ideal` y `--tolerancia`.

`--tolerancia 0,5,10,15,20,25` no usa cámaras: el experto planifica con el cubo
(o el recipiente) corrido esa distancia en una dirección al azar, y la física
dice si igual termina en el recipiente. Con semilla 3 y 20 escenas por valor
(5/11, agarre empujando):

| Error | 0 mm | 5 mm | 10 mm | 15 mm | 20 mm | 25 mm |
|---|---|---|---|---|---|---|
| Cubo, empujando | 20 | 20 | 20 | 20 | 19 | 12 |
| Cubo, sin empujar | 20 | 19 | 7 | 8 | 3 | — |
| Recipiente | 20 | 20 | 20 | 20 | 20 | 20 |

Sin empujar, el dedo fijo bajaba a 3 mm del costado del cubo: si el error iba
hacia ese lado, caía encima; si iba hacia el otro, el dedo móvil, que cierra en
arco, lo tocaba en la arista superior. Ni la altura del agarre (16, 22 o 28 mm)
ni la holgura contra el dedo fijo (3, 8 o 13 mm) pasaban de 9 de 15 con 10 mm
de error: el dedo móvil solo cierra bien con el cubo a pocos milímetros del
dedo fijo.

Por eso el agarre ahora empuja (ver `SIMULACION_MUJOCO.md`): el dedo fijo baja
a 18 mm del costado del cubo y avanza a la altura del agarre hasta 15 mm más
allá, arrastrando el cubo hasta apoyarlo contra sí mismo. Con una versión anterior
(12/12 mm), el cubo girado hasta 15° también funcionaba (20 de 20). El planificador del brazo real acepta los 1118
tramos de 40 planes al azar. El empuje nunca acerca el cubo al recipiente.

Con el recipiente cerca del cubo (semilla 21, centros a 80, 95, 110 y 130 mm),
el plan evita el recipiente con 12 mm de margen y revisa el cierre de la garra
cada 8°. Así, la mayoría de los casos que antes fallaban en la física ahora se
rechazan al planificar, con un aviso:

| Centros a | Éxitos | Rechazados | Fallas |
|---|---|---|---|
| 80 mm | 6 | 6 | 2 |
| 95 mm | 6 | 5 | 1 |
| 110 mm | 6 | 1 | 0 |
| 130 mm | 6 | 1 | 0 |

Conviene dejar unos 12 cm entre los centros del cubo y del recipiente.

Resultado de referencia, con semilla 2, 15 escenas de calibración con
recipiente y 12 de prueba: 11 éxitos, error de ubicación de 1–5 mm y una escena
rechazada por quedar fuera de la zona calibrada. Es un mundo ideal: el modelo
3D coincide con el brazo y las cámaras virtuales están ubicadas a ojo. No
reemplaza la calibración del brazo real.

## Límites conocidos

- No estima la rotación del cubo ni del recipiente: tienen que estar alineados
  con la base.
- Con las demostraciones del 26 y 29/10 como muestras, la homografía daba unos
  3–4 cm, pero la reproyección da 10 mm: el problema era el mapeo, no los
  datos. Esas demostraciones sirven de prueba independiente: la calibración
  del 4/11 ubica sus escenas de Inicio con una mediana de 11 mm (90 % ≤ 23 mm)
  para el cubo y 11 mm (90 % ≤ 22 mm) para el recipiente; la homografía que se
  había guardado ese día daba 31 mm (90 % ≤ 53 mm) y 25 mm. Juntar los dos
  conjuntos no mejora (10–12 mm), así que la calibración activa usa solo las
  capturas del 4/11.
- Las cámaras miran casi al ras de la mesa y resuelven mal la profundidad (la
  lateral sola dio 6–7 cm con aquellas demostraciones). Si el error sigue alto,
  subirlas para que miren la mesa desde arriba y recalibrar.

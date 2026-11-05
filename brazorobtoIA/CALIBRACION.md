# Calibración modelo-brazo

Objetivo: que el modelo 3D (Rerun, simulación MuJoCo y planificadores)
coincida con el brazo real antes de llevar a la realidad lo que se planifica en
simulación. Son cuatro pruebas, unos 20 minutos en total. Ninguna mueve el brazo
sola: todo se hace a mano o con los botones habituales.

## Antes de empezar

- Abrir `CALIBRACION.bat`: abre la aplicación principal y la ventana
  **CALIBRACIÓN · SOLO LECTURA**. También se abre desde **Calibración · toques
  de mesa**, en la sección de entrenamiento.
- Conectar el ESP32 y pulsar **Abrir Rerun**: el modelo 3D sigue la telemetría
  real en vivo.
- Despejar la mesa: sin cubo ni caja.
- Tener a mano una hoja de papel y una regla.

## Prueba 1 · ¿El modelo se mueve igual que el brazo? (3 min)

1. Pulsar **Torque brazo OFF (M1–M5)**. M1–M5 quedan libres y M6 no cambia. Sostener
   el brazo: sin torque cae por su peso.
2. Girar la base a mano, despacio, hacia un lado y mirar Rerun: ¿el modelo gira
   hacia el mismo lado? Conviene mirar los dos desde arriba.
3. Repetir de a un motor: hombro (M2), codo (M3), muñeca (M4) y giro de muñeca (M5).
4. Anotar para cada motor: **igual**, **al revés** o **corrido** (sigue el
   movimiento pero con otro ángulo, por ejemplo la muñeca del modelo doblada
   cuando la real está recta).

Lo más importante es M1. En una demostración, las dos cámaras muestran el cubo
del lado contrario al que predice el modelo. Si al girar la base el modelo va
al revés, el modelo está espejado y hay que invertir M1. Rerun recorta los
ángulos a los límites del URDF: cerca de los extremos el modelo puede quedarse
quieto, así que conviene probar en la zona media.

## Prueba 2 · ¿Las cámaras espejan la imagen? (1 min)

Escribir una palabra en una hoja (por ejemplo HOLA) y ponerla frente a cada
cámara. En la vista previa de la aplicación, ¿se lee bien o al revés? Anotar el
resultado de la frontal y de la lateral. Junto con la prueba 1, esto separa un
espejo del modelo de un espejo de las cámaras.

## Prueba 3 · Toques de mesa (10 min)

Estima la altura real de la mesa respecto del modelo y los ceros de M2, M3 y M4.

1. Cerrar la garra sin nada entre los dedos (**Cerrar garra por presión**).
2. Para cada toque:
   1. **Torque brazo OFF (M1–M5)**.
   2. Llevar la garra a mano hasta que la punta toque apenas la mesa, sin hacer
      fuerza.
   3. **Torque ON**: el brazo queda quieto en esa posición, igual que al tomar
      fotos de una demostración.
   4. **Capturar toque** en la ventana de calibración.
3. Hacer al menos 12 toques variados:
   - cerca de la base (~15 cm), a media distancia (~25 cm) y lejos (~33 cm);
   - en cada distancia, a la izquierda y a la derecha;
   - con la garra vertical y con la garra inclinada unos 30°, hacia la base y
     hacia afuera.
4. Pulsar **Calcular ajuste**. La ventana muestra cuántos milímetros le erra hoy
   el modelo a la altura de la garra, la altura real de la mesa (y del eje M2) y
   las correcciones de cero de M2–M4 con su incertidumbre. Si avisa que los
   toques no alcanzan, sumar toques más variados y volver a calcular.

La columna «Garra-mesa según el modelo actual» ya es informativa: en un toque
real debería decir 0 mm. Los toques se guardan solos en
`datos/calibracion/calibracion_<fecha>.json`. Este ajuste no depende de M1, así
que vale aunque la prueba 1 encuentre un espejo.

## Prueba 4 · Garra cerrada (1 min)

1. Sin nada entre los dedos, **Cerrar garra por presión** y esperar a que se
   detenga.
2. Mirar las puntas: ¿se tocan? Si no, medir la separación con la regla.
3. En la ventana, escribir la separación en mm (0 si se tocan) y pulsar
   **Capturar garra cerrada**.

La ventana calcula cuánto está corrido el cero de M6 en el modelo, a partir de
la lectura de M6 y la separación medida. Un corrimiento cambia también el
ángulo con que la simulación sostiene el cubo.

## Después

Pasar los resultados:

- Prueba 1: igual, al revés o corrido para M1–M5.
- Prueba 2: frontal y lateral, bien o espejada.
- El archivo de `datos/calibracion/`, o el texto que muestra la ventana.

Con eso se proponen los cambios en `assets/calibracion.json` (ceros y sentido
de M1) y en la altura del eje M2, y se verifican contra las 180 poses de las
demostraciones y la simulación antes de aplicarlos. Para revisar un archivo
guardado sin abrir la aplicación:
`.venv\Scripts\python.exe calibracion_app.py datos\calibracion\calibracion_<fecha>.json`.

## Resultados del 30 de octubre de 2026

Doce toques (alcance de 8 a 34 cm, garra inclinada hasta 27°) y garra cerrada
con 7 mm entre las puntas. Se aplicó: eje M2 a 121,9 mm de la mesa (antes
100 mm), cero de M2 +1,5° y cero de M6 −5,5°. El error del modelo en los toques
bajó de 17,8 a 2,2 mm RMS. Detalle y validación en `DIAGNOSTICO_PISO_3D.md`.

Pendiente: los ceros de M3 y M4, que piden toques con la garra inclinada 30–45°.

## M1 invertido (5 de noviembre de 2026)

Se invirtió el sentido de M1 en `assets/calibracion.json` (`direction` −1). Sin
el brazo, con tres pruebas sobre datos ya guardados:

- **Fotos.** En el agarre de `prueba_20261029_234857` (M1 = 153,4°), la cámara
  frontal, que mira el brazo de frente, muestra la garra a su derecha; en el
  soltado (M1 = 213°), a la izquierda. La cámara frontal de la simulación, con
  esos mismos ángulos, dibujaba el brazo del lado opuesto; con M1 invertido,
  del mismo. El usuario confirmó el 2/11 que las cámaras no espejan la imagen.
- **Muñeca.** En los 16 agarres a mano de octubre, el rumbo de la garra (módulo
  90°) solo es consistente con M1 o M5 invertido (R = 0,58 contra 0,37; al
  azar, R > 0,42 el 5 % de las veces). Esta prueba no separa M1 de M5; las
  fotos sí.
- **Punta a punta.** Con la calibración de cámaras recalculada, el modo
  geométrico planifica el agarre de 11 escenas de octubre con un M1 a 1,3° de
  mediana (máximo 5°) del que usó la persona.

La calibración con toques de mesa no cambia: mide alturas, y M1 gira alrededor
de un eje vertical. La prueba 1 sigue sirviendo para confirmarlo en el brazo
real y para revisar M2–M5. Las capturas de cámaras del 4/11 se recalcularon
(`CALIBRACION_CAMARAS.md`).

## Alcance

La ventana solo lee la copia de la telemetría que ya recibe la aplicación
principal: no abre puertos, no cambia torque y no envía movimientos. No modifica
`calibracion.json`, `config.json`, el firmware ni las demostraciones. El ajuste
supone una mesa plana y horizontal, y toques sin fuerza; una garra que empuja la
mesa o un brazo que flexiona agregan error.

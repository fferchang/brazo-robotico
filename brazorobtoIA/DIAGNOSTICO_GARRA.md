# Revisión del agarre por corriente — 8 de octubre de 2026

## Correcciones del 2 de noviembre de 2026

Estado de los diez fallos descritos más abajo, en `firmware/firmware_secuencias.ino`,
`brazoia/robot.py` y `brazoia/execution.py`:

| # | Fallo | Estado |
|---|---|---|
| 1 | Vigilancia sólo al cerrar | Corregido: la corriente de M6 se lee cada 40 ms al cerrar y cada 100 ms al abrir o sostener, también tras `torque on` |
| 2 | Lectura fallida deja torque y abandona la vigilancia | Corregido: una lectura fallida no avanza la consigna; tres seguidas desenergizan M6 |
| 3 | Timeout mantiene esfuerzo sin vigilancia | Parcial: sigue sosteniendo la posición, pero vigilada; la sobrecorriente se evalúa antes que el timeout y la aplicación lo trata como fallo |
| 4 | Parada por sobrecorriente no confirmada | Corregido: `EnableTorque(6, 0)` inmediato, sin leer ni escribir antes, hasta 3 intentos; `GRIPPER_FAULT` si el servo no confirma |
| 5 | Demoras de comunicación | Sin cambios; no medido en hardware |
| 6 | La consigna se adelanta al eje | Sin cambios |
| 7 | Pulsaciones repetidas reinician el cierre | Corregido: el firmware responde `GRIPPER_CLOSE_IGNORED` |
| 8 | Estado de interfaz ambiguo | Corregido: `holding` (contacto), `closed` (cerró sin objeto), `stopped` (manual o timeout), `error`, `released` |
| 9 | Torque global OFF no cancela el cierre | Corregido |
| 10 | Confirmación potencialmente antigua | Corregido: el ejecutor exige un estado posterior a su orden y se detiene si la garra cerró sin el cubo |

Al cerrar basta una lectura sobre el umbral duro (200 unidades, ~1.300 mA);
al abrir o sostener hacen falta dos seguidas, para no cortar por un pico de
arranque. El firmware informa `GRIPPER_I raw=<unidades>` cada 200 ms con torque
(en cada lectura al cerrar) y la interfaz muestra la corriente junto al estado de
la garra. Todo lo enviado y recibido queda en `datos/serie/serie_<fecha>.log`,
un archivo por conexión (no se sube al repositorio).

Sin cambios: el registro interno de límite de torque y la protección interna del
servo no se configuran ni se verifican; tampoco se midió la causa del corte de la
fuente de 5 A.

Verificación: `tests/firmware_host/` compila el firmware en el PC con sustitutos
de Arduino y SCServo y recorre 16 casos con corrientes y fallos de bus simulados
(`tests/test_firmware_gripper.py`; se omite sin compilador de C++). No se compiló
para el ESP32 en esta revisión: hay que compilarlo y cargarlo desde el IDE de
Arduino.

### Prueba con el brazo después de cargar el firmware

Con el monitor de la aplicación abierto y la mesa despejada:

1. **Cerrar garra por presión** sin nada entre los dedos: debe terminar en
   `GRIPPER_HOLD reason=closed_limit` y la interfaz mostrar `closed`.
2. Abrir y cerrar sobre el cubo sostenido a mano: `reason=contact`, estado
   `holding`. Anotar los valores de `GRIPPER_I` mientras sostiene durante 60 s.
3. **Abrir garra** varias veces: no debe aparecer `reason=hard_current`. Si
   aparece al abrir sin carga, avisar: el pico de arranque supera el umbral.
4. `gripper release` en el monitor serie: debe decir `result=1`. Si la garra
   queda floja pero dice `result=0`, la biblioteca instalada informa distinto y
   hay que ajustar la confirmación.

## Resultado y alcance

El botón está conectado correctamente: su callback envía una sola línea
`gripper close\n` a través de `App.send_robot` y `RobotSerial.send`. El parser
del firmware reconoce esa línea y llama a `startClosingGripper()` para M6.
No se encontró una orden extra de posición, de otro motor o de torque global
en ese recorrido.

Se ejecutaron ocho pruebas con un puerto simulado en
`tests/test_gripper_diagnostic.py`. Se comprobaron envío, desconexión, bloqueo
durante otro movimiento, recepción de confirmaciones y limitaciones del estado
mostrado. No se abrió el puerto físico, no se movió la garra ni se cargó firmware.
Las pruebas que documentan fallos describen el comportamiento actual; que pasen
no significa que esos fallos estén corregidos.

El usuario confirmó fuente de 12 V, límite máximo de 5 A y servo de garra
variante 12 V / 30 kg. Sigue pendiente saber si el límite salta durante el cierre
o después de sostener, y qué otros motores comparten la fuente en esa prueba.

## Umbrales que realmente implementa el proyecto

- Contacto: 92 unidades, aproximadamente 598 mA usando 6,5 mA/unidad.
  Se requieren tres lecturas consecutivas, después de los primeros 250 ms.
- Sobrecorriente: 200 unidades, aproximadamente 1.300 mA. No espera tres
  muestras ni aplica la exclusión inicial de 250 ms.
- Lectura durante cierre: intervalos mínimos de 40 ms; no son un período
  garantizado porque todas las operaciones del firmware comparten el mismo loop.
- Tiempo máximo de cierre: 15 s.
- `gripper current` permite cambiar el umbral de contacto durante la sesión.
  No se leyó el valor actualmente cargado en el ESP32.

Son umbrales de reacción del ESP32 a una lectura, no un regulador que garantice
un consumo máximo. El código no configura el registro interno de límite de
torque, ni verifica la configuración interna de protección por corriente del
servo. Su configuración real y la versión binaria actualmente en el ESP32 no
se verificaron en esta revisión.

## Fallos y limitaciones comprobados en el código

1. **Vigilancia sólo durante cierre.** `processGripperClosing()` retorna en su
   primera línea si `gripperClosing` es falso. Apertura y mantenimiento de
   posición quedan sin esta supervisión de corriente del ESP32.
2. **Error de lectura deja torque activo.** Si falla `ReadCurrent`, se pone
   `gripperClosing=false` sin ordenar detener la posición ni quitar torque.
   La última consigna puede seguir ejecutándose y ya no se vuelve a vigilar.
   La rama de error de lectura de posición al límite tiene el mismo problema.
3. **Timeout mantiene esfuerzo.** A los 15 s se llama a
   `freezeGripperAtCurrentPosition("timeout", ...)`, que mantiene posición con
   torque, y se abandona la vigilancia. Además, esta rama se evalúa antes que
   la sobrecorriente: si coinciden, el timeout evita la orden de torque OFF.
4. **Parada por sobrecorriente no confirmada.** Antes de apagar M6 se intenta
   leer posición y volver a escribirla. Eso agrega operaciones de bus a la
   ruta de parada. Se ignora el resultado de `EnableTorque(6, 0)`, pero se
   imprime igualmente `GRIPPER_TORQUE_OFF`.
5. **Demoras de comunicación.** La biblioteca instalada usa `IOTimeOut=100`
   ms por espera. La telemetría consulta secuencialmente presencia y posición
   de los seis motores. Ante fallos del bus, las comprobaciones de corriente
   pueden demorarse bastante más que los 40 ms nominales. No se midió el tiempo
   real en hardware.
6. **La consigna avanza sin esperar al eje.** Se restan 1,5° a la consigna en
   cada ciclo de cierre sin comprobar llegada a cada paso. A 40 ms equivale
   nominalmente a 37,5°/s, mientras que `WritePosEx(..., 220, 18)` solicita
   aproximadamente 19,3°/s si la velocidad se expresa en pasos de encoder/s.
   La consigna puede adelantarse al movimiento real. Es una observación del
   control, no una medición de presión ni una prueba de la causa del fallo.
7. **Pulsaciones repetidas reinician el cierre.** El botón vuelve a enviar la
   orden aunque la garra figure cerrando. El firmware reinicia cronómetro y
   contador de contacto; por tanto, también reinicia la exclusión de 250 ms.
8. **Estado de interfaz ambiguo.** Cualquier `GRIPPER_HOLD` se muestra como
   `holding`, incluso `manual`, `closed_limit` o `timeout`. Un
   `GRIPPER_STOP reason=communication_error` registra fallo pero no actualiza
   `gripper_state`: puede seguir mostrando `closing` aunque el firmware haya
   abandonado el cierre.
9. **Torque global OFF no cancela por sí solo el cierre.** Esa rama del parser
   no pone `gripperClosing=false`. La emergencia de la aplicación envía primero
   `gripper stop`, lo que normalmente cancela el cierre; no debe dependerse de
   esa combinación para que una orden de apagado sea definitiva.
10. **Confirmación automática potencialmente antigua.** El ejecutor manda
    cerrar y espera que el estado sea `holding`, sin exigir una confirmación
    recibida después de esa orden. Si ya figuraba `holding`, puede aceptar un
    estado anterior. La secuencia normal suele pasar antes por apertura, pero
    reintentos y acciones manuales pueden exponer esta condición.

## Relación con la fuente de 5 A

La variante de 12 V publicada por Waveshare especifica 2,7 A de corriente de
bloqueo. Si varios motores comparten fuente, su consumo total puede alcanzar
5 A aunque M6 no consuma esa cantidad por sí solo. No se midieron corriente,
caída de tensión, cableado ni estado mecánico, por lo que no se puede atribuir
el evento a una única causa. No hay un registro persistente TX/RX del incidente
en el proyecto; la consola de la aplicación sólo escribe en el widget de texto.

Los 12 V corresponden a la variante que confirmó el usuario. Existe también
una variante de 7,4 V que no debe confundirse con ésta.

## Corrección propuesta antes de repetir con carga

- Supervisar corriente durante apertura, cierre y sujeción; priorizar el
  umbral de sobrecorriente por encima del timeout y de cualquier orden nueva.
- Definir una parada por lectura fallida que no abandone silenciosamente la
  vigilancia; intentar desenergizar M6 e informar si no hay confirmación.
- Configurar y verificar la protección interna apropiada al modelo, además
  de revisar el límite de torque. Un porcentaje de torque no equivale a un
  límite garantizado en amperios.
- Evitar reinicios involuntarios por pulsaciones repetidas y validar respuestas
  de escritura; distinguir contacto, cierre vacío, timeout y error en la app.
- Guardar telemetría de corriente, tensión, marcas de tiempo y respuestas para
  contrastar el corte de la fuente con la decisión del firmware.

Esta revisión no modifica el control de producción ni deja el sistema declarado
listo para repetir el agarre con carga. La corrección y la verificación física
siguen pendientes.

## Fuentes

- Código: `app.py` (`build`, `send_robot`, `log`), `brazoia/robot.py`,
  `brazoia/execution.py`, `firmware/firmware_secuencias.ino`.
- Biblioteca local SCServo: `SMS_STS.cpp` (`ReadCurrent`, `WritePosEx`),
  `SMS_STS.h` (registro `TORQUE_LIMIT` 48–49), `SCSerial.cpp` (timeout),
  `SCS.cpp` (espera de respuestas y estado del servo).
- [Feetech: STS3215, unidades de corriente y protecciones](https://www.feetechrc.com/20210430-56680.html).
- [Feetech: tutorial de configuración de protecciones](https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf).
- [Waveshare: variantes ST3215, tensión y corriente de bloqueo](https://www.waveshare.com/product/modules/motors-servos/motors-servos/st3215-servo.htm).

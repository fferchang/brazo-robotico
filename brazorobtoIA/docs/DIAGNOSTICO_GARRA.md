# Revisión del agarre por corriente — 8 de octubre de 2026

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

## Cambios en el firmware del 6 de noviembre de 2026

Escritos en `firmware/firmware_secuencias.ino` e imitados en el firmware virtual
del gemelo (`brazoia/gemelo.py`). Compilan con arduino-cli (esp32 3.3.2), pero
**no están cargados en el ESP32**: cargarlos necesita el brazo, la app cerrada y
un OK explícito. Las pruebas están en `tests/test_garra_proteccion.py`.

- **Vigilancia al abrir, al mover M6 por ángulo y al sostener** (fallo 1).
  `processGripperWatch()` lee la corriente de M6 cada 100 ms fuera del cierre:
  - 200 unidades (~1,3 A) dos lecturas seguidas: torque OFF en M6 y
    `GRIPPER_STOP_DETAIL reason=hard_current state=…`, que la app trata como
    falla (la tarea se frena). Al abrir o mover se ignoran los primeros 250 ms.
  - Sosteniendo, 170 unidades (~1,1 A) durante 2 s: afloja 1,5° y avisa con
    `GRIPPER_HOLD_RELIEF`, hasta 3 veces; si sigue alta, torque OFF. El gemelo
    sostiene el cubo con ~140, así que no lo afloja. **El valor real sosteniendo
    no se midió**: `gripper telem on` imprime `GRIPPER_TELEM` cada 100 ms para
    medirlo y ajustar el umbral.
  - Al abrir o mover, la vigilancia termina a los 9 s (`GRIPPER_WATCH_END`).
- **Lectura fallida** (fallo 2): en el cierre, y tras 3 lecturas fallidas
  seguidas al vigilar, se intenta quitar el torque de M6
  (`GRIPPER_TORQUE_OFF reason=communication_error result=…`). Si el servo no lo
  confirma, `GRIPPER_ERROR reason=torque_off_sin_confirmar`.
- **Timeout** (fallo 3): la sobrecorriente se revisa antes que el tiempo máximo,
  y después del timeout la garra queda vigilada al sostener.
- **Parada por sobrecorriente** (fallo 4): corta el torque primero, sin escribir
  antes otra posición, e informa el resultado; si no lo confirma, `GRIPPER_ERROR`.
- **Pulsaciones repetidas** (fallo 7): una segunda orden de cierre se ignora
  (`GRIPPER_CLOSE_IGNORED`) y no reinicia cronómetro, contador ni exclusión.
- **Torque global OFF** (fallo 9): cancela el cierre y la vigilancia.
- **Abrir revisa el piso**: `openGripper()` no abre si el dedo móvil barrería la
  mesa (`GRIPPER_ERROR reason=floor`). Ver `REGLAS_GEOMETRIA.md`.

Siguen pendientes los fallos 5 (demoras del bus), 6 (la consigna se adelanta al
eje), 8 (estados ambiguos en la app) y 10 (confirmación de `holding` anterior a la
orden), la protección interna del servo y la telemetría de tensión.

## Fuentes

- Código: `app.py` (`build`, `send_robot`, `log`), `brazoia/robot.py`,
  `brazoia/execution.py`, `firmware/firmware_secuencias.ino`.
- Biblioteca local SCServo: `SMS_STS.cpp` (`ReadCurrent`, `WritePosEx`),
  `SMS_STS.h` (registro `TORQUE_LIMIT` 48–49), `SCSerial.cpp` (timeout),
  `SCS.cpp` (espera de respuestas y estado del servo).
- [Feetech: STS3215, unidades de corriente y protecciones](https://www.feetechrc.com/20210430-56680.html).
- [Feetech: tutorial de configuración de protecciones](https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf).
- [Waveshare: variantes ST3215, tensión y corriente de bloqueo](https://www.waveshare.com/product/modules/motors-servos/motors-servos/st3215-servo.htm).

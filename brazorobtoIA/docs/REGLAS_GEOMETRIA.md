# Reglas geométricas del brazo real

Las aplican dos capas con el mismo modelo: el planificador de la aplicación
(`brazoia/planner.py`) antes de enviar cada tramo, y el firmware del ESP32
(`firmware/firmware_secuencias.ino`) antes de mover. `tests/test_planner.py`
compara las constantes de los dos archivos.

## Reajuste del 2 de noviembre de 2026

Según el usuario, al brazo no le pasa nada mientras las piezas no se atraviesen
y la garra no atraviese la mesa. Las reglas anteriores eran mucho más
restrictivas: rechazaban los 15 agarres de las demostraciones reales y el 81 %
de las poses que planifica la simulación.

- **Modelo plano calibrado.** Las constantes salen del modelo 3D calibrado con
  toques de mesa (`docs/CALIBRACION.md`): eje M2 a 121,9 mm de la mesa (antes 100),
  eslabones de 116,0 / 135,0 / 60,1 mm y ceros angulares medidos. La cadena
  plana coincide con el modelo 3D a menos de 0,05 mm. Antes el antebrazo tenía
  7,7° de error, unos 18 mm de altura al extenderse.
- **Contorno de la garra según M6.** La regla de piso usa el contorno real de la
  garra alrededor del eje de M5, para cualquier giro. Hay cuatro bandas de M6:
  cerrada ≤ 12°, sosteniendo ≤ 47°, entreabierta ≤ 128° y abierta. Al abrirse,
  el dedo móvil sobresale hasta 104 mm del eje; el rectángulo anterior, de
  65 × 100 mm, no lo cubría. Sin lectura de M6 se usa el contorno más
  desfavorable.
- **Un solo límite de piso: 5 mm.** Se eliminó la franja de riesgo de 30 mm
  (`RIESGO_PISO`).
- **Escape.** Si la pose actual ya está bajo el margen, por ejemplo después de
  cerrar la garra sobre el cubo cerca de la mesa, se aceptan movimientos que no
  la acerquen más a la mesa (tolerancia 0,5 mm). Sin esto, tras un agarre bajo
  el brazo no podía levantar.
- **M4 hasta 278°** en `config.json`, el planificador y el firmware. Las
  demostraciones reales llegaron a 277,3°. El tope del URDF es 278,8°.
- Sin cambios: autocolisiones 2D (base y eslabones, con el rectángulo de la
  garra ahora hasta la punta, 106,4 mm), reglas dinámicas de M4 para brazos
  plegados y límites de los demás motores.

## Validación

- Piso: en 1476 poses al azar cerca de la mesa, con cualquier giro de M5 y
  apertura de M6, el contorno 2D nunca queda por encima de la garra 3D (margen
  mínimo 1,1 mm del lado seguro).
- Simulación: los 380 tramos de 20 planes son ejecutables por el planificador
  real. En 100 episodios, ninguna de las 1200 poses clave queda fuera de las reglas.
- Demostraciones reales: 145 de 180 poses pasan. Las 28 de la pose inicial dan
  `LIMITE` porque M3 descansa en 89,8° y el límite es 90°; la aplicación corrige
  la pose actual dentro de 5°. Otras 7 son agarres bajos e inclinados sosteniendo
  el cubo, que el contorno no permite alcanzar, pero sí abandonar.
- Firmware: compila para ESP32 (`arduino-cli compile --fqbn esp32:esp32:esp32`).
  Se cargó en el ESP32 del brazo (COM11) el 4/11/2026; su `help` muestra M4 75..278.

## Pendiente

- Las constantes dependen de `assets/calibracion.json` y de la altura del eje
  M2. Si se recalibra, hay que recalcularlas; `tests/test_planner.py` falla si
  dejan de coincidir con el modelo 3D.
- Abrir o cerrar la garra no revisa el piso: el dedo móvil puede barrer hacia la
  mesa si la garra está horizontal y baja. La simulación sí lo revisa.
- Las autocolisiones 2D siguen siendo aproximadas: no dependen de M5 ni de M6.

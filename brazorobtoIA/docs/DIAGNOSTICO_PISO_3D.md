# Distancia de la garra al piso en Rerun

La vista muestra el plano del piso, el punto más bajo de las mallas de la
garra y su distancia vertical en milímetros. Usa `so100.urdf`, las piezas
`Fixed_Jaw.stl`, `Fixed_Jaw_Motor.stl`, `Moving_Jaw.stl` y la misma conversión
de ángulos de `assets/calibracion.json` que la visualización. Incluye M1–M6.

El plano se coloca 121,9 mm debajo del eje M2 (`SHOULDER_HEIGHT_MM` en
`brazoia/gripper_geometry.py`). Ese valor sale de 12 toques de la garra
sobre la mesa (`docs/CALIBRACION.md`); la medida a mano anterior era 100 mm. Supone
que el brazo está montado verticalmente respecto a una mesa horizontal. No es
equivalente a poner el piso en el origen del archivo URDF.

Verde significa una altura geométrica mayor a 5 mm; rojo, 5 mm o menos.
Son colores de diagnóstico, no una certificación del movimiento. La altura
es la mínima de todos los vértices de ambas mordazas y el motor de la garra,
no la altura del centro de la muñeca. Para un plano horizontal y una malla
triangular, ese mínimo también cubre caras y aristas.

Si Rerun recorta un ángulo por los límites del URDF, se muestra naranja y
`VISTA RECORTADA`: la etiqueta distingue la altura calculada con los ángulos
recibidos de la altura de la geometría dibujada. Con una pose incompleta se
quita la medición anterior. En prueba offline los ángulos recibidos pueden
haber sido recortados antes por el evaluador: consultar sus avisos y tabla.

## Estado al 30 de octubre de 2026

Es una herramienta de diagnóstico. No modifica las validaciones del
planificador ni el firmware, ni abre puertos serie ni habilita motores.
La coincidencia matemática con las transformaciones de la versión instalada
de Rerun está comprobada con pruebas automatizadas.

Antes de la calibración, en las 15 capturas de agarre exitosas guardadas, el
cálculo sin recortes daba entre -5,3 y +7,9 mm; cinco daban valores negativos.
No se recortaba ningún ángulo URDF en esas 15 poses. El usuario informó que
todas superaban 5 mm reales.

## Calibración del 30 de octubre de 2026

Con 12 toques de la garra sobre la mesa y la garra cerrada sin nada
(`docs/CALIBRACION.md`, datos en `datos/calibracion/`) se ajustó:

- altura del eje M2 sobre la mesa: 121,9 mm (antes 100 mm medidos a mano);
- cero de M2: +1,5° (`visual_offset_degrees` 15 → 16,5);
- cero de M6: −5,5° (−12 → −17,5); la garra real cerrada deja 7 mm entre las puntas.

Con esos valores, el modelo le erra a la altura de los toques por 2,2 mm RMS
(antes 17,8 mm) y los 15 agarres guardados quedan entre +10,3 y +25,1 mm,
como informó el usuario: la discrepancia anterior era la altura de la mesa.
La corrección de M2 se confirmó con validación cruzada: en toques no usados
para ajustar, el error baja de 3,2 a 2,7 mm. La de M6 coincide con otra
fuente independiente: la simulación ahora sostiene el cubo de 5 cm con M6 en
37–39° y las demostraciones lo hacían con 36–45°.

Faltan los ceros de M3 y M4, porque los toques fueron con la garra casi
vertical, y la prueba de espejo de M1. El 2/11 el planificador 2D y el firmware
se reajustaron con este modelo (`docs/REGLAS_GEOMETRIA.md`); el firmware nuevo todavía
hay que cargarlo en el ESP32.

Para sustituir la regla anterior también habrá que comprobar el recorrido
entre poses, incluidos los cambios de M5 y la apertura/cierre de M6, con un
margen que contemple error de calibración y seguimiento. El ESP32 contiene
su propia validación geométrica: cambiar solo la vista no cambia esa regla.

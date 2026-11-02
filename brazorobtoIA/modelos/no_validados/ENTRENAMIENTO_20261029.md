# Entrenamiento offline — 29/10/2026

Modelo: `politica_20261029_15demostraciones.npz`.

Resultado: **experimental no validado; no habilitado para ejecución física**.

- 26 carpetas revisadas: 15 demostraciones completas/exitosas y 11 interrumpidas.
- 180 etapas utilizadas, con las 360 imágenes presentes y datos numéricos finitos.
- 78 etapas contienen avisos. Se incluyeron todas: el cargador no filtra por
  reglas geométricas, límites, torque ni estado de garra en los avisos de captura.
- No se editaron muestras, etiquetas de éxito, imágenes ni ángulos registrados.
- 12 demostraciones para ajuste y 3 reservadas para validación, semilla 101.
- Error absoluto medio: **44,0945°**. Peor etapa: **96,6610°** (agarre).
- Límites de validación existentes: 20° promedio y 45° por etapa.

| Etapa | MAE de validación |
|---|---:|
| Inicio | 22,69° |
| Aproximación 1 | 40,17° |
| Aproximación 2 | 32,12° |
| Aproximación 3 | 38,21° |
| Abrir sobre el cubo | 39,22° |
| Bajar con garra abierta | 46,22° |
| Agarre | 96,66° |
| Traslado 1 | 44,09° |
| Traslado 2 | 11,16° |
| Traslado 3 | 93,48° |
| Soltado | 43,39° |
| Guardado | 21,71° |

Estos errores miden las predicciones numéricas M1–M5 antes del recorte a límites,
no una prueba física ni la tasa de éxito de agarres. El entrenamiento usa las
características de detección guardadas, no los píxeles directamente.

El bypass solicitado se limitó a **exportar** el entrenamiento fallido:
`train_policy(..., export_unvalidated=True)`. El archivo tiene
`validation_passed=false` y `experimental_only=true`; `StagePolicy.load` lo
rechaza, incluso si se lo renombra al nombre del modelo activo. Las fuentes y
la partición entrenamiento/validación están registradas en `metadata_json`.

No se modificaron firmware, límites, planificador ni protección de corriente.
Una demostración incluye un aviso histórico de sobrecorriente de la garra;
conservarla para análisis no certifica que esa condición sea segura.

Verificación: exportación y rechazo de carga comprobados. La suite general
tuvo 55 pruebas correctas y una discrepancia ajena al entrenamiento: el test
de configuración espera brillo lateral -15%, mientras el config actual tiene
-25%. Se conservó la configuración del usuario.

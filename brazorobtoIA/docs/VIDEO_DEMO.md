# Video de demostración

Abrir `lanzadores/VIDEO_DEMO.bat`. Graba un video del brazo virtual haciendo la
tarea completa varias veces seguidas y lo abre al terminar. No se abre ninguna
ventana mientras graba: la consola informa cada ronda. El video queda en
`datos/videos/demo_brazo_<fecha>.mp4`, una carpeta que no se sube al repositorio.

Es la misma simulación de `docs/SIMULACION_MUJOCO.md`: la misma escena física,
el experto por cinemática inversa y el agarre empujando. No abre puertos serie, no
usa las cámaras reales y no mueve el brazo real.

## Qué muestra

- Una portada de 4 s con el nombre del brazo y la tarea.
- Seis rondas. En cada una el cubo y el recipiente cambian de lugar al azar, dentro
  de la zona de las 15 demostraciones reales.
- Arriba a la izquierda, la etapa en curso con palabras simples («Agarrando el
  cubo», «Llevando el cubo al recipiente»…) y una barra con las doce etapas.
- Arriba a la derecha, la ronda y cuántas veces quedó el cubo en el recipiente.
- Al final de cada ronda, un cartel verde si el cubo quedó adentro. Si falla,
  el cartel es rojo y dice por qué: el video no esconde las fallas.
- Un cierre de 4,5 s con el resultado de todas las rondas.

La cámara gira despacio de un lado al otro frente al brazo y se aleja un poco
cuando el brazo se estira. Entre una ronda y la siguiente hay un fundido a negro.

## Tiempos y tamaño

Por defecto el video va a 1,5 veces la velocidad real, y el pie de imagen lo dice.
Seis rondas dan unos 2 min de video y tardan unos 2 min y medio en grabarse.
La notebook tiene que quedar despierta y con la pantalla encendida mientras graba:
con la PC suspendida, una prueba de dos rondas tardó más de una hora.

El formato normal es H.264, que se ve en cualquier reproductor, en PowerPoint y en
el celular, pero pesa ~2,8 MB por segundo a 720p (unos 350 MB las seis rondas),
porque el codificador de Windows que usa OpenCV no deja bajar la calidad.
`--liviano` usa MPEG-4: pesa unas 8 veces menos (unos 45 MB), se ve algo más
borroso y los navegadores no lo abren; el reproductor de Windows sí.

## Opciones

Se agregan al `.bat` o a `video_demo.py`:

- `--rondas N`: cantidad de rondas (6 por defecto).
- `--semilla S`: repite las mismas posiciones. La consola muestra la semilla de cada
  video para poder repetir uno que haya salido bien.
- `--ritmo X`: velocidad respecto del tiempo real (1,5 por defecto). Con múltiplos
  de 0,5 los cuadros quedan parejos.
- `--fullhd`: 1920 × 1080 en lugar de 1280 × 720 (pesa el doble).
- `--liviano`: MPEG-4 en lugar de H.264.
- `--salida archivo.mp4`: otro nombre o carpeta.

Ejemplo: `lanzadores/VIDEO_DEMO.bat --rondas 10 --liviano`.

## Para exponerlo

Es una simulación, no el brazo real: conviene presentarlo como lo que el brazo
hace en el entorno virtual, con la misma lógica que usa la tarea real. La tasa de
éxito del video no es la del brazo real. En el reproductor de Windows se puede
poner en repetición para que corra sin parar.

Solo cambia la imagen, no la física: la mesa se dibuja más grande y con más luz.

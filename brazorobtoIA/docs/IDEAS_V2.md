# Ideas para una v2

Lista viva de lo que se podría agregar o mejorar en una próxima versión del brazo.
Cada idea dice qué es, por qué ayudaría (con lo que se vio en este proyecto, cuando
hay datos) y cuánto trabajo lleva más o menos. Para agregar una idea, sumala en la
sección que corresponda con el mismo formato. Las marcadas con ★ son las que más
cambiarían el resultado.

Esfuerzo: **bajo** (horas, sin comprar nada), **medio** (días o una compra chica),
**alto** (semanas o un cambio grande de hardware o de arquitectura).

## El objetivo: pocas cámaras en vez de muchos sensores

La idea de fondo del proyecto es **reemplazar la gran cantidad de sensores que hoy
necesita una tarea automatizada por una cantidad mínima de cámaras**, y que el brazo
se reconfigure **solo por software**.

El caso que pensamos es una planta fabril: un brazo que pasa un producto de una cinta
transportadora a otra. Hoy eso se resuelve con una celda armada para un solo producto,
con varios tipos de sensores:
- de presencia, para saber que llegó el producto;
- de posición o fines de carrera, para saber dónde está;
- guías mecánicas que lo dejan siempre en el mismo lugar;
- a veces lectores o sensores de color, para distinguir productos.

Si cambia el producto, su tamaño o la disposición de las cintas, hay que mover
sensores, cambiar guías y reprogramar.

Con cámaras, la misma imagen responde todas esas preguntas a la vez: si hay un objeto,
dónde está, cómo está girado, qué es y si quedó bien puesto. Cambiar de producto o de
lugar pasa a ser cambiar la calibración, el detector o la tarea en el software, sin
tocar la instalación. Este proyecto ya lo muestra en chico:
- las mismas dos cámaras sirven para la tarea con recipiente y para la demo entre dos
  puntos de la mesa;
- en la demo, si alguien mueve el cubo, la vuelta siguiente lo encuentra solo;
- la verificación final (si el cubo quedó dentro del recipiente) también es por imagen,
  sin un sensor en el recipiente.

### Lo que falta para que sirva en una planta
Es una dirección que la industria ya usa (robots guiados por visión), así que el
aporte de este enfoque está en hacerlo con hardware barato y reconfigurable por
software. Para pasar de la maqueta a una línea real faltan cosas concretas:
- **Tiempo de ciclo.** Una cinta necesita ciclos de pocos segundos. La demo hoy tarda
  ~39 s por vuelta en el gemelo (antes ~88 s). La «etapa 2» (trayectorias continuas,
  ver la sección 2) y motores más rápidos son el camino.
- **Agarrar en movimiento.** En una cinta el producto no se queda quieto: hay que medir
  la velocidad de la cinta (con las cámaras o con un encoder) y predecir dónde va a
  estar al bajar la garra. Hoy la tarea supone el objeto quieto desde la foto de Inicio.
- **Confiabilidad medida.** En planta importa el porcentaje de éxitos en miles de
  ciclos, no en decenas. Los registros de cada ejecución ya existen: falta medir tandas
  largas y atacar las fallas más comunes (ver «Panel de estadísticas»).
- **Robustez de la visión:** luz controlada, soportes fijos y recalibración rápida con
  marcadores (sección 1). Una cámara que se mueve o una luz que cambia es el equivalente
  a un sensor descalibrado.
- **La seguridad no se reemplaza con cámaras.** Las paradas de emergencia, las
  cortinas de luz y los resguardos de la celda tienen que seguir siendo hardware
  certificado (sección 5). Las cámaras reemplazan los sensores *de la tarea*, no los
  de seguridad de las personas.
- **Recetas por producto.** Guardar por producto su detector, su apertura de garra, su
  altura de agarre y los puntos de origen y destino, para cambiar de producto eligiendo
  una receta en la interfaz.

## 1. Visión

### ★ Cámaras de mejor calidad
Hoy son dos webcams USB genéricas («USB CAMERA» y «GENERAL WEBCAM»). Una cámara con
más resolución, mejor óptica y **exposición y balance de blancos fijos** (que no se
ajusten solos) da bordes más nítidos y colores estables. La detección es por color
(HSV), así que cualquier cambio automático de la cámara mueve los umbrales.
- Dato del proyecto: el cubo se ubica con ~7 mm de mediana y el recipiente con ~8 mm.
  El agarre empujando aguanta ~20 mm de error, pero en la demo el cubo quedó a
  5–13 mm del destino: con mejor imagen ese margen crece.
- Esfuerzo: medio (compra + recalibrar con «Calibración · cámaras»).

### ★ Tercera cámara en el otro lateral y una cuarta desde arriba
- **Otro lateral**: cubre los puntos ciegos que deja el propio brazo. Hoy, con el brazo
  en ciertas poses, una cámara deja de ver el cubo; con tres, siempre quedan dos.
- **Desde arriba (cenital)**: ve la mesa como un plano, casi sin perspectiva. Es la
  mejor vista para medir x/y y, sobre todo, **el giro del cubo** (hoy se supone
  alineado con la base: `cube_yaw = 0`).
- Con más de dos vistas se puede **triangular** de verdad (ver la idea siguiente) y
  descartar una cámara que ve mal.
- Esfuerzo: medio. El código hoy asume dos cámaras (`CAMERAS = ("frontal", "lateral")`,
  las 32 características de `observation_features`, la calibración y la interfaz).
  Pasar a N cámaras es un cambio prolijo pero sin misterio.

### Calibración de cámaras con marcadores (ArUco / tablero)
Hoy cada cámara se calibra con muestras del cubo en la mesa y un mapeo píxel → mesa.
Con un tablero de ajedrez o marcadores ArUco pegados en la mesa se obtienen la lente
(distorsión) y la posición 3D exacta de cada cámara. Eso permite triangular en 3D con
cualquier cantidad de cámaras y recalibrar en segundos si alguien mueve una.
- Esfuerzo: medio.

### Soportes fijos para las cámaras
Si una cámara se mueve un poco, la calibración deja de valer (la app ya lo detecta
por rotación y zoom, pero no por un golpe al trípode). Un soporte atornillado a la
mesa o a un marco evita recalibrar seguido.
- Esfuerzo: bajo.

### Iluminación pareja
Luz difusa fija (tira LED con difusor, o dos paneles) en vez de la luz del ambiente.
La detección por color depende mucho de la luz. Con luz pareja los umbrales del rojo
y del verde lima andan todo el día.
- Esfuerzo: bajo.

### ★ Cámara en la muñeca («eye-in-hand»)
Una cámara chica en la garra mira el cubo de cerca justo antes de bajar y corrige los
últimos milímetros. Las cámaras fijas ubican el cubo y la de la muñeca afina. Es la
forma más directa de pasar de ~7 mm de error a ~1–2 mm.
- Esfuerzo: medio/alto (soporte, cable, calibración mano-cámara y un paso más en la tarea).

### Cámara de profundidad
Una cámara RGB-D (tipo Intel RealSense u OAK-D) da la distancia de cada píxel: altura y
forma de los objetos sin depender del color ni de triangular entre cámaras.
- Esfuerzo: medio.

### Detección de objetos con aprendizaje (no solo por color)
Hoy solo se reconocen un cubo rojo y un recipiente verde lima. Un detector entrenado
(por ejemplo, tipo YOLO) reconoce otros objetos y formas, y además entrega la caja
girada, o sea el giro del objeto.
- Esfuerzo: alto (fotos etiquetadas y entrenamiento).

## 2. Movimiento, motores y alimentación

### ★ Fuente o driver que entregue más corriente
Para mover los seis servos a la vez y más rápido hace falta una fuente que aguante el
pico de todos juntos con carga. Cada STS3215 consume bastante al arrancar o al
sostener peso. Con una fuente justa, mover varios a la vez puede bajar la tensión y
hacer que un servo se reinicie o pierda fuerza.
- Aclaración: el límite actual de **dos motores por orden** no es de corriente: lo
  impone el firmware, que revisa las reglas de seguridad de a pares. La librería de
  los servos ya permite mandar varios a la vez (el firmware la usa para los pares).
  Para el movimiento simultáneo hacen falta las dos cosas: el firmware nuevo (ver
  «Trayectorias continuas») y una alimentación que lo banque.
- La velocidad tampoco está al máximo: el firmware pide ~40°/s (`MOVE_SPEED = 450`),
  muy por debajo de lo que da el servo. Más corriente permite subirla sin que el
  brazo se caiga por su peso al acelerar.
- Esfuerzo: medio (fuente de 12 V con margen, capacitores cerca de los servos y
  cables más gruesos).

### ★ Trayectorias continuas desde el firmware
Es la «etapa 2» de la vuelta más rápida. Hace falta:
- un comando que mueva los cinco motores juntos y sincronizados;
- que la PC mande la trayectoria entera, con velocidad y aceleración suaves;
- que el firmware revise la trayectoria completa en vez de cada par.

Con la etapa 1 la vuelta de la demo bajó de ~88 s a ~39 s en el gemelo. Lo que queda
lento son los movimientos grandes (salir de Inicio y volver), que van «en escalera».
- Esfuerzo: alto (firmware, planificador y gemelo).

### Comunicación más rápida con el ESP32
Hoy los comandos y la telemetría van como texto a 115200 baudios, con las posiciones
cada 50 ms. Un protocolo binario y más velocidad dan lecturas más frecuentes: las
llegadas se confirman antes y el control es más fino.
- Esfuerzo: medio.

### Leer temperatura, tensión y carga de cada servo
Los STS3215 informan temperatura, tensión y carga. En una demo en loop de varias horas
conviene vigilarlos y pausar si un motor se calienta (sobre todo la garra, que sostiene
apretando).
- Esfuerzo: bajo/medio.

## 3. Garra

### Sensor de agarre
Hoy el cierre por presión se detecta por la corriente del servo. Si la garra se cierra
del todo sin encontrar nada («closed_limit»), la tarea lo toma igual como cerrada. Un
sensor de fuerza en los dedos, un fin de carrera o simplemente usar el motivo del
cierre para distinguir «agarró» de «se cerró vacía» evita seguir con el cubo perdido.
- Esfuerzo: bajo (por software, con el motivo del cierre) a medio (sensor).

### Dedos con goma o almohadillas
Más fricción: el cubo no se resbala al levantarlo ni se gira al trasladarlo. En la
simulación, los agarres que fallan son casos donde el cubo no sube con la garra.
- Esfuerzo: bajo.

### ★ Dedos de TPU flexible que se amolden al objeto
Imprimir los dedos (o al menos su parte de contacto) en TPU, un plástico flexible, en
vez del material rígido actual. Un dedo así se deforma y abraza el objeto: lo agarra
con más superficie, lo trata con más suavidad (no marca ni aprieta de más objetos
delicados) y tolera mejor un error de posición o un objeto girado. Con un diseño tipo
«Fin Ray» (un dedo con costillas internas que se curva hacia el objeto al apretarlo)
el efecto es mayor que con un dedo macizo.
- Qué hay que revisar al cambiarlo:
  - El cierre por presión se detecta por la corriente de M6 (umbral `contact_raw=92`).
    Con un dedo blando la corriente sube más despacio y el cierre termina con otro
    ángulo (hoy ~40° con el cubo): hay que volver a medir con `gripper telem on` y
    ajustar el umbral y la protección de corriente nueva.
  - En el agarre empujando, el dedo fijo arrastra el cubo 15 mm antes de cerrar. Si es
    muy blando se dobla en vez de empujar: conviene una base rígida con la cara de
    contacto en TPU, o un TPU duro (Shore 95A).
  - El modelo 3D (MuJoCo y las reglas de piso) supone dedos rígidos con la forma
    actual: si cambian la forma o el largo, hay que actualizar la malla del dedo.
- Esfuerzo: medio (diseño e impresión, y después recalibrar el cierre).

### Garra intercambiable o adaptativa
Dedos con forma para otros objetos (cilindros, piezas chicas), o una garra de dedos
blandos que se adapte sola.
- Esfuerzo: medio.

## 4. Mecánica y montaje

### Base fija y mesa rígida
La calibración de la altura de mesa no se repetía entre días (M2/mesa no generaliza).
Si la base está atornillada a una mesa rígida, esa altura queda fija y el modelo 3D le
erra menos al piso.
- Esfuerzo: bajo.

### Menos juego y flexión en el brazo
Con carga, el hombro cede por el peso (en el gemelo, M2 llega 1,1–1,4° antes; por eso
existe «Llegada con desvío»). Piezas más rígidas o impresas con más relleno, y
rodamientos donde haya juego, mejoran la precisión con el brazo estirado (hoy el
agarre falla más allá de ~30 cm).
- Esfuerzo: medio.

## 5. Seguridad

### Parada de emergencia física
Un botón tipo hongo que corte la alimentación de los servos, independiente de la PC y
del USB. Hoy EMERGENCIA es un botón de la app que manda «torque off» por USB: si la PC
se cuelga o el cable se suelta, no llega.
- Esfuerzo: bajo.

### Zona protegida para exposiciones
Una cortina de luz o un sensor de presencia que frene el brazo si alguien mete la mano
mientras corre la demo en loop.
- Esfuerzo: medio.

## 6. Software y aprendizaje

### Brazo líder para enseñar (teleoperación)
El SO-100 tiene un brazo «líder» (otro brazo sin fuerza que se mueve a mano y el
seguidor lo copia). Enseñar así es mucho más rápido y natural que posar las 12 etapas
y sacar fotos, y da trayectorias completas en vez de 12 poses.
- Esfuerzo: medio (armar el líder y adaptar la grabación).

### Políticas aprendidas modernas (ACT / diffusion, como en LeRobot)
Con demostraciones por teleoperación se pueden entrenar políticas que aprenden el
movimiento entero a partir de las imágenes, no solo 12 poses. Necesita más
demostraciones y, para entrenar en tiempo razonable, una GPU.
- Esfuerzo: alto.

### PC con GPU NVIDIA
Hoy la notebook (i5 con Iris Xe) no corre Isaac Sim/Lab ni entrena modelos grandes.
Con una GPU se puede simular con imágenes realistas, variar luz y colores en la
simulación (domain randomization) y entrenar políticas visuales.
- Esfuerzo: medio (compra).

### Varias tareas y objetos
Elegir qué objeto llevar y adónde desde la interfaz: ordenar por color, apilar,
llevar a varios recipientes. La demo entre dos puntos ya es un primer paso.
- Esfuerzo: medio.

### Panel de estadísticas
Cada ejecución ya queda registrada con fotos. Un panel con el porcentaje de éxitos, el
error de llegada, el tiempo por etapa y los bloqueos más comunes muestra qué conviene
mejorar primero.
- Esfuerzo: bajo.

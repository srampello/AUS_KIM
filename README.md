# AUS_KIM

Proyecto para programar desde cero el robot de laberinto prestado **AUS_KIM**, basado en ESP32-S3.

## Hardware actual

- ESP32-S3 SuperMini
- 4x Sharp GP2Y0E03
- DRV8833
- 2x micro motorreductores metalicos con encoder de cuadratura
- BEC / Step-Down BLITZ 5V/12V/3A
- LiPo CNHL Pizza V2 2S, 7.4 V nominales, 250 mAh

## Etapa 01 - Diagnostico Wi-Fi

La interfaz web permite probar motores, encoders y sensores de forma independiente.

### Red

- SSID: `AUS_KIM`
- Clave: `AUSKIM2026`
- Panel: `http://192.168.4.1`

### Funciones actuales

- Control independiente de motor izquierdo y derecho.
- PWM ajustable de 0 a 255.
- ADELANTE / ATRAS mientras se mantiene presionado.
- STOP general y fail-safe.
- Lectura de encoders con canales A/B.
- Correccion izquierda/derecha de encoders solo en la interfaz.
- Seleccion manual de cual Sharp leer.
- Los otros Sharp dejan de mostrarse, pero siguen alimentados fisicamente.
- Opcion para detener la lectura de sensores.

## Pinout

### Sharp - Vout

| Sensor fisico | GPIO |
|---|---:|
| Frontal izquierdo | 2 |
| Frontal derecho | 1 |
| Lateral izquierdo | 4 |
| Lateral derecho | 3 |

### Sharp - cableado usado

Para cada GP2Y0E03:

- VDD -> 3.3 V
- Vout -> GPIO analogico correspondiente
- GND -> GND
- VIN(IO) -> 3.3 V
- GPIO1 / Pin 5 -> **sin conectar**
- SCL -> sin conectar
- SDA -> sin conectar

No se utilizan GPIO 15, 16, 17 ni 18 para los sensores.

### Motores

| Funcion fisica | GPIO |
|---|---:|
| DRV8833 IN3 motor izquierdo | 7 |
| DRV8833 IN4 motor izquierdo | 8 |
| DRV8833 IN1 motor derecho | 5 |
| DRV8833 IN2 motor derecho | 6 |

El motor fisico izquierdo requiere inversion de sentido por software.

### Encoders - cableado sin cambios

| Conexion | GPIO |
|---|---:|
| Encoder A primer par | 9 |
| Encoder B primer par | 10 |
| Encoder A segundo par | 11 |
| Encoder B segundo par | 12 |

La interfaz intercambia solamente la presentacion izquierda/derecha de esos datos.

## Seleccion manual de sensores

En el panel aparecen cuatro botones:

```text
[ SELECCIONAR Frontal izquierdo ]   [ SELECCIONAR Frontal derecho ]
[ SELECCIONAR Lateral izquierdo ]   [ SELECCIONAR Lateral derecho ]

                    [ DETENER LECTURA ]
```

Cuando se selecciona uno, el ESP32 ejecuta la lectura ADC solamente de ese canal y la interfaz muestra unicamente ese valor.

Esto **no apaga fisicamente los otros sensores**, porque el Pin 5 (GPIO1) de los Sharp no esta conectado.

## ADC y filtro de sensores

El firmware usa resolucion de 12 bits y configura atenuacion de 11 dB en los cuatro pines analogicos:

```cpp
analogReadResolution(12);
analogSetPinAttenuation(..., ADC_11db);
```

Para reducir ruido y picos se agrego un filtro de dos etapas:

1. **Mediana de 9 muestras** para rechazar lecturas aisladas.
2. **Suavizado exponencial** con 75% del valor anterior y 25% de la nueva mediana.

La interfaz muestra:

- valor grande: ADC filtrado;
- valor `Crudo`: mediana instantanea antes del suavizado.

Como cada celda del laberinto tiene un maximo de **25 x 25 cm**, durante la proxima etapa de calibracion vamos a convertir ADC a distancia y considerar cualquier lectura mayor a 25 cm como **sin pared cercana**. No se aplica todavia un corte por distancia porque primero necesitamos medir la curva real de estos sensores.

## Estructura

```text
AUS_KIM/
├── firmware/
│   └── 01_wifi_test/
│       └── AUS_KIM_WIFI_TEST.ino
├── docs/
│   └── conexiones.md
└── README.md
```

## Actualizar la copia local

```bash
git pull origin master
```

## Proximo paso

1. Validar los cuatro canales Sharp individualmente.
2. Confirmar la visualizacion correcta de ambos encoders.
3. Calibrar los Sharp con distancias conocidas.
4. Medir velocidad de ruedas.
5. Implementar PID.
6. Movimiento recto y giros.
7. Seguimiento de pared.
8. Navegacion del laberinto.


## Etapa 02 - Seguimiento de pared derecha

Se agrego:

```text
firmware/02_wall_follow/AUS_KIM_WALL_FOLLOW.ino
```

Esta version:

- usa solamente el Sharp lateral derecho;
- no usa encoders;
- mantiene una referencia inicial de ~6 cm (`ADC = 2400`);
- ejecuta el PID a 100 Hz;
- permite modificar desde la interfaz web `Kp`, `Ki`, `Kd`, objetivo ADC, PWM base, correccion maxima y umbral minimo de pared;
- muestra ADC filtrado, mediana, error, correccion y PWM de ambos motores;
- incorpora START, STOP y fail-safe por perdida de comunicacion web.

Red Wi-Fi:

- SSID: `AUS_KIM_WALL`
- Clave: `AUSKIM2026`
- Panel: `http://192.168.4.1`

Valores iniciales de prueba:

```text
Kp              = 0.12
Ki              = 0.00
Kd              = 0.35
Objetivo ADC    = 2400
PWM base        = 80
Correccion max. = 60
Pared minima    = 1500
```

La mezcla de motores es:

```text
Motor izquierdo = PWM base - correccion
Motor derecho   = PWM base + correccion
```

Como ADC alto significa pared mas cercana, una correccion positiva aleja el robot de la pared derecha.


## Etapa 04 - Maze Solver

Se agrego:

```text
firmware/04_maze_solver/AUS_KIM_MAZE_SOLVER.ino
```

Esta version implementa una navegacion tipo **regla de la mano derecha**:

1. Si detecta una apertura a la derecha, la prioriza.
2. Si encuentra pared frontal:
   - gira a la izquierda si ese lateral esta libre;
   - si tambien esta bloqueado, realiza un giro de 180 grados.
3. En recta sigue la pared derecha con PID.

Los cuatro sensores Sharp se leen en tiempo real y la interfaz muestra:

- frontal izquierdo y derecho;
- lateral izquierdo y derecho;
- ADC filtrado y crudo;
- deteccion de pared/apertura;
- estado actual del robot;
- error y correccion PID;
- PWM firmado de ambos motores;
- telemetria de ambos encoders.

Los giros son temporizados porque el encoder derecho todavia no es confiable. Desde la interfaz pueden ajustarse PID, velocidad base, correccion maxima, umbrales de sensores, PWM de giro y todos los tiempos de maniobra.

Red:

```text
SSID: AUS_KIM_MAZE
Clave: AUSKIM2026
Panel: http://192.168.4.1
```


## Etapa 05 - Test Suite unificado

Se agrego:

```text
firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino
```

Esta version concentra todas las pruebas principales en una sola interfaz web con tres pestañas:

1. **Sensores / Motores / Encoders**
   - lectura en vivo de los 4 Sharp;
   - valor filtrado y crudo;
   - prueba manual independiente de cada motor;
   - PWM manual;
   - lectura de encoders y estados A/B;
   - reset de encoders.

2. **PID pared derecha**
   - seguimiento de pared derecha;
   - ajuste en vivo de Kp, Ki y Kd;
   - objetivo ADC;
   - PWM base;
   - correccion maxima;
   - telemetria de sensores, motores, error y correccion.

3. **Resolver laberinto**
   - regla de la mano derecha;
   - deteccion frontal;
   - deteccion de aperturas laterales;
   - giro derecha, izquierda y 180 grados;
   - ajuste en vivo de umbrales, PWM y tiempos de maniobra;
   - telemetria completa.

Los encoders siguen siendo solo de diagnostico y no intervienen en el PID ni en los giros.

Red Wi-Fi:

```text
SSID: AUS_KIM_TEST
Clave: AUSKIM2026
Panel: http://192.168.4.1
```

Al cambiar de pestaña/modo, el firmware detiene los motores por seguridad. Tambien mantiene un fail-safe de comunicacion web.

### Logo RMP separado del firmware

El logo se guarda como `rmp_logo.h` junto al sketch, usando Base64 en memoria de programa (`PROGMEM`). Esto elimina las miles de lineas de imagen incrustada del `.ino` sin perder el funcionamiento offline.

```text
firmware/
├── 05_test_suite/AUS_KIM_TEST_SUITE/
│   ├── AUS_KIM_TEST_SUITE.ino
│   └── rmp_logo.h             # Fuente del logo
└── 06_race/AUS_KIM_RACE/
    ├── AUS_KIM_RACE.ino       # Generado automaticamente
    └── rmp_logo.h             # Copia sincronizada
```

Arduino requiere que cada archivo `.ino` este dentro de una carpeta con el mismo nombre. El archivo `.h` tiene que permanecer al lado del `.ino` correspondiente. La pagina solicita el logo al propio ESP32 mediante `/api/rmp-logo-b64`, sin necesitar Internet.

Si se cambia el logo del Test Suite, GitHub Actions sincroniza tambien `rmp_logo.h` de carrera. El logo NO se edita directamente dentro de los archivos generados.

## Etapa 06 - Race Control RMP (largada)

Dos interfaces, **un solo codigo de navegacion**:

- Test Suite: `firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino`
  - `http://192.168.4.1/`: interfaz de calibracion y pruebas.
  - `http://192.168.4.1/race`: interfaz sencilla de carrera RMP.
- Race: `firmware/06_race/AUS_KIM_RACE/AUS_KIM_RACE.ino`
  - `http://192.168.4.1/`: interfaz de carrera RMP como pantalla principal.
  - El boton circular central con logo **RMP** inicia `MODE_MAZE`.
  - El boton **STOP** detiene los motores.

En ambos firmwares se usa la misma red `AUS_KIM_TEST`, clave `AUSKIM2026`.

### Sincronizacion automatica

**Siempre modificar el Test Suite**: es la unica fuente del programa. El firmware Race es generado y NO se debe editar a mano.

Cada push que modifique el `.ino` del Test Suite ejecuta el workflow:
`.github/workflows/sincronizar_race.yml`.

El workflow regenera el `.ino` de carrera mediante `tools/generar_aus_kim_race.py` y guarda los cambios automaticamente en GitHub.

Tambien se puede regenerar localmente desde la raiz del repositorio:

```bash
python tools/generar_aus_kim_race.py
```

Esto asegura que Race tenga el ultimo PID de pared, deteccion de sensores, manejo de encoders, giros y calibraciones **definidas en el codigo** del Test Suite. Los ajustes hechos solo en los formularios web son temporales en RAM: para pasarlos al firmware de carrera hay que actualizar sus valores iniciales en el Test Suite y subir el cambio a GitHub. Generar o actualizar el archivo de GitHub NO reprograma automaticamente el ESP32-S3: volver a cargar el `.ino` de carrera cuando se modifique el programa.

**Seguridad:** el firmware conserva el boton STOP y el corte automatico si pierde la conexion con la pagina web; mantener la pagina abierta durante la carrera.

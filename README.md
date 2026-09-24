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

## ADC

El firmware usa resolucion de 12 bits y configura atenuacion de 11 dB en los cuatro pines analogicos:

```cpp
analogReadResolution(12);
analogSetPinAttenuation(..., ADC_11db);
```

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

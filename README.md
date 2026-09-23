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
- Correccion izquierda/derecha de encoders **solo en la interfaz**.
- Seleccion manual de los Sharp.
- Solo un sensor Sharp puede permanecer activo a la vez.
- Boton para apagar todos los sensores.

## Pinout

### Sharp - Vout

| Sensor fisico | GPIO |
|---|---:|
| Frontal izquierdo | 2 |
| Frontal derecho | 1 |
| Lateral izquierdo | 4 |
| Lateral derecho | 3 |

### Sharp - Pin 5 GPIO1 / Enable

| Sensor | GPIO ESP32-S3 |
|---|---:|
| Frontal izquierdo | 15 |
| Frontal derecho | 16 |
| Lateral izquierdo | 17 |
| Lateral derecho | 18 |

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
| Encoder A del primer par | 9 |
| Encoder B del primer par | 10 |
| Encoder A del segundo par | 11 |
| Encoder B del segundo par | 12 |

La interfaz intercambia solamente la presentacion izquierda/derecha de esos datos. No se modifico el cableado.

## Seleccion manual de sensores

Al iniciar, los cuatro Sharp quedan apagados. En el panel aparecen cuatro botones:

```text
[ ACTIVAR Frontal izquierdo ]   [ ACTIVAR Frontal derecho ]
[ ACTIVAR Lateral izquierdo ]   [ ACTIVAR Lateral derecho ]

              [ APAGAR SENSORES ]
```

Cuando se pulsa un sensor:

1. se apagan los cuatro,
2. se activa solamente el seleccionado,
3. se espera el tiempo de estabilizacion,
4. se muestra y actualiza unicamente esa lectura.

Para esto, el Pin 5 (GPIO1) de cada GP2Y0E03 debe estar conectado a GPIO 15, 16, 17 y 18 del ESP32-S3.

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

1. Validar los cuatro botones de sensores.
2. Confirmar la visualizacion correcta de ambos encoders.
3. Calibrar los Sharp.
4. Medir velocidad de ruedas.
5. Implementar PID.
6. Movimiento recto y giros.
7. Seguimiento de pared.
8. Navegacion del laberinto.

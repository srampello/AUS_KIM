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

La primera version del proyecto permite probar por separado todos los elementos principales del robot desde una interfaz web alojada en el ESP32-S3.

### Red

- SSID: `AUS_KIM`
- Clave: `AUSKIM2026`
- Panel: `http://192.168.4.1`

### Funciones

- Lectura en tiempo real de los 4 sensores IR en ADC crudo.
- Control independiente del motor izquierdo.
- Control independiente del motor derecho.
- PWM ajustable de 0 a 255.
- Botones ADELANTE / ATRAS de tipo mantener presionado.
- Lectura de ambos encoders de cuadratura.
- Visualizacion de los canales A/B.
- Reset de encoders.
- STOP general.
- Fail-safe: si se pierde la comunicacion durante 1 segundo, se detienen ambos motores.

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

## Pinout

| Funcion | GPIO |
|---|---:|
| Sharp frontal izquierdo | 1 |
| Sharp frontal derecho | 2 |
| Sharp lateral izquierdo | 3 |
| Sharp lateral derecho | 4 |
| DRV8833 IN1 motor izquierdo | 5 |
| DRV8833 IN2 motor izquierdo | 6 |
| DRV8833 IN3 motor derecho | 7 |
| DRV8833 IN4 motor derecho | 8 |
| Encoder izquierdo A | 9 |
| Encoder izquierdo B | 10 |
| Encoder derecho A | 11 |
| Encoder derecho B | 12 |

Ver [docs/conexiones.md](docs/conexiones.md) para el detalle completo.

## Como probar la Etapa 01

1. Abrir `firmware/01_wifi_test/AUS_KIM_WIFI_TEST.ino` en Arduino IDE.
2. Seleccionar la placa ESP32-S3 correspondiente.
3. Compilar y cargar el firmware.
4. Encender el robot con las ruedas levantadas.
5. Conectarse a la red Wi-Fi `AUS_KIM`.
6. Abrir `http://192.168.4.1`.
7. Probar primero sensores y encoders.
8. Probar cada motor con PWM bajo.

## Plan de desarrollo

1. Diagnostico Wi-Fi.
2. Calibracion de los Sharp.
3. Medicion de velocidad de ruedas mediante encoders.
4. PID independiente de velocidad de cada motor.
5. Movimiento recto y giros de 90 grados.
6. Seguimiento de pared.
7. Navegacion del laberinto.

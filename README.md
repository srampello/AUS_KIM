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

La primera version permite probar por separado los principales elementos del robot desde una interfaz web alojada en el ESP32-S3.

### Red

- SSID: `AUS_KIM`
- Clave: `AUSKIM2026`
- Panel: `http://192.168.4.1`

### Funciones

- Lectura en tiempo real de los 4 sensores IR en ADC crudo.
- Control independiente de motor izquierdo y derecho.
- PWM ajustable de 0 a 255.
- Botones ADELANTE / ATRAS de tipo mantener presionado.
- Lectura de ambos encoders de cuadratura.
- Visualizacion de canales A/B.
- Reset de encoders.
- STOP general.
- Fail-safe: si se pierde la comunicacion durante 1 segundo, se detienen ambos motores.

## Pinout verificado

Durante la primera prueba se detecto que motores y sensores estaban cruzados respecto de la documentacion original.

| Funcion fisica | GPIO |
|---|---:|
| Sharp frontal izquierdo | 2 |
| Sharp frontal derecho | 1 |
| Sharp lateral izquierdo | 4 |
| Sharp lateral derecho | 3 |
| DRV8833 IN3 motor izquierdo | 7 |
| DRV8833 IN4 motor izquierdo | 8 |
| DRV8833 IN1 motor derecho | 5 |
| DRV8833 IN2 motor derecho | 6 |
| Encoder izquierdo A | 9 |
| Encoder izquierdo B | 10 |
| Encoder derecho A | 11 |
| Encoder derecho B | 12 |

El **motor fisico izquierdo** requiere inversion de sentido por software.

## Estado de pruebas

- Motor izquierdo: identificado y corregido.
- Motor derecho: identificado.
- Sensores frontales: lados corregidos.
- Sensores laterales: lados corregidos.
- Encoder izquierdo: funcionando.
- Encoder derecho: pendiente de diagnostico.

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

Ver [docs/conexiones.md](docs/conexiones.md) para el detalle del cableado verificado.

## Como probar

1. Abrir `firmware/01_wifi_test/AUS_KIM_WIFI_TEST.ino` en Arduino IDE.
2. Seleccionar la placa ESP32-S3 correspondiente.
3. Compilar y cargar el firmware.
4. Encender el robot con las ruedas levantadas.
5. Conectarse a la red Wi-Fi `AUS_KIM`.
6. Abrir `http://192.168.4.1`.
7. Confirmar sensores y encoder izquierdo.
8. Probar cada motor con PWM bajo.
9. Para el encoder derecho, observar los estados A/B mientras se gira la rueda manualmente.

## Plan de desarrollo

1. Diagnostico Wi-Fi.
2. Resolver encoder derecho.
3. Calibracion de los Sharp.
4. Medicion de velocidad de ruedas mediante encoders.
5. PID independiente de velocidad de cada motor.
6. Movimiento recto y giros de 90 grados.
7. Seguimiento de pared.
8. Navegacion del laberinto.

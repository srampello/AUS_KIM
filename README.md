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
- **Activacion secuencial de sensores: solo uno emite IR por vez.**
- Control independiente de motor izquierdo y derecho.
- PWM ajustable de 0 a 255.
- Botones ADELANTE / ATRAS de tipo mantener presionado.
- Lectura de ambos encoders de cuadratura.
- Visualizacion de canales A/B.
- Reset de encoders.
- STOP general.
- Fail-safe de motores de 1 segundo.

## Pinout verificado

### Sharp - Vout

| Funcion fisica | GPIO |
|---|---:|
| Sharp frontal izquierdo | 2 |
| Sharp frontal derecho | 1 |
| Sharp lateral izquierdo | 4 |
| Sharp lateral derecho | 3 |

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

El **motor fisico izquierdo** requiere inversion de sentido por software.

### Encoders

| Funcion fisica | GPIO |
|---|---:|
| Encoder izquierdo A | 11 |
| Encoder izquierdo B | 12 |
| Encoder derecho A | 9 |
| Encoder derecho B | 10 |

## Activacion secuencial de los Sharp

La version actual utiliza el Pin 5 (GPIO1) del GP2Y0E03 para poner los sensores en Active / Stand-by.

Solo uno permanece activo:

```text
FL -> FR -> LL -> LR -> repetir
```

El firmware espera 45 ms despues de activar cada sensor, guarda su lectura ADC y pasa al siguiente.

**Importante:** el Pin 5 de cada Sharp debe cablearse a GPIO 15, 16, 17 y 18. Si esos pines siguen desconectados, el firmware no puede apagar individualmente los emisores.

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

Ver [docs/conexiones.md](docs/conexiones.md) para el cableado detallado.

## Como actualizar la copia local

```bash
git pull origin master
```

## Plan de desarrollo

1. Diagnostico Wi-Fi.
2. Validar activacion individual de los cuatro Sharp.
3. Validar ambos encoders con el nuevo mapeo.
4. Calibracion de los Sharp.
5. Medicion de velocidad de ruedas mediante encoders.
6. PID independiente de velocidad de cada motor.
7. Movimiento recto y giros de 90 grados.
8. Seguimiento de pared.
9. Navegacion del laberinto.

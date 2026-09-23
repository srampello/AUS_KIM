# Conexiones de AUS_KIM

Este documento refleja el cableado **verificado fisicamente** durante las primeras pruebas del robot.

## 1. ESP32-S3 SuperMini

El ESP32-S3 recibe **5 V** desde la salida del modulo BEC / Step-Down BLITZ, conectados al pin **5V** y **GND** de la placa.

> Todos los GND del robot deben estar en comun: bateria, BEC, ESP32-S3, DRV8833, sensores y encoders.

## 2. Sensores Sharp GP2Y0E03

Durante la prueba se comprobo que los sensores izquierdos y derechos estaban invertidos respecto del esquema original.

### Salidas analogicas Vout

| Sensor fisico | Pin Vout | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 2 | GPIO 2 |
| Frontal derecho | Pin 2 | GPIO 1 |
| Lateral izquierdo | Pin 2 | GPIO 4 |
| Lateral derecho | Pin 2 | GPIO 3 |

### Control Active / Stand-by

Para evitar que varios emisores IR trabajen al mismo tiempo, ahora se utiliza el **Pin 5 (GPIO1)** de cada GP2Y0E03.

| Sensor fisico | Pin del Sharp | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 5 (GPIO1) | GPIO 15 |
| Frontal derecho | Pin 5 (GPIO1) | GPIO 16 |
| Lateral izquierdo | Pin 5 (GPIO1) | GPIO 17 |
| Lateral derecho | Pin 5 (GPIO1) | GPIO 18 |

Funcion:

- GPIO1 del Sharp en HIGH -> sensor activo.
- GPIO1 del Sharp en LOW -> sensor en stand-by.
- El firmware mantiene un solo sensor activo por vez.
- Cada sensor se mantiene activo 45 ms antes de tomar la lectura analogica.
- Luego se apaga y se activa el siguiente.

El ciclo es:

```text
Frontal izquierdo
      ->
Frontal derecho
      ->
Lateral izquierdo
      ->
Lateral derecho
      ->
repetir
```

Alimentacion de cada sensor:

- Pin 1 (VDD) -> 3V3
- Pin 4 (VIN_IO) -> 3V3
- Pin 3 (GND) -> GND
- Pin 2 (Vout) -> GPIO analogico correspondiente
- Pin 5 (GPIO1) -> GPIO 15/16/17/18 segun sensor
- Pin 6 (SCL) -> sin conectar
- Pin 7 (SDA) -> sin conectar

## 3. Driver DRV8833

Durante la prueba se comprobo que los canales de motores estaban cruzados respecto del esquema original.

| DRV8833 | Funcion real | Conexion |
|---|---|---|
| VM / VCC | Potencia de motores | Positivo LiPo 2S directo |
| GND | Tierra potencia | Negativo LiPo / GND comun |
| SLEEP / EEP | Habilitacion | 3V3 |
| IN1 | Motor fisico derecho | GPIO 5 |
| IN2 | Motor fisico derecho | GPIO 6 |
| IN3 | Motor fisico izquierdo | GPIO 7 |
| IN4 | Motor fisico izquierdo | GPIO 8 |
| OUT1 / OUT2 | Motor fisico derecho | Blanco / Rojo |
| OUT3 / OUT4 | Motor fisico izquierdo | Blanco / Rojo |

El motor fisico izquierdo gira en sentido contrario respecto de la logica esperada, por lo que el firmware lo invierte por software.

## 4. Encoders de motores

Durante la nueva prueba se comprobo que los encoders izquierdo y derecho tambien estaban invertidos respecto del mapeo original.

### Motor izquierdo fisico

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 11 |
| Amarillo (C2) | Canal B | GPIO 12 |

### Motor derecho fisico

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 9 |
| Amarillo (C2) | Canal B | GPIO 10 |

## 5. Alimentacion

- Bateria: LiPo CNHL Pizza V2, 2S, 7.4 V nominales, 250 mAh.
- DRV8833 VM: alimentacion directa desde la bateria.
- BEC / Step-Down BLITZ: salida ajustada a 5 V para el ESP32-S3.
- Sensores y encoders: 3.3 V desde el ESP32-S3.

## 6. Estado de validacion

- Motor fisico izquierdo: identificado correctamente; sentido corregido por software.
- Motor fisico derecho: identificado correctamente.
- Sensores frontales: izquierda/derecha corregidos.
- Sensores laterales: izquierda/derecha corregidos.
- Sensores: agregado control secuencial, uno activo por vez.
- Encoders: lados corregidos en firmware; pendiente volver a validar ambos fisicamente.

## 7. Recomendaciones de prueba

1. Antes de cargar esta version, conectar el Pin 5 (GPIO1) de cada Sharp a GPIO 15, 16, 17 y 18.
2. Mantener VDD y VIN(IO) de los sensores a 3.3 V.
3. Mantener el robot con las ruedas levantadas durante las pruebas de motores.
4. Usar PWM bajo, por ejemplo 60-80.
5. Confirmar que ADELANTE mueve cada rueda en el sentido correcto.
6. Verificar los cuatro sensores individualmente.
7. Girar cada rueda manualmente y comprobar que el encoder mostrado corresponde al lado fisico correcto.

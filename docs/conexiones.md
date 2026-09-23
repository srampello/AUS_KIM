# Conexiones de AUS_KIM

Este documento refleja el cableado actual del robot prestado y sera la referencia para el firmware de diagnostico.

## 1. ESP32-S3 SuperMini

El ESP32-S3 recibe **5 V** desde la salida del modulo BEC / Step-Down BLITZ, conectados al pin **5V** y **GND** de la placa.

> Todos los GND del robot deben estar en comun: bateria, BEC, ESP32-S3, DRV8833, sensores y encoders.

## 2. Sensores Sharp GP2Y0E03

Se utilizan cuatro sensores. Para esta primera etapa se lee solamente la salida analogica **Vout**.

| Sensor | Pin Vout | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 2 | GPIO 1 |
| Frontal derecho | Pin 2 | GPIO 2 |
| Lateral izquierdo | Pin 2 | GPIO 3 |
| Lateral derecho | Pin 2 | GPIO 4 |

Alimentacion de cada sensor:

- Pin 1 (VDD) -> 3V3
- Pin 4 (VIN_IO) -> 3V3
- Pin 3 (GND) -> GND
- Pin 2 (Vout) -> GPIO correspondiente

Los pines de interfaz digital/I2C que no se usan en esta etapa quedan desconectados.

## 3. Driver DRV8833

| DRV8833 | Funcion | Conexion |
|---|---|---|
| VM / VCC | Potencia de motores | Positivo LiPo 2S directo |
| GND | Tierra potencia | Negativo LiPo / GND comun |
| SLEEP / EEP | Habilitacion | 3V3 |
| IN1 | Motor izquierdo | GPIO 5 |
| IN2 | Motor izquierdo | GPIO 6 |
| IN3 | Motor derecho | GPIO 7 |
| IN4 | Motor derecho | GPIO 8 |
| OUT1 / OUT2 | Motor izquierdo | Blanco / Rojo |
| OUT3 / OUT4 | Motor derecho | Blanco / Rojo |

En el firmware de diagnostico los cuatro pines IN usan PWM.

## 4. Encoders de motores

Cada motor dispone de dos cables de potencia y cuatro cables del encoder.

### Motor izquierdo

| Cable | Funcion | Conexion |
|---|---|---|
| Blanco (M1) | Potencia | DRV8833 OUT1 |
| Rojo (M2) | Potencia | DRV8833 OUT2 |
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 9 |
| Amarillo (C2) | Canal B | GPIO 10 |

### Motor derecho

| Cable | Funcion | Conexion |
|---|---|---|
| Blanco (M1) | Potencia | DRV8833 OUT3 |
| Rojo (M2) | Potencia | DRV8833 OUT4 |
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 11 |
| Amarillo (C2) | Canal B | GPIO 12 |

## 5. Alimentacion

- Bateria: LiPo CNHL Pizza V2, 2S, 7.4 V nominales, 250 mAh.
- DRV8833 VM: alimentacion directa desde la bateria.
- BEC / Step-Down BLITZ: salida ajustada a 5 V para el ESP32-S3.
- Sensores y encoders: 3.3 V desde el ESP32-S3.

## 6. Observaciones para las primeras pruebas

1. Levantar el robot de la mesa antes de probar los motores.
2. Comenzar con PWM bajo, por ejemplo 60-80.
3. Verificar cada motor individualmente.
4. Si un boton ADELANTE produce giro inverso, cambiar la constante de inversion correspondiente en el firmware.
5. No convertir aun los Sharp a centimetros: primero registrar sus valores ADC a distancias conocidas.
6. Verificar que los contadores de encoder cambien al girar cada rueda manualmente.

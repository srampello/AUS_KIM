# Conexiones de AUS_KIM

Este documento refleja el cableado **verificado fisicamente** durante las primeras pruebas del robot.

## 1. ESP32-S3 SuperMini

El ESP32-S3 recibe **5 V** desde la salida del modulo BEC / Step-Down BLITZ, conectados al pin **5V** y **GND** de la placa.

> Todos los GND del robot deben estar en comun: bateria, BEC, ESP32-S3, DRV8833, sensores y encoders.

## 2. Sensores Sharp GP2Y0E03

Durante la prueba se comprobo que los sensores izquierdos y derechos estaban invertidos respecto del esquema original. El mapeo correcto es:

| Sensor fisico | Pin Vout | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 2 | GPIO 2 |
| Frontal derecho | Pin 2 | GPIO 1 |
| Lateral izquierdo | Pin 2 | GPIO 4 |
| Lateral derecho | Pin 2 | GPIO 3 |

Alimentacion de cada sensor:

- Pin 1 (VDD) -> 3V3
- Pin 4 (VIN_IO) -> 3V3
- Pin 3 (GND) -> GND
- Pin 2 (Vout) -> GPIO correspondiente

Los pines de interfaz digital/I2C que no se usan en esta etapa quedan desconectados.

## 3. Driver DRV8833

Durante la prueba se comprobo que los canales de motores tambien estaban cruzados respecto del esquema original.

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

### Motor izquierdo fisico

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 9 |
| Amarillo (C2) | Canal B | GPIO 10 |

**Estado:** probado y funcionando correctamente.

### Motor derecho fisico

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 11 |
| Amarillo (C2) | Canal B | GPIO 12 |

**Estado:** no responde actualmente. Pendiente verificar si cambian las señales A/B en GPIO 11 y GPIO 12 para determinar si el problema es cableado, alimentacion, encoder o software.

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
- Encoder izquierdo: OK.
- Encoder derecho: pendiente de diagnostico.

## 7. Recomendaciones de prueba

1. Mantener el robot con las ruedas levantadas durante las pruebas de motores.
2. Usar PWM bajo, por ejemplo 60-80.
3. Confirmar que ADELANTE mueve cada rueda en el sentido correcto.
4. Verificar los cuatro sensores individualmente acercando una mano u objeto.
5. Para el encoder derecho, observar primero los estados A y B en la interfaz mientras se gira la rueda manualmente.

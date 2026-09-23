# Conexiones de AUS_KIM

Este documento refleja el cableado verificado del robot y las correcciones aplicadas en la interfaz de diagnostico.

## 1. ESP32-S3 SuperMini

El ESP32-S3 recibe **5 V** desde la salida del modulo BEC / Step-Down BLITZ, conectados al pin **5V** y **GND** de la placa.

> Todos los GND del robot deben estar en comun: bateria, BEC, ESP32-S3, DRV8833, sensores y encoders.

## 2. Sensores Sharp GP2Y0E03

Los lados de los sensores fueron corregidos respecto del esquema original.

### Salidas analogicas Vout

| Sensor fisico | Pin Vout | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 2 | GPIO 2 |
| Frontal derecho | Pin 2 | GPIO 1 |
| Lateral izquierdo | Pin 2 | GPIO 4 |
| Lateral derecho | Pin 2 | GPIO 3 |

### Seleccion manual Active / Stand-by

Para las pruebas se utiliza el **Pin 5 (GPIO1)** de cada GP2Y0E03 para activar solamente el sensor elegido desde la interfaz web.

| Sensor fisico | Pin del Sharp | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 5 (GPIO1) | GPIO 15 |
| Frontal derecho | Pin 5 (GPIO1) | GPIO 16 |
| Lateral izquierdo | Pin 5 (GPIO1) | GPIO 17 |
| Lateral derecho | Pin 5 (GPIO1) | GPIO 18 |

Funcionamiento:

- Al iniciar el ESP32, los cuatro sensores quedan en stand-by.
- La interfaz tiene un boton **ACTIVAR** para cada sensor.
- Al activar un sensor, los otros tres se apagan automaticamente.
- El sensor elegido queda activo hasta seleccionar otro o pulsar **APAGAR SENSORES**.
- Solo la lectura del sensor activo se muestra y actualiza en pantalla.
- Se espera aproximadamente 45 ms luego de activarlo antes de tomar lecturas.

Alimentacion de cada sensor:

- Pin 1 (VDD) -> 3V3
- Pin 4 (VIN_IO) -> 3V3
- Pin 3 (GND) -> GND
- Pin 2 (Vout) -> GPIO analogico correspondiente
- Pin 5 (GPIO1) -> GPIO 15/16/17/18 segun sensor
- Pin 6 (SCL) -> sin conectar
- Pin 7 (SDA) -> sin conectar

## 3. Driver DRV8833

Los motores fisicos estaban cruzados respecto de la documentacion inicial.

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

El motor fisico izquierdo requiere inversion de sentido por software.

## 4. Encoders de motores

**El cableado de los encoders no se modifica.** Se conserva el mapeo original:

### Encoder conectado al par GPIO 9 / 10

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 9 |
| Amarillo (C2) | Canal B | GPIO 10 |

### Encoder conectado al par GPIO 11 / 12

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 11 |
| Amarillo (C2) | Canal B | GPIO 12 |

Durante la prueba se observo que izquierda y derecha aparecian invertidas **solamente en la interfaz web**. La correccion se hizo en la capa visual: los datos y estados A/B se muestran intercambiados en pantalla sin cambiar las conexiones ni los pines del firmware que leen los encoders.

## 5. Alimentacion

- Bateria: LiPo CNHL Pizza V2, 2S, 7.4 V nominales, 250 mAh.
- DRV8833 VM: alimentacion directa desde la bateria.
- BEC / Step-Down BLITZ: salida ajustada a 5 V para el ESP32-S3.
- Sensores y encoders: 3.3 V desde el ESP32-S3.

## 6. Estado actual

- Motor fisico izquierdo: identificado; sentido corregido por software.
- Motor fisico derecho: identificado.
- Sensores frontales y laterales: lados corregidos.
- Sensores: seleccion manual desde la interfaz, uno activo por vez.
- Encoders: cableado sin cambios; correccion izquierda/derecha realizada solo en la visualizacion.

## 7. Prueba recomendada

1. Conectar el Pin 5 (GPIO1) de cada Sharp a GPIO 15, 16, 17 y 18.
2. Cargar el firmware de `firmware/01_wifi_test/AUS_KIM_WIFI_TEST.ino`.
3. Entrar a `http://192.168.4.1`.
4. Pulsar **ACTIVAR** en un solo sensor y comprobar que responde.
5. Seleccionar otro sensor y verificar que el anterior deje de estar activo.
6. Probar **APAGAR SENSORES**.
7. Girar cada rueda manualmente y verificar que en la interfaz el encoder aparezca bajo el lado fisico correcto.

# Conexiones de AUS_KIM

Este documento refleja el cableado real verificado del robot y las correcciones aplicadas en la interfaz de diagnostico.

## 1. ESP32-S3 SuperMini

El ESP32-S3 recibe **5 V** desde la salida del modulo BEC / Step-Down BLITZ, conectados al pin **5V** y **GND** de la placa.

> Todos los GND del robot deben estar en comun: bateria, BEC, ESP32-S3, DRV8833, sensores y encoders.

## 2. Sensores Sharp GP2Y0E03

Los lados de los sensores fueron corregidos respecto del esquema original.

### Conexion utilizada

| Sensor fisico | Vout | ESP32-S3 |
|---|---:|---:|
| Frontal izquierdo | Pin 2 | GPIO 2 |
| Frontal derecho | Pin 2 | GPIO 1 |
| Lateral izquierdo | Pin 2 | GPIO 4 |
| Lateral derecho | Pin 2 | GPIO 3 |

Para cada Sharp:

- Pin 1 (VDD) -> 3V3
- Pin 2 (Vout) -> GPIO analogico correspondiente
- Pin 3 (GND) -> GND
- Pin 4 (VIN_IO) -> 3V3
- Pin 5 (GPIO1) -> **sin conectar**
- Pin 6 (SCL) -> sin conectar
- Pin 7 (SDA) -> sin conectar

### Seleccion manual desde la interfaz

Como el **pin 5 (GPIO1) no esta conectado**, el ESP32 no puede poner los sensores en Active / Stand-by.

La interfaz no enciende ni apaga fisicamente los Sharp. Lo que hace es permitir elegir **cual salida analogica leer**:

- Frontal izquierdo
- Frontal derecho
- Lateral izquierdo
- Lateral derecho
- Detener lectura

Los cuatro sensores permanecen alimentados. Solo el sensor seleccionado se consulta y actualiza en pantalla.

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

**El cableado de los encoders no se modifica.**

### Primer par

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 9 |
| Amarillo (C2) | Canal B | GPIO 10 |

### Segundo par

| Cable | Funcion | Conexion |
|---|---|---|
| Negro (VCC) | Alimentacion encoder | 3V3 |
| Azul (GND) | Tierra encoder | GND |
| Verde (C1) | Canal A | GPIO 11 |
| Amarillo (C2) | Canal B | GPIO 12 |

Durante la prueba se observo que izquierda y derecha aparecian invertidas solamente en la interfaz web. La correccion se hace **solo en la visualizacion**, sin cambiar conexiones ni GPIO de lectura.

## 5. Alimentacion

- Bateria: LiPo CNHL Pizza V2, 2S, 7.4 V nominales, 250 mAh.
- DRV8833 VM: alimentacion directa desde la bateria.
- BEC / Step-Down BLITZ: salida ajustada a 5 V para el ESP32-S3.
- Sensores y encoders: 3.3 V desde el ESP32-S3.

## 6. Estado actual

- Motor fisico izquierdo: identificado y sentido corregido por software.
- Motor fisico derecho: identificado.
- Sensores frontales y laterales: lados corregidos.
- Sensores: seleccion manual de lectura desde la interfaz.
- Pin 5 de los Sharp: sin conectar.
- Encoders: cableado sin cambios; correccion izquierda/derecha solo visual.

## 7. Prueba recomendada

1. Cargar `firmware/01_wifi_test/AUS_KIM_WIFI_TEST.ino`.
2. Entrar a `http://192.168.4.1`.
3. Pulsar **SELECCIONAR** en un sensor.
4. Verificar que solo esa lectura se actualice.
5. Seleccionar otro sensor y comprobar que la visual cambie.
6. Probar **DETENER LECTURA**.
7. Girar cada rueda manualmente y confirmar que el encoder aparece bajo el lado fisico correcto.


## 8. Filtro de lectura de sensores

Para estabilizar la lectura analogica de los GP2Y0E03 se usa:

- 9 muestras ADC por lectura.
- Mediana de esas 9 muestras para eliminar picos.
- Suavizado exponencial posterior:
  `filtrado = 0.75 * anterior + 0.25 * mediana`.

La interfaz permite comparar el valor filtrado con la mediana instantanea (`Crudo`).

El laberinto tiene celdas de hasta 25 x 25 cm. Cuando se calibre ADC a centimetros, las distancias superiores a 25 cm se trataran como ausencia de pared cercana. Por ahora no se recortan valores ADC porque la relacion ADC/distancia debe medirse primero sobre este robot.

# AUS_KIM V2 — Hardware y pinout ESP32-C3 Super Mini

> **Estado: revisión de hardware / PCB en desarrollo (29/10/2026).** Esta es la referencia de conexiones de la **nueva placa V2** basada en el esquemático EasyEDA compartido el 27/10/2026. El firmware existente del repositorio **sigue siendo para la V1 con ESP32-S3**: NO cargar esos archivos `.ino` en la C3 sin portar y verificar el código.
>
> Documentación histórica V1: [conexiones.md](conexiones.md).

## 1. Qué cambió de la V1 a la V2

| Elemento | AUS_KIM V1 (histórica) | AUS_KIM V2 (nuevo diseño) |
| --- | --- | --- |
| Microcontrolador | ESP32-S3 SuperMini | **ESP32-C3 Super Mini** |
| Sharp de paredes | 4: dos frontales + dos laterales | **3: uno frontal + izquierdo + derecho** |
| Sensor de llegada | Sin sensor de piso | **1 QRE1113GR analógico** en plaqueta externa |
| Motores | Motorreductores anteriores; encoders usados principalmente en diagnóstico | **2 N20 de 1500 rpm con encoder de cuadratura A/B**, por calibrar |
| Driver | DRV8833 | DRV8833 con **JP1 de habilitación EEP/nSLEEP** |
| Largada local | Interfaz web / STOP anterior | **Pulsador START en GPIO9 (BOOT)** |
| Alimentación | LiPo 2S + BEC | LiPo 2S + BEC, salida regulada de **5 V** |
| Navegación | Seguimiento de pared derecha, giros temporizados | Previsto: exploración/mapeo, carrera con mapa + encoders y alternativa por pared derecha |

**Importante:** esta tabla describe **hardware de V2** y **funciones planificadas**. No significa que BFS/Flood Fill, persistencia de mapa, nuevos sensores o encoders de V2 estén implementados en el firmware actual.

## 2. Tabla completa de GPIO — referencia para el firmware V2

El pinout se transcribió del esquema EasyEDA final compartido el 27/10/2026. Conservar los **nombres de red** al desarrollar el firmware.

| GPIO ESP32-C3 | Red EasyEDA | Dispositivo | Uso |
| ---: | --- | --- | --- |
| **0** | `SH_I` | Sharp lateral izquierdo GP2Y0E03 | ADC |
| **1** | `SH_F` | Sharp frontal GP2Y0E03 | ADC |
| **2** | `AIN1` | DRV8833 IN1, motor derecho | Salida PWM |
| **3** | `SH_D` | Sharp lateral derecho GP2Y0E03 | ADC |
| **4** | `SENSOR` | Conector de QRE1113GR (plaqueta externa) | ADC de piso |
| **5** | `ENC_IB` | Encoder izquierdo, canal B | Entrada digital / interrupción |
| **6** | `ENC_IA` | Encoder izquierdo, canal A | Entrada digital / interrupción |
| **7** | `ENC_DB` | Encoder derecho, canal B | Entrada digital / interrupción |
| **8** | `AIN2` | DRV8833 IN2, motor derecho | Salida PWM |
| **9** | `START` | Pulsador BOOT/START | Entrada digital activa en LOW |
| **10** | `ENC_DA` | Encoder derecho, canal A | Entrada digital / interrupción |
| **20** | `BIN1` | DRV8833 IN3, motor izquierdo | Salida PWM |
| **21** | `BIN2` | DRV8833 IN4, motor izquierdo | Salida PWM |

**Atención a los GPIO de arranque:** 2, 8 y 9 son *strapping pins*. En V2, los encoders se trasladaron a 5/6/7/10 para que sus niveles aleatorios no interfieran con el arranque. Las entradas del DRV8833 asignadas a 2 y 8 tienen pull-ups externas de 10 kΩ a 3,3 V. GPIO9 (BOOT) se usa como START: **no presionarlo mientras se enciende o resetea**. GPIO8 puede compartir el LED de algunas C3 Super Mini; verificar el módulo real. GPIO21 puede presentar actividad de UART0 durante el arranque, por lo que el driver debe seguir deshabilitado mientras se inicia.

## 3. Sensores de pared — 3 × Sharp GP2Y0E03

Se eliminó **uno de los dos Sharp frontales de V1**. El frontal restante debe ubicarse mirando hacia adelante, idealmente centrado; los dos laterales apuntan a las paredes izquierda y derecha.

| Sensor | Señal en PCB | GPIO |
| --- | --- | ---: |
| Frontal | `SH_F` | 1 |
| Lateral izquierdo | `SH_I` | 0 |
| Lateral derecho | `SH_D` | 3 |

Cableado de **cada uno** de los tres GP2Y0E03, en modo analógico (sin I²C):

| Pin físico Sharp | Función | Conexión V2 |
| ---: | --- | --- |
| 1 | VDD | **+5 V regulados del BEC** |
| 2 | VOUT(A) | `SH_F` / `SH_I` / `SH_D`, según posición |
| 3 | GND | Masa común |
| 4 | VIN(IO) | **+3,3 V** |
| 5 | GPIO1 (habilitación/selección del módulo Sharp) | **+3,3 V**: sensor activo |
| 6 | SCL | Sin conectar |
| 7 | SDA | Sin conectar |

**No confundir** el pin denominado `GPIO1` del **sensor Sharp** con el GPIO1 de la **ESP32-C3**. El primero se conecta a 3,3 V; el segundo recibe la salida analógica del Sharp frontal.

Se recomienda desacoplar VDD-GND cerca de cada sensor y separar las pistas de señales analógicas de las salidas de motores. Los umbrales ADC de la V1 **no son directamente reutilizables**: hay menos sensores, otro ADC y otra alimentación.

## 4. Sensor de piso — plaqueta externa QRE1113GR

El QRE1113GR está colocado en **otra plaqueta**, no directamente sobre el PCB principal. La placa AUS_KIM V2 sólo aporta un conector de **tres contactos**:

| Pin conector `Sensor` | Red | Conexión |
| ---: | --- | --- |
| 1 | `GND` | GND |
| 2 | `+3.3V` | 3,3 V |
| 3 | `SENSOR` | GPIO4 / señal analógica |

Circuito previsto en la plaqueta externa QRE1113GR:
- LED IR: ánodo con resistencia de **150 Ω** hacia +3,3 V; cátodo a GND.
- Fototransistor: colector a `SENSOR` y mediante **10 kΩ a +3,3 V**; emisor a GND.
- Habitualmente el piso blanco refleja más IR y reduce la lectura en este circuito; **calibrar contra la superficie real** y fijar un umbral medido.
- Montar mirando hacia abajo, centrado transversalmente y cerca del eje de ruedas o ligeramente adelante. Ajustar altura por prueba.

**No agregar otra resistencia pull-up de 10 kΩ en GPIO4** si ya está incorporada en la plaqueta del QRE.

## 5. DRV8833 — potencia y control

**El esquemático define los nombres y la numeración del módulo de 12 pines**; comprobarlos contra la pieza física (las etiquetas `EEP` y `ULT` son propias del símbolo/módulo).

| Pin módulo | Nombre | Red / destino |
| ---: | --- | --- |
| 1 | IN4 | `BIN2` → GPIO21 |
| 2 | IN3 | `BIN1` → GPIO20 |
| 3 | GND | GND común |
| 4 | VCC / VM | **+5 V regulados** en el esquemático V2 |
| 5 | IN2 | `AIN2` → GPIO8 |
| 6 | IN1 | `AIN1` → GPIO2 |
| 7 | ULT (FAULT) | Sin conectar |
| 8 | OUT4 | `M_IZQ+` |
| 9 | OUT3 | `M_IZQ-` |
| 10 | OUT2 | `M_DER-` |
| 11 | OUT1 | `M_DER+` |
| 12 | EEP (nSLEEP, **verificar** módulo) | `EEP` → JP1 → +3,3 V |

- **Motor derecho**: IN1/IN2 (GPIO2/GPIO8), OUT1/OUT2 (redes M_DER+/-).
- **Motor izquierdo**: IN3/IN4 (GPIO20/GPIO21), OUT4/OUT3 (redes M_IZQ+/-).
- **JP1 retirado:** driver deshabilitado por nSLEEP si el módulo implementa el pull-down esperado; comprobar con multímetro/hoja de datos.
- **JP1 colocado:** habilitado. Procedimiento de pruebas: encender con JP1 **retirado**, dejar iniciar firmware y configurar los cuatro pines de motor en parada, después colocar JP1 y finalmente pulsar START.
- JP1 es **manual**: no es un enclavamiento automático. Si se reinicia el ESP32 con JP1 colocado, el driver podría quedar habilitado durante el arranque. No sustituye al interruptor general.

**Polarización de arranque:** R1 = 10 kΩ de `AIN1`/GPIO2 a +3,3 V y R2 = 10 kΩ de `AIN2`/GPIO8 a +3,3 V. Mientras se inicializa la C3, ambas entradas del puente derecho tenderán a HIGH/HIGH; aún así hay que **validar arranque y no movimiento** con el módulo real.

**Alimentación de motores:** el esquemático conecta VCC del DRV8833 a **+5 V regulados**, no al positivo LiPo directo de V1. Confirmar tensión nominal de los N20, picos de arranque/bloqueo y corriente disponible del BEC antes de competir. Añadir o comprobar condensadores de desacoplamiento próximos al driver.

## 6. Motores N20 1500 RPM con encoders completos A/B

Se sustituyen los motores anteriores por **dos motorreductores N20 especificados como 1500 RPM**, cada uno con **encoder de cuadratura**. Se leen **los cuatro canales** (A y B de ambos lados). En ESP32-C3 se prevé decodificar por GPIO/interrupciones y comprobar la capacidad a la velocidad máxima.

### Conectores de 6 pines en la PCB V2

**Orden eléctrico del conector de la PCB**, NO asumir que coincide con el orden de colores del cable original del motor:

| Pin conector | Motor izquierdo (`M_IZQ`) | Motor derecho (`M_DER`) |
| ---: | --- | --- |
| 1 | `M_IZQ+` (DRV OUT4) | `M_DER+` (DRV OUT1) |
| 2 | `M_IZQ-` (DRV OUT3) | `M_DER-` (DRV OUT2) |
| 3 | +3,3 V para encoder | +3,3 V para encoder |
| 4 | GND del encoder | GND del encoder |
| 5 | `ENC_IA` → GPIO6 | `ENC_DA` → GPIO10 |
| 6 | `ENC_IB` → GPIO5 | `ENC_DB` → GPIO7 |

**Precaución:** el arnés original de seis cables de los N20 puede tener otro orden. Verificar cada cable contra el datasheet/motor real y, si es necesario, crimpar un arnés adaptado a esta PCB. Nunca conectar accidentalmente tensión de motor a la alimentación o señales del encoder.

- Encoder VCC a **3,3 V**, no a 5 V, siempre que la versión instalada del Hall admita esa tensión.
- Dejar opción de pull-ups para A/B si las salidas no son push-pull.
- Comprobar conteo y signo de cada rueda, relación de reducción, pulsos por vuelta, **diámetro de rueda** y **distancia entre ruedas** antes de calcular centímetros o ángulos.
- Los valores de ticks, PID y PWM de la V1 no se trasladan automáticamente a los motores de 1500 RPM.

## 7. Botón físico START

| Elemento | Conexión |
| --- | --- |
| `START` | GPIO9 de ESP32-C3 (BOOT) |
| R3 | 10 kΩ entre START y +3,3 V, según esquemático |
| SW1 | Pulsador normalmente abierto entre START y GND |

Funcionamiento: sin pulsar HIGH; pulsado LOW. Implementar antirrebote y comenzar una sola vez por pulsación. **No mantener pulsado START al energizar/resetear**, porque GPIO9 interviene en el modo de arranque. El botón integrado BOOT de la Super Mini podría estar conectado al mismo GPIO: verificar la placa antes de duplicar componentes de polarización.

START es una orden de largada; **no es parada de emergencia**. Mantener un corte físico accesible de alimentación de motores/batería.

## 8. Alimentación y masas

- Fuente: **LiPo 2S (7,4 V nominal; 8,4 V completamente cargada)**.
- El BEC recibe la batería a través del interruptor de alimentación previsto en el esquema; conexiones de entrada/salida confirmadas durante el diseño.
- Salida regulada del BEC: **+5 V** hacia el pin 5V del ESP32-C3, VDD de los tres Sharp y VCC/VM del módulo DRV8833 según la versión V2 del esquema.
- **+3,3 V** de la placa: VIN(IO) y GPIO1 de los Sharp, encoders, plaqueta de piso, polarizaciones de GPIO y habilitación JP1.
- **GND común:** negativo de batería, ambos GND del BEC, ESP32, DRV8833, Sharp, encoders y piso. La salida de GND del BEC es común si el convertidor es no aislado; verificar continuidad sin alimentación.
- **USB + BEC:** comprobar protección/aislamiento del pin 5V antes de alimentar por las dos fuentes simultáneamente.

**Pendiente de confirmación antes de fabricar:** capacidad del regulador 3,3 V de esta Super Mini para sensores/encoders y Wi-Fi; consumo pico y nominal de los N20; si el módulo BEC realmente montado coincide con el símbolo/huella del diseño. No asumir que todos los BEC llamados BLITZ o Matek comparten pinout.

## 9. PCB y validaciones pendientes

Diseño previsto: **dos capas, hasta 67 × 33 mm, componentes TOP**; rutear TOP/BOTTOM. La referencia mecánica anterior usa cuatro tornillos M3 separados **54 mm longitudinales × 15,55 mm transversales**; validar esa huella en la versión final.

Lista antes de fabricar:
1. Confirmar que footprint de ESP32-C3 coincide con la placa física; revisar zona libre de antena y acceso USB.
2. Confirmar footprint, orientación y orden de conectores del BEC, Sharp, motor/encoder, sensor de piso y JP1.
3. Añadir/verificar desacoplamiento local de 5 V, 3,3 V, DRV8833 y Sharp. Plano GND y pistas de motores dimensionadas para corriente real.
4. Probar GPIO2/GPIO8 con las resistencias R1/R2; comprobar BOOT GPIO9, LED de GPIO8 y actividad de UART0/GPIO21.
5. Encender con JP1 abierto, verificar arranque, programación USB y niveles de las cuatro entradas del driver; habilitar JP1 sólo tras inicialización.
6. Probar los dos encoders a 3,3 V, canales A/B y signo de cada rueda; comprobar que los conectores no intercambian alimentación con señales.
7. Calibrar ADC de tres Sharp y del QRE; validar identificación de piso blanco de llegada.
8. Comprobar encendido con batería, alimentación USB, ruido de motores y ausencia de reinicios.
9. Ejecutar **ERC del esquema**, **DRC del PCB**, inspección 3D y verificación de los cuatro agujeros M3.

## 10. Software V2 — próximo desarrollo

Este documento es la **referencia de hardware** para portar el firmware. El código existente en `firmware/01_...` a `firmware/06_race/...` es **V1/S3 con cuatro Sharp y sin los GPIO nuevos**. No se modifican esos sketches automáticamente con esta documentación.

Arquitectura prevista para ESP32-C3:
- **Diagnóstico:** 3 Sharp + sensor blanco, driver, START y cuatro canales de encoders; configuración/telemetría Wi-Fi.
- **Exploración / primera pasada:** leer paredes y odometría, mapear celdas con BFS/Flood Fill, registrar objetivo si el QRE confirma el piso de llegada; guardar mapa con validación.
- **Segunda pasada con mapa:** seguir ruta calculada y controlar distancia/giros con encoders.
- **Segunda pasada alternativa:** seguimiento de pared derecha sin depender del mapa.
- Seleccionar modo antes de presionar START; la carrera no debe depender de mantener la conexión Wi-Fi.
- Parada local de emergencia **por definir y probar** antes de habilitar carreras autónomas, además del interruptor de corte.

Se requieren mediciones de pulsos por vuelta reales, diámetro de rueda, separación entre ruedas, umbrales y dimensiones de celda. **No declarar BFS ni los modos V2 como implementados hasta que exista un firmware compilado, probado y subido al repositorio.**

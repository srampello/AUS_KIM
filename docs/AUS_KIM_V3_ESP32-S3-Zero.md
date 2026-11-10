# AUS_KIM V3 — Propuesta de placa: ESP32-S3-Zero + TB6612FNG

> **Estado: PROPUESTA para revisar (10/11/2026). No es un esquemático ni un PCB terminado.** Reemplaza a la V2 (ESP32-C3 + DRV8833), cuyo driver se dañó. Todo lo marcado **[verificar]** hay que comprobarlo contra la pieza real o la hoja de datos antes de fabricar.
>
> Documentos anteriores: [V2 ESP32-C3](AUS_KIM_V2_ESP32-C3.md) · [V1 histórica](conexiones.md).

## 1. Lista de componentes pedida

| Bloque | Componente | Soldado a la placa |
| --- | --- | --- |
| Microcontrolador | Waveshare **ESP32-S3-Zero** | Por sus pads de borde (o sobre cabezal hembra) |
| Driver de motores | **TB6612FNG** (SSOP24) | **Sí, el chip** + sus capacitores |
| Paredes | **4 × Sharp GP2Y0E03** | Conectores |
| Piso | **1 × QRE1113GR** (plaqueta externa, como en V2) | Conector de 3 pines |
| Motores | **2 × N20 1500 rpm con encoder** | Conectores de 6 pines |
| Batería | LiPo **2S 7,4 V** (6,0–8,4 V) | Conector de batería |
| Alimentación | **Regulador de 5 V con huella TO-220** (7805 o equivalente conmutado, ver §4.2) | **Sí** |

## 2. Esquema por bloques

```
 Batería 2S ─► [conector] ─► [interruptor] ─► [protección polaridad] ─► VBAT ─┬──────────────► VM del TB6612FNG
                                                                              │
                                                                              └─► [regulador 5 V] ─► +5V ─┬─► VDD de los 4 Sharp
                                                                                                        └─► [diodo Schottky] ─► pin 5V del S3-Zero
                                                                                                                                  │
                                                                         pin 3V3 del S3-Zero (LDO de la placa) ◄──────────────────┘
                                                                              ├─► Vcc del TB6612FNG, PWMA, PWMB
                                                                              ├─► VIN(IO) y GPIO1 de cada Sharp
                                                                              ├─► encoders y plaqueta del QRE
                                                                              └─► pull-up de START
```

## 3. Asignación de pines (ESP32-S3-Zero)

El S3-Zero saca a sus bordes los **GPIO 1 a 13, TX (GPIO43) y RX (GPIO44)**, más 3V3, 5V y GND. Los GPIO 14–16 son pads sueltos en la cara superior y los 17, 18, 38–42 y 45 están en la cara inferior; ninguno está en el borde (según las fotos de la placa). La placa usa GPIO21 para su LED RGB y GPIO0 para su botón BOOT. [Fuente: Waveshare](https://docs.waveshare.com/ESP32-S3-Zero)

| GPIO | Red | Dispositivo | Uso |
| ---: | --- | --- | --- |
| 1 | `SH_I` | Sharp lateral izquierdo | ADC1 |
| 2 | `SH_FI` | Sharp frontal izquierdo | ADC1 |
| 4 | `SH_FD` | Sharp frontal derecho | ADC1 |
| 5 | `SH_D` | Sharp lateral derecho | ADC1 |
| 6 | `SENSOR` | QRE1113GR (piso) | ADC1 |
| 7 | `ENC_IA` | Encoder izquierdo, canal A | Entrada / interrupción |
| 8 | `ENC_IB` | Encoder izquierdo, canal B | Entrada / interrupción |
| 9 | `ENC_DA` | Encoder derecho, canal A | Entrada / interrupción |
| 10 | `ENC_DB` | Encoder derecho, canal B | Entrada / interrupción |
| 11 | `AIN1` | TB6612 AIN1, motor derecho | Salida PWM |
| 12 | `AIN2` | TB6612 AIN2, motor derecho | Salida PWM |
| 13 | `BIN1` | TB6612 BIN1, motor izquierdo | Salida PWM |
| 44 (RX) | `BIN2` | TB6612 BIN2, motor izquierdo | Salida PWM |
| 43 (TX) | `STBY` | TB6612 STBY | Salida (habilita el driver) |
| 3 | `START` | Pulsador de largada | Entrada, activa en LOW |
| 14 (pad suelto) | `PWMA` | TB6612 PWMA, motor derecho | Salida PWM (cable al pad) |
| 15 (pad suelto) | `PWMB` | TB6612 PWMB, motor izquierdo | Salida PWM (cable al pad) |
| 16 (pad suelto) | `VBAT_SENSE` | Divisor de la batería | ADC2 (ver §5.1) |

Criterios:
- **Todas las lecturas analógicas van en ADC1 (GPIO1–10).** El ADC2 (GPIO11–20) comparte recursos con el Wi-Fi y puede fallar mientras está activo, y el panel de calibración usa Wi-Fi. [verificar con el core que uses]
- **No quedan pines libres en el borde.** Los pads sueltos 14, 15 y 16 (cara superior del S3-Zero, se sueldan con cable) llevan `PWMA`, `PWMB` y `VBAT_SENSE`. Quedan los pads 17 y 18 (cara inferior) para ampliar, por ejemplo un giróscopo.
- **Los GPIO14–16 son ADC2.** Como salidas PWM no hay problema; `VBAT_SENSE` solo se puede leer con el Wi-Fi apagado (en la carrera). [verificar]
- **GPIO3 es pin de arranque (strapping).** Se usa para START con pull-up a 3,3 V; según la hoja de datos del S3 solo condiciona el origen del JTAG y en la configuración por defecto no tiene efecto. [verificar]
- **GPIO43/44 son la UART0.** Se usan como salidas normales, por eso en el firmware **no hay que usar `Serial0`/UART0**. Con "USB CDC On Boot: Enabled" `Serial` va por el USB y no los toca. Es la misma trampa que dejó el motor izquierdo a fondo en la V2.
- Durante el arranque la ROM puede escribir texto por GPIO43. Como `STBY` tiene pull-down y las entradas IN del driver tienen pull-down interno, los motores quedan sin energía igual. [verificar en el robot real]
- Los 4 Sharp mantienen la convención de la V1: dos laterales y dos frontales. Con dos frontales se puede **alinear el robot contra la pared de enfrente** comparando ambas lecturas.

## 4. Alimentación

### 4.1 Entrada y protección
1. **Conector de batería** (XT30 o JST, el que usen las baterías).
2. **Interruptor general** en el positivo.
3. **Protección contra polaridad invertida:** MOSFET canal P de al menos 20 V y Vgs ±12 V (tipo AO3401A) con la fuente hacia la carga y la compuerta a GND. Pierde mucho menos que un diodo Schottky. Un diodo SS34 también sirve, pero cae unos 0,4 V y disipa en el camino de los motores.
4. **Fusible reseteable de 2 A** en serie (opcional pero recomendable). [verificar corriente de bloqueo de los N20]
5. **Capacitor de 100 µF, 16 V o más, y 10 µF cerámico de 25 V** en VBAT, cerca del driver.

### 4.2 Regulador de 5 V: huella TO-220 (7805 o equivalente conmutado)

Una sola huella de 3 pines (1 = entrada, 2 = GND, 3 = salida) que acepta cualquiera de estas dos piezas:

| Opción | Ventaja | Cuidado |
| --- | --- | --- |
| **L7805CV** (7805 clásico) | Se consigue en cualquier casa de electrónica | Es lineal: pierde (Vbat − 5 V) × corriente en calor. Con 8,4 V y 0,3 A son unos 1 W; con 0,5 A, 1,7 W. Con θJA ≈ 50 °C/W (estimado) eso es entre +50 y +85 °C sobre el ambiente, así que conviene un disipador chico o cobre amplio. Además su caída mínima es de unos 2 V: **por debajo de ~7 V de batería la salida deja de ser 5 V**. [verificar en la hoja de datos de la pieza] |
| **7805 conmutado "drop-in"** (por ejemplo R-78E5.0-1.0 u OKI-78SR-5/1.5) | Mismos 3 pines que un 7805, rendimiento ~90 %, casi no calienta, sin componentes externos | Entrada mínima también de ~7 V. Verificar pinout y rango en la hoja de datos de la pieza que compres |

Recomendación: **soldar primero el drop-in conmutado si lo conseguís, y dejar el L7805 como plan B**; la huella es la misma.

Capacitores (circuito típico del 7805): **0,33 µF en la entrada y 0,1 µF en la salida**, pegados al regulador. Agregar **10 µF** (cerámico de 25 V en la entrada, de 10 V o más en la salida) y 100 nF en cada riel de 5 V cerca de los conectores de Sharp. Para el drop-in, los mismos 10 µF alcanzan.

Consecuencia práctica: **no usar la batería por debajo de ~7,0 V (3,5 V por celda)**, o los Sharp, alimentados a 5 V, empiezan a derivar. Con el regulador lineal o con el drop-in. Un cargador con corte por celda lo evita.

Consumo estimado en 5 V: 4 Sharp (26 mA típico, 36 mA máximo cada uno) + el S3-Zero con Wi-Fi (del orden de 100–350 mA según el momento), es decir, unos 0,3–0,5 A. Dentro de lo que dan las dos opciones.

### 4.2b Alternativa SMD (no elegida): AP63205WU-7
Buck de 5 V fijos, entrada de 3,8 a 32 V, hasta 2 A, encapsulado TSOT26 de 6 pines. Valores de la tabla de la [hoja de datos de Diodes](https://www.mouser.com/datasheet/2/115/AP63200_AP63201_AP63203_AP63205-1525010.pdf):

| Pieza | Valor | Nota |
| --- | --- | --- |
| Inductor L | **4,7 µH** | Corriente de saturación ≥ 2,7 A, DCR < 100 mΩ. Apantallado de 4×4 mm o similar |
| Capacitor de entrada C1 | **10 µF**, 25 V, X7R | Cerámico, pegado a los pines VIN y GND |
| Capacitor de salida C2 | **2 × 22 µF**, 10 V o más | Cerámicos X5R/X7R |
| Bootstrap C3 | **100 nF** | Entre BST y SW |
| EN (pin 2) | a VIN | Arranque automático |
| FB (pin 1) | a la salida de 5 V | Versión de salida fija. [verificar contra el circuito típico del PDF] |

Pines: 1 FB, 2 EN, 3 VIN, 4 GND, 5 SW, 6 BST. Layout: lazo VIN–C1–GND lo más corto posible, SW corto, plano de GND debajo.

Consumo estimado en 5 V: 4 Sharp (26 mA típico, 36 mA máximo cada uno) + el S3-Zero con Wi-Fi (varios cientos de mA en picos). Muy por debajo de los 2 A del regulador.

### 4.3 Alimentación del S3-Zero
- El pad **5V** del S3-Zero acepta de 3,7 a 6 V, y su LDO de 3,3 V (ME6217C33, 800 mA) alimenta el pad **3V3**. [Waveshare](https://docs.waveshare.com/ESP32-S3-Zero)
- Poner un **diodo Schottky** (SS14 o similar) entre los 5 V del regulador y el pad 5V del S3-Zero para que el regulador y el USB no se alimenten entre sí. El S3-Zero recibe unos 4,6 V, dentro de su rango.
- Los Sharp toman los 5 V **antes** del diodo. Con solo USB conectado los Sharp y los motores quedan apagados, que es lo deseable al programar.
- **[verificar]** cómo conecta Waveshare VBUS y el pad 5V dentro de la placa antes de depender de este diodo.

### 4.4 Tensión de los motores
El TB6612FNG admite **VM de 4,5 a 13,5 V** y 1,2 A por canal (3,2 A de pico). Se propone **VM directo de VBAT** (hasta 8,4 V).
- **Los N20 son de 6 V** (dato confirmado). Con la batería llena (8,4 V) el PWM máximo para un promedio de 6 V es 6/8,4 ≈ 71 %, es decir **unos 180 de 255**; con 7,4 V sería 81 % (≈ 207). Se propone **tope fijo de 180** en el firmware al principio y subirlo solo si los motores no se calientan. Con la lectura de batería (`VBAT_SENSE`, §5.1) el límite se ajusta en vivo y el tope fijo queda solo como respaldo.
- Se deja un **puente de soldadura de 3 pads** (SJ1) para elegir VM = VBAT (por defecto) o VM = +5 V. Con 5 V la hoja de datos recomienda no pasar de 0,4 A por canal por debajo de 5 V.

## 5. TB6612FNG

Datos de la [hoja de datos](https://faculty-web.msoe.edu/johnsontimoj/Common/FILES/TB6612FNG.pdf): Vcc 2,7–5,5 V; VM 4,5–13,5 V; las entradas IN1, IN2, PWM y STBY tienen **pull-down interno de 200 kΩ**; PWM hasta 100 kHz.

### Conexiones

| Pin | Nombre | Destino |
| ---: | --- | --- |
| 1, 2 | AO1 | Motor derecho `M_DER+` (unir los dos pines) |
| 5, 6 | AO2 | Motor derecho `M_DER-` (unir) |
| 11, 12 | BO1 | Motor izquierdo `M_IZQ+` (unir) |
| 7, 8 | BO2 | Motor izquierdo `M_IZQ-` (unir) |
| 3, 4, 9, 10 | PGND | GND (todos) |
| 13, 14, 24 | VM | VM (los tres, unidos) |
| 18 | GND | GND |
| 20 | Vcc | +3,3 V |
| 19 | STBY | `STBY` (GPIO43) + pull-down de 100 kΩ a GND |
| 21 | AIN1 | `AIN1` (GPIO11) |
| 22 | AIN2 | `AIN2` (GPIO12) |
| 23 | PWMA | `PWMA` (GPIO14) |
| 17 | BIN1 | `BIN1` (GPIO13) |
| 16 | BIN2 | `BIN2` (GPIO44) |
| 15 | PWMB | `PWMB` (GPIO15) |

### Cómo se maneja (versión estable: PWM en PWMA/PWMB)
- `PWMA`/`PWMB` reciben el PWM (GPIO14/15) y `AIN1/AIN2`, `BIN1/BIN2` solo fijan el **sentido**.
- Avance: IN1 = alto, IN2 = bajo, PWM variable. Retroceso: IN1 = bajo, IN2 = alto.
- En la parte "apagada" del ciclo el puente queda en **freno** (los dos bajos de la carga en corto), no en rueda libre. Es el modo *drive/brake* (decaimiento lento): la corriente y el torque son mucho más lineales con el ciclo de trabajo, y eso es lo que mejor trata un lazo de control de velocidad.
- Freno total: IN1 = IN2 = alto. Parada libre: IN1 = IN2 = bajo con PWM en alto, o `STBY` en bajo.
- Tabla de verdad según la hoja de datos (el texto extraído estaba parcialmente cortado) **[verificar en el PDF original]**.

### Capacitores (circuito típico de la hoja de datos)
- **VM:** 10 µF + 100 nF cerca de los pines 13/14/24, más el de 100 µF de §4.1.
- **Vcc:** 10 µF + 100 nF cerca del pin 20.
- La hoja de datos pide que estos capacitores estén **lo más cerca posible del chip**.

### 5.1 Para que el manejo sea lo más estable posible

**Hardware**
- PWM en PWMA/PWMB con decaimiento lento (arriba). Es el cambio que más pesa.
- **Capacitor de 100 nF cerámico entre los dos bornes de cada motor**, soldado en el motor o lo más cerca posible, y opcionalmente 100 nF de cada borne a la carcasa metálica. Baja el ruido de las escobillas que, si no, aparece en los encoders y en el ADC de los Sharp.
- Condensadores del driver (§5): 10 µF + 100 nF en VM y en Vcc pegados al chip, más 100 µF en VBAT.
- Alimentación del driver directa de VBAT, **separada de los 5 V de los sensores**, con retorno de GND corto y ancho (plano de GND, sin pasar por debajo del S3-Zero).
- Cables de encoder cortos y lejos de los cables de motor. Opcional: 100 pF entre cada línea de encoder y GND, y la huella de pull-up de 10 kΩ.
- **Medir la batería:** divisor de **100 kΩ / 47 kΩ** (de VBAT a GND, punto medio al GPIO16) con 100 nF a GND. A 8,4 V da unos 2,7 V, dentro del rango del ADC con 11 dB. Sirve para compensar la caída de la batería.

**Firmware**
- PWM a **20 kHz** (fuera de lo audible; el TB6612 admite hasta 100 kHz). Resolución de 10 bits.
- **Control de velocidad en lazo cerrado** con los encoders: PID de velocidad por rueda a 1 kHz, con avance directo (feedforward) más corrección, y anti-windup.
- **Compensación por tensión de batería:** el duty = (velocidad pedida) × (6 V / VBAT). Así un robot con batería llena y uno con batería gastada se comportan igual, y con 8,4 V no se superan los 6 V de los motores. Reemplaza el tope fijo de 180.
- **Rampas** de aceleración y frenado, y una zona muerta mínima (PWM por debajo del cual el motor no gira).
- Habilitar `STBY` solo después de iniciar y cortarlo ante cualquier timeout.
- Mantener la regla de no usar `Serial0` (GPIO43/44).

### Seguridad
- `STBY` con pull-down: el driver arranca **deshabilitado** y el firmware lo habilita (`digitalWrite(STBY, HIGH)`) solo después de inicializar.
- Esto reemplaza al jumper JP1 de la V2 y permite además cortar los motores por software (por ejemplo, por tiempo máximo sin avance).
- Sigue siendo necesario el **interruptor general**: STBY no es una parada de emergencia certificada.

## 6. Sensores Sharp GP2Y0E03 (×4)

Según la [nota de aplicación](https://www.makerguides.com/wp-content/uploads/2024/10/GP2Y0E03-application-notes.pdf): VDD de 2,7 a 5,5 V; si VDD es 3,6 V o más, **VIN(IO) debe ir a 3,3 V o más**; salida analógica máxima de 2,3 V; consumo típico 26 mA (36 mA máximo).

| Pin del Sharp | Función | Conexión |
| ---: | --- | --- |
| 1 | VDD | **+5 V** (rama antes del Schottky) |
| 2 | Vout(A) | GPIO analógico del sensor |
| 3 | GND | GND |
| 4 | VIN(IO) | +3,3 V |
| 5 | GPIO1 (del sensor) | +3,3 V (sensor activo) |
| 6, 7 | SCL, SDA | Sin conectar |

Conector por sensor de **4 hilos: +5 V, +3,3 V, GND, OUT**; los pines 4 y 5 del Sharp se unen al hilo de 3,3 V en el lado del sensor.
- La salida máxima de 2,3 V es compatible con el ADC del S3 usando atenuación de 11 dB (rango útil hasta unos 3,1 V).
- **Capacitores:** 10 µF + 100 nF en +5 V cerca de cada conector (la hoja de la nota no especifica valor, es práctica estándar). Opcional: 100 nF de OUT a GND en cada entrada del ADC.
- Pistas de las señales analógicas lejos de las salidas de motores y del regulador conmutado (si se usa).
- Los umbrales de la V2 hay que volver a calibrarlos: cambia el ADC y hay un sensor más. El panel Sensores del programa de calibración sirve para eso.

### 6.1 Ubicación y ángulos recomendados

| Sensor | Ángulo respecto al frente | Dónde | Para qué |
| --- | --- | --- | --- |
| `SH_I` / `SH_D` (laterales) | **90°** (perpendicular a la marcha) | Un poco **adelante del eje** (10–20 mm) y **hundidos hacia adentro** del chasis, ver abajo | Seguir pared, centrar en el pasillo y detectar aberturas laterales antes de pasar |
| `SH_FI` / `SH_FD` (frontales) | **0°** (paralelos al frente) | En la punta del robot, **separados 40–60 mm entre sí**, simétricos respecto del eje | Detectar la pared de enfrente y **alinear el robot contra ella** (cuando las dos lecturas coinciden, el robot está perpendicular) |

Por qué esos ángulos:
- Con los frontales paralelos, la diferencia `SH_FI − SH_FD` es proporcional al error de orientación (con 50 mm de separación, 5° de giro dan unos 4 mm de diferencia), y la media es la distancia a la pared. Es lo más simple de programar.
- Los laterales a 90° dan una lectura directamente proporcional a la distancia a la pared lateral, que es lo que necesita el control de centrado.

**Restricción importante: el Sharp GP2Y0E03 mide de 4 a 50 cm.** Por debajo de 4 cm la lectura deja de ser confiable. Con las medidas del STL (base de 91 mm de ancho, frente a 50 mm del eje) y paredes de 12 mm:

| Celda | Pasillo libre | Margen lateral centrado (cada lado) | Distancia punta–pared de enfrente al estar en el centro |
| --- | ---: | ---: | ---: |
| 18 cm | 168 mm | (168 − 91) / 2 ≈ **38 mm** | 84 − 50 = **34 mm** |
| 25 cm | 238 mm | (238 − 91) / 2 ≈ 73 mm | 119 − 50 = 69 mm |

En la celda de 18 cm (la que queda desde 2027) **los dos casos están por debajo de 40 mm**:
- **Laterales:** hundir cada sensor unos **15–20 mm hacia adentro** respecto del borde del chasis (con ventana o corte), para que centrado lea unos 55–60 mm. Si no se puede, inclinarlos **30° hacia adelante**: la distancia medida pasa a ser d / sin 30° = 2d (≈ 77 mm centrado), pero el error de orientación se mezcla con la lectura de distancia.
- **Frontales:** la pared de enfrente se decide al **entrar en la celda**, unos **20–40 mm antes del centro**, cuando el sensor lee más de 55 mm. Después se llega al centro **por encoders**, sin usar el Sharp.

[verificar] las posiciones reales de montaje contra el CAD del chasis antes de fijar los agujeros.

## 7. Sensor de piso QRE1113GR

Igual que en la V2: plaqueta externa y un conector de 3 pines.

| Pin | Red |
| ---: | --- |
| 1 | GND |
| 2 | +3,3 V |
| 3 | `SENSOR` (GPIO6) |

En la plaqueta: LED con 150 Ω a +3,3 V y fototransistor con pull-up de 10 kΩ a +3,3 V. No agregar otro pull-up en la placa principal.

## 8. Motores N20 con encoder

Conector de 6 pines por motor, en el mismo orden eléctrico que la V2:

| Pin | Motor izquierdo | Motor derecho |
| ---: | --- | --- |
| 1 | `M_IZQ+` (BO1) | `M_DER+` (AO1) |
| 2 | `M_IZQ-` (BO2) | `M_DER-` (AO2) |
| 3 | +3,3 V encoder | +3,3 V encoder |
| 4 | GND encoder | GND encoder |
| 5 | `ENC_IA` (GPIO7) | `ENC_DA` (GPIO9) |
| 6 | `ENC_IB` (GPIO8) | `ENC_DB` (GPIO10) |

- Los encoders van a **3,3 V**, no a 5 V.
- Dejar huella opcional (sin soldar) para pull-ups de 10 kΩ en A/B por si las salidas del Hall son de colector abierto.
- El orden de los cables del motor puede no coincidir con el del conector: verificar cada hilo con el motor real antes de conectar.

## 9. Pulsador START

- SW1 normalmente abierto entre `START` (GPIO3) y GND.
- R de 10 kΩ de `START` a +3,3 V y 100 nF a GND (antirrebote por hardware, además del software).
- El S3-Zero ya tiene su propio botón BOOT en GPIO0 pero **no está accesible** una vez montado en el robot, por eso START es un pulsador aparte.
- El reglamento exige un pulsador visible y accesible para iniciar y detener el robot.

## 10. Masas
GND común: batería, regulador, S3-Zero, TB6612FNG (GND y PGND), Sharp, encoders y plaqueta de piso. Plano de GND continuo en la capa inferior; las corrientes de motor no deben pasar por debajo del S3-Zero ni de los conectores de sensores.

## 11. Layout (para el PCB)
- Dejar libre la zona de la **antena cerámica** del S3-Zero: sin cobre, pistas ni piezas debajo ni alrededor. [Waveshare lo indica](https://docs.waveshare.com/ESP32-S3-Zero)
- Dimensiones y paso de los pads del S3-Zero: **tomarlas del plano de Waveshare** [verificar]. Considerar montarlo sobre cabezales hembra para poder cambiarlo.
- Mantener la huella mecánica de la V2 (4 tornillos M3 a 54 × 15,55 mm) si el chasis no cambia. [verificar]
- Pistas de potencia (VBAT, VM, salidas de motores) anchas, por ejemplo 0,8–1 mm.
- Buck y TB6612 lejos de las entradas analógicas de los Sharp.

## 12. Impacto en el firmware (`AUS_KIM_V2_CALIB`)
- Placa: **ESP32S3 Dev Module**, "USB CDC On Boot: Enabled".
- Cambiar la tabla de pines a la de §3 y agregar el cuarto Sharp (`SH_FI`/`SH_FD`) y la salida `STBY`. Pared de enfrente = las dos lecturas frontales por encima del umbral.
- Habilitar `STBY` después de iniciar los motores en estado seguro y apagarlo al detener.
- `setMotor()` y el control por PWM no cambian. Mantener la protección de no usar Serial en GPIO43/44.
- Revisar la atenuación del ADC y recalibrar umbrales.
- Los N20 son de 6 V: **tope de PWM en 180** al principio (§4.4).

## 13. Lista de verificación antes de fabricar
1. Corriente de bloqueo de los N20 (la tensión nominal es 6 V).
2. Pinout del S3-Zero con el plano de Waveshare y huella exacta.
3. Conexión interna entre VBUS y el pad 5V (para el Schottky).
4. Pinout y rango de entrada del regulador que se compre (L7805CV o drop-in conmutado). Temperatura del 7805 en carga.
5. Tabla de verdad del TB6612FNG en el PDF original.
6. Que STBY se mantenga en bajo durante el arranque y que los motores no se muevan al encender.
7. Orden de cables de motor/encoder contra los conectores.
8. Ruido de los motores sobre el ADC de los Sharp.
9. ERC del esquema y DRC del PCB.

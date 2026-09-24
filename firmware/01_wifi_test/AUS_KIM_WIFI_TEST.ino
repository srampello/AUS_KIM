/*
 * AUS_KIM - Diagnostico Wi-Fi
 * ESP32-S3 SuperMini + 4x Sharp GP2Y0E03 + DRV8833 + 2 motores con encoders
 *
 * Objetivo:
 * - Crear una red Wi-Fi propia.
 * - Seleccionar y leer un sensor IR por vez en crudo (ADC).
 * - Probar cada motor por separado con PWM y sentido.
 * - Leer los encoders de cuadratura.
 * - Incluir STOP general y fail-safe por perdida de comunicacion.
 *
 * Red Wi-Fi:
 *   SSID: AUS_KIM
 *   Clave: AUSKIM2026
 *   Panel: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

// ============================================================
// 1. CONFIGURACION GENERAL
// ============================================================

const char* WIFI_SSID = "AUS_KIM";
const char* WIFI_PASS = "AUSKIM2026";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// Fail-safe: si un motor esta en movimiento y no llega heartbeat,
// se detienen ambos motores.
const uint32_t FAILSAFE_MS = 1000;
uint32_t lastHeartbeatMs = 0;

// PWM motores
const uint32_t PWM_FREQ = 20000; // 20 kHz: fuera del rango audible
const uint8_t PWM_BITS = 8;      // 0..255

// Sensores Sharp:
// El pin 5 (GPIO1) de los GP2Y0E03 NO esta conectado al ESP32.
// Por eso no se encienden/apagan sensores por software.
// La interfaz solo permite elegir CUAL sensor se lee y muestra.
const uint32_t SENSOR_READ_INTERVAL_MS = 20;

// Filtro de sensores:
// 1) Se toman 9 muestras rapidas.
// 2) Se calcula la mediana para eliminar picos.
// 3) Se aplica un suavizado exponencial equivalente a:
//       filtrado = 75% valor anterior + 25% mediana nueva
//
// Como las celdas del laberinto son de hasta 25 x 25 cm, mas adelante,
// cuando calibremos ADC -> cm, cualquier distancia > 25 cm podra tratarse
// como "sin pared cercana". Por ahora filtramos ADC sin inventar una
// conversion a centimetros.
const uint8_t SENSOR_MEDIAN_SAMPLES = 9;
const uint16_t SENSOR_SAMPLE_DELAY_US = 150;

// Si al probar un motor "Adelante" gira al reves, cambiar false -> true.
// Correcciones verificadas en el robot real:
// - El motor fisico izquierdo esta conectado al canal IN3/IN4.
// - El motor fisico derecho esta conectado al canal IN1/IN2.
// - El motor fisico izquierdo requiere invertir el sentido por software.
const bool INVERT_MOTOR_LEFT  = true;
const bool INVERT_MOTOR_RIGHT = false;

// ============================================================
// 2. PINES
// ============================================================

// Sharp GP2Y0E03 - Vout analogico
// Verificado fisicamente: los lados estaban cruzados respecto al esquema original.
const uint8_t PIN_IR_FRONT_LEFT  = 2;
const uint8_t PIN_IR_FRONT_RIGHT = 1;
const uint8_t PIN_IR_SIDE_LEFT   = 4;
const uint8_t PIN_IR_SIDE_RIGHT  = 3;

// DRV8833
// Verificado fisicamente: los canales del DRV8833 estaban cruzados respecto al esquema original.
const uint8_t PIN_MOTOR_L_IN1 = 7; // IN3 -> motor fisico izquierdo
const uint8_t PIN_MOTOR_L_IN2 = 8; // IN4 -> motor fisico izquierdo
const uint8_t PIN_MOTOR_R_IN1 = 5; // IN1 -> motor fisico derecho
const uint8_t PIN_MOTOR_R_IN2 = 6; // IN2 -> motor fisico derecho

// Encoders - cableado fisico original (NO se cambia).
const uint8_t PIN_ENC_L_A = 9;
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 11;
const uint8_t PIN_ENC_R_B = 12;

// Nota: en la interfaz los encoders se muestran intercambiados
// para corregir solamente la visualizacion, sin tocar el cableado.

// Canales LEDC para Arduino-ESP32 2.x
const uint8_t CH_L_IN1 = 0;
const uint8_t CH_L_IN2 = 1;
const uint8_t CH_R_IN1 = 2;
const uint8_t CH_R_IN2 = 3;

// ============================================================
// 3. SENSORES IR - SELECCION MANUAL
// ============================================================

enum SensorIndex : int8_t {
  SENSOR_NONE = -1,
  SENSOR_FRONT_LEFT = 0,
  SENSOR_FRONT_RIGHT,
  SENSOR_SIDE_LEFT,
  SENSOR_SIDE_RIGHT
};

uint16_t irFrontLeftRaw = 0;
uint16_t irFrontRightRaw = 0;
uint16_t irSideLeftRaw = 0;
uint16_t irSideRightRaw = 0;

uint16_t irFrontLeft = 0;
uint16_t irFrontRight = 0;
uint16_t irSideLeft = 0;
uint16_t irSideRight = 0;

bool irFrontLeftInit = false;
bool irFrontRightInit = false;
bool irSideLeftInit = false;
bool irSideRightInit = false;

SensorIndex selectedSensor = SENSOR_NONE;
uint32_t sensorLastReadMs = 0;

// ============================================================
// 4. ESTADO DE MOTORES
// ============================================================
//
// IMPORTANTE:
// Este enum se declara antes de la primera funcion del sketch.
// Arduino IDE genera prototipos automaticamente y, si MotorDir se declara
// despues, puede producir el error: 'MotorDir' has not been declared.
//
enum MotorDir : int8_t {
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
};

volatile MotorDir motorLeftDir = DIR_STOP;
volatile MotorDir motorRightDir = DIR_STOP;
volatile uint8_t motorLeftPwm = 0;
volatile uint8_t motorRightPwm = 0;

void selectSensor(SensorIndex sensor) {
  selectedSensor = sensor;
  sensorLastReadMs = 0;

  // Al seleccionar un sensor reiniciamos su filtro para que no arrastre
  // un valor viejo de una prueba anterior.
  switch (sensor) {
    case SENSOR_FRONT_LEFT:  irFrontLeftInit = false; break;
    case SENSOR_FRONT_RIGHT: irFrontRightInit = false; break;
    case SENSOR_SIDE_LEFT:   irSideLeftInit = false; break;
    case SENSOR_SIDE_RIGHT:  irSideRightInit = false; break;
    case SENSOR_NONE:
    default:
      break;
  }
}

const char* sensorName(SensorIndex sensor) {
  switch (sensor) {
    case SENSOR_FRONT_LEFT:  return "FL";
    case SENSOR_FRONT_RIGHT: return "FR";
    case SENSOR_SIDE_LEFT:   return "LL";
    case SENSOR_SIDE_RIGHT:  return "LR";
    case SENSOR_NONE:
    default:                 return "OFF";
  }
}

uint16_t readMedianADC(uint8_t pin) {
  uint16_t samples[SENSOR_MEDIAN_SAMPLES];

  for (uint8_t i = 0; i < SENSOR_MEDIAN_SAMPLES; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(SENSOR_SAMPLE_DELAY_US);
  }

  // Ordenamiento simple por insercion. Con 9 muestras es rapido y liviano.
  for (uint8_t i = 1; i < SENSOR_MEDIAN_SAMPLES; i++) {
    uint16_t key = samples[i];
    int8_t j = i - 1;

    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }

  return samples[SENSOR_MEDIAN_SAMPLES / 2];
}

uint16_t applySensorFilter(uint16_t medianValue,
                           uint16_t previousFiltered,
                           bool &initialized) {
  if (!initialized) {
    initialized = true;
    return medianValue;
  }

  // EMA alpha = 0.25, usando enteros:
  // nuevo = (3 * anterior + nueva_mediana) / 4
  return (uint16_t)(((uint32_t)previousFiltered * 3UL + medianValue) / 4UL);
}

void updateSelectedSensor() {
  if (selectedSensor == SENSOR_NONE) {
    return;
  }

  uint32_t now = millis();

  if (sensorLastReadMs != 0 && now - sensorLastReadMs < SENSOR_READ_INTERVAL_MS) {
    return;
  }

  sensorLastReadMs = now;

  switch (selectedSensor) {
    case SENSOR_FRONT_LEFT:
      irFrontLeftRaw = readMedianADC(PIN_IR_FRONT_LEFT);
      irFrontLeft = applySensorFilter(irFrontLeftRaw, irFrontLeft, irFrontLeftInit);
      break;

    case SENSOR_FRONT_RIGHT:
      irFrontRightRaw = readMedianADC(PIN_IR_FRONT_RIGHT);
      irFrontRight = applySensorFilter(irFrontRightRaw, irFrontRight, irFrontRightInit);
      break;

    case SENSOR_SIDE_LEFT:
      irSideLeftRaw = readMedianADC(PIN_IR_SIDE_LEFT);
      irSideLeft = applySensorFilter(irSideLeftRaw, irSideLeft, irSideLeftInit);
      break;

    case SENSOR_SIDE_RIGHT:
      irSideRightRaw = readMedianADC(PIN_IR_SIDE_RIGHT);
      irSideRight = applySensorFilter(irSideRightRaw, irSideRight, irSideRightInit);
      break;

    case SENSOR_NONE:
    default:
      break;
  }
}

// ============================================================
// 5. ENCODERS
// ============================================================

volatile int32_t encoderLeft = 0;
volatile int32_t encoderRight = 0;

volatile uint8_t lastStateLeft = 0;
volatile uint8_t lastStateRight = 0;

// Tabla de decodificacion cuadratura x4.
// Indice = (estado_anterior << 2) | estado_actual
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR updateEncoderLeft() {
  uint8_t a = digitalRead(PIN_ENC_L_A);
  uint8_t b = digitalRead(PIN_ENC_L_B);
  uint8_t current = (a << 1) | b;
  uint8_t index = (lastStateLeft << 2) | current;
  encoderLeft += QUAD_TABLE[index];
  lastStateLeft = current;
}

void IRAM_ATTR updateEncoderRight() {
  uint8_t a = digitalRead(PIN_ENC_R_A);
  uint8_t b = digitalRead(PIN_ENC_R_B);
  uint8_t current = (a << 1) | b;
  uint8_t index = (lastStateRight << 2) | current;
  encoderRight += QUAD_TABLE[index];
  lastStateRight = current;
}

// ============================================================
// 6. PWM COMPATIBLE CON ARDUINO-ESP32 2.x / 3.x
// ============================================================

void setupPwmPin(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(channel, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWritePin(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// ============================================================
// 7. CONTROL DE MOTORES
// ============================================================

void setMotorRaw(uint8_t in1, uint8_t ch1,
                 uint8_t in2, uint8_t ch2,
                 MotorDir dir, uint8_t pwm,
                 bool invert) {

  if (dir == DIR_STOP || pwm == 0) {
    pwmWritePin(in1, ch1, 0);
    pwmWritePin(in2, ch2, 0);
    return;
  }

  MotorDir effectiveDir = dir;
  if (invert) {
    effectiveDir = (dir == DIR_FORWARD) ? DIR_REVERSE : DIR_FORWARD;
  }

  if (effectiveDir == DIR_FORWARD) {
    pwmWritePin(in1, ch1, pwm);
    pwmWritePin(in2, ch2, 0);
  } else {
    pwmWritePin(in1, ch1, 0);
    pwmWritePin(in2, ch2, pwm);
  }
}

void setMotorLeft(MotorDir dir, uint8_t pwm) {
  motorLeftDir = dir;
  motorLeftPwm = (dir == DIR_STOP) ? 0 : pwm;

  setMotorRaw(
    PIN_MOTOR_L_IN1, CH_L_IN1,
    PIN_MOTOR_L_IN2, CH_L_IN2,
    dir, pwm, INVERT_MOTOR_LEFT
  );
}

void setMotorRight(MotorDir dir, uint8_t pwm) {
  motorRightDir = dir;
  motorRightPwm = (dir == DIR_STOP) ? 0 : pwm;

  setMotorRaw(
    PIN_MOTOR_R_IN1, CH_R_IN1,
    PIN_MOTOR_R_IN2, CH_R_IN2,
    dir, pwm, INVERT_MOTOR_RIGHT
  );
}

void stopAllMotors() {
  setMotorLeft(DIR_STOP, 0);
  setMotorRight(DIR_STOP, 0);
}

// ============================================================
// 8. INTERFAZ WEB
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AUS_KIM | Diagnostico</title>
  <style>
    :root {
      --bg: #0d1117;
      --card: #161b22;
      --border: #30363d;
      --text: #e6edf3;
      --muted: #8b949e;
      --accent: #7c5cff;
      --good: #2ea043;
      --danger: #da3633;
      --button: #21262d;
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: var(--bg);
      color: var(--text);
    }

    .wrap {
      max-width: 1050px;
      margin: auto;
      padding: 18px;
    }

    h1 {
      margin: 0 0 4px;
      text-align: center;
      letter-spacing: 1px;
    }

    .subtitle {
      text-align: center;
      color: var(--muted);
      margin-bottom: 18px;
    }

    .status {
      display: flex;
      justify-content: center;
      gap: 10px;
      flex-wrap: wrap;
      margin-bottom: 18px;
    }

    .badge {
      border: 1px solid var(--border);
      border-radius: 999px;
      padding: 7px 11px;
      font-size: 13px;
      color: var(--muted);
      background: var(--card);
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
      gap: 14px;
    }

    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px;
    }

    .card h2 {
      margin: 0 0 14px;
      font-size: 18px;
    }

    .sensor-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    .sensor {
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px;
      text-align: center;
    }

    .sensor .label {
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 5px;
    }

    .sensor .value {
      font-size: 26px;
      font-weight: bold;
    }

    .motor-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }

    .motor-state {
      color: var(--muted);
      font-size: 13px;
    }

    input[type=range] {
      width: 100%;
      margin: 12px 0;
    }

    .pwm-row {
      display: flex;
      justify-content: space-between;
      color: var(--muted);
      font-size: 13px;
    }

    .buttons {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 8px;
      margin-top: 10px;
    }

    button {
      border: 1px solid var(--border);
      background: var(--button);
      color: var(--text);
      border-radius: 9px;
      padding: 12px 8px;
      font-weight: bold;
      cursor: pointer;
      touch-action: none;
    }

    button:active {
      transform: translateY(1px);
    }

    .forward { border-color: var(--good); }
    .reverse { border-color: #d29922; }
    .stop { border-color: var(--danger); }
    .sensor-btn.active {
      background: var(--good);
      border-color: var(--good);
    }
    .sensor-controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-top: 12px;
    }

    .encoder-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    .encoder {
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 14px;
      text-align: center;
    }

    .encoder .value {
      font-size: 28px;
      font-weight: bold;
      margin: 8px 0;
    }

    .enc-ab {
      color: var(--muted);
      font-size: 13px;
    }

    .reset {
      width: 100%;
      margin-top: 10px;
    }

    .emergency {
      width: 100%;
      margin-top: 14px;
      background: var(--danger);
      border-color: #ff6b68;
      font-size: 18px;
      padding: 16px;
    }

    .hint {
      color: var(--muted);
      font-size: 12px;
      margin-top: 10px;
      line-height: 1.4;
    }

    @media (max-width: 600px) {
      .wrap { padding: 10px; }
      .buttons { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>AUS_KIM</h1>
    <div class="subtitle">Panel de diagnostico - ESP32-S3</div>

    <div class="status">
      <div class="badge">Wi-Fi: <b>AUS_KIM</b></div>
      <div class="badge">IP: <b>192.168.4.1</b></div>
      <div class="badge">Uptime: <b id="uptime">0 s</b></div>
    </div>

    <div class="grid">
      <section class="card">
        <h2>Sensores IR - seleccion de lectura</h2>
        <div class="badge" style="margin-bottom:10px;">
          Sensor seleccionado: <b id="selectedSensor">NINGUNO</b>
        </div>

        <div class="sensor-grid">
          <div class="sensor">
            <div class="label">Frontal izquierdo</div>
            <div class="value" id="fl">--</div>
            <div class="enc-ab">Crudo: <b id="flRaw">--</b></div>
            <button class="sensor-btn" id="sensorFL" onclick="selectSensor('FL')">SELECCIONAR</button>
          </div>

          <div class="sensor">
            <div class="label">Frontal derecho</div>
            <div class="value" id="fr">--</div>
            <div class="enc-ab">Crudo: <b id="frRaw">--</b></div>
            <button class="sensor-btn" id="sensorFR" onclick="selectSensor('FR')">SELECCIONAR</button>
          </div>

          <div class="sensor">
            <div class="label">Lateral izquierdo</div>
            <div class="value" id="sl">--</div>
            <div class="enc-ab">Crudo: <b id="slRaw">--</b></div>
            <button class="sensor-btn" id="sensorLL" onclick="selectSensor('LL')">SELECCIONAR</button>
          </div>

          <div class="sensor">
            <div class="label">Lateral derecho</div>
            <div class="value" id="sr">--</div>
            <div class="enc-ab">Crudo: <b id="srRaw">--</b></div>
            <button class="sensor-btn" id="sensorLR" onclick="selectSensor('LR')">SELECCIONAR</button>
          </div>
        </div>

        <div class="sensor-controls">
          <button class="stop" onclick="selectSensor('OFF')">DETENER LECTURA</button>
          <button onclick="clearSensorValues()">LIMPIAR VALORES</button>
        </div>

        <div class="hint">
          El numero grande es el ADC filtrado (mediana de 9 muestras + suavizado).
          "Crudo" muestra la mediana instantanea. Los cuatro Sharp siguen alimentados;
          el ESP32 solo consulta el seleccionado.
        </div>
      </section>

      <section class="card">
        <div class="motor-title">
          <h2>Motor izquierdo</h2>
          <div class="motor-state" id="stateL">STOP</div>
        </div>

        <div class="pwm-row">
          <span>PWM</span>
          <span id="pwmLValue">80</span>
        </div>
        <input id="pwmL" type="range" min="0" max="255" value="80">

        <div class="buttons">
          <button class="reverse" id="leftReverse">ATRAS</button>
          <button class="stop" onclick="stopMotor('L')">STOP</button>
          <button class="forward" id="leftForward">ADELANTE</button>
        </div>

        <div class="hint">
          Para seguridad, ADELANTE y ATRAS funcionan mientras se mantiene
          presionado el boton.
        </div>
      </section>

      <section class="card">
        <div class="motor-title">
          <h2>Motor derecho</h2>
          <div class="motor-state" id="stateR">STOP</div>
        </div>

        <div class="pwm-row">
          <span>PWM</span>
          <span id="pwmRValue">80</span>
        </div>
        <input id="pwmR" type="range" min="0" max="255" value="80">

        <div class="buttons">
          <button class="reverse" id="rightReverse">ATRAS</button>
          <button class="stop" onclick="stopMotor('R')">STOP</button>
          <button class="forward" id="rightForward">ADELANTE</button>
        </div>

        <div class="hint">
          El fail-safe detiene ambos motores si deja de recibirse comunicacion
          durante mas de 1 segundo.
        </div>
      </section>

      <section class="card">
        <h2>Encoders</h2>
        <div class="encoder-grid">
          <div class="encoder">
            <div>Izquierdo</div>
            <div class="value" id="encL">0</div>
            <div class="enc-ab">
              A: <b id="encLA">0</b> |
              B: <b id="encLB">0</b>
            </div>
          </div>
          <div class="encoder">
            <div>Derecho</div>
            <div class="value" id="encR">0</div>
            <div class="enc-ab">
              A: <b id="encRA">0</b> |
              B: <b id="encRB">0</b>
            </div>
          </div>
        </div>
        <button class="reset" onclick="resetEncoders()">RESET ENCODERS</button>
      </section>
    </div>

    <button class="emergency" onclick="stopAll()">STOP GENERAL</button>
  </div>

<script>
  let activeMotor = null;
  let activeDir = null;
  let heartbeatTimer = null;

  function motorPwm(motor) {
    return document.getElementById(motor === 'L' ? 'pwmL' : 'pwmR').value;
  }

  async function sendMotor(motor, dir) {
    const pwm = motorPwm(motor);
    await fetch('/api/motor?motor=' + motor + '&dir=' + dir + '&pwm=' + pwm, {
      cache: 'no-store'
    });
  }

  async function stopMotor(motor) {
    if (activeMotor === motor) {
      activeMotor = null;
      activeDir = null;
      stopHeartbeat();
    }
    await fetch('/api/motor?motor=' + motor + '&dir=S&pwm=0', {
      cache: 'no-store'
    });
  }

  async function stopAll() {
    activeMotor = null;
    activeDir = null;
    stopHeartbeat();
    await fetch('/api/stop', { cache: 'no-store' });
  }

  function startHeartbeat() {
    stopHeartbeat();
    heartbeatTimer = setInterval(() => {
      if (activeMotor) {
        fetch('/api/ping', { cache: 'no-store' }).catch(() => {});
      }
    }, 250);
  }

  function stopHeartbeat() {
    if (heartbeatTimer) {
      clearInterval(heartbeatTimer);
      heartbeatTimer = null;
    }
  }

  function bindHold(buttonId, motor, dir) {
    const button = document.getElementById(buttonId);

    const start = async (event) => {
      event.preventDefault();
      activeMotor = motor;
      activeDir = dir;
      await sendMotor(motor, dir);
      startHeartbeat();
    };

    const stop = async (event) => {
      event.preventDefault();
      if (activeMotor === motor) {
        await stopMotor(motor);
      }
    };

    button.addEventListener('pointerdown', start);
    button.addEventListener('pointerup', stop);
    button.addEventListener('pointercancel', stop);
    button.addEventListener('pointerleave', (event) => {
      if (event.buttons !== 0) stop(event);
    });
  }

  bindHold('leftForward', 'L', 'F');
  bindHold('leftReverse', 'L', 'R');
  bindHold('rightForward', 'R', 'F');
  bindHold('rightReverse', 'R', 'R');

  document.getElementById('pwmL').addEventListener('input', async (e) => {
    document.getElementById('pwmLValue').textContent = e.target.value;
    if (activeMotor === 'L' && activeDir) await sendMotor('L', activeDir);
  });

  document.getElementById('pwmR').addEventListener('input', async (e) => {
    document.getElementById('pwmRValue').textContent = e.target.value;
    if (activeMotor === 'R' && activeDir) await sendMotor('R', activeDir);
  });

  async function selectSensor(sensor) {
    await fetch('/api/sensor?sensor=' + sensor, { cache: 'no-store' });
    await updateStatus();
  }

  function clearSensorValues() {
    document.getElementById('fl').textContent = '--';
    document.getElementById('fr').textContent = '--';
    document.getElementById('sl').textContent = '--';
    document.getElementById('sr').textContent = '--';
    document.getElementById('flRaw').textContent = '--';
    document.getElementById('frRaw').textContent = '--';
    document.getElementById('slRaw').textContent = '--';
    document.getElementById('srRaw').textContent = '--';
  }

  function setActiveSensorUi(active) {
    const map = {
      FL: ['Frontal izquierdo', 'fl', 'sensorFL'],
      FR: ['Frontal derecho', 'fr', 'sensorFR'],
      LL: ['Lateral izquierdo', 'sl', 'sensorLL'],
      LR: ['Lateral derecho', 'sr', 'sensorLR']
    };

    ['sensorFL', 'sensorFR', 'sensorLL', 'sensorLR'].forEach(id => {
      document.getElementById(id).classList.remove('active');
    });

    ['fl', 'fr', 'sl', 'sr', 'flRaw', 'frRaw', 'slRaw', 'srRaw'].forEach(id => {
      document.getElementById(id).textContent = '--';
    });

    if (map[active]) {
      document.getElementById('selectedSensor').textContent = map[active][0];
      document.getElementById(map[active][2]).classList.add('active');
    } else {
      document.getElementById('selectedSensor').textContent = 'NINGUNO';
    }
  }

  async function resetEncoders() {
    await fetch('/api/reset_encoders', { cache: 'no-store' });
  }

  function dirText(value) {
    if (value === 1) return 'ADELANTE';
    if (value === -1) return 'ATRAS';
    return 'STOP';
  }

  async function updateStatus() {
    try {
      const response = await fetch('/api/status', { cache: 'no-store' });
      const data = await response.json();

      setActiveSensorUi(data.ir.active);

      if (data.ir.active === 'FL') {
        document.getElementById('fl').textContent = data.ir.fl;
        document.getElementById('flRaw').textContent = data.ir.flRaw;
      }
      if (data.ir.active === 'FR') {
        document.getElementById('fr').textContent = data.ir.fr;
        document.getElementById('frRaw').textContent = data.ir.frRaw;
      }
      if (data.ir.active === 'LL') {
        document.getElementById('sl').textContent = data.ir.sl;
        document.getElementById('slRaw').textContent = data.ir.slRaw;
      }
      if (data.ir.active === 'LR') {
        document.getElementById('sr').textContent = data.ir.sr;
        document.getElementById('srRaw').textContent = data.ir.srRaw;
      }

      // Correccion SOLO VISUAL: el cableado de los encoders no se modifica.
      document.getElementById('encL').textContent = data.enc.right;
      document.getElementById('encR').textContent = data.enc.left;

      document.getElementById('encLA').textContent = data.enc.ra;
      document.getElementById('encLB').textContent = data.enc.rb;
      document.getElementById('encRA').textContent = data.enc.la;
      document.getElementById('encRB').textContent = data.enc.lb;

      document.getElementById('stateL').textContent =
        dirText(data.motor.leftDir) + ' | PWM ' + data.motor.leftPwm;

      document.getElementById('stateR').textContent =
        dirText(data.motor.rightDir) + ' | PWM ' + data.motor.rightPwm;

      document.getElementById('uptime').textContent =
        Math.floor(data.uptimeMs / 1000) + ' s';

    } catch (error) {
      // Si se pierde el ESP32, el fail-safe del firmware detiene los motores.
    }
  }

  setInterval(updateStatus, 250);
  updateStatus();

  window.addEventListener('beforeunload', () => {
    try {
      navigator.sendBeacon('/api/stop');
    } catch (_) {}
  });
</script>
</body>
</html>
)HTML";

// ============================================================
// 9. API HTTP
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  int32_t encL;
  int32_t encR;

  noInterrupts();
  encL = encoderLeft;
  encR = encoderRight;
  interrupts();

  String json;
  json.reserve(320);

  json += "{";
  json += "\"ir\":{";
  json += "\"fl\":" + String(irFrontLeft) + ",";
  json += "\"fr\":" + String(irFrontRight) + ",";
  json += "\"sl\":" + String(irSideLeft) + ",";
  json += "\"sr\":" + String(irSideRight) + ",";
  json += "\"flRaw\":" + String(irFrontLeftRaw) + ",";
  json += "\"frRaw\":" + String(irFrontRightRaw) + ",";
  json += "\"slRaw\":" + String(irSideLeftRaw) + ",";
  json += "\"srRaw\":" + String(irSideRightRaw) + ",";
  json += "\"active\":\"" + String(sensorName(selectedSensor)) + "\"";
  json += "},";

  json += "\"enc\":{";
  json += "\"left\":" + String(encL) + ",";
  json += "\"right\":" + String(encR) + ",";
  json += "\"la\":" + String(digitalRead(PIN_ENC_L_A)) + ",";
  json += "\"lb\":" + String(digitalRead(PIN_ENC_L_B)) + ",";
  json += "\"ra\":" + String(digitalRead(PIN_ENC_R_A)) + ",";
  json += "\"rb\":" + String(digitalRead(PIN_ENC_R_B));
  json += "},";

  json += "\"motor\":{";
  json += "\"leftDir\":" + String((int)motorLeftDir) + ",";
  json += "\"leftPwm\":" + String((int)motorLeftPwm) + ",";
  json += "\"rightDir\":" + String((int)motorRightDir) + ",";
  json += "\"rightPwm\":" + String((int)motorRightPwm);
  json += "},";

  json += "\"uptimeMs\":" + String(millis());
  json += "}";

  server.send(200, "application/json", json);
}

void handleMotor() {
  if (!server.hasArg("motor") || !server.hasArg("dir") || !server.hasArg("pwm")) {
    server.send(400, "text/plain", "Parametros requeridos: motor, dir, pwm");
    return;
  }

  String motor = server.arg("motor");
  String dirArg = server.arg("dir");
  int pwmInt = constrain(server.arg("pwm").toInt(), 0, 255);
  uint8_t pwm = (uint8_t)pwmInt;

  MotorDir dir = DIR_STOP;
  if (dirArg == "F") dir = DIR_FORWARD;
  else if (dirArg == "R") dir = DIR_REVERSE;
  else if (dirArg == "S") dir = DIR_STOP;
  else {
    server.send(400, "text/plain", "dir debe ser F, R o S");
    return;
  }

  if (motor == "L") {
    setMotorLeft(dir, pwm);
  } else if (motor == "R") {
    setMotorRight(dir, pwm);
  } else {
    server.send(400, "text/plain", "motor debe ser L o R");
    return;
  }

  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleSensor() {
  if (!server.hasArg("sensor")) {
    server.send(400, "text/plain", "Parametro requerido: sensor");
    return;
  }

  String sensor = server.arg("sensor");

  if (sensor == "FL") {
    selectSensor(SENSOR_FRONT_LEFT);
  } else if (sensor == "FR") {
    selectSensor(SENSOR_FRONT_RIGHT);
  } else if (sensor == "LL") {
    selectSensor(SENSOR_SIDE_LEFT);
  } else if (sensor == "LR") {
    selectSensor(SENSOR_SIDE_RIGHT);
  } else if (sensor == "OFF") {
    selectSensor(SENSOR_NONE);
  } else {
    server.send(400, "text/plain", "sensor debe ser FL, FR, LL, LR u OFF");
    return;
  }

  server.send(200, "text/plain", sensorName(selectedSensor));
}

void handleStop() {
  stopAllMotors();
  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "STOP");
}

void handlePing() {
  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleResetEncoders() {
  noInterrupts();
  encoderLeft = 0;
  encoderRight = 0;
  interrupts();

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - No encontrado");
}

// ============================================================
// 10. SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=====================================");
  Serial.println(" AUS_KIM - Diagnostico Wi-Fi");
  Serial.println("=====================================");

  // ADC
  analogReadResolution(12); // ESP32-S3: 0..4095
  analogSetPinAttenuation(PIN_IR_FRONT_LEFT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_FRONT_RIGHT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_SIDE_LEFT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_SIDE_RIGHT, ADC_11db);

  // Los Sharp se leen solo por Vout analogico.
  // El pin 5 (GPIO1) de los sensores no esta conectado al ESP32.
  selectSensor(SENSOR_NONE);

  // PWM
  setupPwmPin(PIN_MOTOR_L_IN1, CH_L_IN1);
  setupPwmPin(PIN_MOTOR_L_IN2, CH_L_IN2);
  setupPwmPin(PIN_MOTOR_R_IN1, CH_R_IN1);
  setupPwmPin(PIN_MOTOR_R_IN2, CH_R_IN2);

  stopAllMotors();

  // Encoders
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);

  lastStateLeft =
    (digitalRead(PIN_ENC_L_A) << 1) |
    digitalRead(PIN_ENC_L_B);

  lastStateRight =
    (digitalRead(PIN_ENC_R_A) << 1) |
    digitalRead(PIN_ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), updateEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), updateEncoderRight, CHANGE);

  // Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);

  bool apStarted = WiFi.softAP(WIFI_SSID, WIFI_PASS);

  if (!apStarted) {
    Serial.println("ERROR: no se pudo iniciar el Access Point.");
  } else {
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  }

  // Rutas web
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/motor", HTTP_GET, handleMotor);
  server.on("/api/sensor", HTTP_GET, handleSensor);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.on("/api/ping", HTTP_GET, handlePing);
  server.on("/api/reset_encoders", HTTP_GET, handleResetEncoders);
  server.onNotFound(handleNotFound);

  server.begin();

  lastHeartbeatMs = millis();

  Serial.println("Servidor HTTP iniciado.");
  Serial.println("Abrir: http://192.168.4.1");
}

// ============================================================
// 11. LOOP
// ============================================================

void loop() {
  server.handleClient();
  updateSelectedSensor();

  bool anyMotorRunning =
    (motorLeftDir != DIR_STOP) ||
    (motorRightDir != DIR_STOP);

  if (anyMotorRunning && (millis() - lastHeartbeatMs > FAILSAFE_MS)) {
    stopAllMotors();
    Serial.println("FAIL-SAFE: motores detenidos por timeout.");
  }

  delay(1);
}

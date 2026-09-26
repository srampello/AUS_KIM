/*
 * AUS_KIM - Etapa 02: Seguimiento de pared derecha con PID ajustable por Wi-Fi
 *
 * Hardware:
 * - ESP32-S3 SuperMini
 * - Sharp GP2Y0E03 lateral derecho -> GPIO 3
 * - DRV8833
 * - Motor fisico izquierdo -> IN3/IN4 -> GPIO 7/8
 * - Motor fisico derecho   -> IN1/IN2 -> GPIO 5/6
 *
 * IMPORTANTE:
 * - No se usan encoders en esta etapa.
 * - Los 4 Sharp pueden permanecer activos fisicamente.
 * - El control usa solamente el Sharp lateral derecho.
 *
 * Calibracion lateral derecho medida:
 *   4 cm   -> 2485
 *   5 cm   -> 2447
 *   7.5 cm -> 2330
 *   10 cm  -> 2227
 *   12.5cm -> 2125
 *   15 cm  -> 2020
 *   20 cm  -> 1837
 *   25 cm  -> 1631
 *
 * Objetivo inicial para ~6 cm: ADC ~= 2400
 *
 * Wi-Fi:
 *   SSID: AUS_KIM_WALL
 *   Clave: AUSKIM2026
 *   Panel: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

// ============================================================
// 1. CONFIGURACION GENERAL
// ============================================================

const char* WIFI_SSID = "AUS_KIM_WALL";
const char* WIFI_PASS = "AUSKIM2026";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_BITS = 8;

const uint32_t CONTROL_INTERVAL_MS = 10;  // PID a 100 Hz
const uint32_t WEB_FAILSAFE_MS = 1500;    // Si se pierde la web, detiene el robot

// Filtro rapido: mediana de 5 + EMA 50/50
const uint8_t SENSOR_SAMPLES = 5;
const uint16_t SENSOR_SAMPLE_DELAY_US = 120;

// ============================================================
// 2. PINES
// ============================================================

// Sharp lateral derecho
const uint8_t PIN_IR_RIGHT = 3;

// DRV8833 - mapeo fisico verificado
const uint8_t PIN_MOTOR_L_IN1 = 7; // IN3
const uint8_t PIN_MOTOR_L_IN2 = 8; // IN4
const uint8_t PIN_MOTOR_R_IN1 = 5; // IN1
const uint8_t PIN_MOTOR_R_IN2 = 6; // IN2

const bool INVERT_MOTOR_LEFT = true;
const bool INVERT_MOTOR_RIGHT = false;

// Canales LEDC Arduino-ESP32 2.x
const uint8_t CH_L_IN1 = 0;
const uint8_t CH_L_IN2 = 1;
const uint8_t CH_R_IN1 = 2;
const uint8_t CH_R_IN2 = 3;

// ============================================================
// 3. TIPOS Y ESTADO
// ============================================================

enum MotorDir : int8_t {
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
};

struct PidConfig {
  float kp = 0.12f;
  float ki = 0.0f;
  float kd = 0.35f;

  int targetAdc = 2400;       // ~6 cm segun calibracion
  int basePwm = 80;           // velocidad inicial segura
  int maxCorrection = 60;     // limite de correccion
  int minWallAdc = 1500;      // debajo de esto asumimos pared perdida
};

PidConfig cfg;

bool running = false;
bool wallDetected = false;

uint16_t sensorRaw = 0;       // mediana instantanea
uint16_t sensorFiltered = 0;  // mediana + EMA
bool filterInitialized = false;

float errorPid = 0.0f;
float prevErrorPid = 0.0f;
float integralPid = 0.0f;
float derivativePid = 0.0f;
float correctionPid = 0.0f;

int motorLeftCmd = 0;
int motorRightCmd = 0;

uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

// ============================================================
// 4. PWM / MOTORES
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

void setMotorLeftForward(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorLeftCmd = pwm;

  setMotorRaw(
    PIN_MOTOR_L_IN1, CH_L_IN1,
    PIN_MOTOR_L_IN2, CH_L_IN2,
    DIR_FORWARD, (uint8_t)pwm,
    INVERT_MOTOR_LEFT
  );
}

void setMotorRightForward(int pwm) {
  pwm = constrain(pwm, 0, 255);
  motorRightCmd = pwm;

  setMotorRaw(
    PIN_MOTOR_R_IN1, CH_R_IN1,
    PIN_MOTOR_R_IN2, CH_R_IN2,
    DIR_FORWARD, (uint8_t)pwm,
    INVERT_MOTOR_RIGHT
  );
}

void stopMotors() {
  motorLeftCmd = 0;
  motorRightCmd = 0;

  setMotorRaw(
    PIN_MOTOR_L_IN1, CH_L_IN1,
    PIN_MOTOR_L_IN2, CH_L_IN2,
    DIR_STOP, 0,
    INVERT_MOTOR_LEFT
  );

  setMotorRaw(
    PIN_MOTOR_R_IN1, CH_R_IN1,
    PIN_MOTOR_R_IN2, CH_R_IN2,
    DIR_STOP, 0,
    INVERT_MOTOR_RIGHT
  );
}

// ============================================================
// 5. SENSOR Y FILTRO
// ============================================================

uint16_t readMedianADC(uint8_t pin) {
  uint16_t values[SENSOR_SAMPLES];

  for (uint8_t i = 0; i < SENSOR_SAMPLES; i++) {
    values[i] = analogRead(pin);
    delayMicroseconds(SENSOR_SAMPLE_DELAY_US);
  }

  for (uint8_t i = 1; i < SENSOR_SAMPLES; i++) {
    uint16_t key = values[i];
    int8_t j = i - 1;

    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }

    values[j + 1] = key;
  }

  return values[SENSOR_SAMPLES / 2];
}

void updateRightSensor() {
  sensorRaw = readMedianADC(PIN_IR_RIGHT);

  if (!filterInitialized) {
    sensorFiltered = sensorRaw;
    filterInitialized = true;
  } else {
    // EMA alpha = 0.5: respuesta rapida con algo de suavizado.
    sensorFiltered = (uint16_t)(((uint32_t)sensorFiltered + sensorRaw) / 2UL);
  }
}

// ============================================================
// 6. PID DE PARED DERECHA
// ============================================================

void resetPid() {
  errorPid = 0.0f;
  prevErrorPid = 0.0f;
  integralPid = 0.0f;
  derivativePid = 0.0f;
  correctionPid = 0.0f;
}

void updateWallPid() {
  updateRightSensor();

  wallDetected = sensorFiltered >= cfg.minWallAdc;

  if (!running) {
    stopMotors();
    return;
  }

  // Para esta etapa de ajuste seguimos una pared continua.
  // Si el sensor deja de ver una pared razonable, detenemos el robot.
  if (!wallDetected) {
    stopMotors();
    resetPid();
    return;
  }

  // ADC alto = pared mas cerca.
  // Objetivo medido: ~2400 corresponde aproximadamente a 6 cm.
  errorPid = (float)sensorFiltered - (float)cfg.targetAdc;

  integralPid += errorPid;
  integralPid = constrain(integralPid, -5000.0f, 5000.0f);

  derivativePid = errorPid - prevErrorPid;

  correctionPid =
      cfg.kp * errorPid +
      cfg.ki * integralPid +
      cfg.kd * derivativePid;

  correctionPid = constrain(
    correctionPid,
    -(float)cfg.maxCorrection,
    (float)cfg.maxCorrection
  );

  /*
   * Si esta demasiado cerca de la pared derecha:
   *   error > 0 -> correccion > 0
   *
   * Para alejarse hacia la izquierda:
   *   motor izquierdo mas lento
   *   motor derecho mas rapido
   */
  int leftPwm = cfg.basePwm - (int)correctionPid;
  int rightPwm = cfg.basePwm + (int)correctionPid;

  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);

  setMotorLeftForward(leftPwm);
  setMotorRightForward(rightPwm);

  prevErrorPid = errorPid;
}

// ============================================================
// 7. INTERFAZ WEB
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AUS_KIM | Wall Follow</title>
  <style>
    :root {
      --bg:#0d1117;
      --card:#161b22;
      --border:#30363d;
      --text:#e6edf3;
      --muted:#8b949e;
      --good:#2ea043;
      --danger:#da3633;
      --accent:#7c5cff;
      --button:#21262d;
    }

    * { box-sizing:border-box; }

    body {
      margin:0;
      font-family:Arial,Helvetica,sans-serif;
      background:var(--bg);
      color:var(--text);
    }

    .wrap {
      max-width:1000px;
      margin:auto;
      padding:16px;
    }

    h1 { text-align:center; margin-bottom:4px; }

    .subtitle {
      text-align:center;
      color:var(--muted);
      margin-bottom:16px;
    }

    .grid {
      display:grid;
      grid-template-columns:repeat(auto-fit,minmax(300px,1fr));
      gap:14px;
    }

    .card {
      background:var(--card);
      border:1px solid var(--border);
      border-radius:14px;
      padding:16px;
    }

    .metric-grid {
      display:grid;
      grid-template-columns:1fr 1fr;
      gap:8px;
    }

    .metric {
      border:1px solid var(--border);
      border-radius:10px;
      padding:10px;
      text-align:center;
    }

    .metric .label {
      color:var(--muted);
      font-size:12px;
    }

    .metric .value {
      font-size:24px;
      font-weight:bold;
      margin-top:4px;
    }

    .field {
      display:grid;
      grid-template-columns:1fr 110px;
      gap:10px;
      align-items:center;
      margin:9px 0;
    }

    .field span { color:var(--muted); }

    input {
      width:100%;
      padding:8px;
      border-radius:7px;
      border:1px solid var(--border);
      background:#0d1117;
      color:var(--text);
    }

    button {
      border:1px solid var(--border);
      background:var(--button);
      color:var(--text);
      border-radius:9px;
      padding:12px;
      font-weight:bold;
      cursor:pointer;
    }

    .full { width:100%; margin-top:10px; }

    .start {
      background:var(--good);
      border-color:var(--good);
      font-size:17px;
    }

    .stop {
      background:var(--danger);
      border-color:var(--danger);
      font-size:17px;
    }

    .status {
      padding:10px;
      border-radius:9px;
      text-align:center;
      font-weight:bold;
      margin-bottom:12px;
      border:1px solid var(--border);
    }

    .hint {
      color:var(--muted);
      font-size:12px;
      line-height:1.4;
      margin-top:10px;
    }

    @media(max-width:600px) {
      .wrap { padding:10px; }
    }
  </style>
</head>
<body>
<div class="wrap">
  <h1>AUS_KIM</h1>
  <div class="subtitle">Seguimiento de pared derecha - PID en vivo</div>

  <div class="grid">

    <section class="card">
      <h2>Control</h2>

      <div class="status" id="runState">DETENIDO</div>

      <button class="full start" onclick="runRobot(true)">INICIAR SEGUIMIENTO</button>
      <button class="full stop" onclick="runRobot(false)">STOP</button>

      <div class="hint">
        Durante esta etapa, si el lateral derecho deja de detectar una pared
        por encima del umbral configurado, los motores se detienen.
      </div>
    </section>

    <section class="card">
      <h2>PID y velocidad</h2>

      <div class="field">
        <span>Kp</span>
        <input id="kp" type="number" step="0.01">
      </div>

      <div class="field">
        <span>Ki</span>
        <input id="ki" type="number" step="0.001">
      </div>

      <div class="field">
        <span>Kd</span>
        <input id="kd" type="number" step="0.01">
      </div>

      <div class="field">
        <span>Objetivo ADC (~6 cm)</span>
        <input id="targetAdc" type="number" step="1">
      </div>

      <div class="field">
        <span>PWM base</span>
        <input id="basePwm" type="number" min="0" max="255" step="1">
      </div>

      <div class="field">
        <span>Correccion maxima</span>
        <input id="maxCorrection" type="number" min="0" max="255" step="1">
      </div>

      <div class="field">
        <span>Umbral pared minima</span>
        <input id="minWallAdc" type="number" min="0" max="4095" step="1">
      </div>

      <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>

      <div class="hint">
        Valores iniciales: Kp 0.12, Ki 0, Kd 0.35, objetivo 2400,
        PWM base 80 y correccion maxima 60.
      </div>
    </section>

    <section class="card">
      <h2>Sensor lateral derecho</h2>

      <div class="metric-grid">
        <div class="metric">
          <div class="label">ADC filtrado</div>
          <div class="value" id="sensorFiltered">0</div>
        </div>

        <div class="metric">
          <div class="label">ADC mediana</div>
          <div class="value" id="sensorRaw">0</div>
        </div>

        <div class="metric">
          <div class="label">Error</div>
          <div class="value" id="error">0</div>
        </div>

        <div class="metric">
          <div class="label">Correccion</div>
          <div class="value" id="correction">0</div>
        </div>
      </div>

      <div class="hint">
        ADC alto = mas cerca de la pared. El objetivo 2400 equivale
        aproximadamente a 6 cm con la calibracion actual.
      </div>
    </section>

    <section class="card">
      <h2>Motores</h2>

      <div class="metric-grid">
        <div class="metric">
          <div class="label">PWM izquierdo</div>
          <div class="value" id="motorLeft">0</div>
        </div>

        <div class="metric">
          <div class="label">PWM derecho</div>
          <div class="value" id="motorRight">0</div>
        </div>

        <div class="metric">
          <div class="label">Pared derecha</div>
          <div class="value" id="wall">NO</div>
        </div>

        <div class="metric">
          <div class="label">Loop PID</div>
          <div class="value">100 Hz</div>
        </div>
      </div>
    </section>

  </div>
</div>

<script>
  let firstConfigLoad = true;

  async function runRobot(state) {
    await fetch('/api/run?state=' + (state ? '1' : '0'), { cache:'no-store' });
    await updateStatus();
  }

  async function applyConfig() {
    const p = new URLSearchParams({
      kp: document.getElementById('kp').value,
      ki: document.getElementById('ki').value,
      kd: document.getElementById('kd').value,
      targetAdc: document.getElementById('targetAdc').value,
      basePwm: document.getElementById('basePwm').value,
      maxCorrection: document.getElementById('maxCorrection').value,
      minWallAdc: document.getElementById('minWallAdc').value
    });

    await fetch('/api/config?' + p.toString(), { cache:'no-store' });
    await updateStatus();
  }

  async function updateStatus() {
    try {
      const r = await fetch('/api/status', { cache:'no-store' });
      const d = await r.json();

      document.getElementById('runState').textContent =
        d.running ? 'SIGUIENDO PARED' : 'DETENIDO';

      document.getElementById('sensorFiltered').textContent = d.sensor.filtered;
      document.getElementById('sensorRaw').textContent = d.sensor.raw;
      document.getElementById('error').textContent = d.pid.error.toFixed(1);
      document.getElementById('correction').textContent = d.pid.correction.toFixed(1);

      document.getElementById('motorLeft').textContent = d.motor.left;
      document.getElementById('motorRight').textContent = d.motor.right;
      document.getElementById('wall').textContent = d.wallDetected ? 'SI' : 'NO';

      if (firstConfigLoad) {
        document.getElementById('kp').value = d.config.kp;
        document.getElementById('ki').value = d.config.ki;
        document.getElementById('kd').value = d.config.kd;
        document.getElementById('targetAdc').value = d.config.targetAdc;
        document.getElementById('basePwm').value = d.config.basePwm;
        document.getElementById('maxCorrection').value = d.config.maxCorrection;
        document.getElementById('minWallAdc').value = d.config.minWallAdc;
        firstConfigLoad = false;
      }
    } catch (_) {}
  }

  // Heartbeat de seguridad.
  setInterval(() => {
    fetch('/api/ping', { cache:'no-store' }).catch(() => {});
  }, 400);

  setInterval(updateStatus, 200);
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
// 8. API HTTP
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  String json;
  json.reserve(600);

  json += "{";

  json += "\"running\":" + String(running ? "true" : "false") + ",";
  json += "\"wallDetected\":" + String(wallDetected ? "true" : "false") + ",";

  json += "\"sensor\":{";
  json += "\"raw\":" + String(sensorRaw) + ",";
  json += "\"filtered\":" + String(sensorFiltered);
  json += "},";

  json += "\"pid\":{";
  json += "\"error\":" + String(errorPid, 2) + ",";
  json += "\"integral\":" + String(integralPid, 2) + ",";
  json += "\"derivative\":" + String(derivativePid, 2) + ",";
  json += "\"correction\":" + String(correctionPid, 2);
  json += "},";

  json += "\"motor\":{";
  json += "\"left\":" + String(motorLeftCmd) + ",";
  json += "\"right\":" + String(motorRightCmd);
  json += "},";

  json += "\"config\":{";
  json += "\"kp\":" + String(cfg.kp, 4) + ",";
  json += "\"ki\":" + String(cfg.ki, 4) + ",";
  json += "\"kd\":" + String(cfg.kd, 4) + ",";
  json += "\"targetAdc\":" + String(cfg.targetAdc) + ",";
  json += "\"basePwm\":" + String(cfg.basePwm) + ",";
  json += "\"maxCorrection\":" + String(cfg.maxCorrection) + ",";
  json += "\"minWallAdc\":" + String(cfg.minWallAdc);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

void handleRun() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Parametro requerido: state");
    return;
  }

  bool newState = server.arg("state") == "1";

  if (newState) {
    resetPid();
    filterInitialized = false;
    lastHeartbeatMs = millis();
    running = true;
  } else {
    running = false;
    stopMotors();
    resetPid();
  }

  server.send(200, "text/plain", running ? "RUN" : "STOP");
}

void handleConfig() {
  if (server.hasArg("kp")) cfg.kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) cfg.ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) cfg.kd = server.arg("kd").toFloat();

  if (server.hasArg("targetAdc")) {
    cfg.targetAdc = constrain(server.arg("targetAdc").toInt(), 0, 4095);
  }

  if (server.hasArg("basePwm")) {
    cfg.basePwm = constrain(server.arg("basePwm").toInt(), 0, 255);
  }

  if (server.hasArg("maxCorrection")) {
    cfg.maxCorrection = constrain(server.arg("maxCorrection").toInt(), 0, 255);
  }

  if (server.hasArg("minWallAdc")) {
    cfg.minWallAdc = constrain(server.arg("minWallAdc").toInt(), 0, 4095);
  }

  // Evita arrastrar integral/derivada al cambiar parametros.
  resetPid();

  server.send(200, "text/plain", "OK");
}

void handlePing() {
  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  running = false;
  stopMotors();
  resetPid();
  server.send(200, "text/plain", "STOP");
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - No encontrado");
}

// ============================================================
// 9. SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_IR_RIGHT, ADC_11db);

  setupPwmPin(PIN_MOTOR_L_IN1, CH_L_IN1);
  setupPwmPin(PIN_MOTOR_L_IN2, CH_L_IN2);
  setupPwmPin(PIN_MOTOR_R_IN1, CH_R_IN1);
  setupPwmPin(PIN_MOTOR_R_IN2, CH_R_IN2);

  stopMotors();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/run", HTTP_GET, handleRun);
  server.on("/api/config", HTTP_GET, handleConfig);
  server.on("/api/ping", HTTP_GET, handlePing);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.onNotFound(handleNotFound);

  server.begin();

  lastHeartbeatMs = millis();
  lastControlMs = millis();

  Serial.println();
  Serial.println("AUS_KIM - WALL FOLLOW");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

// ============================================================
// 10. LOOP
// ============================================================

void loop() {
  server.handleClient();

  uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updateWallPid();
  }

  if (running && (now - lastHeartbeatMs > WEB_FAILSAFE_MS)) {
    running = false;
    stopMotors();
    resetPid();
    Serial.println("FAIL-SAFE: perdida de comunicacion web.");
  }

  delay(1);
}

/*
 * AUS_KIM - Etapa 03: Mantener distancia a una pared frontal
 *
 * Prueba:
 * - Colocar una pared movil delante del robot.
 * - El robot avanza si la pared se aleja.
 * - El robot retrocede si la pared se acerca.
 * - Busca mantener una distancia fija usando el Sharp frontal derecho.
 *
 * NO usa encoders para el control.
 *
 * Sharp frontal derecho:
 *   GPIO 1
 *
 * Calibracion medida:
 *   4 cm   -> 2520
 *   5 cm   -> 2456
 *   7.5 cm -> 2370
 *   10 cm  -> 2256
 *   15 cm  -> 2057
 *   20 cm  -> 1842
 *   25 cm  -> 1460
 *
 * Objetivo inicial:
 *   10 cm ~= ADC 2256
 *
 * Wi-Fi:
 *   SSID: AUS_KIM_FRONT
 *   Clave: AUSKIM2026
 *   Panel: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

const char* WIFI_SSID = "AUS_KIM_FRONT";
const char* WIFI_PASS = "AUSKIM2026";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ============================================================
// CONFIGURACION
// ============================================================

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_BITS = 8;
const uint32_t CONTROL_INTERVAL_MS = 10; // 100 Hz
const uint32_t WEB_FAILSAFE_MS = 1500;

const uint8_t SENSOR_SAMPLES = 5;
const uint16_t SENSOR_SAMPLE_DELAY_US = 120;

// ============================================================
// PINES
// ============================================================

// Sharp frontal derecho
const uint8_t PIN_IR_FRONT = 1;

// DRV8833
const uint8_t PIN_MOTOR_L_IN1 = 7;
const uint8_t PIN_MOTOR_L_IN2 = 8;
const uint8_t PIN_MOTOR_R_IN1 = 5;
const uint8_t PIN_MOTOR_R_IN2 = 6;

const bool INVERT_MOTOR_LEFT = true;
const bool INVERT_MOTOR_RIGHT = false;

const uint8_t CH_L_IN1 = 0;
const uint8_t CH_L_IN2 = 1;
const uint8_t CH_R_IN1 = 2;
const uint8_t CH_R_IN2 = 3;

// ============================================================
// TIPOS Y ESTADO
// ============================================================

enum MotorDir : int8_t {
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
};

struct PidConfig {
  float kp = 0.10f;
  float ki = 0.0f;
  float kd = 0.20f;

  int targetAdc = 2256;   // ~10 cm
  int deadbandAdc = 12;   // zona donde no mueve motores
  int maxPwm = 90;        // limite de velocidad para esta prueba
  int minPwm = 40;        // vence zona muerta de motores
};

PidConfig cfg;

bool running = false;

uint16_t sensorRaw = 0;
uint16_t sensorFiltered = 0;
bool filterInitialized = false;

float errorPid = 0.0f;
float prevErrorPid = 0.0f;
float integralPid = 0.0f;
float derivativePid = 0.0f;
float correctionPid = 0.0f;

int motorCommand = 0; // + adelante, - atras, 0 stop

uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

// ============================================================
// PWM / MOTORES
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

void driveSigned(int command) {
  command = constrain(command, -255, 255);
  motorCommand = command;

  if (command == 0) {
    setMotorRaw(PIN_MOTOR_L_IN1, CH_L_IN1, PIN_MOTOR_L_IN2, CH_L_IN2,
                DIR_STOP, 0, INVERT_MOTOR_LEFT);
    setMotorRaw(PIN_MOTOR_R_IN1, CH_R_IN1, PIN_MOTOR_R_IN2, CH_R_IN2,
                DIR_STOP, 0, INVERT_MOTOR_RIGHT);
    return;
  }

  MotorDir dir = command > 0 ? DIR_FORWARD : DIR_REVERSE;
  uint8_t pwm = (uint8_t)abs(command);

  setMotorRaw(PIN_MOTOR_L_IN1, CH_L_IN1, PIN_MOTOR_L_IN2, CH_L_IN2,
              dir, pwm, INVERT_MOTOR_LEFT);
  setMotorRaw(PIN_MOTOR_R_IN1, CH_R_IN1, PIN_MOTOR_R_IN2, CH_R_IN2,
              dir, pwm, INVERT_MOTOR_RIGHT);
}

void stopMotors() {
  driveSigned(0);
}

// ============================================================
// SENSOR
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

void updateFrontSensor() {
  sensorRaw = readMedianADC(PIN_IR_FRONT);

  if (!filterInitialized) {
    sensorFiltered = sensorRaw;
    filterInitialized = true;
  } else {
    // EMA 50/50: suficientemente rapida para seguir una pared movil.
    sensorFiltered =
      (uint16_t)(((uint32_t)sensorFiltered + sensorRaw) / 2UL);
  }
}

// ============================================================
// PID DISTANCIA FRONTAL
// ============================================================

void resetPid() {
  errorPid = 0.0f;
  prevErrorPid = 0.0f;
  integralPid = 0.0f;
  derivativePid = 0.0f;
  correctionPid = 0.0f;
}

void updateDistancePid() {
  updateFrontSensor();

  if (!running) {
    stopMotors();
    return;
  }

  // ADC alto = pared mas cerca.
  errorPid = (float)sensorFiltered - (float)cfg.targetAdc;

  // Zona muerta alrededor del objetivo para que no vibre parado.
  if (abs((int)errorPid) <= cfg.deadbandAdc) {
    integralPid = 0.0f;
    derivativePid = 0.0f;
    correctionPid = 0.0f;
    prevErrorPid = errorPid;
    stopMotors();
    return;
  }

  integralPid += errorPid;
  integralPid = constrain(integralPid, -4000.0f, 4000.0f);

  derivativePid = errorPid - prevErrorPid;

  correctionPid =
      cfg.kp * errorPid +
      cfg.ki * integralPid +
      cfg.kd * derivativePid;

  correctionPid = constrain(
    correctionPid,
    -(float)cfg.maxPwm,
    (float)cfg.maxPwm
  );

  /*
   * error > 0: pared demasiado cerca -> retroceder
   * error < 0: pared demasiado lejos  -> avanzar
   *
   * Por eso se invierte el signo.
   */
  int cmd = -(int)correctionPid;

  // Compensacion de zona muerta mecanica.
  if (cmd > 0 && cmd < cfg.minPwm) cmd = cfg.minPwm;
  if (cmd < 0 && cmd > -cfg.minPwm) cmd = -cfg.minPwm;

  cmd = constrain(cmd, -cfg.maxPwm, cfg.maxPwm);

  driveSigned(cmd);

  prevErrorPid = errorPid;
}

// ============================================================
// INTERFAZ WEB
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM | Distancia frontal</title>
<style>
:root{
  --bg:#0d1117;--card:#161b22;--border:#30363d;
  --text:#e6edf3;--muted:#8b949e;--good:#2ea043;
  --danger:#da3633;--button:#21262d
}
*{box-sizing:border-box}
body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--bg);color:var(--text)}
.wrap{max-width:900px;margin:auto;padding:16px}
h1{text-align:center;margin-bottom:4px}
.subtitle{text-align:center;color:var(--muted);margin-bottom:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
.metric-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.metric{border:1px solid var(--border);border-radius:10px;padding:10px;text-align:center}
.label{color:var(--muted);font-size:12px}
.value{font-size:24px;font-weight:bold;margin-top:4px}
.field{display:grid;grid-template-columns:1fr 110px;gap:10px;align-items:center;margin:9px 0}
.field span{color:var(--muted)}
input{width:100%;padding:8px;border-radius:7px;border:1px solid var(--border);background:#0d1117;color:var(--text)}
button{width:100%;border:1px solid var(--border);background:var(--button);color:var(--text);border-radius:9px;padding:12px;font-weight:bold;cursor:pointer;margin-top:9px}
.start{background:var(--good);border-color:var(--good)}
.stop{background:var(--danger);border-color:var(--danger)}
.status{padding:10px;border:1px solid var(--border);border-radius:9px;text-align:center;font-weight:bold;margin-bottom:10px}
.hint{color:var(--muted);font-size:12px;line-height:1.4;margin-top:10px}
</style>
</head>
<body>
<div class="wrap">
  <h1>AUS_KIM</h1>
  <div class="subtitle">Prueba de distancia frontal constante</div>

  <div class="grid">
    <section class="card">
      <h2>Control</h2>
      <div class="status" id="runState">DETENIDO</div>
      <button class="start" onclick="runRobot(true)">INICIAR</button>
      <button class="stop" onclick="runRobot(false)">STOP</button>
      <div class="hint">
        Si acercas la pared, el robot debe retroceder. Si alejas la pared,
        debe avanzar. Cerca del objetivo queda detenido.
      </div>
    </section>

    <section class="card">
      <h2>PID</h2>

      <div class="field"><span>Kp</span><input id="kp" type="number" step="0.01"></div>
      <div class="field"><span>Ki</span><input id="ki" type="number" step="0.001"></div>
      <div class="field"><span>Kd</span><input id="kd" type="number" step="0.01"></div>
      <div class="field"><span>Objetivo ADC</span><input id="targetAdc" type="number" step="1"></div>
      <div class="field"><span>Zona muerta ADC</span><input id="deadbandAdc" type="number" step="1"></div>
      <div class="field"><span>PWM maximo</span><input id="maxPwm" type="number" min="0" max="255" step="1"></div>
      <div class="field"><span>PWM minimo</span><input id="minPwm" type="number" min="0" max="255" step="1"></div>

      <button onclick="applyConfig()">APLICAR PARAMETROS</button>

      <div class="hint">
        Inicio recomendado: Kp 0.10, Ki 0, Kd 0.20, objetivo 2256 (~10 cm).
      </div>
    </section>

    <section class="card">
      <h2>Telemetria</h2>
      <div class="metric-grid">
        <div class="metric"><div class="label">ADC filtrado</div><div class="value" id="filtered">0</div></div>
        <div class="metric"><div class="label">ADC mediana</div><div class="value" id="raw">0</div></div>
        <div class="metric"><div class="label">Error</div><div class="value" id="error">0</div></div>
        <div class="metric"><div class="label">Correccion</div><div class="value" id="correction">0</div></div>
        <div class="metric"><div class="label">Motor</div><div class="value" id="motor">0</div></div>
        <div class="metric"><div class="label">Movimiento</div><div class="value" id="direction">STOP</div></div>
      </div>
    </section>
  </div>
</div>

<script>
let firstLoad=true;

async function runRobot(state){
  await fetch('/api/run?state='+(state?'1':'0'),{cache:'no-store'});
  await updateStatus();
}

async function applyConfig(){
  const p=new URLSearchParams({
    kp:document.getElementById('kp').value,
    ki:document.getElementById('ki').value,
    kd:document.getElementById('kd').value,
    targetAdc:document.getElementById('targetAdc').value,
    deadbandAdc:document.getElementById('deadbandAdc').value,
    maxPwm:document.getElementById('maxPwm').value,
    minPwm:document.getElementById('minPwm').value
  });
  await fetch('/api/config?'+p.toString(),{cache:'no-store'});
  await updateStatus();
}

function dirText(v){
  if(v>0)return 'ADELANTE';
  if(v<0)return 'ATRAS';
  return 'STOP';
}

async function updateStatus(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    const d=await r.json();

    document.getElementById('runState').textContent=d.running?'ACTIVO':'DETENIDO';
    document.getElementById('filtered').textContent=d.sensor.filtered;
    document.getElementById('raw').textContent=d.sensor.raw;
    document.getElementById('error').textContent=d.pid.error.toFixed(1);
    document.getElementById('correction').textContent=d.pid.correction.toFixed(1);
    document.getElementById('motor').textContent=d.motor.command;
    document.getElementById('direction').textContent=dirText(d.motor.command);

    if(firstLoad){
      document.getElementById('kp').value=d.config.kp;
      document.getElementById('ki').value=d.config.ki;
      document.getElementById('kd').value=d.config.kd;
      document.getElementById('targetAdc').value=d.config.targetAdc;
      document.getElementById('deadbandAdc').value=d.config.deadbandAdc;
      document.getElementById('maxPwm').value=d.config.maxPwm;
      document.getElementById('minPwm').value=d.config.minPwm;
      firstLoad=false;
    }
  }catch(_){}
}

setInterval(()=>fetch('/api/ping',{cache:'no-store'}).catch(()=>{}),400);
setInterval(updateStatus,200);
updateStatus();

window.addEventListener('beforeunload',()=>{
  try{navigator.sendBeacon('/api/stop')}catch(_){}
});
</script>
</body>
</html>
)HTML";

// ============================================================
// API
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  String json;
  json.reserve(520);

  json += "{";
  json += "\"running\":" + String(running ? "true" : "false") + ",";

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
  json += "\"command\":" + String(motorCommand);
  json += "},";

  json += "\"config\":{";
  json += "\"kp\":" + String(cfg.kp, 4) + ",";
  json += "\"ki\":" + String(cfg.ki, 4) + ",";
  json += "\"kd\":" + String(cfg.kd, 4) + ",";
  json += "\"targetAdc\":" + String(cfg.targetAdc) + ",";
  json += "\"deadbandAdc\":" + String(cfg.deadbandAdc) + ",";
  json += "\"maxPwm\":" + String(cfg.maxPwm) + ",";
  json += "\"minPwm\":" + String(cfg.minPwm);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

void handleRun() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Parametro requerido: state");
    return;
  }

  if (server.arg("state") == "1") {
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

  if (server.hasArg("targetAdc"))
    cfg.targetAdc = constrain(server.arg("targetAdc").toInt(), 0, 4095);

  if (server.hasArg("deadbandAdc"))
    cfg.deadbandAdc = constrain(server.arg("deadbandAdc").toInt(), 0, 500);

  if (server.hasArg("maxPwm"))
    cfg.maxPwm = constrain(server.arg("maxPwm").toInt(), 0, 255);

  if (server.hasArg("minPwm"))
    cfg.minPwm = constrain(server.arg("minPwm").toInt(), 0, cfg.maxPwm);

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
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_IR_FRONT, ADC_11db);

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
  Serial.println("AUS_KIM - DISTANCIA FRONTAL");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updateDistancePid();
  }

  if (running && (now - lastHeartbeatMs > WEB_FAILSAFE_MS)) {
    running = false;
    stopMotors();
    resetPid();
    Serial.println("FAIL-SAFE: perdida de comunicacion web.");
  }

  delay(1);
}

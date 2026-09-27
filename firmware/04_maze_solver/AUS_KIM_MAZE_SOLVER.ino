/*
 * AUS_KIM - Etapa 04: Maze Solver / regla de la mano derecha
 *
 * Objetivo:
 * - Seguir la pared derecha con PID.
 * - Detectar aperturas a la derecha.
 * - Detectar pared frontal con los dos Sharp frontales.
 * - Prioridad de navegacion:
 *      1) Si hay apertura a la derecha -> girar a la derecha.
 *      2) Si hay pared frontal:
 *           - si derecha libre -> derecha
 *           - si izquierda libre -> izquierda
 *           - si ambos laterales bloqueados -> giro 180°
 *      3) Si no hay evento -> seguir pared derecha con PID.
 *
 * IMPORTANTE:
 * - Los encoders se muestran solo como telemetria.
 * - Los giros se hacen por tiempo porque el encoder derecho no es confiable.
 * - Todos los parametros principales se pueden ajustar desde la web.
 *
 * Pines sensores:
 *   Frontal izquierdo  -> GPIO 2
 *   Frontal derecho    -> GPIO 1
 *   Lateral izquierdo  -> GPIO 4
 *   Lateral derecho    -> GPIO 3
 *
 * Motores:
 *   Izquierdo -> GPIO 7/8
 *   Derecho   -> GPIO 5/6
 *
 * Encoders (solo telemetria):
 *   GPIO 9/10 y GPIO 11/12, con correccion visual ya verificada.
 *
 * Wi-Fi:
 *   SSID: AUS_KIM_MAZE
 *   Clave: AUSKIM2026
 *   Panel: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

const char* WIFI_SSID = "AUS_KIM_MAZE";
const char* WIFI_PASS = "AUSKIM2026";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ============================================================
// 1. CONFIGURACION BASE
// ============================================================

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_BITS = 8;

const uint32_t CONTROL_INTERVAL_MS = 10; // 100 Hz
const uint32_t WEB_FAILSAFE_MS = 1500;

const uint8_t SENSOR_SAMPLES = 5;
const uint16_t SENSOR_SAMPLE_DELAY_US = 100;

// ============================================================
// 2. PINES
// ============================================================

// Sharp GP2Y0E03
const uint8_t PIN_IR_FRONT_LEFT  = 2;
const uint8_t PIN_IR_FRONT_RIGHT = 1;
const uint8_t PIN_IR_SIDE_LEFT   = 4;
const uint8_t PIN_IR_SIDE_RIGHT  = 3;

// DRV8833
const uint8_t PIN_MOTOR_L_IN1 = 7;
const uint8_t PIN_MOTOR_L_IN2 = 8;
const uint8_t PIN_MOTOR_R_IN1 = 5;
const uint8_t PIN_MOTOR_R_IN2 = 6;

const bool INVERT_MOTOR_LEFT  = true;
const bool INVERT_MOTOR_RIGHT = false;

// Encoders - solo telemetria
const uint8_t PIN_ENC_L_A = 9;
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 11;
const uint8_t PIN_ENC_R_B = 12;

// LEDC
const uint8_t CH_L_IN1 = 0;
const uint8_t CH_L_IN2 = 1;
const uint8_t CH_R_IN1 = 2;
const uint8_t CH_R_IN2 = 3;

// ============================================================
// 3. TIPOS / CONFIGURACION
// ============================================================

enum MotorDir : int8_t {
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
};

enum RobotState : uint8_t {
  STATE_STOPPED = 0,
  STATE_FOLLOW,
  STATE_PRE_RIGHT,
  STATE_TURN_RIGHT,
  STATE_TURN_LEFT,
  STATE_UTURN,
  STATE_SETTLE
};

struct MazeConfig {
  // PID pared derecha
  float kp = 0.10f;
  float ki = 0.0f;
  float kd = 0.15f;

  int targetRightAdc = 2400;      // ~6 cm
  int basePwm = 70;
  int maxCorrection = 55;

  // Umbrales
  int frontWallAdc = 2200;        // ~10-11 cm aprox
  int rightOpenAdc = 1750;        // apertura lateral derecha
  int leftOpenAdc = 1750;         // provisional / ajustable

  // Maniobras temporizadas
  int turnPwm = 75;
  int rightAdvanceMs = 140;       // avanzar para centrar el robot en la abertura
  int rightTurnMs = 310;
  int leftTurnMs = 310;
  int uTurnMs = 620;
  int settleMs = 90;

  // Evita disparos repetidos de una misma abertura
  int junctionCooldownMs = 250;
};

MazeConfig cfg;

// ============================================================
// 4. ESTADO GENERAL
// ============================================================

bool running = false;
RobotState robotState = STATE_STOPPED;
uint32_t stateStartMs = 0;
uint32_t lastDecisionMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

int motorLeftCmd = 0;   // signed: + adelante, - atras
int motorRightCmd = 0;

// Sensores
struct SensorData {
  uint16_t raw = 0;
  uint16_t filtered = 0;
  bool initialized = false;
};

SensorData sFL, sFR, sLL, sLR;

// PID
float errorPid = 0.0f;
float prevErrorPid = 0.0f;
float integralPid = 0.0f;
float derivativePid = 0.0f;
float correctionPid = 0.0f;

// Flags de navegacion
bool frontBlocked = false;
bool rightOpen = false;
bool leftOpen = false;

// Encoders
volatile int32_t encoderLeft = 0;
volatile int32_t encoderRight = 0;
volatile uint8_t lastStateLeft = 0;
volatile uint8_t lastStateRight = 0;

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ============================================================
// 5. ENCODERS
// ============================================================

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
// 6. PWM / MOTORES
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

void setMotorSigned(bool left, int command) {
  command = constrain(command, -255, 255);

  MotorDir dir = DIR_STOP;
  uint8_t pwm = 0;

  if (command > 0) {
    dir = DIR_FORWARD;
    pwm = (uint8_t)command;
  } else if (command < 0) {
    dir = DIR_REVERSE;
    pwm = (uint8_t)(-command);
  }

  if (left) {
    motorLeftCmd = command;
    setMotorRaw(PIN_MOTOR_L_IN1, CH_L_IN1,
                PIN_MOTOR_L_IN2, CH_L_IN2,
                dir, pwm, INVERT_MOTOR_LEFT);
  } else {
    motorRightCmd = command;
    setMotorRaw(PIN_MOTOR_R_IN1, CH_R_IN1,
                PIN_MOTOR_R_IN2, CH_R_IN2,
                dir, pwm, INVERT_MOTOR_RIGHT);
  }
}

void setDrive(int leftCmd, int rightCmd) {
  setMotorSigned(true, leftCmd);
  setMotorSigned(false, rightCmd);
}

void stopMotors() {
  setDrive(0, 0);
}

// ============================================================
// 7. SENSORES
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

void updateOneSensor(uint8_t pin, SensorData &s) {
  s.raw = readMedianADC(pin);

  if (!s.initialized) {
    s.filtered = s.raw;
    s.initialized = true;
  } else {
    // EMA 50/50 para tener respuesta rapida.
    s.filtered = (uint16_t)(((uint32_t)s.filtered + s.raw) / 2UL);
  }
}

void updateAllSensors() {
  updateOneSensor(PIN_IR_FRONT_LEFT,  sFL);
  updateOneSensor(PIN_IR_FRONT_RIGHT, sFR);
  updateOneSensor(PIN_IR_SIDE_LEFT,   sLL);
  updateOneSensor(PIN_IR_SIDE_RIGHT,  sLR);

  // Cualquiera de los dos frontales puede declarar obstaculo.
  frontBlocked =
    (sFL.filtered >= cfg.frontWallAdc) ||
    (sFR.filtered >= cfg.frontWallAdc);

  rightOpen = sLR.filtered < cfg.rightOpenAdc;
  leftOpen  = sLL.filtered < cfg.leftOpenAdc;
}

// ============================================================
// 8. PID PARED DERECHA
// ============================================================

void resetPid() {
  errorPid = 0.0f;
  prevErrorPid = 0.0f;
  integralPid = 0.0f;
  derivativePid = 0.0f;
  correctionPid = 0.0f;
}

void followRightWall() {
  errorPid = (float)sLR.filtered - (float)cfg.targetRightAdc;

  integralPid += errorPid;
  integralPid = constrain(integralPid, -4000.0f, 4000.0f);

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

  // Demasiado cerca de la pared derecha -> gira hacia la izquierda.
  int leftPwm  = cfg.basePwm - (int)correctionPid;
  int rightPwm = cfg.basePwm + (int)correctionPid;

  leftPwm  = constrain(leftPwm,  0, 255);
  rightPwm = constrain(rightPwm, 0, 255);

  setDrive(leftPwm, rightPwm);

  prevErrorPid = errorPid;
}

// ============================================================
// 9. MAQUINA DE ESTADOS / NAVEGACION
// ============================================================

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_FOLLOW:      return "SIGUIENDO PARED";
    case STATE_PRE_RIGHT:   return "CENTRANDO PARA DERECHA";
    case STATE_TURN_RIGHT:  return "GIRO DERECHA";
    case STATE_TURN_LEFT:   return "GIRO IZQUIERDA";
    case STATE_UTURN:       return "GIRO 180";
    case STATE_SETTLE:      return "ESTABILIZANDO";
    case STATE_STOPPED:
    default:                return "DETENIDO";
  }
}

void enterState(RobotState newState) {
  robotState = newState;
  stateStartMs = millis();
  resetPid();
}

void startTurnRight() {
  enterState(STATE_TURN_RIGHT);
}

void startTurnLeft() {
  enterState(STATE_TURN_LEFT);
}

void startUTurn() {
  enterState(STATE_UTURN);
}

void runNavigation() {
  uint32_t now = millis();

  if (!running) {
    robotState = STATE_STOPPED;
    stopMotors();
    return;
  }

  switch (robotState) {

    case STATE_STOPPED:
      enterState(STATE_FOLLOW);
      break;

    case STATE_FOLLOW:
      /*
       * Regla de mano derecha:
       * 1) Si aparece una apertura derecha, la priorizamos.
       * 2) Si adelante esta bloqueado y no hay derecha:
       *      izquierda si esta libre, sino 180°.
       * 3) Si no hay evento, seguimos la pared derecha con PID.
       */
      if ((now - lastDecisionMs) >= (uint32_t)cfg.junctionCooldownMs) {

        if (rightOpen) {
          enterState(STATE_PRE_RIGHT);
          break;
        }

        if (frontBlocked) {
          if (leftOpen) {
            startTurnLeft();
          } else {
            startUTurn();
          }
          lastDecisionMs = now;
          break;
        }
      }

      followRightWall();
      break;

    case STATE_PRE_RIGHT:
      // Avanza un poco para que el centro del robot llegue a la abertura.
      setDrive(cfg.turnPwm, cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.rightAdvanceMs) {
        startTurnRight();
      }
      break;

    case STATE_TURN_RIGHT:
      // Pivot sobre el centro: izquierda adelante, derecha atras.
      setDrive(cfg.turnPwm, -cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.rightTurnMs) {
        lastDecisionMs = now;
        enterState(STATE_SETTLE);
      }
      break;

    case STATE_TURN_LEFT:
      // Pivot sobre el centro: izquierda atras, derecha adelante.
      setDrive(-cfg.turnPwm, cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.leftTurnMs) {
        lastDecisionMs = now;
        enterState(STATE_SETTLE);
      }
      break;

    case STATE_UTURN:
      setDrive(cfg.turnPwm, -cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.uTurnMs) {
        lastDecisionMs = now;
        enterState(STATE_SETTLE);
      }
      break;

    case STATE_SETTLE:
      setDrive(cfg.basePwm, cfg.basePwm);

      if (now - stateStartMs >= (uint32_t)cfg.settleMs) {
        enterState(STATE_FOLLOW);
      }
      break;
  }
}

// ============================================================
// 10. INTERFAZ WEB
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM | Maze Solver</title>
<style>
:root{
  --bg:#0d1117;--card:#161b22;--border:#30363d;--text:#e6edf3;
  --muted:#8b949e;--good:#2ea043;--danger:#da3633;--button:#21262d
}
*{box-sizing:border-box}
body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--bg);color:var(--text)}
.wrap{max-width:1100px;margin:auto;padding:16px}
h1{text-align:center;margin-bottom:4px}
.subtitle{text-align:center;color:var(--muted);margin-bottom:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
.metric-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.metric{border:1px solid var(--border);border-radius:10px;padding:10px;text-align:center}
.label{color:var(--muted);font-size:12px}
.value{font-size:22px;font-weight:bold;margin-top:4px}
.field{display:grid;grid-template-columns:1fr 115px;gap:10px;align-items:center;margin:8px 0}
.field span{color:var(--muted)}
input{width:100%;padding:8px;border-radius:7px;border:1px solid var(--border);background:#0d1117;color:var(--text)}
button{width:100%;border:1px solid var(--border);background:var(--button);color:var(--text);border-radius:9px;padding:12px;font-weight:bold;cursor:pointer;margin-top:9px}
.start{background:var(--good);border-color:var(--good)}
.stop{background:var(--danger);border-color:var(--danger)}
.status{padding:10px;border:1px solid var(--border);border-radius:9px;text-align:center;font-weight:bold;margin-bottom:10px}
.flags{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:10px}
.flag{padding:8px;border:1px solid var(--border);border-radius:8px;text-align:center;font-size:12px}
.hint{color:var(--muted);font-size:12px;line-height:1.4;margin-top:10px}
</style>
</head>
<body>
<div class="wrap">
  <h1>AUS_KIM</h1>
  <div class="subtitle">Maze Solver - regla de la mano derecha</div>

  <div class="grid">
    <section class="card">
      <h2>Control</h2>
      <div class="status" id="state">DETENIDO</div>
      <button class="start" onclick="runRobot(true)">INICIAR LABERINTO</button>
      <button class="stop" onclick="runRobot(false)">STOP</button>

      <div class="flags">
        <div class="flag">Frente<br><b id="frontFlag">LIBRE</b></div>
        <div class="flag">Derecha<br><b id="rightFlag">PARED</b></div>
        <div class="flag">Izquierda<br><b id="leftFlag">PARED</b></div>
      </div>

      <div class="hint">
        Prioridad: derecha libre -> giro derecho. Si el frente esta bloqueado,
        gira a la izquierda si puede; si no, realiza 180 grados.
      </div>
    </section>

    <section class="card">
      <h2>Sensores IR</h2>
      <div class="metric-grid">
        <div class="metric"><div class="label">Frontal izquierdo</div><div class="value" id="fl">0</div><div class="label">crudo <b id="flr">0</b></div></div>
        <div class="metric"><div class="label">Frontal derecho</div><div class="value" id="fr">0</div><div class="label">crudo <b id="frr">0</b></div></div>
        <div class="metric"><div class="label">Lateral izquierdo</div><div class="value" id="ll">0</div><div class="label">crudo <b id="llr">0</b></div></div>
        <div class="metric"><div class="label">Lateral derecho</div><div class="value" id="lr">0</div><div class="label">crudo <b id="lrr">0</b></div></div>
      </div>
    </section>

    <section class="card">
      <h2>PID pared derecha</h2>
      <div class="field"><span>Kp</span><input id="kp" type="number" step="0.01"></div>
      <div class="field"><span>Ki</span><input id="ki" type="number" step="0.001"></div>
      <div class="field"><span>Kd</span><input id="kd" type="number" step="0.01"></div>
      <div class="field"><span>Objetivo derecha ADC</span><input id="targetRightAdc" type="number" step="1"></div>
      <div class="field"><span>PWM base</span><input id="basePwm" type="number" min="0" max="255" step="1"></div>
      <div class="field"><span>Correccion maxima</span><input id="maxCorrection" type="number" min="0" max="255" step="1"></div>

      <div class="metric-grid">
        <div class="metric"><div class="label">Error</div><div class="value" id="error">0</div></div>
        <div class="metric"><div class="label">Correccion</div><div class="value" id="correction">0</div></div>
      </div>
    </section>

    <section class="card">
      <h2>Deteccion</h2>
      <div class="field"><span>Pared frontal ADC</span><input id="frontWallAdc" type="number" step="1"></div>
      <div class="field"><span>Apertura derecha ADC</span><input id="rightOpenAdc" type="number" step="1"></div>
      <div class="field"><span>Apertura izquierda ADC</span><input id="leftOpenAdc" type="number" step="1"></div>

      <div class="hint">
        Frontal: valor mayor al umbral = pared. Laterales: valor menor al umbral = apertura.
      </div>
    </section>

    <section class="card">
      <h2>Giros y velocidades</h2>
      <div class="field"><span>PWM giro</span><input id="turnPwm" type="number" min="0" max="255" step="1"></div>
      <div class="field"><span>Avance antes derecha ms</span><input id="rightAdvanceMs" type="number" min="0" step="10"></div>
      <div class="field"><span>Giro derecha ms</span><input id="rightTurnMs" type="number" min="0" step="10"></div>
      <div class="field"><span>Giro izquierda ms</span><input id="leftTurnMs" type="number" min="0" step="10"></div>
      <div class="field"><span>Giro 180 ms</span><input id="uTurnMs" type="number" min="0" step="10"></div>
      <div class="field"><span>Estabilizacion ms</span><input id="settleMs" type="number" min="0" step="10"></div>
      <div class="field"><span>Cooldown cruce ms</span><input id="junctionCooldownMs" type="number" min="0" step="10"></div>

      <button onclick="applyConfig()">APLICAR PARAMETROS</button>
    </section>

    <section class="card">
      <h2>Motores</h2>
      <div class="metric-grid">
        <div class="metric"><div class="label">Izquierdo</div><div class="value" id="motorL">0</div></div>
        <div class="metric"><div class="label">Derecho</div><div class="value" id="motorR">0</div></div>
      </div>
      <div class="hint">Positivo = adelante. Negativo = atras.</div>
    </section>

    <section class="card">
      <h2>Encoders - telemetria</h2>
      <div class="metric-grid">
        <div class="metric"><div class="label">Izquierdo</div><div class="value" id="encL">0</div><div class="label">A <b id="encLA">0</b> | B <b id="encLB">0</b></div></div>
        <div class="metric"><div class="label">Derecho</div><div class="value" id="encR">0</div><div class="label">A <b id="encRA">0</b> | B <b id="encRB">0</b></div></div>
      </div>
      <button onclick="resetEncoders()">RESET ENCODERS</button>
      <div class="hint">No se usan para decidir giros en esta version.</div>
    </section>
  </div>
</div>

<script>
let firstLoad=true;

async function runRobot(state){
  await fetch('/api/run?state='+(state?'1':'0'),{cache:'no-store'});
  await updateStatus();
}

async function resetEncoders(){
  await fetch('/api/reset_encoders',{cache:'no-store'});
}

async function applyConfig(){
  const ids=[
    'kp','ki','kd','targetRightAdc','basePwm','maxCorrection',
    'frontWallAdc','rightOpenAdc','leftOpenAdc','turnPwm',
    'rightAdvanceMs','rightTurnMs','leftTurnMs','uTurnMs',
    'settleMs','junctionCooldownMs'
  ];

  const p=new URLSearchParams();
  ids.forEach(id=>p.set(id,document.getElementById(id).value));

  await fetch('/api/config?'+p.toString(),{cache:'no-store'});
  await updateStatus();
}

async function updateStatus(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    const d=await r.json();

    document.getElementById('state').textContent=d.state;

    document.getElementById('fl').textContent=d.sensor.fl;
    document.getElementById('fr').textContent=d.sensor.fr;
    document.getElementById('ll').textContent=d.sensor.ll;
    document.getElementById('lr').textContent=d.sensor.lr;

    document.getElementById('flr').textContent=d.sensor.flRaw;
    document.getElementById('frr').textContent=d.sensor.frRaw;
    document.getElementById('llr').textContent=d.sensor.llRaw;
    document.getElementById('lrr').textContent=d.sensor.lrRaw;

    document.getElementById('frontFlag').textContent=d.flags.frontBlocked?'PARED':'LIBRE';
    document.getElementById('rightFlag').textContent=d.flags.rightOpen?'LIBRE':'PARED';
    document.getElementById('leftFlag').textContent=d.flags.leftOpen?'LIBRE':'PARED';

    document.getElementById('error').textContent=d.pid.error.toFixed(1);
    document.getElementById('correction').textContent=d.pid.correction.toFixed(1);

    document.getElementById('motorL').textContent=d.motor.left;
    document.getElementById('motorR').textContent=d.motor.right;

    document.getElementById('encL').textContent=d.enc.left;
    document.getElementById('encR').textContent=d.enc.right;
    document.getElementById('encLA').textContent=d.enc.la;
    document.getElementById('encLB').textContent=d.enc.lb;
    document.getElementById('encRA').textContent=d.enc.ra;
    document.getElementById('encRB').textContent=d.enc.rb;

    if(firstLoad){
      Object.keys(d.config).forEach(k=>{
        const el=document.getElementById(k);
        if(el)el.value=d.config[k];
      });
      firstLoad=false;
    }
  }catch(_){}
}

setInterval(()=>fetch('/api/ping',{cache:'no-store'}).catch(()=>{}),400);
setInterval(updateStatus,180);
updateStatus();

window.addEventListener('beforeunload',()=>{
  try{navigator.sendBeacon('/api/stop')}catch(_){}
});
</script>
</body>
</html>
)HTML";

// ============================================================
// 11. API
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  int32_t encPhysicalLeft;
  int32_t encPhysicalRight;

  noInterrupts();
  encPhysicalLeft = encoderRight;
  encPhysicalRight = -encoderLeft;
  interrupts();

  String json;
  json.reserve(1200);

  json += "{";
  json += "\"running\":" + String(running ? "true" : "false") + ",";
  json += "\"state\":\"" + String(stateName(robotState)) + "\",";

  json += "\"sensor\":{";
  json += "\"fl\":" + String(sFL.filtered) + ",";
  json += "\"fr\":" + String(sFR.filtered) + ",";
  json += "\"ll\":" + String(sLL.filtered) + ",";
  json += "\"lr\":" + String(sLR.filtered) + ",";
  json += "\"flRaw\":" + String(sFL.raw) + ",";
  json += "\"frRaw\":" + String(sFR.raw) + ",";
  json += "\"llRaw\":" + String(sLL.raw) + ",";
  json += "\"lrRaw\":" + String(sLR.raw);
  json += "},";

  json += "\"flags\":{";
  json += "\"frontBlocked\":" + String(frontBlocked ? "true" : "false") + ",";
  json += "\"rightOpen\":" + String(rightOpen ? "true" : "false") + ",";
  json += "\"leftOpen\":" + String(leftOpen ? "true" : "false");
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

  json += "\"enc\":{";
  json += "\"left\":" + String(encPhysicalLeft) + ",";
  json += "\"right\":" + String(encPhysicalRight) + ",";
  json += "\"la\":" + String(digitalRead(PIN_ENC_R_A)) + ",";
  json += "\"lb\":" + String(digitalRead(PIN_ENC_R_B)) + ",";
  json += "\"ra\":" + String(digitalRead(PIN_ENC_L_A)) + ",";
  json += "\"rb\":" + String(digitalRead(PIN_ENC_L_B));
  json += "},";

  json += "\"config\":{";
  json += "\"kp\":" + String(cfg.kp, 4) + ",";
  json += "\"ki\":" + String(cfg.ki, 4) + ",";
  json += "\"kd\":" + String(cfg.kd, 4) + ",";
  json += "\"targetRightAdc\":" + String(cfg.targetRightAdc) + ",";
  json += "\"basePwm\":" + String(cfg.basePwm) + ",";
  json += "\"maxCorrection\":" + String(cfg.maxCorrection) + ",";
  json += "\"frontWallAdc\":" + String(cfg.frontWallAdc) + ",";
  json += "\"rightOpenAdc\":" + String(cfg.rightOpenAdc) + ",";
  json += "\"leftOpenAdc\":" + String(cfg.leftOpenAdc) + ",";
  json += "\"turnPwm\":" + String(cfg.turnPwm) + ",";
  json += "\"rightAdvanceMs\":" + String(cfg.rightAdvanceMs) + ",";
  json += "\"rightTurnMs\":" + String(cfg.rightTurnMs) + ",";
  json += "\"leftTurnMs\":" + String(cfg.leftTurnMs) + ",";
  json += "\"uTurnMs\":" + String(cfg.uTurnMs) + ",";
  json += "\"settleMs\":" + String(cfg.settleMs) + ",";
  json += "\"junctionCooldownMs\":" + String(cfg.junctionCooldownMs);
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
    sFL.initialized = false;
    sFR.initialized = false;
    sLL.initialized = false;
    sLR.initialized = false;
    lastHeartbeatMs = millis();
    lastDecisionMs = millis();
    running = true;
    enterState(STATE_FOLLOW);
  } else {
    running = false;
    robotState = STATE_STOPPED;
    stopMotors();
    resetPid();
  }

  server.send(200, "text/plain", running ? "RUN" : "STOP");
}

void handleConfig() {
  if (server.hasArg("kp")) cfg.kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) cfg.ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) cfg.kd = server.arg("kd").toFloat();

  if (server.hasArg("targetRightAdc"))
    cfg.targetRightAdc = constrain(server.arg("targetRightAdc").toInt(), 0, 4095);

  if (server.hasArg("basePwm"))
    cfg.basePwm = constrain(server.arg("basePwm").toInt(), 0, 255);

  if (server.hasArg("maxCorrection"))
    cfg.maxCorrection = constrain(server.arg("maxCorrection").toInt(), 0, 255);

  if (server.hasArg("frontWallAdc"))
    cfg.frontWallAdc = constrain(server.arg("frontWallAdc").toInt(), 0, 4095);

  if (server.hasArg("rightOpenAdc"))
    cfg.rightOpenAdc = constrain(server.arg("rightOpenAdc").toInt(), 0, 4095);

  if (server.hasArg("leftOpenAdc"))
    cfg.leftOpenAdc = constrain(server.arg("leftOpenAdc").toInt(), 0, 4095);

  if (server.hasArg("turnPwm"))
    cfg.turnPwm = constrain(server.arg("turnPwm").toInt(), 0, 255);

  if (server.hasArg("rightAdvanceMs"))
    cfg.rightAdvanceMs = constrain(server.arg("rightAdvanceMs").toInt(), 0, 2000);

  if (server.hasArg("rightTurnMs"))
    cfg.rightTurnMs = constrain(server.arg("rightTurnMs").toInt(), 0, 3000);

  if (server.hasArg("leftTurnMs"))
    cfg.leftTurnMs = constrain(server.arg("leftTurnMs").toInt(), 0, 3000);

  if (server.hasArg("uTurnMs"))
    cfg.uTurnMs = constrain(server.arg("uTurnMs").toInt(), 0, 5000);

  if (server.hasArg("settleMs"))
    cfg.settleMs = constrain(server.arg("settleMs").toInt(), 0, 2000);

  if (server.hasArg("junctionCooldownMs"))
    cfg.junctionCooldownMs = constrain(server.arg("junctionCooldownMs").toInt(), 0, 3000);

  resetPid();

  server.send(200, "text/plain", "OK");
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

void handleStop() {
  running = false;
  robotState = STATE_STOPPED;
  stopMotors();
  resetPid();
  server.send(200, "text/plain", "STOP");
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - No encontrado");
}

// ============================================================
// 12. SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_IR_FRONT_LEFT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_FRONT_RIGHT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_SIDE_LEFT, ADC_11db);
  analogSetPinAttenuation(PIN_IR_SIDE_RIGHT, ADC_11db);

  setupPwmPin(PIN_MOTOR_L_IN1, CH_L_IN1);
  setupPwmPin(PIN_MOTOR_L_IN2, CH_L_IN2);
  setupPwmPin(PIN_MOTOR_R_IN1, CH_R_IN1);
  setupPwmPin(PIN_MOTOR_R_IN2, CH_R_IN2);

  stopMotors();

  // Encoders: solo telemetria
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

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/run", HTTP_GET, handleRun);
  server.on("/api/config", HTTP_GET, handleConfig);
  server.on("/api/ping", HTTP_GET, handlePing);
  server.on("/api/reset_encoders", HTTP_GET, handleResetEncoders);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.onNotFound(handleNotFound);

  server.begin();

  lastHeartbeatMs = millis();
  lastControlMs = millis();

  Serial.println();
  Serial.println("AUS_KIM - MAZE SOLVER");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

// ============================================================
// 13. LOOP
// ============================================================

void loop() {
  server.handleClient();

  uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;

    updateAllSensors();
    runNavigation();
  }

  if (running && (now - lastHeartbeatMs > WEB_FAILSAFE_MS)) {
    running = false;
    robotState = STATE_STOPPED;
    stopMotors();
    resetPid();
    Serial.println("FAIL-SAFE: perdida de comunicacion web.");
  }

  delay(1);
}

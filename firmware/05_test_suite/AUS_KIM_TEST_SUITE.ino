/*
 * AUS_KIM - Sistema unificado de pruebas
 *
 * Pestañas web:
 *  1) Diagnostico: sensores, encoders y motores.
 *  2) PID pared derecha: seguimiento de pared con parametros en vivo.
 *  3) Maze test: regla de la mano derecha con parametros en vivo.
 *
 * Hardware:
 * - ESP32-S3 SuperMini
 * - 4x Sharp GP2Y0E03, todos activos (GPIO1/pin 5 de cada Sharp a 3.3 V)
 * - DRV8833
 * - Encoders solo para telemetria/pruebas, NO usados por el control autonomo
 *
 * Sensores:
 *   Frontal izquierdo  -> GPIO 2
 *   Frontal derecho    -> GPIO 1
 *   Lateral izquierdo  -> GPIO 4
 *   Lateral derecho    -> GPIO 3
 *
 * Motores:
 *   Motor izquierdo -> GPIO 7/8
 *   Motor derecho   -> GPIO 5/6
 *
 * Encoders:
 *   Grupo nativo L -> GPIO 9/10
 *   Grupo nativo R -> GPIO 11/12
 *
 * Correccion fisica verificada:
 * - rueda izquierda fisica -> contador nativo "right"
 * - rueda derecha fisica   -> contador nativo "left", invertido de signo
 *
 * Wi-Fi:
 *   SSID: AUS_KIM_TEST
 *   Clave: AUSKIM2026
 *   Panel: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

// ============================================================
// 1. CONFIGURACION GENERAL
// ============================================================

const char* WIFI_SSID = "AUS_KIM_TEST";
const char* WIFI_PASS = "AUSKIM2026";

IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_BITS = 8;

const uint32_t CONTROL_INTERVAL_MS = 10; // 100 Hz
const uint32_t WEB_FAILSAFE_MS = 1500;

const uint8_t SENSOR_SAMPLES = 5;
const uint16_t SENSOR_SAMPLE_DELAY_US = 100;

// ============================================================
// 2. PINES
// ============================================================

// Sharp
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

// Encoders
const uint8_t PIN_ENC_L_A = 9;
const uint8_t PIN_ENC_L_B = 10;
const uint8_t PIN_ENC_R_A = 11;
const uint8_t PIN_ENC_R_B = 12;

// LEDC channels (Arduino-ESP32 2.x)
const uint8_t CH_L_IN1 = 0;
const uint8_t CH_L_IN2 = 1;
const uint8_t CH_R_IN1 = 2;
const uint8_t CH_R_IN2 = 3;

// ============================================================
// 3. TIPOS
// ============================================================

enum MotorDir : int8_t {
  DIR_REVERSE = -1,
  DIR_STOP = 0,
  DIR_FORWARD = 1
};

enum RunMode : uint8_t {
  MODE_TEST = 0,
  MODE_WALL,
  MODE_MAZE
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

struct SensorData {
  uint16_t raw = 0;
  uint16_t filtered = 0;
  bool initialized = false;
};

struct ControlConfig {
  // PID pared derecha
  float kp = 0.10f;
  float ki = 0.0f;
  float kd = 0.15f;

  int targetRightAdc = 2400;  // ~6 cm
  int basePwm = 70;
  int maxCorrection = 55;

  // Deteccion de laberinto
  int frontWallAdc = 2200;
  int rightOpenAdc = 1750;
  int leftOpenAdc = 1750;

  // Giros temporizados
  int turnPwm = 75;
  int rightAdvanceMs = 140;
  int rightTurnMs = 310;
  int leftTurnMs = 310;
  int uTurnMs = 620;
  int settleMs = 90;
  int junctionCooldownMs = 250;

  // Prueba manual
  int manualPwm = 80;
};

ControlConfig cfg;

// ============================================================
// 4. ESTADO GLOBAL
// ============================================================

RunMode activeMode = MODE_TEST;
bool running = false;

RobotState robotState = STATE_STOPPED;
uint32_t stateStartMs = 0;
uint32_t lastDecisionMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

SensorData sFL, sFR, sLL, sLR;

bool frontBlocked = false;
bool rightOpen = false;
bool leftOpen = false;

float errorPid = 0.0f;
float prevErrorPid = 0.0f;
float integralPid = 0.0f;
float derivativePid = 0.0f;
float correctionPid = 0.0f;

int motorLeftCmd = 0;   // signed
int motorRightCmd = 0;  // signed

// Encoders nativos
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
    setMotorRaw(
      PIN_MOTOR_L_IN1, CH_L_IN1,
      PIN_MOTOR_L_IN2, CH_L_IN2,
      dir, pwm, INVERT_MOTOR_LEFT
    );
  } else {
    motorRightCmd = command;
    setMotorRaw(
      PIN_MOTOR_R_IN1, CH_R_IN1,
      PIN_MOTOR_R_IN2, CH_R_IN2,
      dir, pwm, INVERT_MOTOR_RIGHT
    );
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
    // EMA 50/50
    s.filtered = (uint16_t)(((uint32_t)s.filtered + s.raw) / 2UL);
  }
}

void updateAllSensors() {
  updateOneSensor(PIN_IR_FRONT_LEFT,  sFL);
  updateOneSensor(PIN_IR_FRONT_RIGHT, sFR);
  updateOneSensor(PIN_IR_SIDE_LEFT,   sLL);
  updateOneSensor(PIN_IR_SIDE_RIGHT,  sLR);

  frontBlocked =
    (sFL.filtered >= cfg.frontWallAdc) ||
    (sFR.filtered >= cfg.frontWallAdc);

  rightOpen = sLR.filtered < cfg.rightOpenAdc;
  leftOpen  = sLL.filtered < cfg.leftOpenAdc;
}

// ============================================================
// 8. PID
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

  int leftPwm  = cfg.basePwm - (int)correctionPid;
  int rightPwm = cfg.basePwm + (int)correctionPid;

  leftPwm  = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);

  setDrive(leftPwm, rightPwm);

  prevErrorPid = errorPid;
}

// ============================================================
// 9. MAZE
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

const char* modeName(RunMode m) {
  switch (m) {
    case MODE_WALL: return "WALL";
    case MODE_MAZE: return "MAZE";
    case MODE_TEST:
    default:        return "TEST";
  }
}

void enterState(RobotState newState) {
  robotState = newState;
  stateStartMs = millis();
  resetPid();
}

void runMaze() {
  uint32_t now = millis();

  switch (robotState) {
    case STATE_STOPPED:
      enterState(STATE_FOLLOW);
      break;

    case STATE_FOLLOW:
      if ((now - lastDecisionMs) >= (uint32_t)cfg.junctionCooldownMs) {

        // Regla de mano derecha: derecha siempre tiene prioridad.
        if (rightOpen) {
          enterState(STATE_PRE_RIGHT);
          break;
        }

        // Si no hay derecha y el frente esta bloqueado:
        // izquierda si esta libre; caso contrario 180 grados.
        if (frontBlocked) {
          if (leftOpen) {
            enterState(STATE_TURN_LEFT);
          } else {
            enterState(STATE_UTURN);
          }

          lastDecisionMs = now;
          break;
        }
      }

      followRightWall();
      break;

    case STATE_PRE_RIGHT:
      setDrive(cfg.turnPwm, cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.rightAdvanceMs) {
        enterState(STATE_TURN_RIGHT);
      }
      break;

    case STATE_TURN_RIGHT:
      setDrive(cfg.turnPwm, -cfg.turnPwm);

      if (now - stateStartMs >= (uint32_t)cfg.rightTurnMs) {
        lastDecisionMs = now;
        enterState(STATE_SETTLE);
      }
      break;

    case STATE_TURN_LEFT:
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
// 10. CONTROL PRINCIPAL
// ============================================================

void updateControl() {
  updateAllSensors();

  if (!running) {
    if (activeMode != MODE_TEST) {
      stopMotors();
    }
    return;
  }

  if (activeMode == MODE_WALL) {
    robotState = STATE_FOLLOW;
    followRightWall();
    return;
  }

  if (activeMode == MODE_MAZE) {
    runMaze();
    return;
  }

  // MODE_TEST no tiene control autonomo.
}

// ============================================================
// 11. WEB
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM | Test Suite</title>
<style>
:root{
  --bg:#0d1117;--card:#161b22;--border:#30363d;--text:#e6edf3;
  --muted:#8b949e;--good:#2ea043;--danger:#da3633;--button:#21262d;
  --accent:#7c5cff
}
*{box-sizing:border-box}
body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--bg);color:var(--text)}
.wrap{max-width:1180px;margin:auto;padding:16px}
h1{text-align:center;margin:0 0 4px}
.subtitle{text-align:center;color:var(--muted);margin-bottom:14px}
.tabs{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:14px}
.tabbtn{padding:12px;border:1px solid var(--border);background:var(--button);color:var(--text);border-radius:10px;font-weight:bold;cursor:pointer}
.tabbtn.active{background:var(--accent);border-color:var(--accent)}
.panel{display:none}
.panel.active{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px}
.metric-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.metric{border:1px solid var(--border);border-radius:10px;padding:10px;text-align:center;min-width:0}
.label{color:var(--muted);font-size:12px}
.value{font-size:22px;font-weight:bold;margin-top:4px;overflow-wrap:anywhere}
.field{display:grid;grid-template-columns:1fr 120px;gap:10px;align-items:center;margin:8px 0}
.field span{color:var(--muted)}
input{width:100%;padding:8px;border-radius:7px;border:1px solid var(--border);background:#0d1117;color:var(--text)}
button{border:1px solid var(--border);background:var(--button);color:var(--text);border-radius:9px;padding:11px;font-weight:bold;cursor:pointer}
.full{width:100%;margin-top:9px}
.start{background:var(--good);border-color:var(--good)}
.stop{background:var(--danger);border-color:var(--danger)}
.status{padding:10px;border:1px solid var(--border);border-radius:9px;text-align:center;font-weight:bold;margin-bottom:10px}
.motor-buttons{display:grid;grid-template-columns:1fr 1fr 1fr;gap:7px;margin-top:8px}
.sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.flags{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:10px}
.flag{padding:8px;border:1px solid var(--border);border-radius:8px;text-align:center;font-size:12px}
.hint{color:var(--muted);font-size:12px;line-height:1.4;margin-top:10px}
.globalbar{display:flex;gap:8px;flex-wrap:wrap;justify-content:center;margin-bottom:14px}
.badge{padding:7px 10px;border:1px solid var(--border);border-radius:999px;color:var(--muted);background:var(--card)}
@media(max-width:650px){
  .tabs{grid-template-columns:1fr}
  .motor-buttons{grid-template-columns:1fr}
  .field{grid-template-columns:1fr 100px}
}
</style>
</head>
<body>
<div class="wrap">
  <h1>AUS_KIM</h1>
  <div class="subtitle">Sistema unificado de pruebas</div>

  <div class="globalbar">
    <div class="badge">Modo: <b id="globalMode">TEST</b></div>
    <div class="badge">Estado: <b id="globalState">DETENIDO</b></div>
    <div class="badge">IP: <b>192.168.4.1</b></div>
  </div>

  <div class="tabs">
    <button class="tabbtn active" id="tabBtnTest" onclick="showTab('test')">Sensores / Motores / Encoders</button>
    <button class="tabbtn" id="tabBtnWall" onclick="showTab('wall')">PID pared derecha</button>
    <button class="tabbtn" id="tabBtnMaze" onclick="showTab('maze')">Resolver laberinto</button>
  </div>

  <!-- ====================================================== -->
  <!-- TAB 1: DIAGNOSTICO -->
  <!-- ====================================================== -->
  <section class="panel active" id="panelTest">
    <div class="grid">

      <div class="card">
        <h2>Sensores IR</h2>
        <div class="sensor-grid">
          <div class="metric"><div class="label">Frontal izquierdo</div><div class="value" id="tFL">0</div><div class="label">crudo <b id="tFLR">0</b></div></div>
          <div class="metric"><div class="label">Frontal derecho</div><div class="value" id="tFR">0</div><div class="label">crudo <b id="tFRR">0</b></div></div>
          <div class="metric"><div class="label">Lateral izquierdo</div><div class="value" id="tLL">0</div><div class="label">crudo <b id="tLLR">0</b></div></div>
          <div class="metric"><div class="label">Lateral derecho</div><div class="value" id="tLR">0</div><div class="label">crudo <b id="tLRR">0</b></div></div>
        </div>
      </div>

      <div class="card">
        <h2>Motor izquierdo</h2>
        <div class="field"><span>PWM manual</span><input id="manualPwmTest" type="number" min="0" max="255" step="1"></div>
        <div class="motor-buttons">
          <button id="leftReverse">ATRAS</button>
          <button onclick="stopManualMotor('L')">STOP</button>
          <button id="leftForward">ADELANTE</button>
        </div>
        <div class="hint">Los botones ADELANTE/ATRAS funcionan mientras los mantenes presionados.</div>
      </div>

      <div class="card">
        <h2>Motor derecho</h2>
        <div class="motor-buttons">
          <button id="rightReverse">ATRAS</button>
          <button onclick="stopManualMotor('R')">STOP</button>
          <button id="rightForward">ADELANTE</button>
        </div>
        <button class="full stop" onclick="stopAll()">STOP GENERAL</button>
      </div>

      <div class="card">
        <h2>Encoders</h2>
        <div class="metric-grid">
          <div class="metric"><div class="label">Izquierdo fisico</div><div class="value" id="tEncL">0</div><div class="label">A <b id="tEncLA">0</b> | B <b id="tEncLB">0</b></div></div>
          <div class="metric"><div class="label">Derecho fisico</div><div class="value" id="tEncR">0</div><div class="label">A <b id="tEncRA">0</b> | B <b id="tEncRB">0</b></div></div>
        </div>
        <button class="full" onclick="resetEncoders()">RESET ENCODERS</button>
      </div>

    </div>
  </section>

  <!-- ====================================================== -->
  <!-- TAB 2: WALL PID -->
  <!-- ====================================================== -->
  <section class="panel" id="panelWall">
    <div class="grid">

      <div class="card">
        <h2>Control PID pared derecha</h2>
        <div class="status" id="wallState">DETENIDO</div>
        <button class="full start" onclick="startAutonomous('WALL')">INICIAR PID</button>
        <button class="full stop" onclick="stopAll()">STOP</button>

        <div class="metric-grid" style="margin-top:10px">
          <div class="metric"><div class="label">Lateral derecho</div><div class="value" id="wLR">0</div></div>
          <div class="metric"><div class="label">Objetivo</div><div class="value" id="wTarget">2400</div></div>
          <div class="metric"><div class="label">Error</div><div class="value" id="wError">0</div></div>
          <div class="metric"><div class="label">Correccion</div><div class="value" id="wCorrection">0</div></div>
          <div class="metric"><div class="label">Motor izq.</div><div class="value" id="wMotorL">0</div></div>
          <div class="metric"><div class="label">Motor der.</div><div class="value" id="wMotorR">0</div></div>
        </div>
      </div>

      <div class="card">
        <h2>Parametros PID</h2>
        <div class="field"><span>Kp</span><input class="cfg" id="kp" type="number" step="0.01"></div>
        <div class="field"><span>Ki</span><input class="cfg" id="ki" type="number" step="0.001"></div>
        <div class="field"><span>Kd</span><input class="cfg" id="kd" type="number" step="0.01"></div>
        <div class="field"><span>Objetivo ADC derecha</span><input class="cfg" id="targetRightAdc" type="number" step="1"></div>
        <div class="field"><span>PWM base</span><input class="cfg" id="basePwm" type="number" min="0" max="255" step="1"></div>
        <div class="field"><span>Correccion maxima</span><input class="cfg" id="maxCorrection" type="number" min="0" max="255" step="1"></div>
        <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>
      </div>

      <div class="card">
        <h2>Sensores en vivo</h2>
        <div class="sensor-grid">
          <div class="metric"><div class="label">Frontal izquierdo</div><div class="value" id="wFL">0</div></div>
          <div class="metric"><div class="label">Frontal derecho</div><div class="value" id="wFR">0</div></div>
          <div class="metric"><div class="label">Lateral izquierdo</div><div class="value" id="wLL">0</div></div>
          <div class="metric"><div class="label">Lateral derecho</div><div class="value" id="wLR2">0</div></div>
        </div>
      </div>

    </div>
  </section>

  <!-- ====================================================== -->
  <!-- TAB 3: MAZE -->
  <!-- ====================================================== -->
  <section class="panel" id="panelMaze">
    <div class="grid">

      <div class="card">
        <h2>Maze Solver</h2>
        <div class="status" id="mazeState">DETENIDO</div>
        <button class="full start" onclick="startAutonomous('MAZE')">INICIAR LABERINTO</button>
        <button class="full stop" onclick="stopAll()">STOP</button>

        <div class="flags">
          <div class="flag">Frente<br><b id="mFrontFlag">LIBRE</b></div>
          <div class="flag">Derecha<br><b id="mRightFlag">PARED</b></div>
          <div class="flag">Izquierda<br><b id="mLeftFlag">PARED</b></div>
        </div>

        <div class="hint">
          Prioridad: derecha libre → derecha. Si frente bloqueado y derecha no disponible:
          izquierda si esta libre; si no, giro 180°.
        </div>
      </div>

      <div class="card">
        <h2>Deteccion</h2>
        <div class="field"><span>Pared frontal ADC</span><input class="cfg" id="frontWallAdc" type="number" step="1"></div>
        <div class="field"><span>Apertura derecha ADC</span><input class="cfg" id="rightOpenAdc" type="number" step="1"></div>
        <div class="field"><span>Apertura izquierda ADC</span><input class="cfg" id="leftOpenAdc" type="number" step="1"></div>
      </div>

      <div class="card">
        <h2>Giros</h2>
        <div class="field"><span>PWM giro</span><input class="cfg" id="turnPwm" type="number" min="0" max="255" step="1"></div>
        <div class="field"><span>Avance antes derecha ms</span><input class="cfg" id="rightAdvanceMs" type="number" min="0" step="10"></div>
        <div class="field"><span>Giro derecha ms</span><input class="cfg" id="rightTurnMs" type="number" min="0" step="10"></div>
        <div class="field"><span>Giro izquierda ms</span><input class="cfg" id="leftTurnMs" type="number" min="0" step="10"></div>
        <div class="field"><span>Giro 180 ms</span><input class="cfg" id="uTurnMs" type="number" min="0" step="10"></div>
        <div class="field"><span>Estabilizacion ms</span><input class="cfg" id="settleMs" type="number" min="0" step="10"></div>
        <div class="field"><span>Cooldown cruce ms</span><input class="cfg" id="junctionCooldownMs" type="number" min="0" step="10"></div>
        <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>
      </div>

      <div class="card">
        <h2>Telemetria Maze</h2>
        <div class="sensor-grid">
          <div class="metric"><div class="label">Frontal izquierdo</div><div class="value" id="mFL">0</div></div>
          <div class="metric"><div class="label">Frontal derecho</div><div class="value" id="mFR">0</div></div>
          <div class="metric"><div class="label">Lateral izquierdo</div><div class="value" id="mLL">0</div></div>
          <div class="metric"><div class="label">Lateral derecho</div><div class="value" id="mLR">0</div></div>
          <div class="metric"><div class="label">Motor izquierdo</div><div class="value" id="mMotorL">0</div></div>
          <div class="metric"><div class="label">Motor derecho</div><div class="value" id="mMotorR">0</div></div>
          <div class="metric"><div class="label">Error PID</div><div class="value" id="mError">0</div></div>
          <div class="metric"><div class="label">Correccion PID</div><div class="value" id="mCorrection">0</div></div>
        </div>
      </div>

    </div>
  </section>
</div>

<script>
let activeTab='test';
let firstLoad=true;
let activeManualMotor=null;

function panelName(tab){
  return tab.charAt(0).toUpperCase()+tab.slice(1);
}

async function showTab(tab){
  activeTab=tab;

  ['test','wall','maze'].forEach(t=>{
    document.getElementById('panel'+panelName(t)).classList.toggle('active',t===tab);
    document.getElementById('tabBtn'+panelName(t)).classList.toggle('active',t===tab);
  });

  const mode = tab==='test' ? 'TEST' : (tab==='wall' ? 'WALL' : 'MAZE');
  await fetch('/api/mode?mode='+mode,{cache:'no-store'});
  await updateStatus();
}

async function startAutonomous(mode){
  await fetch('/api/mode?mode='+mode,{cache:'no-store'});
  await fetch('/api/run?state=1',{cache:'no-store'});
  await updateStatus();
}

async function stopAll(){
  activeManualMotor=null;
  await fetch('/api/stop',{cache:'no-store'});
  await updateStatus();
}

function manualPwm(){
  const v=parseInt(document.getElementById('manualPwmTest').value||'0',10);
  return Math.max(0,Math.min(255,v));
}

async function sendManualMotor(motor,dir){
  activeManualMotor=motor;
  await fetch('/api/motor?motor='+motor+'&dir='+dir+'&pwm='+manualPwm(),{cache:'no-store'});
}

async function stopManualMotor(motor){
  if(activeManualMotor===motor)activeManualMotor=null;
  await fetch('/api/motor?motor='+motor+'&dir=S&pwm=0',{cache:'no-store'});
}

function bindHold(id,motor,dir){
  const el=document.getElementById(id);

  const start=async(e)=>{
    e.preventDefault();
    await sendManualMotor(motor,dir);
  };

  const stop=async(e)=>{
    e.preventDefault();
    await stopManualMotor(motor);
  };

  el.addEventListener('pointerdown',start);
  el.addEventListener('pointerup',stop);
  el.addEventListener('pointercancel',stop);
  el.addEventListener('pointerleave',e=>{if(e.buttons!==0)stop(e)});
}

bindHold('leftForward','L','F');
bindHold('leftReverse','L','R');
bindHold('rightForward','R','F');
bindHold('rightReverse','R','R');

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

  ids.forEach(id=>{
    const el=document.getElementById(id);
    if(el)p.set(id,el.value);
  });

  p.set('manualPwm',manualPwm());

  await fetch('/api/config?'+p.toString(),{cache:'no-store'});
  await updateStatus();
}

function setSensor(prefix,d){
  document.getElementById(prefix+'FL').textContent=d.sensor.fl;
  document.getElementById(prefix+'FR').textContent=d.sensor.fr;
  document.getElementById(prefix+'LL').textContent=d.sensor.ll;
  document.getElementById(prefix+'LR').textContent=d.sensor.lr;
}

async function updateStatus(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    const d=await r.json();

    document.getElementById('globalMode').textContent=d.mode;
    document.getElementById('globalState').textContent=d.state;

    // TEST
    setSensor('t',d);
    document.getElementById('tFLR').textContent=d.sensor.flRaw;
    document.getElementById('tFRR').textContent=d.sensor.frRaw;
    document.getElementById('tLLR').textContent=d.sensor.llRaw;
    document.getElementById('tLRR').textContent=d.sensor.lrRaw;

    document.getElementById('tEncL').textContent=d.enc.left;
    document.getElementById('tEncR').textContent=d.enc.right;
    document.getElementById('tEncLA').textContent=d.enc.la;
    document.getElementById('tEncLB').textContent=d.enc.lb;
    document.getElementById('tEncRA').textContent=d.enc.ra;
    document.getElementById('tEncRB').textContent=d.enc.rb;

    // WALL
    document.getElementById('wallState').textContent=d.mode==='WALL' && d.running ? 'SIGUIENDO PARED' : 'DETENIDO';
    document.getElementById('wLR').textContent=d.sensor.lr;
    document.getElementById('wTarget').textContent=d.config.targetRightAdc;
    document.getElementById('wError').textContent=d.pid.error.toFixed(1);
    document.getElementById('wCorrection').textContent=d.pid.correction.toFixed(1);
    document.getElementById('wMotorL').textContent=d.motor.left;
    document.getElementById('wMotorR').textContent=d.motor.right;
    setSensor('w',d);
    document.getElementById('wLR2').textContent=d.sensor.lr;

    // MAZE
    document.getElementById('mazeState').textContent=d.mode==='MAZE' ? d.state : 'DETENIDO';
    document.getElementById('mFrontFlag').textContent=d.flags.frontBlocked?'PARED':'LIBRE';
    document.getElementById('mRightFlag').textContent=d.flags.rightOpen?'LIBRE':'PARED';
    document.getElementById('mLeftFlag').textContent=d.flags.leftOpen?'LIBRE':'PARED';
    setSensor('m',d);
    document.getElementById('mMotorL').textContent=d.motor.left;
    document.getElementById('mMotorR').textContent=d.motor.right;
    document.getElementById('mError').textContent=d.pid.error.toFixed(1);
    document.getElementById('mCorrection').textContent=d.pid.correction.toFixed(1);

    if(firstLoad){
      Object.keys(d.config).forEach(k=>{
        const el=document.getElementById(k);
        if(el)el.value=d.config[k];
      });

      document.getElementById('manualPwmTest').value=d.config.manualPwm;
      firstLoad=false;
    }
  }catch(_){}
}

// Heartbeat siempre activo mientras la pagina esta abierta.
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
// 12. API
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
  json.reserve(1500);

  json += "{";
  json += "\"running\":" + String(running ? "true" : "false") + ",";
  json += "\"mode\":\"" + String(modeName(activeMode)) + "\",";
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
  json += "\"junctionCooldownMs\":" + String(cfg.junctionCooldownMs) + ",";
  json += "\"manualPwm\":" + String(cfg.manualPwm);
  json += "}";

  json += "}";

  server.send(200, "application/json", json);
}

void handleMode() {
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Parametro requerido: mode");
    return;
  }

  String mode = server.arg("mode");

  // Siempre se detiene al cambiar de modo.
  running = false;
  robotState = STATE_STOPPED;
  stopMotors();
  resetPid();

  if (mode == "TEST") {
    activeMode = MODE_TEST;
  } else if (mode == "WALL") {
    activeMode = MODE_WALL;
  } else if (mode == "MAZE") {
    activeMode = MODE_MAZE;
  } else {
    server.send(400, "text/plain", "mode debe ser TEST, WALL o MAZE");
    return;
  }

  lastHeartbeatMs = millis();
  server.send(200, "text/plain", modeName(activeMode));
}

void handleRun() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Parametro requerido: state");
    return;
  }

  if (activeMode == MODE_TEST) {
    server.send(400, "text/plain", "Modo TEST no usa run autonomo");
    return;
  }

  bool state = server.arg("state") == "1";

  if (state) {
    resetPid();
    lastHeartbeatMs = millis();
    lastDecisionMs = millis();
    running = true;

    if (activeMode == MODE_MAZE) {
      enterState(STATE_FOLLOW);
    } else {
      robotState = STATE_FOLLOW;
    }
  } else {
    running = false;
    robotState = STATE_STOPPED;
    stopMotors();
    resetPid();
  }

  server.send(200, "text/plain", running ? "RUN" : "STOP");
}

void handleMotor() {
  if (activeMode != MODE_TEST) {
    server.send(403, "text/plain", "Control manual solo disponible en modo TEST");
    return;
  }

  if (!server.hasArg("motor") || !server.hasArg("dir") || !server.hasArg("pwm")) {
    server.send(400, "text/plain", "Parametros requeridos: motor, dir, pwm");
    return;
  }

  String motor = server.arg("motor");
  String dir = server.arg("dir");
  int pwm = constrain(server.arg("pwm").toInt(), 0, 255);

  int command = 0;
  if (dir == "F") command = pwm;
  else if (dir == "R") command = -pwm;
  else if (dir == "S") command = 0;
  else {
    server.send(400, "text/plain", "dir debe ser F, R o S");
    return;
  }

  if (motor == "L") {
    setMotorSigned(true, command);
  } else if (motor == "R") {
    setMotorSigned(false, command);
  } else {
    server.send(400, "text/plain", "motor debe ser L o R");
    return;
  }

  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "OK");
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

  if (server.hasArg("manualPwm"))
    cfg.manualPwm = constrain(server.arg("manualPwm").toInt(), 0, 255);

  resetPid();
  server.send(200, "text/plain", "OK");
}

void handleResetEncoders() {
  noInterrupts();
  encoderLeft = 0;
  encoderRight = 0;
  interrupts();

  server.send(200, "text/plain", "OK");
}

void handlePing() {
  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  running = false;
  robotState = STATE_STOPPED;
  stopMotors();
  resetPid();
  lastHeartbeatMs = millis();
  server.send(200, "text/plain", "STOP");
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - No encontrado");
}

// ============================================================
// 13. SETUP
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
  server.on("/api/mode", HTTP_GET, handleMode);
  server.on("/api/run", HTTP_GET, handleRun);
  server.on("/api/motor", HTTP_GET, handleMotor);
  server.on("/api/config", HTTP_GET, handleConfig);
  server.on("/api/reset_encoders", HTTP_GET, handleResetEncoders);
  server.on("/api/ping", HTTP_GET, handlePing);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.onNotFound(handleNotFound);

  server.begin();

  activeMode = MODE_TEST;
  robotState = STATE_STOPPED;
  lastHeartbeatMs = millis();
  lastControlMs = millis();

  Serial.println();
  Serial.println("AUS_KIM - TEST SUITE");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

// ============================================================
// 14. LOOP
// ============================================================

void loop() {
  server.handleClient();

  uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    lastControlMs = now;
    updateControl();
  }

  bool motorsRunning = (motorLeftCmd != 0) || (motorRightCmd != 0);

  if (motorsRunning && (now - lastHeartbeatMs > WEB_FAILSAFE_MS)) {
    running = false;
    robotState = STATE_STOPPED;
    stopMotors();
    resetPid();
    Serial.println("FAIL-SAFE: motores detenidos por perdida de comunicacion.");
  }

  delay(1);
}

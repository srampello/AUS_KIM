// ARCHIVO GENERADO AUTOMATICAMENTE - NO EDITAR A MANO.
// Fuente: firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino
// Regenerar: python tools/generar_aus_kim_race.py
// La logica de manejo es IDENTICA al Test Suite. Solo cambia la portada.

/*
 * AUS_KIM - Sistema unificado de pruebas
 *
 * Web separada de la logica:
 *  - web_test_suite.h: diagnostico, PID y laberinto por derecha/izquierda.
 *  - web_race.h: largada RMP con ambas estrategias.
 *  - rmp_logo.h: logotipo RMP.
 * El .ino se dedica a sensores, encoders, motores, PID y navegacion.
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
#include "rmp_logo.h"
#include "web_test_suite.h"
#include "web_race.h"

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
  MODE_MAZE,
  MODE_ENCODER,
  MODE_WALL_LEFT,
  MODE_MAZE_LEFT
};

enum TimedTestAction : uint8_t {
  TIMED_NONE = 0,
  TIMED_FORWARD,
  TIMED_LEFT_90,
  TIMED_RIGHT_90
};

enum EncoderTestAction : uint8_t {
  ENC_TEST_NONE = 0,
  ENC_TEST_RIGHT_45,
  ENC_TEST_LEFT_45,
  ENC_TEST_RIGHT_90,
  ENC_TEST_LEFT_90,
  ENC_TEST_RIGHT_180,
  ENC_TEST_FORWARD
};

enum RobotState : uint8_t {
  STATE_STOPPED = 0,
  STATE_FOLLOW,
  STATE_SIDE_OPEN_WAIT,
  STATE_SIDE_OPEN_ADVANCE,
  STATE_SIDE_OPEN_WAIT_TURN,
  STATE_FRONT_WAIT,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_POST_TURN_WAIT
};

struct SensorData {
  uint16_t raw = 0;
  uint16_t filtered = 0;
  bool initialized = false;
};

struct ControlConfig {
  // PID pared derecha - calibracion AUS_KIM 30/09/2026
  float kp = 0.065f;
  float ki = 0.0f;
  float kd = 1.00f;

  int targetRightAdc = 2300;
  float leftKp = 0.065f, leftKi = 0.0f, leftKd = 1.00f;
  int targetLeftAdc = 2300;
  int basePwm = 180;
  int maxCorrection = 60;

  // STOP frontal.
  // Sharp: pared cercana = ADC mayor.
  int frontWallAdc = 1900;
  int frontConfirmAdc = 1900;

  // Apertura lateral confirmada cuando el sensor baja de 1600.
  int rightOpenAdc = 1600;
  int leftOpenAdc = 1600;

  // Seguimiento/giro.
  int approachMinPwm = 140; // legacy
  int turnPwm = 180;

  // Ticks de encoders reservados SOLO para pruebas manuales.
  int turn45RightTicks = 80;
  int turn45LeftTicks = 80;
  int turn90RightTicks = 120;
  int turn90LeftTicks = 125;
  int turn180Ticks = 250;

  // Apertura: frenar 500 ms -> avanzar por tiempo (~3 cm iniciales)
  // -> frenar 300 ms -> girar por tiempo -> frenar 300 ms.
  int openingWaitMs = 500;
  float openingAdvanceCm = 3.0f;  // referencia historica, NO mide recorrido
  int openingAdvanceMs = 100;     // calibrar tiempo en pista
  int openingLeftPwm = 180;       // ajuste independiente para motores distintos
  int openingRightPwm = 180;
  int turn90LeftMs = 160;        // punto de partida a calibrar
  int turn90RightMs = 160;
  // Avance recto independiente del PID de pared derecha.
  float straightLeftTicksPerCm = 20.95f;
  float straightRightTicksPerCm = 20.95f;
  float straightKp = 40.0f;
  int straightMaxCorrection = 25;

  // Espera cuando encuentra una pared frontal, antes del giro izquierdo.
  int decisionWaitMs = 300;

  // Prueba manual
  int manualPwm = 80;
};

ControlConfig cfg;

// ============================================================
// CALIBRACION DE MOVIMIENTO / ENCODERS
// ============================================================
// Recta medida: 20 cm = promedio 419 ticks -> 20.95 ticks/cm.
// Giros medidos repetidamente por suma de recorridos absolutos:
// 90 grados ~= 251 ticks totales; 180 grados ~= 501 ticks totales.
// Los valores efectivos se guardan en cfg para poder ajustarlos desde la web.
const float TICKS_PER_CM = 20.95f;

// Umbral fisico medido: por debajo de ~155 PWM AUS_KIM no vence
// el rozamiento y los motores pueden quedar detenidos.
// Todo movimiento autonomo usa 155 como piso real.
const int MIN_MOVING_PWM = 155;

// Pausa mecanica tras frenar frente a una pared.
const uint16_t BRAKE_SETTLE_MS = 100;

// Avance normal por PID.
const int MAZE_FORWARD_MIN_PWM = 120;
const int MAZE_FORWARD_MAX_PWM = 255;

// Maze: los tiempos de maniobra estan acotados desde ControlConfig.

const uint8_t EVENT_CONFIRM_SAMPLES = 3;

// ============================================================
// 4. ESTADO GLOBAL
// ============================================================

RunMode activeMode = MODE_TEST;
bool running = false;

RobotState robotState = STATE_STOPPED;
RobotState pendingTurnState = STATE_STOPPED;
uint32_t stateStartMs = 0;
uint32_t lastDecisionMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastHeartbeatMs = 0;

SensorData sFL, sFR, sLL, sLR;

bool frontBlocked = false;
String stopReason = "NINGUNA";  // Ultimo motivo de detencion
bool rightOpen = false;
bool leftOpen = false;

// Pared frontal confirmada durante varias muestras.
uint8_t frontWallStableCount = 0;

// Apertura derecha: primero tiene que haber una pared real.
uint8_t rightOpenStableCount = 0;
uint8_t rightWallStableCount = 0;
bool rightOpeningArmed = false;
uint8_t leftOpenStableCount = 0, leftWallStableCount = 0;
bool leftOpeningArmed = false;

// Cantidad de giros izquierdos recientes (saturado a 2 para diagnostico).
// Ya NO se usa como limite que detenga la navegacion.
uint8_t frontLeftTurns = 0;

float errorPid = 0.0f;
float prevErrorPid = 0.0f;
float integralPid = 0.0f;
float derivativePid = 0.0f;
float correctionPid = 0.0f;
float straightErrorCm = 0.0f;
float straightCorrectionPwm = 0.0f;

int motorLeftCmd = 0;   // signed
int motorRightCmd = 0;  // signed

// Posicion fisica de encoders al iniciar una maniobra de giro.
int32_t moveStartLeft = 0;
int32_t moveStartRight = 0;

// Prueba/calibracion independiente de encoders.
EncoderTestAction encoderTestAction = ENC_TEST_NONE;
bool encoderTestActive = false;
bool encoderTestCompleted = false;
uint32_t encoderTestTarget = 0;
uint32_t encoderTestStartMs = 0;
int encoderTestPwm = 155;
float encoderTestTicksPerCm = TICKS_PER_CM;
float encoderTestRequestedCm = 0.0f;

// Pruebas de calibracion temporal SIN encoders.
TimedTestAction timedTestAction = TIMED_NONE;
uint32_t timedTestStartMs = 0;

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

// Devuelve los encoders corregidos para las ruedas fisicas.
// La correccion coincide con la telemetria ya verificada en AUS_KIM.
void readPhysicalEncoders(int32_t &left, int32_t &right) {
  noInterrupts();
  left = encoderRight;
  right = -encoderLeft;
  interrupts();
}

void captureMoveStart() {
  readPhysicalEncoders(moveStartLeft, moveStartRight);
  straightErrorCm = 0.0f;
  straightCorrectionPwm = 0.0f;
}

void getMoveDeltas(uint32_t &deltaLeft, uint32_t &deltaRight) {
  int32_t left;
  int32_t right;
  readPhysicalEncoders(left, right);

  deltaLeft = (uint32_t)abs(left - moveStartLeft);
  deltaRight = (uint32_t)abs(right - moveStartRight);
}

uint32_t getMoveTicksSum() {
  uint32_t deltaLeft;
  uint32_t deltaRight;
  getMoveDeltas(deltaLeft, deltaRight);
  return deltaLeft + deltaRight;
}

uint32_t getMoveTicksAverage() {
  uint32_t deltaLeft;
  uint32_t deltaRight;
  getMoveDeltas(deltaLeft, deltaRight);
  return (deltaLeft + deltaRight) / 2UL;
}

void getStraightWheelCm(float &leftCm, float &rightCm) {
  uint32_t deltaLeft, deltaRight;
  getMoveDeltas(deltaLeft, deltaRight);
  leftCm = (float)deltaLeft / cfg.straightLeftTicksPerCm;
  rightCm = (float)deltaRight / cfg.straightRightTicksPerCm;
}

float getMoveStraightCm() {
  float leftCm, rightCm;
  getStraightWheelCm(leftCm, rightCm);
  return (leftCm + rightCm) * 0.5f;
}

// Si una rueda avanza mas que la otra, corregimos los PWM individuales.
void driveStraightEncoders(int basePwm) {
  float leftCm, rightCm;
  getStraightWheelCm(leftCm, rightCm);
  straightErrorCm = leftCm - rightCm;
  straightCorrectionPwm = constrain(
    straightErrorCm * cfg.straightKp,
    -(float)cfg.straightMaxCorrection,
    (float)cfg.straightMaxCorrection
  );
  int correction = (int)roundf(straightCorrectionPwm);
  int leftPwm = constrain(basePwm - correction, MIN_MOVING_PWM, 255);
  int rightPwm = constrain(basePwm + correction, MIN_MOVING_PWM, 255);
  setDrive(leftPwm, rightPwm);
}

const char* encoderTestActionName(EncoderTestAction action) {
  switch (action) {
    case ENC_TEST_RIGHT_45:  return "DERECHA 45";
    case ENC_TEST_LEFT_45:   return "IZQUIERDA 45";
    case ENC_TEST_RIGHT_90:  return "DERECHA 90";
    case ENC_TEST_LEFT_90:   return "IZQUIERDA 90";
    case ENC_TEST_RIGHT_180: return "DERECHA 180";
    case ENC_TEST_FORWARD:   return "AVANCE";
    case ENC_TEST_NONE:
    default:                 return "NINGUNA";
  }
}

void runEncoderTest() {
  if (!encoderTestActive) {
    stopMotors();
    return;
  }

  // Corte de seguridad adicional. El heartbeat web sigue siendo prioritario.
  if (millis() - encoderTestStartMs > 20000UL) {
    encoderTestActive = false;
    encoderTestCompleted = false;
    running = false;
    stopMotors();
    return;
  }

  uint32_t progress = 0;

  if (encoderTestAction == ENC_TEST_FORWARD) {
    // Distancias independientes por rueda.
    progress = (uint32_t)roundf(getMoveStraightCm() * encoderTestTicksPerCm);
  } else {
    // Para giros se usa la suma absoluta, igual que en el Maze Solver.
    progress = getMoveTicksSum();
  }

  if (progress >= encoderTestTarget) {
    encoderTestActive = false;
    encoderTestCompleted = true;
    running = false;
    stopMotors();
    return;
  }

  int pwm = constrain(encoderTestPwm, MIN_MOVING_PWM, 255);

  switch (encoderTestAction) {
    case ENC_TEST_RIGHT_45:
    case ENC_TEST_RIGHT_90:
    case ENC_TEST_RIGHT_180:
      setDrive(+pwm, -pwm);
      break;

    case ENC_TEST_LEFT_45:
    case ENC_TEST_LEFT_90:
      setDrive(-pwm, +pwm);
      break;

    case ENC_TEST_FORWARD:
      driveStraightEncoders(pwm);
      break;

    case ENC_TEST_NONE:
    default:
      encoderTestActive = false;
      running = false;
      stopMotors();
      break;
  }
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

  // Sharp GP2Y0E03: en nuestra calibracion una pared cercana da ADC MAYOR.
  // Exigimos ambos frontales para evitar el falso cercano de un solo sensor.
  bool frontCandidate =
    (sFL.filtered >= cfg.frontWallAdc) &&
    (sFR.filtered >= cfg.frontConfirmAdc);

  if (frontCandidate) {
    if (frontWallStableCount < 10) frontWallStableCount++;
  } else {
    frontWallStableCount = 0;
  }

  frontBlocked = frontWallStableCount >= EVENT_CONFIRM_SAMPLES;

  rightOpen = sLR.filtered < cfg.rightOpenAdc;
  leftOpen  = sLL.filtered < cfg.leftOpenAdc;

  if (rightOpen) {
    if (rightOpenStableCount < 10) rightOpenStableCount++;
  } else {
    rightOpenStableCount = 0;
  }

  // Rehabilitar un giro a derecha SOLO cuando se haya visto pared derecha
  // de forma estable. Histeresis respecto del umbral de apertura.
  int wallThreshold = min(4095, cfg.rightOpenAdc + 200);
  if (sLR.filtered >= wallThreshold) {
    if (rightWallStableCount < 10) rightWallStableCount++;
  } else {
    rightWallStableCount = 0;
  }
  // Detector independiente: pared izquierda observada -> apertura izquierda.
  if (leftOpen) {
    if (leftOpenStableCount < 10) leftOpenStableCount++;
  } else leftOpenStableCount = 0;
  if (sLL.filtered >= min(4095, cfg.leftOpenAdc + 200)) {
    if (leftWallStableCount < 10) leftWallStableCount++;
  } else leftWallStableCount = 0;
}

// Valor frontal conservado para telemetria/aproximacion.
uint16_t getFrontAdc() {
  return max(sFL.filtered, sFR.filtered);
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

void driveRightWallPid(
  int basePwm,
  int targetAdc,
  int maxCorrection,
  int minWheelPwm,
  int maxWheelPwm
) {
  errorPid = (float)sLR.filtered - (float)targetAdc;

  integralPid += errorPid;
  integralPid = constrain(integralPid, -4000.0f, 4000.0f);

  derivativePid = errorPid - prevErrorPid;

  correctionPid =
      cfg.kp * errorPid +
      cfg.ki * integralPid +
      cfg.kd * derivativePid;

  correctionPid = constrain(
    correctionPid,
    -(float)maxCorrection,
    (float)maxCorrection
  );

  maxWheelPwm = constrain(maxWheelPwm, 0, 255);
  minWheelPwm = constrain(minWheelPwm, 0, maxWheelPwm);
  basePwm = constrain(basePwm, minWheelPwm, maxWheelPwm);

  int leftPwm  = basePwm - (int)correctionPid;
  int rightPwm = basePwm + (int)correctionPid;

  // Cada rueda puede usar todo el rango permitido hasta PWM 255.
  leftPwm  = constrain(leftPwm, minWheelPwm, maxWheelPwm);
  rightPwm = constrain(rightPwm, minWheelPwm, maxWheelPwm);

  setDrive(leftPwm, rightPwm);
  prevErrorPid = errorPid;
}

void followRightWallAtPwm(int basePwm) {
  // Seguimiento normal: base configurable y correccion PID.
  // Cada rueda puede variar entre MAZE_FORWARD_MIN_PWM y 255.
  basePwm = constrain(basePwm, MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);

  driveRightWallPid(
    basePwm,
    cfg.targetRightAdc,
    cfg.maxCorrection,
    MAZE_FORWARD_MIN_PWM,
    MAZE_FORWARD_MAX_PWM
  );
}

void followRightWall() {
  followRightWallAtPwm(cfg.basePwm);
}

// PID inverso: cerca de pared izquierda aumenta PWM izquierdo.
void followLeftWallAtPwm(int basePwm) {
  errorPid = (float)sLL.filtered - (float)cfg.targetLeftAdc;
  integralPid = constrain(integralPid + errorPid, -4000.0f, 4000.0f);
  derivativePid = errorPid - prevErrorPid;
  correctionPid = cfg.leftKp*errorPid + cfg.leftKi*integralPid + cfg.leftKd*derivativePid;
  correctionPid = constrain(correctionPid, -(float)cfg.maxCorrection, (float)cfg.maxCorrection);
  basePwm = constrain(basePwm, MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);
  int leftPwm  = constrain(basePwm + (int)correctionPid, MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);
  int rightPwm = constrain(basePwm - (int)correctionPid, MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);
  setDrive(leftPwm, rightPwm);
  prevErrorPid = errorPid;
}
void followLeftWall() { followLeftWallAtPwm(cfg.basePwm); }

int calculateApproachPwm() {
  if (frontBlocked) return 0;
  return cfg.basePwm;
}

// ============================================================
// 9. MAZE
// ============================================================

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_FOLLOW: return (activeMode == MODE_WALL_LEFT || activeMode == MODE_MAZE_LEFT) ? "PID PARED IZQUIERDA" : "PID PARED DERECHA";
    case STATE_SIDE_OPEN_WAIT: return activeMode == MODE_MAZE_LEFT ? "IZQUIERDA / FRENO 500 MS" : "DERECHA / FRENO 500 MS";
    case STATE_SIDE_OPEN_ADVANCE: return activeMode == MODE_MAZE_LEFT ? "IZQUIERDA / AVANCE 3 CM" : "DERECHA / AVANCE 3 CM";
    case STATE_SIDE_OPEN_WAIT_TURN: return activeMode == MODE_MAZE_LEFT ? "IZQUIERDA / ESPERA GIRO" : "DERECHA / ESPERA GIRO";
    case STATE_FRONT_WAIT:           return "PARED FRONTAL / FRENO";
    case STATE_TURN_LEFT:            return "GIRO 90 IZQUIERDA";
    case STATE_TURN_RIGHT:           return "GIRO 90 DERECHA";
    case STATE_POST_TURN_WAIT:       return "PAUSA POST GIRO";
    case STATE_STOPPED:
    default:                         return "DETENIDO";
  }
}

const char* modeName(RunMode m) {
  switch (m) {
    case MODE_WALL:      return "WALL";
    case MODE_WALL_LEFT: return "WALL_LEFT";
    case MODE_MAZE:      return "MAZE";
    case MODE_MAZE_LEFT: return "MAZE_LEFT";
    case MODE_ENCODER: return "ENCODER";
    case MODE_TEST:
    default:           return "TEST";
  }
}

void enterState(RobotState newState) {
  robotState = newState;
  stateStartMs = millis();
  resetPid();
}

void runMaze() {
  uint32_t now = millis();
  bool leftHand = activeMode == MODE_MAZE_LEFT;

  switch (robotState) {
    case STATE_STOPPED:
      enterState(STATE_FOLLOW);
      break;

    case STATE_FOLLOW: {
      // La pared frontal siempre tiene prioridad.
      if (frontBlocked) {
        stopMotors();
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }

      // Habilitar apertura solo despues de detectar pared en ese lado.
      if (leftHand) {
        if (leftWallStableCount >= EVENT_CONFIRM_SAMPLES) leftOpeningArmed = true;
      } else {
        if (rightWallStableCount >= EVENT_CONFIRM_SAMPLES) rightOpeningArmed = true;
      }
      bool armed = leftHand ? leftOpeningArmed : rightOpeningArmed;
      uint8_t stableOpening = leftHand ? leftOpenStableCount : rightOpenStableCount;
      if (armed && stableOpening >= EVENT_CONFIRM_SAMPLES) {
        if (leftHand) leftOpeningArmed = false;
        else rightOpeningArmed = false;
        stopMotors();
        enterState(STATE_SIDE_OPEN_WAIT);
        break;
      }
      bool sideOpen = leftHand ? leftOpen : rightOpen;
      if (!sideOpen) {
        if (leftHand) followLeftWallAtPwm(cfg.basePwm);
        else followRightWallAtPwm(cfg.basePwm);
      } else {
        resetPid();
        setDrive(cfg.openingLeftPwm, cfg.openingRightPwm);
      }
      break;
    }

    case STATE_SIDE_OPEN_WAIT:
      stopMotors();
      if (frontBlocked) {
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }
      if (now - stateStartMs >= (uint32_t)cfg.openingWaitMs) {

        enterState(STATE_SIDE_OPEN_ADVANCE);
      }
      break;

    case STATE_SIDE_OPEN_ADVANCE:
      // Avance corto POR TIEMPO, sin depender de los encoders.
      // El frenado frontal mantiene prioridad durante este avance.
      if (frontBlocked) {
        stopMotors();
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }
      if (now - stateStartMs >= (uint32_t)cfg.openingAdvanceMs) {
        stopMotors();
        enterState(STATE_SIDE_OPEN_WAIT_TURN);
        break;
      }
      setDrive(cfg.openingLeftPwm, cfg.openingRightPwm);
      break;

    case STATE_SIDE_OPEN_WAIT_TURN:
      stopMotors();
      if (now - stateStartMs >= 300UL) {

        enterState(leftHand ? STATE_TURN_LEFT : STATE_TURN_RIGHT);
      }
      break;

    case STATE_FRONT_WAIT:
      stopMotors();
      // Cuando el frente esta bloqueado, girar al lado opuesto del seguimiento.
      if (now - stateStartMs >= (uint32_t)cfg.decisionWaitMs) {
        if (frontLeftTurns < 2) frontLeftTurns++;

        enterState(leftHand ? STATE_TURN_RIGHT : STATE_TURN_LEFT);
      }
      break;

    case STATE_TURN_LEFT:
      // Giro por tiempo, calibrable, no requiere encoder.
      if (now - stateStartMs >= (uint32_t)cfg.turn90LeftMs) {
        stopMotors();
        enterState(STATE_POST_TURN_WAIT);
        break;
      }
      setDrive(-cfg.turnPwm, +cfg.turnPwm);
      break;

    case STATE_TURN_RIGHT:
      if (now - stateStartMs >= (uint32_t)cfg.turn90RightMs) {
        stopMotors();
        enterState(STATE_POST_TURN_WAIT);
        break;
      }
      setDrive(+cfg.turnPwm, -cfg.turnPwm);
      break;

    case STATE_POST_TURN_WAIT:
      stopMotors();
      if (now - stateStartMs < 300UL) {
        break;
      }

      // Si sigue detectando pared frontal, no cancelar la carrera.
      // Repetir evaluacion tras la pausa configurada y girar otra vez.
      // La pausa y el limite temporal de cada giro impiden giros sin fin.
      if (frontBlocked) {
        Serial.println(leftHand ? "AUS_KIM: frente bloqueado; buscando salida a derecha." : "AUS_KIM: frente bloqueado despues del giro; buscando salida a izquierda.");
        enterState(STATE_FRONT_WAIT);
        break;
      }

      frontLeftTurns = 0;
      rightOpenStableCount = 0;
      rightWallStableCount = 0;
      rightOpeningArmed = false;
      leftOpenStableCount = 0;
      leftWallStableCount = 0;
      leftOpeningArmed = false;
      resetPid();
      enterState(STATE_FOLLOW);
      break;
  }
}

// ============================================================
// PRUEBA DE MANIOBRAS POR TIEMPO (sin depender de encoders)
// ============================================================

const char* timedTestName(TimedTestAction action) {
  switch (action) {
    case TIMED_FORWARD:  return "AVANCE CORTO";
    case TIMED_LEFT_90:  return "GIRO IZQUIERDA";
    case TIMED_RIGHT_90: return "GIRO DERECHA";
    default:             return "NINGUNA";
  }
}

void runTimedTest() {
  if (timedTestAction == TIMED_NONE) return;
  if (!running) {
    timedTestAction = TIMED_NONE;
    stopMotors();
    return;
  }

  uint32_t elapsed = millis() - timedTestStartMs;
  uint32_t duration = 0;
  if (timedTestAction == TIMED_FORWARD) duration = (uint32_t)cfg.openingAdvanceMs;
  if (timedTestAction == TIMED_LEFT_90) duration = (uint32_t)cfg.turn90LeftMs;
  if (timedTestAction == TIMED_RIGHT_90) duration = (uint32_t)cfg.turn90RightMs;

  // Frente tiene prioridad durante la prueba de avance.
  if (elapsed >= duration || (timedTestAction == TIMED_FORWARD && frontBlocked)) {
    stopMotors();
    timedTestAction = TIMED_NONE;
    running = false;
    return;
  }
  if (timedTestAction == TIMED_FORWARD) setDrive(cfg.openingLeftPwm, cfg.openingRightPwm);
  else if (timedTestAction == TIMED_LEFT_90) setDrive(-cfg.turnPwm, +cfg.turnPwm);
  else if (timedTestAction == TIMED_RIGHT_90) setDrive(+cfg.turnPwm, -cfg.turnPwm);
}

// ============================================================
// 10. CONTROL PRINCIPAL
// ============================================================

void updateControl() {
  updateAllSensors();

  if (timedTestAction != TIMED_NONE) {
    runTimedTest();
    return;
  }

  if (!running) {
    if (activeMode != MODE_TEST) {
      stopMotors();
    }
    return;
  }

  if (activeMode == MODE_WALL || activeMode == MODE_WALL_LEFT) {
    robotState = STATE_FOLLOW;
    if (activeMode == MODE_WALL_LEFT) followLeftWall();
    else followRightWall();
    return;
  }

  if (activeMode == MODE_MAZE || activeMode == MODE_MAZE_LEFT) {
    runMaze();
    return;
  }

  if (activeMode == MODE_ENCODER) {
    runEncoderTest();
    return;
  }

  // MODE_TEST no tiene control autonomo.
}

// ============================================================
// 11. INTERFACES WEB (en archivos separados)
// ============================================================
// web_test_suite.h: INDEX_HTML -> pagina de pruebas y calibracion
// web_race.h: RACE_HTML -> pantalla de largada RMP
// rmp_logo.h: logo sin conexion a Internet
// Los tres se incluyen arriba; no hay HTML incrustado en este .ino.

// ============================================================
// 12. API
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", RACE_HTML);
}

// Pantalla de carrera: misma logica, PID y parametros del Test Suite.
void handleRace() {
  server.send_P(200, "text/html; charset=utf-8", RACE_HTML);
}

// El logo se lee desde rmp_logo.h, compartido por ambas interfaces.
// Se envia desde PROGMEM; la pagina de carrera lo convierte a imagen local.
void handleRmpLogoBase64() {
  // Enviar desde flash; send_P evita copiar el Base64 a un String grande.
  server.sendHeader("Cache-Control", "public, max-age=3600");
  server.send_P(200, "text/plain", RMP_LOGO_BASE64);
}

void handleStatus() {
  int32_t encPhysicalLeft;
  int32_t encPhysicalRight;

  noInterrupts();
  encPhysicalLeft = encoderRight;
  encPhysicalRight = -encoderLeft;
  interrupts();

  uint32_t encTestDeltaLeft = 0;
  uint32_t encTestDeltaRight = 0;
  getMoveDeltas(encTestDeltaLeft, encTestDeltaRight);
  uint32_t encTestSum = encTestDeltaLeft + encTestDeltaRight;
  uint32_t encTestAverage = encTestSum / 2UL;
  uint32_t encTestProgressTicks =
    (encoderTestAction == ENC_TEST_FORWARD)
      ? (uint32_t)roundf(getMoveStraightCm() * encoderTestTicksPerCm)
      : encTestSum;
  float encTestProgress =
    (encoderTestTarget > 0)
      ? (100.0f * (float)encTestProgressTicks / (float)encoderTestTarget)
      : 0.0f;

  String json;
  json.reserve(2000);

  json += "{";
  json += "\"running\":" + String(running ? "true" : "false") + ",";
  json += "\"timedTestAction\":\"" + String(timedTestName(timedTestAction)) + "\",";
  json += "\"mode\":\"" + String(modeName(activeMode)) + "\",";
  json += "\"state\":\"" + String(stateName(robotState)) + "\",";
  json += "\"stopReason\":\"" + stopReason + "\",";

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

  json += "\"straight\":{";
  json += "\"errorCm\":" + String(straightErrorCm, 3) + ",";
  json += "\"correctionPwm\":" + String(straightCorrectionPwm, 2);
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

  json += "\"encoderTest\":{";
  json += "\"active\":" + String(encoderTestActive ? "true" : "false") + ",";
  json += "\"completed\":" + String(encoderTestCompleted ? "true" : "false") + ",";
  json += "\"action\":\"" + String(encoderTestActionName(encoderTestAction)) + "\",";
  json += "\"target\":" + String(encoderTestTarget) + ",";
  json += "\"deltaLeft\":" + String(encTestDeltaLeft) + ",";
  json += "\"deltaRight\":" + String(encTestDeltaRight) + ",";
  json += "\"sum\":" + String(encTestSum) + ",";
  json += "\"average\":" + String(encTestAverage) + ",";
  json += "\"progress\":" + String(encTestProgress, 2) + ",";
  json += "\"ticksPerCm\":" + String(encoderTestTicksPerCm, 3) + ",";
  json += "\"requestedCm\":" + String(encoderTestRequestedCm, 2);
  json += "},";

  json += "\"config\":{";
  json += "\"kp\":" + String(cfg.kp, 4) + ",";
  json += "\"ki\":" + String(cfg.ki, 4) + ",";
  json += "\"kd\":" + String(cfg.kd, 4) + ",";
  json += "\"targetRightAdc\":" + String(cfg.targetRightAdc) + ",";
  json += "\"targetLeftAdc\":" + String(cfg.targetLeftAdc) + ",";
  json += "\"leftKp\":" + String(cfg.leftKp, 4) + ",";
  json += "\"leftKi\":" + String(cfg.leftKi, 4) + ",";
  json += "\"leftKd\":" + String(cfg.leftKd, 4) + ",";
  json += "\"basePwm\":" + String(cfg.basePwm) + ",";
  json += "\"maxCorrection\":" + String(cfg.maxCorrection) + ",";
  json += "\"frontWallAdc\":" + String(cfg.frontWallAdc) + ",";
  json += "\"frontConfirmAdc\":" + String(cfg.frontConfirmAdc) + ",";
  json += "\"approachMinPwm\":" + String(cfg.approachMinPwm) + ",";
  json += "\"rightOpenAdc\":" + String(cfg.rightOpenAdc) + ",";
  json += "\"leftOpenAdc\":" + String(cfg.leftOpenAdc) + ",";
  json += "\"openingWaitMs\":" + String(cfg.openingWaitMs) + ",";
  json += "\"openingAdvanceCm\":" + String(cfg.openingAdvanceCm, 1) + ",";
  json += "\"openingAdvanceMs\":" + String(cfg.openingAdvanceMs) + ",";
  json += "\"openingLeftPwm\":" + String(cfg.openingLeftPwm) + ",";
  json += "\"openingRightPwm\":" + String(cfg.openingRightPwm) + ",";
  json += "\"turn90LeftMs\":" + String(cfg.turn90LeftMs) + ",";
  json += "\"turn90RightMs\":" + String(cfg.turn90RightMs) + ",";
  json += "\"straightLeftTicksPerCm\":" + String(cfg.straightLeftTicksPerCm, 3) + ",";
  json += "\"straightRightTicksPerCm\":" + String(cfg.straightRightTicksPerCm, 3) + ",";
  json += "\"straightKp\":" + String(cfg.straightKp, 2) + ",";
  json += "\"straightMaxCorrection\":" + String(cfg.straightMaxCorrection) + ",";
  json += "\"turnPwm\":" + String(cfg.turnPwm) + ",";
  json += "\"turn45RightTicks\":" + String(cfg.turn45RightTicks) + ",";
  json += "\"turn45LeftTicks\":" + String(cfg.turn45LeftTicks) + ",";
  json += "\"turn90RightTicks\":" + String(cfg.turn90RightTicks) + ",";
  json += "\"turn90LeftTicks\":" + String(cfg.turn90LeftTicks) + ",";
  json += "\"turn180Ticks\":" + String(cfg.turn180Ticks) + ",";
  json += "\"decisionWaitMs\":" + String(cfg.decisionWaitMs) + ",";
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
  timedTestAction = TIMED_NONE;
  running = false;
  robotState = STATE_STOPPED;
  pendingTurnState = STATE_STOPPED;
  encoderTestActive = false;
  encoderTestCompleted = false;
  stopMotors();
  resetPid();
  frontWallStableCount = 0;
  rightOpenStableCount = 0;
  rightWallStableCount = 0;
  rightOpeningArmed = false;
  leftOpenStableCount = 0;
  leftWallStableCount = 0;
  leftOpeningArmed = false;
  frontLeftTurns = 0;

  if (mode == "TEST") {
    activeMode = MODE_TEST;
  } else if (mode == "WALL") {
    activeMode = MODE_WALL;
  } else if (mode == "MAZE") {
    activeMode = MODE_MAZE;
  } else if (mode == "MAZE_LEFT") {
    activeMode = MODE_MAZE_LEFT;
  } else if (mode == "WALL_LEFT") {
    activeMode = MODE_WALL_LEFT;
  } else if (mode == "ENCODER") {
    activeMode = MODE_ENCODER;
    encoderTestAction = ENC_TEST_NONE;
    encoderTestTarget = 0;
    encoderTestRequestedCm = 0.0f;
    captureMoveStart();
  } else {
    server.send(400, "text/plain", "mode debe ser TEST, WALL, WALL_LEFT, MAZE, MAZE_LEFT o ENCODER");
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

  if (activeMode == MODE_TEST || activeMode == MODE_ENCODER) {
    server.send(400, "text/plain", "Este modo no usa /api/run");
    return;
  }

  bool state = server.arg("state") == "1";

  if (state) {
    stopReason = "NINGUNA";
    resetPid();
    lastHeartbeatMs = millis();
    lastDecisionMs = millis();
    running = true;

    if (activeMode == MODE_MAZE || activeMode == MODE_MAZE_LEFT) {
      frontWallStableCount = 0;
      rightOpenStableCount = 0;
      rightWallStableCount = 0;
      rightOpeningArmed = false;
      leftOpenStableCount = 0;
      leftWallStableCount = 0;
      leftOpeningArmed = false;
      frontLeftTurns = 0;
      enterState(STATE_FOLLOW);
    } else {
      robotState = STATE_FOLLOW;
    }
  } else {
    stopReason = "PARADA_MANUAL_API_RUN";
    running = false;
    robotState = STATE_STOPPED;
    stopMotors();
    resetPid();
  }

  server.send(200, "text/plain", running ? "RUN" : "STOP");
}

void handleMotor() {
  if (timedTestAction != TIMED_NONE) {
    server.send(409, "text/plain", "Prueba temporizada en curso: usa STOP");
    return;
  }
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

// Prueba de avance / giros por tiempo: independiente de Maze y encoders.
void handleTimedTest() {
  if (activeMode != MODE_TEST) {
    server.send(403, "text/plain", "Seleccionar modo TEST");
    return;
  }
  if (!server.hasArg("action")) {
    server.send(400, "text/plain", "Falta action");
    return;
  }
  String action = server.arg("action");
  if (action == "STOP") {
    timedTestAction = TIMED_NONE;
    running = false;
    stopMotors();
    server.send(200, "text/plain", "STOP");
    return;
  }
  if (running || motorLeftCmd != 0 || motorRightCmd != 0) {
    server.send(409, "text/plain", "Detener motores antes de probar");
    return;
  }
  if (action == "FWD") timedTestAction = TIMED_FORWARD;
  else if (action == "L90") timedTestAction = TIMED_LEFT_90;
  else if (action == "R90") timedTestAction = TIMED_RIGHT_90;
  else {
    server.send(400, "text/plain", "action debe ser FWD, L90 o R90");
    return;
  }
  timedTestStartMs = millis();
  lastHeartbeatMs = millis();
  running = true;
  stopReason = "NINGUNA";
  server.send(200, "text/plain", "RUN");
}

void handleEncoderTest() {
  if (activeMode != MODE_ENCODER) {
    server.send(403, "text/plain", "Prueba de encoder solo disponible en modo ENCODER");
    return;
  }

  if (!server.hasArg("action")) {
    server.send(400, "text/plain", "Parametro requerido: action");
    return;
  }

  String action = server.arg("action");

  if (action == "STOP") {
    encoderTestActive = false;
    encoderTestCompleted = false;
    running = false;
    stopMotors();
    server.send(200, "text/plain", "STOP");
    return;
  }

  encoderTestPwm = server.hasArg("pwm")
    ? constrain(server.arg("pwm").toInt(), MIN_MOVING_PWM, 255)
    : MIN_MOVING_PWM;

  encoderTestRequestedCm = 0.0f;

  if (action == "R45") {
    encoderTestAction = ENC_TEST_RIGHT_45;
    encoderTestTarget = server.hasArg("ticks")
      ? (uint32_t)constrain(server.arg("ticks").toInt(), 1, 5000)
      : (uint32_t)cfg.turn45RightTicks;
  } else if (action == "L45") {
    encoderTestAction = ENC_TEST_LEFT_45;
    encoderTestTarget = server.hasArg("ticks")
      ? (uint32_t)constrain(server.arg("ticks").toInt(), 1, 5000)
      : (uint32_t)cfg.turn45LeftTicks;
  } else if (action == "R90") {
    encoderTestAction = ENC_TEST_RIGHT_90;
    encoderTestTarget = server.hasArg("ticks")
      ? (uint32_t)constrain(server.arg("ticks").toInt(), 1, 5000)
      : (uint32_t)cfg.turn90RightTicks;
  } else if (action == "L90") {
    encoderTestAction = ENC_TEST_LEFT_90;
    encoderTestTarget = server.hasArg("ticks")
      ? (uint32_t)constrain(server.arg("ticks").toInt(), 1, 5000)
      : (uint32_t)cfg.turn90LeftTicks;
  } else if (action == "R180") {
    encoderTestAction = ENC_TEST_RIGHT_180;
    encoderTestTarget = server.hasArg("ticks")
      ? (uint32_t)constrain(server.arg("ticks").toInt(), 1, 10000)
      : (uint32_t)cfg.turn180Ticks;
  } else if (action == "FWD") {
    encoderTestAction = ENC_TEST_FORWARD;

    float cm = server.hasArg("cm") ? server.arg("cm").toFloat() : 20.0f;
    float ticksPerCm = server.hasArg("ticksPerCm")
      ? server.arg("ticksPerCm").toFloat()
      : TICKS_PER_CM;

    cm = constrain(cm, 0.5f, 200.0f);
    ticksPerCm = constrain(ticksPerCm, 0.1f, 200.0f);

    encoderTestRequestedCm = cm;
    encoderTestTicksPerCm = ticksPerCm;

    // En avance el objetivo es el promedio de ticks de ambas ruedas.
    encoderTestTarget = (uint32_t)roundf(cm * ticksPerCm);
  } else {
    server.send(400, "text/plain", "action debe ser R45, L45, R90, L90, R180, FWD o STOP");
    return;
  }

  captureMoveStart();
  encoderTestStartMs = millis();
  encoderTestCompleted = false;
  encoderTestActive = true;
  running = true;
  lastHeartbeatMs = millis();

  server.send(200, "text/plain", "RUN");
}

void handleConfig() {
  if (server.hasArg("kp")) cfg.kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) cfg.ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) cfg.kd = server.arg("kd").toFloat();
  if (server.hasArg("leftKp")) cfg.leftKp = constrain(server.arg("leftKp").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("leftKi")) cfg.leftKi = constrain(server.arg("leftKi").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("leftKd")) cfg.leftKd = constrain(server.arg("leftKd").toFloat(), 0.0f, 20.0f);
  if (server.hasArg("targetLeftAdc")) cfg.targetLeftAdc = constrain(server.arg("targetLeftAdc").toInt(), 0, 4095);

  if (server.hasArg("targetRightAdc"))
    cfg.targetRightAdc = constrain(server.arg("targetRightAdc").toInt(), 0, 4095);

  if (server.hasArg("basePwm"))
    cfg.basePwm = constrain(server.arg("basePwm").toInt(), MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);

  if (server.hasArg("maxCorrection"))
    cfg.maxCorrection = constrain(server.arg("maxCorrection").toInt(), 0, 255);

  if (server.hasArg("frontWallAdc"))
    cfg.frontWallAdc = constrain(server.arg("frontWallAdc").toInt(), 0, 4095);

  if (server.hasArg("frontConfirmAdc"))
    cfg.frontConfirmAdc = constrain(server.arg("frontConfirmAdc").toInt(), 0, 4095);

  if (server.hasArg("approachMinPwm"))
    cfg.approachMinPwm = constrain(server.arg("approachMinPwm").toInt(), MAZE_FORWARD_MIN_PWM, MAZE_FORWARD_MAX_PWM);

  if (server.hasArg("rightOpenAdc"))
    cfg.rightOpenAdc = constrain(server.arg("rightOpenAdc").toInt(), 0, 4095);

  if (server.hasArg("leftOpenAdc"))
    cfg.leftOpenAdc = constrain(server.arg("leftOpenAdc").toInt(), 0, 4095);

  if (server.hasArg("openingWaitMs"))
    cfg.openingWaitMs = constrain(server.arg("openingWaitMs").toInt(), 0, 1000);

  if (server.hasArg("openingAdvanceCm"))
    cfg.openingAdvanceCm = constrain(server.arg("openingAdvanceCm").toFloat(), 1.0f, 30.0f);
  if (server.hasArg("openingAdvanceMs"))
    cfg.openingAdvanceMs = constrain(server.arg("openingAdvanceMs").toInt(), 30, 800);
  if (server.hasArg("openingLeftPwm"))
    cfg.openingLeftPwm = constrain(server.arg("openingLeftPwm").toInt(), MIN_MOVING_PWM, 255);
  if (server.hasArg("openingRightPwm"))
    cfg.openingRightPwm = constrain(server.arg("openingRightPwm").toInt(), MIN_MOVING_PWM, 255);
  if (server.hasArg("turn90LeftMs"))
    cfg.turn90LeftMs = constrain(server.arg("turn90LeftMs").toInt(), 50, 1200);
  if (server.hasArg("turn90RightMs"))
    cfg.turn90RightMs = constrain(server.arg("turn90RightMs").toInt(), 50, 1200);
  if (server.hasArg("straightLeftTicksPerCm"))
    cfg.straightLeftTicksPerCm = constrain(server.arg("straightLeftTicksPerCm").toFloat(), 1.0f, 500.0f);
  if (server.hasArg("straightRightTicksPerCm"))
    cfg.straightRightTicksPerCm = constrain(server.arg("straightRightTicksPerCm").toFloat(), 1.0f, 500.0f);
  if (server.hasArg("straightKp"))
    cfg.straightKp = constrain(server.arg("straightKp").toFloat(), 0.0f, 300.0f);
  if (server.hasArg("straightMaxCorrection"))
    cfg.straightMaxCorrection = constrain(server.arg("straightMaxCorrection").toInt(), 0, 80);

  if (server.hasArg("turnPwm"))
    cfg.turnPwm = constrain(server.arg("turnPwm").toInt(), 120, 255);

  if (server.hasArg("turn45RightTicks"))
    cfg.turn45RightTicks = constrain(server.arg("turn45RightTicks").toInt(), 1, 5000);

  if (server.hasArg("turn45LeftTicks"))
    cfg.turn45LeftTicks = constrain(server.arg("turn45LeftTicks").toInt(), 1, 5000);

  if (server.hasArg("turn90RightTicks"))
    cfg.turn90RightTicks = constrain(server.arg("turn90RightTicks").toInt(), 1, 5000);

  if (server.hasArg("turn90LeftTicks"))
    cfg.turn90LeftTicks = constrain(server.arg("turn90LeftTicks").toInt(), 1, 5000);

  if (server.hasArg("turn180Ticks"))
    cfg.turn180Ticks = constrain(server.arg("turn180Ticks").toInt(), 1, 10000);

  if (server.hasArg("decisionWaitMs"))
    cfg.decisionWaitMs = constrain(server.arg("decisionWaitMs").toInt(), 0, 3000);

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
  timedTestAction = TIMED_NONE;
  stopReason = "PARADA_MANUAL_O_PAGINA_CERRADA";
  Serial.println("AUS_KIM STOP: " + stopReason);
  running = false;
  robotState = STATE_STOPPED;
  pendingTurnState = STATE_STOPPED;
  encoderTestActive = false;
  encoderTestCompleted = false;
  stopMotors();
  resetPid();
  rightOpenStableCount = 0;
  rightWallStableCount = 0;
  rightOpeningArmed = false;
  leftOpenStableCount = 0;
  leftWallStableCount = 0;
  leftOpeningArmed = false;
  frontLeftTurns = 0;
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
  server.on("/race", HTTP_GET, handleRace);
  server.on("/api/rmp-logo-b64", HTTP_GET, handleRmpLogoBase64);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/mode", HTTP_GET, handleMode);
  server.on("/api/run", HTTP_GET, handleRun);
  server.on("/api/motor", HTTP_GET, handleMotor);
  server.on("/api/encoder_test", HTTP_GET, handleEncoderTest);
  server.on("/api/timed_test", HTTP_GET, handleTimedTest);
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
    stopReason = "FAILSAFE_WIFI_MS=" + String(now - lastHeartbeatMs);
    Serial.println("AUS_KIM STOP: " + stopReason);
    running = false;
    robotState = STATE_STOPPED;
    pendingTurnState = STATE_STOPPED;
    encoderTestActive = false;
    encoderTestCompleted = false;
    stopMotors();
    resetPid();
    Serial.println("FAIL-SAFE: motores detenidos por perdida de comunicacion.");
  }

  delay(1);
}

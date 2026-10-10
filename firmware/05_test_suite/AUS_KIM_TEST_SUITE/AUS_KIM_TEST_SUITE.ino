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
#include "rmp_logo.h"

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
  MODE_ENCODER
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
  STATE_RIGHT_OPEN_WAIT,
  STATE_RIGHT_OPEN_ADVANCE,
  STATE_RIGHT_OPEN_WAIT_TURN,
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

  // Giros medidos por encoders. Quedan ajustables porque uno de los
  // encoders no esta midiendo de forma totalmente confiable.
  int turn45RightTicks = 80;
  int turn45LeftTicks = 80;
  int turn90RightTicks = 120;
  int turn90LeftTicks = 125;
  int turn180Ticks = 250;

  // Maniobra de apertura lateral:
  // STOP 300 ms -> 5 cm recto -> STOP 300 ms -> giro -> STOP 300 ms.
  int openingWaitMs = 500;
  float openingAdvanceCm = 3.0f;
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

// Avance recto de 5 cm: evita aplicar PID mientras no hay pared derecha.
const int OPENING_ADVANCE_PWM = 180;

// Seguridad si uno de los encoders no reporta movimiento.
const uint16_t OPENING_ADVANCE_TIMEOUT_MS = 1800;
const uint16_t TURN_ENCODER_TIMEOUT_MS = 2200;
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

int calculateApproachPwm() {
  if (frontBlocked) return 0;
  return cfg.basePwm;
}

// ============================================================
// 9. MAZE
// ============================================================

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_FOLLOW:               return "PID PARED DERECHA";
    case STATE_RIGHT_OPEN_WAIT:      return "DERECHA / FRENO 500 MS";
    case STATE_RIGHT_OPEN_ADVANCE:   return "DERECHA / AVANCE 3 CM";
    case STATE_RIGHT_OPEN_WAIT_TURN: return "DERECHA / ESPERA GIRO";
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
    case MODE_WALL:    return "WALL";
    case MODE_MAZE:    return "MAZE";
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

  switch (robotState) {
    case STATE_STOPPED:
      enterState(STATE_FOLLOW);
      break;

    case STATE_FOLLOW:
      // La pared frontal siempre tiene prioridad.
      if (frontBlocked) {
        stopMotors();
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }

      // Si vuelve a ver pared derecha de manera estable,
      // habilita la proxima apertura.
      if (rightWallStableCount >= EVENT_CONFIRM_SAMPLES) {
        rightOpeningArmed = true;
      }

      // Solo reaccionar a PARED -> APERTURA derecha.
      // No usa sensores ni aperturas de izquierda para decidir.
      if (rightOpeningArmed &&
          rightOpenStableCount >= EVENT_CONFIRM_SAMPLES) {
        rightOpeningArmed = false;
        stopMotors();
        enterState(STATE_RIGHT_OPEN_WAIT);
        break;
      }

      // Con pared a derecha PID; sin pared, recto hasta encontrarla.
      if (!rightOpen) {
        followRightWallAtPwm(cfg.basePwm);
      } else {
        resetPid();
        setDrive(cfg.basePwm, cfg.basePwm);
      }
      break;

    case STATE_RIGHT_OPEN_WAIT:
      stopMotors();
      if (frontBlocked) {
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }
      if (now - stateStartMs >= (uint32_t)cfg.openingWaitMs) {
        captureMoveStart();
        enterState(STATE_RIGHT_OPEN_ADVANCE);
      }
      break;

    case STATE_RIGHT_OPEN_ADVANCE: {
      // Solo 5 cm rectos medidos por promedio de ambos encoders.
      // Si aparece pared frontal, detener primero.
      if (frontBlocked) {
        stopMotors();
        frontLeftTurns = 0;
        enterState(STATE_FRONT_WAIT);
        break;
      }

      uint32_t targetTicks =
        (uint32_t)roundf(cfg.openingAdvanceCm * TICKS_PER_CM);
      if (getMoveStraightCm() >= cfg.openingAdvanceCm) {
        stopMotors();
        enterState(STATE_RIGHT_OPEN_WAIT_TURN);
        break;
      }

      if (now - stateStartMs >= OPENING_ADVANCE_TIMEOUT_MS) {
        // No seguir a ciegas si un encoder esta fallando.
        stopReason = "TIMEOUT_AVANCE_5CM_t=" + String(getMoveTicksAverage()) + "_obj="
                   + String((uint32_t)roundf(cfg.openingAdvanceCm * TICKS_PER_CM));
        Serial.println("AUS_KIM STOP: " + stopReason);
        stopMotors();
        running = false;
        robotState = STATE_STOPPED;
        break;
      }

      driveStraightEncoders(OPENING_ADVANCE_PWM);
      break;
    }

    case STATE_RIGHT_OPEN_WAIT_TURN:
      stopMotors();
      if (now - stateStartMs >= 300UL) {
        captureMoveStart();
        enterState(STATE_TURN_RIGHT);
      }
      break;

    case STATE_FRONT_WAIT:
      stopMotors();
      // Mientras el frente siga bloqueado, reevaluar y girar a izquierda.
      // No detener el Maze Solver por la cantidad de giros.
      if (now - stateStartMs >= (uint32_t)cfg.decisionWaitMs) {
        if (frontLeftTurns < 2) frontLeftTurns++;
        captureMoveStart();
        enterState(STATE_TURN_LEFT);
      }
      break;

    case STATE_TURN_LEFT: {
      uint32_t ticks = getMoveTicksSum();
      if (ticks >= (uint32_t)cfg.turn90LeftTicks) {
        stopMotors();
        enterState(STATE_POST_TURN_WAIT);
        break;
      }

      if (now - stateStartMs >= TURN_ENCODER_TIMEOUT_MS) {
        stopReason = "TIMEOUT_GIRO_IZQ_t=" + String(ticks) + "_obj=" + String(cfg.turn90LeftTicks);
        Serial.println("AUS_KIM STOP: " + stopReason);
        stopMotors();
        running = false;
        robotState = STATE_STOPPED;
        break;
      }

      setDrive(-cfg.turnPwm, +cfg.turnPwm);
      break;
    }

    case STATE_TURN_RIGHT: {
      uint32_t ticks = getMoveTicksSum();
      if (ticks >= (uint32_t)cfg.turn90RightTicks) {
        stopMotors();
        enterState(STATE_POST_TURN_WAIT);
        break;
      }

      if (now - stateStartMs >= TURN_ENCODER_TIMEOUT_MS) {
        stopReason = "TIMEOUT_GIRO_DER_t=" + String(ticks) + "_obj=" + String(cfg.turn90RightTicks);
        Serial.println("AUS_KIM STOP: " + stopReason);
        stopMotors();
        running = false;
        robotState = STATE_STOPPED;
        break;
      }

      setDrive(+cfg.turnPwm, -cfg.turnPwm);
      break;
    }

    case STATE_POST_TURN_WAIT:
      stopMotors();
      if (now - stateStartMs < 300UL) {
        break;
      }

      // Si sigue detectando pared frontal, no cancelar la carrera.
      // Repetir evaluacion tras la pausa configurada y girar otra vez.
      // El tiempo de pausa y los encoders evitan giros encadenados sin control.
      if (frontBlocked) {
        Serial.println("AUS_KIM: frente bloqueado despues del giro; buscando salida a izquierda.");
        enterState(STATE_FRONT_WAIT);
        break;
      }

      frontLeftTurns = 0;
      rightOpenStableCount = 0;
      rightWallStableCount = 0;
      rightOpeningArmed = false;
      resetPid();
      enterState(STATE_FOLLOW);
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

  if (activeMode == MODE_ENCODER) {
    runEncoderTest();
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
  --bg:#050507;
  --bg2:#09090d;
  --panel:#101015;
  --panel2:#15151c;
  --panel3:#0c0c11;
  --border:#292934;
  --border-soft:#20202a;
  --text:#f7f7fb;
  --muted:#9696a8;
  --muted2:#6f6f80;
  --violet:#8b5cf6;
  --violet2:#6d28d9;
  --violet-soft:rgba(139,92,246,.14);
  --violet-glow:rgba(139,92,246,.34);
  --good:#22c55e;
  --good-soft:rgba(34,197,94,.13);
  --danger:#ef4444;
  --danger-soft:rgba(239,68,68,.12);
  --warning:#f59e0b;
  --shadow:0 22px 70px rgba(0,0,0,.42);
}
*{box-sizing:border-box}
html{color-scheme:dark;scroll-behavior:smooth}
body{
  margin:0;
  min-height:100vh;
  font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;
  background:
    radial-gradient(circle at 8% 0%,rgba(139,92,246,.16),transparent 28rem),
    radial-gradient(circle at 92% 18%,rgba(109,40,217,.11),transparent 30rem),
    linear-gradient(180deg,#07070a 0%,#050507 100%);
  color:var(--text);
}
body:before{
  content:"";
  position:fixed;
  inset:0;
  pointer-events:none;
  background-image:linear-gradient(rgba(255,255,255,.018) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.018) 1px,transparent 1px);
  background-size:34px 34px;
  mask-image:linear-gradient(to bottom,rgba(0,0,0,.5),transparent 68%);
}
button,input{font:inherit}
button{-webkit-tap-highlight-color:transparent}
.wrap{max-width:1240px;margin:auto;padding:22px 18px 42px;position:relative;z-index:1}

.topbar{
  display:flex;
  align-items:center;
  justify-content:space-between;
  gap:18px;
  margin-bottom:24px;
  padding:10px 2px 18px;
  border-bottom:1px solid rgba(255,255,255,.07);
}
.brand-block{display:flex;align-items:center;gap:14px;min-width:0}
.brand-logo{
  width:auto;
  height:46px;
  max-width:180px;
  object-fit:contain;
  display:block;
  filter:drop-shadow(0 8px 20px rgba(0,0,0,.35));
}
.brand-divider{width:1px;height:34px;background:var(--border)}
.brand-copy{min-width:0}
.brand-kicker{
  color:var(--violet);
  font-size:10px;
  line-height:1;
  font-weight:900;
  letter-spacing:.22em;
  text-transform:uppercase;
  margin-bottom:6px;
}
.brand-name{font-size:14px;font-weight:800;letter-spacing:.04em;white-space:nowrap}
.connection{
  display:flex;
  align-items:center;
  gap:8px;
  padding:8px 12px;
  border:1px solid rgba(139,92,246,.28);
  border-radius:999px;
  background:rgba(139,92,246,.08);
  color:#ddd4ff;
  font-size:12px;
  font-weight:800;
  white-space:nowrap;
}
.dot{
  width:8px;height:8px;border-radius:50%;
  background:var(--good);
  box-shadow:0 0 0 4px rgba(34,197,94,.08),0 0 18px rgba(34,197,94,.65);
}

.hero{
  display:grid;
  grid-template-columns:minmax(0,1fr) auto;
  gap:20px;
  align-items:end;
  padding:16px 0 22px;
}
.hero-eyebrow{
  display:inline-flex;
  align-items:center;
  gap:9px;
  color:#b9a6ff;
  font-size:11px;
  font-weight:900;
  letter-spacing:.18em;
  text-transform:uppercase;
  margin-bottom:10px;
}
.hero-eyebrow:before{
  content:"";
  width:24px;height:2px;border-radius:2px;
  background:linear-gradient(90deg,var(--violet),transparent);
}
h1{
  margin:0;
  font-size:clamp(42px,7vw,76px);
  line-height:.92;
  letter-spacing:-.055em;
  font-weight:950;
  background:linear-gradient(180deg,#fff 0%,#cfcfd7 100%);
  -webkit-background-clip:text;
  background-clip:text;
  color:transparent;
}
.subtitle{
  color:var(--muted);
  margin-top:11px;
  font-size:14px;
  line-height:1.5;
}
.hero-mark{
  width:112px;height:112px;
  border-radius:28px;
  border:1px solid rgba(139,92,246,.34);
  background:
    radial-gradient(circle at 32% 22%,rgba(180,151,255,.24),transparent 42%),
    linear-gradient(145deg,rgba(139,92,246,.16),rgba(12,12,17,.9));
  display:grid;
  place-items:center;
  box-shadow:inset 0 1px rgba(255,255,255,.08),0 18px 50px rgba(75,32,140,.22);
}
.hero-mark span{
  font-size:30px;
  font-weight:950;
  letter-spacing:-.06em;
}
.hero-mark small{
  display:block;
  margin-top:2px;
  color:#ad96ff;
  text-align:center;
  font-size:8px;
  letter-spacing:.18em;
  font-weight:900;
}

.globalbar{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:10px;
  margin:2px 0 16px;
}
.badge{
  min-height:64px;
  display:flex;
  flex-direction:column;
  justify-content:center;
  gap:4px;
  padding:11px 14px;
  border:1px solid var(--border-soft);
  border-radius:14px;
  background:linear-gradient(180deg,rgba(18,18,24,.92),rgba(12,12,17,.92));
  box-shadow:inset 0 1px rgba(255,255,255,.035);
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.1em;
}
.badge b{
  color:var(--text);
  font-size:14px;
  letter-spacing:.02em;
  text-transform:none;
}

.tabs{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:8px;
  padding:6px;
  margin-bottom:18px;
  border:1px solid var(--border-soft);
  border-radius:16px;
  background:rgba(10,10,14,.88);
  box-shadow:var(--shadow);
}
.tabbtn{
  position:relative;
  min-height:48px;
  padding:10px 12px;
  border:1px solid transparent;
  background:transparent;
  color:#9c9cab;
  border-radius:11px;
  font-weight:850;
  font-size:12px;
  letter-spacing:.01em;
  cursor:pointer;
  transition:.18s ease;
}
.tabbtn:hover{color:#fff;background:rgba(255,255,255,.035)}
.tabbtn.active{
  color:#fff;
  background:linear-gradient(180deg,#8b5cf6,#6d28d9);
  border-color:#9f7aea;
  box-shadow:0 10px 24px rgba(109,40,217,.28),inset 0 1px rgba(255,255,255,.2);
}

.panel{display:none;animation:fadeIn .18s ease}
.panel.active{display:block}
@keyframes fadeIn{from{opacity:.35;transform:translateY(3px)}to{opacity:1;transform:none}}

.grid{
  display:grid;
  grid-template-columns:repeat(auto-fit,minmax(300px,1fr));
  gap:14px;
}
.card{
  position:relative;
  overflow:hidden;
  background:
    linear-gradient(180deg,rgba(18,18,24,.96),rgba(12,12,17,.96));
  border:1px solid var(--border-soft);
  border-radius:18px;
  padding:18px;
  box-shadow:0 18px 50px rgba(0,0,0,.24),inset 0 1px rgba(255,255,255,.035);
}
.card:before{
  content:"";
  position:absolute;
  left:0;top:0;
  width:3px;height:52px;
  background:linear-gradient(180deg,var(--violet),transparent);
  opacity:.95;
}
.card h2{
  margin:0 0 16px;
  font-size:15px;
  letter-spacing:-.01em;
  font-weight:900;
}
.card h2:after{
  content:"";
  display:block;
  width:28px;height:2px;
  border-radius:2px;
  background:var(--violet);
  margin-top:8px;
  opacity:.8;
}
.metric-grid,.sensor-grid{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:9px;
}
.metric{
  position:relative;
  border:1px solid var(--border-soft);
  border-radius:13px;
  padding:12px 10px;
  text-align:center;
  min-width:0;
  background:linear-gradient(180deg,rgba(255,255,255,.022),rgba(255,255,255,.008));
}
.metric:hover{border-color:rgba(139,92,246,.3)}
.label{
  color:var(--muted);
  font-size:10px;
  line-height:1.35;
  text-transform:uppercase;
  letter-spacing:.075em;
}
.label b{color:#c7c7d3}
.value{
  font-size:25px;
  font-variant-numeric:tabular-nums;
  font-weight:950;
  line-height:1.05;
  margin:7px 0 5px;
  overflow-wrap:anywhere;
  color:#fff;
}
.field{
  display:grid;
  grid-template-columns:1fr 122px;
  gap:12px;
  align-items:center;
  margin:9px 0;
  padding-bottom:9px;
  border-bottom:1px solid rgba(255,255,255,.045);
}
.field:last-of-type{border-bottom:0}
.field span{
  color:#b1b1bf;
  font-size:12px;
  font-weight:650;
}
input{
  width:100%;
  padding:9px 10px;
  border-radius:9px;
  border:1px solid #30303c;
  outline:0;
  background:#09090d;
  color:#fff;
  font-weight:800;
  font-variant-numeric:tabular-nums;
  transition:.15s ease;
}
input:focus{
  border-color:var(--violet);
  box-shadow:0 0 0 3px rgba(139,92,246,.12);
}
button{
  border:1px solid #30303c;
  background:linear-gradient(180deg,#1b1b23,#131319);
  color:#eeeef5;
  border-radius:10px;
  padding:11px;
  font-weight:900;
  font-size:11px;
  letter-spacing:.025em;
  cursor:pointer;
  transition:.15s ease;
}
button:hover{filter:brightness(1.1);transform:translateY(-1px)}
button:active{transform:translateY(0) scale(.99)}
.full{width:100%;margin-top:9px}
.start{
  background:linear-gradient(180deg,#8b5cf6,#6d28d9);
  border-color:#9f7aea;
  color:#fff;
  box-shadow:0 9px 22px rgba(109,40,217,.22);
}
.stop{
  background:rgba(239,68,68,.08);
  border-color:rgba(239,68,68,.38);
  color:#ff9d9d;
}
.status{
  position:relative;
  padding:11px 12px 11px 30px;
  border:1px solid rgba(139,92,246,.24);
  border-radius:10px;
  text-align:left;
  font-size:11px;
  font-weight:900;
  letter-spacing:.08em;
  margin-bottom:10px;
  color:#ddd5ff;
  background:rgba(139,92,246,.07);
}
.status:before{
  content:"";
  position:absolute;
  left:12px;top:50%;
  width:8px;height:8px;
  margin-top:-4px;
  border-radius:50%;
  background:var(--violet);
  box-shadow:0 0 14px var(--violet-glow);
}
.motor-buttons{
  display:grid;
  grid-template-columns:1fr 1fr 1fr;
  gap:7px;
  margin-top:10px;
}
.motor-buttons button:first-child{color:#c7b8ff}
.motor-buttons button:last-child{
  color:#fff;
  border-color:rgba(139,92,246,.34);
  background:rgba(139,92,246,.10);
}
.flags{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:8px;
  margin-top:12px;
}
.flag{
  padding:10px 8px;
  border:1px solid var(--border-soft);
  border-radius:10px;
  text-align:center;
  font-size:10px;
  line-height:1.5;
  color:var(--muted);
  background:#0c0c11;
  text-transform:uppercase;
  letter-spacing:.06em;
}
.flag b{
  color:#fff;
  font-size:11px;
}
.hint{
  color:var(--muted2);
  font-size:10px;
  line-height:1.5;
  margin-top:11px;
  padding:9px 10px;
  border-left:2px solid rgba(139,92,246,.55);
  background:rgba(139,92,246,.035);
}
.footer-note{
  display:flex;
  align-items:center;
  justify-content:space-between;
  gap:14px;
  color:#666676;
  margin-top:20px;
  padding:16px 2px 0;
  border-top:1px solid rgba(255,255,255,.055);
  font-size:10px;
  letter-spacing:.04em;
}
.footer-note strong{color:#9c89e8}

@media(max-width:760px){
  .topbar{align-items:flex-start}
  .brand-logo{height:38px;max-width:150px}
  .brand-divider,.brand-copy{display:none}
  .hero{grid-template-columns:1fr}
  .hero-mark{display:none}
  .globalbar{grid-template-columns:1fr 1fr}
  .globalbar .badge:last-child{grid-column:1/-1}
}
@media(max-width:650px){
  .wrap{padding:15px 12px 30px}
  .tabs{grid-template-columns:1fr;padding:5px}
  .motor-buttons{grid-template-columns:1fr 1fr 1fr}
  .field{grid-template-columns:1fr 105px}
  .grid{grid-template-columns:1fr}
  .hero{padding-top:7px}
  h1{font-size:46px}
  .connection{font-size:10px;padding:7px 9px}
  .metric-grid,.sensor-grid{gap:7px}
  .card{padding:15px;border-radius:15px}
  .value{font-size:22px}
  .footer-note{flex-direction:column;align-items:flex-start}
}
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <div class="brand-block">
      <img class="brand-logo" id="rmpBrandLogo" alt="RMP Robotics">
      <div class="brand-divider"></div>
      <div class="brand-copy">
        <div class="brand-kicker">Robotics Control</div>
        <div class="brand-name">AUS_KIM TEST SUITE</div>
      </div>
    </div>
    <div class="connection"><span class="dot"></span> ESP32-S3 ONLINE</div>
  </div>

  <div class="hero">
    <div>
      <div class="hero-eyebrow">RMP Robotics · Micromouse Lab</div>
      <h1>AUS_KIM</h1>
      <div class="subtitle">Diagnóstico, seguimiento de pared y resolución de laberinto en tiempo real.</div>
    </div>
    <div class="hero-mark"><div><span>AK</span><small>CONTROL</small></div></div>
  </div>

  <div class="globalbar">
    <div class="badge">Modo: <b id="globalMode">TEST</b></div>
    <div class="badge">Estado: <b id="globalState">DETENIDO</b></div>
    <div class="badge">IP: <b>192.168.4.1</b></div>
    <div class="badge"><a href="/race" style="color:#c4b5fd;text-decoration:none;font-weight:900">MODO CARRERA ↗</a></div>
  </div>

  <div class="tabs">
    <button class="tabbtn active" id="tabBtnTest" onclick="showTab('test')">Sensores / Motores / Encoders</button>
    <button class="tabbtn" id="tabBtnWall" onclick="showTab('wall')">PID pared derecha</button>
    <button class="tabbtn" id="tabBtnMaze" onclick="showTab('maze')">Resolver laberinto</button>
    <button class="tabbtn" id="tabBtnEncoder" onclick="showTab('encoder')">Calibrar encoders</button>
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
        <div class="field"><span>PWM base PID</span><input class="cfg" id="wallBasePwm" type="number" min="120" max="255" step="1"></div>
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
        <div class="hint">Ultima detencion: <b id="mazeStopReason">NINGUNA</b></div>
        <button class="full start" onclick="startAutonomous('MAZE')">INICIAR LABERINTO</button>
        <button class="full stop" onclick="stopAll()">STOP</button>

        <div class="flags">
          <div class="flag">Frente<br><b id="mFrontFlag">LIBRE</b></div>
          <div class="flag">Derecha<br><b id="mRightFlag">PARED</b></div>
          <div class="flag">Izquierda (lectura)<b id="mLeftFlag">PARED</b></div>
        </div>

        <div class="hint">
          PID derecha a PWM configurable (inicial 180). Solo detecta APERTURA DERECHA (<1600) despues de haber visto pared derecha: frena 500 ms, avanza 3 cm, espera 300 ms, gira 90° derecha y espera 300 ms. Con pared frontal (1900), gira 90° izquierda; si sigue bloqueado, otros 90° izquierda para regresar. No usa apertura izquierda.
        </div>
      </div>

      <div class="card">
        <h2>Deteccion y frenado</h2>
        <div class="field"><span>STOP frontal ADC (≥)</span><input class="cfg" id="frontWallAdc" type="number" step="1"></div>
        <div class="field"><span>Confirmacion segundo frontal ADC</span><input class="cfg" id="frontConfirmAdc" type="number" step="1"></div>
        <div class="field"><span>Apertura derecha ADC</span><input class="cfg" id="rightOpenAdc" type="number" step="1"></div>
        <div class="field"><span>Espera antes de avanzar (ms)</span><input class="cfg" id="openingWaitMs" type="number" min="0" max="1000" step="10"></div>
        <div class="field"><span>Avance antes de girar derecha (cm)</span><input class="cfg" id="openingAdvanceCm" type="number" min="1" max="30" step="0.5"></div>
      </div>

      <div class="card">
        <h2>Giros por encoders</h2>
        <div class="field"><span>PWM avance Maze</span><input class="cfg" id="basePwm" type="number" min="120" max="255" step="1"></div>
        <div class="field"><span>PWM giro</span><input class="cfg" id="turnPwm" type="number" min="120" max="255" step="1"></div>
        <div class="field"><span>Giro 90° derecha (ticks suma)</span><input class="cfg" id="turn90RightTicks" type="number" min="1" max="5000" step="1"></div>
        <div class="field"><span>Giro 90° izquierda (ticks suma)</span><input class="cfg" id="turn90LeftTicks" type="number" min="1" max="5000" step="1"></div>
        <div class="field"><span>STOP previo a giro frontal (ms)</span><input class="cfg" id="decisionWaitMs" type="number" min="0" max="1500" step="10"></div>
        <div class="hint">Los giros usan la suma absoluta de ambos encoders. Como uno está midiendo mal, los ticks quedan ajustables para calibrarlos físicamente. Si luego identificamos cuál encoder falla, podemos pasar a usar solo el bueno.</div>
        <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>
      </div>

      <div class="card">
        <h2>Avance recto con encoders</h2>
        <div class="hint">Sincroniza las ruedas en el avance de 3 cm y en la prueba AVANCE. Si los encoders son distintos, calibra ticks/cm de cada rueda.</div>
        <div class="field"><span>Ticks/cm izquierdo</span><input class="cfg" id="straightLeftTicksPerCm" type="number" min="1" max="500" step="0.05"></div>
        <div class="field"><span>Ticks/cm derecho</span><input class="cfg" id="straightRightTicksPerCm" type="number" min="1" max="500" step="0.05"></div>
        <div class="field"><span>Kp avance recto (PWM/cm)</span><input class="cfg" id="straightKp" type="number" min="0" max="300" step="5"></div>
        <div class="field"><span>Correccion maxima PWM</span><input class="cfg" id="straightMaxCorrection" type="number" min="0" max="80" step="1"></div>
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
          <div class="metric"><div class="label">Encoder izquierdo</div><div class="value" id="mEncL">0</div></div>
          <div class="metric"><div class="label">Encoder derecho</div><div class="value" id="mEncR">0</div></div>
          <div class="metric"><div class="label">Error PID</div><div class="value" id="mError">0</div></div>
          <div class="metric"><div class="label">Correccion PID</div><div class="value" id="mCorrection">0</div></div>
          <div class="metric"><div class="label">Error avance recto (cm)</div><div class="value" id="mStraightError">0</div></div>
          <div class="metric"><div class="label">Correccion avance recto (PWM)</div><div class="value" id="mStraightTrim">0</div></div>
        </div>
      </div>

    </div>
  </section>

  <!-- ====================================================== -->
  <!-- TAB 4: CALIBRACION DE ENCODERS -->
  <!-- ====================================================== -->
  <section class="panel" id="panelEncoder">
    <div class="grid">

      <div class="card">
        <h2>Calibracion de encoders</h2>
        <div class="status" id="calState">DETENIDO</div>
        <div class="hint">
          Cada prueba toma una referencia nueva de ambos encoders y se detiene automaticamente.
          Para giros se usa la suma |ΔL| + |ΔR|. Para distancia se usa el promedio (|ΔL| + |ΔR|) / 2.
        </div>
        <div class="field"><span>PWM de prueba</span><input id="calPwm" type="number" min="155" max="255" step="1" value="155"></div>
        <button class="full stop" onclick="stopAll()">STOP</button>
      </div>

      <div class="card">
        <h2>Giro derecha 45°</h2>
        <div class="field"><span>Ticks objetivo (suma)</span><input id="cal45RightTicks" type="number" min="1" step="1" value="80"></div>
        <button class="full start" onclick="startEncoderTest('R45')">PROBAR 45° DERECHA</button>
        <div class="hint">Calibración independiente del giro de 45° hacia la derecha.</div>
      </div>

      <div class="card">
        <h2>Giro izquierda 45°</h2>
        <div class="field"><span>Ticks objetivo (suma)</span><input id="cal45LeftTicks" type="number" min="1" step="1" value="80"></div>
        <button class="full start" onclick="startEncoderTest('L45')">PROBAR 45° IZQUIERDA</button>
        <div class="hint">Calibración independiente del giro de 45° hacia la izquierda.</div>
      </div>

      <div class="card">
        <h2>Giro derecha 90°</h2>
        <div class="field"><span>Ticks objetivo (suma)</span><input id="cal90RightTicks" type="number" min="1" step="1" value="170"></div>
        <button class="full start" onclick="startEncoderTest('R90')">PROBAR 90° DERECHA</button>
        <div class="hint">Este valor se usa también para los giros a derecha del Maze.</div>
      </div>

      <div class="card">
        <h2>Giro izquierda 90°</h2>
        <div class="field"><span>Ticks objetivo (suma)</span><input id="cal90LeftTicks" type="number" min="1" step="1" value="175"></div>
        <button class="full start" onclick="startEncoderTest('L90')">PROBAR 90° IZQUIERDA</button>
        <div class="hint">Este valor se usa también para los giros a izquierda del Maze.</div>
      </div>

      <div class="card">
        <h2>Giro derecha 180°</h2>
        <div class="field"><span>Ticks objetivo (suma)</span><input id="cal180Ticks" type="number" min="1" step="1" value="387"></div>
        <button class="full start" onclick="startEncoderTest('R180')">PROBAR 180° DERECHA</button>
        <div class="hint">Prueba independiente del encoder. El Maze no usa este valor.</div>
      </div>

      <div class="card">
        <h2>Avance por distancia</h2>
        <div class="field"><span>Distancia ordenada (cm)</span><input id="calCm" type="number" min="0.5" max="200" step="0.5" value="20"></div>
        <div class="field"><span>Ticks por cm</span><input id="calTicksPerCm" type="number" min="0.1" max="200" step="0.01" value="20.95"></div>
        <button class="full start" onclick="startEncoderTest('FWD')">AVANZAR X CM</button>
        <div class="field"><span>Distancia real medida (cm)</span><input id="calMeasuredCm" type="number" min="0.1" max="300" step="0.1" value="20"></div>
        <button class="full" onclick="calculateTicksPerCm()">CALCULAR TICKS/CM CON RESULTADO</button>
        <div class="hint">Despues de medir con regla la distancia real recorrida, el boton calcula: promedio de ticks / centimetros reales.</div>
      </div>

      <div class="card">
        <h2>Resultado en vivo</h2>
        <div class="metric-grid">
          <div class="metric"><div class="label">Encoder izquierdo</div><div class="value" id="calEncL">0</div></div>
          <div class="metric"><div class="label">Encoder derecho</div><div class="value" id="calEncR">0</div></div>
          <div class="metric"><div class="label">Δ izquierdo</div><div class="value" id="calDeltaL">0</div></div>
          <div class="metric"><div class="label">Δ derecho</div><div class="value" id="calDeltaR">0</div></div>
          <div class="metric"><div class="label">Suma |ΔL|+|ΔR|</div><div class="value" id="calSum">0</div></div>
          <div class="metric"><div class="label">Promedio</div><div class="value" id="calAvg">0</div></div>
          <div class="metric"><div class="label">Objetivo activo</div><div class="value" id="calTarget">0</div></div>
          <div class="metric"><div class="label">Progreso</div><div class="value" id="calProgress">0%</div></div>
        </div>
      </div>

    </div>
  </section>

  <div class="footer-note">
    <span><strong>RMP ROBOTICS</strong> · AUS_KIM</span>
    <span>ESP32-S3 · DRV8833 · 4× Sharp GP2Y0E03</span>
  </div>
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

  ['test','wall','maze','encoder'].forEach(t=>{
    document.getElementById('panel'+panelName(t)).classList.toggle('active',t===tab);
    document.getElementById('tabBtn'+panelName(t)).classList.toggle('active',t===tab);
  });

  const mode = tab==='test' ? 'TEST' : (tab==='wall' ? 'WALL' : (tab==='maze' ? 'MAZE' : 'ENCODER'));
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

async function startEncoderTest(action){
  const pwm=Math.max(155,Math.min(255,parseInt(document.getElementById('calPwm').value||'155',10)));
  const p=new URLSearchParams({action:action,pwm:String(pwm)});

  if(action==='R45'){
    p.set('ticks',document.getElementById('cal45RightTicks').value);
  }else if(action==='L45'){
    p.set('ticks',document.getElementById('cal45LeftTicks').value);
  }else if(action==='R90'){
    p.set('ticks',document.getElementById('cal90RightTicks').value);
  }else if(action==='L90'){
    p.set('ticks',document.getElementById('cal90LeftTicks').value);
  }else if(action==='R180'){
    p.set('ticks',document.getElementById('cal180Ticks').value);
  }else if(action==='FWD'){
    p.set('cm',document.getElementById('calCm').value);
    p.set('ticksPerCm',document.getElementById('calTicksPerCm').value);
  }

  await fetch('/api/encoder_test?'+p.toString(),{cache:'no-store'});
  await updateStatus();
}

function calculateTicksPerCm(){
  const realCm=parseFloat(document.getElementById('calMeasuredCm').value||'0');
  const avg=parseFloat(document.getElementById('calAvg').textContent||'0');
  if(realCm>0 && avg>0){
    document.getElementById('calTicksPerCm').value=(avg/realCm).toFixed(3);
  }
}

async function applyConfig(){
  const ids=[
    'kp','ki','kd','targetRightAdc','basePwm','maxCorrection',
    'frontWallAdc','frontConfirmAdc',
    'rightOpenAdc','openingWaitMs','openingAdvanceCm','turnPwm',
    'straightLeftTicksPerCm','straightRightTicksPerCm','straightKp','straightMaxCorrection',
    'turn45RightTicks','turn45LeftTicks',
    'turn90RightTicks','turn90LeftTicks','turn180Ticks','decisionWaitMs'
  ];

  const p=new URLSearchParams();

  ids.forEach(id=>{
    const el=document.getElementById(id);
    if(el)p.set(id,el.value);
  });

  // Una sola configuracion PWM para PID/Maze, sin IDs HTML duplicados.
  if(activeTab==='wall'){
    p.set('basePwm',document.getElementById('wallBasePwm').value);
    document.getElementById('basePwm').value=p.get('basePwm');
  }else if(activeTab==='maze'){
    document.getElementById('wallBasePwm').value=p.get('basePwm');
  }

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
    document.getElementById('mazeStopReason').textContent=d.stopReason||'NINGUNA';
    document.getElementById('mFrontFlag').textContent=d.flags.frontBlocked?'PARED':'LIBRE';
    document.getElementById('mRightFlag').textContent=d.flags.rightOpen?'LIBRE':'PARED';
    document.getElementById('mLeftFlag').textContent=d.flags.leftOpen?'LIBRE':'PARED';
    setSensor('m',d);
    document.getElementById('mMotorL').textContent=d.motor.left;
    document.getElementById('mMotorR').textContent=d.motor.right;
    document.getElementById('mEncL').textContent=d.enc.left;
    document.getElementById('mEncR').textContent=d.enc.right;
    document.getElementById('mError').textContent=d.pid.error.toFixed(1);
    document.getElementById('mCorrection').textContent=d.pid.correction.toFixed(1);
    document.getElementById('mStraightError').textContent=d.straight.errorCm.toFixed(2);
    document.getElementById('mStraightTrim').textContent=d.straight.correctionPwm.toFixed(1);

    // CALIBRACION ENCODERS
    document.getElementById('calState').textContent=
      d.encoderTest.active ? d.encoderTest.action :
      (d.encoderTest.completed ? 'COMPLETADO · '+d.encoderTest.action : 'DETENIDO');
    document.getElementById('calEncL').textContent=d.enc.left;
    document.getElementById('calEncR').textContent=d.enc.right;
    document.getElementById('calDeltaL').textContent=d.encoderTest.deltaLeft;
    document.getElementById('calDeltaR').textContent=d.encoderTest.deltaRight;
    document.getElementById('calSum').textContent=d.encoderTest.sum;
    document.getElementById('calAvg').textContent=d.encoderTest.average;
    document.getElementById('calTarget').textContent=d.encoderTest.target;
    document.getElementById('calProgress').textContent=d.encoderTest.progress.toFixed(1)+'%';

    if(firstLoad){
      Object.keys(d.config).forEach(k=>{
        const el=document.getElementById(k);
        if(el)el.value=d.config[k];
      });

      document.getElementById('wallBasePwm').value=d.config.basePwm;
      document.getElementById('manualPwmTest').value=d.config.manualPwm;
      document.getElementById('calPwm').value=Math.max(155,d.config.turnPwm);
      document.getElementById('cal45RightTicks').value=d.config.turn45RightTicks;
      document.getElementById('cal45LeftTicks').value=d.config.turn45LeftTicks;
      document.getElementById('cal90RightTicks').value=d.config.turn90RightTicks;
      document.getElementById('cal90LeftTicks').value=d.config.turn90LeftTicks;
      document.getElementById('cal180Ticks').value=d.config.turn180Ticks;
      document.getElementById('calTicksPerCm').value=d.encoderTest.ticksPerCm.toFixed(2);
      firstLoad=false;
    }
  }catch(_){}
}

// El logo se obtiene desde el ESP32, sin depender de Internet.
// Se almacena en rmp_logo.h (PROGMEM) y no dentro del HTML.
async function cargarLogoRmp(){
  try{
    const response=await fetch('/api/rmp-logo-b64',{cache:'force-cache'});
    if(!response.ok)throw new Error('Logo no disponible');
    const base64=(await response.text()).trim();
    const logo=document.getElementById('rmpBrandLogo');
    if(logo)logo.src='data:image/png;base64,'+base64;
  }catch(_){/* La interfaz sigue operativa sin el logo. */}
}
cargarLogoRmp();

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

const char RACE_HTML[] PROGMEM = R"RACE(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#08070e">
<title>RMP Robotics | AUS_KIM Race</title>
<style>
:root{--bg:#08070e;--violet:#a78bfa;--bright:#8b5cf6;--white:#f8f7ff;--muted:#9490a7;--red:#f43f5e}
*{box-sizing:border-box}html,body{margin:0;min-height:100%;background:var(--bg);color:var(--white);font-family:system-ui,-apple-system,Segoe UI,Arial,sans-serif}
body{min-height:100dvh;display:flex;align-items:center;justify-content:center;padding:24px;overflow-x:hidden;background:radial-gradient(ellipse 57% 42% at 50% 40%,#27164a 0%,#100b1d 60%,#08070e 100%)}
main{width:min(100%,480px);display:flex;flex-direction:column;align-items:center;text-align:center}
.overline{font-weight:800;letter-spacing:.32em;font-size:11px;color:#a594c9;text-transform:uppercase}
h1{font-size:clamp(24px,6vw,33px);font-weight:900;letter-spacing:.04em;margin:12px 0 0}
.subtitle{font-size:12px;color:var(--muted);letter-spacing:.24em;margin:8px 0 30px;text-transform:uppercase}
.ring{position:relative;width:min(69vw,306px);height:min(69vw,306px);max-width:306px;max-height:306px;display:grid;place-items:center}
.ring:before{content:'';position:absolute;inset:-13px;border-radius:50%;border:1px solid rgba(167,139,250,.26);box-shadow:0 0 65px #5b21b650}
.launch{position:relative;border:1px solid #b095ff;cursor:pointer;background:radial-gradient(circle at 50% 30%,#462384 0%,#251448 64%,#130e22 100%);border-radius:50%;height:100%;width:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:7px;box-shadow:inset 0 1px 27px #b99aff36,0 0 38px #8b5cf640,0 21px 45px #0008;color:var(--white);-webkit-tap-highlight-color:transparent;touch-action:manipulation;transition:transform .16s,box-shadow .16s}
.launch:active:not(:disabled){transform:scale(.967)}.launch:disabled{cursor:default;opacity:.87}
.launch img{width:63%;height:60%;object-fit:contain;filter:drop-shadow(0 5px 10px #0007)}
.logoFallback{font-weight:1000;font-size:clamp(48px,14vw,78px);letter-spacing:-.09em;font-style:italic;text-shadow:0 3px 20px #c4b5fd88}
.launch .action{font-weight:1000;font-size:19px;letter-spacing:.19em;padding-left:.19em}
.launch.running{border-color:#ddd0ff;box-shadow:0 0 12px #c7b1ff60,0 0 70px #8b5cf663,inset 0 1px 30px #c4a1ff4d}
.state{margin:34px 0 10px;display:flex;align-items:center;gap:9px;color:#bdb4cd;font-size:12px;font-weight:850;letter-spacing:.17em;text-transform:uppercase}
.dot{width:9px;height:9px;border-radius:50%;background:#8b849e;box-shadow:0 0 10px #73678b}.dot.live{background:#a78bfa;box-shadow:0 0 15px #a78bfa}
.stateDetail{margin:0;min-height:22px;font-size:13px;color:var(--muted);max-width:310px;overflow-wrap:anywhere}
.stop{margin:24px 0 0;width:min(100%,260px);padding:17px 22px;border:1px solid #d74f65;border-radius:14px;background:#4e1421;color:#fff;font-size:15px;letter-spacing:.15em;font-weight:900;cursor:pointer;touch-action:manipulation}
.stop:disabled{opacity:.4;cursor:default}.stop:active:not(:disabled){background:#831e37}
footer{margin-top:34px;font-size:10px;font-weight:700;letter-spacing:.18em;color:#7f7794}
.notice{min-height:21px;font-size:12px;color:#ffb4c0;margin-top:12px}
@media (max-height:650px){.subtitle{margin-bottom:15px}.ring{width:min(56vh,270px);height:min(56vh,270px)}.state{margin-top:22px}.stop{margin-top:15px}footer{margin-top:15px}}
</style>
</head>
<body>
<main>
  <div class="overline">RMP ROBOTICS</div>
  <h1>AUS_KIM</h1>
  <div class="subtitle">Race control · Micromouse</div>
  <div class="ring">
    <button id="launch" class="launch" type="button" onclick="startRace()" aria-label="Largar AUS KIM">
      <span id="fallback" class="logoFallback">RMP</span>
      <img id="logo" alt="RMP Robotics" hidden>
      <span id="launchText" class="action">LARGAR</span>
    </button>
  </div>
  <div class="state"><span id="dot" class="dot"></span><span id="status">CONECTANDO</span></div>
  <p class="stateDetail" id="detail">Estableciendo conexion con el robot...</p>
  <button id="stop" type="button" class="stop" onclick="stopRace()">■ STOP</button>
  <div class="notice" id="message" role="status" aria-live="polite"></div>
  <footer>NEGRO / VIOLETA / BLANCO · RMP</footer>
</main>
<script>
let busy=false;
let connected=false;
let racing=false;
let seenStop=false;
const el=id=>document.getElementById(id);
const req=async path=>{
  const r=await fetch(path,{cache:'no-store'});
  if(!r.ok)throw new Error('HTTP '+r.status);
  return r;
};
async function startRace(){
  if(busy||racing)return;
  busy=true;el('launch').disabled=true;el('message').textContent='';
  try{
    await req('/api/mode?mode=MAZE');
    await req('/api/run?state=1');
    seenStop=false;
    await refresh();
  }catch(e){el('message').textContent='No se pudo largar: '+e.message;}
  finally{busy=false;await refresh();}
}
async function stopRace(){
  if(busy)return;
  busy=true;el('message').textContent='';
  try{await req('/api/stop');seenStop=true;await refresh();}
  catch(e){el('message').textContent='No se pudo enviar STOP: '+e.message;}
  finally{busy=false;await refresh();}
}
async function refresh(){
  try{
    const data=await (await req('/api/status')).json();
    connected=true;
    racing=data.mode==='MAZE'&&data.running===true;
    el('status').textContent=racing?'EN CARRERA':(seenStop||data.stopReason&&data.stopReason!=='NINGUNA'?'DETENIDO':'LISTO PARA LARGAR');
    el('detail').textContent=racing?data.state:(data.stopReason&&data.stopReason!=='NINGUNA'?data.stopReason:'Toca el logo para iniciar el laberinto');
    el('dot').classList.toggle('live',racing);
    el('launch').classList.toggle('running',racing);
    el('launchText').textContent=racing?'CORRIENDO':'LARGAR';
  }catch(_){
    connected=false;racing=false;el('status').textContent='SIN CONEXION';
    el('detail').textContent='Revisa la red Wi-Fi AUS_KIM_TEST';
    el('dot').classList.remove('live');
  }
  el('launch').disabled=busy||!connected||racing;
  el('stop').disabled=busy||!connected;
}
async function loadLogo(){
  try{
    const encoded=await (await req('/api/rmp-logo-b64')).text();
    if(!/^[A-Za-z0-9+\/=\s]+$/.test(encoded))throw Error('logo');
    el('logo').src='data:image/png;base64,'+encoded.trim();
    el('logo').onload=()=>{el('fallback').hidden=true;el('logo').hidden=false};
  }catch(_){/* Fallback RMP sigue visible sin internet. */}
}
setInterval(()=>req('/api/ping').catch(()=>{}),400);
setInterval(refresh,400);
loadLogo();refresh();
</script>
</body>
</html>
)RACE";

// ============================================================
// 12. API
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

// Pantalla de carrera: misma logica, PID y parametros del Test Suite.
void handleRace() {
  server.send_P(200, "text/html; charset=utf-8", RACE_HTML);
}

// Reutiliza el logo exacto ya embebido en INDEX_HTML, sin duplicarlo en flash.
// Se envia su base64; la pagina de carrera lo convierte a imagen local.
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
  json += "\"basePwm\":" + String(cfg.basePwm) + ",";
  json += "\"maxCorrection\":" + String(cfg.maxCorrection) + ",";
  json += "\"frontWallAdc\":" + String(cfg.frontWallAdc) + ",";
  json += "\"frontConfirmAdc\":" + String(cfg.frontConfirmAdc) + ",";
  json += "\"approachMinPwm\":" + String(cfg.approachMinPwm) + ",";
  json += "\"rightOpenAdc\":" + String(cfg.rightOpenAdc) + ",";
  json += "\"leftOpenAdc\":" + String(cfg.leftOpenAdc) + ",";
  json += "\"openingWaitMs\":" + String(cfg.openingWaitMs) + ",";
  json += "\"openingAdvanceCm\":" + String(cfg.openingAdvanceCm, 1) + ",";
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
  frontLeftTurns = 0;

  if (mode == "TEST") {
    activeMode = MODE_TEST;
  } else if (mode == "WALL") {
    activeMode = MODE_WALL;
  } else if (mode == "MAZE") {
    activeMode = MODE_MAZE;
  } else if (mode == "ENCODER") {
    activeMode = MODE_ENCODER;
    encoderTestAction = ENC_TEST_NONE;
    encoderTestTarget = 0;
    encoderTestRequestedCm = 0.0f;
    captureMoveStart();
  } else {
    server.send(400, "text/plain", "mode debe ser TEST, WALL, MAZE o ENCODER");
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

    if (activeMode == MODE_MAZE) {
      frontWallStableCount = 0;
      rightOpenStableCount = 0;
      rightWallStableCount = 0;
      rightOpeningArmed = false;
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

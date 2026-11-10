// AUS_KIM V3 - Pruebas de motores y sensores por Wi-Fi (ESP32-S3-Zero + TB6612FNG)
//
// Red:   SSID AUS_KIM_V3   clave AUSKIM2026   panel http://192.168.4.1
// Placa en Arduino IDE: "ESP32S3 Dev Module", "USB CDC On Boot: Enabled".
// Compatible con arduino-esp32 core 2.x y 3.x. Archivos de la carpeta: este .ino (logica) y web_calib.h (pagina web).
// Pines y componentes: docs/AUS_KIM_V3_ESP32-S3-Zero.md
//
// Pestana MOTORES: medir cuantas cuentas de encoder son 1 mm de avance y cuantas hacen
// falta para girar un angulo (con la separacion de ruedas), y probar avances y giros
// controlados por encoders. Los N20 son de 6 V: el PWM se compensa con la tension de la bateria.
// Pestana SENSORES: lecturas de los 4 Sharp (2 laterales + 2 frontales) y del QRE, umbral de pared
// por sensor, deteccion de piso blanco/negro, estado del boton START y registro de lecturas por distancia.
// Los valores calibrados se guardan en la memoria del robot.
//
// Seguridad:
//   - El TB6612FNG arranca deshabilitado (STBY en bajo) y se habilita al final de setup().
//   - Cada movimiento tiene limite de tiempo.
//   - STOP en la pagina, o pulsar START durante un movimiento, lo detiene.

#include <Arduino.h>
#include <cmath>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "web_calib.h"   // pagina web (misma carpeta)

// GPIO43 (STBY) y GPIO44 (BIN2) son tambien TX/RX de la UART0. Si Serial usara la UART0
// tomaria esos pines y el driver quedaria fuera de control. Por eso Serial solo se usa cuando es
// USB (CDC), que no toca esos pines ("USB CDC On Boot: Enabled").
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#define DBG_BEGIN(b) Serial.begin(b)
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#else
#define DBG_BEGIN(b)
#define DBG(x)
#define DBGLN(x)
#endif


// ---------------------------------------------------------------- Pines V3
constexpr uint8_t PIN_SH_I   = 1;   // Sharp lateral izquierdo  (ADC1)
constexpr uint8_t PIN_SH_FI  = 2;   // Sharp frontal izquierdo  (ADC1)
constexpr uint8_t PIN_VBAT   = 3;   // divisor de bateria 100k/47k (ADC1)
constexpr uint8_t PIN_SH_FD  = 4;   // Sharp frontal derecho    (ADC1)
constexpr uint8_t PIN_SH_D   = 5;   // Sharp lateral derecho    (ADC1)
constexpr uint8_t PIN_QRE    = 6;   // QRE1113 de piso          (ADC1)
constexpr uint8_t PIN_ENC_IA = 7;
constexpr uint8_t PIN_ENC_IB = 8;
constexpr uint8_t PIN_ENC_DA = 9;
constexpr uint8_t PIN_ENC_DB = 10;
constexpr uint8_t PIN_AIN1   = 11;  // TB6612 AIN1 (motor derecho)
constexpr uint8_t PIN_AIN2   = 12;  // TB6612 AIN2 (motor derecho)
constexpr uint8_t PIN_BIN1   = 13;  // TB6612 BIN1 (motor izquierdo)
constexpr uint8_t PIN_PWMA   = 14;  // TB6612 PWMA (pad suelto del S3-Zero)
constexpr uint8_t PIN_PWMB   = 15;  // TB6612 PWMB (pad suelto del S3-Zero)
constexpr uint8_t PIN_START  = 16;  // pulsador, activo en LOW (pad suelto del S3-Zero)
constexpr uint8_t PIN_STBY   = 43;  // TB6612 STBY (TX); pull-down de 100k a GND en la placa
constexpr uint8_t PIN_BIN2   = 44;  // TB6612 BIN2 (RX)

constexpr float VBAT_DIV = 147.0f / 47.0f;  // (100k + 47k) / 47k

// ------------------------------------------------------------------- Wi-Fi
const char *WIFI_SSID = "AUS_KIM_V3";
const char *WIFI_PASS = "AUSKIM2026";
constexpr bool LOWER_WIFI_TX_POWER = false;

WebServer server(80);
Preferences prefs;

float vbat = 0.0f;  // tension de bateria en V (0 = sin lectura)

// ----------------------------------------------------------- Configuracion
struct CalConfig {
  float cpm = 0.0f;          // cuentas de cuadratura por mm de avance (0 = sin calibrar)
  float trackMm = 79.0f;     // separacion entre ruedas (mm)
  float wheelDia = 0.0f;     // diametro de rueda informado en calibracion (mm, 0 = desconocido)
  float minPwm = 70;         // PWM minimo para vencer la friccion
  float maxPwm = 140;        // PWM maximo de crucero
  float decelMm = 40;        // distancia (mm de rueda) en la que baja de maxPwm a minPwm
  float rampStep = 4;        // aumento maximo de PWM cada 5 ms
  float tolMm = 2.0f;        // tolerancia de llegada en avance (mm)
  float tolDeg = 1.0f;       // tolerancia de llegada en giro (grados)
  float kHead = 0.8f;        // correccion de recta/equilibrio (PWM por cuenta de diferencia)
  int invML = 0, invMR = 0;  // invertir sentido de motor izquierdo / derecho
  int invEL = 0, invER = 0;  // invertir signo del encoder izquierdo / derecho
  int thrI = 1500, thrFI = 1500, thrFD = 1500, thrD = 1500;  // umbral ADC de pared por Sharp (SIN CALIBRAR)
  int qreBlack = -1, qreWhite = -1;           // ADC del QRE sobre negro / blanco (-1 = sin capturar)
  float vMotor = 6.0f;       // tension nominal de los N20 (V)
  int vComp = 1;             // 1 = compensar el PWM con la tension de bateria
  float vbatK = 1.0f;        // factor de calibracion de la lectura de bateria
} cfg;

// ------------------------------------------------------ Tipos (antes de las funciones)
// Arduino genera prototipos antes de la primera funcion: los tipos deben estar definidos antes.
struct Encoder {
  volatile int32_t count;
  volatile uint8_t state;
  uint8_t pinA, pinB;
};

enum MotionMode : uint8_t { M_IDLE, M_MOVE, M_TURN, M_TIMED, M_SETTLE };

struct Motion {
  MotionMode mode = M_IDLE;
  uint8_t kind = 0;          // 0 avance, 1 giro, 2 motor
  float cmd = 0;             // mm, grados o PWM pedido
  float targetCounts = 0;    // progreso objetivo en cuentas, con signo
  float cmdMag = 0;          // PWM actual con rampa
  bool timedOut = false;
  char side = 'B';
  uint32_t startMs = 0, endMs = 0, settleAt = 0;
} mo;

struct LastResult {
  bool valid = false;
  uint8_t kind = 0;
  char side = 'B';
  float cmd = 0, progress = 0;
  int32_t cL = 0, cR = 0;
  uint32_t ms = 0;
  bool timeout = false;
} last;

void loadConfig() {
  prefs.begin("auskim_v3", false);
  cfg.cpm      = prefs.getFloat("cpm", cfg.cpm);
  cfg.trackMm  = prefs.getFloat("track", cfg.trackMm);
  cfg.wheelDia = prefs.getFloat("wdia", cfg.wheelDia);
  cfg.minPwm   = prefs.getFloat("minPwm", cfg.minPwm);
  cfg.maxPwm   = prefs.getFloat("maxPwm", cfg.maxPwm);
  cfg.decelMm  = prefs.getFloat("decel", cfg.decelMm);
  cfg.rampStep = prefs.getFloat("ramp", cfg.rampStep);
  cfg.tolMm    = prefs.getFloat("tolMm", cfg.tolMm);
  cfg.tolDeg   = prefs.getFloat("tolDeg", cfg.tolDeg);
  cfg.kHead    = prefs.getFloat("kHead", cfg.kHead);
  cfg.invML    = prefs.getInt("invML", cfg.invML);
  cfg.invMR    = prefs.getInt("invMR", cfg.invMR);
  cfg.invEL    = prefs.getInt("invEL", cfg.invEL);
  cfg.invER    = prefs.getInt("invER", cfg.invER);
  cfg.thrI     = prefs.getInt("thrI", cfg.thrI);
  cfg.thrFI    = prefs.getInt("thrFI", cfg.thrFI);
  cfg.thrFD    = prefs.getInt("thrFD", cfg.thrFD);
  cfg.thrD     = prefs.getInt("thrD", cfg.thrD);
  cfg.qreBlack = prefs.getInt("qreBlack", cfg.qreBlack);
  cfg.qreWhite = prefs.getInt("qreWhite", cfg.qreWhite);
  cfg.vMotor   = prefs.getFloat("vMotor", cfg.vMotor);
  cfg.vComp    = prefs.getInt("vComp", cfg.vComp);
  cfg.vbatK    = prefs.getFloat("vbatK", cfg.vbatK);
}

void saveConfig() {
  prefs.putFloat("cpm", cfg.cpm);
  prefs.putFloat("track", cfg.trackMm);
  prefs.putFloat("wdia", cfg.wheelDia);
  prefs.putFloat("minPwm", cfg.minPwm);
  prefs.putFloat("maxPwm", cfg.maxPwm);
  prefs.putFloat("decel", cfg.decelMm);
  prefs.putFloat("ramp", cfg.rampStep);
  prefs.putFloat("tolMm", cfg.tolMm);
  prefs.putFloat("tolDeg", cfg.tolDeg);
  prefs.putFloat("kHead", cfg.kHead);
  prefs.putInt("invML", cfg.invML);
  prefs.putInt("invMR", cfg.invMR);
  prefs.putInt("invEL", cfg.invEL);
  prefs.putInt("invER", cfg.invER);
  prefs.putInt("thrI", cfg.thrI);
  prefs.putInt("thrFI", cfg.thrFI);
  prefs.putInt("thrFD", cfg.thrFD);
  prefs.putInt("thrD", cfg.thrD);
  prefs.putInt("qreBlack", cfg.qreBlack);
  prefs.putInt("qreWhite", cfg.qreWhite);
  prefs.putFloat("vMotor", cfg.vMotor);
  prefs.putInt("vComp", cfg.vComp);
  prefs.putFloat("vbatK", cfg.vbatK);
}

// ------------------------------------------------------------------ Encoders

Encoder encL = {0, 0, PIN_ENC_IA, PIN_ENC_IB};
Encoder encR = {0, 0, PIN_ENC_DA, PIN_ENC_DB};

// Decodificacion en cuadratura x4: indice = (estadoPrevio << 2) | estadoNuevo
static const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR isrEncoder(void *arg) {
  Encoder *e = static_cast<Encoder *>(arg);
  uint8_t s = (gpio_get_level((gpio_num_t)e->pinA) << 1) | gpio_get_level((gpio_num_t)e->pinB);
  e->count += QEM[(e->state << 2) | s];
  e->state = s;
}

void setupEncoder(Encoder &e) {
  pinMode(e.pinA, INPUT_PULLUP);
  pinMode(e.pinB, INPUT_PULLUP);
  e.state = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
  attachInterruptArg(e.pinA, isrEncoder, &e, CHANGE);
  attachInterruptArg(e.pinB, isrEncoder, &e, CHANGE);
}

int32_t countL() { return cfg.invEL ? -encL.count : encL.count; }
int32_t countR() { return cfg.invER ? -encR.count : encR.count; }

void zeroEncoders() {
  noInterrupts();
  encL.count = 0;
  encR.count = 0;
  interrupts();
}

// ------------------------------------------------------------------- Motores
// TB6612FNG: PWMA/PWMB reciben el PWM (modo avance/freno, decaimiento lento, mas lineal);
// IN1/IN2 fijan el sentido; STBY en alto habilita el driver. Los N20 son de 6 V.
constexpr uint32_t PWM_HZ = 31250;  // igual que los seguidores con TB6612FNG

void pwmWrite(bool right, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(right ? PIN_PWMA : PIN_PWMB, duty);
#else
  ledcWrite(right ? 0 : 1, duty);
#endif
}

void setDirPins(bool in1, bool in2) {
  digitalWrite(PIN_AIN1, in1); digitalWrite(PIN_AIN2, in2);
  digitalWrite(PIN_BIN1, in1); digitalWrite(PIN_BIN2, in2);
}

void setupMotors() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);  // driver deshabilitado hasta el final de setup()
  const uint8_t dirPins[4] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};
  for (uint8_t i = 0; i < 4; i++) pinMode(dirPins[i], OUTPUT);
  setDirPins(false, false);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWMA, PWM_HZ, 8);
  ledcAttach(PIN_PWMB, PWM_HZ, 8);
#else
  ledcSetup(0, PWM_HZ, 8);
  ledcAttachPin(PIN_PWMA, 0);
  ledcSetup(1, PWM_HZ, 8);
  ledcAttachPin(PIN_PWMB, 1);
#endif
  pwmWrite(true, 0);
  pwmWrite(false, 0);
}

void enableDriver(bool on) { digitalWrite(PIN_STBY, on ? HIGH : LOW); }

// El PWM pedido equivale a una fraccion de vMotor (6 V). Con la bateria llena se reduce el
// ciclo de trabajo para que los motores no reciban mas de vMotor y la velocidad sea la misma
// con la bateria llena o gastada. Sin lectura de bateria (menos de 4,5 V) no se compensa.
int compDuty(int mag) {
  if (!cfg.vComp || vbat < 4.5f || vbat <= cfg.vMotor) return mag;
  return (int)(mag * cfg.vMotor / vbat + 0.5f);
}

// pwm firmado -255..255. Positivo: IN1 alto, IN2 bajo. Negativo: al reves. Cero: parada libre.
void setMotor(bool right, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (right ? cfg.invMR : cfg.invML) pwm = -pwm;
  digitalWrite(right ? PIN_AIN1 : PIN_BIN1, pwm > 0);
  digitalWrite(right ? PIN_AIN2 : PIN_BIN2, pwm < 0);
  pwmWrite(right, compDuty(abs(pwm)));
}

void coastMotors() {
  setDirPins(false, false);
  pwmWrite(true, 0);
  pwmWrite(false, 0);
}

void brakeMotors() {  // IN1 = IN2 = alto con PWM alto: freno en corto
  setDirPins(true, true);
  pwmWrite(true, 255);
  pwmWrite(false, 255);
}

// ------------------------------------------------------------------ Sensores
uint16_t readMedian5(uint8_t pin) {
  uint16_t v[5];
  for (auto &x : v) x = analogRead(pin);
  for (int i = 1; i < 5; i++) {
    uint16_t k = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[2];
}

// Promedio de 16 lecturas (cada una ya es una mediana de 5): para capturar umbrales y valores del QRE.
int avgRead(uint8_t pin) {
  long a = 0;
  for (int i = 0; i < 16; i++) a += readMedian5(pin);
  return (int)(a / 16);
}

// Estado de pared con histeresis (~7 %): el Sharp da mas valor cuanto mas cerca esta la pared.
bool wallI = false, wallFI = false, wallFD = false, wallD = false;
bool wallState(bool prev, int v, int thr) {
  int off = thr - thr / 15;
  return prev ? (v >= off) : (v >= thr);
}

// Piso blanco: "true"; piso negro: "false"; sin los dos valores capturados: "null".
const char *qreWhiteJson(int v) {
  if (cfg.qreBlack < 0 || cfg.qreWhite < 0) return "null";
  return abs(v - cfg.qreWhite) < abs(v - cfg.qreBlack) ? "true" : "false";
}

// Tension de bateria: divisor 100k/47k en PIN_VBAT (ADC1). Promedio de 8 lecturas en mV.
float readVbatNow() {
  long a = 0;
  for (int i = 0; i < 8; i++) a += analogReadMilliVolts(PIN_VBAT);
  return (a / 8.0f) / 1000.0f * VBAT_DIV * cfg.vbatK;
}

void updateVbat() {
  float v = readVbatNow();
  vbat = (vbat <= 0.0f || fabsf(v - vbat) > 1.0f) ? v : vbat * 0.9f + v * 0.1f;
}

bool startPressedStable() {
  static uint8_t cnt = 0;
  if (digitalRead(PIN_START) == LOW) { if (cnt < 255) cnt++; } else cnt = 0;
  return cnt >= 4;  // ~20 ms
}

// -------------------------------------------------------------- Movimientos

const char *modeName() {
  switch (mo.mode) {
    case M_MOVE: return "AVANZANDO";
    case M_TURN: return "GIRANDO";
    case M_TIMED: return "MOTOR";
    case M_SETTLE: return "FRENANDO";
    default: return "LISTO";
  }
}

// Progreso en cuentas: avance = promedio de ruedas; giro = (der - izq) / 2.
float progressNow(bool turn) {
  float l = countL(), r = countR();
  return turn ? (r - l) / 2.0f : (l + r) / 2.0f;
}

void abortAll() {
  brakeMotors();
  mo.mode = M_IDLE;
}

const char *beginMotion(bool turn, float value) {
  if (mo.mode != M_IDLE) return "El robot esta ocupado";
  if (cfg.cpm <= 0) return "Primero calibrar cuentas/mm (paso 2)";
  if (value == 0) return "El valor no puede ser cero";
  float arcMm = turn ? cfg.trackMm * PI * value / 360.0f : value;
  zeroEncoders();
  mo.kind = turn ? 1 : 0;
  mo.cmd = value;
  mo.targetCounts = arcMm * cfg.cpm;
  mo.cmdMag = cfg.minPwm;
  mo.timedOut = false;
  mo.startMs = millis();
  mo.mode = turn ? M_TURN : M_MOVE;
  return nullptr;
}

const char *beginTimed(char side, int pwm) {
  if (mo.mode != M_IDLE) return "El robot esta ocupado";
  zeroEncoders();
  mo.kind = 2;
  mo.side = side;
  mo.cmd = pwm;
  mo.timedOut = false;
  mo.startMs = millis();
  mo.endMs = mo.startMs + 800;
  if (side == 'L' || side == 'B') setMotor(false, pwm);
  if (side == 'R' || side == 'B') setMotor(true, pwm);
  mo.mode = M_TIMED;
  return nullptr;
}

void finishMotion(uint32_t now) {
  brakeMotors();
  mo.endMs = now;
  mo.mode = M_SETTLE;
  mo.settleAt = now + 300;  // esperar a que se asienten los encoders antes de medir
}

void updateMotion() {
  uint32_t now = millis();

  if (mo.mode == M_IDLE) return;
  if (mo.mode == M_TIMED) {
    if (now >= mo.endMs) finishMotion(now);
    return;
  }
  if (mo.mode == M_SETTLE) {
    if (now >= mo.settleAt) {
      last.valid = true;
      last.kind = mo.kind;
      last.side = mo.side;
      last.cmd = mo.cmd;
      last.progress = mo.kind == 2 ? 0 : progressNow(mo.kind == 1);
      last.cL = countL();
      last.cR = countR();
      last.ms = mo.endMs - mo.startMs;
      last.timeout = mo.timedOut;
      mo.mode = M_IDLE;
    }
    return;
  }

  // M_MOVE o M_TURN
  bool turn = (mo.mode == M_TURN);
  float err = mo.targetCounts - progressNow(turn);
  float dir = mo.targetCounts >= 0 ? 1.0f : -1.0f;
  float errMm = fabsf(err) / cfg.cpm;
  float tolMm = turn ? cfg.trackMm * PI * cfg.tolDeg / 360.0f : cfg.tolMm;
  bool reached = (errMm <= tolMm) || (err * dir <= 0);
  bool timeout = (now - mo.startMs) > (turn ? 6000UL : 8000UL);
  if (reached || timeout) {
    mo.timedOut = timeout && !reached;
    finishMotion(now);
    return;
  }

  // Velocidad: baja de maxPwm a minPwm en la zona de frenado y sube con rampa.
  float frac = constrain(errMm / max(cfg.decelMm, 1.0f), 0.0f, 1.0f);
  float goal = cfg.minPwm + (cfg.maxPwm - cfg.minPwm) * frac;
  mo.cmdMag = min(goal, mo.cmdMag + cfg.rampStep);

  float l = countL(), r = countR();
  float magL, magR;
  if (!turn) {
    // Recta: si una rueda va adelantada, se frena y se acelera la otra.
    float head = l - r;
    magL = mo.cmdMag - dir * cfg.kHead * head;
    magR = mo.cmdMag + dir * cfg.kHead * head;
    setMotor(false, (int)(dir * constrain(magL, 0.0f, 255.0f)));
    setMotor(true, (int)(dir * constrain(magR, 0.0f, 255.0f)));
  } else {
    // Giro sobre el eje: las ruedas deben sumar cero cuentas.
    float drift = l + r;
    magL = mo.cmdMag + dir * cfg.kHead * drift;
    magR = mo.cmdMag - dir * cfg.kHead * drift;
    setMotor(false, (int)(-dir * constrain(magL, 0.0f, 255.0f)));
    setMotor(true, (int)(dir * constrain(magR, 0.0f, 255.0f)));
  }
}

// --------------------------------------------------------------------- HTTP
void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}
void sendOk(const String &text) { sendJson(200, "{\"ok\":\"" + text + "\"}"); }
void sendErr(const String &text) { sendJson(400, "{\"error\":\"" + text + "\"}"); }

String f(float v, int d) { return std::isfinite(v) ? String(v, d) : String("0"); }

String cfgJson() {
  String s = "{";
  s += "\"cpm\":" + f(cfg.cpm, 4);
  s += ",\"trackMm\":" + f(cfg.trackMm, 2);
  s += ",\"wheelDia\":" + f(cfg.wheelDia, 2);
  s += ",\"minPwm\":" + f(cfg.minPwm, 0);
  s += ",\"maxPwm\":" + f(cfg.maxPwm, 0);
  s += ",\"decelMm\":" + f(cfg.decelMm, 1);
  s += ",\"rampStep\":" + f(cfg.rampStep, 1);
  s += ",\"tolMm\":" + f(cfg.tolMm, 2);
  s += ",\"tolDeg\":" + f(cfg.tolDeg, 2);
  s += ",\"kHead\":" + f(cfg.kHead, 2);
  s += ",\"invML\":" + String(cfg.invML);
  s += ",\"invMR\":" + String(cfg.invMR);
  s += ",\"invEL\":" + String(cfg.invEL);
  s += ",\"invER\":" + String(cfg.invER);
  s += ",\"thrI\":" + String(cfg.thrI);
  s += ",\"thrFI\":" + String(cfg.thrFI);
  s += ",\"thrFD\":" + String(cfg.thrFD);
  s += ",\"thrD\":" + String(cfg.thrD);
  s += ",\"qreBlack\":" + String(cfg.qreBlack);
  s += ",\"qreWhite\":" + String(cfg.qreWhite);
  s += ",\"vMotor\":" + f(cfg.vMotor, 2);
  s += ",\"vComp\":" + String(cfg.vComp);
  s += ",\"vbatK\":" + f(cfg.vbatK, 4);
  s += "}";
  return s;
}

// Velocidad de cada rueda (cuentas por segundo), calculada cada 50 ms en loop().
float speedL = 0, speedR = 0;

void handleStatus() {
  int vI = readMedian5(PIN_SH_I), vFI = readMedian5(PIN_SH_FI);
  int vFD = readMedian5(PIN_SH_FD), vD = readMedian5(PIN_SH_D);
  int vQ = readMedian5(PIN_QRE);
  wallI = wallState(wallI, vI, cfg.thrI);
  wallFI = wallState(wallFI, vFI, cfg.thrFI);
  wallFD = wallState(wallFD, vFD, cfg.thrFD);
  wallD = wallState(wallD, vD, cfg.thrD);

  String s = "{";
  s += "\"mode\":\"" + String(modeName()) + "\"";
  s += ",\"cL\":" + String(countL());
  s += ",\"cR\":" + String(countR());
  s += ",\"vL\":" + f(speedL, 0);
  s += ",\"vR\":" + f(speedR, 0);
  s += ",\"shI\":" + String(vI);
  s += ",\"shFI\":" + String(vFI);
  s += ",\"shFD\":" + String(vFD);
  s += ",\"shD\":" + String(vD);
  s += ",\"qre\":" + String(vQ);
  s += ",\"wI\":" + String(wallI ? "true" : "false");
  s += ",\"wFI\":" + String(wallFI ? "true" : "false");
  s += ",\"wFD\":" + String(wallFD ? "true" : "false");
  s += ",\"wD\":" + String(wallD ? "true" : "false");
  s += ",\"wF\":" + String(wallFI && wallFD ? "true" : "false");  // pared al frente: las dos frontales
  s += ",\"vbat\":" + f(vbat, 2);
  s += ",\"qreWhite\":" + String(qreWhiteJson(vQ));
  s += ",\"start\":" + String(digitalRead(PIN_START) == LOW ? "true" : "false");
  s += ",\"cfg\":" + cfgJson();
  s += ",\"last\":{";
  s += "\"valid\":" + String(last.valid ? "true" : "false");
  s += ",\"kind\":" + String(last.kind);
  s += ",\"side\":\"" + String(last.side) + "\"";
  s += ",\"cmd\":" + f(last.cmd, 1);
  s += ",\"progress\":" + f(last.progress, 1);
  s += ",\"cL\":" + String(last.cL);
  s += ",\"cR\":" + String(last.cR);
  s += ",\"ms\":" + String(last.ms);
  s += ",\"timeout\":" + String(last.timeout ? "true" : "false");
  s += "}}";
  sendJson(200, s);
}

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleStop() {
  abortAll();
  sendOk("Detenido");
}

void handleZero() {
  zeroEncoders();
  sendOk("Encoders en cero");
}

void handleMotor() {
  String m = server.arg("m");
  int pwm = server.arg("pwm").toInt();
  if (m != "L" && m != "R" && m != "B") { sendErr("Motor invalido"); return; }
  if (pwm == 0 || abs(pwm) > 255) { sendErr("PWM invalido (1..255 con signo)"); return; }
  const char *e = beginTimed(m.charAt(0), pwm);
  if (e) sendErr(e); else sendOk("Motor " + m + " 0,8 s");
}

void handleMove() {
  if (!server.hasArg("mm")) { sendErr("Falta mm"); return; }
  float mm = server.arg("mm").toFloat();
  if (fabsf(mm) > 2000) { sendErr("Maximo 2000 mm"); return; }
  const char *e = beginMotion(false, mm);
  if (e) sendErr(e); else sendOk("Avanzando " + f(mm, 0) + " mm");
}

void handleTurn() {
  if (!server.hasArg("deg")) { sendErr("Falta deg"); return; }
  float deg = server.arg("deg").toFloat();
  if (fabsf(deg) > 720) { sendErr("Maximo 720 grados"); return; }
  const char *e = beginMotion(true, deg);
  if (e) sendErr(e); else sendOk("Girando " + f(deg, 0) + " grados");
}

void handleCalDist() {
  float mm = server.arg("mm").toFloat();
  float avg = (abs(countL()) + abs(countR())) / 2.0f;
  if (mm <= 0) { sendErr("Distancia invalida"); return; }
  if (avg < 20) { sendErr("Casi no hay cuentas: pon en cero y empuja el robot a mano"); return; }
  cfg.cpm = avg / mm;
  saveConfig();
  sendOk("cuentas/mm = " + f(cfg.cpm, 3));
}

void handleCalWheel() {
  float dia = server.arg("dia").toFloat();
  float perRev = max(abs(countL()), abs(countR()));
  if (dia <= 0) { sendErr("Diametro invalido"); return; }
  if (perRev < 20) { sendErr("Casi no hay cuentas: pon en cero y gira una rueda una vuelta"); return; }
  cfg.cpm = perRev / (PI * dia);
  cfg.wheelDia = dia;
  saveConfig();
  sendOk("cuentas/vuelta = " + String((int)perRev) + ", cuentas/mm = " + f(cfg.cpm, 3));
}

void handleCalTrack() {
  float deg = server.arg("deg").toFloat();
  if (cfg.cpm <= 0) { sendErr("Primero calibrar cuentas/mm"); return; }
  if (deg <= 0) { sendErr("Angulo invalido"); return; }
  float arcMm = fabsf((float)(countR() - countL())) / 2.0f / cfg.cpm;
  if (arcMm < 5) { sendErr("Casi no hay giro: pon en cero y gira el robot a mano"); return; }
  cfg.trackMm = arcMm * 360.0f / (PI * deg);
  saveConfig();
  sendOk("separacion = " + f(cfg.trackMm, 1) + " mm");
}

void handleAdjustMove() {
  float real = server.arg("real").toFloat();
  if (!last.valid || last.kind != 0) { sendErr("Primero hacer una prueba de avance"); return; }
  if (real <= 0 || fabsf(last.progress) < 20) { sendErr("Medida invalida"); return; }
  cfg.cpm = fabsf(last.progress) / real;
  saveConfig();
  sendOk("cuentas/mm = " + f(cfg.cpm, 3));
}

void handleAdjustTurn() {
  float real = fabsf(server.arg("real").toFloat());
  if (!last.valid || last.kind != 1) { sendErr("Primero hacer una prueba de giro"); return; }
  if (real < 1) { sendErr("Angulo invalido"); return; }
  cfg.trackMm = cfg.trackMm * fabsf(last.cmd) / real;
  saveConfig();
  sendOk("separacion efectiva = " + f(cfg.trackMm, 1) + " mm");
}

void handleConfig() {
  if (mo.mode != M_IDLE) { sendErr("Esperar a que termine el movimiento"); return; }
  if (server.hasArg("minPwm"))   cfg.minPwm   = constrain(server.arg("minPwm").toFloat(), 0.0f, 255.0f);
  if (server.hasArg("maxPwm"))   cfg.maxPwm   = constrain(server.arg("maxPwm").toFloat(), 0.0f, 255.0f);
  if (server.hasArg("decelMm"))  cfg.decelMm  = max(server.arg("decelMm").toFloat(), 1.0f);
  if (server.hasArg("rampStep")) cfg.rampStep = max(server.arg("rampStep").toFloat(), 0.5f);
  if (server.hasArg("tolMm"))    cfg.tolMm    = max(server.arg("tolMm").toFloat(), 0.1f);
  if (server.hasArg("tolDeg"))   cfg.tolDeg   = max(server.arg("tolDeg").toFloat(), 0.1f);
  if (server.hasArg("kHead"))    cfg.kHead    = constrain(server.arg("kHead").toFloat(), 0.0f, 10.0f);
  if (server.hasArg("invML"))    cfg.invML    = server.arg("invML").toInt() ? 1 : 0;
  if (server.hasArg("invMR"))    cfg.invMR    = server.arg("invMR").toInt() ? 1 : 0;
  if (server.hasArg("invEL"))    cfg.invEL    = server.arg("invEL").toInt() ? 1 : 0;
  if (server.hasArg("invER"))    cfg.invER    = server.arg("invER").toInt() ? 1 : 0;
  if (server.hasArg("vMotor"))   cfg.vMotor   = constrain(server.arg("vMotor").toFloat(), 3.0f, 12.0f);
  if (server.hasArg("vComp"))    cfg.vComp    = server.arg("vComp").toInt() ? 1 : 0;
  if (cfg.maxPwm < cfg.minPwm) cfg.maxPwm = cfg.minPwm;
  zeroEncoders();  // un cambio de signo de encoder deja las cuentas viejas sin sentido
  saveConfig();
  sendOk("Guardado");
}

// Umbral de pared de un Sharp (s = I, FI, FD o D): valor escrito (v=) o el valor actual promediado (cur=1).
void handleThr() {
  String s = server.arg("s");
  uint8_t pin;
  int *thr;
  if (s == "I") { pin = PIN_SH_I; thr = &cfg.thrI; }
  else if (s == "FI") { pin = PIN_SH_FI; thr = &cfg.thrFI; }
  else if (s == "FD") { pin = PIN_SH_FD; thr = &cfg.thrFD; }
  else if (s == "D") { pin = PIN_SH_D; thr = &cfg.thrD; }
  else { sendErr("Sensor invalido (I, FI, FD o D)"); return; }
  int v;
  if (server.hasArg("cur")) v = avgRead(pin);
  else if (server.hasArg("v")) v = server.arg("v").toInt();
  else { sendErr("Falta v o cur"); return; }
  if (v < 1 || v > 4095) { sendErr("Valor fuera de rango (1 a 4095)"); return; }
  *thr = v;
  saveConfig();
  sendOk("Umbral " + s + " = " + String(v));
}

// Calibrar la lectura de bateria con un voltimetro: real=<V medidos>; clear=1 vuelve al factor 1.
void handleVbat() {
  if (server.hasArg("clear")) {
    cfg.vbatK = 1.0f;
    vbat = 0.0f;
    saveConfig();
    sendOk("Factor de bateria = 1");
    return;
  }
  float real = server.arg("real").toFloat();
  float raw = readVbatNow() / cfg.vbatK;
  if (real < 4.5f || real > 9.0f) { sendErr("Tension real fuera de rango (4,5 a 9 V)"); return; }
  if (raw < 1.0f) { sendErr("Sin lectura de bateria: revisar el divisor"); return; }
  cfg.vbatK = constrain(real / raw, 0.5f, 2.0f);
  vbat = 0.0f;
  saveConfig();
  sendOk("Factor de bateria = " + f(cfg.vbatK, 4));
}

// Captura del QRE sobre piso negro o blanco (c=black / c=white); c=clear borra los dos.
void handleCalQre() {
  String c = server.arg("c");
  if (c == "clear") {
    cfg.qreBlack = -1;
    cfg.qreWhite = -1;
    saveConfig();
    sendOk("Valores del QRE borrados");
    return;
  }
  if (c != "black" && c != "white") { sendErr("Usar c=black, c=white o c=clear"); return; }
  int v = avgRead(PIN_QRE);
  if (c == "black") cfg.qreBlack = v; else cfg.qreWhite = v;
  saveConfig();
  String m = "QRE " + String(c == "black" ? "negro" : "blanco") + " = " + String(v);
  if (cfg.qreBlack >= 0 && cfg.qreWhite >= 0 && abs(cfg.qreBlack - cfg.qreWhite) < 150)
    m += " (negro y blanco muy parecidos: revisar la altura del sensor)";
  sendOk(m);
}

// ------------------------------------------------------------------- Setup
void setup() {
  // Lo primero: pines del driver en estado seguro (STBY en bajo, entradas en bajo, PWM en cero).
  setupMotors();
  coastMotors();

  DBG_BEGIN(115200);
  pinMode(PIN_START, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SH_I, ADC_11db);
  analogSetPinAttenuation(PIN_SH_FI, ADC_11db);
  analogSetPinAttenuation(PIN_SH_FD, ADC_11db);
  analogSetPinAttenuation(PIN_SH_D, ADC_11db);
  analogSetPinAttenuation(PIN_QRE, ADC_11db);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  loadConfig();
  setupEncoder(encL);
  setupEncoder(encR);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  if (LOWER_WIFI_TX_POWER) WiFi.setTxPower(WIFI_POWER_8_5dBm);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.on("/api/zero", HTTP_GET, handleZero);
  server.on("/api/motor", HTTP_GET, handleMotor);
  server.on("/api/move", HTTP_GET, handleMove);
  server.on("/api/turn", HTTP_GET, handleTurn);
  server.on("/api/cal_dist", HTTP_GET, handleCalDist);
  server.on("/api/cal_wheel", HTTP_GET, handleCalWheel);
  server.on("/api/cal_track", HTTP_GET, handleCalTrack);
  server.on("/api/adjust_move", HTTP_GET, handleAdjustMove);
  server.on("/api/adjust_turn", HTTP_GET, handleAdjustTurn);
  server.on("/api/config", HTTP_GET, handleConfig);
  server.on("/api/thr", HTTP_GET, handleThr);
  server.on("/api/cal_qre", HTTP_GET, handleCalQre);
  server.on("/api/vbat", HTTP_GET, handleVbat);
  server.begin();

  updateVbat();
  enableDriver(true);  // recien ahora se habilita el TB6612FNG (STBY en alto)

  DBGLN(F("AUS_KIM V3 pruebas de motores y sensores listas"));
  DBG(F("Wi-Fi: "));
  DBG(WIFI_SSID);
  DBG(F("  panel: http://"));
  DBGLN(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick >= 5) {
    lastTick = now;
    bool startHeld = startPressedStable();
    if (startHeld && mo.mode != M_IDLE) abortAll();
    else updateMotion();
  }

  static uint32_t lastBat = 0;
  if (now - lastBat >= 200) {
    lastBat = now;
    updateVbat();
  }

  static uint32_t lastSpeed = 0;
  static int32_t prevL = 0, prevR = 0;
  if (now - lastSpeed >= 50) {
    int32_t l = countL(), r = countR();
    float dt = (now - lastSpeed) / 1000.0f;
    speedL = (l - prevL) / dt;
    speedR = (r - prevR) / dt;
    prevL = l;
    prevR = r;
    lastSpeed = now;
  }
}

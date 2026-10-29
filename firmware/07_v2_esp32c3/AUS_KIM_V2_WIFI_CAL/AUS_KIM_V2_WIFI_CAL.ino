// AUS_KIM V2 - Calibracion de movimiento por Wi-Fi (ESP32-C3 Super Mini)
//
// Interfaz web para probar encoders y medir cuanto avanzar / cuanto girar:
//   - Avanzar X mm y girar X grados con lazo cerrado por encoders.
//   - Muestra cuentas objetivo vs reales y el avance/giro estimado.
//   - Permite corregir "cuentas por mm" y "separacion de ruedas" con lo medido en pista.
//   - Guarda la calibracion en memoria flash (Preferences / NVS).
//
// Red:   SSID AUS_KIM_V2   clave AUSKIM2026   panel http://192.168.4.1
//
// Arduino IDE: placa "ESP32C3 Dev Module", "USB CDC On Boot: Enabled".
// Compatible con arduino-esp32 core 2.x y 3.x.
//
// Seguridad (ver docs/AUS_KIM_V2_ESP32-C3.md):
//   - Los motores arrancan detenidos. Encender con JP1 RETIRADO y colocarlo despues.
//   - Probar con el robot elevado la primera vez.
//   - Si el telefono deja de consultar el estado durante 1,5 s, los motores se detienen.
//   - El boton STOP de la pagina y /api/stop detienen todo.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "web_cal.h"

// ---------------------------------------------------------------- Red
const char *WIFI_SSID = "AUS_KIM_V2";
const char *WIFI_PASS = "AUSKIM2026";
IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
Preferences prefs;

// ---------------------------------------------------------------- Pines V2
constexpr uint8_t PIN_SH_I   = 0;
constexpr uint8_t PIN_SH_F   = 1;
constexpr uint8_t PIN_AIN1   = 2;   // motor derecho
constexpr uint8_t PIN_SH_D   = 3;
constexpr uint8_t PIN_QRE    = 4;
constexpr uint8_t PIN_ENC_IB = 5;
constexpr uint8_t PIN_ENC_IA = 6;
constexpr uint8_t PIN_ENC_DB = 7;
constexpr uint8_t PIN_AIN2   = 8;   // motor derecho
constexpr uint8_t PIN_START  = 9;
constexpr uint8_t PIN_ENC_DA = 10;
constexpr uint8_t PIN_BIN1   = 20;  // motor izquierdo
constexpr uint8_t PIN_BIN2   = 21;  // motor izquierdo

// Si un motor gira al reves o un encoder cuenta al reves, cambiar el signo aqui.
constexpr int MOTOR_SIGN_L = 1;
constexpr int MOTOR_SIGN_R = 1;
constexpr int ENC_SIGN_L   = 1;
constexpr int ENC_SIGN_R   = 1;

constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t  PWM_BITS    = 8;
constexpr uint32_t CONTROL_MS  = 5;       // lazo de control a 200 Hz
constexpr uint32_t FAILSAFE_MS = 1500;    // sin consultas de la pagina: detener
constexpr uint32_t SETTLE_MS   = 150;     // tiempo dentro de tolerancia para dar por terminado
constexpr uint32_t BLOCKED_MS  = 700;     // sin avance de cuentas con PWM aplicado

// ------------------------------------------------------------- Parametros
struct Params {
  float cpm = 4.5f;          // cuentas de cuadratura (x4) por mm. ESTIMADO hasta calibrar
  float trackMm = 79.0f;     // separacion entre ruedas
  float kp = 6.0f;           // PWM por mm de error
  int   pwmMax = 140;
  int   pwmMin = 70;
  float tolMm = 2.0f;
  int   accel = 4;           // maximo cambio de PWM por tick de 5 ms
  int   frontStopAdc = 0;    // 0 = desactivado. Si el Sharp frontal supera este ADC, frena el avance
  bool  calibrated = false;
} P;

uint32_t cfgVer = 1;

void loadParams() {
  prefs.begin("auskim", true);
  P.cpm          = prefs.getFloat("cpm", P.cpm);
  P.trackMm      = prefs.getFloat("track", P.trackMm);
  P.kp           = prefs.getFloat("kp", P.kp);
  P.pwmMax       = prefs.getInt("pwmMax", P.pwmMax);
  P.pwmMin       = prefs.getInt("pwmMin", P.pwmMin);
  P.tolMm        = prefs.getFloat("tol", P.tolMm);
  P.accel        = prefs.getInt("accel", P.accel);
  P.frontStopAdc = prefs.getInt("front", P.frontStopAdc);
  P.calibrated   = prefs.getBool("cal", false);
  prefs.end();
}

void saveParams() {
  prefs.begin("auskim", false);
  prefs.putFloat("cpm", P.cpm);
  prefs.putFloat("track", P.trackMm);
  prefs.putFloat("kp", P.kp);
  prefs.putInt("pwmMax", P.pwmMax);
  prefs.putInt("pwmMin", P.pwmMin);
  prefs.putFloat("tol", P.tolMm);
  prefs.putInt("accel", P.accel);
  prefs.putInt("front", P.frontStopAdc);
  prefs.putBool("cal", P.calibrated);
  prefs.end();
}

// ------------------------------------------------------------- Encoders
struct Encoder {
  volatile int32_t count;
  volatile uint8_t state;
  uint8_t pinA, pinB;
};

Encoder encL = {0, 0, PIN_ENC_IA, PIN_ENC_IB};
Encoder encR = {0, 0, PIN_ENC_DA, PIN_ENC_DB};

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

int32_t countL() { return ENC_SIGN_L * encL.count; }
int32_t countR() { return ENC_SIGN_R * encR.count; }

void zeroEncoders() {
  noInterrupts();
  encL.count = 0;
  encR.count = 0;
  interrupts();
}

// --------------------------------------------------------------- Motores
const uint8_t MOTOR_PINS[4] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};

void pwmWrite(uint8_t idx, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[idx], duty);
#else
  ledcWrite(idx, duty);
#endif
}

void setupMotors() {
  for (uint8_t i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[i], PWM_FREQ_HZ, PWM_BITS);
#else
    ledcSetup(i, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(MOTOR_PINS[i], i);
#endif
    pwmWrite(i, 0);
  }
}

// pwm firmado -255..255. Positivo: IN1 con PWM, IN2 en bajo.
void setMotor(bool right, int pwm) {
  pwm = constrain(pwm, -255, 255);
  pwm *= right ? MOTOR_SIGN_R : MOTOR_SIGN_L;
  uint8_t base = right ? 0 : 2;
  if (pwm >= 0) {
    pwmWrite(base, pwm);
    pwmWrite(base + 1, 0);
  } else {
    pwmWrite(base, 0);
    pwmWrite(base + 1, -pwm);
  }
}

void stopMotors() {
  setMotor(false, 0);
  setMotor(true, 0);
}

// -------------------------------------------------------------- Sensores
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

// ------------------------------------------------------------ Movimiento
enum Motion : uint8_t { M_IDLE, M_MOVE, M_TURN, M_OPEN };
Motion motion = M_IDLE;

int32_t startL = 0, startR = 0, tgtL = 0, tgtR = 0;
float cmdValue = 0;
uint32_t motionStartMs = 0, motionTimeoutMs = 0, lastProgressMs = 0, settleStartMs = 0, openEndMs = 0;
int32_t lastProgL = 0, lastProgR = 0;
int curPwmL = 0, curPwmR = 0, openPwmL = 0, openPwmR = 0;
bool moveForward = true;
uint32_t lastHeartbeatMs = 0;

struct Result {
  uint32_t id = 0;
  char type[8] = "-";
  float cmd = 0;
  int32_t cmdL = 0, cmdR = 0, gotL = 0, gotR = 0;
  float mmL = 0, mmR = 0, deg = 0;
  uint32_t ms = 0;
  char reason[16] = "-";
} lastRes;

void finishMotion(const char *reason) {
  stopMotors();
  curPwmL = curPwmR = 0;
  if (motion != M_IDLE) {
    lastRes.id++;
    const char *t = motion == M_MOVE ? "AVANCE" : motion == M_TURN ? "GIRO" : "MOTOR";
    strncpy(lastRes.type, t, sizeof(lastRes.type) - 1);
    lastRes.type[sizeof(lastRes.type) - 1] = 0;
    lastRes.cmd = cmdValue;
    lastRes.cmdL = tgtL - startL;
    lastRes.cmdR = tgtR - startR;
    lastRes.gotL = countL() - startL;
    lastRes.gotR = countR() - startR;
    lastRes.mmL = lastRes.gotL / P.cpm;
    lastRes.mmR = lastRes.gotR / P.cpm;
    // Giro sobre el eje: arco de cada rueda = PI * separacion * grados / 360
    lastRes.deg = ((lastRes.gotL - lastRes.gotR) / 2.0f) / P.cpm / (PI * P.trackMm) * 360.0f;
    lastRes.ms = millis() - motionStartMs;
    strncpy(lastRes.reason, reason, sizeof(lastRes.reason) - 1);
    lastRes.reason[sizeof(lastRes.reason) - 1] = 0;
  }
  motion = M_IDLE;
}

int pwmFromErrMm(float errMm) {
  float a = fabsf(errMm);
  if (a <= P.tolMm) return 0;
  int v = (int)(P.kp * a);
  if (v < P.pwmMin) v = P.pwmMin;
  if (v > P.pwmMax) v = P.pwmMax;
  return errMm > 0 ? v : -v;
}

int rampTo(int cur, int want) {
  if (want == 0) return 0;
  if (want > cur + P.accel) return cur + P.accel;
  if (want < cur - P.accel) return cur - P.accel;
  return want;
}

void beginMotion(Motion m, float value) {
  startL = countL();
  startR = countR();
  lastProgL = startL;
  lastProgR = startR;
  cmdValue = value;
  curPwmL = curPwmR = 0;
  settleStartMs = 0;
  motionStartMs = lastProgressMs = millis();
  motion = m;
}

bool startMove(float mm) {
  if (motion != M_IDLE) return false;
  startL = countL();
  startR = countR();
  int32_t c = lroundf(mm * P.cpm);
  tgtL = startL + c;
  tgtR = startR + c;
  moveForward = mm > 0;
  motionTimeoutMs = 2500 + (uint32_t)(fabsf(mm) * 25.0f);
  beginMotion(M_MOVE, mm);
  return true;
}

bool startTurn(float deg) {
  if (motion != M_IDLE) return false;
  startL = countL();
  startR = countR();
  float arcMm = P.trackMm * PI * deg / 360.0f;
  int32_t c = lroundf(arcMm * P.cpm);
  tgtL = startL + c;   // derecha (deg>0): izquierda avanza, derecha retrocede
  tgtR = startR - c;
  motionTimeoutMs = 3500 + (uint32_t)(fabsf(deg) * 10.0f);
  beginMotion(M_TURN, deg);
  return true;
}

bool startOpen(char side, int pwm, uint32_t ms) {
  if (motion != M_IDLE) return false;
  openPwmL = (side == 'l' || side == 'b') ? pwm : 0;
  openPwmR = (side == 'r' || side == 'b') ? pwm : 0;
  openEndMs = millis() + ms;
  tgtL = countL();  // sin objetivo: el resultado solo informa las cuentas
  tgtR = countR();
  motionTimeoutMs = ms + 500;
  beginMotion(M_OPEN, pwm);
  tgtL = startL;
  tgtR = startR;
  return true;
}

void controlTick() {
  if (motion == M_IDLE) return;
  uint32_t now = millis();

  if (now - lastHeartbeatMs > FAILSAFE_MS) { finishMotion("FAILSAFE_WIFI"); return; }

  if (motion == M_OPEN) {
    if (now >= openEndMs) { finishMotion("OK"); return; }
    curPwmL = rampTo(curPwmL, openPwmL);
    curPwmR = rampTo(curPwmR, openPwmR);
    setMotor(false, curPwmL);
    setMotor(true, curPwmR);
    return;
  }

  if (now - motionStartMs > motionTimeoutMs) { finishMotion("TIMEOUT"); return; }

  if (motion == M_MOVE && moveForward && P.frontStopAdc > 0) {
    if (analogRead(PIN_SH_F) >= P.frontStopAdc) { finishMotion("FRENTE"); return; }
  }

  int32_t pl = countL(), pr = countR();
  float errL = (tgtL - pl) / P.cpm;
  float errR = (tgtR - pr) / P.cpm;
  int wantL = pwmFromErrMm(errL);
  int wantR = pwmFromErrMm(errR);

  if (wantL == 0 && wantR == 0) {
    if (settleStartMs == 0) settleStartMs = now;
    if (now - settleStartMs >= SETTLE_MS) { finishMotion("OK"); return; }
  } else {
    settleStartMs = 0;
  }

  // Deteccion de bloqueo: se aplica PWM pero los encoders no avanzan
  if (abs(pl - lastProgL) + abs(pr - lastProgR) >= 3) {
    lastProgL = pl;
    lastProgR = pr;
    lastProgressMs = now;
  } else if ((wantL != 0 || wantR != 0) && now - lastProgressMs > BLOCKED_MS) {
    finishMotion("BLOQUEADO");
    return;
  }

  curPwmL = rampTo(curPwmL, wantL);
  curPwmR = rampTo(curPwmR, wantR);
  setMotor(false, curPwmL);
  setMotor(true, curPwmR);
}

// ------------------------------------------------------------------ API
void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void sendOk(const String &msg) { sendJson(200, "{\"ok\":true,\"msg\":\"" + msg + "\"}"); }
void sendErr(const String &err) { sendJson(400, "{\"ok\":false,\"error\":\"" + err + "\"}"); }

const char *motionName() {
  switch (motion) {
    case M_MOVE: return "AVANCE";
    case M_TURN: return "GIRO";
    case M_OPEN: return "MOTOR";
    default: return "IDLE";
  }
}

// Velocidad de ruedas (cuentas/s), actualizada cada 100 ms en loop()
int32_t spdL = 0, spdR = 0;

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleStatus() {
  lastHeartbeatMs = millis();
  String j;
  j.reserve(800);
  j += "{\"encL\":" + String(countL()) + ",\"encR\":" + String(countR());
  j += ",\"spdL\":" + String(spdL) + ",\"spdR\":" + String(spdR);
  j += ",\"shI\":" + String(readMedian5(PIN_SH_I));
  j += ",\"shF\":" + String(readMedian5(PIN_SH_F));
  j += ",\"shD\":" + String(readMedian5(PIN_SH_D));
  j += ",\"qre\":" + String(readMedian5(PIN_QRE));
  j += ",\"start\":" + String(digitalRead(PIN_START) == LOW ? 1 : 0);
  j += ",\"motion\":\"" + String(motionName()) + "\"";
  j += ",\"cfgVer\":" + String(cfgVer);
  j += ",\"calibrated\":" + String(P.calibrated ? "true" : "false");
  j += ",\"cpm\":" + String(P.cpm, 4) + ",\"track\":" + String(P.trackMm, 2);
  j += ",\"kp\":" + String(P.kp, 2) + ",\"pwmMax\":" + String(P.pwmMax) + ",\"pwmMin\":" + String(P.pwmMin);
  j += ",\"tol\":" + String(P.tolMm, 2) + ",\"accel\":" + String(P.accel) + ",\"frontStop\":" + String(P.frontStopAdc);
  j += ",\"res\":{\"id\":" + String(lastRes.id) + ",\"type\":\"" + lastRes.type + "\"";
  j += ",\"cmd\":" + String(lastRes.cmd, 2);
  j += ",\"cmdL\":" + String(lastRes.cmdL) + ",\"cmdR\":" + String(lastRes.cmdR);
  j += ",\"gotL\":" + String(lastRes.gotL) + ",\"gotR\":" + String(lastRes.gotR);
  j += ",\"mmL\":" + String(lastRes.mmL, 2) + ",\"mmR\":" + String(lastRes.mmR, 2);
  j += ",\"deg\":" + String(lastRes.deg, 2) + ",\"ms\":" + String(lastRes.ms);
  j += ",\"reason\":\"" + String(lastRes.reason) + "\"}}";
  sendJson(200, j);
}

void handleMove() {
  lastHeartbeatMs = millis();
  float mm = server.arg("mm").toFloat();
  if (fabsf(mm) < 1.0f || fabsf(mm) > 2000.0f) { sendErr("DISTANCIA_INVALIDA"); return; }
  if (!startMove(mm)) { sendErr("OCUPADO"); return; }
  sendOk("Avanzando " + String(mm, 0) + " mm");
}

void handleTurn() {
  lastHeartbeatMs = millis();
  float deg = server.arg("deg").toFloat();
  if (fabsf(deg) < 1.0f || fabsf(deg) > 720.0f) { sendErr("ANGULO_INVALIDO"); return; }
  if (!startTurn(deg)) { sendErr("OCUPADO"); return; }
  sendOk("Girando " + String(deg, 0) + " grados");
}

void handleMotor() {
  lastHeartbeatMs = millis();
  String s = server.arg("side");
  char side = s.length() ? s.charAt(0) : 'b';
  int pwm = server.arg("pwm").toInt();
  uint32_t ms = (uint32_t)constrain(server.arg("ms").toInt(), 100, 3000);
  if (side != 'l' && side != 'r' && side != 'b') { sendErr("LADO_INVALIDO"); return; }
  if (!startOpen(side, constrain(pwm, -255, 255), ms)) { sendErr("OCUPADO"); return; }
  sendOk("Probando motor");
}

void handleStop() {
  if (motion != M_IDLE) finishMotion("STOP"); else stopMotors();
  sendOk("Detenido");
}

void handleReset() {
  zeroEncoders();
  spdL = spdR = 0;
  sendOk("Encoders en cero");
}

void handleConfig() {
  lastHeartbeatMs = millis();
  if (motion != M_IDLE) { sendErr("OCUPADO"); return; }
  if (server.hasArg("cpm"))       { float v = server.arg("cpm").toFloat();   if (v > 0.1f && v < 100) { P.cpm = v; } }
  if (server.hasArg("track"))     { float v = server.arg("track").toFloat(); if (v > 20 && v < 300)   { P.trackMm = v; } }
  if (server.hasArg("kp"))        { float v = server.arg("kp").toFloat();    if (v > 0 && v < 100)    { P.kp = v; } }
  if (server.hasArg("pwmMax"))    { P.pwmMax = constrain(server.arg("pwmMax").toInt(), 30, 255); }
  if (server.hasArg("pwmMin"))    { P.pwmMin = constrain(server.arg("pwmMin").toInt(), 0, 255); }
  if (server.hasArg("tol"))       { float v = server.arg("tol").toFloat();   if (v >= 0.2f && v < 20) { P.tolMm = v; } }
  if (server.hasArg("accel"))     { P.accel = constrain(server.arg("accel").toInt(), 1, 50); }
  if (server.hasArg("frontStop")) { P.frontStopAdc = constrain(server.arg("frontStop").toInt(), 0, 4095); }
  if (P.pwmMin > P.pwmMax) P.pwmMin = P.pwmMax;
  cfgVer++;
  sendOk("Parametros aplicados (aun sin guardar)");
}

void handleSave() {
  saveParams();
  sendOk("Calibracion guardada en memoria");
}

void handleCal() {
  lastHeartbeatMs = millis();
  if (motion != M_IDLE) { sendErr("OCUPADO"); return; }
  String what = server.arg("what");
  float avg = (abs(countL()) + abs(countR())) / 2.0f;

  if (what == "dist") {
    float mm = server.arg("mm").toFloat();
    if (mm <= 0 || avg < 5) { sendErr("SIN_CUENTAS_O_DISTANCIA_INVALIDA"); return; }
    P.cpm = avg / mm;
    P.calibrated = true;
  } else if (what == "wheel") {
    float d = server.arg("diam").toFloat();
    float perRev = (float)max(abs(countL()), abs(countR()));
    if (d <= 0 || perRev < 5) { sendErr("SIN_CUENTAS_O_DIAMETRO_INVALIDO"); return; }
    P.cpm = perRev / (PI * d);
    P.calibrated = true;
  } else if (what == "track") {
    if (!P.calibrated) { sendErr("PRIMERO_CALIBRAR_CUENTAS_POR_MM"); return; }
    if (avg < 5) { sendErr("SIN_CUENTAS"); return; }
    P.trackMm = avg / P.cpm / PI;   // 360 grados: cada rueda recorre PI * separacion
  } else if (what == "fix_move") {
    float actual = server.arg("actual").toFloat();
    if (strcmp(lastRes.type, "AVANCE") != 0 || actual <= 0) { sendErr("HACER_UN_AVANCE_E_INGRESAR_MEDIDA"); return; }
    float got = (abs(lastRes.gotL) + abs(lastRes.gotR)) / 2.0f;
    if (got < 5) { sendErr("SIN_CUENTAS"); return; }
    P.cpm = got / actual;
    P.calibrated = true;
  } else if (what == "fix_turn") {
    float actual = server.arg("actual").toFloat();
    if (strcmp(lastRes.type, "GIRO") != 0 || actual <= 0) { sendErr("HACER_UN_GIRO_E_INGRESAR_MEDIDA"); return; }
    float got = (abs(lastRes.gotL) + abs(lastRes.gotR)) / 2.0f;
    if (got < 5) { sendErr("SIN_CUENTAS"); return; }
    P.trackMm = (got / P.cpm) * 360.0f / (PI * actual);
  } else {
    sendErr("CALIBRACION_DESCONOCIDA");
    return;
  }
  cfgVer++;
  sendOk("cuentas/mm=" + String(P.cpm, 3) + "  separacion=" + String(P.trackMm, 1) + " mm (recordar Guardar)");
}

void handleNotFound() { server.send(404, "text/plain", "No encontrado"); }

// ------------------------------------------------------------------ Setup
void setup() {
  // Lo primero: pines de motor en estado seguro.
  setupMotors();
  stopMotors();

  Serial.begin(115200);
  delay(200);

  pinMode(PIN_START, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SH_I, ADC_11db);
  analogSetPinAttenuation(PIN_SH_F, ADC_11db);
  analogSetPinAttenuation(PIN_SH_D, ADC_11db);
  analogSetPinAttenuation(PIN_QRE, ADC_11db);

  setupEncoder(encL);
  setupEncoder(encR);
  loadParams();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/move", HTTP_GET, handleMove);
  server.on("/api/turn", HTTP_GET, handleTurn);
  server.on("/api/motor", HTTP_GET, handleMotor);
  server.on("/api/stop", HTTP_ANY, handleStop);
  server.on("/api/reset", HTTP_GET, handleReset);
  server.on("/api/config", HTTP_GET, handleConfig);
  server.on("/api/save", HTTP_GET, handleSave);
  server.on("/api/cal", HTTP_GET, handleCal);
  server.onNotFound(handleNotFound);
  server.begin();

  lastHeartbeatMs = millis();
  Serial.print("AUS_KIM V2 calibracion. Panel: http://");
  Serial.println(WiFi.softAPIP());
  Serial.printf("cuentas/mm=%.3f separacion=%.1f mm calibrado=%d\n", P.cpm, P.trackMm, P.calibrated);
}

void loop() {
  server.handleClient();

  static uint32_t lastTick = 0, lastSpd = 0;
  static int32_t prevL = 0, prevR = 0;
  uint32_t now = millis();

  if (now - lastTick >= CONTROL_MS) {
    lastTick = now;
    controlTick();
  }

  if (now - lastSpd >= 100) {
    int32_t l = countL(), r = countR();
    spdL = (l - prevL) * 10;
    spdR = (r - prevR) * 10;
    prevL = l;
    prevR = r;
    lastSpd = now;
  }
}

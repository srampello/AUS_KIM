// AUS_KIM V2 - Diagnostico y calibracion (ESP32-C3 Super Mini)
//
// Primer firmware de la V2. NO navega: sirve para validar el hardware y MEDIR
// lo que falta para el control por encoders (cuentas por mm, separacion de ruedas).
//
// Uso: abrir el monitor serie a 115200 baudios con "Fin de linea: Nueva linea".
// Arduino IDE: placa "ESP32C3 Dev Module" y "USB CDC On Boot: Enabled".
// Compatible con arduino-esp32 core 2.x y 3.x.
//
// Procedimiento de seguridad (ver docs/AUS_KIM_V2_ESP32-C3.md):
//   1. Encender con JP1 RETIRADO, dejar que arranque este firmware.
//   2. Colocar JP1 recien despues. Los motores arrancan siempre detenidos.
//   3. Probar con el robot elevado antes de apoyarlo en pista.
//
// Comandos (enviar y Enter):
//   h                 ayuda
//   s                 activa/desactiva stream de sensores (Sharp, QRE, START)
//   e                 activa/desactiva stream de encoders
//   z                 pone en cero los encoders
//   m <l|r|b> <pwm>   mueve motor izq/der/ambos 1 s con PWM firmado (-255..255)
//   x                 detiene los motores
//   d <mm>            calibra: mover el robot a mano <mm> en recta tras 'z'
//   w <diam_mm>       calibra: girar UNA rueda una vuelta a mano tras 'z'
//   g                 calibra: girar el robot a mano 360 grados sobre su eje tras 'z'
//                     y calcula la separacion de ruedas (requiere 'd' o 'w' antes)

#include <Arduino.h>
#include <esp_arduino_version.h>
#include "driver/gpio.h"

// ---------------------------------------------------------------- Pines V2
constexpr uint8_t PIN_SH_I   = 0;   // Sharp lateral izquierdo
constexpr uint8_t PIN_SH_F   = 1;   // Sharp frontal
constexpr uint8_t PIN_AIN1   = 2;   // DRV8833 IN1 (motor derecho)
constexpr uint8_t PIN_SH_D   = 3;   // Sharp lateral derecho
constexpr uint8_t PIN_QRE    = 4;   // QRE1113 de piso
constexpr uint8_t PIN_ENC_IB = 5;
constexpr uint8_t PIN_ENC_IA = 6;
constexpr uint8_t PIN_ENC_DB = 7;
constexpr uint8_t PIN_AIN2   = 8;   // DRV8833 IN2 (motor derecho)
constexpr uint8_t PIN_START  = 9;   // activo en LOW
constexpr uint8_t PIN_ENC_DA = 10;
constexpr uint8_t PIN_BIN1   = 20;  // DRV8833 IN3 (motor izquierdo)
constexpr uint8_t PIN_BIN2   = 21;  // DRV8833 IN4 (motor izquierdo)

// ------------------------------------------------------- Configuracion
// Si un motor gira al reves o un encoder cuenta al reves, cambiar el signo aqui.
constexpr int  MOTOR_SIGN_L = 1;
constexpr int  MOTOR_SIGN_R = 1;
constexpr int  ENC_SIGN_L   = 1;
constexpr int  ENC_SIGN_R   = 1;

constexpr uint32_t PWM_FREQ_HZ  = 20000;
constexpr uint8_t  PWM_BITS     = 8;
constexpr uint32_t MOTOR_TEST_MS = 1000;

// Separacion entre ruedas medida (mm). La calibracion 'g' la verifica.
float trackMm = 79.0f;
// Cuentas de cuadratura (x4) por mm de avance. 0 = todavia sin calibrar.
float countsPerMm = 0.0f;

// ------------------------------------------------------------- Encoders
struct Encoder {
  volatile int32_t count;
  volatile uint8_t state;
  uint8_t pinA, pinB;
};

Encoder encL = {0, 0, PIN_ENC_IA, PIN_ENC_IB};
Encoder encR = {0, 0, PIN_ENC_DA, PIN_ENC_DB};

// Tabla de decodificacion en cuadratura: indice = (estadoPrevio << 2) | estadoNuevo
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
  uint8_t base = right ? 0 : 2;  // derecho: AIN1/AIN2, izquierdo: BIN1/BIN2
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

// --------------------------------------------------------------- Estado
bool streamSensors = false;
bool streamEncoders = false;
uint32_t lastStream = 0;

void printHelp() {
  Serial.println(F("Comandos: h | s | e | z | m <l|r|b> <pwm> | x | d <mm> | w <diam_mm> | g"));
  Serial.println(F("Motores arrancan detenidos. Colocar JP1 solo despues del arranque."));
}

void printCalibration() {
  Serial.printf("cuentas/mm=%.4f  separacion ruedas=%.1f mm\n", countsPerMm, trackMm);
}

void runMotorTest(char side, int pwm) {
  zeroEncoders();
  uint32_t t0 = millis();
  if (side == 'l' || side == 'b') setMotor(false, pwm);
  if (side == 'r' || side == 'b') setMotor(true, pwm);
  while (millis() - t0 < MOTOR_TEST_MS) {
    if (Serial.available()) break;  // cualquier caracter aborta
    delay(5);
  }
  uint32_t dt = millis() - t0;
  stopMotors();
  while (Serial.available()) Serial.read();
  Serial.printf("Test motor %c pwm=%d: %lu ms  cuentasL=%ld cuentasR=%ld  (%.0f / %.0f cuentas/s)\n",
                side, pwm, (unsigned long)dt, (long)countL(), (long)countR(),
                countL() * 1000.0f / dt, countR() * 1000.0f / dt);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  switch (c) {
    case 'h': printHelp(); break;
    case 's': streamSensors = !streamSensors; Serial.println(streamSensors ? F("Stream sensores ON") : F("Stream sensores OFF")); break;
    case 'e': streamEncoders = !streamEncoders; Serial.println(streamEncoders ? F("Stream encoders ON") : F("Stream encoders OFF")); break;
    case 'z': zeroEncoders(); Serial.println(F("Encoders en cero")); break;
    case 'x': stopMotors(); Serial.println(F("Motores detenidos")); break;
    case 'm': {
      int sp = arg.indexOf(' ');
      if (sp < 1) { Serial.println(F("Uso: m <l|r|b> <pwm>")); break; }
      char side = arg.charAt(0);
      int pwm = arg.substring(sp + 1).toInt();
      if (side != 'l' && side != 'r' && side != 'b') { Serial.println(F("Lado invalido (l, r o b)")); break; }
      runMotorTest(side, pwm);
      break;
    }
    case 'd': {
      float mm = arg.toFloat();
      if (mm <= 0) { Serial.println(F("Uso: d <mm> (distancia recorrida a mano desde 'z')")); break; }
      float avg = (abs(countL()) + abs(countR())) / 2.0f;
      countsPerMm = avg / mm;
      Serial.printf("L=%ld R=%ld promedio=%.1f en %.1f mm\n", (long)countL(), (long)countR(), avg, mm);
      printCalibration();
      break;
    }
    case 'w': {
      float diam = arg.toFloat();
      if (diam <= 0) { Serial.println(F("Uso: w <diametro_mm> (una vuelta de rueda desde 'z')")); break; }
      float avg = (abs(countL()) + abs(countR())) / 2.0f;
      // Si solo giraste una rueda, la otra marca ~0: se usa la de mayor valor.
      float perRev = max((float)abs(countL()), (float)abs(countR()));
      Serial.printf("Cuentas por vuelta de rueda: L=%ld R=%ld (usando %.0f)\n", (long)countL(), (long)countR(), perRev);
      (void)avg;
      countsPerMm = perRev / (PI * diam);
      printCalibration();
      break;
    }
    case 'g': {
      if (countsPerMm <= 0) { Serial.println(F("Primero calibrar cuentas/mm con 'd' o 'w'")); break; }
      float avg = (abs(countL()) + abs(countR())) / 2.0f;
      // Giro de 360 grados sobre el eje: cada rueda recorre PI * separacion.
      float track = avg / countsPerMm / PI;
      Serial.printf("L=%ld R=%ld promedio=%.1f -> separacion estimada %.1f mm (medida: %.1f mm)\n",
                    (long)countL(), (long)countR(), avg, track, trackMm);
      break;
    }
    default: Serial.println(F("Comando desconocido. 'h' para ayuda")); break;
  }
}

// ------------------------------------------------------------------ Setup
void setup() {
  // Lo primero: pines de motor en estado seguro.
  setupMotors();
  stopMotors();

  Serial.begin(115200);
  delay(300);

  pinMode(PIN_START, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SH_I, ADC_11db);
  analogSetPinAttenuation(PIN_SH_F, ADC_11db);
  analogSetPinAttenuation(PIN_SH_D, ADC_11db);
  analogSetPinAttenuation(PIN_QRE, ADC_11db);

  setupEncoder(encL);
  setupEncoder(encR);

  Serial.println(F("AUS_KIM V2 - diagnostico y calibracion"));
  printHelp();
  printCalibration();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  if ((streamSensors || streamEncoders) && millis() - lastStream >= 100) {
    lastStream = millis();
    if (streamSensors) {
      Serial.printf("SH_I=%4u SH_F=%4u SH_D=%4u QRE=%4u START=%d\n",
                    readMedian5(PIN_SH_I), readMedian5(PIN_SH_F), readMedian5(PIN_SH_D),
                    readMedian5(PIN_QRE), digitalRead(PIN_START) == LOW ? 1 : 0);
    }
    if (streamEncoders) {
      Serial.printf("ENC L=%ld R=%ld\n", (long)countL(), (long)countR());
    }
  }
}

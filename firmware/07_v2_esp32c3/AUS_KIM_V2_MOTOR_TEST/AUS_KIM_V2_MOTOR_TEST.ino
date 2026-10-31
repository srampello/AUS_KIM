// AUS_KIM V2 - Prueba minima de motores (ESP32-C3 Super Mini + DRV8833)
//
// Al pulsar START (GPIO9) los dos motores van ADELANTE, se detienen un instante y van ATRAS,
// a velocidad 200 (de 255). Despues quedan detenidos hasta la proxima pulsacion.
// Pulsar START durante la secuencia la detiene en el acto.
//
// No usa Wi-Fi, ni Serial, ni encoders, ni sensores. Serial se evita a proposito: GPIO20/21
// (motor izquierdo) son tambien RX/TX de la UART0 y Serial.begin() los dejaria en alto.
//
// Seguridad: encender con el robot levantado (ruedas sin tocar la mesa) y JP1 retirado;
// colocar JP1 cuando el firmware ya arranco. No presionar START al encender (GPIO9 es pin de arranque).
// Si un motor gira al reves de lo esperado, cambiar INVERTIR_IZQ / INVERTIR_DER.
// Compatible con arduino-esp32 core 2.x y 3.x.

#include <Arduino.h>
#include <esp_arduino_version.h>

// ------------------------------------------------------------ Ajustes
const int  VELOCIDAD     = 200;    // 0..255
const uint32_t T_ADELANTE_MS = 2000;
const uint32_t T_PAUSA_MS    = 500;
const uint32_t T_ATRAS_MS    = 2000;
const bool INVERTIR_IZQ  = false;
const bool INVERTIR_DER  = false;

// ------------------------------------------------------------ Pines V2
constexpr uint8_t PIN_AIN1  = 2;    // motor derecho
constexpr uint8_t PIN_AIN2  = 8;
constexpr uint8_t PIN_START = 9;    // activo en LOW
constexpr uint8_t PIN_BIN1  = 20;   // motor izquierdo
constexpr uint8_t PIN_BIN2  = 21;

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
    ledcAttach(MOTOR_PINS[i], 20000, 8);
#else
    ledcSetup(i, 20000, 8);
    ledcAttachPin(MOTOR_PINS[i], i);
#endif
    pwmWrite(i, 0);
  }
}

// pwm firmado -255..255. Positivo: IN1/IN3 con PWM, IN2/IN4 en bajo.
void setMotor(bool derecho, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (derecho ? INVERTIR_DER : INVERTIR_IZQ) pwm = -pwm;
  uint8_t base = derecho ? 0 : 2;
  if (pwm >= 0) { pwmWrite(base, pwm); pwmWrite(base + 1, 0); }
  else          { pwmWrite(base, 0);   pwmWrite(base + 1, -pwm); }
}

void motores(int izq, int der) { setMotor(false, izq); setMotor(true, der); }
void parar() { motores(0, 0); }

// ------------------------------------------------------------ Boton START (con antirrebote)
bool startPulsado() {          // true una sola vez por pulsacion
  static bool previo = false;
  static uint32_t t = 0;
  bool down = digitalRead(PIN_START) == LOW;
  if (down && !previo && millis() - t > 50) { previo = true; t = millis(); return true; }
  if (!down && previo) { previo = false; t = millis(); }
  return false;
}

// ------------------------------------------------------------ Secuencia
enum Fase { QUIETO, ADELANTE, PAUSA, ATRAS };
Fase fase = QUIETO;
uint32_t tFase = 0;

void irA(Fase f) {
  fase = f;
  tFase = millis();
  switch (f) {
    case ADELANTE: motores(VELOCIDAD, VELOCIDAD); break;
    case ATRAS:    motores(-VELOCIDAD, -VELOCIDAD); break;
    default:       parar(); break;
  }
}

void setup() {
  setupMotors();               // lo primero: motores detenidos
  parar();
  pinMode(PIN_START, INPUT_PULLUP);
  while (digitalRead(PIN_START) == LOW) delay(10);   // esperar a que se suelte si estaba apretado
}

void loop() {
  bool press = startPulsado();
  uint32_t dt = millis() - tFase;

  if (press) {
    if (fase == QUIETO) irA(ADELANTE); else irA(QUIETO);
    return;
  }
  if (fase == ADELANTE && dt >= T_ADELANTE_MS) irA(PAUSA);
  else if (fase == PAUSA && dt >= T_PAUSA_MS) irA(ATRAS);
  else if (fase == ATRAS && dt >= T_ATRAS_MS) irA(QUIETO);
}

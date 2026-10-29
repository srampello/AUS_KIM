// AUS_KIM V2 - Calibracion de encoders, avance y giros por Wi-Fi (ESP32-C3 Super Mini)
//
// Red:   SSID AUS_KIM_V2   clave AUSKIM2026   panel http://192.168.4.1
// Placa en Arduino IDE: "ESP32C3 Dev Module", "USB CDC On Boot: Enabled".
// Compatible con arduino-esp32 core 2.x y 3.x. Es un unico archivo: no hace falta nada mas.
//
// Para que sirve: medir cuantas cuentas de encoder son 1 mm de avance y cuantas hacen
// falta para girar un angulo (con la separacion de ruedas), y probar avances y giros
// controlados por encoders. Los valores calibrados se guardan en la memoria del robot.
//
// Seguridad (ver docs/AUS_KIM_V2_ESP32-C3.md):
//   - Encender con JP1 retirado; colocar JP1 recien cuando el firmware ya arranco.
//   - Los motores arrancan detenidos y cada movimiento tiene limite de tiempo.
//   - STOP en la pagina, o pulsar START (GPIO9) durante un movimiento, lo detiene.
//   - No presionar START al encender o resetear (GPIO9 es pin de arranque).

#include <Arduino.h>
#include <cmath>
#include <esp_arduino_version.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "driver/gpio.h"

// Interfaz de calibracion V2 (en PROGMEM, sin Internet ni LittleFS).
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM V2 | Calibracion</title>
<style>
:root{--bg:#07070a;--panel:#111117;--panel2:#17171f;--border:#2a2a36;--text:#f4f4f8;--muted:#9a9aab;
--violet:#8b5cf6;--violet2:#6d28d9;--good:#22c55e;--bad:#ef4444;--warn:#f59e0b}
*{box-sizing:border-box}
html{color-scheme:dark}
body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;padding-bottom:90px}
.wrap{max-width:760px;margin:auto;padding:16px}
h1{margin:4px 0 2px;font-size:26px;letter-spacing:-.02em}
.sub{color:var(--muted);font-size:13px;margin-bottom:14px}
.card{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:14px;margin-bottom:14px}
.card h2{margin:0 0 4px;font-size:15px}
.card p{margin:2px 0 10px;color:var(--muted);font-size:12.5px;line-height:1.45}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px}
.kv{background:var(--panel2);border:1px solid var(--border);border-radius:10px;padding:8px 10px}
.kv small{display:block;color:var(--muted);font-size:10.5px;text-transform:uppercase;letter-spacing:.08em}
.kv b{font-size:19px;font-variant-numeric:tabular-nums}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:8px 0}
label{font-size:12px;color:var(--muted)}
input[type=number]{width:96px;padding:9px;border-radius:9px;border:1px solid var(--border);background:#0c0c11;color:var(--text);font-size:15px}
button{padding:10px 14px;border-radius:10px;border:1px solid var(--border);background:var(--panel2);color:var(--text);font-weight:700;font-size:13px;cursor:pointer}
button:active{transform:scale(.98)}
button.main{background:linear-gradient(180deg,var(--violet),var(--violet2));border-color:#9f7aea}
button.small{padding:7px 10px;font-weight:600}
.res{background:#0c0c11;border:1px dashed var(--border);border-radius:10px;padding:10px;font-size:13px;line-height:1.55;margin-top:8px;font-variant-numeric:tabular-nums}
#msg{min-height:20px;font-size:13px;margin:6px 0}
#msg.ok{color:var(--good)} #msg.bad{color:var(--bad)}
.warn{color:var(--warn)}
#stop{position:fixed;left:0;right:0;bottom:0;padding:12px 16px;background:linear-gradient(180deg,transparent,#000 40%)}
#stop button{width:100%;max-width:728px;display:block;margin:auto;padding:16px;font-size:17px;background:var(--bad);border-color:#f87171;color:#fff;letter-spacing:.08em}
.chk{display:flex;gap:6px;align-items:center;font-size:12.5px;color:var(--text)}
</style>
</head>
<body>
<div class="wrap">
<h1>AUS_KIM V2</h1>
<div class="sub">Calibracion de encoders, avance y giros &middot; <span id="conn" class="warn">conectando...</span></div>
<div id="msg"></div>

<div class="card">
<h2>Estado en vivo</h2>
<div class="grid">
<div class="kv"><small>Encoder izq</small><b id="cL">-</b></div>
<div class="kv"><small>Encoder der</small><b id="cR">-</b></div>
<div class="kv"><small>Vel izq (c/s)</small><b id="vL">-</b></div>
<div class="kv"><small>Vel der (c/s)</small><b id="vR">-</b></div>
<div class="kv"><small>Sharp izq</small><b id="shI">-</b></div>
<div class="kv"><small>Sharp frente</small><b id="shF">-</b></div>
<div class="kv"><small>Sharp der</small><b id="shD">-</b></div>
<div class="kv"><small>QRE piso</small><b id="qre">-</b></div>
<div class="kv"><small>Estado</small><b id="mode">-</b></div>
<div class="kv"><small>Cuentas / mm</small><b id="cpm">-</b></div>
<div class="kv"><small>Separacion ruedas</small><b id="trk">-</b></div>
</div>
<div class="row"><button onclick="act('/api/zero')">Poner encoders en cero</button></div>
</div>

<div class="card">
<h2>Paso 1 &middot; Verificar motores y encoders</h2>
<p>Con el robot <b>elevado</b>. Cada boton mueve 0,8 s. Al avanzar, los dos encoders deben dar cuentas <b>positivas</b>. Si una rueda gira al reves o el encoder cuenta negativo, marca la casilla de inversion y vuelve a probar.</p>
<div class="row"><label>PWM</label><input type="number" id="tpwm" value="120" min="40" max="255">
<button onclick="mt('L',1)">Izq &uarr;</button><button onclick="mt('R',1)">Der &uarr;</button><button onclick="mt('B',1)">Ambos &uarr;</button>
<button onclick="mt('L',-1)">Izq &darr;</button><button onclick="mt('R',-1)">Der &darr;</button><button onclick="mt('B',-1)">Ambos &darr;</button></div>
<div class="row">
<label class="chk"><input type="checkbox" id="invML" onchange="inv()"> invertir motor izq</label>
<label class="chk"><input type="checkbox" id="invMR" onchange="inv()"> invertir motor der</label>
<label class="chk"><input type="checkbox" id="invEL" onchange="inv()"> invertir encoder izq</label>
<label class="chk"><input type="checkbox" id="invER" onchange="inv()"> invertir encoder der</label>
</div>
</div>

<div class="card">
<h2>Paso 2 &middot; Calibrar cuentas por mm</h2>
<p>No hace falta saber los pulsos por vuelta. Opcion A: pon encoders en cero, empuja el robot <b>a mano en linea recta</b> una distancia medida con regla (por ejemplo 500 mm) y escribela. Opcion B: gira <b>una rueda una vuelta completa</b> a mano e indica su diametro.</p>
<div class="row"><label>A) Distancia real (mm)</label><input type="number" id="calMm" value="500"><button class="main" onclick="act('/api/cal_dist?mm='+$('calMm').value)">Calibrar con distancia</button></div>
<div class="row"><label>B) Diametro rueda (mm)</label><input type="number" id="calDia" value="20" step="0.1"><button onclick="act('/api/cal_wheel?dia='+$('calDia').value)">Calibrar con 1 vuelta</button></div>
<div class="res" id="wheelInfo">-</div>
</div>

<div class="card">
<h2>Paso 3 &middot; Probar avance</h2>
<p>El robot avanza la distancia pedida usando los encoders. Luego <b>mide con regla lo que realmente avanzo</b>, escribelo abajo y ajusta. Repite hasta que coincida. La celda del reglamento es de 250 mm.</p>
<div class="row"><label>Distancia (mm)</label><input type="number" id="mvMm" value="250">
<button class="main" onclick="act('/api/move?mm='+$('mvMm').value)">Avanzar</button>
<button class="small" onclick="act('/api/move?mm=-'+Math.abs($('mvMm').value))">Retroceder</button>
<button class="small" onclick="$('mvMm').value=250">1 celda</button>
<button class="small" onclick="$('mvMm').value=125">media celda</button></div>
<div class="row"><label>Avance real medido (mm)</label><input type="number" id="adjMm" step="0.5"><button onclick="act('/api/adjust_move?real='+$('adjMm').value)">Ajustar cuentas/mm</button></div>
</div>

<div class="card">
<h2>Paso 4 &middot; Probar giros</h2>
<p>Positivo = giro a la <b>izquierda</b>. Marca el piso con cinta, mide el angulo real con transportador y ajusta la separacion de ruedas (medida: 79 mm). Alternativa sin motores: pon encoders en cero, gira el robot a mano una vuelta completa sobre su eje y usa el boton de abajo.</p>
<div class="row"><label>Angulo (&deg;)</label><input type="number" id="tuDeg" value="90">
<button class="main" onclick="act('/api/turn?deg='+$('tuDeg').value)">Girar izquierda</button>
<button class="main" onclick="act('/api/turn?deg=-'+Math.abs($('tuDeg').value))">Girar derecha</button>
<button class="small" onclick="$('tuDeg').value=90">90</button><button class="small" onclick="$('tuDeg').value=180">180</button><button class="small" onclick="$('tuDeg').value=360">360</button></div>
<div class="row"><label>Angulo real medido (&deg;)</label><input type="number" id="adjDeg" step="0.5"><button onclick="act('/api/adjust_turn?real='+$('adjDeg').value)">Ajustar separacion</button></div>
<div class="row"><label>Giro manual de (&deg;)</label><input type="number" id="trkDeg" value="360"><button onclick="act('/api/cal_track?deg='+$('trkDeg').value)">Calcular separacion con giro a mano</button></div>
</div>

<div class="card">
<h2>Ultimo resultado</h2>
<div class="res" id="last">Sin pruebas todavia.</div>
</div>

<div class="card">
<h2>Parametros de movimiento</h2>
<p>Se guardan en la memoria del robot. Si se detiene antes de llegar, sube el PWM minimo. Si se pasa, baja el PWM maximo o aumenta la zona de frenado.</p>
<div class="row"><label>PWM min</label><input type="number" id="minPwm"><label>PWM max</label><input type="number" id="maxPwm"><label>Frenado (mm)</label><input type="number" id="decelMm" step="1"></div>
<div class="row"><label>Rampa/5ms</label><input type="number" id="rampStep" step="0.5"><label>Tol (mm)</label><input type="number" id="tolMm" step="0.1"><label>Tol (&deg;)</label><input type="number" id="tolDeg" step="0.1"></div>
<div class="row"><label>Correccion recta</label><input type="number" id="kHead" step="0.05"><button class="main" onclick="saveCfg()">Guardar</button></div>
</div>

<p class="sub">Seguridad: el boton START del robot tambien detiene un movimiento. Los movimientos tienen limite de tiempo. No presiones START al encender.</p>
</div>
<div id="stop"><button onclick="act('/api/stop')">STOP</button></div>

<script>
const $=id=>document.getElementById(id);
let first=true,S=null;
async function api(p){
  try{const r=await fetch(p,{cache:'no-store'});const t=await r.text();let j;try{j=JSON.parse(t)}catch(e){j={error:t}}return j}
  catch(e){return{error:'sin conexion con el robot'}}
}
function msg(t,bad){const m=$('msg');m.textContent=t;m.className=bad?'bad':'ok'}
async function act(p){const j=await api(p);if(j.error)msg(j.error,1);else msg(j.ok||'OK');poll();return j}
function mt(side,sign){act('/api/motor?m='+side+'&pwm='+sign*Math.abs($('tpwm').value))}
function inv(){
  act('/api/config?invML='+($('invML').checked?1:0)+'&invMR='+($('invMR').checked?1:0)+'&invEL='+($('invEL').checked?1:0)+'&invER='+($('invER').checked?1:0));
}
function saveCfg(){
  const k=['minPwm','maxPwm','decelMm','rampStep','tolMm','tolDeg','kHead'];
  act('/api/config?'+k.map(x=>x+'='+$(x).value).join('&'));
}
function fill(c){
  ['minPwm','maxPwm','decelMm','rampStep','tolMm','tolDeg','kHead'].forEach(k=>$(k).value=c[k]);
  ['invML','invMR','invEL','invER'].forEach(k=>$(k).checked=!!c[k]);
}
function render(s){
  $('conn').textContent='conectado';$('conn').className='';
  $('cL').textContent=s.cL;$('cR').textContent=s.cR;
  $('vL').textContent=Math.round(s.vL);$('vR').textContent=Math.round(s.vR);
  $('shI').textContent=s.shI;$('shF').textContent=s.shF;$('shD').textContent=s.shD;$('qre').textContent=s.qre;
  $('mode').textContent=s.mode;
  const c=s.cfg;
  $('cpm').textContent=c.cpm>0?c.cpm.toFixed(3):'sin calibrar';
  $('trk').textContent=c.trackMm.toFixed(1)+' mm';
  let w='';
  if(c.cpm>0){w+='1 mm = '+c.cpm.toFixed(3)+' cuentas &middot; 250 mm (1 celda) = '+Math.round(c.cpm*250)+' cuentas';
    if(c.wheelDia>0)w+='<br>Cuentas por vuelta de rueda: '+(c.cpm*Math.PI*c.wheelDia).toFixed(0)}
  else w='<span class="warn">Falta calibrar. Usa el paso 2.</span>';
  $('wheelInfo').innerHTML=w;
  const l=s.last;
  if(l.valid){
    let t='';
    if(l.kind===0){t='<b>Avance</b> pedido: '+l.cmd+' mm &middot; contado: '+(l.progress/c.cpm).toFixed(1)+' mm'}
    else if(l.kind===1){t='<b>Giro</b> pedido: '+l.cmd+'&deg; &middot; contado: '+(l.progress/c.cpm*360/(Math.PI*c.trackMm)).toFixed(1)+'&deg; (separacion '+c.trackMm.toFixed(1)+' mm)'}
    else{t='<b>Motor '+l.side+'</b> PWM '+l.cmd}
    t+='<br>Encoders finales: izq '+l.cL+' &middot; der '+l.cR+' &middot; tiempo '+l.ms+' ms';
    if(l.timeout)t+='<br><span class="warn">Se agoto el tiempo: no llego al objetivo. Sube el PWM minimo.</span>';
    $('last').innerHTML=t;
  }
  if(first){fill(c);first=false}
}
async function poll(){
  const s=await api('/api/status');
  if(s.error){$('conn').textContent='sin conexion';$('conn').className='warn';return}
  S=s;render(s);
}
setInterval(poll,300);poll();
</script>
</body>
</html>
)HTML";

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

// ------------------------------------------------------------------- Wi-Fi
const char *WIFI_SSID = "AUS_KIM_V2";
const char *WIFI_PASS = "AUSKIM2026";
constexpr bool LOWER_WIFI_TX_POWER = true;  // algunas C3 Super Mini conectan mejor asi

WebServer server(80);
Preferences prefs;

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
} cfg;

void loadConfig() {
  prefs.begin("auskim_v2", false);
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
}

// ------------------------------------------------------------------ Encoders
struct Encoder {
  volatile int32_t count;
  volatile uint8_t state;
  uint8_t pinA, pinB;
};

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
void setMotor(bool right, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (right ? cfg.invMR : cfg.invML) pwm = -pwm;
  uint8_t base = right ? 0 : 2;  // derecho: AIN1/AIN2, izquierdo: BIN1/BIN2
  if (pwm >= 0) {
    pwmWrite(base, pwm);
    pwmWrite(base + 1, 0);
  } else {
    pwmWrite(base, 0);
    pwmWrite(base + 1, -pwm);
  }
}

void coastMotors() { for (uint8_t i = 0; i < 4; i++) pwmWrite(i, 0); }
void brakeMotors() { for (uint8_t i = 0; i < 4; i++) pwmWrite(i, 255); }

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

bool startPressedStable() {
  static uint8_t cnt = 0;
  if (digitalRead(PIN_START) == LOW) { if (cnt < 255) cnt++; } else cnt = 0;
  return cnt >= 4;  // ~20 ms
}

// -------------------------------------------------------------- Movimientos
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

String f(float v, int d = 3) { return std::isfinite(v) ? String(v, d) : String("0"); }

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
  s += "}";
  return s;
}

// Velocidad de cada rueda (cuentas por segundo), calculada cada 50 ms en loop().
float speedL = 0, speedR = 0;

void handleStatus() {
  String s = "{";
  s += "\"mode\":\"" + String(modeName()) + "\"";
  s += ",\"cL\":" + String(countL());
  s += ",\"cR\":" + String(countR());
  s += ",\"vL\":" + f(speedL, 0);
  s += ",\"vR\":" + f(speedR, 0);
  s += ",\"shI\":" + String(readMedian5(PIN_SH_I));
  s += ",\"shF\":" + String(readMedian5(PIN_SH_F));
  s += ",\"shD\":" + String(readMedian5(PIN_SH_D));
  s += ",\"qre\":" + String(readMedian5(PIN_QRE));
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
  if (cfg.maxPwm < cfg.minPwm) cfg.maxPwm = cfg.minPwm;
  zeroEncoders();  // un cambio de signo de encoder deja las cuentas viejas sin sentido
  saveConfig();
  sendOk("Guardado");
}

// ------------------------------------------------------------------- Setup
void setup() {
  // Lo primero: pines del driver en estado seguro.
  setupMotors();
  coastMotors();

  Serial.begin(115200);
  pinMode(PIN_START, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SH_I, ADC_11db);
  analogSetPinAttenuation(PIN_SH_F, ADC_11db);
  analogSetPinAttenuation(PIN_SH_D, ADC_11db);
  analogSetPinAttenuation(PIN_QRE, ADC_11db);

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
  server.begin();

  Serial.println(F("AUS_KIM V2 calibracion lista"));
  Serial.print(F("Wi-Fi: "));
  Serial.print(WIFI_SSID);
  Serial.print(F("  panel: http://"));
  Serial.println(WiFi.softAPIP());
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

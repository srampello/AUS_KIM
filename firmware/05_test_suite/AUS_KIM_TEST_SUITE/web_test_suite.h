#pragma once
#include <Arduino.h>

// Interfaz RMP almacenada en memoria de programa (PROGMEM).
// Archivo fuente. No modificar las copias de firmware/06_race.
// Se incluye desde el .ino, sin LittleFS ni conexion a Internet.

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
.hand-tabs{display:grid;grid-template-columns:1fr 1fr;gap:7px;margin:12px 0 15px;padding:5px;background:#11101a;border:1px solid #403153;border-radius:13px}
.hand-tabs button{font-size:12px;padding:12px 7px;background:#15121e;border-color:transparent;color:#bbb2cb}
.hand-tabs button.active{background:linear-gradient(180deg,#8b5cf6,#6d28d9);color:white;box-shadow:0 5px 18px #6d28d944}
.hand-tabs button:disabled{opacity:.6;cursor:not-allowed}
[hidden]{display:none!important}
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
    <button class="tabbtn" id="tabBtnWall" onclick="showTab('wall')">PID pared lateral</button>
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
        <h2>Control PID lateral</h2>
        <div class="hand-tabs" role="tablist" aria-label="Seleccion de pared para PID">
          <button type="button" class="active" id="wallHandRight" onclick="selectWallHand('WALL')">PARED DERECHA</button>
          <button type="button" id="wallHandLeft" onclick="selectWallHand('WALL_LEFT')">PARED IZQUIERDA</button>
        </div>
        <div class="status" id="wallState">DETENIDO</div>
        <button class="full start" onclick="startAutonomous(selectedWallMode)">INICIAR PID</button>
        <button class="full stop" onclick="stopAll()">STOP</button>

        <div class="metric-grid" style="margin-top:10px">
          <div class="metric"><div class="label">Lateral de referencia</div><div class="value" id="wLR">0</div></div>
          <div class="metric"><div class="label">Objetivo</div><div class="value" id="wTarget">2400</div></div>
          <div class="metric"><div class="label">Error</div><div class="value" id="wError">0</div></div>
          <div class="metric"><div class="label">Correccion</div><div class="value" id="wCorrection">0</div></div>
          <div class="metric"><div class="label">Motor izq.</div><div class="value" id="wMotorL">0</div></div>
          <div class="metric"><div class="label">Motor der.</div><div class="value" id="wMotorR">0</div></div>
        </div>
      </div>

      <div class="card">
        <h2>Parametros PID</h2>
        <div id="wallRightControls">
          <div class="field"><span>Kp derecha</span><input class="cfg" id="kp" type="number" step="0.01"></div>
          <div class="field"><span>Ki derecha</span><input class="cfg" id="ki" type="number" step="0.001"></div>
          <div class="field"><span>Kd derecha</span><input class="cfg" id="kd" type="number" step="0.01"></div>
          <div class="field"><span>Objetivo ADC derecha</span><input class="cfg" id="targetRightAdc" type="number" step="1"></div>
        </div>
        <div id="wallLeftControls" hidden>
          <div class="field"><span>Kp izquierda</span><input class="cfg" id="leftKp" type="number" step="0.01"></div>
          <div class="field"><span>Ki izquierda</span><input class="cfg" id="leftKi" type="number" step="0.001"></div>
          <div class="field"><span>Kd izquierda</span><input class="cfg" id="leftKd" type="number" step="0.01"></div>
          <div class="field"><span>Objetivo ADC izquierda</span><input class="cfg" id="targetLeftAdc" type="number" step="1"></div>
        </div>
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
          <div class="metric"><div class="label">Lateral de referencia</div><div class="value" id="wLR2">0</div></div>
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
        <h2>Maze Solver · elegir pared</h2>
        <div class="hand-tabs" role="tablist" aria-label="Seleccion de pared para laberinto">
          <button type="button" class="active" id="mazeHandRight" onclick="selectMazeHand('MAZE')">PARED DERECHA</button>
          <button type="button" id="mazeHandLeft" onclick="selectMazeHand('MAZE_LEFT')">PARED IZQUIERDA</button>
        </div>
        <div class="status" id="mazeState">DETENIDO</div>
        <div class="hint">Ultima detencion: <b id="mazeStopReason">NINGUNA</b></div>
        <button class="full start" onclick="startAutonomous(selectedMazeMode)">INICIAR LABERINTO</button>
        <button class="full stop" onclick="stopAll()">STOP</button>

        <div class="flags">
          <div class="flag">Frente<br><b id="mFrontFlag">LIBRE</b></div>
          <div class="flag">Derecha<br><b id="mRightFlag">PARED</b></div>
          <div class="flag">Izquierda (lectura)<b id="mLeftFlag">PARED</b></div>
        </div>

        <div class="hint" id="mazeHandDescription">
          Pared derecha: PWM 165; apertura derecha confirmada en ADC menor a 1750. Frena 500 ms, intenta avanzar 15 cm por tiempo y gira derecha. Frente desde ADC 1650: si no hay salida derecha, gira izquierda. Si el frente aparece durante el avance, frena y toma la salida derecha sin completar el recorrido.
        </div>
      </div>

      <div class="card">
        <h2>Deteccion y frenado</h2>
        <div class="field"><span>STOP frontal ADC (≥)</span><input class="cfg" id="frontWallAdc" type="number" step="1"></div>
        <div class="field"><span>Confirmacion segundo frontal ADC</span><input class="cfg" id="frontConfirmAdc" type="number" step="1"></div>
        <div class="field"><span>Apertura derecha ADC</span><input class="cfg" id="rightOpenAdc" type="number" step="1"></div>
        <div class="field"><span>Apertura izquierda ADC</span><input class="cfg" id="leftOpenAdc" type="number" step="1"></div>
        <div class="field"><span>Espera antes de avanzar (ms)</span><input class="cfg" id="openingWaitMs" type="number" min="0" max="1000" step="10"></div>
        <div class="field"><span>Avance hacia apertura (ms; objetivo ~15 cm)</span><input class="cfg" id="openingAdvanceMs" type="number" min="30" max="2500" step="10"></div>
        <div class="hint">500 ms es un punto de partida, NO equivale necesariamente a 15 cm. Calibrar con el robot y piso reales.</div>
      </div>

      <div class="card">
        <h2>Giros temporizados (sin encoders)</h2>
        <div class="field"><span>PWM avance Maze</span><input class="cfg" id="basePwm" type="number" min="155" max="255" step="1"></div>
        <div class="field"><span>PWM giro</span><input class="cfg" id="turnPwm" type="number" min="120" max="255" step="1"></div>
        <div class="field"><span>Giro 90° derecha (ms)</span><input class="cfg" id="turn90RightMs" type="number" min="50" max="1200" step="10"></div>
        <div class="field"><span>Giro 90° izquierda (ms)</span><input class="cfg" id="turn90LeftMs" type="number" min="50" max="1200" step="10"></div>
        <div class="field"><span>Espera antes de girar ante pared frontal (ms)</span><input class="cfg" id="decisionWaitMs" type="number" min="0" max="1500" step="10"></div>
        <div class="hint">160 ms por giro es un valor inicial. Calibrar por separado izquierda y derecha sobre el piso real.</div>
        <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>
      </div>

      <div class="card">
        <h2>Compensacion de motores</h2>
        <div class="hint">Si se desvía durante el avance de la apertura, ajustá cada PWM por separado. También se aplican a los tramos sin pared lateral.</div>
        <div class="field"><span>PWM motor izquierdo</span><input class="cfg" id="openingLeftPwm" type="number" min="155" max="255" step="1"></div>
        <div class="field"><span>PWM motor derecho</span><input class="cfg" id="openingRightPwm" type="number" min="155" max="255" step="1"></div>
        <button class="full" onclick="applyConfig()">APLICAR PARAMETROS</button>
      </div>

      <div class="card">
        <h2>Calibrar SIN encoders</h2>
        <div class="hint">Con el robot en un espacio despejado y apoyado en el piso: aplica los parametros de arriba y prueba cada maniobra por separado. El avance hacia la apertura y ambos giros se detienen automaticamente al terminar su tiempo.</div>
        <button class="full start" onclick="testTimedMove('FWD')">PROBAR AVANCE CORTO</button>
        <button class="full" onclick="testTimedMove('L90')">PROBAR GIRO 90° IZQUIERDA</button>
        <button class="full" onclick="testTimedMove('R90')">PROBAR GIRO 90° DERECHA</button>
        <button class="full stop" onclick="stopAll()">STOP</button>
        <div class="hint">Prueba activa: <b id="timedTestState">NINGUNA</b>. Repetí ajustando ms y PWM hasta medir cerca de 15 cm de avance y 90° reales.</div>
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
        <h2>Diagnostico de encoders (no intervienen en Maze)</h2>
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
let selectedMazeMode='MAZE';
let selectedWallMode='WALL';
let robotRunning=false;
let firstLoad=true;
let activeManualMotor=null;

function panelName(tab){
  return tab.charAt(0).toUpperCase()+tab.slice(1);
}

function selectMazeHand(mode){
  if(robotRunning)return; // Cambiar estrategia solo detenido.
  selectedMazeMode=mode;
  document.getElementById('mazeHandRight').classList.toggle('active',mode==='MAZE');
  document.getElementById('mazeHandLeft').classList.toggle('active',mode==='MAZE_LEFT');
  document.getElementById('mazeHandDescription').textContent=mode==='MAZE'
    ? 'Pared derecha: apertura ADC inferior a 1750 y prioridad derecha. Freno frontal ADC 1650; sin salida derecha gira izquierda. Apertura: 500 ms, avance objetivo 15 cm por tiempo, 300 ms y giro derecha.'
    : 'Pared izquierda (solo para pruebas): apertura izquierda; frente bloqueado gira derecha. Avance a apertura con tiempo calibrable, no usa encoders.';
}
function selectWallHand(mode){
  if(robotRunning)return;
  selectedWallMode=mode;
  document.getElementById('wallHandRight').classList.toggle('active',mode==='WALL');
  document.getElementById('wallHandLeft').classList.toggle('active',mode==='WALL_LEFT');
  document.getElementById('wallRightControls').hidden=mode!=='WALL';
  document.getElementById('wallLeftControls').hidden=mode!=='WALL_LEFT';
}
async function showTab(tab){
  activeTab=tab;

  ['test','wall','maze','encoder'].forEach(t=>{
    document.getElementById('panel'+panelName(t)).classList.toggle('active',t===tab);
    document.getElementById('tabBtn'+panelName(t)).classList.toggle('active',t===tab);
  });

  const mode = tab==='test' ? 'TEST' : (tab==='wall' ? selectedWallMode : (tab==='maze' ? selectedMazeMode : 'ENCODER'));
  await fetch('/api/mode?mode='+mode,{cache:'no-store'});
  await updateStatus();
}

async function startAutonomous(mode){
  await fetch('/api/mode?mode='+mode,{cache:'no-store'});
  await fetch('/api/run?state=1',{cache:'no-store'});
  await updateStatus();
}

async function testTimedMove(action){
  if(robotRunning){alert('Detene el robot antes de iniciar una prueba');return;}
  try{
    await applyConfig();
    const mode=await fetch('/api/mode?mode=TEST',{cache:'no-store'});
    if(!mode.ok)throw new Error(await mode.text());
    const r=await fetch('/api/timed_test?action='+encodeURIComponent(action),{cache:'no-store'});
    if(!r.ok)throw new Error(await r.text());
    await updateStatus();
  }catch(e){alert('No se pudo iniciar la prueba: '+e.message);}
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
    'kp','ki','kd','targetRightAdc','leftKp','leftKi','leftKd','targetLeftAdc','basePwm','maxCorrection',
    'frontWallAdc','frontConfirmAdc',
    'rightOpenAdc','leftOpenAdc','openingWaitMs','openingAdvanceMs','openingLeftPwm','openingRightPwm',
    'turnPwm','turn90RightMs','turn90LeftMs','decisionWaitMs'
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
    robotRunning=d.running===true;
    ['wallHandRight','wallHandLeft','mazeHandRight','mazeHandLeft'].forEach(id=>{
      document.getElementById(id).disabled=robotRunning;
    });

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
    document.getElementById('wallState').textContent=(d.mode==='WALL'||d.mode==='WALL_LEFT')&&d.running ? 'SIGUIENDO PARED' : 'DETENIDO';
    document.getElementById('wLR').textContent=selectedWallMode==='WALL_LEFT' ? d.sensor.ll : d.sensor.lr;
    document.getElementById('wTarget').textContent=selectedWallMode==='WALL_LEFT' ? d.config.targetLeftAdc : d.config.targetRightAdc;
    document.getElementById('wError').textContent=d.pid.error.toFixed(1);
    document.getElementById('wCorrection').textContent=d.pid.correction.toFixed(1);
    document.getElementById('wMotorL').textContent=d.motor.left;
    document.getElementById('wMotorR').textContent=d.motor.right;
    setSensor('w',d);
    document.getElementById('wLR2').textContent=selectedWallMode==='WALL_LEFT' ? d.sensor.ll : d.sensor.lr;

    // MAZE
    document.getElementById('timedTestState').textContent=d.timedTestAction||'NINGUNA';
    document.getElementById('mazeState').textContent=(d.mode==='MAZE'||d.mode==='MAZE_LEFT') ? d.state : 'DETENIDO';
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

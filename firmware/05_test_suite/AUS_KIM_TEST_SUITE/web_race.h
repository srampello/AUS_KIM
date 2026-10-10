#pragma once
#include <Arduino.h>

// Interfaz RMP almacenada en memoria de programa (PROGMEM).
// Archivo fuente. No modificar las copias de firmware/06_race.
// Se incluye desde el .ino, sin LittleFS ni conexion a Internet.

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
.raceRule{width:100%;margin:0 0 26px;padding:13px 18px;border:1px solid #6d4da0;border-radius:13px;background:#20152f;color:#d8c6ff;letter-spacing:.13em;font-size:12px;font-weight:900}
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
  <div class="raceRule">ESTRATEGIA · SIEMPRE PARED DERECHA</div>
  <p class="stateDetail" style="margin:-14px 0 26px;max-width:390px">CARRERA AUTONOMA: el robot sigue si se corta el Wi-Fi. STOP local con BOOT/GPIO0. Tiempo maximo: 120 segundos.</p>
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
    await req('/api/run?state=1&autonomous=1');
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
    el('detail').textContent=racing
      ? data.state + (data.autonomousRace ? ' · Corre sin Wi-Fi' : ' · Dependiente de Wi-Fi')
      : (data.stopReason&&data.stopReason!=='NINGUNA'?data.stopReason:'Toca el logo para iniciar el laberinto');
    el('dot').classList.toggle('live',racing);
    el('launch').classList.toggle('running',racing);
    el('launchText').textContent=racing?'CORRIENDO':'LARGAR';
  }catch(_){
    connected=false;racing=false;el('status').textContent='SIN CONEXION';
    el('detail').textContent='Sin conexion: la carrera puede seguir en el robot. Usar STOP local BOOT/GPIO0 o cortar alimentacion.';
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

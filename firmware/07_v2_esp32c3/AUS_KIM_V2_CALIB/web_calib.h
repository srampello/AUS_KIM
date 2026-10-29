#pragma once
#include <Arduino.h>

// Pagina web de calibracion AUS_KIM V2 (HTML + CSS + JavaScript).
// Se guarda en memoria de programa (PROGMEM): no usa Internet ni LittleFS.
// Se puede editar este archivo sin tocar la logica del robot.

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

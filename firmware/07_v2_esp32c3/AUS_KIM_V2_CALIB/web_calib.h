#pragma once
#include <Arduino.h>

// Pagina web de pruebas AUS_KIM V2 (HTML + CSS + JavaScript): pestanas Motores y Sensores.
// Se guarda en memoria de programa (PROGMEM): no usa Internet ni LittleFS.
// Se puede editar este archivo sin tocar la logica del robot.

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM V2 | Pruebas</title>
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
.tabs{display:flex;gap:8px;margin:10px 0 14px}
.tabs button{flex:1;padding:13px;font-size:14px}
.tabs button.active{background:linear-gradient(180deg,var(--violet),var(--violet2));border-color:#9f7aea}
.panel{display:none}
.panel.active{display:block}
.bar{position:relative;height:14px;background:#0c0c11;border:1px solid var(--border);border-radius:8px;overflow:hidden;margin:8px 0}
.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--violet2),var(--violet))}
.bar u{position:absolute;top:0;bottom:0;width:2px;background:var(--warn)}
.badge{display:inline-block;padding:3px 9px;border-radius:999px;font-size:11.5px;font-weight:800;letter-spacing:.06em;margin-left:6px}
.badge.on{background:rgba(239,68,68,.18);color:#fca5a5;border:1px solid var(--bad)}
.badge.off{background:rgba(34,197,94,.15);color:#86efac;border:1px solid var(--good)}
.badge.na{background:#1b1b24;color:var(--muted);border:1px solid var(--border)}
.cross{display:grid;grid-template-columns:76px 96px 76px;grid-template-rows:46px 72px;gap:6px;justify-content:center;margin:10px 0}
.cross div{display:flex;align-items:center;justify-content:center;border-radius:10px;font-weight:800;font-size:13px;background:#0c0c11;border:1px solid var(--border);color:var(--muted)}
.cross .w{background:rgba(239,68,68,.25);border-color:var(--bad);color:#fff}
.cross .rb{background:linear-gradient(180deg,var(--violet),var(--violet2));color:#fff;border-color:#9f7aea}
.big{font-size:30px;font-weight:800;font-variant-numeric:tabular-nums}
textarea{width:100%;height:120px;background:#0c0c11;color:var(--text);border:1px solid var(--border);border-radius:10px;padding:8px;font:12px ui-monospace,Menlo,Consolas,monospace}
</style>
</head>
<body>
<div class="wrap">
<h1>AUS_KIM V2</h1>
<div class="sub">Pruebas de motores y sensores &middot; <span id="conn" class="warn">conectando...</span></div>
<div class="tabs">
<button id="tbMot" class="active" onclick="showTab('mot')">Motores</button>
<button id="tbSen" onclick="showTab('sen')">Sensores</button>
</div>
<div id="msg"></div>

<!-- ============================ PESTANA MOTORES ============================ -->
<div id="tabMot" class="panel active">

<div class="card">
<h2>Estado en vivo</h2>
<div class="grid">
<div class="kv"><small>Encoder izq</small><b id="cL">-</b></div>
<div class="kv"><small>Encoder der</small><b id="cR">-</b></div>
<div class="kv"><small>Vel izq (c/s)</small><b id="vL">-</b></div>
<div class="kv"><small>Vel der (c/s)</small><b id="vR">-</b></div>
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
<p>El robot avanza la distancia pedida usando los encoders. Luego <b>mide con regla lo que realmente avanzo</b>, escribelo abajo y ajusta. Repite hasta que coincida. La celda mide 250 mm o 180 mm, segun la sede.</p>
<div class="row"><label>Distancia (mm)</label><input type="number" id="mvMm" value="250">
<button class="main" onclick="act('/api/move?mm='+$('mvMm').value)">Avanzar</button>
<button class="small" onclick="act('/api/move?mm=-'+Math.abs($('mvMm').value))">Retroceder</button></div>
<div class="row">
<button class="small" onclick="$('mvMm').value=250">1 celda (250)</button>
<button class="small" onclick="$('mvMm').value=125">media (125)</button>
<button class="small" onclick="$('mvMm').value=180">1 celda (180)</button>
<button class="small" onclick="$('mvMm').value=90">media (90)</button></div>
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

</div>

<!-- ============================ PESTANA SENSORES ============================ -->
<div id="tabSen" class="panel">

<div class="card">
<h2>Como ve el robot las paredes</h2>
<p>Pared = lectura del Sharp mayor o igual al umbral (el Sharp da un valor <b>mas alto cuanto mas cerca</b> esta la pared). Con el robot en el centro de una celda, esto es lo que decide la mano derecha.</p>
<div class="cross">
<span></span><div id="xF">F</div><span></span>
<div id="xI">I</div><div class="rb">ROBOT</div><div id="xD">D</div>
</div>
<div class="res" id="decision">-</div>
</div>

<div id="senCards"></div>

<div class="card">
<h2>Sensor de piso QRE <span id="sQ" class="badge na">-</span></h2>
<div class="row"><span class="big" id="vQ">-</span><small style="color:var(--muted)">ADC (0-4095)</small></div>
<p>Apoya el sensor sobre el piso <b>negro</b> y toca "Capturar negro". Despues sobre el <b>blanco</b> (celda de llegada) y toca "Capturar blanco". Con los dos valores decide solo si ve blanco o negro. Si quedan muy parecidos, acerca o aleja el sensor del piso.</p>
<div class="row"><button class="main" onclick="act('/api/cal_qre?c=black').then(()=>firstSen=true)">Capturar negro</button>
<button class="main" onclick="act('/api/cal_qre?c=white').then(()=>firstSen=true)">Capturar blanco</button>
<button class="small" onclick="act('/api/cal_qre?c=clear')">Borrar</button></div>
<div class="res" id="qreInfo">-</div>
</div>

<div class="card">
<h2>Boton START (GPIO9)</h2>
<div class="row"><span class="big" id="vS">-</span><small style="color:var(--muted)">presionalo para probarlo (no lo mantengas al encender)</small></div>
</div>

<div class="card">
<h2>Registro de lecturas por distancia</h2>
<p>Pone una pared frente a los sensores a una distancia conocida (por ejemplo 40, 60, 80, 100, 150, 200 mm), escribila y toca "Anotar". Despues copia la tabla: sirve para pasar de valor ADC a milimetros. Este registro queda solo en la pagina; si la recargas se borra.</p>
<div class="row"><label>Distancia (mm)</label><input type="number" id="lgMm" value="60"><button class="main" onclick="addRow()">Anotar lectura</button>
<button onclick="copyLog()">Copiar</button><button class="small" onclick="clearLog()">Borrar</button></div>
<textarea id="lgTxt" readonly></textarea>
</div>

</div>

<p class="sub">Seguridad: el boton START del robot tambien detiene un movimiento. Los movimientos tienen limite de tiempo. No presiones START al encender. Al cambiar de pestana el robot se detiene.</p>
</div>
<div id="stop"><button onclick="act('/api/stop')">STOP</button></div>

<script>
const $=id=>document.getElementById(id);
let first=true,firstSen=true,S=null,tab='mot';
async function api(p){
  try{const r=await fetch(p,{cache:'no-store'});const t=await r.text();let j;try{j=JSON.parse(t)}catch(e){j={error:t}}return j}
  catch(e){return{error:'sin conexion con el robot'}}
}
function msg(t,bad){const m=$('msg');m.textContent=t;m.className=bad?'bad':'ok'}
async function act(p){const j=await api(p);if(j.error)msg(j.error,1);else msg(j.ok||'OK');poll();return j}
function showTab(t){
  if(t!==tab)api('/api/stop');
  tab=t;
  $('tabMot').className='panel'+(t==='mot'?' active':'');
  $('tabSen').className='panel'+(t==='sen'?' active':'');
  $('tbMot').className=t==='mot'?'active':'';
  $('tbSen').className=t==='sen'?'active':'';
}

/* ---------- Motores ---------- */
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
function renderMot(s){
  $('cL').textContent=s.cL;$('cR').textContent=s.cR;
  $('vL').textContent=Math.round(s.vL);$('vR').textContent=Math.round(s.vR);
  $('mode').textContent=s.mode;
  const c=s.cfg;
  $('cpm').textContent=c.cpm>0?c.cpm.toFixed(3):'sin calibrar';
  $('trk').textContent=c.trackMm.toFixed(1)+' mm';
  let w='';
  if(c.cpm>0){w+='1 mm = '+c.cpm.toFixed(3)+' cuentas &middot; 250 mm = '+Math.round(c.cpm*250)+' cuentas &middot; 180 mm = '+Math.round(c.cpm*180)+' cuentas';
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

/* ---------- Sensores ---------- */
const SEN=[['I','Izquierda'],['F','Frente'],['D','Derecha']];
$('senCards').innerHTML=SEN.map(function(a){var k=a[0],n=a[1];return ''+
 '<div class="card"><h2>Sharp '+n+' <span id="s'+k+'" class="badge na">-</span></h2>'+
 '<div class="row"><span class="big" id="v'+k+'">-</span><small style="color:var(--muted)">ADC (0-4095)</small></div>'+
 '<div class="bar"><i id="b'+k+'" style="width:0"></i><u id="m'+k+'" style="left:0"></u></div>'+
 '<div class="row"><label>Umbral de pared (ADC &ge;)</label><input type="number" id="t'+k+'" min="1" max="4095">'+
 '<button onclick="setThr(\''+k+'\')">Guardar</button>'+
 '<button class="main" onclick="useCur(\''+k+'\')">Usar valor actual</button></div></div>';}).join('');
function setThr(k){act('/api/thr?s='+k+'&v='+$('t'+k).value).then(function(){firstSen=true})}
function useCur(k){act('/api/thr?s='+k+'&cur=1').then(function(){firstSen=true})}
function badge(el,txt,cls){el.textContent=txt;el.className='badge '+cls}
function renderSen(s){
  const c=s.cfg;
  const W={I:s.wI,F:s.wF,D:s.wD},V={I:s.shI,F:s.shF,D:s.shD},T={I:c.thrI,F:c.thrF,D:c.thrD};
  SEN.forEach(function(a){
    const k=a[0];
    $('v'+k).textContent=V[k];
    $('b'+k).style.width=Math.min(100,V[k]/40.95)+'%';
    $('m'+k).style.left=Math.min(100,T[k]/40.95)+'%';
    badge($('s'+k),W[k]?'PARED':'LIBRE',W[k]?'on':'off');
    $('x'+k).className=W[k]?'w':'';
  });
  $('decision').innerHTML='I F D = <b>'+(W.I?1:0)+' '+(W.F?1:0)+' '+(W.D?1:0)+'</b> &middot; '+
    (!W.D?'derecha libre: giraria a la derecha':(!W.F?'sigue recto':(!W.I?'gira a la izquierda':'callejon: media vuelta')));
  $('vQ').textContent=s.qre;
  if(s.qreWhite===null)badge($('sQ'),'SIN CALIBRAR','na');
  else badge($('sQ'),s.qreWhite?'BLANCO':'NEGRO',s.qreWhite?'off':'on');
  $('qreInfo').innerHTML='Negro capturado: '+(c.qreBlack>=0?c.qreBlack:'-')+' &middot; Blanco capturado: '+(c.qreWhite>=0?c.qreWhite:'-');
  $('vS').textContent=s.start?'PRESIONADO':'libre';
  if(firstSen){SEN.forEach(function(a){$('t'+a[0]).value=T[a[0]]});firstSen=false}
}
/* registro de lecturas */
let LOG=[];
function logText(){
  return 'dist_mm\tSharp_I\tSharp_F\tSharp_D\tQRE\n'+LOG.map(function(r){return r.join('\t')}).join('\n');
}
function addRow(){
  if(!S){msg('Sin lecturas todavia',1);return}
  LOG.push([$('lgMm').value,S.shI,S.shF,S.shD,S.qre]);
  $('lgTxt').value=logText();
}
function clearLog(){LOG=[];$('lgTxt').value=''}
function copyLog(){
  const t=$('lgTxt');t.value=logText();t.focus();t.select();
  let ok=false;try{ok=document.execCommand('copy')}catch(e){}
  msg(ok?'Tabla copiada':'Seleccionada: mantene presionado y copia',!ok);
}

/* ---------- comun ---------- */
function render(s){
  $('conn').textContent='conectado';$('conn').className='';
  renderMot(s);renderSen(s);
}
async function poll(){
  const s=await api('/api/status');
  if(s.error){$('conn').textContent='sin conexion';$('conn').className='warn';return}
  S=s;render(s);
}
setInterval(poll,250);poll();
</script>
</body>
</html>
)HTML";

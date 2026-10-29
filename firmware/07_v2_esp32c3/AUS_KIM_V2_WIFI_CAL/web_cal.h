#pragma once
#include <Arduino.h>

// Interfaz de calibracion de movimiento AUS_KIM V2. Vive en PROGMEM:
// funciona sin Internet y sin LittleFS.

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>AUS_KIM V2 | Calibracion</title>
<style>
:root{--bg:#050507;--panel:#101015;--panel2:#15151c;--border:#292934;--text:#f7f7fb;--muted:#9696a8;
--violet:#8b5cf6;--violet2:#6d28d9;--good:#22c55e;--danger:#ef4444;--warn:#f59e0b}
*{box-sizing:border-box}
html{color-scheme:dark}
body{margin:0;min-height:100vh;padding-bottom:84px;font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;
background:radial-gradient(circle at 8% 0%,rgba(139,92,246,.16),transparent 28rem),#050507;color:var(--text)}
button,input,select{font:inherit}
.wrap{max-width:980px;margin:auto;padding:18px 14px}
h1{font-size:20px;margin:0}
h2{font-size:13px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:0 0 12px}
.top{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:16px;padding-bottom:14px;border-bottom:1px solid rgba(255,255,255,.07)}
.chip{padding:5px 11px;border-radius:99px;font-size:12px;border:1px solid var(--border);background:var(--panel2)}
.chip.ok{color:var(--good);border-color:rgba(34,197,94,.4)}
.chip.bad{color:var(--danger);border-color:rgba(239,68,68,.4)}
.chip.run{color:var(--warn);border-color:rgba(245,158,11,.4)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px}
.card{background:var(--panel);border:1px solid var(--border);border-radius:14px;padding:16px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
.row label{font-size:12px;color:var(--muted);min-width:96px}
input,select{background:#0c0c11;color:var(--text);border:1px solid var(--border);border-radius:8px;padding:9px 10px;width:100px}
button{background:var(--violet2);color:#fff;border:0;border-radius:9px;padding:10px 14px;cursor:pointer}
button:active{background:var(--violet)}
button.alt{background:var(--panel2);border:1px solid var(--border)}
button.warn{background:#92400e}
.big{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums}
.kv{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid rgba(255,255,255,.05);font-size:14px}
.kv span:first-child{color:var(--muted)}
.kv b{font-variant-numeric:tabular-nums}
.two{display:grid;grid-template-columns:1fr 1fr;gap:10px}
pre{background:#0c0c11;border:1px solid var(--border);border-radius:8px;padding:10px;font-size:12px;overflow:auto;margin:8px 0 0}
.note{font-size:12px;color:var(--muted);margin:8px 0 0;line-height:1.45}
#stop{position:fixed;left:0;right:0;bottom:0;padding:12px 14px;background:rgba(5,5,7,.94);border-top:1px solid var(--border)}
#stop button{width:100%;background:var(--danger);font-size:18px;font-weight:700;padding:14px}
#toast{position:fixed;top:12px;left:50%;transform:translateX(-50%);background:#1d1d27;border:1px solid var(--border);border-radius:10px;padding:10px 16px;font-size:14px;display:none;z-index:9}
</style>
</head>
<body>
<div id="toast"></div>
<div class="wrap">
  <div class="top">
    <h1>AUS_KIM V2 &middot; Calibraci&oacute;n de movimiento</h1>
    <span id="conn" class="chip bad">sin conexi&oacute;n</span>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Encoders</h2>
      <div class="two">
        <div><div class="note">Izquierdo</div><div class="big" id="encL">0</div><div class="note" id="spdL">0 c/s</div></div>
        <div><div class="note">Derecho</div><div class="big" id="encR">0</div><div class="note" id="spdR">0 c/s</div></div>
      </div>
      <div class="row"><button class="alt" onclick="go('/api/reset')">Poner en cero</button></div>
      <p class="note">Estado: <b id="motion">-</b>. Si una rueda cuenta al rev&eacute;s, cambiar <code>ENC_SIGN_*</code> en el firmware.</p>
    </div>

    <div class="card">
      <h2>Sensores</h2>
      <div class="kv"><span>Sharp izquierdo (SH_I)</span><b id="shI">-</b></div>
      <div class="kv"><span>Sharp frontal (SH_F)</span><b id="shF">-</b></div>
      <div class="kv"><span>Sharp derecho (SH_D)</span><b id="shD">-</b></div>
      <div class="kv"><span>Piso QRE</span><b id="qre">-</b></div>
      <div class="kv"><span>Bot&oacute;n START</span><b id="start">-</b></div>
    </div>

    <div class="card">
      <h2>Avanzar</h2>
      <div class="row"><label>Distancia (mm)</label><input id="mm" type="number" value="250" step="10"></div>
      <div class="row">
        <button onclick="move(1)">Avanzar</button>
        <button class="alt" onclick="move(-1)">Retroceder</button>
      </div>
      <div class="row">
        <button class="alt" onclick="$('mm').value=100">100</button>
        <button class="alt" onclick="$('mm').value=180">180</button>
        <button class="alt" onclick="$('mm').value=250">250</button>
        <button class="alt" onclick="$('mm').value=500">500</button>
      </div>
      <p class="note">La celda del reglamento es de 250 mm.</p>
    </div>

    <div class="card">
      <h2>Girar sobre su eje</h2>
      <div class="row"><label>&Aacute;ngulo (&deg;)</label><input id="deg" type="number" value="90" step="5"></div>
      <div class="row">
        <button onclick="turn(1)">Derecha</button>
        <button class="alt" onclick="turn(-1)">Izquierda</button>
      </div>
      <div class="row">
        <button class="alt" onclick="$('deg').value=90">90</button>
        <button class="alt" onclick="$('deg').value=180">180</button>
        <button class="alt" onclick="$('deg').value=360">360</button>
      </div>
      <p class="note">Derecha = sentido horario visto desde arriba.</p>
    </div>
  </div>

  <div class="grid" style="margin-top:14px">
    <div class="card">
      <h2>&Uacute;ltimo resultado</h2>
      <div class="kv"><span>Tipo / orden</span><b id="rType">-</b></div>
      <div class="kv"><span>Cuentas objetivo L / R</span><b id="rCmd">-</b></div>
      <div class="kv"><span>Cuentas reales L / R</span><b id="rGot">-</b></div>
      <div class="kv"><span>Avance real L / R (mm)</span><b id="rMm">-</b></div>
      <div class="kv"><span>Giro estimado (&deg;)</span><b id="rDeg">-</b></div>
      <div class="kv"><span>Tiempo / motivo</span><b id="rEnd">-</b></div>
      <p class="note">Med&iacute; en pista lo que realmente avanz&oacute; o gir&oacute; y corrig&eacute; con estos botones:</p>
      <div class="row"><label>Avance real (mm)</label><input id="aMm" type="number" step="0.5">
        <button class="alt" onclick="go('/api/cal?what=fix_move&actual='+$('aMm').value)">Corregir cuentas/mm</button></div>
      <div class="row"><label>Giro real (&deg;)</label><input id="aDeg" type="number" step="0.5">
        <button class="alt" onclick="go('/api/cal?what=fix_turn&actual='+$('aDeg').value)">Corregir separaci&oacute;n</button></div>
      <p class="note">Corregir primero cuentas/mm con un avance y despu&eacute;s la separaci&oacute;n con un giro.</p>
    </div>

    <div class="card">
      <h2>Par&aacute;metros</h2>
      <div class="row"><label>Cuentas por mm</label><input id="p_cpm" type="number" step="0.01"></div>
      <div class="row"><label>Separaci&oacute;n (mm)</label><input id="p_track" type="number" step="0.1"></div>
      <div class="row"><label>Kp (PWM/mm)</label><input id="p_kp" type="number" step="0.5"></div>
      <div class="row"><label>PWM m&aacute;ximo</label><input id="p_pwmMax" type="number"></div>
      <div class="row"><label>PWM m&iacute;nimo</label><input id="p_pwmMin" type="number"></div>
      <div class="row"><label>Tolerancia (mm)</label><input id="p_tol" type="number" step="0.5"></div>
      <div class="row"><label>Rampa (PWM/5ms)</label><input id="p_accel" type="number"></div>
      <div class="row"><label>Freno frontal (ADC)</label><input id="p_frontStop" type="number"></div>
      <div class="row"><button onclick="applyCfg()">Aplicar</button><button class="alt" onclick="go('/api/save')">Guardar en memoria</button></div>
      <p class="note" id="calState"></p>
      <pre id="code">-</pre>
    </div>

    <div class="card">
      <h2>Calibraci&oacute;n a mano</h2>
      <p class="note">1) Poner en cero. 2) Empujar el robot en l&iacute;nea recta una distancia medida. 3) Ingresarla.</p>
      <div class="row"><label>Distancia (mm)</label><input id="cMm" type="number" value="500">
        <button onclick="go('/api/cal?what=dist&mm='+$('cMm').value)">Calcular cuentas/mm</button></div>
      <p class="note">Alternativa: poner en cero, girar UNA rueda una vuelta completa e ingresar su di&aacute;metro.</p>
      <div class="row"><label>Di&aacute;metro rueda (mm)</label><input id="cD" type="number" step="0.1">
        <button class="alt" onclick="go('/api/cal?what=wheel&diam='+$('cD').value)">Calcular</button></div>
      <p class="note">Para la separaci&oacute;n: poner en cero y girar el robot 360&deg; a mano sobre su eje.</p>
      <div class="row"><button class="alt" onclick="go('/api/cal?what=track')">Calcular separaci&oacute;n</button></div>
    </div>

    <div class="card">
      <h2>Motores (prueba manual)</h2>
      <div class="row"><label>Motor</label>
        <select id="mSide"><option value="b">Ambos</option><option value="l">Izquierdo</option><option value="r">Derecho</option></select></div>
      <div class="row"><label>PWM (-255..255)</label><input id="mPwm" type="number" value="120"></div>
      <div class="row"><label>Duraci&oacute;n (ms)</label><input id="mMs" type="number" value="1000" step="100"></div>
      <div class="row"><button onclick="go('/api/motor?side='+$('mSide').value+'&pwm='+$('mPwm').value+'&ms='+$('mMs').value)">Probar</button></div>
      <p class="note">El resultado muestra cu&aacute;ntas cuentas dio cada rueda. Si una rueda gira al rev&eacute;s, cambiar <code>MOTOR_SIGN_*</code>.</p>
    </div>
  </div>
</div>

<div id="stop"><button onclick="go('/api/stop')">STOP</button></div>

<script>
const $=id=>document.getElementById(id);
let lastResId=-1,lastCfgVer=-1;
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(toast.t);toast.t=setTimeout(()=>e.style.display='none',2600)}
async function api(p){try{const r=await fetch(p);return await r.json()}catch(e){return null}}
async function go(p){const j=await api(p);if(!j){toast('Sin respuesta del robot');return}
  if(j.ok===false)toast('Error: '+j.error);else if(j.msg)toast(j.msg)}
function move(s){const v=parseFloat($('mm').value);if(!(v>0)){toast('Distancia invalida');return}go('/api/move?mm='+(s*v))}
function turn(s){const v=parseFloat($('deg').value);if(!(v>0)){toast('Angulo invalido');return}go('/api/turn?deg='+(s*v))}
function applyCfg(){
  const q=['cpm','track','kp','pwmMax','pwmMin','tol','accel','frontStop'].map(k=>k+'='+$('p_'+k).value).join('&');
  go('/api/config?'+q)}
function fillCfg(s){
  ['cpm','track','kp','pwmMax','pwmMin','tol','accel','frontStop'].forEach(k=>$('p_'+k).value=s[k]);}
function f(x,d){return Number(x).toFixed(d)}
async function poll(){
  const s=await api('/api/status');
  const c=$('conn');
  if(!s){c.textContent='sin conexión';c.className='chip bad';return}
  c.textContent=s.motion==='IDLE'?'conectado':'moviendo';c.className='chip '+(s.motion==='IDLE'?'ok':'run');
  $('encL').textContent=s.encL;$('encR').textContent=s.encR;
  $('spdL').textContent=s.spdL+' c/s';$('spdR').textContent=s.spdR+' c/s';
  $('motion').textContent=s.motion;
  $('shI').textContent=s.shI;$('shF').textContent=s.shF;$('shD').textContent=s.shD;
  $('qre').textContent=s.qre;$('start').textContent=s.start?'PRESIONADO':'suelto';
  if(s.cfgVer!==lastCfgVer){lastCfgVer=s.cfgVer;fillCfg(s)}
  $('calState').textContent=s.calibrated?'Cuentas/mm calibradas.':'ATENCION: cuentas/mm sin calibrar (valor estimado).';
  $('code').textContent='const float COUNTS_PER_MM = '+f(s.cpm,4)+'f;\nconst float TRACK_MM = '+f(s.track,2)+'f;';
  const r=s.res;
  if(r.id!==lastResId){lastResId=r.id;
    $('rType').textContent=r.type+' #'+r.id+(r.cmd?'  ('+f(r.cmd,1)+')':'');
    $('rCmd').textContent=r.cmdL+' / '+r.cmdR;
    $('rGot').textContent=r.gotL+' / '+r.gotR;
    $('rMm').textContent=f(r.mmL,1)+' / '+f(r.mmR,1);
    $('rDeg').textContent=f(r.deg,1);
    $('rEnd').textContent=r.ms+' ms / '+r.reason;}
}
setInterval(poll,250);poll();
</script>
</body>
</html>
)HTML";

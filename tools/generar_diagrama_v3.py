#!/usr/bin/env python3
"""Genera docs/img/AUS_KIM_V3_diagrama.svg (esquema ilustrativo de la placa V3).
Para el PNG: abrir el SVG en un navegador y exportar/capturar (1900 x 1210 px).
Uso: python tools/generar_diagrama_v3.py"""
import html
import os
W,H=1900,1210
C=dict(vbat="#d62828",v5="#e07a00",v3="#1d6fd8",gnd="#555555",sig="#1f9d8b",mot="#7b2cbf",ink="#1d2430",mut="#5b6574",bg="#f3f5f8")
o=[]
def esc(t): return html.escape(t,quote=False)
def text(x,y,t,size=15,color=None,weight="400",anchor="start",mono=False):
    o.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color or C["ink"]}" font-weight="{weight}" text-anchor="{anchor}"'+(' font-family="DejaVu Sans Mono, monospace"' if mono else '')+f'>{esc(t)}</text>')
def pill(x,y,label,color):
    w=len(label)*8+16
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="20" rx="10" fill="{color}"/>')
    text(x+w/2,y+14.5,label,12,"#fff","700","middle")
    return w
TAGC={"VBAT":C["vbat"],"5V":C["v5"],"3V3":C["v3"],"GND":C["gnd"]}
def box(x,y,w,title,lines,accent,tags=(),ext=False,h=None):
    h=h or 56+22*len(lines)+20
    dash=' stroke-dasharray="7 5"' if ext else ''
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="#fff" stroke="{accent}" stroke-width="2.4"{dash}/>')
    o.append(f'<rect x="{x}" y="{y}" width="8" height="{h}" rx="4" fill="{accent}"/>')
    text(x+22,y+27,title,19,C["ink"],"700")
    tx=x+w-12
    for t in reversed(tags):
        ww=len(t)*8+16; tx-=ww; pill(tx,y+h-28,t,TAGC[t]); tx-=5
    for i,l in enumerate(lines): text(x+22,y+56+22*i,l,15,C["ink"])
    return h
def marker(name,color):
    return f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{color}"/></marker>'
defs="".join(marker("a"+k,v) for k,v in C.items() if k in("vbat","v5","v3","sig","mot","gnd"))
def line(pts,color,width=3.2,dash=None,arrow=True):
    d="M"+" L".join(f"{x},{y}" for x,y in pts)
    k=[k for k,v in C.items() if v==color][0]
    o.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"'+(f' stroke-dasharray="{dash}"' if dash else '')+(f' marker-end="url(#a{k})"' if arrow else '')+' stroke-linejoin="round"/>')

# ---------- titulo y leyenda
text(40,48,"AUS_KIM V3 — módulos y componentes (esquema ilustrativo, no es el layout del PCB)",30,C["ink"],"800")
text(40,76,"ESP32-S3-Zero + TB6612FNG + 4 Sharp + QRE1113GR + 2 N20 6 V con encoder + LiPo 2S",16,C["mut"])
lx=40
for lab,col in (("VBAT 7,4 V",C["vbat"]),("+5 V",C["v5"]),("+3,3 V",C["v3"]),("GND",C["gnd"]),("señal",C["sig"]),("motor",C["mot"])):
    o.append(f'<rect x="{lx}" y="96" width="22" height="8" rx="4" fill="{col}"/>'); text(lx+28,106,lab,14,C["mut"]); lx+=len(lab)*8+58
text(lx+10,106,"·  Borde punteado = fuera de la placa",14,C["mut"])

# ---------- cadena de alimentacion (x=40..360)
PX,PW=40,330
y=140
ys={}
for key,title,lines,tags,ext in (
 ("bat","Batería LiPo 2S",["7,4 V nominal (6,0 – 8,4 V)","No usarla por debajo de ~7,0 V"],("VBAT",),True),
 ("sw","Interruptor general",["Slide switch en el positivo"],("VBAT",),False),
 ("prot","Protección de entrada",["MOSFET canal P (tipo AO3401A):","polaridad invertida","Fusible reseteable 2 A"],("VBAT",),False),
 ("bus","Bus VBAT",["100 µF 16 V + 10 µF 25 V","cerca del TB6612FNG"],("VBAT",),False),
 ("reg","Regulador de 5 V (TO-220)",["L7805CV  o  drop-in conmutado","(R-78E5.0-1.0 / OKI-78SR-5/1.5)","Entrada: 0,33 µF + 10 µF 25 V","Salida:  0,1 µF + 10 µF","Pines: 1 Vin · 2 GND · 3 Vout"],("VBAT","5V"),False),
 ("sch","Diodo Schottky (SS14)",["5 V del regulador → pin 5V del S3-Zero","evita realimentar el USB"],("5V",),False)):
    h=box(PX,y,PW,title,lines,C["vbat"] if key!="reg" and key!="sch" else C["v5"],tags,ext)
    ys[key]=(y,h); y+=h+34
cx=PX+PW/2
order=["bat","sw","prot","bus","reg","sch"]
for a,b in zip(order,order[1:]):
    ya=ys[a][0]+ys[a][1]; yb=ys[b][0]
    line([(cx,ya),(cx,yb)],C["v5"] if b=="sch" else C["vbat"])

# ---------- S3-Zero
SX,SY,SW=430,140,590
sh=650
o.append(f'<rect x="{SX}" y="{SY}" width="{SW}" height="{sh}" rx="12" fill="#fff" stroke="{C["sig"]}" stroke-width="2.4"/><rect x="{SX}" y="{SY}" width="8" height="{sh}" rx="4" fill="{C["sig"]}"/>')
text(SX+22,SY+27,"ESP32-S3-Zero (Waveshare)",19,C["ink"],"700")
tx=SX+SW-12
for t in reversed(("5V","3V3","GND")):
    ww=len(t)*8+16; tx-=ww; pill(tx,SY+10,t,TAGC[t]); tx-=5
MX0,MX1,MY0,MY1=615,835,195,505
o.append(f'<rect x="{MX0}" y="{MY0}" width="{MX1-MX0}" height="{MY1-MY0}" rx="10" fill="#1b2b45"/>')
o.append(f'<rect x="{(MX0+MX1)/2-28}" y="{MY0-14}" width="56" height="22" rx="4" fill="#aeb6c4"/>')
text((MX0+MX1)/2,MY0+40,"USB-C",13,"#cbd3df","400","middle")
text((MX0+MX1)/2,(MY0+MY1)/2+6,"ESP32-S3",20,"#fff","700","middle")
text((MX0+MX1)/2,(MY0+MY1)/2+30,"Zero",16,"#cbd3df","400","middle")
text((MX0+MX1)/2,MY1-18,"antena",13,"#cbd3df","400","middle")
L=[("5V",C["v5"]),("GND",C["gnd"]),("3V3",C["v3"]),("GPIO1  SH_I",C["sig"]),("GPIO2  SH_FI",C["sig"]),("GPIO3  VBAT",C["vbat"]),("GPIO4  SH_FD",C["sig"]),("GPIO5  SH_D",C["sig"]),("GPIO6  QRE",C["sig"])]
R=[("GPIO43  STBY",C["mot"]),("GPIO44  BIN2",C["mot"]),("GPIO13  BIN1",C["mot"]),("GPIO12  AIN2",C["mot"]),("GPIO11  AIN1",C["mot"]),("GPIO10  ENC_DB",C["sig"]),("GPIO9  ENC_DA",C["sig"]),("GPIO8  ENC_IB",C["sig"]),("GPIO7  ENC_IA",C["sig"])]
for i,((l,lc),(r,rc)) in enumerate(zip(L,R)):
    py=MY0+32+i*29
    o.append(f'<circle cx="{MX0}" cy="{py}" r="5.5" fill="#d9b44a"/><circle cx="{MX1}" cy="{py}" r="5.5" fill="#d9b44a"/>')
    text(MX0-12,py+5,l,15,lc,"700","end"); text(MX1+12,py+5,r,15,rc,"700","start")
for i,(lab,col) in enumerate((("14  PWMA",C["mot"]),("15  PWMB",C["mot"]),("16  START",C["sig"]))):
    px=MX0+30+i*80
    o.append(f'<rect x="{px-6}" y="{MY1+16}" width="12" height="12" fill="#d9b44a"/>')
    text(px,MY1+50,lab,14,col,"700","middle")
text(MX0-12,MY1+50,"pads sueltos:",13,C["mut"],"400","end")
notes=["3V3 (LDO de la placa, 800 mA) → encoders, QRE,","Vcc del TB6612FNG y VIN(IO) de los Sharp",
 "5V ← regulador vía Schottky (la placa acepta 3,7 – 6 V)",
 "10 µF + 100 nF en 5V y en 3V3, cerca de los pads",
 "Pads sueltos 14, 15, 16: se conectan con cable",
 "Zona de la antena libre de cobre, pistas y piezas",
 "Programación por USB-C (USB CDC On Boot: Enabled)"]
for i,n in enumerate(notes): text(SX+22,MY1+96+i*22,n,14.5,C["ink"])

# ---------- TB6612FNG
TX,TY,TW=1100,140,380
th=box(TX,TY,TW,"TB6612FNG (SSOP24, soldado)",
 ["AIN1 ← GPIO11    AIN2 ← GPIO12","BIN1 ← GPIO13    BIN2 ← GPIO44","PWMA ← GPIO14    PWMB ← GPIO15","STBY ← GPIO43  + pull-down 100 kΩ","VM ← VBAT (SJ1: o +5 V)   Vcc ← 3,3 V","VM: 10 µF + 100 nF    Vcc: 10 µF + 100 nF","(capacitores pegados al chip)","Modo avance/freno · PWM 31,25 kHz","PWM efectivo = duty × 6 V / VBAT"],C["mot"],("VBAT","3V3","GND"))
line([(SX+SW,250),(TX,250)],C["mot"],4)
text((SX+SW+TX)/2,228,"7 señales",13,C["mut"],"400","middle")

# ---------- motores
MXX,MW=1560,320
y1=140
h1=box(MXX,y1,MW,"Motor derecho N20 (6 V)",["6 pines: M+ M− 3V3 GND A B","ENC_DA → GPIO9 · ENC_DB → GPIO10","100 nF entre los bornes del motor"],C["mot"],("3V3","GND"),True)
y2=y1+h1+22
h2=box(MXX,y2,MW,"Motor izquierdo N20 (6 V)",["6 pines: M+ M− 3V3 GND A B","ENC_IA → GPIO7 · ENC_IB → GPIO8","100 nF entre los bornes del motor"],C["mot"],("3V3","GND"),True)
line([(TX+TW,y1+h1/2),(MXX,y1+h1/2)],C["mot"],4); text((TX+TW+MXX)/2,y1+h1/2-8,"AO",12.5,C["mut"],"400","middle")
line([(TX+TW,y2+h2/2),(MXX,y2+h2/2)],C["mot"],4); text((TX+TW+MXX)/2,y2+h2/2-8,"BO",12.5,C["mut"],"400","middle")

# ---------- banda inferior (sensores, divisor, boton)
BY=840
dh=box(430,BY,290,"Divisor de batería",["100 kΩ (a VBAT) + 47 kΩ (a GND)","100 nF en el punto medio","8,4 V → ≈2,7 V en GPIO3"],C["vbat"],("VBAT","GND"))
sh2=box(750,BY,410,"4 × Sharp GP2Y0E03 (4–50 cm)",["I → GPIO1 · FI → GPIO2","FD → GPIO4 · D → GPIO5","Cada uno: +5 V (VDD), +3,3 V (VIN(IO) y GPIO1),","GND y OUT. 10 µF + 100 nF en +5 V por conector"],C["sig"],("5V","3V3","GND"),True)
qh=box(1190,BY,290,"QRE1113GR (piso)",["Plaqueta externa, 3 pines","LED 150 Ω · fototransistor 10 kΩ","SENSOR → GPIO6"],C["sig"],("3V3","GND"),True)
stx=box(1190,BY+qh+20,290,"Pulsador START",["Contra GND → GPIO16 (cable)","10 kΩ a 3,3 V + 100 nF"],C["sig"],("3V3",),True)
# flechas a S3 (desde la banda inferior hacia arriba)
sbot=SY+sh
line([(565,BY),(565,sbot)],C["sig"]); text(572,(BY+sbot)/2+4,"VBAT_SENSE",12.5,C["mut"])
line([(920,BY),(920,sbot)],C["sig"]); text(912,(BY+sbot)/2+4,"4 Sharp · ADC1",12.5,C["mut"],"400","end")
line([(1335,BY),(1335,BY-30),(990,BY-30),(990,sbot)],C["sig"]); text(1160,BY-38,"QRE · START",12.5,C["mut"],"400","middle")
# 5V del Schottky al S3
sy_,sh_=ys["sch"]
line([(PX+PW,sy_+sh_/2),(408,sy_+sh_/2),(408,720),(SX,720)],C["v5"]); text(PX+PW+6,sy_+sh_/2-8,"5V",12.5,C["mut"])
# VBAT a TB6612 y al divisor
by_,bh_=ys["bus"]
bus_y=by_+bh_/2
BYL=BY+qh+20+stx+36
line([(PX+PW,bus_y),(384,bus_y),(384,BYL),(1520,BYL),(1520,468),(1300,468),(1300,TY+th)],C["vbat"],3.2,"9 6")
line([(565,BYL),(565,BY+dh)],C["vbat"],3.2,"9 6")
text(640,BYL-8,"VBAT (rama de potencia, directa de la batería)",13,C["vbat"],"700")
# ---------- inset sensores
IX,IY,IW,IH=1560,510,320,560
o.append(f'<rect x="{IX}" y="{IY}" width="{IW}" height="{IH}" rx="12" fill="#fff" stroke="{C["sig"]}" stroke-width="2.4"/><rect x="{IX}" y="{IY}" width="8" height="{IH}" rx="4" fill="{C["sig"]}"/>')
text(IX+22,IY+27,"Posición de los Sharp",19,C["ink"],"700"); text(IX+22,IY+48,"(vista desde arriba, ↑ = frente)",13.5,C["mut"])
rx,ry,rw,rh=IX+100,IY+165,120,128
o.append(f'<rect x="{rx}" y="{ry}" width="{rw}" height="{rh}" rx="10" fill="#e8ebf2" stroke="{C["ink"]}" stroke-width="2"/>')
o.append(f'<rect x="{rx+14}" y="{ry+rh-22}" width="{rw-28}" height="10" rx="3" fill="#9aa4b5"/>')
text(rx+rw/2,ry+rh/2+4,"AUS_KIM",15,C["ink"],"700","middle")
# ruedas
o.append(f'<rect x="{rx-14}" y="{ry+78}" width="14" height="40" rx="3" fill="#333"/><rect x="{rx+rw}" y="{ry+78}" width="14" height="40" rx="3" fill="#333"/>')
# frontales (paralelos)
for fx,lab in ((rx+30,"FI"),(rx+rw-30,"FD")):
    o.append(f'<circle cx="{fx}" cy="{ry}" r="5" fill="{C["sig"]}"/>')
    line([(fx,ry-4),(fx,ry-95)],C["sig"],3)
    text(fx,ry-102,lab,14,C["sig"],"700","middle")
# laterales (90 grados)
ly=ry+52
o.append(f'<circle cx="{rx+8}" cy="{ly}" r="5" fill="{C["sig"]}"/><circle cx="{rx+rw-8}" cy="{ly}" r="5" fill="{C["sig"]}"/>')
line([(rx+4,ly),(rx-72,ly)],C["sig"],3); text(rx-76,ly-8,"I",14,C["sig"],"700","end")
line([(rx+rw-4,ly),(rx+rw+72,ly)],C["sig"],3); text(rx+rw+76,ly-8,"D",14,C["sig"],"700","start")
o.append(f'<circle cx="{rx+rw/2}" cy="{ry+rh-8}" r="6" fill="{C["gnd"]}"/>'); text(rx+rw/2+14,ry+rh+15,"QRE (mira al piso)",12.5,C["mut"])
tips=["Laterales a 90°, hundidos 15–20 mm","en el chasis (celda de 18 cm).","Frontales a 0°, separados 40–60 mm:","FI − FD = error de orientación.","Pared al frente = las dos la ven.","Decidirla 20–40 mm antes del centro,","llegar al centro por encoders."]
for i,t in enumerate(tips): text(IX+22,IY+IH-176+i*22,t,14,C["ink"])
# ---------- estado
text(40,H-12,"Estado: propuesta para revisar. Verificar con las piezas reales: huella del S3-Zero, pinout del regulador, VBUS/5V del S3-Zero y tabla de verdad del TB6612FNG.",14,C["mut"])
svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Arial, sans-serif"><defs>{defs}</defs><rect width="{W}" height="{H}" fill="{C["bg"]}"/>'+"".join(o)+'</svg>'
RAIZ=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.makedirs(os.path.join(RAIZ,"docs","img"),exist_ok=True)
open(os.path.join(RAIZ,"docs","img","AUS_KIM_V3_diagrama.svg"),"w",encoding="utf-8").write(svg)
print("svg ok",len(svg))

#!/usr/bin/env python3
"""Genera los diagramas por etapa de la V3 en docs/img/etapas/*.svg
(esquemas ilustrativos de conexión, no son el layout del PCB).
Para los PNG: python tools/render_svg_etapas.py   (Playwright / Chromium)
Uso: python tools/generar_diagramas_etapas_v3.py"""
import html
import os

C = dict(vbat="#d62828", v5="#e07a00", v3="#1d6fd8", gnd="#555555", sig="#1f9d8b",
         mot="#7b2cbf", ink="#1d2430", mut="#5b6574", bg="#f3f5f8")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "img", "etapas")


def esc(t):
    return html.escape(t, quote=False)


class Cv:
    def __init__(self, w, h, title, sub):
        self.w, self.h, self.o = w, h, []
        self.t(40, 48, title, 30, C["ink"], "800")
        self.t(40, 76, sub, 16, C["mut"])
        lx = 40
        for lab, col in (("VBAT 7,4 V", C["vbat"]), ("+5 V", C["v5"]), ("+3,3 V", C["v3"]),
                         ("GND", C["gnd"]), ("señal", C["sig"]), ("motor", C["mot"])):
            self.o.append(f'<rect x="{lx}" y="96" width="22" height="8" rx="4" fill="{col}"/>')
            self.t(lx + 28, 106, lab, 14, C["mut"])
            lx += len(lab) * 8 + 58
        self.t(lx + 10, 106, "·  Borde punteado = fuera de la placa principal", 14, C["mut"])

    # --- primitivas
    def t(self, x, y, s, size=15, color=None, weight="400", anchor="start", mono=False):
        self.o.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color or C["ink"]}" font-weight="{weight}" '
                      f'text-anchor="{anchor}"' + (' font-family="DejaVu Sans Mono, monospace"' if mono else '')
                      + f'>{esc(s)}</text>')

    def pill(self, x, y, label, color, anchor="start"):
        w = len(label) * 8 + 16
        if anchor == "end":
            x -= w
        elif anchor == "middle":
            x -= w / 2
        self.o.append(f'<rect x="{x}" y="{y}" width="{w}" height="20" rx="10" fill="{color}"/>')
        self.t(x + w / 2, y + 14.5, label, 12, "#fff", "700", "middle")
        return w

    def rect(self, x, y, w, h, fill="#fff", stroke=None, sw=2.4, dash=False, rx=12):
        self.o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}"'
                      + (f' stroke="{stroke}" stroke-width="{sw}"' if stroke else '')
                      + (' stroke-dasharray="7 5"' if dash else '') + '/>')

    def box(self, x, y, w, title, lines, accent, ext=False, h=None, size=18):
        h = h or 56 + 22 * len(lines) + 14
        self.rect(x, y, w, h, "#fff", accent, 2.4, ext)
        self.o.append(f'<rect x="{x}" y="{y}" width="8" height="{h}" rx="4" fill="{accent}"/>')
        self.t(x + 22, y + 27, title, size, C["ink"], "700")
        for i, l in enumerate(lines):
            self.t(x + 22, y + 54 + 22 * i, l, 15, C["ink"])
        return h

    def wire(self, pts, color, width=3.2, arrow=None, dash=None):
        d = "M" + " L".join(f"{x},{y}" for x, y in pts)
        self.o.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"'
                      + (f' stroke-dasharray="{dash}"' if dash else '')
                      + (' marker-end="url(#a%s)"' % self.k(color) if arrow in ("end", "both") else '')
                      + (' marker-start="url(#a%s)"' % self.k(color) if arrow in ("start", "both") else '')
                      + '/>')

    @staticmethod
    def k(color):
        return [k for k, v in C.items() if v == color][0]

    def dot(self, x, y, color):
        self.o.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="{color}"/>')

    def gnd(self, x, y):
        for i, wd in enumerate((22, 14, 6)):
            self.o.append(f'<line x1="{x-wd/2}" y1="{y+i*5}" x2="{x+wd/2}" y2="{y+i*5}" stroke="{C["gnd"]}" stroke-width="3"/>')

    def cap(self, x, y, label, color, drop=62, lab=True, side="r"):
        """capacitor a GND colgado del nodo (x,y)"""
        self.dot(x, y, color)
        self.wire([(x, y), (x, y + drop - 22)], color, 3)
        for dy in (22, 14):
            self.o.append(f'<line x1="{x-13}" y1="{y+drop-dy}" x2="{x+13}" y2="{y+drop-dy}" stroke="{color}" stroke-width="3.2"/>')
        self.wire([(x, y + drop - 14), (x, y + drop)], C["gnd"], 3)
        self.gnd(x, y + drop)
        if lab:
            if side == "r":
                self.t(x + 17, y + drop - 12, label, 12.5, C["mut"])
            else:
                self.t(x - 17, y + drop - 12, label, 12.5, C["mut"], "400", "end")

    def res_v(self, x, y, label, color, h=34):
        self.wire([(x, y), (x, y + 10)], color, 3)
        self.rect(x - 8, y + 10, 16, h, "#fff", color, 2.4, rx=3)
        self.wire([(x, y + 10 + h), (x, y + h + 20)], color, 3)
        self.t(x + 16, y + 10 + h / 2 + 5, label, 13, C["ink"])
        return y + h + 20

    def diode_h(self, x, y, label):
        self.wire([(x - 40, y), (x, y)], C["v5"], 3.2)
        self.o.append(f'<path d="M{x},{y-15} L{x},{y+15} L{x+32},{y} z" fill="{C["v5"]}"/>')
        self.o.append(f'<line x1="{x+32}" y1="{y-15}" x2="{x+32}" y2="{y+15}" stroke="{C["v5"]}" stroke-width="4"/>')
        self.wire([(x + 32, y), (x + 72, y)], C["v5"], 3.2)
        self.t(x + 16, y - 24, label, 14, C["ink"], "700", "middle")

    def save(self, name):
        defs = "".join(
            f'<marker id="a{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
            f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{v}"/></marker>'
            for k, v in C.items() if k in ("vbat", "v5", "v3", "sig", "mot", "gnd"))
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" viewBox="0 0 {self.w} {self.h}" '
               f'font-family="DejaVu Sans, Arial, sans-serif"><defs>{defs}</defs>'
               f'<rect width="100%" height="100%" fill="{C["bg"]}"/>' + "".join(self.o) + '</svg>')
        os.makedirs(OUT, exist_ok=True)
        p = os.path.join(OUT, name + ".svg")
        with open(p, "w", encoding="utf-8") as f:
            f.write(svg)
        print(name, self.w, self.h)


def notes(cv, x, y, w, title, lines, accent, h=None):
    cv.box(x, y, w, title, lines, accent, h=h)


# =====================================================================
# 1. Entrada de bateria y VBAT
# =====================================================================
def etapa1():
    cv = Cv(1600, 800, "Etapa 1 — Batería, protección y bus VBAT",
            "De la batería LiPo 2S hasta el riel VBAT (6,0 – 8,4 V) que alimenta driver, regulador y medición de batería")
    by = 300
    cv.box(30, 229, 200, "Batería LiPo 2S", ["7,4 V nominal", "6,0 – 8,4 V", "conector XT30/JST"], C["vbat"], True)
    cv.wire([(230, by), (250, by)], C["vbat"])
    cv.box(250, 255, 110, "SW1", ["interruptor", "general"], C["vbat"], h=90)
    cv.wire([(360, by), (410, by)], C["vbat"])
    cv.box(410, 255, 110, "F1", ["fusible", "reset. 2 A"], C["vbat"], h=90)
    cv.wire([(520, by), (580, by)], C["vbat"])
    cv.box(580, 255, 175, "Q1 AO3401A", ["MOSFET canal P", "anti-inversión"], C["vbat"], h=90)
    cv.t(655, 372, "compuerta a GND:", 13, C["mut"], "400", "middle")
    cv.t(655, 390, "conduce solo con polaridad correcta", 13, C["mut"], "400", "middle")
    cv.wire([(755, by), (1540, by)], C["vbat"], 4.2)
    cv.pill(775, by - 32, "VBAT 6,0 – 8,4 V", C["vbat"])
    # masa de bateria
    cv.wire([(127, 371), (127, 470)], C["gnd"])
    cv.gnd(127, 470)
    cv.t(147, 468, "negativo de la batería → plano de GND", 13, C["mut"])
    # capacitores del bus
    cv.cap(840, by, "C1 100 µF", C["vbat"])
    cv.cap(960, by, "C2 10 µF", C["vbat"])
    cv.t(840, 392, "100 µF ≥ 16 V · 10 µF cerámico 25 V", 12.5, C["mut"])
    # TB6612
    cv.dot(1110, by, C["vbat"])
    cv.wire([(1110, by), (1110, 420)], C["vbat"], arrow="end")
    cv.box(1005, 420, 210, "TB6612FNG", ["VM: pines 13, 14, 24", "10 µF + 100 nF al chip"], C["mot"])
    cv.t(1110, 570, "→ Etapa 3", 14, C["mut"], "700", "middle")
    # divisor
    cv.dot(1230, by, C["vbat"])
    y = cv.res_v(1230, by, "R1 100 kΩ", C["vbat"])
    cv.dot(1230, y, C["sig"])
    cv.wire([(1230, y), (1430, y)], C["sig"], arrow="end")
    cv.t(1352, y - 12, "GPIO3 (ADC1)", 15, C["sig"], "700")
    cv.cap(1340, y, "100 nF", C["sig"], drop=52)
    y2 = cv.res_v(1230, y, "R2 47 kΩ", C["gnd"])
    cv.gnd(1230, y2)
    cv.t(1252, y2 + 40, "Divisor de batería", 14, C["ink"], "700")
    cv.t(1252, y2 + 58, "8,4 V → ≈ 2,7 V", 13, C["mut"])
    # regulador
    cv.dot(1500, by, C["vbat"])
    cv.wire([(1500, by), (1500, 420)], C["vbat"], arrow="end")
    cv.box(1405, 420, 190, "U1 regulador", ["Vin = VBAT", "salida +5 V"], C["v5"])
    cv.t(1500, 570, "→ Etapa 2", 14, C["mut"], "700", "middle")
    notes(cv, 40, 600, 1520, "Por qué cada parte", [
        "Q1 pierde mucho menos que un diodo (un SS34 cae ~0,4 V y disipa en el camino de los motores).",
        "F1: fusible reseteable de 2 A. [verificar la corriente de bloqueo de los N20]",
        "C1 y C2 van pegados al TB6612FNG, no al conector de batería: ahí se necesita la energía de los picos de motor.",
        "El divisor R1/R2 deja la batería medible en un pin del ADC1 (funciona con el Wi-Fi encendido). Con 100 nF filtra el ruido de los motores.",
        "No usar la batería por debajo de ~7,0 V (3,5 V por celda): el regulador de 5 V pierde margen y los Sharp derivan."], C["vbat"])
    cv.save("etapa1_bateria_vbat")


# =====================================================================
# 2. Regulador 5 V y 3,3 V
# =====================================================================
def etapa2():
    cv = Cv(1600, 900, "Etapa 2 — Regulador de +5 V y riel de +3,3 V",
            "VBAT → 7805 → +5 V → diodo → pad 5V del S3-Zero → LDO interno → +3,3 V para los periféricos")
    by = 300
    cv.pill(40, by - 34, "VBAT (etapa 1)", C["vbat"])
    cv.wire([(40, by), (300, by)], C["vbat"], 4)
    cv.cap(140, by, "0,33 µF", C["vbat"])
    cv.cap(225, by, "10 µF", C["vbat"])
    cv.rect(300, 245, 200, 130, "#fff", C["v5"], 2.4)
    cv.t(400, 275, "U1  L7805CV", 18, C["ink"], "700", "middle")
    cv.t(400, 295, "TO-220 · o drop-in conmutado", 12, C["mut"], "400", "middle")
    cv.t(400, 311, "R-78E5.0-1.0 / OKI-78SR-5/1.5", 12, C["mut"], "400", "middle")
    cv.t(308, 342, "1 Vin", 14, C["vbat"], "700")
    cv.t(492, 342, "3 Vout", 14, C["v5"], "700", "end")
    cv.t(400, 366, "2 GND", 14, C["gnd"], "700", "middle")
    cv.wire([(400, 375), (400, 410)], C["gnd"])
    cv.gnd(400, 410)
    cv.wire([(500, by), (960, by)], C["v5"], 4)
    cv.cap(570, by, "0,1 µF", C["v5"])
    cv.cap(650, by, "10 µF", C["v5"])
    cv.dot(740, by, C["v5"])
    cv.wire([(740, by), (740, 420)], C["v5"], arrow="end")
    cv.box(625, 420, 240, "4 × Sharp GP2Y0E03", ["VDD = +5 V (antes del", "diodo) · 10 µF + 100 nF", "por conector"], C["sig"], True)
    cv.t(745, 580, "→ Etapa 5", 14, C["mut"], "700", "middle")
    cv.diode_h(860, by, "D1 SS14")
    cv.t(930, by + 34, "≈ 4,6 V", 12.5, C["mut"], "400", "middle")
    # S3
    cv.rect(960, 195, 340, 395, "#fff", C["sig"], 2.4)
    cv.o.append(f'<rect x="960" y="195" width="8" height="395" rx="4" fill="{C["sig"]}"/>')
    cv.t(985, 222, "ESP32-S3-Zero", 19, C["ink"], "700")
    cv.rect(1100, 160, 70, 26, "#aeb6c4", None, rx=4)
    cv.t(1135, 178, "USB-C", 12, C["ink"], "700", "middle")
    cv.t(1180, 178, "solo para programar", 12.5, C["mut"])
    cv.o.append(f'<circle cx="960" cy="{by}" r="6" fill="#d9b44a"/>')
    cv.t(972, by + 26, "pad 5V", 13, C["v5"], "700")
    cv.wire([(966, by), (1040, by)], C["v5"], 3.2)
    cv.rect(1040, 265, 190, 70, "#eef1f6", C["v3"], 2.2, rx=8)
    cv.t(1135, 292, "LDO ME6217C33", 15, C["ink"], "700", "middle")
    cv.t(1135, 314, "3,3 V · 800 mA", 13, C["mut"], "400", "middle")
    cv.wire([(1230, by), (1294, by)], C["v3"], 3.2)
    cv.o.append(f'<circle cx="1300" cy="{by}" r="6" fill="#d9b44a"/>')
    cv.t(1290, by + 26, "pad 3V3", 13, C["v3"], "700", "end")
    cv.t(985, 372, "Con solo el USB conectado, los Sharp y los", 13.5, C["ink"])
    cv.t(985, 392, "motores quedan apagados: es lo deseable", 13.5, C["ink"])
    cv.t(985, 412, "al programar.", 13.5, C["ink"])
    cv.t(985, 452, "10 µF + 100 nF en el pad 5V y en el 3V3,", 13.5, C["ink"])
    cv.t(985, 472, "cerca del S3-Zero.", 13.5, C["ink"])
    cv.t(985, 512, "[verificar] cómo une Waveshare VBUS y el", 13, C["mut"])
    cv.t(985, 530, "pad 5V antes de depender del diodo D1.", 13, C["mut"])
    cv.o.append(f'<circle cx="1130" cy="590" r="6" fill="#d9b44a"/>')
    cv.wire([(1130, 596), (1130, 620)], C["gnd"])
    cv.gnd(1130, 620)
    cv.t(1148, 612, "pad GND", 13, C["gnd"], "700")
    # rail 3V3
    cv.wire([(1300, by), (1340, by), (1340, 635)], C["v3"], 4)
    cv.pill(1316, by - 38, "+3,3 V", C["v3"])
    items = [("Encoders ×2", "pin 3 de cada motor"), ("QRE1113GR", "pin 2 del conector"),
             ("TB6612FNG", "Vcc, pin 20"), ("Sharp ×4", "VIN(IO) y GPIO1"), ("START", "pull-up 10 kΩ")]
    for i, (a, b) in enumerate(items):
        yy = 355 + i * 65
        cv.dot(1340, yy + 25, C["v3"])
        cv.wire([(1340, yy + 25), (1400, yy + 25)], C["v3"], arrow="end")
        cv.rect(1400, yy, 190, 50, "#fff", C["v3"], 2.2, rx=8)
        cv.t(1412, yy + 21, a, 15, C["ink"], "700")
        cv.t(1412, yy + 40, b, 12.5, C["mut"])
    notes(cv, 40, 640, 880, "Notas del regulador", [
        "Con L7805CV (lineal) pierde (Vbat − 5 V) × I en calor: con 8,4 V son ≈ 1 W a 0,3 A y ≈ 1,7 W a 0,5 A.",
        "Su caída mínima es ~2 V: por debajo de ~7 V de batería la salida deja de ser 5 V.",
        "El drop-in conmutado tiene los mismos 3 pines (1 Vin · 2 GND · 3 Vout), ~90 % de rendimiento",
        "y casi no calienta. Soldar primero el drop-in si se consigue; el L7805 queda de plan B.",
        "Consumo en 5 V ≈ 0,3 – 0,5 A (4 Sharp de 26–36 mA + S3-Zero con Wi-Fi de 100–350 mA)."], C["v5"])
    cv.save("etapa2_regulador_5v_3v3")


# =====================================================================
# 3. TB6612FNG
# =====================================================================
def etapa3():
    cv = Cv(1600, 1360, "Etapa 3 — Driver TB6612FNG (SSOP24)",
            "Pines según la hoja de datos · canal A = motor derecho · canal B = motor izquierdo · todos los pines GND/VM se unen en el PCB")
    cx0, cx1, top = 620, 940, 200
    cv.rect(cx0, top, cx1 - cx0, 620, "#1b2b45", None, rx=10)
    cv.t(780, 470, "TB6612FNG", 28, "#fff", "800", "middle")
    cv.t(780, 500, "driver doble puente H", 15, "#cbd3df", "400", "middle")
    cv.t(780, 540, "1,2 A por canal", 14, "#cbd3df", "400", "middle")
    cv.t(780, 560, "(3,2 A de pico)", 14, "#cbd3df", "400", "middle")
    cv.t(780, 600, "PWM hasta 100 kHz", 14, "#cbd3df", "400", "middle")
    cv.t(780, 220, "● marca pin 1", 12, "#9fb0c8", "400", "middle")
    left = [(1, "AO1"), (2, "AO1"), (3, "PGND"), (4, "PGND"), (5, "AO2"), (6, "AO2"),
            (7, "BO2"), (8, "BO2"), (9, "PGND"), (10, "PGND"), (11, "BO1"), (12, "BO1")]
    right = [(24, "VM"), (23, "PWMA"), (22, "AIN2"), (21, "AIN1"), (20, "Vcc"), (19, "STBY"),
             (18, "GND"), (17, "BIN1"), (16, "BIN2"), (15, "PWMB"), (14, "VM"), (13, "VM")]
    ly = {}
    for i, (n, name) in enumerate(left):
        y = top + 25 + i * 50
        ly[n] = y
        col = C["gnd"] if name == "PGND" else C["mot"]
        cv.wire([(cx0, y), (560, y)], col, 3)
        cv.t(cx0 - 8, y - 6, str(n), 12, C["mut"], "700", "end")
        cv.t(cx0 + 12, y + 5, name, 14, "#fff", "700")
    for n in (3, 4, 9, 10):
        cv.wire([(560, ly[n]), (560, ly[n] + 12)], C["gnd"], 3)
        cv.gnd(560, ly[n] + 12)
    cv.t(548, ly[3] + 30, "PGND → GND", 12, C["mut"], "400", "end")
    groups = [("M_DER+", 1, 2, "der"), ("M_DER−", 5, 6, "der"), ("M_IZQ−", 7, 8, "izq"), ("M_IZQ+", 11, 12, "izq")]
    gy = {}
    for name, a, b, side in groups:
        ya, yb = ly[a], ly[b]
        ym = (ya + yb) / 2
        gy[name] = ym
        cv.wire([(560, ya), (560, yb)], C["mot"], 3)
        cv.dot(560, ym, C["mot"])
        cv.wire([(560, ym), (260, ym)], C["mot"], 3)
        cv.pill(430, ym - 30, name, C["mot"])
    for side, y1, y2, nm in (("der", gy["M_DER+"], gy["M_DER−"], "Motor derecho"),
                              ("izq", gy["M_IZQ−"], gy["M_IZQ+"], "Motor izquierdo")):
        cv.rect(40, y1 - 40, 220, y2 - y1 + 80, "#fff", C["mot"], 2.4, True)
        cv.o.append(f'<rect x="40" y="{y1-40}" width="8" height="{y2-y1+80}" rx="4" fill="{C["mot"]}"/>')
        mid = (y1 + y2) / 2
        cv.t(62, mid - 12, nm, 18, C["ink"], "700")
        cv.t(62, mid + 12, "N20 · 6 V", 15, C["ink"])
        cv.t(62, mid + 34, "con encoder (etapa 4)", 13, C["mut"])
        xcap = 340
        cv.dot(xcap, y1, C["mot"]); cv.dot(xcap, y2, C["mot"])
        cv.wire([(xcap, y1), (xcap, mid - 8)], C["mot"], 3)
        cv.wire([(xcap, mid + 8), (xcap, y2)], C["mot"], 3)
        for dy in (-8, 8):
            cv.o.append(f'<line x1="{xcap-14}" y1="{mid+dy}" x2="{xcap+14}" y2="{mid+dy}" stroke="{C["mot"]}" stroke-width="3.2"/>')
        cv.t(xcap - 20, mid + 5, "100 nF", 12.5, C["mut"], "400", "end")
    for i, (n, name) in enumerate(right):
        y = top + 25 + i * 50
        wc = {"VM": C["vbat"], "Vcc": C["v3"], "GND": C["gnd"]}.get(name, C["mot"])
        cv.wire([(cx1, y), (1010, y)], wc, 3)
        cv.t(cx1 + 8, y - 14, str(n), 12, C["mut"], "700")
        cv.t(cx1 - 12, y + 5, name, 14, "#fff", "700", "end")
    def flag(n, label, color, desc, arrow=True):
        y = top + 25 + [r[0] for r in right].index(n) * 50
        if arrow:
            cv.wire([(1040, y), (cx1 + 2, y)], color, 3, arrow="end")
        else:
            cv.wire([(1010, y), (1040, y)], color, 3)
        cv.pill(1048, y - 10, label, color)
        cv.t(1048 + len(label) * 8 + 28, y + 5, desc, 14, C["ink"])
    flag(24, "VBAT", C["vbat"], "red VM (condensadores abajo)", False)
    flag(23, "GPIO14", C["mot"], "PWM motor derecho · 31,25 kHz")
    flag(22, "GPIO12", C["mot"], "sentido motor derecho (IN2)")
    flag(21, "GPIO11", C["mot"], "sentido motor derecho (IN1)")
    flag(20, "3V3", C["v3"], "red Vcc (condensadores abajo)", False)
    flag(19, "GPIO43", C["mot"], "habilita el driver · pull-down 100 kΩ")
    flag(17, "GPIO13", C["mot"], "sentido motor izquierdo (IN1)")
    flag(16, "GPIO44", C["mot"], "sentido motor izquierdo (IN2)")
    flag(15, "GPIO15", C["mot"], "PWM motor izquierdo · 31,25 kHz")
    flag(14, "VBAT", C["vbat"], "VM (unido a los otros 2 pines VM)", False)
    flag(13, "VBAT", C["vbat"], "VM (unido a los otros 2 pines VM)", False)
    y18 = top + 25 + 6 * 50
    cv.wire([(1010, y18), (1010, y18 + 12)], C["gnd"], 3)
    cv.gnd(1010, y18 + 12)
    cv.t(1034, y18 + 22, "GND (pin 18) al plano de GND", 14, C["ink"])
    # paneles inferiores
    py = 880
    cv.rect(40, py, 480, 215, "#fff", C["vbat"], 2.4)
    cv.t(60, py + 28, "Red VM  (pines 13, 14 y 24)", 17, C["ink"], "700")
    cv.pill(60, py + 60, "VBAT", C["vbat"])
    cv.wire([(60, py + 100), (480, py + 100)], C["vbat"], 3.6, arrow="end")
    for x, lab in ((150, "100 µF"), (225, "10 µF"), (300, "100 nF")):
        cv.cap(x, py + 100, lab, C["vbat"], 58)
    cv.t(500, py + 76, "→ VM", 13, C["mut"], "700", "end")
    cv.t(60, py + 200, "pegados al chip; el de 100 µF puede ir en el bus", 12.5, C["mut"])
    cv.rect(560, py, 480, 215, "#fff", C["v3"], 2.4)
    cv.t(580, py + 28, "Red Vcc  (pin 20)", 17, C["ink"], "700")
    cv.pill(580, py + 60, "3V3", C["v3"])
    cv.wire([(580, py + 100), (1000, py + 100)], C["v3"], 3.6, arrow="end")
    for x, lab in ((690, "10 µF"), (790, "100 nF")):
        cv.cap(x, py + 100, lab, C["v3"], 58)
    cv.t(1020, py + 76, "→ Vcc", 13, C["mut"], "700", "end")
    cv.t(580, py + 200, "10 µF + 100 nF a menos de pocos mm del pin 20", 12.5, C["mut"])
    cv.rect(1080, py, 480, 215, "#fff", C["mot"], 2.4)
    cv.t(1100, py + 28, "STBY  (pin 19) con pull-down", 17, C["ink"], "700")
    cv.pill(1100, py + 60, "GPIO43", C["mot"])
    cv.wire([(1100, py + 100), (1520, py + 100)], C["mot"], 3.6, arrow="end")
    cv.dot(1300, py + 100, C["mot"])
    y = cv.res_v(1300, py + 100, "100 kΩ", C["mot"], 28)
    cv.wire([(1300, y), (1300, y)], C["gnd"])
    cv.gnd(1300, y)
    cv.t(1520, py + 76, "→ STBY", 13, C["mut"], "700", "end")
    cv.t(1100, py + 200, "el driver arranca apagado hasta que el firmware lo habilita", 12.5, C["mut"])
    # tabla de verdad
    ty = 1130
    cv.box(40, ty, 1520, "Cómo se maneja un canal (PWM en PWMx, dirección en INx)", [], C["mot"], h=215)
    rows = [("IN1", "IN2", "PWM", "STBY", "Resultado"),
            ("H", "L", "PWM", "H", "avance (decaimiento lento: en la parte apagada del ciclo queda en freno corto)"),
            ("L", "H", "PWM", "H", "retroceso"),
            ("H", "H", "x", "H", "freno corto (se usa para frenar de golpe)"),
            ("L", "L", "H", "H", "parada libre (rueda suelta)"),
            ("x", "x", "x", "L", "standby: puentes apagados")]
    for i, r in enumerate(rows):
        yy = ty + 58 + i * 24
        wgt = "700" if i == 0 else "400"
        for xx, v in zip((70, 140, 210, 290, 380), r):
            cv.t(xx, yy, v, 14.5, C["ink"], wgt, mono=True)
    cv.t(1540, ty + 202, "[verificar la tabla de verdad en la hoja de datos original]", 12.5, C["mut"], "400", "end")
    cv.save("etapa3_driver_tb6612fng")


# =====================================================================
# 4. Motores y encoders
# =====================================================================
def motor_panel(cv, y0, nombre, mp, mm, ea, eb, gpa, gpb, pinsA):
    x0 = 40
    cv.rect(x0, y0, 1520, 350, "#fff", C["mot"], 2.4)
    cv.o.append(f'<rect x="{x0}" y="{y0}" width="8" height="350" rx="4" fill="{C["mot"]}"/>')
    cv.t(x0 + 24, y0 + 30, nombre, 20, C["ink"], "700")
    rows = [(mp, f"TB6612  {pinsA[0]}", C["mot"], "in"), (mm, f"TB6612  {pinsA[1]}", C["mot"], "in"),
            ("+3,3 V enc.", "pad 3V3", C["v3"], "in"), ("GND enc.", "plano de GND", C["gnd"], "in"),
            (ea, gpa, C["sig"], "out"), (eb, gpb, C["sig"], "out")]
    ys = [y0 + 88 + i * 46 for i in range(6)]
    cx0, cx1 = x0 + 400, x0 + 560
    cv.rect(cx0, ys[0] - 24, cx1 - cx0, ys[5] - ys[0] + 48, "#eef1f6", C["mut"], 2, rx=6)
    cv.t((cx0 + cx1) / 2, ys[0] - 32, "CN 6 pines", 13, C["mut"], "700", "middle")
    for i, ((lab, src, col, d), y) in enumerate(zip(rows, ys)):
        cv.pill(x0 + 24, y - 10, src, col)
        xf = x0 + 24 + len(src) * 8 + 16 + 10
        if d == "in":
            cv.wire([(xf, y), (cx0, y)], col, 3, arrow="end")
        else:
            cv.wire([(cx0, y), (xf, y)], col, 3, arrow="end")
        cv.t(cx0 + 12, y + 5, str(i + 1), 13, C["mut"], "700")
        cv.t(cx0 + 36, y + 5, lab, 15, C["ink"], "700")
    # motor: pines 1 y 2
    xm = cx1 + 190
    ym = (ys[0] + ys[1]) / 2
    cv.wire([(cx1, ys[0]), (xm - 25, ys[0])], C["mot"], 3)
    cv.wire([(cx1, ys[1]), (xm - 25, ys[1])], C["mot"], 3)
    cv.o.append(f'<circle cx="{xm}" cy="{ym}" r="34" fill="#fff" stroke="{C["mot"]}" stroke-width="3"/>')
    cv.t(xm, ym + 8, "M", 22, C["mot"], "800", "middle")
    xc = cx1 + 90
    cv.dot(xc, ys[0], C["mot"]); cv.dot(xc, ys[1], C["mot"])
    cv.wire([(xc, ys[0]), (xc, ym - 8)], C["mot"], 3)
    cv.wire([(xc, ym + 8), (xc, ys[1])], C["mot"], 3)
    for dy in (-8, 8):
        cv.o.append(f'<line x1="{xc-13}" y1="{ym+dy}" x2="{xc+13}" y2="{ym+dy}" stroke="{C["mot"]}" stroke-width="3.2"/>')
    cv.t(xc + 20, ym + 5, "100 nF", 12.5, C["mut"])
    cv.t(xm + 50, ym - 6, "Motor N20 de 6 V", 16, C["ink"], "700")
    cv.t(xm + 50, ym + 16, "con 100 nF entre los bornes, soldado en el motor", 13, C["mut"])
    # encoder
    ex = cx1 + 60
    cv.rect(ex, ys[2] - 26, 480, ys[5] - ys[2] + 52, "#fff", C["sig"], 2.2, True, 8)
    cv.t(ex + 14, ys[2] - 5, "Encoder Hall (cuadratura)", 15, C["ink"], "700")
    cv.wire([(cx1, ys[2]), (ex, ys[2])], C["v3"], 3)
    cv.wire([(cx1, ys[3]), (ex, ys[3])], C["gnd"], 3)
    cv.wire([(ex, ys[4]), (cx1, ys[4])], C["sig"], 3)
    cv.wire([(ex, ys[5]), (cx1, ys[5])], C["sig"], 3)
    cv.t(ex + 14, ys[2] + 22, "VCC 3,3 V", 13, C["mut"])
    cv.t(ex + 14, ys[3] + 22, "GND", 13, C["mut"])
    cv.t(ex + 14, ys[4] + 5, "canal A", 14, C["ink"])
    cv.t(ex + 14, ys[5] + 5, "canal B", 14, C["ink"])
    cv.t(ex + 150, ys[4] + 5, "A y B desfasados 90°", 13, C["mut"])
    cv.t(ex + 150, ys[5] + 5, "→ sentido y velocidad", 13, C["mut"])


def etapa4():
    cv = Cv(1600, 910, "Etapa 4 — Motores N20 con encoder",
            "Un conector de 6 pines por motor: 2 de potencia (al TB6612FNG) + 4 del encoder (alimentación y señales)")
    motor_panel(cv, 130, "Motor derecho  (canal A del TB6612)", "M_DER+", "M_DER−", "ENC_DA", "ENC_DB", "GPIO9", "GPIO10", ("AO1, pines 1 y 2", "AO2, pines 5 y 6"))
    motor_panel(cv, 500, "Motor izquierdo  (canal B del TB6612)", "M_IZQ+", "M_IZQ−", "ENC_IA", "ENC_IB", "GPIO7", "GPIO8", ("BO1, pines 11 y 12", "BO2, pines 7 y 8"))
    cv.t(40, 893, "Los encoders van a 3,3 V (no a 5 V). Huella opcional de pull-up 10 kΩ en A/B por si las salidas son de colector abierto. "
         "Verificar el orden de cada hilo con el motor real.", 13, C["mut"])
    cv.save("etapa4_motores_encoders")


# =====================================================================
# 5. Sharp
# =====================================================================
def etapa5():
    cv = Cv(1600, 960, "Etapa 5 — Cuatro sensores Sharp GP2Y0E03",
            "Alimentados a +5 V, con VIN(IO) y GPIO1 del sensor a 3,3 V · salida analógica de hasta 2,3 V al ADC1 · rango 4 – 50 cm")
    rows = [("+5 V", C["v5"], 250), ("OUT", C["sig"], 340), ("GND", C["gnd"], 430), ("+3,3 V", C["v3"], 520)]
    cv.t(40, 180, "Un sensor (los 4 se conectan igual)", 19, C["ink"], "700")
    cv.rect(40, 200, 150, 470, "#fff", C["mut"], 2, True, 8)
    cv.t(115, 222, "Placa", 13, C["mut"], "700", "middle")
    for lab, col, y in rows:
        cv.t(60, y + 5, lab, 15, col, "700")
    # 5V
    cv.wire([(135, 250), (330, 250)], C["v5"], 3.4)
    cv.cap(190, 250, "10 µF", C["v5"], 62)
    cv.cap(285, 250, "100 nF", C["v5"], 62)
    cv.wire([(330, 250), (480, 250)], C["v5"], 3.4, arrow="end")
    # OUT
    cv.wire([(480, 340), (135, 340)], C["sig"], 3.4, arrow="end")
    cv.cap(200, 340, "100 nF", C["sig"], 52)
    # GND
    cv.wire([(135, 430), (480, 430)], C["gnd"], 3.4, arrow="end")
    # 3V3
    cv.wire([(135, 520), (470, 520)], C["v3"], 3.4)
    cv.wire([(470, 520), (480, 520)], C["v3"], 3.4, arrow="end")
    cv.dot(440, 520, C["v3"])
    cv.wire([(440, 520), (440, 610), (480, 610)], C["v3"], 3.4, arrow="end")
    # conector
    cv.rect(375, 215, 40, 330, "#eef1f6", C["mut"], 2, rx=6)
    cv.t(395, 207, "CN 4 hilos", 12.5, C["mut"], "700", "middle")
    # sensor
    cv.rect(480, 200, 260, 530, "#1b2b45", None, rx=10)
    cv.t(610, 232, "GP2Y0E03", 20, "#fff", "800", "middle")
    cv.t(610, 254, "4 – 50 cm", 13, "#cbd3df", "400", "middle")
    names = [("1  VDD", 250), ("2  Vout", 340), ("3  GND", 430), ("4  VIN(IO)", 520), ("5  GPIO1", 610)]
    for n, y in names:
        cv.t(496, y + 5, n, 15, "#fff", "700")
    cv.t(496, 668, "6 SCL · 7 SDA", 14, "#9fb0c8", "700")
    cv.t(496, 690, "sin conectar", 13, "#9fb0c8")
    cv.t(40, 700, "Obligatorio: si VDD ≥ 3,6 V, VIN(IO) va a 3,3 V.", 13.5, C["ink"], "700")
    cv.t(40, 722, "Pines 4 y 5 unidos al hilo de 3,3 V en el sensor.", 13.5, C["mut"])
    cv.t(40, 744, "El 100 nF en OUT es opcional.", 13.5, C["mut"])
    # tabla de GPIO
    cv.box(800, 170, 760, "Dónde va cada sensor", [], C["sig"], h=215)
    tab = [("Sharp", "posición", "GPIO (ADC1)"), ("I", "lateral izquierdo · 90°", "GPIO1"),
           ("FI", "frontal izquierdo · 0°", "GPIO2"), ("FD", "frontal derecho · 0°", "GPIO4"),
           ("D", "lateral derecho · 90°", "GPIO5")]
    for i, r in enumerate(tab):
        yy = 232 + i * 28
        w = "700" if i == 0 else "400"
        for xx, v in zip((830, 940, 1330), r):
            cv.t(xx, yy, v, 15, C["sig"] if (i and xx == 1330) else C["ink"], w if xx != 1330 or not i else "700")
    # vista superior
    cv.rect(800, 410, 760, 520, "#fff", C["mut"], 2, False)
    cv.t(824, 440, "Ubicación y ángulos (vista superior, frente hacia arriba)", 17, C["ink"], "700")
    rx, ry, rw, rh = 1060, 570, 190, 270
    cv.rect(rx, ry, rw, rh, "#e8ecf3", C["ink"], 2.4, rx=10)
    for xx in (rx - 22, rx + rw):
        cv.rect(xx, ry + 140, 22, 90, "#445", None, rx=4)
    cv.t(rx + rw / 2, ry + 190, "AUS_KIM", 18, C["ink"], "800", "middle")
    # frontales
    for xx, lab in ((rx + 45, "FI"), (rx + rw - 45, "FD")):
        cv.o.append(f'<path d="M{xx},{ry} L{xx-26},{ry-70} L{xx+26},{ry-70} z" fill="{C["sig"]}" fill-opacity="0.22" stroke="{C["sig"]}" stroke-width="1.5"/>')
        cv.rect(xx - 9, ry - 4, 18, 14, C["sig"], None, rx=3)
        cv.t(xx, ry + 30, lab, 15, C["sig"], "700", "middle")
    cv.wire([(rx + 45, ry - 82), (rx + rw - 45, ry - 82)], C["mut"], 2, arrow="both")
    cv.t(rx + rw / 2, ry - 92, "40 – 60 mm", 13, C["mut"], "700", "middle")
    # laterales
    for xx, d, lab in ((rx, -1, "I"), (rx + rw, 1, "D")):
        yy = ry + 100
        cv.o.append(f'<path d="M{xx},{yy} L{xx+d*70},{yy-26} L{xx+d*70},{yy+26} z" fill="{C["sig"]}" fill-opacity="0.22" stroke="{C["sig"]}" stroke-width="1.5"/>')
        cv.rect(xx - 7 if d == -1 else xx - 7, yy - 9, 14, 18, C["sig"], None, rx=3)
        cv.t(xx + d * 90, yy + 5, lab, 15, C["sig"], "700", "middle")
    cv.t(824, 862, "Frontales paralelos: FI − FD da el error de orientación; pared al frente = ambos sobre el umbral.", 13.5, C["ink"])
    cv.t(824, 882, "Laterales 10–20 mm delante del eje y hundidos 15–20 mm en el chasis", 13.5, C["ink"])
    cv.t(824, 900, "(el mínimo de 4 cm importa en celdas de 18 cm).", 13.5, C["ink"])
    cv.t(824, 920, "[verificar] las posiciones reales contra el CAD del chasis.", 13, C["mut"])
    cv.save("etapa5_sensores_sharp")


# =====================================================================
# 6. QRE + START
# =====================================================================
def etapa6():
    cv = Cv(1600, 780, "Etapa 6 — Sensor de piso QRE1113GR y pulsador START",
            "El QRE va en una plaqueta externa con su propio circuito · START es un pulsador con pull-up y antirrebote por hardware")
    # QRE
    cv.rect(40, 150, 760, 560, "#fff", C["sig"], 2.4, True)
    cv.t(64, 182, "Plaqueta externa QRE1113GR", 19, C["ink"], "700")
    cv.pill(64, 205, "+3,3 V", C["v3"])
    cv.wire([(64, 255), (560, 255)], C["v3"], 4)
    # LED
    cv.dot(190, 255, C["v3"])
    y = cv.res_v(190, 255, "150 Ω", C["v3"], 34)
    cv.rect(160, y, 60, 56, "#fff", C["sig"], 2.4, rx=6)
    cv.t(190, y + 24, "LED IR", 14, C["ink"], "700", "middle")
    cv.t(190, y + 43, "emisor", 12, C["mut"], "400", "middle")
    cv.wire([(190, y + 56), (190, 560)], C["gnd"], 3)
    # fototransistor
    cv.dot(400, 255, C["v3"])
    y2 = cv.res_v(400, 255, "10 kΩ", C["v3"], 34)
    cv.dot(400, y2 + 2, C["sig"])
    cv.rect(350, y2 + 20, 100, 70, "#fff", C["sig"], 2.4, rx=6)
    cv.t(400, y2 + 46, "Foto-", 14, C["ink"], "700", "middle")
    cv.t(400, y2 + 64, "transistor", 14, C["ink"], "700", "middle")
    cv.wire([(400, y2 + 2), (400, y2 + 20)], C["sig"], 3)
    cv.wire([(400, y2 + 90), (400, 560)], C["gnd"], 3)
    cv.wire([(400, y2 + 2), (540, y2 + 2)], C["sig"], 3.4, arrow="end")
    cv.t(552, y2 + 7, "SENSOR → GPIO6", 15, C["sig"], "700")
    cv.wire([(64, 560), (560, 560)], C["gnd"], 3.6)
    cv.gnd(560, 560)
    cv.pill(64, 570, "GND", C["gnd"])
    cv.t(64, 640, "Conector de 3 pines:  1 GND  ·  2 +3,3 V  ·  3 SENSOR", 15, C["ink"], "700")
    cv.t(64, 664, "No agregar otro pull-up en la placa principal (ya está el de 10 kΩ en la plaqueta).", 13.5, C["mut"])
    cv.t(64, 686, "Se calibra desde la pestaña Sensores (captura blanco / negro).", 13.5, C["mut"])
    # START
    cv.rect(840, 150, 720, 560, "#fff", C["sig"], 2.4, True)
    cv.t(864, 182, "Pulsador START (accesible desde afuera)", 19, C["ink"], "700")
    cv.pill(864, 205, "+3,3 V", C["v3"])
    cv.wire([(864, 255), (1060, 255)], C["v3"], 4)
    cv.dot(950, 255, C["v3"])
    y = cv.res_v(950, 255, "10 kΩ", C["v3"], 34)
    cv.dot(950, y, C["sig"])
    cv.wire([(950, y), (1180, y)], C["sig"], 3.4, arrow="end")
    cv.t(1192, y + 5, "START → GPIO16", 15, C["sig"], "700")
    cv.cap(1040, y, "100 nF", C["sig"], 62)
    cv.rect(920, y + 30, 60, 50, "#fff", C["sig"], 2.4, rx=6)
    cv.t(950, y + 52, "SW1", 15, C["ink"], "700", "middle")
    cv.t(950, y + 70, "N.A.", 12, C["mut"], "400", "middle")
    cv.wire([(950, y), (950, y + 30)], C["sig"], 3)
    cv.wire([(950, y + 80), (950, 560)], C["gnd"], 3)
    cv.gnd(950, 560)
    cv.t(864, 640, "GPIO16 es un pad suelto: va con un cable al pulsador.", 14, C["ink"], "700")
    cv.t(864, 664, "El botón BOOT del S3-Zero (GPIO0) no es accesible con la placa montada.", 13.5, C["mut"])
    cv.t(864, 686, "El reglamento exige un pulsador visible y accesible para iniciar y detener.", 13.5, C["mut"])
    cv.save("etapa6_qre_start")


# =====================================================================
# 7. Mapa de pines del S3-Zero
# =====================================================================
def etapa7():
    cv = Cv(1600, 900, "Etapa 7 — Mapa de pines del ESP32-S3-Zero",
            "Qué va conectado a cada pad · todas las lecturas analógicas en ADC1 (GPIO1–10) · no usar Serial0 (GPIO43/44): USB CDC")
    MX0, MX1, MY0, MY1 = 700, 900, 150, 660
    cv.rect(MX0, MY0, MX1 - MX0, MY1 - MY0, "#1b2b45", None, rx=10)
    cv.rect((MX0 + MX1) / 2 - 36, MY0 - 14, 72, 28, "#aeb6c4", None, rx=4)
    cv.t((MX0 + MX1) / 2, MY0 + 6, "USB-C", 13, C["ink"], "700", "middle")
    cv.t((MX0 + MX1) / 2, 130, "ESP32-S3-Zero (vista superior)", 17, C["ink"], "700", "middle")
    cv.t((MX0 + MX1) / 2, MY1 - 22, "antena: zona libre", 13, "#cbd3df", "400", "middle")
    L = [("5V", C["v5"], "regulador vía diodo D1", 2), ("GND", C["gnd"], "plano de GND", 0), ("3V3", C["v3"], "encoders, QRE, Vcc TB6612, Sharp IO", 2),
         ("GPIO1", C["sig"], "Sharp lateral izq. (I)", 5), ("GPIO2", C["sig"], "Sharp frontal izq. (FI)", 5),
         ("GPIO3", C["vbat"], "divisor de batería", 1), ("GPIO4", C["sig"], "Sharp frontal der. (FD)", 5),
         ("GPIO5", C["sig"], "Sharp lateral der. (D)", 5), ("GPIO6", C["sig"], "QRE1113GR piso", 6)]
    R = [("GPIO43", C["mot"], "STBY del TB6612", 3), ("GPIO44", C["mot"], "BIN2 (motor izq.)", 3), ("GPIO13", C["mot"], "BIN1 (motor izq.)", 3),
         ("GPIO12", C["mot"], "AIN2 (motor der.)", 3), ("GPIO11", C["mot"], "AIN1 (motor der.)", 3),
         ("GPIO10", C["sig"], "encoder der., canal B", 4), ("GPIO9", C["sig"], "encoder der., canal A", 4),
         ("GPIO8", C["sig"], "encoder izq., canal B", 4), ("GPIO7", C["sig"], "encoder izq., canal A", 4)]
    for i, ((l, lc, ld, ln), (r, rc, rd, rn)) in enumerate(zip(L, R)):
        y = MY0 + 45 + i * 52
        cv.o.append(f'<circle cx="{MX0}" cy="{y}" r="6" fill="#d9b44a"/><circle cx="{MX1}" cy="{y}" r="6" fill="#d9b44a"/>')
        cv.wire([(MX0 - 8, y), (640, y)], lc, 2.6)
        cv.wire([(MX1 + 8, y), (960, y)], rc, 2.6)
        cv.t(MX0 + 14, y + 5, l, 13.5, "#fff", "700")
        cv.t(MX1 - 14, y + 5, r, 13.5, "#fff", "700", "end")
        cv.t(630, y - 8, ld, 14.5, C["ink"], "400", "end")
        if ln:
            cv.pill(630, y + 2, f"etapa {ln}", lc, "end")
        cv.t(970, y - 8, rd, 14.5, C["ink"])
        cv.pill(970, y + 2, f"etapa {rn}", rc)
    for i, (n, d, e, col) in enumerate((("GPIO14", "PWMA (motor der.)", 3, C["mot"]), ("GPIO15", "PWMB (motor izq.)", 3, C["mot"]),
                                         ("GPIO16", "pulsador START", 6, C["sig"]))):
        px = MX0 + 40 + i * 60
        cv.rect(px - 7, MY1 + 20, 14, 14, "#d9b44a", None, rx=2)
        cv.t(px, MY1 + 56, n[4:], 14, col, "700", "middle")
    cv.t(MX0 - 12, MY1 + 33, "pads sueltos (cara superior):", 13, C["mut"], "400", "end")
    cv.t(MX1 + 12, MY1 + 33, "14 → PWMA (etapa 3) · 15 → PWMB (etapa 3) · 16 → START (etapa 6)", 13.5, C["ink"])
    notes(cv, 40, 750, 1520, "Reglas del S3-Zero", [
        "GPIO3 es pin de arranque (strapping), pero con el divisor (≤ 2,7 V) no afecta la configuración por defecto. [verificar]",
        "GPIO43 y 44 son la UART0: se usan como salidas normales. En el IDE: ESP32S3 Dev Module + USB CDC On Boot: Enabled."], C["sig"])
    cv.save("etapa7_mapa_pines_s3")


if __name__ == "__main__":
    for f in (etapa1, etapa2, etapa3, etapa4, etapa5, etapa6, etapa7):
        f()

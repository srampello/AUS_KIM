#!/usr/bin/env python3
"""Genera la plantilla (stencil) para ubicar AUS_KIM en el centro de una celda.

Salida (en cad/):
  PLANTILLA_CELDA.dxf         dibujo 2D en mm, para importar en Fusion (Insertar > Insertar DXF)
  PLANTILLA_CELDA_vista.png   vista previa con el robot dibujado dentro

Uso:  python tools/generar_plantilla_celda.py [--clear 238] [--axle-z 0] ...

Funcionamiento: una placa plana cuadrada que entra entre las paredes de la celda y tiene un
hueco del tamano del robot. El centro de la placa es el centro de la celda y el eje de las
ruedas queda justo sobre ese centro. Se apoya la placa en el piso, se baja el robot al hueco,
y se levanta la placa hacia arriba.

SUPUESTOS que hay que verificar con el robot real (ver los parametros de abajo):
  - El eje de las ruedas esta en z = 0 del modelo STL (--axle-z).
  - Las paredes tienen 12 mm de espesor: el ancho libre del pasillo es 238 mm (--clear).
  - Ninguna pieza (ruedas, cables, bateria) sobresale del contorno de los STL.
Coordenadas del dibujo: origen = centro de la celda = centro del eje; +Y = frente del robot.
"""
import argparse
import math
import os
import struct

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)

ap = argparse.ArgumentParser()
ap.add_argument("--stl", nargs="+", default=[
    os.path.join(RAIZ, "stl", "AUS_KIM_BASE.stl"),
    os.path.join(RAIZ, "stl", "AUS_KIM_top.stl"),
    os.path.join(RAIZ, "stl", "AUS_KIM_soporteplaqueta.stl")],
    help="STL del robot (vista desde arriba = plano X-Z, Y es la altura)")
ap.add_argument("--axle-z", type=float, default=0.0, help="posicion z del eje de ruedas en el STL (mm)")
ap.add_argument("--cell", type=float, default=250.0, help="lado de la celda (mm)")
ap.add_argument("--clear", type=float, default=238.0,
                help="ancho LIBRE entre paredes de un pasillo (mm). MEDIRLO en la pista real")
ap.add_argument("--slack", type=float, default=1.5, help="holgura total de la placa dentro del pasillo (mm)")
ap.add_argument("--clearance", type=float, default=0.6, help="holgura por lado entre robot y hueco (mm)")
ap.add_argument("--finger", type=float, default=22.0, help="diametro de los agujeros para levantar (mm)")
ap.add_argument("--out", default=os.path.join(RAIZ, "cad"))
args = ap.parse_args()


def leer_stl_binario(ruta):
    d = open(ruta, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    tris = []
    for i in range(n):
        v = struct.unpack("<12f", d[84 + 50 * i: 84 + 50 * i + 48])
        tris.append([(v[3], v[5]), (v[6], v[8]), (v[9], v[11])])  # (x, z) de cada vertice
    return tris


tris = []
for r in args.stl:
    tris += leer_stl_binario(r)
xs = [p[0] for t in tris for p in t]
zs = [p[1] for t in tris for p in t]
rob_x_min, rob_x_max = min(xs), max(xs)
# Dibujo: y = -(z - eje). El frente del robot es z negativo -> y positivo.
rob_y_max = -(min(zs) - args.axle_z)   # punta delantera
rob_y_min = -(max(zs) - args.axle_z)   # parte trasera

c = args.clearance
hueco = (rob_x_min - c, rob_y_min - c, rob_x_max + c, rob_y_max + c)  # x0,y0,x1,y1
placa = args.clear - args.slack
mitad = placa / 2

# --- comprobaciones
assert hueco[2] - hueco[0] < placa - 20, "el hueco casi no deja borde en la placa"
assert hueco[3] < mitad - 10 and -hueco[1] < mitad - 10

# --- entidades:  (capa, tipo, datos)
ent = []


def rect(capa, x0, y0, x1, y1):
    ent.extend([(capa, "L", (x0, y0, x1, y0)), (capa, "L", (x1, y0, x1, y1)),
                (capa, "L", (x1, y1, x0, y1)), (capa, "L", (x0, y1, x0, y0))])


rect("CORTE", -mitad, -mitad, mitad, mitad)                 # contorno de la placa
rect("CORTE", *hueco)                                       # hueco del robot
dedo = args.finger / 2
for sx in (-1, 1):                                          # agujeros para levantar la placa
    ent.append(("CORTE", "C", (sx * (mitad - 28), 0, dedo)))
# marcas grabadas: eje de ruedas (y=0) y eje longitudinal (x=0) hasta los bordes + flecha de frente
ent.append(("GRABADO", "L", (-mitad, 0, hueco[0], 0)))
ent.append(("GRABADO", "L", (hueco[2], 0, mitad, 0)))
ent.append(("GRABADO", "L", (0, hueco[3], 0, mitad)))
ent.append(("GRABADO", "L", (0, -mitad, 0, hueco[1])))
fy = (hueco[3] + mitad) / 2
ent.extend([("GRABADO", "L", (-9, fy - 8, 0, fy + 8)), ("GRABADO", "L", (0, fy + 8, 9, fy - 8)),
            ("GRABADO", "L", (9, fy - 8, -9, fy - 8))])
# referencia (no se corta): contorno exacto del robot, la celda y el pasillo libre
rect("REFERENCIA", rob_x_min, rob_y_min, rob_x_max, rob_y_max)
rect("REFERENCIA", -args.cell / 2, -args.cell / 2, args.cell / 2, args.cell / 2)
rect("REFERENCIA", -args.clear / 2, -args.clear / 2, args.clear / 2, args.clear / 2)

# --- DXF (R12, ASCII, en mm)
CAPAS = {"CORTE": 1, "GRABADO": 5, "REFERENCIA": 8}
o = ["0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1009", "0", "ENDSEC",
     "0", "SECTION", "2", "TABLES",
     "0", "TABLE", "2", "LTYPE", "70", "1", "0", "LTYPE", "2", "CONTINUOUS", "70", "0",
     "3", "Solid line", "72", "65", "73", "0", "40", "0.0", "0", "ENDTAB",
     "0", "TABLE", "2", "LAYER", "70", str(len(CAPAS))]
for nombre, col in CAPAS.items():
    o += ["0", "LAYER", "2", nombre, "70", "0", "62", str(col), "6", "CONTINUOUS"]
o += ["0", "ENDTAB", "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]
for capa, tipo, d in ent:
    if tipo == "L":
        o += ["0", "LINE", "8", capa, "10", f"{d[0]:.4f}", "20", f"{d[1]:.4f}", "30", "0.0",
              "11", f"{d[2]:.4f}", "21", f"{d[3]:.4f}", "31", "0.0"]
    else:
        o += ["0", "CIRCLE", "8", capa, "10", f"{d[0]:.4f}", "20", f"{d[1]:.4f}", "30", "0.0", "40", f"{d[2]:.4f}"]
o += ["0", "ENDSEC", "0", "EOF"]

os.makedirs(args.out, exist_ok=True)
ruta_dxf = os.path.join(args.out, "PLANTILLA_CELDA.dxf")
open(ruta_dxf, "w", newline="\n").write("\n".join(o) + "\n")

print(f"Robot (desde STL): ancho {rob_x_max - rob_x_min:.2f} mm, largo {rob_y_max - rob_y_min:.2f} mm")
print(f"  delante del eje: {rob_y_max:.2f} mm   detras del eje: {-rob_y_min:.2f} mm")
print(f"Hueco: {hueco[2] - hueco[0]:.2f} x {hueco[3] - hueco[1]:.2f} mm (holgura {c} mm por lado)")
print(f"Placa: {placa:.1f} x {placa:.1f} mm   (pasillo libre {args.clear} mm)")
print("DXF:", ruta_dxf)

# --- vista previa
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Rectangle, Circle

    fig, ax = plt.subplots(figsize=(8, 8))
    t = (args.cell - args.clear) / 2
    ax.add_patch(Rectangle((-args.cell / 2 - 14, -args.cell / 2 - 14), args.cell + 28, args.cell + 28,
                           facecolor="#d8d8d8", edgecolor="none"))
    ax.add_patch(Rectangle((-args.clear / 2, -args.clear / 2), args.clear, args.clear, facecolor="#1a1a1a"))
    ax.add_patch(Rectangle((-mitad, -mitad), placa, placa, facecolor="#f2c879", edgecolor="#8a5a00", lw=1.5))
    ax.add_patch(Rectangle((hueco[0], hueco[1]), hueco[2] - hueco[0], hueco[3] - hueco[1],
                           facecolor="#1a1a1a", edgecolor="#8a5a00", lw=1.2))
    poly = [[(p[0], -(p[1] - args.axle_z)) for p in tri] for tri in tris]
    ax.add_collection(PolyCollection(poly, facecolors=(0.35, 0.45, 0.95, 0.35), edgecolors="none"))
    for sx in (-1, 1):
        ax.add_patch(Circle((sx * (mitad - 28), 0), dedo, facecolor="#1a1a1a", edgecolor="#8a5a00"))
    ax.plot([-mitad, mitad], [0, 0], color="#c0392b", lw=0.8, ls="--")
    ax.plot([0, 0], [-mitad, mitad], color="#c0392b", lw=0.8, ls="--")
    ax.annotate("", xy=(0, mitad - 6), xytext=(0, hueco[3] + 6), arrowprops=dict(arrowstyle="->", color="#8a5a00"))
    ax.text(0, mitad - 2, "FRENTE", ha="center", va="top", fontsize=8, color="#8a5a00")
    ax.text(-mitad + 4, mitad - 6, f"placa {placa:.1f} x {placa:.1f} mm", fontsize=8, va="top")
    ax.text(-mitad + 4, -mitad + 4, f"hueco {hueco[2]-hueco[0]:.1f} x {hueco[3]-hueco[1]:.1f} mm\n"
            f"eje de ruedas = centro de la celda (linea roja)", fontsize=8, va="bottom")
    ax.set_xlim(-args.cell / 2 - 20, args.cell / 2 + 20)
    ax.set_ylim(-args.cell / 2 - 20, args.cell / 2 + 20)
    ax.set_aspect("equal")
    ax.set_title("Plantilla de celda AUS_KIM (vista desde arriba, mm)")
    ax.axis("off")
    ruta_png = os.path.join(args.out, "PLANTILLA_CELDA_vista.png")
    fig.savefig(ruta_png, dpi=90, bbox_inches="tight")
    print("PNG:", ruta_png)
except ImportError:
    print("(sin matplotlib: no se genera la vista previa)")

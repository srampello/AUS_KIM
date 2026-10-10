#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Genera AUS_KIM_RACE.ino a partir de la ultima version del Test Suite.

Fuente unica de firmware: firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino
La version de carrera conserva la navegacion, sensores, encoders y parametros
y solo usa la pantalla simplificada RMP como pagina de inicio.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino"
TARGET = ROOT / "firmware/06_race/AUS_KIM_RACE/AUS_KIM_RACE.ino"
SHARED_HEADERS = ("rmp_logo.h", "web_test_suite.h", "web_race.h")

ORIGINAL_ROOT = '''void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}'''
RACE_ROOT = '''void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", RACE_HTML);
}'''


def main():
    test_suite = SOURCE.read_text(encoding="utf-8")
    if test_suite.count(ORIGINAL_ROOT) != 1:
        raise SystemExit("No se encontro una unica funcion handleRoot() en Test Suite")
    if test_suite.count('#include "web_test_suite.h"') != 1 or test_suite.count('#include "web_race.h"') != 1:
        raise SystemExit("El Test Suite debe incluir ambas interfaces web externas")
    if 'server.on("/race", HTTP_GET, handleRace);' not in test_suite:
        raise SystemExit("Falta registrar el endpoint de carrera")

    race = test_suite.replace(ORIGINAL_ROOT, RACE_ROOT, 1)
    header = (
        "// ARCHIVO GENERADO AUTOMATICAMENTE - NO EDITAR A MANO.\n"
        "// Fuente: firmware/05_test_suite/AUS_KIM_TEST_SUITE/AUS_KIM_TEST_SUITE.ino\n"
        "// Regenerar: python tools/generar_aus_kim_race.py\n"
        "// La logica de manejo es IDENTICA al Test Suite. Solo cambia la portada.\n\n"
    )
    TARGET.parent.mkdir(parents=True, exist_ok=True)
    # Arduino requiere que los .h se encuentren junto a cada .ino.
    # Test Suite es la fuente unica; sincronizar logo y ambas interfaces web.
    for name in SHARED_HEADERS:
        origin = SOURCE.parent / name
        destination = TARGET.parent / name
        if not origin.is_file():
            raise SystemExit(f"Falta archivo fuente: {origin}")
        content = origin.read_bytes()
        if not destination.exists() or destination.read_bytes() != content:
            destination.write_bytes(content)
            print("Sincronizado:", destination.relative_to(ROOT))

    generated = header + race
    if TARGET.exists() and TARGET.read_text(encoding="utf-8") == generated:
        print("AUS_KIM_RACE.ino ya estaba sincronizado.")
    else:
        TARGET.write_text(generated, encoding="utf-8")
        print("Generado:", TARGET.relative_to(ROOT))


if __name__ == "__main__":
    main()

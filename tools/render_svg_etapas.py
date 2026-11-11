#!/usr/bin/env python3
"""Convierte docs/img/etapas/*.svg a PNG con Chromium (Playwright).
Uso: python tools/render_svg_etapas.py"""
import glob
import os
import re
from playwright.sync_api import sync_playwright

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "img", "etapas")
with sync_playwright() as p:
    b = p.chromium.launch()
    for svg in sorted(glob.glob(os.path.join(D, "*.svg"))):
        s = open(svg, encoding="utf-8").read()
        w, h = (int(v) for v in re.search(r'width="(\d+)" height="(\d+)"', s).groups())
        pg = b.new_page(viewport={"width": w, "height": h}, device_scale_factor=1)
        pg.goto("file://" + os.path.abspath(svg))
        pg.wait_for_timeout(200)
        pg.screenshot(path=svg[:-4] + ".png", full_page=False)
        pg.close()
        print("ok", os.path.basename(svg))
    b.close()

# -*- coding: utf-8 -*-
"""字体加载"""
from config import load_font

FONT = None
FONT_SM = None
FONT_BIG = None


def init_fonts():
    global FONT, FONT_SM, FONT_BIG
    if FONT is None:
        FONT = load_font(18)
        FONT_SM = load_font(15)
        FONT_BIG = load_font(28, bold=True)
# -*- coding: utf-8 -*-
"""太空背景 + 星星"""
import math
import random
import pygame

from config import WIDTH, HEIGHT


def draw_sky(surf, cam):
    """太空就是黑的。天空亮色由 earth 模块沿地表弧线绘制的大气提供。"""
    surf.fill((0, 0, 0))


# 固定屏幕位置 + 高度淡入 + 缓慢闪烁
STARS = [(random.random(), random.random(),
          random.random() * math.tau, random.uniform(0.4, 1.2))
         for _ in range(420)]


def draw_stars(surf, cam, alt, t):
    ta = (alt - 25_000.0) / 65_000.0
    ta = max(0.0, min(1.0, ta))
    if ta <= 0.01:
        return
    for fx, fy, phase, base in STARS:
        sx = int(fx * WIDTH)
        sy = int(fy * HEIGHT)
        blink = 0.7 + 0.3 * math.sin(t * 2.0 + phase)
        v = int(255 * ta * base * blink)
        v = max(0, min(255, v))
        if v > 10:
            surf.fill((v, v, v), (sx, sy, 1, 1))
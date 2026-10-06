# -*- coding: utf-8 -*-
"""月球：远看是一个带环形山的圆盘，贴近时按地表弧线填充。

和地球一样分两种模式：
  · 圆盘模式（屏幕半径 < 2000 px）：整块月面 + 环形山 + 明暗界线
  · 弧线模式：月面太大，沿可见边缘用扫描线多边形填充
位置一律取自 config.moon_state(t)，和物理共用同一份运动学，不会两边算错。
"""
import math
import random
import pygame

from config import WIDTH, HEIGHT, MOON_RADIUS, moon_state
from . import fonts


# 环形山：固定种子，反复启动长得一样
_CRATERS = None
_DISC_THRESHOLD = 2000.0


def _craters():
    global _CRATERS
    if _CRATERS is None:
        rng = random.Random(20240927)
        items = []
        for _ in range(90):
            while True:
                cx = rng.uniform(-1.0, 1.0)
                cy = rng.uniform(-1.0, 1.0)
                if cx * cx + cy * cy < 0.92:
                    break
            items.append((cx, cy, rng.uniform(0.02, 0.11),
                          rng.uniform(-22.0, 18.0)))
        _CRATERS = items
    return _CRATERS


def draw_moon(surf, cam, rocket=None):
    """画月球。rocket 只用来取时间 t（月球位置随时间变化）。"""
    t = rocket.t if rocket is not None else 0.0
    (mx, my), _, _ = moon_state(t)
    cx, cy = cam.w2s(mx, my)
    r = MOON_RADIUS * cam.scale

    if (cy + r < -50 or cy - r > HEIGHT + 50
            or cx + r < -50 or cx - r > WIDTH + 50):
        return

    if r < 1.5:
        # 太远就画一个亮点，至少能看出月亮在哪
        pygame.draw.circle(surf, (228, 228, 232), (int(cx), int(cy)), 2)
        return

    if r < _DISC_THRESHOLD:
        _draw_disc(surf, cx, cy, r)
    else:
        _draw_arc(surf, cam, mx, my, r)


def _draw_disc(surf, cx, cy, r):
    ri = max(2, int(r))
    cxi, cyi = int(cx), int(cy)
    pygame.draw.circle(surf, (150, 150, 156), (cxi, cyi), ri)

    if ri > 8:
        for fx, fy, fr, fb in _craters():
            px = cxi + int(fx * ri)
            py = cyi + int(fy * ri)
            rr = max(1, int(fr * ri))
            base = max(0, min(255, 150 + int(fb)))
            pygame.draw.circle(surf, (base, base, min(255, base + 6)), (px, py), rr)
            if rr > 2:
                pygame.draw.circle(surf, (min(255, base + 20),
                                          min(255, base + 20), min(255, base + 26)),
                                   (px - max(1, rr // 3), py - max(1, rr // 3)),
                                   max(1, rr // 2))

    # 明暗界线：一块偏心大圆当阴影，成本很低
    if ri > 12:
        shadow = pygame.Surface((ri * 2 + 4, ri * 2 + 4), pygame.SRCALPHA)
        pygame.draw.circle(shadow, (0, 0, 10, 150),
                           (ri + 2 - int(ri * 0.75), ri + 2 + int(ri * 0.15)),
                           int(ri * 2.2))
        surf.blit(shadow, (cxi - ri - 2, cyi - ri - 2))

    pygame.draw.circle(surf, (196, 196, 202), (cxi, cyi), ri, 2)

    if ri > 40 and fonts.FONT_SM is not None:
        s = fonts.FONT_SM.render("月球", True, (236, 236, 240))
        surf.blit(s, (cxi - s.get_width() // 2, cyi - ri - 20))


def _draw_arc(surf, cam, mwx, mwy, r):
    """月面弧线模式：取月面上"朝相机一侧"的点，逐列连线后向下填充。

    月心世界坐标 (mwx, mwy) 已知，所以某一列的世界 y = mwy + sqrt(R²-dx²)，
    和地球那边的取法一致（都取上半圆）。
    """
    step = 8 if cam.scale > 40 else (4 if cam.scale > 15 else 2)
    pts = []
    for sx in range(-step, WIDTH + 2 * step, step):
        wx = cam.x + (sx - WIDTH * 0.5) / cam.scale
        dx = wx - mwx
        if abs(dx) >= MOON_RADIUS:
            pts.append((sx, HEIGHT + 3000.0))
            continue
        wy = mwy + math.sqrt(max(0.0, MOON_RADIUS * MOON_RADIUS - dx * dx))
        _, sy = cam.w2s(wx, wy)
        pts.append((sx, max(-3000.0, min(HEIGHT + 3000.0, sy))))

    if len(pts) < 2:
        return
    poly = pts + [(WIDTH + 2 * step, HEIGHT + 3000.0),
                  (-step, HEIGHT + 3000.0)]
    pygame.draw.polygon(surf, (150, 150, 156), poly)
    pygame.draw.lines(surf, (196, 196, 202), False, pts, 2)

# -*- coding: utf-8 -*-
"""地球（球体 / 弧线） + 贴弧大气 + 地表建筑 + 昼夜分界 + 自转

大气渐变 Surface 缓存，避免低空每帧重建。
"""
import math
import random
import pygame

from config import WIDTH, HEIGHT, R_EARTH


_earth_decor = None
_ground_decor = None
_shadow_buf = None
_atmo_grad_cache = None
_atmo_grad_thickness = 0

_SY_MIN = -2000.0
_SY_MAX = HEIGHT + 2000.0


# ---------- 大气层颜色梯度 ----------
ATMO_STOPS = [
    (0.00, (200, 232, 252, 255)),
    (0.05, (172, 214, 246, 255)),
    (0.12, (142, 194, 240, 252)),
    (0.22, (112, 172, 234, 244)),
    (0.34, (84,  148, 222, 228)),
    (0.48, (60,  122, 202, 200)),
    (0.62, (40,  94,  178, 165)),
    (0.75, (24,  66,  146, 120)),
    (0.86, (12,  40,  106, 70)),
    (0.94, (5,   20,  62,  30)),
    (1.00, (0,   0,   0,   0)),
]


def _interp_rgba(stops, f):
    if f <= stops[0][0]:
        return stops[0][1]
    if f >= stops[-1][0]:
        return stops[-1][1]
    for i in range(len(stops) - 1):
        f0, c0 = stops[i]
        f1, c1 = stops[i + 1]
        if f0 <= f <= f1:
            t = (f - f0) / (f1 - f0)
            return (int(c0[0] + (c1[0] - c0[0]) * t),
                    int(c0[1] + (c1[1] - c0[1]) * t),
                    int(c0[2] + (c1[2] - c0[2]) * t),
                    int(c0[3] + (c1[3] - c0[3]) * t))
    return stops[-1][1]


def _get_atmo_grad(thickness):
    """缓存未拉伸的 1px 渐变竖列。thickness 变了才重建。"""
    global _atmo_grad_cache, _atmo_grad_thickness
    if _atmo_grad_cache is not None and _atmo_grad_thickness == thickness:
        return _atmo_grad_cache
    grad = pygame.Surface((1, thickness), pygame.SRCALPHA)
    inv = 1.0 / max(1, thickness - 1)
    for i in range(thickness):
        f = 1.0 - i * inv
        grad.set_at((0, i), _interp_rgba(ATMO_STOPS, f))
    _atmo_grad_cache = grad
    _atmo_grad_thickness = thickness
    return grad


# ---------- 装饰数据 ----------
def _build_earth_decor():
    rng = random.Random(20240927)
    continents = []
    for _ in range(14):
        continents.append((rng.uniform(-1, 1), rng.uniform(-1, 1),
                           rng.uniform(0.07, 0.26), rng.uniform(0.05, 0.18)))
    cities = []
    for _ in range(140):
        cities.append((rng.uniform(-1, 1), rng.uniform(-1, 1), rng.random()))
    return continents, cities


def _gen_ground_decor():
    global _ground_decor
    if _ground_decor is not None:
        return _ground_decor
    rng = random.Random(1234)
    items = [("tower", 8.0, 45.0)]
    for _ in range(24):
        x = rng.uniform(-420, 420)
        if abs(x) < 30:
            continue
        w = rng.uniform(8, 28)
        h = rng.uniform(5, 22)
        col = rng.choice([(90, 92, 98), (120, 110, 95),
                          (80, 85, 95), (105, 105, 110)])
        items.append(("bldg", x, w, h, col))
    _ground_decor = items
    return items


def surface_y_world(dx):
    if abs(dx) > R_EARTH:
        return None
    root = math.sqrt(max(R_EARTH * R_EARTH - dx * dx, 0.0))
    return -dx * dx / (R_EARTH + root)


def _get_shadow_buf():
    global _shadow_buf
    if _shadow_buf is None:
        _shadow_buf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    else:
        _shadow_buf.fill((0, 0, 0, 0))
    return _shadow_buf


# ---------- 地球主入口 ----------
def draw_earth(surf, cam, t):
    global _earth_decor
    if _earth_decor is None:
        _earth_decor = _build_earth_decor()
    continents, cities = _earth_decor

    cx, cy = cam.w2s(0.0, -R_EARTH)
    r = R_EARTH * cam.scale
    if r < 2:
        return

    if r < 2000.0:
        _draw_earth_disc(surf, cam, cx, cy, r, continents, cities, t)
    else:
        _draw_earth_arc(surf, cam)


def _draw_earth_disc(surf, cam, cx, cy, r, continents, cities, t):
    if cy + r < 0 or cy - r > HEIGHT + 200:
        return
    ri = max(2, int(r))
    cxi, cyi = int(cx), int(cy)

    if ri < 1500:
        ar = ri + max(6, int(r * 0.07))
        atmo = pygame.Surface((ar * 2 + 4, ar * 2 + 4), pygame.SRCALPHA)
        N = 24
        for k in range(N):
            f = k / (N - 1)
            radius = ri + int(f * (ar - ri))
            alpha = int(90 * (1 - f) ** 1.6)
            if alpha < 2:
                continue
            pygame.draw.circle(atmo, (110, 165, 230, alpha),
                               (ar + 2, ar + 2), radius)
        surf.blit(atmo, (cxi - ar - 2, cyi - ar - 2))

    pygame.draw.circle(surf, (30, 80, 145), (cxi, cyi), ri)

    rot_speed = 0.02

    if ri > 20:
        for lon0, lat, sx, sy in continents:
            lon = ((lon0 + t * rot_speed + 1) % 2) - 1
            if abs(lon) >= 0.99:
                continue
            shrink = math.sqrt(max(0.001, 1 - lon * lon))
            ox = lon * ri
            oy = lat * ri * shrink
            w = int(ri * sx * shrink)
            h = int(ri * sy)
            if w > 1 and h > 1:
                try:
                    pygame.draw.ellipse(
                        surf, (55, 115, 70),
                        pygame.Rect(cxi + int(ox) - w,
                                    cyi + int(oy) - h, w * 2, h * 2))
                except Exception:
                    pass

        for lon0, lat, br in cities:
            lon = ((lon0 + t * rot_speed + 1) % 2) - 1
            if abs(lon) >= 0.98:
                continue
            shrink = math.sqrt(max(0.001, 1 - lon * lon))
            ox = lon * ri
            oy = lat * ri * shrink
            dist = math.hypot(ox, oy) / max(ri, 1)
            if dist > 0.55:
                v = int(220 * br * min(1.0, (dist - 0.55) * 2.5))
                v = max(0, min(255, v))
                try:
                    surf.fill((v, int(v * 0.85), int(v * 0.5), 255),
                              (cxi + int(ox), cyi + int(oy), 1, 1))
                except Exception:
                    pass

    if ri > 20:
        shadow = _get_shadow_buf()
        sr = ri * 3
        scx = cxi - sr + int(0.35 * ri)
        scy = cyi + int(0.25 * ri)
        pygame.draw.circle(shadow, (0, 0, 12, 170), (scx, scy), sr)
        surf.blit(shadow, (0, 0))

    pygame.draw.circle(surf, (70, 140, 200), (cxi, cyi), ri, 2)


def _draw_earth_arc(surf, cam):
    """地表弧。步长随 scale 自适应。"""
    if cam.scale > 40:
        step = 8
    elif cam.scale > 15:
        step = 4
    elif cam.scale > 5:
        step = 2
    else:
        step = 1

    pts = []
    for sx in range(0, WIDTH + step, step):
        wx = cam.x + (sx - WIDTH * 0.5) / cam.scale
        wy = surface_y_world(wx)
        if wy is None:
            pts.append((sx, _SY_MAX))
            continue
        _, sy = cam.w2s(wx, wy)
        if sy > _SY_MAX:
            sy = _SY_MAX
        elif sy < _SY_MIN:
            sy = _SY_MIN
        pts.append((sx, sy))

    if len(pts) < 2:
        return

    _draw_atmo_along_arc(surf, cam, pts, step)

    poly = pts + [(WIDTH + step, _SY_MAX), (-step, _SY_MAX)]
    pygame.draw.polygon(surf, (30, 80, 145), poly)
    pygame.draw.lines(surf, (85, 155, 215), False, pts, 2)


def _draw_atmo_along_arc(surf, cam, pts, step=1):
    """贴弧大气。缓存 1px 渐变，按 step 横向拉伸后 blit。"""
    alt = max(0.0, cam.y)
    thickness = HEIGHT / (1.0 + alt / 8000.0)
    thickness = int(max(20, min(HEIGHT * 3 // 2, thickness)))

    grad = _get_atmo_grad(thickness)
    if step > 1:
        grad = pygame.transform.scale(grad, (step, thickness))

    for sx, sy in pts:
        top_y = int(sy) - thickness + 1
        if top_y > HEIGHT or top_y + thickness < 0:
            continue
        surf.blit(grad, (sx, top_y))


def draw_surface_objects(surf, cam):
    if cam.y > 12000:
        return
    if cam.scale > 30 or cam.scale < 0.05:
        return

    _, sy_surface = cam.w2s(0.0, 0.0)
    if sy_surface < -200 or sy_surface > HEIGHT + 200:
        return

    for item in _gen_ground_decor():
        if item[0] == "tower":
            _, x_off, h = item
            x0, y0 = cam.w2s(x_off, 0.0)
            x1, y1 = cam.w2s(x_off, h)
            w = max(2, int(1.0 * cam.scale))
            pygame.draw.rect(surf, (100, 100, 110),
                             (int(x0 - w), int(y1), w * 2, int(y0 - y1)))
            pygame.draw.rect(surf, (140, 140, 150),
                             (int(x0 - w * 2), int(y1 - 5), w * 4, 5))
        elif item[0] == "bldg":
            _, x_c, w, h, col = item
            x0, y0 = cam.w2s(x_c - w / 2, 0.0)
            x1, y1 = cam.w2s(x_c + w / 2, h)
            r = pygame.Rect(int(x0), int(y1),
                            max(1, int(x1 - x0)), max(1, int(y0 - y1)))
            pygame.draw.rect(surf, col, r)
            pygame.draw.rect(surf, (60, 60, 70), r, 1)
# -*- coding: utf-8 -*-
"""激波锥 + 音爆云"""
import math
import pygame

from config import WIDTH, HEIGHT, RHO0, H_SCALE
from .util import l2w


_shock_buf = None


def _get_shock_buf():
    global _shock_buf
    if _shock_buf is None:
        _shock_buf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    else:
        _shock_buf.fill((0, 0, 0, 0))
    return _shock_buf


def _sound_speed(alt):
    if alt < 11000:
        T = 288.15 - 0.0065 * alt
    elif alt < 20000:
        T = 216.65
    else:
        T = 216.65 + 0.001 * (alt - 20000)
    return math.sqrt(1.4 * 287.05 * max(T, 150.0))


def draw_shock_and_sonicboom(surf, cam, rocket):
    alt = rocket.altitude
    if alt > 40_000 or alt < 0:
        return
    vx, vy = rocket.body.velocity
    spd = math.hypot(vx, vy)
    if spd < 200:
        return
    a = _sound_speed(alt)
    M = spd / a
    if M < 0.85:
        return
    rho = RHO0 * math.exp(-alt / H_SCALE)
    if rho < 0.02:
        return

    pos = rocket.body.position
    theta = rocket.theta
    stack_len = rocket.stack_len
    half_ang = math.asin(min(1.0, 1.0 / M))
    tip_local_y = stack_len
    top_world = l2w(pos, theta, 0.0, tip_local_y)
    cone_len = max(3.0, stack_len * 0.9)

    alpha = int(140 * min(1.0, (M - 0.85) * 3.0) * min(1.0, rho * 3))
    if alpha > 5:
        line_surf = _get_shock_buf()
        for side in (-1, 1):
            end_local_y = tip_local_y - cone_len
            end_local_x = side * math.tan(half_ang) * cone_len
            end_world = l2w(pos, theta, end_local_x, end_local_y)
            p0 = cam.w2s(*top_world)
            p1 = cam.w2s(*end_world)
            pygame.draw.line(line_surf, (200, 220, 255, alpha), p0, p1, 2)
        surf.blit(line_surf, (0, 0))

    # 音爆云
    if 0.9 < M < 1.15 and rho > 0.1:
        mid_y = stack_len * 0.5
        mx, my = l2w(pos, theta, 0.0, mid_y)
        sx, sy = cam.w2s(mx, my)
        rr = max(6, int(1.5 * cam.scale))
        if rr > 2000:
            return
        cloud = pygame.Surface((rr * 2 + 4, rr * 2 + 4), pygame.SRCALPHA)
        c_alpha = int(90 * (1.0 - abs(M - 1.0) / 0.15))
        pygame.draw.circle(cloud, (240, 240, 250, c_alpha),
                           (rr + 2, rr + 2), rr)
        surf.blit(cloud, (int(sx) - rr - 2, int(sy) - rr - 2))
# -*- coding: utf-8 -*-
"""历史轨迹 + 引力预测轨道（RK4 预测 + overlay Surface 复用）"""
import math
import pygame

from config import (WIDTH, HEIGHT, R_EARTH, G0, MOON_RADIUS, MOON_MU,
                    moon_state, gravity_accel)


_trail_buf = None
_orbit_buf = None

TRAIL_MAX_SCALE = 8.0


def _get_buf(which):
    global _trail_buf, _orbit_buf
    if which == "trail":
        if _trail_buf is None:
            _trail_buf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        else:
            _trail_buf.fill((0, 0, 0, 0))
        return _trail_buf
    else:
        if _orbit_buf is None:
            _orbit_buf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        else:
            _orbit_buf.fill((0, 0, 0, 0))
        return _orbit_buf


def _to_screen_runs(cam, world_pts):
    W, H = WIDTH, HEIGHT
    margin = 200
    runs = []
    current = []
    for wx, wy in world_pts:
        sx, sy = cam.w2s(wx, wy)
        if -margin < sx < W + margin and -margin < sy < H + margin:
            current.append((sx, sy))
        else:
            if len(current) >= 2:
                runs.append(current)
            current = []
    if len(current) >= 2:
        runs.append(current)
    return runs


def draw_trail(surf, cam, trail):
    if len(trail) < 2:
        return
    if cam.scale > TRAIL_MAX_SCALE:
        return
    overlay = _get_buf("trail")
    n = len(trail)
    CHUNK = 120
    for start in range(0, n - 1, CHUNK):
        end = min(start + CHUNK + 1, n)
        chunk = trail[start:end]
        runs = _to_screen_runs(cam, chunk)
        if not runs:
            continue
        f = (start + end) * 0.5 / n
        col = (int(80 + 130 * f), int(130 + 100 * f),
               int(210 + 45 * f), 140)
        for run in runs:
            pygame.draw.lines(overlay, col, False, run, 2)
    surf.blit(overlay, (0, 0))


# ============================================================
# 引力预测（RK4 积分，含月球引力）
# ============================================================
def predict_trajectory(rocket, n_steps=200, dt_pred=2.0):
    """纯引力预测：地球 + 月球，撞到任一表面就停。

    用的是 config.gravity_accel，和物理里的引力项同一个来源；
    月球位置按时间推进（预测几十秒到几十分钟，月球会动）。
    """
    px, py = rocket.body.position.x, rocket.body.position.y
    vx, vy = rocket.body.velocity
    t = rocket.t
    pts = [(px, py)]

    for _ in range(n_steps):
        ax1, ay1 = gravity_accel(px, py, t)
        ax2, ay2 = gravity_accel(px + vx * dt_pred * 0.5,
                                 py + vy * dt_pred * 0.5, t + dt_pred * 0.5)
        ax3, ay3 = gravity_accel(
            px + (vx + ax1 * dt_pred * 0.5) * dt_pred * 0.5,
            py + (vy + ay1 * dt_pred * 0.5) * dt_pred * 0.5, t + dt_pred * 0.5)
        ax4, ay4 = gravity_accel(
            px + (vx + ax2 * dt_pred * 0.5) * dt_pred,
            py + (vy + ay2 * dt_pred * 0.5) * dt_pred, t + dt_pred)

        vx += dt_pred / 6.0 * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
        vy += dt_pred / 6.0 * (ay1 + 2 * ay2 + 2 * ay3 + ay4)
        px += vx * dt_pred
        py += vy * dt_pred
        t += dt_pred
        if py < 0:
            break
        mp, _, _ = moon_state(t)
        if math.hypot(px - mp[0], py - mp[1]) < MOON_RADIUS:
            break
        pts.append((px, py))
    return pts


def draw_predicted_orbit(surf, cam, rocket):
    if rocket.in_orbit:
        return
    if cam.scale > TRAIL_MAX_SCALE:
        return
    pts = predict_trajectory(rocket)
    if len(pts) < 2:
        return
    overlay = _get_buf("orbit")
    runs = _to_screen_runs(cam, pts)
    for run in runs:
        pygame.draw.lines(overlay, (120, 200, 255, 90), False, run, 1)
    surf.blit(overlay, (0, 0))
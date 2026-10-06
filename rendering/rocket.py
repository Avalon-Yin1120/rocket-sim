# -*- coding: utf-8 -*-
"""火箭本体：分级 / 整流罩 / TVC 摆管 / 尾焰 / RCS / 再入辉光"""
import math
import random
import pygame

from .util import l2w


def _flame_params(alt, rho):
    _ = alt
    exp = min(1.0, max(0.0, 1.0 - rho / 0.6))
    length_scale = 1.0 + 2.6 * exp
    width_scale = 1.0 + 1.8 * exp
    if exp < 0.3:
        col_out = (255, 180, 60)
        col_in = (255, 244, 180)
    elif exp < 0.7:
        col_out = (220, 180, 120)
        col_in = (240, 230, 210)
    else:
        col_out = (140, 180, 240)
        col_in = (220, 230, 255)
    return length_scale, width_scale, col_out, col_in


def draw_rocket(surf, cam, rocket):
    if rocket.crashed or rocket.broken or rocket.burned_up:
        return
    pos = rocket.body.position
    theta = rocket.theta

    y_local = 0.0
    for i in range(rocket.active_idx, len(rocket.stages)):
        st = rocket.stages[i]
        _draw_stage_poly(surf, cam, pos, theta, y_local, st,
                         i == rocket.active_idx)
        y_local += st.length

    # 载荷 / 整流罩（画在整叠结构的最顶端，与 _mass_properties 一致）
    if rocket.active_idx < len(rocket.stages):
        r_top = max(rocket.stages[-1].radius, rocket.payload_radius)
        p1 = l2w(pos, theta, -r_top, y_local)
        p2 = l2w(pos, theta, r_top, y_local)
        p3 = l2w(pos, theta, 0.0, y_local + rocket.payload_len)
        pts = [cam.w2s(*p1), cam.w2s(*p2), cam.w2s(*p3)]
        pygame.draw.polygon(surf, (232, 234, 240), pts)
        pygame.draw.polygon(surf, (95, 98, 108), pts, 1)
    else:
        px, py = cam.w2s(pos.x, pos.y + rocket.payload_len * 0.5)
        rr = max(3, int(rocket.payload_radius * cam.scale))
        pygame.draw.circle(surf, (232, 234, 240), (int(px), int(py)), rr)
        pygame.draw.circle(surf, (95, 98, 108), (int(px), int(py)), rr, 1)

    st = rocket.current_stage()
    # 尾焰只在"飞行中 + 有燃料 + 节流 > 0"时显示
    in_flight = (rocket.state != rocket.STATE_PRELAUNCH)
    engine_on = (st is not None and st.fuel > 0.0
                 and rocket.throttle > 0.01
                 and in_flight)

    if engine_on:
        r_nozzle = st.radius * 0.75
        noz_len = 0.9
        vis = rocket.delta * 4.0
        npts_local = [
            (-r_nozzle, 0.0), (r_nozzle, 0.0),
            (r_nozzle + (-noz_len) * math.sin(vis),
             (-noz_len) * math.cos(vis)),
            (-r_nozzle + (-noz_len) * math.sin(vis),
             (-noz_len) * math.cos(vis)),
        ]
        npts_screen = [cam.w2s(*l2w(pos, theta, lx, ly))
                       for lx, ly in npts_local]
        pygame.draw.polygon(surf, (140, 142, 152), npts_screen)
        pygame.draw.polygon(surf, (60, 62, 72), npts_screen, 1)

        rho = rocket.rho_local
        ls, ws, c_out, c_in = _flame_params(rocket.altitude, rho)
        fl = st.length * (0.55 + 0.30 * random.random()) * ls
        eff = theta + rocket.delta
        tip_wx = pos.x - math.sin(eff) * fl
        tip_wy = pos.y - math.cos(eff) * fl
        pA = l2w(pos, theta, -st.radius * 0.75 * ws, 0.0)
        pB = l2w(pos, theta, st.radius * 0.75 * ws, 0.0)
        pygame.draw.polygon(surf, c_out,
                            [cam.w2s(*pA), cam.w2s(*pB), cam.w2s(tip_wx, tip_wy)])
        pA2 = l2w(pos, theta, -st.radius * 0.40 * ws, 0.0)
        pB2 = l2w(pos, theta, st.radius * 0.40 * ws, 0.0)
        t2x = pos.x - math.sin(eff) * fl * 0.62
        t2y = pos.y - math.cos(eff) * fl * 0.62
        pygame.draw.polygon(surf, c_in,
                            [cam.w2s(*pA2), cam.w2s(*pB2), cam.w2s(t2x, t2y)])

    # RCS 喷流
    if st is not None and abs(rocket.rcs_cmd) > 0.05 and in_flight:
        stack_len = rocket.stack_len
        y_rcs = max(stack_len - 1.5, 1.0)
        side = -1.0 if rocket.rcs_cmd > 0 else 1.0
        p0 = l2w(pos, theta, side * st.radius, y_rcs)
        p1 = l2w(pos, theta, side * (st.radius + 1.6 * abs(rocket.rcs_cmd)), y_rcs)
        pygame.draw.line(surf, (255, 220, 120), cam.w2s(*p0), cam.w2s(*p1), 3)
        pygame.draw.circle(surf, (255, 245, 190), cam.w2s(*p1), 2)

    # 再入等离子鞘套
    if rocket.reentry_heat > 0.3e6:
        _draw_reentry_glow(surf, cam, rocket)


def _draw_reentry_glow(surf, cam, rocket):
    pos = rocket.body.position
    stack_len = rocket.stack_len
    radius_world = stack_len * 0.8

    intensity = min(1.0, rocket.reentry_heat / 5e6)
    alpha = int(180 * intensity)
    if alpha < 5:
        return

    cx, cy = cam.w2s(pos.x, pos.y + stack_len * 0.5)
    r_px = max(8, int(radius_world * cam.scale))
    if r_px > 300:
        r_px = 300
    glow = pygame.Surface((r_px * 2 + 4, r_px * 2 + 4), pygame.SRCALPHA)
    N = 8
    for k in range(N):
        f = k / N
        rr = int(r_px * (1 - f * 0.5))
        a = int(alpha * (0.4 + 0.6 * f))
        if a < 3:
            continue
        pygame.draw.circle(glow, (255, 130, 50, a),
                           (r_px + 2, r_px + 2), rr)
    surf.blit(glow, (int(cx) - r_px - 2, int(cy) - r_px - 2),
              special_flags=pygame.BLEND_RGBA_ADD)


def _draw_stage_poly(surf, cam, pos, theta, y_bottom, st, is_active):
    corners = [(-st.radius, y_bottom), (st.radius, y_bottom),
               (st.radius, y_bottom + st.length),
               (-st.radius, y_bottom + st.length)]
    screen_pts = [cam.w2s(*l2w(pos, theta, lx, ly)) for lx, ly in corners]
    body_col = (218, 221, 228) if is_active else (168, 170, 178)
    pygame.draw.polygon(surf, body_col, screen_pts)
    pygame.draw.polygon(surf, (92, 95, 104), screen_pts, 1)
    if is_active and st.fuel_frac > 0.02:
        frac = st.fuel_frac
        inner_r = st.radius * 0.65
        y_fb = y_bottom + st.length * (1 - frac) * 0.85 + st.length * 0.05
        y_ft = y_bottom + st.length * 0.95
        fcorners = [(-inner_r, y_fb), (inner_r, y_fb),
                    (inner_r, y_ft), (-inner_r, y_ft)]
        fpts = [cam.w2s(*l2w(pos, theta, lx, ly)) for lx, ly in fcorners]
        pygame.draw.polygon(surf, (255, 120, 80), fpts)
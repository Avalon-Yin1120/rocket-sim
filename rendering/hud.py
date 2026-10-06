# -*- coding: utf-8 -*-
"""右侧高度表 + 左上角 HUD（标签缓存优化）

HUD 会跟着主天体走：在月球影响球内显示的是"距月面高度 / 环绕月速度 /
近月点-远月点"，其余时候显示地球那一套。这样同一块面板在两个天体都能读。
"""
import math
import pygame

from config import (WIDTH, HEIGHT, R_EARTH, G0, MOON_RADIUS, MOON_MU,
                    ORBIT_MIN_PERIGEE_ALT, MOON_ORBIT_MIN_PERI)
from . import fonts


# ---- 标签渲染缓存 ----
_LABEL_CACHE = {}

# 制导阶段的中文名（见 Rocket._upper_stage_guidance）
_PHASE_TEXT = {"boost": "动力上升", "coast": "滑行", "circ": "圆化",
               "park": "停泊待机", "tli": "地月转移", "transit": "地月滑行",
               "loi": "近月制动", "done": "已关机"}

_MAX_EVENTS = 5


def _cached_label(text, color):
    key = (text, color)
    surf = _LABEL_CACHE.get(key)
    if surf is None:
        surf = fonts.FONT.render(text, True, color)
        _LABEL_CACHE[key] = surf
    return surf


def _nice_step(span, target=6):
    raw = span / max(target, 1)
    if raw <= 0:
        return 1.0
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def _fmt_alt(v):
    if abs(v) >= 1000:
        return f"{v/1000:,.1f} km"
    return f"{v:,.0f} m"


def _velocity_components(rocket):
    """相对**当前主天体**的径向 / 切向速度分解。"""
    vx = rocket.body.velocity.x
    vy = rocket.body.velocity.y
    if rocket.primary == "moon":
        mx, my = rocket.moon_pos
        mvx, mvy = rocket.moon_vel
        rx = rocket.body.position.x - mx
        ry = rocket.body.position.y - my
        vx -= mvx
        vy -= mvy
    else:
        rx = rocket.body.position.x
        ry = rocket.body.position.y + R_EARTH
    r = math.hypot(rx, ry)
    if r < 1e-6:
        return (0.0, 0.0)
    ux, uy = rx / r, ry / r
    tx, ty = uy, -ux
    return (vx * tx + vy * ty, vx * ux + vy * uy)


def draw_altimeter(surf, alt, max_alt, primary="earth"):
    """右侧高度表。alt 是 focus_alt（近月时即距月面高度）。"""
    bx = WIDTH - 128
    bw = 42
    by = 100
    bh = HEIGHT - 220

    panel = pygame.Surface((bw + 110, bh + 90), pygame.SRCALPHA)
    panel.fill((10, 14, 22, 165))
    surf.blit(panel, (bx - 96, by - 55))
    title = "月面高度表" if primary == "moon" else "高度表"
    surf.blit(fonts.FONT.render(title, True, (210, 220, 235)),
              (bx - 96 + 12, by - 48))

    pygame.draw.rect(surf, (36, 42, 55), (bx, by, bw, bh))
    pygame.draw.rect(surf, (110, 122, 145), (bx, by, bw, bh), 2)

    frac = max(0.0, min(1.0, alt / max_alt if max_alt > 0 else 0.0))
    fill_h = int(bh * frac)
    if fill_h > 0:
        pygame.draw.rect(surf, (72, 205, 132),
                         (bx + 2, by + bh - fill_h, bw - 4, fill_h))

    step = _nice_step(max_alt, 6)
    v = 0.0
    while v <= max_alt * 1.0001:
        y = by + bh * (1.0 - v / max_alt)
        pygame.draw.line(surf, (170, 180, 196),
                         (bx - 9, int(y)), (bx, int(y)), 1)
        lbl = fonts.FONT_SM.render(_fmt_alt(v), True, (196, 208, 226))
        surf.blit(lbl, (bx - 16 - lbl.get_width(),
                        int(y) - lbl.get_height() // 2))
        v += step

    y_now = by + bh * (1.0 - frac)
    pygame.draw.polygon(surf, (255, 96, 96),
                        [(bx + bw, y_now), (bx + bw + 12, y_now - 7),
                         (bx + bw + 12, y_now + 7)])
    pygame.draw.line(surf, (255, 255, 255),
                     (bx, int(y_now)), (bx + bw, int(y_now)), 2)

    surf.blit(fonts.FONT_BIG.render(_fmt_alt(alt), True, (255, 255, 255)),
              (bx - 96 + 12, by + bh + 14))
    surf.blit(fonts.FONT_SM.render(f"量程 0 - {_fmt_alt(max_alt)}",
                                   True, (150, 162, 180)),
              (bx - 96 + 12, by + bh + 48))


def draw_hud(surf, rocket, time_scale, eff_scale=None):
    st = rocket.current_stage()
    M, y_cm, _ = rocket._mass_properties()

    moon = rocket.primary == "moon"
    body_R = MOON_RADIUS if moon else R_EARTH
    body_mu = MOON_MU if moon else G0 * R_EARTH * R_EARTH
    min_peri = MOON_ORBIT_MIN_PERI if moon else ORBIT_MIN_PERIGEE_ALT
    body_name = "绕月轨道" if moon else "地球轨道"
    rp_word = "近月点" if moon else "近地点"
    ra_word = "远月点" if moon else "远地点"

    v_horiz, v_vert = _velocity_components(rocket)
    if moon:
        ref_r = max(rocket.moon_dist, MOON_RADIUS)
    else:
        ref_r = math.hypot(rocket.body.position.x,
                           rocket.body.position.y + R_EARTH)
    v_circ = math.sqrt(body_mu / ref_r) if ref_r > 1.0 else 0.0

    lines = [
        ("时间", f"{rocket.t:8.2f} s"),
        ("高度", f"{rocket.focus_alt:12,.1f} m"),
        ("最大高度", f"{rocket.max_alt:12,.1f} m"),
        ("垂直速度", f"{v_vert:12,.1f} m/s"),
        ("水平速度", f"{v_horiz:12,.1f} m/s"),
        ("环绕速度", f"{v_circ:12,.1f} m/s"),
        ("当前质量", f"{M:12,.1f} kg"),
        ("俯仰角", f"{math.degrees(rocket.theta):11.1f} °"),
        ("角速度", f"{math.degrees(rocket.omega):11.2f} °/s"),
        ("动压", f"{rocket.last_q/1000:11.2f} kPa"),
        ("攻角", f"{math.degrees(rocket.last_aoa):11.1f} °"),
        ("节流", f"{rocket.throttle*100:11.1f} %"),
        ("当地重力", f"{rocket.local_gravity:12.4f} m/s²"),
        ("风速", f"{rocket.wind_speed(max(rocket.altitude,0)):12.1f} m/s"),
        ("再入热流", f"{rocket.reentry_heat/1e6:11.2f} MW/m²"),
    ]
    if st is not None:
        lines.append(("当前级", st.name))
        lines.append(("本级燃料", f"{st.fuel_frac*100:11.1f} %"))
    else:
        lines.append(("当前级", "全部抛离"))

    rp_alt = rocket.orbit_rp - body_R
    ra_alt = rocket.orbit_ra - body_R
    if rocket.in_orbit:
        lines.append(("轨道", f"✓ {body_name}"))
        lines.append((rp_word, f"{rp_alt/1000:8.1f} km"))
        lines.append((ra_word, f"{ra_alt/1000:8.1f} km"))
    elif rocket.orbit_rp > 0:
        if rp_alt < 0:
            lines.append(("轨道", "亚轨道（会撞上）"))
            lines.append((rp_word, "撞面"))
            lines.append((ra_word, f"{ra_alt/1000:8.1f} km"))
        elif rp_alt < min_peri:
            lines.append(("轨道", "亚轨道（近心点过低）"))
            lines.append((rp_word, f"{rp_alt/1000:8.1f} km"))
            lines.append((ra_word, f"{ra_alt/1000:8.1f} km"))
        else:
            lines.append(("轨道", "亚轨道"))
            lines.append((rp_word, f"{rp_alt/1000:8.1f} km"))
            lines.append((ra_word, f"{ra_alt/1000:8.1f} km"))

    if rocket.mission == "lunar":
        lines.append(("距月心", f"{rocket.moon_dist/1000:11,.0f} km"))
        lines.append(("最近月距", f"{rocket.moon_min_dist/1000:11,.0f} km"))

    lines.append(("控制", "自动" if rocket.auto_mode else "手动"))
    lines.append(("执行", rocket.control_mode))
    lines.append(("制导阶段", _PHASE_TEXT.get(rocket.upper_phase,
                                             rocket.upper_phase)))
    lines.append(("主天体", "月球" if moon else "地球"))
    ts = f"{time_scale:.0f}×" if eff_scale is None else \
        f"{time_scale:.0f}×→{eff_scale:.0f}×"
    lines.append(("时倍", ts))

    if rocket.broken:
        lines.append(("状态", "!! 结构解体"))
    elif rocket.crashed:
        lines.append(("状态", "!! 撞月爆炸" if rocket.primary == "moon"
                      else "!! 撞地爆炸"))
    elif rocket.burned_up:
        lines.append(("状态", "!! 再入烧毁"))
    elif rocket.mission_done:
        lines.append(("状态", "✓ 任务完成"))
    elif rocket.landed_soft:
        lines.append(("状态", "软着陆"))

    events = rocket.events[-_MAX_EVENTS:]
    pad = 10
    lh = 18
    box_h = pad * 2 + lh * len(lines) + lh * (len(events) + 1)
    box_w = 320
    panel = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
    panel.fill((10, 14, 22, 150))
    surf.blit(panel, (16, 16))

    y = 16 + pad
    for k, v in lines:
        ks = _cached_label(k, (150, 164, 186))
        vs = fonts.FONT.render(v, True, (235, 242, 252))
        surf.blit(ks, (16 + pad, y))
        surf.blit(vs, (16 + pad + 110, y))
        y += lh

    y += lh // 2
    for t, msg in events:
        s = fonts.FONT_SM.render(f"[{t:7.1f}s] {msg}", True, (255, 208, 120))
        surf.blit(s, (16 + pad, y))
        y += lh

# -*- coding: utf-8 -*-
"""粒子和碎片渲染"""
import math
import pygame


def draw_particles(surf, cam, particles):
    for p in particles:
        t = max(0.0, p.life / p.max_life)
        col = (max(0, min(255, int(p.color[0] * t))),
               max(0, min(255, int(p.color[1] * t))),
               max(0, min(255, int(p.color[2] * t))))
        if col[0] + col[1] + col[2] < 20:
            continue
        sx, sy = cam.w2s(p.x, p.y)
        r = max(1, int(p.radius * (0.5 + t)))
        pygame.draw.circle(surf, col, (int(sx), int(sy)), r)


def draw_debris(surf, cam, d):
    st = d.stage
    c, s = math.cos(d.theta), math.sin(d.theta)
    corners = [(-st.radius, -st.length * 0.5), (st.radius, -st.length * 0.5),
               (st.radius, st.length * 0.5), (-st.radius, st.length * 0.5)]
    pts = [cam.w2s(d.x + lx * c + ly * s, d.y - lx * s + ly * c)
           for lx, ly in corners]
    pygame.draw.polygon(surf, (170, 172, 180), pts)
    pygame.draw.polygon(surf, (95, 98, 108), pts, 1)
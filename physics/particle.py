# -*- coding: utf-8 -*-
"""粒子：
  · Particle      —— 世界坐标粒子（爆炸、火光等）
  · LocalParticle —— 相对火箭本地坐标粒子（尾气）
"""
import math
import random


class Particle:
    __slots__ = ("x", "y", "vx", "vy", "life", "max_life", "color", "radius")

    def __init__(self, x, y, vx, vy, life, color, radius=2.0):
        self.x, self.y = x, y
        self.vx, self.vy = vx, vy
        self.life, self.max_life = life, life
        self.color = color
        self.radius = radius

    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.vy -= 6.0 * dt
        damp = 1 - min(1.0, dt * 0.6)
        self.vx *= damp
        self.vy *= damp
        self.life -= dt
        return self.life > 0


class LocalParticle:
    """相对火箭本地的粒子。
    lx, ly 是相对火箭参考点（喷管出口）的本地坐标。
    vx, vy 是本地坐标系里的漂移速度（不会因为火箭运动而改变）。
    x, y   是每帧渲染前由 Rocket 从 (lx, ly) 转换出的世界坐标。
    """
    __slots__ = ("lx", "ly", "vx", "vy", "life", "max_life",
                 "color", "radius", "x", "y")

    def __init__(self, lx, ly, vx, vy, life, color, radius=2.0):
        self.lx = lx
        self.ly = ly
        self.vx = vx
        self.vy = vy
        self.life = life
        self.max_life = life
        self.color = color
        self.radius = radius
        self.x = 0.0
        self.y = 0.0


def spawn_explosion(particles, x, y, n=90, scale=1.0, palette=None):
    if palette is None:
        palette = [(255, 220, 130), (255, 160, 60),
                   (255, 90, 30), (200, 60, 20)]
    for _ in range(n):
        ang = random.random() * math.tau
        spd = random.uniform(15, 70) * scale
        vx = math.cos(ang) * spd
        vy = math.sin(ang) * spd
        life = random.uniform(0.6, 1.8)
        col = random.choice(palette)
        rad = random.uniform(1.2, 3.0) * scale
        particles.append(Particle(x, y, vx, vy, life, col, rad))
# -*- coding: utf-8 -*-
"""级间分离 / 解体后的碎片（独立积分，受重力和阻力）"""
import math

from config import G0, R_EARTH, RHO0, H_SCALE


class Debris:
    def __init__(self, stage, pos, vel, theta, omega, label="分离级"):
        self.stage = stage
        self.x = float(pos.x)
        self.y = float(pos.y)
        self.vx = float(vel.x)
        self.vy = float(vel.y)
        self.theta = theta
        self.omega = omega
        self.t = 0.0
        self.landed = False
        self.label = label

    @property
    def altitude(self):
        """当前海拔（径向距离减地球半径），让每块碎片按自己的高度取风。"""
        return math.hypot(self.x, self.y + R_EARTH) - R_EARTH

    def update(self, dt, wind_x):
        if self.landed:
            self.omega *= (1 - min(1.0, dt * 1.5))
            self.theta += self.omega * dt
            self.t += dt
            return

        rx = self.x
        ry = self.y + R_EARTH
        r_center = math.hypot(rx, ry)
        alt = r_center - R_EARTH
        if r_center < 1.0:
            r_center = 1.0

        g = G0 * (R_EARTH / r_center) ** 2
        ux = -rx / r_center
        uy = -ry / r_center
        ax = g * ux
        ay = g * uy

        if 0.0 < alt < 200_000.0:
            rho = RHO0 * math.exp(-alt / H_SCALE)
            vxa = self.vx - wind_x
            vya = self.vy
            spd = math.hypot(vxa, vya)
            if spd > 1e-9 and rho > 1e-12:
                st = self.stage
                area = math.pi * st.radius * st.radius
                drag = 0.5 * rho * st.cd * area * spd
                ax -= (vxa / spd) * (drag / st.dry)
                ay -= (vya / spd) * (drag / st.dry)
            if spd > 10:
                self.omega *= (1 - min(1.0, dt * 0.5 * rho * spd * 0.02))

        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.theta += self.omega * dt
        self.t += dt

        rx = self.x
        ry = self.y + R_EARTH
        r_center = math.hypot(rx, ry)
        if r_center < R_EARTH:
            scale = R_EARTH / r_center
            self.x = rx * scale
            self.y = ry * scale - R_EARTH
            self.vx *= 0.3
            self.vy *= 0.3
            self.landed = True
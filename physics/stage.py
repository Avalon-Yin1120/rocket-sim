# -*- coding: utf-8 -*-
"""单级火箭参数与状态"""


class Stage:
    def __init__(self, cfg):
        self.name = cfg["name"]
        self.dry = float(cfg["dry"])
        self.fuel = float(cfg["fuel"])
        self.fuel0 = float(cfg["fuel"])
        self.thrust = float(cfg["thrust"])
        self.isp = float(cfg["isp"])
        self.radius = float(cfg["radius"])
        self.length = float(cfg["length"])
        self.cd = float(cfg["cd"])

    @property
    def mass(self):
        return self.dry + max(0.0, self.fuel)

    @property
    def fuel_frac(self):
        return self.fuel / self.fuel0 if self.fuel0 > 0 else 0.0
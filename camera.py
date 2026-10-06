# -*- coding: utf-8 -*-
"""相机：追焦 + 手动/自动缩放模式"""
import math
from config import WIDTH, HEIGHT, CAM_MAX_SCALE

# 手动模式允许的最小缩放：1e-7 px/m 时地月距离只有 38 px、地球半径 0.6 px，
# 手动缩到底可以把整个地月系统塞进屏幕。
MANUAL_MIN_SCALE = 1e-7

# 自动缩放的三个锚点（高度 px/m 的对应关系，全部对数插值）：
#   地面 40 → 200 km 0.05 → 1000 km 0.001 → 400000 km 2e-6
# 最后一段是为了地月转移：384400 km 时地月间距约 770 px，一眼能看全。
_SCALE_NEAR = (0.0, 40.0)
_SCALE_MID = (200_000.0, 0.05)
_SCALE_FAR = (1_000_000.0, 0.001)
_SCALE_LUNAR = (400_000_000.0, 2e-6)


def _log_interp(a, b, f):
    return math.exp(math.log(a) * (1.0 - f) + math.log(b) * f)


def _auto_target_scale(alt):
    """自动模式下按高度给出目标像素缩放（px/m），全程对数插值。"""
    if alt <= _SCALE_NEAR[0]:
        return _SCALE_NEAR[1]
    if alt <= _SCALE_MID[0]:
        f = math.log1p(alt / 200.0) / math.log1p(_SCALE_MID[0] / 200.0)
        return _log_interp(_SCALE_NEAR[1], _SCALE_MID[1], f)
    if alt <= _SCALE_FAR[0]:
        f = ((alt - _SCALE_MID[0]) / (_SCALE_FAR[0] - _SCALE_MID[0])) ** 0.5
        return _log_interp(_SCALE_MID[1], _SCALE_FAR[1], f)
    f = min(1.0, (alt - _SCALE_FAR[0]) / (_SCALE_LUNAR[0] - _SCALE_FAR[0]))
    return _log_interp(_SCALE_FAR[1], _SCALE_LUNAR[1], f)


class Camera:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.scale = 40.0
        self.manual_scale = 40.0    # 手动模式下的绝对缩放 (px/m)
        self.init = False
        self.mode = "auto"          # "auto" 或 "manual"
        self.rocket_screen_frac = 0.55
        self._alt_max = 500.0

    # ------------------------------------------------------------
    def toggle_mode(self):
        """自动 ↔ 手动。

        切到手动时直接锁定当前 scale。旧实现用**另一套**公式
        （4/(1+alt/2500)^0.9）反推 manual_zoom：切换瞬间虽然连续，之后缩放却会
        随高度按完全不同的曲线漂移，滚轮手感取决于火箭飞到哪了。
        """
        self.mode = "manual" if self.mode == "auto" else "auto"
        if self.mode == "manual":
            self.manual_scale = self.scale

    def update(self, dt, wx, wy, alt):
        if self.mode == "auto":
            target = _auto_target_scale(alt)
        else:
            # 手动模式：缩放只由滚轮决定，不随高度变化
            target = self.manual_scale

        target = max(MANUAL_MIN_SCALE, min(CAM_MAX_SCALE, target))

        if not self.init:
            self.scale = target
            self.init = True
        else:
            # 自动模式平滑一些，手动模式响应快一点
            k = min(1.0, dt * (3.0 if self.mode == "auto" else 6.0))
            self.scale += (target - self.scale) * k

        self.x = wx
        self.y = wy + (self.rocket_screen_frac - 0.5) * HEIGHT / self.scale

    def zoom(self, factor):
        # 滚轮：如果当前是自动模式，先切到手动再调
        if self.mode == "auto":
            self.toggle_mode()
        self.manual_scale *= factor
        self.manual_scale = max(MANUAL_MIN_SCALE,
                                min(CAM_MAX_SCALE, self.manual_scale))

    def update_altitude_span(self, dt, alt):
        """高度表量程上限的平滑。

        原先写在 main 里直接改 cam._alt_max，属于跨层伸私有字段；
        而且用的是原始帧时间，倍速越高量程越跟不上。
        """
        target_max = max(500.0, alt * 1.25)
        self._alt_max += (target_max - self._alt_max) * min(1.0, dt * 3.0)

    @property
    def alt_max(self):
        return self._alt_max

    def w2s(self, wx, wy):
        sx = (wx - self.x) * self.scale + WIDTH * 0.5
        sy = HEIGHT * 0.5 - (wy - self.y) * self.scale
        return (sx, sy)

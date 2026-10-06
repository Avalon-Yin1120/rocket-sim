# -*- coding: utf-8 -*-
"""rendering 包：把各模块的公开接口汇总，保持 `import rendering as R` 兼容"""
from . import fonts
from .fonts import init_fonts

from .background import draw_sky, draw_stars, STARS

from .earth import draw_earth, draw_surface_objects, surface_y_world

from .moon import draw_moon

from .objects import draw_particles, draw_debris

from .rocket import draw_rocket

from .effects import draw_shock_and_sonicboom

from .overlays import draw_trail, draw_predicted_orbit, predict_trajectory

from .hud import draw_altimeter, draw_hud


def __getattr__(name):
    """延迟代理 FONT / FONT_SM / FONT_BIG，
    保证 init_fonts() 之后再访问拿到的是真正的字体对象。"""
    if name in ("FONT", "FONT_SM", "FONT_BIG"):
        return getattr(fonts, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
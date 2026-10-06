# -*- coding: utf-8 -*-
"""无头渲染若干截图，供 README / 文档使用。

用法：

    python tools/make_screenshots.py                # 输出到 docs/images/
    python tools/make_screenshots.py --out D:\\tmp  # 换目录

用 SDL 的 dummy 视频驱动，不需要真实窗口，所以在 CI / 远程也能跑。
渲染顺序与 main.py 完全一致，画的是真程序的真画面。

关于"绕月"那张：地球轨道（orbit）那张是真飞出来的——跑完整制导直到入轨；
绕月（moon）那张直接把火箭放到一条低月轨上（构造初值，与
tools/moon_orbit_check.py 同样的做法），因为完整飞一次地月转移要几分钟。
"""
import argparse
import math
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pygame                                            # noqa: E402

from config import (WIDTH, HEIGHT, load_vehicle, MOON_MU,   # noqa: E402
                    MOON_RADIUS, moon_state)
import rendering as R                                    # noqa: E402
from camera import Camera                                # noqa: E402
from physics import Rocket, FlightLearner                # noqa: E402


def new_rocket(preset):
    veh = load_vehicle(preset)
    learner = FlightLearner(preset=veh["preset"], v_ref=veh["pitch_v_ref"])
    learner.reset_current()
    r = Rocket(veh, learner=learner)
    r.auto_mode = True
    return r


def step(r, seconds, dt_burn=0.05, dt_coast=0.5):
    """推进 seconds 秒物理时间。"""
    t = 0.0
    while t < seconds:
        dt = dt_burn if (r.engine_on
                         or r.state == r.STATE_PRELAUNCH) else dt_coast
        r.update(dt, False, False)
        t += dt


def step_until(r, cond, max_t=600000.0, dt_coast=0.5):
    t = 0.0
    next_check = 0.0
    while t < max_t:
        dt = 0.05 if (r.engine_on or r.state == r.STATE_PRELAUNCH) else dt_coast
        r.update(dt, False, False)
        t += dt
        if t >= next_check:
            next_check = t + 1.0
            if cond(r):
                return True
    return False


def render(r, cam, t_anim=0.0):
    """按 main.py 的顺序画一帧并返回 Surface。"""
    surf = pygame.Surface((WIDTH, HEIGHT))
    R.draw_sky(surf, cam)
    R.draw_stars(surf, cam, r.altitude, t_anim)
    R.draw_earth(surf, cam, t_anim)
    R.draw_moon(surf, cam, r)
    R.draw_surface_objects(surf, cam)
    R.draw_trail(surf, cam, r.trail)
    R.draw_predicted_orbit(surf, cam, r)
    for d in r.debris:
        R.draw_debris(surf, cam, d)
    R.draw_rocket(surf, cam, r)
    R.draw_shock_and_sonicboom(surf, cam, r)
    R.draw_particles(surf, cam, r.exhaust)
    R.draw_particles(surf, cam, r.particles)
    cam.update_altitude_span(1.0, r.focus_alt)
    R.draw_altimeter(surf, r.focus_alt, cam.alt_max, r.primary)
    R.draw_hud(surf, r, 1.0)
    return surf


def snap_camera(r, dt=0.05, scale=None):
    """新建相机并让它立刻收敛（Camera.init=False 时第一帧就取目标值）。

    scale 给了就切到手动模式并锁定该缩放——和用户滚滚轮得到的是同一种视角。
    为什么需要它：自动相机把火箭钉在屏幕纵向 55% 处，于是"能看到的地面深度"只有
    0.45×700/scale 米；起飞头 26 km 自动缩放下地面完全在屏幕外，画面是纯黑的。
    """
    cam = Camera()
    for _ in range(3):
        cam.update(dt, r.body.position.x, r.body.position.y, r.focus_alt)
    if scale is not None:
        cam.toggle_mode()
        cam.manual_scale = scale
        # 用一个大 dt 让平滑系数取到 1，否则只走 65% 就到不了目标缩放
        cam.update(1.0, r.body.position.x, r.body.position.y, r.focus_alt)
    cam.update_altitude_span(60.0, r.focus_alt)
    return cam


def place_in_lunar_orbit(r, alt=200_000.0):
    """把火箭直接放到一条圆月轨上（构造初值，用于截图）。"""
    r.state = r.STATE_FLIGHT
    r.active_idx = len(r.stages) - 1
    r.upper_phase = "done"
    r.earth_rp = r.earth_ra = 0.0
    rr = MOON_RADIUS + alt
    v = math.sqrt(MOON_MU / rr)
    mx, my = moon_state(0.0)[0]
    mvx, mvy = moon_state(0.0)[1]
    mwx, mwy = r.moon_pos
    r.body.position = (mwx, mwy + rr)          # 月球正上方
    # 注意要加上月球自身的速度：轨道根数算的是**相对月球**的速度，
    # 少加这 1023 m/s 会得到一条"必撞月"的亚轨道（HUD 会如实这么说）。
    r.body.velocity = (mvx + v, mvy)           # 相对月球切向
    r.theta = math.pi / 2
    r.altitude = math.hypot(r.body.position.x,
                            r.body.position.y + 6371000.0) - 6371000.0
    for _ in range(3):
        r.update(0.5, False, False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "docs", "images"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    pygame.init()
    pygame.display.set_mode((WIDTH, HEIGHT))
    R.init_fonts()

    shots = []

    # 1) 发射台：点火后 2 秒，用宽视角把发射塔和地面一起收进来
    r = new_rocket("light")
    step_until(r, lambda x: x.state == x.STATE_FLIGHT)
    step(r, 2.0)
    shots.append(("01-pad", r, 3.0, "发射台 · 一级点火、塔架与地表"))

    # 2) 上升段：Max-Q 附近，拉远到能看到地球弧线与大气层
    r2 = new_rocket("light")
    step_until(r2, lambda x: x.state == x.STATE_FLIGHT)
    step(r2, 55.0)
    shots.append(("02-ascent", r2, 0.012, "上升段 · 最大动压附近"))

    # 3) 地球轨道：真跑到入轨，拉远到能看见整个地球与上升轨迹
    r3 = new_rocket("light")
    step_until(r3, lambda x: x.mission_done, max_t=20000.0)
    shots.append(("03-orbit", r3, 6.0e-5, "地球轨道 · 整条上升轨迹"))

    # 4) 绕月：构造一条低月轨，取景能看到月球圆盘
    r4 = new_rocket("lunar")
    for _ in range(2):
        r4.update(0.05, False, False)
    place_in_lunar_orbit(r4, 200_000.0)
    shots.append(("04-moon", r4, 2.2e-4, "绕月轨道 · 距月面 200 km"))

    # 注：地月转移途中地球与月球分居两侧（各约 25 万公里），而相机把火箭钉在
    # 屏幕纵向 55% 处，可见范围只有 0.55×700/scale，两者无法同时入框
    # （试过 3e-6 px/m，地球和月球都在画面外）。这种"全程视角"更适合录 GIF。

    for name, rocket, scale, desc in shots:
        cam = snap_camera(rocket, scale=scale)
        surf = render(rocket, cam, t_anim=12.0)
        path = os.path.join(args.out, name + ".png")
        pygame.image.save(surf, path)
        print(f"  {name}.png  {desc}")
        print(f"     高度 {rocket.focus_alt/1000:,.1f} km  主天体 {rocket.primary}"
              f"  阶段 {rocket.upper_phase}  缩放 {cam.scale:.3g} px/m"
              f"  模式 {cam.mode}")

    print(f"\n截图已写入 {os.path.abspath(args.out)}")
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())

# -*- coding: utf-8 -*-
"""绕月轨道自检：把火箭直接放到一条已知的月球交会弹道上，看制导能不能捕获。

用法：
    python tools/moon_orbit_check.py
    python tools/moon_orbit_check.py --perilune 200      # 近月点 200 km
    python tools/moon_orbit_check.py --vinf 826          # 到达双曲线剩余速度

为什么单独做这个：
  月球转移的成功率受"5 天飞行 + 近月点极敏感"影响，端到端很难稳定复现；
  把捕获段单独拿出来测，才能分清"制导写错了"和"瞄准偏了"。
  这个脚本跳过发射与转移，直接构造交会初值，只检验近月制动 → 绕月轨道。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import (load_vehicle, MOON_MU, MOON_RADIUS, MOON_SOI,  # noqa: E402
                    moon_state)
from physics import Rocket, FlightLearner              # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default="lunar")
    ap.add_argument("--perilune", type=float, default=15000.0,
                    help="初始弹道的近月点高度 (km)")
    ap.add_argument("--vinf", type=float, default=826.0,
                    help="到达时的双曲线剩余速度 (m/s)")
    ap.add_argument("--dt", type=float, default=0.5)
    ap.add_argument("--max-t", type=float, default=400000.0)
    args = ap.parse_args()

    veh = load_vehicle(args.preset)
    lr = FlightLearner(preset=veh["preset"], v_ref=veh["pitch_v_ref"])
    lr.reset_current()
    r = Rocket(veh, learner=lr)
    r.auto_mode = True

    # 直接跳到"转移级正在工作"的状态
    r.state = Rocket.STATE_FLIGHT
    r.active_idx = len(r.stages) - 1
    r.engine_cut = False
    r.upper_phase = "transit"
    r.phase_t0 = 0.0
    r.ever_earth_orbit = True

    # 构造月球交会：在月心参考系里给一条双曲线，近月点 = perilune
    rp = MOON_RADIUS + args.perilune * 1000.0
    vinf = args.vinf
    vp = math.sqrt(vinf * vinf + 2.0 * MOON_MU / rp)
    # 起始点放在近月点之前 30 度的方向上
    nu0 = math.radians(-30.0)
    e = 1.0 + rp * vinf * vinf / MOON_MU
    p = rp * (1.0 + e)
    rr = p / (1.0 + e * math.cos(nu0))
    v_r = MOON_MU * e * math.sin(nu0) / math.sqrt(MOON_MU * p)
    v_t = math.sqrt(MOON_MU * p) / rr
    mx, my = moon_state(0.0)[0]
    mvx, mvy = moon_state(0.0)[1]
    ux, uy = math.cos(nu0), math.sin(nu0)
    relx = rr * ux
    rely = rr * uy
    relvx = v_r * ux - v_t * uy
    relvy = v_r * uy + v_t * ux
    r.body.position = (mx + relx, my + rely)
    # 位置和速度都要算上月球自身的运动：轨道根数用的是相对月球的速度，
    # 少加这 1023 m/s 会构造出一条"必撞月"的亚轨道，而不是想要的交会弹道。
    r.body.velocity = (mvx + relvx, mvy + relvy)

    print(f"构造交会: 近月点 {args.perilune:,.0f} km  v_inf {vinf:.0f} m/s "
          f"起始月心距 {rr / 1000:,.0f} km")
    print(f"{'t[s]':>9} {'距月[km]':>11} {'v_rel':>8} {'阶段':>8} "
          f"{'近月[km]':>10} {'远月[km]':>10} 燃料")

    t = 0.0
    next_report = 0.0
    while t < args.max_t:
        r.update(args.dt, False, False)
        t += args.dt
        if t >= next_report:
            st = r.current_stage()
            mrp = (r.moon_rp - MOON_RADIUS) / 1000 if r.moon_rp > 0 else float("nan")
            mra = (r.moon_ra - MOON_RADIUS) / 1000 if r.moon_ra > 0 else float("nan")
            vrel = math.hypot(*r.moon_relative_velocity())
            print(f"{r.t:9.1f} {r.moon_dist / 1000:11,.1f} {vrel:8.1f} "
                  f"{r.upper_phase:>8s} {mrp:10.1f} {mra:10.1f} "
                  f"{st.fuel if st else -1:.1f}")
            next_report += max(60.0, args.max_t / 20.0)
        if r.crashed or r.upper_phase == "done":
            break

    print("-" * 70)
    vrel = math.hypot(*r.moon_relative_velocity())
    print(f"结束: 阶段={r.upper_phase} 距月心={r.moon_dist / 1000:,.1f} km "
          f"v_rel={vrel:.1f} m/s")
    if r.moon_rp > 0:
        print(f"绕月轨道: 近月点 {(r.moon_rp - MOON_RADIUS) / 1000:,.1f} km / "
              f"远月点 {(r.moon_ra - MOON_RADIUS) / 1000:,.1f} km "
              f"e={r.moon_e:.3f}")
    if r.ever_moon_orbit:
        print("结果: ✓ 已进入绕月轨道")
        return 0
    print("结果: ✗ 未捕获" + ("（撞月）" if r.crashed else "（飞离月球）"))
    return 1


if __name__ == "__main__":
    sys.exit(main())

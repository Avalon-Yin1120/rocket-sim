# -*- coding: utf-8 -*-
"""轨道守恒自检：把火箭放进一条已知圆轨道，关掉发动机，看根数是否守恒。

用法：
    python tools/orbit_check.py
    python tools/orbit_check.py --alt 400000 --dt 0.05 --dur 300

为什么需要它：
  · 这是"轨道计算是否自洽"的判据。没有任何力做功时，半长轴、近地点、远地点
    必须**严格**守恒（数值积分只允许极小的振荡）。
  · 早期版本把"发射场自转线速度"逐帧贴到地面系轨迹上，这条自检会立刻失败
    （实测 300 秒能量涨 16%），而画面上完全看不出来。
  · 顺带验证两套积分路径：dt ≤ 0.06 走子步显式欧拉，dt > 0.06 走 RK4。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import load_stages, G0, R_EARTH            # noqa: E402
from physics import Rocket                              # noqa: E402

MU = G0 * R_EARTH * R_EARTH
TOL = 1e-6          # 相对能量漂移容差


def elements(rocket):
    vx, vy = rocket.inertial_velocity()
    rx = rocket.body.position.x
    ry = rocket.body.position.y + R_EARTH
    r = math.hypot(rx, ry)
    eps = (vx * vx + vy * vy) / 2.0 - MU / r
    h = rx * vy - ry * vx
    e = math.sqrt(max(0.0, 1.0 + 2.0 * eps * h * h / (MU * MU)))
    a = -MU / (2.0 * eps) if eps < 0 else float("inf")
    return eps, a, a * (1 - e) - R_EARTH, a * (1 + e) - R_EARTH


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alt", type=float, default=400_000.0)
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--dur", type=float, default=300.0)
    args = ap.parse_args()

    rocket = Rocket(load_stages())
    rocket.auto_mode = False             # 关掉制导
    rocket.engine_cut = True             # 关掉发动机
    rocket.state = Rocket.STATE_FLIGHT   # 跳过倒计时
    rocket.t = 0.0
    r = R_EARTH + args.alt
    v = math.sqrt(MU / r)
    rocket.body.position = (0.0, args.alt)
    rocket.body.velocity = (v, 0.0)      # 圆轨道：速度垂直于地心矢径
    rocket.altitude = args.alt

    eps0, a0, rp0, ra0 = elements(rocket)
    print(f"初始条件: 圆轨道 alt={args.alt / 1000:.0f} km v={v:.3f} m/s "
          f"dt={args.dt}")
    print(f"初始根数: eps={eps0:.6f} rp={rp0 / 1000:.4f} km "
          f"ra={ra0 / 1000:.4f} km")
    print(f"{'t':>8} {'eps':>18} {'rp[km]':>10} {'ra[km]':>10} {'alt[km]':>9}")

    t = 0.0
    worst = 0.0
    while t < args.dur:
        rocket.update(args.dt, False, False)
        t += args.dt
        eps, a, rp, ra = elements(rocket)
        worst = max(worst, abs(eps - eps0) / abs(eps0))
        if abs(t % 30.0) < args.dt / 2 or t >= args.dur - 1e-9:
            print(f"{t:8.1f} {eps:18.6f} {rp / 1000:10.4f} {ra / 1000:10.4f} "
                  f"{rocket.altitude / 1000:9.4f}")

    print(f"最大相对能量漂移: {worst:.3e}（容差 {TOL:.0e}）")
    if worst > TOL:
        print("结论: 失败——存在非保守力或参考系不一致")
        return 1
    print("结论: 通过——轨道根数守恒")
    return 0


if __name__ == "__main__":
    sys.exit(main())

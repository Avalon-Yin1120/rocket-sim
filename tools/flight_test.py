# -*- coding: utf-8 -*-
"""无头飞行测试：不开窗口，跑完整自动飞行并打印遥测。

用法（在项目根目录，也就是 main.py 所在目录执行）：

    python tools/flight_test.py                          # 默认预设
    python tools/flight_test.py --preset lunar           # 探月火箭
    python tools/flight_test.py --preset medium --csv f.csv
    python tools/flight_test.py --preset lunar --full --max-t 700000

为什么需要它：
  · 物理结论（Max-Q 多少、什么时候燃尽、到底有没有入轨/捕获）不该只能靠肉眼看画面；
  · 不依赖 pygame 的显示子系统，所以在没有显示器的环境（CI、远程）也能跑；
  · 输出可与上一次运行逐行对比，作为改物理之后的回归依据；
  · 制导参数可以在这里扫描（--apogee / --loi-trigger / --loi-stop ...）。

本文件只 import config 与 physics，不触碰 rendering，也不会写任何文件
（不会调用 FlightLearner.observe_flight_end）。
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import (load_vehicle, G0, R_EARTH, MOON_RADIUS, MOON_SOI,  # noqa: E402
                    MOON_DIST, CONFIG_SOURCE)
from physics import Rocket, FlightLearner              # noqa: E402


def ideal_dv(veh):
    """火箭方程分段递推。返回 (总 Δv, 各级 Δv 列表, 起飞质量)。"""
    stages = veh["stages"]
    total = veh["payload"] + sum(s["dry"] + s["fuel"] for s in stages)
    m = total
    dv = 0.0
    parts = []
    for s in stages:
        ve = s["isp"] * G0
        m_after = m - s["fuel"]
        d = ve * math.log(m / m_after)
        parts.append(d)
        dv += d
        m = m_after - s["dry"]
    return dv, parts, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", default=None,
                    help="内置预设名（sounding/light/medium/lunar）；"
                         "不给就按 rocket_config.json")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="推力段步长 (s)，默认 0.05 与 main.py 一致")
    ap.add_argument("--coast-dt", type=float, default=0.5,
                    help="滑行段步长 (s)，默认 0.5")
    ap.add_argument("--max-t", type=float, default=1200.0,
                    help="最长物理时间 (s)")
    ap.add_argument("--report-every", type=float, default=20.0,
                    help="每隔多少秒打印一行遥测")
    ap.add_argument("--csv", default="", help="把逐步遥测写到该文件")
    ap.add_argument("--default-table", action="store_true",
                    help="忽略学习表，使用分析式默认俯仰表")
    ap.add_argument("--v-ref", type=float, default=None,
                    help="覆盖俯仰参考速度 (m/s)：越小重力转弯越快（并重置该预设的表）")
    ap.add_argument("--full", action="store_true",
                    help="任务结束/失败后不提前退出，跑满 --max-t")
    ap.add_argument("--manual", action="store_true",
                    help="手动模式起飞且不按任何键（验证点火后无需按住 W）")
    ap.add_argument("--apogee", type=float, default=None,
                    help="覆盖末级制导的远地点目标 (km)")
    ap.add_argument("--circ-perigee", type=float, default=None,
                    help="覆盖末级制导的圆化近地点目标 (km)")
    ap.add_argument("--circ-vradial", type=float, default=None,
                    help="覆盖圆化再点火时的径向速度阈值 (m/s)")
    ap.add_argument("--loi-trigger", type=float, default=None,
                    help="覆盖近月制动触发距离 (km，月心距)")
    ap.add_argument("--loi-stop", type=float, default=None,
                    help="覆盖近月制动停止系数（相对环绕速度）")
    ap.add_argument("--perilune", type=float, default=None,
                    help="覆盖 TLI 期望近月点高度 (km)")
    args = ap.parse_args()

    veh = load_vehicle(args.preset)
    dv, parts, m0 = ideal_dv(veh)
    print(f"参数来源 : {CONFIG_SOURCE}")
    print(f"火箭     : {veh['title']}（预设 {veh['preset']}）")
    print(f"任务     : {veh['mission']}"
          + ("（入轨 → 地月转移 → 绕月）" if veh["mission"] == "lunar" else "（入地球轨道）"))
    print(f"起飞质量 : {m0:,.1f} kg   载荷 {veh['payload']:.0f} kg "
          f"({veh['payload'] / m0 * 100:.2f}%)   {len(veh['stages'])} 级")
    print(f"一级推重比: {veh['stages'][0]['thrust'] / (m0 * G0):.2f}   "
          f"俯仰参考速度 {veh['pitch_v_ref']:.0f} m/s")
    print("理想 Δv  : " + " + ".join(f"{p:.0f}" for p in parts)
          + f" = {dv:,.0f} m/s")
    print("-" * 92)

    learner = FlightLearner(preset=veh["preset"], v_ref=veh["pitch_v_ref"])
    if args.v_ref is not None:
        learner.v_ref = args.v_ref
        learner.reset_current()
        print(f"俯仰表   : 按 v_ref={args.v_ref:.0f} m/s 重置分析式初值")
    elif args.default_table:
        learner.reset_current()
        print("俯仰表   : 分析式默认（忽略学习表）")
    else:
        print(f"俯仰表   : 第 {learner.generation} 代（预设 {veh['preset']}）")

    rocket = Rocket(veh, learner=learner)
    rocket.auto_mode = not args.manual
    if args.manual:
        print("模式     : 手动（且全程不按任何键，只验证点火后能自己起飞）")
    if args.apogee is not None:
        rocket.apogee_target_alt = args.apogee * 1000.0
        print(f"制导     : 远地点目标 {args.apogee:.0f} km")
    if args.circ_perigee is not None:
        rocket.circ_target_perigee_alt = args.circ_perigee * 1000.0
        print(f"制导     : 圆化近地点目标 {args.circ_perigee:.0f} km")
    if args.circ_vradial is not None:
        rocket.circ_relight_vradial = args.circ_vradial
        print(f"制导     : 圆化再点火径向速度阈值 {args.circ_vradial:.0f} m/s")
    if args.loi_trigger is not None:
        rocket.loi_trigger_dist = args.loi_trigger * 1000.0
        print(f"制导     : 近月制动触发距离 {args.loi_trigger:,.0f} km")
    if args.loi_stop is not None:
        rocket.loi_stop_ratio = args.loi_stop
        print(f"制导     : 近月制动停止系数 {args.loi_stop:.3f}")
    if args.perilune is not None:
        import physics.rocket as PR
        PR.TLI_PERILUNE_ALT = args.perilune * 1000.0
        print(f"制导     : TLI 期望近月点高度 {args.perilune:,.0f} km")
    print("-" * 92)

    # 推完倒计时（预发射段只需要一个占位参数）
    guard = 0
    while rocket.state == Rocket.STATE_PRELAUNCH and guard < 100000:
        rocket.update(args.dt, False, False)
        guard += 1

    moon_mission = rocket.mission == "lunar"
    header = (f"{'t[s]':>10} {'focus[km]':>11} {'v[m/s]':>10} {'q[kPa]':>7} "
              f"{'thr':>5} {'级':>3} {'阶段':>8} {'e_rp[km]':>10} "
              f"{'e_ra[km]':>11}")
    if moon_mission:
        header += f" {'距月心[km]':>12} {'m_rp[km]':>9}"
    print(header)
    print("-" * 92)

    rows = []
    max_q, max_q_t, max_q_focus = 0.0, 0.0, 0.0
    min_moon = MOON_DIST
    min_moon_t = 0.0
    t = 0.0
    next_report = 0.0
    while t < args.max_t:
        dt = args.dt if (rocket.engine_on
                         or rocket.state == Rocket.STATE_PRELAUNCH) else args.coast_dt
        rocket.update(dt, False, False)
        t += dt
        vx, vy = rocket.body.velocity
        spd = math.hypot(vx, vy)
        if rocket.last_q > max_q:
            max_q, max_q_t, max_q_focus = rocket.last_q, rocket.t, rocket.focus_alt
        if rocket.moon_dist < min_moon:
            min_moon, min_moon_t = rocket.moon_dist, rocket.t

        if args.csv:
            rows.append((round(rocket.t, 2), round(rocket.focus_alt, 1),
                         round(spd, 2), round(rocket.last_q, 1),
                         round(rocket.throttle, 3), rocket.active_idx,
                         rocket.upper_phase, round(rocket.theta, 4),
                         round(rocket.earth_rp - R_EARTH, 1),
                         round(rocket.earth_ra - R_EARTH, 1),
                         round(rocket.moon_dist, 1),
                         round(rocket.moon_rp - MOON_RADIUS, 1),
                         int(rocket.in_orbit), int(rocket.mission_done)))

        if rocket.t >= next_report:
            line = (f"{rocket.t:10.1f} {rocket.focus_alt / 1000:11.2f} "
                    f"{spd:10.1f} {rocket.last_q / 1000:7.2f} "
                    f"{rocket.throttle:5.2f} {rocket.active_idx:3d} "
                    f"{rocket.upper_phase:>8s} "
                    f"{(rocket.earth_rp - R_EARTH) / 1000:10.1f} "
                    f"{(rocket.earth_ra - R_EARTH) / 1000:11.1f}")
            if moon_mission:
                mrp = (rocket.moon_rp - MOON_RADIUS) / 1000 if rocket.moon_rp > 0 \
                    else float("nan")
                line += f" {rocket.moon_dist / 1000:12,.0f} {mrp:9.1f}"
            print(line)
            next_report += args.report_every

        if rocket.mission_done and not args.full:
            break
        if (not args.full
                and (rocket.crashed or rocket.broken or rocket.burned_up
                     or rocket.landed_soft)):
            break

    print("-" * 92)
    print(f"最大动压 : {max_q / 1000:.1f} kPa @ t={max_q_t:.1f}s "
          f"alt={max_q_focus / 1000:.1f}km")
    print(f"结束状态 : t={rocket.t:.1f}s 高度={rocket.focus_alt / 1000:,.1f}km "
          f"速度={math.hypot(rocket.body.velocity.x, rocket.body.velocity.y):.1f}m/s "
          f"级={rocket.active_idx} 阶段={rocket.upper_phase} "
          f"主天体={rocket.primary}")
    print(f"标志     : crashed={rocket.crashed} broken={rocket.broken} "
          f"burned_up={rocket.burned_up} landed_soft={rocket.landed_soft} "
          f"in_orbit={rocket.in_orbit} mission_done={rocket.mission_done}")
    if rocket.earth_rp > 0:
        print(f"地球轨道 : 近地点 {(rocket.earth_rp - R_EARTH) / 1000:,.1f} km / "
              f"远地点 {(rocket.earth_ra - R_EARTH) / 1000:,.1f} km")
    if moon_mission:
        print(f"最近月距 : {min_moon / 1000:,.1f} km @ t={min_moon_t:.1f}s "
              f"（月球影响球 {MOON_SOI / 1000:,.0f} km）")
        if rocket.moon_rp > 0:
            print(f"绕月轨道 : 近月点 {(rocket.moon_rp - MOON_RADIUS) / 1000:,.1f} km"
                  f" / 远月点 {(rocket.moon_ra - MOON_RADIUS) / 1000:,.1f} km"
                  f"  e={rocket.moon_e:.3f}")
        print(f"任务结果 : {'✓ 已进入绕月轨道' if rocket.ever_moon_orbit else '✗ 未进入绕月轨道'}"
              + ("（错过月球）" if rocket.missed_moon else ""))
    print("事件：")
    for tt, msg in rocket.events:
        print(f"  [{tt:9.1f}s] {msg}")

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write("t,focus_alt,v,q,throttle,stage,phase,theta,"
                    "earth_rp,earth_ra,moon_dist,moon_rp,in_orbit,mission_done\n")
            for r in rows:
                f.write(",".join(str(x) for x in r) + "\n")
        print(f"遥测已写入 {os.path.abspath(args.csv)}（{len(rows)} 行）")

    return 0


if __name__ == "__main__":
    sys.exit(main())

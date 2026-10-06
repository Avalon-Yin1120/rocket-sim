# -*- coding: utf-8 -*-
"""火箭：动力学 / 姿态控制 / 级间逻辑 / 轨道计算"""
import math
import random
import pymunk

from config import (G0, R_EARTH, RHO0, H_SCALE, PAYLOAD,
                    PAYLOAD_LEN, PAYLOAD_RADIUS, PAYLOAD_CD,
                    MOON_MU, MOON_RADIUS, MOON_DIST, MOON_SOI, MOON_OMEGA,
                    MOON_ORBIT_MIN_PERI, MOON_ORBIT_EXIT_PERI,
                    moon_state, gravity_accel,
                    TVC_MAX_DEG_LOW, TVC_MAX_DEG_UP, TVC_ACT_TAU,
                    TVC_RATE_MAX_DEG, TVC_BIAS_DEG,
                    RCS_FORCE, RCS_LIMIT,
                    CRASH_SPEED, BREAK_Q, BREAK_AOA, BURNUP_HEAT,
                    ORBIT_MIN_PERIGEE_ALT, ORBIT_EXIT_PERIGEE_ALT,
                    LAUNCH_LAT_DEG, EARTH_OMEGA, COUNTDOWN_SECONDS,
                    CONFIG_SOURCE)

from .stage import Stage
from .particle import Particle, LocalParticle, spawn_explosion
from .debris import Debris
from .autolearn import FlightLearner


RK4_DT_THRESHOLD = 0.06

# ---- 末级制导（判据驱动，不依赖具体火箭参数）----
APOGEE_TARGET_ALT = 300_000.0          # 第一段 burn 把远地点抬到这里就滑行 (m)
CIRC_TARGET_PERIGEE_ALT = ORBIT_MIN_PERIGEE_ALT + 20_000.0   # 圆化目标近地点 (m)
# 圆化再点火判据：径向速度降到这个值就点火（0 = 正好在远地点）。
# 实测偏差别很大：0 m/s 时最终远地点 1234 km，提前到 +120 m/s 就变成 1755 km——
# 在远地点之外的地方加速等于把能量灌进远地点，越烧越扁。
CIRC_RELIGHT_VRADIAL = 0.0
COAST_MAX_TIME = 12000.0               # 滑行段最长时间（要能等下一圈远地点）(s)
# 入轨任务里，圆化 burn 停机的额外条件：远地点-近地点之差。
# 只按"近地点 ≥ 目标"停机会在燃料还有富余时留下很扁的轨道；让它继续烧到
# 形状也收敛，结果更接近真实入轨。探月任务例外——停泊轨道越省越好，
# 剩下的 Δv 要留给 TLI 和近月制动。
CIRC_MAX_SPREAD = 1_500_000.0
# 圆化阶段的"保命底线"：近地点低于这个高度时，即使不在远地点窗口里也必须继续烧。
# 只按远地点窗口烧虽然效率最高，但近地点还在地下时一旦熄火，火箭会在飞到下一个
# 远地点之前就再入（实测 3 级火箭因此烧掉全部燃料后坠毁）。
CIRC_SAFE_PERIGEE = 130_000.0

# ---- 探月制导 ----
# 期望近月点高度。故意定得**很远**（15000 km）：
#   · 瞄准精度：偏移 ±5000 km 只会让交会距离变化 ±4800 km，仍然是安全的远距交会，
#     不会像 200 km 目标那样在"撞月"和"擦肩 25 万公里"之间随机跳；
#   · 捕获代价：远距交会的相对速度低（约 1.1 km/s），一次约 570 m/s 的减速就能
#     把近月点抬到当前半径，直接得到一条绕月轨道；
#   · 真实任务也是先捕获到大椭圆轨道再慢慢圆化，这里只是把第一步做扎稳。
TLI_PERILUNE_ALT = 15_000_000.0
LOI_TRIGGER_DIST = 5.0e7        # 月心距小于这个值就允许进入近月制动（精度靠时机）
LOI_DANGER_PERI = 100_000.0    # 预测近月点低于这个高度就不等距离门限，提前制动 (m)
LOI_STOP_RATIO = 1.02          # 相对速度降到 sqrt(mu_m/r) × 该系数即停（>1 更安全）
PARK_MAX_TIME = 4.0 * 86400.0  # 停泊轨道最长等待时间 (s)


def _cull_dead_abs(plist, dt):
    """绝对速度粒子（爆炸等）"""
    i = 0
    while i < len(plist):
        if plist[i].update(dt):
            i += 1
        else:
            plist[i] = plist[-1]
            plist.pop()


class Rocket:
    SUBSTEPS_MIN = 18
    SUBSTEPS_PER_SEC = 400
    SUBSTEPS_MAX = 250

    STATE_PRELAUNCH = "prelaunch"
    STATE_FLIGHT = "flight"

    def __init__(self, cfg, learner=None, mission=None):
        if isinstance(cfg, list):        # 兼容旧接口：直接给分级列表
            cfg = {"stages": cfg}

        self.preset = cfg.get("preset", "custom")
        self.vehicle_title = cfg.get("title", self.preset)
        self.mission = mission or cfg.get("mission", "leo")
        self.payload = float(cfg.get("payload", PAYLOAD))
        self.payload_len = float(cfg.get("payload_len", PAYLOAD_LEN))
        self.payload_radius = float(cfg.get("payload_radius", PAYLOAD_RADIUS))
        self.pitch_v_ref = float(cfg.get("pitch_v_ref", 3500.0))

        self.space = pymunk.Space()
        self.space.gravity = (0.0, 0.0)

        self.body = pymunk.Body(1.0, float("inf"))
        self.body.position = (0.0, 0.0)
        self.body.velocity = (0.0, 0.0)
        self.body.moment = float("inf")
        self.space.add(self.body)

        self.stages = [Stage(c) for c in cfg["stages"]]
        self.active_idx = 0

        self.state = self.STATE_PRELAUNCH
        self.countdown_t = COUNTDOWN_SECONDS

        self.theta = 0.0
        self.omega = 0.0
        self.delta = 0.0
        self.rcs_cmd = 0.0
        self.tvc_bias = math.radians(TVC_BIAS_DEG)

        self.t = 0.0
        self.altitude = 0.0
        self.max_alt = 0.0
        self.max_speed = 0.0
        self.last_accel = 0.0
        self.events = []
        self.debris = []
        self.particles = []      # 世界坐标粒子（爆炸）
        self.exhaust = []        # 本地坐标粒子（尾气）
        self.trail = []

        self._exhaust_accum = 0.0

        self.auto_mode = False
        self.engine_cut = False     # 发动机主阀：True = 已关机（W 键锁存切换）
        self.crashed = False
        self.broken = False
        self.burned_up = False
        self.landed_soft = False
        self.in_orbit = False
        self.used_auto = False
        self.control_mode = "TVC"

        self.last_q = 0.0
        self.last_aoa = 0.0
        self.throttle = 0.0
        self.engine_on = False       # 主循环按它决定积分步长（推力段必须细）
        self.reentry_heat = 0.0

        self.orbit_rp = 0.0
        self.orbit_ra = 0.0
        self.orbit_e = 0.0
        self.launch_boost = 0.0      # 仅用于显示：真实发射场的自转借力（未建模）
        self.upper_phase = "boost"   # 制导阶段，见 _upper_stage_guidance
        self.phase_t0 = 0.0          # 进入当前制导阶段的时刻

        # 天体状态：月球位置每块重算一次，块内视为常量
        self.moon_pos, self.moon_vel, self.moon_ang = moon_state(0.0)
        self.moon_dist = MOON_DIST
        self.moon_min_dist = MOON_DIST
        self.focus_alt = 0.0         # 相机/高度表的参考高度（近月时是距月面高度）
        self.primary = "earth"       # 当前主天体："earth" 或 "moon"
        # 地球与月球各自的轨道根数（半径，不是海拔）
        self.earth_eps = self.earth_rp = self.earth_ra = 0.0
        self.earth_e = 0.0
        self.moon_eps = self.moon_rp = self.moon_ra = 0.0
        self.moon_e = 0.0
        self.ever_earth_orbit = False
        self.ever_moon_orbit = False
        self.missed_moon = False
        self._in_moon_soi = False
        self._tli_err = None
        self._pred_cache_t = None    # 近月点预测的节流缓存
        self._pred_cache_v = None

        # 制导目标（实例属性，便于按任务/载荷调整或做参数扫描）
        self.apogee_target_alt = APOGEE_TARGET_ALT
        self.circ_target_perigee_alt = CIRC_TARGET_PERIGEE_ALT
        self.circ_relight_vradial = CIRC_RELIGHT_VRADIAL
        self.tli_apogee_dist = MOON_DIST
        self.tli_aim_offset = 0.0     # 到达角度的沿轨偏移 (m)
        self.loi_trigger_dist = LOI_TRIGGER_DIST
        self.loi_stop_ratio = LOI_STOP_RATIO

        if learner is None:
            learner = FlightLearner(preset=self.preset,
                                    v_ref=self.pitch_v_ref)
        self.learner = learner

        self._log(f"参数来源：{CONFIG_SOURCE}")
        self._log(f"{self.vehicle_title} · 任务"
                  f"{'探月' if self.mission == 'lunar' else '入轨'}")
        self._log("进入发射倒计时")

    # ------------------------------------------------------------
    def _log(self, msg):
        self.events.append((self.t, msg))
        if len(self.events) > 8:
            self.events.pop(0)

    def total_mass(self):
        m = self.payload
        for st in self.stages[self.active_idx:]:
            m += st.mass
        return m

    def current_stage(self):
        return (self.stages[self.active_idx]
                if self.active_idx < len(self.stages) else None)

    @property
    def stack_len(self):
        """当前存活箭体总长（含载荷舱），即本地坐标里喷管到整流罩顶端的距离。
        渲染、气动力臂、RCS 力臂都从这里取，不再各自硬编码 "+3.0"。"""
        return (sum(st.length for st in self.stages[self.active_idx:])
                + self.payload_len)

    @property
    def mission_done(self):
        """本次任务的最终目标是否已经达成。

        mission = "leo"   ：在地球轨道上，并且制导已经收工（圆化 burn 结束）
        mission = "lunar" ：在绕月轨道上，并且近月制动已经结束

        "制导收工"这一条很重要：只看"近心点过了线"就宣布完成，会在圆化 burn 才
        刚开始时就结束任务，把一条很扁的转移轨道当成入轨结果记下来。
        另外末级确实已经燃尽时也算完成——否则一条已经入轨但没烧完的轨迹会永远
        不结束，学习器也就永远拿不到样本。
        """
        if not self.in_orbit:
            return False
        st = self.current_stage()
        burned_out = st is None or st.fuel <= 1e-9
        if self.mission == "lunar":
            if self.primary != "moon":
                return False
            return self.upper_phase == "done" or burned_out
        if self.primary != "earth":
            return False
        return self.upper_phase == "done" or burned_out

    def moon_altitude(self):
        """距月面高度 (m)。"""
        return self.moon_dist - MOON_RADIUS

    def moon_relative_velocity(self):
        """相对月球的速度矢量（月球本身在运动，必须减掉）。"""
        mx, my = self.moon_vel
        return (self.body.velocity.x - mx, self.body.velocity.y - my)

    def wind_speed(self, alt):
        """高度风廓线：低空切变 + 11 km 附近急流 + 缓慢时变。

        时变项的时间尺度必须是小时级。旧实现是 sin(t·0.7)，周期 9 秒——
        那等于给箭体持续加一个物理上不存在的高频激励，会直接污染攻角、
        动压和气动载荷。
        """
        a = 8.0 * (1 - math.exp(-alt / 800.0))
        b = 45.0 * math.exp(-((alt - 11000.0) / 7000.0) ** 2)
        near = min(1.0, alt / 150.0)
        slow = 1.0 + 0.25 * math.sin(self.t / 900.0 + 1.1)   # 周期约 94 分钟
        return (a + b) * near * slow

    def _mass_properties(self):
        """质心与转动惯量。

        本地坐标原点在**当前最下面一级的底部（喷管出口）**，+y 指向箭体头部，
        载荷舱/整流罩在整叠结构的最顶端——与 rendering.rocket 的绘制顺序一致。
        旧实现先把载荷放在原点附近再往上叠分级，等价于"整流罩长在屁股上"。
        """
        segs = []
        y = 0.0
        for i in range(self.active_idx, len(self.stages)):
            st = self.stages[i]
            segs.append((st.mass, st.length, st.radius, y + st.length * 0.5))
            y += st.length
        segs.append((self.payload, self.payload_len, self.payload_radius,
                     y + self.payload_len * 0.5))

        M = sum(s[0] for s in segs)
        if M <= 1e-9:
            return 0.0, 0.0, 1.0
        y_cm = sum(s[0] * s[3] for s in segs) / M
        I = 0.0
        for m, L, r, yc in segs:
            I += (m * L * L / 12.0 + m * r * r / 4.0) + m * (yc - y_cm) ** 2
        return M, y_cm, max(I, 1.0)

    def payload_stage(self):
        """把载荷舱包成一个 Stage，供解体时生成碎片用。"""
        return Stage(dict(name="载荷舱", dry=self.payload, fuel=0.0,
                          thrust=1.0, isp=1.0, radius=self.payload_radius,
                          length=self.payload_len, cd=PAYLOAD_CD))

    def _v_radial(self):
        """当前（相对地面）速度的径向分量，>0 表示在上升。"""
        vx = self.body.velocity.x
        vy = self.body.velocity.y
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        r = math.hypot(rx, ry)
        if r < 1e-6:
            return 0.0
        return (vx * rx + vy * ry) / r

    def inertial_velocity(self):
        """轨道计算用的速度。

        本仿真的世界**本身就是惯性系**：地面静止、没有科氏力也没有离心力项，
        所以"相对地面的速度"就是这里该用的速度。

        早期版本在这里额外叠加了一个"发射场自转线速度"（沿当地东向切向），想把
        地球自转借力算进去。那是错的：把一个速度偏置**逐帧**贴到一条已经在另一个
        参考系里积出来的轨迹上，得到的不是任何一条真实轨道——偏置会贡献 c·v 这一
        项，而 v 的方向沿轨迹一直在转，于是"轨道"在发动机完全关机的情况下自己漂移
        （实测纯滑行 220 s，近地点自己漂了 200 km，然后被判成"已入轨"）。
        自转借力要真做，只能把动力学整体搬到转动系并补上科氏力/离心力，
        或者干脆从点火起就在惯性系里积分——两者都超出当前二维模型的范围。
        所以这里选择诚实：不模拟它，并在点火日志里注明这个差额。
        """
        return (self.body.velocity.x, self.body.velocity.y)

    def _max_q_throttle(self):
        q = self.last_q
        if q > 30_000.0:
            return max(0.7, 1.0 - (q - 30_000.0) / 150_000.0)
        return 1.0

    def _ignite(self):
        self.state = self.STATE_FLIGHT
        self.countdown_t = 0.0
        self.t = 0.0
        self.throttle = 1.0
        self.engine_cut = False
        self.launch_boost = (EARTH_OMEGA * R_EARTH
                             * math.cos(math.radians(LAUNCH_LAT_DEG)))
        # 只作为信息显示：本仿真不模拟地球自转借力（原因见 inertial_velocity）。
        # 也就是说这里的入轨速度要求比真实发射场"贵" 约 438 m/s。
        self._log(f"点火升空（真实发射场另有 {self.launch_boost:.0f} m/s "
                  f"自转借力，本仿真未计入）")

    def toggle_engine(self):
        """W 键：切换发动机主阀。

        真实火箭的点火/关机是一条指令（锁存），不是"必须一直按住某个键"。
        旧实现里手动模式每帧检查是否按着 W，于是 T-0 自动点火后立刻就被
        自己的检查关掉了——火箭根本不会起飞。
        """
        if self.state == self.STATE_PRELAUNCH:
            return
        st = self.current_stage()
        if self.engine_cut:
            if st is None or st.fuel <= 0.0:
                self._log("无可用燃料，无法点火")
                return
            self.engine_cut = False
            self.landed_soft = False
            self._log(f"{st.name}发动机点火")
        else:
            self.engine_cut = True
            self._log("发动机关机")

    # ============================================================
    # 尾气粒子：本地坐标生成
    # ============================================================
    def _spawn_exhaust(self):
        st = self.current_stage()
        if st is None or st.fuel <= 0:
            return

        # 排气方向在本地坐标系里：theta=0 时指向 -y
        eff = self.delta + self.tvc_bias   # 相对于火箭本地坐标
        ex_dir = -math.sin(eff)
        ey_dir = -math.cos(eff)

        rho = self.rho_local
        if rho > 0.3:
            base_col = (255, 210, 130)
        elif rho > 0.05:
            base_col = (250, 220, 180)
        else:
            base_col = (200, 215, 255)

        n = random.randint(3, 6)
        for _ in range(n):
            # 本地坐标，喷管出口略低于参考点
            lx = random.uniform(-0.55, 0.55) * st.radius
            ly = -0.5

            # 散开角 ±14°
            spread = random.uniform(-0.25, 0.25)
            dx = ex_dir * math.cos(spread) - ey_dir * math.sin(spread)
            dy = ex_dir * math.sin(spread) + ey_dir * math.cos(spread)

            speed = random.uniform(20.0, 50.0)
            vx = dx * speed
            vy = dy * speed

            life = random.uniform(0.3, 0.6)
            rad = random.uniform(1.2, 2.4)

            jitter = random.randint(-20, 20)
            col = (max(0, min(255, base_col[0] + jitter)),
                   max(0, min(255, base_col[1] + jitter)),
                   max(0, min(255, base_col[2] + jitter)))
            self.exhaust.append(
                LocalParticle(lx, ly, vx, vy, life, col, rad))

    def _update_exhaust(self, dt):
        """本地坐标更新 + 世界坐标转换。
        无论火箭飞多快、转多少度，粒子始终紧贴火箭尾部。"""
        decay = max(0.0, 1.0 - dt * 3.0)

        pos = self.body.position
        theta = self.theta
        c = math.cos(theta)
        s = math.sin(theta)

        i = 0
        while i < len(self.exhaust):
            p = self.exhaust[i]
            # 本地坐标漂移
            p.lx += p.vx * dt
            p.ly += p.vy * dt
            # 相对速度衰减（粒子很快"锁定"在尾部）
            p.vx *= decay
            p.vy *= decay
            p.life -= dt
            if p.life > 0:
                # 本地 -> 世界，供渲染使用
                p.x = pos.x + p.lx * c + p.ly * s
                p.y = pos.y - p.lx * s + p.ly * c
                i += 1
            else:
                self.exhaust[i] = self.exhaust[-1]
                self.exhaust.pop()

    # ============================================================
    # RK4 大步长积分
    # ============================================================
    def _accel_at(self, x, y, vx, vy, mass, wind_x):
        rx = x
        ry = y + R_EARTH
        r = math.hypot(rx, ry)
        if r < 1.0:
            r = 1.0

        g = G0 * (R_EARTH / r) ** 2
        ax = -g * rx / r
        ay = -g * ry / r

        # 月球引力（世界坐标，月球位置在块内视为常量）
        mx, my = self.moon_pos
        dx = mx - x
        dy = my - y
        d2 = dx * dx + dy * dy
        if d2 > 1.0:
            f = MOON_MU / (d2 * math.sqrt(d2))
            ax += f * dx
            ay += f * dy

        st = self.current_stage()
        if st is not None and st.fuel > 0.0 and mass > 1e-9:
            eff = self.theta + self.delta + self.tvc_bias
            T = st.thrust * self.throttle
            ax += math.sin(eff) * T / mass
            ay += math.cos(eff) * T / mass

        alt = r - R_EARTH
        if 0.0 < alt < 200_000.0 and mass > 1e-9:
            rho = RHO0 * math.exp(-alt / H_SCALE)
            vxa = vx - wind_x
            vya = vy
            spd = math.hypot(vxa, vya)
            if spd > 1e-9 and rho > 1e-12:
                cd = st.cd if st else 0.30
                rad = st.radius if st else 0.80
                area = math.pi * rad * rad
                drag = 0.5 * rho * cd * area * spd * spd
                ax -= (vxa / spd) * drag / mass
                ay -= (vya / spd) * drag / mass

        return ax, ay

    def _rk4_translate(self, dt, mass, wind_x):
        x0, y0 = self.body.position.x, self.body.position.y
        vx0, vy0 = self.body.velocity.x, self.body.velocity.y

        ax1, ay1 = self._accel_at(x0, y0, vx0, vy0, mass, wind_x)

        x2 = x0 + vx0 * dt * 0.5
        y2 = y0 + vy0 * dt * 0.5
        vx2 = vx0 + ax1 * dt * 0.5
        vy2 = vy0 + ay1 * dt * 0.5
        ax2, ay2 = self._accel_at(x2, y2, vx2, vy2, mass, wind_x)

        x3 = x0 + vx2 * dt * 0.5
        y3 = y0 + vy2 * dt * 0.5
        vx3 = vx0 + ax2 * dt * 0.5
        vy3 = vy0 + ay2 * dt * 0.5
        ax3, ay3 = self._accel_at(x3, y3, vx3, vy3, mass, wind_x)

        x4 = x0 + vx3 * dt
        y4 = y0 + vy3 * dt
        vx4 = vx0 + ax3 * dt
        vy4 = vy0 + ay3 * dt
        ax4, ay4 = self._accel_at(x4, y4, vx4, vy4, mass, wind_x)

        x_new = x0 + dt / 6.0 * (vx0 + 2 * vx2 + 2 * vx3 + vx4)
        y_new = y0 + dt / 6.0 * (vy0 + 2 * vy2 + 2 * vy3 + vy4)
        vx_new = vx0 + dt / 6.0 * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
        vy_new = vy0 + dt / 6.0 * (ay1 + 2 * ay2 + 2 * ay3 + ay4)

        self.body.position = (x_new, y_new)
        self.body.velocity = (vx_new, vy_new)

    def substep(self, h, mass, wind_x):
        b = self.body
        vx = b.velocity.x
        vy = b.velocity.y
        rx = b.position.x
        ry = b.position.y + R_EARTH
        r_center = math.hypot(rx, ry)
        if r_center < 1.0:
            r_center = 1.0
        alt = r_center - R_EARTH

        g = G0 * (R_EARTH / r_center) ** 2
        fx = -mass * g * (rx / r_center)
        fy = -mass * g * (ry / r_center)

        # 月球引力（substep 里的 rx/ry 是地心坐标，所以月球也要换成地心坐标）
        mx, my = self.moon_pos
        dx = mx - rx
        dy = (my + R_EARTH) - ry
        d2 = dx * dx + dy * dy
        if d2 > 1.0:
            fm = mass * MOON_MU / (d2 * math.sqrt(d2))
            fx += fm * dx
            fy += fm * dy

        st = self.current_stage()
        if st is not None and st.fuel > 0.0 and self.throttle > 0.01:
            ve = st.isp * G0
            mdot = st.thrust / ve * self.throttle
            burn = min(mdot * h, st.fuel)
            if burn > 0.0:
                st.fuel -= burn
                mass -= burn
                T = (burn / h) * ve
                eff = self.theta + self.delta + self.tvc_bias
                fx += math.sin(eff) * T
                fy += math.cos(eff) * T

        if 0.0 < alt < 200_000.0:
            rho = RHO0 * math.exp(-alt / H_SCALE)
        else:
            rho = 0.0

        vxa = vx - wind_x
        vya = vy
        spd = math.hypot(vxa, vya)
        if spd > 1e-9 and rho > 1e-12:
            cd = st.cd if st else 0.30
            rad = st.radius if st else 0.80
            area = math.pi * rad * rad
            drag = 0.5 * rho * cd * area * spd * spd
            fx -= (vxa / spd) * drag
            fy -= (vya / spd) * drag

        inv_m = 1.0 / mass
        vx_new = vx + fx * inv_m * h
        vy_new = vy + fy * inv_m * h
        b.velocity = (vx_new, vy_new)
        b.position = (rx + vx_new * h, ry - R_EARTH + vy_new * h)

        self.last_accel = (fx * rx + fy * ry) / r_center * inv_m
        return mass

    # ============================================================
    # 主更新
    # ============================================================
    def update(self, dt, key_a, key_d, real_dt=None):
        if real_dt is None:
            real_dt = dt

        # 月球位置：每块算一次，块内视为常量（一块最长 0.5 s，月球只走 0.5 m）
        self.moon_pos, self.moon_vel, self.moon_ang = moon_state(self.t)

        if self.state == self.STATE_PRELAUNCH:
            self.countdown_t -= dt
            self.t += dt
            self.throttle = 0.0
            self._update_exhaust(dt)
            _cull_dead_abs(self.particles, dt)
            if self.countdown_t <= 0:
                self._ignite()
            return

        if self.crashed or self.broken or self.burned_up:
            for d in self.debris:
                d.update(dt, self.wind_speed(max(d.altitude, 0.0)))
            _cull_dead_abs(self.particles, dt)
            self._update_exhaust(dt)
            self.t += dt
            return

        M, y_cm, I = self._mass_properties()
        st = self.current_stage()

        # 高度用**当前位置**实时算，不要依赖 self.altitude 这个上一块的缓存值：
        # 正常飞行只差一块（0.05~0.5 s），无所谓；但程序化构造初值（测试脚本、
        # 截图工具、将来的存档回放）时缓存值可能是 0，于是大气密度按海平面算——
        # 一条 1.4 km/s 的月球交会弹道会被算出 1.25 MPa 动压，瞬间判定"结构解体"。
        _rx = self.body.position.x
        _ry = self.body.position.y + R_EARTH
        self.altitude = math.hypot(_rx, _ry) - R_EARTH

        # 主阀（W 键锁存）优先于一切节流指令：点火之后不再需要按住任何键
        self.throttle = 0.0 if self.engine_cut else self._max_q_throttle()

        if self.auto_mode:
            self.used_auto = True
            if (self.active_idx == len(self.stages) - 1
                    and st is not None and st.fuel > 0.0):
                if self._upper_stage_guidance():
                    self.throttle = 0.0
        else:
            self._set_phase("boost")

        engine_on = (st is not None and st.fuel > 0.0
                     and self.throttle > 0.01)
        self.engine_on = engine_on

        tau = 0.0
        self.control_mode = "TVC" if engine_on else "RCS"

        if engine_on:
            max_delta = math.radians(TVC_MAX_DEG_LOW if self.active_idx == 0
                                     else TVC_MAX_DEG_UP)
            if self.auto_mode:
                target = self._auto_target_pitch()
                err = target - self.theta
                while err > math.pi:
                    err -= math.tau
                while err < -math.pi:
                    err += math.tau
                Kp, Kd = 0.8, 0.6
                delta_cmd = Kp * err - Kd * self.omega
                delta_cmd = max(-max_delta, min(max_delta, delta_cmd))
            else:
                cmd = 0.0
                if key_a:
                    cmd -= 1.0
                if key_d:
                    cmd += 1.0
                delta_cmd = cmd * max_delta

            rate_max = math.radians(TVC_RATE_MAX_DEG)
            delta_rate = (delta_cmd - self.delta) / TVC_ACT_TAU
            delta_rate = max(-rate_max, min(rate_max, delta_rate))
            self.delta += delta_rate * dt
            self.delta = max(-max_delta, min(max_delta, self.delta))

            self.rcs_cmd = 0.0
            eff_delta = self.delta + self.tvc_bias
            L_engine = max(0.5, y_cm)
            tau += st.thrust * self.throttle * L_engine * math.sin(eff_delta)
        else:
            self.delta *= (1 - min(1.0, dt * 4.0))
            stack_len = self.stack_len
            y_rcs = max(stack_len - 1.5, y_cm + 0.5)
            r_arm = y_rcs - y_cm
            tau_max = RCS_FORCE * r_arm
            if self.auto_mode:
                target = self._auto_target_pitch()
                err = target - self.theta
                while err > math.pi:
                    err -= math.tau
                while err < -math.pi:
                    err += math.tau
                Kp, Kd = I * 2.0, I * 2.5
                tau_cmd = Kp * err - Kd * self.omega
                tau += max(-tau_max, min(tau_max, tau_cmd))
                self.rcs_cmd = max(-1.0, min(1.0, tau_cmd / max(tau_max, 1e-6)))
            else:
                cmd = 0.0
                if key_a:
                    cmd -= RCS_LIMIT
                if key_d:
                    cmd += RCS_LIMIT
                tau += cmd * tau_max
                self.rcs_cmd = cmd

        if self.altitude < 200_000.0:
            rho = RHO0 * math.exp(-max(self.altitude, 0.0) / H_SCALE)
            vx = self.body.velocity.x
            vy = self.body.velocity.y
            spd = math.hypot(vx, vy)
            if spd > 20.0 and rho > 1e-9:
                q = 0.5 * rho * spd * spd
                self.last_q = q
                vel_angle = math.atan2(vx, vy)
                aoa = vel_angle - self.theta
                while aoa > math.pi:
                    aoa -= math.tau
                while aoa < -math.pi:
                    aoa += math.tau
                self.last_aoa = aoa

                st_c = self.current_stage()
                r_ref = st_c.radius if st_c else 0.8
                A_ref = math.pi * r_ref * r_ref
                stack_len = self.stack_len
                # 注意：0.4·stack_len−1 对任何完整的箭体都 >3，这个 clamp 实际上
                # 让 arm 恒等于 3.0 m，等于把"力臂随构型变化"写成了常数。
                arm = max(0.8, min(3.0, 0.4 * stack_len - 1.0))
                CN = 2.0 * math.sin(aoa)
                tau += q * A_ref * CN * arm
                C_damp = 0.8
                L_ref = max(stack_len, 1.0)
                tau -= C_damp * q * A_ref * L_ref * L_ref * self.omega / max(spd, 1.0)
        else:
            self.last_q = 0.0
            self.last_aoa = 0.0

        if self.last_q > BREAK_Q and abs(self.last_aoa) > BREAK_AOA:
            self._break_apart()
            return

        alpha_ang = tau / I
        omega_half = self.omega + alpha_ang * dt * 0.5
        self.theta += omega_half * dt
        self.omega = omega_half + alpha_ang * dt * 0.5

        while self.theta > math.pi:
            self.theta -= math.tau
        while self.theta < -math.pi:
            self.theta += math.tau
        self.omega = max(-6.0, min(6.0, self.omega))

        mass = self.total_mass()
        alt0 = (math.hypot(self.body.position.x,
                           self.body.position.y + R_EARTH) - R_EARTH)
        wind_x_cached = self.wind_speed(max(alt0, 0.0))

        if dt <= RK4_DT_THRESHOLD:
            spd = math.hypot(self.body.velocity.x, self.body.velocity.y)
            sps = self.SUBSTEPS_PER_SEC
            if spd > 2000:
                sps *= 2
            substeps = max(self.SUBSTEPS_MIN,
                           min(self.SUBSTEPS_MAX, int(dt * sps)))
            h = dt / substeps
            for _ in range(substeps):
                mass = self.substep(h, mass, wind_x_cached)
        else:
            st_f = self.current_stage()
            if (st_f is not None and st_f.fuel > 0.0
                    and self.throttle > 0.01):
                ve = st_f.isp * G0
                mdot = st_f.thrust / ve * self.throttle
                n_fuel_sub = 4
                sub_dt = dt / n_fuel_sub
                for _ in range(n_fuel_sub):
                    burn = min(mdot * sub_dt, st_f.fuel)
                    if burn <= 0.0:
                        break
                    st_f.fuel -= burn
                    mass -= burn

            self._rk4_translate(dt, mass, wind_x_cached)
            ax_now, ay_now = self._accel_at(
                self.body.position.x, self.body.position.y,
                self.body.velocity.x, self.body.velocity.y,
                mass, wind_x_cached)
            rx = self.body.position.x
            ry = self.body.position.y + R_EARTH
            r_center = max(1.0, math.hypot(rx, ry))
            self.last_accel = (ax_now * rx + ay_now * ry) / r_center

        px, py = self.body.position.x, self.body.position.y
        rx = px
        ry = py + R_EARTH
        r_center = math.hypot(rx, ry)

        if r_center < R_EARTH:
            vx = self.body.velocity.x
            vy = self.body.velocity.y
            ux = rx / r_center
            uy = ry / r_center
            v_radial = vx * ux + vy * uy

            scale = R_EARTH / r_center
            new_rx = rx * scale
            new_ry = ry * scale
            self.body.position = (new_rx, new_ry - R_EARTH)

            if v_radial < -CRASH_SPEED:
                self.body.velocity = (0.0, 0.0)
                self._crash()
            else:
                v_tx = vx - v_radial * ux
                v_ty = vy - v_radial * uy
                self.body.velocity = (v_tx * 0.3, v_ty * 0.3)
                if not self.landed_soft:
                    self._land_soft()

        # ---- 月面接触 ----
        mx, my = self.moon_pos
        dxm = self.body.position.x - mx
        dym = self.body.position.y - my
        dm = math.hypot(dxm, dym)
        self.moon_dist = dm
        if dm < MOON_RADIUS:
            ux, uy = dxm / dm, dym / dm
            scale = MOON_RADIUS / dm
            self.body.position = (mx + dxm * scale, my + dym * scale)
            vx, vy = self.body.velocity
            mvx, mvy = self.moon_vel
            # 相对月球的径向速度（月球本身在动）
            v_radial = (vx - mvx) * ux + (vy - mvy) * uy
            if v_radial < -CRASH_SPEED:
                self.body.velocity = (0.0, 0.0)
                self._crash("撞月爆炸")
            else:
                v_tx = (vx - mvx) - v_radial * ux
                v_ty = (vy - mvy) - v_radial * uy
                self.body.velocity = (mvx + v_tx * 0.3, mvy + v_ty * 0.3)
                if not self.landed_soft:
                    self._land_soft("月面软着陆")

        self.t += dt

        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        self.altitude = math.hypot(rx, ry) - R_EARTH
        self.max_alt = max(self.max_alt, self.altitude)
        self.max_speed = max(self.max_speed,
                             math.hypot(self.body.velocity.x,
                                        self.body.velocity.y))

        if engine_on and dt < 0.3:
            self._exhaust_accum += real_dt
            if self._exhaust_accum >= 0.015:
                self._spawn_exhaust()
                self._exhaust_accum = 0.0
        else:
            self._exhaust_accum = 0.0

        px, py = self.body.position.x, self.body.position.y
        if not self.trail or math.hypot(px - self.trail[-1][0],
                                        py - self.trail[-1][1]) > 800:
            self.trail.append((px, py))
            if len(self.trail) > 400:
                self.trail.pop(0)

        self._update_orbit()
        self._update_reentry_heat(dt)
        if self.reentry_heat > BURNUP_HEAT:
            self._burn_up()
            return

        st_c = self.current_stage()
        if st_c is not None and st_c.fuel <= 1e-9:
            self._separate()

        # 每块碎片按自己的高度取风：抛级后各碎片高度差别很大
        for d in self.debris:
            d.update(dt, self.wind_speed(max(d.altitude, 0.0)))

        _cull_dead_abs(self.particles, dt)
        self._update_exhaust(dt)

    # ============================================================
    # 末级制导：抬远地点 → 滑行 → 远地点圆化
    # ============================================================
    def _upper_stage_guidance(self):
        """判据驱动，返回 True 表示本帧应当熄火滑行。

        旧实现是"燃料剩 40% 且 vy>400 就关机 / vy<80 就再点火"——两个硬编码的
        垂直速度阈值，用户一改推力/比冲/燃料量整段制导就失效，正好把"参数外置
        可调"这个设计意图废掉。这里改成看轨道根数：
          1) boost：一直烧到"远地点 ≥ apogee_target_alt"（而不是烧到某个 vy）
          2) coast：滑行到远地点（径向速度转负）；超过 COAST_MAX_TIME 也强制点火
          3) circ ：水平加速抬高近地点，达到目标后永久关机

        目标高度是实例属性（apogee_target_alt / circ_target_perigee_alt），
        可以按任务或载荷调整，也方便做参数扫描。
        """
        rp_alt = (self.orbit_rp - R_EARTH) if self.orbit_rp > 0 else -R_EARTH
        ra_alt = (self.orbit_ra - R_EARTH) if self.orbit_ra > 0 else -R_EARTH

        if self.upper_phase == "boost":
            if ra_alt >= self.apogee_target_alt:
                self._set_phase("coast")
                self._log(f"末级关机 · 滑行至远地点 {ra_alt / 1000:.0f} km")
        elif self.upper_phase == "coast":
            # 圆化 burn 必须大致跨在远地点两侧。旧实现是"径向速度转负就点火"，
            # 也就是**过了远地点才点火**：实测 400 秒的 burn 全落在下降侧，
            # 能量灌进远地点（近地点只抬到 152 km 就回落），最后冲到逃逸轨道。
            # 现在用开普勒方程估"还有多久到远地点"，提前半个燃烧时长点火；
            # 如果刚过远地点，它会自然等到下一个远地点（最多等一圈）。
            if (self._circ_relight_ready()
                    or (self.t - self.phase_t0) > COAST_MAX_TIME):
                self._set_phase("circ")
                self._log("远地点再点火 · 抬升近地点")
        elif self.upper_phase == "circ":
            # 近地点达标就收工；入轨任务再多烧一会儿把轨道修圆（燃料够的话）
            spread_ok = (self.mission != "leo"
                         or (ra_alt - rp_alt) <= CIRC_MAX_SPREAD)
            if rp_alt >= self.circ_target_perigee_alt and spread_ok:
                if self.mission == "lunar":
                    self._set_phase("park")
                    v_inf, b = self._update_tli_target()
                    self._log(f"进入地球停泊轨道 · 等待地月转移窗口"
                              f"（瞄准偏移 {b / 1000:,.0f} km，"
                              f"到达相对速度 {v_inf:.0f} m/s）")
                else:
                    self._set_phase("done")
                    self._log(f"圆化完成 · 近地点 {rp_alt / 1000:.0f} km / "
                              f"远地点 {ra_alt / 1000:.0f} km")
                return True
            # 只在远地点附近点火：一旦烧过了远地点窗口就熄火，等下一圈。
            # 否则半个 burn 会落在下降侧，能量全灌进远地点
            # （实测：一台上 4.5 km/s 的转移级因为连烧 400 秒直接烧到逃逸轨道）。
            return not self._circ_burn_now()
        elif self.upper_phase == "park":
            if self._tli_window_ready():
                self._set_phase("tli")
                self._log(f"TLI 点火 · 目标远地点 "
                          f"{self.tli_apogee_dist / 1000:,.0f} km")
            elif (self.t - self.phase_t0) > PARK_MAX_TIME:
                self._set_phase("done")
                self._log("!! 等待转移窗口超时，任务失败")
        elif self.upper_phase == "tli":
            if self._tli_done():
                self._set_phase("transit")
                rp = self._predicted_perilune_alt()
                extra = (f" · 预测近月点 {rp / 1000:,.0f} km"
                         if rp is not None else "")
                self._log(f"TLI 关机 · 地月转移滑行（约 "
                          f"{self._transfer_days():.1f} 天）{extra}")
        elif self.upper_phase == "transit":
            if self._loi_ready():
                self._set_phase("loi")
                self._log("近月制动点火")
        elif self.upper_phase == "loi":
            if self._loi_done():
                self._set_phase("done")
                self._log(f"近月制动完成 · 近月点 "
                          f"{self.moon_altitude() / 1000:.0f} km")

        # 需要动力的阶段返回 False（不熄火）
        return self.upper_phase in ("coast", "park", "transit", "done")

    def _set_phase(self, phase):
        self.upper_phase = phase
        self.phase_t0 = self.t
        self._tli_err = None          # 相位误差只在停泊段累计

    def _transfer_days(self):
        """当前转移轨道的半圈时间（天），只用于显示。"""
        mu = G0 * R_EARTH * R_EARTH
        r = math.hypot(self.body.position.x, self.body.position.y + R_EARTH)
        a = 0.5 * (r + self.tli_apogee_dist)
        return math.pi * math.sqrt(a ** 3 / mu) / 86400.0

    # ------------------------------------------------------------
    # 开普勒小工具（探月制导用）
    # ------------------------------------------------------------
    @staticmethod
    def _wrap_pi(x):
        while x > math.pi:
            x -= math.tau
        while x < -math.pi:
            x += math.tau
        return x

    @staticmethod
    def _rv_to_elements(rx, ry, vx, vy, mu):
        """由相对位置速度求 (r, v_r, v_t, nu, e, a)。
        nu 归到 (-π, π]：负值表示还没到近心点。"""
        r = math.hypot(rx, ry)
        if r < 1e-9:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        v2 = vx * vx + vy * vy
        v_r = (rx * vx + ry * vy) / r
        h = rx * vy - ry * vx
        v_t = h / r
        eps = v2 / 2.0 - mu / r
        e = math.sqrt(max(0.0, 1.0 + 2.0 * eps * h * h / (mu * mu)))
        # 注意：双曲线（eps>0）的 a 是**负数**，同一个公式就够。
        # 早期版本在这里写成"eps<0 才算 a"，于是交会段的 a=inf，
        # 双曲线开普勒方程算出无穷大的"到近月点时间"，近月制动永远不点火。
        a = -mu / (2.0 * eps) if abs(eps) > 1e-12 else float("inf")
        if e > 1e-9:
            p = h * h / mu
            cosnu = max(-1.0, min(1.0, (p / r - 1.0) / e))
            nu = math.acos(cosnu)
            if v_r < 0.0:
                nu = -nu
        else:
            nu = 0.0
        return r, v_r, v_t, nu, e, a

    @staticmethod
    def _time_to_periapsis(nu, e, a, mu):
        """从真近点角 nu 到**下一次**经过近心点的时间 (s)。

        椭圆：按周期取模，所以"刚过近心点"返回的是接近一整圈的时间；
        双曲线：只经过一次，过了就返回负值（调用方据此判断"已经过了"）。
        """
        try:
            if e < 1.0:
                if not (a > 0.0) or not math.isfinite(a):
                    return float("inf")
                E = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nu / 2.0),
                                     math.sqrt(1.0 + e) * math.cos(nu / 2.0))
                M = E - e * math.sin(E)
                n = math.sqrt(mu / (a * a * a))
                return ((-M) % math.tau) / n
            arg = math.sqrt((e - 1.0) / (e + 1.0)) * math.tan(nu / 2.0)
            if abs(arg) >= 1.0:
                return float("inf")
            H = 2.0 * math.atanh(arg)
            M = e * math.sinh(H) - H
            n = math.sqrt(mu / ((-a) ** 3))
            return -M / n
        except (ValueError, OverflowError, ZeroDivisionError):
            return float("inf")

    @staticmethod
    def _time_to_apoapsis(nu, e, a, mu):
        """到**下一次**远心点的时间 (s)，只对椭圆有意义。

        注意不能靠"把真近点角旋转 π 再求到近心点的时间"来偷懒：那样得到的区间是
        关于近心点对称的，而我们要的是关于远心点对称的（远心点附近运动慢），
        两者差得很远（实测 e=0.12 时差 500 秒）。直接从远心点的平近点角 M=π 减。
        """
        if e >= 1.0 or not (a > 0.0) or not math.isfinite(a):
            return float("inf")
        try:
            E = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nu / 2.0),
                                 math.sqrt(1.0 + e) * math.cos(nu / 2.0))
            M = E - e * math.sin(E)
            n = math.sqrt(mu / (a * a * a))
            return ((math.pi - M) % math.tau) / n
        except (ValueError, OverflowError, ZeroDivisionError):
            return float("inf")

    def _tli_transfer(self):
        """假设现在做一次顺行脉冲把远地点抬到 tli_apogee_dist，估算转移轨道。

        返回 (飞行时间 s, 到达点的地心角 rad, 需要的 Δv m/s)，条件不成立返回 None。
        这一步只用于"何时点火"的相位判断，真正的关机判据是远地点够不够。
        """
        mu = G0 * R_EARTH * R_EARTH
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        r = math.hypot(rx, ry)
        vx, vy = self.inertial_velocity()
        v_now = math.hypot(vx, vy)
        if r < 1.0 or v_now < 1.0:
            return None

        a_t = 0.5 * (r + self.tli_apogee_dist)
        v_need2 = mu * (2.0 / r - 1.0 / a_t)
        if v_need2 <= 0.0:
            return None
        v_need = math.sqrt(v_need2)
        dv = v_need - v_now
        if dv <= 0.0:
            return None

        ux, uy = vx / v_now, vy / v_now
        _, _, _, nu, e_t, a_new = self._rv_to_elements(
            rx, ry, ux * v_need, uy * v_need, mu)
        if e_t >= 1.0:
            return None
        t_transfer = self._time_to_apoapsis(nu, e_t, a_new, mu)
        if not math.isfinite(t_transfer):
            return None
        psi = math.atan2(rx, ry)
        return t_transfer, psi + (math.pi - nu), dv

    def _update_tli_target(self):
        """算出地月转移的瞄准偏移 (m)。

        关键几何：转移轨道的远地点就设在月球轨道半径上，于是"到远地点"和
        "穿过月球轨道"是同一时刻，火箭与月球的距离完全由**沿轨道方向的相位偏差**
        决定（不是半径差）。所以瞄准偏移要加在到达角度上，而不是加在远地点上。
        早期版本把它加在远地点半径上，结果只是让火箭提前穿过月球轨道，
        撞月点和偏移量没有可控关系——实测直接撞上月球。

        偏移量由双曲线近心点公式反解：b = (mu/v²)·sqrt((1 + r_p·v²/mu)² - 1)。
        """
        mu_e = G0 * R_EARTH * R_EARTH
        r = math.hypot(self.body.position.x, self.body.position.y + R_EARTH)
        a_t = 0.5 * (r + MOON_DIST)
        v_apo2 = mu_e * (2.0 / MOON_DIST - 1.0 / a_t)
        v_apo = math.sqrt(max(0.0, v_apo2))
        v_moon = MOON_OMEGA * MOON_DIST
        v_inf = abs(v_moon - v_apo)
        if v_inf < 1.0:
            self.tli_aim_offset = 0.0
            return v_inf, 0.0
        r_peri = MOON_RADIUS + TLI_PERILUNE_ALT
        x = r_peri * v_inf * v_inf / MOON_MU
        b = (MOON_MU / (v_inf * v_inf)) * math.sqrt(max(0.0, (1.0 + x) ** 2 - 1.0))
        self.tli_aim_offset = b
        return v_inf, b

    def _predict_lunar_miss(self, dvx=0.0, dvy=0.0):
        """用当前状态向前积分，返回 (最近月距, 到达时刻)。

        可选给速度加一个偏置 (dvx, dvy)，用来求"单位 Δv 对最近月距的灵敏度"。
        引力用 config.gravity_accel（地球+月球），月球位置按时间推进，
        步长在远离月球时 600 s、进入月球附近后 60 s。
        """
        x = self.body.position.x
        y = self.body.position.y
        vx = self.body.velocity.x + dvx
        vy = self.body.velocity.y + dvy
        t = self.t

        # 短路：地球轨道的远地点还没够到月球轨道，就一定打不中，
        # 没必要在近地轨道上积分数百圈（那既慢又容易积出假轨道）。
        if self.earth_ra and self.earth_ra < MOON_DIST * 0.8:
            return MOON_DIST, t

        best = float("inf")
        best_t = t
        t_limit = self.t + 12.0 * 86400.0
        for _ in range(40000):
            mp, _, _ = moon_state(t)
            d = math.hypot(x - mp[0], y - mp[1])
            if d < best:
                best = d
                best_t = t
            if d < MOON_RADIUS:
                return d, t
            if t > t_limit:
                break
            r_earth = math.hypot(x, y + R_EARTH)
            if r_earth > 4.0 * MOON_DIST:
                break                      # 已经飞出地月系统
            v_now = math.hypot(vx, vy)
            # 步长按时标 r/v 取：近地点要细，远地点可以很粗。
            # 早期版本只看"离月球多远"，于是在近地轨道上用了 600 s 的步长
            # （接近轨道周期的 1/10），RK4 直接把轨道积飞，预测出假的近月交会。
            if r_earth < 2.0e7:
                h = max(20.0, min(300.0, 0.05 * r_earth / max(v_now, 1.0)))
            else:
                h = 600.0
            ax1, ay1 = gravity_accel(x, y, t)
            ax2, ay2 = gravity_accel(x + vx * h * 0.5, y + vy * h * 0.5,
                                     t + h * 0.5)
            ax3, ay3 = gravity_accel(x + (vx + ax1 * h * 0.5) * h * 0.5,
                                     y + (vy + ay1 * h * 0.5) * h * 0.5,
                                     t + h * 0.5)
            ax4, ay4 = gravity_accel(x + (vx + ax2 * h * 0.5) * h,
                                     y + (vy + ay2 * h * 0.5) * h, t + h)
            vx += h / 6.0 * (ax1 + 2 * ax2 + 2 * ax3 + ax4)
            vy += h / 6.0 * (ay1 + 2 * ay2 + 2 * ay3 + ay4)
            x += vx * h
            y += vy * h
            t += h
        return best, best_t

    def _predicted_perilune_alt(self):
        """预测近月点海拔 (m)；预测失败返回 None。

        预测一次要积分几千步，所以按 10 秒节流（TLI 段每块 0.5 s，等于每 20 块算一次）。
        """
        if (self._pred_cache_t is not None
                and abs(self.t - self._pred_cache_t) < 10.0):
            return self._pred_cache_v
        try:
            miss, _ = self._predict_lunar_miss()
        except (ValueError, OverflowError):
            return None
        val = None if not math.isfinite(miss) else miss - MOON_RADIUS
        self._pred_cache_t = self.t
        self._pred_cache_v = val
        return val

    def _tli_done(self):
        """TLI 关机判据：用**预测**的近月点高度，而不是地球远地点。

        只按"远地点 ≥ 月球轨道半径"关机的话，实际近月点由相位误差决定，
        实测在撞月和 25 万公里擦肩之间随机跳。真正可控的是把预测近月点压到目标以内：
        这就是真实任务里 TLI 的中段瞄准方式。

        注意方向：预测近月点**大**意味着打偏（根本没靠近月球），要继续烧；
        随着 Δv 增加它会一路减小（穿过目标带，直到正中月球 = 0），
        所以"预测近月点 ≤ 目标"就是收工条件。
        """
        rp_alt = self._predicted_perilune_alt()
        if rp_alt is None:
            # 预测不可用就退回几何判据
            return self.earth_ra >= self.tli_apogee_dist
        if rp_alt <= TLI_PERILUNE_ALT:
            return True
        # 保险：远地点已经明显超过月球轨道还没压下来，就别再烧了
        return self.earth_ra >= self.tli_apogee_dist * 1.06

    def _tli_window_ready(self):
        """停泊轨道上的转移窗口：预测到达点与月球位置对齐时点火。

        做法：按当前状态假设一次顺行脉冲，用开普勒方程算"飞到远地点"的时间与
        角度，再看那一刻月球在哪。误差穿越零点就点火——相位精度因此只取决于
        积分步长（0.5 s → 0.03°），而不是靠经验常数。
        """
        info = self._tli_transfer()
        if info is None:
            return False
        t_transfer, arrive_angle, _ = info
        if t_transfer <= 0.0 or t_transfer > 60.0 * 86400.0:
            return False
        # 瞄准偏移加在到达角度上（见 _update_tli_target 的几何说明）
        arrive_angle += self.tli_aim_offset / MOON_DIST
        moon_angle = self.moon_ang + MOON_OMEGA * t_transfer
        err = self._wrap_pi(moon_angle - arrive_angle)
        prev = self._tli_err
        self._tli_err = err
        # 误差随时间**减小**（火箭公转角速度远大于月球），所以要抓的是
        # 正→负的过零点。误差每圈还会扫过 ±π 一次，那里出现的是 -π→+π 的跳变，
        # 必须排除——早期版本正是在回绕点点的火，相位差了 137°，
        # 火箭飞到月球轨道另一侧，最近月距还有 37 万公里。
        # 每块误差变化约 2e-3 rad，用 1.0 rad 判断足以区分过零与回绕。
        return (prev is not None and prev > 0.0 >= err
                and (prev - err) < 1.0)

    def _circ_apsis_times(self):
        """当前地心轨道的 (到下一个远地点的时间, 距上一个远地点的时间)。"""
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        vx, vy = self.inertial_velocity()
        mu = G0 * R_EARTH * R_EARTH
        _, _, _, nu, e, a = self._rv_to_elements(rx, ry, vx, vy, mu)
        if e >= 1.0 or a <= 0.0 or not math.isfinite(a):
            return None
        t_apo = self._time_to_apoapsis(nu, e, a, mu)
        if not math.isfinite(t_apo):
            return None
        period = math.tau * math.sqrt(a ** 3 / mu)
        return t_apo, max(0.0, period - t_apo)

    def _circ_burn_half(self):
        """圆化 burn 的半个窗口时长 (s)：按"还差多少 Δv"估算。

        返回 None 表示估不出来（那就别限制，一路烧）。
        """
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        vx, vy = self.inertial_velocity()
        mu = G0 * R_EARTH * R_EARTH
        _, _, _, _, e, a = self._rv_to_elements(rx, ry, vx, vy, mu)
        if e >= 1.0 or a <= 0.0 or not math.isfinite(a):
            return None
        st = self.current_stage()
        if st is None or st.fuel <= 0.0 or self.throttle <= 0.01:
            return None
        a_thr = st.thrust * self.throttle / max(self.total_mass(), 1e-6)
        if a_thr <= 1e-6:
            return None
        r_apo = a * (1.0 + e)
        r_peri_want = R_EARTH + self.circ_target_perigee_alt
        a_new = 0.5 * (r_peri_want + r_apo)
        v_now = math.sqrt(max(0.0, mu * (2.0 / r_apo - 1.0 / a)))
        v_want = math.sqrt(max(0.0, mu * (2.0 / r_apo - 1.0 / a_new)))
        return max(0.5, max(0.0, v_want - v_now) / a_thr * 0.5)

    def _circ_burn_now(self):
        """本块是否应该点火：在远地点窗口里，或者近地点还低得活不下来。"""
        rp_alt = self.earth_rp - R_EARTH if self.earth_rp > 0 else -R_EARTH
        if rp_alt < CIRC_SAFE_PERIGEE:
            return True
        half = self._circ_burn_half()
        times = self._circ_apsis_times()
        if half is None or times is None:
            return True
        t_apo, t_since = times
        return t_apo <= half or t_since <= half

    def _circ_relight_ready(self):
        """圆化点火判据：快到（或刚过）远地点时点火。

        旧实现是"径向速度转负就点火"，也就是**过了远地点才点火**。对长 burn 来说
        这意味着整段 burn 都落在下降侧：能量灌进远地点，近地点几乎不涨
        （实测 400 秒的 burn 把近地点从 -680 km 只抬到 +152 km，然后冲到逃逸）。
        现在按"还差多少 Δv"估出燃烧时长，只在远地点两侧半个时长的窗口里点火，
        一圈烧不完就滑行到下一圈再烧。
        """
        half = self._circ_burn_half()
        times = self._circ_apsis_times()
        rp_alt = self.earth_rp - R_EARTH if self.earth_rp > 0 else -R_EARTH
        if half is None or times is None or rp_alt < CIRC_SAFE_PERIGEE:
            # 估不出来，或近地点还在保命线以下：过了远地点就点火，先把轨道撑起来
            return self._v_radial() <= self.circ_relight_vradial
        t_apo, _ = times
        return t_apo <= half

    def _moon_peri_time(self):
        """还有多久到近月点 (s)；已经过了返回负值。"""
        mx, my = self.moon_pos
        rx = self.body.position.x - mx
        ry = self.body.position.y - my
        vx, vy = self.moon_relative_velocity()
        _, _, _, nu, e, a = self._rv_to_elements(rx, ry, vx, vy, MOON_MU)
        return self._time_to_periapsis(nu, e, a, MOON_MU)

    def _loi_ready(self):
        """近月制动点火判据：让整段减速 burn 大致跨在近月点两侧。"""
        if self.ever_moon_orbit:
            return False
        t_peri = self._moon_peri_time()
        if not math.isfinite(t_peri) or t_peri < 0.0:
            return False
        st = self.current_stage()
        if st is None or st.fuel <= 0.0 or self.throttle <= 0.01:
            return False
        a_thr = st.thrust * self.throttle / max(self.total_mass(), 1e-6)
        if a_thr <= 1e-6:
            return False
        r_rel = max(self.moon_dist, MOON_RADIUS)
        v_rel = math.hypot(*self.moon_relative_velocity())
        dv = max(0.0, v_rel - math.sqrt(MOON_MU / r_rel) * self.loi_stop_ratio)
        t_burn = dv / a_thr
        if t_peri > max(1.0, t_burn * 0.5):
            return False
        # 距离门限：正常情况等到进入 2500 km 再烧（省 Δv）；
        # 但如果预测近月点已经低到要撞月，就不等门限了，先把速度减下来。
        rp_alt = (self.moon_rp - MOON_RADIUS) if self.moon_rp > 0 else -MOON_RADIUS
        if self.moon_dist <= self.loi_trigger_dist:
            return True
        return rp_alt < LOI_DANGER_PERI

    def _loi_done(self):
        """减速到该高度的环绕速度即停：此时当前点就是近月点，
        轨道近乎圆形，且不会因为多烧一点点就把近月点压到月面以下。"""
        r_rel = max(self.moon_dist, MOON_RADIUS)
        v_rel = math.hypot(*self.moon_relative_velocity())
        return v_rel <= math.sqrt(MOON_MU / r_rel) * self.loi_stop_ratio

    @staticmethod
    def _tangential_pitch(rx, ry, retro=False):
        """相对某天体中心的切向姿态（pitch 约定：theta = atan2(方向x, 方向y)）。

        为什么变轨用切向而不是"顺着速度"：顺着速度烧会把一部分 Δv 变成径向速度，
        径向分量只抬高当前这一侧的拱点、几乎不增加角动量，结果是远地点暴涨而近地点
        不动（实测能把近地点从 130 km 烧到 2 km）。切向烧不引入径向速度，
        对角动量的贡献最大，正是抬升另一侧拱点需要的。
        """
        r = math.hypot(rx, ry)
        if r < 1.0:
            return None
        ux, uy = rx / r, ry / r
        if retro:
            return math.atan2(-uy, ux)
        return math.atan2(uy, -ux)

    def _auto_target_pitch(self):
        # 这些阶段的推力方向由制导直接指定，不能再走上升段俯仰程序。
        # 俯仰程序是"把箭体压到水平"的上升段程序，而圆化时火箭往往在下降或陡升，
        # 照它指方向会白白丢掉大部分 Δv。
        if self.upper_phase in ("circ", "tli"):
            a = self._tangential_pitch(self.body.position.x,
                                       self.body.position.y + R_EARTH)
            if a is not None:
                return a                            # 顺行切向
        elif self.upper_phase == "loi":
            vx, vy = self.moon_relative_velocity()
            if math.hypot(vx, vy) > 1.0:
                return math.atan2(-vx, -vy)         # 逆着相对月球速度减速

        alt = self.altitude
        if alt < 200.0:
            return 0.0

        vx, vy = self.body.velocity
        spd = math.hypot(vx, vy)
        if spd < 30.0:
            return math.radians(2.0)

        base = self.learner.target_pitch(spd)

        vel_angle = math.atan2(vx, vy)
        if self.last_q > 30_000.0:
            max_aoa = math.radians(8.0)
        elif alt < 30_000.0:
            max_aoa = math.radians(20.0)
        else:
            max_aoa = math.radians(40.0)

        lo = vel_angle - max_aoa
        hi = vel_angle + max_aoa
        b = base
        while b - lo > math.pi:
            b -= math.tau
        while b - lo < -math.pi:
            b += math.tau
        if b < lo:
            base = lo
        elif b > hi:
            base = hi
        else:
            base = b

        if base > math.radians(89.5):
            base = math.radians(89.5)
        if base < 0.0:
            base = 0.0
        return base

    def _separate(self):
        st = self.stages[self.active_idx]
        pos = self.body.position
        c, s = math.cos(self.theta), math.sin(self.theta)
        lx, ly = 0.0, st.length * 0.5
        wx = pos.x + lx * c + ly * s
        wy = pos.y - lx * s + ly * c

        v_rel_mag = 2.5
        m_debris = st.mass
        m_rocket_after = self.total_mass() - m_debris
        m_total = m_rocket_after + m_debris
        dv_debris = -v_rel_mag * (m_rocket_after / m_total)
        dv_rocket = +v_rel_mag * (m_debris / m_total)

        debris_vx = self.body.velocity.x
        debris_vy = self.body.velocity.y + dv_debris

        d = Debris(st, pymunk.Vec2d(wx, wy),
                   pymunk.Vec2d(debris_vx, debris_vy),
                   self.theta, self.omega)
        d.omega = self.omega + (random.random() - 0.5) * 0.3
        self.debris.append(d)

        lx2, ly2 = 0.0, st.length
        self.body.position = pymunk.Vec2d(pos.x + lx2 * c + ly2 * s,
                                          pos.y - lx2 * s + ly2 * c)
        self.body.velocity = pymunk.Vec2d(
            self.body.velocity.x,
            self.body.velocity.y + dv_rocket)

        self.active_idx += 1
        self._set_phase("boost")
        self.omega += (random.random() - 0.5) * 0.03

        self.exhaust.clear()
        self._exhaust_accum = 0.0

        if self.active_idx < len(self.stages):
            self._log(f"{st.name}分离 → {self.stages[self.active_idx].name}点火")
        else:
            self._log("末级分离 · 发动机关机")

    def _crash(self, label="撞地爆炸"):
        if self.crashed:
            return
        self.crashed = True
        pos = self.body.position
        spawn_explosion(self.particles, pos.x, pos.y, n=140, scale=1.6)
        spawn_explosion(self.particles, pos.x, pos.y, n=60, scale=2.4,
                        palette=[(255, 240, 200), (255, 180, 90)])
        self.body.velocity = (0.0, 0.0)
        self.omega = 0.0
        self.exhaust.clear()
        self._log(f"!! {label}")

    def _land_soft(self, label="软着陆"):
        """触地（径向速度小于阈值）：关掉发动机并锁死。

        旧实现只置了一个 landed_soft 标志，主更新路径根本不看它，于是自动模式
        下火箭会坐在发射台上继续烧燃料 → 抛级 → 下一级从台上再次"起飞"。
        """
        self.landed_soft = True
        self.engine_cut = True
        self.throttle = 0.0
        self._set_phase("done")
        self._log(f"{label} · 发动机关机")

    def _break_apart(self):
        if self.broken or self.crashed:
            return
        self.broken = True
        pos = self.body.position
        c, s = math.cos(self.theta), math.sin(self.theta)
        y_off = 0.0
        parts = [self.stages[i]
                 for i in range(self.active_idx, len(self.stages))]
        parts.append(self.payload_stage())   # 载荷舱也要跟着碎，否则它会凭空消失
        for st in parts:
            lx, ly = 0.0, y_off + st.length * 0.5
            wx = pos.x + lx * c + ly * s
            wy = pos.y - lx * s + ly * c
            vx = self.body.velocity.x + (random.random() - 0.5) * 40
            vy = self.body.velocity.y + (random.random() - 0.5) * 40
            d = Debris(st, pymunk.Vec2d(wx, wy), pymunk.Vec2d(vx, vy),
                       self.theta, self.omega + (random.random() - 0.5) * 1.2,
                       label="解体")
            self.debris.append(d)
            y_off += st.length
        spawn_explosion(self.particles, pos.x, pos.y, n=100, scale=1.4)
        spawn_explosion(self.particles, pos.x, pos.y, n=40, scale=2.0,
                        palette=[(255, 255, 220), (255, 200, 120)])
        self.body.velocity = (0.0, 0.0)
        self.omega = 0.0
        self.exhaust.clear()
        self._log("!! 结构解体")

    def _burn_up(self):
        if self.burned_up or self.crashed or self.broken:
            return
        self.burned_up = True
        pos = self.body.position
        spawn_explosion(self.particles, pos.x, pos.y, n=120, scale=1.6,
                        palette=[(255, 220, 180), (255, 150, 80), (255, 80, 30)])
        self.body.velocity = (0.0, 0.0)
        self.omega = 0.0
        self.exhaust.clear()
        self._log("!! 再入烧毁")

    @staticmethod
    def _elements(rx, ry, vx, vy, mu):
        """算以某个天体中心为原点的轨道根数。

        返回 (eps, rp, ra, e)；rp / ra 是到该天体中心的距离（不是海拔）。
        eps >= 0（抛物线/双曲线）时 rp / ra 返回 0，换算成"海拔"必然是很负的数，
        入轨判据因此不会误判。
        """
        r = math.hypot(rx, ry)
        if r < 1e-9:
            return (0.0, 0.0, 0.0, 0.0)
        eps = (vx * vx + vy * vy) / 2.0 - mu / r
        h = rx * vy - ry * vx
        e = math.sqrt(max(0.0, 1.0 + 2.0 * eps * h * h / (mu * mu)))
        if eps >= -1e-9:
            return (eps, 0.0, 0.0, e)
        a = -mu / (2.0 * eps)
        return (eps, a * (1.0 - e), a * (1.0 + e), e)

    def _update_orbit(self):
        """同时算地球与月球两套轨道根数，取近的那个当主天体，再维护 in_orbit。

        为什么要两套：地月转移途中地球根数仍然决定"会不会掉回地球"，而进入月球
        影响球之后地球根数就没意义了，得看月球根数。主天体按影响球半径切换。

        判据必须是"近（某）点真正高出表面"：100 km 只是卡门线，近地点贴着 100 km
        的轨迹几分钟内就会被阻力吃掉，而旧判据会把它算成"已入轨"——项目自带的
        flight_learn.json 里就躺着 ra≈8000 km / rp≈100 km 的"成功飞行"。
        """
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH      # 地心矢量
        vx, vy = self.inertial_velocity()
        mu_e = G0 * R_EARTH * R_EARTH

        (self.earth_eps, self.earth_rp,
         self.earth_ra, self.earth_e) = self._elements(rx, ry, vx, vy, mu_e)

        mx, my = self.moon_pos
        mvx, mvy = self.moon_vel
        relx = rx - mx
        rely = ry - (my + R_EARTH)
        self.moon_dist = math.hypot(relx, rely)
        self.moon_min_dist = min(self.moon_min_dist, self.moon_dist)
        (self.moon_eps, self.moon_rp,
         self.moon_ra, self.moon_e) = self._elements(
            relx, rely, vx - mvx, vy - mvy, MOON_MU)

        # 进过月球影响球又飞出去 = 没捕获住
        if self.moon_dist < MOON_SOI:
            self._in_moon_soi = True
        elif self._in_moon_soi:
            self._in_moon_soi = False
            if not self.ever_moon_orbit:
                self.missed_moon = True

        self.primary = "moon" if self.moon_dist < MOON_SOI else "earth"
        if self.primary == "moon":
            self.orbit_rp = self.moon_rp
            self.orbit_ra = self.moon_ra
            self.orbit_e = self.moon_e
            rp_alt = self.moon_rp - MOON_RADIUS
            ra_alt = self.moon_ra - MOON_RADIUS
            enter, exit_alt = MOON_ORBIT_MIN_PERI, MOON_ORBIT_EXIT_PERI
            body_name, rp_word, ra_word = "绕月轨道", "近月点", "远月点"
        else:
            self.orbit_rp = self.earth_rp
            self.orbit_ra = self.earth_ra
            self.orbit_e = self.earth_e
            rp_alt = self.earth_rp - R_EARTH
            ra_alt = self.earth_ra - R_EARTH
            enter, exit_alt = ORBIT_MIN_PERIGEE_ALT, ORBIT_EXIT_PERIGEE_ALT
            body_name, rp_word, ra_word = "地球轨道", "近地点", "远地点"

        if not self.in_orbit:
            if rp_alt >= enter and ra_alt >= enter:
                self.in_orbit = True
                if self.primary == "moon":
                    self.ever_moon_orbit = True
                else:
                    self.ever_earth_orbit = True
                self._log(f"已进入{body_name} · {rp_word} {rp_alt / 1000:.0f} km"
                          f" / {ra_word} {ra_alt / 1000:.0f} km")
        elif rp_alt < exit_alt or ra_alt < exit_alt:
            # 滞回：进入与掉出用不同阈值，避免在边界反复横跳
            self.in_orbit = False
            self._log(f"脱离{body_name}")

        # 相机/高度表的参考高度：在月球影响球内看"距月面高度"
        self.focus_alt = (self.moon_dist - MOON_RADIUS
                          if self.moon_dist < MOON_SOI
                          else math.hypot(rx, ry) - R_EARTH)

    def _update_reentry_heat(self, dt):
        """Sutton-Graves 热流估计。

        离开大气段时按物理时间衰减：旧实现写 `*= 0.9`（每调用一次衰减 10%），
        倍速越高衰减越快，结果与 dt 无关。
        """
        alt = self.altitude
        vx, vy = self.body.velocity
        spd = math.hypot(vx, vy)
        if 0 < alt < 100_000 and spd > 1500:
            rho = RHO0 * math.exp(-alt / H_SCALE)
            k = 1.7415e-4
            Rn = 0.5
            self.reentry_heat = k * math.sqrt(rho / Rn) * spd ** 3
        else:
            self.reentry_heat *= math.exp(-dt / 1.0)

    @property
    def g_local(self):
        rx = self.body.position.x
        ry = self.body.position.y + R_EARTH
        r_center = math.hypot(rx, ry)
        if r_center < 1.0:
            r_center = 1.0
        return G0 * (R_EARTH / r_center) ** 2

    @property
    def local_gravity(self):
        """当前主天体的当地重力加速度 (m/s²)。"""
        if self.primary == "moon":
            r = max(self.moon_dist, MOON_RADIUS)
            return MOON_MU / (r * r)
        return self.g_local

    @property
    def primary_radius(self):
        return MOON_RADIUS if self.primary == "moon" else R_EARTH

    @property
    def primary_mu(self):
        return MOON_MU if self.primary == "moon" else G0 * R_EARTH * R_EARTH

    @property
    def rho_local(self):
        if self.altitude >= 200_000.0:
            return 0.0
        return RHO0 * math.exp(-max(self.altitude, 0.0) / H_SCALE)
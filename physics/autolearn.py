# -*- coding: utf-8 -*-
"""自动学习：记录每次飞行的结果，微调俯仰曲线，形成反馈。

原理：
  · 俯仰曲线是一张"速度 → 目标姿态"的查找表（15 个节点）
  · 每次飞行结束后，根据结果微调整张表
      - 入轨成功          → 所有节点略增（更水平），固定收益
      - 失败且垂直速度过大 → 所有节点增 3%
      - 失败且水平速度不足 → 所有节点减 1%
  · 表保存在 flight_learn.json，**按火箭预设分开存**
  · 删除 flight_learn.json 可重置学习

为什么按预设分开：俯仰程序的合适形状取决于推重比（推重比越低，重力转弯应该
越慢）。旧版只有一张全局表，换一枚火箭接着学，等于把上一枚火箭的经验硬套到
新火箭上。现在每枚火箭有自己的代数和历史，并且记住当初用的俯仰参考速度。
"""
import json
import math
import os

from config import R_EARTH

# 查找表的速度节点 (m/s)
SPEED_BUCKETS = [500, 1000, 1500, 2000, 2500, 3000, 3500,
                 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500]

# 目标姿态上限（弧度）
PITCH_MAX = math.radians(89.5)

# 分析式初值用的默认参考速度
DEFAULT_V_REF = 3500.0

# 默认文件路径（项目根目录）
_DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "flight_learn.json")

_SCHEMA_VERSION = 2


def default_pitch_table(v_ref=DEFAULT_V_REF):
    """分析式初值：target = PITCH_MAX × min(1, speed / v_ref)，截断到上限。

    v_ref 是"俯仰程序压到水平所需的速度"，也就是重力转弯的快慢：
    推重比越大、上升越快，v_ref 应该越小。
    """
    v_ref = max(200.0, float(v_ref))
    return [PITCH_MAX * min(1.0, v / v_ref) for v in SPEED_BUCKETS]


# 兼容旧名字
def _default_pitch_table():
    return default_pitch_table(DEFAULT_V_REF)


class FlightLearner:
    """按火箭预设分别维护俯仰表。

    对外仍然暴露 pitch_table / generation / history（当前预设的那一份），
    所以 Rocket 和 HUD 不用改调用方式。
    """

    def __init__(self, path=None, preset="light", v_ref=DEFAULT_V_REF):
        self.path = path or _DEFAULT_PATH
        self.presets = {}          # preset 名 → {"pitch_table", "generation", "history", "v_ref"}
        self.preset = preset
        self.v_ref = float(v_ref)
        self._load_file()
        self._ensure(self.preset, self.v_ref)

    # ------------------------------------------------------------
    def _ensure(self, preset, v_ref=None):
        """确保某个预设有一张表；没有就用该预设的参考速度生成初值。"""
        ent = self.presets.get(preset)
        if ent is None:
            ent = {
                "v_ref": float(v_ref if v_ref else DEFAULT_V_REF),
                "pitch_table": default_pitch_table(
                    v_ref if v_ref else DEFAULT_V_REF),
                "generation": 0,
                "history": [],
            }
            self.presets[preset] = ent
        elif v_ref:
            ent.setdefault("v_ref", float(v_ref))
        return ent

    def for_preset(self, preset, v_ref=None):
        """切换当前预设（P 键换火箭时调用）。"""
        self.preset = preset
        if v_ref:
            self.v_ref = float(v_ref)
        self._ensure(preset, v_ref)
        return self

    def reset_current(self):
        """把当前预设的表恢复到分析式初值。"""
        self.presets[self.preset] = {
            "v_ref": self.v_ref,
            "pitch_table": default_pitch_table(self.v_ref),
            "generation": 0,
            "history": [],
        }

    # ------------------------------------------------------------
    # 当前预设的视图
    # ------------------------------------------------------------
    @property
    def _entry(self):
        return self._ensure(self.preset, self.v_ref)

    @property
    def pitch_table(self):
        return self._entry["pitch_table"]

    @pitch_table.setter
    def pitch_table(self, value):
        self._entry["pitch_table"] = list(value)

    @property
    def generation(self):
        return self._entry["generation"]

    @generation.setter
    def generation(self, value):
        self._entry["generation"] = int(value)

    @property
    def history(self):
        return self._entry["history"]

    @history.setter
    def history(self, value):
        self._entry["history"] = list(value)

    # ------------------------------------------------------------
    # 读写文件
    # ------------------------------------------------------------
    def _load_file(self):
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[autolearn] 读取 {self.path} 失败，使用默认: {e}")
            return

        try:
            if data.get("version", 1) >= _SCHEMA_VERSION and \
                    isinstance(data.get("presets"), dict):
                for name, ent in data["presets"].items():
                    tbl = ent.get("pitch_table_rad")
                    if not (isinstance(tbl, list) and len(tbl) == len(SPEED_BUCKETS)):
                        continue
                    self.presets[name] = {
                        "v_ref": float(ent.get("v_ref", DEFAULT_V_REF)),
                        "pitch_table": [float(x) for x in tbl],
                        "generation": int(ent.get("generation", 0)),
                        "history": ent.get("history", []),
                    }
                print(f"[autolearn] 载入 {os.path.basename(self.path)}："
                      f"{len(self.presets)} 套预设的表")
            else:
                # v1 旧文件：一张全局表，归到 light 预设名下（当年的默认构型）
                tbl = data.get("pitch_table_rad")
                if isinstance(tbl, list) and len(tbl) == len(SPEED_BUCKETS):
                    self.presets["light"] = {
                        "v_ref": DEFAULT_V_REF,
                        "pitch_table": [float(x) for x in tbl],
                        "generation": int(data.get("generation", 0)),
                        "history": data.get("history", []),
                    }
                    print(f"[autolearn] 载入旧版（v1）{os.path.basename(self.path)}，"
                          f"归入预设 light")
        except Exception as e:
            print(f"[autolearn] {self.path} 内容异常，使用默认: {e}")

    def save(self):
        out = {"version": _SCHEMA_VERSION, "speed_buckets": SPEED_BUCKETS,
               "presets": {}}
        for name, ent in self.presets.items():
            out["presets"][name] = {
                "v_ref": ent.get("v_ref", DEFAULT_V_REF),
                "generation": ent["generation"],
                "pitch_table_rad": ent["pitch_table"],
                "pitch_table_deg": [math.degrees(p) for p in ent["pitch_table"]],
                "history": ent["history"][-20:],
            }
        try:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[autolearn] 写入 {self.path} 失败: {e}")

    # ------------------------------------------------------------
    def target_pitch(self, speed):
        """按速度插值查表，返回目标俯仰角（弧度）"""
        tbl = self.pitch_table
        if speed <= SPEED_BUCKETS[0]:
            return tbl[0]
        if speed >= SPEED_BUCKETS[-1]:
            return tbl[-1]
        for i in range(len(SPEED_BUCKETS) - 1):
            v0 = SPEED_BUCKETS[i]
            v1 = SPEED_BUCKETS[i + 1]
            if v0 <= speed <= v1:
                f = (speed - v0) / (v1 - v0)
                return tbl[i] * (1.0 - f) + tbl[i + 1] * f
        return tbl[-1]

    # ------------------------------------------------------------
    def observe_flight_end(self, rocket):
        """飞行结束时调用。分析结果，微调 pitch_table 并保存。
        返回本次奖励值（用于日志）。"""
        in_orbit = bool(rocket.in_orbit)
        crashed = bool(rocket.crashed or rocket.broken or rocket.burned_up)
        lunar = getattr(rocket, "mission", "leo") == "lunar"
        moon_orbit = bool(getattr(rocket, "ever_moon_orbit", False))

        # 解算终态径向 / 切向速度（相对主天体）
        vx = rocket.body.velocity.x
        vy = rocket.body.velocity.y
        rx = rocket.body.position.x
        ry = rocket.body.position.y + R_EARTH
        if getattr(rocket, "primary", "earth") == "moon":
            mx, my = rocket.moon_pos
            mvx, mvy = rocket.moon_vel
            rx -= mx
            ry -= my + R_EARTH
            vx -= mvx
            vy -= mvy
        r = math.hypot(rx, ry)
        if r > 1e-6:
            ux, uy = rx / r, ry / r
            v_radial = vx * ux + vy * uy
            v_tangent = vx * uy - vy * ux
        else:
            v_radial, v_tangent = vy, vx

        # 奖励：探月任务以"进入绕月轨道"为最高目标；入轨任务以地球轨道为准
        if lunar and moon_orbit:
            reward = 2.0
        elif lunar and crashed:
            reward = -1.0
        elif lunar and self._earth_orbit_ok(rocket):
            reward = 0.4            # 到了停泊轨道但没到月球：部分分
        elif in_orbit:
            reward = 1.0
        elif crashed:
            reward = -1.0
        else:
            reward = v_tangent / 7800.0 - abs(v_radial) / 2000.0

        self.history.append({
            "generation": self.generation,
            "preset": self.preset,
            "mission": "lunar" if lunar else "leo",
            "in_orbit": in_orbit,
            "ever_moon_orbit": moon_orbit,
            "missed_moon": bool(getattr(rocket, "missed_moon", False)),
            "crashed": crashed,
            "final_v_tangent": v_tangent,
            "final_v_radial": v_radial,
            "final_alt": rocket.altitude,
            "reward": reward,
        })

        # ---- 调整查找表（依据"地球上升段"的结果，因为这张表只管上升段）----
        # 探月任务的成败由末级制导决定，不能拿它去改上升段俯仰程序。
        earth_ok = (in_orbit and getattr(rocket, "primary", "earth") == "earth") \
            or self._earth_orbit_ok(rocket)
        if earth_ok:
            factor = 1.005                # 成功入轨：微增强
        elif abs(v_radial) > 1000:
            factor = 1.03                 # 垂直速度过大：整体更水平
        elif v_tangent < 7000:
            factor = 0.99                 # 水平速度不足：整体略回正
        else:
            factor = 1.0

        tbl = self.pitch_table
        for i in range(len(tbl)):
            tbl[i] = min(PITCH_MAX, max(0.0, tbl[i] * factor))

        self.generation += 1
        self.save()
        return reward

    @staticmethod
    def _earth_orbit_ok(rocket):
        """曾经进入过地球轨道（探月任务的中途目标）。"""
        return bool(getattr(rocket, "ever_earth_orbit", False))

# -*- coding: utf-8 -*-
"""物理常量、控制参数与火箭配置加载"""
import json
import math
import os

# ---- 窗口 ----
WIDTH, HEIGHT = 1200, 700
FPS = 60

# ---- 物理常量 ----
G0 = 9.80665
R_EARTH = 6.371e6
RHO0 = 1.225
H_SCALE = 8500.0

# ---- 载荷 / 整流罩（质量模型的顶端一段）----
PAYLOAD = 100.0                  # 载荷质量 (kg)
PAYLOAD_LEN = 3.0                # 载荷舱长度 (m)
PAYLOAD_RADIUS = 0.8             # 载荷舱半径 (m)
PAYLOAD_CD = 0.30                # 载荷舱单独再入时的阻力系数

# ---- 发射场 ----
LAUNCH_LAT_DEG = 19.6            # 文昌发射场纬度
EARTH_OMEGA = 7.2921159e-5       # 地球自转角速度 (rad/s)
COUNTDOWN_SECONDS = 10.0         # 倒计时长度

# ---- 月球 ----
# 二维模型里月球只能在同一个平面内运动，所以倾角、偏心率、交点进动都没有表示。
MOON_MU = 4.9048695e12           # 月球引力常数 G·M (m³/s²)
MOON_RADIUS = 1.7374e6           # 月球平均半径 (m)
MOON_DIST = 384.4e6              # 地月平均距离 (m)
MOON_PERIOD = 27.321661 * 86400  # 恒星月 (s)
MOON_OMEGA = 2.0 * math.pi / MOON_PERIOD        # 月球公转角速度 (rad/s)
MOON_SOI = 66.2e6                # 月球影响球半径 (m)：进出这个半径就换主天体
MOON_PHASE0_DEG = 0.0            # 起飞时的月球相位（0 = 正在发射场头顶）

# 绕月轨道判据（近月点海拔）
MOON_ORBIT_MIN_PERI = 50_000.0   # 判定"绕月运行"的近月点下限 (m)
MOON_ORBIT_EXIT_PERI = 30_000.0  # 掉出判据（滞回）

# ---- 控制参数 ----
TVC_MAX_DEG_LOW = 8.0            # 第一级 TVC 最大摆角
TVC_MAX_DEG_UP = 10.0            # 上面级 TVC 最大摆角
TVC_ACT_TAU = 0.05               # 作动器一阶延迟时间常数 (s)
TVC_RATE_MAX_DEG = 45.0          # 作动器最大角速率 (°/s)
TVC_BIAS_DEG = 0.25              # 推力固定偏置 (°)
RCS_FORCE = 6000.0               # 单侧 RCS 推力 (N)
RCS_LIMIT = 1.0                  # RCS 输出限幅

# ---- 失败阈值 ----
CRASH_SPEED = 30.0               # 撞地速度阈值 (m/s)
BREAK_Q = 100_000.0              # 解体动压阈值 (Pa)
BREAK_AOA = math.radians(30)     # 解体攻角阈值
BURNUP_HEAT = 5.0e6              # 再入烧毁热流阈值 (W/m²)

# ---- 入轨判据 ----
# 100 km 只是卡门线：近地点贴着 100 km 的轨迹会立刻被阻力吃掉，不算入轨。
ORBIT_MIN_PERIGEE_ALT = 180_000.0    # 判定入轨的近地点下限 (m)
ORBIT_EXIT_PERIGEE_ALT = 150_000.0   # 掉出轨道的判据（滞回，避免状态每帧抖动）

# ---- 时间加速 ----
# 上限由积分预算决定（见 main.py）：
#   推力段每块 ≤0.05 s、滑行段每块 ≤0.5 s、每帧最多 400 块
#   → 滑行段最高约 12000×，推力段最高约 1200×，超出时实际倍速会自动降低。
# 地月转移约 5 天 = 4.3e5 s，用 10000× 大约 43 秒真实时间看完。
TIME_SCALES = [1.0, 2.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0,
               2000.0, 5000.0, 10000.0]

# ---- 相机 ----
CAM_MAX_SCALE = 150.0            # 相机像素缩放上限 (px/m)

# ---- 配置文件开关 ----
USE_JSON_CONFIG = True

# ---- 默认火箭配置（能入轨的版本） ----
DEFAULT_STAGES = [
    dict(name="第一级", dry=550.0, fuel=5000.0, thrust=280e3, isp=300.0,
         radius=1.20, length=15.0, cd=0.35),
    dict(name="第二级", dry=220.0, fuel=1800.0, thrust=120e3, isp=345.0,
         radius=1.00, length=8.0, cd=0.32),
    dict(name="第三级", dry=100.0, fuel=900.0, thrust=35e3, isp=380.0,
         radius=0.80, length=4.5, cd=0.30),
]


# ============================================================
# 内置火箭预设
# ------------------------------------------------------------
# 每个预设是一枚完整火箭：
#   title / desc   显示名与一句话说明
#   mission        "leo"（入地球轨道）或 "lunar"（探月：入轨 → 地月转移 → 绕月）
#   payload        载荷质量 (kg)
#   payload_len / payload_radius   载荷舱尺寸 (m)
#   pitch_v_ref    俯仰程序参考速度：目标俯仰角 = 89.5° × min(1, v/v_ref)
#                  推重比越低，重力转弯应该越慢，v_ref 就要越大
#   stages         分级参数（见 _STAGE_REQUIRED）
#
# 这些数字都经过 tools/flight_test.py 实飞校核，见 README 9.4。
# ============================================================
PRESETS = {
    "sounding": {
        "title": "探空火箭 · 2 级",
        "desc": "亚轨道探空火箭，推重比 3.7。够不到入轨速度，"
                "正好用来演示\"入轨失败 → 再入 → 撞地\"的完整流程。",
        "mission": "leo",
        "payload": 30.0, "payload_len": 1.0, "payload_radius": 0.25,
        "pitch_v_ref": 3000.0,
        "stages": [
            dict(name="助推级", dry=120.0, fuel=900.0, thrust=45e3, isp=235.0,
                 radius=0.35, length=4.0, cd=0.40),
            dict(name="上面级", dry=40.0, fuel=150.0, thrust=12e3, isp=280.0,
                 radius=0.30, length=2.0, cd=0.35),
        ],
    },
    "light": {
        "title": "轻型三级 · 默认",
        "desc": "原默认构型。理想 Δv 约 11.8 km/s，余量很大；"
                "起飞推重比 3.3 偏高，实测 Max-Q 59 kPa（真实火箭 30~35）。",
        "mission": "leo",
        "payload": 100.0, "payload_len": 3.0, "payload_radius": 0.80,
        "pitch_v_ref": 3500.0,
        "stages": [dict(s) for s in DEFAULT_STAGES],
    },
    "medium": {
        "title": "中型三级 · 真实推重比",
        "desc": "按真实运载火箭重标定：起飞推重比 1.42、结构系数 8%/12%/19%、"
                "载荷占比 1.28%、理想 Δv 约 11.0 km/s。"
                "实测 Max-Q 约 31 kPa（真实火箭 30~35），重力损失比默认构型大得多。",
        "mission": "leo",
        "payload": 1200.0, "payload_len": 4.0, "payload_radius": 1.60,
        "pitch_v_ref": 3200.0,
        "stages": [
            dict(name="第一级", dry=6000.0, fuel=67770.0, thrust=1.304e6,
                 isp=300.0, radius=3.00, length=40.0, cd=0.35),
            dict(name="第二级", dry=1900.0, fuel=13820.0, thrust=900e3,
                 isp=340.0, radius=2.60, length=18.0, cd=0.32),
            dict(name="第三级", dry=550.0, fuel=2400.0, thrust=120e3,
                 isp=380.0, radius=1.80, length=8.0, cd=0.30),
        ],
    },
    "lunar": {
        "title": "重型四级 · 探月",
        "desc": "四级构型，理想 Δv 约 14.7 km/s，其中前三级的 10.1 km/s 负责进地球"
                "停泊轨道，转移级的 4.5 km/s 留给地月转移和近月制动。"
                "任务链：停泊轨道待机 → 等相位 → TLI → 滑行约 5 天 → 近月制动捕获。",
        "mission": "lunar",
        "payload": 250.0, "payload_len": 2.5, "payload_radius": 0.90,
        "pitch_v_ref": 3200.0,
        "stages": [
            dict(name="第一级", dry=3000.0, fuel=33000.0, thrust=740e3,
                 isp=300.0, radius=2.20, length=24.0, cd=0.35),
            dict(name="第二级", dry=1200.0, fuel=11000.0, thrust=250e3,
                 isp=340.0, radius=2.00, length=12.0, cd=0.32),
            dict(name="第三级", dry=500.0, fuel=4000.0, thrust=150e3,
                 isp=380.0, radius=1.60, length=7.0, cd=0.30),
            dict(name="转移级", dry=250.0, fuel=900.0, thrust=9e3,
                 isp=450.0, radius=1.20, length=5.0, cd=0.28),
        ],
    },
}

DEFAULT_PRESET = "light"
PRESET_ORDER = ["sounding", "light", "medium", "lunar"]


def preset_info(name):
    """返回预设的副本，避免调用方改到内置字典。"""
    p = PRESETS.get(name) or PRESETS[DEFAULT_PRESET]
    out = {k: v for k, v in p.items() if k != "stages"}
    out["stages"] = [dict(s) for s in p["stages"]]
    out["preset"] = name if name in PRESETS else DEFAULT_PRESET
    return out


def next_preset(name):
    """按 PRESET_ORDER 取下一个预设名（P 键切换用）。"""
    if name not in PRESET_ORDER:
        return PRESET_ORDER[0]
    return PRESET_ORDER[(PRESET_ORDER.index(name) + 1) % len(PRESET_ORDER)]

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "rocket_config.json")


# ============================================================
# JSON 注释剔除（让配置文件支持中文注释）
# ============================================================
def _strip_json_comments(text):
    result = []
    i = 0
    n = len(text)
    in_string = False
    while i < n:
        c = text[i]
        if in_string:
            result.append(c)
            if c == "\\" and i + 1 < n:
                result.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_string = False
            i += 1
        else:
            if c == '"':
                in_string = True
                result.append(c)
                i += 1
            elif c == "/" and i + 1 < n and text[i + 1] == "/":
                while i < n and text[i] != "\n":
                    i += 1
            elif c == "/" and i + 1 < n and text[i + 1] == "*":
                i += 2
                while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                    i += 1
                i += 2
            else:
                result.append(c)
                i += 1
    return "".join(result)


# ============================================================
# 默认配置文件文本
# ------------------------------------------------------------
# 唯一事实来源是 DEFAULT_STAGES：这里只负责把它渲染成带中文注释的 JSON。
# 以前 DEFAULT_STAGES 与 DEFAULT_CONFIG_TEXT 是两份手写内容，改一边忘一边
# 就会出现"删掉 json 之后行为变了"的鬼故事。
# ============================================================
_CONFIG_HEADER = """{
  // ============================================================
  // 火箭参数
  // ------------------------------------------------------------
  // 单位： 质量 kg | 推力 N | 比冲 s | 长度 m | 半径 m | cd 无量纲
  // ------------------------------------------------------------
  // preset 可选内置预设：__PRESETS__
  //   预设给出一整枚火箭（分级 + 载荷 + 任务）。下面这些字段可以覆盖预设值：
  //     "mission"        : "leo" 或 "lunar"
  //     "payload"        : 载荷质量 (kg)
  //     "payload_len" / "payload_radius" : 载荷舱尺寸 (m)
  //     "pitch_v_ref"    : 俯仰程序参考速度 (m/s)，推重比越低、重力转弯应越慢
  //   一旦给出 "stages"，就完全以它为准，不再使用预设的分级。
  // ------------------------------------------------------------
  // 比冲 Isp 参考：
  //   固体 240~270 | 液氧煤油 280~310 | 液氧液氢 380~450
  // 干重 dry 越低越好（不含燃料），现代火箭结构质量占比 <8%
  // 推重比 = 推力 / (起飞质量 × 9.8)，真实一级通常在 1.3~1.6
  // 注意："light" 预设的起飞推重比约 3.3，远高于真实火箭——它是早期为
  //       「必定入轨」留足余量而设的示范值。要真实感请用 "medium" 预设。
  // ============================================================
  "preset": "__PRESET__",
  "stages": [
"""

_FIELD_COMMENTS = {
    "name": "显示名称",
    "dry": "干重 (kg)：箭体+发动机+贮箱，不含燃料",
    "fuel": "燃料质量 (kg)",
    "thrust": "真空推力 (N)",
    "isp": "比冲 (s)：液氧煤油 280~310 / 液氧液氢 380~450",
    "radius": "箭体半径 (m)：决定气动阻力面积",
    "length": "箭体长度 (m)：决定转动惯量",
    "cd": "阻力系数：圆柱 0.3~0.5，尖头整流罩低至 0.1",
}


def _default_config_text(preset_name=DEFAULT_PRESET):
    """把某个内置预设渲染成带注释的 JSON 文本。"""
    info = preset_info(preset_name)
    header = _CONFIG_HEADER.replace("__PRESETS__", " / ".join(PRESET_ORDER))
    header = header.replace("__PRESET__", preset_name)
    keys = list(info["stages"][0].keys())
    blocks = []
    for st in info["stages"]:
        rows = []
        for idx, k in enumerate(keys):
            val = json.dumps(st[k], ensure_ascii=False)
            left = f'      "{k}": {val}' + ("," if idx < len(keys) - 1 else "")
            rows.append(left.ljust(30) + f"// {_FIELD_COMMENTS.get(k, '')}")
        blocks.append("    {\n" + "\n".join(rows) + "\n    }")
    return header + ",\n".join(blocks) + "\n  ]\n}\n"


DEFAULT_CONFIG_TEXT = _default_config_text()


# ============================================================
# 配置加载与校验
# ============================================================
CONFIG_SOURCE = "内置默认"        # 最近一次实际生效的参数来源（供日志/HUD 显示）

_STAGE_REQUIRED = ("dry", "fuel", "thrust", "isp", "radius", "length")


def _normalize_stages(raw, source):
    """校验并规范化分级参数。

    缺字段 / 非数值 / 非正数都会抛出带级号的 ValueError，由调用方决定回退策略。
    目的是让"配置写错"这件事立刻可见，而不是静默退回默认参数继续飞。
    """
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{source}: stages 必须是非空数组")
    out = []
    for i, cfg in enumerate(raw):
        tag = f"{source} 第 {i + 1} 级"
        if not isinstance(cfg, dict):
            raise ValueError(f"{tag}: 必须是 JSON 对象")
        missing = [k for k in _STAGE_REQUIRED if k not in cfg]
        if missing:
            raise ValueError(f"{tag}: 缺少字段 {', '.join(missing)}")
        try:
            st = dict(
                name=str(cfg.get("name", f"第{i + 1}级")),
                dry=float(cfg["dry"]),
                fuel=float(cfg["fuel"]),
                thrust=float(cfg["thrust"]),
                isp=float(cfg["isp"]),
                radius=float(cfg["radius"]),
                length=float(cfg["length"]),
                cd=float(cfg.get("cd", 0.35)),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{tag}: 字段不是数值 ({exc})") from exc
        if st["dry"] < 0.0 or st["fuel"] < 0.0:
            raise ValueError(f"{tag}: 干重与燃料不能为负")
        for k in ("thrust", "isp", "radius", "length", "cd"):
            if st[k] <= 0.0:
                raise ValueError(f"{tag}: {k} 必须为正数（当前 {st[k]}）")
        out.append(st)
    return out


def _positive(data, key, default, source, allow_none=True):
    """从配置里取一个正数标量，缺省时用预设值。"""
    val = data.get(key, None)
    if val is None:
        return default
    try:
        f = float(val)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source}: {key} 不是数值 ({exc})") from exc
    if f <= 0.0:
        raise ValueError(f"{source}: {key} 必须为正数（当前 {f}）")
    return f


def load_vehicle(preset=None):
    """载入本次要飞的火箭（内置预设 + 可选 JSON 覆盖），返回一个配置字典。

    返回字段：preset / title / desc / mission / payload / payload_len /
    payload_radius / pitch_v_ref / stages。
    生效来源记在 CONFIG_SOURCE，并显示在 HUD 事件区第一行。
    """
    global CONFIG_SOURCE

    if preset is not None:
        # 显式指定预设：完全按预设来，忽略 JSON（P 键切换、测试脚本、扫描用）
        if preset not in PRESETS:
            raise ValueError(f"未知预设 {preset!r}（可选：{', '.join(PRESET_ORDER)}）")
        base = preset_info(preset)
        base["stages"] = _normalize_stages(base["stages"], "内置预设")
        CONFIG_SOURCE = f"内置预设 {base['preset']}"
        print(f"[config] 使用内置预设 {base['preset']}：任务 {base['mission']}、"
              f"{len(base['stages'])} 级、载荷 {base['payload']:.0f} kg")
        return base

    base = preset_info(DEFAULT_PRESET)

    if not USE_JSON_CONFIG:
        CONFIG_SOURCE = f"内置预设 {base['preset']}（USE_JSON_CONFIG=False）"
        base["stages"] = _normalize_stages(base["stages"], "内置预设")
        print(f"[config] {CONFIG_SOURCE}")
        return base

    name = os.path.basename(CONFIG_FILE)
    if not os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                f.write(DEFAULT_CONFIG_TEXT)
            print(f"[config] 已生成默认配置 {CONFIG_FILE}")
        except Exception as e:
            print(f"[config] 无法生成 {CONFIG_FILE}: {e}")
        CONFIG_SOURCE = f"内置预设 {base['preset']}（{name} 不可用）"
        base["stages"] = _normalize_stages(base["stages"], "内置预设")
        return base

    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            raw = f.read()
        data = json.loads(_strip_json_comments(raw))
        if not isinstance(data, dict):
            raise ValueError("顶层必须是 JSON 对象")

        want = preset or data.get("preset") or DEFAULT_PRESET
        if want not in PRESETS:
            raise ValueError(f"未知预设 {want!r}（可选：{', '.join(PRESET_ORDER)}）")
        base = preset_info(want)

        mission = data.get("mission")
        if mission is not None:
            if mission not in ("leo", "lunar"):
                raise ValueError(f"mission 只能是 leo 或 lunar（当前 {mission!r}）")
            base["mission"] = mission
        base["payload"] = _positive(data, "payload", base["payload"], name)
        base["payload_len"] = _positive(data, "payload_len",
                                        base["payload_len"], name)
        base["payload_radius"] = _positive(data, "payload_radius",
                                           base["payload_radius"], name)
        base["pitch_v_ref"] = _positive(data, "pitch_v_ref",
                                        base["pitch_v_ref"], name)

        if data.get("stages") is not None:
            base["stages"] = _normalize_stages(data["stages"], name)
            CONFIG_SOURCE = f"{name}（自定义分级）"
        else:
            base["stages"] = _normalize_stages(base["stages"], "内置预设")
            CONFIG_SOURCE = f"{name}（预设 {base['preset']}）"
        print(f"[config] 已载入 {CONFIG_FILE}：预设 {base['preset']}、"
              f"任务 {base['mission']}、{len(base['stages'])} 级、"
              f"载荷 {base['payload']:.0f} kg")
        return base
    except Exception as e:
        fallback = preset_info(DEFAULT_PRESET)
        CONFIG_SOURCE = f"内置预设 {fallback['preset']}（{name} 不可用）"
        print(f"[config] 读取 {CONFIG_FILE} 失败，改用内置预设: {e}")
        fallback["stages"] = _normalize_stages(fallback["stages"], "内置预设")
        return fallback


def load_stages(preset=None):
    """兼容旧接口：只要分级参数。"""
    return load_vehicle(preset)["stages"]


# ============================================================
# 天体运动学（物理与渲染共用，保证两边算的是同一个位置）
# ============================================================
def moon_state(t):
    """返回 (世界坐标位置, 速度, 地心相位角)。

    近似：月球在发射场所在平面内做匀速圆周运动。二维模型只能这样——
    所以月球的轨道倾角、偏心率、交点进动、以及太阳摄动都没有表示。
    世界坐标里地球中心位于 (0, -R_EARTH)，月球的地心矢量为
    MOON_DIST * (sin φ, cos φ)，φ 从发射场正上方起算、向东为正。
    """
    ang = math.radians(MOON_PHASE0_DEG) + MOON_OMEGA * t
    ca, sa = math.cos(ang), math.sin(ang)
    pos = (MOON_DIST * sa, MOON_DIST * ca - R_EARTH)
    vel = (MOON_OMEGA * MOON_DIST * ca, -MOON_OMEGA * MOON_DIST * sa)
    return pos, vel, ang


def body_altitude(x, y, t):
    """返回 (地心距-地球半径, 月心距-月球半径, 月心距)。

    给相机取景用：谁近就以谁为参考，地月转移途中则以地球高度为准。
    """
    r_earth = math.hypot(x, y + R_EARTH) - R_EARTH
    mp, _, _ = moon_state(t)
    d_moon = math.hypot(x - mp[0], y - mp[1])
    return r_earth, d_moon - MOON_RADIUS, d_moon


def focus_altitude(x, y, t):
    """相机/高度表的参考高度：在月球影响球内看"距月面高度"，否则看"距地面高度"。"""
    r_earth, m_alt, d_moon = body_altitude(x, y, t)
    return m_alt if d_moon < MOON_SOI else r_earth


def gravity_accel(x, y, t):
    """地球 + 月球的引力加速度 (m/s²)，用于轨道预测等只关心引力的地方。"""
    rx = x
    ry = y + R_EARTH
    r = math.hypot(rx, ry)
    if r < 1.0:
        r = 1.0
    g = G0 * (R_EARTH / r) ** 2
    ax = -g * rx / r
    ay = -g * ry / r

    mp, _, _ = moon_state(t)
    dx = mp[0] - x
    dy = mp[1] - y
    d2 = dx * dx + dy * dy
    if d2 > 1.0:
        f = MOON_MU / (d2 * math.sqrt(d2))
        ax += f * dx
        ay += f * dy
    return ax, ay


# ============================================================
# 中文字体加载
# ============================================================
_FONT_FILES = [
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/msyhbd.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "C:/Windows/Fonts/simsun.ttc",
    "/System/Library/Fonts/PingFang.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
]


def load_font(size, bold=False):
    import pygame
    for p in _FONT_FILES:
        if os.path.exists(p):
            try:
                f = pygame.font.Font(p, size)
                if bold:
                    f.set_bold(True)
                return f
            except Exception:
                continue
    f = pygame.font.Font(None, size)
    if bold:
        f.set_bold(True)
    return f
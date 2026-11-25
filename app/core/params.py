"""机械臂物理口径的唯一来源。

`config/robot.json` / `config/vision.json` 是真值；本模块只做加载、校验和派生换算。
不放业务逻辑，不 import rclpy / cv2 / yaml —— 这样 ROS 包、视觉脚本和离线自检
都能在各自的环境里安全使用。

厂商/硬件节点（robo_driver、cello_controller、C++）不在运行时读本模块，
它们保留源码常量，由 scripts/checks/grip_check.py 强制对齐。
"""

import json
import math
import os
from pathlib import Path


def find_root() -> Path:
    env = os.environ.get("ROARM_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    for base in Path(__file__).resolve().parents:
        if (base / "config" / "static" / "robot.json").is_file():
            return base
    raise FileNotFoundError(
        "找不到 config/static/robot.json；请在仓库内运行，或设置 ROARM_ROOT")


ROOT = find_root()


def _load(name):
    with open(ROOT / "config" / "static" / name, encoding="utf-8") as f:
        return json.load(f)


ROBOT = _load("robot.json")
VISION = _load("vision.json")

ARM = ROBOT["arm"]
GRIP = ROBOT["gripper"]
SERIAL = ROBOT["serial"]

# ---------------------------------------------------------------- 位姿/路点
WAYPOINTS = {k: tuple(float(x) for x in v) for k, v in ARM["waypoints"].items()}
HOME_XYZ = WAYPOINTS["home"]
DEFAULT_QUAT = tuple(float(x) for x in ARM["default_quat"])
BOUNDS = {k: tuple(float(x) for x in v) for k, v in ARM["bounds"].items()}
ZERO_OFFSET_DEG = [float(x) for x in ARM["zero_offset_deg"]]
#: 收纳姿态（6 个臂关节的**舵机读数**，度）；不含夹爪。
PARK_POSE_DEG = [float(x) for x in ARM["park_pose_deg"]]

SERIAL_PORT = str(SERIAL["port"])
SERIAL_BAUD = int(SERIAL["baud"])

# ---------------------------------------------------------------- 夹爪端点
# 2026-11-07 队友标定（夹爪修复后换了一套几何）。**角度是相对起栈基准的偏移量**，
# 不是绝对舵机角：换个爪/断个电，多圈零点就变，同一个绝对角两次起栈可以一次
# 合拢、一次张到顶。driver 起栈时读一次实际角记为 base，之后 target = base + deg。
#
#   0°    = 基准位（约定：起栈前两指拨到合拢）  两指 1.6 mm  joint7_left -0.025
#   -100° = 从基准再张 100°（机械极限）         两指 50.9 mm joint7_left  0.000
#
# 角度→间距是二次曲线，不是线性：前 40° 基本是空程，线性映射会把中间开度算错。
CLOSE = float(GRIP["close_deg"])            # 基准位（合拢）
OPEN = float(GRIP["open_deg"])              # 全开（相对基准的偏移，负数）
FULL_MM = float(GRIP["full_mm"])            # 曲线满量程（= GMAX）
CURVE_K = float(GRIP["curve_k"])            # 曲线弯曲度，正号=前段快
JMIN = float(GRIP["joint_closed_m"])        # 闭合 joint7_left
JMAX = float(GRIP["joint_open_m"])          # 张开 joint7_left
GMIN = float(GRIP["gap_closed_mm"])         # 闭合两指间距
GMAX = float(GRIP["gap_open_mm"])           # 张开两指间距
START_DEG = GRIP.get("start_deg")           # None = 起栈原地不动（推荐）
START_DEG = None if START_DEG is None else float(START_DEG)

#: 语义化目标（mm）。别把 open 定在机械极限上，顶久了舵机发烫。
OPEN_TARGET_MM = float(GRIP.get("open_target_mm", GMAX))
CLOSE_TARGET_MM = float(GRIP.get("close_target_mm", GMIN))

#: 夹爪走完整个行程的时长（毫秒）。夹爪打印件脆，这个值别调小 ——
#: 1000 ms 走 100° 会把打印件和舵机一起打；驱动层还会再 +400 ms 缓冲。
MOVE_MS = int(GRIP.get("move_ms", 3000))
#: 发一条夹爪命令后，等它真的停下来需要多久（秒）。给 CLI 的 --wait 用。
MOVE_S = MOVE_MS / 1000.0 + 0.4
#: 建议的命令等待时间（秒）：运动时长 + 2 s 判据余量，向上取到整秒。
GRIP_WAIT_S = float(math.ceil(MOVE_S + 1.6))

#: 兼容旧代码的别名
GRIP_DEG_MIN, GRIP_DEG_MAX = min(CLOSE, OPEN), max(CLOSE, OPEN)
GAP_MIN_MM, GAP_MAX_MM = GMIN, GMAX
GAP_ZERO_POS = JMIN
GRIP_START_DEG = START_DEG

GRIP_EXPECT = {"close_fingers": JMIN, "open_fingers": JMAX}
GRIP_EXPECT_CLI = {"open": JMAX, "close": JMIN}

SERVO_SPAN = OPEN - CLOSE
JOINT_SPAN = JMAX - JMIN
GAP_SPAN = GMAX - GMIN

# ---------------------------------------------------------------- 视觉默认值
SCENE_Z_DEFAULT = float(VISION["scene"]["z_default_m"])
OBJECT_HEIGHT_DEFAULT_M = float(VISION["scene"]["object_height_default_m"])
GRASP_OFFSET_DEFAULT_MM = tuple(float(x) for x in VISION["grasp"]["offset_default_mm"])
Z_CMD_OFFSET_M = float(VISION["grasp"]["z_cmd_offset_m"])
#: 抓取点额外的基座 x/y 补偿（mm）。正值往左/往前，负值往右/往后
#: （与日志坐标轴一致）。2026-11-04 实抓：x 偏 +3mm → 补 -3；y 偏 +2mm → 补 -2。
GRASP_X_BIAS_MM = float(VISION["grasp"].get("x_bias_mm", 0.0))
GRASP_Y_BIAS_MM = float(VISION["grasp"].get("y_bias_mm", 0.0))
#: 是否在闭合后判断“夹到了东西”。夹爪机械件损坏/只空夹时设 false，
#: 流程继续抬升/放置；夹爪命令没送达（err=None）仍然会中止。
GRASP_CHECK = bool(VISION["grasp"].get("check_grasp", True))
REACH_BOUNDS = {k: tuple(float(x) for x in v)
                for k, v in VISION["reach"].items()
                if k in ("x", "y", "z")}
#: 颜色检测自己的可达包络 (x0, x1, y0, y1) mm，与 pick 的任务工作区分开。
#: 队友 color_detect.REACH_MM = (250, 670, -430, 380)；检测比抓取宽，
#: 是为了先看见画面边缘的物料，再由 pick 的 BOUNDS 决定能不能抓。
_DETECT_REACH = VISION.get("detect", {}).get("reach_mm")
DETECT_REACH_MM = tuple(float(x) for x in _DETECT_REACH) if _DETECT_REACH else None
#: 抓取点距底座的软上限（mm）。685 约为全展；超过它默认拒绝，可用 --allow-far
#: 或 --max-reach-mm 覆盖。这是经验值，不是机械极限。
MAX_PICK_RADIUS_MM = float(VISION["reach"].get("max_pick_radius_mm", 685.0))


def _color_ranges(kind):
    out = {}
    for name, d in VISION.get("colors", {}).items():
        out[name] = [(tuple(lo), tuple(hi)) for lo, hi in d.get(kind, [])]
    return out


#: HSV 阈值（OpenCV H 0~179）。material 是物料；tray 是托盘，检测时从
#: material mask 里减掉，避免大片托盘被当成物料。光照变化时改这里。
COLOR_RANGES = _color_ranges("material")
TRAY_RANGES = _color_ranges("tray")
#: 哪些颜色默认减托盘掩码。绿色物料和托盘 HSV 重叠，减了会把物料一起削掉，
#: 所以 green 默认 false；改用 --max-area 按面积排除大块托盘。
MASK_TRAY_COLORS = {
    name for name, d in VISION.get("colors", {}).items()
    if d.get("mask_tray", True)
}

_DETECT = VISION.get("detect", {})
#: 自动颜色识别的形态偏好：圆柱物料 aspect=w/h 小、实心度高；
#: 扁平托盘/背景布 aspect 大、实心度低。阈值可在 vision.json 调。
TALL_ASPECT_MAX = float(_DETECT.get("tall_aspect_max", 1.25))
TALL_SOLIDITY_MIN = float(_DETECT.get("tall_solidity_min", 0.78))
MAX_AREA_PX = float(_DETECT.get("max_area_px", 20000.0))


# ---------------------------------------------------------------- 校验
def _validate():
    if abs(SERVO_SPAN) < 1e-6:
        raise ValueError("夹爪 close_deg 与 open_deg 相同（%.1f），无法归一化"
                         % CLOSE)
    if not JMIN < JMAX:
        raise ValueError("joint7 端点顺序非法：%+.4f >= %+.4f" % (JMIN, JMAX))
    if not GMIN < GMAX:
        raise ValueError("两指间距端点顺序非法：%.1f >= %.1f mm" % (GMIN, GMAX))
    if START_DEG is not None and not (GRIP_DEG_MIN <= START_DEG <= GRIP_DEG_MAX):
        raise ValueError("启动位 %.1f° 不在夹爪区间 [%.1f, %.1f]°"
                         % (START_DEG, GRIP_DEG_MIN, GRIP_DEG_MAX))
    if not GRIP_DEG_MIN < GRIP_DEG_MAX:
        raise ValueError("夹爪角度区间非法：[%.1f, %.1f]"
                         % (GRIP_DEG_MIN, GRIP_DEG_MAX))
    if not GMIN <= CLOSE_TARGET_MM < OPEN_TARGET_MM <= GMAX:
        raise ValueError("夹爪目标开度非法：close=%.1f open=%.1f，须满足 "
                         "%.1f <= close < open <= %.1f"
                         % (CLOSE_TARGET_MM, OPEN_TARGET_MM, GMIN, GMAX))
    if len(ZERO_OFFSET_DEG) != 6:
        raise ValueError("zero_offset_deg 必须是 6 个臂关节，当前 %d 个"
                         % len(ZERO_OFFSET_DEG))
    if MOVE_MS < 1500:
        raise ValueError("夹爪 move_ms=%.0f 太快：夹爪打印件会崩，别低于 1500"
                         % MOVE_MS)


_validate()


# ---------------------------------------------------------------- 换算
def _clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


def _shape(x):
    """归一化开度 0..1 → 间距形状 0..1（二次曲线，前段空程）。"""
    x = _clamp(x, 0.0, 1.0)
    return x * x + CURVE_K * x * (1.0 - x)


def _shape_inv(y):
    """间距形状 0..1 → 归一化开度。解 (1−k)x² + kx − y = 0。"""
    y = _clamp(y, 0.0, 1.0)
    a = 1.0 - CURVE_K
    if abs(a) < 1e-9:
        return y
    return _clamp((-CURVE_K + math.sqrt(max(0.0, CURVE_K * CURVE_K
                                          + 4.0 * a * y))) / (2.0 * a), 0.0, 1.0)


def servo_to_frac(deg):
    """夹爪舵机角（相对基准）→ 归一化开度 0..1。"""
    if abs(SERVO_SPAN) < 1e-9:
        return 0.0
    return _clamp((float(deg) - CLOSE) / SERVO_SPAN, 0.0, 1.0)


def frac_to_servo(frac):
    """归一化开度 0..1（0=合拢，1=全开）→ 夹爪舵机角（相对基准）。"""
    return CLOSE + _clamp(frac, 0.0, 1.0) * SERVO_SPAN


def servo_to_joint(deg):
    """夹爪舵机角（度）→ joint7_left（米）。"""
    return JMIN + JOINT_SPAN * _shape(servo_to_frac(deg))


def joint_to_servo(pos_m):
    """joint7_left（米）→ 夹爪舵机角（度）。"""
    y = (float(pos_m) - JMIN) / JOINT_SPAN if abs(JOINT_SPAN) > 1e-9 else 0.0
    return CLOSE + _shape_inv(y) * SERVO_SPAN


def gap_mm_from_joint(pos_m):
    """joint7_left（米）→ 两指间距（毫米）。"""
    y = (float(pos_m) - JMIN) / JOINT_SPAN if abs(JOINT_SPAN) > 1e-9 else 0.0
    return GMIN + GAP_SPAN * _shape(y)


def gap_mm_from_servo(deg):
    """夹爪舵机角（度）→ 两指间距（毫米）。"""
    return GMIN + GAP_SPAN * _shape(servo_to_frac(deg))


def deg_of_gap(mm):
    """两指间距（毫米）→ 夹爪舵机角（度）。语义目标用它，别手写角度。"""
    y = (float(mm) - GMIN) / GAP_SPAN if abs(GAP_SPAN) > 1e-9 else 0.0
    return CLOSE + _shape_inv(y) * SERVO_SPAN


def frac_to_gap_mm(frac):
    """开度比例 0..1 → 两指间距（毫米）。"""
    return GMIN + GAP_SPAN * _shape(frac)


def gap_to_frac(mm):
    """两指间距（毫米）→ 开度比例 0..1。给"按物宽算合拢量"用。"""
    y = (float(mm) - GMIN) / GAP_SPAN if abs(GAP_SPAN) > 1e-9 else 0.0
    return _shape_inv(y)


def clamp_servo(deg):
    """把舵机角夹到安全区间；用于发命令前的最后一道保护。"""
    return _clamp(deg, GRIP_DEG_MIN, GRIP_DEG_MAX)


def clamp_joint(pos_m):
    """把 joint7_left 夹到 URDF 区间。"""
    return _clamp(pos_m, JMIN, JMAX)


def _selftest():
    eps = 1e-9
    assert abs(servo_to_joint(CLOSE) - JMIN) < eps
    assert abs(servo_to_joint(OPEN) - JMAX) < eps
    assert abs(joint_to_servo(JMIN) - CLOSE) < eps
    assert abs(joint_to_servo(JMAX) - OPEN) < eps
    assert abs(gap_mm_from_joint(JMIN) - GMIN) < eps
    assert abs(gap_mm_from_joint(JMAX) - GMAX) < eps
    assert abs(gap_mm_from_servo(CLOSE) - GMIN) < eps
    assert abs(gap_mm_from_servo(OPEN) - GMAX) < eps
    assert abs(frac_to_servo(0) - CLOSE) < eps
    assert abs(frac_to_servo(1) - OPEN) < eps
    assert abs(frac_to_gap_mm(0.0) - GMIN) < eps
    assert abs(frac_to_gap_mm(1.0) - GMAX) < eps
    # 曲线：中间开度不能退化成线性（线性 0.5 → 25.45 mm，曲线应更小）
    mid = frac_to_gap_mm(0.5)
    assert GMIN < mid < GMIN + GAP_SPAN * 0.5, "曲线没生效：%.2f mm" % mid
    # 语义目标来回：mm → 角 → mm 要能回来
    assert abs(gap_mm_from_servo(deg_of_gap(OPEN_TARGET_MM))
               - OPEN_TARGET_MM) < 1e-6
    assert GRIP_DEG_MIN <= deg_of_gap(OPEN_TARGET_MM) <= GRIP_DEG_MAX
    assert abs(clamp_servo(GRIP_DEG_MIN - 50) - GRIP_DEG_MIN) < eps
    assert GRIP_EXPECT_CLI["open"] == JMAX and GRIP_EXPECT_CLI["close"] == JMIN
    assert GRIP_EXPECT["close_fingers"] == JMIN
    assert GRIP_EXPECT["open_fingers"] == JMAX
    print("params selftest ok: close=%.1f° open=%.1f° gap=%.1f~%.1fmm "
          "open_target=%.1fmm(%.1f°) 中位=%.1fmm move=%dms(等%.0fs)"
          % (CLOSE, OPEN, GMIN, GMAX, OPEN_TARGET_MM,
             deg_of_gap(OPEN_TARGET_MM), frac_to_gap_mm(0.5),
             MOVE_MS, GRIP_WAIT_S))


if __name__ == "__main__":
    _selftest()
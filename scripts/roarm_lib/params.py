"""机械臂物理口径的唯一来源。

`config/robot.json` / `config/vision.json` 是真值；本模块只做加载、校验和派生换算。
不放业务逻辑，不 import rclpy / cv2 / yaml —— 这样 ROS 包、视觉脚本和离线自检
都能在各自的环境里安全使用。

厂商/硬件节点（robo_driver、cello_controller、C++）不在运行时读本模块，
它们保留源码常量，由 scripts/checks/grip_check.py 强制对齐。
"""

import json
import os
from pathlib import Path


def find_root() -> Path:
    env = os.environ.get("ROARM_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    for base in Path(__file__).resolve().parents:
        if (base / "config" / "robot.json").is_file():
            return base
    raise FileNotFoundError(
        "找不到 config/robot.json；请在仓库内运行，或设置 ROARM_ROOT")


ROOT = find_root()


def _load(name):
    with open(ROOT / "config" / name, encoding="utf-8") as f:
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
CLOSE = float(GRIP["servo_close_deg"])      # 闭合舵机角
OPEN = float(GRIP["servo_open_deg"])        # 张开舵机角
JMIN = float(GRIP["joint_closed_m"])        # 闭合 joint7_left
JMAX = float(GRIP["joint_open_m"])          # 张开 joint7_left
GMIN = float(GRIP["gap_closed_mm"])         # 闭合两指间距
GMAX = float(GRIP["gap_open_mm"])           # 张开两指间距
START_DEG = float(GRIP["start_deg"])        # 上电启动位

#: 兼容旧代码的别名
GRIP_DEG_MIN, GRIP_DEG_MAX = CLOSE, OPEN
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
    if not CLOSE < OPEN:
        raise ValueError("夹爪端点顺序非法：close=%.1f° >= open=%.1f°" % (CLOSE, OPEN))
    if not JMIN < JMAX:
        raise ValueError("joint7 端点顺序非法：%+.4f >= %+.4f" % (JMIN, JMAX))
    if not GMIN < GMAX:
        raise ValueError("两指间距端点顺序非法：%.1f >= %.1f mm" % (GMIN, GMAX))
    if not CLOSE <= START_DEG <= OPEN:
        raise ValueError("启动位 %.1f° 不在夹爪区间 [%.1f, %.1f]°"
                         % (START_DEG, CLOSE, OPEN))
    if len(ZERO_OFFSET_DEG) != 6:
        raise ValueError("zero_offset_deg 必须是 6 个臂关节，当前 %d 个"
                         % len(ZERO_OFFSET_DEG))


_validate()


# ---------------------------------------------------------------- 换算
def _clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


def servo_to_joint(deg):
    """夹爪舵机角（度）→ joint7_left（米）。"""
    return JMIN + (float(deg) - CLOSE) * JOINT_SPAN / SERVO_SPAN


def joint_to_servo(pos_m):
    """joint7_left（米）→ 夹爪舵机角（度）。"""
    return CLOSE + (float(pos_m) - JMIN) * SERVO_SPAN / JOINT_SPAN


def gap_mm_from_joint(pos_m):
    """joint7_left（米）→ 两指间距（毫米）。"""
    return GMIN + (float(pos_m) - JMIN) * GAP_SPAN / JOINT_SPAN


def gap_mm_from_servo(deg):
    """夹爪舵机角（度）→ 两指间距（毫米）。"""
    return GMIN + (float(deg) - CLOSE) * GAP_SPAN / SERVO_SPAN


def frac_to_servo(frac):
    """开度比例 0..1（0=全闭，1=全开）→ 舵机角（度）。"""
    return CLOSE + _clamp(frac, 0.0, 1.0) * SERVO_SPAN


def frac_to_gap_mm(frac):
    """开度比例 0..1 → 两指间距（毫米）。"""
    return GMIN + _clamp(frac, 0.0, 1.0) * GAP_SPAN


def clamp_servo(deg):
    """把舵机角夹到安全区间；用于发命令前的最后一道保护。"""
    return _clamp(deg, CLOSE, OPEN)


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
    assert GRIP_EXPECT_CLI["open"] == JMAX and GRIP_EXPECT_CLI["close"] == JMIN
    assert GRIP_EXPECT["close_fingers"] == JMIN
    assert GRIP_EXPECT["open_fingers"] == JMAX
    print("params selftest ok: close=%.1f° open=%.1f° gap=%.0f~%.0fmm"
          % (CLOSE, OPEN, GMIN, GMAX))


if __name__ == "__main__":
    _selftest()
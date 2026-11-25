"""仓库路径的唯一来源。

任何模块或工具都不要再手写 parents[N] / "data" 字符串；统一从这里取。
路径按「静态配置 / 标定产物 / 运行期产物 / 代码分层」清晰拆分。
"""

import os
from pathlib import Path


def _root() -> Path:
    env = os.environ.get("ROARM_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    # app/core/paths.py -> core -> app -> 仓库根
    return Path(__file__).resolve().parents[2]


ROOT = _root()

# 代码分层目录
APP = ROOT / "app"
APP_CORE = APP / "core"
APP_PERCEPTION = APP / "perception"
APP_CONTROL = APP / "control"
APP_SERVICE = APP / "service"
TOOLS = ROOT / "tools"
TOOLS_BRINGUP = TOOLS / "bringup"
TOOLS_CALIB_ARM = TOOLS / "calib" / "arm"
TOOLS_CALIB_CAM = TOOLS / "calib" / "camera"
TOOLS_DIAG = TOOLS / "diag"
TESTS = ROOT / "tests"
SCRIPTS = ROOT / "scripts"  # 仅保留给外部 ssvep-bci 的 3 个兼容 shim

# ROS 2 工作区
WS = ROOT / "star-arm-moveit2"
SRC = WS / "src"
WS_SETUP = WS / "install" / "setup.bash"
URDF = SRC / "cello_description" / "urdf" / "cello_description.urdf"

# 配置与标定数据（静态 vs 标定彻底分离）
CONFIG = ROOT / "config"
CONFIG_STATIC = CONFIG / "static"
CONFIG_CALIB = CONFIG / "calibrated"
CONFIG_CALIB_ARM = CONFIG_CALIB / "arm"
CONFIG_CALIB_CAM = CONFIG_CALIB / "camera"

ROBOT_JSON = CONFIG_STATIC / "robot.json"
VISION_JSON = CONFIG_STATIC / "vision.json"

SCENE_REAL = CONFIG_CALIB_ARM / "scene_real.yaml"
POSITIONS_6 = CONFIG_CALIB_ARM / "positions_6.yaml"
TEACH_POINTS = CONFIG_CALIB_ARM / "teach_points.json"
TRAYS_JSON = CONFIG_CALIB_CAM / "trays.json"

# 运行期产物（gitignored）
RUNTIME = ROOT / "runtime"
RUNTIME_RUNS = RUNTIME / "runs"
RUNTIME_PREVIEW = RUNTIME / "preview"
ARM_SOCK = RUNTIME / "arm.sock"
TIMING_JSONL = RUNTIME / "timing.jsonl"

# 核心入口路径
CLI = APP_CONTROL / "roarm_cli.py"
PICK_VISION = APP_CONTROL / "pick_vision.py"
PICK_FIXED = APP_CONTROL / "pick_fixed.py"
SIM_GRIPPER_BRIDGE = APP_CONTROL / "sim_gripper_bridge.py"

# 过渡期语义别名（便于未迁移的旧脚本平滑过渡）
DATA = CONFIG_CALIB_ARM
DATA_VISION = CONFIG_CALIB_CAM

"""任务层的可调参数与口径映射。

设计原则：这里只放"数"和"映射"，不放逻辑。
改行为请改 primitives.py / sequence.py；改**物理口径**（夹爪端点、零位、home、
边界）请改仓库根 `config/robot.json`，本文件只保留任务层自己的调参。
"""

import importlib.util
import os
import sys
from pathlib import Path


# ---------------------------------------------------------------- 共享参数
def _repo_root() -> Path:
    """定位仓库根（含 scripts/roarm_cli.py 的目录）。

    --symlink-install 时 __file__ 指向 src；普通安装指向 install/…/site-packages。
    两种层级都能通过向上查找命中仓库根。非常规部署用 ROARM_ROOT 覆盖。
    """
    env = os.environ.get("ROARM_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    here = Path(__file__).resolve()
    for base in here.parents:
        if (base / "scripts" / "roarm_cli.py").is_file():
            return base
    raise RuntimeError(
        "找不到仓库根（scripts/roarm_cli.py）；用 ROARM_ROOT 指定")


def _load_params():
    """加载 scripts/roarm_lib/params.py。

    任务层可能运行在 install/build 拷贝里，不能假设 scripts 在 sys.path 上；
    按路径显式加载，保证和脚本侧读的是同一份 config/robot.json。
    """
    path = _repo_root() / "scripts" / "roarm_lib" / "params.py"
    spec = importlib.util.spec_from_file_location("_roarm_params", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("无法加载 %s" % path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


P = _load_params()


# ---------------------------------------------------------------- 底层通道
def _find_cli() -> Path:
    env = os.environ.get("ROARM_CLI")
    if env:
        return Path(env).expanduser()
    return _repo_root() / "scripts" / "roarm_cli.py"


#: 终端控臂脚本，任务层通过子进程调用它。
#: 不直接发 ROS 消息是刻意的：到位判据只保留一份（CLI 里的 TF 校验 +
#: 三档退出码），避免任务层和 CLI 各有一套判据、日后口径漂移。
CLI = _find_cli()

#: 单条原语的超时（秒）。CLI 内部还会自己等 --wait，这是外层保险。
CALL_TIMEOUT = 60.0

# ---------------------------------------------------------------- 位姿约定
#: 末端固定姿态（零位实测 world->link6，RPY = -90/0/-90）。
#: 保持它就可以"只改位置不改朝向"，夹爪始终朝下。
DEFAULT_QUAT = P.DEFAULT_QUAT

#: 笛卡尔回位点（注意：不是关节零位，IK 会解出 j4≈5.9 / j6≈-5.9）
HOME_XYZ = P.HOME_XYZ

#: 到位容差（毫米）。
#: ⚠ 必须按**实测精度**定，不能按期望。2026-10-18 实测：
#:   - 绝对定位误差稳定在 10~11 mm（IK 每次解出的关节角带 1° 级舍入）
#:   - home 点本身就有 15.4 mm 系统偏差
#: 原来写 5 mm，比硬件能力还严 → 第 1 步回位就必然失败，然后在"失败→回位→再失败"里打转。
TOL_MM = 15

#: 二次定位补偿的目标容差（毫米）。取物下探用：补偿一次能把 10~11 mm 压到 2~3 mm。
PRECISE_TOL_MM = 5.0

#: 回位（home）单独用更宽的容差：它只要求"回到大致位置"，不需要准。
#: 15 mm 的实测偏差 + 余量，取 25。
HOME_TOL_MM = 25

#: 接近/撤离的站立偏移（米）
STANDOFF_M = 0.08

#: 抬起高度（米）。只为把夹爪抬离台面，不做避障。
LIFT_M = 0.10

# ---------------------------------------------------------------- 夹爪口径
#: 端点/开度全部来自 config/robot.json（2026-11-03 拆装后卸力手推实测）：
#:   舵机  86.7° -> joint7_left -0.025 m -> 两指合拢 4 mm   ← CLI "close"
#:   舵机 174.8° -> joint7_left  0.000 m -> 两指张开 65 mm  ← CLI "open"
#: 旧值 21.9°~100°、8~57 mm 已作废。
#:
#: 任务层仍然不写 open/close 字面量，只用物理语义：
#:   close_fingers = 两指合拢 = 夹住
#:   open_fingers  = 两指张开 = 松开
GRIP_CMD = {
    "close_fingers": "close",
    "open_fingers": "open",
}

GRIP_DEG_MIN, GRIP_DEG_MAX = P.CLOSE, P.OPEN
GAP_MIN_MM, GAP_MAX_MM = P.GMIN, P.GMAX
GAP_ZERO_POS = P.JMIN

#: 夹爪到位判据：joint7_left 的期望值（米），由端点派生。
GRIP_EXPECT = dict(P.GRIP_EXPECT)

# ---------------------------------------------------------------- 序列行为
#: 单个原语失败后是否重试一次。抓取类动作重试有风险（可能已撞到东西），
#: 所以默认只对移动类重试，见 primitives.RETRYABLE。
RETRY_ONCE = True

#: 失败时是否自动回 HOME。留 True：任务中断后停在半空更危险。
RETURN_HOME_ON_FAIL = True

#: 判定夹空后最多重试几次。每次重试都会完整走一遍
#: 「张开 → 退回接近点 → 下探 → 合爪」，代价约 8~10 s。
GRASP_MAX_RETRY = 1

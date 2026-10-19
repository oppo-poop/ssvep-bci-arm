"""任务层的全部可调参数与口径常量。

设计原则：这里只放"数"和"映射"，不放逻辑。
改行为请改 primitives.py / sequence.py，改口径请只改本文件。
"""

import os
from pathlib import Path

# ---------------------------------------------------------------- 底层通道
def _find_cli() -> Path:
    """定位 scripts/roarm_cli.py。

    不写死 parents[N]：--symlink-install 下 __file__ 可能指向 src，也可能
    指向 install，层级数不一样。改为从自身向上逐级找 scripts/roarm_cli.py。
    """
    env = os.environ.get("ROARM_CLI")
    if env:
        return Path(env).expanduser()
    here = Path(__file__).resolve()
    for base in here.parents:
        cand = base / "scripts" / "roarm_cli.py"
        if cand.is_file():
            return cand
    return here.parent / "scripts" / "roarm_cli.py"  # 兜底，调用时会报缺文件


#: 终端控臂脚本，任务层通过子进程调用它。
#: 不直接发 ROS 消息是刻意的：到位判据只保留一份（CLI 里的 TF 校验 +
#: 三档退出码），避免任务层和 CLI 各有一套判据、日后口径漂移。
CLI = _find_cli()

#: 单条原语的超时（秒）。CLI 内部还会自己等 --wait，这是外层保险。
CALL_TIMEOUT = 60.0

# ---------------------------------------------------------------- 位姿约定
#: 末端固定姿态（零位实测 world->link6，RPY = -90/0/-90）。
#: 保持它就可以"只改位置不改朝向"，夹爪始终朝下。
DEFAULT_QUAT = (0.5, -0.5, 0.5, -0.5)

#: 笛卡尔回位点（注意：不是关节零位，IK 会解出 j4≈5.9 / j6≈-5.9）
HOME_XYZ = (0.479, 0.0, 0.369)

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
#: ⚠ 2026-10-18 **实物实测**，推翻了 10-17 从仿真推出来的结论：
#:
#:   舵机 100° -> joint7_left  0.000 -> 两指**张开**（约 50 mm）   ← CLI "open"
#:   舵机 50°  -> joint7_left -0.016 -> 两指半开（约 32 mm）       ← driver 启动位
#:   舵机 21.9°-> joint7_left -0.025 -> 两指**合拢**（0 mm）       ← CLI "close"
#:
#: 所以 CLI 的 open/close 现在**是自然语义**（open=张开、close=合拢），
#: 不再是 10-17 记录的"名字相反"。下面这张表因此变成恒等映射。
#:
#: 任务层仍然不写 open/close 字面量，只用物理语义：
#:   close_fingers = 两指合拢 = 夹住
#:   open_fingers  = 两指张开 = 松开
GRIP_CMD = {
    "close_fingers": "close",
    "open_fingers": "open",
}

#: 夹爪到位判据：joint7_left 的期望值（米）。
#: 实测：0.000 = 张开，-0.025 = 合拢（URDF 下限）。
GRIP_EXPECT = {
    "close_fingers": -0.025,
    "open_fingers": 0.0,
}

# ---------------------------------------------------------------- 序列行为
#: 单个原语失败后是否重试一次。抓取类动作重试有风险（可能已撞到东西），
#: 所以默认只对移动类重试，见 primitives.RETRYABLE。
RETRY_ONCE = True

#: 失败时是否自动回 HOME。留 True：任务中断后停在半空更危险。
RETURN_HOME_ON_FAIL = True

#: 判定夹空后最多重试几次。每次重试都会完整走一遍
#: 「张开 → 退回接近点 → 下探 → 合爪」，代价约 8~10 s。
GRASP_MAX_RETRY = 1

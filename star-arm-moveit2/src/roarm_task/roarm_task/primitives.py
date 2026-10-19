"""七个动作原语。序列层只该调用这里的函数，不该直接碰 link。

坐标一律是**世界系绝对坐标**（米），相对位移由序列层自己算 ——
无状态的原语更容易测试，也更容易在纸上画清楚每一步在哪儿。
"""

import math
from typing import Optional, Tuple

from . import config, link

Xyz = Tuple[float, float, float]

#: 允许自动重试一次的原语。夹爪动作不在里面：夹爪可能已经夹住了东西，
#: 重复发命令会让它再夹一次，物理上没意义、有时还有害。
RETRYABLE = {"approach", "descend", "lift", "transfer", "retreat", "home"}


def _run(name: str, fn) -> link.Result:
    res = fn()
    if res.ok or res.rejected:
        # 参数被拒（越界/非法四元数）重试也没用，直接返回
        return res
    if config.RETRY_ONCE and name in RETRYABLE:
        retry = fn()
        retry.action = name + " (retry)"
        return retry
    return res


def approach(target: Xyz, standoff: Optional[float] = None) -> link.Result:
    """移动到目标点正上方 standoff 米处。"""
    s = config.STANDOFF_M if standoff is None else standoff
    x, y, z = target
    return _run("approach", lambda: link.move(x, y, z + s, action="approach"))


def _actual_from(raw: str):
    """从 CLI 的 pose 输出里取实际位置，省掉单独一次 where 调用。

    CLI 本来就打印 `实际 x=.. y=.. z=..`，独立跑 where 要多起一个进程约 2 s。
    """
    import re
    m = re.search(r"实际 x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)", raw or "")
    return tuple(float(g) for g in m.groups()) if m else None


def move_precise(target: Xyz, tol_mm: float = 3.0, max_iter: int = 2,
                 action: str = "move_precise") -> link.Result:
    """带二次定位补偿的移动：走到目标后读实测，把偏差反着补进下一次目标。

    为什么有效：开口环的绝对误差是**稳定的系统偏差**（10~11 mm，同一轮内 σ 只有 0.7 mm），
    而重复定位精度 3σ 只有 2.2 mm。所以"目标 = 2×原目标 − 实测"这一步能把落点从
    ~11 mm 压到 2~3 mm 量级。

    代价：每次多一次 `where` 调用（约 1.5 s）+ 可能多一次移动。
    只用在真正影响成败的点上（取物下探），不要每步都用。
    """
    want = tuple(float(v) for v in target)
    res = link.move(*want, tol_mm=tol_mm, action=action)
    if res.rejected:
        return res

    for it in range(max_iter + 1):
        # 先从刚那次 move 的输出里拿位置；拿不到才另起一次 where
        act = _actual_from(res.raw)
        pos = act if act is not None else link.where()
        if pos is None:
            res.detail = "读不到实测位置，保持原结果（%s）" % res.detail
            return res
        act = tuple(pos[:3])
        err = math.dist(act, want) * 1000
        if err <= tol_mm:
            # ⚠ Result.ok 是 property（rc == 0 派生），不能赋值，改 rc
            res.rc = link.EXIT_OK
            res.detail = ("首轮即到 %.1f mm" % err if it == 0
                          else "补偿 %d 次后 %.1f mm" % (it, err))
            return res
        if it == max_iter:
            res.rc = link.EXIT_NOT_REACHED
            res.detail = "补偿 %d 次仍差 %.1f mm" % (it, err)
            return res

        # 反着补：目标 = 2×原目标 − 实测
        aim = tuple(2 * want[k] - act[k] for k in range(3))
        res = link.move(*aim, tol_mm=tol_mm,
                        action="%s/corr%d" % (action, it + 1))
        if res.rejected:
            return res
        print("    补偿 %d：实测偏 %.1f mm，目标修正 %+.1f/%+.1f/%+.1f mm"
              % (it + 1, err, (aim[0] - want[0]) * 1000,
                 (aim[1] - want[1]) * 1000, (aim[2] - want[2]) * 1000))
    return res


def descend(target: Xyz, precise: bool = False) -> link.Result:
    """竖直下降到目标点。

    precise=True 时用二次定位补偿（取物点用；放置点不需要这么准）。
    """
    x, y, z = target
    if precise:
        # max_iter=1：补一次就够。实测第二次补偿往往只再收 0.3~0.5 mm，
        # 却要多花一次 move + 一次 where（约 4.5 s），不划算。
        return _run("descend", lambda: move_precise(
            (x, y, z), tol_mm=config.PRECISE_TOL_MM, max_iter=1,
            action="descend"))
    return _run("descend", lambda: link.move(x, y, z, action="descend"))


def grasp() -> link.Result:
    """两指合拢（夹住）。

    注意：底层 CLI 命令是 "open" —— 实测它才是物理合拢的那个。
    见 config.GRIP_CMD 的说明，别在这里改字符串。
    """
    return _run("grasp", lambda: link.grip("close_fingers", action="grasp"))


def lift(target: Xyz, height: Optional[float] = None) -> link.Result:
    """从目标点竖直抬升。"""
    h = config.LIFT_M if height is None else height
    x, y, z = target
    return _run("lift", lambda: link.move(x, y, z + h, action="lift"))


def transfer(place: Xyz, standoff: Optional[float] = None) -> link.Result:
    """平移到放置点上方（保持夹持）。"""
    s = config.STANDOFF_M if standoff is None else standoff
    x, y, z = place
    return _run("transfer", lambda: link.move(x, y, z + s, action="transfer"))


def release() -> link.Result:
    """两指张开（松开）。底层 CLI 命令是 "close"。"""
    return _run("release", lambda: link.grip("open_fingers", action="release"))


def retreat(place: Xyz, height: Optional[float] = None) -> link.Result:
    """放开后从放置点撤离。"""
    h = config.STANDOFF_M if height is None else height
    x, y, z = place
    return _run("retreat", lambda: link.move(x, y, z + h, action="retreat"))


def go_home() -> link.Result:
    """回笛卡尔 home 点。注意它不是关节零位。

    用比 pick/place 更宽的容差：home 只要求"回到大致位置"，
    而它本身有 15.4 mm 系统偏差（IK 关节舍入），按 15 mm 判会随机失败。
    """
    return _run("home", lambda: link.move(*config.HOME_XYZ,
                                          tol_mm=config.HOME_TOL_MM,
                                          action="home"))


# ---------------------------------------------------------------- 抓取校验
#: 两指间距换算。**2026-10-18 卡尺实测**（不是推算）：
#:   joint7_left  0.000（舵机 100.0°）→ 两指张开 **65 mm**
#:   joint7_left -0.025（舵机  21.9°）→ 两指 **8 mm**（合到底，两指间有结构间隙）
#: 即行程 57 mm 对应 25 mm 的 joint7_left 位移。
GAP_MIN_MM = 8.0
GAP_MAX_MM = 65.0
#: 完全合拢时 joint7_left 的值（URDF 下限）
GAP_ZERO_POS = -0.025
GAP_MM_PER_M = (GAP_MAX_MM - GAP_MIN_MM) / abs(GAP_ZERO_POS)

#: 对应的舵机角度可用区间（低于 21.9° 会让 joint7_left 超 URDF 下限，
#: 一旦超限 MoveIt 会把整机状态判为 invalid，之后所有规划全部失败）
GRIP_DEG_MIN, GRIP_DEG_MAX = 21.9, 100.0


def gripper_gap_mm(pos: Optional[float]) -> Optional[float]:
    """由 joint7_left 换算两指间距（毫米）。

    注意最小不是 0 而是 8 mm：夹爪合到底时两指仍有结构间隙，
    所以"夹空"的判据不能用 <2 mm，要用 ≤ GAP_MIN_MM + 1。
    """
    if pos is None:
        return None
    return GAP_MIN_MM + (pos - GAP_ZERO_POS) * GAP_MM_PER_M


def set_grip_frac(frac: float, action: str = "grip_open") -> link.Result:
    """按比例开夹爪：0 = 合到底(8 mm)，1 = 全开(65 mm)。

    为什么需要它：二值 open/close 只有 65 / 8 两档。
    张开要有度（全开可能碰到旁边的物体），合拢也要有度（合到略小于物宽即可，
    合到底会让舵机长时间堵转）。
    """
    frac = max(0.0, min(1.0, float(frac)))
    deg = GRIP_DEG_MIN + frac * (GRIP_DEG_MAX - GRIP_DEG_MIN)
    return _run(action, lambda: link.grip_deg(deg, action=action))


def check_grasp(object_width: float, tol_mm: float = 3.0):
    """合爪后判断是否真的夹住了东西。

    返回 (grabbed, joint7_left, note)：
        grabbed = True   夹住了（间距接近物宽）
        grabbed = False  夹空（合到底）
        grabbed = None   读不到夹爪状态，无法判断

    依据：真机上夹爪夹住物体后会停在物宽附近，夹空则会合到底。
    ⚠ 仿真里 FakeSystem 直接把夹爪送到目标位置，**这个判据不成立**，
      所以 Scene.verify_grasp 默认是 False。要真判必须上真机或物理仿真。
    """
    pos = link.gripper_state()
    gap = gripper_gap_mm(pos)
    if gap is None:
        return None, None, "读不到 joint7_left（仿真桥没起？）"
    # 合到底时两指仍有 8 mm 结构间隙，不能再用 "<2 mm" 判夹空
    if gap <= GAP_MIN_MM + 1.0:
        return False, pos, "间距 %.1f mm（≈最小开口 %.0f mm）→ 判夹空" % (
            gap, GAP_MIN_MM)
    if gap <= object_width * 1000.0 + tol_mm:
        return True, pos, "间距 %.1f mm（物宽 %.0f mm）→ 判夹住" % (
            gap, object_width * 1000)
    return False, pos, "间距 %.1f mm 明显大于物宽 %.0f mm → 判没夹到" % (
        gap, object_width * 1000)

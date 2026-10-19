"""任务层与执行节点之间的唯一通道。

任务层不自己发 ROS 消息、也不自己算 TF —— 它调用 scripts/roarm_cli.py，
把"到位与否"的判断权留在那一个地方。这样 CLI 的三档退出码就成了任务层
可以依赖的契约：

    0  到位 / 命令被接受并完成
    1  发出去了但没到位（超时、IK 失败、规划失败）
    2  输入被拒（越界、四元数非法）—— 根本没发给执行器

这样做的代价是每个原语多 0.5~1 s 的进程启动开销；
换来的是判据只有一份，不会出现"任务层以为到了、CLI 以为没到"。
"""

import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

from . import config

EXIT_OK = 0
EXIT_NOT_REACHED = 1
EXIT_REJECTED = 2


@dataclass
class Result:
    """一条原语的执行结果。"""

    action: str
    rc: int
    detail: str = ""
    elapsed: float = 0.0
    argv: List[str] = field(default_factory=list)
    raw: str = ""

    @property
    def ok(self) -> bool:
        return self.rc == EXIT_OK

    @property
    def rejected(self) -> bool:
        """参数/边界被拒，命令没有发到执行器。"""
        return self.rc == EXIT_REJECTED

    def __str__(self) -> str:
        if self.ok:
            tag = "OK"
        elif self.rejected:
            tag = "REJECT"
        else:
            tag = "FAIL"
        return "%-16s %-6s %5.1fs  %s" % (self.action, tag, self.elapsed, self.detail)


def _tail(text: str, n: int = 3) -> str:
    """取输出的最后几行非空内容，用于结果摘要。"""
    lines = [ln.strip() for ln in (text or "").splitlines() if ln.strip()]
    return " | ".join(lines[-n:])


def call(args: List[str], action: str, timeout: Optional[float] = None) -> Result:
    """调用 CLI。不抛异常，任何失败都折成 Result。"""
    if not config.CLI.is_file():
        return Result(action=action, rc=EXIT_REJECTED,
                      detail="找不到 CLI: %s（用 ROARM_CLI 环境变量指定）" % config.CLI)
    argv = [sys.executable, str(config.CLI)] + [str(a) for a in args]
    t0 = time.time()
    try:
        proc = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout or config.CALL_TIMEOUT,
        )
        elapsed = time.time() - t0
        out = (proc.stdout or "") + (proc.stderr or "")
        return Result(action=action, rc=proc.returncode,
                      detail=_tail(out), elapsed=elapsed, argv=argv, raw=out)
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        return Result(action=action, rc=EXIT_NOT_REACHED,
                      detail="外层超时 %.0fs，已放弃等待" % (timeout or config.CALL_TIMEOUT),
                      elapsed=elapsed, argv=argv)
    except FileNotFoundError as exc:
        elapsed = time.time() - t0
        return Result(action=action, rc=EXIT_REJECTED,
                      detail="找不到 CLI 或解释器: %s" % exc,
                      elapsed=elapsed, argv=argv)
    except Exception as exc:  # noqa: BLE001 - 任务层不该因单条原语崩溃
        elapsed = time.time() - t0
        return Result(action=action, rc=EXIT_NOT_REACHED,
                      detail="调用异常: %s" % exc, elapsed=elapsed, argv=argv)


def move(x: float, y: float, z: float, quat=None,
         tol_mm: Optional[float] = None, action: str = "move") -> Result:
    """笛卡尔移动 + 到位校验。

    CLI 里四元数是位置参数：pose x y z qx qy qz qw。
    """
    q = quat or config.DEFAULT_QUAT
    tol = tol_mm if tol_mm is not None else config.TOL_MM
    argv = ["pose", "%.4f" % x, "%.4f" % y, "%.4f" % z,
            "%.4f" % q[0], "%.4f" % q[1], "%.4f" % q[2], "%.4f" % q[3],
            "--wait", "25", "--tol", "%.1f" % tol]
    return call(argv, action)


def grip(semantic: str, action: Optional[str] = None) -> Result:
    """夹爪动作。semantic 只接受 close_fingers / open_fingers。"""
    if semantic not in config.GRIP_CMD:
        return Result(action=action or semantic, rc=EXIT_REJECTED,
                      detail="未知夹爪语义 %r，只接受 %s" % (semantic, list(config.GRIP_CMD)))
    cmd = config.GRIP_CMD[semantic]
    expect = config.GRIP_EXPECT[semantic]
    argv = ["grip", cmd, "--wait", "12", "--tol", "1.0"]
    res = call(argv, action or semantic)
    if res.ok:
        res.detail = "%s -> CLI '%s' -> joint7_left 期望 %.3f m；%s" % (
            semantic, cmd, expect, res.detail)
    return res


def grip_deg(deg: float, action: str = "grip_deg") -> Result:
    """把夹爪开到指定舵机角度（21.9~100）。

    `GripperCommand.msg` 只有 string，做不到连续开度；这条走 set_angle_topic。

    ⚠ `--tol` 必须收到 1.5 mm。默认 5 mm 时实测过：目标 48 mm、夹爪只走到 56.8 mm
    （误差 4.7 mm）就被判"到位"，而物体 50 mm —— 两指根本没碰到，夹空。
    依据：78.1° 走 57 mm，1° ≈ 0.73 mm；舵机死区 0.5~0.8° ≈ 0.4~0.6 mm。
    """
    argv = ["grip_deg", "%.1f" % deg, "--wait", "12", "--tol", "1.5"]
    return call(argv, action)


def gripper_state() -> Optional[float]:
    """读 joint7_left 实际位置（米）。0 = 两指合拢，-0.025 = 全开。"""
    res = call(["where"], "where", timeout=25)
    if not res.ok:
        return None
    m = re.search(r"joint7_left=(-?[\d.]+)", res.raw)
    return float(m.group(1)) if m else None


def where() -> Optional[tuple]:
    """读当前末端位姿 (x, y, z, qx, qy, qz, qw)。失败返回 None。"""
    res = call(["where"], "where", timeout=25)
    if not res.ok:
        return None
    m = re.search(r"x=(-?[\d.]+)\s+y=(-?[\d.]+)\s+z=(-?[\d.]+)", res.raw)
    if not m:
        return None
    xyz = tuple(float(g) for g in m.groups())
    q = re.search(r"q=\(([^)]+)\)", res.raw)
    if q:
        try:
            return xyz + tuple(float(v) for v in q.group(1).split(","))
        except ValueError:
            pass
    return xyz

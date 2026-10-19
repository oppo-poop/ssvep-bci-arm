"""取放任务逻辑。

一次取放 = 十个阶段。核心是「下探 → 合爪 → 校验 → 抬起」这四步，
其余是过渡和保护动作。

三个设计选择：

1. **阶段标记（Phase）不只是给日志看的。** 失败时要知道停在哪一类动作上，
   才能决定"重试"还是"放弃"。合爪失败可以重来，越界失败重来一万次也一样。

2. **抓取校验做成"合爪"这一步的一部分**，不拆成独立步骤。
   因为校验失败时要做的是「张开 → 退回接近点 → 重新下探 → 再合」，
   这是一段内聚的恢复逻辑，散成步骤反而难读。

3. **场景（Scene）从外面传进来**，不在本文件写死坐标。
   将来视觉或眼动只要产出同样的 Scene，这里一行都不用改。
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional, Tuple

from . import config, link, primitives
from .scene import Scene


class Phase(str, Enum):
    """动作所属的阶段。用于失败归因与日志。"""

    PREPARE = "准备"
    APPROACH = "接近"
    DESCEND = "下探"
    GRASP = "合爪"
    LIFT = "抬起"
    TRANSFER = "递送"
    PLACE = "放置"
    RELEASE = "释放"
    RETREAT = "撤离"
    DONE = "完成"


@dataclass
class Step:
    name: str
    phase: Phase
    fn: Callable[[], link.Result]


@dataclass
class RunResult:
    ok: bool
    steps: List[Tuple[Step, link.Result]] = field(default_factory=list)
    failed_at: Optional[str] = None
    failed_phase: Optional[Phase] = None
    recovered: bool = False

    @property
    def passed(self) -> int:
        return sum(1 for _, r in self.steps if r.ok)

    def summary(self) -> str:
        if self.ok:
            return "全部 %d 步通过" % len(self.steps)
        return "在「%s」（%s阶段）失败，%d/%d 步通过" % (
            self.failed_at,
            self.failed_phase.value if self.failed_phase else "?",
            self.passed, len(self.steps))


# ---------------------------------------------------------------- 执行器
def run(steps: List[Step]) -> RunResult:
    """顺序执行。任何一步失败即停，并按配置尝试回位。"""
    done: List[Tuple[Step, link.Result]] = []
    for step in steps:
        try:
            res = step.fn()
        except Exception as exc:  # noqa: BLE001
            # 代码异常不能绕过失败处理：否则臂停在半空，既不回位也不报警。
            # 把异常折成一条失败结果，走同一条「停 + 回位」的路。
            res = link.Result(action=step.name, rc=link.EXIT_NOT_REACHED,
                              detail="执行异常：%s: %s" % (type(exc).__name__, exc))
        done.append((step, res))
        if not res.ok:
            recovered = False
            if config.RETURN_HOME_ON_FAIL:
                # 停在半空比退回原点更危险。回位失败也不掩盖原始失败。
                recovered = primitives.go_home().ok
            return RunResult(ok=False, steps=done, failed_at=step.name,
                             failed_phase=step.phase, recovered=recovered)
    return RunResult(ok=True, steps=done)


# ---------------------------------------------------------------- 抓取
def grasp_with_retry(scene: Scene) -> Callable[[], link.Result]:
    """合爪 + 可选校验 + 夹空时撤回重试。返回一个可调用，供 Step 使用。"""

    def _do() -> link.Result:
        t0 = time.time()

        def _done(res: link.Result) -> link.Result:
            """统一填上本步的累计耗时（含重试）。"""
            res.elapsed = time.time() - t0
            return res

        # 按场景比例合拢：略小于物宽即可，合到底会让舵机持续堵转
        res = primitives.set_grip_frac(scene.grip_close, action="grasp")
        if not res.ok:
            return _done(res)

        if not scene.verify_grasp:
            res.detail = "未校验夹持（verify_grasp=False）"
            return _done(res)

        grabbed, pos, note = primitives.check_grasp(scene.object_width)
        if grabbed is not False:
            # True = 夹住了；None = 读不到状态，不阻断任务但要说清
            res.detail = note if pos is None else "%s（joint7_left=%.4f）" % (note, pos)
            return _done(res)

        # 判定夹空：张开 → 退回接近点 → 重新下探 → 再合。最多重试 N 次。
        for attempt in range(1, config.GRASP_MAX_RETRY + 1):
            primitives.set_grip_frac(scene.grip_open, action="release")
            primitives.approach(scene.pick, scene.standoff)
            primitives.descend(scene.pick, precise=True)
            retry = primitives.set_grip_frac(scene.grip_close, action="grasp")
            if not retry.ok:
                return _done(retry)
            grabbed, _pos, note2 = primitives.check_grasp(scene.object_width)
            if grabbed:
                retry.detail = "第 %d 次重试夹住：%s" % (attempt, note2)
                return _done(retry)
            note = note2

        return _done(link.Result(
            action="grasp", rc=link.EXIT_NOT_REACHED,
            detail="连续 %d 次未夹住（%s），放弃该目标"
                   % (config.GRASP_MAX_RETRY + 1, note)))


    return _do


# ---------------------------------------------------------------- 序列
def _pick_steps(sc: "Scene", with_home: bool = True) -> List[Step]:
    """取物段：回位 → 张开 → 接近 → 下探 → 合爪 → 抬起。

    结束在**抬起位**，不放置、不回位 —— 等下一步「放置」注入。
    """
    pick = sc.pick
    approach = sc.approach_point(pick)
    lift = (pick[0], pick[1], pick[2] + sc.lift_height)
    steps: List[Step] = []
    if with_home:
        steps.append(Step("回位待命", Phase.PREPARE, primitives.go_home))
    steps += [
        # 先张开到场景指定的开度：二值命令只有 65 / 8 mm 两档，
        # 下探时全开可能碰到旁边的物体，也不能随物体大小调整。
        Step("张开夹爪", Phase.PREPARE,
             lambda: primitives.set_grip_frac(sc.grip_open)),
        Step("接近物体", Phase.APPROACH,
             lambda: link.move(*approach, action="approach")),
        # 取物下探带二次定位补偿：落点从 ~11 mm 压到 2~3 mm，直接决定夹不夹得住
        Step("下探到位", Phase.DESCEND,
             lambda: primitives.descend(pick, precise=True)),
        Step("合爪夹持", Phase.GRASP, grasp_with_retry(sc)),
        Step("垂直抬起", Phase.LIFT,
             lambda: link.move(*lift, action="lift")),
    ]
    return steps


def _place_steps(sc: "Scene", with_home: bool = True) -> List[Step]:
    """放置段：递送 → 下探 → 松爪 → 撤离（→ 回位）。

    从**当前位姿**开始，所以它单独跑也成立（前一步应停在抬起位）。
    不检查"手上有没有东西"——那是操作者的责任，硬判会误伤。
    """
    place = sc.place
    transfer = sc.retreat_point(place)
    steps: List[Step] = [
        Step("平移递送", Phase.TRANSFER,
             lambda: link.move(*transfer, action="transfer")),
        Step("下探放置", Phase.PLACE,
             lambda: link.move(*place, action="descend")),
        Step("松爪释放", Phase.RELEASE, primitives.release),
        Step("撤离", Phase.RETREAT,
             lambda: link.move(*transfer, action="retreat")),
    ]
    if with_home:
        steps.append(Step("回位收尾", Phase.DONE, primitives.go_home))
    return steps


def build_pick(scene: Optional[Scene] = None) -> List[Step]:
    """只取不放：跑完停在抬起位，供之后单独下「放置」。"""
    return _pick_steps(scene or Scene.default())


def build_place(scene: Optional[Scene] = None) -> List[Step]:
    """只放不取：从当前位姿递送到放置点、松爪、回位。"""
    return _place_steps(scene or Scene.default())


def build_pick_place(scene: Optional[Scene] = None) -> List[Step]:
    """完整取放（取物段 + 放置段）。行为与拆分前一致。"""
    sc = scene or Scene.default()
    return _pick_steps(sc) + _place_steps(sc)

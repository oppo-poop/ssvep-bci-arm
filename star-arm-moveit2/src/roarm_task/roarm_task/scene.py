"""场景描述：这一次要抓什么、放到哪。

为什么要单独抽一层，而不是把坐标写进 sequence.py：

  视觉、眼动、终端手动输入，最终产出的都是"一个取物点 + 一个放置点"。
  把它们的共同形状固定成 Scene，将来换输入源时任务层一行都不用改。

注意：边界校验（工作区越界）**不在这里做**。那是 roarm_cli 的职责，
scene 只检查"逻辑上说不说得通"（高度为正、取放点不重合之类），
避免同一个判断出现两份实现。
"""

from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Tuple

Xyz = Tuple[float, float, float]

#: 夹爪行程（2026-10-18 卡尺实测）：grip 0 -> 8 mm，grip 1 -> 65 mm
GAP_MIN_MM, GAP_MAX_MM = 8.0, 65.0


@dataclass(frozen=True)
class Scene:
    """一次取放的全部输入。"""

    pick: Xyz                      # 末端取物点（世界系，米）
    place: Xyz                     # 末端放置点
    object_height: float = 0.05    # 物体高度，决定抬起量
    object_width: float = 0.03     # 物体宽度，决定夹爪该停在多开
    grip_open: float = 1.0         # 抓取前张开比例（0=8mm，1=65mm）
    grip_close: float = 0.7        # 合爪时合到多少比例（0=8mm，1=65mm）
    standoff: float = 0.08         # 接近/撤离的竖直偏移
    clearance: float = 0.06        # 抬起后离台面的净空
    verify_grasp: bool = False     # 是否校验"真的夹住了"
    name: str = "default"

    # ---------------------------------------------------------- 派生量
    @property
    def lift_height(self) -> float:
        """抬起量 = 物体高度 + 净空。"""
        return self.object_height + self.clearance

    def approach_point(self, target: Xyz) -> Xyz:
        return (target[0], target[1], target[2] + self.standoff)

    def retreat_point(self, target: Xyz) -> Xyz:
        return (target[0], target[1], target[2] + self.standoff)

    # ---------------------------------------------------------- 校验
    def problems(self) -> List[str]:
        """返回逻辑问题清单。空列表 = 通过。"""
        out = []
        if self.object_height <= 0:
            out.append("object_height 必须为正（物体有多高）")
        if self.object_width < 0:
            out.append("object_width 不能为负")
        if self.standoff <= 0:
            out.append("standoff 必须为正，否则会擦着物体平移")
        if self.clearance < 0:
            out.append("clearance 不能为负")
        dist = sum((a - b) ** 2 for a, b in zip(self.pick, self.place)) ** 0.5
        if dist < 0.01:
            out.append("取物点与放置点相距 %.1f mm，几乎重合" % (dist * 1000))
        # 抬升后必须比取物点高，否则"抬起"没有意义
        if self.lift_height <= self.standoff:
            out.append("抬起量 %.3f 不大于接近偏移 %.3f，抬起会低于接近点"
                       % (self.lift_height, self.standoff))
        return out

    def with_overrides(self, **kw) -> "Scene":
        clean = {k: v for k, v in kw.items() if v is not None}
        return replace(self, **clean) if clean else self

    # ---------------------------------------------------------- 载入
    @classmethod
    def default(cls) -> "Scene":
        """演示用默认场景：取物点 = README 路点 start 的最低点，放置点 = 工作区中部。

        ⚠ 这两个坐标是占位值，不是标定值。真机上必须先量一遍。
        """
        return cls(
            pick=(0.330, -0.324, 0.074),
            place=(0.400, 0.100, 0.200),
            name="default",
        )

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Scene":
        return cls(
            pick=tuple(d["pick"])[:3],
            place=tuple(d["place"])[:3],
            object_height=float(d.get("object_height", 0.05)),
            object_width=float(d.get("object_width", 0.03)),
            grip_open=float(d.get("grip_open", 1.0)),
            grip_close=float(d.get("grip_close", 0.7)),
            standoff=float(d.get("standoff", 0.08)),
            clearance=float(d.get("clearance", 0.06)),
            verify_grasp=bool(d.get("verify_grasp", False)),
            name=str(d.get("name", "file")),
        )

    @classmethod
    def from_yaml(cls, path: str) -> "Scene":
        import yaml  # 延迟导入：纯逻辑测试不需要它

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "pick": list(self.pick),
            "place": list(self.place),
            "object_height": self.object_height,
            "object_width": self.object_width,
            "grip_open": self.grip_open,
            "grip_close": self.grip_close,
            "standoff": self.standoff,
            "clearance": self.clearance,
            "verify_grasp": self.verify_grasp,
        }

    def describe(self) -> str:
        """场景摘要。开度同时给比例和毫米，方便和示教时的 grip 对得上。"""
        o_mm = GAP_MIN_MM + self.grip_open * (GAP_MAX_MM - GAP_MIN_MM)
        c_mm = GAP_MIN_MM + self.grip_close * (GAP_MAX_MM - GAP_MIN_MM)
        w_mm = self.object_width * 1000
        warn = ""
        if o_mm < w_mm + 2:
            warn += "\n  ⚠ 张开 %.0f mm 不大于物宽 %.0f mm，下探会顶到物体" % (o_mm, w_mm)
        if c_mm > w_mm:
            warn += "\n  ⚠ 合拢到 %.0f mm 仍大于物宽 %.0f mm —— 夹不到，会空合" % (c_mm, w_mm)
        elif w_mm - c_mm > 8:
            warn += ("\n  ⚠ 合拢到 %.0f mm 比物宽 %.0f mm 小 %.0f mm，"
                     "过盈偏大会顶对象或持续堵转" % (c_mm, w_mm, w_mm - c_mm))
        return ("场景「%s」  取物 (%.3f, %.3f, %.3f)  放置 (%.3f, %.3f, %.3f)\n"
                "  物高 %.0f mm  物宽 %.0f mm  张开 %.0f mm(grip %.2f)  "
                "合拢 %.0f mm(grip %.2f)\n"
                "  抬起 %.0f mm  接近偏移 %.0f mm  抓取校验 %s%s"
                % (self.name, *self.pick, *self.place,
                   self.object_height * 1000, w_mm,
                   o_mm, self.grip_open, c_mm, self.grip_close,
                   self.lift_height * 1000, self.standoff * 1000,
                   "开" if self.verify_grasp else "关", warn))

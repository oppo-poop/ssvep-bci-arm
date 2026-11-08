#!/usr/bin/env python3
# coding: utf-8
"""夹爪换算的**跨文件数值**一致性检查。

`grip_check.py` 只比对常量字面量；如果有人在 driver 里把公式改成线性、常量却
不动，它是看不出来的。本脚本用假 rclpy / 假舵机 SDK 把 `robo_driver` 直接
import 进来，逐点比较它和 `roarm_lib.params` 的结果。

为什么要假模块
    robo_driver 是 ROS 节点，import 就要 rclpy、robo_interfaces、serial 和
    fashionstar 的 SDK。离线环境（CI / 没有硬件）不该为了一个换算装这些东西。

检查内容
    1. 角度 → 间距 / joint：与 params 一致（含曲线，线性和曲线在此差距 >5 mm）
    2. joint → 角度：反算一致
    3. 端点：0° → 1.6 mm，-100° → 50.9 mm

用法
    python3 scripts/checks/grip_math_check.py
退出码 0 一致 / 1 不一致
"""

import importlib.util
import sys
import types
from pathlib import Path


def _root() -> Path:
    for base in Path(__file__).resolve().parents:
        if (base / "scripts" / "roarm_lib" / "params.py").is_file():
            return base
    raise SystemExit("找不到仓库根")


ROOT = _root()
sys.path.insert(0, str(ROOT / "scripts"))
from roarm_lib import params as P  # noqa: E402


def _stub(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


class _Node:                      # 只为满足 `class Arm_contorl(Node)`
    def __init__(self, *a, **k):
        pass


def _load_driver():
    _stub("rclpy", init=lambda *a, **k: None, shutdown=lambda: None,
          spin=lambda *a, **k: None, ok=lambda: True)
    _stub("rclpy.node", Node=_Node)
    _stub("robo_interfaces")
    _stub("robo_interfaces.srv", ReadData=object, WriteData=object)
    _stub("robo_interfaces.msg", SetAngle=object)
    _stub("sensor_msgs")
    _stub("sensor_msgs.msg", JointState=object)
    _stub("serial", SerialException=Exception, Serial=object, PARITY_NONE=0,
          PARITY_EVEN=0, STOPBITS_ONE=1)
    _stub("fashionstar_uart_sdk", UartServoManager=object,
          CODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL=0)
    path = ROOT / "star-arm-moveit2/src/robo_driver/robo_driver/robo_driver.py"
    spec = importlib.util.spec_from_file_location("_robo_driver_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    mod = _load_driver()
    S = mod.uservo_ex
    bad = []

    # 1) 角度 → 间距 / joint
    degs = [0.0, -10.0, -30.0, -50.0, -75.0, -97.6, -100.0]
    print("  %8s %9s %9s %11s %11s" % ("相对角", "驱动mm", "params", "驱动joint", "params"))
    for d in degs:
        g1, g2 = S.grip_gap_mm(d), P.gap_mm_from_servo(d)
        j1, j2 = S.grip_deg_to_joint(d), P.servo_to_joint(d)
        ok = abs(g1 - g2) < 1e-6 and abs(j1 - j2) < 1e-9
        if not ok:
            bad.append("角度 %.1f° 两侧不一致（mm %.3f/%.3f，joint %.5f/%.5f）"
                       % (d, g1, g2, j1, j2))
        print("  %8.1f %9.3f %9.3f %11.5f %11.5f %s"
              % (d, g1, g2, j1, j2, "✓" if ok else "✗"))

    # 2) joint → 角度
    for j in (-0.025, -0.018, -0.0125, -0.005, 0.0):
        a, b = S.grip_joint_to_deg(j), P.joint_to_servo(j)
        ok = abs(a - b) < 1e-6
        if not ok:
            bad.append("joint %+.4f 反算不一致（%.3f / %.3f）" % (j, a, b))
        print("  joint %+.4f → %8.3f° / %8.3f° %s" % (j, a, b, "✓" if ok else "✗"))

    # 3) 端点必须落在实测值上（曲线把中间拉弯，端点不能跟着跑）
    checks = [
        ("0° → 闭合间距", S.grip_gap_mm(0.0), P.GMIN),
        ("-100° → 全开间距", S.grip_gap_mm(-100.0), P.GMAX),
        ("0° → joint 下限", S.grip_deg_to_joint(0.0), P.JMIN),
        ("-100° → joint 上限", S.grip_deg_to_joint(-100.0), P.JMAX),
    ]
    for name, got, want in checks:
        ok = abs(got - want) < 1e-6
        if not ok:
            bad.append("%s：%.4f，应为 %.4f" % (name, got, want))
        print("  %-22s %.4f (期望 %.4f) %s" % (name, got, want, "✓" if ok else "✗"))

    # 4) 曲线不能退化成线性：半开必须明显小于线性中点
    mid = S.grip_gap_mm(P.CLOSE + P.SERVO_SPAN * 0.5)
    linear = (P.GMIN + P.GMAX) / 2.0
    ok = mid < linear - 5.0
    if not ok:
        bad.append("曲线失效：半开 %.1f mm ≈ 线性 %.1f mm" % (mid, linear))
    print("  半开 %.1f mm < 线性 %.1f mm - 5 %s" % (mid, linear, "✓" if ok else "✗"))

    if bad:
        print("\n✗ 有 %d 处不一致：" % len(bad))
        for b in bad:
            print("    - " + b)
        return 1
    print("\n✓ 驱动与 params 的夹爪换算逐点一致。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

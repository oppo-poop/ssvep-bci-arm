#!/usr/bin/env python3
# coding: utf-8
"""夹爪标定一致性自检。

真值只有一个：`config/robot.json`。
本脚本检查三类消费者：

  1. 可 import 的 Python 代码（roarm_task 的 config/primitives/scene）——
     直接读运行期常量，和 config 比对；
  2. 厂商/硬件节点（robo_driver、cello_controller、C++）——不从运行时读配置，
     用正则解析源码常量，和 config 比对；
  3. 源码结构——关键换算必须委托给 `roarm_lib.params`，不允许再出现旧公式
     （`(deg-100)/100*0.032`、`8 + ...*(57/0.025)` 等）。

公式本身的数值正确性由 `scripts/checks/traj_check.py` 执行验证（它会用假 rclpy
把 cello_controller / robo_driver import 进来实际调用）。

用法
    python3 scripts/checks/grip_check.py            # 人读
    python3 scripts/checks/grip_check.py --json     # 机器读

退出码
    0 全部一致    1 有不一致    2 配置/文件缺失
"""

import argparse
import json
import re
import sys
from pathlib import Path


def _scripts_dir() -> Path:
    for base in Path(__file__).resolve().parents:
        if (base / "roarm_lib" / "params.py").is_file():
            return base
    return Path(__file__).resolve().parent


SCRIPTS = _scripts_dir()
sys.path.insert(0, str(SCRIPTS))

from roarm_lib import params, paths  # noqa: E402

WS = paths.SRC
FILES = {
    "driver": WS / "robo_driver/robo_driver/robo_driver.py",
    "cpp": WS / "arm_moveit_write/src/arm_moveit_write.cpp",
    "controller": WS / "cello_controller/cello_controller/cello_controller.py",
    "bag": WS / "ros2_bag_recorder/src/bag_recorder.cpp",
    "read": WS / "arm_moveit_read/src/arm_moveit_read.cpp",
    "prim": WS / "roarm_task/roarm_task/primitives.py",
    "conf": WS / "roarm_task/roarm_task/config.py",
    "scene": WS / "roarm_task/roarm_task/scene.py",
    "cli": paths.CLI,
    "teach": SCRIPTS / "teach.py",
    "servo": SCRIPTS / "servo_check.py",
    "sim": SCRIPTS / "sim_gripper_bridge.py",
    "zero_calib": SCRIPTS / "calib" / "zero_calib.py",
    "go_zero": SCRIPTS / "calib" / "go_zero.py",
    "limit_probe": SCRIPTS / "calib" / "limit_probe.py",
}


def read(path: Path):
    if not path.is_file():
        return None
    return path.read_text(encoding="utf-8")


def grab_num(text, pattern):
    if text is None:
        return None
    m = re.search(pattern, text)
    return float(m.group(1)) if m else None


def grab_list(text, pattern):
    if text is None:
        return None
    m = re.search(pattern, text)
    if not m:
        return None
    try:
        return [float(x) for x in m.group(1).replace(" ", "").split(",") if x]
    except ValueError:
        return None


def same(a, b, tol=1e-6):
    return a is not None and b is not None and abs(a - b) <= tol


def add(checks, name, ok, detail=""):
    checks.append({"name": name, "ok": bool(ok), "detail": detail})


def check_params(checks):
    """派生换算的端点/往返/开度。"""
    p = params
    ok = (
        same(p.servo_to_joint(p.CLOSE), p.JMIN)
        and same(p.servo_to_joint(p.OPEN), p.JMAX)
        and same(p.joint_to_servo(p.JMIN), p.CLOSE)
        and same(p.joint_to_servo(p.JMAX), p.OPEN)
        and same(p.gap_mm_from_joint(p.JMIN), p.GMIN)
        and same(p.gap_mm_from_joint(p.JMAX), p.GMAX)
        and same(p.gap_mm_from_servo(p.CLOSE), p.GMIN)
        and same(p.gap_mm_from_servo(p.OPEN), p.GMAX)
        and same(p.frac_to_servo(0.0), p.CLOSE)
        and same(p.frac_to_servo(1.0), p.OPEN)
        and p.GRIP_EXPECT_CLI["open"] == p.JMAX
        and p.GRIP_EXPECT_CLI["close"] == p.JMIN
        and p.GRIP_EXPECT["close_fingers"] == p.JMIN
        and p.GRIP_EXPECT["open_fingers"] == p.JMAX
    )
    add(checks, "params 派生量", ok,
        "端点/往返/比例换算" if ok else "端点映射不是线性往返")


def check_task_modules(checks):
    """roarm_task 是纯 Python，可以直接 import 后比较运行期值。"""
    sys.path.insert(0, str(WS / "roarm_task"))
    try:
        from roarm_task import config as task_config
        from roarm_task import primitives, scene
    except Exception as exc:  # noqa: BLE001
        add(checks, "roarm_task 导入", False, "%s: %s" % (type(exc).__name__, exc))
        return

    cmin = getattr(task_config, "GRIP_DEG_MIN", None)
    cmax = getattr(task_config, "GRIP_DEG_MAX", None)
    add(checks, "roarm_task/config 端点",
        same(cmin, params.CLOSE) and same(cmax, params.OPEN),
        "%.1f/%.1f" % (cmin, cmax) if cmin is not None else "未定义 GRIP_DEG_MIN/MAX")
    add(checks, "roarm_task/config GRIP_EXPECT",
        same(task_config.GRIP_EXPECT["close_fingers"], params.JMIN)
        and same(task_config.GRIP_EXPECT["open_fingers"], params.JMAX),
        str(task_config.GRIP_EXPECT))
    add(checks, "roarm_task/primitives 端点",
        same(primitives.GRIP_DEG_MIN, params.CLOSE)
        and same(primitives.GRIP_DEG_MAX, params.OPEN),
        "%.1f/%.1f" % (primitives.GRIP_DEG_MIN, primitives.GRIP_DEG_MAX))
    add(checks, "roarm_task/primitives 开度",
        same(primitives.GAP_MIN_MM, params.GMIN)
        and same(primitives.GAP_MAX_MM, params.GMAX)
        and same(primitives.GAP_ZERO_POS, params.JMIN),
        "%.1f~%.1f mm, joint %+.4f" % (
            primitives.GAP_MIN_MM, primitives.GAP_MAX_MM, primitives.GAP_ZERO_POS))
    add(checks, "roarm_task/scene 开度",
        same(scene.GAP_MIN_MM, params.GMIN)
        and same(scene.GAP_MAX_MM, params.GMAX),
        "%.1f~%.1f mm" % (scene.GAP_MIN_MM, scene.GAP_MAX_MM))
    add(checks, "roarm_task home/四元数",
        tuple(task_config.HOME_XYZ) == tuple(params.HOME_XYZ)
        and tuple(task_config.DEFAULT_QUAT) == tuple(params.DEFAULT_QUAT),
        "home=%s" % (task_config.HOME_XYZ,))


def check_vendor_constants(checks, src):
    p = params
    d = src["driver"]
    add(checks, "robo_driver 端点",
        same(grab_num(d, r"GRIP_DEG_CLOSE\s*=\s*([\d.]+)"), p.CLOSE)
        and same(grab_num(d, r"GRIP_DEG_OPEN\s*=\s*([\d.]+)"), p.OPEN),
        "close=%s open=%s" % (
            grab_num(d, r"GRIP_DEG_CLOSE\s*=\s*([\d.]+)"),
            grab_num(d, r"GRIP_DEG_OPEN\s*=\s*([\d.]+)")))

    start = grab_num(d, r"GRIP_START_DEG\s*=\s*([\d.]+)")
    if start is None:
        raw = grab_num(d, r"ZERO_ANGLE\.append\((\d+)\)")
        start = raw / 10.0 if raw is not None else None
    add(checks, "robo_driver 启动位", same(start, p.START_DEG),
        "%.1f°" % start if start is not None else "找不到启动位")

    zero = grab_list(d, r"ZERO_OFFSET_DEG\s*=\s*\[([^\]]+)\]")
    add(checks, "robo_driver 零位偏移", zero == p.ZERO_OFFSET_DEG,
        str(zero))

    cpp = src["cpp"]
    # 这两个名字是历史反的：kServoClosed 实际是"张开"，看值不看名
    add(checks, "arm_moveit_write.cpp 端点",
        same(grab_num(cpp, r"kServoOpened\s*=\s*([\d.]+)"), p.CLOSE)
        and same(grab_num(cpp, r"kServoClosed\s*=\s*([\d.]+)"), p.OPEN),
        "close=%s open=%s" % (
            grab_num(cpp, r"kServoOpened\s*=\s*([\d.]+)"),
            grab_num(cpp, r"kServoClosed\s*=\s*([\d.]+)")))

    ctrl = src["controller"]
    add(checks, "cello_controller 端点常量",
        same(grab_num(ctrl, r"GRIP_DEG_CLOSE\s*=\s*([\d.]+)"), p.CLOSE)
        and same(grab_num(ctrl, r"GRIP_DEG_OPEN\s*=\s*([\d.]+)"), p.OPEN)
        and same(grab_num(ctrl, r"GRIP_JOINT_MIN\s*=\s*(-?[\d.]+)"), p.JMIN)
        and same(grab_num(ctrl, r"GRIP_JOINT_MAX\s*=\s*(-?[\d.]+)"), p.JMAX),
        "stage3 已补：joint7 换算用实测端点")

    bag = src["bag"]
    add(checks, "bag_recorder 端点",
        same(grab_num(bag, r"kServoClose\s*=\s*([\d.]+)"), p.CLOSE)
        and same(grab_num(bag, r"kServoOpen\s*=\s*([\d.]+)"), p.OPEN),
        "kServoClose=%s kServoOpen=%s" % (
            grab_num(bag, r"kServoClose\s*=\s*([\d.]+)"),
            grab_num(bag, r"kServoOpen\s*=\s*([\d.]+)")))

    rd = src["read"]
    add(checks, "arm_moveit_read 半开阈值",
        rd is not None and ">= 80" not in rd and "> 80" not in rd,
        "仍是旧 80° 阈值" if rd and (">= 80" in rd or "> 80" in rd) else "已按 joint 值判定")


def check_source_structure(checks, src):
    """关键换算必须委托 params；旧公式/旧字面量不允许再出现。"""
    forbidden = [
        ("roarm_cli 旧 grip_deg 公式", "cli", r"\(\s*deg\s*-\s*100"),
        ("roarm_cli 旧开度 57mm", "cli", r"57\.0\s*/\s*0\.025"),
        ("sim_bridge 旧公式", "sim", r"\(\s*deg\s*-\s*100"),
        ("teach 旧 GAP_MIN=8", "teach", r"GAP_MIN_MM[^=\n]*=\s*8\.0"),
        ("servo_check 旧端点字面量", "servo", r"GRIP_DEG_CLOSE\s*=\s*86\.7"),
        ("cello_controller 旧 0.032 公式", "controller", r"/\s*0\.032"),
        ("bag_recorder 旧 -10 修正", "bag", r"angle\s*-=\s*10"),
    ]
    for name, key, pattern in forbidden:
        text = src[key]
        hit = text is not None and re.search(pattern, text)
        add(checks, name, not hit,
            "仍存在 %s" % pattern if hit else "ok")

    required = [
        ("roarm_cli 使用 params", "cli", r"servo_to_joint\s*\("),
        ("roarm_cli 开度使用 params", "cli", r"gap_mm_from_servo\s*\("),
        ("sim_bridge 使用 params", "sim", r"servo_to_joint\s*\("),
        ("teach 使用 params 比例", "teach", r"frac_to_servo\s*\("),
        ("servo_check 使用 params", "servo", r"\bparams\."),
        ("go_zero 零位来自 params", "go_zero", r"params\.ZERO_OFFSET_DEG"),
        ("limit_probe 零位来自 params", "limit_probe", r"params\.ZERO_OFFSET_DEG"),
        ("zero_calib 端点来自 params", "zero_calib", r"params\.CLOSE"),
    ]
    for name, key, pattern in required:
        text = src[key]
        ok = text is not None and re.search(pattern, text) is not None
        add(checks, name, ok, "未找到 %s" % pattern if not ok else "ok")


def main():
    ap = argparse.ArgumentParser(description="夹爪标定一致性自检")
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    args = ap.parse_args()

    try:
        _ = params.CLOSE
    except Exception as exc:  # noqa: BLE001
        print("✗ 读取 config/robot.json 失败：%s" % exc, file=sys.stderr)
        return 2

    src = {k: read(p) for k, p in FILES.items()}
    missing = [str(p) for k, p in FILES.items() if src[k] is None]
    if missing:
        print("✗ 找不到这些文件：", file=sys.stderr)
        for p in missing:
            print("    " + p, file=sys.stderr)
        return 2

    checks = []
    check_params(checks)
    check_task_modules(checks)
    check_vendor_constants(checks, src)
    check_source_structure(checks, src)

    bad = [c for c in checks if not c["ok"]]
    out = {
        "close_deg": params.CLOSE,
        "open_deg": params.OPEN,
        "span_deg": round(params.SERVO_SPAN, 2),
        "start_deg": params.START_DEG,
        "start_gap_mm": round(params.gap_mm_from_servo(params.START_DEG), 1),
        "gap_min_mm": params.GMIN,
        "gap_max_mm": params.GMAX,
        "checks": checks,
        "problems": ["%s：%s" % (c["name"], c["detail"]) for c in bad],
        "ok": not bad,
    }

    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0 if not bad else 1

    print("  真值 config/robot.json：%.1f° ~ %.1f°  |  joint %+.3f ~ %+.3f m  |  %.0f ~ %.0f mm"
          % (params.CLOSE, params.OPEN, params.JMIN, params.JMAX,
             params.GMIN, params.GMAX))
    print("  启动位 %.1f° → 约 %.1f mm" % (
        params.START_DEG, params.gap_mm_from_servo(params.START_DEG)))
    print()
    print("  %-32s %-4s %s" % ("检查项", "结果", "说明"))
    print("  " + "-" * 78)
    for c in checks:
        print("  %-32s %-4s %s" % (c["name"], "✓" if c["ok"] else "✗", c["detail"]))
    print()
    if bad:
        print("  ✗ %d 项不一致。改法：先改 config/robot.json，再同步厂商常量/源码。" % len(bad))
        return 1
    print("  ✓ 全部一致。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
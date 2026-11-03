#!/usr/bin/env python3
# coding: utf-8
"""夹爪标定一致性自检：一处改动，六个地方必须同时对上。

为什么需要它
    夹爪的"舵机角度 ↔ joint7_left"这条链被硬编码在 6 个文件里：
        robo_driver.py            GRIP_DEG_CLOSE / GRIP_DEG_OPEN、启动位
        arm_moveit_write.cpp      kServoClosed / kServoOpened
        roarm_task/primitives.py  GRIP_DEG_MIN / GRIP_DEG_MAX
        roarm_task/config.py      GRIP_EXPECT
        scripts/roarm_cli.py      GRIP_DEG_MIN / GRIP_DEG_MAX、GRIP_EXPECT
        scripts/teach.py          GRIP_CLOSED_DEG / GRIP_OPEN_DEG
    拆装一次夹爪，六处全要改。漏一处就出怪现象：
        · 漏 cpp      → grip close 发旧角度，舵机顶死、发热
        · 漏启动位     → 一上电就把爪子顶到底
        · 漏 GRIP_EXPECT → close 永远报"未到位"
    2026-11-03 就是"改了五处、漏了启动位"，上电堵转。

用法
    python3 scripts/grip_check.py            # 人读
    python3 scripts/grip_check.py --json     # 机器读

退出码
    0 全部一致    1 有不一致    2 文件找不到
"""

import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.join(ROOT, "star-arm-moveit2", "src")

DRIVER = os.path.join(WS, "robo_driver/robo_driver/robo_driver.py")
CPP = os.path.join(WS, "arm_moveit_write/src/arm_moveit_write.cpp")
PRIM = os.path.join(WS, "roarm_task/roarm_task/primitives.py")
CONF = os.path.join(WS, "roarm_task/roarm_task/config.py")
CLI = os.path.join(ROOT, "scripts", "roarm_cli.py")
TEACH = os.path.join(ROOT, "scripts", "teach.py")

#: joint7 的 URDF 限位（不是夹爪行程，是关节坐标范围）
URDF_LOW, URDF_HIGH = -0.025, 0.0


def read(path):
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        return f.read()


def num(text, pattern, what):
    """从源码里抓一个数。抓不到返回 (None, 说明)。"""
    if text is None:
        return None, "%s：文件不存在" % what
    m = re.search(pattern, text)
    if not m:
        return None, "%s：正则没匹配到" % what
    try:
        return float(m.group(1)), None
    except (TypeError, ValueError):
        return None, "%s：抓到的东西不是数" % what


def main():
    ap = argparse.ArgumentParser(description="夹爪标定一致性自检")
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    args = ap.parse_args()

    src = {p: read(p) for p in (DRIVER, CPP, PRIM, CONF, CLI, TEACH)}

    # ---- 抓值 ----
    close_d, e1 = num(src[DRIVER], r"GRIP_DEG_CLOSE\s*=\s*([\d.]+)", "robo_driver 闭合角")
    open_d, e2 = num(src[DRIVER], r"GRIP_DEG_OPEN\s*=\s*([\d.]+)", "robo_driver 张开角")
    start_d, e3 = num(src[DRIVER], r"ZERO_ANGLE\.append\((\d+)\)", "robo_driver 启动位(0.1°)")
    cpp_open, e4 = num(src[CPP], r"kServoOpened\s*=\s*([\d.]+)", "cpp 闭合角")
    cpp_close, e5 = num(src[CPP], r"kServoClosed\s*=\s*([\d.]+)", "cpp 张开角")
    prim_min, e6 = num(src[PRIM], r"GRIP_DEG_MIN,\s*GRIP_DEG_MAX\s*=\s*([\d.]+)", "primitives 下限")
    prim_max, e7 = num(src[PRIM], r"GRIP_DEG_MIN,\s*GRIP_DEG_MAX\s*=\s*[\d.]+,\s*([\d.]+)", "primitives 上限")
    cli_min, e8 = num(src[CLI], r"GRIP_DEG_MIN,\s*GRIP_DEG_MAX\s*=\s*([\d.]+)", "roarm_cli 下限")
    cli_max, e9 = num(src[CLI], r"GRIP_DEG_MIN,\s*GRIP_DEG_MAX\s*=\s*[\d.]+,\s*([\d.]+)", "roarm_cli 上限")
    t_close, e10 = num(src[TEACH], r"GRIP_CLOSED_DEG,\s*GRIP_OPEN_DEG\s*=\s*([\d.]+)", "teach 闭合角")
    t_open, e11 = num(src[TEACH], r"GRIP_CLOSED_DEG,\s*GRIP_OPEN_DEG\s*=\s*[\d.]+,\s*([\d.]+)", "teach 张开角")
    ge_conf, e12 = num(src[CONF], r'"close_fingers":\s*(-?[\d.]+)', "config GRIP_EXPECT close")
    ge_cli, e13 = num(src[CLI], r'"close":\s*(-?[\d.]+)', "roarm_cli GRIP_EXPECT close")
    gap_lo, e14 = num(src[PRIM], r"GAP_MIN_MM\s*=\s*([\d.]+)", "primitives GAP_MIN")
    gap_hi, e15 = num(src[PRIM], r"GAP_MAX_MM\s*=\s*([\d.]+)", "primitives GAP_MAX")

    errors = [e for e in (e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12, e13, e14, e15) if e]
    if errors:
        print("✗ 抓不到这些值：", file=sys.stderr)
        for e in errors:
            print("    " + e, file=sys.stderr)
        print("\n  正则失效通常意味着代码结构变了，这个脚本要跟着改。", file=sys.stderr)
        return 2

    start_deg = start_d / 10.0

    # ---- 比对 ----
    rows = [
        ("robo_driver", close_d, open_d),
        ("arm_moveit_write.cpp", cpp_open, cpp_close),
        ("primitives", prim_min, prim_max),
        ("roarm_cli", cli_min, cli_max),
        ("teach", t_close, t_open),
    ]
    bad = []
    for name, c, o in rows:
        if abs(c - close_d) > 0.01 or abs(o - open_d) > 0.01:
            bad.append("%s 的端点 (%.1f, %.1f) 和 robo_driver (%.1f, %.1f) 不一致"
                       % (name, c, o, close_d, open_d))

    if not (close_d <= start_deg <= open_d):
        bad.append("启动位 %.1f° 不在 [%.1f, %.1f] 内 —— 上电会顶死舵机"
                   % (start_deg, close_d, open_d))
    if close_d >= open_d:
        bad.append("闭合角 %.1f° 不小于张开角 %.1f° —— 端点写反了" % (close_d, open_d))

    # joint 坐标必须覆盖 URDF 限位（否则 URDF 要改，或换算错了）
    span = open_d - close_d
    if span <= 0:
        bad.append("行程为 0")
    # 端点映射应为 close→URDF_LOW、open→URDF_HIGH
    for ge_name, ge in (("config", ge_conf), ("roarm_cli", ge_cli)):
        if abs(ge - URDF_LOW) > 1e-4:
            bad.append("%s 的 GRIP_EXPECT close = %+.4f，应为 %.3f"
                       % (ge_name, ge, URDF_LOW))

    gap_span = gap_hi - gap_lo
    if gap_span <= 0:
        bad.append("GAP_MAX(%.1f) 不大于 GAP_MIN(%.1f)" % (gap_hi, gap_lo))

    # 启动位对应开度（线性换算，仅供人看）
    def gap_of(deg):
        pos = (deg - open_d) * ((URDF_HIGH - URDF_LOW) / span)
        ratio = (pos - URDF_LOW) / (URDF_HIGH - URDF_LOW)
        return gap_lo + ratio * gap_span

    out = {
        "close_deg": close_d, "open_deg": open_d, "span_deg": round(span, 2),
        "start_deg": start_deg, "start_gap_mm": round(gap_of(start_deg), 1),
        "gap_min_mm": gap_lo, "gap_max_mm": gap_hi,
        "modules": [{"name": n, "close": c, "open": o} for n, c, o in rows],
        "problems": bad,
        "ok": not bad,
    }

    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0 if not bad else 1

    print("  %-24s %-9s %-9s" % ("模块", "闭合角", "张开角"))
    print("  " + "-" * 46)
    for name, c, o in rows:
        flag = "✓" if abs(c - close_d) <= 0.01 and abs(o - open_d) <= 0.01 else "✗"
        print("  %-24s %-9.1f %-9.1f %s" % (name, c, o, flag))
    print()
    print("  行程 %.1f°  开度 %.1f ~ %.1f mm" % (span, gap_lo, gap_hi))
    print("  启动位 %.1f° → 约 %.1f mm" % (start_deg, gap_of(start_deg)))
    print()

    if bad:
        print("  ✗ %d 处不一致：" % len(bad))
        for b in bad:
            print("    · " + b)
        print()
        print("  改法：以 robo_driver.GRIP_DEG_CLOSE/OPEN 为准，其余文件同步；")
        print("        arm_moveit_write.cpp 改完要 colcon build（C++ 不重编不生效）。")
        return 1

    print("  ✓ 全部一致。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

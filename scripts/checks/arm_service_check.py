#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""8 指令服务的离线自检：不连机械臂、不起 ROS、不开相机。

    python3 scripts/checks/arm_service_check.py
"""

import os
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

# 自检不写 timing.jsonl，免得零耗时记录混进真机基线。
os.environ["ROARM_TIMING"] = "0"

import arm_service as svc  # noqa: E402

fails = []
calls = []


def check(cond, label):
    print("  %s %s" % ("✓" if cond else "✗", label))
    if not cond:
        fails.append(label)


def make_controller():
    c = svc.Controller(tray_file="/nonexistent/trays.json")
    c.trays = {"red": (640, 520), "green": (760, 520), "blue": (880, 520)}
    c.refresh_trays = lambda log=False: None

    def fake_run(cmd, timeout):
        calls.append(("run", cmd))
        return 0, ""

    def fake_ros(args, timeout):
        calls.append(("ros", list(args)))
        return 0, ""

    c._run = fake_run
    c._ros = fake_ros
    return c


def main():
    print("[1] 指令表 0~7")
    for k in range(8):
        got = svc.parse_request(str(k))
        check(got[0] == k, "k=%d -> %s" % (k, svc.LABELS[k]))
    check(svc.parse_request("pick red") == (0, "pick", "red"), "文本 pick red")
    check(svc.parse_request("place blue") == (5, "place", "blue"), "文本 place blue")
    check(svc.parse_request("home") == (6, "home", None), "文本 home")
    check(svc.parse_request("stop") == (7, "stop", None), "文本 stop")

    print("[2] 取 → 放 → 状态")
    c = make_controller()
    r = c.execute("0")
    check(r["state"] == "ok" and r["holding"] == "red", "取红成功，holding=red")
    r = c.execute("place green")
    check(r["state"] == "ok" and r["holding"] is None, "放绿成功，holding 清空")
    check(any(c[0] == "run" and "--place-only" in c[1] for c in calls),
          "place 走 pick.py --place-only")
    r = c.execute("place blue")
    check(r["state"] == "no_object", "手上没物料时 place 返回 no_object")

    print("[3] 忙时丢弃 / STOP 例外")
    c = make_controller()
    c.busy = True
    r = c.execute("1")
    check(r["state"] == "busy", "忙时 pick 返回 busy")
    r = c.execute("7")
    check(r["state"] == "stopped", "忙时 stop 仍然执行")
    check(not c.busy, "stop 后 busy 清空")
    check(any(c[0] == "ros" and "home" in c[1] for c in calls),
          "stop 调 roarm_cli home 回零位")

    print("[4] 回位 = 收纳 park")
    c = make_controller()
    calls.clear()
    r = c.execute("home")
    check(r["state"] == "ok", "home 成功")
    check(any(c[0] == "ros" and c[1] == ["park"] for c in calls),
          "home 调 roarm_cli park")

    print()
    if fails:
        print("✗ %d 项不通过：" % len(fails))
        for f in fails:
            print("  - " + f)
        return 1
    print("✓ 全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
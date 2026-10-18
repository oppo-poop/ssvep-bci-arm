#!/usr/bin/env python3
# coding: utf-8
"""零位标定：只读测量舵机角度，建立「机械姿态 → 舵机读数」对照表。

用途
    验证 robo_driver 的那个隐含假设：**舵机 0° == URDF 零位**。
    cello_controller 里 `radians_to_degrees(x) = x*180/pi` 没有任何零位偏移，
    一旦假设不成立，整条链会系统性偏，而且偏得越多飞得越远。

这个脚本能做的
    · 读 7 个舵机的实时角度 + 多圈角度
    · 把若干姿态连同你的文字标注记成 CSV
    · 算出每个关节相对「机械零位」那一行的偏差 → 建议的零点修正值
    · 可选解除扭矩（`--free`），让你能徒手摆臂

这个脚本绝不做的
    · 不发任何运动指令（不调 set_angle / send_sync_*）
    · 不设零点（不调 set_origin_point）—— 那个动作一旦错，整个臂的读数全废
    · 不启动 robo_driver —— 它 __init__ 里就会 reset_multi_turn_angle + 驱到 ZERO_ANGLE

依赖
    pip install fashionstar-uart-sdk

用法
    # 1. 先只读一次，确认能连上、看当前读数
    python3 scripts/zero_calib.py

    # 2. 解除扭矩，徒手把臂摆到机械零位，记录为 baseline（先托住臂！）
    python3 scripts/zero_calib.py --free --pose 机械零位

    # 3. 摆到其他姿态各记一次，最后看偏差汇总
    python3 scripts/zero_calib.py --pose 直立 --pose 全部收起
    python3 scripts/zero_calib.py --summary
"""

import argparse
import csv
import os
import statistics
import sys
import time
from pathlib import Path

SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4",
               "joint5", "joint6", "joint7_left"]

#: 参与零位判据的关节。joint7 是夹爪开合，不属于"关节零位"，排除在外。
POSITION_JOINTS = [0, 1, 2, 3, 4, 5]

#: 各关节的「有效零位」舵机读数。
#: 前六个 robo_driver 直接当角度用（0° 即零位）；
#: joint7_left 的换算公式是 (deg-100)/100*0.032，所以它的零位在 100°。
EXPECTED_ZERO = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 100.0}

# robo_driver 用的是 1000000，官方例程写的是 115200 —— 两个都试
DEFAULT_PORT = "/dev/ttyUSB0"
CANDIDATE_BAUDS = [1000000, 115200]

#: 标定数据放项目根 data/ 下（2026-10-18 从 scripts/ 移出，运行产物不该混在代码里）
CSV_PATH = Path(__file__).resolve().parent.parent / "data" / "zero_calib.csv"


def die(msg, code=2):
    print("✗ " + msg, file=sys.stderr)
    sys.exit(code)


def load_sdk():
    try:
        import fashionstar_uart_sdk as uservo  # noqa: F401
        return uservo
    except ImportError:
        die("缺少舵机 SDK。先装：pip install fashionstar-uart-sdk\n"
            "  （不要复用 robo_driver 里的 uservo_ex：它初始化就断电复位）")


def open_manager(uservo, port, baud=None):
    """打开串口并建立 UartServoManager。baud=None 时自动探测。"""
    import serial

    if not os.path.exists(port):
        die("串口不存在：%s\n  插上 UC-01 后先确认设备名（ls /dev/ttyUSB*）" % port)

    bauds = [baud] if baud else CANDIDATE_BAUDS
    last_err = None
    for b in bauds:
        try:
            uart = serial.Serial(port=port, baudrate=b, parity=serial.PARITY_NONE,
                                 stopbits=1, bytesize=8, timeout=0.2)
            mgr = uservo.UartServoManager(uart)
            # 探测：能读到任意一个舵机就说明这个波特率对
            for sid in SERVO_IDS:
                try:
                    if mgr.query_servo_angle(sid) is not None:
                        print("✓ 串口 %s @ %d baud，舵机 %d 有响应" % (port, b, sid))
                        return mgr, b, uart
                except Exception as exc:  # noqa: BLE001
                    last_err = exc
            uart.close()
        except Exception as exc:  # noqa: BLE001
            last_err = exc
    die("打不开或没有舵机响应（试过 %s）。最后一个错误：%s" % (bauds, last_err))


def read_all(mgr, samples=1, delay=0.05):
    """读一轮全部舵机，多次采样取中位数。返回 [(id, angle, mturn)]。"""
    rows = []
    for sid in SERVO_IDS:
        angles, mturns = [], []
        for _ in range(max(1, samples)):
            try:
                a = mgr.query_servo_angle(sid)
                m = mgr.query_servo_mturn_angle(sid)
                if a is not None:
                    angles.append(float(a))
                if m is not None:
                    mturns.append(float(m))
            except Exception:  # noqa: BLE001
                pass
            time.sleep(delay)
        ang = statistics.median(angles) if angles else None
        mt = statistics.median(mturns) if mturns else None
        rows.append((sid, ang, mt))
    return rows


def fmt(v, width=8, prec=2):
    return ("%*.{}f".format(prec) % (width, v)) if v is not None else " " * (width - 1) + "-"


def print_table(rows, title=""):
    if title:
        print("\n" + title)
    print("  ID  关节          角度(°)   多圈角(°)")
    print("  " + "-" * 44)
    for sid, ang, mt in rows:
        print("  %-3d %-12s %s  %s"
              % (sid, JOINT_NAMES[sid], fmt(ang), fmt(mt)))
    missing = [sid for sid, ang, _ in rows if ang is None]
    if missing:
        print("  ⚠ 无读数：%s（检查总线/ID/供电）" % missing)


def append_csv(label, rows, baud):
    new = not CSV_PATH.exists()
    if not new:
        try:
            dup = sum(1 for r in load_csv() if r.get("label") == label)
            if dup:
                print("  ⚠ CSV 里已有 %d 条「%s」。同名记录在 summary 里分不清，"
                      "建议换个标签。" % (dup, label))
        except Exception:  # noqa: BLE001
            pass
    with open(CSV_PATH, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "label", "baud"]
                       + ["id%d_angle" % i for i in SERVO_IDS]
                       + ["id%d_mturn" % i for i in SERVO_IDS])
        w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), label, baud]
                   + ["" if a is None else "%.3f" % a for _, a, _ in rows]
                   + ["" if m is None else "%.3f" % m for _, _, m in rows])
    print("  → 已追加到 %s" % CSV_PATH)


def load_csv():
    if not CSV_PATH.exists():
        die("还没有记录：%s\n  先跑 --pose <标签> 记一条" % CSV_PATH)
    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def cmd_summary(baseline_label=None):
    rows = load_csv()
    if len(rows) < 2:
        die("至少需要两条记录才能对比（现在 %d 条）" % len(rows))

    labels = [r["label"] for r in rows]
    dup = sorted({l for l in labels if labels.count(l) > 1})
    if dup:
        print("⚠ CSV 里有重复标签：%s" % "、".join(dup))
        print("  同名记录会都出现在表里，容易看错行。要清理就编辑 %s" % CSV_PATH)
        print()
    base = None
    if baseline_label:
        for r in rows:
            if r["label"] == baseline_label:
                base = r
                break
        if base is None:
            die("找不到标签 %r。现有标签：%s" % (baseline_label, labels))
    else:
        base = rows[0]

    print("基线：%s（%s）" % (base["label"], base["timestamp"]))
    print("\n相对基线的角度偏差（度）：")
    print("  姿态                " + "".join("%9s" % n.replace("joint", "j")
                                             for n in JOINT_NAMES))
    print("  " + "-" * (20 + 9 * 7))
    for r in rows:
        if r is base:
            continue
        cells = []
        for i in SERVO_IDS:
            a, b = r.get("id%d_angle" % i), base.get("id%d_angle" % i)
            cells.append("%9s" % ("%.2f" % (float(a) - float(b))
                                  if a and b else "        -"))
        print("  %-18s" % r["label"] + "".join(cells))

    print("\n「%s」这一行的原始读数（相对各自有效零位的偏差）：" % base["label"])
    dev = []
    for i in POSITION_JOINTS:
        v = base.get("id%d_angle" % i)
        if not v:
            continue
        delta = float(v) - EXPECTED_ZERO[i]
        if abs(delta) >= 0.5:
            dev.append("%s %+7.2f" % (JOINT_NAMES[i], delta))
    if dev:
        print("  " + "   ".join(dev))
        print("\n  → 这些关节在机械零位上读数偏离了，把上面这些数当 zero_offset：")
        print("     robo_driver 发布 joint_states 时：q = (deg - offset) * pi/180")
        print("     cello_controller 下发角度时：      deg = q * 180/pi + offset")
        print("     两处都要改，只改一边会让读数与指令不一致。")
    else:
        print("  所有位置关节都落在零位上 → 「舵机 0° == URDF 零位」这个假设成立。")

    g = base.get("id6_angle")
    if g:
        print("\n  joint7_left 读数 %.1f°（不参与零位判据：它是夹爪开合，不是关节零位）。"
              % float(g))


def ask(prompt):
    """input 的包装：没有 TTY 时给出可操作的提示，而不是抛栈。"""
    try:
        return input(prompt)
    except EOFError:
        die("这一步需要交互确认，但当前没有终端输入。\n"
            "  接在脚本里跑时请加 --yes，并用 --pose 一次传完标签。")


def load_fk():
    """加载同目录的 fk.py（不是包，用 importlib 显式加载，避免污染 sys.path）。"""
    import importlib.util

    path = Path(__file__).resolve().parent / "fk.py"
    if not path.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location("_roarm_fk", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:  # noqa: BLE001
        return None


def tip_of(rows):
    """用 FK 把读数翻成末端位置 (x, y, z)（米）。失败返回 None。"""
    fk = load_fk()
    if fk is None:
        return None
    angs = [a for _, a, _ in rows[:6]]
    if any(a is None for a in angs):
        return None
    try:
        urdf = fk.parse_urdf(str(fk.DEFAULT_URDF))
        return fk.fk(urdf, angs)[-1][1]
    except Exception:  # noqa: BLE001
        return None


def report_tip(rows):
    """把读数翻成末端位置，好让人当场判断「摆对了没有」。"""
    tip = tip_of(rows)
    if tip is None:
        return
    # URDF 零位：前方 479.6 mm，侧向 0，高 370.5 mm
    dx = tip[0] * 1000 - 479.6
    dy = tip[1] * 1000
    dz = tip[2] * 1000 - 370.5
    print("  FK 末端：前方 %.1f mm，侧向 %+.1f mm，高 %.1f mm"
          % (tip[0] * 1000, tip[1] * 1000, tip[2] * 1000))
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
    if dist < 15:
        print("  → 距 URDF 零位 %.1f mm，**这个姿态就是零位**" % dist)
    else:
        print("  → 距零位 %.1f mm（差 前 %+.0f / 侧 %+.0f / 高 %+.0f；零位应为 480 / 0 / 371）"
              % (dist, dx, dy, dz))


def watch(mgr):
    """实时刷新：边掰边看末端往哪儿走。"""
    print("实时读数，Ctrl-C 结束。目标是末端「前 480 / 高 371」：")
    print("     j1     j2     j3     j4     j5     j6 |     前      侧      高   距零位")
    print("  " + "-" * 74)
    try:
        while True:
            rows = read_all(mgr, samples=1, delay=0.0)
            angs = [a for _, a, _ in rows[:6]]
            tip = tip_of(rows)
            if tip is not None:
                dx, dy, dz = (tip[0] * 1000 - 479.6, tip[1] * 1000, tip[2] * 1000 - 370.5)
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                line = ("  " + " ".join("%6.1f" % a for a in angs)
                        + " | %6.0f  %+6.0f  %6.0f   %5.0f"
                        % (tip[0] * 1000, tip[1] * 1000, tip[2] * 1000, dist))
                print("\r" + line, end="", flush=True)
            else:
                print("\r  （读不到全部关节，检查总线）" + " " * 40, end="", flush=True)
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="只读测量舵机角度，用于零位标定（不发运动指令）")
    ap.add_argument("--port", default=DEFAULT_PORT, help="串口，默认 %s" % DEFAULT_PORT)
    ap.add_argument("--baud", type=int, help="波特率，默认自动探测 1000000 / 115200")
    ap.add_argument("--samples", type=int, default=3,
                    help="每个舵机采样次数，取中位数（去抖），默认 3")
    ap.add_argument("--free", action="store_true",
                    help="解除全部舵机扭矩，让你能徒手摆臂（会掉！先托住）")
    ap.add_argument("--pose", action="append", metavar="标签",
                    help="读一次并记录到 CSV，标签写这个姿态是什么；可重复")
    ap.add_argument("--watch", action="store_true",
                    help="实时刷新读数与末端位置，边掰边看（Ctrl-C 结束）")
    ap.add_argument("--summary", action="store_true", help="读 CSV 并打印偏差汇总")
    ap.add_argument("--baseline", metavar="标签", help="指定哪一行作为基线")
    ap.add_argument("--yes", action="store_true", help="跳过安全确认")
    args = ap.parse_args()

    if args.summary:
        cmd_summary(args.baseline)
        return 0

    if not args.yes:
        print("=" * 68)
        print("只读测量，不会发任何运动指令。但请注意：")
        print("  · 本脚本不启动 robo_driver（那个节点一初始化就会断电复位并驱到零位）")
        print("  · 确认机械臂已固定、周围没有人和障碍物")
        if args.free:
            print("  · --free 会解除全部扭矩：臂会因重力掉落，**请先用手托住**")
        print("=" * 68)
        if ask("继续？[y/N] ").strip().lower() not in ("y", "yes"):
            print("已取消")
            return 1

    uservo = load_sdk()
    mgr, baud, uart = open_manager(uservo, args.port, args.baud)

    try:
        if args.free:
            print("解除全部舵机扭矩…（臂会掉，托住）")
            mgr.disable_torque(0xFF)
            time.sleep(0.2)

        if args.watch:
            watch(mgr)
        elif args.pose:
            for label in args.pose:
                ask("\n把臂摆到「%s」，摆好后回车（已记录的姿势不会被改动）… " % label)
                rows = read_all(mgr, args.samples)
                print_table(rows, "「%s」的读数：" % label)
                report_tip(rows)
                append_csv(label, rows, baud)
        else:
            rows = read_all(mgr, args.samples)
            print_table(rows, "当前读数：")
            report_tip(rows)
            print("\n提示：加 --pose <标签> 可以把当前姿态记进 CSV，供后续对比。")
            if not args.free:
                print("      要徒手摆臂需要先 --free（会掉，先托住）。")
    finally:
        try:
            uart.close()
        except Exception:  # noqa: BLE001
            pass
        if args.free:
            print("\n⚠ 舵机仍处于无力矩状态。重新上电或启动 robo_driver 才会恢复保持力。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

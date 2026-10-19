#!/usr/bin/env python3
"""limit_probe —— 逐关节实测行程，对照 URDF 限位

为什么要做这件事：
    URDF 里的关节限位是从模型文件抄来的，**从没在实物上验证过**。
    如果 URDF 允许的范围比实物大，MoveIt 会规划出一条"理论上可行"的路径，
    舵机走过去就撞上机械限位——堵转、异响，甚至顶坏东西。

做法（借 LeRobot 校准的思路，但只取"扫极值"这一半）：
    每次**只解开一个关节的扭矩**，其余关节继续撑着臂。
    你手动把它推到两个方向的极限，脚本记录角度。
    最后和 URDF 对比，指出哪些关节的 URDF 限位定宽了。

关键安全点：
    只解一个关节 ≠ 安全。j2/j3 解开后大臂/小臂会因重力下落，
    提示出现时**必须托住那一段**。测完会自动恢复力矩。

用法（⚠ 先断电确认周围无人，再执行）：
    python3 scripts/limit_probe.py              # 全部 6 个关节
    python3 scripts/limit_probe.py --joints 1 6 # 只测指定关节（从安全到危险）

输出：终端表格 + data/joint_limits.csv
"""

import argparse
import csv as csvmod
import os
import statistics
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
URDF = ROOT / "star-arm-moveit2/src/cello_description/urdf/cello_description.urdf"
CSV_PATH = ROOT / "data" / "joint_limits.csv"

DEFAULT_PORT = "/dev/ttyUSB0"
CANDIDATE_BAUDS = [1000000, 115200]

#: URDF 零位对应的舵机读数（2026-10-18 标定，与 robo_driver.ZERO_OFFSET_DEG 一致）
ZERO_OFFSET_DEG = [0.4, -0.2, -0.1, 1.3, -0.3, 0.8]

#: 按"从安全到危险"排列：底座旋转最安全，大臂最危险
PROBE_ORDER = [
    (1, "底座旋转", "安全"),
    (6, "腕部自转", "轻，注意别碰桌面"),
    (5, "腕部俯仰", "轻"),
    (4, "臂轴旋转", "轻"),
    (3, "小臂", "⚠ 解开后会下落，托住小臂"),
    (2, "大臂", "⚠ 解开后会倒，双手托住大臂"),
]

#: 实测行程低于 URDF 行程的这个比例，就判"没掰到"而不是"URDF 偏宽"
INCOMPLETE_RATIO = 0.7

#: 建议保留的余量（度）。URDF 限位应比实测极限再收这么多，避免贴着机械死点工作
SAFETY_MARGIN_DEG = 3.0


def die(msg):
    print("✗ %s" % msg)
    sys.exit(1)


def ask(prompt):
    try:
        return input(prompt)
    except (EOFError, KeyboardInterrupt):
        print()
        return "q"


def load_sdk():
    try:
        import fashionstar_uart_sdk as uservo
        return uservo
    except ImportError:
        die("缺少舵机 SDK。先装：pip install fashionstar-uart-sdk")


def open_manager(uservo, port, baud=None):
    import serial

    if not os.path.exists(port):
        die("串口不存在：%s（ls /dev/ttyUSB* 看设备名）" % port)

    for b in ([baud] if baud else CANDIDATE_BAUDS):
        try:
            uart = serial.Serial(port=port, baudrate=b, parity=serial.PARITY_NONE,
                                 stopbits=1, bytesize=8, timeout=0.2)
            mgr = uservo.UartServoManager(uart)
            mgr.query_servo_angle(0)   # 探活
            print("✓ %s @ %d baud" % (port, b))
            return mgr, b
        except Exception:
            continue
    die("连不上舵机。检查串口是否被占用（robo_driver 在跑？）")


def read_deg(mgr, sid, samples=5):
    """多次采样取中位数，避免单次读数抖动。"""
    vals = []
    for _ in range(samples):
        a = mgr.query_servo_angle(sid)
        if a is not None:
            vals.append(float(a))
        time.sleep(0.05)
    return statistics.median(vals) if vals else None


def urdf_limits():
    """从 URDF 读各关节的限位（弧度），转成"等效舵机角度"。"""
    if not URDF.is_file():
        return {}
    root = ET.parse(str(URDF)).getroot()
    out = {}
    for j in root.findall("joint"):
        name = j.get("name")
        if j.get("type") != "revolute" or not name or not name.startswith("joint"):
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        try:
            idx = int(name.replace("joint", "")) - 1
        except ValueError:
            continue
        lo = float(lim.get("lower")) * 180.0 / 3.141592653589793
        hi = float(lim.get("upper")) * 180.0 / 3.141592653589793
        off = ZERO_OFFSET_DEG[idx] if idx < len(ZERO_OFFSET_DEG) else 0.0
        # URDF 限位 + 零点偏移 = 该姿态应有的舵机读数
        out[name] = (lo + off, hi + off, lo, hi)
    return out


def sample_until_enter(mgr, sid):
    """后台持续采样，用户按回车停。返回 (min, max, 样本数)。

    为什么用线程：input() 会阻塞，而掰的过程中需要一直读角度才能抓到极值。
    串口只由这条线程读，主线程不碰，所以没有并发访问问题。
    """
    samples = []
    stop = threading.Event()

    def _worker():
        while not stop.is_set():
            a = mgr.query_servo_angle(sid)
            if a is not None:
                samples.append(float(a))
            time.sleep(0.12)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    try:
        input("  → 掰到两个方向的极限（顶住别硬掰），掰完回车： ")
    except (EOFError, KeyboardInterrupt):
        pass
    stop.set()
    t.join(timeout=1.0)
    if not samples:
        return None, None, 0
    return min(samples), max(samples), len(samples)


def probe_one(mgr, jid, desc, risk, limits, free_all=False):
    sid = jid - 1
    name = "joint%d" % jid
    print("\n" + "-" * 66)
    print("[%d/6] %-8s %s   %s" % (jid, name, desc, risk))

    before = read_deg(mgr, sid)
    if before is None:
        print("  ✗ 读不到该关节角度，跳过")
        return None
    print("  当前 %.1f°" % before)

    if free_all:
        print("  全臂已松开，随便摆到什么姿态都行")
        lo, hi, n = sample_until_enter(mgr, sid)
        # 全部测完再统一恢复：中途上力矩就又变成"其他关节锁着"了
    else:
        mgr.disable_torque(sid)
        time.sleep(0.2)
        print("  已解锁（其余关节仍保持力矩）")
        try:
            lo, hi, n = sample_until_enter(mgr, sid)
        finally:
            # 无论如何都恢复：发回解锁前的角度，舵机会重新上力矩
            mgr.set_servo_angle(sid, before, interval=800)
            time.sleep(0.9)
            print("  已恢复力矩 → %.1f°" % before)

    if lo is None:
        print("  ✗ 没采到读数，跳过")
        return None

    travel = hi - lo
    print("  实测 %.1f° ~ %.1f°（行程 %.1f°，%d 个采样）" % (lo, hi, travel, n))
    if travel < 10:
        print("  ⚠ 行程只有 %.1f°：可能没掰动，或被外力卡住" % travel)

    rec = {"joint": name, "lo": lo, "pos": before, "hi": hi, "travel": travel,
           "udf_lo": None, "udf_hi": None, "verdict": ""}
    if name in limits:
        udf_lo, udf_hi, _lo_rad, _hi_rad = limits[name]
        rec["udf_lo"], rec["udf_hi"] = udf_lo, udf_hi
        udf_travel = udf_hi - udf_lo
        # 可信度闸门：实测行程明显小于 URDF 行程时，说明**没掰到极限**
        # （其他关节锁着，几何上挡住了），此时不能判"URDF 偏宽"——
        # 照这个结论去收窄 URDF 会把还能到的地方也砍掉。
        if udf_travel > 0 and travel < udf_travel * INCOMPLETE_RATIO:
            rec["verdict"] = ("测量不完整（实测 %.0f°，仅 URDF 的 %.0f%%）"
                              % (travel, travel / udf_travel * 100))
        else:
            wider_lo = udf_lo < lo - 0.5    # URDF 下限比实测更低 → URDF 太宽
            wider_hi = udf_hi > hi + 0.5
            if wider_lo or wider_hi:
                gaps = []
                if wider_lo:
                    gaps.append("下限宽 %.1f°" % (lo - udf_lo))
                if wider_hi:
                    gaps.append("上限宽 %.1f°" % (udf_hi - hi))
                rec["verdict"] = "URDF 偏宽：" + "、".join(gaps)
            else:
                rec["verdict"] = "URDF 保守（安全）"
    return rec


def report(records):
    """输出 MIN / POS / MAX 三列（对齐 LeRobot 校准表的格式）。"""
    # 表格按关节号排，不看测试顺序——测试是从安全到危险（j1→j6→…），
    # 但看结果时按 1..6 更顺眼
    records = sorted(records, key=lambda r: int(r["joint"].replace("joint", "")))
    print("\n" + "-" * 46)
    print("  %-9s | %8s | %8s | %8s" % ("NAME", "MIN", "POS", "MAX"))
    print("-" * 46)
    for r in records:
        print("  %-9s | %8.1f | %8.1f | %8.1f"
              % (r["joint"], r["lo"], r["pos"], r["hi"]))
    print("-" * 46)

    # URDF 对照只留结论，不占版面
    incomplete = [r for r in records if r["verdict"].startswith("测量不完整")]
    if incomplete:
        print("\n  ○ 这几个关节没测全（其他关节锁着，几何上挡住了）：")
        for r in incomplete:
            print("      %-8s %s" % (r["joint"], r["verdict"]))
        print("    要测全用 --free-all（全部解锁，臂会塌，托住）")

    bad = [r for r in records if r["verdict"].startswith("URDF 偏宽")]
    if bad:
        print("\n  ⚠ URDF 限位比实物宽（MoveIt 会规划到不了的关节角）：")
        for r in bad:
            print("      %-8s %s" % (r["joint"], r["verdict"]))
        print("    要收窄就把 lower/upper 收到实测值以内（留 %.0f° 余量），改完重建。"
              % SAFETY_MARGIN_DEG)
    elif records:
        print("\n  ✓ URDF 限位都不宽于实测，规划不会撞机械限位。")

    # 合并写：分批测（--joints 3）时不能把上一批的结果抹掉。
    # 同名关节用本次数据覆盖，其余保留。
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    merged = {}
    if CSV_PATH.exists():
        try:
            with open(CSV_PATH, newline="", encoding="utf-8") as f:
                for row in csvmod.DictReader(f):
                    if row.get("joint"):
                        merged[row["joint"]] = row
        except Exception:
            pass
    for r in records:
        merged[r["joint"]] = {
            "joint": r["joint"], "min_deg": "%.2f" % r["lo"],
            "pos_deg": "%.2f" % r["pos"], "max_deg": "%.2f" % r["hi"],
            "travel_deg": "%.2f" % r["travel"],
            "urdf_min_deg": "" if r["udf_lo"] is None else "%.2f" % r["udf_lo"],
            "urdf_max_deg": "" if r["udf_hi"] is None else "%.2f" % r["udf_hi"],
            "verdict": r["verdict"],
        }
    cols = ["joint", "min_deg", "pos_deg", "max_deg", "travel_deg",
            "urdf_min_deg", "urdf_max_deg", "verdict"]
    with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
        w = csvmod.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for name in sorted(merged, key=lambda s: int(s.replace("joint", ""))):
            w.writerow(merged[name])
    print("\n  明细已合并写入 %s（共 %d 个关节）" % (CSV_PATH, len(merged)))


def main():
    ap = argparse.ArgumentParser(
        description="逐关节实测行程并与 URDF 限位对照（只解一个关节的扭矩）")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int)
    ap.add_argument("--joints", nargs="+", type=int,
                    help="只测这些关节（1~6）。建议先测 1 6 5 4，最后测 3 2")
    ap.add_argument("--yes", action="store_true", help="跳过开头的确认")
    ap.add_argument("--free-all", action="store_true",
                    help="全部解锁再测（LeRobot 式）。单关节锁着时某些关节掰不全，"
                         "比如 j2 会被整条臂的姿态挡住。臂会塌，托住")
    args = ap.parse_args()

    print("=" * 66)
    print("逐关节行程实测。每次只解开一个关节的扭矩，其余关节保持力矩。")
    print("⚠ j2/j3 解开后大臂/小臂会下落，提示时必须托住。")
    print("⚠ 测完会自动恢复力矩，但过程中请一直扶着。")
    print("=" * 66)
    if not args.yes and ask("确认机械臂周围无人、无障碍，继续？[y/N] ").strip().lower() not in ("y", "yes"):
        print("已取消。")
        return 0

    uservo = load_sdk()
    mgr, _baud = open_manager(uservo, args.port, args.baud)
    limits = urdf_limits()
    if not limits:
        print("⚠ 读不到 URDF 限位，只记录实测值")

    order = PROBE_ORDER
    if args.joints:
        want = set(args.joints)
        order = [o for o in PROBE_ORDER if o[0] in want]

    if args.free_all:
        print("\n解除全部舵机扭矩…（臂会塌，托住）")
        mgr.disable_torque(0xFF)
        time.sleep(0.3)
        print("全臂已松开。逐个关节掰到两个极限即可，其他关节跟着动是正常的。")

    records = []
    try:
        for jid, desc, risk in order:
            rec = probe_one(mgr, jid, desc, risk, limits, args.free_all)
            if rec:
                records.append(rec)
    except KeyboardInterrupt:
        print("\n\n中断：正在恢复所有关节到当前角度…")
        for jid, _, _ in order:
            sid = jid - 1
            cur = read_deg(mgr, sid, samples=1)
            if cur is not None:
                mgr.set_servo_angle(sid, cur, interval=500)
        print("已恢复。")
        return 1

    if args.free_all:
        print("\n逐个恢复力矩（发回各自当前角度，臂不会动）…")
        for jid, _, _ in order:
            cur = read_deg(mgr, jid - 1, samples=1)
            if cur is not None:
                mgr.set_servo_angle(jid - 1, cur, interval=600)
                time.sleep(0.3)
        print("已全部恢复。")

    if records:
        report(records)
    else:
        print("\n没有有效记录。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# coding: utf-8
"""意图桥：把 SSVEP 的类别判定，翻译成机械臂动作。

它不关心类别是怎么来的——真实解码、回放文件、手敲都行。
只要按下面的格式喂进来，就执行对应动作。

输入格式（每行一条，从 stdin 读）
    <k>              只给类别
    <k> <conf>       类别 + 置信度（低于阈值则丢弃）

类别索引与 `ssvep-bci` 的 FREQS 对齐：
    0 → 15.0 Hz   1 → 16.5 Hz   2 → 18.33 Hz   3 → 20.625 Hz   4 → 13.75 Hz(停止块)

用法
    # 回放一段序列（不需要脑电硬件）
    python3 scripts/intent_bridge.py --replay "0,1,0,2"

    # 从管道读（真实解码器只要 print 出 k 就能接）
    python3 scripts/intent_bridge.py < intents.txt
    ssvep_realtime | python3 scripts/intent_bridge.py

    # 只看不动
    python3 scripts/intent_bridge.py --replay "0,1" --dry-run

为什么不做成"持续注视=持续运动"（像小车那样）：
    机械臂是离散定位，一次判定对应一个目标位姿，执行完就停。
    连续跟随没有意义，而且一旦误判会一直走。
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
CLI = SCRIPTS / "roarm_cli.py"

#: 类别 → 动作。索引与 ssvep-bci 的 FREQ_NAMES 对应。
#: 第一版全部选**已验证可达、且姿态不极端**的目标，避免一上手就撞。
INTENT_MAP = {
    0: ("home",  ["home"]),
    1: ("up",    ["up"]),
    2: ("right", ["right"]),
    3: ("safe",  ["pose", "0.45", "0", "0.40"]),
    4: ("stop",  None),                    # 停止块：不动作
}

WAIT_S = 25           # 单条动作等待上限
COOLDOWN_S = 1.0      # 两次动作之间的最小间隔
CONF_MIN = 0.30       # 低于此置信度直接丢弃


def log(msg):
    print(msg, flush=True)


def run_action(argv, timeout=WAIT_S + 10):
    """调用 roarm_cli，返回 (rc, 摘要)。"""
    cmd = [sys.executable, str(CLI)] + argv + ["--wait", str(WAIT_S)]
    t0 = time.time()
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        out = (p.stdout or "") + (p.stderr or "")
        tail = [ln.strip() for ln in out.splitlines() if ln.strip()][-1:] or [""]
        return p.returncode, tail[0], time.time() - t0
    except subprocess.TimeoutExpired:
        return 124, "外层超时", time.time() - t0


def parse_line(line):
    """返回 (k, conf) 或 None。"""
    parts = line.replace(",", " ").split()
    if not parts:
        return None
    try:
        k = int(parts[0])
    except ValueError:
        return None
    conf = None
    if len(parts) > 1:
        try:
            conf = float(parts[1])
        except ValueError:
            conf = None
    return k, conf


def iter_intents(args):
    if args.replay:
        for chunk in args.replay.replace(";", ",").split(","):
            chunk = chunk.strip()
            if chunk:
                yield chunk
        return
    for line in sys.stdin:
        line = line.strip()
        if line:
            yield line


def main():
    ap = argparse.ArgumentParser(
        description="把 SSVEP 类别判定翻译成机械臂动作")
    ap.add_argument("--replay", metavar="SEQUENCE",
                    help="回放序列，如 \"0,1,0,2\"；不给则从 stdin 读")
    ap.add_argument("--dry-run", action="store_true", help="只打印不执行")
    ap.add_argument("--conf-min", type=float, default=CONF_MIN,
                    help="置信度下限，默认 %.2f" % CONF_MIN)
    ap.add_argument("--cooldown", type=float, default=COOLDOWN_S,
                    help="两次动作最小间隔（秒），默认 %.1f" % COOLDOWN_S)
    ap.add_argument("--stop-on-fail", action="store_true",
                    help="任一动作失败即退出（默认继续）")
    args = ap.parse_args()

    if not args.dry_run and not CLI.is_file():
        print("✗ 找不到 %s" % CLI, file=sys.stderr)
        return 2

    log("意图桥启动（%s）" % ("回放" if args.replay else "stdin"))
    log("%-4s %-8s %-8s %-6s %s" % ("#", "类别", "动作", "耗时", "结果"))
    log("-" * 62)

    n_ok = n_skip = n_fail = 0
    last_end = 0.0

    for i, raw in enumerate(iter_intents(args), 1):
        parsed = parse_line(raw)
        if parsed is None:
            log("%-4d %-8s %-8s %-6s 跳过（无法解析）" % (i, raw, "-", "-"))
            n_skip += 1
            continue

        k, conf = parsed
        if k not in INTENT_MAP:
            log("%-4d %-8d %-8s %-6s 跳过（未知类别）" % (i, k, "-", "-"))
            n_skip += 1
            continue

        if conf is not None and conf < args.conf_min:
            log("%-4d %-8d %-8s %-6s 跳过（置信度 %.2f < %.2f）"
                % (i, k, "-", "-", conf, args.conf_min))
            n_skip += 1
            continue

        name, argv = INTENT_MAP[k]
        if argv is None:
            log("%-4d %-8d %-8s %-6s 停止块，不动作" % (i, k, name, "-"))
            continue

        if args.dry_run:
            log("%-4d %-8d %-8s %-6s dry-run: %s" % (i, k, name, "-", " ".join(argv)))
            n_ok += 1
            continue

        # 冷却：避免连续判定把同一条指令排队重发
        gap = time.time() - last_end
        if gap < args.cooldown:
            time.sleep(args.cooldown - gap)

        rc, detail, elapsed = run_action(argv)
        tag = "OK" if rc == 0 else "FAIL"
        log("%-4d %-8d %-8s %5.1fs %s %s" % (i, k, name, elapsed, tag, detail[:34]))
        last_end = time.time()

        if rc == 0:
            n_ok += 1
        else:
            n_fail += 1
            if args.stop_on_fail:
                log("\n失败即停（--stop-on-fail）")
                break

    log("-" * 62)
    log("完成：成功 %d，跳过 %d，失败 %d" % (n_ok, n_skip, n_fail))
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

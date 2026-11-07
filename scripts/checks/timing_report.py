#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""汇总 data/vision/timing.jsonl，按阶段给出均值/中位数/最大值。

    python3 scripts/checks/timing_report.py
    python3 scripts/checks/timing_report.py --last 5

只看最近 N 条记录；先清空 timing.jsonl 再测基线更干净。
"""
import argparse
import json
import statistics
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from roarm_lib import paths  # noqa: E402


def load(path):
    rows = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    except FileNotFoundError:
        pass
    return rows


def summary(rows, key_fn, value_fn):
    grouped = {}
    for r in rows:
        key = key_fn(r)
        if key is None:
            continue
        grouped.setdefault(key, []).append(float(value_fn(r)))
    out = []
    for key, vals in grouped.items():
        out.append((key, len(vals), statistics.mean(vals),
                    statistics.median(vals), max(vals)))
    return sorted(out, key=lambda x: -x[2])


def print_table(title, rows):
    if not rows:
        return
    print("\n=== %s ===" % title)
    print("  %-26s %5s %8s %8s %8s" % ("阶段", "次数", "均值s", "中位s", "最大s"))
    for name, n, mean, med, mx in rows:
        print("  %-26s %5d %8.2f %8.2f %8.2f" % (name, n, mean, med, mx))


def main():
    ap = argparse.ArgumentParser(description="汇总机械臂阶段计时")
    ap.add_argument("--file", default=str(paths.DATA_VISION / "timing.jsonl"))
    ap.add_argument("--last", type=int, default=20,
                    help="只统计最近 N 条记录（pick / service 各自）")
    args = ap.parse_args()

    rows = load(args.file)
    if not rows:
        print("没有计时记录：%s\n先跑一次 pick/place。" % args.file)
        return 1

    picks = [r for r in rows if r.get("source") == "pick"][-args.last:]
    svc = [r for r in rows if r.get("source") == "service"][-args.last:]

    if picks:
        n_pick = sum(1 for r in picks if r.get("mode") == "pick")
        n_place = sum(1 for r in picks if r.get("mode") == "place")
        print("pick.py 条数=%d（pick %d / place %d）"
              % (len(picks), n_pick, n_place))
        for mode in ("pick", "place"):
            subset = [r for r in picks if r.get("mode") == mode]
            if not subset:
                continue
            totals = [r.get("total_s", 0.0) for r in subset]
            print("\n%s 总耗时：均值 %.2fs / 中位 %.2fs / 最大 %.2fs（n=%d）"
                  % (mode, statistics.mean(totals),
                     statistics.median(totals), max(totals), len(subset)))
            steps = []
            for r in subset:
                steps.extend(r.get("steps", []))
            print_table("%s 阶段" % mode,
                        summary(steps, lambda x: x.get("name"),
                                lambda x: x.get("s", 0.0)))
    if svc:
        print_table("service 事件",
                    summary(svc, lambda x: x.get("event"),
                            lambda x: x.get("s", 0.0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
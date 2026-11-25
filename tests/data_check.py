#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运行数据布局检查：代码要的 data/vision/* 是否真的在。

2026-11-08 踩过：数据被移到 data/ 根目录，check_all 全绿，但 pick.py 一启动
就 FileNotFoundError。本检查只查存在性和关键文件，不连硬件/相机。
"""
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "app" / "core" / "paths.py").is_file())
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from app.core import paths  # noqa: E402

REQUIRED = [
    paths.DATA_VISION / "T_base_cam.npz",
    paths.DATA_VISION / "cam_intrinsics.npz",
    paths.DATA_VISION / "grasp_offset.json",
    paths.DATA_VISION / "xy_correction.json",
]
OPTIONAL = [
    paths.SCENE_REAL,
    paths.TEACH_POINTS,
    paths.DATA_VISION / "handeye_poses.txt",
]

fails = []
print("[1] 运行数据目录")
if not paths.DATA_VISION.is_dir():
    fails.append("%s 不存在" % paths.DATA_VISION)
    print("  ✗ %s 不存在" % paths.DATA_VISION)
else:
    print("  ✓ %s" % paths.DATA_VISION)

print("[2] 必需文件")
for p in REQUIRED:
    ok = p.is_file()
    print("  %s %s" % ("✓" if ok else "✗", p))
    if not ok:
        fails.append(str(p))

print("[3] 可选/任务层文件")
for p in OPTIONAL:
    print("  %s %s" % ("✓" if p.is_file() else "⚠ 缺失", p))

if fails:
    print("\n✗ 数据布局不完整：%d 项缺失" % len(fails))
    print("  旧布局要求 data/vision/；如果坚持扁平 data/，请同步改 paths.py。")
    sys.exit(1)
print("\n✓ 运行数据布局正常")
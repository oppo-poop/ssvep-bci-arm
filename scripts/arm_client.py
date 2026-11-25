#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compat shim：实际实现已迁至 app/service/arm_client.py。
保留本文件仅为兼容外部 ssvep-bci 与历史脚本调用。
"""
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "app/service/arm_client.py"
for d in (str(ROOT), str(TARGET.parent)):
    if d not in sys.path:
        sys.path.insert(0, d)
runpy.run_path(str(TARGET), run_name="__main__")

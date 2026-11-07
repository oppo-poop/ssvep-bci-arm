#!/usr/bin/env python3
# compat shim：实际实现已移到 scripts/calib/repeatability.py。
# 保留本文件是为了让 `python3 scripts/repeatability.py` 继续可用。
import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "calib" / "repeatability.py"),
    run_name="__main__",
)
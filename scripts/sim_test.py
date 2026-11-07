#!/usr/bin/env python3
# compat shim：实际实现已移到 scripts/checks/sim_test.py。
# 保留本文件是为了让 `python3 scripts/sim_test.py` 继续可用。
import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "checks" / "sim_test.py"),
    run_name="__main__",
)
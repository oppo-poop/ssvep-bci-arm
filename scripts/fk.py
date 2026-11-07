#!/usr/bin/env python3
# compat shim：实际实现已移到 scripts/roarm_lib/fk.py。
# 保留本文件是为了让 `python3 scripts/fk.py` 继续可用。
# 新代码请用 `from roarm_lib import fk`。
import runpy
from pathlib import Path

runpy.run_path(
    str(Path(__file__).resolve().parent / "roarm_lib" / "fk.py"),
    run_name="__main__",
)
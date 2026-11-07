"""仓库路径的唯一来源。

任何脚本都不要再手写 parents[N] / "scripts" 字符串；从这里取。
被移动/被旧路径 runpy shim 调用时都成立，因为路径只取决于本文件位置。
"""

import os
from pathlib import Path


def _root() -> Path:
    env = os.environ.get("ROARM_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    # scripts/roarm_lib/paths.py -> scripts -> 仓库根
    return Path(__file__).resolve().parents[2]


ROOT = _root()
SCRIPTS = ROOT / "scripts"
WS = ROOT / "star-arm-moveit2"
SRC = WS / "src"
DATA = ROOT / "data"
DATA_VISION = DATA / "vision"
SCENE_REAL = DATA / "scene_real.yaml"
TEACH_POINTS = DATA / "teach_points.json"
WS_SETUP = WS / "install" / "setup.bash"
URDF = SRC / "cello_description" / "urdf" / "cello_description.urdf"
CLI = SCRIPTS / "roarm_cli.py"
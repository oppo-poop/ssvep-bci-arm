#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固定位置抓放 —— 不连相机，坐标来自 data/positions_6.yaml。

为什么单独一个文件而不在 pick.py 里加分支
    pick.py 有 2000 行、相机初始化/点选/自动检测三条岔路，为了一个"不用相机"
    的模式往里塞条件，等于给已经跑通的视觉链路引入回归风险。
    这里只做一件事：查表 → 发 pose → 发夹爪命令，逻辑一眼能看完。

接口与 pick.py 同形（这是和 ssvep-bci / arm_service 的契约）
    取一个：      python3 scripts/pick_fixed.py --color red --no-place --hold-here
    放回对应托盘： python3 scripts/pick_fixed.py --place-only --color red
    取 + 放一趟： python3 scripts/pick_fixed.py --color red
    取红放绿：    python3 scripts/pick_fixed.py --color red --pad green

    arm_service.py 只要把 `PICK` 换成这个文件、place 时传 --color，
    脑电那边（arm_sink）完全不用改。

坐标口径
    positions_6.yaml 里的 objects/pads 是 **link6（法兰）实际应该到达的位置**，
    不含 z 命令补偿；补偿单独存在 z_cmd_offset_m，由本脚本加在**发出去的 z** 上。
    （2026-11-10 实测：这台臂 z 落点系统性偏低 9~16 mm。）

用法前提
    先 `bash scripts/robot_up.sh` 起 ROS 栈。本脚本自己 source ROS 再调 roarm_cli，
    所以对当前 shell 有没有 source 不敏感。

退出码
    0 完成    1 中途失败（某一步未到位）    2 输入/配置被拒
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent                 # scripts/
sys.path.insert(0, str(HERE))
from roarm_lib import paths, params                    # noqa: E402

CLI = paths.SCRIPTS / "roarm_cli.py"
POS_YAML = paths.DATA / "positions_6.yaml"

#: 调 roarm_cli 的解释器。与 arm_service 保持一致：用系统 python3 而不是
#: 当前解释器，避免在 venv 里跑时找不到 rclpy。
PY_ROS = os.environ.get("ARM_PYTHON_ROS", "/usr/bin/python3")

EXIT_OK, EXIT_FAIL, EXIT_REJECT = 0, 1, 2


# ---------------------------------------------------------------- 调 CLI
def _ros_setup():
    distro = os.environ.get("ROS_DISTRO", "humble")
    return (os.environ.get("ROS_SETUP", "/opt/ros/%s/setup.bash" % distro),
            str(paths.WS_SETUP))


def call(argv, timeout=180):
    """调一条 roarm_cli 命令，返回 (rc, out)。

    自己 source ROS + 工作区：调用方可能来自 systemd / 裸 python，
    不该要求它先手动 source。
    """
    ros, ws = _ros_setup()
    inner = " ".join(shlex.quote(x) for x in
                     [PY_ROS, str(CLI)] + [str(a) for a in argv])
    cmd = "source %s >/dev/null 2>&1; source %s >/dev/null 2>&1; exec %s" % (
        shlex.quote(ros), shlex.quote(ws), inner)
    debug = os.environ.get("PICK_FIXED_DEBUG", "0") == "1"
    if debug:
        print("    $ " + cmd, flush=True)
    try:
        p = subprocess.run(["bash", "-lc", cmd], capture_output=True,
                           text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, "外层超时（%ds）" % timeout
    out = (p.stdout or "") + (p.stderr or "")
    return p.returncode, out


def show_tail(out, n=3):
    lines = [ln for ln in (out or "").strip().splitlines() if ln.strip()]
    for ln in lines[-n:]:
        print("    " + ln.strip())


# ---------------------------------------------------------------- 读点位
def load_positions(path=POS_YAML):
    """返回 dict：objects/pads/object_width/standoff/z_cmd_offset_m。"""
    try:
        import yaml
    except ImportError:
        raise SystemExit("需要 pyyaml：pip3 install pyyaml")
    if not os.path.isfile(path):
        raise SystemExit("找不到点位文件：%s" % path)
    with open(path, encoding="utf8") as f:
        d = yaml.safe_load(f)
    for key in ("objects", "pads"):
        if not d.get(key):
            raise SystemExit("%s 里缺 %s 段" % (path, key))
    return d


def get_point(d, kind, color):
    """取某个颜色物料/托盘的坐标（米），并算上 z 命令补偿。

    返回 (x, y, z_real, z_send)。
    """
    xyz = d[kind][color]
    if len(xyz) != 3:
        raise SystemExit("%s.%s 必须是 [x, y, z]" % (kind, color))
    x, y, z = (float(v) for v in xyz)
    off = float(d.get("z_cmd_offset_m", 0.0))
    return x, y, z, z + off


# ---------------------------------------------------------------- 单步动作
def go_pose(x, y, z_send, args, what):
    """走一个位姿。z_send 是**已经补过** z 补偿的值。"""
    argv = ["pose", "%.4f" % x, "%.4f" % y, "%.4f" % z_send]
    argv += ["--wait", "%.0f" % args.wait, "--tol", "%.1f" % args.tol]
    if not args.no_force:
        argv.append("--force")
    print("  → %-18s (%.3f, %+.3f, %.3f)" % (what, x, y, z_send), flush=True)
    t0 = time.time()
    rc, out = call(argv, timeout=args.wait + 40)
    if rc == 2:
        print("    ✗ 被拒绝（工作区/输入）")
        show_tail(out)
        return EXIT_REJECT
    if rc != 0:
        print("    ✗ 未到位（%.0f s）" % (time.time() - t0))
        show_tail(out)
        return EXIT_FAIL
    print("    ✓ 到位（%.1f s）" % (time.time() - t0))
    return EXIT_OK


def go_gap(mm, args, what):
    """把两指开到指定间距（mm）。角度按 params 的二次曲线反算，别写线性。"""
    mm = max(params.GMIN, min(params.GMAX, float(mm)))
    deg = params.deg_of_gap(mm)
    print("  → %-18s 两指 %.1f mm（%.1f°）" % (what, mm, deg), flush=True)
    rc, out = call(["grip_deg", "%.2f" % deg,
                    "--wait", "%.0f" % params.GRIP_WAIT_S,
                    "--tol", "3.0"], timeout=30)
    if rc != 0:
        print("    ✗ 夹爪未到位")
        show_tail(out)
        return EXIT_FAIL
    print("    ✓ 到位")
    return EXIT_OK


# ---------------------------------------------------------------- 流程
def do_pick(d, args):
    """取：抬高 → 下降 → 夹 → 抬起。"""
    x, y, z_real, z_send = get_point(d, "objects", args.color)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    lift = args.lift if args.lift is not None else standoff
    width_mm = float(d.get("object_width", 0.05)) * 1000.0
    # 合拢目标比物宽小 2 mm：留 2 mm 过盈才夹得住，又不至于让舵机堵转。
    # ⚠ 不能用 `grip close`（目标 4 mm），50 mm 物料会被压住还继续往里顶。
    close_mm = width_mm - 2.0

    if go_pose(x, y, z_send + standoff, args, "取:上方"):
        return EXIT_FAIL
    if go_pose(x, y, z_send, args, "取:下降"):
        return EXIT_FAIL
    if go_gap(close_mm, args, "取:夹紧"):
        return EXIT_FAIL
    if go_pose(x, y, z_send + lift, args, "取:抬起"):
        return EXIT_FAIL
    print("  ✓ 已夹住 %s 物料（目标开度 %.1f mm）" % (args.color, close_mm))
    return EXIT_OK


def do_place(d, args):
    """放：抬高 → 下降 → 张开 → 抬起。"""
    x, y, z_real, z_send = get_point(d, "pads", args.pad)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    lift = args.lift if args.lift is not None else standoff

    if go_pose(x, y, z_send + standoff, args, "放:上方"):
        return EXIT_FAIL
    if go_pose(x, y, z_send, args, "放:下降"):
        return EXIT_FAIL
    # 张开到语义目标（默认 60 mm），不是机械极限 68 —— 顶在极限上会让舵机发烫
    if go_gap(params.OPEN_TARGET_MM, args, "放:松爪"):
        return EXIT_FAIL
    if go_pose(x, y, z_send + lift, args, "放:抬起"):
        return EXIT_FAIL
    print("  ✓ 已放到 %s 托盘" % args.pad)
    return EXIT_OK


def main():
    ap = argparse.ArgumentParser(
        description="固定位置抓放（不连相机，坐标来自 data/positions_6.yaml）")
    ap.add_argument("--color", required=True,
                    choices=("red", "green", "blue"),
                    help="取哪个物料（与 pick.py --color 同义）")
    ap.add_argument("--pad", default=None,
                    choices=("red", "green", "blue"),
                    help="放到哪个托盘，默认与 --color 相同")
    ap.add_argument("--yaml", default=str(POS_YAML),
                    help="点位文件，默认 data/positions_6.yaml")
    ap.add_argument("--no-place", action="store_true",
                    help="只取不放（取完举着）")
    ap.add_argument("--hold-here", action="store_true",
                    help="配合 --no-place：停在原地，不回 home。"
                         "已有 --no-place 时不回 home，这个参数只为和 pick.py 对齐")
    ap.add_argument("--place-only", action="store_true",
                    help="物料已在爪里：跳过取物，直接从放置段开始")
    ap.add_argument("--no-home", action="store_true",
                    help="开始/结束时都不回 home")
    ap.add_argument("--standoff", type=float, default=None,
                    help="接近/撤离的竖直偏移（米），默认取 yaml 的 standoff")
    ap.add_argument("--lift", type=float, default=None,
                    help="抬起/撤离高度（米），默认同 standoff")
    ap.add_argument("--wait", type=float, default=40.0,
                    help="每个位姿最长等待（秒）。默认速度 0.1 时 50mm 约 15s")
    ap.add_argument("--tol", type=float, default=12.0,
                    help="到位容差（毫米）。实测残差 2~6mm，取 12 留余量；"
                         "别为了日志好看放大到 30（会读到半路位置）")
    ap.add_argument("--no-force", action="store_true",
                    help="不加 --force（点位 z 低于 bounds 下限时会被 CLI 拒绝）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要走的点，不动机械臂")
    ap.add_argument("--json", action="store_true",
                    help="最后输出一行 JSON（给 arm_service 解析）")
    args = ap.parse_args()
    args.pad = args.pad or args.color

    try:
        d = load_positions(args.yaml)
    except SystemExit as e:
        print("✗ %s" % e, file=sys.stderr)
        return EXIT_REJECT

    for kind, name in (("objects", args.color), ("pads", args.pad)):
        if name not in d[kind]:
            print("✗ %s 里没有 %s.%s" % (args.yaml, kind, name), file=sys.stderr)
            return EXIT_REJECT

    # ---- 打印计划（--dry-run 到此为止）
    ox, oy, oz, oz_send = get_point(d, "objects", args.color)
    px, py, pz, pz_send = get_point(d, "pads", args.pad)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    width_mm = float(d.get("object_width", 0.05)) * 1000.0
    print("固定位置抓放：取 %s → 放 %s" % (args.color, args.pad))
    print("  物料 (%.3f, %+.3f, %.3f)  z+%.0fmm 发出"
          % (ox, oy, oz, (oz_send - oz) * 1000))
    print("  托盘 (%.3f, %+.3f, %.3f)  z+%.0fmm 发出"
          % (px, py, pz, (pz_send - pz) * 1000))
    print("  接近 %.0fmm / 物宽 %.0fmm → 合拢 %.0fmm"
          % (standoff * 1000, width_mm, width_mm - 2.0))
    if args.dry_run:
        print("  --dry-run：不动机械臂")
        return EXIT_OK
    print()

    rc = EXIT_OK
    t0 = time.time()

    # ---- 起始回零（从上次失败的位置起步最容易撞）
    if not args.no_home and not args.place_only:
        print("  → %-18s" % "回 home", flush=True)
        r, out = call(["home", "--wait", "180", "--tol", "25"], timeout=220)
        if r != 0:
            print("    ✗ 回 home 失败"); show_tail(out); return EXIT_FAIL
        print("    ✓ 已回 home")

    # ---- 取
    if not args.place_only:
        rc = do_pick(d, args)
        if rc != EXIT_OK:
            return rc
        if args.no_place:
            print("\n✓ 取完（--no-place，停在原地，不回 home）")
            return EXIT_OK

    # ---- 放
    rc = do_place(d, args)
    if rc != EXIT_OK:
        return rc

    # ---- 收尾回零
    if not args.no_home:
        print("  → %-18s" % "回 home", flush=True)
        r, out = call(["home", "--wait", "180", "--tol", "25"], timeout=220)
        if r != 0:
            print("    ✗ 收尾回 home 失败（物料已放下）")
            show_tail(out)
            # 物料已经放好了，回不去 home 不算抓放失败

    print("\n✓ 完成，用时 %.0f s" % (time.time() - t0))
    if args.json:
        print(json.dumps({"ok": True, "color": args.color, "pad": args.pad,
                          "elapsed_s": round(time.time() - t0, 1)},
                         ensure_ascii=False))
    return EXIT_OK


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⛔ Ctrl+C：已中断（物料状态未知，先 where 看一眼再动）")
        sys.exit(130)

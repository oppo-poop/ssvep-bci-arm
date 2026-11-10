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


def get_point(d, kind, color, dz_mm=0.0):
    """取某个颜色物料/托盘的坐标（米），并算上 z 命令补偿。

    返回 (x, y, z_real, z_send)。dz_mm 是额外的"抓取高度微调"（只用于物料点）：
    示教时人眼判断"指尖贴住物料"，容易偏高几毫米，用它在不改示教值的前提下压低。
    """
    xyz = d[kind][color]
    if len(xyz) != 3:
        raise SystemExit("%s.%s 必须是 [x, y, z]" % (kind, color))
    x, y, z = (float(v) for v in xyz)
    off = float(d.get("z_cmd_offset_m", 0.0))
    return x, y, z + dz_mm / 1000.0, z + off + dz_mm / 1000.0


# ---------------------------------------------------------------- 单步动作
def go_pose(x, y, z_send, args, what, wait=None):
    """走一个位姿。z_send 是**已经补过** z 补偿的值。

    wait 默认用 args.wait（下降等关键动作），过渡动作传 args.wait_fast。
    为什么过渡动作要短：这台臂远端抬不过 z≈0.10，抬起那步**本来就到不了**，
    等满只是白等 —— 原来每趟多花 40 秒。
    """
    w = args.wait if wait is None else wait
    argv = ["pose", "%.4f" % x, "%.4f" % y, "%.4f" % z_send]
    argv += ["--wait", "%.0f" % w, "--tol", "%.1f" % args.tol]
    if not args.no_force:
        argv.append("--force")
    print("  → %-18s (%.3f, %+.3f, %.3f)" % (what, x, y, z_send), flush=True)
    t0 = time.time()
    rc, out = call(argv, timeout=w + 40)
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


def parse_err(out):
    """从 CLI 输出里抠出误差（mm）。成功/失败两种格式都有"误差 x mm"。"""
    import re
    m = re.search(r"误差\s*([\d.]+)\s*mm", out or "")
    return float(m.group(1)) if m else None


def go_gap(mm, args, what, tol=3.0, accept_blocked=False, wait=None):
    """把两指开到指定间距（mm）。角度按 params 的二次曲线反算，别写线性。

    accept_blocked：夹着物料时**到不了目标**才是正常情况（物宽 > 目标开度），
    CLI 会返回 rc=1 并报误差。这时只要误差在 2~25mm（说明确实咬住了东西），
    就算成功 —— 这是 2026-11-07 grasp_closed_ok 那次踩出来的判据。
    """
    mm = max(params.GMIN, min(params.GMAX, float(mm)))
    deg = params.deg_of_gap(mm)
    print("  → %-18s 两指 %.1f mm（%.1f°）" % (what, mm, deg), flush=True)
    w = params.GRIP_WAIT_S if wait is None else wait
    rc, out = call(["grip_deg", "%.2f" % deg,
                    "--wait", "%.0f" % w,
                    "--tol", "%.1f" % tol], timeout=w + 20)
    err = parse_err(out)
    if rc == 0:
        print("    ✓ 到位（误差 %s mm）" % ("?" if err is None else "%.1f" % err))
        return EXIT_OK
    if accept_blocked and err is not None and 2.0 <= err <= 25.0:
        print("    ✓ 已咬住（到不了 %.1f mm，误差 %.1f mm —— 夹着物料时这属正常）"
              % (mm, err))
        return EXIT_OK
    print("    ✗ 夹爪未到位（rc=%d，误差 %s）"
          % (rc, "?" if err is None else "%.1f mm" % err))
    show_tail(out)
    return EXIT_FAIL


# ---------------------------------------------------------------- 流程
def do_pick(d, args, lift_after=True):
    """取：张开 → 抬到上方 → 下降 → 夹 → 抬起。"""
    # 抓取高度微调：示教时人眼判"指尖贴住物料"容易偏高几毫米。
    # 只作用于物料点，托盘点不跟着降（降了会往下压托盘底）。
    grasp_dz = args.grasp_dz_mm if args.grasp_dz_mm is not None \
        else float(d.get("grasp_dz_mm", 0.0))
    x, y, z_real, z_send = get_point(d, "objects", args.color, grasp_dz)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    lift = args.lift if args.lift is not None \
        else float(d.get("lift", standoff))
    width_mm = float(d.get("object_width", 0.05)) * 1000.0
    # 张开：用 grip_open_mm（默认 open_target 60mm ≡ grip 0.9）。
    # 60mm 对 50mm 物料单边 5mm 余量，链神 2026-11-10 确认够用；
    # 不必张到 68 全开 —— 只在取/放瞬间全开没意义，顶久了舵机发烫。
    open_mm = args.open_mm if args.open_mm is not None \
        else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))
    # 合拢目标 = 物宽 − 过盈量。
    # ⚠ 过盈量不能只给 2mm：这爪是**位置控制**，被物料挡住时实际停在物宽处，
    #   目标离实际越近，舵机使的力越小 —— 差 2mm 基本是"搭着"，一抬就掉。
    #   默认 10mm（50mm 物料 → 目标 40mm），舵机持续压迫才有夹持力。
    #   代价是夹着的时候一直在顶，别长时间保持（会堵转发热）。
    margin = args.close_margin if args.close_margin is not None \
        else float(d.get("grip_close_margin_mm", 10.0))
    close_mm = width_mm - margin

    # ① 先张开。合着爪下降 = 直接把指尖顶在物料上（会顶翻/压坏）
    if go_gap(open_mm, args, "取:张开", tol=5.0):
        return EXIT_FAIL
    if go_pose(x, y, z_send + standoff, args, "取:上方",
                wait=args.wait_fast) != EXIT_OK:
        print("    ⚠ 上方未完全到位，继续下降（差几毫米不影响夹取）")
    # 下降必须到位：它决定指尖落在物料哪一段
    if go_pose(x, y, z_send, args, "取:下降"):
        return EXIT_FAIL
    # ② 夹紧：夹着物料时到不了目标才是对的（误差 ≈ 过盈量）。
    #    tol 收到 12 让它在贴住物料后就返回，别等满 5s —— 舵机自己会继续压到位，
    #    CLI 只是不等它。等下一步「抬起」发出时，爪早就咬紧了。
    if go_gap(close_mm, args, "取:夹紧", tol=12.0, wait=3.0,
              accept_blocked=True):
        return EXIT_FAIL
    # ③ 抬起。
    #    lift_after=False（取完还要去托盘）时**跳过这一步**：单独垂直抬升正好撞在
    #    这台臂的短板上（远端 z 抬不过 ~0.10，垂直抬最吃力）。让下一步的横移顺带
    #    把高度带上去 —— MoveIt 一次规划斜线，比"先垂直抬到极限、再水平挪"顺得多，
    #    还少一个动作。
    #    lift_after=True（--no-place，取完要停住）才真的抬一下。
    if lift_after and go_pose(x, y, z_send + lift, args, "取:抬起",
                              wait=args.wait_fast) != EXIT_OK:
        print("    ⚠ 抬起未完全到位，继续（离桌即可横移）")
    print("  ✓ 已夹住 %s 物料（目标开度 %.1f mm，过盈 %.0f mm）"
          % (args.color, close_mm, margin))
    return EXIT_OK


def do_place(d, args):
    """放：抬到托盘上方 → 下降 → 张开 → 抬起。"""
    x, y, z_real, z_send = get_point(d, "pads", args.pad)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    lift = args.lift if args.lift is not None \
        else float(d.get("lift", standoff))
    open_mm = args.open_mm if args.open_mm is not None \
        else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))

    # 「上方」只是过渡：差几毫米不影响放，判失败就中止反而把流程卡死
    # （这台臂本来就只有 10~20mm 精度）。判断保留用于日志，不用于拦路。
    # ⚠ 这一步是**大跨度横移**（物料点 x≈0.35~0.55 → 托盘 x≈0.56，y 还差 450mm），
    # 必须给足时间。用 wait_fast(10s) 会在半路返回，紧接着的「下降」把横移打断，
    # 表现就是"还没到托盘上方就往下扎"。
    if go_pose(x, y, z_send + standoff, args, "放:上方",
                wait=args.wait) != EXIT_OK:
        print("    ⚠ 上方未完全到位，继续下降")
    # 「下降」必须到位：它决定物料落在哪
    if go_pose(x, y, z_send, args, "放:下降"):
        return EXIT_FAIL
    # 松开要开到比物宽大不少，靠摩擦带起物料就说明开得不够
    if go_gap(open_mm, args, "放:松爪", tol=5.0):
        return EXIT_FAIL
    if go_pose(x, y, z_send + lift, args, "放:抬起",
                wait=args.wait_fast) != EXIT_OK:
        print("    ⚠ 抬起未完全到位，继续（物料已放下）")
    print("  ✓ 已放到 %s 托盘" % args.pad)
    return EXIT_OK


def main():
    ap = argparse.ArgumentParser(
        description="固定位置抓放（不连相机，坐标来自 data/positions_6.yaml）")
    ap.add_argument("--color", default=None,
                    choices=("red", "green", "blue"),
                    help="取哪个物料（与 pick.py --color 同义）；--release 时可省略")
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
    ap.add_argument("--open-mm", type=float, default=None,
                    help="张开开度（mm）。取物前用它套物料，放完也用它松爪；"
                         "默认取 yaml 的 grip_open_mm，再默认 open_target %.0f"
                         % params.OPEN_TARGET_MM)
    ap.add_argument("--close-margin", type=float, default=None,
                    help="合拢过盈量（mm）：合拢目标 = 物宽 − 这个值。"
                         "默认取 yaml 的 grip_close_margin_mm（10）。"
                         "太小会夹不紧——这是位置控制，目标离实际越近力越小")
    ap.add_argument("--grasp-dz-mm", type=float, default=None,
                    help="抓取高度微调（mm，负=更低）。只作用于物料点，不动托盘。"
                         "默认取 yaml 的 grasp_dz_mm")
    ap.add_argument("--wait", type=float, default=25.0,
                    help="**关键动作**（下降）的最长等待（秒），默认 25")
    ap.add_argument("--wait-fast", type=float, default=10.0,
                    help="**过渡动作**（上方/抬起）的最长等待（秒），默认 10。"
                         "这台臂远端抬不过 z≈0.10，抬起本来就到不了，"
                         "等久只是白等")
    ap.add_argument("--tol", type=float, default=20.0,
                    help="到位容差（毫米）。实测残差 2~20mm（远端更大），"
                         "默认 20。别放到 30 以上 —— 会在运动途中判到位")
    ap.add_argument("--no-force", action="store_true",
                    help="不加 --force（点位 z 低于 bounds 下限时会被 CLI 拒绝）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要走的点，不动机械臂")
    ap.add_argument("--release", action="store_true",
                    help="**只张开夹爪放下物料**（不动臂），配 --no-home 则停在原地。"
                         "夹着物料想收手、或想手动摆位时用；不需要 --color")
    ap.add_argument("--json", action="store_true",
                    help="最后输出一行 JSON（给 arm_service 解析）")
    args = ap.parse_args()
    # --release 之外都必须给颜色
    if not args.release and not args.color:
        ap.error("--color 必填（只有 --release 可以不带）")
    args.pad = args.pad or args.color

    try:
        d = load_positions(args.yaml)
    except SystemExit as e:
        print("✗ %s" % e, file=sys.stderr)
        return EXIT_REJECT

    # ---- 应急出口：只松开夹爪。夹着物料想收手、或者想手动摆位时用。
    # 位置控制的爪夹着时会一直顶着舵机，没有"夹着但不发力"的状态，
    # 所以必须有一条明确的放手指令，否则人就卡在那儿了。
    if args.release:
        open_mm = args.open_mm if args.open_mm is not None \
            else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))
        print("松开夹爪 → %.1f mm（臂不动）" % open_mm)
        if args.dry_run:
            print("  --dry-run：不动机械臂")
            return EXIT_OK
        if go_gap(open_mm, args, "松开", tol=5.0):
            return EXIT_FAIL
        if args.no_home:
            print("✓ 已松开，停在原地")
            return EXIT_OK
        print("  → %-18s" % "回 home", flush=True)
        r, out = call(["home", "--wait", "180", "--tol", "25"], timeout=220)
        if r != 0:
            print("    ✗ 回 home 失败（夹爪已松开）")
            show_tail(out)
            return EXIT_FAIL
        print("✓ 已松开并回 home")
        return EXIT_OK

    for kind, name in (("objects", args.color), ("pads", args.pad)):
        if name not in d[kind]:
            print("✗ %s 里没有 %s.%s" % (args.yaml, kind, name), file=sys.stderr)
            return EXIT_REJECT

    # ---- 打印计划（--dry-run 到此为止）
    grasp_dz = args.grasp_dz_mm if args.grasp_dz_mm is not None \
        else float(d.get("grasp_dz_mm", 0.0))
    ox, oy, oz, oz_send = get_point(d, "objects", args.color, grasp_dz)
    px, py, pz, pz_send = get_point(d, "pads", args.pad)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    width_mm = float(d.get("object_width", 0.05)) * 1000.0
    open_mm = args.open_mm if args.open_mm is not None \
        else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))
    margin = args.close_margin if args.close_margin is not None \
        else float(d.get("grip_close_margin_mm", 10.0))
    z_off = float(d.get("z_cmd_offset_m", 0.0)) * 1000.0
    z_seed = float(d["objects"][args.color][2])
    print("固定位置抓放：取 %s → 放 %s" % (args.color, args.pad))
    print("  物料 (%.3f, %+.3f)  z: 示教 %.3f %+.0f微调 %+.0f补偿 → 发 %.3f"
          % (ox, oy, z_seed, grasp_dz, z_off, oz_send))
    print("  托盘 (%.3f, %+.3f)  z: 示教 %.3f          %+.0f补偿 → 发 %.3f"
          % (px, py, float(d["pads"][args.pad][2]), z_off, pz_send))
    lift = args.lift if args.lift is not None \
        else float(d.get("lift", standoff))
    print("  接近 %.0fmm / 抬起 %.0fmm（物高 %.0fmm）→ 张 %.0fmm 合 %.0fmm（过盈 %.0fmm）"
          % (standoff * 1000, lift * 1000,
             float(d.get("object_height", 0.0)) * 1000,
             open_mm, width_mm - margin, margin))
    if args.no_place:
        print("  顺序：张开 → 上方 → 下降 → 夹 → 抬起（停住）")
    else:
        print("  顺序：张开 → 上方 → 下降 → 夹 → 托盘上方（斜线，顺带抬高）"
              " → 下降 → 松开 → 抬起")
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
        rc = do_pick(d, args, lift_after=bool(args.no_place))
        if rc != EXIT_OK:
            return rc
        if args.no_place:
            margin = args.close_margin if args.close_margin is not None \
                else float(d.get("grip_close_margin_mm", 10.0))
            close_mm = float(d.get("object_width", 0.05)) * 1000.0 - margin
            print()
            print("✓ 取完，停在 %s 物料上方" % args.color)
            print("  ⚠ 夹爪正以 %.0f mm 为目标顶着舵机：位置控制没有「夹着不发力」"
                  "的状态，" % close_mm)
            print("    一直这么举着会持续出力发热，尽快往下走。")
            print("  下一步二选一：")
            print("    python3 scripts/pick_fixed.py --place-only --color %s"
                  "    # 放到托盘" % args.color)
            print("    python3 scripts/pick_fixed.py --release"
                  "                  # 松手，臂不动")
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
    _rc = EXIT_FAIL
    try:
        _rc = main()
    except KeyboardInterrupt:
        print("\n⛔ Ctrl+C：已中断（臂停住，夹爪保持当前开度）")
        _rc = 130
    finally:
        # 失败/中断时最容易被卡住：物料还在爪里，而人不知道该敲什么。
        # 这里把出口直接打出来，别让人去翻文档。
        if _rc != EXIT_OK:
            print("\n物料若还在夹爪里，按下面这条松手（臂不动）：")
            print("  python3 scripts/pick_fixed.py --release --no-home")
            print("臂位置不明先看一眼：  python3 scripts/roarm_cli.py where")
    sys.exit(_rc)

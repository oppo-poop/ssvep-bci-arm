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
_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "app" / "core" / "paths.py").is_file())
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from app.core import paths, params                    # noqa: E402

CLI = paths.CLI
POS_YAML = paths.POSITIONS_6

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


def grasp_dz_for(d, color, args):
    """抓取高度微调（mm）：命令行 > 按颜色 > 全局。

    为什么需要按颜色：三个物料是分开示教的，高度判断各自有偏差
    （2026-11-10 绿的就比另两个高 10mm）。全局一个数补不平，硬改示教值
    又会让"实测记录"失真，所以单独留一层。
    """
    if args.grasp_dz_mm is not None:
        return float(args.grasp_dz_mm)
    over = (d.get("grasp_dz_mm_by_color") or {}).get(color)
    if over is not None:
        return float(over)
    return float(d.get("grasp_dz_mm", 0.0))


def show_points(d, args):
    """列出所有示教点位，以及换算后的命令值。

    为什么要这个函数：物料被夹走之后，原位上就没东西了，光看桌面不知道
    "它原本该在哪"。这里把示教值和补偿值一起打出来，既能看到真实位置，
    也能看到实际会发给机械臂的数。
    """
    z_off = float(d.get("z_cmd_offset_m", 0.0)) * 1000.0
    g_base = float(d.get("grasp_dz_mm", 0.0))
    g_over = d.get("grasp_dz_mm_by_color") or {}
    p_dz = args.place_dz_mm if args.place_dz_mm is not None \
        else float(d.get("place_dz_mm", 0.0))
    print("示教点位：%s" % args.yaml)
    print("  z 命令补偿 %+.0f mm   物高 %.0f mm" %
          (z_off, float(d.get("object_height", 0.0)) * 1000))
    print("  抓取微调 %+.0f mm%s   放置微调 %+.0f mm"
          % (g_base,
             ("（按颜色覆盖：%s）" % "、".join(
                 "%s %+.0f" % (k, float(v)) for k, v in g_over.items()))
             if g_over else "",
             p_dz))
    print()
    print("物料（归位：--place-only --color X --to-object）")
    for name, xyz in d.get("objects", {}).items():
        x, y, z = (float(v) for v in xyz)
        dz = grasp_dz_for(d, name, args)
        print("  %-6s 实测 (%+.3f, %+.3f, %+.4f)  微调 %+.0f  命令 z=%+.4f"
              % (name, x, y, z, dz, z + (z_off + dz) / 1000.0))
    print()
    print("托盘（--place-only --color X）")
    for name, xyz in d.get("pads", {}).items():
        x, y, z = (float(v) for v in xyz)
        print("  %-6s 实测 (%+.3f, %+.3f, %+.4f)  微调 %+.0f  命令 z=%+.4f"
              % (name, x, y, z, p_dz, z + (z_off + p_dz) / 1000.0))
    print()


def read_gap_mm():
    """读夹爪两指实际间距（mm）。读不到返回 None。

    为什么要这个：`grip_deg` 的 success 只看"关节位移误差在不在容差内"，
    它分不清"夹住 50mm 物料"和"夹了个空"——空夹会合到底（4mm），
    而夹偏一半时误差 13mm 也可能落在 accept_blocked 的范围里。
    唯一可靠的证据是**读实际开度**：夹住 50mm 物料时它就该停在 50mm 左右。
    """
    import re as _re
    rc, out = call(["where"], timeout=30)
    if rc != 0:
        return None
    m = _re.search(r"joint7_left=([-\d.]+)", out)
    if not m:
        return None
    try:
        return params.gap_mm_from_joint(float(m.group(1)))
    except Exception:                                   # noqa: BLE001
        return None


# ---------------------------------------------------------------- 单步动作
def go_pose(x, y, z_send, args, what, wait=None, tol=None):
    """走一个位姿。z_send 是**已经补过** z 补偿的值。

    wait 默认用 args.wait（下降等关键动作），过渡动作传 args.wait_fast。
    为什么过渡动作要短：这台臂远端抬不过 z≈0.10，抬起那步**本来就到不了**，
    等满只是白等 —— 原来每趟多花 40 秒。
    """
    w = args.wait if wait is None else wait
    t = args.tol if tol is None else tol
    argv = ["pose", "%.4f" % x, "%.4f" % y, "%.4f" % z_send]
    argv += ["--wait", "%.0f" % w, "--tol", "%.1f" % t]
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
    grasp_dz = grasp_dz_for(d, args.color, args)
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
    # 下降必须真的下去：它决定指尖落在物料哪一段。
    # ⚠ tol 必须明显小于这段位移（上方→下降只有 standoff=30mm）。
    #   用默认 20 时，臂走 10mm 就满足"距目标 20mm"→ 在 0.100 处就夹了，
    #   物料在 0.070，结果夹空或夹到上沿（2026-11-10 就这么发生的）。
    #   小 tol 会超时报"未到位"（这台臂有 10~20mm 系统偏差），
    #   但 MoveIt 会把整条轨迹走完，位置是对的。
    if go_pose(x, y, z_send, args, "取:下降", wait=12.0, tol=6.0):
        return EXIT_FAIL
    # ② 夹紧：夹着物料时到不了目标才是对的（误差 ≈ 过盈量）。
    #    tol 收到 12 让它在贴住物料后就返回，别等满 5s —— 舵机自己会继续压到位，
    #    CLI 只是不等它。等下一步「抬起」发出时，爪早就咬紧了。
    if go_gap(close_mm, args, "取:夹紧", tol=12.0, wait=5.0,
              accept_blocked=True):
        return EXIT_FAIL
    # 物理验证：读实际开度，对不对得上物料宽度。
    # 这是唯一能区分"夹住了"和"夹了个空"的证据（见 read_gap_mm 的注释）。
    # 容差取 6mm：夹住 50mm 物料时读数在 48~52；夹空会读到 56~68（指间没东西，
    # 爪停在比物料宽的位置），6mm 能把这两种分开。实测 joint=-0.0031 → 57.8mm。
    gap = read_gap_mm()
    if gap is None:
        print("    ⚠ 读不到夹爪开度，无法确认是否夹住")
    elif abs(gap - width_mm) > 6.0:
        print("    ✗ 夹爪实际开度 %.1f mm，物宽 %.0f mm —— 没夹住或夹偏了"
              % (gap, width_mm))
        print("      可能原因：下降没到物料中段（调 grasp_dz_mm）、"
              "物料不在指间（x/y 偏）")
        return EXIT_FAIL
    else:
        print("    ✓ 夹住确认：实际开度 %.1f mm（物宽 %.0f mm）"
              % (gap, width_mm))
    # ③ 抬起。
    #    lift_after=False（取完还要去托盘）时**跳过这一步**：单独垂直抬升正好撞在
    #    这台臂的短板上（远端 z 抬不过 ~0.10，垂直抬最吃力）。让下一步的横移顺带
    #    把高度带上去 —— MoveIt 一次规划斜线，比"先垂直抬到极限、再水平挪"顺得多，
    #    还少一个动作。
    #    lift_after=True（--no-place，取完要停住）才真的抬一下。
    # tol 也要小于位移：抬 30mm 用 tol 20 时走 10mm 就判到了，
    # 看起来"根本没抬"。这里收到 10，让它至少真的离开桌面一个物料高度。
    if lift_after and go_pose(x, y, z_send + lift, args, "取:抬起",
                              wait=args.wait_fast, tol=10.0) != EXIT_OK:
        print("    ⚠ 抬起未完全到位，继续（离桌即可横移）")
    print("  ✓ 已夹住 %s 物料（目标开度 %.1f mm，过盈 %.0f mm）"
          % (args.color, close_mm, margin))
    return EXIT_OK


def do_place(d, args):
    """放：上方 → 下降 → 张开 → 抬起。

    目标是托盘（默认），还是物料自己的示教原位（--to-object）。
    """
    if args.to_object:
        # 归位：落到和抓取时同一个点（含 grasp_dz 微调），松爪后物料回到原处
        x, y, z_real, z_send = get_point(d, "objects", args.color,
                                         grasp_dz_for(d, args.color, args))
    else:
        # 托盘统一微调：三种托盘的示教高度一致，偏差也一致，一个数就够
        place_dz = args.place_dz_mm if args.place_dz_mm is not None \
            else float(d.get("place_dz_mm", 0.0))
        x, y, z_real, z_send = get_point(d, "pads", args.pad, place_dz)
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
    # 「下降」必须真的下去：它决定物料落在哪。tol 同「取:下降」，
    # 否则会在半空松爪 —— 用户看到的就是"没到 z 就张开了"。
    if go_pose(x, y, z_send, args, "放:下降", wait=12.0, tol=6.0):
        return EXIT_FAIL
    # 松开要开到比物宽大不少，靠摩擦带起物料就说明开得不够
    if go_gap(open_mm, args, "放:松爪", tol=5.0):
        return EXIT_FAIL
    # 到这里物料已经放下了 —— 立刻报，别让收尾动作吃掉这句话。
    # 用户的原话："都已经放下物料了还不显示完成"。
    print("  ✓ 物料已放到 %s 托盘（收尾：抬起/回位）" % args.pad, flush=True)
    # 抬起只在不回 home 时才需要：回 home 本身就是条斜线，会把臂带离托盘。
    # 单独"垂直抬起 30mm"在这台臂上要 18s 且经常到不了，纯属浪费。
    if args.no_home:
        if go_pose(x, y, z_send + lift, args, "放:抬起",
                   wait=args.wait_fast, tol=10.0) != EXIT_OK:
            print("    ⚠ 抬起未完全到位（物料已放下）")
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
                         "默认取 yaml 的 grasp_dz_mm（可按颜色用 grasp_dz_mm_by_color 覆盖）")
    ap.add_argument("--place-dz-mm", type=float, default=None,
                    help="放置高度微调（mm，负=更低）。只作用于托盘，不动物料。"
                         "默认取 yaml 的 place_dz_mm")
    ap.add_argument("--wait", type=float, default=25.0,
                    help="**关键动作**（下降）的最长等待（秒），默认 25")
    ap.add_argument("--wait-fast", type=float, default=25.0,
                    help="**过渡动作**（上方/抬起）的最长等待（秒），默认 25。"
                         "实测抬升只有 1.7mm/s，抬 30mm 要 18s —— 给 10s 会在"
                         "半路返回，下一步立刻打断，表现就是「没抬起来就去托盘」")
    ap.add_argument("--tol", type=float, default=20.0,
                    help="到位容差（毫米）。实测残差 2~20mm（远端更大），"
                         "默认 20。别放到 30 以上 —— 会在运动途中判到位")
    ap.add_argument("--no-force", action="store_true",
                    help="不加 --force（点位 z 低于 bounds 下限时会被 CLI 拒绝）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要走的点，不动机械臂")
    ap.add_argument("--goto", metavar="COLOR",
                    choices=("red", "green", "blue"),
                    help="**只定位**：移到该物料的示教点、张开夹爪、停住。"
                         "不抓、不放、不回 home —— 用来手动放物料或核对位置")
    ap.add_argument("--goto-pad", metavar="COLOR",
                    choices=("red", "green", "blue"),
                    help="同上，目标是该颜色的托盘")
    ap.add_argument("--show", action="store_true",
                    help="**列出所有示教点位**（物料 + 托盘），含补偿后的命令值，"
                         "不动机械臂。不知道物料原来在哪就用这个")
    ap.add_argument("--to-object", action="store_true",
                    help="把物料放回**它自己的示教原位**（objects），而不是托盘。"
                         "配 --place-only：物料已在爪里、想归位")
    ap.add_argument("--release", action="store_true",
                    help="**只张开夹爪放下物料**（不动臂），配 --no-home 则停在原地。"
                         "夹着物料想收手、或想手动摆位时用；不需要 --color")
    ap.add_argument("--json", action="store_true",
                    help="最后输出一行 JSON（给 arm_service 解析）")
    args = ap.parse_args()
    # --release（松手）和 --show（列点位）之外都必须给颜色
    if not (args.release or args.show or args.goto or args.goto_pad) \
            and not args.color:
        ap.error("--color 必填（--release / --show / --goto 可以不带）")
    args.pad = args.pad or args.color

    try:
        d = load_positions(args.yaml)
    except SystemExit as e:
        print("✗ %s" % e, file=sys.stderr)
        return EXIT_REJECT

    # ---- 只查点位：不动机械臂 ----
    if args.show:
        show_points(d, args)
        return EXIT_OK

    # ---- 只定位：移到某个示教点 + 张开夹爪 + 停住 ----
    # 用途是"手动把物料放进去"或"核对示教位置对不对"，所以：
    #   · 不加 standoff（要的是那个点本身，不是点上方的悬停）
    #   · 张开但不夹、不下降、不回 home
    if args.goto or args.goto_pad:
        name = args.goto or args.goto_pad
        kind = "objects" if args.goto else "pads"
        if name not in d[kind]:
            print("✗ %s 里没有 %s.%s" % (args.yaml, kind, name), file=sys.stderr)
            return EXIT_REJECT
        gdz = args.grasp_dz_mm if args.grasp_dz_mm is not None \
            else float(d.get("grasp_dz_mm", 0.0))
        gx, gy, gz, gz_send = get_point(d, kind, name,
                                        gdz if kind == "objects" else 0.0)
        open_mm = args.open_mm if args.open_mm is not None \
            else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))
        print("定位到 %s.%s → (%.3f, %+.3f) 命令 z=%.3f，张开 %.0f mm 后停住"
              % (kind, name, gx, gy, gz_send, open_mm))
        if args.dry_run:
            print("  --dry-run：不动机械臂")
            return EXIT_OK
        # 先张开再移动：合着爪移过去，指尖可能顶到旁边的物料
        if go_gap(open_mm, args, "张开", tol=5.0):
            return EXIT_FAIL
        if go_pose(gx, gy, gz_send, args, "定位") != EXIT_OK:
            print("    ⚠ 没完全到位，但位置已接近，可以手动放料")
        print()
        print("✓ 已到位并张开，物料放进两指之间即可。之后：")
        print("    python3 scripts/roarm_cli.py grip close --wait 8 --tol 5")
        print("        → 合上夹住（空爪会合到底；有物料会压住，误差 2~25mm 属正常）")
        print("    python3 scripts/pick_fixed.py --release --no-home")
        print("        → 只是松开，不动臂")
        return EXIT_OK

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

    need = [("objects", args.color)]
    if not args.to_object:               # 归位到物料原位，不需要托盘
        need.append(("pads", args.pad))
    for kind, name in need:
        if name not in d[kind]:
            print("✗ %s 里没有 %s.%s" % (args.yaml, kind, name), file=sys.stderr)
            return EXIT_REJECT

    # ---- 打印计划（--dry-run 到此为止）
    grasp_dz = grasp_dz_for(d, args.color, args)
    ox, oy, oz, oz_send = get_point(d, "objects", args.color, grasp_dz)
    place_dz = args.place_dz_mm if args.place_dz_mm is not None \
        else float(d.get("place_dz_mm", 0.0))
    if args.to_object:
        px, py, pz, pz_send = get_point(d, "objects", args.color, grasp_dz)
    else:
        px, py, pz, pz_send = get_point(d, "pads", args.pad, place_dz)
    standoff = args.standoff if args.standoff is not None \
        else float(d.get("standoff", 0.05))
    width_mm = float(d.get("object_width", 0.05)) * 1000.0
    open_mm = args.open_mm if args.open_mm is not None \
        else float(d.get("grip_open_mm", params.OPEN_TARGET_MM))
    margin = args.close_margin if args.close_margin is not None \
        else float(d.get("grip_close_margin_mm", 10.0))
    z_off = float(d.get("z_cmd_offset_m", 0.0)) * 1000.0
    z_seed = float(d["objects"][args.color][2])
    print("固定位置抓放：取 %s → %s"
          % (args.color,
             "归位（放回原位）" if args.to_object else "放 %s" % args.pad))
    print("  物料 (%.3f, %+.3f)  z: 示教 %.3f %+.0f微调 %+.0f补偿 → 发 %.3f"
          % (ox, oy, z_seed, grasp_dz, z_off, oz_send))
    if args.to_object:
        print("  归位 (%.3f, %+.3f)  z: 示教 %.3f %+.0f微调 %+.0f补偿 → 发 %.3f"
              "  ← 放回 %s 物料原位"
              % (px, py, float(d["objects"][args.color][2]), grasp_dz, z_off,
                 pz_send, args.color))
    else:
        print("  托盘 (%.3f, %+.3f)  z: 示教 %.3f %+.0f微调 %+.0f补偿 → 发 %.3f"
              % (px, py, float(d["pads"][args.pad][2]), place_dz, z_off,
                 pz_send))
    lift = args.lift if args.lift is not None \
        else float(d.get("lift", standoff))
    print("  接近 %.0fmm / 抬起 %.0fmm（物高 %.0fmm）→ 张 %.0fmm 合 %.0fmm（过盈 %.0fmm）"
          % (standoff * 1000, lift * 1000,
             float(d.get("object_height", 0.0)) * 1000,
             open_mm, width_mm - margin, margin))
    if args.no_place:
        print("  顺序：张开 → 上方 → 下降 → 夹 → 抬起（停住）")
    else:
        tail = " → 抬起" if args.no_home else " → 回 home"
        print("  顺序：张开 → 上方 → 下降 → 夹 → %s（斜线，顺带抬高）"
              " → 下降 → 松开%s"
              % ("原位上方" if args.to_object else "托盘上方", tail))
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

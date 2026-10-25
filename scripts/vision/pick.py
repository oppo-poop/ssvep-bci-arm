#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉引导抓取：拍照 -> 检测物料 -> 基座坐标 -> 加夹爪偏移 -> 下降/闭合/抬起。

用法（在 vision/ 目录下，用 .venv_astra 的 Python 3.12）:

    .venv_astra/bin/python scripts/vision_pick.py --dry-run      # 只算目标点，不动机械臂
    .venv_astra/bin/python scripts/vision_pick.py                # 真抓
    .venv_astra/bin/python scripts/vision_pick.py --tag p2       # 抓完给这次记录打标签
    .venv_astra/bin/python scripts/vision_pick.py --dz -0.004    # 抓取高度微调 -4mm

为什么用 subprocess 调 roarm_cli.py 而不直接 import rclpy:
    vision 的 venv 是 Python 3.12，把 rclpy 装进 venv 容易和系统 ROS 的
    PYTHONPATH 打架；roarm_cli.py 在系统 Python 下已验证可用，隔进程调用最稳，
    而且能直接复用它「到位校验 + 退出码」的契约。

前置：机械臂 ROS 栈已启动（bash scripts/robot_up.sh），相机已插好。
"""
import argparse
import csv
import datetime
import json
import os
import re
import shlex
import subprocess
import sys
import time

import numpy as np
import cv2

from camera import grab_frame, load_scene_z
from project import pixel_to_base

HERE = os.path.dirname(os.path.abspath(__file__))   # scripts/vision/
SCRIPTS = os.path.dirname(HERE)                     # scripts/
ROOT = os.path.dirname(SCRIPTS)                     # 项目根
WS_SETUP = os.path.join(ROOT, "star-arm-moveit2", "install", "setup.bash")
CLI = os.path.join(SCRIPTS, "roarm_cli.py")

DATA = os.path.join(ROOT, "data", "vision")
OFFSET_JSON = os.path.join(DATA, "grasp_offset.json")
LOG_CSV = os.path.join(DATA, "pick_log.csv")
SCENE_YAML = os.path.join(ROOT, "data", "scene_real.yaml")

#: 默认末端姿态（world -> link6，零位实测），与 roarm_cli.py 保持一致
DEFAULT_QUAT = (0.5, -0.5, 0.5, -0.5)

#: 单次成功抓取反推的夹爪偏移（夹爪系，mm）。有 grasp_offset.json 时以文件为准。
#: 2026-10-21 用 8 条 grasp_ok=1 的记录最小二乘反解（剔除残差最大的 2 条后 RMS 1.99 mm）。
#: 原占位值 (16.6, 23.6, 86.2) 与之相差 32 mm —— 那是"重标前的占位值"，抓空的原因。
DEFAULT_OFFSET_MM = (40.3, 35.0, 67.8)

#: roarm_cli.py 里的任务工作区（米），不是机械极限
BOUNDS = {"x": (0.278, 0.529), "y": (-0.324, 0.113), "z": (0.074, 0.438)}

#: 物料到基座的水平距离合理范围（米）。背景里的蓝色物体算出来动辄几米，
#: 用它挡掉；2026-10-21 实际踩到：背景蓝色托盘算出 10.8 米。
R_MIN, R_MAX = 0.10, 0.75


# ---------------------------------------------------------------- 数学
def quat_to_R(q):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def load_offset():
    """返回 (offset_mm, 说明字符串)。"""
    if os.path.isfile(OFFSET_JSON):
        try:
            d = json.load(open(OFFSET_JSON, encoding="utf8"))
            off = np.array(d["offset_mm"], dtype=np.float64)
            return off, "grasp_offset.json (n=%s)" % d.get("n", "?")
        except Exception as e:
            print("⚠ grasp_offset.json 读不出来(%s)，改用默认值" % e)
    return np.array(DEFAULT_OFFSET_MM, dtype=np.float64), "默认值(单次抓取反推)"


def out_of_bounds(xyz):
    bad = []
    for name, v in zip(("x", "y", "z"), xyz):
        lo, hi = BOUNDS[name]
        if not (lo <= v <= hi):
            bad.append("%s=%.1fmm" % (name, v * 1000))
    return bad


def blue_candidates(img, min_area=500.0):
    """画面里所有蓝色块，返回 [(uv, area), ...]（面积从大到小）。

    为什么不用 grasp_xy.detect_blue：它只返回"最大的那块"，
    背景里一块蓝色托盘就能把真正的小物料挤掉（2026-10-21 踩到）。
    这里把所有候选都交出来，交给物理距离再筛。
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([100, 120, 60]), np.array([130, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        out.append(((m["m10"] / m["m00"], m["m01"] / m["m00"]), a))
    out.sort(key=lambda t: -t[1])
    return out, mask


# ---------------------------------------------------------------- 调 CLI
def find_ros_setup():
    for d in ("jazzy", "humble", "iron", "rolling"):
        p = "/opt/ros/%s/setup.bash" % d
        if os.path.isfile(p):
            return p
    return None


def run_cli(args, timeout=45):
    """调 roarm_cli.py，返回 (退出码, 合并输出)。自动 source ROS + 工作区。"""
    ros = find_ros_setup()
    if ros is None:
        sys.exit("找不到 /opt/ros/*/setup.bash")
    if not os.path.isfile(WS_SETUP):
        sys.exit("工作区没构建：%s 不存在" % WS_SETUP)
    # 每次调用都是新进程，DDS 发现默认只等 2 秒；偶尔会不够，表现为
    # "⚠ 这些话题没有订阅者：pose、gripper"（2026-10-21 踩到，夹爪没闭上）。
    # 统一放宽到 5 秒，并且放在子命令**前面**，避免 argparse 混淆。
    full = list(args)
    if "--timeout" not in full:
        full = ["--timeout", "5.0"] + full
    inner = "set +u; source %s; source %s; exec /usr/bin/python3 %s %s" % (
        shlex.quote(ros), shlex.quote(WS_SETUP), shlex.quote(CLI),
        " ".join(shlex.quote(str(a)) for a in full))
    try:
        p = subprocess.run(["bash", "-c", inner], capture_output=True,
                           text=True, timeout=timeout)
        out = (p.stdout or "") + (p.stderr or "")
        return p.returncode, out.strip()
    except subprocess.TimeoutExpired:
        return 124, "子进程超时 %ss" % timeout


def show(out):
    for line in out.splitlines():
        if line.strip():
            print("    " + line.strip())


RE_ACTUAL = re.compile(r"实际\s*x=\s*([-\d.]+)\s*y=\s*([-\d.]+)\s*z=\s*([-\d.]+)")
RE_POS = re.compile(r"当前位置\s*x=\s*([-\d.]+)\s*y=\s*([-\d.]+)\s*z=\s*([-\d.]+)")
RE_ERR = re.compile(r"误差\s*([-\d.]+)\s*mm")


def parse_actual(out):
    """从输出里抠出实测末端位置（mm）。"""
    m = RE_ACTUAL.findall(out)
    if not m:
        return None
    x, y, z = (float(v) for v in m[-1])
    return np.array([x * 1000.0, y * 1000.0, z * 1000.0])


def parse_err(out):
    m = RE_ERR.findall(out)
    return float(m[-1]) if m else None


def read_pose():
    """单独调一次 where，拿当前稳定位置（mm）。拿不到返回 None。"""
    rc, out = run_cli(["where"], timeout=20)
    m = RE_POS.findall(out)
    if not m:
        return None
    x, y, z = (float(v) for v in m[-1])
    return np.array([x * 1000.0, y * 1000.0, z * 1000.0])


def cli_pose(xyz_mm, wait, tol, force, allow_far=False):
    x, y, z = (v / 1000.0 for v in xyz_mm)
    # 四元数必须写在 --wait/--tol 前面：roarm_cli 的位置参数是 nargs="*"，
    # 混合「位置参数 + 选项 + 位置参数」会被 argparse 判成 unrecognized。
    args = ["pose", "%.5f" % x, "%.5f" % y, "%.5f" % z]
    args += ["%g" % v for v in DEFAULT_QUAT]
    args += ["--wait", "%.1f" % wait, "--tol", "%.1f" % tol]
    bad = out_of_bounds((x, y, z))
    far = [b for b in bad if not b.startswith("z=")]
    if far and not allow_far:
        # x/y 超界说明物料根本不在可达范围，硬发只会让机械臂去撞限位
        print("  ⛔ 超出工作区（%s），不硬发" % "、".join(far))
        return 3, None
    if bad:
        if not force:
            print("  ⚠ 超工作区 %s，但 --no-force 已指定 -> 放弃这一步" % "、".join(bad))
            return 2, None
        # z 低于下限是抓取的正常姿态（夹爪要贴到台面），这里加 --force 是预期行为
        print("  ⚠ 超工作区 %s，自动加 --force" % "、".join(bad))
        args.append("--force")
    rc, out = run_cli(args, timeout=wait + 25)
    show(out)
    act = parse_actual(out)
    if rc == 0:
        # --wait 里 tol=15 时可能在"路过"目标区就判到位，报的位置不是最终落点。
        # 停 1 秒再读一次 TF，才是真正稳住的位置（标偏移要用它）。
        time.sleep(1.0)
        settled = read_pose()
        if settled is not None:
            act = settled
    return rc, act


def cli_pose_zfix(target, wait, tol, force, allow_far=False, iters=1,
                  dead=4.0, max_step=8.0):
    """移动到一个点，z 落点偏超过 dead 就补发一次。

    为什么需要：`repeatability.csv` 里同一个目标点 10 次落点极差只有 **2.5mm**
    （重复性很好），但落点整体比目标偏 9~10mm（绝对精度差，且随姿态变）。
    也就是"每次都很稳地停在错的地方"。量一次偏差、把目标反着挪一点再发一次，
    落点能进 ±5mm。

    安全约束（2026-10-21 踩到才加的）：
      * 一次只补 max_step=8mm，避免一脚踩过头
      * 补偿后如果反而更远，就回退到第一次的位置
      * **不管补没补上，只要有位置就返回 0** —— 抓取看的是最终落点，
        不是"有没有进容差"。之前在这里 return 非零，导致明明能抓却中止。
    """
    rc, act = cli_pose(target, wait, tol, force, allow_far)
    if act is None:
        return rc, None
    for i in range(iters):
        dz = target[2] - act[2]
        if abs(dz) <= dead:
            break
        step = max(-max_step, min(max_step, dz))
        fix = np.array([target[0], target[1], target[2] + step])
        print("  z 偏 %+.1f mm → 目标改发 z=%.1f（第 %d 次补偿）"
              % (dz, fix[2], i + 1))
        rc2, act2 = cli_pose(fix, wait, tol, force, allow_far)
        if act2 is None:
            print("  ⚠ 补偿那一步没拿到位置，沿用上一次")
            break
        if abs(target[2] - act2[2]) >= abs(dz):
            print("  ⚠ 补偿后反而更远（%+.1f mm），回退"
                  % (target[2] - act2[2]))
            break
        act = act2
    return 0, act


def cli_grip(cmd, wait, tol=5.0):
    rc, out = run_cli(["grip", cmd, "--wait", "%.1f" % wait, "--tol", "%.1f" % tol],
                      timeout=wait + 15)
    show(out)
    return rc, parse_err(out)


# ---------------------------------------------------------------- 记录
CSV_HEADER = ["ts", "tag", "u", "v", "obj_x", "obj_y", "obj_z",
              "tgt_x", "tgt_y", "tgt_z", "act_x", "act_y", "act_z",
              "qx", "qy", "qz", "qw", "close_err", "grasp_ok", "note"]


def append_log(row):
    new = not os.path.isfile(LOG_CSV)
    with open(LOG_CSV, "a", newline="", encoding="utf8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER)
        if new:
            w.writeheader()
        w.writerow(row)


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="视觉引导抓取（一条命令）")
    ap.add_argument("--dry-run", action="store_true", help="只算目标点并出图，不动机械臂")
    ap.add_argument("--u", type=float, default=None, help="手动指定像素 u")
    ap.add_argument("--v", type=float, default=None, help="手动指定像素 v")
    ap.add_argument("--z-grasp", type=float, default=None, help="抓取平面高度（米）")
    ap.add_argument("--dz", type=float, default=0.0, help="抓取高度微调（米），负数更低")
    ap.add_argument("--standoff", type=float, default=0.06, help="预抓取抬高（米），默认 60mm")
    ap.add_argument("--min-pre-z", type=float, default=0.13,
                    help="预抓取最低高度（米），默认 0.13")
    ap.add_argument("--tol", type=float, default=15.0,
                    help="到位容差（毫米）。机械臂绝对定位本身 10~30mm，"
                         "设 5 会一直判失败；预抓取/抬起用 15 够，抓取点可单独收紧")
    ap.add_argument("--wait", type=float, default=30.0, help="每个位姿最长等待（秒）")
    ap.add_argument("--tag", default="", help="给这次记录打标签")
    ap.add_argument("--offset", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "Z"),
                    help="临时指定夹爪偏移（夹爪系 mm），覆盖 grasp_offset.json")
    ap.add_argument("--no-lift", action="store_true", help="闭合后不要抬起来")
    ap.add_argument("--no-place", action="store_true",
                    help="抓成功后不要放回原处（默认会放回去，方便连续试）")
    ap.add_argument("--no-home", action="store_true",
                    help="跳过开始前的复位（默认会先回 home，避免从上次失败的位置起步）")
    ap.add_argument("--no-zfix", action="store_true",
                    help="不做 z 落点补偿（默认会量一次偏差再补发一次，把落点压进 ±3mm）")
    ap.add_argument("--no-force", action="store_true", help="目标超工作区时不自动加 --force")
    ap.add_argument("--allow-far", action="store_true",
                    help="允许 x/y 超出任务工作区（默认拒绝，防止臂去撞限位）")
    ap.add_argument("--image", default=None,
                    help="叠加结果图保存路径")
    args = ap.parse_args()

    # ---- 1) 标定参数
    d = np.load(os.path.join(DATA, "T_base_cam.npz"))
    T_base_cam = d["T_base_cam"]
    Kd = np.load(os.path.join(DATA, "cam_intrinsics.npz"))
    K, dist = Kd["K"], Kd["dist"]
    z_g = (args.z_grasp if args.z_grasp is not None else load_scene_z(SCENE_YAML)) + args.dz
    if args.offset:
        off_mm = np.array(args.offset, dtype=np.float64)
        off_src = "命令行 --offset"
    else:
        off_mm, off_src = load_offset()
    R = quat_to_R(DEFAULT_QUAT)

    # ---- 2) 先回 home 再拍
    # 臂如果停在物料上方，会正好把物料挡在身后（2026-10-21 实际踩到：
    # 抓空后臂悬在 z=116mm，拍到的画面里物料被两指完全遮住）。
    if args.dry_run:
        print("dry-run：不动机械臂。若画面里找不到物料，先手动回 home：")
        print("  ./scripts/roarm_cli.py home --wait 20")
    elif args.no_home:
        print("[0/6] --no-home：跳过复位，直接拍照")
    else:
        print("[0/6] 复位到 home（先让臂离开相机视线）")
        # home 只是"挪开让相机看得见物料"，不需要精度；而且这台臂有固定偏差
        # （目标 365 → 落点 354，x 还偏 +4mm），用默认 5mm 容差必然判失败，
        # 还会白等满 30 秒。所以这里单独放宽到 25mm。
        rc, out = run_cli(["home", "--wait", "%.1f" % args.wait, "--tol", "25.0"],
                          timeout=args.wait + 25)
        show(out)
        if rc != 0:
            print("  ⚠ home 没进 25mm（臂有固定偏差属正常），继续下一步")

    # ---- 3) 拍照 + 检测
    print("拍照…")
    try:
        img = grab_frame()
    except SystemExit as e:
        print("❌ %s" % e)
        return 1
    cands = []          # [(uv, area, P_base)]
    if args.u is not None and args.v is not None:
        uv = (args.u, args.v)
        print("用指定像素 (%.0f, %.0f)" % uv)
        P = pixel_to_base(uv[0], uv[1], T_base_cam, K, dist, z_g)
        if P is None:
            print("❌ 该像素与 z=%.3f 平面没有交点（视线朝上？）" % z_g)
            return 1
        P_mm = P * 1000.0
    else:
        raw, _mask = blue_candidates(img)
        if not raw:
            print("❌ 画面里没有蓝色区域（可用 --u --v 指定像素，或调颜色阈值）")
            cv2.imwrite(args.image, img)
            return 1
        print("画面里 %d 个蓝色区域：" % len(raw))
        for uv_i, a in raw:
            P_i = pixel_to_base(uv_i[0], uv_i[1], T_base_cam, K, dist, z_g)
            if P_i is None:
                print("  (%4.0f,%4.0f) 面积%6.0f  视线与平面无交点"
                      % (uv_i[0], uv_i[1], a))
                continue
            r = float(np.hypot(P_i[0], P_i[1]))
            good = R_MIN <= r <= R_MAX
            if good:
                cands.append((uv_i, a, P_i))
            print("  (%4.0f,%4.0f) 面积%6.0f  基座%+7.0f,%+7.0f mm  距底座%5.0f mm  %s"
                  % (uv_i[0], uv_i[1], a, P_i[0] * 1000, P_i[1] * 1000, r * 1000,
                     "✓" if good else "✗ 不在 %.2f~%.2f m 内" % (R_MIN, R_MAX)))
        if not cands:
            print("❌ 蓝色区域都不在合理距离内（%.2f~%.2f m），大概率认错东西了"
                  % (R_MIN, R_MAX))
            print("   物料放台面上、离底座 0.1~0.75 m；也可用 --u --v 手动指定像素")
            cv2.imwrite(args.image, img)
            return 1
        uv, area, P = cands[0]
        P_mm = P * 1000.0
        print("选用 像素 (%.0f, %.0f)，面积 %.0f" % (uv[0], uv[1], area))

    # ---- 4) 夹爪偏移
    d_mm = R @ off_mm
    tgt = P_mm + d_mm
    pre_z = max(tgt[2] + args.standoff * 1000.0, args.min_pre_z * 1000.0)
    pre = np.array([tgt[0], tgt[1], pre_z])

    print("\n──────────────── 抓取计划 ────────────────")
    print("  物料基座坐标   x=%+7.1f  y=%+7.1f  z=%+7.1f mm" % tuple(P_mm))
    print("  夹爪偏移(局部) x=%+7.1f  y=%+7.1f  z=%+7.1f mm   ← %s"
          % (off_mm[0], off_mm[1], off_mm[2], off_src))
    print("  偏移(世界系)   x=%+7.1f  y=%+7.1f  z=%+7.1f mm" % tuple(d_mm))
    print("  link6 抓取点   x=%+7.1f  y=%+7.1f  z=%+7.1f mm" % tuple(tgt))
    print("  预抓取点       x=%+7.1f  y=%+7.1f  z=%+7.1f mm" % tuple(pre))
    far = [b for b in out_of_bounds(tuple(v / 1000.0 for v in tgt))
           if not b.startswith("z=")]
    if far:
        print("  ⛔ 抓取点超出工作区：%s" % "、".join(far))
        print("     夹爪偏移会让 link6 再往前 86mm，物料得往底座方向挪。")
    print("──────────────────────────────────────────\n")

    view = img.copy()
    cv2.drawMarker(view, (int(uv[0]), int(uv[1])), (0, 0, 255),
                   cv2.MARKER_CROSS, 40, 3)
    cv2.putText(view, "obj (%.0f,%.0f)" % uv, (int(uv[0]) + 18, int(uv[1]) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(view, "base %+.0f,%+.0f mm  ->  link6 %+.0f,%+.0f,%+.0f" %
                (P_mm[0], P_mm[1], tgt[0], tgt[1], tgt[2]),
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
    cv2.imwrite(args.image, view)
    print("叠加图已存 %s" % args.image)

    if args.dry_run:
        print("\n--dry-run：不动机械臂。确认上面的点没问题就去掉这个参数。")
        return 0

    if far and not args.allow_far:
        print("\n⛔ 抓取点超出工作区 %s，已中止（机械臂没动）。" % "、".join(far))
        print("   把物料往底座方向挪，或加 --allow-far 强行试。")
        return 2

    # ---- 5) 执行
    print("\n3 秒后开始动作，手离机械臂远一点…")
    for i in (3, 2, 1):
        print("  %d" % i, flush=True)
        time.sleep(1)

    rec = dict(ts=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
               tag=args.tag, u="%.1f" % uv[0], v="%.1f" % uv[1],
               obj_x="%.2f" % P_mm[0], obj_y="%.2f" % P_mm[1], obj_z="%.2f" % P_mm[2],
               tgt_x="%.2f" % tgt[0], tgt_y="%.2f" % tgt[1], tgt_z="%.2f" % tgt[2],
               qx=DEFAULT_QUAT[0], qy=DEFAULT_QUAT[1], qz=DEFAULT_QUAT[2],
               qw=DEFAULT_QUAT[3], close_err="", grasp_ok="0", note="")
    act = None

    def bail(step):
        rec["note"] = "在「%s」失败" % step
        append_log(rec)
        print("\n❌ %s 失败，已中止。记录写进 %s" % (step, LOG_CSV))
        return 1

    print("\n[1/6] 张开夹爪")
    rc, _ = cli_grip("open", 20)
    if rc != 0:
        print("  ⚠ 张开没确认到位，继续（多数情况下已经张开）")

    print("\n[2/6] 移到预抓取点（抬高 %.0f mm）"
          % (pre_z - tgt[2]))
    rc, act = cli_pose(pre, args.wait, args.tol, not args.no_force, args.allow_far)
    if rc == 3:
        print("\n⛔ 目标超出可达范围，已中止。把物料往底座方向挪 8~10cm 再试。")
        rec["note"] = "目标超出工作区"
        append_log(rec)
        return 2
    if rc != 0:
        return bail("预抓取移动")

    print("\n[3/6] 下降到抓取点")
    if args.no_zfix:
        rc, act = cli_pose(tgt, args.wait, args.tol, not args.no_force, args.allow_far)
    else:
        rc, act = cli_pose_zfix(tgt, args.wait, args.tol, not args.no_force,
                                args.allow_far)
    # 只有"完全没拿到位置"才中止；没进容差但停在可用高度就继续抓
    if act is None:
        return bail("下降")
    rec["act_x"], rec["act_y"], rec["act_z"] = ("%.2f" % v for v in act)
    e = np.linalg.norm(act - tgt)
    dz = act[2] - tgt[2]
    print("  实测与目标差 %.1f mm（z 偏 %+.1f）" % (e, dz))
    if abs(dz) > 12:
        print("  ⚠ z 偏得多，夹爪会落在比预期%s %.0f mm 的位置，注意别蹭台面"
              % ("低" if dz < 0 else "高", abs(dz)))

    print("\n[4/6] 闭合夹爪")
    rc, err = cli_grip("close", 20)
    rec["close_err"] = "" if err is None else "%.2f" % err
    if err is None:
        # 一个数据都没收到，说明命令没送达（DDS 发现失败）或 joint_states 断了。
        # 这时候抬起也是空手，不如停在原地，别白跑一遍。
        rec["note"] = "夹爪命令没送达"
        append_log(rec)
        print("\n❌ 夹爪没有响应（命令没送达 / joint_states 断了），停在原地不再抬起。")
        print("   先确认：ros2 topic echo /joint_states --once")
        return 1
    if err is not None:
        if rc == 0:
            print("  ⚠ 夹爪完全闭合（误差 %.1fmm）——大概率抓空了" % err)
        elif 2.0 <= err <= 22.0:
            print("  ✅ 夹爪被物体挡住 %.1f mm，疑似抓稳" % err)
            rec["grasp_ok"] = "1"
        else:
            print("  ? 闭合误差 %.1fmm，状态可疑" % err)
    if rc != 0 and rec["grasp_ok"] == "0":
        rec["note"] = "夹爪闭合异常"

    if args.no_lift:
        append_log(rec)
        print("\n--no-lift：停在抓取点。记录已写入 %s" % LOG_CSV)
        return 0

    print("\n[5/6] 抬起")
    # 带物抬升时臂会下沉 15mm 左右，用 15 的容差必然判失败、白等 30 秒；
    # 抬起只是"离开物料"，不需要精度，放宽到 22mm。
    rc, _ = cli_pose(pre, args.wait, max(args.tol, 22.0), not args.no_force,
                     args.allow_far)
    if rc != 0:
        print("  ⚠ 抬起没确认到位，检查是否撞到东西")

    append_log(rec)
    if rec["grasp_ok"] == "1":
        print("\n✅ 抓取完成，夹爪被挡住 %.1f mm，疑似抓稳" % (err or 0))
        print("   想标夹爪偏移：换 3~5 个位置各抓一次，再跑 gripper_offset_calib.py")
    else:
        print("\n⚠ 动作走完了，但夹爪没夹到东西（闭合误差 %.1f mm）"
              % (err if err is not None else -1))
        print("   常见原因：抓取高度(z_g)不对 / 物料比预想的矮 / 臂落了 10~15mm")
        print("   先看 output/pick_view.png 里夹爪和物料的相对位置")
        return 0

    if args.no_place:
        print("   --no-place：物料留在夹爪里")
        return 0

    # ---- 6) 放回原处：抓完不落地的话，下一轮就没得抓了
    print("\n[6/6] 放回原处")
    back = np.array([tgt[0], tgt[1], tgt[2] + 8.0])
    rc, act_b = cli_pose(back, args.wait, args.tol, not args.no_force, args.allow_far)
    if act_b is None:
        print("  ⚠ 降不到放置高度，物料还在夹爪里，请手动处理（或 --no-place）")
        return 1
    cli_grip("open", 15)
    print("  已松爪，物料落回台面")
    cli_pose(pre, args.wait, max(args.tol, 22.0), not args.no_force, args.allow_far)
    return 0


if __name__ == "__main__":
    sys.exit(main())

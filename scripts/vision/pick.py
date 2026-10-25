#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉引导抓取：拍照 -> 检测物料 -> 基座坐标 -> 加夹爪偏移 -> 下降/闭合/抬起。

用法（在 vision/ 目录下，用 .venv_astra 的 Python 3.12）:

    .venv_astra/bin/python scripts/vision_pick.py --dry-run      # 只算目标点，不动机械臂
    .venv_astra/bin/python scripts/vision_pick.py                # 真抓（弹窗点选目标）
    .venv_astra/bin/python scripts/vision_pick.py --auto         # 不点选，自动识别蓝色
    .venv_astra/bin/python scripts/vision_pick.py --tag p2       # 抓完给这次记录打标签
    .venv_astra/bin/python scripts/vision_pick.py --dz -0.004    # 抓取高度微调 -4mm

为什么用 subprocess 调 roarm_cli.py 而不直接 import rclpy:
    vision 的 venv 是 Python 3.12，把 rclpy 装进 venv 容易和系统 ROS 的
    PYTHONPATH 打架；roarm_cli.py 在系统 Python 下已验证可用，隔进程调用最稳，
    而且能直接复用它「到位校验 + 退出码」的契约。

前置：机械臂 ROS 栈已启动（bash scripts/robot_up.sh），相机已插好。
"""
import argparse
import atexit
import csv
import datetime
import json
import os
import random
import re
import selectors
import shlex
import signal
import subprocess
import sys
import time

import numpy as np
import cv2

from camera import find_camera_index as find_astra_color_index, grab_frame, load_scene_z
from project import pixel_to_base as _pixel_to_base_impl


def pixel_to_base(u, v, T_base_cam, K, dist, z_plane):
    """兼容队友的调用顺序（T 在前）。

    project.py 的签名是 (u, v, K, dist, T_base_cam, z_plane)，而本文件是从
    队友那份改来的、所有调用都写成 T 在前。**刻意把差异隔离在这一个包装里** ——
    之前直接照抄调用，4x4 的 T 被当 3x3 内参，cv2 只报
    `CV_IS_MAT(_cameraMatrix) && rows == 3`，完全看不出是参数顺序问题。
    """
    return _pixel_to_base_impl(u, v, K, dist, T_base_cam, z_plane)

HERE = os.path.dirname(os.path.abspath(__file__))   # scripts/vision/
SCRIPTS = os.path.dirname(HERE)                     # scripts/
ROOT = os.path.dirname(SCRIPTS)                     # 项目根
DATA = os.path.join(ROOT, "data", "vision")
WS_SETUP = os.path.join(ROOT, "star-arm-moveit2", "install", "setup.bash")
CLI = os.path.join(ROOT, "scripts", "roarm_cli.py")
SCENE_YAML = os.path.join(ROOT, "data", "scene_real.yaml")

OFFSET_JSON = os.path.join(DATA, "grasp_offset.json")
LOG_CSV = os.path.join(DATA, "pick_log.csv")
XY_JSON = os.path.join(DATA, "xy_correction.json")
PROBE_CSV = os.path.join(DATA, "probe_points.csv")

#: 诊断图输出目录。--image 只作为**可选的输出路径**，不指定就走这里，
#: 免得没给参数时 cv2.imwrite(None, ...) 崩在最后一步。
PREVIEW_DIR = os.path.join(DATA, "preview")


def save_preview(img, name="pick_view.png"):
    os.makedirs(PREVIEW_DIR, exist_ok=True)
    path = os.path.join(PREVIEW_DIR, name)
    if not cv2.imwrite(path, img):
        print("⚠ 诊断图写入失败：%s" % path, file=sys.stderr)
    return path
PROBE_HEADER = ["ts", "obj_x", "obj_y", "dx", "dy", "act_x", "act_y"]

#: 默认末端姿态（world -> link6，零位实测），与 roarm_cli.py 保持一致
DEFAULT_QUAT = (0.5, -0.5, 0.5, -0.5)

#: 单次成功抓取反推的夹爪偏移（夹爪系，mm）。有 grasp_offset.json 时以文件为准。
DEFAULT_OFFSET_MM = (16.6, 23.6, 86.2)

#: roarm_cli.py 里的任务工作区（米），不是机械极限
#: x 上限 0.670 来自实测（probe_reach.py，z=0.200 固定抓取姿态可到 0.685）
#: y 取 [-0.400, +0.350]：覆盖相机横向视野（物料 y 约 -430~+377），
#: 且两端都实测可达 —— 目标是"看得见就能抓"，不再有软件侧的额外收缩。
BOUNDS = {"x": (0.278, 0.670), "y": (-0.400, 0.350), "z": (0.074, 0.438)}

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


def load_object_height(default=0.070):
    """从 scene_real.yaml 读物料高度（米），读不到就用 70mm。"""
    try:
        for line in open(SCENE_YAML, encoding="utf8"):
            s = line.strip()
            if s.startswith("object_height:"):
                return float(s.split(":", 1)[1].split("#")[0].strip())
    except Exception:
        pass
    return default


def load_xy_correction():
    """读 calib_xy.py 生成的线性修正。返回 (A,b,p0,rms,n,right_dy,right_full_at) 或 None。

    right_dy / right_full_at 是"只在画面右侧（y<p0）生效"的补充补偿：
    线性模型只能全局改斜率，改了右边会同时把左边带跑；而实测是中间、
    左边都准、只有右边差 5mm（差这 5mm 会让两指从物料侧面撞上去卡住）。
    所以单独加一项，从 p0 处为 0 线性增长到 right_full_at 处的 right_dy。
    """
    if not os.path.isfile(XY_JSON):
        return None
    try:
        d = json.load(open(XY_JSON, encoding="utf8"))
        return (np.array(d["A"], dtype=np.float64),
                np.array(d["b"], dtype=np.float64),
                np.array(d["p0"], dtype=np.float64),
                d.get("rms_mm", -1.0), len(d.get("points", [])),
                float(d.get("right_dy_mm", 0.0)),
                float(d.get("right_full_at_mm", -60.0)),
                float(d.get("place_dx_mm", 0.0)),
                float(d.get("place_dy_mm", 0.0)))
    except Exception as e:
        print("⚠ xy_correction.json 读不出来(%s)，本次不做 XY 修正" % e)
        return None


def right_side_extra(y_mm, corr):
    """右侧（y < p0）附加补偿量；p0 处为 0，到 right_full_at 处给满 right_dy。

    左侧和中间完全不受影响（t 被夹到 0），这是刻意的：
    只修"右边差 5mm"，不去动已经准的左边。
    """
    dy = corr[5]
    if dy == 0.0:
        return 0.0
    p0_y = corr[2][1]
    full_at = corr[6]
    if p0_y <= full_at:            # 参数写反了，保守起见不补偿
        return 0.0
    t = (p0_y - y_mm) / (p0_y - full_at)
    return dy * float(min(1.0, max(0.0, t)))


def apply_xy_correction(P_mm, corr):
    """p_corr = p + (A p + b - p) - (A p0 + b - p0)

    减掉参考点那一段，是为了让 p0 处修正为 0 —— 现有的 grasp_offset 是在
    p0 附近调出来的，这样它不会被重复补偿。
    """
    if corr is None:
        return P_mm
    A, b, p0 = corr[0], corr[1], corr[2]
    p = np.asarray(P_mm[:2], dtype=np.float64)
    delta = (A @ p + b - p) - (A @ p0 + b - p0)
    out = np.array(P_mm, dtype=np.float64)
    out[:2] = p + delta
    out[1] += right_side_extra(float(p[1]), corr)
    return out


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


# ---------------------------------------------------------------- 中断处理
_ABORT = {"flag": False}
_ABORT_HINT = (
    "\n⚠ 已取消当前动作，舵机会保持当前角度 —— 机械臂不会立刻掉下来。\n"
    "   先别去推它。要重启栈（或它已经在抖、已经瘫软）时：\n"
    "   ① 用手托住机械臂\n"
    "   ② cd ~/brain/ssvep-bci-arm-main/ssvep-bci-arm-main\n"
    "      bash scripts/robot_recover.sh\n"
)


def install_sigint():
    """把 Ctrl+C 变成"取消当前动作、保持位置"，而不是"杀掉客户端就走"。

    为什么不能直接退出：roarm_cli 只是发目标点的客户端，Ctrl+C 杀掉它并不会
    让机械臂停下——驱动会停在半路，舵机收不到后续指令就可能松力矩，臂直接垂
    下来（2026-10-23 真踩到）。现在改成：只终止当前那条 roarm_cli 子进程
    （等价于取消轨迹），controller 收到取消后会下发"保持当前角度"，臂稳稳停住；
    后续不再发任何新指令，脚本立刻收尾退出。
    """
    def handler(signum, frame):
        if _ABORT["flag"]:
            print("\n⚠ 再次 Ctrl+C：强制退出（之后务必先托住臂再重启栈）")
            raise KeyboardInterrupt
        _ABORT["flag"] = True
        print("\n\n⚠ 收到 Ctrl+C，正在取消当前动作（不会再发新指令）…")

    try:
        signal.signal(signal.SIGINT, handler)
    except Exception:
        pass


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
    # 用 Popen 而不是 run：机械臂走一条轨迹要 1~3 秒，这段时间主线程本来是
    # 空等。改成轮询 stdout 的同时刷新画面，就能做到"动作期间画面也在动"，
    # 而且 imshow 全在主线程（后台线程 imshow 会把这套 OpenCV 后端搞崩）。
    try:
        p = subprocess.Popen(["bash", "-c", inner],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, bufsize=1)
    except Exception as e:
        return 1, "启动子进程失败：%s" % e

    lines, timed_out, t0 = [], False, time.time()
    sel = selectors.DefaultSelector()
    sel.register(p.stdout, selectors.EVENT_READ)
    try:
        while True:
            if _ABORT["flag"]:
                # Ctrl+C：只终止这条 roarm_cli 子进程＝取消轨迹，
                # controller 会下发"保持当前角度"，臂停在原地。
                p.kill()
                break
            if PEEK is not None:
                PEEK.pump(fresh=True)        # ← 画面就在这里持续刷新
            if time.time() - t0 > timeout:
                timed_out = True
                p.kill()
                break
            for key, _ in sel.select(timeout=0.02):
                line = key.fileobj.readline()
                if line:
                    lines.append(line)
            if p.poll() is not None:
                break
        rest = p.stdout.read() or ""
        if rest:
            lines.append(rest)
    finally:
        sel.close()
        try:
            p.stdout.close()
        except Exception:
            pass
        p.wait()

    if _ABORT["flag"]:
        return 130, "用户中断（Ctrl+C）"
    if timed_out:
        return 124, "子进程超时 %ss" % timeout
    return p.returncode, "".join(lines).strip()


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


def cli_pose(xyz_mm, wait, tol, force, allow_far=False, retries=2, settle=True):
    """走一个位姿。settle=False 时不做"停一下再读真值"的复查。

    复查一次要 sleep 0.4s + 起一个 roarm_cli 进程读 where（约 1.5~2s）。
    抬升/平移这类**不要求精度**的动作每次都会被这个多花 2 秒
    （实测抬起误差常在 15~20mm，必然触发复查）。抓取下降那一步必须保留，
    因为标定用的 act_x/y/z 来自它。
    """
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

    # 分段逼近：长轨迹会被驱动丢点、停在半路（2026-10-23）。
    # 从当前实际位置重新规划，剩下的路短了，丢点影响就小很多。
    last_rc, last_act = 1, None
    for k in range(retries + 1):
        rc, out = run_cli(args, timeout=wait + 25)
        if rc == 130:                       # Ctrl+C：不重试、不再发指令
            return 130, None
        show(out)
        act = parse_actual(out)
        if rc == 0:
            # 原来的做法是每次"停 1 秒 + 再调一次 where 读稳定位置"，
            # 一次多花 3 秒（sleep + 一个 CLI 进程），5 次移动就 15 秒。
            # 现在：本来就够准（<8mm）的，直接用 CLI 报的数，不再复查；
            # 只有误差偏大时才多花那 3 秒去取真值（标定要用）。
            err = parse_err(out)
            need_recheck = settle and (err is None or err > 8.0)
            if need_recheck:
                time.sleep(0.4)
                settled = read_pose()
                if settled is not None:
                    act = settled
            return rc, act
        if act is None:
            return rc, None
        last_rc, last_act = rc, act
        err = float(np.linalg.norm(np.asarray(xyz_mm, dtype=float) - act))
        if err <= tol * 1.5:
            break
        if k < retries:
            print("  ⚠ 停在半路（差 %.0f mm），从当前位置重试 %d/%d"
                  % (err, k + 1, retries))
    return last_rc, last_act


def lift_to(up_mm, args, tol=30.0):
    """抬起（垂直上移）。短 wait 先试，没起来再补一次。

    抬起只是"离开物料"，不需要精度，所以：
      · tol 直接给 30mm（带物时臂会下沉 15~20mm）；
      · retries=0，不走分段逼近（那段会从当前位置重规划，带物时更慢）；
      · 第一次只用 25 秒的 wait —— 之前用 90 秒，一旦规划卡住就要干等
        两分多钟，用户看到的就是"卡住不动"。
    只有确实没抬起来（z 还比目标低 25mm 以上）才补发一次，且这次给足 wait。
    """
    rc, act = cli_pose(up_mm, min(args.wait, 25.0), tol, not args.no_force,
                       args.allow_far, retries=0, settle=False)
    if rc == 130:                       # Ctrl+C：不再补发
        return 130, None
    z = None if act is None else act[2]
    if z is None or z < up_mm[2] - 25.0:
        print("  ⚠ 没抬到位（z=%s，目标 %.0f），补发一次"
              % ("?" if z is None else "%.0f" % z, up_mm[2]))
        rc, act = cli_pose(up_mm, min(args.wait, 45.0), tol + 10.0,
                           not args.no_force, args.allow_far,
                           retries=0, settle=False)
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
        # dz = 目标 - 实测。dz < 0 表示臂比目标"高"（要往下压）；
        # dz > 0 表示偏低。偏低对抓取无害（指尖更低照样能夹），而且
        # "往上抬"的短移动实测会卡满 90 秒不动（2026-10-23），所以不补。
        dz = target[2] - act[2]
        if dz >= -dead:
            if dz > dead:
                print("  落点偏低 %.0f mm，直接抓（不再上抬）" % dz)
            break
        step = max(-max_step, min(max_step, dz))
        fix = np.array([target[0], target[1], target[2] + step])
        print("  z 偏高 %+.1f mm → 目标改发 z=%.1f（第 %d 次补偿）"
              % (dz, fix[2], i + 1))
        rc2, act2 = cli_pose(fix, wait, tol, force, allow_far)
        if rc2 == 130:                  # Ctrl+C：不沿用、不再补发
            return 130, act
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


def append_probe(row):
    """--probe --record 的落盘：视觉坐标 + 这次用的 dxy（用来当"真值"）。"""
    new = not os.path.isfile(PROBE_CSV)
    with open(PROBE_CSV, "a", newline="", encoding="utf8") as f:
        w = csv.DictWriter(f, fieldnames=PROBE_HEADER)
        if new:
            w.writeheader()
        w.writerow(row)


def retreat_xy(p_mm, dist_mm, lo_r=300.0):
    """把 link6 沿"朝底座"方向退回 dist_mm，z 不动；退不动返回 None。

    为什么要退：夹着物料抬起来时，整条前臂+夹爪正挡在相机和桌面之间，
    默认投放区（x 0.38~0.55）有一大块落在阴影里，用户根本点不准
    （2026-10-25 反馈）。相机在 +y 一侧、底座上方俯视，臂伸出去时挡住
    的正是"靠前那片"，所以往回缩半径就行：整条臂的投影一起往底座收，
    姿态不变（夹着的物料不会晃）。

    限制：半径不小于 lo_r=300mm —— 这台臂缩得太厉害时肘部会折到很别扭
    的位形，规划又慢又容易碰到自己（BOUNDS 下限 278mm 是极限，别贴上去）。
    """
    x, y, z = (float(v) for v in p_mm)
    r = float(np.hypot(x, y))
    if r <= lo_r:
        return None
    dist = min(float(dist_mm), r - lo_r)
    if dist < 25.0:
        return None
    k = (r - dist) / r
    return np.array([x * k, y * k, z])


def choose_place_slot(obj_y_mm, args):
    """按「左边→放右边、右边→放左边、中间→随机」选投放点。

    以基座系 y 判断左右（实测：放右边时 y 为负、左边为正）：
        y >  +side_mm  → 在左边，投到右边槽
        y <  -side_mm  → 在右边，投到左边槽
        其余（中间）    → 随机
    返回物料坐标（mm）np.array([x, y])，或 None 表示放回原处。
    注意：命令行槽位参数是米，这里统一换算成 mm 返回，因为后面的
    夹爪偏移 d_mm、P_mm 全是 mm。
    """
    if args.place == "origin":
        return None
    m = args.side_mm
    if obj_y_mm > m:
        slot = np.array(args.place_right, dtype=np.float64) * 1000.0
        print("  物料在左边（y=%+.0f）→ 投到右边槽 (%+.0f, %+.0f) mm"
              % (obj_y_mm, slot[0], slot[1]))
        return slot
    if obj_y_mm < -m:
        slot = np.array(args.place_left, dtype=np.float64) * 1000.0
        print("  物料在右边（y=%+.0f）→ 投到左边槽 (%+.0f, %+.0f) mm"
              % (obj_y_mm, slot[0], slot[1]))
        return slot
    slot = np.array([random.uniform(*args.random_x),
                     random.uniform(*args.random_y)], dtype=np.float64) * 1000.0
    print("  物料在中间（y=%+.0f）→ 随机投放到 (%+.0f, %+.0f) mm"
          % (obj_y_mm, slot[0], slot[1]))
    return slot


# ---------------------------------------------------------------- 画面
PEEK = None     # 由 main() 赋值；run_cli 在等外部命令时用它持续刷新画面


class LiveCam:
    """主线程独占的相机：只 open 一次，之后反复 read。

    不用后台线程，原因（2026-10-23 连续踩三次）：
      · 后台线程 cv2.imshow + 主线程再开第二个窗口 → 进程被"已杀死"；
      · 关掉窗口后重新 open(/dev/video2) → 直接打不开（can't open camera by index）。
    所以相机全程常开不关、所有 imshow 都放在主线程，
    动作等待期间靠 run_cli 的轮询循环刷新画面。
    """
    def __init__(self, w=1280, h=720, warm=8):
        idx = find_astra_color_index()
        if idx is None:
            raise RuntimeError("找不到 Astra 彩色设备")
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        if not self.cap.isOpened():
            raise RuntimeError("打不开相机 idx=%s" % idx)
        for _ in range(warm):          # 丢掉开头几张（自动曝光/缓冲）
            self.cap.read()
        self.frame = None
        self.read()

    def read(self):
        ok, f = self.cap.read()
        if ok and f is not None:
            self.frame = f
        return self.frame

    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass


def base_to_pixel(P_mm, T_base_cam, K, dist):
    """基座坐标 (mm) → 像素。给"把可投放区域画到画面上"用。

    T_base_cam 是 handeye_result 里那份（米制），把相机系变到基座系，
    所以反变换是 R^T (P - t)。
    """
    Tb = np.asarray(T_base_cam, dtype=np.float64)
    P = np.asarray(P_mm, dtype=np.float64).reshape(3) / 1000.0
    Pc = Tb[:3, :3].T @ (P - Tb[:3, 3])
    rvec = np.zeros(3)
    uv, _ = cv2.projectPoints(Pc.reshape(1, 1, 3), rvec, np.zeros(3), K, dist)
    return uv.reshape(2)


class Peek:
    """单窗口画面：主线程 imshow，接口和原来的 LiveView 兼容。"""

    WIN = "vision_pick"

    def __init__(self, cam, enable=True, scale=0.75):
        self.cam = cam
        self.scale = scale
        self.on = bool(enable)
        self.status = ""
        self.target = None
        self.region = None         # 可投放区域（原图像素，4 个角）
        self.last_key = 255        # 最近一次按下的键（255 = 没有）
        if not self.on:
            return
        try:
            # WINDOW_NORMAL 才能让 resizeWindow / moveWindow / TOPMOST 生效；
            # AUTOSIZE 下窗口位置不由程序控制，容易被压在其他窗口后面，
            # 用户看不到窗口就以为程序卡住了（2026-10-25 踩到，Ctrl+C 两次）。
            f = self.cam.frame
            h, w = (720, 1280) if f is None else f.shape[:2]
            cv2.namedWindow(self.WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.WIN, int(w * self.scale), int(h * self.scale))
            cv2.moveWindow(self.WIN, 60, 40)
            try:
                cv2.setWindowProperty(self.WIN, cv2.WND_PROP_TOPMOST, 1)
            except Exception:
                pass
        except Exception as e:
            print("  （画面窗口开不了：%s）" % e)
            self.on = False

    def _compose(self):
        img = self.cam.frame
        if img is None:
            return None
        view = cv2.resize(img, None, fx=self.scale, fy=self.scale)
        if self.region is not None:
            pts = np.array([[(int(x * self.scale), int(y * self.scale))
                             for x, y in self.region]], dtype=np.int32)
            overlay = view.copy()
            cv2.fillPoly(overlay, pts, (0, 200, 0))
            cv2.addWeighted(overlay, 0.18, view, 0.82, 0, view)
            cv2.polylines(view, pts, True, (0, 255, 0), 2)
            x0, y0 = pts[0][0]
            cv2.putText(view, "DROP AREA", (max(4, x0 + 6), max(20, y0 + 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                        cv2.LINE_AA)
        if self.target is not None:
            u = int(self.target[0] * self.scale)
            v = int(self.target[1] * self.scale)
            cv2.circle(view, (u, v), 18, (0, 255, 0), 2)      # 绿圈标范围
            cv2.drawMarker(view, (u, v), (0, 0, 255),
                           cv2.MARKER_CROSS, 30, 2)           # 红十字标中心
            cv2.putText(view, "PICK", (u + 22, v - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        cv2.LINE_AA)
        if self.status:
            cv2.putText(view, self.status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2, cv2.LINE_AA)
        return view

    def pump(self, fresh=True):
        """刷新一帧。run_cli 在等外部命令时每轮都会调一次。"""
        if not self.on:
            return self.cam.frame
        if fresh:
            self.cam.read()
        view = self._compose()
        if view is None:
            return None
        try:
            cv2.imshow(self.WIN, view)
            # waitKey 只有一个按键队列：这里不取走，wait_click 里就永远读不到
            # （以前 Esc 只能靠右键兜底，就是这个原因）。取到就存起来给它读。
            k = cv2.waitKey(1) & 0xFF
            if k != 255:
                self.last_key = k
        except Exception:
            self.on = False
        return self.cam.frame

    # ---- 兼容原 LiveView 的接口
    def set_status(self, text):
        self.status = text
        self.pump(fresh=False)

    def set_target(self, uv, area):
        self.target = (float(uv[0]), float(uv[1]))

    def set_region(self, pts_px):
        """设置要画出来的可投放区域（原图像素，4 个角）；None = 不画。"""
        self.region = None if pts_px is None else [tuple(map(float, p)) for p in pts_px]

    def snapshot(self):
        return None if self.cam.frame is None else self.cam.frame.copy()

    def close(self):
        if not self.on:
            return
        try:
            cv2.destroyWindow(self.WIN)
            cv2.waitKey(1)
        except Exception:
            pass
        self.on = False

    # ---- 点选（复用同一个窗口，不再新开第二个）
    def wait_click(self, status=None, on_key=None):
        """等一次左键点击，返回原图像素 (u, v)；取消返回 None。

        on_key: 可选回调 f(ch) -> str | None。按下可打印字符时调用，
        返回非空字符串就当作新的状态栏文字（用来做"再退一点"这类操作）。
        回调里抛异常只打印，不影响继续等点击。
        """
        if not self.on:
            return None
        hits = []

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                hits.append((x / self.scale, y / self.scale))
            elif event == cv2.EVENT_RBUTTONDOWN:
                hits.append(None)       # 用 None 当"取消"标记

        cv2.setMouseCallback(self.WIN, on_mouse)
        old = self.status
        self.status = status or "LEFT click = pick target    RIGHT / Esc = cancel"
        self.pump(fresh=False)
        print("  点选窗口已弹出（标题 vision_pick）。看不到就按 Alt+Tab 切过去；"
              "右键或 Esc 取消")
        try:
            while not hits:
                if _ABORT["flag"]:      # 按过 Ctrl+C 就别再干等点击了
                    hits.append(None)
                    break
                self.pump(fresh=True)
                k, self.last_key = self.last_key, 255
                if k == 27:
                    hits.append(None)
                elif on_key is not None and 32 <= k < 127:
                    try:
                        txt = on_key(chr(k).lower())
                    except Exception as ex:
                        print("  ⚠ 按键 %r 处理失败：%s" % (chr(k), ex))
                        txt = None
                    if txt:
                        self.status = txt
                        self.pump(fresh=False)
            # 点完先给反馈：把点到的位置画成红十字，停 0.8 秒让你确认。
            # 这里刻意不重新取帧（fresh=False），否则十字会和画面错开。
            hit = hits[0]
            if hit is not None:
                self.target = (float(hit[0]), float(hit[1]))
                self.status = "picked (%.0f, %.0f) px" % hit
                t0 = time.time()
                while time.time() - t0 < 0.8:
                    self.pump(fresh=False)
                    cv2.waitKey(20)
        finally:
            cv2.setMouseCallback(self.WIN, lambda *a: None)
            self.status = old
        return hits[0]


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
    ap.add_argument("--no-show", action="store_true",
                    help="不开画面窗口（只跑流程）；点选模式下必须有窗口，"
                         "要么去掉它，要么用 --u/--v 直接给像素")
    ap.add_argument("--wait", type=float, default=90.0,
                    help="每个位姿最长等待（秒）。臂慢的时候别设小，超时会主动取消轨迹，"
                         "等于半路停车（2026-10-23 踩过：30s 只走了 100mm）")
    ap.add_argument("--tag", default="", help="给这次记录打标签")
    ap.add_argument("--offset", type=float, nargs=3, default=None,
                    metavar=("X", "Y", "Z"),
                    help="临时指定夹爪偏移（夹爪系 mm），覆盖 grasp_offset.json")
    ap.add_argument("--no-lift", action="store_true", help="闭合后不要抬起来")
    ap.add_argument("--probe", action="store_true",
                    help="只移到物料上方悬停、不下降不闭合，方便用眼睛量偏差")
    ap.add_argument("--probe-lift", type=float, default=40.0,
                    help="--probe 时停在抓取点上方多少 mm，默认 40")
    ap.add_argument("--dxy", type=float, nargs=2, metavar=("DX", "DY"),
                    help="在偏移基础上再把 link6 挪 (DX,DY) mm（世界系；只试算，不写文件）")
    ap.add_argument("--record", action="store_true",
                    help="配合 --probe：回车时把这次「视觉 + 当时的 dxy」记成一个标定点")
    ap.add_argument("--click", dest="click", action="store_true", default=True,
                    help="弹窗左键点选物料（**默认**）；点哪算哪，不受蓝色阈值/背景干扰")
    ap.add_argument("--auto", dest="click", action="store_false",
                    help="不弹窗，自动识别蓝色区域（原来那套）")
    ap.add_argument("--place", choices=("swap", "origin"), default="swap",
                    help="抓完放哪：swap=左→右/右→左/中间随机（默认），origin=放回原处")
    ap.add_argument("--auto-place", action="store_true",
                    help="不点选投放位置，直接用 --place 的自动规则（默认会弹窗让你点）")
    ap.add_argument("--drop-rect", type=float, nargs=4,
                    default=[0.38, -0.25, 0.55, 0.25],
                    metavar=("X1", "Y1", "X2", "Y2"),
                    help="画在画面上的推荐投放区（物料坐标，米），"
                         "默认 x 0.38~0.55、y -0.25~0.25")
    ap.add_argument("--drop-standoff", type=float, default=160.0,
                    metavar="MM",
                    help="点选投放位置前先往后退多少 mm（朝底座方向），"
                         "让臂别挡住桌面；默认 160，0 = 不退")
    ap.add_argument("--place-dxy", type=float, nargs=2, default=None,
                    metavar=("DX", "DY"),
                    help="点选投放的固定微调（物料坐标 mm），比如放下去总是偏 "
                         "y+8 就写 --place-dxy 0 -8")
    ap.add_argument("--save-place-dxy", action="store_true",
                    help="把本次 --place-dxy 写进 xy_correction.json，下次不用再写")
    ap.add_argument("--drop-z", type=float, default=None, metavar="M",
                    help="点选投放时用的桌面高度（米）。默认用 z_g - 物料高/2，"
                         "即把物料中心高度换算成桌面；还偏高/偏低就用它微调")
    ap.add_argument("--place-left", type=float, nargs=2, default=[0.46, 0.11],
                    metavar=("X", "Y"), help="左侧投放槽（物料坐标，米），默认 0.46 0.11")
    ap.add_argument("--place-right", type=float, nargs=2, default=[0.46, -0.10],
                    metavar=("X", "Y"), help="右侧投放槽（物料坐标，米），默认 0.46 -0.10")
    ap.add_argument("--random-x", type=float, nargs=2, default=[0.42, 0.52],
                    metavar=("X0", "X1"), help="中间随机投放的 x 范围（米）")
    ap.add_argument("--random-y", type=float, nargs=2, default=[-0.04, 0.06],
                    metavar=("Y0", "Y1"), help="中间随机投放的 y 范围（米）")
    ap.add_argument("--side-mm", type=float, default=30.0,
                    help="y 超过这个绝对值就算在左/右侧，默认 30mm")
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
    install_sigint()        # Ctrl+C = 取消当前动作 + 保持位置，不是直接退出

    # ---- 0) 相机 + 画面窗口（单窗口、主线程刷新、相机全程常开）
    global PEEK
    try:
        cam = LiveCam()
    except Exception as e:
        print("❌ 相机打不开：%s" % e)
        return 1
    lv = Peek(cam, enable=not args.no_show)
    PEEK = lv
    atexit.register(cam.release)
    atexit.register(lv.close)
    if lv.on:
        lv.set_status("[0/7] idle")
        print("实时画面已开启（窗口 vision_pick，动作期间也持续刷新）")
    click_mode = args.click and args.u is None

    # ---- 1) 标定参数
    d = np.load(os.path.join(DATA, "T_base_cam.npz"))
    T_base_cam = d["T_base_cam"]
    Kd = np.load(os.path.join(DATA, "cam_intrinsics.npz"))
    K, dist = Kd["K"], Kd["dist"]
    z_g = (args.z_grasp if args.z_grasp is not None else load_scene_z()) + args.dz
    # 点选投放时点的是**桌面**，不是物料中心。z_g 是"物料被点选/被夹"的那个
    # 高度 ≈ 物料中心 = 桌面 + 物料高/2，直接拿 z_g 去和桌面求交，整块投放区
    # 会朝相机方向缩 30~40mm（2026-10-25 "放置偏得严重"的主因之一）。
    obj_h_m = load_object_height()
    z_drop = (args.drop_z if args.drop_z is not None
              else z_g - obj_h_m / 2.0)
    if args.offset:
        off_mm = np.array(args.offset, dtype=np.float64)
        off_src = "命令行 --offset"
    else:
        off_mm, off_src = load_offset()
    if args.dxy:
        # 直接在**世界系**上挪 link6 的目标点，省得每次记 off_x/off_y/off_z 的映射：
        #   off_x ↑ → link6 y ↓ ；off_z ↑ → link6 x ↑
        dx, dy = args.dxy
        off_mm = off_mm.copy()
        off_mm[0] -= dy
        off_mm[2] += dx
        off_src += " +dxy(%+.0f,%+.0f)" % (dx, dy)
    R = quat_to_R(DEFAULT_QUAT)
    xy_corr = load_xy_correction()      # 米尺线性修正，没有就是 None
    # 点选投放的固定微调（物料坐标 mm）：优先 --place-dxy，其次读 json 里的
    # place_dx_mm / place_dy_mm。抓取准但投放整体偏一点时用它补，别去改 XY 修正
    # （那个一改会把抓取带跑）。
    if args.place_dxy:
        place_dxy = np.array(args.place_dxy, dtype=np.float64)
    elif xy_corr is not None:
        place_dxy = np.array([xy_corr[7], xy_corr[8]], dtype=np.float64)
    else:
        place_dxy = np.zeros(2)
    if args.place_dxy and args.save_place_dxy:
        try:
            j = json.load(open(XY_JSON, encoding="utf8"))
            j["place_dx_mm"] = float(place_dxy[0])
            j["place_dy_mm"] = float(place_dxy[1])
            json.dump(j, open(XY_JSON, "w", encoding="utf8"),
                      ensure_ascii=False, indent=2)
            print("已把投放微调写入 xy_correction.json：%+.1f / %+.1f mm"
                  % (place_dxy[0], place_dxy[1]))
        except Exception as e:
            print("⚠ 投放微调没保存成功：%s" % e)

    # ---- 2) 先回 home 再拍
    # 臂如果停在物料上方，会正好把物料挡在身后（2026-10-21 实际踩到：
    # 抓空后臂悬在 z=116mm，拍到的画面里物料被两指完全遮住）。
    if args.dry_run:
        print("dry-run：不动机械臂。若画面里找不到物料，先手动回 home：")
        print("  ./scripts/roarm_cli.py home --wait 20")
    elif args.no_home:
        print("[0/7] --no-home：跳过复位，直接拍照")
    else:
        print("[0/7] 复位到 home（先让臂离开相机视线）")
        # home 只是"挪开让相机看得见物料"，不需要精度；而且这台臂有固定偏差
        # （目标 365 → 落点 354，x 还偏 +4mm），用默认 5mm 容差必然判失败，
        # 还会白等满 30 秒。所以这里单独放宽到 25mm。
        rc, out = run_cli(["home", "--wait", "%.1f" % args.wait, "--tol", "40.0"],
                          timeout=args.wait + 25)
        show(out)
        if rc != 0:
            print("  ⚠ home 没进 25mm（臂有固定偏差属正常），继续下一步")

    def go_home(label="[7/7] 闭合夹爪并回 home", close_grip=True):
        """收尾：先把夹爪闭合（收拢、不张着），再回 home。"""
        print("\n%s" % label)
        if lv is not None:
            lv.set_status("[7/7] closing grip + home" if close_grip
                          else "[7/7] going home")
        if close_grip:
            cli_grip("close", 10)
            print("  夹爪已闭合")
        rc2, out2 = run_cli(["home", "--wait", "%.1f" % args.wait, "--tol", "40.0"],
                            timeout=args.wait + 25)
        show(out2)
        if rc2 != 0:
            print("  ⚠ home 没进 40mm（臂有固定偏差），不影响下一轮")
        if lv is not None:
            lv.set_status("[7/7] done - at home")

    # ---- 3) 拍照 + 检测
    print("拍照…")
    try:
        img = lv.pump(fresh=True)        # 相机常开，这里取最新一帧并刷新显示
        if img is None:
            raise SystemExit("相机没有画面")
    except SystemExit as e:
        print("❌ %s" % e)
        return 1
    cands = []          # [(uv, area, P_base)]
    # 注意：--click 现在是默认行为，但显式给了 --u/--v 时应该用像素，
    # 否则 "指定像素" 会被点选窗口抢走（click_mode 已含这个判断）。
    if click_mode:
        if not lv.on:
            print("❌ --no-show 下没有窗口，无法点选。")
            print("   去掉 --no-show，或用 --u U --v V 直接给像素")
            return 1
        # 复用同一个窗口点选：不再"关一个开一个"，相机也不用重新打开。
        p = lv.wait_click()
        if p is None:
            print("❌ 没有点选目标（取消了），已退出")
            return 1
        uv = p
        area = 3000.0               # 只用于画圈，没有物理含义
        P = pixel_to_base(uv[0], uv[1], T_base_cam, K, dist, z_g)
        if P is None:
            print("❌ 该像素与 z=%.3f 平面没有交点（点到了地平线以上？）" % z_g)
            return 1
        P_mm = P * 1000.0
        print("手动点选像素 (%.0f, %.0f) → 基座 (%+.0f, %+.0f) mm"
              % (uv[0], uv[1], P_mm[0], P_mm[1]))
        lv.set_target(uv, area)          # 后续每一步画面里都标着这个点
        lv.set_status("[1/7] target locked")
    elif args.u is not None and args.v is not None:
        uv = (args.u, args.v)
        area = 3000.0
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
            save_preview(img, "pick_no_blue.png")
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
            save_preview(img, "pick_no_candidate.png")
            return 1
        uv, area, P = cands[0]
        P_mm = P * 1000.0
        print("选用 像素 (%.0f, %.0f)，面积 %.0f" % (uv[0], uv[1], area))
        if lv is not None:
            lv.set_target(uv, area)      # 实时窗口里直接圈出来
            lv.set_status("[1/7] target locked")

    # ---- 3.5) 米尺线性修正（calib_xy.py 生成，没有就跳过）
    P_raw_mm = np.array(P_mm, dtype=np.float64)   # 修正前的原始投影，记录/标定用
    if xy_corr is not None:
        before = np.array(P_mm[:2], dtype=np.float64)
        P_mm = apply_xy_correction(P_mm, xy_corr)
        print("已应用 XY 线性修正：(%+.0f,%+.0f) → (%+.0f,%+.0f) mm"
              "（%d 点拟合，RMS %.1f）"
              % (before[0], before[1], P_mm[0], P_mm[1],
                 xy_corr[4], xy_corr[3]))
        extra = right_side_extra(float(before[1]), xy_corr)
        if extra:
            print("  其中含右侧补偿 %+.1f mm（只作用于 y<p0，左侧/中间不动）" % extra)
        if abs(P_raw_mm[1]) > 160:
            print("  ⚠ 物料 y=%+.0f mm 超出 XY 标定范围（标定点最远只到 ±143）：" % P_raw_mm[1])
            print("     这里是线性外推，误差可能 10mm 以上。想准就二选一：")
            print("     · 把物料放到 |y|<150 内再抓")
            print("     · 或在当前位置用 --probe --record 补一个标定点，再 calib_xy --from-probe")

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
    # 半径检查：x/y 都在界内不等于"舒服"。这台臂全伸展约 685mm，
    # 接近极限时规划会变得极慢甚至卡死（2026-10-23 实测：半径 675mm 时
    # 预抓取那一步直接卡住不返回）。
    radius = float(np.hypot(tgt[0], tgt[1])) / 1000.0
    if radius > 0.685:
        print("  ⛔ 抓取点距底座 %.0f mm，超过伸展极限（约 685 mm），不硬发。" % (radius * 1000))
        print("     把物料往底座方向挪 40mm 以上，或用 --allow-far 强行试。")
        far = far or ["r=%.0fmm" % (radius * 1000)]
    elif radius > 0.650:
        print("  ⚠ 抓取点距底座 %.0f mm，接近伸展极限（约 685 mm）：" % (radius * 1000))
        print("     这个位形下动作会明显变慢、z 也更容易偏，必要时把物料挪近 30~50mm。")
    print("──────────────────────────────────────────\n")

    view = img.copy()
    # 圈出目标：用面积折算等效半径，直观看到"它认的是哪个蓝色块"
    _r = int(max(14, (float(area) / 3.14159) ** 0.5))
    _c = (int(uv[0]), int(uv[1]))
    cv2.circle(view, _c, _r, (0, 255, 0), 2)
    cv2.putText(view, "TARGET", (_c[0] - _r, max(20, _c[1] - _r - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.drawMarker(view, _c, (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    cv2.putText(view, "obj (%.0f,%.0f)" % uv, (_c[0] + 18, _c[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(view, "base %+.0f,%+.0f mm  ->  link6 %+.0f,%+.0f,%+.0f" %
                (P_mm[0], P_mm[1], tgt[0], tgt[1], tgt[2]),
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
    if args.image:
        os.makedirs(os.path.dirname(os.path.abspath(args.image)) or ".", exist_ok=True)
        saved = args.image
        cv2.imwrite(saved, view)
    else:
        saved = save_preview(view, "pick_view.png")
    print("叠加图已存 %s" % saved)

    # 只有"窗口没开起来"时才单独弹一次叠加图兜底（正常情况窗口一直有画面）
    if not lv.on and not args.no_show:
        try:
            cv2.namedWindow("vision_pick", cv2.WINDOW_NORMAL)
            h, w = view.shape[:2]
            cv2.resizeWindow("vision_pick", min(1000, w),
                             int(min(1000, w) * h / float(w)))
            cv2.imshow("vision_pick", view)
            cv2.waitKey(1500)
            cv2.destroyWindow("vision_pick")
        except Exception as e:
            print("  （弹窗失败，看 %s 即可：%s）" % (saved, e))

    if args.dry_run:
        print("\n--dry-run：不动机械臂。确认上面的点没问题就去掉这个参数。")
        return 0

    if far and not args.allow_far:
        print("\n⛔ 抓取点不可用（%s），已中止（机械臂没动）。" % "、".join(far))
        print("   把物料往底座方向挪 40mm 以上，或加 --allow-far 强行试。")
        return 2

    # ---- 5) 执行
    print("\n1 秒后开始动作，手离机械臂远一点…")
    time.sleep(1.0)

    rec = dict(ts=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
               tag=args.tag, u="%.1f" % uv[0], v="%.1f" % uv[1],
               obj_x="%.2f" % P_mm[0], obj_y="%.2f" % P_mm[1], obj_z="%.2f" % P_mm[2],
               tgt_x="%.2f" % tgt[0], tgt_y="%.2f" % tgt[1], tgt_z="%.2f" % tgt[2],
               qx=DEFAULT_QUAT[0], qy=DEFAULT_QUAT[1], qz=DEFAULT_QUAT[2],
               qw=DEFAULT_QUAT[3], close_err="", grasp_ok="0", note="")
    act = None

    def bail(step):
        if _ABORT["flag"]:
            rec["note"] = "在「%s」被 Ctrl+C 中断" % step
            append_log(rec)
            print(_ABORT_HINT)
            return 130
        rec["note"] = "在「%s」失败" % step
        append_log(rec)
        print("\n❌ %s 失败，已中止。记录写进 %s" % (step, LOG_CSV))
        return 1

    print("\n[1/7] 张开夹爪")
    if lv is not None:
        lv.set_status("[1/7] opening grip")
    rc, _ = cli_grip("open", 20)
    if rc != 0:
        print("  ⚠ 张开没确认到位，继续（多数情况下已经张开）")

    print("\n[2/7] 移到预抓取点（抬高 %.0f mm）"
          % (pre_z - tgt[2]))
    if lv is not None:
        lv.set_status("[2/7] moving to pre-grasp")
    rc, act = cli_pose(pre, args.wait, args.tol, not args.no_force, args.allow_far,
                       settle=False)
    if rc == 3:
        print("\n⛔ 目标超出可达范围，已中止。把物料往底座方向挪 8~10cm 再试。")
        rec["note"] = "目标超出工作区"
        append_log(rec)
        return 2
    if rc != 0:
        return bail("预抓取移动")

    # ---- 悬浮观察：停在物料上方不落爪，方便用眼睛量偏差
    if args.probe:
        pz = tgt[2] + args.probe_lift
        probe = np.array([tgt[0], tgt[1], pz])
        if args.dry_run:
            print("\n[观察] --dry-run：不动臂。实际运行会停在这里：")
            print("  link6 (%+.1f, %+.1f, %+.1f)（物料上方 %.0fmm）"
                  % (*probe, args.probe_lift))
            return 0
        print("\n[观察] 移到物料上方 %.0fmm 停住（不下降、不闭合）" % args.probe_lift)
        if lv is not None:
            lv.set_status("[probe] holding above target")
        rc, act = cli_pose(probe, args.wait, args.tol, not args.no_force,
                           args.allow_far)
        # 悬浮观察要看得准，两指的离地高度必须一致：这台臂在伸展姿态下
        # z 常偏 10~14mm（今天实测 -13.8 / +7.1 都有），高度不齐会让人
        # 从侧面看时误判前后距离。所以补发一次到真正的高度。
        for _ in range(2):
            if act is None:
                break
            dz = pz - act[2]
            if abs(dz) <= 4.0:
                break
            print("  高度差 %+.1f mm，补发一次 z=%.1f" % (dz, probe[2] + dz))
            rc, act = cli_pose(np.array([probe[0], probe[1], probe[2] + dz]),
                               args.wait, args.tol, not args.no_force,
                               args.allow_far)
        if act is not None:
            print("  目标 link6 (%+.1f, %+.1f, %+.1f)  实际 (%+.1f, %+.1f, %+.1f)"
                  % (*probe, *act))
            print("  与目标的差: (%+.1f, %+.1f, %+.1f) mm"
                  % tuple(act - probe))
        print("\n  停在这里不动。从正上方/侧面看两指相对物料的偏差：")
        print("    · 需要往 +y 走 N mm  → 说「y+Nmm」")
        print("    · 需要往 +x 走 N mm  → 说「x+Nmm」")
        print("    · 高度不合适          → 说「抬高/降低 N mm」")

        if args.record:
            print("\n  --record：这次若两指正好对称，回车就记成标定点；"
                  "不对齐请 Ctrl+C 退出，加 --dxy 再来。")
        try:
            input("\n  量好后按回车结束（窗口会关闭）> ")
        except (EOFError, KeyboardInterrupt):
            print("\n  （已取消，本次不记录）")
            return 0

        if args.record:
            # 高度没补准就记录 = 废点：臂悬在物料上方 10mm 以外时，
            # 从侧面看两指的前后位置会被压缩，人眼很容易判错（2026-10-25
            # 两个点就是这么记废的，拟合出 A[1][0]=-0.136、RMS 10.9mm）。
            zerr = None if act is None else act[2] - probe[2]
            if zerr is not None and abs(zerr) > 8.0:
                print("\n  ⚠ 高度还差 %+.0f mm（目标 %.0f，实际 %.0f），"
                      "这个高度看两指会误判前后 —— **本次不记录**。"
                      % (zerr, probe[2], act[2]))
                print("     先把高度补准（或加 --probe-lift 减小观察高度）再来。")
                return 0
            ddx, ddy = args.dxy if args.dxy else (0.0, 0.0)
            append_probe(dict(
                ts=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                obj_x="%.2f" % P_raw_mm[0], obj_y="%.2f" % P_raw_mm[1],
                dx="%.1f" % ddx, dy="%.1f" % ddy,
                act_x="" if act is None else "%.2f" % act[0],
                act_y="" if act is None else "%.2f" % act[1]))
            print("\n  ✅ 已记录：视觉 (%+.1f, %+.1f)  真值 (%+.1f, %+.1f)"
                  % (P_raw_mm[0], P_raw_mm[1],
                     P_raw_mm[0] + ddx, P_raw_mm[1] + ddy))
            print("     （记录时高度差 %+.1f mm，在容差内）"
                  % (0.0 if zerr is None else zerr))
            print("     换到另一个位置再观察一次，攒 2 点以上后跑：")
            print("     .venv_astra/bin/python scripts/calib_xy.py --from-probe")
        return 0

    print("\n[3/7] 下降到抓取点")
    if lv is not None:
        lv.set_status("[3/7] descending")
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

    print("\n[4/7] 闭合夹爪")
    if lv is not None:
        lv.set_status("[4/7] closing grip")
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

    print("\n[5/7] 抬起")
    if lv is not None:
        lv.set_status("[5/7] lifting")
    # 带物抬升时臂会下沉 15~20mm，容差给到 30；并且用短 wait（25s）先试，
    # 真卡住最多等 25 秒，不会像以前那样一卡就两分钟。
    rc, _ = lift_to(pre, args, tol=max(args.tol, 30.0))
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
        go_home()
        return 0

    if args.no_place:
        print("   --no-place：物料留在夹爪里")
        go_home(label="[7/7] 回到 home（夹爪夹着物料，不再闭合）",
                close_grip=False)
        return 0

    # ---- 5.4) 后退让开视野：夹着物料时前臂会把桌面挡掉一大块
    cur_pose = np.asarray(pre, dtype=np.float64)   # 臂大致在哪（后退时从这里算）
    if not args.auto_place and lv.on and args.drop_standoff > 0:
        print("\n[5.4/7] 后退让开视野")
        lv.set_status("[5.4/7] backing off")
        back = retreat_xy(cur_pose, args.drop_standoff)
        if back is None:
            print("  已经离底座很近了，退不动，就在当前位置点（投放区可能被臂挡一部分）")
        else:
            rc, act = cli_pose(back, min(args.wait, 25.0), max(args.tol, 25.0),
                               not args.no_force, args.allow_far,
                               retries=1, settle=False)
            if rc == 130:
                # 物料还在爪里，别继续往下走；臂已停在原地（controller 保持角度）
                print(_ABORT_HINT)
                return 130
            if act is None:
                print("  ⚠ 后退没走成，就在当前位置点（投放区可能被臂挡住）")
            else:
                moved = float(np.linalg.norm(np.asarray(act[:2]) - cur_pose[:2]))
                cur_pose = np.asarray(act, dtype=np.float64)
                print("  后退 %.0f mm 到 link6 (%+.0f, %+.0f, %+.0f)，"
                      "投放区应该看得见了" % (moved, *act))

    # ---- 5.5) 点选投放位置（物料已在爪里、已抬起，画面里能看到桌面）
    manual_slot = None          # 物料坐标（mm），None = 走 --place 的自动规则
    if not args.auto_place and lv.on:
        print("\n[5.5/7] 点选投放位置")
        lv.set_status("[5.5/7] click drop area")
        x1, y1, x2, y2 = args.drop_rect
        pts = None
        cur = [cur_pose]        # 按 b 反复后退时，从"现在的实际位置"再退

        def _key(ch):
            """画面里按 b = 再后退 80mm（有时候一次退不到位，还得再让一点）。"""
            if ch != "b":
                return None
            nb = retreat_xy(cur[0], 80.0)
            if nb is None:
                print("  已经退到底了（半径到 300mm），不能再退")
                return "[5.5/7] already at rear limit  click to drop"
            rc, act = cli_pose(nb, min(args.wait, 25.0), max(args.tol, 25.0),
                               not args.no_force, args.allow_far,
                               retries=0, settle=False)
            if act is None:
                print("  ⚠ 再后退没走成")
                return None
            cur[0] = np.asarray(act, dtype=np.float64)
            print("  再后退 80mm → link6 (%+.0f, %+.0f, %+.0f)" % tuple(act))
            return "[5.5/7] backed off  click to drop"

        try:
            zc = z_drop * 1000.0
            print("  投放点按桌面平面 z=%.1f mm 反算（物料高 %.0f，物料中心 %.1f）"
                  % (zc, obj_h_m * 1000.0, z_g * 1000.0))
            pts = [base_to_pixel((x * 1000.0, y * 1000.0, zc), T_base_cam, K, dist)
                   for (x, y) in ((x1, y1), (x2, y1), (x2, y2), (x1, y2))]
            if not np.all(np.isfinite(pts)):
                print("  ⚠ 推荐区域投影到画面失败（有几个点跑到相机背后），"
                      "不画框，你直接点也行")
                pts = None
            else:
                lv.set_region(pts)
                print("  绿色框 = 推荐投放区（物料坐标 x %+.0f~%+.0f、y %+.0f~%+.0f mm）"
                      % (x1 * 1000, x2 * 1000, y1 * 1000, y2 * 1000))
            print("  在画面里点你要放的位置（框外也能点，但可能超工作区）")
            print("  要是臂还挡着一部分，按键盘 b 原地再后退 80mm（可连按）")
            p = lv.wait_click(
                status="LEFT click = drop here   b = back off   RIGHT/Esc = auto swap",
                on_key=_key)
        except Exception as e:
            print("  ⚠ 投放区点选出问题（%s），改用自动换边" % e)
            p = None
        finally:
            lv.set_region(None)
        if p is None:
            print("  没点选（或按了取消），改用自动换边")
        else:
            P = pixel_to_base(p[0], p[1], T_base_cam, K, dist, z_drop)
            if P is None:
                print("  ⚠ 这个像素和桌面平面没有交点，改用自动换边")
            else:
                # 必须和抓取走同一套 XY 修正！pixel_to_base 给的是"相机测到"
                # 的坐标，抓取那边已经用 calib_xy 的 A/b 换算成真实桌面坐标了。
                # 之前投放漏了这一步，y 就按 1.27 倍缩放地偏（y=-150 处偏 49mm）。
                P_drop = np.array([P[0] * 1000.0, P[1] * 1000.0, P[2] * 1000.0])
                if xy_corr is not None:
                    P_used = apply_xy_correction(P_drop, xy_corr)
                    fix = P_used[:2] - P_drop[:2]
                    print("  原始投影 (%+.0f, %+.0f) mm，XY 修正 %+.1f/%+.1f mm"
                          % (P_drop[0], P_drop[1], fix[0], fix[1]))
                else:
                    P_used = P_drop
                manual_slot = np.array(P_used[:2], dtype=np.float64) + place_dxy
                inside = True
                if pts is not None:
                    try:
                        inside = cv2.pointPolygonTest(
                            np.array(pts, dtype=np.float32),
                            (float(p[0]), float(p[1])), False) >= 0
                    except Exception:
                        inside = True
                tail = "" if inside else "   ⚠ 在推荐区外"
                if abs(place_dxy[0]) + abs(place_dxy[1]) > 0.5:
                    tail += "   （含投放微调 %+.0f/%+.0f）" % (place_dxy[0], place_dxy[1])
                print("  投放点：像素 (%.0f,%.0f) → 物料坐标 (%+.0f, %+.0f) mm%s"
                      % (p[0], p[1], manual_slot[0], manual_slot[1], tail))

    # ---- 6) 投放：默认「左→右、右→左、中间随机」，让下一轮换个位置抓
    print("\n[6/7] 投放物料")
    if lv is not None:
        lv.set_status("[6/7] placing")
    if manual_slot is not None:
        # 用户点选的位置：slot 已经是物料坐标（mm），直接加夹爪偏移
        cand = np.array([manual_slot[0] + d_mm[0], manual_slot[1] + d_mm[1],
                         tgt[2] + 8.0])
        bad = out_of_bounds(tuple(v / 1000.0 for v in cand))
        rad = float(np.hypot(cand[0], cand[1]))
        if bad:
            print("  ⚠ 你点的投放点超出工作区（%s），改放回原处" % "、".join(bad))
            pl6 = np.array([tgt[0], tgt[1], tgt[2] + 8.0])
            where = "原处"
        elif rad > 650.0:
            print("  ⚠ 你点的投放点距底座 %.0f mm（舒适区 650），太远容易卡，"
                  "改放回原处" % rad)
            pl6 = np.array([tgt[0], tgt[1], tgt[2] + 8.0])
            where = "原处"
        else:
            pl6 = cand
            where = "你点的位置"
    else:
        slot = choose_place_slot(P_mm[1], args)
        if slot is None:
            pl6 = np.array([tgt[0], tgt[1], tgt[2] + 8.0])
            where = "原处"
        else:
            # 物料坐标 → link6 目标（同一个夹爪偏移，只是 xy 换了）
            cand = np.array([slot[0] + d_mm[0], slot[1] + d_mm[1], tgt[2] + 8.0])
            bad = out_of_bounds(tuple(v / 1000.0 for v in cand))
            if bad:
                print("  ⚠ 投放点超出工作区（%s），改放回原处" % "、".join(bad))
                pl6 = np.array([tgt[0], tgt[1], tgt[2] + 8.0])
                where = "原处"
            else:
                pl6 = cand
                where = "左" if slot[1] > 0 else "右"
    # 先在预抓取高度平移到投放点正上方，再垂直下落：斜着走会拖着物料横扫台面
    hover = np.array([pl6[0], pl6[1], pre[2]])
    rc, act_h = cli_pose(hover, args.wait, max(args.tol, 22.0), not args.no_force,
                         args.allow_far, settle=False)
    if rc != 130 and act_h is not None:
        # 平移到不到位（这台臂在伸展姿态常差 10~20mm），下面那段"垂直下落"就会
        # 变成斜着落，物料落地偏一截。好在悬停高度离台面还有 60mm，可以在**这个
        # 高度**把 xy 补回来（绝不能在贴着台面时横着走，会刮到旁边的物料）。
        ex, ey = hover[0] - act_h[0], hover[1] - act_h[1]
        if abs(ex) + abs(ey) > 8.0:
            fix = np.array([hover[0] + ex, hover[1] + ey, hover[2]])
            print("  悬停落点差 (%+.0f, %+.0f) mm，在悬停高度补一次再下落"
                  % (act_h[0] - hover[0], act_h[1] - hover[1]))
            cli_pose(fix, args.wait, max(args.tol, 22.0), not args.no_force,
                     args.allow_far, settle=False)
    # 下落这一段必须量高度：臂在伸展姿态常比目标高 10~15mm，直接从高处松爪
    # 物料会砸下去弹跳/歪掉，看起来就是"放得偏"（只补"偏高"这一侧，偏低不动，
    # 免得把物料压进桌面）。
    if args.no_zfix:
        rc, act_b = cli_pose(pl6, args.wait, args.tol, not args.no_force,
                             args.allow_far, settle=False)
    else:
        rc, act_b = cli_pose_zfix(pl6, args.wait, args.tol, not args.no_force,
                                  args.allow_far)
    if act_b is None:
        print("  ⚠ 降不到放置高度，物料还在夹爪里，请手动处理（或 --no-place）")
        return 1
    cli_grip("open", 15)
    print("  已松爪，物料落在%s（物料坐标 %+.0f, %+.0f mm）"
          % (where, pl6[0] - d_mm[0], pl6[1] - d_mm[1]))
    if act_b is not None:
        # 落点差：目标 vs 实际。这里面含"机械臂自身落点误差"（不含视觉/夹爪偏移），
        # 以后觉得放得偏，先看这一行判断是臂走不准还是点位算错。
        print("  link6 实际落点比目标 (%+.1f, %+.1f) mm（臂自身误差，"
              "偏得多就调 --place-dxy）" % (act_b[0] - pl6[0], act_b[1] - pl6[1]))
    # 松爪后必须**先在原地垂直抬起**再横向离开：此时两指还在物料两侧，
    # 从 90mm 高度斜着回 pre 会横扫过去把刚放下的物料刮倒（2026-10-23 实测）。
    lift = np.array([pl6[0], pl6[1], pre[2]])
    cli_pose(lift, args.wait, max(args.tol, 22.0), not args.no_force,
             args.allow_far, settle=False)
    go_home()
    return 0


if __name__ == "__main__":
    sys.exit(main())

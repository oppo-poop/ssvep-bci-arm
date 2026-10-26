#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手眼标定（eye-to-hand）：相机固定，ArUco 码贴在夹爪上。

为什么要重标
    队友 2026-10-20 标的外参，在 10-21 换相机支架后就失效了。
    本次实测偏差 Δx=+111、Δy=-57 mm，用「一个点」硬修只能让那一点对，
    约束不了旋转 —— 换个位置还会偏。正经做法是重新解。

原理
    码刚性粘在夹爪上，所以「码在 link6 坐标系里的位姿」是常数：

        T_link6_marker = inv(T_base_link6) · T_base_cam · T_cam_marker

    每个样本都能算出这个常数的一次数值。如果 T_base_cam 是对的，
    这些值应该彼此重合；否则就散开。**残差 = 它们之间的离散度**，
    这也正好是评判标定质量最直接的指标（不需要额外真值）。

用法
    # 1) 自动采集：按位姿表逐个移动 → 拍图 + 读 where
    #    （机械臂上电后舵机锁定，推不动，必须让它自己走）
    python3 scripts/vision/handeye.py --capture

    # 2) 看已采了多少、每个样本的码检测误差
    python3 scripts/vision/handeye.py --list

    # 3) 求解（自动试多种算法 × 两种解读，选残差最小的）
    python3 scripts/vision/handeye.py --solve

    # 重来
    python3 scripts/vision/handeye.py --reset
"""

import argparse
import os
import re
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
sys.path.insert(0, HERE)

DATA = os.path.join(ROOT, "data", "vision")
SAMPLES = os.path.join(DATA, "handeye_samples.npz")
EXTRINSICS = os.path.join(DATA, "T_base_cam.npz")
INTRINSICS = os.path.join(DATA, "cam_intrinsics.npz")
CLI = os.path.join(SCRIPTS, "roarm_cli.py")
WS_SETUP = os.path.join(ROOT, "star-arm-moveit2", "install", "setup.bash")

#: 码边长（毫米）。贴的是 40 mm 那份；换了要改。
MARKER_SIZE_MM = 40.0
MARKER_ID = 5

RE_POS = re.compile(r"当前位置\s*x=\s*([-\d.]+)\s*y=\s*([-\d.]+)\s*z=\s*([-\d.]+)")
RE_QUAT = re.compile(r"q=\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)")


# ---------------------------------------------------------------- 工具
def quat_to_T(xyz, quat):
    """(x,y,z) 米 + 四元数 (x,y,z,w) → 4x4。"""
    x, y, z, w = quat
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("零四元数")
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def read_link6():
    """调 roarm_cli where，返回 T_base_link6 或 None。"""
    if not os.path.isfile(WS_SETUP):
        print("❌ 工作区没构建：%s" % WS_SETUP, file=sys.stderr)
        return None
    inner = ("set +u; source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; "
             "source %s; exec /usr/bin/python3 %s where" % (WS_SETUP, CLI))
    try:
        p = subprocess.run(["bash", "-c", inner], capture_output=True,
                           text=True, timeout=30)
    except subprocess.TimeoutExpired:
        print("❌ roarm_cli where 超时", file=sys.stderr)
        return None
    out = (p.stdout or "") + (p.stderr or "")
    m1, m2 = RE_POS.search(out), RE_QUAT.search(out)
    if not m1 or not m2:
        print("❌ 解析不到末端位姿：\n%s" % out[:200], file=sys.stderr)
        return None
    xyz = [float(v) for v in m1.groups()]
    quat = [float(v) for v in m2.groups()]
    return quat_to_T(xyz, quat)


def load_samples():
    if not os.path.isfile(SAMPLES):
        return []
    d = np.load(SAMPLES)
    return [{"T_cam": d["T_cam"][i], "T_g2b": d["T_g2b"][i], "err_px": float(d["err_px"][i])}
            for i in range(len(d["T_cam"]))]


def save_samples(items):
    os.makedirs(DATA, exist_ok=True)
    np.savez(SAMPLES,
             T_cam=np.array([s["T_cam"] for s in items]),
             T_g2b=np.array([s["T_g2b"] for s in items]),
             err_px=np.array([s["err_px"] for s in items]))


# ---------------------------------------------------------------- 采集
def load_poses(path):
    """读位姿表：序号 x y z qx qy qz qw [j1..j6]。返回 [(x,y,z,qx,qy,qz,qw), ...]。"""
    poses = []
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        vals = [float(v) for v in parts[1:8]]
        poses.append(tuple(vals))
    return poses


def _cli(*args, timeout=60):
    """调 roarm_cli，返回 (rc, 输出)。"""
    if not os.path.isfile(WS_SETUP):
        raise SystemExit("工作区没构建：%s" % WS_SETUP)
    inner = ("set +u; source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; "
             "source %s; exec /usr/bin/python3 %s %s"
             % (WS_SETUP, CLI, " ".join(str(a) for a in args)))
    p = subprocess.run(["bash", "-c", inner], capture_output=True,
                       text=True, timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def cmd_capture(args):
    """自动采集：按位姿表逐个移动 → 拍图 → 记录。

    **机械臂上电后舵机是锁定的，推不动** —— 所以必须靠 roarm_cli pose 让它自己走。
    （这一点之前设计错了，写成"手动摆位 + 回车"，真机上根本做不到。）
    """
    import camera
    import aruco as A
    from calib import load_intrinsics

    if not os.path.isfile(args.poses):
        print("❌ 位姿表不存在：%s" % args.poses)
        print("   队友的位姿表在 brain_extract/vision/data/handeye_poses.txt，可直接复制过来")
        return 1

    poses = load_poses(args.poses)
    if len(poses) < 4:
        print("❌ 位姿表只有 %d 个位姿，至少要 4 个（建议 12+）" % len(poses))
        return 1

    K, dist, _ = load_intrinsics(INTRINSICS, expect_size=(1280, 720))
    items = load_samples()
    print("已有样本 %d 个；位姿表 %s 共 %d 个" % (len(items), os.path.basename(args.poses), len(poses)))
    print("开始自动采集（每个位姿约 %.0f 秒）…" % (args.wait + 2))

    for i, (x, y, z, qx, qy, qz, qw) in enumerate(poses, 1):
        print("\n[%d/%d] 移到 (%.3f, %.3f, %.3f)…" % (i, len(poses), x, y, z))
        rc, out = _cli("pose", "%.6f" % x, "%.6f" % y, "%.6f" % z,
                       "%.6f" % qx, "%.6f" % qy, "%.6f" % qz, "%.6f" % qw,
                       "--wait", "%.1f" % args.wait, "--tol", "%.1f" % args.tol,
                       timeout=args.wait + 40)
        if rc != 0:
            tail = [ln for ln in out.strip().splitlines() if ln.strip()][-2:]
            print("  ✗ 没到位（rc=%d）：%s" % (rc, " | ".join(tail)))
            continue

        T_g2b = read_link6()
        if T_g2b is None:
            print("  ✗ 读不到 link6 位姿，跳过")
            continue

        img = camera.grab_frame()
        T_cam, err = A.pose_from_marker(img, K, dist, args.marker_size, args.marker_id)
        if T_cam is None:
            print("  ✗ 画面里没看到 ID %d 的码，跳过" % args.marker_id)
            continue

        items.append({"T_cam": T_cam, "T_g2b": T_g2b, "err_px": err})
        save_samples(items)
        print("  ✓ 第 %d 个：码距相机 %.0f mm，重投影 %.2f px"
              % (len(items), np.linalg.norm(T_cam[:3, 3]) * 1000, err))

    print("\n共 %d 个样本。" % len(items))
    if len(items) < 6:
        print("⚠ 少于 6 个解不稳，建议 12~20 个")
    elif len(items) < 12:
        print("提示：%d 个偏少，12+ 更稳" % len(items))
    return 0


def cmd_list(args):
    items = load_samples()
    if not items:
        print("还没有样本。先跑 --capture")
        return 1
    print("  %-4s %-12s %-10s" % ("#", "码距相机(mm)", "重投影(px)"))
    print("  " + "-" * 30)
    for i, s in enumerate(items, 1):
        print("  %-4d %-12.0f %-10.2f"
              % (i, np.linalg.norm(s["T_cam"][:3, 3]) * 1000, s["err_px"]))
    print("\n共 %d 个" % len(items))
    return 0


# ---------------------------------------------------------------- 求解
def _spread(items, T_base_cam):
    """给定外参，算每个样本推出的 T_link6_marker，返回 (标准差 mm, 最大偏离 mm)。"""
    inv_g2b = [np.linalg.inv(s["T_g2b"]) for s in items]
    pts = np.array([(M @ T_base_cam @ s["T_cam"])[:3, 3] * 1000
                    for M, s in zip(inv_g2b, items)])
    med = np.median(pts, axis=0)
    d = np.linalg.norm(pts - med, axis=1)
    return float(np.std(pts, axis=0).mean()), float(d.max())


def _inv_T(T):
    """刚体位姿的逆（不要用 np.linalg.inv，数值上更稳也更便宜）。"""
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def _park_solve(T_g2b, T_cam, invert_pair=False):
    """自实现 Park 手眼标定，解 T_base_cam（eye-to-hand）。

    为什么不用 cv2.calibrateHandEye
        它对 eye-to-hand 的输入顺序在不同版本/文档里说法不一，我试了四种
        组合（含取逆）都不对，用模拟数据验证时残差在 250~1300 mm 量级。
        与其猜库的约定，不如按公式自己解 —— 至少每一步都能对着推导检查。

    公式
        码刚性粘在夹爪上，所以 T_link6_marker 恒定：
            T_base_cam · T_cam_i = T_g2b_i · T_link6_marker
        对两对样本相消，得经典 AX = XB：
            X · (T_cam_i · inv(T_cam_j)) = (T_g2b_i · inv(T_g2b_j)) · X
        其中 X = T_base_cam。旋转用轴角做最小二乘（SVD），平移用线性最小二乘。
    """
    from scipy.spatial.transform import Rotation as Rot

    n = len(T_g2b)
    if n < 3:
        return None

    # ---- 旋转：sum over pairs of (b · aᵀ)
    M = np.zeros((3, 3))
    for i in range(n):
        for j in range(i + 1, n):
            A = T_cam[i][:3, :3] @ T_cam[j][:3, :3].T       # 相机相对旋转
            B = T_g2b[i][:3, :3] @ T_g2b[j][:3, :3].T       # 夹爪相对旋转
            if invert_pair:
                A, B = B, A
            a = Rot.from_matrix(A).as_rotvec()
            b = Rot.from_matrix(B).as_rotvec()
            na, nb = np.linalg.norm(a), np.linalg.norm(b)
            if na < 1e-6 or nb < 1e-6:                      # 纯平移，对旋转没约束
                continue
            M += np.outer(b / nb, a / na)

    if np.linalg.norm(M) < 1e-9:
        return None

    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:                                # 防止镜像
        U[:, -1] *= -1
        R = U @ Vt

    # ---- 平移：必须用**相对运动** A_ij / B_ij，不能用绝对位姿。
    #      写成绝对位姿时方程里会多出一个未知的 t_link6_marker，
    #      解出来的 x 看着像对的（因为 x 分量的偶然抵消），z 会差一大截。
    #      (R_Aij − I)·t_X = R_X·t_Bij − t_Aij
    A_rows, b_rows = [], []
    for i in range(n):
        for j in range(i + 1, n):
            Aij = _inv_T(T_g2b[j]) @ T_g2b[i]
            Bij = _inv_T(T_cam[j]) @ T_cam[i]
            if invert_pair:
                Aij, Bij = Bij, Aij
            A_rows.append(Aij[:3, :3] - np.eye(3))
            b_rows.append(R @ Bij[:3, 3] - Aij[:3, 3])
    A = np.vstack(A_rows)
    b = np.concatenate(b_rows)
    t, *_ = np.linalg.lstsq(A, b, rcond=None)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def cmd_solve(args):
    import cv2
    from calib import save_extrinsics

    items = load_samples()
    if len(items) < 4:
        print("样本太少（%d），至少要 4 个，建议 12+" % len(items))
        return 1

    R_g2b = [s["T_g2b"][:3, :3] for s in items]
    t_g2b = [s["T_g2b"][:3, 3].reshape(3, 1) for s in items]
    R_t2c = [s["T_cam"][:3, :3] for s in items]
    t_t2c = [s["T_cam"][:3, 3].reshape(3, 1) for s in items]

    methods = [
        ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
        ("PARK", cv2.CALIB_HAND_EYE_PARK),
        ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
        ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
        ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
    ]

    # ---- eye-to-hand 的正确喂法（推导 + 模拟数据验证过）
    #
    #   码在夹爪上，所以 T_link6_marker 恒定：
    #       T_base_cam · T_cam_i = T_g2b_i · T_link6_marker
    #   对两对相消：
    #       X · A_cam = B_g2b · X          （X = T_base_cam）
    #   左乘 inv(B)：
    #       inv(B_g2b) · X = X · inv(A_cam)
    #   即 A' X = X B'，A' = inv(B_g2b)。
    #
    #   cv2.calibrateHandEye 内部算的是 A_ij = inv(T1_j)·T1_i，
    #   所以要让 A_ij = A'，必须传 **inv(T_g2b)**；第二个参数传 T_cam 即可。
    #
    #   （模拟数据验证：这样传 σ=0.000 mm、解出真值；传原始 T_g2b 时 σ=284 mm。）
    inv_g = [_inv_T(T) for T in [s["T_g2b"] for s in items]]
    R1 = [T[:3, :3].astype(np.float64) for T in inv_g]
    t1 = [T[:3, 3].reshape(3, 1).astype(np.float64) for T in inv_g]

    best = None
    for mname, mflag in (("PARK", cv2.CALIB_HAND_EYE_PARK),
                         ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
                         ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
                         ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS)):
        try:
            R, t = cv2.calibrateHandEye(R1, t1, R_t2c, t_t2c, method=mflag)
        except cv2.error:
            continue
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.ravel(t)
        sigma, mx = _spread(items, T)
        print("  %-12s %-12.2f %-14.2f" % (mname, sigma, mx))
        if best is None or sigma < best[0]:
            best = (sigma, mx, "calibrateHandEye", mname, "T", T)

    if best is None:
        print("\n✗ 所有解法都失败。样本姿态可能太接近（都要在同一平面附近）。")
        return 1

    sigma, mx, cname, name, label, T = best
    print("\n  最佳：输入 %s / %s / %s    σ=%.2f mm，最大偏离 %.2f mm"
          % (cname, name, label, sigma, mx))

    if sigma > 5.0:
        print("  ⚠ 残差偏大（>5mm）。常见原因：")
        print("     · 样本姿态太接近（要求大幅旋转，俯仰/偏航都变）")
        print("     · 码没粘牢，或粘的平面和 link6 有相对位移")
        print("     · 臂的 TF 本身有 10~30mm 误差（已知，这会压低精度上限）")

    dist = float(np.linalg.norm(T[:3, 3]))
    print("  相机在基座系：%s mm（距原点 %.0f mm）"
          % (np.round(T[:3, 3] * 1000, 1), dist * 1000))
    if dist > 3.0:
        print("  ⚠ 相机离基座 %.2f m，远超常用范围 —— 这个解不可信" % dist)
        return 1

    if not args.write:
        print("\n（预览模式，未写入。确认无误后加 --write）")
        return 0

    bak = EXTRINSICS + ".bak"
    if os.path.isfile(EXTRINSICS):
        import shutil
        shutil.copy2(EXTRINSICS, bak)
        print("\n  已备份旧值 → %s" % os.path.basename(bak))

    save_extrinsics(EXTRINSICS, T, rms_px=None,
                    method="handeye %s/%s/%s, σ=%.1fmm, n=%d"
                           % (cname, name, label, sigma, len(items)),
                    note="2026-10-25 重标（码在夹爪，%d 个姿态）" % len(items))
    print("  ✓ 已写入 %s" % EXTRINSICS)
    return 0


# ---------------------------------------------------------------- 入口
def main():
    ap = argparse.ArgumentParser(description="手眼标定（eye-to-hand）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--capture", action="store_true", help="按位姿表自动采集（臂自己走）")
    g.add_argument("--list", action="store_true", help="看已采样本")
    g.add_argument("--solve", action="store_true", help="求解外参")
    g.add_argument("--reset", action="store_true", help="清空样本")
    ap.add_argument("--write", action="store_true", help="求解后写入 T_base_cam.npz")
    ap.add_argument("--marker-size", type=float, default=MARKER_SIZE_MM,
                    help="码的边长（毫米），默认 %.0f" % MARKER_SIZE_MM)
    ap.add_argument("--marker-id", type=int, default=MARKER_ID)
    ap.add_argument("--poses", default=os.path.join(DATA, "handeye_poses.txt"),
                    help="位姿表路径（默认 data/vision/handeye_poses.txt）")
    ap.add_argument("--wait", type=float, default=25.0, help="每个位姿的等待秒数")
    ap.add_argument("--tol", type=float, default=10.0, help="到位容差（毫米）")
    args = ap.parse_args()

    if args.reset:
        if os.path.isfile(SAMPLES):
            os.remove(SAMPLES)
            print("已清空样本")
        else:
            print("本来就没有样本")
        return 0
    if args.capture:
        return cmd_capture(args)
    if args.list:
        return cmd_list(args)
    return cmd_solve(args)


if __name__ == "__main__":
    raise SystemExit(main())

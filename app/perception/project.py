#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""像素 → 机械臂基座坐标（桌面平面假设）。

移植自队友的 `vision/scripts/grasp_xy.py`（2026-10-22 交付），公式一致，
只做了三件事：去掉对特定目录结构的依赖、参数全部显式传入、加自测。

原理
    已知相机内参 K/dist、相机在基座系的位姿 T_base_cam，以及抓取平面高度 z_g：

        1. 去畸变            (u,v) → 相机系归一化视线 d_cam
        2. 转到基座系        d = R_base_cam @ d_cam
        3. 与平面 z=z_g 求交  t = (z_g - O_z) / d_z
        4. P = O + t · d

    适用于"物料躺在桌面上"这一前提。物体越高，用平面假设算出的 xy 偏差越大
    （偏差 ≈ h · tan(俯角)）；70 mm 高、俯角 55° 时约 100 mm，必须改用深度或
    用检测框底边（接触点）。

自测
    python3 scripts/vision/project.py
    用队友 2026-10-21 日志里的一个真实点做回归：像素 (684.5, 621.3)
    应得到基座系 (419.22, 29.60) mm，平面 z=92.60 mm。
"""

import numpy as np


def pixel_to_base(u, v, K, dist, T_base_cam, z_plane):
    """像素 → 基座系坐标（米）。视线与平面无交点时返回 None。

    Args:
        u, v:        像素坐标
        K:           3x3 内参矩阵
        dist:        畸变系数（1x4 / 1x5 均可，传 None 表示不去畸变）
        T_base_cam:  4x4，相机在基座系的位姿
        z_plane:     抓取平面高度（米），一般取场景的 pick.z

    Returns:
        np.ndarray(3,) 或 None
    """
    import cv2

    K = np.asarray(K, dtype=np.float64)
    T = np.asarray(T_base_cam, dtype=np.float64)

    # 防呆：K 和 T_base_cam 传反时，cv2 只会报一句
    # "CV_IS_MAT(_cameraMatrix) && rows == 3" —— 完全看不出是参数顺序问题。
    # 这里直接点名。移植队友代码时踩过一次（他们的签名是 u,v,T,K,dist,z）。
    if K.shape == (4, 4) or T.shape == (3, 3):
        raise ValueError(
            "参数顺序疑似传反：本函数签名是 "
            "pixel_to_base(u, v, K, dist, T_base_cam, z_plane)，"
            "其中 K 为 3x3、T_base_cam 为 4x4。"
            "收到 K%s / T%s" % (K.shape, T.shape))
    if T.shape != (4, 4):
        raise ValueError("T_base_cam 必须是 4x4，收到 %s" % (T.shape,))

    if dist is None:
        # 不去畸变：直接归一化
        x = (u - K[0, 2]) / K[0, 0]
        y = (v - K[1, 2]) / K[1, 1]
        d_cam = np.array([x, y, 1.0])
    else:
        pts = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float64), K,
            np.asarray(dist, dtype=np.float64).reshape(-1, 1))
        d_cam = np.array([pts[0, 0, 0], pts[0, 0, 1], 1.0])

    R = T[:3, :3]
    O = T[:3, 3]
    d = R @ d_cam

    if abs(d[2]) < 1e-9:
        return None                      # 视线平行于平面
    t = (z_plane - O[2]) / d[2]
    if t <= 0:
        return None                      # 平面在相机背后
    return O + t * d


def project_many(uv_list, K, dist, T_base_cam, z_plane):
    """批量版：[(u,v), ...] → [(x,y,z), ...]，失败的项为 None。"""
    return [pixel_to_base(u, v, K, dist, T_base_cam, z_plane)
            for u, v in uv_list]


# ---------------------------------------------------------------- 自测
#: 队友 handeye_result.npz + calib_intrinsics_clean.npz 的实际值
_K = [[994.44, 0.0, 651.79],
      [0.0, 983.01, 368.40],
      [0.0, 0.0, 1.0]]
_DIST = [0.064961, 0.016781, 0.000355, -0.000854, -0.446533]
def _selftest():
    """用真实标定数据验证公式（需要队友的 npz，找不到就跳过）。"""
    import os
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    cand = [
        os.path.join(here, "..", "..", "data", "handeye_result.npz"),
        os.path.join(here, "..", "..", "data", "vision", "handeye_result.npz"),
    ]
    base = os.environ.get("TEAMMATE_VISION_DIR")
    if base:
        cand.insert(0, os.path.join(base, "data", "handeye_result.npz"))
    npz = next((p for p in cand if os.path.isfile(p)), None)
    if npz is None:
        print("跳过：找不到 handeye_result.npz（可用 TEAMMATE_VISION_DIR 指定队友目录）")
        return 0

    T = np.load(npz)["T_base_cam"]
    K = np.array(_K)
    dist = np.array(_DIST)
    # 队友日志第一条：像素 (684.5, 621.3) → 基座 (419.22, 29.60) mm，平面 92.60 mm
    P = pixel_to_base(684.5, 621.3, K, dist, T, 0.0926)
    if P is None:
        print("✗ 视线与平面无交点")
        return 1
    got = P * 1000.0
    want = np.array([419.22, 29.60, 92.60])
    err = np.linalg.norm(got[:2] - want[:2])
    print("  像素 (684.5, 621.3)")
    print("  期望 (%.2f, %.2f) mm" % (want[0], want[1]))
    print("  实算 (%.2f, %.2f) mm" % (got[0], got[1]))
    print("  偏差 %.3f mm" % err)
    ok = err < 1.0
    print("  " + ("✓ 通过" if ok else "✗ 偏差过大"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())

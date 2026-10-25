#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用一次实测点修正外参平移（换支架后的应急手段）。

适用条件
    偏差主要来自**相机平移**（比如换支架时相机往前挪了 3 cm），
    旋转没变。这种情况下，视觉算出的所有点会**同向偏相同的量**。

    如果偏差随距离变化（近处准、远处偏），那是旋转错了，
    本工具修不准 —— 得重标。

原理
    视觉算出 P_v = f(T, 像素, z_plane)
    实测得到 P_m（把夹爪移到该点正上方、读 where 的 x,y）
    平移偏差 Δ = P_m − P_v
    修正：T[0:3, 3] += Δ

    把视线方向按相机移动 Δ 平移，等价于交点也平移 Δ（平面不变时严格成立）。

用法
    # 只看修正后长什么样，不写文件
    python3 scripts/vision/fix_extrinsics.py \
        --pixel 611 632 --z 0.0926 --measured 530 70

    # 确认无误再写入
    python3 scripts/vision/fix_extrinsics.py \
        --pixel 611 632 --z 0.0926 --measured 530 70 --write

    # 写入前会自动备份成 T_base_cam.npz.bak
"""

import argparse
import os
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from calib import check_pose, load_intrinsics, load_extrinsics, save_extrinsics  # noqa: E402
from project import pixel_to_base  # noqa: E402

DEFAULT_INTR = os.path.join(HERE, "..", "..", "data", "vision", "cam_intrinsics.npz")
DEFAULT_EXTR = os.path.join(HERE, "..", "..", "data", "vision", "T_base_cam.npz")


def main():
    ap = argparse.ArgumentParser(description="用一次实测点修正外参平移")
    ap.add_argument("--intrinsics", default=os.path.normpath(DEFAULT_INTR))
    ap.add_argument("--extrinsics", default=os.path.normpath(DEFAULT_EXTR))
    ap.add_argument("--pixel", nargs=2, type=float, required=True, metavar=("U", "V"),
                    help="物料上那个点的像素坐标")
    ap.add_argument("--z", type=float, required=True, help="该点所在平面的高度（米）")
    ap.add_argument("--measured", nargs=2, type=float, required=True,
                    metavar=("X_MM", "Y_MM"), help="该点的实测基座坐标（毫米）")
    ap.add_argument("--write", action="store_true", help="写回外参文件（默认只预览）")
    args = ap.parse_args()

    K, dist, _ = load_intrinsics(args.intrinsics)
    T, meta = load_extrinsics(args.extrinsics)

    P_v = pixel_to_base(args.pixel[0], args.pixel[1], K, dist, T, args.z)
    if P_v is None:
        print("❌ 该像素与平面 z=%.4f 无交点" % args.z)
        return 1

    P_m = np.array([args.measured[0], args.measured[1], args.z * 1000.0])
    delta = P_m[:2] - P_v[:2] * 1000.0

    print("  当前外参：相机在基座系 %s mm"
          % np.round(T[:3, 3] * 1000, 1))
    print("  像素 (%g, %g) @ z=%.4f m" % (args.pixel[0], args.pixel[1], args.z))
    print("  视觉算出   x=%+8.1f  y=%+8.1f mm" % (P_v[0] * 1000, P_v[1] * 1000))
    print("  实测       x=%+8.1f  y=%+8.1f mm" % (P_m[0], P_m[1]))
    print("  偏差       Δx=%+7.1f  Δy=%+7.1f mm   |Δ|=%.1f mm"
          % (delta[0], delta[1], np.hypot(*delta)))
    print()

    T_new = T.copy()
    T_new[0, 3] += delta[0] / 1000.0
    T_new[1, 3] += delta[1] / 1000.0

    try:
        check_pose(T_new)
    except ValueError as e:
        print("❌ 修正后位姿不合法：%s" % e)
        return 1

    # 验证：用新 T 重算，应该正好落到实测点
    P_after = pixel_to_base(args.pixel[0], args.pixel[1], K, dist, T_new, args.z)
    err = np.hypot(P_after[0] * 1000 - P_m[0], P_after[1] * 1000 - P_m[1])
    print("  修正后相机在基座系 %s mm" % np.round(T_new[:3, 3] * 1000, 1))
    print("  修正后该点重算 x=%+.1f y=%+.1f mm（残差 %.2f mm）"
          % (P_after[0] * 1000, P_after[1] * 1000, err))
    print()

    if err > 0.5:
        print("⚠ 残差偏大 —— 说明偏差不只是平移，可能有旋转误差，本工具修不准")
        return 1

    print("⚠ 这是**应急修正**：只保证这一处对的，别处的误差取决于偏差是否真为纯平移。")
    print("   如果修正后换个位置还是偏，就必须用 ArUco 板重标外参。")
    print()

    if not args.write:
        print("（预览模式，未写入。确认无误后加 --write）")
        return 0

    bak = args.extrinsics + ".bak"
    shutil.copy2(args.extrinsics, bak)
    save_extrinsics(
        args.extrinsics, T_new,
        rms_px=meta.get("rms_px"),
        method=(meta.get("method") or "") + "+x1点平移修正",
        note="%s | 用像素(%g,%g)@z=%.4f 与实测(%g,%g)修正，Δ=(%.1f,%.1f)mm"
             % (meta.get("note", ""), args.pixel[0], args.pixel[1], args.z,
                args.measured[0], args.measured[1], delta[0], delta[1]))
    print("✓ 已写入 %s" % args.extrinsics)
    print("  备份     %s" % bak)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

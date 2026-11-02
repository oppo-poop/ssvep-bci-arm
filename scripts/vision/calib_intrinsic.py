#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Astra Pro 彩色相机内参标定（4 码 ArUco 板）

用法:
    .venv_astra/bin/python calib_intrinsic.py --a 30 --dx 200 --dy 150

参数:
    --a   单个 ArUco 码的黑色方块边长 (mm)
    --dx  左上码中心 -> 右上码中心的水平距离 (mm)
    --dy  左上码中心 -> 左下码中心的垂直距离 (mm)
    三个数用同一单位即可（内参只与比值有关，但你后面做手眼标定要真实尺度）

操作:
    空格  采集一帧（要求 4 个码全部可见）
    ESC   结束采集并计算内参

输出:
    calib_intrinsics.npz   内参矩阵 K + 畸变系数 dist
    calib_images/          采集的原始图片
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np
import cv2.aruco as aruco

HERE = os.path.dirname(os.path.abspath(__file__))   # scripts/vision/
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
DATA = os.path.join(ROOT, "data", "vision")
OUT_IMG = os.path.join(DATA, "calib_images")
OUT_NPZ = os.path.join(DATA, "calib_intrinsics_raw.npz")

sys.path.insert(0, HERE)
import aruco as _aru   # noqa: E402

DICT = aruco.DICT_6X6_50
# 板上四个码的 ID -> 在板坐标系中的位置
# 板坐标系: 原点在四码中心, X 向右, Y 向下(与 ArUco 码自身朝向一致)
GRID = {4: (-1, -1), 3: (+1, -1), 2: (-1, +1), 1: (+1, +1)}

# 实测比例(用真实照片拟合, 残差 0.5px): dx/a=5.450, dy/a=3.505
RATIO_DX = 5.450
RATIO_DY = 3.505


def find_astra_color_index():
    for d in sorted(glob.glob('/sys/class/video4linux/video*')):
        try:
            with open(d + '/name') as f:
                if 'astra' in f.read().lower():
                    return int(d.rsplit('video', 1)[1])
        except OSError:
            continue
    return None


def board_object_points(a, dx, dy):
    """返回 {id: 4x3 板坐标系下的角点}, 顺序与 ArUco 返回一致: 左上/右上/右下/左下"""
    pts = {}
    for mid, (sx, sy) in GRID.items():
        cx, cy = sx * dx / 2.0, sy * dy / 2.0
        # 角点顺序必须与 ArUco 返回一致: 0 右上, 1 左上, 2 左下, 3 右下
        # (这块板的码朝向如此, 与坐标轴 Y 向下对应)
        pts[mid] = np.array([
            [cx + a / 2, cy + a / 2, 0],
            [cx - a / 2, cy + a / 2, 0],
            [cx - a / 2, cy - a / 2, 0],
            [cx + a / 2, cy - a / 2, 0],
        ], dtype=np.float32)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", type=float, required=True, help="码的黑色方块边长 mm(只需量这一个)")
    ap.add_argument("--dx", type=float, default=None, help="横向码中心距 mm(默认 5.450*a)")
    ap.add_argument("--dy", type=float, default=None, help="纵向码中心距 mm(默认 3.505*a)")
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    args = ap.parse_args()

    if args.dx is None:
        args.dx = RATIO_DX * args.a
    if args.dy is None:
        args.dy = RATIO_DY * args.a
    print(f"板参数: a={args.a} dx={args.dx:.1f} dy={args.dy:.1f} (mm)")

    idx = find_astra_color_index()
    if idx is None:
        sys.exit("没找到 Astra 彩色设备")
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    if not cap.isOpened():
        sys.exit(f"打不开 /dev/video{idx}")
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"彩色 {aw}x{ah}")

    # 检测走自己的兼容层：本机 OpenCV 4.5.4 没有 ArucoDetector 类
    detect, dictionary = _aru.make_detector(DICT)

    os.makedirs(OUT_IMG, exist_ok=True)
    obj_pts_by_id = board_object_points(args.a, args.dx, args.dy)

    all_obj, all_img = [], []
    win = "Calibration - SPACE capture / ESC finish"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    print("空格采集 / ESC 结束")

    count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = detect(gray)
        view = frame.copy()

        found = 0 if ids is None else len(ids.ravel())
        all_four = ids is not None and set(ids.ravel().tolist()) == set(GRID)

        if found:
            aruco.drawDetectedMarkers(view, corners, ids)
        cv2.putText(view, f"markers {found}/4   captured {count}",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0) if all_four else (0, 0, 255), 2)
        if not all_four:
            cv2.putText(view, "need ALL 4 markers", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.imshow(win, view)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == 32 and all_four:
            # 按检出的 ID 顺序组装对应的 3D 点
            id_list = ids.ravel().tolist()
            objp = np.concatenate([obj_pts_by_id[i] for i in id_list], axis=0)
            imgp = np.concatenate([c[0] for c in corners], axis=0).astype(np.float32)
            all_obj.append(objp.reshape(-1, 1, 3))
            all_img.append(imgp.reshape(-1, 1, 2))
            path = os.path.join(OUT_IMG, f"calib_{count:02d}.png")
            cv2.imwrite(path, frame)
            count += 1
            print(f"采集第 {count} 张 -> {os.path.basename(path)}")

    cap.release()
    cv2.destroyAllWindows()

    if count < 8:
        print(f"只采集了 {count} 张, 至少需要 8 张(推荐 20 张以上), 未计算内参")
        return 1

    print(f"开始计算内参, 共 {count} 张, 图像 {aw}x{ah} ...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        all_obj, all_img, (aw, ah), None, None, flags=0)
    print(f"\n重投影误差 RMS = {rms:.3f} px  (应 < 0.5)")
    print(f"fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}")
    print(f"畸变 = {dist.ravel()}")
    np.savez(OUT_NPZ, K=K, dist=dist, rms=rms, size=np.array([aw, ah]),
             a=args.a, dx=args.dx, dy=args.dy)
    print(f"已保存 {OUT_NPZ}")

    # 每张的误差
    for i, (o, im, r, t) in enumerate(zip(all_obj, all_img, rvecs, tvecs)):
        proj, _ = cv2.projectPoints(o, r, t, K, dist)
        err = np.linalg.norm(proj - im, axis=2).mean()
        print(f"  {os.path.basename(sorted(glob.glob(os.path.join(OUT_IMG, '*.png')))[i]):>14s}  {err:.3f} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())

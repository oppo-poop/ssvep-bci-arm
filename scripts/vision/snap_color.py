#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只用彩色相机拍一张（不碰 OpenNI2，最稳）。

用法:
    python3 scripts/vision/snap_color.py
    python3 scripts/vision/snap_color.py --out view.png --w 1280 --h 720

输出: 默认 data/vision/preview/color.png
"""
import argparse
import glob
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))   # scripts/vision
VISION = os.path.dirname(HERE)                      # scripts/
if VISION not in sys.path:
    sys.path.insert(0, VISION)

from roarm_lib import paths  # noqa: E402


from camera import find_camera_index


def find_astra_color_index():
    """兼容旧名字。设备发现统一走 camera.find_camera_index
    （CAM_INDEX 环境变量 > 名字匹配 realsense/astra/orbbec > 第一个）。"""
    return find_camera_index()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(paths.DATA_VISION / "preview" / "color.png"))
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--warmup", type=int, default=20)
    args = ap.parse_args()

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

    img = None
    for _ in range(args.warmup):      # 预热等自动曝光稳定
        ok, img = cap.read()
        if not ok:
            img = None
        time.sleep(0.03)
    if img is None:
        sys.exit("读不到画面")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    cv2.imwrite(args.out, img)
    print(f"已保存 {args.out}  ({img.shape[1]}x{img.shape[0]})")
    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())

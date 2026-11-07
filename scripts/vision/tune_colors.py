#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时调物料颜色阈值（移植自队友 cap/tune_colors.py）。

看摄像头画面拖滑块，左边原始图+轮廓，右边掩码。
鼠标左键点画面 → 打印该点周围 9x9 的 HSV 中位数。
按 s 打印可直接粘进 config/vision.json 的 material 片段。

用法：
    python3 scripts/vision/tune_colors.py --color green
    python3 scripts/vision/tune_colors.py --color red --index 0
    python3 scripts/vision/tune_colors.py --freeze     # 冻结首帧调参

按键：
    s  打印当前参数    r  恢复初始值    q/Esc  退出
"""

import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from roarm_lib import params  # noqa: E402

WIN = "tune_colors"


def open_camera():
    from camera import find_camera_index
    idx = find_camera_index()
    if idx is None:
        sys.exit("没找到 /dev/video*；插相机或用 CAM_INDEX=2 指定")
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    for _ in range(12):
        cap.read()
    return cap


def main():
    ap = argparse.ArgumentParser(description="实时调物料 HSV 阈值")
    ap.add_argument("--color", default="green", choices=list(params.COLOR_RANGES))
    ap.add_argument("--index", type=int, default=0,
                    help="多段区间时调第几段（默认 0）")
    ap.add_argument("--freeze", action="store_true", help="冻结第一帧调参")
    args = ap.parse_args()

    ranges = params.COLOR_RANGES[args.color]
    if not ranges:
        sys.exit("config 里没有 %s 的 material 区间" % args.color)
    idx = max(0, min(args.index, len(ranges) - 1))
    lo0, hi0 = ranges[idx]
    h_lo0, s_lo0, v_lo0 = (int(v) for v in lo0)
    h_hi0 = int(hi0[0])
    print("调 %s 第 %d 段：H %d~%d  S≥%d  V≥%d"
          % (args.color, idx, h_lo0, h_hi0, s_lo0, v_lo0))
    print("左键点画面看 HSV；按 s 打印参数；q 退出。")

    cap = open_camera()
    base = None
    if args.freeze:
        for _ in range(10):
            ok, f = cap.read()
            if ok and f is not None:
                base = f
                break
        if base is None:
            sys.exit("没拿到画面")

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 480)
    cv2.createTrackbar("H low", WIN, h_lo0, 179, lambda x: None)
    cv2.createTrackbar("H high", WIN, h_hi0, 179, lambda x: None)
    cv2.createTrackbar("S low", WIN, s_lo0, 255, lambda x: None)
    cv2.createTrackbar("V low", WIN, v_lo0, 255, lambda x: None)

    clicked = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append((x, y))

    cv2.setMouseCallback(WIN, on_mouse)

    while True:
        if args.freeze:
            img = base
        else:
            ok, img = cap.read()
            if not ok or img is None:
                continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hl = cv2.getTrackbarPos("H low", WIN)
        hh = cv2.getTrackbarPos("H high", WIN)
        sl = cv2.getTrackbarPos("S low", WIN)
        vl = cv2.getTrackbarPos("V low", WIN)
        if hh <= hl:
            hh = hl + 1

        m = cv2.inRange(hsv, (hl, sl, vl), (hh, 255, 255))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        n, _lab, stats, _cents = cv2.connectedComponentsWithStats(m, 8)
        vis = img.copy()
        big = 0
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 300:
                continue
            big += 1
            x, y = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
            w, h = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(vis, "%d" % area, (x, max(16, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        while clicked:
            cx, cy = clicked.pop()
            r = 4
            y0, y1 = max(0, cy - r), min(hsv.shape[0], cy + r)
            x0, x1 = max(0, cx - r), min(hsv.shape[1], cx + r)
            patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
            hh_, ss_, vv_ = (int(np.median(patch[:, k])) for k in range(3))
            print("  点(%d,%d) → H %3d  S %3d  V %3d" % (cx, cy, hh_, ss_, vv_))
            cv2.drawMarker(vis, (cx, cy), (255, 255, 255),
                           cv2.MARKER_CROSS, 22, 2)

        info = "H %d~%d  S>=%d  V>=%d   块数 %d" % (hl, hh, sl, vl, big)
        cv2.putText(vis, info, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2)
        mv = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        both = np.hstack([cv2.resize(vis, (640, 360)),
                          cv2.resize(mv, (640, 360))])
        cv2.imshow(WIN, both)

        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("s"):
            print("\n当前参数（%s 第 %d 段），可粘进 config/vision.json："
                  % (args.color, idx))
            print('  "material": [\n'
                  '    [[%d, %d, %d], [%d, 255, 255]]\n'
                  '  ]' % (hl, sl, vl, hh))
        if k == ord("r"):
            cv2.setTrackbarPos("H low", WIN, h_lo0)
            cv2.setTrackbarPos("H high", WIN, h_hi0)
            cv2.setTrackbarPos("S low", WIN, s_lo0)
            cv2.setTrackbarPos("V low", WIN, v_lo0)

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
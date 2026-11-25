#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时 HSV 滑块调参工具（支持物料 material 与托盘 tray，按 s 直接写入 config/static/vision.json）。

用法：
    python3 tools/diag/tune_colors.py --color green                  # 调绿物料
    python3 tools/diag/tune_colors.py --color green --kind tray      # 调绿托盘
    python3 tools/diag/tune_colors.py --color red --index 1          # 调红物料第 2 段 (173~179)
    python3 tools/diag/tune_colors.py --color blue --freeze          # 冻结首帧调参

按键：
    s / w   直接写入 config/static/vision.json
    f       切换 冻结画面 / 实时画面
    r       恢复本次启动时的初始值
    q / Esc 退出
    左键点击左侧画面 → 在终端打印该点周围 9x9 的 HSV 中位数
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "app" / "core" / "paths.py").is_file())
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from app.core import paths  # noqa: E402

WIN = "tune_colors"


def open_camera():
    from app.perception.camera import find_camera_index
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
    vision_path = paths.VISION_JSON
    cfg = json.loads(vision_path.read_text(encoding="utf-8"))
    colors = list(cfg["colors"].keys())

    ap = argparse.ArgumentParser(description="实时滑块调 HSV 并直接写入 config/static/vision.json")
    ap.add_argument("--color", default="green", choices=colors)
    ap.add_argument("--kind", default="material", choices=("material", "tray"),
                    help="调物料(material)还是托盘(tray)，默认 material")
    ap.add_argument("--index", type=int, default=0, help="多段区间时调第几段（如红物料第 2 段用 --index 1）")
    ap.add_argument("--freeze", action="store_true", help="启动即冻结首帧")
    args = ap.parse_args()

    ranges = cfg["colors"][args.color].get(args.kind, [])
    if not ranges:
        ranges = [[[0, 100, 80], [179, 255, 255]]]
        cfg["colors"][args.color][args.kind] = ranges
    idx = max(0, min(args.index, len(ranges) - 1))
    lo0, hi0 = [int(v) for v in ranges[idx][0]], [int(v) for v in ranges[idx][1]]

    print(f"正在调 [{args.color}.{args.kind}[{idx}]] 初始: {lo0} ~ {hi0}")
    print("操作：拖滑块调掩码 | 左键点图看 HSV | s/w 直接写入 vision.json | f 冻结/实时 | r 复位 | q 退出")

    cap = open_camera()
    frozen = args.freeze
    frame_buf = None
    for _ in range(10):
        ok, f = cap.read()
        if ok and f is not None:
            frame_buf = f
            break
    if frame_buf is None:
        sys.exit("没拿到相机画面")

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 520)
    cv2.createTrackbar("H low",  WIN, lo0[0], 179, lambda x: None)
    cv2.createTrackbar("H high", WIN, hi0[0], 179, lambda x: None)
    cv2.createTrackbar("S low",  WIN, lo0[1], 255, lambda x: None)
    cv2.createTrackbar("S high", WIN, hi0[1], 255, lambda x: None)
    cv2.createTrackbar("V low",  WIN, lo0[2], 255, lambda x: None)
    cv2.createTrackbar("V high", WIN, hi0[2], 255, lambda x: None)

    clicked = []
    saved_until = 0.0

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append((x, y))

    cv2.setMouseCallback(WIN, on_mouse)

    while True:
        if not frozen:
            ok, img = cap.read()
            if ok and img is not None:
                frame_buf = img
        img = frame_buf
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hl = cv2.getTrackbarPos("H low", WIN)
        hh = max(hl, cv2.getTrackbarPos("H high", WIN))
        sl = cv2.getTrackbarPos("S low", WIN)
        sh = max(sl, cv2.getTrackbarPos("S high", WIN))
        vl = cv2.getTrackbarPos("V low", WIN)
        vh = max(vl, cv2.getTrackbarPos("V high", WIN))

        m = cv2.inRange(hsv, (hl, sl, vl), (hh, sh, vh))
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
            cv2.putText(vis, str(area), (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        while clicked:
            mx, my = clicked.pop()
            # 拼接显示缩放为 640x360，映射回 1280x720 原图坐标
            cx = min(img.shape[1] - 1, (mx % 640) * 2)
            cy = min(img.shape[0] - 1, my * 2)
            r = 4
            patch = hsv[max(0, cy - r):min(hsv.shape[0], cy + r + 1),
                        max(0, cx - r):min(hsv.shape[1], cx + r + 1)].reshape(-1, 3)
            hm, sm, vm = (int(np.median(patch[:, k])) for k in range(3))
            print(f"  点({cx},{cy}) -> H={hm:3d}  S={sm:3d}  V={vm:3d}")
            cv2.drawMarker(vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)

        info = f"{args.color}.{args.kind}[{idx}]  H:{hl}-{hh} S:{sl}-{sh} V:{vl}-{vh}  cnt={big}  {'[FROZEN]' if frozen else '[LIVE]'}"
        cv2.putText(vis, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2)
        if time.time() < saved_until:
            cv2.putText(vis, "SAVED to config/static/vision.json!", (10, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        both = np.hstack([cv2.resize(vis, (640, 360)),
                          cv2.resize(cv2.cvtColor(m, cv2.COLOR_GRAY2BGR), (640, 360))])
        cv2.imshow(WIN, both)

        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("f"):
            frozen = not frozen
        elif k in (ord("s"), ord("w")):
            cfg = json.loads(vision_path.read_text(encoding="utf-8"))
            cfg["colors"][args.color].setdefault(args.kind, ranges)
            cfg["colors"][args.color][args.kind][idx] = [[hl, sl, vl], [hh, sh, vh]]
            vision_path.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            saved_until = time.time() + 2.0
            print(f"✓ 已写入 {vision_path}: {args.color}.{args.kind}[{idx}] = [[{hl},{sl},{vl}], [{hh},{sh},{vh}]]")
        elif k == ord("r"):
            for name, val in [("H low", lo0[0]), ("H high", hi0[0]),
                              ("S low", lo0[1]), ("S high", hi0[1]),
                              ("V low", lo0[2]), ("V high", hi0[2])]:
                cv2.setTrackbarPos(name, WIN, val)

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())

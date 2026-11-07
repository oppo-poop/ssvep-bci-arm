#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摄像头/图片实时颜色检测 —— 用移植后的 color_detect 判据。

和 `pick.py --auto` 完全同一套检测：
    V 均值、面积上下限、宽高比/“立着”、可达范围、同色选最近底座。

用法：
    python3 scripts/vision/detect_color.py                    # 实时，默认绿色
    python3 scripts/vision/detect_color.py --color red
    python3 scripts/vision/detect_color.py --image xxx.png    # 单张

实时按键：
    q / Esc  退出
    s        保存叠加图
    c        切换 red→green→blue→yellow
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from roarm_lib import params, paths  # noqa: E402
import color_detect as cd  # noqa: E402

COLORS = tuple(cd.NAMES)
BGR = {"red": (0, 0, 255), "green": (0, 255, 0),
       "blue": (255, 0, 0), "yellow": (0, 255, 255)}


def detect(img, color, calib, z_grasp, z_table, max_area, no_tall,
           with_pads=False):
    shape_kw = {}
    if no_tall:
        shape_kw = {"min_shape": 0.0, "aspect_range": (0.0, 99.0)}
    found, report = cd.find_targets_adaptive(
        img, calib, z_grasp, max_area=max_area, **shape_kw)
    pads = cd.find_pads(img, calib, z_table) if with_pads else None
    return found, report, pads


def draw(img, found, color, pads=None):
    out = img.copy()
    if pads:
        for name, pad in pads.items():
            if not pad:
                continue
            bx, by, bw, bh = pad["box"]
            col = BGR.get(name, (255, 255, 255))
            cv2.rectangle(out, (int(bx), int(by)),
                          (int(bx + bw), int(by + bh)), col, 1)
            cv2.putText(out, "%s纸" % name, (int(bx), int(by) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    for name, hit in found.items():
        if not hit:
            continue
        bx, by, bw, bh = hit["box"]
        col = BGR.get(name, (255, 255, 255))
        thick = 3 if name == color else 1
        cv2.rectangle(out, (int(bx), int(by)),
                      (int(bx + bw), int(by + bh)), col, thick)
        u, v = hit["uv"]
        cv2.drawMarker(out, (int(u), int(v)), col, cv2.MARKER_CROSS, 22, 2)
        cv2.putText(out, "%s %.0f" % (name, hit["area"]),
                    (int(bx), max(18, int(by) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    hit = found.get(color)
    if hit:
        status = ("%s  uv=(%.0f,%.0f) area=%.0f Vmean=%.0f base=(%.0f,%.0f)"
                  % (color, hit["uv"][0], hit["uv"][1], hit["area"],
                     hit["v_mean"], hit["base"][0], hit["base"][1]))
    else:
        status = "%s  NOT FOUND" % color
    if pads:
        parts = []
        for name in cd.PAD_NAMES:
            p = pads.get(name)
            parts.append("%s纸%s" % (name, "✓" if p else "✗"))
        status += "  pads: " + " ".join(parts)
    cv2.rectangle(out, (8, 8), (8 + 9 * len(status), 38), (32, 32, 32), -1)
    cv2.putText(out, status, (14, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255), 1, cv2.LINE_AA)
    return out, status


def save_overlay(out, color, out_path=None):
    path = out_path or os.path.join(
        paths.DATA_VISION, "preview", "detect_%s.png" % color)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if cv2.imwrite(path, out):
        print("叠加图：%s" % path)
    else:
        print("⚠ 叠加图写入失败：%s" % path, file=sys.stderr)


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


def run_image(args, calib, z_grasp, z_table):
    img = cv2.imread(args.image)
    if img is None:
        sys.exit("读不到图片：%s" % args.image)
    found, report, pads = detect(img, args.color, calib, z_grasp, z_table,
                                 args.max_area, args.no_tall, args.pads)
    for name in COLORS:
        hit = found.get(name)
        if hit:
            print("%-5s uv=(%.0f,%.0f) area=%.0f Vmean=%.0f base=(%.0f,%.0f)"
                  % (name, hit["uv"][0], hit["uv"][1], hit["area"],
                     hit["v_mean"], hit["base"][0], hit["base"][1]))
        else:
            print("%-5s 未找到" % name)
            for line in report.get(name, [])[-3:]:
                print("   ", line)
    if pads:
        for name in cd.PAD_NAMES:
            p = pads.get(name)
            if p:
                print("%s纸 uv=(%.0f,%.0f) %.0fmm² base=(%.0f,%.0f)"
                      % (name, p["uv"][0], p["uv"][1], p["area_mm2"],
                         p["base"][0], p["base"][1]))
            else:
                print("%s纸 未找到" % name)
    out, _ = draw(img, found, args.color, pads)
    save_overlay(out, args.color, args.out)
    return 0


def run_live(args, calib, z_grasp, z_table):
    cap = open_camera()
    win = "color_detect"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 540)
    color = args.color
    last = 0.0
    print("实时检测（color_detect）：%s；q/Esc 退出，s 保存，c 换色" % color)
    while True:
        ok, img = cap.read()
        if not ok or img is None:
            continue
        found, report, pads = detect(img, color, calib, z_grasp, z_table,
                                     args.max_area, args.no_tall, args.pads)
        out, status = draw(img, found, color, pads)
        cv2.imshow(win, out)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            save_overlay(out, color, args.out)
        elif key == ord("c"):
            color = COLORS[(COLORS.index(color) + 1) % len(COLORS)]
            print("颜色 → %s" % color)
        now = time.time()
        if now - last > 1.0:
            print("  " + status)
            if found.get(color) is None:
                for line in report.get(color, [])[-2:]:
                    print("     · " + line)
            last = now
    cap.release()
    cv2.destroyAllWindows()
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="摄像头/图片颜色检测（与 pick.py --auto 同一套判据）")
    ap.add_argument("--image", metavar="PNG", help="读图片；不给则实时摄像头")
    ap.add_argument("--color", default="green", choices=COLORS)
    ap.add_argument("--max-area", type=float, default=params.MAX_AREA_PX,
                    help="最大候选面积 px（默认 %.0f；0=不限制）"
                         % params.MAX_AREA_PX)
    ap.add_argument("--no-tall", action="store_true",
                    help="关闭“立着”/宽高比形态判据")
    ap.add_argument("--pads", action="store_true",
                    help="同时检测红/绿/蓝投放色纸（托盘）")
    ap.add_argument("--out", default=None,
                    help="叠加图路径；默认 data/vision/preview/detect_<color>.png")
    args = ap.parse_args()

    calib = cd.load_calib()
    z_grasp, z_table = cd.load_z_planes()
    print("标定：%s" % os.path.join(paths.DATA_VISION, "T_base_cam.npz"))
    print("阈值 material=%s" % params.COLOR_RANGES.get(args.color))
    print("判据：V均值≥%s；面积 %.0f~%.0f；宽高比 %s；min_shape %.2f"
          % (cd.V_MEAN_MIN.get(args.color), cd.MIN_AREA, args.max_area,
             cd.ASPECT_RANGE, cd.MIN_SHAPE))

    if args.image:
        return run_image(args, calib, z_grasp, z_table)
    return run_live(args, calib, z_grasp, z_table)


if __name__ == "__main__":
    sys.exit(main())
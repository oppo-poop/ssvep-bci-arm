#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机采集（单帧）。

移植自队友的 `vision/scripts/grasp_xy.py::grab_frame`，主要改动是**设备发现**：
他们按名字匹配 `astra`，我们可能用 RealSense，所以改成
「环境变量优先 → 按名字找常见深度相机 → 逐个试」。

参数沿用他们踩出来的：
    - 必须 MJPG：YUYV 下 720p 只有 ~9 fps
    - BUFFERSIZE=4：设 1 会让帧率从 30 掉到 16
    - 先读 12 帧清空缓冲：V4L2 的队列里有旧帧，不丢会拿到几秒前的画面
"""

import os
import sys

import numpy as np

_SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from roarm_lib import params, paths  # noqa: E402

#: 常见深度相机的设备名关键字（顺序即优先级）
NAME_HINTS = ("realsense", "astra", "orbbec", "camera")


def list_video_devices():
    """返回 [(index, name), ...]，按编号排序。"""
    import glob

    out = []
    for d in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            with open(d + "/name") as f:
                name = f.read().strip()
        except OSError:
            name = ""
        try:
            idx = int(d.rsplit("video", 1)[1])
        except ValueError:
            continue
        out.append((idx, name))
    return out


def find_camera_index():
    """找彩色相机设备号。

    优先顺序：
        1. 环境变量 CAM_INDEX（显式指定，最可靠）
        2. 设备名里含 realsense / astra / orbbec
        3. 编号最小的 /dev/video*
    """
    env = os.environ.get("CAM_INDEX")
    if env is not None:
        return int(env)

    devs = list_video_devices()
    if not devs:
        return None

    for hint in NAME_HINTS:
        for idx, name in devs:
            if hint in name.lower():
                return idx

    return devs[0][0]


def grab_frame(w=1280, h=720, index=None, warmup=12, tries=2):
    """抓一帧彩色图（BGR）。失败时 sys.exit。

    Args:
        w, h:     分辨率。**必须与内参标定时一致**（本项目的约定是 1280x720）
        index:    设备号；None 表示自动找
        warmup:   先丢弃多少帧清空 V4L2 队列
    """
    import cv2

    idx = find_camera_index() if index is None else int(index)
    if idx is None:
        sys.exit("没找到任何 /dev/video* 设备；插上相机或用 CAM_INDEX=2 指定")

    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    for _ in range(warmup):
        cap.read()

    img = None
    for _ in range(tries):
        ok, frame = cap.read()
        if ok and frame is not None:
            img = frame
    cap.release()

    if img is None:
        sys.exit("读不到画面：/dev/video%d（用 CAM_INDEX 换一个试试）" % idx)

    if img.shape[1] != w or img.shape[0] != h:
        print("⚠ 实际分辨率 %dx%d 与请求 %dx%d 不一致 —— 内参是按标定时的分辨率算的，"
              "不一致会让反投影系统性偏移" % (img.shape[1], img.shape[0], w, h),
              file=sys.stderr)

    return img


def load_scene_z(scene_yaml=None, default=None):
    """从场景 yaml 读 pick 的 z（抓取平面高度，米）。

    不传参时读 data/scene_real.yaml，默认值取 config/vision.json。
    """
    if scene_yaml is None:
        scene_yaml = str(paths.SCENE_REAL)
    if default is None:
        default = params.SCENE_Z_DEFAULT
    try:
        for line in open(scene_yaml, encoding="utf-8"):
            s = line.strip()
            if s.startswith("pick:"):
                parts = s.split("[")[1].split("]")[0].split(",")
                return float(parts[2])
    except Exception:                                    # noqa: BLE001
        pass
    return default


def _selftest():
    """只列设备，不打开相机（避免在本机无相机时报错）。"""
    devs = list_video_devices()
    if not devs:
        print("  没有 /dev/video* 设备（正常，本机可能没接相机）")
        return 0
    print("  %-6s %s" % ("索引", "设备名"))
    for idx, name in devs:
        print("  %-6d %s" % (idx, name))
    print("  自动选择：/dev/video%s" % find_camera_index())
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())

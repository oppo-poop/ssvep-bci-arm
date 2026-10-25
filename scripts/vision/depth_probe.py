#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读一帧深度图，给出指定像素邻域的中值距离（米）。

用途：替代 `pick.py` 里手填的 `z_g`（抓取平面高度）。
单目平面假设必须知道"物料在哪个高度"，换一个花瓶就要重填；
深度相机直接测，这个参数就不需要了。

为什么单独一个脚本
    主流程 `pick.py` 刻意不依赖 rclpy（为了跨 ROS 版本、跨机器都能跑）。
    读深度必须用 rclpy，所以拆成子进程，主流程通过 stdout 取一个数。

用法
    # 打印深度图的基本信息（看话题在不在、什么编码）
    python3 scripts/vision/depth_probe.py --info

    # 读某个像素附近的深度（输出一行浮点数，单位米）
    python3 scripts/vision/depth_probe.py --u 617 --v 662

    # 指定话题
    python3 scripts/vision/depth_probe.py --u 617 --v 662 \
        --topic /camera/camera/aligned_depth_to_color/image_raw

退出码
    0 成功（stdout 只有一行数字，便于被别的程序读）
    1 拿不到数据 / 邻域全是无效值
"""

import argparse
import sys

import numpy as np


def _to_depth_mm(msg):
    """把 sensor_msgs/Image 转成 uint16 深度矩阵（毫米）。"""
    h, w = msg.height, msg.width

    if msg.encoding in ("16UC1", "mono16"):
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        step = msg.step // 2                    # step 是字节数
        arr = arr.reshape(h, step)[:, :w]
        return arr

    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32)
        step = msg.step // 4
        arr = arr.reshape(h, step)[:, :w]
        # 32FC1 一般是米；转成"毫米"保持和 16UC1 一致
        return np.nan_to_num(arr * 1000.0, nan=0.0).astype(np.uint16)

    raise ValueError("不支持的深度编码：%s（需要 16UC1 或 32FC1）" % msg.encoding)


def main():
    ap = argparse.ArgumentParser(description="读一帧深度图的中值距离")
    ap.add_argument("--topic", default="/camera/camera/depth/image_rect_raw",
                    help="深度图话题（默认按队友 my_distance 用的那个）")
    ap.add_argument("--u", type=int, help="像素 u")
    ap.add_argument("--v", type=int, help="像素 v")
    ap.add_argument("--radius", type=int, default=5, help="取样半径（像素）")
    ap.add_argument("--timeout", type=float, default=5.0, help="等一帧的超时（秒）")
    ap.add_argument("--info", action="store_true", help="只打印图像信息")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError:
        print("❌ 没有 rclpy —— 需要先 source ROS 环境：", file=sys.stderr)
        print("   source /opt/ros/$ROS_DISTRO/setup.bash", file=sys.stderr)
        print("   source star-arm-moveit2/install/setup.bash", file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("depth_probe")

    state = {"msg": None}

    def cb(msg):
        if state["msg"] is None:                # 只要第一帧
            state["msg"] = msg

    node.create_subscription(Image, args.topic, cb, 10)

    import time
    t0 = time.time()
    while state["msg"] is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = state["msg"]
    if msg is None:
        print("❌ %.1f 秒内没收到 %s 的帧" % (args.timeout, args.topic), file=sys.stderr)
        print("   ros2 topic list | grep depth  看看话题名对不对", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.info:
        print("话题     %s" % args.topic)
        print("分辨率   %dx%d" % (msg.width, msg.height))
        print("编码     %s" % msg.encoding)
        try:
            d = _to_depth_mm(msg)
            valid = d[d > 0]
            if valid.size:
                print("有效像素 %.1f%%   范围 %.0f~%.0f mm   中值 %.0f mm"
                      % (100.0 * valid.size / d.size, valid.min(), valid.max(),
                         np.median(valid)))
            else:
                print("有效像素 0%（整帧都是 0）")
        except ValueError as e:
            print("解析失败：%s" % e)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if args.u is None or args.v is None:
        print("需要 --u 和 --v（或 --info）", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        d = _to_depth_mm(msg)
    except ValueError as e:
        print("❌ %s" % e, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    r = max(1, args.radius)
    v0, v1 = max(0, args.v - r), min(d.shape[0], args.v + r + 1)
    u0, u1 = max(0, args.u - r), min(d.shape[1], args.u + r + 1)
    patch = d[v0:v1, u0:u1]
    valid = patch[patch > 0]

    if valid.size == 0:
        print("❌ 像素 (%d,%d) 周围 %.0f%% 是无效深度（反光/透明/太远）"
              % (args.u, args.v, 100.0 * (patch == 0).size / patch.size),
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    z_m = float(np.median(valid)) / 1000.0
    # stdout 只输出一个数，方便被 pick.py 直接读
    print("%.4f" % z_m)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# coding: utf-8
"""用 URDF 做前向运动学：给关节角，算各 link 在世界系的位置。

用途
    · 把舵机读数翻译成"臂实际在什么姿态"（标定时判断摆得对不对）
    · 粗判某个关节组合是否离谱（不需要起仿真、不需要 ROS）

用法
    python3 scripts/fk.py                                  # 零位
    python3 scripts/fk.py --q 0 -90 90 0 0 0               # 指定 6 个关节角（度）
    python3 scripts/fk.py --q 0.4 -90.6 89.3 0.1 -2.2 -3.2 # 直接把舵机读数丢进来
    python3 scripts/fk.py --q ... --json                   # 机器可读

注意
    这是纯运动学，不含碰撞、不含限位检查、不含 IK。
    关节角单位是**度**，和零位标定脚本读出来的舵机角度同一个口径。
"""

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_URDF = (Path(__file__).resolve().parent.parent
                / "star-arm-moveit2/src/cello_description/urdf/cello_description.urdf")
BASE_LINK = "base_link"
TIP_LINK = "link6"


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def rpy_to_R(r, p, y):
    return matmul(rz(y), matmul(ry(p), rx(r)))


def axis_angle(axis, q):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def parse_urdf(path):
    """返回 {joint_name: (parent, child, xyz, rpy, axis, type)}。"""
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall("joint"):
        name = j.get("name")
        org = j.find("origin")
        xyz = [float(v) for v in (org.get("xyz", "0 0 0").split() if org is not None else "0 0 0".split())]
        rpy = [float(v) for v in (org.get("rpy", "0 0 0").split() if org is not None else "0 0 0".split())]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz").split() if ax is not None else [0, 0, 1])]
        out[name] = (j.find("parent").get("link"), j.find("child").get("link"),
                     xyz, rpy, axis, j.get("type"))
    return out


def chain_to(urdf, tip):
    """从 tip 往回找到 base，返回 [(joint_name, ...)] 的顺向链。"""
    by_child = {v[1]: (k, v) for k, v in urdf.items()}
    chain, cur = [], tip
    while cur in by_child:
        name, spec = by_child[cur]
        chain.append((name, spec))
        cur = spec[0]
    chain.reverse()
    return chain


def fk(urdf, q_deg):
    """q_deg: 按链顺序给的关节角（度）。fixed 关节不消耗角度。"""
    chain = chain_to(urdf, TIP_LINK)
    pos = [0.0, 0.0, 0.0]
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    out, i = [], 0
    for name, (parent, child, xyz, rpy, axis, jtype) in chain:
        # T_parent_child = T_origin(rpy, xyz) @ T_joint(q)
        # 平移用**父坐标系**的旋转，所以先算 pos 再更新 R
        offset = matvec(R, xyz)
        pos = [pos[k] + offset[k] for k in range(3)]
        R = matmul(R, rpy_to_R(*rpy))
        if jtype in ("revolute", "continuous"):
            q = q_deg[i] if i < len(q_deg) else 0.0
            i += 1
            R = matmul(R, axis_angle(axis, math.radians(q)))
        out.append((child, [round(v, 5) for v in pos]))
    return out


def main():
    ap = argparse.ArgumentParser(description="URDF 前向运动学（度输入，米输出）")
    ap.add_argument("--q", nargs=6, type=float, metavar="DEG",
                    help="joint1..joint6 的角度（度），默认全 0")
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--json", action="store_true", help="输出 JSON")
    args = ap.parse_args()

    if not Path(args.urdf).is_file():
        print("找不到 URDF：%s" % args.urdf, file=sys.stderr)
        return 2

    urdf = parse_urdf(args.urdf)
    q = args.q or [0.0] * 6
    pts = fk(urdf, q)
    tip = pts[-1][1]

    if args.json:
        print(json.dumps({"q_deg": q, "links": dict(pts), "tip": tip}, ensure_ascii=False))
        return 0

    print("关节角(度): " + "  ".join("j%d=%+7.2f" % (i + 1, v) for i, v in enumerate(q)))
    print()
    for link, p in pts:
        print("  %-10s (%+.4f, %+.4f, %+.4f)" % (link, *p))
    dist = math.sqrt(sum(v * v for v in tip))
    print("\n末端 %s：前方 %.1f mm，侧向 %+.1f mm，高 %.1f mm，距 base %.1f mm"
          % (TIP_LINK, tip[0] * 1000, tip[1] * 1000, tip[2] * 1000, dist * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""米尺标定：修正「视觉坐标 → 真实基座坐标」的线性误差。

为什么要这个：手眼标定的残差压不到 3mm 以内（瓶颈是机械臂 FK，见 README 坑 5），
实际表现是「远处偏左/偏右时 y 不准，x/z 很准」——这是相机绕竖直轴有个小角度，
误差与横向位移成正比。这种**比例型**误差用一个 2x2 仿射就能吃掉，
不需要重新标定，也不需要相信任何视觉自洽性。

做法（不动机械臂，只要一把米尺）：
    1. 把物料放在桌面上某个位置；
    2. 从**机械臂底座**量出它的 x（往前）、y（往右）是多少毫米；
    3. 按提示输入 → 脚本自动拍照、检测、记录；
    4. 重复 4 次，尽量覆盖 近/远 × 左/右 四角。

结果写 data/xy_correction.json，vision_pick.py 自动读取并应用：

    p_corr = p + (A p + b - p) - (A p0 + b - p0)

注意最后那一项：**参考点 p0 处修正为 0**，所以现有的 grasp_offset
不用重调（它是在 p0 附近调出来的），远端的斜率误差才被改掉。

用法:
    .venv_astra/bin/python scripts/calib_xy.py           # 交互录入
    .venv_astra/bin/python scripts/calib_xy.py --show    # 看当前修正
    .venv_astra/bin/python scripts/calib_xy.py --clear   # 删除修正
"""
import argparse
import csv
import datetime
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))   # scripts/vision/
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
DATA = os.path.join(ROOT, "data", "vision")
JSON_PATH = os.path.join(DATA, "xy_correction.json")
PROBE_CSV = os.path.join(DATA, "probe_points.csv")
IMG_PATH = os.path.join(DATA, "preview", "calib_xy_point.png")

sys.path.insert(0, HERE)
from camera import grab_frame, load_scene_z   # noqa: E402
from project import pixel_to_base as _pixel_to_base_impl   # noqa: E402


def pixel_to_base(u, v, T_base_cam, K, dist, z_plane):
    """兼容队友的调用顺序（T 在前），内部转成 project.py 的 (u,v,K,dist,T,z)。"""
    return _pixel_to_base_impl(u, v, K, dist, T_base_cam, z_plane)
from pick import blue_candidates, R_MIN, R_MAX          # noqa: E402


def load_calib():
    d = np.load(os.path.join(DATA, "T_base_cam.npz"))
    kd = np.load(os.path.join(DATA, "cam_intrinsics.npz"))
    return d["T_base_cam"], kd["K"], kd["dist"]


def capture(T, K, dist, z_g):
    """拍一张，返回 ((u,v), area, P_mm) 里最像目标的一个。"""
    for attempt in (1, 2):
        try:
            img = grab_frame()
        except SystemExit as e:
            print("  拍照失败（%s），重试…" % e)
            continue
        raw, _mask = blue_candidates(img)
        best = None
        for uv, a in raw:
            P = pixel_to_base(uv[0], uv[1], T, K, dist, z_g)
            if P is None:
                continue
            r = float(np.hypot(P[0], P[1]))
            if R_MIN <= r <= R_MAX and (best is None or a > best[1]):
                best = ((uv[0], uv[1]), a, P * 1000.0)
        if best is not None:
            vis = img.copy()
            cv2.drawMarker(vis, (int(best[0][0]), int(best[0][1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 40, 3)
            cv2.putText(vis, "calib point", (int(best[0][0]) + 16,
                                             int(best[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            try:
                cv2.imwrite(IMG_PATH, vis)
            except Exception:
                pass
            return best
        print("  画面里没有合适的蓝色目标（或者都不在 0.1~0.75m 内），重拍…")
    return None


def fit(pts):
    """pts: [((vx,vy),(tx,ty)), ...] 返回 A(2x2), b(2), rms。"""
    M, rhs = [], []
    for (vx, vy), (tx, ty) in pts:
        M.append([vx, vy, 0, 0, 1, 0])
        rhs.append(tx)
        M.append([0, 0, vx, vy, 0, 1])
        rhs.append(ty)
    M = np.array(M, dtype=np.float64)
    rhs = np.array(rhs, dtype=np.float64)
    sol, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    A = np.array([[sol[0], sol[1]], [sol[2], sol[3]]])
    b = np.array([sol[4], sol[5]])
    rms = float(np.sqrt(np.mean((M @ sol - rhs) ** 2)))
    return A, b, rms


def show_current():
    if not os.path.isfile(JSON_PATH):
        print("还没有 xy_correction.json —— 当前不做任何修正（p_corr = p_vis）")
        return 0
    d = json.load(open(JSON_PATH, encoding="utf8"))
    A = np.array(d["A"])
    b = np.array(d["b"])
    p0 = np.array(d["p0"])
    print("当前 XY 修正（在 %s）" % d.get("updated", "?"))
    print("  A =\n%s" % np.round(A, 4))
    print("  b = %s mm" % np.round(b, 1).tolist())
    print("  参考点 p0 = (%+.0f, %+.0f) mm（此处修正恒为 0）" % tuple(p0))
    print("  拟合残差 RMS %.2f mm，样本 %d 点" % (d.get("rms_mm", -1), len(d.get("points", []))))
    for q in d.get("points", []):
        pv = np.array(q["vis"])
        pt = np.array(q["true"])
        pc = pv + (A @ pv + b - pv) - (A @ p0 + b - p0)
        print("    视觉(%+6.0f,%+6.0f) 米尺(%+6.0f,%+6.0f) → 修正后(%+6.0f,%+6.0f) 余差 %+.0f,%+.0f"
              % (*pv, *pt, *pc, *(pt - pc)))
    return 0


def fit_yline(pts):
    """只拟合 y：y_true = a*y_vis + b，x 不动。

    为什么单独留这个模型：实测症状是「物料在右要 -10mm、在左要 +10mm」——
    y 的**比例**错了、x 是准的。用 2~3 个点去拟合 6 个参数的仿射会过拟合，
    点少时只解 y 的两个参数最稳。
    """
    V = np.array([[p[0][1], 1.0] for p in pts], dtype=np.float64)
    T = np.array([p[1][1] for p in pts], dtype=np.float64)
    sol, *_ = np.linalg.lstsq(V, T, rcond=None)
    a, b = float(sol[0]), float(sol[1])
    rms = float(np.sqrt(np.mean((V @ sol - T) ** 2)))
    return np.array([[1.0, 0.0], [0.0, a]]), np.array([0.0, b]), rms


def load_probe_points(path):
    """读 vision_pick --probe --record 攒的点：真值 = 视觉 + 当时用的 dxy。"""
    if not os.path.isfile(path):
        sys.exit("找不到 %s —— 先跑 vision_pick.py --probe --record 记几个点" % path)
    pts = []
    with open(path, newline="", encoding="utf8") as f:
        for r in csv.DictReader(f):
            try:
                vx, vy = float(r["obj_x"]), float(r["obj_y"])
                dx, dy = float(r["dx"]), float(r["dy"])
            except (KeyError, TypeError, ValueError):
                continue
            pts.append(((vx, vy), (vx + dx, vy + dy)))
    return pts


def save_correction(A, b, p0, rms, pts, source, keep_right_dy=True):
    # 保留"只在右侧生效"的附加补偿：重新拟合线性模型时它不该被冲掉，
    # 否则右侧那 5mm 会悄悄消失（表现为夹爪又从物料侧面撞上去）。
    # 但用 2x2 仿射（>=4 点）重标时，右侧的偏差应该已经被仿射吃掉了，
    # 这时再用 --zero-right-dy 把它清掉，否则右边会被补两遍。
    keep = {}
    if os.path.isfile(JSON_PATH):
        try:
            old = json.load(open(JSON_PATH, encoding="utf8"))
            keys = ("place_dx_mm", "place_dy_mm")          # 投放微调，跟拟合无关
            if keep_right_dy:
                keys += ("right_dy_mm", "right_full_at_mm")
            for k in keys:
                if k in old:
                    keep[k] = old[k]
        except Exception:
            pass
    out = {
        "A": [[round(float(v), 6) for v in row] for row in A],
        "b": [round(float(v), 2) for v in b],
        "p0": [round(float(v), 1) for v in p0],
        "rms_mm": round(rms, 2),
        "source": source,
        "points": [{"vis": [round(float(v), 1) for v in pv],
                    "true": [round(float(v), 1) for v in pt]} for pv, pt in pts],
        "updated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "note": "p_corr = p + (A p + b - p) - (A p0 + b - p0)；p0 处修正为 0，grasp_offset 无需重调"
                "；right_dy_mm 是只在 y<p0（画面右侧）生效的附加补偿",
    }
    out.update(keep)
    with open(JSON_PATH, "w", encoding="utf8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n已写入 %s" % JSON_PATH)
    print("vision_pick.py 下次运行会自动应用（会打印「已应用 XY 线性修正」）。")
    return 0


def run_from_probe(args):
    """用 vision_pick --probe --record 攒的点拟合（不用米尺）。"""
    pts = load_probe_points(PROBE_CSV)
    if len(pts) < 2:
        print("probe 点只有 %d 个，至少 2 个（物料放左、放右各记一次）" % len(pts))
        return 1
    print("用 %d 个 probe 点拟合：" % len(pts))
    for vis, true in pts:
        print("   视觉(%+7.1f,%+7.1f)  真值(%+7.1f,%+7.1f)  Δ(%+.1f,%+.1f)"
              % (*vis, *true, true[0] - vis[0], true[1] - vis[1]))

    model = args.model
    if model == "auto":
        model = "affine" if len(pts) >= 4 else "yline"
    if model == "yline":
        A, b, rms = fit_yline(pts)
        print("\n模型：只修 y 的比例  y_true = %.4f * y_vis %+.1f" % (A[1, 1], b[1]))
    else:
        A, b, rms = fit(pts)
        print("\n模型：2x2 仿射")
    print("   A =\n%s" % np.round(A, 4))
    print("   b = %s mm" % np.round(b, 1).tolist())
    print("   残差 RMS %.2f mm" % rms)

    p0 = np.array(args.p0 if args.p0 else
                  np.mean([p[0] for p in pts], axis=0), dtype=np.float64)

    print("\n逐点检查（余差 = 修正后与真值之差，应接近 0）：")
    for pv, pt in pts:
        pv_a = np.array(pv, dtype=np.float64)
        pt_a = np.array(pt, dtype=np.float64)
        pc = pv_a + (A @ pv_a + b - pv_a) - (A @ p0 + b - p0)
        print("  视觉(%+6.1f,%+6.1f) 真值(%+6.1f,%+6.1f) → 修正后(%+6.1f,%+6.1f) "
              "余差 %+.1f,%+.1f"
              % (*pv_a, *pt_a, *pc, *(pt_a - pc)))

    if rms > 8.0:
        print("\n⚠ 残差偏大：这几个点自相矛盾（可能某次录入时并没真的对齐）。")
        print("  删掉 data/probe_points.csv 重新记几个再试。")
        return 1
    return save_correction(A, b, p0, rms, pts, "probe",
                           keep_right_dy=not args.zero_right_dy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--from-probe", action="store_true",
                    help="用 --probe --record 攒的点拟合（不用米尺）")
    ap.add_argument("--model", choices=("auto", "yline", "affine"), default="auto",
                    help="auto：点数>=4 时用 2x2 仿射，否则只拟合 y 的比例")
    ap.add_argument("--p0", type=float, nargs=2, default=None,
                    help="参考点 x y（mm），默认取录入点的重心")
    ap.add_argument("--z-grasp", type=float, default=None)
    ap.add_argument("--zero-right-dy", action="store_true",
                    help="重标时清掉 right_dy_mm（用 2x2 仿射重标时该清，否则右边补两遍）")
    args = ap.parse_args()

    if args.show:
        return show_current()
    if args.clear:
        if os.path.isfile(JSON_PATH):
            os.remove(JSON_PATH)
            print("已删除 %s（vision_pick 回到无修正）" % JSON_PATH)
        else:
            print("本来就没有修正文件")
        return 0

    if args.from_probe:
        return run_from_probe(args)

    T, K, dist = load_calib()
    z_g = (args.z_grasp if args.z_grasp is not None else load_scene_z())

    print("米尺标定 —— 检测平面 z=%.4f m（与 vision_pick 一致）" % z_g)
    print("每个点：把物料放好 → 从机械臂底座量出它的 x（往前）、y（往右）mm → 输入")
    print("建议 4 个点：近/远 × 左/右。直接回车结束录入。\n")

    pts = []
    while True:
        s = input("第 %d 点  x_mm y_mm > " % (len(pts) + 1)).strip()
        if not s:
            break
        parts = s.replace(",", " ").split()
        if len(parts) != 2:
            print("  要两个数：x_mm y_mm")
            continue
        try:
            tx, ty = float(parts[0]), float(parts[1])
        except ValueError:
            print("  解析不了，重输")
            continue

        got = capture(T, K, dist, z_g)
        if got is None:
            print("  ❌ 这个点没拍到，跳过（重跑一次就好）")
            continue
        uv, area, P = got
        print("    像素(%.0f,%.0f) 面积%.0f  视觉(%+.0f,%+.0f) mm  "
              "米尺(%+.0f,%+.0f)  Δ(%+.0f,%+.0f)"
              % (uv[0], uv[1], area, P[0], P[1], tx, ty, tx - P[0], ty - P[1]))
        pts.append(((P[0], P[1]), (tx, ty)))

    if len(pts) < 3:
        print("\n至少要 3 个点（2x2 仿射有 6 个未知量），只有 %d 个，不写文件。"
              % len(pts))
        return 1

    A, b, rms = fit(pts)
    p0 = np.array(args.p0 if args.p0 else
                  np.mean([p[0] for p in pts], axis=0), dtype=np.float64)

    print("\n解得 A =\n%s" % np.round(A, 4))
    print("     b = %s mm" % np.round(b, 1).tolist())
    print("拟合残差 RMS %.2f mm（>8mm 说明米尺量的位置或检测点有问题）" % rms)

    print("\n逐点检查（余差=修正后与米尺的差，应接近 0）：")
    for pv, pt in pts:
        pv_a = np.array(pv)
        pt_a = np.array(pt)
        pc = pv_a + (A @ pv_a + b - pv_a) - (A @ p0 + b - p0)
        print("  视觉(%+6.0f,%+6.0f) 米尺(%+6.0f,%+6.0f) → 修正后(%+6.0f,%+6.0f) 余差 %+.0f,%+.0f"
              % (*pv_a, *pt_a, *pc, *(pt_a - pc)))

    if rms > 8.0:
        print("\n⚠ 残差偏大，先别急着用。常见原因：米尺的零点不是底座中心、"
              "或某个点检测到了别的蓝色物体。")
        print("  重跑一次，或者看 output/calib_xy_point.png 确认十字压在物料上。")
        return 1

    return save_correction(A, b, p0, rms, pts, "tape",
                           keep_right_dy=not args.zero_right_dy)


if __name__ == "__main__":
    sys.exit(main())

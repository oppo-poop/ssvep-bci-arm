#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""物料 / 投放色纸颜色检测（移植自队友 cap/color_detect.py，适配本仓库标定与路径）。

物料：`find_targets` / `find_targets_adaptive` —— 每种颜色取一个代表点。
投放色纸：`find_pads` —— 红/绿/蓝三张纸各自的中心和基座坐标。

真正区分物料和托盘/背景的判据，来自队友在真机上踩出来的组合：

    · 按**整块平均亮度** V 排除绿地毯/桌面（V_MEAN_MIN，默认 130）；
    · 面积上下限：物料约 3000~11000 px，托盘/背景通常 >16000；
    · 宽高比 + “立着”判据：物料是柱子，托盘是扁平大片；
    · 面积超过 big_area 时放宽“立着”判据（红柱倾斜时会被压扁）；
    · 同一颜色多个候选时，选**离底座最近**的，而不是面积最大的；
    · 找不到时用 find_targets_adaptive 放宽面积/饱和度再补一次。

色纸则相反：用更宽的 PAD_RANGES、按**毫米面积**（≥8000mm²）、填充率、
宽高比和边缘亮度判断，并用多档 (S,V) 兜住被灯照白的纸。

HSV 阈值仍取 config/vision.json 的 colors.<色>.material；颜色空间没有换 LAB，
因为托盘和物料的 a/b 几乎一样，真正分开它们的是 V 和形状/面积。

队友原文件：/home/ljl/桌面/brain_extract/cap/color_detect.py（2026-11-01 版）。
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from roarm_lib import params, paths  # noqa: E402
from project import pixel_to_base  # noqa: E402

NAMES = tuple(n for n in ("red", "green", "blue", "yellow")
              if n in params.COLOR_RANGES)
NAMES_CN = {"red": "红", "green": "绿", "blue": "蓝", "yellow": "黄"}

_DETECT = params.VISION.get("detect", {})
V_MEAN_MIN = {n: float(_DETECT.get("v_mean_min", {}).get(n, 130.0))
              for n in NAMES}
MIN_AREA = float(_DETECT.get("min_area_px", 400.0))
MAX_AREA = float(_DETECT.get("max_area_px", 16000.0))
ASPECT_RANGE = tuple(_DETECT.get("aspect_range", (0.30, 2.60)))
MIN_SHAPE = float(_DETECT.get("min_shape", 1.05))
BIG_AREA = float(_DETECT.get("big_area_px", 1000.0))

#: 基座 xy 可达范围 (x0,x1,y0,y1) mm。队友 color_detect.REACH_MM 是
#: (250, 670, -430, 380)，比 pick 的任务工作区宽：检测先看见画面边缘的物料，
#: 能不能抓由 pick 的 BOUNDS 再判。不要在识别阶段提前误杀。
if params.DETECT_REACH_MM:
    REACH_MM = params.DETECT_REACH_MM
else:
    REACH_MM = (params.REACH_BOUNDS["x"][0] * 1000.0,
                params.REACH_BOUNDS["x"][1] * 1000.0,
                params.REACH_BOUNDS["y"][0] * 1000.0,
                params.REACH_BOUNDS["y"][1] * 1000.0)


def _ranges():
    """config 的 [(lo,hi), ...] → 队友 build_masks 要的 ([lo...],[hi...])。"""
    out = {}
    for name, rs in params.COLOR_RANGES.items():
        out[name] = ([tuple(lo) for lo, _hi in rs],
                     [tuple(hi) for _lo, hi in rs])
    return out


RANGES = _ranges()

#: 色纸/托盘用**另一套更宽**的 HSV 区间（队友原 PAD_RANGES）。
#: 物料是固定柱子，可以卡窄；纸是随手买的卡纸，色相跨度大、还会被灯照白。
#: 红纸和红物料按色相分开：物料红到 164，纸从 165 开始，互不冒充。
PAD_NAMES = ("red", "green", "blue")
PAD_RANGES = {
    "red":   ([(165, 90, 50)], [(180, 255, 255)]),
    "green": ([(45, 90, 80)], [(97, 255, 255)]),
    "blue":  ([(98, 90, 50)], [(150, 255, 255)]),
}
#: 多档“降低 S、抬高 V”的组合，用来兜住被照白的纸。先到先得，不覆盖已找到的颜色。
PAD_PASSES = ((None, None), (75, 130), (40, 170))


def load_calib():
    """读本仓库的相机外参/内参。"""
    T = np.load(os.path.join(paths.DATA_VISION, "T_base_cam.npz"))["T_base_cam"]
    kd = np.load(os.path.join(paths.DATA_VISION, "cam_intrinsics.npz"))
    return {"T": T, "K": kd["K"], "dist": kd["dist"]}


def load_z_planes():
    """返回 (z_grasp, z_table)，单位米；和 pick.py 共用 data/scene_real.yaml。"""
    z_pick = params.SCENE_Z_DEFAULT
    obj_h = params.OBJECT_HEIGHT_DEFAULT_M
    try:
        for line in open(str(paths.SCENE_REAL), encoding="utf-8"):
            s = line.strip()
            if s.startswith("pick:"):
                z_pick = float(s.split("[")[1].split("]")[0].split(",")[2])
            elif s.startswith("object_height:"):
                obj_h = float(s.split(":", 1)[1].split("#")[0].strip())
    except Exception:                                   # noqa: BLE001
        pass
    return z_pick, z_pick - obj_h / 2.0


def to_base(calib, uv, z_plane):
    """像素 → 基座 xy/z（mm）。射线和平面没交点返回 None。"""
    P = pixel_to_base(uv[0], uv[1], calib["K"], calib["dist"],
                      calib["T"], z_plane)
    if P is None:
        return None
    return np.array([P[0] * 1000.0, P[1] * 1000.0, P[2] * 1000.0],
                    dtype=np.float64)


def build_masks(img, ranges=None, s_min=None, v_min=None, area_min=250.0,
                s_floor=None):
    """返回 {颜色: mask}。mask 已做开闭运算并去掉 area_min 以下的小块。"""
    import cv2
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    out = {}
    use_ranges = ranges or RANGES
    for name in use_ranges:
        lo_list, hi_list = use_ranges[name]
        m = None
        for lo, hi in zip(lo_list, hi_list):
            lo = list(lo)
            hi = list(hi)
            if s_floor is not None:
                lo[1] = int(s_floor)
            if s_min is not None:
                lo[1] = max(lo[1], int(s_min))
            if v_min is not None:
                lo[2] = max(lo[2], int(v_min))
            m1 = cv2.inRange(hsv, np.array(lo, np.uint8),
                             np.array(hi, np.uint8))
            m = m1 if m is None else cv2.bitwise_or(m, m1)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        if area_min > 0:
            n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
            keep = np.zeros_like(m)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] >= area_min:
                    keep[lab == i] = 255
            m = keep
        out[name] = m
    return out


def blobs(mask, min_area=250.0):
    """mask → [{'uv','cen','area','box'}]，按面积从大到小。

    uv 用**外接矩形中心**，不是面积质心：物料是上窄下宽的柱子，
    质心会被底座往下拽几十像素。
    """
    import cv2
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        a = float(cv2.contourArea(c))
        if a < min_area:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        out.append({"uv": (bx + bw / 2.0, by + bh / 2.0),
                    "cen": (m["m10"] / m["m00"], m["m01"] / m["m00"]),
                    "area": a, "box": (bx, by, bw, bh)})
    out.sort(key=lambda d: -d["area"])
    return out


def find_targets(img, calib, z_grasp, exclude_uv=(), exclude_r=120.0,
                 min_area=None, max_area=None, s_min=None, v_min=None,
                 aspect_range=None, min_shape=None, reach_mm=None,
                 big_area=None, v_min_mean=None):
    """每种颜色找一个物料，返回 (found, report)。

    found = {颜色: {'uv','cen','box','base','area','v_mean'} 或 None}
    report = {颜色: [被拒候选说明]}，用于调阈值。
    """
    import cv2
    min_area = MIN_AREA if min_area is None else float(min_area)
    max_area = MAX_AREA if max_area is None else float(max_area)
    aspect_range = ASPECT_RANGE if aspect_range is None else tuple(aspect_range)
    min_shape = MIN_SHAPE if min_shape is None else float(min_shape)
    big_area = BIG_AREA if big_area is None else float(big_area)
    reach_mm = REACH_MM if reach_mm is None else reach_mm
    if v_min_mean is None:
        v_min_mean = V_MEAN_MIN

    xmin, xmax = aspect_range
    masks = build_masks(img, s_min=s_min, v_min=v_min, area_min=min_area * 0.5)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    found, report = {}, {}
    for name in NAMES:
        kept, drop = [], []
        for c in blobs(masks[name], min_area=min_area):
            u, v = c["uv"]
            bx, by, bw, bh = c["box"]
            ar = bw / float(max(bh, 1))
            x0b, y0b = max(0, bx), max(0, by)
            x1b = min(img.shape[1], bx + bw)
            y1b = min(img.shape[0], by + bh)
            sub_v = hsv_img[y0b:y1b, x0b:x1b, 2]
            sub_m = masks[name][y0b:y1b, x0b:x1b] > 0
            v_mean = float(sub_v[sub_m].mean()) if sub_m.any() else 0.0

            if v_mean < v_min_mean.get(name, 0.0):
                drop.append("(%4.0f,%4.0f) 面积%5.0f 平均亮度%.0f < %.0f，"
                            "太暗（背景/桌面/地胶）"
                            % (u, v, c["area"], v_mean,
                               v_min_mean.get(name, 0.0)))
                continue
            if not (xmin <= ar <= xmax):
                drop.append("(%4.0f,%4.0f) 面积%5.0f 宽高比%.2f 太扁"
                            % (u, v, c["area"], ar))
                continue
            if bh / float(max(bw, 1)) < min_shape and c["area"] < big_area:
                drop.append("(%4.0f,%4.0f) 面积%5.0f 高宽比%.2f 不够立着"
                            % (u, v, c["area"], bh / float(max(bw, 1))))
                continue
            if max_area and c["area"] > max_area:
                drop.append("(%4.0f,%4.0f) 面积%5.0f > %.0f，太大"
                            % (u, v, c["area"], max_area))
                continue
            if any(np.hypot(u - eu, v - ev) < exclude_r
                   for eu, ev in exclude_uv):
                drop.append("(%4.0f,%4.0f) 面积%5.0f 压在排除点附近"
                            % (u, v, c["area"]))
                continue
            P = to_base(calib, c["uv"], z_grasp)
            if P is None:
                drop.append("(%4.0f,%4.0f) 面积%5.0f 射线和平面无交点"
                            % (u, v, c["area"]))
                continue
            if reach_mm is not None:
                x0, x1, y0, y1 = reach_mm
                if not (x0 <= P[0] <= x1 and y0 <= P[1] <= y1):
                    drop.append("(%4.0f,%4.0f) 面积%5.0f 基座(%+.0f,%+.0f) "
                                "在工作区外" % (u, v, c["area"], P[0], P[1]))
                    continue
            kept.append((c, P, v_mean))
        report[name] = drop
        if not kept:
            found[name] = None
            continue
        # 同一颜色多个候选：选离底座最近的，而不是面积最大的。
        c, P, v_mean = min(
            kept, key=lambda t: float(np.hypot(t[1][0], t[1][1])))
        found[name] = {"uv": c["uv"], "cen": c["cen"], "box": c["box"],
                       "base": P, "area": c["area"], "v_mean": v_mean}
    return found, report


def find_targets_adaptive(img, calib, z_grasp, **kw):
    """先按正常判据找；只对没找到的颜色放宽面积/饱和度再补一次。"""
    found, report = find_targets(img, calib, z_grasp, **kw)
    missing = [n for n in NAMES if found.get(n) is None]
    if not missing:
        return found, report
    loose = dict(kw)
    loose["min_area"] = min(float(kw.get("min_area") or MIN_AREA), 120.0)
    _s = kw.get("s_min")
    loose["s_min"] = 70 if _s is None else min(float(_s), 70.0)
    _v = kw.get("v_min")
    loose["v_min"] = 40 if _v is None else min(float(_v), 40.0)
    f2, r2 = find_targets(img, calib, z_grasp, **loose)
    for n in missing:
        hit = f2.get(n)
        if not hit:
            continue
        found[n] = hit
        report[n] = (["（兜底档）正常判据没找到，放宽面积/饱和度后找到："
                      "面积 %.0fpx²" % hit["area"]]
                     + list(r2.get(n, [])))
    return found, report


def mm_per_px_xy(calib, z_plane, uv):
    """(u,v) 附近横向/纵向 1 像素各多少 mm（必须按点算，不能当全图常数）。"""
    u, v = float(uv[0]), float(uv[1])
    p0 = to_base(calib, (u, v), z_plane)
    p1 = to_base(calib, (u + 10.0, v), z_plane)
    p2 = to_base(calib, (u, v + 10.0), z_plane)
    if p0 is None or p1 is None or p2 is None:
        return 1.0, 1.0
    return (float(np.linalg.norm(p1[:2] - p0[:2])) / 10.0,
            float(np.linalg.norm(p2[:2] - p0[:2])) / 10.0)


def area_mm2(calib, z_plane, uv, area_px):
    mu, mv = mm_per_px_xy(calib, z_plane, uv)
    return float(area_px) * mu * mv


_PAD_V_MEAN = {"red": 90.0, "green": 90.0, "blue": 90.0}


def _pads_once(img, calib, z_table, min_area_mm2=8000.0,
               max_area_mm2=40000.0, edge_margin=4, edge_v_min=170.0,
               fill_min=0.40, aspect_range=(0.40, 2.50), cluster_mm=60.0,
               s_min=None, s_floor=None, v_min=None, reach_mm=None,
               v_min_mean=None, log=None, ranges=None):
    """单档找色纸；对外请用 find_pads（会自动多档重试）。"""
    import cv2
    if v_min_mean is None:
        v_min_mean = _PAD_V_MEAN
    if reach_mm is None:
        reach_mm = REACH_MM
    pad_ranges = ranges or PAD_RANGES
    masks = build_masks(img, s_min=s_min, v_min=v_min, area_min=200.0,
                        s_floor=s_floor, ranges=pad_ranges)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    out = {}
    for name in PAD_NAMES:
        keep = []
        for c in blobs(masks[name], min_area=200.0):
            bx, by, bw, bh = c["box"]
            u, v = c["uv"]
            ar = bw / float(max(bh, 1))
            a_mm2 = area_mm2(calib, z_table, c["uv"], c["area"])
            x0b, y0b = max(0, bx), max(0, by)
            x1b = min(img.shape[1], bx + bw)
            y1b = min(img.shape[0], by + bh)
            sub_v = hsv_img[y0b:y1b, x0b:x1b, 2]
            sub_m = masks[name][y0b:y1b, x0b:x1b] > 0
            v_mean = float(sub_v[sub_m].mean()) if sub_m.any() else 0.0
            tag = "(%4.0f,%4.0f) %dx%d %.0fmm²" % (u, v, bw, bh, a_mm2)

            H_, W_ = img.shape[:2]
            m_ = int(edge_margin)
            edge_hit = (bx <= m_ or by <= m_ or bx + bw >= W_ - m_
                        or by + bh >= H_ - m_)
            if edge_hit:
                if v_mean < edge_v_min:
                    if log:
                        log("  ✗ %s纸 %s 贴边且偏暗（V%.0f）→ 台面/墙"
                            % (NAMES_CN.get(name, name), tag, v_mean))
                    continue
                if ar > 4.0 or ar < 0.25:
                    if log:
                        log("  ✗ %s纸 %s 贴边且极细长（宽高比 %.2f）"
                            % (NAMES_CN.get(name, name), tag, ar))
                    continue
            if not (aspect_range[0] <= ar <= aspect_range[1]):
                if log:
                    log("  ✗ %s纸 %s 宽高比 %.2f 不像纸"
                        % (NAMES_CN.get(name, name), tag, ar))
                continue
            fill = float(c["area"]) / float(max(1, bw * bh))
            if fill < fill_min:
                if log:
                    log("  ✗ %s纸 %s 填充率 %.2f 太散"
                        % (NAMES_CN.get(name, name), tag, fill))
                continue
            P = to_base(calib, c["uv"], z_table)
            if P is None:
                continue
            if reach_mm is not None:
                x0, x1, y0, y1 = reach_mm
                if not (x0 <= P[0] <= x1 and y0 <= P[1] <= y1):
                    if log:
                        log("  ✗ %s纸 %s 基座(%+.0f,%+.0f) 超出可达范围"
                            % (NAMES_CN.get(name, name), tag, P[0], P[1]))
                    continue
            if a_mm2 < min_area_mm2:
                if log:
                    log("  ✗ %s纸 %s 面积偏小（<%.0f），可能是物料"
                        % (NAMES_CN.get(name, name), tag, min_area_mm2))
                continue
            if a_mm2 > max_area_mm2:
                if log:
                    log("  ✗ %s纸 %s 面积过大（>%.0f），是台面/箱子"
                        % (NAMES_CN.get(name, name), tag, max_area_mm2))
                continue
            if v_mean < v_min_mean.get(name, 0.0):
                if log:
                    log("  ✗ %s纸 %s 平均亮度 %.0f < %.0f"
                        % (NAMES_CN.get(name, name), tag, v_mean,
                           v_min_mean.get(name, 0.0)))
                continue
            if log:
                log("  ✓ %s纸 %s 填充率 %.2f" % (NAMES_CN.get(name, name),
                                                 tag, fill))
            keep.append({"uv": np.array(c["uv"], float), "base": P,
                         "area_px": c["area"], "area_mm2": a_mm2,
                         "box": c["box"], "v_mean": v_mean})
        if not keep:
            out[name] = None
            continue
        # 同一张纸被切断时按基座 xy 聚类，中心按新像素重算
        groups = []
        for it in sorted(keep, key=lambda t: -t["area_mm2"]):
            for g in groups:
                gb = np.mean(np.array(g["bases"]), axis=0)
                if np.linalg.norm(it["base"][:2] - gb[:2]) < cluster_mm:
                    g["uv"].append(it["uv"])
                    g["bases"].append(it["base"])
                    g["boxes"].append(it["box"])
                    g["area_px"] += it["area_px"]
                    g["area_mm2"] += it["area_mm2"]
                    break
            else:
                groups.append({"uv": [it["uv"]], "bases": [it["base"]],
                               "boxes": [it["box"]],
                               "area_px": it["area_px"],
                               "area_mm2": it["area_mm2"],
                               "v_mean": it["v_mean"]})
        g = max(groups, key=lambda t: t["area_mm2"])
        uv_c = tuple(np.mean(g["uv"], axis=0))
        P_c = to_base(calib, uv_c, z_table)
        if P_c is None:
            P_c = np.mean(np.array(g["bases"]), axis=0)
        bx0 = min(int(b[0]) for b in g["boxes"])
        by0 = min(int(b[1]) for b in g["boxes"])
        bx1 = max(int(b[0] + b[2]) for b in g["boxes"])
        by1 = max(int(b[1] + b[3]) for b in g["boxes"])
        out[name] = {"uv": uv_c, "base": P_c, "area_px": g["area_px"],
                     "area_mm2": g["area_mm2"],
                     "box": (bx0, by0, bx1 - bx0, by1 - by0),
                     "n_parts": len(g["uv"]), "v_mean": g["v_mean"]}
    return out


def find_pads(img, calib, z_table, min_area_mm2=8000.0,
              max_area_mm2=40000.0, edge_margin=4, edge_v_min=170.0,
              fill_min=0.40, aspect_range=(0.40, 2.50), cluster_mm=60.0,
              s_min=None, s_floor=None, v_min=None, reach_mm=None,
              v_min_mean=None, log=None, retry=True):
    """找红/绿/蓝三张投放色纸；返回 {颜色: pad 或 None}。

    多档 S/V 先到先得：纸被灯照白时，固定的 S 下限不可能同时盖住三张纸；
    先用严格档，缺的颜色再用宽松档补，后档不覆盖前档已找到的。
    """
    if s_floor is not None or v_min is not None or not retry:
        return _pads_once(img, calib, z_table,
                          min_area_mm2=min_area_mm2, max_area_mm2=max_area_mm2,
                          edge_margin=edge_margin, edge_v_min=edge_v_min,
                          fill_min=fill_min, aspect_range=aspect_range,
                          cluster_mm=cluster_mm, s_min=s_min, s_floor=s_floor,
                          v_min=v_min,
                          reach_mm=REACH_MM if reach_mm is None else reach_mm,
                          v_min_mean=v_min_mean, log=log)
    merged = {}
    for i, (fl, vl) in enumerate(PAD_PASSES):
        got = _pads_once(
            img, calib, z_table, min_area_mm2=min_area_mm2,
            max_area_mm2=max_area_mm2, edge_margin=edge_margin,
            edge_v_min=edge_v_min, fill_min=fill_min,
            aspect_range=aspect_range, cluster_mm=cluster_mm,
            s_min=s_min, s_floor=fl, v_min=vl, reach_mm=reach_mm,
            v_min_mean=v_min_mean, log=log if i == 0 else None)
        for k in PAD_NAMES:
            if merged.get(k) is None and got.get(k):
                merged[k] = got[k]
        if sum(1 for k in PAD_NAMES if merged.get(k)) >= len(PAD_NAMES):
            break
    return {k: merged.get(k) for k in PAD_NAMES}


def _selftest(image_path, show_pads=False):
    import cv2
    img = cv2.imread(image_path)
    if img is None:
        raise SystemExit("读不到 %s" % image_path)
    calib = load_calib()
    z_grasp, z_table = load_z_planes()
    found, report = find_targets_adaptive(img, calib, z_grasp)
    for name in NAMES:
        hit = found.get(name)
        if hit:
            print("%s: uv=(%.0f,%.0f) area=%.0f v_mean=%.0f base=(%.0f,%.0f)"
                  % (name, hit["uv"][0], hit["uv"][1], hit["area"],
                     hit["v_mean"], hit["base"][0], hit["base"][1]))
        else:
            print("%s: 未找到" % name)
            for line in report.get(name, [])[-3:]:
                print("   ", line)
    if show_pads:
        print("\n投放色纸：")
        pads = find_pads(img, calib, z_table)
        for name in PAD_NAMES:
            p = pads.get(name)
            if p:
                print("%s纸: uv=(%.0f,%.0f) %.0fmm² base=(%.0f,%.0f)"
                      % (name, p["uv"][0], p["uv"][1], p["area_mm2"],
                         p["base"][0], p["base"][1]))
            else:
                print("%s纸: 未找到" % name)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="color_detect 自测")
    ap.add_argument("image")
    ap.add_argument("--pads", action="store_true", help="同时打印投放色纸")
    a = ap.parse_args()
    _selftest(a.image, a.pads)
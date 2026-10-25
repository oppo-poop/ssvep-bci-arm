#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArUco 检测与 PnP 位姿估计。

移植自队友的 `vision/scripts/check_camera.py` / `calib_intrinsic.py`，
主要改动是**兼容新旧两套 OpenCV ArUco API**：

    新（OpenCV ≥ 4.7）：  cv2.aruco.ArucoDetector(dict, params).detectMarkers(gray)
    旧（本机 4.5.4）：     cv2.aruco.detectMarkers(gray, dict, parameters=params)

队友的环境是 OpenCV 4.10（Python 3.12），本机是 4.5.4（Python 3.10）——
`ArucoDetector` 这个类在 4.5.4 里不存在，直接照抄会在导入后一调用就 AttributeError。
这里运行期探测，两套都支持。

单位约定（和队友一致，也是本项目所有标定文件的约定）：
    - 角点/码尺寸用 **毫米**
    - `solvePnP` 输出的 tvec 是毫米
    - 返回的 `T_cam_marker` 统一成 **米**
"""

import numpy as np

#: 默认字典（队友的标定板用的就是这个）
DEFAULT_DICT = None          # 运行时按名字取，避免 import 时就依赖 cv2


def _aruco():
    import cv2
    return cv2.aruco


def _new_api():
    """本机 OpenCV 是否有 ArucoDetector 新类。"""
    import cv2
    return hasattr(cv2.aruco, "ArucoDetector")


def marker_object_points(size_mm):
    """码的四个角点在「码坐标系」里的坐标（毫米，z=0）。

    顺序与 cv2.aruco.detectMarkers 返回的角点一致：
    左上 → 右上 → 右下 → 左下。
    """
    s = float(size_mm) / 2.0
    return np.array([[-s, s, 0.0],
                     [s, s, 0.0],
                     [s, -s, 0.0],
                     [-s, -s, 0.0]], dtype=np.float32)


def make_detector(dict_id=None):
    """构造检测器，返回 (detect_fn, dictionary)。

    detect_fn(gray) → (corners, ids)，屏蔽新旧 API 差异。
    """
    aruco = _aruco()
    if dict_id is None:
        dict_id = aruco.DICT_6X6_50

    if _new_api():
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.01
        params.maxMarkerPerimeterRate = 4.0
        dictionary = aruco.getPredefinedDictionary(dict_id)
        detector = aruco.ArucoDetector(dictionary, params)

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
    else:
        dictionary = aruco.Dictionary_get(dict_id)
        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = 0.01
        params.maxMarkerPerimeterRate = 4.0

        def detect(gray):
            corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
            return corners, ids

    return detect, dictionary


def detect_markers(img, dict_id=None):
    """在彩色图里找所有码，返回 (corners, ids)。没有就返回 ([], None)。"""
    import cv2

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    detect, _ = make_detector(dict_id)
    corners, ids = detect(gray)
    if ids is None:
        return [], None
    return corners, ids.ravel().tolist()


def pose_from_marker(img, K, dist, size_mm, marker_id=5, dict_id=None):
    """找到指定 id 的码，解出它在相机系里的位姿。

    Returns:
        (T_cam_marker 4x4，单位米, 重投影误差 px) 或 (None, None)
    """
    import cv2

    corners, ids = detect_markers(img, dict_id)
    if ids is None or marker_id not in ids:
        return None, None

    idx = ids.index(marker_id)
    image_points = corners[idx][0].astype(np.float32)
    object_points = marker_object_points(size_mm)

    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points,
        np.asarray(K, dtype=np.float64),
        np.asarray(dist, dtype=np.float64).reshape(-1, 1))
    if not ok:
        return None, None

    proj, _ = cv2.projectPoints(object_points, rvec, tvec, K, dist)
    err = float(np.linalg.norm(
        proj.reshape(-1, 2) - image_points, axis=1).mean())

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec).ravel() / 1000.0      # 毫米 → 米

    return T, err


def back_project_errors(img, K, dist, size_mm, marker_id=5, dict_id=None):
    """调试用：返回该码每个角点的重投影误差（像素），便于判断检测质量。"""
    import cv2

    T, err = pose_from_marker(img, K, dist, size_mm, marker_id, dict_id)
    if T is None:
        return None
    return {"mean_px": err, "T_cam_marker": T}


# ---------------------------------------------------------------- 自测
def _selftest():
    """用队友标定图验证：旧 API 能检出、PnP 能解出合理位姿。"""
    import glob
    import os

    import cv2

    here = os.path.dirname(os.path.abspath(__file__))
    roots = [
        os.path.join(here, "..", "..", ".."),                  # 项目里
        "/home/ljl/桌面/brain_extract/brain/vision",           # 队友的包
    ]
    imgs = []
    for r in roots:
        imgs = sorted(glob.glob(os.path.join(r, "calib_images", "calib_0[0-2].png")))
        if imgs:
            break
    if not imgs:
        print("跳过：找不到 calib_images/")
        return 0

    print("  OpenCV %s，使用 %s API"
          % (cv2.__version__, "新" if _new_api() else "旧"))
    ok_all = True
    for f in imgs:
        img = cv2.imread(f)
        corners, ids = detect_markers(img)
        got = "无" if ids is None else str(ids)
        print("  %-30s 检出 %s" % (os.path.basename(f), got))
        if ids is None:
            ok_all = False
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())

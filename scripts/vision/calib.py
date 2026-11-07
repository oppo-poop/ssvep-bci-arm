#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定文件的读写与校验（内参 + 外参）。

为什么需要这一层
    队友和我们在同一台机械臂、同一台相机上开发，但**两套环境**
    （他 Python 3.12 + Jazzy，我们 3.10 + Humble）。如果各自定义标定文件格式，
    硬件一搬动就得两边各标一次，而且很容易踩单位坑——他们的 npz 里同时有
    `T_base_cam`（米）和 `T_base_cam_mm`（毫米），用错一个差 1000 倍，
    表现是"目标点飞到天上"，很难第一时间想到是单位问题。

约定（本项目所有标定文件的唯一标准）
    内参 npz:  K (3x3, 像素) · dist (N,) · image_size (w, h) · rms_px
    外参 npz:  T_base_cam (4x4, **米**) · rms_px · method · note

    外参一律用**米**，不再存毫米副本。要显示时自己乘 1000。
"""

import numpy as np


# ---------------------------------------------------------------- 内参
def save_intrinsics(path, K, dist, image_size, rms_px=None, note=""):
    """保存内参。image_size = (width, height)，像素。"""
    np.savez(
        path,
        K=np.asarray(K, dtype=np.float64),
        dist=np.asarray(dist, dtype=np.float64).ravel(),
        image_size=np.asarray(image_size, dtype=np.int32),
        rms_px=np.float64(rms_px) if rms_px is not None else np.float64(-1),
        note=np.array(note),
    )
    return path


def load_intrinsics(path, expect_size=None):
    """读内参。expect_size 给了就检查分辨率是否一致。

    ⚠ 内参只在**标定时的分辨率**下有效。换了分辨率必须重标，
    否则反投影会系统性偏移（不是线性缩放那么简单）。
    """
    d = np.load(path, allow_pickle=False)
    K = d["K"]
    dist = d["dist"]
    size = tuple(int(v) for v in d["image_size"]) if "image_size" in d else None
    rms = float(d["rms_px"]) if "rms_px" in d else None

    if size is None:
        raise ValueError("%s 缺 image_size 字段，无法确认内参对应哪一档分辨率" % path)
    if expect_size is not None and tuple(expect_size) != size:
        raise ValueError(
            "内参是给 %dx%d 标定的，当前帧是 %dx%d —— 分辨率不一致，必须重标"
            % (size[0], size[1], expect_size[0], expect_size[1]))

    return K, dist, {"image_size": size, "rms_px": rms,
                     "note": str(d["note"]) if "note" in d else ""}


# ---------------------------------------------------------------- 外参
def save_extrinsics(path, T_base_cam, rms_px=None, method="", note=""):
    """保存相机在基座系的位姿。**T 用米**。"""
    T = np.asarray(T_base_cam, dtype=np.float64)
    check_pose(T)
    np.savez(
        path,
        T_base_cam=T,
        rms_px=np.float64(rms_px) if rms_px is not None else np.float64(-1),
        method=np.array(method),
        note=np.array(note),
    )
    return path


def load_extrinsics(path):
    """读外参，返回 (T_base_cam 米, 元信息)。"""
    d = np.load(path, allow_pickle=False)
    if "T_base_cam" not in d:
        raise KeyError("%s 里没有 T_base_cam" % path)
    T = d["T_base_cam"]
    check_pose(T)
    # 单位体检：正规的相机-基座距离是 0.2~2 m；
    # 如果数量级像毫米（>100），几乎可以肯定是单位用错了
    dist = float(np.linalg.norm(T[:3, 3]))
    if dist > 100.0:
        raise ValueError(
            "%s 的平移量 %.1f —— 看起来是毫米，但本约定要求米。差 1000 倍。"
            % (path, dist))
    return T, {
        "rms_px": float(d["rms_px"]) if "rms_px" in d else None,
        "method": str(d["method"]) if "method" in d else "",
        "note": str(d["note"]) if "note" in d else "",
    }


def check_pose(T):
    """检查一个 4x4 位姿的数值健康度：旋转正交、行列式为 1。"""
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("位姿必须是 4x4，收到 %s" % (T.shape,))

    R = T[:3, :3]
    ortho = float(np.max(np.abs(R @ R.T - np.eye(3))))
    det = float(np.linalg.det(R))

    if ortho > 1e-6:
        raise ValueError("旋转矩阵不正交，最大偏差 %.2e（标定求解出问题了）" % ortho)
    if abs(det - 1.0) > 1e-6:
        raise ValueError("旋转矩阵行列式 %.6f ≠ 1（可能是镜像或转置写反）" % det)
    return True


# ---------------------------------------------------------------- 兼容队友的旧文件
def import_teammate_npz(handeye_path, out_path=None):
    """把队友的 `handeye_result.npz` 转成本项目格式。

    他们存了两份平移（`T_base_cam` 米 + `T_base_cam_mm` 毫米），
    这里只保留米制的 `T_base_cam`，避免以后有人拿错键。
    """
    d = np.load(handeye_path, allow_pickle=False)
    T = d["T_base_cam"]
    rms = float(d["rms_px"]) if "rms_px" in d else None
    method = str(d["method"]) if "method" in d else ""
    note = "从队友的 handeye_result.npz 转换；注意该标定在 2026-10-21 换支架后已作废"

    if out_path:
        save_extrinsics(out_path, T, rms_px=rms, method=method, note=note)
        return out_path, T
    return None, T


# ---------------------------------------------------------------- 自测
def _selftest():
    import os
    import tempfile

    print("  1) 用队友的真实外参验证")
    here = os.path.dirname(os.path.abspath(__file__))
    cand = [
        os.path.join(here, "..", "..", "data", "handeye_result.npz"),
        os.path.join(here, "..", "..", "data", "vision", "handeye_result.npz"),
    ]
    base = os.environ.get("TEAMMATE_VISION_DIR")
    if base:
        cand.insert(0, os.path.join(base, "data", "handeye_result.npz"))
    src = next((p for p in cand if os.path.isfile(p)), None)
    if src is None:
        print("     跳过：找不到 handeye_result.npz（可用 TEAMMATE_VISION_DIR 指定队友目录）")
    else:
        _, T = import_teammate_npz(src)
        print("     T 平移(mm) = %s" % np.round(T[:3, 3] * 1000, 1))
        print("     正交偏差 %.2e，行列式 %.6f" % (
            float(np.max(np.abs(T[:3, :3] @ T[:3, :3].T - np.eye(3)))),
            float(np.linalg.det(T[:3, :3]))))

    print("  2) 单位体检（把米当毫米写会报错）")
    bad = np.eye(4)
    bad[:3, 3] = np.array([-63.9, 165.5, 439.5])       # 毫米
    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as f:
        tmp = f.name
    np.savez(tmp, T_base_cam=bad)
    try:
        load_extrinsics(tmp)
        print("     ✗ 应该报错却没报")
        ok = False
    except ValueError as e:
        print("     ✓ 正确拒绝：%s" % str(e)[:48])
        ok = True
    finally:
        os.unlink(tmp)

    print("  3) 内参分辨率检查")
    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as f:
        tmp2 = f.name
    save_intrinsics(tmp2, np.eye(3), np.zeros(5), (1280, 720), rms_px=0.83)
    try:
        load_intrinsics(tmp2, expect_size=(640, 480))
        print("     ✗ 分辨率不符却没报")
        ok = False
    except ValueError:
        print("     ✓ 正确拒绝分辨率不一致")
    finally:
        os.unlink(tmp2)

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())

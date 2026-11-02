# scripts/vision — 视觉抓取

相机 → 检测 → 基座坐标 → 机械臂抓取。代码来自合作方（2026-11-01 最终版），
按本项目 Humble 环境适配。

> 设计取舍、为什么这么移植、踩过哪些坑，见 [`docs/视觉.md`](../../docs/视觉.md)。
> 这里只讲**怎么用**。

## 一、模块

| 文件 | 作用 | 备注 |
|---|---|---|
| `camera.py` | 抓一帧彩色图 | 设备发现：`CAM_INDEX` 环境变量 > 名字匹配 > 第一个 `/dev/video*` |
| `project.py` | 像素 → 基座坐标（桌面平面反投影） | **20 行核心**，`pixel_to_base(u, v, K, dist, T, z)` |
| `aruco.py` | ArUco 检测 + PnP | **兼容新旧 OpenCV API**（本机 4.5.4 没有 `ArucoDetector`） |
| `calib.py` | 标定文件读写与校验 | 单位体检（平移 >100 判为毫米误用）、分辨率一致性 |
| `pick.py` | 抓取主流程（1872 行） | 一条龙 / 两段式都可；详见下面 |
| `calib_intrinsic.py` | 内参标定（4 码大板，空格采集） | 输出 `calib_intrinsics_raw.npz`，剔坏图后才是 clean 版 |
| `calib_xy.py` | XY 线性修正标定（探针法） | 生成 `xy_correction.json` |
| `handeye.py` | 手眼标定（码在夹爪，多姿态） | 相机被动过时用它重标 |
| `fix_extrinsics.py` | 外参平移**应急**修正 | 只修平移不修旋转，最终还是要 `handeye.py` |
| `depth_probe.py` | 读一帧深度图 | 需要 ROS 话题；当前相机没深度，暂未接 |
| `live_view.py` | 实时画面窗口 | 调试用，按 `s` 存图 |
| `snap_color.py` | 拍一张存盘 | 最小依赖，不碰深度 |

## 二、数据（`data/vision/`）

| 文件 | 内容 | 能不能用 |
|---|---|---|
| `cam_intrinsics.npz` | 内参 K/dist + 分辨率 | ✅ **能用**（内参只跟镜头有关，与安装无关） |
| `T_base_cam.npz` | 相机在基座系的位姿（米） | ⚠️ **只在标定那一刻的安装下有效**；换支架/搬动就要重标 |
| `grasp_offset.json` | 夹爪 TCP 相对 link6 的偏移 | ⚠️ **换夹爪就要重解**（2026-11-01 换过爪，x 从 28.6 → 50.6） |
| `xy_correction.json` | 视觉 XY 的系统性偏差（线性） | ⚠️ 是**经验补偿**，掩盖了标定问题；标定准了就不需要 |
| `handeye_poses.txt` | 手眼标定的 26 个位姿 | ✅ 已用 MoveIt IK 验证可达，`handeye.py` 直接用 |
| `pick_log.csv` | 抓取记录（自动追加） | — |

**判断数据还能不能用**：看文件时间 vs 相机最后一次被碰的时间。
`T_base_cam` 比"上次碰相机"早 → 作废。

## 三、常用命令

### 抓一次（先空跑）

```bash
# 只算目标点、出图，不动机械臂
python3 scripts/vision/pick.py --dry-run

# 确认十字压在物体上，再真抓
python3 scripts/vision/pick.py

# 不连相机，用已有图片跑（离线调检测逻辑）
python3 scripts/vision/pick.py --dry-run --from-file some.png
```

**两段式**（BCI 用，先取再放）：

```bash
# 取完举着别动（回 home 会把物料搬出视野）
python3 scripts/vision/pick.py --no-place --hold-here

# 物料已在爪里，投放到指定像素
python3 scripts/vision/pick.py --place-only --drop-u 640 --drop-v 480
```

### 标定

```bash
# ① 内参（同一台相机标过一次就够，通常不用重做）
python3 scripts/vision/calib_intrinsic.py        # 空格采集 / ESC 结束
#    输出 calib_intrinsics_raw.npz；剔掉坏图重算才是 clean 版
#    （合作方的 clean 版 rms 0.83 px，未剔的 1.378 px，差一倍）

# ② 手眼（换支架/搬动后必做）
python3 scripts/vision/handeye.py --capture        # 按位姿表自动走 26 个姿态
python3 scripts/vision/handeye.py --list           # 看每个样本的重投影误差
python3 scripts/vision/handeye.py --solve          # 先看 σ，别急着写
python3 scripts/vision/handeye.py --solve --write

#    σ < 2 mm 好 / 2~5 能用 / > 5 别用

# ③ 夹爪偏移（换夹爪后）
#    队友的 gripper_offset_calib.py 注释写着"不要再用"（它算的是历次偏移平均），
#    当前值是手调 + 探针复测出来的：grasp_offset.json 的 source 字段有记录

# ④ XY 修正（视觉算出的点系统性偏，且偏差随位置线性变化时）
python3 scripts/vision/calib_xy.py
```

### 调试

```bash
# 实时画面（按 s 存图）
python3 scripts/vision/live_view.py

# 拍一张
python3 scripts/vision/snap_color.py

# 看标定文件的数值健康度
python3 -c "
import sys; sys.path.insert(0,'scripts/vision')
from calib import load_extrinsics
import numpy as np
T, m = load_extrinsics('data/vision/T_base_cam.npz')
print('相机位置(mm):', np.round(T[:3,3]*1000, 1))
"
```

## 四、三个容易踩的坑

**① 分辨率必须和标定时一致**
内参是按像素标的，换分辨率**不是简单缩放**。`calib.load_intrinsics(expect_size=...)`
会直接拒绝不一致的画面。

**② `pixel_to_base` 的参数顺序两边不同**
`project.py` 是 `(u, v, K, dist, T, z)`，`pick.py` / `calib_xy.py` 内部用的是
`(u, v, T, K, dist, z)`（队友的习惯）。每个文件里都有一个**本地包装**
把顺序转过来。

**改这两个文件时，别直接调 `project.pixel_to_base`，用文件里的包装。**
否则 4x4 的 T 会被当 3x3 内参，cv2 只报 `CV_IS_MAT(_cameraMatrix) && rows == 3`，
完全看不出是顺序问题。

**③ `z_g` 依赖物体高度**
反投影用"桌面平面"求交，`z_g` 是**物料质心**高度，不是桌面高度。
换一个更高的物体，`x/y` 会一起偏（视线是斜的）。

偏差量 ≈ `Δh × tan(俯角)`。俯角 55° 时，物体高 70 mm 就偏约 100 mm。

## 五、当前限制

| 限制 | 影响 |
|---|---|
| **相机没有深度**（`realsense2_camera` 找不到设备，实际是 Astra Pro） | `z_g` 只能手填；`depth_probe.py` 接不上 |
| **`T_base_cam` 是合作方标的** | 相机被动过就要重标，见 `handeye.py` |
| **`xy_correction` 是线性经验补偿** | rms 0.78 mm，但换安装位置后不成立 |
| **检测是 HSV 颜色阈值** | 换物料/换光照要重调阈值，`pick.py` 的 `blue_candidates` |
| **`--place-only` 需要像素** | BCI 侧还没有"选像素"的能力（检测框即刺激未实现） |

## 六、外部依赖

| 依赖 | 版本 | 说明 |
|---|---|---|
| `opencv-python` | 本机 4.5.4 | `aruco.py` 已兼容新旧 API |
| `numpy` | 1.26.4 | |
| `scipy` | 有 | 仅 `handeye.py` 用（`Rotation`） |
| `rclpy` | Humble | 仅 `depth_probe.py` 用；其他都不依赖 ROS |

**除 `depth_probe.py` 外，所有脚本都不需要 source ROS**，也不需要机械臂在线
（`--dry-run` / `--from-file`）。

# config/ —— 物理口径的唯一来源

本目录只放**跨文件共享的物理参数和默认值**。业务逻辑、任务调参、MoveIt 参数不放这里。

| 文件 | 内容 |
|---|---|
| `robot.json` | 臂与夹爪：夹爪端点、joint 映射、启动位、零位偏移、home/路点、默认姿态、任务工作区、串口 |
| `vision.json` | 视觉默认值：场景 z、物高、夹爪偏移兜底、z 命令补偿、视觉可达包络 |

## 读取方式

- Python 脚本：`from roarm_lib import params`（`scripts/roarm_lib/params.py` 会从 `config/` 加载）。
- `roarm_task`：`roarm_task/config.py` 用 importlib 加载同一个 `params.py`。
- 厂商/硬件节点（`robo_driver.py`、`cello_controller.py`、C++ 节点）：**不在运行时读本目录**，保留源码常量；由 `scripts/checks/grip_check.py` 强制对齐。原因见 `ARCHITECTURE.md`。

`ROARM_ROOT=/path/to/ssvep-Roarm` 可以在非常规运行方式下指定仓库根。

## 数据优先级

运行时数据文件优先于本目录的默认值：

1. `data/scene_real.yaml` —— 示教场景（工作区权威）
2. `data/vision/grasp_offset.json` —— 夹爪偏移（当前实测权威）
3. `data/vision/xy_correction.json` —— XY 线性修正与投放微调
4. `data/vision/cam_intrinsics.npz` / `T_base_cam.npz` —— 相机内外参
5. `config/vision.json` —— 仅在上述文件缺失时兜底

标定/示教数据不要复制到本目录；本目录只放“默认值”和“物理口径”。

## 当前值的依据

### 夹爪（2026-11-03 拆装后，卸力手推实测）

| 项 | 值 | 说明 |
|---|---|---|
| 闭合舵机角 | 86.7° | 两指完全闭合，结构间隙 4 mm |
| 张开舵机角 | 174.8° | 两指全开 65 mm |
| `joint7_left` 闭合 | -0.025 m | URDF 下限 |
| `joint7_left` 张开 | 0.000 m | URDF 上限 |
| 行程 | 88.1° / 61 mm | 旧值 78.1° / 57 mm 已作废 |
| 启动位 | 121.0° | 约 27.8 mm 半开，旧值 50° 会顶死 |

旧值 `21.9°~100°`、`8~57 mm` 是换爪前的标定，**不要再用**。历史原因见 `docs/素材/04-技术决策.md` 和 git log。

### 零位偏移（2026-10-18 标定）

`[0.4, -0.2, -0.1, 1.3, -0.3, 0.8]`，含义是 URDF 角度为 0 时的舵机读数。
`robo_driver.py`、`go_zero.py`、`limit_probe.py` 必须一致；`grip_check.py` 会检查。

### 视觉 z / y 命令补偿

`vision.json.grasp.z_cmd_offset_m = -0.001`：2026-11-04 同步队友重标后的值
（此前 11-03 实抓曾降到 -0.006）。该偏差随姿态变化，常数只能压住当前工作区；
临时可用 `pick.py --z-offset-mm` 覆盖。

`vision.json.grasp.y_bias_mm = -2.0`：2026-11-04 实抓复核，y 偏 +2mm，
目标点反向补 -2mm（基座系）。保留这个旋钮，换姿态/换爪后重新确认。

`vision.json.grasp.x_bias_mm = -3.0`：2026-11-04 实抓复核，x 偏 +3mm，
目标点反向补 -3mm（基座系）。它只作用于抓取点，且在 520/右上象限判据之后叠加，
不会改变是否触发远端 +20mm 补偿；换姿态/换爪后重新确认。

`vision.json.grasp.check_grasp = false`：2026-11-05 夹爪 3D 打印件损坏，
只能空夹，暂时跳过“闭合误差是否说明夹到物体”的判据。流程会继续抬升/放置，
pick 日志记 `grasp_ok=skipped`；夹爪命令没送达（读不到误差）仍然会中止。
机械件修好后改回 `true`。

**2026-11-04 同步的运行时数据**（与队友 `vision/data/` 一致）：
`grasp_offset.json` = `[32.1, 23.6, 79.2]`、`xy_correction.json`
（`A[1][1]=1.24432`、`place_dx/dy=-5/-21.5`）、`T_base_cam.npz`（20:59 重标）。
同步前备份在 `/tmp/roarm_calib_before_sync_20261104/`。

### 颜色阈值

**2026-11-04 起，物料三色的 HSV 与检测判据全部同步队友 `cap/color_detect.py`
（2026-11-01 版）当前值。** 改之前先确认队友那边是否也改了。

`vision.json.colors.<红/绿/蓝/黄>.material` 是物料 HSV 范围，
`.tray` 是同色托盘范围（只给旧点选/`--snap-color` 路径用；`--auto` 走
`color_detect`，不套托盘掩码，靠面积/形态/亮度排除托盘）。

`vision.json.detect` 控制自动识别（`scripts/vision/color_detect.py`，移植自队友）：
`max_area_px=16000`、`min_area_px=400`、`aspect_range=[0.30,2.60]`、
`min_shape=1.05`、`big_area_px=1000`；
`v_mean_min` 按整块平均亮度排除绿地毯/暗背景，三色都是 `130`。
绿色物料与托盘 HSV 重叠，默认 `green.mask_tray=false`，靠这组面积/形态/亮度判据区分。
`detect.reach_mm = [250, 670, -430, 380]` 是队友 `color_detect.REACH_MM`，
比 pick 的任务工作区 `reach`（278~670 / -400~350）宽：识别先看见画面边缘，
能不能抓再由 pick 的 BOUNDS 判。
`tall_aspect_max`/`tall_solidity_min` 只保留给 `pick.py --snap-color` 的旧候选路径。

投放色纸（`color_detect.find_pads`）使用队友的 `PAD_RANGES`（更宽 + 多档 S/V +
毫米面积/填充率），当前没有并进 `colors.<色>.tray`；后者只用于旧掩码/点选路径。

### home / 路点 / 边界

- `home` 是笛卡尔点，不是关节零位。
- `bounds` 是 `roarm_cli.py` 的“任务工作区”（上位机旧推荐区），越界默认拒绝，`--force` 可越过。
- 视觉侧的可达包络 `vision.json.reach` 更宽，是 `probe_reach.py` 实测值；两套边界语义不同，不要混用。
- `arm.park_pose_deg` 是 6 个臂关节的**舵机读数**（不含夹爪），供 `roarm_cli.py park`
  和 `calib/go_zero.py --preset park` 使用；值 `[0.4,-90.6,89.3,2.4,-6.1,-1.7]`。

## 更新协议

1. 改 `config/*.json`；
2. 跑 `python3 scripts/checks/grip_check.py`；
3. 若提示厂商节点常量不一致，同步修改 `robo_driver.py` / `cello_controller.py` / C++ 常量；
4. 重新构建受影响的 ROS 包（`TMPDIR=$HOME/cctmp colcon build --symlink-install --packages-select ...`）；
5. 跑 `bash scripts/check_all.sh`；
6. 在提交信息里写清“实测日期 + 测量方法”，不要只写新数值。
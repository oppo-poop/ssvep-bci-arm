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

### 夹爪（2026-11-07 队友标定，3D 打印件换新后重标）

**角度是相对起栈基准的偏移量，不是绝对舵机角。** 换爪 / 断一次电，多圈零点就变，
同一个绝对角两次起栈可以一次合拢、一次张到顶（实测过），所以 driver 起栈时读一次
实际角度记为 `base`，之后 `target = base + deg`。这也意味着：

- `start_deg` 必须是 `null`（原地不动）。写死一个绝对角会"绕一圈再顶到机械极限"。
- 多圈复位只能 `reset_multi_turn_angle(6)`（单独 ID）。`0xff` 广播这类舵机不认，
  会被静默忽略，等于没复位。
- 读方向必须减 `base`，否则界面会把"合拢"显示成"全开 50.9 mm"。

| 项 | 值 | 说明 |
|---|---|---|
| 基准角 `close_deg` | 0.0° | 起栈时两指拨到合拢，记这个位置当 0 |
| 全开角 `open_deg` | -100.0° | 从基准再张 100°，**机械极限** |
| 闭合间距 `gap_closed_mm` | 1.6 mm | 合到底仍有结构间隙 |
| 全开间距 `gap_open_mm` | 50.9 mm | 实测表 100° → 50.9 |
| 曲线 `curve_k` | 0.412 | 前 40° 基本空程；线性会把半开算高 6 mm |
| `joint7_left` | -0.025 ~ 0.000 m | URDF 上下限，没变 |
| 语义目标 | open 50.9 / close 3.0 mm | `grip open|close` 走这两个值 |
| 行程时长 `move_ms` | 3000 ms | 走完 100° 的时长，**下限 1500**。夹爪是打印件，1000 ms 的冲击会把爪子打坏（11-07 就是这么坏的）。驱动再 +400 ms → 实际 3.4 s；梯形加减速 `acc=interval/2` 跟着变柔。CLI / `cello_controller` / `arm_moveit_write.cpp` 三处都读它，`grip_check` 查一致 |

夹爪命令的**等待**要跟着 `move_ms` 走：`params.GRIP_WAIT_S = ceil(move_ms/1000 + 0.4 + 1.6)`
（当前 5 s）。`link.py`、`pick.py cli_grip` 都用它；写死 4 s 在 3.4 s 行程下只剩 0.6 s 余量，
抖一下就误判"没到位"。

实测标定表（相对角 → 两指间距，mm）：
`0→1.6  5→2.7  10→3.9  20→6.7  30→10.2  40→14.3  50→19.0  60→24.1  70→30.0  80→36.4  90→43.4  100→50.9`

⚠ **这副爪全开 50.9 mm，夹 50 mm 物料单边只有 0.45 mm 余量**，而且 `open` 只能顶在
机械极限上（再小就套不进物料）。`check_holding` 里"接近全开就判没夹住"的判据
对 50 mm 物料已经失效。要长期跑，得换爪或换更小的物料。

旧值（`86.7°~174.8°` / `4~65 mm`，绝对角、线性映射）是 11-03 那次装配的口径，
已作废；历史原因见 `docs/素材/04-技术决策.md` 和 git log。

### 零位偏移（2026-10-18 标定）

`[0.4, -0.2, -0.1, 1.3, -0.3, 0.8]`，含义是 URDF 角度为 0 时的舵机读数。
`robo_driver.py`、`go_zero.py`、`limit_probe.py` 必须一致；`grip_check.py` 会检查。

### 视觉 z / y 命令补偿

`vision.json.grasp.z_cmd_offset_m = 0.014`：2026-11-10 实机重标（原 -0.001）。
这台臂的 z 落点系统性**低于**命令值，实测：

| 命令 z | 实际 z | 偏差 |
|---|---|---|
| 0.060 | 0.0506 | −9.4 mm |
| 0.200 | 0.1836 | −16.4 mm |
| 0.350 | 0.3340 | −16.0 mm |

补 +14 mm 后验算：命令 0.074 → 实际 0.0623（目标 0.060，残余 +2.3 mm）。
`pick.py` 把它加在**发出去**的 z 上（`z_send = z + offset`），所以调用方和
边界检查仍用意图值。偏差随姿态略变，常数只能压住当前工作区；
临时可用 `pick.py --z-offset-mm` 覆盖。

> ⚠ 量这个偏差时 **tol 必须收紧（≤5 mm）**。判据是"连续 3 次进入容差就返回"，
> tol 设大了（试过 30）臂会在运动途中被判"到位"，读到的是半路位置 ——
> 2026-11-10 就这么废掉三组数据（偏差显示 ±28~30 mm 且符号随运动方向翻转）。

`vision.json.grasp.y_bias_mm = -1.0`：2026-11-04 实抓复核，y 偏 +2mm，
目标点反向补 -2mm（基座系）。保留这个旋钮，换姿态/换爪后重新确认。

`vision.json.grasp.x_bias_mm = -2.0`：2026-11-04 实抓复核，x 偏 +3mm，
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
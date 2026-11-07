# data/ —— 标定、示教与运行数据

本目录混有两类东西，**处理前先分清**：

## 一、必须保留（标定/示教/实测结果）

| 路径 | 内容 | 谁在用 |
|---|---|---|
| `scene_real.yaml` | 当前真机示教场景 | `roarm_task`、`vision/pick.py`、`run_task.sh` |
| `teach_points.json` | 示教原始点（`teach.py` 中间产物） | `teach.py` |
| `zero_calib.csv` | 零位标定姿态记录 | `zero_calib.py` / 人工 |
| `joint_limits.csv` | 六轴行程实测 | `limit_probe.py` / 人工 |
| `repeatability.csv` | 重复定位精度样本 | `repeatability.py` / 人工 |
| `runs/summary.csv` | 取放结果汇总 | 人工/文档 |
| `vision/cam_intrinsics.npz` | 相机内参（1280×720） | 视觉脚本 |
| `vision/T_base_cam.npz` | 相机外参（米） | 视觉脚本 |
| `vision/grasp_offset.json` | 夹爪 TCP 偏移（当前实测权威） | `pick.py` |
| `vision/xy_correction.json` | XY 线性修正 + 投放微调 | `pick.py`、`calib_xy.py` |
| `vision/handeye_poses.txt` | 手眼采集的 26 个位姿 | `handeye.py` |
| `vision/trays.json` | **可选**：红/绿/蓝托盘像素 `{color:[u,v]}`，给 `arm_service.py` | 不写则服务自动用 `find_pads` 检测 |

**不要**为了目录整齐移动这些文件；代码里有大量相对 `data/` 的路径，
移动要同步改 `scripts/roarm_lib/paths.py`、`teach.py`、`run_task.sh` 和文档。

## 二、运行产物（可删/可重建，已进 .gitignore）

| 路径 | 产生者 |
|---|---|
| `runs/*.log` | `run_task.sh`（只保留 `summary.csv`） |
| `vision/pick_log.csv` | `pick.py` |
| `vision/timing.jsonl` | `pick.py` / `arm_service.py` 阶段计时（JSONL） |
| `vision/preview/` | `pick.py` / `calib_xy.py` 诊断图 |
| `vision/calib_images/` | `calib_intrinsic.py` 采集图 |
| `vision/calib_intrinsics_raw.npz` | `calib_intrinsic.py` 原始输出（剔坏图后才成为 clean 版） |
| `vision/handeye_samples.npz` | `handeye.py --capture` 中间样本 |
| `vision/probe_points.csv` | `pick.py --probe --record` 探针记录 |
| `vision/T_base_cam.npz.bak` | `fix_extrinsics.py` 写回前的备份 |

## 三、优先级

`data/` 中的运行时数据优先于 `config/vision.json` 的默认值：

1. `scene_real.yaml`（示教/工作区）
2. `vision/grasp_offset.json`（夹爪偏移）
3. `vision/xy_correction.json`（XY 修正/投放微调）
4. `vision/cam_intrinsics.npz` / `T_base_cam.npz`（相机标定）
5. `config/vision.json`（仅在以上缺失时兜底）

标定/示教数据的含义与更新方法见 `config/README.md`、`docs/视觉.md`、
`docs/零位标定.md`。
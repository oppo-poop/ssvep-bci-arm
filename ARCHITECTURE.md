# 架构与分层

> 一句话：ROS 2 负责“机器怎么动”，`scripts/` 负责“人怎么用”。
> 不是所有功能都要做成节点；但所有入口必须可以独立运行、可离线自检。

## 一、分层

```
┌──────────────────────────────────────────────────────────────┐
│ 入口层                                                       │
│   scripts/roarm_cli.py   唯一控臂契约（stdout + 0/1/2 退出码）│
│   scripts/teach.py       交互示教                            │
│   scripts/vision/*.py    视觉抓取（队友子系统，扁平目录）     │
│   ros2 run roarm_task    任务层 CLI                          │
│   scripts/intent_bridge.py / 外部 ssvep-bci                  │
└──────────────────────────────┬───────────────────────────────┘
                               │ 子进程 / ROS 话题 / action
┌──────────────────────────────▼───────────────────────────────┐
│ 应用层                                                       │
│   roarm_task        场景 → 序列 → 七原语 → CLI 子进程         │
│   vision pick.py    相机 → 反投影 → 夹爪偏移 → CLI 子进程     │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│ 公共库 / 参数                                                │
│   scripts/roarm_lib/params.py  物理口径 + 派生换算（唯一实现）│
│   scripts/roarm_lib/paths.py   仓库路径唯一来源              │
│   config/robot.json            夹爪/零位/home/边界/串口      │
│   config/vision.json           视觉默认值/可达包络           │
└──────────────────────────────┬───────────────────────────────┘
                               │ ROS 2
┌──────────────────────────────▼───────────────────────────────┐
│ ROS 运行时（star-arm-moveit2，厂商/上游代码 + 本项目补丁）    │
│   arm_moveit_write → move_group → cello_controller            │
│   → robo_driver → 7× 舵机（1 Mbaud 串口）                    │
└──────────────────────────────────────────────────────────────┘
```

**不做的事**：不把标定/诊断脚本改成 ROS 节点，不把任务层改成直接发 action。
原因见 `README.md` 的“架构决定”和 `roarm_task/config.py`：到位判据只保留
`roarm_cli.py` 一份，任务层通过子进程复用，避免两套判据漂移。

## 二、目录职责

| 目录 | 职责 | 不要放什么 |
|---|---|---|
| `config/` | 跨文件共享的物理参数和默认值 | 业务逻辑、标定结果、运行日志 |
| `scripts/roarm_lib/` | 公共库：路径、参数派生、FK | ROS 依赖、视觉算法、业务流程 |
| `scripts/calib/` | 一次性标定/测量工具 | 运行期会被任务层调用的逻辑 |
| `scripts/checks/` | 离线自检和回归测试 | 需要真机才能跑的主流程 |
| `scripts/vision/` | 视觉链路（队友移植，保持扁平） | ROS 节点、任务层状态 |
| `scripts/` 根 | 日常入口：CLI、示教、诊断、启停 | 第二份参数/换算 |
| `data/` | 标定/示教/实验数据 | 代码、构建产物 |
| `docs/` | 现状说明、决策记录、申报素材 | 自动生成的内容 |
| `star-arm-moveit2/` | ROS 2 工作区（上游代码 + 补丁） | 项目脚本、视觉代码 |

## 三、不能改的契约

### 1. `roarm_cli.py`

- 路径：`scripts/roarm_cli.py`（`roarm_task/config.py` 和外部 `ssvep-bci` 依赖）
- stdout 关键字（被正则解析）：
  `当前位置/实际 x=... y=... z=...`、`q=(...)`、`joint7_left=...`、
  `关节角(度): j1=...`、`到位  误差 ... mm`、`未到位  误差 ... mm`
- 退出码：`0` 到位，`1` 未到位/超时，`2` 输入被拒
- 不要改这些文案和退出码；要改先全局搜索消费者。

### 2. `roarm_task`

```bash
ros2 run roarm_task task --scene FILE --phase all|pick|place [--dry-run]
```

- 退出码 `0/1/2` 同上。
- 外部 `ssvep-bci/sinks/arm_sink.py` 依赖此入口。
- 场景 YAML 字段：`pick/place/object_height/object_width/grip_open/grip_close/
  standoff/clearance/verify_grasp/name`。改字段要同步 `scene.py`、`teach.py`、
  `pick.py::load_object_height`、`camera.py::load_scene_z`。

### 3. ROS 接口（`star-arm-moveit2`）

话题：`position_orientation_topic`、`gripper_command_topic`、`set_angle_topic`、
`joint_states`；action：`/arm_controller/follow_joint_trajectory`、
`/hand_controller/gripper_cmd`；TF：`world → link6`。
包名/launch 名/msg 字段不要改，除非全工作区重建并同步所有脚本。

### 4. 数据文件

`data/vision/*.npz/*.json`、`data/scene_real.yaml`、`data/teach_points.json`
的键名不要改。详见 `data/README.md`。

## 四、参数归属（唯一来源）

| 参数 | 唯一来源 | 消费者 |
|---|---|---|
| 夹爪端点/启动位、joint7 映射 | `config/robot.json` | 所有 Python 脚本；厂商节点保留常量，由 `grip_check` 对齐 |
| 零位偏移 | `config/robot.json` | `robo_driver`（常量）、`go_zero`、`limit_probe` |
| home/路点/默认四元数/任务工作区 | `config/robot.json` | `roarm_cli`、`roarm_task`、`pick.py` |
| 视觉场景 z/物高/夹爪偏移兜底/z 补偿/可达包络 | `config/vision.json` | `pick.py`、`camera.py`、`calib_xy.py` |
| 任务容差/接近/抬起/重试 | `roarm_task/config.py` | 任务层 |
| 速度/加速度/IK 超时/控制器参数 | MoveIt YAML | `move_group`、`arm_moveit_write` |
| 标定结果 | `data/vision/*` | 视觉脚本 |
| 示教场景 | `data/scene_real.yaml` | 任务层、视觉、`teach.py` |

**厂商/硬件节点例外**：`robo_driver.py`、`cello_controller.py`、C++ 节点不在
运行时读 `config/`（避免 ROS install 路径脆弱、启动失败）。它们保留源码常量，
由 `scripts/checks/grip_check.py` 强制与 `config/robot.json` 对齐；改配置后必须
重新 `colcon build` 受影响的包。

## 五、第三方边界

- `star-arm-moveit2/` 的上游来源、补丁清单见 `star-arm-moveit2/NOTICE.md`。
- `scripts/vision/` 是队友 2026-11-01 最终版的移植：
  - 保持扁平目录（模块间是 `import camera/project/...`）；
  - 入口只有 `pick.py`，通过子进程调用 `roarm_cli.py`，ROS 节点不 import 视觉代码；
  - 环境基线（Python 3.10 / OpenCV 4.5.4 / 新旧 ArUco API）见
    `scripts/vision/README.md`；
  - 硬编码的个人路径已改为 `TEAMMATE_VISION_DIR` 环境变量或仓库内候选路径。

## 六、检查与回归

```bash
bash scripts/check_all.sh          # 一键离线检查（不需要硬件）
```

单项：

```bash
python3 scripts/checks/grip_check.py   # 夹爪口径 / 参数漂移
python3 scripts/checks/pick_check.py   # 视觉边界 / 落点补偿
python3 scripts/checks/traj_check.py   # 轨迹下发 / 取消保持
python3 -m unittest discover -s star-arm-moveit2/src/roarm_task/test -v
```

需要真机的项（夹爪端点、取消、完整取放）见 `README.md` 的验证清单；Agent
无法访问 `/dev`，真机结果由操作者确认。

## 七、新增脚本放哪

- 会被任务层/视觉调用的公共函数 → `scripts/roarm_lib/`；
- 一次性标定/测量 → `scripts/calib/`；
- 离线自检/回归 → `scripts/checks/`；
- 日常入口（需要最短路径）→ `scripts/` 根，并在 `scripts/README.md` 登记；
- 任何新参数先判断是不是跨文件共享：是 → `config/`；否 → 留在使用它的模块。
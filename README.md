# SSVEP-Roarm 脑控机械臂

基于 SSVEP 脑机接口的 6+1 自由度机械臂控制系统。从臂为 Starai Cello，软件栈为
ROS 2 Humble + MoveIt 2；SSVEP 解码由同级仓库 `../ssvep-bci` 提供，本仓库负责
机械臂执行、任务编排、视觉抓取、标定与验证。

> **设计取向**：ROS 2 负责实时执行，脚本层负责人机接口；物理参数只有一个来源；
> 关键逻辑均提供离线回归。分层与接口契约见 [`ARCHITECTURE.md`](ARCHITECTURE.md)。

## 快速开始

```bash
# 1) 构建（本机 /tmp 仅 10 MB，必须给 TMPDIR）
source /opt/ros/humble/setup.bash
mkdir -p "$HOME/cctmp"
cd star-arm-moveit2
TMPDIR="$HOME/cctmp" colcon build --symlink-install
cd ..
source star-arm-moveit2/install/setup.bash

# 2) 仿真（与第 3 步二选一）
bash scripts/sim_up.sh
ros2 run roarm_task task --dry-run

# 3) 真机（与第 2 步二选一；确认工作区无人、无障碍，电源开关在手边）
bash scripts/robot_up.sh

# 停止：先收节点，再用一条 3 s 命令回收纳位并解锁力矩（会先确认）
bash scripts/robot_up.sh --stop

# 4) 取放 / 视觉
bash scripts/run_task.sh --n 5
python3 scripts/vision/pick.py --dry-run

# 5) 离线自检（不需要 ROS、相机或真机）
bash scripts/check_all.sh
```

> 完整的依赖、构建、仿真、真机、视觉与诊断步骤见第 6 节；
> 真机验收项见第 7.2 节。

---

## 1. 系统架构

```
../ssvep-bci/realtime.py --target arm
        │  sinks/arm_sink.py
        ▼
ros2 run roarm_task task --phase pick|place
        │  scene → sequence → primitives
        ▼
scripts/roarm_cli.py          ← 唯一的到位判据（TF 实测 + 退出码 0/1/2）
        │  ROS 2 topics / actions
        ▼
arm_moveit_write → move_group → cello_controller → robo_driver → 7× 舵机
        ▲
        │  同一条 CLI 契约
scripts/vision/pick.py        视觉抓取：相机 → 反投影 → 夹爪偏移 → 抓/放
```

三条控制路径（SSVEP 任务、终端示教、视觉抓取）共用 `roarm_cli.py`，避免出现
多套到位判据。`scripts/` 不是 ROS 节点的替代品，而是操作员接口与一次性工具；
ROS 运行时仍由 `star-arm-moveit2` 的厂商/上游包承担。

---

## 2. 当前状态（2026-11-03）

| 模块 | 状态 | 证据 / 说明 |
|---|---|---|
| 仿真全链路 | 已验证 | 场景矩阵 16/16（107.5 s）；取放 10 步 ×2（38 s） |
| 任务层 | 已验证 | 24 项单元测试；`ros2 run roarm_task task` |
| 真机零点标定 | 已验证 | FK 距零位 4.8 mm；`joint_states` 残差 ≤0.6° |
| 真机 MoveIt 单点 | 已验证 | 误差 4.92 mm，9.9 s |
| 关节行程实测 | 已验证 | 6 轴全测；URDF 限位不宽于实测值 |
| 真机完整取放 | 已验证 | 连续 13 次通过（2026-10-19）；节拍 30~70 s，中位 64.7 s |
| SSVEP → 动作链路 | 部分验证 | 2026-10-19 首次脑控完成示教动作；端到端延迟、误判率尚未系统测量 |
| 视觉抓取 | 部分验证 | 12 个模块，真机已运行；2026-11-03 的精度修复待复测 |
| 检测框即刺激 | 未开始 | 当前为固定 5 块 SSVEP 选择，尚未由检测框直接生成刺激 |

---

## 3. 硬件与接口

| 项 | 参数 |
|---|---|
| 自由度 | 6 + 1（平行夹爪） |
| 臂展 / 全展 | 650 / 1340 mm |
| 额定负载 | 750 g（70% 臂展内） |
| 重复定位精度（官方） | 1 mm |
| 实测重复定位精度 | 3σ = 2.2 mm（10 次往返） |
| 实测绝对定位精度 | 10~11 mm（含 IK 关节舍入与零点残差） |
| 舵机 | RX8-U50H-M ×2 · RA8-U25H-M ×4 · RA8-U26H-M ×1 |
| 通信 | UC-01 USB 转总线（CH340），1 Mbaud，ID 0~6 |
| 零位末端 | 前方 479.6 mm、高 370.5 mm（URDF 实算，与真机 TF 吻合 0.5 mm） |
| URDF 关节限位 | j1/j4/j6 ±130°，j2/j3/j5 ±90° |
| 实测关节行程 | j1 339° / j6 356° / j4 291° / j5 211° / j2 187° / j3 183° |

> 重复定位精度与绝对定位精度是两个指标：前者描述“同一目标多次是否一致”，
> 后者描述“一次定位是否落在目标上”。任务层的二次定位补偿针对后者。

### 3.1 夹爪口径

| CLI 命令 | 舵机角 | `joint7_left` | 两指间距 |
|---|---|---|---|
| `close` | 86.7° | −0.025 m | 4 mm（合到底） |
| `open` | 174.8° | 0.000 m | 65 mm（全开） |

- 行程：88.1° / 61 mm；启动位 121°（约 28 mm 半开）。
- 换算：`joint7_left = (servo_deg − 174.8) × 0.025 / 88.1`。
- 唯一真值：[`config/robot.json`](config/robot.json)。厂商 ROS 节点保留源码常量，
  由 `scripts/checks/grip_check.py` 强制对齐；修改配置后必须重新构建相关包。

### 3.2 舵机内部控制参数

舵机自带梯形速度规划，主机侧不运行控制环：

```
DEFAULT_BAUDRATE            = 1_000_000
DEFAULT_MOTION_TIME         = 350 ms
DEFAULT_GRIPPER_MOTION_TIME = 100 ms
DEFAULT_ACC_TIME / DEC_TIME = 50 ms    ← 脑控场景应显式放长
```

`robo_driver` 订阅 `set_angle_topic` 写入舵机，并发布 `joint_states`。
通信健康状态可用 `python3 scripts/servo_check.py` 检查（只读）：

- 设备存在性与权限、被占用情况
- 波特率探测
- 7 个 ID 在线与丢包
- 电压、温度
- 夹爪开度

电压阈值按 12 V 供电设置（本机实测 11.8~12.2 V）：`VOLT_WARN=11.0`、
`VOLT_FAIL=10.2`。若改用 2S 锂电供电，应改为 7.0 / 6.4。

---

## 4. 工程结构

```
ssvep-Roarm/
├── ARCHITECTURE.md           分层、接口契约、参数归属、第三方边界
├── config/                   物理参数唯一来源
│   ├── robot.json            夹爪端点/零位/home/边界/串口
│   └── vision.json           视觉默认值/可达包络
├── scripts/
│   ├── roarm_cli.py          单条控臂指令（home/pose/grip/grip_deg/list/where）
│   ├── teach.py              交互示教：摆位 → 记录 → 生成场景 YAML
│   ├── servo_check.py        7 舵机通信与供电诊断
│   ├── sim_gripper_bridge.py 仿真夹爪桥
│   ├── intent_bridge.py      SSVEP 调试点位桥
│   ├── robot_up.sh           真机一键启动
│   ├── sim_up.sh             仿真一键启动
│   ├── run_task.sh           取放连跑与结果归档
│   ├── calib/                一次性标定/测量
│   ├── checks/               离线自检与回归
│   ├── roarm_lib/            公共库（params/paths/fk）
│   └── vision/               视觉链路（12 个模块）
├── data/                     标定、示教与运行数据
├── docs/                     技术文档与申报素材
├── star-arm-moveit2/         ROS 2 工作区（上游代码 + 本项目补丁）
└── 参考资料/                  厂商 SDK、官方示例（不入库）
```

旧脚本路径（如 `scripts/zero_calib.py`、`scripts/grip_check.py`）保留为兼容
包装，行为、参数与退出码不变。完整索引见 [`scripts/README.md`](scripts/README.md)。

---

## 5. 参数与数据

### 5.1 唯一来源

| 类型 | 位置 | 说明 |
|---|---|---|
| 机械臂/夹爪物理参数 | `config/robot.json` | 端点、零位、home、默认姿态、任务工作区、串口 |
| 视觉默认值 | `config/vision.json` | 场景 z、物高、夹爪偏移兜底、z 补偿、可达包络 |
| 相机内外参 | `data/vision/cam_intrinsics.npz`、`T_base_cam.npz` | 安装相关；搬动相机后必须重标 |
| 夹爪偏移 / XY 修正 | `data/vision/grasp_offset.json`、`xy_correction.json` | 运行时权威数据 |
| 示教场景 | `data/scene_real.yaml` | 任务层与视觉共同读取 |
| 任务层调参 | `roarm_task/config.py` | 容差、接近、抬起、重试策略 |
| 速度/加速度/IK | MoveIt YAML | `joint_limits.yaml`、`kinematics.yaml` |

优先级：`data/` 中的运行时标定/示教数据高于 `config/vision.json` 的默认值。
详见 [`config/README.md`](config/README.md) 与 [`data/README.md`](data/README.md)。

### 5.2 参数更新流程

1. 修改 `config/*.json`；
2. 运行 `python3 scripts/checks/grip_check.py`；
3. 若提示厂商节点常量不一致，同步修改对应源码；
4. 重新构建受影响的 ROS 包；
5. 运行 `bash scripts/check_all.sh`；
6. 提交时注明实测日期与测量方法。

---

## 6. 详细启动步骤

### 6.1 环境要求

- Ubuntu + ROS 2 Humble + MoveIt 2
- Python 3.10；视觉需要 numpy、OpenCV（本机 4.5.4）、scipy（仅手眼标定）
- 仓库路径必须是纯 ASCII（`rosidl_generate_interfaces` 会在中文字符处截断）

### 6.2 构建

```bash
# 首次 clone 或修改 ROS 源码后执行
source /opt/ros/humble/setup.bash
mkdir -p "$HOME/cctmp"          # 本机 /tmp 仅 10 MB，编译需要更大的临时目录
cd star-arm-moveit2
TMPDIR="$HOME/cctmp" colcon build --symlink-install
cd ..
source star-arm-moveit2/install/setup.bash
```

### 6.3 仿真

```bash
bash scripts/sim_up.sh
ros2 run roarm_task task --dry-run
ros2 run roarm_task task --scene data/scene_real.yaml
```

### 6.4 真机

```bash
# 确认工作区无人、无障碍，电源开关在手边
bash scripts/robot_up.sh

# 上电后不自动驱零位，仅上力矩
ZERO_ON_START=false bash scripts/robot_up.sh

# 查看节点状态 / 停止
bash scripts/robot_up.sh --status

# 停止：先收节点，再用一条 3 s 命令回收纳位并解除力矩（默认会先确认）
bash scripts/robot_up.sh --stop

# 只停节点，不动臂（舵机仍保持力矩）
bash scripts/robot_up.sh --stop --no-park

# 非交互执行（脚本里用）
bash scripts/robot_up.sh --stop --yes
```

`robot_up.sh` 依次启动 `robo_driver`、`move_group`、`arm_moveit_write`，并在
`zero_on_start=true` 时用 3 s 将机械臂缓慢驱到零位。

停止流程：先终止节点以释放串口，再用**一条 3 s 命令**（`PARK_MS` 可调）调用
`scripts/calib/go_zero.py --preset park --direct --step-ms 3000 --go --unlock`，
回到折叠收纳位并解除全部舵机力矩。**解除力矩后机械臂将失去支撑，操作前请先托住。**
只停节点用 `--no-park`；非交互环境需显式加 `--yes`，否则默认跳过收纳/解锁。

### 6.5 取放任务

```bash
# 连跑 5 次，日志与汇总写入 data/runs/
bash scripts/run_task.sh --n 5

# 分阶段执行
ros2 run roarm_task task --scene data/scene_real.yaml --phase pick
ros2 run roarm_task task --scene data/scene_real.yaml --phase place
```

### 6.6 视觉抓取

```bash
# 只计算目标点，不动机械臂
python3 scripts/vision/pick.py --dry-run

# 真机抓取
python3 scripts/vision/pick.py

# 两段式：先取物并保持，再投放到指定像素
python3 scripts/vision/pick.py --no-place --hold-here
python3 scripts/vision/pick.py --place-only --drop-u 640 --drop-v 480
```

### 6.7 硬件诊断

```bash
python3 scripts/servo_check.py              # 完整检查
python3 scripts/servo_check.py --ping-only  # 仅检查 7 个 ID
python3 scripts/servo_check.py --mock       # 无硬件时验证输出
```

> 运行诊断前应停止机械臂驱动，串口会被独占。

### 6.8 8 指令服务（给 ssvep-bci）

```bash
# 先起 ROS 栈，再起服务（Unix socket，默认 data/arm.sock）
bash scripts/robot_up.sh
python3 scripts/arm_service.py

# 另开终端：0/1/2=取红/绿/蓝，3/4/5=放红/绿/蓝，6=回位，7=停止
python3 scripts/arm_client.py 0
python3 scripts/arm_client.py place green
python3 scripts/arm_client.py home
python3 scripts/arm_client.py stop
```

指令索引与 `ssvep-bci/config.py` 的 `ARM_NAMES_COLORS` 一致；协议、忙时丢弃、
停止/回位语义见 [`docs/接口.md`](docs/接口.md)。

---

## 7. 验证与回归

### 7.1 离线检查（无需硬件）

```bash
bash scripts/check_all.sh
```

包含：

| 检查 | 内容 |
|---|---|
| 编译 | `compileall` 覆盖脚本与 ROS Python 源码 |
| 参数 | `roarm_lib.params` 端点、往返、比例换算自测 |
| 任务层 | 24 项单元测试 |
| 夹爪口径 | 端点、启动位、零位偏移、公式与配置一致性 |
| 视觉 | 边界、落点补偿、参数单源回归 |
| 轨迹 | 整段单命令下发、取消保持、joint7 端点换算 |

### 7.2 真机检查清单

| 项 | 操作 | 通过标准 |
|---|---|---|
| 夹爪端点 | `roarm_cli.py grip close/open --wait 10 --tol 1` | `joint7_left` = −0.025 / 0.000 |
| 半开 | `grip_deg 121.0 --wait 10` | 约 28 mm，无越界拒绝 |
| 示教夹爪 | `teach.py` → `grip 0.5` | 10 s 内到位并记录比例 |
| 完整取放 | `run_task.sh` | 全步骤通过，夹爪不超时 |
| 取消 | 运动过程中请求取消 | 臂原地停止，夹爪不被推向错误角度 |
| 通信 | `servo_check.py` | 7 个舵机在线，电压/温度正常 |
| 停机 | `bash scripts/robot_up.sh --stop` | 回 park 并解除力矩；`--no-park` 只停节点 |

---

## 8. 已知限制与风险

| 限制 | 影响 / 处理 |
|---|---|
| SSVEP 端到端延迟、按类别准确率未系统测量 | 需真实脑电数据；`rho` 阈值目前为经验值 |
| STOP 不中断正在执行的任务 | 只阻止后续指令；急停仍依赖电源开关 |
| 检测框即刺激未实现 | 当前为固定 5 块选择；视觉检测与刺激尚未闭环 |
| action 层偶发 `unknown result response` | 动作本身执行，但结果可能被上层忽略，待排查 |
| 相机无可用深度 | 抓取平面 `z_g` 依赖手填/示教；换深度相机后接 `depth_probe` |
| 绝对定位误差 10~11 mm | 通过二次定位补偿压到 2~3 mm；换姿态后仍会有毫米级漂移 |
| 视觉标定与安装强相关 | 相机/支架移动后 `T_base_cam`、`xy_correction` 失效，必须重标 |
| 舵机型号表缺 `ra8-u26` | LeRobot 统一声明为 `rx8-u50`；若做型号握手校验可能报错 |

---

## 9. 设计决策

| 决策 | 理由 |
|---|---|
| 任务级宏动作，不做关节级连续控制 | BCI 带宽不足以支撑连续闭环；人决定目标，机器决定路径 |
| 任务层通过 CLI 子进程控臂 | 到位判据只保留一份（TF 实测 + 三档退出码），避免两套判据漂移 |
| 参数集中在 `config/` | 夹爪/零位等常量曾在 6 个文件重复，换爪后出现漂移 |
| 检测框直接作为 SSVEP 刺激（规划中） | 省去眼动硬件；看到什么盯什么，无需学习“第几行第几列” |
| 眼动暂缓 | 主流眼动仪 Linux 支持不足；L2CS-Net 精度有限且需单独标定 |
| 闪烁框位置冻结 N 秒 | 避免同时引入“检测抖动”和“动态刺激”两个未知量 |
| 取消 = 冻结当前角度，不是急停 | 舵机无急停指令；真正断电仍依靠电源开关 |

---

## 10. 文档与出处

| 文档 | 内容 |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | 分层、接口契约、参数归属、第三方边界 |
| [`docs/00-索引.md`](docs/00-索引.md) | 全部文档入口 |
| [`scripts/README.md`](scripts/README.md) | 脚本索引、兼容入口、自检命令 |
| [`config/README.md`](config/README.md) | 参数依据、优先级、更新协议 |
| [`data/README.md`](data/README.md) | 数据分类：保留 / 可删 |
| [`docs/视觉.md`](docs/视觉.md) | 视觉链路移植、数据可信度、踩坑 |
| [`docs/任务层.md`](docs/任务层.md) | 任务层结构、场景、阶段、抓取校验 |
| [`docs/意图层.md`](docs/意图层.md) | SSVEP → 机械臂的映射与联调 |
| [`docs/接口.md`](docs/接口.md) | 给 `ssvep-bci` 的 8 指令接口：取/放三色、回位、停止 |
| [`docs/零位标定.md`](docs/零位标定.md) | 零位测量与补偿 |
| [`docs/仿真使用与测试.md`](docs/仿真使用与测试.md) | 仿真工具链与场景矩阵 |
| [`docs/素材/00-说明.md`](docs/素材/00-说明.md) | 申报书章节映射与素材缺口 |

ROS 工作区的上游来源与本项目补丁见 [`star-arm-moveit2/NOTICE.md`](star-arm-moveit2/NOTICE.md)。
上游仓库未包含 LICENSE，本仓库仅用于课题组内部复现与技术验证。
# SSVEP-Roarm — 脑控机械臂

从臂 Starai Cello + ROS 2 Humble + MoveIt2。与 `../ssvep-bci`（脑控小车）解耦，共用 SSVEP 解码。

## 当前状态（2026-10-19）

| 环节 | 状态 | 证据 |
|---|---|---|
| 仿真全链路 | ✅ | 场景矩阵 16/16（107.5 s）；取放 10 步 ×2（38 s） |
| 任务层 | ✅ | 24 项单元测试；`ros2 run roarm_task task` |
| 真机零点标定 | ✅ | FK 距零位 4.8 mm；`joint_states` 残差 ≤0.6° |
| 真机 MoveIt 单点 | ✅ | 误差 4.92 mm，9.9 s |
| 关节行程实测 | ✅ | 6 轴全测；URDF 限位全线不宽于实测，不用改 |
| 真机完整取放 | ✅ | **连续 13 次全通过**（2026-10-19）；节拍 30~70 s，中位 64.7 s。30 s 档=未触发补偿（落点 4.8 mm），70 s 档=补偿一次（2.5 mm） |
| 意图层（SSVEP→动作） | ✅ | **脑控已驱动机械臂完成示教动作**（2026-10-19 19:19 首次）；`realtime.py --target arm` → `sinks/arm_sink.py` → `task --phase pick/place`。延迟与成功率**未系统测量**，action 层有 `unknown result response` 告警待修 |
| 视觉检测 / 检测框即刺激 | ❌ | 未开始 |

## 文档导航

| 我想… | 去这里 |
|---|---|
| 看全部文档 | [`docs/00-索引.md`](docs/00-索引.md) |
| 写说明书 / 申报书找料 | [`docs/素材/00-说明.md`](docs/素材/00-说明.md)（含章节映射与缺口） |
| 系统怎么搭的 | [`docs/素材/02-系统架构.md`](docs/素材/02-系统架构.md) |
| 某个数字怎么测的 | [`docs/素材/03-实测数据.md`](docs/素材/03-实测数据.md) |
| 某个设计为什么这么定 | [`docs/素材/04-技术决策.md`](docs/素材/04-技术决策.md) |
| 跑仿真 / 跑真机 | [`docs/仿真使用与测试.md`](docs/仿真使用与测试.md)、`bash scripts/robot_up.sh` |
| 标定零点 | [`docs/零位标定.md`](docs/零位标定.md) |
| 接 SSVEP | [`docs/意图层.md`](docs/意图层.md) |

## 硬件

| 项 | 值 |
|---|---|
| 自由度 | 6 + 1（平行夹爪） |
| 臂展 / 全展 | 650 / 1340 mm |
| 重复精度（官方） | 1 mm |
| 负载 | 750 g（70% 臂展内） |
| 舵机 | RX8-U50H-M ×2 · RA8-U25H-M ×4 · RA8-U26H-M ×1 |
| 通信 | UC-01 USB 转总线（CH340），**1 Mbaud**，ID 0~6 |
| 零位末端 | 前方 479.6 mm、高 370.5 mm（URDF 实算，与真机 TF 吻合 0.5 mm） |
| 关节限位（URDF） | j1/j4/j6 ±130°，j2/j3/j5 ±90° |
| **关节行程（实测）** | j1 339° / j6 356° / j4 291° / j5 211° / j2 187° / j3 183° |

> 规格的 1 mm 是**重复定位精度**（同一目标多次）；实测 **3σ = 2.2 mm**（10 次往返）。
> 而**绝对定位精度**是 10~11 mm（含 IK 关节舍入与零点残差）——两个指标，别混。
> URDF 的关节限位**全线不宽于实物**，规划不会撞机械限位；真正有硬限位的只有 j2/j3。

## 舵机内部控制接口

舵机自带梯形速度规划，主机不用跑控制环：

```
DEFAULT_BAUDRATE            = 1_000_000
DEFAULT_MOTION_TIME         = 350 ms
DEFAULT_GRIPPER_MOTION_TIME = 100 ms
DEFAULT_ACC_TIME / DEC_TIME = 50 ms    ← 脑控场景应显式放长
```

`robo_driver` 订阅 `set_angle_topic` → 写舵机；发布 `JointState`。

夹爪换算：`joint7_left = (servo_deg − 100) / 100 × 0.032`。
**卡尺实测**：舵机 100° = 张开 **65 mm**，21.9° = 合到底 **8 mm**（行程 57 mm）。
可用角 21.9°~100°，发 0° 会算出 −0.032 m 超 URDF 下限，把整机状态判为非法、规划全瘫。

## 会咬人的环境约束

1. **仓库路径必须纯 ASCII**。`rosidl_generate_interfaces` 在 `通` 字处截断，`colcon build` 必失败。
   不要用 bind mount / 软链 / `/etc/fstab`——挂载方案曾导致本机无法进系统。
2. **无 X 环境跑 `move_group` 必崩**。`sensors_3d.yaml` 的 `sensors` 必须为 `[]`，
   否则 octomap 的 `mesh_filter` 要开 GLUT 窗口，进程以 exit −11 静默死掉。
3. **编译 C++ 要给 TMPDIR**。本机 `/tmp` 是 10 MB tmpfs，"设备上没有空间"不是磁盘满：
   `TMPDIR=/home/ljl/cctmp colcon build ...`
4. **`robo_driver` 改了必须重建**。`--symlink-install` 对它生成的是 egg-link 指向 `build/`，
   那里是拷贝不是符号链接。

## 架构决定

| 决定 | 理由 |
|---|---|
| 任务级宏动作，不做关节级连续控制 | BCI 带宽不足以支撑连续闭环；人决定"要什么"，机器决定"怎么拿到" |
| 检测框直接作为 SSVEP 刺激 | 省掉眼动的硬件死路；看到什么盯什么，无需学"第几行第几列" |
| 眼动暂缓 | Tobii/Gazepoint/七鑫易维 Linux 全不支持；L2CS-Net 精度有限还要单独标定 |
| 闪烁框位置**冻结** N 秒 | 避免同时面对"检测抖动"和"动态刺激"两个未知量 |
| 任务层调 CLI 子进程，不直接发 ROS | 到位判据只保留一份（TF 实测 + 三档退出码），避免两套判据漂移 |

## 已修 / 未修

**已修（2026-10-17 ~ 19）**

| 问题 | 处理 |
|---|---|
| 夹爪 `close` 发 0° → `joint7_left = −0.032` 超 URDF 下限 | 改 21.9°（正好 −0.025）；超限会让 MoveIt 判整机 invalid、规划全瘫 |
| 夹爪到位容差 5 mm 太松 | 收到 1.5 mm。实测过：目标 48 mm 时夹爪只走到 56.8 mm 就判"到位"，物体 50 mm 根本没碰到 |
| `wait_for_subscribers` 只等 pose 话题 | 改等三条。`set_angle` 的 DDS 发现没完成时，夹爪命令会静默丢失 |
| 任务层容差 5 mm 严于实测精度 | 改 15 mm（home 单独 25 mm）。原来第 1 步回位就必然失败 |
| 绝对定位误差 10~11 mm | 加**二次定位补偿**（目标 = 2×原目标 − 实测），落点压到 2.5 mm |
| 执行异常绕过回位 | `sequence.run` 包 try/except，代码崩了也走"停 + 回位" |
| joint6 限速 13.14→3.14 / 加速度限值打开 | 见 `star-arm-moveit2` 的 commit |
| `sensors_3d` 非空导致 move_group 静默死 | 置为 `[]`；IK 超时 0.005→0.05 s；零位偏移补偿 |

**未修**

| 问题 | 后果 |
|---|---|
| `cello_controller` 不检查 `is_cancel_requested` | **软件停不下来**，急停只能断电。意图层的 STOP 块因此只保证"不再发新指令" |
| `robo_driver` 上电无条件 `reset_multi_turn_angle` + 驱到零位 | 启动瞬间断电 50 ms，臂会掉一点 |
| URDF `velocity="1"`(57°/s) 与 yaml `3.14`(180°/s) 不一致 | 限速依据有两份 |
| 舵机型号表缺 `ra8-u26`，LeRobot 全声明为 `rx8-u50` | 若用型号握手校验，真机可能报错 |
| `task --phase place` 不检查手上有没有东西 | 空按会空放（有意为之：硬判会误伤夹薄片） |

## 目录

```
ssvep-Roarm/
├── scripts/
│   ├── robot_up.sh          真机一键启动（driver → move_group → 执行节点）
│   ├── roarm_cli.py         单条指令：home/pose/grip/grip_deg/list/where
│   ├── go_zero.py           无力矩逐步逼近（--preset park --fast 收纳）
│   ├── zero_calib.py        只读零位标定
│   ├── teach.py             交互示教：摆位 → 记录 → 生成场景 yaml
│   ├── repeatability.py     重复定位精度（双口径：TF / FK）
│   ├── limit_probe.py       逐关节行程实测（--free-all）
│   ├── run_task.sh          取放连跑，日志与成功率自动归档
│   ├── intent_bridge.py     调试点位桥（--status 读 ssvep-bci 状态文件）
│   └── fk.py                URDF 解析 + 正解
├── data/                    实测数据、示教点、场景 yaml
│   ├── scene_real.yaml      当前示教场景（真机取放就读它）
│   ├── runs/summary.csv     每次取放的耗时与结果
│   ├── repeatability.csv    重复精度原始样本
│   └── joint_limits.csv     六轴行程实测
├── docs/                    文档（`00-索引.md` 是入口）
├── star-arm-moveit2/        ROS 2 工作区（源码已并入本仓库，build/install 不入库）
└── 参考资料/                 厂商 SDK、官方示例（不入库）
```

## 快速开始

```bash
# 仿真
bash scripts/sim_up.sh
ros2 run roarm_task task --dry-run

# 真机（先确认工作区无人无障碍）
bash scripts/robot_up.sh
python3 scripts/roarm_cli.py home --wait 30 --tol 25

# 完整取放（连跑 5 次，日志进 data/runs/）
bash scripts/run_task.sh --n 5

# 只取物 / 只放置
ros2 run roarm_task task --scene data/scene_real.yaml --phase pick
ros2 run roarm_task task --scene data/scene_real.yaml --phase place
```

## 下一步

1. **意图层真机联调**：`python3 realtime.py --target arm` → 注视「夹取」→ 注视「放置」，
   测端到端延迟与误判率（桥接与阶段拆分都已就绪，缺真实脑电数据）
2. **视觉检测**：YOLO → 检测框即刺激，去掉"固定路点"这个前提
3. 夹爪口径最后统一：目前三套（舵机角 / `joint7_left` / 开口 mm）靠常量换算，
   等换夹爪或换驱动时一并收拢

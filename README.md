# SSVEP-Roarm — 脑控机械臂

从臂 Starai Cello + ROS 2 Humble + MoveIt2。与 `../ssvep-bci`（脑控小车）解耦，共用 SSVEP 解码。

## 当前状态（2026-10-18）

| 环节 | 状态 | 证据 |
|---|---|---|
| 仿真全链路 | ✅ | 场景矩阵 16/16（107.5 s）；取放 10 步 ×2（38 s） |
| 任务层 | ✅ | 24 项单元测试；`ros2 run roarm_task task` |
| 真机零点标定 | ✅ | FK 距零位 4.8 mm；`joint_states` 残差 ≤0.6° |
| 真机 MoveIt 单点 | ✅ | 误差 4.92 mm，9.9 s |
| 真机完整取放 | ❌ | 只到单点定位 |
| 意图层（SSVEP→动作） | 🟡 | 桥接层就绪，缺解码器 stdout 输出 |
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
| 关节限位 | j1/j4/j6 ±130°，j2/j3/j5 ±90° |

> 规格的 1 mm 是**重复定位精度**（同一目标多次）；实测 4.92 mm 是**绝对定位精度**（含 IK 与零点残差）。两个指标，别混。

## 舵机内部控制接口

舵机自带梯形速度规划，主机不用跑控制环：

```
DEFAULT_BAUDRATE            = 1_000_000
DEFAULT_MOTION_TIME         = 350 ms
DEFAULT_GRIPPER_MOTION_TIME = 100 ms
DEFAULT_ACC_TIME / DEC_TIME = 50 ms    ← 脑控场景应显式放长
```

`robo_driver` 订阅 `set_angle_topic` → 写舵机；发布 `JointState`。

夹爪换算：`joint7_left = (servo_deg − 100) / 100 × 0.032`（0 mm 合拢，−25 mm 全开）。

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

**已修**：joint6 限速 13.14→3.14；加速度限值打开（2.0 rad/s²）；
4 个 launch 的 `MoveItConfigsBuilder` 包名；速度缩放改为从 yaml 读；
`sensors_3d` 置空；IK 超时 0.005→0.05 s；零位偏移补偿。

**未修**：

| 问题 | 后果 |
|---|---|
| `cello_controller` 不检查 `is_cancel_requested` | 软件停不下来，急停只能断电 |
| `robo_driver` 上电无条件 `reset_multi_turn_angle` + 驱到零位 | 启动瞬间断电 50 ms，臂会掉 |
| 夹爪三套角度口径未统一 | `close` 的 0° 对应 −0.032 m，超 URDF 下限 28% |
| URDF `velocity="1"`(57°/s) 与 yaml `3.14`(180°/s) 不一致 | 限速依据有两份 |
| 舵机型号表缺 `ra8-u26`，LeRobot 全部声明为 `rx8-u50` | 若用型号握手校验，真机可能报错 |

## 目录

```
ssvep-Roarm/
├── docs/              文档（00-索引.md 是入口）
├── data/              zero_calib.csv（标定记录）
├── scripts/           robot_up.sh / roarm_cli.py / intent_bridge.py / zero_calib.py / fk.py ...
├── star-arm-moveit2/  ROS 2 工作区（src/ build/ install/ log/）
└── 参考资料/           厂商 SDK、官方示例
```

## 下一步

1. 真机完整取放（`roarm_task` 十步）
2. `ssvep-bci` 加 stdout 输出模式 → 接 `intent_bridge.py`
3. 视觉检测 → 检测框即刺激

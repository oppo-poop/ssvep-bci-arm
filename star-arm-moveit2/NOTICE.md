# 代码出处与许可

## 原始来源

`src/` 下的基础代码来自 Starai Cello 机械臂的 ROS 2 示例仓库：

```
https://github.com/Welt-liu/star-arm-moveit2
```

原始仓库**未包含 LICENSE 文件**。因此本仓库不对外分发，仅用于
课题组内部复现与技术验证。

## 本仓库的改动

相对上游 `29e57cc`，新增与修改如下（详见 `git log`）：

**新增**

- `src/roarm_task/` —— 任务层：场景解析（`scene.py`）、序列编排（`sequence.py`）、
  可复用原语（`primitives.py`）、CLI 桥（`link.py`）、命令行（`cli.py`）、单元测试

**修改**

| 文件 | 改动 | 原因 |
|---|---|---|
| `src/arm_moveit_write/src/arm_moveit_write.cpp` | 夹爪目标角改为实测端点（闭合 86.7° / 张开 174.8°）；去掉"目标未变就跳过"；速度/加速度缩放改为参数 | 旧值 0°/21.9° 会算出超 URDF 下限的 joint7，MoveIt 判整机 invalid；显式参数避免顶掉 joint_limits.yaml |
| `src/robo_driver/robo_driver/robo_driver.py` | 六轴零位偏移；启动 3 s 驱零位或保持；夹爪端点/启动位 121°；写方向补偏移；id6 越界 clamp | 2026-10-18 零位标定、2026-11-02 启动摔臂、2026-11-03 换爪后端点偏移 |
| `src/cello_controller/cello_controller/cello_controller.py` | 整段轨迹合成一条命令；cancel 时保持当前角度；joint7 用实测端点反算；未知关节不误发 | 逐点下发互相覆盖导致抖动/半路停车；换爪后旧夹爪公式会把保持角打错 |
| `src/roarm_task/roarm_task/*` | 容差、二次定位补偿、比例夹爪开度、夹持校验、场景 schema | 按真机实测精度重定；绝对误差是稳定系统偏差，反补一次可压到 2–3 mm |
| `src/cello_description/urdf/cello_description.urdf` | 关节 `velocity="1"` → `3.14` | 与 `joint_limits.yaml` 统一，避免误导 |
| `src/cello_moveit_config/config/joint_limits.yaml` | joint6 限速 13.14 → 3.14；加速度限值打开 | 真机实测 |
| `src/cello_moveit_config/config/sensors_3d.yaml` | `sensors: []` | 无 X 环境时 octomap 的 `mesh_filter` 会开 GLUT 窗口，move_group 静默崩溃 |
| `src/cello_moveit_config/config/kinematics.yaml` | IK 超时 0.005 → 0.05 s | KDL 在零位附近需要更多迭代 |
| `src/cello_moveit_config/launch/arm_moveit_write.launch.py` | 去掉错误参数文件路径；暴露 `velocity_scaling` / `acceleration_scaling` | 启动脚本需要按场景调速 |
| `src/cello_moveit_config/launch/driver.launch.py` | 增加 `zero_on_start` 参数 | 支持"上电完全不动" |
| `src/ros2_bag_recorder/src/bag_recorder.cpp` | joint7 用实测端点反算；去掉 `angle -= 10` | 旧公式录出的夹爪角无法回放 |
| `src/arm_moveit_read/src/arm_moveit_read.cpp` | 夹爪半开判定改按 joint 值（-0.0125 m） | 旧 80° 阈值是换爪前口径 |

> 夹爪端点、零位偏移等物理口径的唯一真值在仓库根 `config/robot.json`；
> 本工作区内的常量由 `scripts/checks/grip_check.py` 强制对齐。改完必须重新 build。

## 使用方

- `ssvep-Roarm`（脑控机械臂主项目）通过 `ros2 run roarm_task task` 调用本包
- 上游更新可用 `git fetch upstream` 拉取，不会与本仓库的提交冲突

## 如果将来要开源

需要先确认上游的授权条款，或把本仓库的改动整理成 patch 单独发布。

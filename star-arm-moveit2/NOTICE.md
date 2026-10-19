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
| `src/arm_moveit_write/src/arm_moveit_write.cpp` | 夹爪目标角 0° → 21.9°；去掉"目标未变就跳过" | 0° 算出 `joint7_left = -0.032`，超出 URDF 下限 28%，会让 MoveIt 判整机 invalid、OMPL 拒绝一切规划 |
| `src/roarm_task/roarm_task/config.py` | 容差、夹爪口径、重试策略 | 按真机实测重定（原值严于/松于实际能力） |
| `src/roarm_task/roarm_task/primitives.py` | 加二次定位补偿、比例夹爪开度 | 绝对误差 10~11 mm 是稳定系统偏差，反补一次可降到 2.5 mm |

## 使用方

- `ssvep-Roarm`（脑控机械臂主项目）通过 `ros2 run roarm_task task` 调用本包
- 上游更新可用 `git fetch upstream` 拉取，不会与本仓库的提交冲突

## 如果将来要开源

需要先确认上游的授权条款，或把本仓库的改动整理成 patch 单独发布。

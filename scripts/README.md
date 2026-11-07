# scripts/ 入口索引

> 旧命令仍然可用：`scripts/` 根下保留了一组 `compat shim`，只是转发到新位置。
> 新代码请用下面的新路径。

## 一、日常入口（不需要移动）

| 脚本 | 用途 | 是否会动机器 |
|---|---|---|
| `roarm_cli.py` | 单条控臂命令 + TF 到位校验（唯一契约） | ✅ |
| `teach.py` | 交互示教：摆位 → 记录 → 生成场景 yaml | ✅ |
| `servo_check.py` | 7 舵机串口/供电/在线检查（只读） | ❌ |
| `sim_gripper_bridge.py` | 仿真夹爪桥（`sim_up.sh` 自动启动） | ❌ 只动仿真 |
| `intent_bridge.py` | SSVEP 类别调试桥（生产路径在外部 `ssvep-bci`） | ✅ |
| `arm_service.py` | 8 指令机械臂服务（Unix socket，给 ssvep-bci） | ✅ |
| `arm_client.py` | 向 `arm_service` 发一条指令并等结果 | ✅ |
| `robot_up.sh` | 真机一键启动/停止；停止时一条 3 s 命令回收纳位并解锁力矩（`--no-park` 跳过） | ✅ |
| `sim_up.sh` | 仿真一键起停 | ❌ 只动仿真 |
| `run_task.sh` | 取放连跑 N 次，日志进 `data/runs/` | ✅ |
| `install_moveit.sh` / `fix_moveit_install.sh` | 环境安装/修复 | ❌ |
| `clean_git_privacy.sh` | 清理同级 `ssvep-bci` 的隐私信息（维护用） | ❌ |

## 二、标定 / 测量（`calib/`）

| 脚本 | 用途 |
|---|---|
| `calib/zero_calib.py` | 只读零位标定，读舵机角度并记录到 `data/zero_calib.csv` |
| `calib/go_zero.py` | 无力矩逐步逼近零位/收纳位（`--go` 才会动） |
| `calib/limit_probe.py` | 逐关节行程实测，写 `data/joint_limits.csv` |
| `calib/repeatability.py` | 重复定位精度，写 `data/repeatability.csv` |

旧路径 `scripts/zero_calib.py` 等仍可用（compat shim）。

## 三、离线自检（`checks/`）

| 脚本 | 检查什么 | 需要硬件 |
|---|---|---|
| `checks/grip_check.py` | 夹爪端点/启动位/零位/公式是否与 `config/robot.json` 一致 | ❌ |
| `checks/pick_check.py` | 视觉边界、落点补偿、参数单源 | ❌ |
| `checks/traj_check.py` | 轨迹单命令、取消保持、joint7 端点换算 | ❌ |
| `checks/sim_test.py` | 仿真场景矩阵（需要 ROS/仿真） | ❌ 只动仿真 |
| `checks/timing_report.py` | 汇总 `data/vision/timing.jsonl` 的阶段耗时 | ❌ |

一键：

```bash
bash scripts/check_all.sh
```

旧路径 `scripts/grip_check.py` 等仍可用（compat shim）。

## 四、公共库（`roarm_lib/`）

| 模块 | 内容 |
|---|---|
| `roarm_lib/params.py` | 加载 `config/*.json`，提供 servo↔joint、开度、比例换算 |
| `roarm_lib/paths.py` | 仓库根/工作区/数据/URDF 的唯一路径来源 |
| `roarm_lib/fk.py` | URDF 解析 + 正运动学（原 `scripts/fk.py`） |

## 五、兼容入口

以下旧路径保留为薄包装（`runpy`，行为/argv/stdin/退出码不变）：

```
scripts/zero_calib.py      → scripts/calib/zero_calib.py
scripts/go_zero.py         → scripts/calib/go_zero.py
scripts/limit_probe.py     → scripts/calib/limit_probe.py
scripts/repeatability.py   → scripts/calib/repeatability.py
scripts/fk.py              → scripts/roarm_lib/fk.py
scripts/grip_check.py      → scripts/checks/grip_check.py
scripts/pick_check.py      → scripts/checks/pick_check.py
scripts/traj_check.py      → scripts/checks/traj_check.py
scripts/sim_test.py        → scripts/checks/sim_test.py
```

新增脚本不要再往根目录堆；按 `ARCHITECTURE.md` 的分层放。
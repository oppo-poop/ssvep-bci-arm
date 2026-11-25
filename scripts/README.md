# scripts/ —— 外部兼容入口（薄 shim）

目录重构（阶段 1）完成后，所有实现已迁移至 `app/`、`tools/`、`tests/`：

- 业务运行时：`app/{core,perception,control,service}/`
- 启停 / 标定 / 诊断工具：`tools/{bringup,calib,diag,setup}/`
- 离线回归测试：`tests/check_all.sh`

本目录仅保留 3 个面向同级仓库 `ssvep-bci` 的薄转发入口，保证外部契约零破坏：

| 兼容入口 | 实际实现 |
|---|---|
| `scripts/arm_service.py` | `app/service/arm_service.py` |
| `scripts/arm_client.py` | `app/service/arm_client.py` |
| `scripts/roarm_cli.py` | `app/control/roarm_cli.py` |

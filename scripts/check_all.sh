#!/usr/bin/env bash
# 一键离线检查：不启动 ROS、不连相机、不动机械臂。
#
#   bash scripts/check_all.sh
#
# 真机项（夹爪端点、取消、完整取放）不在本脚本里，见 README.md 的验证清单。
#
# 顺序说明：数据布局放在第 1 项 —— 2026-11-08 踩过"数据被挪到 data/ 根目录，
# check_all 全绿，pick.py 一启动就 FileNotFoundError"。缺数据是比语法错误
# 更早发生的失败，应该最先报出来。
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

echo "== 1/8 运行数据布局 =="
python3 scripts/checks/data_check.py

echo "== 2/8 编译检查 =="
python3 -m compileall -q \
  scripts \
  star-arm-moveit2/src/roarm_task/roarm_task \
  star-arm-moveit2/src/cello_controller/cello_controller \
  star-arm-moveit2/src/robo_driver/robo_driver

echo "== 3/8 参数派生自测 =="
python3 scripts/roarm_lib/params.py

echo "== 4/8 任务层单元测试 =="
(cd star-arm-moveit2/src/roarm_task && python3 -m unittest discover -s test)

echo "== 5/8 夹爪口径 =="
python3 scripts/checks/grip_check.py
python3 scripts/checks/grip_math_check.py

echo "== 6/8 视觉 / 轨迹回归 =="
python3 scripts/checks/pick_check.py
python3 scripts/checks/traj_check.py

echo "== 7/8 8 指令服务 =="
python3 scripts/checks/arm_service_check.py

echo "== 8/8 断连自停 =="
python3 scripts/checks/check_disconnect.py

echo
echo "✓ 全部离线检查通过。"

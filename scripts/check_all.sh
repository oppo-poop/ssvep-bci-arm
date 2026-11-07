#!/usr/bin/env bash
# 一键离线检查：不启动 ROS、不连相机、不动机械臂。
#
#   bash scripts/check_all.sh
#
# 真机项（夹爪端点、取消、完整取放）不在本脚本里，见 README.md 的验证清单。
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

echo "== 1/6 编译检查 =="
python3 -m compileall -q \
  scripts \
  star-arm-moveit2/src/roarm_task/roarm_task \
  star-arm-moveit2/src/cello_controller/cello_controller \
  star-arm-moveit2/src/robo_driver/robo_driver

echo "== 2/6 参数派生自测 =="
python3 scripts/roarm_lib/params.py

echo "== 3/6 任务层单元测试 =="
(cd star-arm-moveit2/src/roarm_task && python3 -m unittest discover -s test)

echo "== 4/6 夹爪口径 =="
python3 scripts/checks/grip_check.py

echo "== 5/6 视觉 / 轨迹回归 =="
python3 scripts/checks/pick_check.py
python3 scripts/checks/traj_check.py

echo "== 6/6 8 指令服务 =="
python3 scripts/checks/arm_service_check.py

echo
echo "✓ 全部离线检查通过。"
#!/usr/bin/env bash
# 一键离线检查：不启动 ROS、不连相机、不动机械臂。
#   bash tests/check_all.sh
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

echo "== 1/8 配置与标定数据布局 =="
python3 tests/data_check.py

echo "== 2/8 编译检查 =="
python3 -m compileall -q   app   tools   tests   scripts   star-arm-moveit2/src/roarm_task/roarm_task   star-arm-moveit2/src/cello_controller/cello_controller   star-arm-moveit2/src/robo_driver/robo_driver

echo "== 3/8 参数派生自测 =="
python3 app/core/params.py

echo "== 4/8 任务层单元测试 =="
(cd star-arm-moveit2/src/roarm_task && python3 -m unittest discover -s test)

echo "== 5/8 夹爪口径 =="
python3 tests/grip_check.py
python3 tests/grip_math_check.py

echo "== 6/8 视觉 / 轨迹回归 =="
python3 tests/pick_check.py
python3 tests/traj_check.py

echo "== 7/8 8 指令服务 =="
python3 tests/arm_service_check.py

echo "== 8/8 断连自停 =="
python3 tests/check_disconnect.py

echo
echo "✓ 全部 8 项离线检查通过。"

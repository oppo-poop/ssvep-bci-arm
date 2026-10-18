#!/usr/bin/env bash
# sim_up.sh —— 一键起/停 Cello 仿真（幂等，可无头）
#
# 用法：
#   bash scripts/sim_up.sh              # 无头启动（不需要 X）
#   bash scripts/sim_up.sh --rviz       # 带 RViz（需要可用 DISPLAY）
#   bash scripts/sim_up.sh --stop       # 停掉仿真
#   bash scripts/sim_up.sh --status     # 只看状态
#
# 做的事：
#   1. 清掉上一次的残留（幂等，重复执行不会起两份）
#   2. demo.launch.py（rsp + ros2_control fake + 三个控制器 + move_group）
#   3. arm_moveit_write.launch.py（接 position_orientation_topic 的执行节点）
#   4. 等到真的能规划（不是只等进程起来）
#
# 为什么必须这样起：
#   - arm_moveit_write.launch.py 只启动节点本身，不含 move_group
#   - 本机没有 X 服务，demo 必须 use_rviz:=false；带 rviz 会因 GLX 失败而死
#   - move_group 在无显示下需要 sensors_3d.yaml 的 sensors 为空（已改，见该文件注释）

set -uo pipefail

WS="${WS:-/home/ljl/BCI/DeepBCI-8ch/ssvep-Roarm/star-arm-moveit2}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="$WS/.sim.pids"
DEMOLOG="$WS/.sim_demo.log"
NODELOG="$WS/.sim_node.log"
BRIDGELOG="$WS/.sim_bridge.log"
READY_TIMEOUT="${READY_TIMEOUT:-40}"

USE_RVIZ=0
ACTION="up"
for a in "$@"; do
  case "$a" in
    --rviz) USE_RVIZ=1 ;;
    --stop) ACTION="stop" ;;
    --status) ACTION="status" ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "未知参数：$a"; exit 2 ;;
  esac
done

msg()  { printf '%s\n' "$*"; }
ok()   { printf '  ✅ %s\n' "$*"; }
bad()  { printf '  ❌ %s\n' "$*"; }

alive_pids() {
  [ -f "$PIDFILE" ] || return 0
  local p
  while read -r p _; do
    [ -n "$p" ] && kill -0 "$p" 2>/dev/null && echo "$p"
  done < "$PIDFILE"
}

# 只杀我们自己 setsid 出来的进程组，不做任何全局 pkill ——
# 全局 pkill 会误杀用户手动启动的 move_group / rsp。
stop_all() {
  [ -f "$PIDFILE" ] || return 0
  local pid pgid
  while read -r pid pgid; do
    [ -z "${pgid:-}" ] && pgid=$pid
    case "$pgid" in ''|*[!0-9]*) continue ;; esac
    [ "$pgid" -le 1 ] && continue
    kill -TERM -"$pgid" 2>/dev/null
  done < "$PIDFILE"
  sleep 3
  while read -r pid pgid; do
    [ -z "${pgid:-}" ] && pgid=$pid
    case "$pgid" in ''|*[!0-9]*) continue ;; esac
    [ "$pgid" -le 1 ] && continue
    kill -KILL -"$pgid" 2>/dev/null
  done < "$PIDFILE"
  rm -f "$PIDFILE"
}

status() {
  msg "=== 进程 ==="
  local any=0 pattern label pids
  for entry in \
      "moveit_ros_move_group/move_group|move_group" \
      "ros2_control_node|ros2_control_node" \
      "robot_state_publisher|robot_state_publisher" \
      "arm_moveit_write|arm_moveit_write" \
      "sim_gripper_bridge|sim_gripper_bridge"; do
    pattern="${entry%%|*}"
    label="${entry##*|}"
    pids=$(pgrep -f "$pattern" 2>/dev/null | tr '\n' ' ')
    if [ -n "$pids" ]; then
      printf '  %-22s 运行中 (pid %s)\n' "$label" "$pids"
      any=1
    else
      printf '  %-22s -\n' "$label"
    fi
  done
  [ "$any" -eq 0 ] && msg "  （没有任何仿真进程）"
}

if [ "$ACTION" = "stop" ]; then
  msg "停止仿真…"
  stop_all
  ok "已停止"
  exit 0
fi
if [ "$ACTION" = "status" ]; then
  status
  exit 0
fi

# ---------------------------------------------------------------- 启动
# 注意：ROS 的 setup.bash 里用了未绑定变量（如 AMENT_TRACE_SETUP_FILES），
# 在 `set -u` 下 source 会直接报错退出，所以 source 期间必须临时关掉 nounset。
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

msg "=== 1) 清理残留 ==="
stop_all
ok "干净"

RVIZ_ARG="use_rviz:=false"
ENV_PREFIX="env -u DISPLAY"
if [ "$USE_RVIZ" -eq 1 ]; then
  if [ -z "${DISPLAY:-}" ] || [ ! -d /tmp/.X11-unix ]; then
    bad "没有可用的 X 服务，无法带 RViz。改用无头模式。"
    USE_RVIZ=0
  else
    RVIZ_ARG="use_rviz:=true"
    ENV_PREFIX="env"
    ok "RViz 模式"
  fi
fi
[ "$USE_RVIZ" -eq 0 ] && ok "无头模式（不需要 X）"

msg "=== 2) 启动 demo（rsp + fake 控制器 + move_group）==="
cd "$WS" || exit 1
rm -f "$DEMOLOG" "$NODELOG"
# shellcheck disable=SC2086
setsid $ENV_PREFIX ros2 launch cello_moveit_config demo.launch.py "$RVIZ_ARG" > "$DEMOLOG" 2>&1 &
DEMO_PID=$!
DEMO_PGID=$(ps -o pgid= -p "$DEMO_PID" 2>/dev/null | tr -d ' ')

for i in $(seq 1 "$READY_TIMEOUT"); do
  if grep -aq "You can start planning" "$DEMOLOG" 2>/dev/null; then
    ok "move_group 就绪（${i}s）"
    break
  fi
  if ! kill -0 "$DEMO_PID" 2>/dev/null; then
    bad "demo 启动失败，日志尾部："
    tail -20 "$DEMOLOG"
    exit 1
  fi
  sleep 1
done
if ! grep -aq "You can start planning" "$DEMOLOG" 2>/dev/null; then
  bad "等待 ${READY_TIMEOUT}s 仍未就绪，日志尾部："
  tail -20 "$DEMOLOG"
  kill "$DEMO_PID" 2>/dev/null
  exit 1
fi

msg "=== 3) 启动执行节点 arm_moveit_write ==="
# shellcheck disable=SC2086
setsid $ENV_PREFIX ros2 launch cello_moveit_config arm_moveit_write.launch.py > "$NODELOG" 2>&1 &
NODE_PID=$!
NODE_PGID=$(ps -o pgid= -p "$NODE_PID" 2>/dev/null | tr -d ' ')

for i in $(seq 1 20); do
  if grep -aq "Ready to take commands for planning group arm" "$NODELOG" 2>/dev/null; then
    ok "执行节点就绪（${i}s）"
    break
  fi
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    bad "执行节点启动失败，日志尾部："
    tail -20 "$NODELOG"
    kill "$DEMO_PID" 2>/dev/null
    exit 1
  fi
  sleep 1
done

msg "=== 4) 启动仿真夹爪桥（set_angle_topic → hand_controller）==="
setsid $ENV_PREFIX python3 "$SCRIPT_DIR/sim_gripper_bridge.py" > "$BRIDGELOG" 2>&1 &
BRIDGE_PID=$!
BRIDGE_PGID=$(ps -o pgid= -p "$BRIDGE_PID" 2>/dev/null | tr -d ' ')
for i in $(seq 1 15); do
  if grep -aq "SimGripperBridge ready" "$BRIDGELOG" 2>/dev/null; then
    ok "夹爪桥就绪（${i}s）"
    break
  fi
  sleep 1
done
if ! grep -aq "SimGripperBridge ready" "$BRIDGELOG" 2>/dev/null; then
  bad "夹爪桥未就绪（不影响机械臂运动，但 grip 命令不会生效）"
  bad "日志：$(tail -3 "$BRIDGELOG" | tr '\n' ' ')"
fi

printf '%s %s\n%s %s\n%s %s\n' \
  "$DEMO_PID" "${DEMO_PGID:-$DEMO_PID}" \
  "$NODE_PID" "${NODE_PGID:-$NODE_PID}" \
  "$BRIDGE_PID" "${BRIDGE_PGID:-$BRIDGE_PID}" > "$PIDFILE"

msg ""
status
msg ""
msg "就绪。终端里试："
msg "  python3 scripts/roarm_cli.py list"
msg "  python3 scripts/roarm_cli.py home --wait 10"
msg ""
msg "停止：bash scripts/sim_up.sh --stop"

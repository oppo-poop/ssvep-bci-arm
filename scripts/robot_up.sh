#!/usr/bin/env bash
# 真机一键启动：driver + move_group/rviz + 执行节点
#
#   bash scripts/robot_up.sh            # 启动（会先确认）
#   bash scripts/robot_up.sh --status   # 看各节点状态
#   bash scripts/robot_up.sh --stop     # 全部停掉
#
# 与 sim_up.sh 的区别：
#   · 真机不用 sim_gripper_bridge —— robo_driver 直接订阅 set_angle_topic
#   · 不摘 DISPLAY（rviz 要用）
#   · 启动 driver 会让臂回到零位，且中途断电 50ms，**必须先托住**
#
# 日志：$WS/.robot_*.log

set -o pipefail

WS="${WS:-/home/ljl/BCI/DeepBCI-8ch/ssvep-Roarm/star-arm-moveit2}"
PIDFILE="$WS/.robot.pids"
DRVLOG="$WS/.robot_driver.log"
DEMOLOG="$WS/.robot_demo.log"
NODELOG="$WS/.robot_node.log"

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; DIM=$'\033[2m'; RST=$'\033[0m'
msg()  { printf '%s\n' "$*"; }
ok()   { printf '%s✓%s %s\n' "$GRN" "$RST" "$*"; }
warn() { printf '%s!%s %s\n' "$YEL" "$RST" "$*"; }
err()  { printf '%s✗%s %s\n' "$RED" "$RST" "$*" >&2; }

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set +u
}

# 判断某进程组的成员是否还活着
alive() {
  local pgid=$1
  pgrep -g "$pgid" >/dev/null 2>&1
}

wait_for_log() {
  local file=$1 pattern=$2 timeout=$3 name=$4
  local i
  for ((i = 0; i < timeout; i++)); do
    if [ -f "$file" ] && grep -aq "$pattern" "$file" 2>/dev/null; then
      ok "$name 就绪（${i}s）"
      return 0
    fi
    sleep 1
  done
  err "$name 在 ${timeout}s 内没有就绪，看 $file"
  return 1
}

stop_all() {
  [ -f "$PIDFILE" ] || { msg "没有记录在跑的进程"; return 0; }
  local pgid
  while read -r pgid; do
    [ -n "$pgid" ] || continue
    kill -TERM -"$pgid" 2>/dev/null
  done < "$PIDFILE"
  sleep 3
  while read -r pgid; do
    [ -n "$pgid" ] || continue
    alive "$pgid" && kill -KILL -"$pgid" 2>/dev/null
  done < "$PIDFILE"
  rm -f "$PIDFILE"
  ok "已停止"
}

status() {
  msg "=== 节点 ==="
  local any=0
  local pair
  for pair in "move_group:moveit_ros_move_group/move_group" \
              "cello_driver:robo_driver" \
              "cello_controller:cello_controller" \
              "arm_moveit_write:arm_moveit_write" \
              "rviz2:rviz2"; do
    local label=${pair%%:*} pat=${pair#*:}
    if pgrep -f "$pat" >/dev/null 2>&1; then
      printf '  %-20s 运行中\n' "$label"; any=1
    else
      printf '  %-20s -\n' "$label"
    fi
  done
  [ "$any" -eq 0 ] && msg "  （没有任何真机进程）"

  msg ""
  msg "=== joint_states 发布者 ==="
  local n
  n=$(timeout 6 ros2 topic info /joint_states 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
  if [ "${n:-0}" -gt 0 ]; then
    ok "有 $n 个发布者"
  else
    warn "0 个发布者 —— robo_driver 没起来，TF 会算不出 link6"
  fi
}

start_all() {
  msg "${YEL}即将启动真机。driver 会先断电 50ms 再驱到零位 —— 请先托住机械臂。${RST}"
  msg "周围无人、电源开关在手边。"
  printf '继续？[y/N] '
  read -r ans
  case "$ans" in [yY]|[yY][eE][sS]) ;; *) msg "已取消"; exit 1 ;; esac

  source_ros
  : > "$PIDFILE"

  # ---- 1) driver（robo_driver + cello_controller）
  msg "启动 driver…"
  setsid ros2 launch cello_moveit_config driver.launch.py > "$DRVLOG" 2>&1 &
  local drv_pid=$!
  echo "$drv_pid" >> "$PIDFILE"
  wait_for_log "$DRVLOG" "初始化完成\|Initialization complete" 20 "cello_driver" || { stop_all; exit 1; }

  # ---- 2) 等 joint_states 真的有发布者（TF 的前提）
  local i n
  for ((i = 0; i < 15; i++)); do
    n=$(timeout 5 ros2 topic info /joint_states 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
    [ "${n:-0}" -gt 0 ] && break
    sleep 1
  done
  if [ "${n:-0}" -gt 0 ]; then
    ok "joint_states 有发布者（${i}s）"
  else
    err "joint_states 没有发布者，driver 可能没打开串口"
    msg "  看 $DRVLOG"
    stop_all; exit 1
  fi

  # ---- 3) move_group + rviz + rsp
  msg "启动 move_group / rviz…"
  setsid ros2 launch cello_moveit_config actual_robot_demo.launch.py > "$DEMOLOG" 2>&1 &
  local demo_pid=$!
  echo "$demo_pid" >> "$PIDFILE"
  wait_for_log "$DEMOLOG" "You can start planning" 45 "move_group" || { stop_all; exit 1; }

  # ---- 4) 执行节点，速度压到 0.05
  msg "启动执行节点（velocity_scaling=0.05）…"
  setsid ros2 launch cello_moveit_config arm_moveit_write.launch.py \
    velocity_scaling:=0.05 > "$NODELOG" 2>&1 &
  local node_pid=$!
  echo "$node_pid" >> "$PIDFILE"
  wait_for_log "$NODELOG" "ArmMoveitControl ready" 20 "arm_moveit_write" || { stop_all; exit 1; }

  msg ""
  ok "全部就绪。试："
  msg "  ${DIM}python3 scripts/roarm_cli.py where${RST}"
  msg "  ${DIM}python3 scripts/roarm_cli.py pose 0.45 0 0.40 --wait 30${RST}"
  msg ""
  msg "日志：$DRVLOG / $DEMOLOG / $NODELOG"
  msg "停止：bash scripts/robot_up.sh --stop"
}

case "${1:-}" in
  --status) status ;;
  --stop)   source_ros 2>/dev/null; stop_all ;;
  "")       start_all ;;
  *)        msg "用法：$0 [--status|--stop]"; exit 2 ;;
esac

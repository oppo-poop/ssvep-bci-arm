#!/usr/bin/env bash
# 真机一键启动：driver + move_group/rviz + 执行节点
#
#   bash scripts/robot_up.sh            # 启动（会先确认，默认无 RViz）
#   VELOCITY=0.15 ACCELERATION=0.30 bash scripts/robot_up.sh   # 约 1.5 倍速（默认 0.10）
#   RVIZ=true bash scripts/robot_up.sh  # 带 RViz（需要可用的 X/GLX）
#   bash scripts/robot_up.sh --status   # 看各节点状态
#   bash scripts/robot_up.sh --stop     # 停止；随后回 park 位并解锁力矩
#   bash scripts/robot_up.sh --stop --no-park   # 只停节点，不动臂
#   bash scripts/robot_up.sh --stop --yes       # 非交互（脚本里用）
#
# 与 sim_up.sh 的区别：
#   · 真机不用 sim_gripper_bridge —— robo_driver 直接订阅 set_angle_topic
#   · 不摘 DISPLAY（rviz 要用）
#   · 启动 driver 默认用 3 s 驱到零位；ZERO_ON_START=false 则只上力矩
#   · zero_on_start=false 时，回零由本脚本最后一步做：MoveIt 规划 + 慢速
#   · 停止时默认先把节点收掉，再用一条 3 s 命令回收纳位并解锁力矩
#
# 日志：$WS/.robot_*.log

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认用本仓库自带的 star-arm-moveit2；换机器/换工作区时用 WS=... 覆盖
WS="${WS:-$(realpath -m "$SCRIPT_DIR/../star-arm-moveit2")}"
PIDFILE="$WS/.robot.pids"
DRVLOG="$WS/.robot_driver.log"
DEMOLOG="$WS/.robot_demo.log"
NODELOG="$WS/.robot_node.log"

RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; DIM=$'\033[2m'; RST=$'\033[0m'
msg()  { printf '%s\n' "$*"; }
ok()   { printf '%s✓%s %s\n' "$GRN" "$RST" "$*"; }
warn() { printf '%s!%s %s\n' "$YEL" "$RST" "$*"; }
err()  { printf '%s✗%s %s\n' "$RED" "$RST" "$*" >&2; }

# ROS 节点用系统 python；conda 的 python3 没有 rclpy（可用 PYTHON=... 覆盖）
PY="${PYTHON:-/usr/bin/python3}"

# ROS 发行版自动识别：优先当前 shell 的 $ROS_DISTRO，其次找一个装了的
# （覆盖：ROS_SETUP=/path/to/setup.bash）
find_ros_setup() {
  if [ -n "${ROS_SETUP:-}" ]; then printf '%s\n' "$ROS_SETUP"; return 0; fi
  local d
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    printf '%s\n' "/opt/ros/${ROS_DISTRO}/setup.bash"; return 0
  fi
  for d in humble jazzy iron rolling; do
    if [ -f "/opt/ros/${d}/setup.bash" ]; then
      printf '%s\n' "/opt/ros/${d}/setup.bash"; return 0
    fi
  done
  return 1
}

source_ros() {
  local setup
  if ! setup=$(find_ros_setup); then
    err "找不到 ROS 2：/opt/ros/*/setup.bash 都不存在"
    return 1
  fi
  if [ ! -f "$WS/install/setup.bash" ]; then
    err "工作区还没构建：$WS/install/setup.bash 不存在"
    msg "  先构建：cd \"$WS\" && colcon build --symlink-install"
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$setup"
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

stop_nodes() {
  if [ ! -f "$PIDFILE" ]; then
    msg "没有记录在跑的进程"
    return 0
  fi
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
  ok "节点已停止"
}

park_and_unlock() {
  local gz="$SCRIPT_DIR/calib/go_zero.py"
  local park_ms="${PARK_MS:-3000}"
  if [ ! -f "$gz" ]; then
    warn "找不到 $gz，跳过收纳/解锁"
    return 1
  fi
  msg "收纳：一条 ${park_ms} ms 命令驱到 park 位并解除力矩…"
  msg "⚠ 解除力矩后机械臂将失去支撑，请先托住。"
  if $PY "$gz" --preset park --direct --step-ms "$park_ms" --go --unlock --yes; then
    ok "已到收纳位并解除力矩"
    return 0
  fi
  warn "收纳/解锁未完成：机械臂可能仍在力矩保持（看上面的输出）"
  return 1
}

#: --stop 的完整流程：先停节点释放串口，再回 park 并解锁。
#: start_all 的失败路径只调用 stop_nodes，不会额外驱动机械臂。
stop_all() {
  local rc=0
  stop_nodes
  sleep 1                       # 等已退出的 driver 释放串口

  case "${PARK_ON_STOP:-true}" in
    false|0|no)
      msg "PARK_ON_STOP=false：跳过收纳/解锁"
      return 0
      ;;
  esac

  if [ "${STOP_YES:-false}" = "true" ]; then
    park_and_unlock || rc=1
    return $rc
  fi

  if [ -t 0 ]; then
    printf '停止后将把机械臂驱到 park 位并解除力矩。继续？[y/N] '
    read -r ans
    case "$ans" in
      [yY]|[yY][eE][sS]) park_and_unlock || rc=1 ;;
      *) msg "跳过收纳/解锁（舵机仍保持力矩）" ;;
    esac
  else
    warn "非交互模式：跳过收纳/解锁。需要时加 --yes，或手动执行："
    msg "  $PY $SCRIPT_DIR/calib/go_zero.py --preset park --direct --step-ms ${PARK_MS:-3000} --go --unlock --yes"
  fi
  return $rc
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
  msg "${YEL}即将启动真机。${RST}"
  msg "  · driver 启动后会把臂慢速驱到零位（约 3 秒），臂会动"
  msg "  · 臂停在大幅展开的位置时先托住，或改用：ZERO_ON_START=false bash scripts/robot_up.sh"
  msg "周围无人、电源开关在手边。"
  printf '继续？[y/N] '
  read -r ans
  case "$ans" in [yY]|[yY][eE][sS]) ;; *) msg "已取消"; exit 1 ;; esac

  source_ros || exit 1

  if [ ! -e /dev/ttyUSB0 ]; then
    warn "/dev/ttyUSB0 不存在 —— driver 会因串口初始化失败而退出"
  elif [ ! -r /dev/ttyUSB0 ] || [ ! -w /dev/ttyUSB0 ]; then
    err "/dev/ttyUSB0 没有读写权限；把自己加进 dialout 组后重新登录："
    msg "  sudo usermod -aG dialout $USER"
  fi

  : > "$PIDFILE"

  # ---- 1) driver（robo_driver + cello_controller）
  # ZERO_ON_START=true（默认）：驱动在 3 s 内把臂驱到零位 —— 一条命令就到位。
  #   历史：原来 1.5 s，折叠态展开 90° 太猛，2026-11-02 上升到一半失力回落。
  # ZERO_ON_START=false：只上力矩保持原姿态，之后由本脚本用 MoveIt 慢速回零。
  local zero_start="${ZERO_ON_START:-true}"
  msg "启动 driver…（zero_on_start=${zero_start}）"
  setsid ros2 launch cello_moveit_config driver.launch.py \
    zero_on_start:="$zero_start" > "$DRVLOG" 2>&1 &
  local drv_pid=$!
  echo "$drv_pid" >> "$PIDFILE"
  wait_for_log "$DRVLOG" "初始化完成\|Initialization complete" 20 "cello_driver" || { stop_nodes; exit 1; }

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
    stop_nodes; exit 1
  fi

  # 光有 publisher 还不够：driver 可能刚初始化完就因为串口异常退出。
  # 再确认节点还活着、并且真的有一帧数据。
  if pgrep -f "robo_driver/lib/robo_driver/driver" >/dev/null 2>&1 \
     && timeout 8 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    ok "joint_states 有实际数据"
  else
    err "joint_states 没有实际数据，driver 可能已因串口异常退出"
    msg "  看 $DRVLOG"
    stop_nodes; exit 1
  fi

  # ---- 3) move_group + rsp（RViz 默认关闭：本机 GLX 不可用时 rviz2 会崩）
  local rviz="${RVIZ:-false}"
  msg "启动 move_group / rsp（use_rviz=$rviz）…"
  setsid ros2 launch cello_moveit_config actual_robot_demo.launch.py \
    use_rviz:="$rviz" > "$DEMOLOG" 2>&1 &
  local demo_pid=$!
  echo "$demo_pid" >> "$PIDFILE"
  wait_for_log "$DEMOLOG" "You can start planning" 45 "move_group" || { stop_nodes; exit 1; }

  # ---- 4) 执行节点。默认 VELOCITY=0.10（约 6 mm/s，是旧值 0.05 的 2 倍），
  #         与 joint_limits.yaml 的 default_velocity_scaling_factor 一致。
  #         要更稳：VELOCITY=0.05（约 3 mm/s）
  #         示教/粗定位：VELOCITY=0.15 ACCELERATION=0.30（约 9 mm/s）
  #         acceleration 默认 -1.0 = 交给 MoveIt 用 joint_limits.yaml 的值。
  # ⚠ 提速前先确认抖动已修好（cello_controller 已改整段单命令），快了更抖。
  local vel="${VELOCITY:-0.10}"
  local acc="${ACCELERATION:--1.0}"
  msg "启动执行节点（velocity_scaling=$vel, acceleration_scaling=$acc）…"
  setsid ros2 launch cello_moveit_config arm_moveit_write.launch.py \
    velocity_scaling:="$vel" acceleration_scaling:="$acc" > "$NODELOG" 2>&1 &
  local node_pid=$!
  echo "$node_pid" >> "$PIDFILE"
  wait_for_log "$NODELOG" "ArmMoveitControl ready" 20 "arm_moveit_write" || { stop_nodes; exit 1; }

  # ---- 5) 自动回零（一条命令就到位，不用再手敲第二条）
  # 走 MoveIt 规划，速度受上面的 velocity_scaling 控制；
  # 不是驱动里那种"1.5 秒甩到 0°"（那是 2026-11-02 摔臂的原因）。
  # 不想回零：AUTO_HOME=false bash scripts/robot_up.sh
  # 驱动已经把臂驱到零位了，就别再用 MoveIt 回一次（那条路径要 1~2 分钟）。
  # 只有 zero_on_start=false（驱动不动）时，才由这里补一次慢速回零。
  local auto_home="${AUTO_HOME:-$([ "$zero_start" = "true" ] && echo false || echo true)}"
  if [ "$auto_home" = "true" ]; then
    msg "自动回零（MoveIt 规划；AUTO_HOME=false 跳过）…"
    # 180 s 不是随手写的：从折叠态（j2/j3 各差 90°）展开到 home 是大行程，
    # velocity_scaling=0.05~0.10 下要 0.5~2 分钟。原来给 60 s 会在半路报"未到位"，
    # 而 --wait 超时**不会让轨迹停下**，臂还在走 —— 后面紧接着发命令就会打架。
    if $PY "$SCRIPT_DIR/roarm_cli.py" home --wait 180 --tol 25; then
      ok "已回零"
    else
      warn "回零没到位。注意：--wait 超时只表示「不再等」，臂可能还在走。"
      msg "  先确认停在哪，再决定下一步："
      msg "  ${DIM}$PY $SCRIPT_DIR/roarm_cli.py where${RST}"
      msg "  ${DIM}$PY $SCRIPT_DIR/roarm_cli.py home --wait 180 --tol 25${RST}"
      msg "  若反复失败（规划不出来），看 ${NODELOG}，或先无力矩逼近："
      msg "  ${DIM}$PY $SCRIPT_DIR/calib/go_zero.py --preset park${RST}"
    fi
  else
    msg "AUTO_HOME=false：跳过回零，臂停在当前姿态（已上力矩）"
  fi

  msg ""
  msg "当前 shell 先加载工作区："
  msg "  ${DIM}source $WS/install/setup.bash${RST}"
  ok "全部就绪。试："
  msg "  ${DIM}$PY $SCRIPT_DIR/roarm_cli.py where${RST}"
  msg "  ${DIM}$PY $SCRIPT_DIR/roarm_cli.py pose 0.45 0 0.40 --wait 30${RST}"
  msg ""
  msg "日志：$DRVLOG / $DEMOLOG / $NODELOG"
  msg "停止：bash scripts/robot_up.sh --stop"
}

case "${1:-}" in
  --status) source_ros >/dev/null 2>&1 || true; status ;;
  --stop)
    shift
    while [ $# -gt 0 ]; do
      case "$1" in
        --yes|-y)  STOP_YES=true; shift ;;
        --no-park) PARK_ON_STOP=false; shift ;;
        *)         msg "用法：$0 --stop [--yes] [--no-park]"; exit 2 ;;
      esac
    done
    stop_all
    ;;
  "")       start_all ;;
  *)        msg "用法：$0 [--status|--stop [--yes] [--no-park]]"; exit 2 ;;
esac

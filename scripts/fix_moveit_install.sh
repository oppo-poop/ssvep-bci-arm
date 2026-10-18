#!/usr/bin/env bash
# 修复 MoveIt 安装被「上游打包冲突」中断的问题 —— ssvep-Roarm
#
# 用法：  sudo bash scripts/fix_moveit_install.sh
#
# ── 出了什么事 ────────────────────────────────────────────────
#   apt 在 unpack 阶段报：
#     正试图覆盖 /opt/ros/humble/local/lib/python3.10/dist-packages/moveit/__init__.py，
#     它同时被包含于软件包 ros-humble-moveit-py 2.5.10
#
#   这是 ROS 2 Humble 上游的 Debian 打包 bug：
#     · ros-humble-moveit-py              提供 moveit/ 包（__init__.py 内容是
#                                          `from moveit.core import *` 等）
#     · ros-humble-moveit-task-constructor-core  也往同一个 moveit/ 目录写
#                                          __init__.py（pkgutil 命名空间风格）
#   两者都没声明 Replaces/Breaks，dpkg 拒绝覆盖。
#   整个 ros-humble-moveit* 集合里，两边文件交集**只有这 1 个文件**。
#
#   为什么不 `dpkg -i --force-overwrite`：
#     覆盖后 moveit-py 的 __init__.py 会被换成命名空间版本，
#     `import moveit` 不再自动带入 core/planning —— 静默降级，不报错。
#   本项目 0 处引用 task_constructor，反向依赖也只有它自家那几个包，
#   所以正确做法是**把 task-constructor 摘掉**，而不是强行覆盖。
#
# ── 本脚本做什么 ──────────────────────────────────────────────
#   1. 摘掉 task-constructor 系列（5 个，其中 core 本来就没装上）
#   2. 把 70 个「已解包未配置」的包装配完
#   3. 补齐依赖
#   4. 校验

set -uo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行：  sudo bash $0" >&2
  exit 1
fi

TMC="ros-humble-moveit-task-constructor-core
ros-humble-moveit-task-constructor-capabilities
ros-humble-moveit-task-constructor-demo
ros-humble-moveit-task-constructor-visualization
ros-humble-moveit-task-constructor-msgs"

# 独占锁：别和正在跑的 apt / 软件中心抢
if fuser /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock /var/lib/apt/lists/lock >/dev/null 2>&1; then
  echo "检测到其它 apt/dpkg 进程正在运行，请等它结束再跑本脚本。" >&2
  echo "（可直接在终端跑：  ps aux | grep -E 'apt|dpkg' | grep -v grep）" >&2
  exit 1
fi

echo "═══ 1/4  摘掉与 ros-humble-moveit-py 冲突的 task-constructor 系列 ═══"
for p in $TMC; do
  # 注意：不要用 ${db:Status-Abbrev}，它返回 "ii " 带尾随空格，字符串比较会误判
  st=$(dpkg-query -W -f='${db:Status-Status}' "$p" 2>/dev/null || echo "not-installed")
  case "$st" in
    not-installed|config-files|"")
      printf '    %-52s 未安装，无需处理\n' "$p" ;;
    installed)
      printf '    %-52s 已装好 → 移除\n' "$p"
      dpkg --purge --force-depends "$p" >/dev/null 2>&1 || echo "      [警告] 移除失败" ;;
    *)
      printf '    %-52s %s → 移除\n' "$p" "$st"
      dpkg --purge --force-depends "$p" >/dev/null 2>&1 || echo "      [警告] 移除失败" ;;
  esac
done

# 防止将来 apt upgrade 又把这个冲突包拉回来
apt-mark hold ros-humble-moveit-task-constructor-core >/dev/null 2>&1 \
  && echo "    已 apt-mark hold task-constructor-core（防升级时重新引入冲突）" \
  || true

echo
echo "═══ 2/4  配置所有已解包未配置的包 ═══"
dpkg --configure -a
RC_CFG=$?

echo
echo "═══ 3/4  补齐依赖 ═══"
apt-get -f install -y
RC_FIX=$?

echo
echo "═══ 4/4  校验 ═══"
RC=0
KEY="ros-humble-moveit-core
ros-humble-moveit-ros-visualization
ros-humble-moveit-planners-ompl
ros-humble-moveit-simple-controller-manager
ros-humble-moveit-configs-utils
ros-humble-joint-trajectory-controller
ros-humble-gripper-controllers"

for p in $KEY; do
  # 关键：用 ${Status}（"install ok installed"），不要用 ${db:Status-Abbrev}（"ii " 带尾随空格）
  st=$(dpkg-query -W -f='${Status}' "$p" 2>/dev/null || echo "not installed")
  if [ "$st" = "install ok installed" ]; then
    printf '    ✓ %-52s %s\n' "$p" "$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null)"
  else
    printf '    ✗ %-52s 【%s】\n' "$p" "$st"
    RC=1
  fi
done

echo
INIT=/opt/ros/humble/local/lib/python3.10/dist-packages/moveit/__init__.py
if [ -f "$INIT" ] && grep -q "from moveit.core import" "$INIT"; then
  echo "    ✓ moveit/__init__.py 仍是 moveit-py 版本（未被覆盖）"
else
  echo "    ✗ moveit/__init__.py 异常，内容："
  head -5 "$INIT" 2>/dev/null | sed 's/^/        /'
  RC=1
fi

if [ -d /opt/ros/humble/local/lib/python3.10/dist-packages/moveit/task_constructor ]; then
  echo "    ⚠ 残留 moveit/task_constructor 目录（不影响使用，但建议清掉）"
fi

echo
if [ "$RC" -eq 0 ] && [ "$RC_CFG" -eq 0 ]; then
  echo "═══ 修复完成。接下来（不需要 sudo）═══"
  echo "  cd $(dirname "$(dirname "$(readlink -f "$0")")")"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  colcon build --symlink-install"
  echo "  source install/setup.bash"
  echo "  ros2 launch cello_moveit_config demo.launch.py"
else
  echo "═══ 仍有问题（configure 退出码 $RC_CFG，fix 退出码 $RC_FIX）═══" >&2
  echo "请把上面带 ✗ 的行贴回来。" >&2
fi
exit $RC

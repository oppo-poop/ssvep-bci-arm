#!/usr/bin/env bash
# 安装 ROS 2 Humble MoveIt2 —— ssvep-Roarm 项目
#
# 用法：  sudo bash scripts/install_moveit.sh
#
# 为什么不用上游 README 的 `sudo apt install ros-humble-moveit*`：
#   1. 那个通配符会把 33 个 -dbgsym 调试符号包一起拉进来，
#      安装体积从 131 MB 涨到 659 MB，全是无用的调试符号；
#   2. 反过来，只装 metapackage `ros-humble-moveit` 又不够 ——
#      它不依赖 joint_trajectory_controller / gripper_controllers，
#      而 cello_moveit_config/config/ros2_controllers.yaml 正需要这两类控制器
#      （arm_controller = JointTrajectoryController，hand_controller = GripperActionController），
#      漏了它们 demo.launch.py 起不来；
#   3. 更麻烦的是，通配符里还包含 ros-humble-moveit-task-constructor-core，
#      它和 ros-humble-moveit-py 争抢同一个文件
#        /opt/ros/humble/local/lib/python3.10/dist-packages/moveit/__init__.py
#      两者都没声明 Replaces/Breaks → dpkg 在 unpack 阶段直接失败（上游打包 bug）。
#      本项目 0 处引用 task_constructor，所以直接排除掉，不要用 --force-overwrite
#      （覆盖后 `import moveit` 会静默失去 core/planning）。
#
# 所以正确做法是：同范围 + 排除 dbgsym + 排除 task-constructor。

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行：  sudo bash $0" >&2
  exit 1
fi

ALL=$(apt-cache search --names-only '^ros-humble-moveit' | awk '{print $1}' | grep -v -- '-dbgsym$' | grep -v -- '^ros-humble-moveit-task-constructor-')
NDBG=$(apt-cache search --names-only '^ros-humble-moveit' | awk '{print $1}' | grep -c -- '-dbgsym$' || true)
NTMC=$(apt-cache search --names-only '^ros-humble-moveit' | awk '{print $1}' | grep -c -- '^ros-humble-moveit-task-constructor-' || true)

# 这两个控制器包名不以 moveit 开头，不在上面的模式里。它们只被 moveit-servo 传递带入 ——
# 但 cello_moveit_config/config/ros2_controllers.yaml 明确需要它们
# （arm_controller = JointTrajectoryController，hand_controller = GripperActionController），
# 所以显式列出，不依赖传递关系。
ALL="$ALL
ros-humble-joint-trajectory-controller
ros-humble-gripper-controllers"

echo "==> 1/3  解析包清单"
echo "    moveit 相关包 $(echo "$ALL" | wc -l) 个"
echo "      · 已排除 $NDBG 个 dbgsym 调试符号包"
echo "      · 已排除 $NTMC 个 task-constructor 包（与 moveit-py 存在文件冲突）"
echo "    其中关键的控制器包："
for p in ros-humble-joint-trajectory-controller ros-humble-gripper-controllers; do
  dpkg -l "$p" 2>/dev/null | grep -q '^ii' && echo "      · $p  已装" || echo "      · $p  ← 本次要装"
done

echo "==> 2/3  更新源并安装"
apt-get -q update || echo "    [警告] 源更新失败，用现有缓存继续"
# shellcheck disable=SC2086
apt-get install -y $ALL

echo "==> 3/3  校验关键组件"
RC=0
for p in ros-humble-moveit-core \
         ros-humble-moveit-ros-visualization \
         ros-humble-moveit-planners-ompl \
         ros-humble-moveit-simple-controller-manager \
         ros-humble-moveit-configs-utils \
         ros-humble-joint-trajectory-controller \
         ros-humble-gripper-controllers ; do
  # 关键：用 ${Status}（"install ok installed"），不要用 ${db:Status-Abbrev}
  # —— 后者返回 "ii " 带尾随空格，字符串比较会误判成失败
  st=$(dpkg-query -W -f='${Status}' "$p" 2>/dev/null || echo "not installed")
  if [ "$st" = "install ok installed" ]; then
    printf '    ✓ %-45s %s\n' "$p" "$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null)"
  else
    printf '    ✗ %-45s 【%s】\n' "$p" "$st"
    RC=1
  fi
done

if [ "$RC" -eq 0 ]; then
  echo
  echo "全部就绪。接下来（不需要 sudo）："
  echo "  cd $(dirname "$(dirname "$(readlink -f "$0")")")"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  colcon build --symlink-install"
  echo "  source install/setup.bash"
  echo "  ros2 launch cello_moveit_config demo.launch.py"
else
  echo
  echo "有组件缺失，请把上面的输出贴回来。" >&2
fi
exit $RC

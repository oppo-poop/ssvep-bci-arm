#!/usr/bin/env python3
"""roarm_cli —— 终端指令控制 Cello 机械臂（带到位校验）

和仿真、真机走同一套话题接口（不依赖 RViz）：
    position_orientation_topic  (robo_interfaces/PositionOrientation)  位姿目标
    gripper_command_topic       (robo_interfaces/GripperCommand)       夹爪 open/close

执行端由 arm_moveit_write 节点负责（MoveGroupInterface："arm" 组）。

输入输出契约（重要）：
    * 输入：命名路点 / 位姿（世界系，米）/ 夹爪命令
    * 输出：--wait 时**用 TF 实测末端位置并对比目标**，打印误差与耗时
    * 退出码：0 = 到位；1 = 未到位/超时；2 = 输入被拒绝（越界、非法四元数等）

用法：
    source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash

    python3 scripts/roarm_cli.py list
    python3 scripts/roarm_cli.py home --wait 15
    python3 scripts/roarm_cli.py pose 0.40 0.00 0.40 --wait 15
    python3 scripts/roarm_cli.py pose 0.40 0 0.40 0.5 -0.5 0.5 -0.5 --tol 3
    python3 scripts/roarm_cli.py grip open
    python3 scripts/roarm_cli.py            # 交互模式

注意：
    * 默认姿态四元数 (0.5, -0.5, 0.5, -0.5) 是零位实测的末端姿态（world->link6，RPY −90/0/−90）
    * home 是**笛卡尔点**不是关节零位：IK 会解出 j4≈5.9°/j6≈−5.9°
    * 仿真里夹爪要能动，必须先起 sim_gripper_bridge（sim_up.sh 会自动起）。
      ⚠ 实测 open/close 与物理开合方向相反，见 GRIP_EXPECT 上方注释
"""
import argparse
import math
import shlex
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from robo_interfaces.msg import PositionOrientation, GripperCommand
from sensor_msgs.msg import JointState

# 工程 README 记录的 4 个 Cello 路点（位置，米，世界系）
WAYPOINTS = {
    "home":  (0.479, 0.000, 0.369),
    "start": (0.330, -0.324, 0.074),
    "right": (0.529, 0.113, 0.246),
    "up":    (0.278, 0.000, 0.438),
}

# 零位末端姿态（2026-10-17 实测 world->link6）
DEFAULT_QUAT = (0.5, -0.5, 0.5, -0.5)

# 任务工作区（README §七）。越界默认拒绝，--force 可越过。
BOUNDS = {"x": (0.278, 0.529), "y": (-0.324, 0.113), "z": (0.074, 0.438)}

# 夹爪：arm_moveit_write 发的是舵机角度（open=100°, close=0°），
# sim_gripper_bridge 再换算成关节位置（米）。2026-10-17 实测对应关系：
#   100° -> joint7_left =  0.000 m   （URDF 里两指合拢，间距 0 mm）
#     0° -> joint7_left = -0.025 m   （限位下限，两指张开 50 mm）
# ⚠ 即命令名与物理开合方向相反。这里按“当前实现的实测值”校验，不改语义，
#   口径待定，见 docs/仿真使用与测试.md §六。
GRIP_EXPECT = {"open": 0.0, "close": -0.025}

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
BASE_FRAME = "world"
EE_FRAME = "link6"

EXIT_OK, EXIT_FAIL, EXIT_REJECT = 0, 1, 2


class RoArmCli(Node):
    def __init__(self):
        super().__init__("roarm_cli")
        self.pose_pub = self.create_publisher(PositionOrientation, "position_orientation_topic", 10)
        self.grip_pub = self.create_publisher(GripperCommand, "gripper_command_topic", 10)
        self.joint_positions = None
        self.gripper_pos = None
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            self.joint_positions = [pos[n] for n in JOINT_NAMES]
        if "joint7_left" in pos:
            self.gripper_pos = pos["joint7_left"]

    # ---------- 通信 ----------
    def wait_for_subscribers(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose_pub.get_subscription_count() > 0:
                return True
        return False

    def send_pose(self, xyz, quat):
        msg = PositionOrientation()
        msg.position_x, msg.position_y, msg.position_z = xyz
        msg.orientation_x, msg.orientation_y, msg.orientation_z, msg.orientation_w = quat
        self.pose_pub.publish(msg)

    def send_grip(self, cmd):
        msg = GripperCommand()
        msg.command = cmd
        self.grip_pub.publish(msg)

    # ---------- 观测 ----------
    def ee_position(self):
        """世界系末端位置；TF 还没就绪时返回 None。"""
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, Time())
        except Exception:
            return None
        p = t.transform.translation
        return (p.x, p.y, p.z)

    def ee_pose(self):
        """世界系末端位姿 (x, y, z, qx, qy, qz, qw)；TF 未就绪返回 None。"""
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, Time())
        except Exception:
            return None
        p, r = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z, r.x, r.y, r.z, r.w)

    def joint_angles_deg(self):
        if self.joint_positions is None:
            return None
        return [p * 180.0 / math.pi for p in self.joint_positions]

    # ---------- 到位校验 ----------
    def wait_for_gripper(self, target, timeout, tol_mm, settle=3):
        """等 joint7_left 进入容差并稳定 settle 次。返回 (ok, err_mm, elapsed)。"""
        t0 = time.time()
        hit = 0
        err = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gripper_pos is not None:
                err = abs(self.gripper_pos - target) * 1000.0
                if err <= tol_mm:
                    hit += 1
                    if hit >= settle:
                        return True, err, time.time() - t0
                else:
                    hit = 0
        return False, err, time.time() - t0

    def wait_until_reached(self, target, timeout, tol_mm, settle=3):
        """等末端进入目标容差并稳定 settle 次。返回 (ok, err_mm, elapsed, final_pos)。"""
        t0 = time.time()
        hit = 0
        err_mm = None
        pos = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self.ee_position()
            if pos is not None:
                err_mm = math.dist(pos, target) * 1000.0
                if err_mm <= tol_mm:
                    hit += 1
                    if hit >= settle:
                        return True, err_mm, time.time() - t0, pos
                else:
                    hit = 0
        return False, err_mm, time.time() - t0, pos


def in_bounds(xyz):
    bad = []
    for name, v in zip(("x", "y", "z"), xyz):
        lo, hi = BOUNDS[name]
        if not (lo <= v <= hi):
            bad.append("%s=%.3f 不在 [%.3f, %.3f]" % (name, v, lo, hi))
    return bad


def parse_quat(args):
    if not args:
        return DEFAULT_QUAT
    if len(args) != 4:
        raise ValueError("四元数要 4 个数：qx qy qz qw")
    q = tuple(float(a) for a in args)
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-6:
        raise ValueError("四元数范数接近 0（全零），非法")
    return tuple(v / n for v in q)


def cmd_list():
    print("命名路点（位置，米，世界系）：")
    for name, (x, y, z) in WAYPOINTS.items():
        print("  %-6s x=%.3f  y=%+.3f  z=%.3f" % (name, x, y, z))
    print("\n任务工作区：x[%.3f, %.3f]  y[%.3f, %.3f]  z[%.3f, %.3f]"
          % (BOUNDS["x"] + BOUNDS["y"] + BOUNDS["z"]))
    print("默认姿态四元数：", DEFAULT_QUAT)
    print("校验坐标系：%s -> %s" % (BASE_FRAME, EE_FRAME))


def report(node, xyz, quat, timeout, tol_mm):
    """发目标 + 校验到位，返回退出码。"""
    node.send_pose(xyz, quat)
    print("→ 目标 x=%.3f y=%.3f z=%.3f  q=(%.3f, %.3f, %.3f, %.3f)"
          % (*xyz, *quat))

    if timeout <= 0:
        return EXIT_OK

    ok, err_mm, elapsed, pos = node.wait_until_reached(xyz, timeout, tol_mm)
    ang = node.joint_angles_deg()
    if ang:
        print("  关节角(度): " + "  ".join("j%d=%7.2f" % (i + 1, d) for i, d in enumerate(ang)))
    if ok:
        print("  ✅ 到位  误差 %.2f mm  用时 %.1f s" % (err_mm, elapsed))
        return EXIT_OK
    if err_mm is None:
        print("  ❌ 未拿到末端位姿（TF %s->%s 不可用），无法校验" % (BASE_FRAME, EE_FRAME))
    else:
        print("  ❌ 未到位  误差 %.2f mm（容忍 %.1f）用时 %.1f s"
              % (err_mm, tol_mm, elapsed))
        if pos:
            print("     实际 x=%.3f y=%.3f z=%.3f" % pos)
    return EXIT_FAIL


def handle(node, argv, force, timeout, tol_mm):
    """执行一条指令，返回退出码（None 表示继续交互）。"""
    if not argv:
        return None
    op = argv[0].lower()

    if op in ("quit", "exit", "q"):
        return "quit"

    if op == "list":
        cmd_list()
        return None

    if op == "where":
        # 需要同时拿到 TF 和 joint_states。新起的进程要重新建立 DDS 连接，
        # 固定 spin 1 秒经常不够 → 用「两者都到齐就退出，最多等 6 秒」。
        pose = None
        deadline = time.time() + 6.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            pose = node.ee_pose()
            if pose is not None and node.gripper_pos is not None:
                break
        if pose is None:
            print("❌ 拿不到 %s->%s，TF 不可用（执行节点没起？）" % (BASE_FRAME, EE_FRAME))
            return EXIT_FAIL
        print("当前位置 x=%.3f y=%.3f z=%.3f" % pose[:3])
        print("            q=(%.3f, %.3f, %.3f, %.3f)" % pose[3:])
        if node.gripper_pos is not None:
            print("夹爪 joint7_left=%.4f m" % node.gripper_pos)
        ang = node.joint_angles_deg()
        if ang:
            print("  关节角(度): " + "  ".join("j%d=%7.2f" % (i + 1, d)
                                               for i, d in enumerate(ang)))
        return EXIT_OK

    if op in ("grip", "gripper"):
        if len(argv) < 2 or argv[1] not in GRIP_EXPECT:
            print("用法：grip open|close")
            return EXIT_REJECT
        cmd = argv[1]
        node.send_grip(cmd)
        target = GRIP_EXPECT[cmd]
        print("→ 夹爪 %s（目标 joint7_left = %.3f m）" % (cmd, target))
        if timeout <= 0:
            return EXIT_OK
        ok, err, elapsed = node.wait_for_gripper(target, timeout, tol_mm)
        if ok:
            print("  ✅ 到位  误差 %.2f mm  用时 %.1f s" % (err, elapsed))
            return EXIT_OK
        if err is None:
            print("  ❌ 没收到 joint7_left（仿真里需要 sim_gripper_bridge 在跑）")
        else:
            print("  ❌ 未到位  误差 %.2f mm（容忍 %.1f）" % (err, tol_mm))
        return EXIT_FAIL

    if op in WAYPOINTS:
        xyz, quat = WAYPOINTS[op], DEFAULT_QUAT
    elif op == "pose":
        if len(argv) < 4:
            print("用法：pose x y z [qx qy qz qw]")
            return EXIT_REJECT
        try:
            xyz = tuple(float(a) for a in argv[1:4])
            quat = parse_quat(argv[4:])
        except ValueError as e:
            print("拒绝：%s" % e)
            return EXIT_REJECT
    else:
        print("未知指令：%s（list 看可用指令）" % op)
        return EXIT_REJECT

    bad = in_bounds(xyz)
    if bad and not force:
        print("拒绝：目标超出任务工作区")
        for b in bad:
            print("  " + b)
        print("（确实要发就加 --force）")
        return EXIT_REJECT

    return report(node, xyz, quat, timeout, tol_mm)


def main():
    ap = argparse.ArgumentParser(description="终端指令控制 Cello 机械臂（带到位校验）")
    ap.add_argument("command", nargs="*", help="指令（不给则进交互模式）")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="等待到达的最长秒数；给了就用 TF 校验并返回退出码")
    ap.add_argument("--tol", type=float, default=5.0, help="到位容差（毫米），默认 5")
    ap.add_argument("--force", action="store_true", help="跳过任务工作区检查")
    ap.add_argument("--timeout", type=float, default=5.0, help="等待订阅者连接的秒数")
    args = ap.parse_args()

    rclpy.init()
    node = RoArmCli()
    try:
        if not node.wait_for_subscribers(args.timeout):
            print("⚠ 没有发现订阅者：arm_moveit_write 没在跑？")
            print("  先起：bash scripts/sim_up.sh")
            return EXIT_FAIL

        if args.command:
            rc = handle(node, args.command, args.force, args.wait, args.tol)
            return EXIT_OK if rc is None else rc

        print("交互模式。list / home / start / right / up / pose x y z / grip open|close / quit")
        print("（交互模式下不做到位校验；要校验请用单条命令加 --wait）")
        while True:
            try:
                line = input("roarm> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            r = handle(node, shlex.split(line), args.force, 0.0, args.tol)
            if r == "quit":
                break
        return EXIT_OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

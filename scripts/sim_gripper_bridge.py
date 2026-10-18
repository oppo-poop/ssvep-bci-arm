#!/usr/bin/env python3
"""sim_gripper_bridge —— 仿真专用夹爪转接节点

为什么需要它：
    arm_moveit_write::control_gripper 不走 MoveIt，直接把舵机角度发到
    set_angle_topic。真机上由 robo_driver 订阅该话题，所以**真机能动**；
    但仿真里没有 robo_driver，于是夹爪命令石沉大海。

    这个节点只跑在仿真里，把 set_angle_topic（舵机角度，度）翻译成
    hand_controller 的 GripperCommand 动作（关节位置，米），让仿真的夹爪真的动。

单位换算（与 cello_controller.py 的 meters_to_degrees 互逆）：
    meters_to_degrees(m) = (m / 0.032) * 100 + 100
    ⇒ m = (deg - 100) * 0.032 / 100

    注意：arm_moveit_write 的 "close" 是 0.0° → m = −0.032，超出 URDF 限位 [−0.025, 0]。
    本节点会把超限值**夹紧到 −0.025 并打日志**，不静默吞掉——这条正是核查报告 §六·F
    记录的「三套角度口径」在实际运行中的表现。

用法（通常由 sim_up.sh 自动启动）：
    python3 scripts/sim_gripper_bridge.py
    python3 scripts/sim_gripper_bridge.py --action /hand_controller/gripper_cmd
"""
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from robo_interfaces.msg import SetAngle

GRIPPER_SERVO_ID = 6
ARM_LENGTH_M = 0.032
LIMIT_LOW, LIMIT_HIGH = -0.025, 0.0
CLAMP_WARN_M = 2e-4


def deg_to_meters(deg):
    return (deg - 100.0) * ARM_LENGTH_M / 100.0


class SimGripperBridge(Node):
    def __init__(self, action_name):
        super().__init__("sim_gripper_bridge")
        self.action_name = action_name
        self.client = ActionClient(self, GripperCommand, action_name)
        self.create_subscription(SetAngle, "set_angle_topic", self._on_set_angle, 10)
        self._pending = False
        self.get_logger().info(
            "SimGripperBridge ready: set_angle_topic -> %s" % action_name)

    def _on_set_angle(self, msg):
        if GRIPPER_SERVO_ID not in list(msg.servo_id):
            return
        idx = list(msg.servo_id).index(GRIPPER_SERVO_ID)
        if idx >= len(msg.target_angle):
            return
        deg = float(msg.target_angle[idx])
        meters = deg_to_meters(deg)

        clamped = min(max(meters, LIMIT_LOW), LIMIT_HIGH)
        if abs(clamped - meters) > CLAMP_WARN_M:
            self.get_logger().warn(
                "夹爪角度 %.1f° → %.4f m 超出限位 [%.3f, %.3f]，已夹紧到 %.4f m"
                % (deg, meters, LIMIT_LOW, LIMIT_HIGH, clamped))
        else:
            self.get_logger().info("夹爪 %.1f° → %.4f m" % (deg, clamped))

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("动作服务器 %s 不可用" % self.action_name)
            return

        goal = GripperCommand.Goal()
        goal.command.position = clamped
        goal.command.max_effort = 5.0
        self._pending = True
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("夹爪目标被拒绝")
            self._pending = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result().result
        self.get_logger().info(
            "夹爪到位: pos=%.4f m  reached_goal=%s  stalled=%s"
            % (res.position, res.reached_goal, res.stalled))
        self._pending = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--action", default="/hand_controller/gripper_cmd")
    args = ap.parse_args()

    rclpy.init()
    node = SimGripperBridge(args.action)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""cello_controller 轨迹下发的离线回归自检（不需要 ROS / 真机）。

    python3 scripts/traj_check.py

用假的 rclpy 模块把控制器 import 进来，绕过 Node.__init__ 直接调
arm_execute_callback，检查：
  1. 默认模式：整段轨迹只发 1 条命令，time = 最后一个点的 time_from_start
  2. 等待时长 = time + 400ms（对齐 robo_driver.set_angle_callback 的 +400）
  3. 目标角度取最后一个点、按 joint_names 映射
  4. ARM_LEGACY_TRAJECTORY=1 时逐点下发，回退路径仍可用
  5. 空轨迹 / 取消请求不崩，且状态正确

改运动时序后跑一次。真机效果仍要链神确认（Agent 看不到 /dev）。
"""
import importlib.util
import re
import sys
import types
from pathlib import Path

CTRL = (Path(__file__).resolve().parent.parent
        / "star-arm-moveit2/src/cello_controller/cello_controller/cello_controller.py")
DRV = (CTRL.parent.parent.parent
       / "robo_driver/robo_driver/robo_driver.py")


# ---------- 1) 最小 ROS 替身 ----------
def _module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    if "." in name:
        parent, child = name.rsplit(".", 1)
        if parent in sys.modules:
            setattr(sys.modules[parent], child, m)
    return m


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Action:
    class Result:
        def __init__(self, **kw):
            self.__dict__.update(kw)


class _Node:
    def __init__(self, name, *a, **k):
        self.name = name


_module("rclpy", init=lambda *a, **k: None, shutdown=lambda: None)
_module("rclpy.action", ActionServer=object, CancelResponse=object)
_module("rclpy.node", Node=_Node)
_module("rclpy.callback_groups", ReentrantCallbackGroup=object)
_module("rclpy.executors", MultiThreadedExecutor=object)
_module("std_msgs")
_module("std_msgs.msg", Float32MultiArray=_Msg)
_module("robo_interfaces")
_module("robo_interfaces.msg", SetAngle=_Msg)
_module("control_msgs")
_module("control_msgs.action", FollowJointTrajectory=_Action, GripperCommand=_Action)
_module("sensor_msgs")
_module("sensor_msgs.msg", JointState=_Msg)


class _Clock:
    """虚拟时钟：sleep 直接推进时间，让等待循环瞬间跑完。"""
    def __init__(self):
        self.t = 0.0
    def time(self):
        return self.t
    def sleep(self, s):
        self.t += s


class _FakePublisher:
    def __init__(self):
        self.msgs = []
    def publish(self, msg):
        self.msgs.append(msg)


class _FakeLogger:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


class _Goal:
    def __init__(self, traj, cancel=False):
        self.request = types.SimpleNamespace(trajectory=traj)
        self._cancel = cancel
        self.succeeded = False
        self.canceled_flag = False
    @property
    def is_cancel_requested(self):
        # rclpy 的 ServerGoalHandle.is_cancel_requested 是 **property**，不是方法。
        # 写成方法对象会恒为真，导致等待循环一进去就误判取消。
        return self._cancel
    def succeed(self):
        self.succeeded = True
    def canceled(self):
        self.canceled_flag = True


def _pt(t, angles):
    return types.SimpleNamespace(
        time_from_start=types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9)),
        positions=list(angles))


NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _traj(points):
    return types.SimpleNamespace(joint_names=NAMES, points=points)


def _node(mod, clock):
    mod.time = clock
    node = mod.RoboActionClient.__new__(mod.RoboActionClient)
    node.set_angle_publisher = _FakePublisher()
    node.get_logger = lambda: _FakeLogger()
    node.last_time = 0
    node._have_state = True
    node.current_angle = [0.0] * 7
    return node


def main():
    spec = importlib.util.spec_from_file_location("ctrl_under_test", CTRL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    C = mod.RoboActionClient
    fails = []

    # 跨文件校验：驱动层的 +N ms 必须等于控制器的 INTERVAL_PAD_MS
    pad_drv = int(re.search(r"int\(msg\.time\[i\]\)\s*\+\s*(\d+)",
                            DRV.read_text(encoding="utf-8")).group(1))
    pad_ctrl = int(re.search(r"INTERVAL_PAD_MS\s*=\s*(\d+)",
                             CTRL.read_text(encoding="utf-8")).group(1))

    def check(cond, label):
        print("  %s %s" % ("✓" if cond else "✗", label))
        if not cond:
            fails.append(label)

    # --- 默认模式：3 个点，总时长 2.0 s ---
    C.LEGACY = False
    clock = _Clock()
    node = _node(mod, clock)
    goal = _Goal(_traj([
        _pt(0.5, [0.1] * 6), _pt(1.0, [0.2] * 6), _pt(2.0, [0.3] * 6)]))
    node.arm_execute_callback(goal)
    msgs = node.set_angle_publisher.msgs
    print("[默认模式] 3 点轨迹，总时长 2.0s")
    check(len(msgs) == 1, "只发 1 条命令（旧逻辑会发 3 条互相覆盖）")
    check(msgs[0].time == [2000] * 6, "time = 2000ms（取最后一点的 time_from_start）")
    expect = mod.radians_to_degrees(0.3)
    check(abs(msgs[0].target_angle[0] - expect) < 1e-6,
          "目标角取最后一点（%.2f°）并按 joint_names 映射" % expect)
    check(pad_drv == pad_ctrl,
          "驱动层 +%dms == 控制器 INTERVAL_PAD_MS %d（跨文件一致）"
          % (pad_drv, pad_ctrl))
    check(abs(clock.t - (2.0 + pad_drv / 1000.0)) < 0.02,
          "等待 %.1fs = 2000ms + 驱动层 %dms 缓冲"
          % (2.0 + pad_drv / 1000.0, pad_drv))
    check(goal.succeeded and not goal.canceled_flag, "走完才 succeed")

    # --- 回退模式：逐点 ---
    C.LEGACY = True
    clock2 = _Clock()
    node2 = _node(mod, clock2)
    goal2 = _Goal(_traj([
        _pt(0.5, [0.1] * 6), _pt(1.0, [0.2] * 6), _pt(2.0, [0.3] * 6)]))
    node2.arm_execute_callback(goal2)
    msgs2 = node2.set_angle_publisher.msgs
    print("[回退模式] ARM_LEGACY_TRAJECTORY=1")
    check(len(msgs2) == 3, "逐点下发 3 条")
    check([m.time[0] for m in msgs2] == [500, 500, 1000], "各点时长 = 点间隔")
    check(abs(clock2.t - 2.0) < 0.02, "累计等待 2.0s")
    C.LEGACY = False

    # --- 空轨迹 ---
    clock3 = _Clock()
    node3 = _node(mod, clock3)
    goal3 = _Goal(_traj([]))
    node3.arm_execute_callback(goal3)
    print("[边界] 空轨迹 / 取消")
    check(not node3.set_angle_publisher.msgs and goal3.succeeded,
          "空轨迹：不发命令、直接成功")

    # --- 取消请求 ---
    clock4 = _Clock()
    node4 = _node(mod, clock4)
    goal4 = _Goal(_traj([_pt(1.0, [0.3] * 6)]), cancel=True)
    node4.arm_execute_callback(goal4)
    check(goal4.canceled_flag and not goal4.succeeded,
          "取消：进入 canceled，不误报成功")
    check(len(node4.set_angle_publisher.msgs) == 2,
          "取消：先发轨迹命令、再发保持当前角度")

    print()
    if fails:
        print("✗ %d 项不通过：" % len(fails))
        for f in fails:
            print("    - " + f)
        return 1
    print("✓ 全部通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

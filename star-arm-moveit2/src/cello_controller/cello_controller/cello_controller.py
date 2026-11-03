import rclpy                            
from rclpy.action import ActionServer, CancelResponse  # 动作服务器与取消响应 / Action server & cancel response
from rclpy.node import Node
import time
import os
from std_msgs.msg import Float32MultiArray
# from uservo.uservo_ex import uservo_ex
from robo_interfaces.msg import SetAngle  # 舵机角度控制消息 / Servo angle control message
from control_msgs.action import FollowJointTrajectory  # 机械臂轨迹动作 / Arm trajectory action
from control_msgs.action import GripperCommand  # 夹爪控制动作 / Gripper control action
from rclpy.callback_groups import ReentrantCallbackGroup  # 可重入回调组 / Reentrant callback group
from rclpy.executors import MultiThreadedExecutor  # 多线程执行器 / Executor for multithread
import math
from sensor_msgs.msg import JointState
# 常量定义 / Constants
ROBO_ACTION_NODE = 'cello_controller_node'  # 节点名称 / Node name
ROBO_CURRENT_ANGLE_SUBSCRIPTION = "joint_states"  # 当前关节角度话题 / Current joint states topic
ROBO_SET_ANGLE_PUBLISHER = 'set_angle_topic'  # 发布舵机设定角度话题 / Publish set_angle topic
ROBO_ARM_ACTION_SERVER = '/arm_controller/follow_joint_trajectory'  # 机械臂动作服务器名 / Arm action server name
ROBO_GRIPPER_ACTION_SERVER = '/hand_controller/gripper_cmd'  # 夹爪动作服务器名 / Gripper action server name

# 机器人关节名称与索引映射 / Joint name to index mapping
ROBO_TYPE_1 = "cello"
ROBO_TYPE_1_JOINT_ = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7_left",
]
ROBO_TYPE_1_INDEX_JOINT_ = {name: idx for idx, name in enumerate(ROBO_TYPE_1_JOINT_)}

def radians_to_degrees(radians):
    """弧度转角度 / convert radians to degrees"""
    return radians * (180 / math.pi)

def meters_to_degrees(meters):
    """长度(米)转角度（基于臂长0.032m）/ convert meters to degrees"""
    return (meters / 0.032) * 100

def jointstate2servoangle(servo_id, joint_state):
    """关节状态转舵机命令角度 / joint state to servo angle"""
    if servo_id < 6:
        return radians_to_degrees(joint_state)
    elif servo_id == 6:
        return meters_to_degrees(joint_state) + 100


class RoboActionClient(Node):
    """机械臂与夹爪动作服务器节点 / Action server node for arm & gripper"""
    def __init__(self):
        super().__init__(ROBO_ACTION_NODE)
        # 使用可重入回调组支持并发 / Use reentrant callback group for concurrency
        self.callback_group = ReentrantCallbackGroup()
        # 创建机械臂轨迹跟踪动作服务器 / Arm trajectory action server
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ROBO_ARM_ACTION_SERVER,
            execute_callback=self.arm_execute_callback,
            cancel_callback=self.arm_cancel_callback,
            callback_group=self.callback_group,
        )
        # 创建夹爪控制动作服务器 / Gripper control action server
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            ROBO_GRIPPER_ACTION_SERVER,
            execute_callback=self.gripper_execute_callback,
            cancel_callback=self.gripper_cancel_callback,
            callback_group=self.callback_group,
        )
        # 订阅当前关节状态话题 / Subscribe current joint states
        self.current_angle_subscription = self.create_subscription(
            JointState,
            ROBO_CURRENT_ANGLE_SUBSCRIPTION,
            self.current_angle_callback,
            1,
            callback_group=self.callback_group,
        )
        # 发布舵机角度设定话题 / Publisher for SetAngle messages
        self.set_angle_publisher = self.create_publisher(
            SetAngle, ROBO_SET_ANGLE_PUBLISHER, 1
        )
        self.get_logger().info(f"{ROBO_ACTION_NODE} is ready.")  # 节点就绪日志 / ready log
        self.current_angle = [0.0] * 7  # 当前舵机角度缓存 / current servo angles
        self.last_time = 0  # 上一次时间戳 / last trajectory time index
        # 是否收到过 joint_states。下发"保持当前角度"前必须确认：
        # current_angle 的初值全是 0.0，未收到状态时发出去等于命令所有关节转到 0°。
        self._have_state = False

    def arm_cancel_callback(self, cancel_request):
        """处理机械臂动作取消请求 / handle cancel request"""
        self.get_logger().info('Received arm cancel request')
        # 利用 ReentrantCallbackGroup + MultiThreadedExecutor，这里能和 execute 并发跑，
        # 所以能比 execute 里的检查更早把舵机停住。
        self._hold_current('cancel')
        return CancelResponse.ACCEPT

    def _hold_current(self, why):
        """把当前角度当目标发出去，让舵机以最快速度原地停住。

        舵机没有急停命令，只能把"当前位置"当目标下发（time 100 ms），
        等价于冻结。真正的断电急停仍然只能靠电源开关。
        """
        if not self._have_state:
            self.get_logger().warn(
                f'arm {why}: 尚未收到 joint_states，跳过保持（避免把 0° 当成目标）')
            return
        try:
            hold = SetAngle(
                servo_id=list(range(7)),
                target_angle=[float(a) for a in self.current_angle],
                time=[100] * 7,
            )
            self.set_angle_publisher.publish(hold)
            self.get_logger().warn(f'arm {why}: 已下发保持当前角度')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'arm {why}: 保持当前角度失败 {exc}')

    def current_angle_callback(self, msg):
        """接收并转换 JointState 到舵机角度 / receive joint states"""
        for name, pos in zip(msg.name, msg.position):
            idx = ROBO_TYPE_1_INDEX_JOINT_[name]
            self.current_angle[idx] = jointstate2servoangle(idx, pos)
        self._have_state = True

    # ---- 轨迹下发模型（2026-11-03）----
    # 这类串口总线舵机是「给目标角度 + 给运行时间，自己跑梯形加减速」，
    # **不是**能实时跟随密集轨迹点的高速伺服。
    #
    # 旧逻辑逐点下发，MoveIt 轨迹点通常每 0.05~0.1 s 一个，控制器每点
    # 只 sleep duration 就发下一条；但 robo_driver.set_angle_callback 会把
    # 每条命令的 time 固定 +400 ms（interval = time + 400），即一条命令
    # 至少要 500 ms 才走完。三个现象其实是同一个根因：
    #   ① 舵机还在加速段就被下一条目标覆盖 -> 速度/方向反复重算 -> **抖动**
    #   ② 最后一条发出后只等 0.1 s 就 succeed，舵机根本没走完
    #      -> cli_pose 里「长轨迹会被驱动丢点、停在半路」的真正来源
    #      （之前一直当玄学，靠 retries 分段逼近绕过去）
    #   ③ 每条都被打断、到位要重发，体感「速度太慢」
    #
    # 新逻辑：整段轨迹合成**一条**命令。目标 = 最后一个点，时长 = 最后一个
    # 点的 time_from_start（MoveIt 已算好的总时长），并让等待时间与驱动层
    # interval 对齐（time_ms + 400），舵机走完才返回。
    #
    # 代价：中间路径点丢弃，各关节用同一时长线性到位（关节空间直线），
    # 不再是 MoveIt 规划的曲线。取放每段都是短距离位姿，影响很小；
    # 若发现需要绕障，ARM_LEGACY_TRAJECTORY=1 回退逐点模式做 A/B 对比。
    LEGACY = os.environ.get("ARM_LEGACY_TRAJECTORY", "") == "1"

    # robo_driver.set_angle_callback 里 interval = msg.time + 400 ms。
    # 控制器必须等满这个时长才 succeed，否则舵机没走完就被下一条命令打断。
    # **这个值必须和驱动层一致** —— scripts/traj_check.py 会从两处源码
    # 分别抓来比对，改了驱动层却忘了这里，自检会报出来。
    INTERVAL_PAD_MS = 400

    def _angles_from_point(self, trajectory, pt, time_ms):
        """构造一条 SetAngle：目标取该点关节角，6 个关节共用 time_ms。"""
        msg = SetAngle(
            servo_id=list(range(6)),
            target_angle=[0.0] * 6,
            time=[int(time_ms)] * 6,
        )
        for name, angle in zip(trajectory.joint_names, pt.positions):
            idx = ROBO_TYPE_1_INDEX_JOINT_[name]
            msg.target_angle[idx] = jointstate2servoangle(idx, angle)
        return msg

    def _wait_cancelable(self, goal_handle, seconds, tag):
        """分段等待（对齐舵机执行时间），期间响应取消。返回是否已被取消。"""
        deadline = time.time() + seconds
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                self._hold_current(tag)
                goal_handle.canceled()
                self.get_logger().warn('Trajectory canceled by client')
                return True
            time.sleep(0.01)
        return False

    def _canceled_at_end(self, goal_handle):
        if goal_handle.is_cancel_requested:
            self._hold_current('end')
            goal_handle.canceled()
            self.get_logger().warn('Trajectory canceled at end')
            return True
        return False

    def arm_execute_callback(self, goal_handle):
        """执行轨迹。默认整段一条命令；ARM_LEGACY_TRAJECTORY=1 回退逐点。"""
        trajectory = goal_handle.request.trajectory
        pts = trajectory.points
        if not pts:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        if self.LEGACY:
            self.last_time = 0
            for pt in pts:
                tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                duration = max(tfs - self.last_time, 0.1)
                self.last_time = tfs
                self.set_angle_publisher.publish(
                    self._angles_from_point(trajectory, pt, duration * 1000))
                if self._wait_cancelable(goal_handle, duration, 'exec'):
                    return FollowJointTrajectory.Result()
            if self._canceled_at_end(goal_handle):
                return FollowJointTrajectory.Result()
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # ---- 整段一条命令 ----
        last = pts[-1]
        total = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        total = max(total, 0.3)          # 单点 / 零时长轨迹的急动保护
        time_ms = int(total * 1000)
        self.set_angle_publisher.publish(
            self._angles_from_point(trajectory, last, time_ms))
        self.get_logger().info(
            'trajectory: %d points -> 1 command, target time %d ms'
            % (len(pts), time_ms))
        # 对齐驱动层：set_angle_callback 里 interval = time + INTERVAL_PAD_MS。
        # 不等满就 succeed 会让舵机在半路被下一条命令打断（旧 bug）。
        if self._wait_cancelable(
                goal_handle, (time_ms + self.INTERVAL_PAD_MS) / 1000.0, 'exec'):
            return FollowJointTrajectory.Result()
        if self._canceled_at_end(goal_handle):
            return FollowJointTrajectory.Result()
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def gripper_cancel_callback(self, cancel_request):
        """处理夹爪取消请求 / handle gripper cancel"""
        self.get_logger().info('Received gripper cancel request')
        return CancelResponse.ACCEPT

    def gripper_execute_callback(self, goal_handle):
        """执行夹爪命令 / execute gripper command"""
        pos = goal_handle.request.command.position
        # 构造单舵机 SetAngle / build single servo command
        msg = SetAngle(
            servo_id=[6],
            target_angle=[jointstate2servoangle(6, pos)],
            time=[1000],
        )
        self.set_angle_publisher.publish(msg)
        goal_handle.succeed()
        return GripperCommand.Result()


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 客户端 / init ROS
    node = RoboActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)  # 添加节点到执行器 / add node
    executor.spin()  # 运行执行器 / spin executor
    rclpy.shutdown()  # 退出 ROS / shutdown


if __name__ == '__main__':
    main()

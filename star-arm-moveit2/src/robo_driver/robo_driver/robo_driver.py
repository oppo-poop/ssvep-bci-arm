#!/usr/bin/env python
# coding:utf-8
"""
机械臂驱动节点
Robot arm driver node
"""
import rclpy
from rclpy.node import Node
from robo_interfaces.srv import ReadData, WriteData
from robo_interfaces.msg import SetAngle
import struct
from sensor_msgs.msg import JointState
import math
import os
import sys
import fashionstar_uart_sdk as uservo
import time
import serial

ROBO_DRIVER_NODE = "robo_driver_node"  # 驱动节点名称 / driver node name
ROBO_SET_ANGLE_SUBSCRIBER = "set_angle_topic"  # 设置角度话题 / topic for setting angles

# PORT_NAME: 设置舵机串口号，默认使用/dev/ttyUSB0，当需要在同一个设备上使用多个机械臂时，需要修改该参数
# Port name: Serial port for servos, default "/dev/ttyUSB0", change when using multiple arms
# Whether to check servo angle on power-up, default True to ensure consistent init
SERVO_PORT_NAME = "/dev/ttyUSB0"  # 舵机串口号 <<< 修改为实际串口号
                                # Servo serial port <<< modify to actual port name
SERVO_BAUDRATE = 1000000  # 舵机的波特率 / Servo communication baud rate
servo_ids = list()

"""
舵机转手臂控制通用类
Universal class for servo-to-arm control
"""

class uservo_ex:
    JOINT_ = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7_left",
    ]
    ROBO_TYPE_INDEX_JOINT = {
        value: index for index, value in enumerate(JOINT_)
    }
    SRV_NUM =7

    # 零位偏移（度）—— 2026-10-18 标定得到。
    # 含义：URDF 角度为 0 时，舵机的实际读数。
    # 读和写两个方向必须用同一张表镜像，只改一边会让 joint_states 与指令不一致。
    #   读：q = (deg - offset) * pi/180
    #   写：deg = q * 180/pi + offset
    # joint7（夹爪）不参与：它是开合行程，不是关节零位。
    # 标定方法见 docs/零位标定.md。
    # 2026-10-18 实测：把臂摆成 Γ 形（FK 末端距 URDF 零位 4.8 mm）时的读数。
    ZERO_OFFSET_DEG = [0.4, -0.2, -0.1, 1.3, -0.3, 0.8]

    #: 夹爪（joint7）口径，2026-11-07 队友标定（夹爪修复后换了几何）。
    #: **角度是相对起栈基准的偏移量，不是绝对舵机角**：换爪/断电后多圈零点就变，
    #: 同一个绝对角两次起栈可以一次合拢、一次张到顶（实测 60° 两副样）。
    #: 起栈时读一次实际角记为 base，之后 target = base + deg。
    #:   0°    = 基准位（约定起栈前两指拨到合拢）  1.6 mm  joint7_left -0.025
    #:   -100° = 再张 100°（机械极限）             50.9 mm  joint7_left  0.000
    #: 角度→间距是二次曲线，前 40° 基本空程；线性会把中间开度算错 4 mm 以上。
    GRIP_DEG_CLOSE = 0.0
    GRIP_DEG_OPEN = -100.0
    GRIP_FULL_MM = 68.0
    GRIP_CURVE_K = 0.724861
    GRIP_GAP_MIN_MM = 1.6
    GRIP_JOINT_MIN = -0.025
    GRIP_JOINT_MAX = 0.0
    #: 舵机限功率。旧值 8000 mW 在夹紧时顶不动硬物，队友标定时用 12000。
    GRIP_POWER_MW = 12000
    #: 起栈夹爪角。None = **原地不动**（读回当前角原样发回）。绝不要写死一个
    #: 绝对角：多圈零点每次上电都可能不同，写死会"绕一圈再顶到机械极限"。
    GRIP_START_DEG = None


    # 参数 / Parameters:
    def __init__(self,robo_type,log = None):
        self.ROBO_TYPE = robo_type
        self.INDEX_JOINT_ = {value: index for index, value in enumerate(self.JOINT_)}
        self.log = log
        # 初始化串口 / Initialize serial port
        try:
            self.uart = serial.Serial(port=SERVO_PORT_NAME,baudrate=SERVO_BAUDRATE,parity=serial.PARITY_NONE,stopbits=1,bytesize=8,timeout=0)
        except serial.SerialException as e:
            if self.log != None:
                self.log.error(f"{e}")
            raise ValueError(f"串口初始化失败: {e}")  # Serial port init failed
        try:
            self.uservo = uservo.UartServoManager(self.uart)
        except Exception as e:
            raise

        self.servo_ids = [0,1,2,3,4,5,6]
        # 启动时驱到的位置 = URDF 零位对应的舵机角度，而不是舵机 0°。
        # 单位是 0.1°（target_angle 的格式）；offset 全 0 时与旧行为完全一致。
        self.ZERO_ANGLE = [int(10 * o) for o in self.ZERO_OFFSET_DEG]

        # ---------------- 夹爪：只重置 6 号多圈，然后**原地不动** ----------------
        # 为什么不是 reset_multi_turn_angle(0xff)
        #   0xff 是广播 ID，这类舵机只认单独 ID，广播会被静默忽略 —— 多圈零点
        #   根本没归零，"基准 + 100°" 就可能算到 260° 这种超出舵机范围的值，
        #   命令被吞（表现为"发了 open 夹爪没反应"）。臂的 6 个关节更不能 reset：
        #   那会把绝对位置全抹掉，整臂位置就无从得知了。
        # 为什么"读什么发什么"
        #   多圈零点既然不可靠，就**不假设**它是多少：读一次当前角记为 base，
        #   把当前角原样发回去 = 命令"留在原地"。写死某个绝对角的话，差值大时
        #   会绕整整一圈、途中顶到机械极限再弹回来（用户看见"舵机转了一圈"）。
        self.grip_base_deg = 0.0
        self.grip_ok = False
        try:
            self.reset_multi_turn_angle(6)
            time.sleep(0.15)
            _cur = self.query_servo_current_angle(6)
            if _cur is None:
                raise ValueError("读回 None")
            self.grip_base_deg = float(_cur)
            self.grip_ok = True
        except Exception as _e:                       # noqa: BLE001
            msg = ("[gripper] ⚠ 舵机6 基准读取失败（%s）—— 夹爪命令将被拒绝，"
                   "臂不受影响" % _e)
            if self.log is not None:
                self.log.error(msg)
            print(msg, flush=True)
        self.ZERO_ANGLE.append(int(round(10 * self.grip_base_deg)))
        print("[gripper] close=%.1f° open=%.1f° full=%.1fmm k=%+.3f 基准=%.1f°%s"
              % (self.GRIP_DEG_CLOSE, self.GRIP_DEG_OPEN, self.GRIP_FULL_MM,
                 self.GRIP_CURVE_K, self.grip_base_deg,
                 "" if self.grip_ok else "  ← 基准不可用"), flush=True)
        time.sleep(0.1)

    # 将角度转为弧度 / Convert degrees to radians
    @classmethod
    def degrees_to_radians(cls, degrees):
        radians = degrees * (math.pi / 180)
        return radians

    # ---------------- 夹爪换算（相对基准角）----------------
    @classmethod
    def _grip_shape(cls, x):
        """归一化开度 0..1 → 间距形状 0..1。二次曲线，前段空程。"""
        x = max(0.0, min(1.0, float(x)))
        k = cls.GRIP_CURVE_K
        return x * x + k * x * (1.0 - x)

    @classmethod
    def _grip_shape_inv(cls, y):
        """间距形状 0..1 → 归一化开度。解 (1−k)x² + kx − y = 0。"""
        y = max(0.0, min(1.0, float(y)))
        k = cls.GRIP_CURVE_K
        a = 1.0 - k
        if abs(a) < 1e-9:
            return y
        x = (-k + math.sqrt(max(0.0, k * k + 4.0 * a * y))) / (2.0 * a)
        return max(0.0, min(1.0, x))

    @classmethod
    def grip_x(cls, deg_rel):
        span = cls.GRIP_DEG_OPEN - cls.GRIP_DEG_CLOSE
        if abs(span) < 1e-9:
            return 0.0
        return max(0.0, min(1.0, (float(deg_rel) - cls.GRIP_DEG_CLOSE) / span))

    @classmethod
    def grip_gap_mm(cls, deg_rel):
        """夹爪舵机角（相对基准）→ 两指间距（毫米）。"""
        span_mm = cls.GRIP_FULL_MM - cls.GRIP_GAP_MIN_MM
        return cls.GRIP_GAP_MIN_MM + span_mm * cls._grip_shape(cls.grip_x(deg_rel))

    @classmethod
    def grip_deg_to_joint(cls, deg_rel):
        """夹爪舵机角（相对基准）→ joint7_left（米）。

        必须 clamp 在 URDF 限位里：一旦超限，MoveIt 会把整机状态判成 invalid，
        OMPL 报 "Skipping invalid start state"，之后**任何目标都不动**。
        """
        travel = cls.GRIP_JOINT_MAX - cls.GRIP_JOINT_MIN
        v = cls.GRIP_JOINT_MIN + travel * cls._grip_shape(cls.grip_x(deg_rel))
        return max(cls.GRIP_JOINT_MIN, min(cls.GRIP_JOINT_MAX, v))

    @classmethod
    def grip_joint_to_deg(cls, joint):
        """joint7_left（米）→ 夹爪舵机角（相对基准）。"""
        travel = cls.GRIP_JOINT_MAX - cls.GRIP_JOINT_MIN
        y = 0.0 if abs(travel) < 1e-9 else \
            (float(joint) - cls.GRIP_JOINT_MIN) / travel
        return cls.GRIP_DEG_CLOSE + cls._grip_shape_inv(y) * (
            cls.GRIP_DEG_OPEN - cls.GRIP_DEG_CLOSE)

    # 将角度转为米 / Convert degrees back to meters
    @classmethod
    def servoangle2jointstate(cls, servo_id, servo_angle, grip_base=0.0):
        if servo_id in range(6):
            # 先扣掉零位偏移，再换成弧度
            return cls.degrees_to_radians(servo_angle - cls.ZERO_OFFSET_DEG[servo_id])
        elif servo_id == 6:
            # 夹爪**必须先减基准**，否则读数和物理对不上：基准 160°、爪子就停在
            # 160°（物理上合拢），按绝对角算会落到 [-100, 0] 之外 → clamp 成全开，
            # 于是界面显示"50.9 mm"，人眼看着明明是闭合的。
            return cls.grip_deg_to_joint(servo_angle - grip_base)

    # 设置角度（指定转速） / Send angle commands with specified speed
    def set_angle_by_interval(self, size, command_data_list):
        self.uservo.send_sync_multiturnanglebyinterval(
            self.uservo.CODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL,
            size, command_data_list
        )

    # 查询角度 / Query current servo angle
    def query_servo_current_angle(self, servo_id):
        if servo_id in self.uservo.servos:
            return self.uservo.query_servo_angle(servo_id)

    # 失能舵机 / Disable servo torque
    def disable_torque(self, servo_id):
        self.servo_stop(servo_id)

    # 重设指定舵机多圈圈数 / Reset multi-turn count for a single servo
    def reset_multi_turn_angle(self, servo_id):
        self.uservo.disable_torque(servo_id)
        time.sleep(0.05)
        self.uservo.reset_multi_turn_angle(servo_id)

    # 停止并释放锁力 / Stop servo and release torque lock
    def servo_stop(self, servo_id, mode=2, power=500):
        self.uservo.stop_on_control_mode(servo_id, mode, power)

class Arm_contorl(Node):

    def __init__(self):
        super().__init__(ROBO_DRIVER_NODE)
        self.declare_parameter("robo_type", "robo")

        self.declare_parameter('lock', 'enable')
        #: 启动时是否驱到零位。默认 True —— 一条命令就到位，不用再补一条 home。
        #: 2026-11-02 从折叠态启动时 1.5 s 展开 90° 失力回落后，把时长改成 3 s。
        #: 想上电完全不动：zero_on_start:=false（或 ZERO_ON_START=false）。
        self.declare_parameter('zero_on_start', True)

        self.robo_type = (self.get_parameter("robo_type").get_parameter_value().string_value)
        self.lock = (self.get_parameter("lock").get_parameter_value().string_value)
        self.zero_on_start = (
            self.get_parameter("zero_on_start").get_parameter_value().bool_value)

        try:
            self.Servo = uservo_ex(self.robo_type,log = self.get_logger())
        except ValueError as e:
            raise
        self.target_angle = self.Servo.ZERO_ANGLE
        # self.target_angle 初始目标角度列表 / initial target angle list
        #: 启动驱零位用的时间。原来 1500 ms —— 折叠态展开 90° 时太猛，
        #: 2026-11-02 上升到一半失力回落。改 3000 ms，加速段拉平、冲击电流小一半。
        #: 只影响启动这一次；之后每条运动命令都会用自己的 time（见 set_angle_callback）。
        self.interval = [3000 for _ in range(self.Servo.SRV_NUM)]
        self.current_angle = [0.0 for _ in range(self.Servo.SRV_NUM)]
        if self.lock == "enable":
            if self.zero_on_start:
                # 旧行为：直接驱到 ZERO_ANGLE。臂若停在折叠态，等于满速展开。
                self.get_logger().warn(
                    "zero_on_start=true：正在驱到零位（臂会大幅运动，注意）")
                self.arm_move_by_time()
            else:
                self.hold_current_pose()
        else:
            self.Servo.disable_torque(0xff)

        # 创建话题：发布joint_states / Create publisher for joint_states topic
        self.joint_states_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        # 创建话题：处理设置角度 / Create subscription for SetAngle commands
        self.angle_subscribers = self.create_subscription(
            SetAngle, ROBO_SET_ANGLE_SUBSCRIBER, self.set_angle_callback, 10
        )
        self.timer2 = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info(f"初始化完成 / Initialization complete")

    def node_close(self):
        pass
        # self.Servo.servo_all_stop()

    # 新的执行命令 / Callback for new angle commands
    def set_angle_callback(self, msg):
        for i in range(len(msg.servo_id)):
            id = msg.servo_id[i]
            angle = float(msg.target_angle[i])
            # 写方向补零位偏移：URDF 角度 → 舵机角度（夹爪不参与零位）
            offset = self.Servo.ZERO_OFFSET_DEG[id] if id < 6 else 0.0
            if id == 6:
                # 夹爪：消息里的角度是**相对基准**的偏移，先夹到安全区间再换成
                # 绝对舵机角。旧节点 / bag 回放可能发越界值，直接夹住并告警。
                lo = min(self.Servo.GRIP_DEG_CLOSE, self.Servo.GRIP_DEG_OPEN)
                hi = max(self.Servo.GRIP_DEG_CLOSE, self.Servo.GRIP_DEG_OPEN)
                rel = min(max(angle, lo), hi)
                if abs(rel - angle) > 1e-6:
                    self.get_logger().warn(
                        "夹爪目标 %.1f° 越界，已夹到 [%.1f, %.1f]°"
                        % (angle, lo, hi))
                if self.Servo.grip_ok:
                    angle = self.Servo.grip_base_deg + rel
                else:
                    # 基准没读到就别动夹爪：相对角没有参照系，发出去可能是
                    # "从当前位置再张 100°" 的整圈运动。
                    self.get_logger().error(
                        "夹爪基准不可用，拒绝执行（臂不受影响）；"
                        "先跑 scripts/servo_check.py 查舵机 6")
                    angle = self.Servo.grip_base_deg
            self.target_angle[id] = int(10 * (angle + offset))
            if int(msg.time[i]) < 40:
                (msg.time[i]) = 40
            self.interval[id] = int(msg.time[i])+400

            self.arm_move_by_time()

    # 定时任务 / Timer callback
    def timer_callback(self):
        self.publish_current_angle()

    # 查询舵机角度发布 / Query servo angles and publish
    def publish_current_angle(self):
        JointState_msg = JointState()
        JointState_msg.header.stamp = self.get_clock().now().to_msg()
        JointState_msg.velocity = []
        JointState_msg.effort = []
        self.Servo.uservo.send_sync_servo_monitor(self.Servo.servo_ids)

        for i in range(self.Servo.SRV_NUM):
            self.current_angle[i] = self.Servo.uservo.servos[i].angle_monitor
            JointState_msg.name.append(self.Servo.JOINT_[i])
            JointState_msg.position.append(
                self.Servo.servoangle2jointstate(
                    servo_id=i, servo_angle=self.current_angle[i],
                    grip_base=self.Servo.grip_base_deg,
                )
            )
        self.joint_states_publisher.publish(JointState_msg)

    def hold_current_pose(self):
        """上力矩并**保持当前姿态**，不做任何运动。

        为什么不用 arm_move_by_time() 直接驱零位
            原来的一行 `self.arm_move_by_time()` 目标是 ZERO_ANGLE（各关节约 0°），
            interval 固定 1500 ms。臂若停在折叠态（实测 j2=-90.5°、j3=+89.5°），
            等于 1.5 秒内各转 90° 展开整臂 —— 重心外移、j2 力臂变长，
            力矩在中段达到峰值。2026-11-02 就是这么摔的：展开到一半整臂失力。

        为什么"读什么发什么"
            reset_multi_turn_angle() 只发请求、不等确认（SDK 里没有响应处理），
            所以"多圈 0 == 当前位置"这个前提**不可靠**。写死 0 可能反而触发大动作。
            读回来的多圈角作为目标，天然等于"原地不动"。

        读不全就一条命令都不发
            目标里混进一个 ZERO_ANGLE 就会让那个关节动起来。
            上电时舵机本身带力矩，不发命令是安全的。
        """
        targets = []
        for i in range(self.Servo.SRV_NUM):
            try:
                a = self.Servo.uservo.query_servo_mturn_angle(i, realtime=True)
            except Exception as e:                    # noqa: BLE001
                self.get_logger().error("读舵机 %d 多圈角异常：%s" % (i, e))
                a = None
            if a is None:
                self.get_logger().error(
                    "启动保持姿态失败：读不到舵机 %d 的多圈角。"
                    "**不发任何运动命令**（舵机自身带力矩，臂保持原位）。"
                    "先跑 scripts/servo_check.py 查通信。" % i)
                return False
            targets.append(int(round(10 * float(a))))

        self.target_angle = targets
        self.interval = [3000 for _ in range(self.Servo.SRV_NUM)]
        self.arm_move_by_time()
        self.get_logger().info(
            "启动保持当前姿态（未驱零位）：%s"
            % ["%.1f" % (t / 10.0) for t in targets])
        return True

    # 默认：控制by时间 / Default control mode: time-based
    def arm_move_by_time(self):
        acc = [0.0 for _ in range(self.Servo.SRV_NUM)]
        for i in range(self.Servo.SRV_NUM):
            acc[i] = int(self.interval[i]/2)
        power_mw = int(os.environ.get("CELLO_POWER_MW", str(self.Servo.GRIP_POWER_MW)))
        command_data_list = [struct.pack("<BlLHHH", i, self.target_angle[i], self.interval[i], acc[i], acc[i], power_mw) for i in range(self.Servo.SRV_NUM)]
        self.Servo.set_angle_by_interval(self.Servo.SRV_NUM, command_data_list)



def main(args=None):
    rclpy.init(args=args)
    try:
        robo_driver_node = Arm_contorl()
    except Exception:                                   # noqa: BLE001
        # ⚠ 这里**绝对不能静默 return**。初始化一挂，臂就完全不动，而 launch 只会
        #   打一句 "process has finished cleanly"（退出码 0），现场表现是
        #   "启动没反应、臂不动"，看不出任何原因 —— 2026-11-08 就是这么把
        #   arm_move_by_time 里一个 NameError（漏 import os）藏了一整天，
        #   中间还查了串口、查了日志、怀疑过 ^C。
        #   打到 stderr + 非 0 退出：launch 报 "process has died"，
        #   robot_up.sh 的启动检查也能立刻失败，而不是干等 move_group。
        import traceback
        traceback.print_exc()
        print("[robo_driver] 初始化失败，进程退出（臂不会动）。看上面的 traceback。",
              file=sys.stderr, flush=True)
        try:
            rclpy.try_shutdown()
        except Exception:                               # noqa: BLE001
            pass
        sys.exit(1)

    try:
        rclpy.spin(robo_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        robo_driver_node.node_close()
        robo_driver_node.destroy_node()


if __name__ == "__main__":
    main()

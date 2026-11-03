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

    #: 夹爪（joint7）的实测端点，2026-11-03 拆装后**卸力手推**量的（不是推算）：
    #:   舵机  86.7° → 两指完全闭合（4 mm）   → joint7_left = -0.025
    #:   舵机 174.8° → 两指完全张开（65 mm）  → joint7_left =  0.000
    #: 行程 88.1°。拆装前是 21.9°~100.0°，零位偏了约 65°。
    #: ⚠ 只对"当前这次装配"有效，再拆装一次要重量（两端都要量）。
    GRIP_DEG_CLOSE = 86.7
    GRIP_DEG_OPEN = 174.8


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
        # 夹爪启动位，单位 0.1°。**不参与零位标定**，只是给一个"半开"的落点。
        # 旧值是 500（50°），旧爪 21.9°~100° 时对应约 28 mm 开度。
        # 2026-11-03 拆装后新范围是 86.7°（闭合）~174.8°（张开），
        # **50° 已经低于闭合角，启动时会硬顶 36.7° → 堵转发热。**
        # 取 1210（121°）≈ 27.8 mm，和旧爪的"半开 28 mm"物理上一致。
        self.ZERO_ANGLE.append(1210)
        self.reset_multi_turn_angle(0xff)
        time.sleep(0.1)

    # 将角度转为弧度 / Convert degrees to radians
    @classmethod
    def degrees_to_radians(cls, degrees):
        radians = degrees * (math.pi / 180)
        return radians

    # 将角度转为米 / Convert degrees back to meters
    @classmethod
    def degrees_to_meters(cls, degrees):
        meters = (degrees / 100) * 0.032
        return meters

    # 将舵机角度转换为关节位置 / Servo angle to joint state mapping
    @classmethod
    def servoangle2jointstate(cls, servo_id, servo_angle):
        if servo_id in range(6):
            # 先扣掉零位偏移，再换成弧度
            return cls.degrees_to_radians(servo_angle - cls.ZERO_OFFSET_DEG[servo_id])
        elif servo_id == 6:
            # 2026-11-03 拆装后零位偏移，旧的 (deg-100)/100*0.032 作废：
            # 它把 100° 当全开，而实测全开在 154.1°，于是 100° 之后
            # joint 会转正、超 URDF upper=0，臂就卡在"只开 10 mm"。
            # 现在按实测两端线性映射，joint 始终落在 [-0.025, 0]。
            span = cls.GRIP_DEG_OPEN - cls.GRIP_DEG_CLOSE      # 79.1°
            return (servo_angle - cls.GRIP_DEG_OPEN) * (0.025 / span)

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
            # 写方向补零位偏移：URDF 角度 → 舵机角度
            offset = self.Servo.ZERO_OFFSET_DEG[id] if id < 6 else 0.0
            self.target_angle[id] = int(10 * (msg.target_angle[i] + offset))
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
                    servo_id=i, servo_angle=self.current_angle[i]
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
        command_data_list = [struct.pack("<BlLHHH", i, self.target_angle[i], self.interval[i], acc[i], acc[i], 8000) for i in range(self.Servo.SRV_NUM)]
        self.Servo.set_angle_by_interval(self.Servo.SRV_NUM, command_data_list)



def main(args=None):
    rclpy.init(args=args)
    try:
        robo_driver_node = Arm_contorl()
    except Exception as e:
        return

    try:
        rclpy.spin(robo_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        robo_driver_node.node_close()
        robo_driver_node.destroy_node()


if __name__ == "__main__":
    main()

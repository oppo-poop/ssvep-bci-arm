#!/usr/bin/env python
# coding:utf-8
"""
机械臂驱动节点
Robot arm driver node
"""
import rclpy
import os
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
        self.ZERO_ANGLE.append(500)   # 夹爪：50°（半开），不参与零位标定
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
            return cls.degrees_to_meters(servo_angle-100)

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

    # 串口掉线后重新打开（不重置多圈、不驱到零位，避免意外运动）
    def reconnect(self):
        try:
            self.uart.close()
        except Exception:
            pass
        try:
            self.uart = serial.Serial(
                port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=0)
            self.uservo = uservo.UartServoManager(self.uart)
            return True
        except Exception as e:
            if self.log is not None:
                self.log.warning(f"串口重连失败: {e}")
            return False

class Arm_contorl(Node):

    def __init__(self):
        super().__init__(ROBO_DRIVER_NODE)
        self.declare_parameter("robo_type", "robo")

        self.declare_parameter('lock', 'enable')

        self.robo_type = (self.get_parameter("robo_type").get_parameter_value().string_value)
        self.lock = (self.get_parameter("lock").get_parameter_value().string_value)

        try:
            self.Servo = uservo_ex(self.robo_type,log = self.get_logger())
        except ValueError as e:
            raise
        self.target_angle = self.Servo.ZERO_ANGLE
        # 阻尼余量（ms，加到每段规划时长上）。用环境变量调，改手感不用重编译：
        #   CELLO_DAMP_MS=0（默认，最快）/ 30（更顺）/ 100（明显更柔但慢一倍）
        self.damp_ms = int(os.environ.get("CELLO_DAMP_MS", "0"))
        # self.target_angle 初始目标角度列表 / initial target angle list
        self.interval = [1500 for _ in range(self.Servo.SRV_NUM)]
        self.current_angle = [0.0 for _ in range(self.Servo.SRV_NUM)]
        # 是否已经读到过一帧完整的舵机角度。没读全之前不发布 joint_states，
        # 避免把初值 0.0 当成真实角度发给 MoveIt（会被判成全机 invalid）。
        self._have_angles = False
        self._last_warn_time = 0.0
        self._last_reconnect_time = 0.0
        if(self.lock == "enable"):
            self.arm_move_by_time()
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
            # 这里原来写的是 time[i] + 400 / +100（阻尼余量）。
            # 后果：每个轨迹点都在规划时长之外再多走 100ms，等于给臂加了
            # 一个固定减速，**velocity_scaling 完全失效**——0.1 和 0.3 倍速
            # 实测速度都是 3mm/s 左右（2026-10-23 踩到）。
            # 现在严格按规划给时长，只留 40ms 下限（太短的段舵机来不及响应）。
            if int(msg.time[i]) < 40:
                msg.time[i] = 40
            # 阻尼余量用环境变量调，不用改代码重编译：
            #   CELLO_DAMP_MS=0   严格按规划走（最快，稍硬）
            #   CELLO_DAMP_MS=30  每段多给 30ms，动起来更顺（速度略降）
            #   CELLO_DAMP_MS=100 明显更柔，但速度大约只有一半
            self.interval[id] = int(msg.time[i]) + self.damp_ms

        # 原来这行写在 for 循环里：一条轨迹点要写 7 次串口（每次写全部 7 个舵机），
        # 回调阻塞时间被放大 7 倍，QoS 队列(depth 10)很快灌满，
        # 后面的轨迹点——包括最后一个目标点——被丢掉，臂走一半就停住。
        # 2026-10-23 实测：home 只走了 100mm 就静止。移到循环外只写一次。
        self.arm_move_by_time()

    # 定时任务 / Timer callback
    def timer_callback(self):
        self.publish_current_angle()

    def _warn_throttled(self, text, period=2.0):
        """限频打警告，避免串口抖动时刷屏。"""
        now = time.monotonic()
        if now - self._last_warn_time >= period:
            self._last_warn_time = now
            self.get_logger().warning(text)

    def _handle_serial_error(self, where, exc):
        """串口读写异常：记日志并尝试重连，不让节点退出。"""
        self._warn_throttled(f"{where} 串口异常: {exc}", period=1.0)
        now = time.monotonic()
        if now - self._last_reconnect_time >= 1.0:
            self._last_reconnect_time = now
            if self.Servo.reconnect():
                self.get_logger().info("串口已重连")
                # 断线期间臂可能被移动过，旧角度不可信，等重新读全再发布
                self._have_angles = False

    # 查询舵机角度发布 / Query servo angles and publish
    def publish_current_angle(self):
        try:
            self.Servo.uservo.send_sync_servo_monitor(self.Servo.servo_ids)
        except Exception as e:
            # OSError(Errno 5)、SerialException、SDK 内部异常都走这里
            self._handle_serial_error("查询舵机角度", e)
            return

        names = []
        positions = []
        for i in range(self.Servo.SRV_NUM):
            srv = self.Servo.uservo.servos.get(i)
            angle = srv.angle_monitor if srv is not None else None
            if angle is None:
                if not self._have_angles:
                    # 还没拿到过完整状态，这一帧不发（不能把 0.0 当真实角度）
                    self._warn_throttled("舵机角度还没读全，暂不发布 joint_states")
                    return
                # 个别舵机丢一帧：沿用上一次的有效值，别让整条状态链断掉
                angle = self.current_angle[i]
            self.current_angle[i] = angle
            names.append(self.Servo.JOINT_[i])
            positions.append(
                self.Servo.servoangle2jointstate(servo_id=i, servo_angle=angle))

        self._have_angles = True
        JointState_msg = JointState()
        JointState_msg.header.stamp = self.get_clock().now().to_msg()
        JointState_msg.velocity = []
        JointState_msg.effort = []
        JointState_msg.name = names
        JointState_msg.position = positions
        self.joint_states_publisher.publish(JointState_msg)

    # 默认：控制by时间 / Default control mode: time-based
    def arm_move_by_time(self):
        acc = [0.0 for _ in range(self.Servo.SRV_NUM)]
        for i in range(self.Servo.SRV_NUM):
            acc[i] = int(self.interval[i]/2)
        command_data_list = [struct.pack("<BlLHHH", i, self.target_angle[i], self.interval[i], acc[i], acc[i], 8000) for i in range(self.Servo.SRV_NUM)]
        try:
            self.Servo.set_angle_by_interval(self.Servo.SRV_NUM, command_data_list)
        except Exception as e:
            self._handle_serial_error("发送舵机指令", e)



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

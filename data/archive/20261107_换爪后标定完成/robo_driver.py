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
import json
import sys

# launch 会把 stdout 重定向到 .robot_driver.log，重定向时 Python 默认用**块缓冲**，
# 进程一直不退就永远看不到 print。改成行缓冲，起栈日志才能实时读到。
# （2026-11-07 排查夹爪配置时踩到：`grep gripper .robot_driver.log` 一直为空，
#   以为代码没生效，其实是被缓冲吞了。）
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# ---------------------------------------------------------------- 夹爪参数
# 唯一真源：~/brain/gripper.json（换爪只改它，不用改任何代码）。
# 2026-11-07 抽出来：原来 robo_driver / roarm_cli / grip_tune / vision_arm_color
# 各自有一份换算，换爪要改四处，漏一处就是"软件算 40mm、实际夹到 20mm"。
_GRIPPER_DEFAULTS = {
    "close_deg": 0.0,     # 两指完全夹紧时的舵机角
    "open_deg": 180.0,    # 张到最大时的舵机角（换爪后可能 < close_deg = 方向反）
    "full_mm": 68.0,      # 全开时两指间距
    "curve_k": -0.3956,   # 间距曲线弯曲度
    "joint_min_m": -0.025,
    "joint_max_m": 0.0,
    "move_ms": 2500,
    "power_mw": 12000,
    "start_ms": 3000,
}
_GRIPPER_PATH = os.environ.get(
    "CELLO_GRIPPER_JSON", os.path.expanduser("~/brain/gripper.json"))


def _load_gripper():
    """读配置。文件缺失/坏了就用默认值 —— 一个 json 不该让整机起不来。"""
    cfg = dict(_GRIPPER_DEFAULTS)
    try:
        with open(_GRIPPER_PATH, encoding="utf8") as f:
            got = json.load(f)
        for k, v in got.items():
            if k in cfg and isinstance(v, (int, float)):
                cfg[k] = float(v)
    except FileNotFoundError:
        print("[gripper] 没找到 %s，用内置默认值" % _GRIPPER_PATH, flush=True)
    except Exception as e:
        print("[gripper] %s 读取失败（%s），用默认值" % (_GRIPPER_PATH, e), flush=True)
    if abs(cfg["open_deg"] - cfg["close_deg"]) < 1e-6:
        print("[gripper] ⚠ close_deg == open_deg，归一化会除零，退回 0/180", flush=True)
        cfg["close_deg"], cfg["open_deg"] = 0.0, 180.0
    # 启动时打一行，方便在 .robot_driver.log 里确认到底读到了什么
    print("[gripper] close=%.1f° open=%.1f° full=%.1fmm k=%+.3f  ← %s"
          % (cfg["close_deg"], cfg["open_deg"], cfg["full_mm"],
             cfg["curve_k"], _GRIPPER_PATH))
    return cfg


_GRIPPER = _load_gripper()

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

    # ---------------- 夹爪行程（2026-11-01 换爪后重标）----------------
    # 旧映射写死了 "舵机 21.9°~100° ↔ joint7 0~50mm"，换爪后完全对不上：
    # 实测全开时舵机在 -166.9°、闭合在 -276.9°，两个端点都落在旧范围外，
    # 于是 open/close 都顶死在机械极限，只走 11°，舵机堵转发烫。
    #
    # 新做法：**不再写死绝对角度**，而是用"相对起点的行程跨度"。
    # driver 启动时会 reset_multi_turn_angle(0xff)，把夹爪**当前位置**设为 0°，
    # 所以只要约定「起栈时两指是闭合的」，下面这两个量就固定：
    #     deg=0    → 闭合（两指贴住，0mm）
    #     deg=SPAN → 全开（65mm）
    # 换爪后重新标定时，只需改 GRIP_DEG_SPAN（= 全开读数 − 闭合读数）。
    #
    # **角度→间距是二次曲线，不是线性**。四个实测点拟合：
    #     0°→0   110°→19   150°→45   170°→60   180°→68（全开极限）
    #   间距 = FULL × [x² + K·x·(1−x)]，x = deg/SPAN，K = −0.3956
    #   前 50° 左右基本是空程（爪子几乎不动），所以曲线要向下压。
    #   线性映射会把 60mm 算成 159°（实际只张 53mm），差得离谱。
    GRIP_DEG_SPAN = float(os.environ.get("CELLO_GRIP_SPAN", "180"))
    GRIP_FULL_MM = float(os.environ.get("CELLO_GRIP_FULL_MM", "68"))
    GRIP_CURVE_K = float(os.environ.get("CELLO_GRIP_K", "-0.3956"))
    GRIP_TRAVEL_M = 0.025      # 单指行程，和 URDF joint7 限位 -0.025~0 对应
    GRIP_JOINT_MIN, GRIP_JOINT_MAX = -0.025, 0.0
    # 起栈后驱到的位置（0.1° 单位）。
    # ⚠ 2026-11-07 换爪后**不能再写 0**：这套舵机的命令 0° 会走到张开极限
    # （实测 j 0 → 张到顶死再弹回），起栈时爪子会猛张一次，又吵又伤舵机。
    # 改成 None = 跟 gripper.json 的 close_deg 走（现在是命令 60° = 两指 1.6mm），
    # 起栈时爪子是合上的。要用再 grip open。
    GRIP_START_DEG = None
    #: 起栈瞬间夹爪舵机的绝对角度。所有夹爪目标角都是「这个值 + 相对量」。
    #: 换爪断电后多圈零点会变，绝对角不可重复（同一个 60° 两次起栈可以是
    #: 闭合也可以是全开），所以只能相对起栈位置算。
    grip_base_deg = 0.0


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

        # 只重置**夹爪**的多圈（ID 6）。
        # 2026-11-07：原来用 reset_multi_turn_angle(0xff) 广播，这类舵机只认
        # 单独 ID，广播被静默忽略 —— 多圈角度没归零，起栈位置随机，
        # 于是"基准 + 100°"可能算到 260° 这种超出舵机范围的值，命令被吞掉
        # （表现为"发了 open 但夹爪没反应"）。
        # 臂的 6 个关节绝对不能 reset（会把绝对位置全抹掉），只动 6 号。
        try:
            self.reset_multi_turn_angle(6)
            time.sleep(0.15)
            print("[gripper] 已单独重置舵机6 多圈角度（臂关节不动）", flush=True)
        except Exception as e:
            print("[gripper] ⚠ 舵机6 多圈重置失败：%s（继续，基准改用读数）" % e,
                  flush=True)
        time.sleep(0.1)

        # ---- 夹爪：起栈时**原地不动** ----
        # 2026-11-07 踩了三次才定位：reset_multi_turn_angle(0xff) 用的是**广播 ID**，
        # 这类舵机通常只认单独 ID，广播被静默忽略 —— 多圈角度并没有归零。
        # 于是"起栈驱到某个固定角"就等于从当前绝对位置往那个角走，差值大时会
        # 绕整整一圈（用户肉眼看到"舵机转了一圈"），途中顶到机械极限再弹回来。
        # 正确做法：reset 之后**读一次当前角度，原样发回去** = 命令"留在原地"。
        if self.GRIP_START_DEG is None:
            try:
                cur = self.query_servo_current_angle(6)   # 6 = 夹爪（SRV_NUM=7 是总数，别拿它当 ID）
                if cur is None:
                    raise ValueError("读回 None")
                self.grip_base_deg = float(cur)          # 后面所有夹爪角度都相对它
                self.ZERO_ANGLE.append(int(10 * cur))
                print("[gripper] 起栈保持原位：舵机6 当前 %.1f°（记为基准）" % cur)
            except Exception as e:
                self.grip_base_deg = 0.0
                self.ZERO_ANGLE.append(int(10 * _GRIPPER["close_deg"]))
                print("[gripper] ⚠ 读不到舵机6角度（%s），基准按 0 处理" % e)
        else:
            self.grip_base_deg = 0.0
            self.ZERO_ANGLE.append(self.GRIP_START_DEG)

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
    def grip_gap_mm(cls, deg):
        """舵机角度 → 两指间距(mm)。实测曲线：FULL × [x² + K·x·(1−x)]。

        2026-11-07 起参数来自 ~/brain/gripper.json。x 是归一化开度：
        0 = close_deg（夹紧）、1 = open_deg（张到最大）。换爪后方向反了
        （close_deg > open_deg）照样能算 —— 分母为负，x 仍然 0→1。
        """
        c = _GRIPPER
        span = c["open_deg"] - c["close_deg"]
        if abs(span) < 1e-6:
            return 0.0
        x = (deg - c["close_deg"]) / span
        x = max(0.0, min(1.0, x))
        k = c["curve_k"]
        return c["full_mm"] * (x * x + k * x * (1.0 - x))

    @classmethod
    def grip_deg_to_joint(cls, deg):
        """夹爪舵机角度 → joint7_left（米）。

        joint7_left 与间距成正比（间距 ↔ 单指位移）。
        必须 clamp 在 URDF 限位内：一旦超限，MoveIt 会把整机状态判为
        invalid，之后所有规划都失败（2026-10-18 与 11-01 各踩一次）。
        """
        c = _GRIPPER
        full = c["full_mm"] if abs(c["full_mm"]) > 1e-6 else 68.0
        travel = c["joint_max_m"] - c["joint_min_m"]
        v = (cls.grip_gap_mm(deg) / full) * travel + c["joint_min_m"]
        return max(c["joint_min_m"], min(c["joint_max_m"], v))

    @classmethod
    def grip_joint_to_deg(cls, joint):
        """joint7_left（米）→ 夹爪舵机角度。上面那个的逆运算（解二次方程）。"""
        c = _GRIPPER
        j = max(c["joint_min_m"], min(c["joint_max_m"], joint))
        travel = c["joint_max_m"] - c["joint_min_m"]
        y = (j - c["joint_min_m"]) / travel if abs(travel) > 1e-9 else 0.0
        k = c["curve_k"]
        a = 1.0 - k                                        # (1−k)x² + kx − y = 0
        x = (-k + math.sqrt(max(0.0, k * k + 4.0 * a * y))) / (2.0 * a)
        x = max(0.0, min(1.0, x))
        return c["close_deg"] + x * (c["open_deg"] - c["close_deg"])

    @classmethod
    def servoangle2jointstate(cls, servo_id, servo_angle, grip_base=0.0):
        if servo_id in range(6):
            # 先扣掉零位偏移，再换成弧度
            return cls.degrees_to_radians(servo_angle - cls.ZERO_OFFSET_DEG[servo_id])
        elif servo_id == 6:
            # 夹爪：**必须减基准**，否则读数和物理对不上。
            # 2026-11-07 踩到：基准 160°、舵机就停在 160°（物理闭合），
            # 但这里按绝对角算 → 160° 超出 open_deg=100 → clamp 成全开，
            # 于是 joint7_left 报 0、界面显示"两指 50.9mm"，人眼看着明明是闭合的。
            # 命令方向已经减了基准（set_angle_callback），反馈方向也必须减。
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
        # 启动/保持时的默认运动时长（ms）。
        # 2026-11-04：原来是 1500（1.5s），上电后机械臂要在这 1.5 秒内走到保持角度，
        # 6 个舵机同时使劲、峰值电流过大 → 触发过流保护 → 整臂断电瘫软。
        # 改成 3000（3s）：同样的位移用一半的速度，峰值电流明显下降。
        # 想再柔就调环境变量 CELLO_START_MS=4000，不用改代码。
        # ⚠ 这个值只在"还没有轨迹指令"时用；正常运动用的是 msg.time[i] + damp_ms。
        self.interval = [int(os.environ.get("CELLO_START_MS", str(int(_GRIPPER["start_ms"]))))
                         for _ in range(self.Servo.SRV_NUM)]
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
            if id < 6:
                # 写方向补零位偏移：URDF 角度 → 舵机角度
                offset = self.Servo.ZERO_OFFSET_DEG[id]
                self.target_angle[id] = int(10 * (msg.target_angle[i] + offset))
            else:
                # ---- 夹爪：绝对角度不可用，改成「基准 + 相对量」 ----
                # 2026-11-07 实测：同一个绝对角（60°）两次起栈对应完全不同的
                # 物理状态（一次闭合、一次张到最大）——换爪断电后多圈零点被
                # 重置成了不同参照，**绝对角不可重复**。
                # 所以 msg 里的角度一律理解为"相对起栈位置走多少度"：
                #   0 = 起栈位置（约定：起栈前把两指拨到闭合）
                #   160 = 从闭合再往张开方向走 160°（= 实测全开 50.9mm）
                self.target_angle[id] = int(
                    10 * (self.Servo.grip_base_deg + msg.target_angle[i]))
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
                self.Servo.servoangle2jointstate(
                    servo_id=i, servo_angle=angle,
                    grip_base=self.Servo.grip_base_deg))

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
        # 最后一个字段是**功率上限（mW）**，不是速度。
        # 原来写死 8000（8W）。2026-11-01 换爪后新爪阻力明显更大：
        # 断电手动能开合 65mm，通电却只走 33°（两指 19mm）就停住、并且发烫
        # —— 典型"全力顶但推不动"，也就是力矩不够。
        # 改成可用环境变量调，不用重编译：
        #   CELLO_POWER_MW=12000（默认）/ 8000（回到旧行为）/ 16000（再猛一档）
        # ⚠ 别盲目往上加：真堵转时功率越大电流越大、烧得越快。
        power_mw = int(os.environ.get("CELLO_POWER_MW", str(int(_GRIPPER["power_mw"]))))
        command_data_list = [struct.pack("<BlLHHH", i, self.target_angle[i], self.interval[i], acc[i], acc[i], power_mw) for i in range(self.Servo.SRV_NUM)]
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

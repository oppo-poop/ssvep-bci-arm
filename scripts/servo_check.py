#!/usr/bin/env python3
# coding: utf-8
"""7 舵机串口检查：一次跑完，判断整条串口链路是否健康。

为什么需要这个
    "臂不动" 有四五种完全不同的原因，而且表现都一样：
        · 串口没插 / 设备名变了
        · 没权限（不在 dialout 组）
        · robo_driver 还在跑，串口被独占
        · 波特率不对（1M 还是 115200）
        · 某个舵机掉线 / ID 不对
        · 电压不够，舵机反复复位
    逐个猜要几十分钟。这个脚本按顺序把它们分开，一次告出是哪一层。

它检查什么（五层，从下往上）
    1. 设备层   /dev/ttyUSB0 存在？权限够？被谁占着？
    2. 链路层   哪个波特率能通？
    3. 总线层   7 个 ID 谁在线？ping 往返多少？丢包多少？
    4. 数据层   角度是否合理？夹爪开度换算多少？
    5. 供电层   电压、温度、状态字

⚠ 关于"响应时间"这个指标的坑
    SDK 的 ping() / query_*() 内部都是 `update(wait_response=True)`，而 update 的
    循环是"没有新数据就一直转，直到 timeout"，**没有提前 break**，默认 timeout
    正好 0.1 s。所以用它们计时，无论舵机多快，测出来永远是 100 ms。

    本脚本自己发包、自己轮询（timed_ping），1 Mbaud 下正常值 1~5 ms。
    这个数字才有诊断价值：某一路到 30 ms，就是那根线/那个接口开始劣化了。

这个脚本绝不做的
    · 不发任何运动指令（不调 set_angle / send_sync_*）
    · 不设零点（不调 set_origin_point）
    · 不调 reset_multi_turn_angle（会清多圈计数）
    · 不启动 robo_driver —— 它 __init__ 里就会驱到零位

依赖
    pip install fashionstar-uart-sdk pyserial

用法
    # 机械臂必须停掉（robo_driver 会独占串口）
    python3 scripts/servo_check.py

    python3 scripts/servo_check.py --ping-only      # 只测在线，约 1 秒
    python3 scripts/servo_check.py --samples 5      # 丢包率采样 5 轮（默认 3）
    python3 scripts/servo_check.py --baud 115200    # 指定波特率，跳过探测
    python3 scripts/servo_check.py --scan           # 额外扫 ID 7~15，看有没有多余的
    python3 scripts/servo_check.py --json           # 机器可读，给 CI / 上位机
    python3 scripts/servo_check.py --mock           # 无硬件自测（假数据，验证判据与输出）

退出码
    0  全部通过
    1  有警告（可以跑，但要知道）
    2  有失败（不能跑：缺舵机 / 打不开串口）
"""

import argparse
import glob
import json
import os
import statistics
import sys
import time

# ---------------------------------------------------------------- 常量

SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4",
               "joint5", "joint6", "joint7(夹爪)"]

#: 夹爪舵机的零位读数与换算（与 robo_driver 的 servoangle2jointstate 一致）
#:   pos(m) = (deg - 100) / 100 * 0.032
#:   开度(mm) = 8 + (pos + 0.025) / 0.025 * 57      → 8 mm(合到底) ~ 65 mm(全开)
GRIP_ZERO_DEG = 100.0
GRIP_POS_MIN = -0.025
GRIP_GAP_MIN_MM = 8.0
GRIP_GAP_MAX_MM = 65.0
#: 夹爪舵机的可用角区间（2026-10 卡尺 + 行程实测）。超出就说明没校准或读数异常。
GRIP_DEG_RANGE = (21.0, 101.0)

#: 各舵机的读数是否应该接近 0（前六个）。这里只做"明显跑飞"的粗判，
#: 精确的零位偏差是 zero_calib.py 的活，不是这个脚本的。
ARM_DEG_SANE = (-360.0, 360.0)

#: 判据阈值。**按你的电池/负载改这三行**，别照抄。
#: 本机是幻尔 RX8/RA8 系列（12 V 供电），实测满电 12.0~12.2 V。
#: 若换成 2S 锂电（7.4 V 标称）供的舵机，改成 7.0 / 6.4。
VOLT_WARN = 11.0         # 低于此值：3S 锂电约剩 20% / 线损过大
VOLT_FAIL = 10.2         # 低于此值：舵机会反复复位，读数不可信
TEMP_WARN = 60.0         # ℃
TEMP_FAIL = 75.0         # ℃
#: 真实 ping 往返（ms）。1 Mbaud 下一帧 6 字节，正常 1~5 ms。
#: 这个值**必须自己计时**才能拿到 —— SDK 的 query/ping 会死等满 100 ms，
#: 用它算出来永远是 100 ms（见 timed_ping 的注释）。
LATENCY_WARN_MS = 20.0
LATENCY_FAIL_MS = 60.0   # 超过这个基本等于链路要断了
PING_WINDOW_MS = 80.0    # 等响应的窗口，超过判为没响应

#: 端口与波特率（robo_driver 用 1000000，官方例程写 115200，两个都试）
DEFAULT_PORT = "/dev/ttyUSB0"
CANDIDATE_BAUDS = [1000000, 115200]

OK, WARN, FAIL = "ok", "warn", "fail"


# ---------------------------------------------------------------- 前置检查

def find_port_users(port):
    """扫 /proc/*/fd，找出打开了这个串口的进程。

    为什么不用 lsof：不是每台机器都装了，而且它需要额外权限。
    直接读 /proc 更可靠，报错信息也能直接给出进程名。

    ⚠ 局限：别人的进程（比如 root 起的 robo_driver）读不到 /proc/PID/fd，
    会被跳过。这种情况表现为"没人占用但打不开"，那时看 pyserial 的报错。

    Returns: [(pid, cmdline), ...]
    """
    target = os.path.realpath(port)
    users = []
    for pid_dir in glob.glob("/proc/[0-9]*"):
        fd_dir = os.path.join(pid_dir, "fd")
        try:
            fds = os.listdir(fd_dir)
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
        hit = False
        for fd in fds:
            try:
                # 两边都 realpath：进程可能通过 /dev/serial/by-id/xxx 打开，
                # 直接字符串比对会漏判。
                if os.path.realpath(os.readlink(os.path.join(fd_dir, fd))) == target:
                    hit = True
                    break
            except OSError:
                continue
        if not hit:
            continue
        pid = os.path.basename(pid_dir)
        try:
            with open(os.path.join(pid_dir, "cmdline"), "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        except OSError:
            cmd = "(读不到命令行)"
        users.append((pid, cmd[:80]))
    return users


def check_device(port):
    """设备层检查。返回 (ok, 说明, 阻断原因)。"""
    if not os.path.exists(port):
        near = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        msg = "设备不存在：%s" % port
        if near:
            msg += "\n    找到了这些，可能就是它：%s" % " ".join(near)
        else:
            msg += "\n    一个 ttyUSB/ttyACM 都没有 —— 检查 USB 线和 UC-01 供电"
        return False, msg, "no_device"

    if not os.access(port, os.R_OK | os.W_OK):
        grp = "dialout"
        return False, ("没有读写权限：%s\n"
                       "    当前用户组：%s\n"
                       "    修：sudo usermod -aG %s $USER   然后重新登录"
                       % (port, os.getgroups(), grp)), "no_perm"

    users = find_port_users(port)
    if users:
        detail = "\n".join("    pid %s  %s" % (p, c) for p, c in users)
        return False, ("串口被占用：%s\n%s\n"
                       "    robo_driver 会独占串口。先停掉机械臂再检查：\n"
                       "      bash scripts/robot_up.sh --stop   （或 kill 掉那些进程）"
                       % (port, detail)), "busy"

    st = os.stat(port)
    return True, "设备就绪：%s（权限正常，没被占用）" % port, None


# ---------------------------------------------------------------- 串口

def load_sdk():
    try:
        import fashionstar_uart_sdk as uservo
        return uservo
    except ImportError:
        print("✗ 缺少舵机 SDK。先装：pip install fashionstar-uart-sdk", file=sys.stderr)
        sys.exit(2)


def open_bus(port, baud=None, probe_timeout=0.2):
    """打开串口并探测波特率。

    探测方式：能 ping 通**任意一个**舵机就认为这个波特率对。
    返回 (mgr, uart, baud, 探测日志)；失败时 mgr 为 None。
    """
    import serial

    uservo = load_sdk()
    bauds = [baud] if baud else CANDIDATE_BAUDS
    log = []
    last_err = None

    for b in bauds:
        try:
            uart = serial.Serial(port=port, baudrate=b, parity=serial.PARITY_NONE,
                                 stopbits=1, bytesize=8, timeout=probe_timeout)
        except Exception as exc:                      # noqa: BLE001
            last_err = exc
            log.append("%d: 打不开（%s）" % (b, exc))
            continue

        try:
            mgr = uservo.UartServoManager(uart)
        except Exception as exc:                      # noqa: BLE001
            last_err = exc
            log.append("%d: SDK 初始化失败（%s）" % (b, exc))
            uart.close()
            continue

        # 探测：随便哪个舵机回应，就说明这个波特率是对的
        found = None
        for sid in SERVO_IDS:
            t0 = time.time()
            try:
                if mgr.ping(sid):
                    found = (sid, (time.time() - t0) * 1000)
                    break
            except Exception:                         # noqa: BLE001
                pass
        if found:
            log.append("%d: ✓ 舵机 %d 有响应（%.0f ms）" % (b, found[0], found[1]))
            return mgr, uart, b, log
        log.append("%d: 无响应" % b)
        uart.close()
        time.sleep(0.1)

    log.append("最后一个错误：%s" % last_err if last_err else "都试过了，全无响应")
    return None, None, None, log


def timed_ping(mgr, sid, window_ms=None):
    """精确测一次 ping 往返（ms）。没响应返回 None。

    为什么不用 mgr.ping()
        它的实现是 `update(wait_response=True)`，而 update 的循环是
        "没有新数据就继续转，直到 timeout"——**没有提前 break**。
        默认 timeout=0.1，所以无论舵机多快，测出来永远是 100 ms。
        2026-11-02 真机跑出来 7 个全是 100 ms，就是踩的这个。

        这里自己发帧、自己轮询，收到响应立刻返回，测的才是真实往返。

    内部结构（pkt_buffer / CODE_PING）来自 SDK 的公开属性；万一哪天 SDK 改了，
    由调用方捕获异常回退到 SDK 的 ping。
    """
    import struct
    import fashionstar_uart_sdk as uservo

    if window_ms is None:
        window_ms = PING_WINDOW_MS

    # 清掉上一次的残留，否则可能读到别的舵机的帧而误判
    try:
        mgr.uart.reset_input_buffer()
    except Exception:                                 # noqa: BLE001
        pass
    try:
        mgr.pkt_buffer.empty_buffer()
    except Exception:                                 # noqa: BLE001
        pass

    mgr.send_request(mgr.CODE_PING, struct.pack("<B", sid))

    t0 = time.perf_counter()
    deadline = t0 + window_ms / 1000.0
    while time.perf_counter() < deadline:
        n = mgr.uart.in_waiting
        if n:
            data = mgr.uart.read(n)
            for b in data:
                mgr.pkt_buffer.update(bytes([b]))
            while mgr.pkt_buffer.has_valid_packet():
                pkt = mgr.pkt_buffer.get_packet()
                res = uservo.Packet.unpack(pkt)
                if not res:
                    continue
                code, param = res
                # 只认这个 ID 的 PING 响应；残留帧的 code/param 对不上
                if code == mgr.CODE_PING and param and param[0] == sid:
                    elapsed = (time.perf_counter() - t0) * 1000.0
                    try:
                        mgr.response_ping(param)
                    except Exception:                 # noqa: BLE001
                        pass
                    return elapsed
        time.sleep(0.0005)        # 0.5 ms 轮询，1 Mbaud 下一帧约 0.06 ms
    return None


def read_one(mgr, sid, samples=1, ping_only=False):
    """读一个舵机的完整状态，多次采样。

    Returns: dict，字段缺失即为 None（读失败）。
    """
    angles, mturns, volts, temps, status = [], [], [], [], []
    lat, fails = [], 0

    # 先 ping：它和"读角度失败"是两件事，要分开。
    # ping 通但角度读不到 = 数据链路异常（线质量/干扰）；
    # ping 不通 = 压根没这个舵机（ID 错 / 没上电 / 掉线）。
    try:
        ping_ms = timed_ping(mgr, sid)
    except Exception:                                 # noqa: BLE001
        # SDK 内部结构变了，退回它的 ping。
        # 注意写法：不能写成 `elapsed if mgr.ping() else None` —— 条件表达式
        # 先求条件，这样时间是 ping 返回之后才开始算的，永远是 0。
        t0 = time.perf_counter()
        try:
            ok = bool(mgr.ping(sid))
        except Exception:                             # noqa: BLE001
            ok = False
        ping_ms = (time.perf_counter() - t0) * 1000.0 if ok else None
    ping_ok = ping_ms is not None

    # 多 ping 几次取最大，偶发高延迟比中位数更能说明接触不良
    ping_max = ping_ms
    if ping_ok and not ping_only:
        for _ in range(min(2, max(0, samples - 1))):
            try:
                extra = timed_ping(mgr, sid)
            except Exception:                         # noqa: BLE001
                break
            if extra is not None and (ping_max is None or extra > ping_max):
                ping_max = extra
            time.sleep(0.005)

    if ping_only or not ping_ok:
        return {
            "id": sid, "online": ping_ok, "ping_ok": ping_ok,
            "angle": None, "mturn": None, "volt": None, "temp": None,
            "status": None,
            "latency_ms": ping_ms, "latency_max_ms": ping_max,
            "fails": 0 if ping_ok else max(1, samples),
            "samples": max(1, samples), "jitter": 0.0,
        }

    for i in range(max(1, samples)):
        t0 = time.perf_counter()
        try:
            a = mgr.query_servo_angle(sid)
        except Exception:                             # noqa: BLE001
            a = None
        dt = (time.perf_counter() - t0) * 1000
        if a is None:
            fails += 1
        else:
            angles.append(float(a))
            lat.append(dt)

        # 电压/温度/状态/多圈角变化慢，只读第一轮。
        # SDK 每个查询固定等 100 ms（见 timed_ping 注释），少读一次省 0.1 s。
        if i == 0:
            for bucket, fn in ((mturns, mgr.query_servo_mturn_angle),
                               (volts, mgr.query_voltage),
                               (temps, mgr.query_temperature),
                               (status, mgr.query_status)):
                try:
                    v = fn(sid)
                    if v is not None:
                        bucket.append(float(v))
                except Exception:                     # noqa: BLE001
                    pass
        if i < samples - 1:
            time.sleep(0.01)

    med = lambda xs: statistics.median(xs) if xs else None   # noqa: E731
    return {
        "id": sid,
        "online": bool(angles),
        "ping_ok": True,
        "angle": med(angles),
        "mturn": med(mturns),
        "volt": med(volts),
        "temp": med(temps),
        "status": med(status),
        "latency_ms": med(lat),
        "latency_max_ms": ping_max,
        "fails": fails,
        "samples": max(1, samples),
        # 角度抖动：静止时应该很小，大说明读数不稳或舵机在抖
        "jitter": (max(angles) - min(angles)) if len(angles) > 1 else 0.0,
    }


def grip_gap_mm(deg):
    """舵机角度 → 夹爪开度（mm）。与 robo_driver 的换算一致。"""
    pos = (deg - GRIP_ZERO_DEG) / 100.0 * 0.032
    ratio = (pos - GRIP_POS_MIN) / (0.0 - GRIP_POS_MIN)
    return GRIP_GAP_MIN_MM + ratio * (GRIP_GAP_MAX_MM - GRIP_GAP_MIN_MM)


def scan_extra(mgr, lo=7, hi=16):
    """扫一遍 ID 7~15，看总线上有没有计划外的舵机。

    为什么不用 refresh_srv_list：它会 ping 0~253，慢且会重建内部 servos 字典。
    """
    extra = []
    for sid in range(lo, hi):
        try:
            if mgr.ping(sid):
                extra.append(sid)
        except Exception:                             # noqa: BLE001
            pass
    return extra


# ---------------------------------------------------------------- 判据

def judge(row):
    """给一行结果判级。返回 (level, [问题...])。"""
    issues = []
    level = OK

    def up(new):
        nonlocal level
        order = {OK: 0, WARN: 1, FAIL: 2}
        if order[new] > order[level]:
            level = new

    if not row.get("ping_ok", row["online"]):
        return FAIL, ["无响应（检查总线/ID/供电）"]

    if not row["online"]:
        # ping 通但角度一次都没读到 —— 不是"没这个舵机"，是数据链路有问题
        return FAIL, ["ping 通但读不到角度（数据帧丢失：查线质量/干扰/波特率）"]

    if row["fails"]:
        up(WARN)
        issues.append("%d/%d 次读失败（丢包）" % (row["fails"], row["samples"]))

    lat = row.get("latency_max_ms") or row.get("latency_ms")
    if lat is not None:
        if lat > LATENCY_FAIL_MS:
            up(FAIL)
            issues.append("ping 往返 %.0f ms，链路快断了（正常 1~5 ms）" % lat)
        elif lat > LATENCY_WARN_MS:
            up(WARN)
            issues.append("ping 往返 %.0f ms，偏慢（正常 1~5 ms）" % lat)

    if row["jitter"] > 1.0:
        up(WARN)
        issues.append("角度抖动 %.1f°（静止时应该 <1°）" % row["jitter"])

    v = row["volt"]
    if v is not None:
        if v < VOLT_FAIL:
            up(FAIL)
            issues.append("电压 %.2f V，低于 %.1f V —— 舵机会反复复位" % (v, VOLT_FAIL))
        elif v < VOLT_WARN:
            up(WARN)
            issues.append("电压 %.2f V，偏低（<%.1f V）" % (v, VOLT_WARN))

    t = row["temp"]
    if t is not None:
        if t > TEMP_FAIL:
            up(FAIL)
            issues.append("温度 %.0f℃，超过 %.0f℃" % (t, TEMP_FAIL))
        elif t > TEMP_WARN:
            up(WARN)
            issues.append("温度 %.0f℃，偏高" % t)

    a = row["angle"]
    if a is not None:
        if row["id"] == 6:
            lo, hi = GRIP_DEG_RANGE
            if not (lo <= a <= hi):
                up(WARN)
                issues.append("夹爪角度 %.1f°，超出实测可用区间 %.0f~%.0f°"
                              "（没校准？）" % (a, lo, hi))
        else:
            lo, hi = ARM_DEG_SANE
            if not (lo <= a <= hi):
                up(WARN)
                issues.append("角度 %.1f° 明显异常" % a)

    return level, issues


# ---------------------------------------------------------------- 输出

def fmt(v, width=8, prec=2, suffix=""):
    if v is None:
        return " " * (width - 1) + "-"
    return "%*.{}f%s".format(prec) % (width, v, suffix)


def print_table(rows, levels):
    print("\n  ID  关节           角度(°)   多圈角(°)   电压(V)  温度(℃)  ping(ms)  判定")
    print("  " + "-" * 82)
    mark = {OK: "✓", WARN: "⚠", FAIL: "✗"}
    for r in rows:
        sid = r["id"]
        tag = mark[levels[sid][0]]
        print("  %-3d %-13s %s %s %s %s %s   %s"
              % (sid, JOINT_NAMES[sid],
                 fmt(r["angle"], 8, 2), fmt(r["mturn"], 10, 1),
                 fmt(r["volt"], 8, 2), fmt(r["temp"], 8, 1),
                 fmt(r.get("latency_max_ms") or r["latency_ms"], 8, 1), tag))

    # 夹爪单独给一行换算值 —— 角度对夹爪没意义，开度才有
    g = rows[6]
    if g["online"] and g["angle"] is not None:
        gap = grip_gap_mm(g["angle"])
        note = ""
        if gap <= GRIP_GAP_MIN_MM + 0.5:
            note = "（合到底）"
        elif gap >= GRIP_GAP_MAX_MM - 1.0:
            note = "（全开）"
        print("\n  夹爪开度：%.1f mm %s  ← 由 %.1f° 换算" % (gap, note, g["angle"]))


def mock_rows():
    """造一组假数据，用来验证判据和输出（不需要硬件）。"""
    base = [
        # id, angle, mturn, volt, temp, ping_ms, fails, jitter
        # 电压按本机实测的 12 V 系统（幻尔 RX8/RA8）
        (0, 0.4, 12.5, 12.04, 27.1, 1.8, 0, 0.05),
        (1, -90.5, -90.5, 12.24, 26.8, 1.6, 0, 0.08),
        (2, 89.5, 89.5, 11.98, 26.2, 2.0, 0, 0.03),
        (3, 2.3, 2.3, 11.82, 28.1, 14.0, 1, 0.60),
        (4, 32.2, 32.2, 11.93, 27.8, 1.9, 0, 0.04),
        (5, 0.2, 0.3, 11.85, 27.3, 2.2, 0, 0.06),
        (6, 88.9, 88.9, 11.82, 28.2, 1.7, 0, 0.10),
    ]
    rows = []
    for (sid, a, m, v, t, lat, f, j) in base:
        rows.append({"id": sid, "online": True, "ping_ok": True,
                     "angle": a, "mturn": m, "volt": v, "temp": t,
                     "status": 0, "latency_ms": lat, "latency_max_ms": lat,
                     "fails": f, "samples": 3, "jitter": j})
    return rows


# ---------------------------------------------------------------- 主流程

def run(args):
    print("═" * 62)
    print(" 7 舵机串口检查  %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print("═" * 62)

    result = {"port": args.port, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
              "servos": [], "warnings": [], "failures": []}

    # ---- 1. 设备层（--mock 跳过）
    if args.mock:
        print("\n[1/4] 设备层     --mock：跳过（用假数据）")
        dev_ok = True
    else:
        print("\n[1/4] 设备层")
        dev_ok, msg, why = check_device(args.port)
        print("  " + ("✓ " if dev_ok else "✗ ") + msg)
        result["device"] = {"ok": dev_ok, "detail": msg, "reason": why}
        if not dev_ok:
            result["failures"].append(msg.splitlines()[0])
            print("\n" + "─" * 62)
            print("✗ 设备层就不通，后面的检查没有意义。先解决上面这条。")
            return 2, result

    # ---- 2. 拿数据
    if args.mock:
        print("\n[2/4] 链路层     --mock：跳过")
        print("\n[3/4] 总线层     --mock：假数据")
        rows = mock_rows()
        baud = 1000000
        result["baud"] = baud
        extra = []
        uart = None
    else:
        print("\n[2/4] 链路层（探测波特率）")
        mgr, uart, baud, log = open_bus(args.port, args.baud)
        for ln in log:
            print("  " + ln)
        result["baud"] = baud
        if mgr is None:
            result["failures"].append("串口打不开或没有舵机响应")
            print("\n" + "─" * 62)
            print("✗ 链路层不通：串口能开但一个舵机都没回应。")
            print("  常见原因：UC-01 没供电 / A/B 线接反 / 波特率都不是这两个 /")
            print("            舵机压根没上电（总线电压 0）")
            return 2, result

        print("\n[3/4] 总线层（%d 个舵机 × %d 次采样）" % (len(SERVO_IDS), args.samples))
        rows = []
        for sid in SERVO_IDS:
            r = read_one(mgr, sid, samples=1 if args.ping_only else args.samples)
            rows.append(r)
            sys.stdout.write("\r  已查 %d/%d" % (sid + 1, len(SERVO_IDS)))
            sys.stdout.flush()
        print("\r  已查 %d/%d，完成      " % (len(SERVO_IDS), len(SERVO_IDS)))

        extra = scan_extra(mgr) if args.scan else []

    # ---- 3. 判据
    levels = {r["id"]: judge(r) for r in rows}

    print("\n[4/4] 数据层与供电层")
    print_table(rows, levels)

    if extra:
        print("\n  ⚠ 扫到计划外的舵机 ID：%s（本机只用 0~6）" % extra)
        result["warnings"].append("计划外 ID：%s" % extra)

    # ---- 4. 汇总
    for sid, (lvl, issues) in levels.items():
        for it in issues:
            line = "ID %d %s：%s" % (sid, JOINT_NAMES[sid], it)
            if lvl == FAIL:
                result["failures"].append(line)
            else:
                result["warnings"].append(line)

    result["servos"] = [{k: v for k, v in r.items()} for r in rows]
    result["grip_gap_mm"] = (round(grip_gap_mm(rows[6]["angle"]), 1)
                             if rows[6]["online"] and rows[6]["angle"] is not None
                             else None)

    online = [r["id"] for r in rows if r["online"]]
    missing = [i for i in SERVO_IDS if i not in online]

    print("\n  角度是舵机原始读数（未扣零位偏移），臂停在任意姿态都正常 ——")
    print("  \"在不在零位\"是 zero_calib.py 的判据，不是这里要判的。")

    print("\n" + "─" * 62)
    print("  在线：%d/%d %s" % (len(online), len(SERVO_IDS),
                             "" if not missing else "  缺 %s" % missing))
    if baud:
        print("  波特率：%d" % baud)

    n_fail = len(result["failures"])
    n_warn = len(result["warnings"])

    if n_fail:
        print("\n✗ %d 项失败：" % n_fail)
        for x in result["failures"]:
            print("    · " + x)
    if n_warn:
        print("\n⚠ %d 项警告：" % n_warn)
        for x in result["warnings"]:
            print("    · " + x)
    if not n_fail and not n_warn:
        print("\n✓ 全部通过，7 个舵机都能正常通信。")

    if uart is not None:
        try:
            uart.close()
        except Exception:                             # noqa: BLE001
            pass

    return (2 if n_fail else 1 if n_warn else 0), result


def main():
    ap = argparse.ArgumentParser(
        description="7 舵机串口检查（只读，不驱动）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="退出码：0 全通过 / 1 有警告 / 2 有失败")
    ap.add_argument("--port", default=DEFAULT_PORT, help="串口设备（默认 %s）" % DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=None,
                    help="波特率；不给就自动探测 %s" % CANDIDATE_BAUDS)
    ap.add_argument("--samples", type=int, default=3,
                    help="每个舵机采样次数，用于统计丢包与抖动（默认 3）")
    ap.add_argument("--ping-only", action="store_true",
                    help="只测在线，不做多次采样（最快，约 1 秒）")
    ap.add_argument("--scan", action="store_true",
                    help="额外扫 ID 7~15，看总线上有没有计划外的舵机")
    ap.add_argument("--json", action="store_true", help="额外输出 JSON（给 CI / 上位机）")
    ap.add_argument("--mock", action="store_true",
                    help="用假数据跑一遍，验证判据与输出（不需要硬件）")
    args = ap.parse_args()

    if args.samples < 1:
        ap.error("--samples 至少 1")

    if not args.json:
        code, _ = run(args)
        return code

    # --json：stdout 只放 JSON，人能读的那份走 stderr。
    # 否则 `servo_check.py --json | jq` 会被表格行噎住。
    real_stdout = sys.stdout
    sys.stdout = sys.stderr
    try:
        code, result = run(args)
    finally:
        sys.stdout = real_stdout
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return code


if __name__ == "__main__":
    sys.exit(main())

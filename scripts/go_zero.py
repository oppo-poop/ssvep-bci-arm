#!/usr/bin/env python3
# coding: utf-8
"""把机械臂逐步驱回标定零位。

与 robo_driver 启动流程的区别（也是为什么不用它来回零）：

    robo_driver.__init__            本脚本
    ─────────────────────────       ─────────────────────────
    reset_multi_turn_angle(0xff)    不动多圈计数
      └─ 内部先 disable_torque        └─ 全程保持力矩
         → 臂会掉 50ms
    一次发全部 7 个关节              每步最多 3°，读回再走下一步
    出错只能拔电                     Ctrl-C 立即停发新指令

用法
    python3 scripts/go_zero.py           # 干跑：只算差值，不动
    python3 scripts/go_zero.py --go      # 真的动（会先确认）

判据
    每个关节与零位的差都小于 TOL（默认 0.5°）即收敛。
"""

import argparse
import struct
import sys
import time

#: 与 robo_driver.uservo_ex.ZERO_OFFSET_DEG 保持一致（改一处必须同时改另一处）
ZERO_DEG = [0.4, -0.2, -0.1, 1.3, -0.3, 0.8]

#: 预设姿态（舵机读数，度）。park = 2026-10-18 16:28 记录的折叠收纳姿态。
PRESETS = {
    "zero": None,                                   # 用下面的 ZERO_DEG
    "park": [0.4, -90.6, 89.3, 2.4, -6.1, -1.7],
}

SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4",
               "joint5", "joint6", "joint7_left"]

STEP_DEG = 3.0      # 单步最大角度，越小越稳
STEP_MS = 2000      # 单步耗时，越大越慢越安全
#: 舵机稳态误差约 0.5~0.8°（位置伺服死区），容差定得比这更严只会死循环
TOL_DEG = 1.0
#: 连续 N 步没有改善就判定"卡在死区"，停止重发
STALL_STEPS = 3
#: 停滞时若差值仍在这个范围内，视为已到位（允许解锁）
STALL_ACCEPT_DEG = 2.0
MAX_STEPS = 20

DEFAULT_PORT = "/dev/ttyUSB0"
CANDIDATE_BAUDS = [1000000, 115200]


def die(msg, code=2):
    print("✗ " + msg, file=sys.stderr)
    sys.exit(code)


def load_sdk():
    try:
        import fashionstar_uart_sdk as uservo
        return uservo
    except ImportError:
        die("缺少舵机 SDK：pip install fashionstar-uart-sdk")


def open_manager(uservo, port, baud=None):
    import os
    import serial

    if not os.path.exists(port):
        die("串口不存在：%s\n  ls /dev/ttyUSB* 看看设备名" % port)
    for b in ([baud] if baud else CANDIDATE_BAUDS):
        try:
            uart = serial.Serial(port=port, baudrate=b, parity=serial.PARITY_NONE,
                                 stopbits=1, bytesize=8, timeout=0.2)
            mgr = uservo.UartServoManager(uart)
            if mgr.query_servo_angle(0) is not None:
                print("✓ %s @ %d baud" % (port, b))
                return mgr, uart
            uart.close()
        except Exception:  # noqa: BLE001
            continue
    die("连不上舵机（试过 %s）" % ([baud] if baud else CANDIDATE_BAUDS))


def read_angles(mgr):
    out = []
    for sid in SERVO_IDS:
        try:
            out.append(mgr.query_servo_angle(sid))
        except Exception:  # noqa: BLE001
            out.append(None)
    return out


def send_targets(mgr, degs, interval_ms):
    """一次下发 7 个目标角度（与 robo_driver.arm_move_by_time 同格式）。"""
    acc = interval_ms // 2
    data = [struct.pack("<BlLHHH", i, int(10 * degs[i]), interval_ms, acc, acc, 8000)
            for i in range(7)]
    mgr.send_sync_multiturnanglebyinterval(
        mgr.CODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL, 7, data)


def unlock_all(mgr):
    """解除全部舵机扭矩。调用后臂会因重力下落，只能手动托着。"""
    print("\n解除全部舵机扭矩…")
    mgr.disable_torque(0xFF)
    time.sleep(0.2)
    print("⚠ 已无力矩：臂会因重力下落。可以手动摆位，然后断电。")


def main():
    ap = argparse.ArgumentParser(description="逐步驱回标定零位")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int)
    ap.add_argument("--go", action="store_true", help="真的驱动（默认只算差值）")
    ap.add_argument("--target", nargs=6, type=float, metavar="DEG",
                    help="6 个关节的目标角度（度）；不给则用标定零位")
    ap.add_argument("--preset", choices=["zero", "park"],
                    help="预设：zero=标定零位，park=折叠收纳姿态")
    ap.add_argument("--unlock", action="store_true",
                    help="到达后解除全部扭矩（收纳/断电前用；臂会掉）")
    ap.add_argument("--step-deg", type=float, default=STEP_DEG)
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS,
                    help="最多走几步；每步最大 --step-deg 度。默认 %d" % MAX_STEPS)
    ap.add_argument("--tol", type=float, default=TOL_DEG)
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    args = ap.parse_args()

    if args.preset and args.target:
        die("--preset 和 --target 只能给一个")
    if args.preset == "park":
        goal = list(PRESETS["park"])
    elif args.target:
        goal = list(args.target)
    else:
        goal = list(ZERO_DEG)

    uservo = load_sdk()
    mgr, uart = open_manager(uservo, args.port, args.baud)

    try:
        cur = read_angles(mgr)
        if any(c is None for c in cur[:6]):
            die("有舵机读不到角度：%s（检查总线/供电）" % cur)

        print("\n%-12s %8s %8s %8s" % ("关节", "当前", "目标", "差值"))
        print("  " + "-" * 40)
        worst = 0.0
        for i in range(6):
            d = goal[i] - cur[i]
            worst = max(worst, abs(d))
            print("  %-10s %+8.2f %+8.2f %+8.2f"
                  % (JOINT_NAMES[i], cur[i], goal[i], d))
        print("  %-10s %+8.2f %+8.2f %+8s"
              % (JOINT_NAMES[6], cur[6] if cur[6] is not None else float("nan"),
                 cur[6] if cur[6] is not None else float("nan"), "保持"))

        if worst < args.tol:
            print("\n✓ 最大差 %.2f° < %.1f°，已经在目标位，不需要动。" % (worst, args.tol))
            if args.unlock:
                unlock_all(mgr)
            return 0

        n_steps = min(args.max_steps, int(worst / args.step_deg) + 1)
        print("\n最大差 %.2f°，按每步 %.1f° 估算需要约 %d 步，总计约 %.0f 秒。"
              % (worst, args.step_deg, n_steps, n_steps * STEP_MS / 1000.0))

        if not args.go:
            print("\n（干跑结束。加 --go 才会真的动。）")
            return 0

        if not args.yes:
            print("\n即将驱动机械臂。确认周围无人、电源开关在手边。")
            if input("继续？[y/N] ").strip().lower() not in ("y", "yes"):
                print("已取消")
                return 1

        # 夹爪保持在当前值，不参与回零
        hold_j7 = cur[6] if cur[6] is not None else 50.0

        converged = False
        prev_worst = None
        stall = 0
        for step in range(1, args.max_steps + 1):
            cur = read_angles(mgr)
            if any(c is None for c in cur[:6]):
                die("第 %d 步读不到角度，停止（不做任何补偿）" % step)
            diff = [goal[i] - cur[i] for i in range(6)]
            worst = max(abs(d) for d in diff)
            if worst < args.tol:
                print("\n✓ 第 %d 步后收敛，最大差 %.2f°" % (step - 1, worst))
                converged = True
                break

            # 停滞检测：舵机有死区，逼近到一定程度就不再改善，继续发只会空转
            if prev_worst is not None and worst > prev_worst - 0.05:
                stall += 1
                if stall >= STALL_STEPS:
                    print("\n⚠ 连续 %d 步没有改善（最大差 %.2f°），"
                          "判定已到舵机稳态极限，停止重发。" % (stall, worst))
                    break
            else:
                stall = 0
            prev_worst = worst

            target = [cur[i] + max(-args.step_deg, min(args.step_deg, diff[i]))
                      for i in range(6)]
            target.append(hold_j7)
            print("  步 %2d  目标: %s  最大差 %.2f°"
                  % (step, " ".join("%+6.2f" % t for t in target[:6]), worst))
            send_targets(mgr, target, STEP_MS)
            time.sleep(STEP_MS / 1000.0 + 0.3)
        else:
            print("\n⚠ 达到 %d 步上限仍未收敛，停在当前位置。再跑一次会继续逼近。" % args.max_steps)

        final = read_angles(mgr)
        print("\n最终读数：")
        for i in range(6):
            d = goal[i] - final[i]
            print("  %-10s %+7.2f  差 %+5.2f°  %s"
                  % (JOINT_NAMES[i], final[i], d, "" if abs(d) < args.tol else "← 未收敛"))

        final_worst = max(abs(goal[i] - final[i]) for i in range(6))
        if args.unlock:
            if converged or final_worst <= STALL_ACCEPT_DEG:
                if not converged:
                    print("\n（差值 %.2f° 在可接受范围内，视为已到位）" % final_worst)
                unlock_all(mgr)
            else:
                print("\n⚠ 距目标还差 %.2f°，**不解锁**（避免臂在半路掉下来）。"
                      % final_worst)
        return 0
    except KeyboardInterrupt:
        print("\n\n中断：已停止下发新指令。舵机会走完当前这一小步（≤%.0f°）。" % args.step_deg)
        return 130
    finally:
        try:
            uart.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())

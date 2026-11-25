#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘模拟 SSVEP —— 用键盘代替脑电解码，驱动机械臂。

用途
    演示 / 联调时不想戴电极：键盘输入代替 SSVEP 的类别输出。
    **走的是和真脑电完全相同的链路**（同一个 socket、同一份 8 指令表、
    同样的忙时丢弃与 STOP 语义），所以真机上验过的东西，换回脑电时行为一致。

    为什么不直接调 pick_fixed.py：那样就绕开了 arm_service 那一层，
    等于跳过了真正要验的东西（忙时丢弃、holding、STOP）。

前置
    bash scripts/robot_up.sh                      # ROS 栈
    ROARM_FIXED=1 python3 scripts/arm_service.py  # 服务（固定位置模式）
    python3 scripts/kb_arm.py                     # 本脚本

按键
    0 / 1 / 2     取 红 / 绿 / 蓝
    3 / 4 / 5     放 红 / 绿 / 蓝
    6             回位（home）
    7             停止（当前动作 + 回零位）
    q             退出
    ?             帮助

模拟注视时长（--dwell）
    真 SSVEP 要"持续注视 N 秒"才确认。默认 0 = 按下即发（调试快）。
    演示时用 `--dwell 1.5`：按下后倒计时 1.5 秒才生效，中途按任意键取消
    —— 手感和真脑电更接近，也能演示"瞟一眼不算"。

退出码
    0 正常退出    2 连不上服务
"""

import argparse
import json
import os
import select
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

_ROOT = next(p for p in Path(__file__).resolve().parents if (p / "app" / "core" / "paths.py").is_file())
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from app.core import paths  # noqa: E402

DEFAULT_SOCKET = paths.ARM_SOCK

#: 键 → 指令。顺序与 ssvep-bci/config.py 的 ARM_NAMES_COLORS、arm_service 的
#: INDICES 完全一致。改这里必须同步那两处，否则"脑电按下 0 取红、键盘按下 0
#: 取绿"这种错会很难查。
KEYS = {
    "0": ("pick red", "取 红"),
    "1": ("pick green", "取 绿"),
    "2": ("pick blue", "取 蓝"),
    "3": ("place red", "放 红"),
    "4": ("place green", "放 绿"),
    "5": ("place blue", "放 蓝"),
    "6": ("home", "回位"),
    "7": ("stop", "停止"),
}

HELP = """
  0/1/2  取 红 / 绿 / 蓝        3/4/5  放 红 / 绿 / 蓝
  6      回位（home）           7      停止（STOP）
  q      退出                   ?      这份帮助
"""

STATE_CN = {
    "ok": "✓ 完成",
    "stopped": "■ 已停止",
    "busy": "… 忙，指令被丢弃",
    "no_object": "✗ 手上没有物料",
    "no_tray": "✗ 找不到托盘",
    "failed": "✗ 失败",
    "error": "✗ 出错",
    "aborted": "⚡ 断连自停（已收爪，原地保持）",
}


# ---------------------------------------------------------------- 终端按键
def read_key(prompt=""):
    """读一个字符，不等待回车。

    非 tty（管道/重定向）时退化成按行读，方便脚本化测试。
    """
    if prompt:
        print(prompt, end="", flush=True)
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        return (line.strip()[:1] or "q")
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def dwell(seconds):
    """模拟注视确认：倒计时，期间按任意键 = 取消。返回 True 表示确认。

    非 tty（管道/脚本化测试）直接放行：那种输入下 select 立刻可读，
    会把每一次都判成"取消"。
    """
    if seconds <= 0 or not sys.stdin.isatty():
        return True
    t0 = time.time()
    while True:
        left = seconds - (time.time() - t0)
        if left <= 0:
            print("\r  ⏳ 确认        " + " " * 12, flush=True)
            return True
        print("\r  ⏳ %.1f s（任意键取消）" % left, end="", flush=True)
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            sys.stdin.read(1)
            print("\r  ✗ 已取消      " + " " * 12, flush=True)
            return False


# ---------------------------------------------------------------- 通信
def send(sock_path, line, timeout=600):
    """发一条指令，等一行 JSON 回复。服务端是「一连接一指令」。"""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(sock_path)
    except OSError as exc:
        return None, "连不上 %s：%s\n  先启动：python3 scripts/arm_service.py" % (
            sock_path, exc)
    try:
        s.sendall((line + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    except socket.timeout:
        return None, "等回复超时（%ds）" % timeout
    finally:
        s.close()
    if not buf:
        return None, "服务没有返回"
    try:
        return json.loads(buf.decode("utf-8", "replace")), None
    except Exception:                                   # noqa: BLE001
        return None, buf.decode("utf-8", "replace")


def show(resp):
    """把响应打印成人看得懂的一行。

    失败时显示 detail 的**尾部**：pick_fixed 会先打计划再打每一步，
    真正出错的那行在最后，截前面等于什么都没看到。
    """
    if resp is None:
        return
    state = resp.get("state", "?")
    label = STATE_CN.get(state, state)
    detail = (resp.get("detail") or "").replace("\n", " ")
    if len(detail) > 240:
        detail = ("…" + detail[-240:]) if state in ("failed", "error") \
            else (detail[:240] + "…")
    holding = resp.get("holding")
    print("  %s %s" % (label, detail.strip()))
    if holding:
        print("  ⚠ 物料还在爪里（holding=%s）。要松手："
              "python3 scripts/pick_fixed.py --release --no-home" % holding)


# ---------------------------------------------------------------- 主循环
def main():
    ap = argparse.ArgumentParser(
        description="键盘模拟 SSVEP，通过 arm_service 驱动机械臂")
    ap.add_argument("--socket", default=str(DEFAULT_SOCKET),
                    help="arm_service 的 Unix socket（默认 %s）" % DEFAULT_SOCKET)
    ap.add_argument("--dwell", type=float, default=0.0, metavar="SEC",
                    help="模拟注视时长：按下后等 N 秒才生效，期间按键取消。"
                         "默认 0（按下即发）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要发的指令，不连服务")
    args = ap.parse_args()

    sock = Path(args.socket)
    if not args.dry_run and not sock.exists():
        print("✗ 找不到 socket：%s" % sock, file=sys.stderr)
        print("  先启动服务：ROARM_FIXED=1 python3 scripts/arm_service.py",
              file=sys.stderr)
        return 2

    print("键盘模拟 SSVEP → arm_service")
    print("  socket: %s%s" % (sock, "（--dry-run，不发送）" if args.dry_run else ""))
    if args.dwell > 0:
        print("  模拟注视：按住/按下后 %.1f 秒确认，中途按键取消" % args.dwell)
    print(HELP)

    try:
        while True:
            k = read_key("\n指令 > ")
            if not k:
                continue
            k = k.lower()
            if k in ("q", "quit", "exit", "\x03", "\x04"):
                print("\n退出。")
                return 0
            if k in ("?", "h"):
                print(HELP)
                continue
            if k in ("\n", "\r", " "):
                continue
            if k not in KEYS:
                print("  未知按键 %r（? 看帮助）" % k)
                continue

            line, name = KEYS[k]
            print("  → %s   [%s]" % (name, line))
            if not dwell(args.dwell):
                continue
            if args.dry_run:
                print("  （--dry-run：不发送）")
                continue
            t0 = time.time()
            resp, err = send(str(sock), line)
            if err:
                print("  ✗ " + err)
                continue
            show(resp)
            print("  （%.1f s）" % (time.time() - t0))
    except KeyboardInterrupt:
        print("\n中断退出（机械臂可能还在动，需要时发 stop）")
        return 0


if __name__ == "__main__":
    sys.exit(main())

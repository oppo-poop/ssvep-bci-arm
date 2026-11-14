#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""断连自停的离线测试。

场景
    客户端连上 → 发一个「长动作」→ 0.8 秒后直接关掉 socket（模拟脑电侧崩溃/拔线）。

预期
    1. 服务发现连接断开（handle() 里的 _peer_problem 返回 "断开"）
    2. 给当前动作发 SIGINT
    3. 收爪（`grip close`）
    4. **不回 home** —— 原地保持
    5. busy 复位，服务不崩

为什么不用真机
    这个测的是"监控连接→打断"这条控制流，和机械臂动不动无关。
    用假 Controller：`_pick` 里 sleep 3 秒当长动作，`_ros` 只记调用不执行；
    进程用假对象，能记录 SIGINT 是否发出。

用法
    python3 scripts/checks/check_disconnect.py
退出码 0 通过 / 1 失败
"""

import os
import signal
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import arm_service as S  # noqa: E402


class FakeProc:
    """假子进程：只记录信号，不真的跑。"""

    def __init__(self):
        self.signals = []
        self.alive = True
        self.pid = 0

    def poll(self):
        return None if self.alive else 0

    def send_signal(self, sig):
        self.signals.append(sig)

    def wait(self, timeout=None):
        self.alive = False
        return 0

    def terminate(self):
        self.alive = False

    def kill(self):
        self.alive = False


class FakeController(S.Controller):
    """长动作 + 记录所有 roarm_cli 调用。"""

    def __init__(self):
        super().__init__(tray_file=str(ROOT / "data" / "vision" / "trays.json"))
        self.ros_calls = []
        self.abort_calls = []
        self._fake = None

    def _ros(self, args, timeout):
        self.ros_calls.append(list(args))
        return 0, "fake ros ok"

    def abort(self, reason="客户端断开"):
        self.abort_calls.append(reason)
        return super().abort(reason)

    def _pick(self, color, label):
        # 模拟一个 30~60s 的动作：这里压到 3s，够测试用
        self._fake = FakeProc()
        self.proc = self._fake
        time.sleep(3.0)
        self.proc = None
        return self._resp(color, "ok", "fake pick done")


def main():
    ctrl = FakeController()
    sock_path = Path(tempfile.mkdtemp()) / "test_arm.sock"
    server = S._Server(str(sock_path), S._Handler)
    server.controller = ctrl
    th = threading.Thread(target=server.serve_forever, daemon=True)
    th.start()

    bad = []
    try:
        # ---- 客户端：连上、发指令、0.8s 后断开
        t0 = time.time()
        c = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        c.connect(str(sock_path))
        c.sendall(b"pick red\n")
        time.sleep(0.8)          # 让动作真的跑起来
        c.close()
        closed_at = time.time()

        # ---- 等服务检测 + 打断（_pick 还有 2 秒多才自然结束）
        deadline = time.time() + 15
        while time.time() < deadline:
            if ctrl.abort_calls:
                break
            time.sleep(0.1)

        took = time.time() - closed_at
        print("  客户端断开 → 服务 abort 用了 %.1f s" % took)
        if not ctrl.abort_calls:
            bad.append("断开后没有触发 abort（服务没检测到断连）")
        elif took > 3.0:
            bad.append("检测太慢（%.1f s），会拖到动作自己跑完" % took)
        else:
            print("  ✓ 检测到断开并打断（%.1f s）" % took)

        # ---- 打断方式：SIGINT
        if ctrl._fake is not None:
            if signal.SIGINT in ctrl._fake.signals:
                print("  ✓ 已给子进程发 SIGINT")
            else:
                bad.append("没给子进程发 SIGINT（signals=%r）"
                           % (ctrl._fake.signals,))

        # ---- 收爪，且不回 home
        names = [a[0] if a else "" for a in ctrl.ros_calls]
        if any(n == "grip" for n in names):
            print("  ✓ 已收爪（grip close）")
        else:
            bad.append("没调 grip close，物料可能掉（calls=%r）" % (names,))
        if any(n == "home" for n in names):
            bad.append("回 home 了 —— 断连时应原地保持（calls=%r）" % (names,))
        else:
            print("  ✓ 没有回 home（原地保持）")

        # ---- busy 复位
        time.sleep(0.5)
        if ctrl.busy:
            bad.append("busy 没复位，后续指令会被当成忙丢弃")
        else:
            print("  ✓ busy 已复位")

        print("  总耗时 %.1f s" % (time.time() - t0))
    finally:
        server.shutdown()
        server.server_close()
        try:
            sock_path.unlink()
        except OSError:
            pass

    if bad:
        print("\n✗ %d 项不通过：" % len(bad))
        for b in bad:
            print("    - " + b)
        return 1
    print("\n✓ 断连自停工作正常：断开→SIGINT→收爪→原地保持")
    return 0


if __name__ == "__main__":
    sys.exit(main())

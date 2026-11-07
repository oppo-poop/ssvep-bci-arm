#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_service 的客户端：发一条指令并等待结果。

    python3 scripts/arm_client.py 0
    python3 scripts/arm_client.py pick red
    python3 scripts/arm_client.py place green
    python3 scripts/arm_client.py home
    python3 scripts/arm_client.py stop

退出码：0 = ok/stopped，1 = failed/busy/no_object/no_tray/error，2 = 连不上。
"""

import argparse
import json
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from roarm_lib import paths  # noqa: E402

DEFAULT_SOCKET = paths.DATA / "arm.sock"


def main():
    ap = argparse.ArgumentParser(description="向 arm_service 发一条指令")
    ap.add_argument("command", nargs="+", help="如 0 / pick red / place green / home / stop")
    ap.add_argument("--socket", default=str(DEFAULT_SOCKET))
    args = ap.parse_args()
    line = " ".join(args.command) + "\n"
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        s.connect(args.socket)
    except OSError as exc:
        print("✗ 连不上 %s：%s" % (args.socket, exc), file=sys.stderr)
        print("  先启动：python3 scripts/arm_service.py", file=sys.stderr)
        return 2
    try:
        s.sendall(line.encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    finally:
        s.close()
    if not buf:
        print("✗ 服务没有返回", file=sys.stderr)
        return 1
    try:
        resp = json.loads(buf.decode("utf-8", "replace"))
    except Exception:
        print(buf.decode("utf-8", "replace"), end="")
        return 1
    print(json.dumps(resp, ensure_ascii=False))
    return 0 if resp.get("state") in ("ok", "stopped") else 1


if __name__ == "__main__":
    sys.exit(main())
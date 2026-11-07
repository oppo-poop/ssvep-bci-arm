#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""8 指令机械臂服务（给 ssvep-bci 的稳定接口）。

指令表（与 ssvep-bci/config.py 的 ARM_NAMES_COLORS 完全一致）：

    k=0 取红色物料      0 / pick red
    k=1 取绿色物料      1 / pick green
    k=2 取蓝色物料      2 / pick blue
    k=3 放到红色托盘    3 / place red
    k=4 放到绿色托盘    4 / place green
    k=5 放到蓝色托盘    5 / place blue
    k=6 回位/收纳       home / storage
    k=7 停止当前动作并回零位   stop

协议：Unix socket，一行一个 JSON（也接受纯文本）：
    请求  {"k":0}  或  {"cmd":"pick","color":"red"}  或  pick red
    响应  {"seq":1,"cmd":"pick red","state":"ok","detail":"..."}

忙时丢弃（STOP 除外）。停止会向当前 pick/place 子进程发 SIGINT，
等它收爪/保持，然后闭合夹爪并回 home 零位。

前置：先 `bash scripts/robot_up.sh` 起 ROS 栈；服务本身不启动 ROS 节点。
用法：
    python3 scripts/arm_service.py                 # 默认 data/arm.sock
    python3 scripts/arm_service.py --socket /tmp/arm.sock
    python3 scripts/arm_service.py --stdio         # 从 stdin 读，调试用

客户端：
    python3 scripts/arm_client.py 0
    python3 scripts/arm_client.py pick red
"""

import argparse
import atexit
import json
import os
import shlex
import signal
import socketserver
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from roarm_lib import paths, params  # noqa: E402

PICK = paths.SCRIPTS / "vision" / "pick.py"
CLI = paths.SCRIPTS / "roarm_cli.py"
DEFAULT_SOCKET = paths.DATA / "arm.sock"
TIMING_JSONL = paths.DATA_VISION / "timing.jsonl"


def _timing(event, seconds, **fields):
    """阶段 0：记录服务侧耗时，追加到 data/vision/timing.jsonl。"""
    if os.environ.get("ROARM_TIMING", "1") == "0":
        return
    payload = " ".join("%s=%s" % (k, v) for k, v in fields.items())
    print("[time] service %-22s %6.2f s  %s"
          % (event, seconds, payload), flush=True)
    row = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"), "source": "service",
           "event": event, "s": round(float(seconds), 3)}
    row.update(fields)
    try:
        with open(TIMING_JSONL, "a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
    except Exception as exc:                            # noqa: BLE001
        print("[time] 写 timing.jsonl 失败：%s" % exc, file=sys.stderr)

#: 与 ssvep-bci config.ARM_NAMES_COLORS 对齐
INDICES = {
    0: ("pick", "red"), 1: ("pick", "green"), 2: ("pick", "blue"),
    3: ("place", "red"), 4: ("place", "green"), 5: ("place", "blue"),
    6: ("home", None), 7: ("stop", None),
}
LABELS = {0: "取红色", 1: "取绿色", 2: "取蓝色",
          3: "放红色", 4: "放绿色", 5: "放蓝色",
          6: "回位", 7: "停止"}
COLORS = ("red", "green", "blue")

PY_PICK = os.environ.get("ARM_PYTHON", sys.executable)
PY_ROS = os.environ.get("ROS_PYTHON", "/usr/bin/python3")


def _ros_setup():
    distro = os.environ.get("ROS_DISTRO", "humble")
    return (os.environ.get("ROS_SETUP", "/opt/ros/%s/setup.bash" % distro),
            str(paths.WS_SETUP))


def parse_request(line):
    """返回 (k, action, color) 或抛 ValueError。"""
    line = (line or "").strip()
    if not line:
        raise ValueError("空指令")
    if line.startswith("{"):
        d = json.loads(line)
        if "k" in d:
            k = int(d["k"])
            if k not in INDICES:
                raise ValueError("未知类别 k=%d" % k)
            action, color = INDICES[k]
            return k, action, color
        action = str(d.get("cmd", "")).lower()
        color = d.get("color")
        if action not in ("pick", "place", "home", "stop", "storage"):
            raise ValueError("未知命令 %r" % action)
        if action in ("home", "storage"):
            return 6, "home", None
        if action == "stop":
            return 7, "stop", None
        if color not in COLORS:
            raise ValueError("pick/place 需要 color=red|green|blue")
        return (0 if action == "pick" else 3) + COLORS.index(color), action, color
    parts = line.lower().split()
    action = parts[0]
    if action in ("home", "storage"):
        return 6, "home", None
    if action == "stop":
        return 7, "stop", None
    if action in ("pick", "place"):
        if len(parts) < 2 or parts[1] not in COLORS:
            raise ValueError("%s 后面要跟 red|green|blue" % action)
        color = parts[1]
        return (0 if action == "pick" else 3) + COLORS.index(color), action, color
    if action.isdigit():
        k = int(action)
        if k not in INDICES:
            raise ValueError("未知类别 k=%d" % k)
        a, c = INDICES[k]
        return k, a, c
    raise ValueError("未知命令 %r" % action)


class Controller:
    def __init__(self, tray_file=None):
        self.lock = threading.RLock()
        self.busy = False
        self.proc = None
        self.last_color = None
        self.seq = 0
        self.trays = {}
        self.tray_file = Path(tray_file) if tray_file else Path(
            os.environ.get("ARM_TRAYS", paths.DATA_VISION / "trays.json"))

    # ------------------------------------------------------------ 外部
    def execute(self, line):
        try:
            k, action, color = parse_request(line)
        except Exception as exc:                       # noqa: BLE001
            return self._resp(None, "error", str(exc))

        if action == "stop":
            return self.stop()

        _t0 = time.perf_counter()
        with self.lock:
            if self.busy:
                _timing("busy_drop", 0.0, action=action, color=color or "")
                return self._resp(action + (" " + color if color else ""),
                                  "busy", "上一个动作还没结束")
            self.busy = True
        label = LABELS.get(k, action)
        try:
            if action == "pick":
                return self._pick(color, label)
            if action == "place":
                return self._place(color, label)
            return self._home(label)
        except Exception as exc:                       # noqa: BLE001
            return self._resp(label, "error", "%s: %s" % (type(exc).__name__, exc))
        finally:
            with self.lock:
                self.busy = False
            _timing("total", time.perf_counter() - _t0,
                    action=action, color=color or "")

    def stop(self):
        # STOP 不参与 busy 排队：先打断当前子进程
        _t0 = time.perf_counter()
        p = self.proc
        if p is not None and p.poll() is None:
            try:
                p.send_signal(signal.SIGINT)           # 让 pick.py 走取消/收爪
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        self.proc = None
        self.last_color = None
        self._ros(["grip", "close", "--wait", "4"], timeout=20)
        rc, out = self._ros(["home", "--wait", "180", "--tol", "25"],
                            timeout=200)
        with self.lock:
            self.busy = False
        _timing("stop", time.perf_counter() - _t0, rc=rc)
        return self._resp("停止", "stopped" if rc == 0 else "failed",
                          "已停止并回零位" if rc == 0 else out)

    # ------------------------------------------------------------ 动作
    def _pick(self, color, label):
        self.last_color = None
        _t = time.perf_counter()
        self.refresh_trays(log=True)
        _timing("refresh_trays", time.perf_counter() - _t, color=color)
        cmd = [PY_PICK, str(PICK), "--auto", "--color", color,
               "--no-place", "--hold-here", "--no-show"]
        _t = time.perf_counter()
        rc, out = self._run(cmd, timeout=240)
        _timing("pick_subprocess", time.perf_counter() - _t, color=color, rc=rc)
        if rc != 0:
            return self._resp(label, "failed", out)
        self.last_color = color
        detail = ("已夹住 %s 物料，等待放置" % color if params.GRASP_CHECK
                  else "已执行夹取（夹取判断已关闭，空夹）")
        return self._resp(label, "ok", detail)

    def _place(self, color, label):
        if not self.last_color:
            return self._resp(label, "no_object", "手上没有物料，先执行 pick")
        uv = self.trays.get(color)
        if uv is None:
            self.refresh_trays(log=True)
            uv = self.trays.get(color)
        if uv is None:
            return self._resp(label, "no_tray",
                              "没有 %s 托盘像素；放 data/vision/trays.json "
                              "或先跑 detect_color.py --pads" % color)
        cmd = [PY_PICK, str(PICK), "--place-only",
               "--drop-u", "%.1f" % uv[0], "--drop-v", "%.1f" % uv[1],
               "--no-show"]
        _t = time.perf_counter()
        rc, out = self._run(cmd, timeout=240)
        _timing("place_subprocess", time.perf_counter() - _t,
                color=color, rc=rc)
        if rc != 0:
            return self._resp(label, "failed", out)
        self.last_color = None
        return self._resp(label, "ok", "已放到 %s 托盘" % color)

    def _home(self, label):
        self.last_color = None
        _t = time.perf_counter()
        self._ros(["grip", "close", "--wait", "4"], timeout=20)
        _timing("home_grip", time.perf_counter() - _t)
        _t = time.perf_counter()
        rc, out = self._ros(["park"], timeout=20)
        _timing("home_park", time.perf_counter() - _t, rc=rc)
        return self._resp(label, "ok" if rc == 0 else "failed", out)

    # ------------------------------------------------------------ 托盘
    def refresh_trays(self, log=False, force=False):
        """优先读 trays.json；没有就用 color_detect.find_pads 现场检测。

        三色都缓存后直接返回，避免每次 pick 都开相机重测（2026-11-05
        计时：trays_detect 平均 1.6s/pick）。托盘位置变了用 force=True
        或重启服务。
        """
        if not force and all(c in self.trays for c in COLORS):
            return
        if self.tray_file.is_file():
            try:
                d = json.load(open(self.tray_file, encoding="utf-8"))
                for name in COLORS:
                    if name in d and len(d[name]) >= 2:
                        self.trays[name] = (float(d[name][0]), float(d[name][1]))
                if log:
                    print("[服务] 托盘来自 %s：%s" % (self.tray_file, self.trays),
                          flush=True)
                if all(c in self.trays for c in COLORS):
                    return
            except Exception as exc:                   # noqa: BLE001
                print("[服务] trays.json 读不了：%s" % exc, file=sys.stderr)
        try:
            sys.path.insert(0, str(paths.SCRIPTS / "vision"))
            import camera
            import color_detect as cd
            _t = time.perf_counter()
            img = camera.grab_frame()
            calib = cd.load_calib()
            _zg, z_table = cd.load_z_planes()
            pads = cd.find_pads(img, calib, z_table)
            _timing("trays_detect", time.perf_counter() - _t, found=len(pads))
            got = {n: pads[n]["uv"] for n in COLORS if pads.get(n)}
            if got:
                self.trays.update(got)
            if log:
                print("[服务] 自动识别托盘：%s" % self.trays, flush=True)
        except Exception as exc:                       # noqa: BLE001
            print("[服务] 托盘自动识别失败：%s" % exc, file=sys.stderr)

    # ------------------------------------------------------------ 进程
    def _run(self, cmd, timeout):
        print("[服务] $ %s" % " ".join(shlex.quote(c) for c in cmd), flush=True)
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
        self.proc = p
        try:
            out, _ = p.communicate(timeout=timeout)
            return p.returncode, (out or "")[-800:]
        except subprocess.TimeoutExpired:
            p.kill()
            out, _ = p.communicate()
            return 124, (out or "")[-800:] + "\n(外层超时)"
        finally:
            self.proc = None

    def _ros(self, args, timeout):
        ros, ws = _ros_setup()
        inner = " ".join(shlex.quote(x) for x in
                         [PY_ROS, str(CLI)] + [str(a) for a in args])
        cmd = "source %s >/dev/null 2>&1; source %s >/dev/null 2>&1; exec %s" % (
            shlex.quote(ros), shlex.quote(ws), inner)
        print("[服务] $ bash -lc %s" % cmd, flush=True)
        p = subprocess.Popen(["bash", "-lc", cmd], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
        self.proc = p
        try:
            out, _ = p.communicate(timeout=timeout)
            return p.returncode, (out or "")[-800:]
        except subprocess.TimeoutExpired:
            p.kill()
            out, _ = p.communicate()
            return 124, (out or "")[-800:] + "\n(外层超时)"
        finally:
            self.proc = None

    def _resp(self, cmd, state, detail=""):
        with self.lock:
            self.seq += 1
            return {"seq": self.seq, "cmd": cmd, "state": state,
                    "detail": detail, "holding": self.last_color}


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        line = self.rfile.readline().decode("utf-8", "replace")
        resp = self.server.controller.execute(line)
        self.wfile.write((json.dumps(resp, ensure_ascii=False) + "\n").encode())


class _Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


def run_stdio(ctrl):
    print("[服务] stdio 模式；一行一个指令，q 退出", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if line.lower() in ("q", "quit", "exit"):
            return 0
        if not line:
            continue
        resp = ctrl.execute(line)
        print(json.dumps(resp, ensure_ascii=False), flush=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description="8 指令机械臂服务")
    ap.add_argument("--socket", default=str(DEFAULT_SOCKET),
                    help="Unix socket 路径（默认 %s）" % DEFAULT_SOCKET)
    ap.add_argument("--trays", default=None,
                    help="托盘像素 JSON（默认 data/vision/trays.json）")
    ap.add_argument("--stdio", action="store_true", help="从 stdin 读指令")
    args = ap.parse_args()

    ctrl = Controller(tray_file=args.trays)
    if args.stdio:
        return run_stdio(ctrl)

    sock = Path(args.socket)
    sock.parent.mkdir(parents=True, exist_ok=True)
    try:
        sock.unlink()
    except FileNotFoundError:
        pass
    server = _Server(str(sock), _Handler)
    server.controller = ctrl
    atexit.register(lambda: (server.server_close(), sock.unlink(missing_ok=True)))
    print("[服务] 8 指令服务监听 %s" % sock, flush=True)
    print("[服务] 指令：0/1/2=取红/绿/蓝 3/4/5=放红/绿/蓝 6=回位 7=停止",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        try:
            sock.unlink()
        except FileNotFoundError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
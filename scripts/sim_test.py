#!/usr/bin/env python3
"""sim_test —— Cello 仿真场景矩阵测试（可复现、可无头）

做三件事：
    1. 自己起仿真（demo + arm_moveit_write），或 --attach 复用已跑的
    2. 逐个跑场景，用 roarm_cli 的真实退出码判定成败
    3. 打印结果表，全绿返回 0，否则返回 1

用法：
    source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
    python3 scripts/sim_test.py              # 起一套新仿真，跑完自动清掉
    python3 scripts/sim_test.py --attach     # 复用已经起好的仿真
    python3 scripts/sim_test.py --only up,right   # 只跑部分场景

设计约定：
    * 判定标准是**末端位置误差**（TF world->link6 对比目标），不是"看日志有没有报错"
    * 每个场景都断言**退出码**：0=到位 / 1=未到位 / 2=输入被拒绝
    * 单帧假绿的问题：要求连续 3 次进入容差才算到位（见 roarm_cli.wait_until_reached）
"""
import argparse
import os
import subprocess
import sys
import time

WS = "/home/ljl/BCI/DeepBCI-8ch/ssvep-Roarm/star-arm-moveit2"
SCRIPTS = "/home/ljl/BCI/DeepBCI-8ch/ssvep-Roarm/scripts"
CLI = os.path.join(SCRIPTS, "roarm_cli.py")

TOL = "5"          # 毫米
WAIT = "15"        # 可达目标的等待上限
WAIT_SHORT = "8"   # 预期失败的目标，少等一会

# (名称, CLI 参数, 期望退出码, 说明)
SCENARIOS = [
    ("home 回位",        ["home", "--wait", WAIT, "--tol", TOL],           0, "基准：四个路点都该到达"),
    ("up 到高处",        ["up", "--wait", WAIT, "--tol", TOL],             0, "z 上界附近"),
    ("right 右侧",       ["right", "--wait", WAIT, "--tol", TOL],          0, "y 上界 + x 上界附近"),
    ("start 低处",       ["start", "--wait", WAIT, "--tol", TOL],          0, "z 下界 + y 下界附近"),
    ("home 再回位",      ["home", "--wait", WAIT, "--tol", TOL],           0, "重复同一目标应幂等"),
    ("越界 x",           ["pose", "0.90", "0", "0.30", "--wait", "1"],     2, "x 超上界 0.529"),
    ("越界 z",           ["pose", "0.40", "0", "0.010", "--wait", "1"],    2, "z 低于下界 0.074"),
    ("越界 y",           ["pose", "0.40", "-0.60", "0.30", "--wait", "1"], 2, "y 低于下界 -0.324"),
    ("非法四元数全零",   ["pose", "0.40", "0", "0.30", "0", "0", "0", "0", "--wait", "1"], 2, "范数为 0 应拒绝"),
    ("四元数未归一化",   ["pose", "0.40", "0", "0.30", "0.6", "-0.6", "0.6", "-0.6",
                          "--wait", WAIT, "--tol", TOL],                   0, "应自动归一化后仍到位"),
    ("工作区内不可达",   ["pose", "0.529", "-0.324", "0.438", "--wait", WAIT_SHORT], 1,
                                                                           "极端角点，IK 大概率无解 → 应优雅失败"),
    ("夹爪 close",       ["grip", "close", "--wait", "10", "--tol", "1"], 0,
                                                                           "joint7_left → -0.025 m"),
    ("夹爪 open",        ["grip", "open", "--wait", "10", "--tol", "1"],  0,
                                                                           "joint7_left → 0.000 m"),
    ("夹爪 交替回 close", ["grip", "close", "--wait", "10", "--tol", "1"], 0,
                                                                           "open→close 可重复，不卡死"),
    ("读取当前位姿",     ["where"],                                         0,
                                                                           "world->link6 可查，供序列层用"),
    # 特殊标记 "task"：不是调 roarm_cli，而是 ros2 run roarm_task task
    ("完整取放序列",     ["task"],                                          0,
                                                                           "10 步串联：接近→合爪→抬起→递送→松爪"),
]


def free_display_env():
    env = os.environ.copy()
    env.pop("DISPLAY", None)     # 本机没有 X，DISPLAY=:1 是过期变量
    return env


def run_cli(args, timeout=60):
    # ["task"] 是特殊标记：走 ros2 run 而不是 roarm_cli.py。
    # 它验证的是任务层（原语+序列）能否真的串起一次取放。
    if args and args[0] == "task":
        cmd = ["ros2", "run", "roarm_task", "task"]
    else:
        cmd = [sys.executable, CLI] + args
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           env=free_display_env())
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "（超时被 kill）"


def start_sim():
    env = free_display_env()
    procs = []
    demo_log = open(os.path.join(WS, ".simtest_demo.log"), "w")
    node_log = open(os.path.join(WS, ".simtest_node.log"), "w")
    procs.append(subprocess.Popen(
        ["ros2", "launch", "cello_moveit_config", "demo.launch.py", "use_rviz:=false"],
        cwd=WS, env=env, stdout=demo_log, stderr=subprocess.STDOUT, start_new_session=True))

    print("等待 move_group 就绪…", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < 45:
        with open(os.path.join(WS, ".simtest_demo.log"), errors="ignore") as f:
            if "You can start planning" in f.read():
                break
        time.sleep(1)
    else:
        print(" 失败")
        stop_sim(procs)
        sys.exit(1)
    print(" %.0fs" % (time.time() - t0))

    procs.append(subprocess.Popen(
        ["ros2", "launch", "cello_moveit_config", "arm_moveit_write.launch.py"],
        cwd=WS, env=env, stdout=node_log, stderr=subprocess.STDOUT, start_new_session=True))

    print("等待执行节点就绪…", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < 30:
        with open(os.path.join(WS, ".simtest_node.log"), errors="ignore") as f:
            if "Ready to take commands for planning group arm" in f.read():
                break
        time.sleep(1)
    else:
        print(" 失败")
        stop_sim(procs)
        sys.exit(1)
    print(" %.0fs" % (time.time() - t0))

    print("启动夹爪桥…", end="", flush=True)
    bridge_log = open(os.path.join(WS, ".simtest_bridge.log"), "w")
    procs.append(subprocess.Popen(
        [sys.executable, os.path.join(SCRIPTS, "sim_gripper_bridge.py")],
        cwd=WS, env=env, stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True))
    t0 = time.time()
    while time.time() - t0 < 20:
        with open(os.path.join(WS, ".simtest_bridge.log"), errors="ignore") as f:
            if "SimGripperBridge ready" in f.read():
                break
        time.sleep(0.5)
    else:
        print(" 失败")
        stop_sim(procs)
        sys.exit(1)
    print(" %.0fs" % (time.time() - t0))
    return procs


def stop_sim(procs):
    # 只杀我们自己 start_new_session 出来的进程组，不做全局 pkill
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), 15)
        except Exception:
            pass
    time.sleep(2)
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), 9)
        except Exception:
            pass


def handshake():
    """起手式：先回 home 并校验，确认整条链路真的可用。失败重试一次。"""
    for attempt in (1, 2):
        print("握手 %d/2：home --wait 25 …" % attempt, end="", flush=True)
        rc, out = run_cli(["home", "--wait", "25", "--tol", TOL], timeout=60)
        print(" rc=%d" % rc)
        if rc == 0:
            return True
        print("  输出：", out.strip().replace("\n", "\n        "))
        time.sleep(3)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--attach", action="store_true", help="复用已启动的仿真")
    ap.add_argument("--only", default="", help="只跑名字包含这些关键词的场景，逗号分隔")
    args = ap.parse_args()

    if not os.environ.get("AMENT_PREFIX_PATH"):
        print("没 source ROS 环境。先：")
        print("  source /opt/ros/humble/setup.bash")
        print("  source %s/install/setup.bash" % WS)
        return 1

    procs = []
    if not args.attach:
        procs = start_sim()

    try:
        if not handshake():
            print("\n❌ 握手失败，仿真链路不可用，测试中止。")
            return 1

        picked = SCENARIOS
        if args.only:
            keys = [k.strip() for k in args.only.split(",") if k.strip()]
            picked = [s for s in SCENARIOS if any(k in s[0] or k == s[1][0] for k in keys)]

        print("\n=== 场景矩阵（%d 项）===" % len(picked))
        print("%-18s %-8s %-8s %-6s %s" % ("场景", "期望rc", "实际rc", "结果", "说明"))
        print("-" * 88)

        fails = []
        t0 = time.time()
        for name, argv, expect, note in picked:
            rc, out = run_cli(argv, timeout=180 if argv[:1] == ["task"] else 90)
            ok = (rc == expect)
            print("%-18s %-8d %-8d %-6s %s" % (name, expect, rc, "PASS" if ok else "FAIL", note))
            if not ok:
                fails.append((name, expect, rc, out))
            time.sleep(0.5)

        dt = time.time() - t0
        print("-" * 88)
        print("耗时 %.1f s  通过 %d/%d" % (dt, len(picked) - len(fails), len(picked)))

        if fails:
            print("\n=== 失败详情 ===")
            for name, expect, rc, out in fails:
                print("\n[%s] 期望 rc=%d 实际 rc=%d" % (name, expect, rc))
                for line in out.strip().splitlines()[-8:]:
                    print("    " + line)
            return 1

        print("\n✅ 全部通过")
        return 0
    finally:
        if procs:
            print("\n清理仿真进程…")
            stop_sim(procs)


if __name__ == "__main__":
    sys.exit(main())

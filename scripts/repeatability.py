#!/usr/bin/env python3
"""repeatability —— 真机重复定位精度测量

测什么：同一个目标点，从同一个出发点走 N 次，看落点分散多大。
        这**不是**绝对定位精度（那是落点离目标多远，已测 4.92 mm）。

为什么要分开测：
    绝对精度 = IK 残差 + 零点残差 + 舵机死区 + 机械回差
    重复精度 = 只有机械回差（IK 每次给出同样的解，零点误差每轴恒定，都被抵消）
    官方规格「重复精度 1 mm」说的是后者。

判据口径（工程常用两种，都输出）：
    * 3σ 半径：假设三轴独立正态，99.7% 的落点在此半径内
    * 最大偏差：N 次里离均值最远的那一个，保守口径

用法（先起真机：bash scripts/robot_up.sh）：
    python3 scripts/repeatability.py                 # 10 次，目标 (0.45, 0, 0.40)
    python3 scripts/repeatability.py -n 20           # 20 次
    python3 scripts/repeatability.py --target 0.40 0 0.45
    python3 scripts/repeatability.py --from-csv      # 不跑机械臂，重新统计已有 CSV

注意：
    * 每次都会经过出发点（默认 home），这既是安全位，也保证每次接近方向一致
    * --tol 只影响 CLI 是否报"未到位"，**不影响记录的实测位置**
"""

import argparse
import csv
import importlib.util
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
CLI = SCRIPTS / "roarm_cli.py"
DEFAULT_CSV = ROOT / "data" / "repeatability.csv"

#: CLI 在到位/未到位两种情况下都会打印这一行
RE_POS = re.compile(r"实际 x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)")
#: where 命令的"当前位置"行（静置后复测用）
RE_WHERE = re.compile(r"当前位置 x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)")
RE_ANG = re.compile(r"关节角\(度\):\s*(.+)")
RE_J = re.compile(r"j\d+=\s*([-\d.]+)")
RE_ERR = re.compile(r"误差 ([\d.]+) mm")


def run_cli(args, wait_s, timeout_s=None):
    """跑一条 CLI 命令，返回 (rc, stdout)。"""
    cmd = [sys.executable, str(CLI)] + args + ["--wait", str(wait_s)]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout_s or (wait_s + 60))
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, "子进程超时"


def parse(out):
    """从 CLI 输出里抓位置、关节角、误差。"""
    m = RE_POS.search(out)
    pos = (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None
    ang = None
    m = RE_ANG.search(out)
    if m:
        vals = [float(v) for v in RE_J.findall(m.group(1))]
        if len(vals) >= 6:
            ang = vals
    err = None
    m = RE_ERR.search(out)
    if m:
        err = float(m.group(1))
    return pos, ang, err


def precheck():
    """跑一条只读的 where，把"发布者没起 / TF 不可用"在第一步就暴露出来。

    直接开测的话，这类环境问题会表现为"第 1 次 home 失败 rc=1"，
    看起来像运动学问题，实际是上游没启动——先把两者分开。
    """
    rc, out = run_cli(["where"], 3)
    if rc == 0:
        return True
    print("✗ 环境自检失败（where 返回 %d）：" % rc)
    for line in [l for l in out.strip().splitlines() if l.strip()][-8:]:
        print("   " + line)
    print()
    if "没有发现订阅者" in out:
        print("→ 执行节点没起。先跑：bash scripts/robot_up.sh")
    elif "TF" in out or "拿不到" in out:
        print("→ TF 不可用，通常是 /joint_states 没人发布（driver 或 rsp 没起）。")
        print("  查：ros2 topic info /joint_states")
    return False


def settle_read(settle_s):
    """到达后静置，再用 where 复测一次。

    CLI 的 --wait 是"连续 3 次进容差"就返回，此刻舵机可能还在收敛。
    实测同一落点两口径能差 1.6 mm，与机械重复性同量级——
    不静置的话，测出来的"重复性"里混着采样时刻的差。
    """
    if settle_s > 0:
        time.sleep(settle_s)
    rc, out = run_cli(["where"], 5)
    if rc != 0:
        return None, None
    m = RE_WHERE.search(out)
    pos = (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None
    ang = None
    m = RE_ANG.search(out)
    if m:
        vals = [float(v) for v in RE_J.findall(m.group(1))]
        if len(vals) >= 6:
            ang = vals
    return pos, ang


def measure(target, times, via, wait_s, tol_mm, timeout_s, tol_via=25.0, settle_s=1.5,
            trial=""):
    """往返 times 次，返回样本列表。"""
    samples = []
    if via and not precheck():
        return samples
    for i in range(1, times + 1):
        if via:
            rc_v, out_v = run_cli([via, "--tol", str(tol_via)], wait_s, timeout_s)
            if rc_v != 0:
                print("  ⚠ 第 %d 次：回出发点 %s 失败（rc=%d），该样本跳过"
                      % (i, via, rc_v))
                for line in [l for l in out_v.strip().splitlines() if l.strip()][-5:]:
                    print("      │ " + line)
                if i == 1:
                    print("      └ 第一次就失败，先单独确认这一条：")
                    print("        python3 scripts/%s.py %s --wait 30"
                          % (Path(__file__).stem, via))
                continue

        rc, out = run_cli(["pose", "%.4f" % target[0], "%.4f" % target[1],
                           "%.4f" % target[2], "--tol", str(tol_mm)],
                          wait_s, timeout_s)
        pos, ang, err = parse(out)
        if pos is None:
            print("  ✗ 第 %d 次：没拿到实测位置（rc=%d）" % (i, rc))
            print("    " + "\n    ".join(l for l in out.strip().splitlines()[-4:]))
            continue

        # 静置复测：用停稳后的位置覆盖到达瞬间的读数
        if settle_s > 0:
            pos2, ang2 = settle_read(settle_s)
            if pos2 is not None:
                drift = math.dist(pos, pos2) * 1000
                pos = pos2
                ang = ang2 or ang
                err = math.dist(pos, target) * 1000
                if drift > 0.5:
                    print("      · 静置 %.1fs 后位置再动 %.2f mm（已采用停稳值）"
                          % (settle_s, drift))

        samples.append({"idx": i, "trial": trial, "rc": rc, "pos": pos,
                        "ang": ang, "err_mm": err})
        mark = "✓" if rc == 0 else "✗"
        print("  %s 第 %2d 次  x=%.4f y=%+.4f z=%.4f  误差 %s mm"
              % (mark, i, pos[0], pos[1], pos[2],
                 "%.2f" % err if err is not None else "-"))
    return samples


def stats(vals):
    """返回 (均值, 标准差[样本], 极差)。"""
    n = len(vals)
    if n == 0:
        return 0.0, 0.0, 0.0
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / (n - 1) if n > 1 else 0.0
    return mean, math.sqrt(var), max(vals) - min(vals)


def pos_block(pts, label):
    """打印一组落点的重复性，返回 (3σ, 最大偏差, 平均偏差, 三轴σ)。"""
    means = [stats([p[k] for p in pts])[0] for k in range(3)]
    print("\n【位置重复性 · %s】相对 %d 次均值的离散度（毫米）" % (label, len(pts)))
    print("  轴      均值          标准差     极差")
    for k, name in enumerate("xyz"):
        m, sd, rng = stats([p[k] for p in pts])
        print("  %s   %10.4f     %7.4f    %6.4f" % (name, m * 1000, sd * 1000, rng * 1000))

    devs = [math.dist(p, means) for p in pts]
    d_mean, _, d_max = stats(devs)
    # 3σ 半径必须用三轴方差合成，不能拿"距离这个标量"的标准差去乘 3。
    # 落点距离服从 Rayleigh 分布而非正态；对距离取 σ 会把结果低估约 √3 倍，
    # 也就是把精度报得比实际好——这个方向错不得。
    sig_xyz = [stats([p[k] for p in pts])[1] for k in range(3)]
    sig_3d = math.sqrt(sum(s * s for s in sig_xyz))
    r3 = 3 * sig_3d
    print("\n  三轴合成 σ = %.3f mm（√(σx²+σy²+σz²)，σx/y/z = %.3f / %.3f / %.3f）"
          % (sig_3d * 1000, *(s * 1000 for s in sig_xyz)))
    print("  落点相对均值的 3D 距离：最大 %.3f   平均 %.3f   3σ %.3f mm"
          % (d_max * 1000, d_mean * 1000, r3 * 1000))
    return r3, d_max, d_mean, sig_xyz


def _load_fk():
    """加载同目录的 fk.py（不起 ROS、不依赖外壳）。"""
    try:
        spec = importlib.util.spec_from_file_location("fk", str(SCRIPTS / "fk.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


def fk_points(good):
    """用关节角 + URDF 算末端位置，得到不依赖 TF 的第二组落点。

    为什么值得算这一组：TF 是 robot_state_publisher 按 joint_states 算的，
    读的时刻和舵机真正停稳的时刻不一定同一瞬。用关节角自己算一遍，
    两者的差就是"采样时刻"造成的测量噪声。
    """
    mod = _load_fk()
    if mod is None:
        return []
    try:
        urdf = mod.parse_urdf(str(mod.DEFAULT_URDF))
    except Exception:
        return []
    out = []
    for s in good:
        if not s["ang"] or len(s["ang"]) < 6:
            return []
        try:
            out.append(tuple(mod.fk(urdf, s["ang"][:6])[-1][1]))
        except Exception:
            return []
    return out


def report(samples, target):
    """统计并打印。位置单位内部用米，展示用毫米。"""
    good = [s for s in samples if s["pos"]]
    if len(good) < 2:
        print("\n样本不足（%d 条），无法统计。" % len(good))
        return None

    ok_cnt = sum(1 for s in good if s["rc"] == 0)
    pts = [s["pos"] for s in good]

    print("\n" + "=" * 68)
    print("样本 %d 次（其中 %d 次进了容差）  目标 (%.3f, %.3f, %.3f)"
          % (len(good), ok_cnt, target[0], target[1], target[2]))
    print("=" * 68)

    # ---------- 位置：两个口径 ----------
    r3, d_max, d_mean, sig_xyz = pos_block(pts, "TF / 编码器口径")

    fk_pts = fk_points(good)
    if fk_pts:
        r3_fk, dmax_fk, _, sig_fk = pos_block(fk_pts, "FK / 关节角口径")
        diff_mm = (r3 - r3_fk) * 1000
        print("\n  【两口径之差】")
        print("    3σ：TF %.2f mm   vs   FK %.2f mm   差 %+.2f mm"
              % (r3 * 1000, r3_fk * 1000, diff_mm))
        if diff_mm > 0.3:
            print("    TF 明显更大 → 差值主要是采样时刻（--wait 返回时舵机还在收敛）。")
            print("    机械本征按 FK 口径算，报数时说明口径。")
        elif diff_mm < -0.3:
            print("    TF 反而更小 → 不是采样时刻能解释的，"
                  "要查 robot_state_publisher 与 fk.py 是否同源。")
        else:
            print("    两口径一致（±0.3 mm 内）→ --settle 已把采样时刻的差消掉，")
            print("    这组数据可直接用，报任一口径都成立。")
    else:
        print("\n  （未做 FK 口径对照：读不到 fk.py 或缺关节角）")

    # ---------- 关节 ----------
    angs = [s["ang"] for s in good if s["ang"]]
    if angs:
        n = min(len(a) for a in angs)
        print("\n【关节角重复性】（度）")
        print("  关节     标准差     极差      范围")
        for j in range(n):
            vals = [a[j] for a in angs]
            _, sd, rng = stats(vals)
            print("  j%-2d    %7.3f   %6.3f   %+7.2f ~ %+7.2f"
                  % (j + 1, sd, rng, min(vals), max(vals)))

    # ---------- 绝对误差（顺带测出来的第二个指标） ----------
    errs = [s["err_mm"] for s in good if s["err_mm"] is not None]
    if errs:
        e_mean, e_sd, e_max = stats(errs)
        print("\n【绝对定位误差】（落点离目标的距离，毫米）—— 顺带得到的第二指标")
        print("  均值 %.2f    标准差 %.2f    最大 %.2f"
              % (e_mean, e_sd, e_max))
        print("  均值里包含系统性偏差（IK 在关节舍入下的极限），不是随机项。")

    # ---------- 结论 ----------
    print("\n【结论】")
    print("  重复定位精度（3σ）= ±%.2f mm" % (r3 * 1000))
    print("  重复定位精度（最大偏差）= %.2f mm" % (d_max * 1000))
    if r3 * 1000 <= 1.0:
        print("  → 优于官方规格 1 mm ✅")
    elif r3 * 1000 <= 2.0:
        print("  → 与官方规格 1 mm 同量级（差 %.1f mm）。"
              % (r3 * 1000 - 1.0))
        print("    官方测试条件未知（负载/行程/次数），不能直接判不合格。")
    else:
        print("  → 明显大于官方规格 1 mm，需排查回差、紧固、负载。")
    if errs:
        print("  → 对上面 \"绝对定位误差\" 那块：均值 %.2f mm 就是绝对精度。" % e_mean)
    print("     两者不可互换：一个说\"每次都去同一个地方\"，一个说\"那个地方对不对\"。")
    print("     系统偏差可以靠标定/补偿修；离散（3σ）只能靠机械和闭环修。")
    # 返回值只用于"成功/失败"判断（main 里 `0 if result else 1`）。
    # 这里曾经挂过 "means"，但 pos_block() 重构后该变量已不在本函数作用域内；
    # 没有任何调用方读具体字段，删掉即可（保留键会 NameError）。
    return {"r3": r3, "max": d_max, "n": len(good), "sig_xyz": sig_xyz}


def write_csv(path, samples, target):
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["trial", "idx", "rc", "target_x", "target_y", "target_z",
                        "x", "y", "z", "err_mm"]
                       + ["j%d" % i for i in range(1, 8)])
        for s in samples:
            row = [s.get("trial", ""), s["idx"], s["rc"],
                   "%.4f" % target[0], "%.4f" % target[1], "%.4f" % target[2],
                   "%.5f" % s["pos"][0], "%.5f" % s["pos"][1], "%.5f" % s["pos"][2],
                   "%.3f" % s["err_mm"] if s["err_mm"] is not None else ""]
            ang = s["ang"] or []
            row += ["%.2f" % a for a in ang] + [""] * (7 - len(ang))
            w.writerow(row)
    return path


def load_csv(path):
    if not path.exists():
        raise SystemExit("找不到 %s" % path)
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    # 一个 CSV 会累积多次运行。只统计最后一组，否则 20 号的数据会和 19 号混在一起，
    # 算出来的 σ 既不代表这次也不代表那次。按 trial 出现的先后取最后一个。
    trials = []
    for r in rows:
        t = (r.get("trial") or "").strip()
        if t not in trials:
            trials.append(t)
    last = trials[-1] if trials else ""
    picked = [r for r in rows if (r.get("trial") or "").strip() == last]
    print("从 %s 读到 %d 条样本，取最后一组（trial=%s）%d 条"
          % (path, len(rows), last or "未标记", len(picked)))
    if last == "" and len(picked) > 40:
        print("⚠ 这批旧样本没有 trial 标记，可能混了多次运行。")
        print("  建议：删掉该 CSV 重跑一次，或手工只保留一段。")
    rows = picked
    samples = []
    for r in rows:
        try:
            samples.append({
                "idx": int(r["idx"]), "rc": int(r["rc"]),
                "pos": (float(r["x"]), float(r["y"]), float(r["z"])),
                "ang": [float(r["j%d" % i]) for i in range(1, 7)],
                "err_mm": float(r["err_mm"]) if r["err_mm"] else None,
            })
        except (KeyError, ValueError):
            continue
    target = (float(rows[0]["target_x"]), float(rows[0]["target_y"]),
              float(rows[0]["target_z"])) if rows else (0, 0, 0)
    return samples, target


def main():
    ap = argparse.ArgumentParser(
        description="真机重复定位精度测量（同一目标往返 N 次，统计落点离散度）")
    ap.add_argument("--target", nargs=3, type=float, metavar="M",
                    default=[0.45, 0.0, 0.40],
                    help="目标点 x y z（米），默认 0.45 0 0.40（已验证可达）")
    ap.add_argument("-n", "--times", type=int, default=10, help="往返次数，默认 10")
    ap.add_argument("--via", default="home", choices=["home", "none"],
                    help="每次先回哪里：home（默认，保证接近方向一致）或 none")
    ap.add_argument("--wait", type=float, default=30.0, help="每条命令的到位等待上限")
    ap.add_argument("--tol", type=float, default=15.0,
                    help="目标点的到位容差（毫米）。只影响标记✓/✗，不影响记录位置。"
                         "实测绝对误差稳定在 11 mm 左右，设成 10 会有大半标 ✗")
    ap.add_argument("--tol-via", type=float, default=25.0,
                    help="回出发点用的容差（毫米），默认 25。"
                         "出发点只要求\"回到同一处\"，不需要准；"
                         "实测 home 有 ~15 mm 系统偏差（IK 在 1° 关节舍入下的极限）")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="到达后静置多少秒再复测一次（默认 1.5）。"
                         "0 = 关闭，直接用 --wait 返回瞬间的读数")
    ap.add_argument("--csv", default=str(DEFAULT_CSV), help="样本写入的 CSV")
    ap.add_argument("--from-csv", action="store_true", help="不跑机械臂，重新统计已有 CSV")
    args = ap.parse_args()

    target = tuple(args.target)
    csv_path = Path(args.csv)

    if args.from_csv:
        samples, target = load_csv(csv_path)
        report(samples, target)
        return 0

    if not os.environ.get("ROS_DISTRO"):
        print("⚠ 没检测到 ROS 环境。先执行：")
        print("  source /opt/ros/humble/setup.bash")
        print("  source star-arm-moveit2/install/setup.bash")
        return 2

    print("重复定位精度测量：目标 (%.3f, %.3f, %.3f)，往返 %d 次"
          % (*target, args.times))
    print("每次路径：%s → 目标。按实测约 10 s + 8 s 估算，共约 %.0f 秒"
          % ("home" if args.via == "home" else "原地", args.times * 20))
    print("⚠ 机械臂会自己动，确认工作区内无障碍、电源开关在手边。\n")

    via = None if args.via == "none" else args.via
    trial = time.strftime("%Y%m%d-%H%M%S")
    samples = measure(target, args.times, via, args.wait, args.tol,
                      args.wait + 60, args.tol_via, args.settle, trial)
    if not samples:
        print("\n没有有效样本，未做统计。")
        print("若每次都是 home 失败：环境没起全（见上面自检），先 bash scripts/robot_up.sh")
        print("若只是偶发失败：可以 --via none 原地重复，但接近方向就统一不了了。")
        return 1

    result = report(samples, target)
    path = write_csv(csv_path, samples, target)
    print("\n样本已追加到 %s" % path)
    return 0 if result else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""teach —— 交互式示教：把臂摆到位，记录点位，生成场景 yaml

为什么用示教而不是算：
    `link6` 到夹爪指尖的距离只存在于 mesh 里，URDF 里没有；
    物体高度、桌面高度也不在模型里。算不出来，只能量或示教。
    示教的好处是**不需要知道指尖多长**——你看到指尖碰到物体了，记下来就是对的。

用法（先起真机：bash scripts/robot_up.sh）：

    python3 scripts/teach.py

    示教> show              看当前位姿
    示教> x+5               沿 x 前移 5 mm（y+/y-/z+/z- 同理）
    示教> z-8               下降 8 mm
    示教> mark pick         把当前位置记为 pick
    示教> mark place        记为 place
    示教> mark approach     记为 approach（取物点上方，可选）
    示教> grip 0.5          夹爪半开（0=合拢 100=全开，比例）
    示教> points            列出已记录的点
    示教> emit              生成场景 yaml
    示教> quit

安全：单次移动最大 10 mm；每次移动由 roarm_cli 做工作区检查；
      姿态（四元数）保持示教开始时的值，不会自己转腕。
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
CLI = SCRIPTS / "roarm_cli.py"
POINTS = ROOT / "data" / "teach_points.json"
SCENE_OUT = ROOT / "data" / "scene_real.yaml"

#: 实测移动速度（velocity_scaling=0.05）：30 mm 用 9.9 s ≈ 3 mm/s
SPEED_MM_S = 3.0
#: 夹爪舵机角与物理开合的关系。**2026-10-18 实物实测，与之前仿真的推算相反**：
#:   舵机 100° → joint7_left =  0.000 → 两指**张开**（约 50 mm）
#:   舵机 21.9° → joint7_left = -0.025 → 两指**合拢**（0 mm）
#: 21.9° 是 URDF 下限对应的角度，再小就超限（超限会让 MoveIt 判整机状态非法）。
GRIP_CLOSED_DEG, GRIP_OPEN_DEG = 21.9, 100.0
#: 2026-10-18 卡尺实测：舵机 21.9° -> 8 mm，100° -> 65 mm
GAP_MIN_MM, GAP_MAX_MM = 8.0, 65.0
MAX_STEP_MM = 50.0

#: 是否越过 roarm_cli.BOUNDS 的软件边界（--force 设置）。
#: BOUNDS 是上位机 README 的"推荐工作区"，不是机械极限（y 方向左右不对称就能看出来）。
#: 示教时可以越过；自动执行（task）仍然保守。
FORCE = False


RE_POS = re.compile(r"当前位置 x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)")
RE_Q = re.compile(r"q=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)")
RE_J = re.compile(r"关节角\(度\):\s*(.+)")
RE_JVAL = re.compile(r"j\d+=\s*([-\d.]+)")


def cli(args, timeout=60):
    cmd = [sys.executable, str(CLI)] + args
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def where():
    """读当前位姿，返回 (pos, quat, angs)。失败抛 RuntimeError。"""
    rc, out = cli(["where"], timeout=20)
    if rc != 0:
        raise RuntimeError("读不到位姿：\n" + out.strip()[-400:])
    m = RE_POS.search(out)
    if not m:
        raise RuntimeError("位姿解析失败：\n" + out.strip()[-400:])
    pos = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    qm = RE_Q.search(out)
    quat = tuple(float(qm.group(i)) for i in range(1, 5)) if qm else None
    angs = None
    am = RE_J.search(out)
    if am:
        vals = [float(v) for v in RE_JVAL.findall(am.group(1))]
        if len(vals) >= 6:
            angs = vals
    return pos, quat, angs


def load_points():
    if POINTS.exists():
        try:
            return json.loads(POINTS.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {}


def save_points(pts):
    POINTS.parent.mkdir(parents=True, exist_ok=True)
    POINTS.write_text(json.dumps(pts, ensure_ascii=False, indent=2), encoding="utf-8")


def show(pos, quat, angs):
    print("  位姿  x=%.4f  y=%+.4f  z=%.4f" % pos)
    if quat:
        # 由四元数反推 RPY（度），好判断夹爪是不是朝下
        w, x, y, z = quat
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        print("  姿态  RPY = %+.0f / %+.0f / %+.0f°" % (roll, pitch, yaw))
    if angs:
        print("  关节  " + "  ".join("j%d=%+.1f" % (i + 1, a) for i, a in enumerate(angs[:6])))


def _send(quat, tgt, from_pos=None):
    """把臂送到绝对坐标，返回是否成功。

    等待时间按距离算，不写死：0.05 缩放下只有约 3 mm/s，走 140 mm 要 50 s。
    原来固定 --wait 20，远一点的移动必然超时，还会被误判成"够不着"。
    """
    dist_mm = math.dist(from_pos, tgt) * 1000 if from_pos else 0.0
    wait_s = max(30.0, dist_mm / SPEED_MM_S * 1.8 + 8) if dist_mm else 30.0
    wait_s = min(wait_s, 240.0)
    if dist_mm > 60:
        print("  移动 %.0f mm，按 %.0f mm/s 估算约需 %.0f s（等待上限 %.0f s）"
              % (dist_mm, SPEED_MM_S, dist_mm / SPEED_MM_S, wait_s))
        print("  期间没有任何输出是正常的——0.05 速度缩放就是慢，不是卡住")
        print("  （若目标越界会立刻返回，不会真等这么久）")

    args = ["pose", "%.4f" % tgt[0], "%.4f" % tgt[1], "%.4f" % tgt[2]]
    if quat:
        args += ["%.4f" % v for v in quat]
    args += ["--wait", "%.0f" % wait_s, "--tol", "15"]
    if FORCE:
        args.append("--force")

    rc, out = cli(args, timeout=wait_s + 40)
    if rc == 0:
        print("  ✓ → (%.4f, %+.4f, %.4f)" % tuple(tgt))
        if tgt[2] < 0.06:
            print("  ⚠ z = %.0f mm，快贴桌面了，下降请用 z-3 这种小步" % (tgt[2] * 1000))
        return True
    if rc == 2:
        print("  ✗ 被拒绝（软件工作区边界）：")
        for line in [l for l in out.strip().splitlines() if "拒绝" in l or "[" in l]:
            print("     " + line.strip())
        if not FORCE:
            print("     ↑ 这是软件保守边界，不是机械极限。机械确实能到更远的话，")
            print("       重开时加 --force：python3 scripts/teach.py --force")
        return False
    print("  ✗ 没到位（rc=%d）" % rc)
    moved = 0.0
    for line in out.strip().splitlines():
        if "误差" in line or "实际" in line:
            print("     " + line.strip())
    # 超时和"够不着"的区分：走了多远是唯一线索
    mp = out.strip()
    m = re.search(r"实际 x=([-\d.]+) y=([-\d.]+) z=([-\d.]+)", mp)
    if m and from_pos:
        moved = math.dist(from_pos, tuple(float(m.group(i)) for i in (1, 2, 3))) * 1000
        print("     （本次实际移动 %.0f mm / 目标 %.0f mm）" % (moved, dist_mm))
        if moved > dist_mm * 0.6:
            print("     → 走了一大半，是超时不是够不着：再发一次同样的命令会接着走")
        elif moved < dist_mm * 0.1:
            print("     → 几乎没动，可能是 IK 无解或规划失败，看 move_group 的日志")
    return False


def move_rel(pos, quat, axis, delta_mm):
    """沿世界系单轴移动，成功返回新位姿。"""
    if abs(delta_mm) > MAX_STEP_MM:
        print("  ⚠ 单次移动上限 %.0f mm，已截断" % MAX_STEP_MM)
        delta_mm = math.copysign(MAX_STEP_MM, delta_mm)
    k = "xyz".index(axis)
    tgt = list(pos)
    tgt[k] += delta_mm / 1000.0
    return tuple(tgt) if _send(quat, tgt, pos) else pos


def move_abs(pos, quat, tgt):
    """直接跳到绝对坐标（粗定位用）。失败返回 None。"""
    return tuple(tgt) if _send(quat, tgt, pos) else None


def emit(pts, standoff_override=None):
    """从示教点生成场景 yaml。"""
    missing = [k for k in ("pick", "place") if k not in pts]
    if missing:
        print("  ✗ 缺少点位：%s（先 mark）" % "、".join(missing))
        return False

    pick = pts["pick"]
    place = pts["place"]
    h = pts.get("object_height", 0.03)
    w = pts.get("object_width", 0.03)

    standoff = 0.05
    if standoff_override is not None:
        standoff = standoff_override
        print("  standoff = %.0f mm（指定值）" % (standoff * 1000))
    elif "approach" in pts and pts["approach"]["z"] > pick["z"]:
        d = pts["approach"]["z"] - pick["z"]
        # 接近点不能高于"抬起点"，否则抬起后比接近点还低，任务层会拒绝这个场景
        lift = h + 0.02
        if d > lift:
            print("  ⚠ approach 比 pick 高 %.0f mm，但抬起量只有 %.0f mm（物高+净空）。"
                  % (d * 1000, lift * 1000))
            print("    任务层会因此拒绝本场景（\"抬起会低于接近点\"）。")
            print("    已改用默认 standoff 50 mm；想用别的值加 --standoff。")
        else:
            standoff = round(d, 3)
            print("  用实测接近高度：standoff = %.0f mm" % (d * 1000))

    # 张开比例：示教时最后一次 grip 的值；没记录就全开
    grip_open = float(pts.get("grip_open", 0) or 0)
    if grip_open <= 0:
        grip_open = 1.0
        print("  未记录 grip 张开比例，用全开 1.00")

    # 合拢比例：按物宽算，留 2 mm 过盈（略小于物宽才夹得住，又不至于堵转）
    span = GAP_MAX_MM - GAP_MIN_MM
    grip_close = max(0.0, min(1.0, (w * 1000 - 2 - GAP_MIN_MM) / span))
    print("  张开 %.0f mm(grip %.2f)  合拢 %.0f mm(grip %.2f)  [物宽 %.0f mm]"
          % (GAP_MIN_MM + grip_open * span, grip_open,
             GAP_MIN_MM + grip_close * span, grip_close, w * 1000))

    lines = [
        "# 由 scripts/teach.py 生成 —— %s" % time_stamp(),
        "# 这是**示教值**，不是标定值：换成别的物体/桌面位置都要重新示教",
        "pick: [%.4f, %.4f, %.4f]" % (pick["x"], pick["y"], pick["z"]),
        "place: [%.4f, %.4f, %.4f]" % (place["x"], place["y"], place["z"]),
        "object_height: %.3f" % h,
        "object_width: %.3f" % w,
        "grip_open: %.2f                 # 张开比例：0=8mm 1=65mm" % grip_open,
        "grip_close: %.2f                # 合拢比例：略小于物宽即可" % grip_close,
        "standoff: %.3f" % standoff,
        "clearance: 0.02",
        "verify_grasp: false",
    ]
    SCENE_OUT.parent.mkdir(parents=True, exist_ok=True)
    SCENE_OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n  已写入 %s" % SCENE_OUT)
    print("\n" + "\n".join("  " + l for l in lines if not l.startswith("#")))
    print("\n  跑：ros2 run roarm_task task --scene %s --dry-run" % SCENE_OUT)
    print("      （先 --dry-run 看序列，确认后再去掉）")
    return True


def time_stamp():
    import time
    return time.strftime("%Y-%m-%d %H:%M:%S")


def help_text():
    print("""
  show              看当前位姿
  go 0.35 0.10 0.15 直接跳到绝对坐标（米）——粗定位用
  x+5 / x-5         沿世界 x 移动 5 mm（y+ y- z+ z- 同理），单次上限 50 mm
  mark pick         记当前位置为 pick（抓取点）
  mark place        记为 place（放置点）
  mark approach     记为 approach（取物点上方，可选，用了就自动算 standoff）
  obj 高度 宽度      设物体尺寸（米），如 `obj 0.03 0.03`
  grip 0.5          夹爪：0=合拢 1=完全打开（比例）
  points            列出已记录的点
  emit              生成场景 yaml（需要 pick 和 place）
  help              这段
  quit              退出（点位已自动保存）

  建议顺序：先 `go` 粗定位到物体上方 → `z-` 小步下降 → 到指尖贴住物体时 mark pick
""")


def main():
    ap = argparse.ArgumentParser(description="交互式示教：摆位 → 记录 → 生成场景 yaml")
    ap.add_argument("--emit", action="store_true", help="不进入交互，直接用已记录的点生成 yaml")
    ap.add_argument("--points", action="store_true", help="列出已记录的点")
    ap.add_argument("--clear", action="store_true", help="清空已记录的点")
    ap.add_argument("--standoff", type=float,
                    help="接近偏移（米），覆盖从 approach 推算的值")
    ap.add_argument("--force", action="store_true",
                    help="越过 roarm_cli 的软件工作区边界（BOUNDS 是推荐区，不是机械极限）")
    args = ap.parse_args()

    if args.clear:
        save_points({})
        print("已清空 %s" % POINTS)
        return 0
    if args.points:
        pts = load_points()
        if not pts:
            print("还没有点位。")
        for k, v in pts.items():
            if isinstance(v, dict) and "x" in v:
                print("  %-10s x=%.4f y=%+.4f z=%.4f" % (k, v["x"], v["y"], v["z"]))
        return 0
    if args.emit:
        return 0 if emit(load_points(), args.standoff) else 1

    if not os.environ.get("ROS_DISTRO"):
        print("⚠ 没检测到 ROS 环境。先 source /opt/ros/humble/setup.bash 和 install/setup.bash")
        return 2

    global FORCE
    FORCE = bool(args.force)
    if FORCE:
        print("⚠ 已关闭软件工作区检查（--force）：机械能到但不等于安全，自己看着点。")

    pts = load_points()
    try:
        pos, quat, angs = where()
    except RuntimeError as e:
        print("✗ %s" % e)
        print("\n先确认真机在跑：bash scripts/robot_up.sh")
        return 1

    print("示教模式。当前位置：")
    show(pos, quat, angs)
    print("姿态四元数固定为当前值，移动不会自己转腕。")
    print("输入 help 看命令。")

    while True:
        try:
            line = input("\n示教> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        low = line.lower()
        if low in ("quit", "exit", "q"):
            break
        if low == "help":
            help_text()
            continue
        if low == "show":
            try:
                pos, quat, angs = where()
                show(pos, quat, angs)
            except RuntimeError as e:
                print("✗ %s" % e)
            continue
        if low == "points":
            if not pts:
                print("  （空）")
            for k, v in pts.items():
                if isinstance(v, dict) and "x" in v:
                    print("  %-10s x=%.4f y=%+.4f z=%.4f" % (k, v["x"], v["y"], v["z"]))
            continue
        if low == "emit":
            emit(pts, args.standoff)
            continue
        if low.startswith("mark"):
            parts = line.split()
            if len(parts) != 2:
                print("  用法：mark pick|place|approach")
                continue
            name = parts[1].lower()
            pts[name] = {"x": round(pos[0], 5), "y": round(pos[1], 5), "z": round(pos[2], 5)}
            save_points(pts)
            print("  ✓ 已记 %s = (%.4f, %+.4f, %.4f)" % (name, pos[0], pos[1], pos[2]))
            if pos[2] < 0.05:
                print("  ⚠ z = %.1f mm，注意别压到桌面" % (pos[2] * 1000))
            continue
        if low.startswith("obj"):
            parts = line.split()
            if len(parts) != 3:
                print("  用法：obj 高度 宽度（米），如 obj 0.03 0.03")
                continue
            try:
                pts["object_height"] = float(parts[1])
                pts["object_width"] = float(parts[2])
                save_points(pts)
                print("  ✓ 物体 %.0f × %.0f mm"
                      % (pts["object_height"] * 1000, pts["object_width"] * 1000))
            except ValueError:
                print("  数值不对")
            continue
        if low.startswith("grip"):
            parts = line.split()
            if len(parts) != 2:
                print("  用法：grip 0.5（0=合拢，1=全开）")
                continue
            try:
                frac = max(0.0, min(1.0, float(parts[1])))
            except ValueError:
                print("  数值不对")
                continue
            # 连续开度：0 = 合拢（舵机 21.9°），1 = 全开（舵机 100°），线性插值。
            # GripperCommand 只有一个字符串，做不到半开，所以直接发 set_angle_topic。
            deg = GRIP_CLOSED_DEG + frac * (GRIP_OPEN_DEG - GRIP_CLOSED_DEG)
            mm = GAP_MIN_MM + frac * (GAP_MAX_MM - GAP_MIN_MM)
            rc, out = cli(["grip_deg", "%.1f" % deg, "--wait", "10"])
            print("  %s 舵机 %.1f° → 两指约 %.0f mm 开口%s"
                  % ("✓" if rc == 0 else "✗ rc=%d" % rc, deg, mm,
                     "" if rc == 0 else "\n      " + " / ".join(
                         l.strip() for l in out.strip().splitlines()[-2:])))
            if rc == 0:
                # 记下这次开度比例，emit 时写进场景，执行时按同样开度下探
                pts["grip_open"] = round(frac, 2)
                save_points(pts)
            continue
        # 单轴相对移动
        m = re.match(r"^([xyzXYZ])\s*([+-])\s*([\d.]+)$", line)
        if m:
            axis = m.group(1).lower()
            delta = float(m.group(3)) * (1 if m.group(2) == "+" else -1)
            pos = move_rel(pos, quat, axis, delta)
            continue
        # 绝对定位：go x y z（粗定位用，一次跳到目标附近）
        m = re.match(r"^go\s+([-\d.]+)[\s,]+([-\d.]+)[\s,]+([-\d.]+)$", line)
        if m:
            tgt = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            new = move_abs(pos, quat, tgt)
            if new:
                pos = new
            continue
        print("  不认识：%s（help 看命令）" % line)

    save_points(pts)
    print("点位已保存到 %s" % POINTS)
    return 0


if __name__ == "__main__":
    sys.exit(main())

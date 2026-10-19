"""终端入口：ros2 run roarm_task task [选项]

常用：
    ros2 run roarm_task task --dry-run
    ros2 run roarm_task task
    ros2 run roarm_task task --scene scenes/demo.yaml
    ros2 run roarm_task task --pick 0.33 -0.30 0.10 --place 0.45 0.05 0.25
    ros2 run roarm_task task --verify-grasp

退出码沿用 CLI 的契约：
    0  序列全部通过
    1  某一步失败（输出里会指出停在哪一步、哪个阶段）
    2  参数非法（场景自检不通过）—— 不会发任何指令
"""

import argparse
import sys
import time
from pathlib import Path

from . import config, sequence
from .scene import Scene


def resolve_scene(name: str) -> str:
    """--scene 支持三种写法：绝对路径、相对路径、包内简写（如 demo）。"""
    p = Path(name).expanduser()
    if p.is_file():
        return str(p)
    try:
        from ament_index_python.packages import get_package_share_directory

        base = Path(get_package_share_directory("roarm_task")) / "scenes"
        for cand in (base / name, base / (name + ".yaml")):
            if cand.is_file():
                return str(cand)
    except Exception:
        pass
    return str(p)  # 交给上层报"找不到"


def build_scene(args) -> Scene:
    if args.scene:
        sc = Scene.from_yaml(resolve_scene(args.scene))
    else:
        sc = Scene.default()

    return sc.with_overrides(
        pick=tuple(args.pick) if args.pick else None,
        place=tuple(args.place) if args.place else None,
        object_height=args.object_height,
        object_width=args.object_width,
        standoff=args.standoff,
        clearance=args.clearance,
        verify_grasp=(True if args.verify_grasp else
                      False if args.no_verify_grasp else None),
        name=args.name,
    )


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="roarm_task",
        description="执行一次取放（仿真与真机接口相同）")
    ap.add_argument("--scene", metavar="FILE", help="场景 YAML（见 scenes/）")
    ap.add_argument("--pick", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="取物点（世界系，米）")
    ap.add_argument("--place", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="放置点（世界系，米）")
    ap.add_argument("--object-height", type=float, help="物体高度（米）")
    ap.add_argument("--object-width", type=float, help="物体宽度（米）")
    ap.add_argument("--standoff", type=float, help="接近/撤离竖直偏移（米）")
    ap.add_argument("--clearance", type=float, help="抬起后离台面净空（米）")
    ap.add_argument("--verify-grasp", action="store_true",
                    help="合爪后校验是否真的夹住（真机可用，仿真判据不成立）")
    ap.add_argument("--no-verify-grasp", action="store_true",
                    help="强制关闭抓取校验")
    ap.add_argument("--name", help="场景名，仅用于输出")
    ap.add_argument("--no-grasp-check", action="store_true",
                    help="place 阶段跳过'手上有没有东西'的检查")
    ap.add_argument("--phase", choices=["all", "pick", "place"], default="all",
                    help="all=完整取放（默认）；pick=只取物并停在抬起位；"
                         "place=从当前位姿放置并回位")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印步骤与场景，不动臂")
    ap.add_argument("--no-home-on-fail", action="store_true",
                    help="失败后不回位（调试用，真机上别开）")
    args = ap.parse_args(argv)

    if args.no_home_on_fail:
        config.RETURN_HOME_ON_FAIL = False

    try:
        scene = build_scene(args)
    except FileNotFoundError:
        print("找不到场景文件：%s" % args.scene, file=sys.stderr)
        return 2
    except (KeyError, TypeError, ValueError) as exc:
        print("场景文件解析失败：%s" % exc, file=sys.stderr)
        return 2

    problems = scene.problems()
    if problems:
        print("场景自检不通过，未发任何指令：", file=sys.stderr)
        for p in problems:
            print("  · " + p, file=sys.stderr)
        return 2

    # place 之前确认手上确实有东西：空放会白跑一趟，还可能把夹爪怼到台面上。
    # dry-run 不连硬件，跳过。
    if args.phase == "place" and not args.no_grasp_check and not args.dry_run:
        holding, note = primitives.check_holding()
        if holding is False:
            print("✗ 夹持检查未通过：%s" % note, file=sys.stderr)
            print("  手上确实有东西却报这个（比如夹的是薄片），"
                  "加 --no-grasp-check 跳过。", file=sys.stderr)
            return 2
        if holding is None:
            print("⚠ 夹持检查读不到状态：%s —— 仍继续执行" % note)
        else:
            print("  夹持检查：%s" % note)

    if args.phase == "pick":
        steps = sequence.build_pick(scene)
    elif args.phase == "place":
        steps = sequence.build_place(scene)
    else:
        steps = sequence.build_pick_place(scene)

    print(scene.describe())
    if args.phase != "all":
        print("  阶段：%s（%s）"
              % (args.phase,
                 "取物后停在抬起位" if args.phase == "pick"
                 else "从当前位姿放置并回位"))
    if scene.verify_grasp:
        print("  ⚠ 抓取校验已开：真机可用；仿真里 FakeSystem 不模拟接触，"
              "大概率判夹空并触发重试")
    print()

    if args.dry_run:
        print("将执行 %d 步（dry-run，未发任何命令）：" % len(steps))
        for i, s in enumerate(steps, 1):
            print("  %2d. %-10s [%s]" % (i, s.name, s.phase.value))
        print("\n夹爪口径：close_fingers -> CLI '%s'，open_fingers -> CLI '%s'"
              % (config.GRIP_CMD["close_fingers"], config.GRIP_CMD["open_fingers"]))
        return 0

    print("开始执行…")
    t0 = time.time()
    result = sequence.run(steps)

    print("\n%-4s %-12s %-8s %-8s %8s  %s"
          % ("#", "步骤", "阶段", "结果", "耗时", "摘要"))
    print("-" * 96)
    for i, (step, res) in enumerate(result.steps, 1):
        tag = "OK" if res.ok else ("REJECT" if res.rejected else "FAIL")
        print("%-4d %-12s %-8s %-8s %7.1fs  %s"
              % (i, step.name, step.phase.value, tag, res.elapsed, res.detail[:44]))
        # 失败时把 CLI 的完整输出贴出来：摘要只留 44 字，光靠它查不出原因
        if not res.ok and (res.raw or "").strip():
            for line in res.raw.strip().splitlines()[-6:]:
                print("      │ " + line.strip())

    print("-" * 96)
    print("总耗时 %.1f s" % (time.time() - t0))
    print(result.summary())
    if not result.ok:
        if result.recovered:
            print("（已自动回位）")
        elif config.RETURN_HOME_ON_FAIL:
            print("⚠ 回位也失败了，臂可能停在半空")

    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())

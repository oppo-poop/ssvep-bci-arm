#!/usr/bin/env python3
"""pick.py 精度 / 边界逻辑的离线回归自检（不需要 ROS / 相机 / 真机）。

    python3 scripts/checks/pick_check.py

用假模块（cv2 / camera / project）把 pick.py 加载进来，替换掉会起子进程、
读相机的函数，直接验证：

  1. cli_pose：x/y 超出 roarm_cli 的窄「任务工作区」时自动补 --force。
     2026-11-03 投放点 link6 (571,181) 就是栽在两套边界不一致：投放那步
     因 z 超界碰巧加了 force，后面抬升/回位全被拒，一路「z=读不到」。
  2. cli_pose_zfix：up=True（投放）连「落点偏低」也补，落点误差变小；
     up=False（抓取）保持只补偏高。
  3. 补发用短 wait（<=25s），不再干等 90s；补偿后更远就不再继续。
  4. 三处 home 命令不再带不存在的 --close-grip（那会让整条复位不执行）。

改落点补偿或边界后跑一次。真机效果仍要链神确认（Agent 看不到 /dev）。
"""
import contextlib
import importlib.util
import io
import sys
import types
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parents[1]
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from roarm_lib import params, paths  # noqa: E402

PICK = paths.SCRIPTS / "vision" / "pick.py"


def _stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


_stub("cv2")
_stub("camera", find_camera_index=lambda *a, **k: 0,
      grab_frame=lambda *a, **k: None, load_scene_z=lambda *a, **k: None)
_stub("project", pixel_to_base=lambda *a, **k: None)

spec = importlib.util.spec_from_file_location("pick_under_test", PICK)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
np = mod.np

fails = []


def check(cond, label):
    print("  %s %s" % ("✓" if cond else "✗", label))
    if not cond:
        fails.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


# ---------------------------------------------------------------- 1) 边界
print("[1] cli_pose：两套边界一致，超 CLI 推荐区自动补 --force")
captured = []


def fake_run_cli(args, timeout=45):
    captured.append(list(args))
    return 0, ""


mod.run_cli = fake_run_cli
mod.show = lambda out: None
mod.parse_actual = lambda out: None
mod.parse_err = lambda out: 0.0          # 触发不了 settle 复查

real_cli_pose = mod.cli_pose
TGT = [571.0, 181.0, 72.0]               # 日志里的投放点 link6(mm)

rc, _ = quiet(real_cli_pose, TGT, 90.0, 15.0, True, False)
check("--force" in captured[-1],
      "投放点 (571,181,72)：z 超界 + x/y 超 CLI 区 → 自动加 --force")

captured.clear()
rc2, _ = quiet(real_cli_pose, [571.0, 181.0, 97.0], 90.0, 15.0, True, False)
check("--force" in captured[-1],
      "松爪抬升 (571,181,97)：z 正常但 x/y 超 CLI 区 → 也自动加 --force")

captured.clear()
rc3, _ = quiet(real_cli_pose, [400.0, -100.0, 150.0], 90.0, 15.0, True, False)
check("--force" not in captured[-1],
      "正常区内点 (400,-100,150)：不需要 --force")

captured.clear()
rc4, _ = quiet(real_cli_pose, TGT, 90.0, 15.0, False, False)
check(rc4 == 2 and not captured,
      "--no-force 时超界直接放弃，不再发一条注定被 CLI 拒绝的命令")


# ---------------------------------------------------------------- 2) zfix
print("[2] cli_pose_zfix：up=True 双向补偿；up=False 只补偏高")
bias = [-13.0]                            # 实测：命令发 72，落点 59（低 13）
calls = []


def fixed_bias_cli_pose(target, wait, tol, force, allow_far=False,
                        retries=2, settle=True):
    t = np.asarray(target, dtype=np.float64)
    calls.append((t.copy(), wait))
    return 0, t + np.array([0.0, 0.0, bias[0]])


mod.cli_pose = fixed_bias_cli_pose
target = np.array([480.0, -140.0, 72.0])

calls.clear()
rc, act = quiet(mod.cli_pose_zfix, target, 90.0, 15.0, True, up=False)
check(len(calls) == 1 and abs(act[2] - 59.0) < 1e-6,
      "up=False（保守模式）：偏低 13mm 不补，落点保持 59")

calls.clear()
rc, act = quiet(mod.cli_pose_zfix, target, 90.0, 15.0, True, up=True)
check(len(calls) == 2, "up=True（投放）：偏低触发一次补发")
check(abs(act[2] - 67.0) < 1e-6,
      "补 8mm 后落点 59 → 67，误差 13 → 5mm")
check(calls[1][1] == 5.0, "补发 wait = 5s（min(90,5)），小修正不干等")

# 偏高侧：原本就补，别回归
calls.clear()
bias[0] = 13.0
rc, act = quiet(mod.cli_pose_zfix, target, 90.0, 15.0, True, up=False)
check(len(calls) == 2 and abs(act[2] - 77.0) < 1e-6,
      "偏高侧（bias +13）仍会补：落点 85 → 77，误差 13 → 5mm")
bias[0] = -13.0

# 补偿后反而更远 → 不再继续补
worse_calls = []


def worse_cli_pose(target, wait, tol, force, allow_far=False,
                   retries=2, settle=True):
    t = np.asarray(target, dtype=np.float64)
    worse_calls.append(t.copy())
    delta = -13.0 if len(worse_calls) == 1 else -30.0
    return 0, t + np.array([0.0, 0.0, delta])


mod.cli_pose = worse_cli_pose
worse_calls.clear()
rc, act = quiet(mod.cli_pose_zfix, target, 90.0, 15.0, True, up=True, iters=3)
check(len(worse_calls) == 2, "补偿后反而更远 → 立刻停，不再补第三次")


# ---------------------------------------------------------------- 3) lift_to
print("[3] lift_to：达到 min_z 后不再补发第二条命令")
lift_calls = []


def lift_high_cli_pose(target, wait, tol, force, allow_far=False,
                       retries=2, settle=True):
    lift_calls.append(np.asarray(target, dtype=np.float64).copy())
    return 0, np.array([target[0], target[1], 100.0])


args = types.SimpleNamespace(wait=90.0, no_force=False, allow_far=False)
mod.cli_pose = lift_high_cli_pose
lift_calls.clear()
rc, act = quiet(mod.lift_to, np.array([480.0, -140.0, 130.0]), args,
                tol=22.0, min_z=92.0)
check(len(lift_calls) == 1, "第一次就达到 min_z=92 → 不补发")


def lift_low_cli_pose(target, wait, tol, force, allow_far=False,
                      retries=2, settle=True):
    lift_calls.append(np.asarray(target, dtype=np.float64).copy())
    z = 80.0 if len(lift_calls) == 1 else 82.0
    return 0, np.array([target[0], target[1], z])


mod.cli_pose = lift_low_cli_pose
lift_calls.clear()
rc, act = quiet(mod.lift_to, np.array([480.0, -140.0, 130.0]), args,
                tol=22.0, min_z=92.0)
check(len(lift_calls) == 2, "低于 min_z 才补发一次")


# ---------------------------------------------------------------- 3.5) 夹空判定
print("[3.5] 夹空判定")
check(mod.grasp_closed_ok(1, 16.5), "rc=1、误差 16.5mm → 判夹住")
check(not mod.grasp_closed_ok(0, 0.1), "rc=0、误差 0.1mm → 判空夹")
check(not mod.grasp_closed_ok(1, 1.0), "误差 1.0mm → 太薄/空夹")
check(not mod.grasp_closed_ok(1, 23.0), "误差 23mm → 没夹进指间")
check(not mod.grasp_closed_ok(1, None), "读不到误差 → 不判夹住")


# ---------------------------------------------------------------- 3.6) 人工偏差
print("[3.6] 人工测量偏差 → 新夹爪偏移")
off_old = np.array([50.6, 23.6, 89.2], dtype=np.float64)
R = mod.quat_to_R(mod.DEFAULT_QUAT)
off_new, corr = mod.offset_after_measurement(off_old, R, 10.0, -5.0)
world_old = R @ off_old
world_new = R @ off_new
check(np.allclose(corr, [-10.0, 5.0, 0.0]),
      "指尖偏 +10/-5 → 目标反向修正 -10/+5")
check(np.allclose(world_new, world_old + corr),
      "修正量在世界系叠加后与局部偏移一致")


# ---------------------------------------------------------------- 4) 静态
print("[4] 静态检查：home 命令不再带 --close-grip")
src = PICK.read_text(encoding="utf-8")
bad = [ln.strip() for ln in src.splitlines()
       if "--close-grip" in ln and not ln.lstrip().startswith("#")]
check(not bad, "只剩注释里的防再犯说明，实际命令已清空")


# ---------------------------------------------------------------- 5) 参数
print("[5] 参数单源：pick.py 的边界/姿态与 config/robot.json 一致")
check(dict(mod.CLI_BOUNDS) == dict(params.BOUNDS),
      "CLI_BOUNDS == robot.json arm.bounds")
check(tuple(mod.DEFAULT_QUAT) == tuple(params.DEFAULT_QUAT),
      "DEFAULT_QUAT == robot.json arm.default_quat")
check(dict(mod.BOUNDS) == dict(params.REACH_BOUNDS),
      "BOUNDS == vision.json reach")
check(tuple(mod.DEFAULT_OFFSET_MM) == tuple(params.GRASP_OFFSET_DEFAULT_MM),
      "DEFAULT_OFFSET_MM == vision.json grasp.offset_default_mm")

print()
if fails:
    print("✗ %d 项不通过：" % len(fails))
    for f in fails:
        print("    - " + f)
    sys.exit(1)
print("✓ 全部通过。")

#!/usr/bin/env bash
# run_task —— 跑一次取放，日志与结果自动归档（不再互相覆盖）
#
# 为什么需要它：直接用 `task ... | tee /tmp/run1.log` 时，第二次跑就把第一次覆盖了，
# 失败那次的输出永远找不回来。这里每次运行单独一个带时间戳的文件，
# 并往 summary.csv 追加一行结果，连跑 N 次后能直接看成功率。
#
# 用法：
#   bash scripts/run_task.sh                       # 默认场景 data/scene_real.yaml
#   bash scripts/run_task.sh path/to/scene.yaml    # 指定场景
#   bash scripts/run_task.sh --n 5                 # 连跑 5 次（每次一份日志）
#
# 产出：
#   data/runs/MMDD-HHMMSS.log    完整输出
#   data/runs/summary.csv        每次一行：时间/场景/结果/总耗时/失败步

set -u
cd "$(dirname "$0")/.." || exit 1

SCENE="data/scene_real.yaml"
N=1
while [ $# -gt 0 ]; do
  case "$1" in
    --n) N="$2"; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) SCENE="$1"; shift ;;
  esac
done

mkdir -p data/runs
SUM="data/runs/summary.csv"
[ -f "$SUM" ] || echo "time,scene,result,total_s,failed_at,log" > "$SUM"

fail=0
for i in $(seq 1 "$N"); do
  OUT="data/runs/$(date +%m%d-%H%M%S).log"
  echo "───────── 第 $i/$N 次  →  $OUT ─────────"

  ros2 run roarm_task task --scene "$SCENE" 2>&1 | tee "$OUT"
  rc=${PIPESTATUS[0]}

  # 从输出里提取摘要（task 的固定格式："总耗时 xx s" / "全部 N 步通过" / "在「X」…失败"）
  total=$(grep -oE "总耗时 [0-9.]+ s" "$OUT" | tail -1 | grep -oE "[0-9.]+")
  result=$(grep -E "全部 [0-9]+ 步通过|在「" "$OUT" | tail -1 | sed 's/^[[:space:]]*//')
  # ⚠ 不能用 tr -d 删中文：它按字节操作，会把多字节字符切坏，写进 CSV 就是乱码
  failed=$(printf '%s' "$result" | sed -n 's/.*在「\([^」]*\)」.*/\1/p')
  [ -z "$failed" ] && failed="-"
  [ -z "$result" ] && result="（无摘要，rc=$rc）"
  [ -z "$total" ] && total="-"

  # 失败步名里可能含逗号，CSV 里加引号
  echo "$(date +%m-%d\ %H:%M:%S),$SCENE,\"$result\",$total,\"$failed\",$OUT" >> "$SUM"

  [ "$rc" -eq 0 ] || fail=$((fail + 1))
  sleep 2
done

echo
echo "════════════════════════════════════"
if [ "$N" -gt 1 ]; then
  echo "共 $N 次，失败 $fail 次，成功 $((N - fail)) 次"
fi
echo "汇总：$SUM"
echo "明细：data/runs/"

#!/usr/bin/env bash
# 仓库隐私清理 — 把脑电原始数据从 ssvep-bci 的 git 追踪中摘出
#
# 用法：
#   bash scripts/clean_git_privacy.sh                     # 默认 dry-run：只体检 + 打印计划
#   bash scripts/clean_git_privacy.sh --apply-phase1      # 执行阶段 1（安全，不重写历史，可回滚）
#   bash scripts/clean_git_privacy.sh --apply-phase2 --i-coordinated-with-teammates
#                                                         # 执行阶段 2（重写历史，危险）
#
# 阶段划分：
#   阶段 1  补 .gitignore 漏网规则 + git rm --cached  → 只改追踪状态，普通 push 即可
#   阶段 2  git filter-repo 重写历史                  → 全部 commit hash 变化，必须 force push，
#                                                      且队友必须重新 clone
#
# 依据（2026-09-22 实测）：
#   - data/ 下 14 个 .npz 已被追踪并推到 GitHub（私有仓库）
#   - 其中 5 个 calib_*.npz 不被任何 .gitignore 规则覆盖 ← 只删不补规则 = 下次 add 又会回来
#   - 引入 data/ 的提交只有 1 个（共 36 个提交）
#   - 存在 3 个远程分支：main / cjh / ican-bci（有队友）
#   - git-filter-repo 未安装

set -euo pipefail

REPO="${REPO:-/home/ljl/BCI/DeepBCI-8ch/ssvep-bci}"
BACKUP_DIR="${BACKUP_DIR:-/home/ljl/git-backups}"
STAMP="$(date +%Y%m%d-%H%M%S)"

MODE="dry-run"
ACK_TEAM=0

for arg in "$@"; do
  case "$arg" in
    --apply-phase1) MODE="phase1" ;;
    --apply-phase2) MODE="phase2" ;;
    --i-coordinated-with-teammates) ACK_TEAM=1 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "未知参数：$arg（用 --help 看用法）"; exit 2 ;;
  esac
done

say()  { printf '%s\n' "$*"; }
head1() { printf '\n=== %s ===\n' "$*"; }
abort() { printf '\n[中止] %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 前置检查
preflight() {
  [ -d "$REPO/.git" ] || abort "找不到 git 仓库：$REPO"
  git -C "$REPO" rev-parse --abbrev-ref HEAD >/dev/null 2>&1 || abort "不是 git 仓库：$REPO"
}

# ---------------------------------------------------------------- 体检
diagnose() {
  head1 "① 当前分支与远程"
  git -C "$REPO" branch -a
  say "标签数：$(git -C "$REPO" tag | wc -l)"

  head1 "② data/ 下已被追踪的脑电数据"
  local files
  mapfile -t files < <(git -C "$REPO" ls-files -- 'data/*.npz' || true)
  if [ "${#files[@]}" -eq 0 ]; then
    say "（无 —— 已经清理过了）"
  else
    local total=0
    for f in "${files[@]}"; do
      local size
      size=$(du -k "$REPO/$f" 2>/dev/null | cut -f1 || echo 0)
      total=$(( total + size ))
      printf '  %8d KB  %s\n' "$size" "$f"
    done
    printf '  ---- 合计约 %d KB / %d 个文件\n' "$total" "${#files[@]}"
  fi

  head1 "③ 忽略规则覆盖检查（--no-index，包含已追踪文件）"
  local npz
  mapfile -t npz < <(cd "$REPO" && ls data/*.npz 2>/dev/null || true)
  if [ "${#npz[@]}" -eq 0 ]; then
    say "（data/ 下无 .npz）"
  else
    ( cd "$REPO" && git check-ignore -v --no-index data/*.npz 2>/dev/null || true )
    say ""
    say "⬆ 没出现在上面的文件 = 不被任何规则覆盖，删掉后会被重新 add 回来："
    for f in "${npz[@]}"; do
      git -C "$REPO" check-ignore -q --no-index "$f" || printf '  ⚠ %s\n' "$f"
    done
  fi

  head1 "④ 其它「已追踪但已被 .gitignore 声明忽略」的文件（非隐私，属卫生问题）"
  git -C "$REPO" ls-files -i -c --exclude-standard 2>/dev/null | grep -v '^data/.*\.npz$' || say "（无）"

  head1 "⑤ 历史体积前 10"
  git -C "$REPO" rev-list --objects --all 2>/dev/null \
    | git -C "$REPO" cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' 2>/dev/null \
    | awk '$1=="blob"' | sort -k3 -nr | head -10 \
    | awk '{printf "  %8.2f MB  %s\n", $3/1048576, $4}'
  say "  .git 目录：$(du -sh "$REPO/.git" | cut -f1)"

  head1 "⑥ 引入 data/ 的提交数"
  say "  $(git -C "$REPO" log --oneline -- data/ | wc -l) 个（总共 $(git -C "$REPO" log --oneline | wc -l) 个提交）"
}

# ---------------------------------------------------------------- 备份
backup() {
  mkdir -p "$BACKUP_DIR"
  local out="$BACKUP_DIR/ssvep-bci-$STAMP.bundle"
  head1 "备份"
  say "  生成完整 bundle（含全部分支与历史）：$out"
  git -C "$REPO" bundle create "$out" --all >/dev/null
  say "  ✅ 完成，大小 $(du -h "$out" | cut -f1)"
  say "  校验：git bundle verify \"$out\""
  say "  还原：git clone \"$out\" <新目录>"
}

# ---------------------------------------------------------------- 补规则
patch_gitignore() {
  head1 "补 .gitignore 漏网规则"
  if grep -q '^data/\*\.npz$' "$REPO/.gitignore" 2>/dev/null; then
    say "  规则 data/*.npz 已存在，跳过"
    return
  fi
  cat >> "$REPO/.gitignore" <<'EOF'

# ── 脑电原始数据（data/ 下全部 .npz）──
# 2026-09-22 补：原有 ssvep_calib.npz / snr_*.npz 两条漏掉了 calib_*.npz 这类命名
data/*.npz
EOF
  say "  ✅ 已追加 data/*.npz"
}

# ---------------------------------------------------------------- 阶段 1
phase1() {
  head1 "阶段 1：停止追踪脑电数据（不重写历史）"

  local dirty
  dirty=$(git -C "$REPO" status --porcelain | wc -l)
  if [ "$dirty" -ne 0 ]; then
    say "⚠ 工作区有 $dirty 处未提交改动，建议先 commit 或 stash："
    git -C "$REPO" status --short
    abort "工作区不干净，阶段 1 暂停"
  fi

  backup
  patch_gitignore

  head1 "git rm --cached（本地文件保留，只退出追踪）"
  local files
  mapfile -t files < <(git -C "$REPO" ls-files -- 'data/*.npz' || true)
  if [ "${#files[@]}" -eq 0 ]; then
    say "  无可移除文件"
  else
    git -C "$REPO" rm --cached --quiet -- "${files[@]}"
    say "  已移除 ${#files[@]} 个文件的追踪"
  fi

  head1 "改动摘要"
  git -C "$REPO" diff --cached --stat | tail -3
  git -C "$REPO" status --short | head -20

  read -r -p $'\n提交这次改动？[y/N] ' ans
  if [ "${ans:-N}" != "y" ] && [ "${ans:-N}" != "Y" ]; then
    say "已暂存但未提交。手动提交：git -C \"$REPO\" commit -m '...'"
    return
  fi
  git -C "$REPO" add .gitignore
  git -C "$REPO" commit -q -m "chore: 停止追踪脑电原始数据 data/*.npz 并补全 .gitignore 漏网规则"
  say "  ✅ 已本地提交 $(git -C "$REPO" rev-parse --short HEAD)"

  say ""
  say "回滚这一次提交（如需）：git -C \"$REPO\" reset --soft HEAD~1"
  say "推送到私有仓库（你自己跑，脚本不代推）："
  say "  git -C \"$REPO\" push origin main"
  say ""
  say "⚠ 注意：这一步之后 HEAD 不再含脑电数据，但**历史里的旧提交仍可 git show 出来**。"
  say "   想彻底清历史才需要阶段 2。"
}

# ---------------------------------------------------------------- 阶段 2
phase2() {
  head1 "阶段 2：重写历史（危险）"

  [ "$ACK_TEAM" -eq 1 ] || abort "缺少 --i-coordinated-with-teammates。
  本仓库有 3 个远程分支（main / cjh / ican-bci），重写历史 + force push 会让队友的本地
  仓库、分支、未推送提交全部失配。**必须先与队友沟通并让他们备份。**"

  local dirty
  dirty=$(git -C "$REPO" status --porcelain | wc -l)
  [ "$dirty" -eq 0 ] || abort "工作区有 $dirty 处未提交改动，先处理干净再重写历史"

  if ! git filter-repo --version >/dev/null 2>&1; then
    abort "git-filter-repo 未安装。先装：
  pip3 install --user git-filter-repo
  # 或：sudo apt install git-filter-repo
  装完确认：git filter-repo --version"
  fi

  say "重置性操作：会重写全部 36 个提交的 hash。"
  read -r -p '确认继续？输入 yes 全小写：' ans
  [ "$ans" = "yes" ] || abort "已取消"

  backup

  head1 "重写历史：移除所有 *.npz 与 data/ 目录"
  git -C "$REPO" filter-repo --force \
    --path data/ --path-glob '*.npz' --invert-paths

  say ""
  say "⚠ git-filter-repo 会自动删除 origin 远程（防误推），需要手动加回："
  say "  git -C \"$REPO\" remote add origin git@github.com:oppo-poop/ssvep-bci.git"
  say ""
  say "瘦身："
  say "  git -C \"$REPO\" reflog expire --expire=now --all"
  say "  git -C \"$REPO\" gc --prune=now --aggressive"
  say ""
  say "强制推送（你自己跑，脚本不代推）——三个分支都要推："
  say "  git -C \"$REPO\" push --force --all"
  say "  git -C \"$REPO\" push --force --tags"
  say ""
  say "队友侧：重新 clone，不要试图 pull 合并。"
  say "验证历史已清：git -C \"$REPO\" log --all --oneline -- data/   # 应为空"
}

# ---------------------------------------------------------------- 主流程
preflight
case "$MODE" in
  dry-run) diagnose
    head1 "计划（dry-run，未改动任何文件）"
    say "  阶段 1  bash scripts/clean_git_privacy.sh --apply-phase1"
    say "    补 .gitignore 的 data/*.npz → git rm --cached data/*.npz → 本地提交"
    say "    风险：低。本地文件保留，回滚 = git reset --soft HEAD~1"
    say "  阶段 2  bash scripts/clean_git_privacy.sh --apply-phase2 --i-coordinated-with-teammates"
    say "    重写全部历史 + force push。风险：高，必须与队友协调"
    say ""
    say "  可选（仓库卫生，非隐私，自己决定）："
    say "    git -C \"$REPO\" rm --cached -r stim_freqs.json stm32_car/.vscode docs/patent/.patent_build"
    ;;
  phase1) diagnose; phase1 ;;
  phase2) diagnose; phase2 ;;
esac

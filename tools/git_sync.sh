#!/usr/bin/env bash
# ============================================================
# 一键提交并推送（给不熟悉 Git 的人用）
#
# 用法：
#   ./tools/git_sync.sh "docs: 补充左手接线照片"
#
# 它会做这些事（每步都会告诉你结果）：
#   1. 检查提交说明格式
#   2. 检查提交身份是否配置
#   3. 扫描大文件（>5MB）和疑似密钥，有就拦下来
#   4. 显示 git status，让你确认改动清单
#   5. add → commit → pull --rebase → push
#
# 设计原则：宁可拦下来让你手动处理，也不要往公司仓库里塞脏东西。
# ============================================================

set -euo pipefail

# ── 配色 ──────────────────────────────────────────────
if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'
  BLUE=$'\033[34m'; BOLD=$'\033[1m'; RESET=$'\033[0m'
else
  RED=""; GREEN=""; YELLOW=""; BLUE=""; BOLD=""; RESET=""
fi

info()  { printf '%s\n' "${BLUE}[信息]${RESET} $*"; }
ok()    { printf '%s\n' "${GREEN}[通过]${RESET} $*"; }
warn()  { printf '%s\n' "${YELLOW}[注意]${RESET} $*"; }
err()   { printf '%s\n' "${RED}[中止]${RESET} $*" >&2; }
step()  { printf '\n%s\n' "${BOLD}── $* ──${RESET}"; }

die() { err "$*"; exit 1; }

# ── 0. 必须在 git 仓库里 ──────────────────────────────
step "0/6 环境检查"
git rev-parse --is-inside-work-tree >/dev/null 2>&1 \
  || die "当前目录不是 Git 仓库。请先 cd 到仓库目录再运行本脚本。"
# 路径风格要统一：Git for Windows 的 git 可能返回 "D:/..."，
# 而 shell 的 pwd 返回 "/d/..."。两者前缀风格不一致时，下面的 MY_REL 推导会失败，
# MY_REL 退化成绝对路径，后续 `git add -- <该路径>` 就报
# `fatal: Invalid path '/d'`。这里统一规范成 POSIX 风格（cd 走一遍再 pwd）。
REPO_ROOT="$(cd "$(git rev-parse --show-toplevel)" && pwd)"

# 关键：本次只操作「本脚本所在的那块目录」，绝不碰同事的文件。
# 脚本位于 <仓库>/<你的子目录>/tools/git_sync.sh，所以它的上级目录就是你的地盘。
# 用 bash 参数扩展取目录名，不依赖外部 dirname（个别精简 shell 环境里没有该命令）。
SCRIPT_DIR="$(cd "${BASH_SOURCE[0]%/*}" && pwd)"
MY_DIR="${SCRIPT_DIR%/*}"
if [ "$MY_DIR" = "$REPO_ROOT" ]; then
  MY_REL="."
else
  MY_REL="${MY_DIR#"$REPO_ROOT"/}"
fi

cd "$REPO_ROOT"
info "仓库根目录：$REPO_ROOT"
info "本次提交范围：$MY_REL"
if [ "$MY_REL" = "." ]; then
  warn "本脚本位于仓库根目录，因此本次会提交整个仓库的改动。"
  warn "资料并入 elf3-humanoid 之后，请在子目录里运行本脚本，以免误提交同事的文件。"
fi

# ── 1. 提交说明格式 ───────────────────────────────────
step "1/6 提交说明检查"
MSG="${1:-}"
if [ -z "$MSG" ]; then
  warn "没有提供提交说明。"
  printf '请输入提交说明（格式建议：类型: 说明，例如 "docs: 补充接线照片"）：\n'
  read -r MSG
fi
[ -n "$MSG" ] || die "提交说明不能为空。"

# 校验类型前缀
if ! printf '%s' "$MSG" | grep -qE '^(feat|fix|docs|refactor|chore|test|style|perf)(\(.+\))?: .+'; then
  warn "提交说明没有使用规范的「类型: 说明」格式。"
  printf '  推荐类型：feat / fix / docs / refactor / chore / test / style / perf\n'
  printf '  你的说明：%s\n' "$MSG"
  printf '  例如：%s\n' "docs: 补充左手 CAN5 接线照片"
  printf '是否仍然继续？[y/N] '
  read -r ans
  case "$ans" in [yY]*) ;; *) die "已取消，请重新运行并写规范说明。" ;; esac
else
  ok "提交说明格式正确：$MSG"
fi

# ── 2. 提交身份 ───────────────────────────────────────
step "2/6 提交身份检查"
NAME="$(git config user.name  || true)"
MAIL="$(git config user.email || true)"
if [ -z "$NAME" ] || [ -z "$MAIL" ]; then
  err "还没有配置提交身份，Git 无法记录「是谁提交的」。"
  printf '\n请先执行下面两条命令（换成你自己的信息）：\n\n'
  printf '  git config --global user.name  "你的名字"\n'
  printf '  git config --global user.email "你的公司邮箱"\n\n'
  printf '配置完再重新运行本脚本。\n'
  exit 1
fi
ok "提交身份：$NAME <$MAIL>"

# ── 3. 安全扫描：大文件 + 密钥 ─────────────────────────
step "3/6 安全扫描"

# 3.1 大文件：所有将被 add 的文件（含未跟踪的）
MAX_MB=5
BIG_FILES=""
while IFS= read -r f; do
  [ -f "$f" ] || continue
  size=$(wc -c < "$f" 2>/dev/null || echo 0)
  if [ "$size" -gt $((MAX_MB * 1024 * 1024)) ]; then
    BIG_FILES="${BIG_FILES}$(awk -v s="$size" 'BEGIN{printf "%.1f MB", s/1048576}')  $f"$'\n'
  fi
done < <(git ls-files --others --exclude-standard -- "$MY_REL"; git diff --name-only -- "$MY_REL"; git diff --cached --name-only -- "$MY_REL")

if [ -n "$BIG_FILES" ]; then
  err "发现超过 ${MAX_MB}MB 的文件，已拦下："
  printf '%s' "$BIG_FILES" | sed 's/^/      /'
  printf '\n'
  printf '处理建议（二选一）：\n'
  printf '  ① 官方 PDF / 安装包 / 数据集 → 放公司文件服务器，README 里放链接\n'
  printf '  ② 确认确有必要版本化 → 加进 .gitignore 或改用 Git LFS（先问主管）\n\n'
  printf '如果确认要提交，请手动执行：\n'
  printf '  git add <文件> && git commit -m "..." && git push\n\n'
  exit 1
fi

# 3.2 疑似密钥
SECRET_HITS=""
while IFS= read -r f; do
  [ -f "$f" ] || continue
  case "$f" in
    *.pem|*.key|*.p12|*.pfx|*id_rsa*|*id_ed25519*|*.env) SECRET_HITS="${SECRET_HITS}${f}（文件名即敏感）"$'\n'; continue ;;
  esac
  # 内容启发式：私钥块 / GitHub token / AWS key
  if grep -lE 'BEGIN (RSA|OPENSSH|EC|PGP) PRIVATE KEY|gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}' "$f" >/dev/null 2>&1; then
    SECRET_HITS="${SECRET_HITS}${f}（内容疑似密钥/Token）"$'\n'
  fi
done < <(git ls-files --others --exclude-standard -- "$MY_REL"; git diff --name-only -- "$MY_REL"; git diff --cached --name-only -- "$MY_REL")

if [ -n "$SECRET_HITS" ]; then
  err "发现疑似密钥文件，已拦下："
  printf '%s' "$SECRET_HITS" | sed 's/^/      /'
  printf '\n'
  printf '密钥一旦推送就很难彻底清除，且会被视为安全事故。\n'
  printf '请把这些文件加进 .gitignore，并把密钥换掉。\n\n'
  exit 1
fi

ok "没有发现大文件或密钥"

# ── 4. 确认改动清单 ───────────────────────────────────
step "4/6 即将提交的改动"
git status --short -- "$MY_REL"
printf '\n'
UNTRACKED=$(git ls-files --others --exclude-standard -- "$MY_REL" | wc -l | tr -d ' ')
info "未跟踪的新文件：${UNTRACKED} 个"
printf '确认无误、继续提交？[y/N] '
read -r ans2
case "$ans2" in [yY]*) ;; *) die "已取消。未做任何改动。" ;; esac

# ── 5. 提交 ───────────────────────────────────────────
step "5/6 提交"
git add -- "$MY_REL"
if git diff --cached --quiet; then
  warn "暂存区是空的，没有可提交的内容。"
  printf '可能是：改动已经被提交过，或该目录被 .gitignore 排除了。\n'
  printf '想确认某个文件是否被忽略：git check-ignore -v <文件>\n'
  exit 0
fi
git commit -m "$MSG"
ok "已提交：$MSG"

# ── 6. 同步并推送 ─────────────────────────────────────
step "6/6 同步远程并推送"

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
info "当前分支：$BRANCH"

if ! git remote get-url origin >/dev/null 2>&1; then
  err "还没有配置远程仓库 origin。"
  printf '\n请先执行（URL 换成公司仓库地址）：\n'
  printf '  git remote add origin <URL>\n'
  printf '再重新运行本脚本。\n'
  exit 1
fi
info "远程仓库：$(git remote get-url origin)"

# 先拉，避免 non-fast-forward
if git ls-remote --exit-code origin "$BRANCH" >/dev/null 2>&1; then
  info "远程已有该分支，先拉取..."
  if ! git pull --rebase origin "$BRANCH"; then
    err "拉取时出现冲突，需要你手动解决。"
    printf '\n处理步骤：\n'
    printf '  1) 打开带冲突标记的文件，找到 <<<<<<< ======= >>>>>>>\n'
    printf '  2) 改成你想要的内容，删掉那三行标记\n'
    printf '  3) git add <文件>\n'
    printf '  4) git rebase --continue\n'
    printf '  5) 重新运行本脚本\n\n'
    printf '想放弃这次 rebase 回到原状：git rebase --abort\n'
    exit 1
  fi
  ok "拉取完成"
else
  info "远程还没有这个分支，本次为首次推送"
fi

info "推送到 origin/$BRANCH ..."
git push -u origin "$BRANCH"
ok "推送完成"

printf '\n%s\n' "${GREEN}${BOLD}全部完成。${RESET}"
printf '提示：别忘了到 GitHub 页面确认文件已出现；如果是 feature 分支，可以发起 Pull Request。\n'

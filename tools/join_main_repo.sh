#!/usr/bin/env bash
# ============================================================
# 把本地资料并入公司主仓库 elf3-humanoid（向导，可重复运行）
#
# 用法：
#   cd "D:/Desktop/机器人实习/灵巧手/elf3-revo2-hand-integration"
#   ./tools/join_main_repo.sh
#
# 为什么这么设计：
#   1. elf3-humanoid 是「多人共用」的主仓库，根目录属于公共区域，
#      你的资料必须放进一个属于你的子目录。
#   2. 仓库完整体积约 11 MB / 465 个文件（含大量 STL、npz）。
#      本脚本用「部分克隆 + 稀疏检出」，只下载目录树（约 100 KB）
#      和你自己要用的那个目录，慢网络下也能几秒完成。
#
# 它会做这些事：
#   0. 定位自己
#   1. 检查 Git 与 SSH 通道（公钥没配好会明确告诉你）
#   2. 取主仓库结构（只下目录树，不下载任何文件内容）
#   3. 打印现有结构，让你照着决定子目录名
#   4. 稀疏检出你的子目录
#   5. 把资料放进去
#   6. 提交并推送
#
# 中途失败可以放心重跑 —— 已完成的步骤会自动跳过。
# 之后日常提交用：./tools/git_sync.sh "说明"
# ============================================================

set -uo pipefail

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'
  BLUE=$'\033[34m'; BOLD=$'\033[1m'; RESET=$'\033[0m'
else
  RED=""; GREEN=""; YELLOW=""; BLUE=""; BOLD=""; RESET=""
fi
info() { printf '%s\n' "${BLUE}[信息]${RESET} $*"; }
ok()   { printf '%s\n' "${GREEN}[通过]${RESET} $*"; }
warn() { printf '%s\n' "${YELLOW}[注意]${RESET} $*"; }
err()  { printf '%s\n' "${RED}[失败]${RESET} $*" >&2; }
step() { printf '\n%s\n' "${BOLD}════ $* ════${RESET}"; }
hr()   { printf '%s\n' "──────────────────────────────────────────────────────────"; }
ask()  { printf '%s' "$1"; read -r REPLY </dev/tty || REPLY=""; }

MAIN_URL="git@github.com:Bake-Humanoid/elf3-humanoid.git"
DEFAULT_SUB="dexterous-hand"

printf '%s\n' "${BOLD}并入公司主仓库 elf3-humanoid${RESET}"

# ── 0. 定位自己 ───────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="$(dirname "$SCRIPT_DIR")"
PARENT="$(dirname "$SRC_ROOT")"
MAIN_DIR="$PARENT/elf3-humanoid"

step "0/6 定位"
info "资料所在目录：$SRC_ROOT"
info "主仓库位置：$MAIN_DIR"

# ── 1. 环境与通道检查 ─────────────────────────────────
step "1/6 检查 Git 与 SSH 通道"

command -v git >/dev/null 2>&1 || {
  err "没有找到 git 命令。请先安装 Git for Windows：https://git-scm.com/download/win"
  exit 1
}
ok "$(git --version)"

info "测试 ssh -T git@github.com ..."
SSH_OUT="$(ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15 -T git@github.com 2>&1 || true)"

if printf '%s' "$SSH_OUT" | grep -q "successfully authenticated"; then
  ok "SSH 通道正常：$(printf '%s' "$SSH_OUT" | head -1)"
elif printf '%s' "$SSH_OUT" | grep -qi "permission denied"; then
  err "能连上 GitHub，但你的公钥还没被认可 —— 这是目前唯一挡路的东西。"
  printf '\n需要做的（一次性，约 2 分钟）：\n\n'
  printf '  1) 浏览器打开  %s\n' "${BOLD}https://github.com/settings/keys${RESET}"
  printf '  2) 点 %s\n' "${BOLD}New SSH key${RESET}"
  printf '  3) Title 随便填（例如 公司笔记本），Key 里粘贴下面这一整行：\n\n'
  hr
  cat "$HOME/.ssh/id_ed25519.pub" 2>/dev/null || printf '(找不到公钥文件 ~/.ssh/id_ed25519.pub)\n'
  hr
  printf '\n  4) 点 %s，然后重新运行本脚本\n\n' "${BOLD}Add SSH key${RESET}"
  printf '如果浏览器打不开 github.com：换手机热点 / 挂公司 VPN / 请同事代加。\n'
  exit 1
else
  err "连不上 GitHub 的 SSH 通道。"
  printf '  · 先换手机热点试试\n'
  printf '  · 或让公司 IT 放行 github.com 的 22 端口\n'
  exit 1
fi

# ── 2. 取主仓库结构（不下载文件内容）──────────────────
step "2/6 取主仓库结构"

if [ -d "$MAIN_DIR/.git" ]; then
  ok "主仓库已存在：$MAIN_DIR"
  cd "$MAIN_DIR" || exit 1
else
  info "部分克隆中（只下目录树，不下载任何文件内容）..."
  if ! git clone --filter=blob:none --no-checkout --depth 1 --single-branch \
        "$MAIN_URL" "$MAIN_DIR"; then
    err "克隆失败。检查网络、仓库地址是否正确、以及账号是否有访问权限。"
    exit 1
  fi
  cd "$MAIN_DIR" || exit 1
  ok "结构已取到（.git 约 $(du -sh .git 2>/dev/null | cut -f1)）"
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
info "分支：$BRANCH"
info "远程：$(git remote get-url origin)"

# ── 3. 看结构 + 确定子目录 ────────────────────────────
step "3/6 确定你的子目录"

printf '%s\n' "${BOLD}顶层内容（这是公共区域，不要往这里直接丢文件）：${RESET}"
hr
git ls-tree HEAD --name-only | sed 's/^/  /'
hr
printf '\n'
info "仓库共有 $(git ls-tree -r HEAD --name-only | wc -l | tr -d ' ') 个文件（内容尚未下载）"
printf '想看完整路径清单：git ls-tree -r HEAD --name-only\n\n'

printf '别人怎么分目录，你就照着分。上面如果已经有同类目录，\n'
printf '建议放进它下面的子目录，而不是新开一个顶层目录。\n\n'
printf '当前建议：%s\n' "${BOLD}${DEFAULT_SUB}${RESET}（顶层功能域之一，便于评审一次性看到全部成果）"
printf '也可以写成嵌套路径，例如 %s\n\n' "middleware/${DEFAULT_SUB}"

ask "你的子目录名（直接回车用默认 ${DEFAULT_SUB}）："
SUB="${REPLY:-$DEFAULT_SUB}"
SUB="${SUB#/}"; SUB="${SUB%/}"
[ -n "$SUB" ] || { err "子目录名不能为空。"; exit 1; }

if printf '%s' "$SUB" | grep -q '[\\ :*?"<>|]'; then
  warn "目录名里有空格或特殊字符（: * ? \" < > |），在 Git 里容易出问题。"
  ask "确认继续？[y/N] "
  case "${REPLY:-}" in [yY]*) ;; *) exit 1 ;; esac
fi
info "使用子目录：$SUB"

# ── 4. 稀疏检出 ───────────────────────────────────────
step "4/6 稀疏检出你的目录"

if git sparse-checkout list 2>/dev/null | grep -qx "$SUB"; then
  ok "已经设定过：$SUB"
else
  if ! git sparse-checkout set --cone "$SUB"; then
    err "设置稀疏检出失败。"
    exit 1
  fi
  ok "已设定只检出：$SUB（同事的文件不会被下载到本地）"
fi

git checkout "$BRANCH" >/dev/null 2>&1 || true

# 安全检查：暂存区必须没有「删除」条目
DEL_COUNT="$(git status --short | grep -c '^D' || true)"
if [ "${DEL_COUNT:-0}" -gt 0 ]; then
  err "检测到 $DEL_COUNT 个「删除」条目 —— 索引状态不对，继续操作会误删同事的文件。"
  printf '请先执行：git sparse-checkout set --cone %s && git checkout %s\n' "$SUB" "$BRANCH"
  exit 1
fi
ok "索引状态正常（无删除条目，不会误伤同事文件）"

DEST="$MAIN_DIR/$SUB"
mkdir -p "$DEST"

# ── 5. 放入资料 ───────────────────────────────────────
step "5/6 放入你的资料"

printf '源：%s\n' "$SRC_ROOT"
printf '目标：%s\n\n' "$DEST"

if [ -n "$(ls -A "$DEST" 2>/dev/null)" ]; then
  warn "目标目录已存在且不为空，本次会覆盖同名文件（不删除多余文件）。"
  ask "继续？[y/N] "
  case "${REPLY:-}" in [yY]*) ;; *) info "已取消。"; exit 0 ;; esac
fi

if ! tar -C "$SRC_ROOT" --exclude='./.git' -cf - . | tar -C "$DEST" -xf - ; then
  err "拷贝失败。"
  exit 1
fi
ok "拷贝完成"

git add -- "$SUB" || { err "git add 失败。"; exit 1; }

printf '\n即将提交的内容（只包含 %s/）：\n' "$SUB"
hr
git status --short -- "$SUB" | head -40
hr
COUNT="$(git diff --cached --name-only -- "$SUB" | wc -l | tr -d ' ')"
info "共 $COUNT 个文件"

# ── 6. 提交并推送 ─────────────────────────────────────
step "6/6 提交并推送"

NAME="$(git config user.name || true)"
MAIL="$(git config user.email || true)"
if [ -z "$NAME" ] || [ -z "$MAIL" ]; then
  err "还没有配置提交身份，请先执行："
  printf '  git config --global user.name  "你的名字"\n'
  printf '  git config --global user.email "你的邮箱"\n'
  exit 1
fi
info "提交身份：$NAME <$MAIL>"

if git diff --cached --quiet; then
  warn "没有检测到新内容（可能之前已经提交过）。"
  ask "仍要继续推送吗？[y/N] "
  case "${REPLY:-}" in [yY]*) ;; *) exit 0 ;; esac
else
  printf '\n%s\n' "${BOLD}注意：只会提交 $SUB 这一个目录，同事的文件不受影响。${RESET}"
  ask "确认提交并推送？[y/N] "
  case "${REPLY:-}" in
    [yY]*) ;;
    *) info "已取消。文件已放进 $DEST，你可以之后手动提交。"; exit 0 ;;
  esac
  git commit -F - <<'EOF' || { err "提交失败。"; exit 1; }
添加灵巧手接入资料：Revo2 触觉版接线手册、单机调试与 ROS2 整机联调

- docs/01 硬件接线与上电操作手册：型号对照、安全须知、接口识别、
  供电与电源开关操作、485/CANFD/EtherCAT 接线、上电验证、验收标准、故障速查
- docs/00 环境边界与方案选型：Windows 单手调试 vs Linux ROS2 整机联调
- docs/02/03 Windows 单机调试指南、ELF3 整机 ROS2 联调指南
- docs/04/05 通信协议速查（Modbus / CAN FD 寄存器表）、五层故障排查手册
- docs/06 Git 协作与仓库规范
- src/revo2_standalone 纯标准库 Modbus 通信库与六步自检脚本
- src/elf3_ros2 ROS2 bridge 节点与上位 commander 控制端
- tools/ 分层通信诊断与本目录提交工具
- config/ 灵巧手参数台账与手势预设库
EOF
  ok "已提交"
fi

if git ls-remote --exit-code origin "$BRANCH" >/dev/null 2>&1; then
  info "先拉取同事的更新..."
  if ! git pull --rebase origin "$BRANCH"; then
    err "拉取时出现冲突。处理步骤："
    printf '  1) 打开带冲突标记的文件，找 <<<<<<< ======= >>>>>>>\n'
    printf '  2) 改成你想要的内容，删掉那三行标记\n'
    printf '  3) git add <文件>\n'
    printf '  4) git rebase --continue\n'
    printf '  5) 重新运行本脚本\n'
    printf '想放弃：git rebase --abort\n'
    exit 1
  fi
  ok "拉取完成"
fi

info "推送到 origin/$BRANCH ..."
if git push -u origin "$BRANCH"; then
  printf '\n%s\n' "${GREEN}${BOLD}上传成功！${RESET}"
  printf '仓库地址：https://github.com/Bake-Humanoid/elf3-humanoid\n'
  printf '你的资料在：%s/\n\n' "$SUB"
  printf '之后日常提交：\n'
  printf '  cd "%s"\n' "$MAIN_DIR"
  printf '  ./%s/tools/git_sync.sh "说明"\n' "$SUB"
else
  err "推送失败。常见原因："
  printf '  · 没有该仓库的写权限  → 找主管把你的账号加进 Bake-Humanoid 组织\n'
  printf '  · 同事刚推了新提交    → 再跑一次本脚本（会自动先拉后推）\n'
  exit 1
fi

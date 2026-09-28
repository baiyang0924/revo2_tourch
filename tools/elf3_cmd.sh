#!/usr/bin/env bash
# ============================================================================
#  elf3_cmd.sh —— 按「动作名」给 ELF3 发一条 motion_commands
#
#  为什么需要它：MotionCommands 消息有 10 个 btn_ + 10 个 axis_ 共 20 个字段，
#  手抄那串 JSON 极易漏一个字段而**完全不报错**（消息类型是对的，只是动作不对）。
#  这里把「动作 → 字段」的映射固化下来，避免手抄。
#
#  用法：
#     ./elf3_cmd.sh --list                 列出所有动作名
#     ./elf3_cmd.sh stand --dry-run        只打印将要发送的消息，不发
#     ./elf3_cmd.sh stand                  真正发送（默认 --once）
#     ./elf3_cmd.sh cheer --topic /motion_commands
#
#  注意：
#     1. 必须在已 source 好 ROS 2 环境的 shell 里跑（真机上还要在 root shell 里）。
#     2. 发之前请确认机器人周围无人、急停在手边。
#     3. 真机上顺序通常是：先 stand（初始姿态），再发别的动作。
#
#  作者：师兄 / 小陶  日期：2026-09-20
# ============================================================================
set -u

TOPIC="/motion_commands"
MSGTYPE="communication/msg/MotionCommands"
DRY_RUN=0
ACTION=""

# ── 动作表：名字|btn 序号|值|说明 ─────────────────────────────────────────────
ACTIONS="
zero|4|1|零位模式（各关节回零位）
stand|3|1|初始姿态（后续动作的前提）
walk|1|1|normal 走路
cheer|10|7|cheer 欢呼（对应仿真键盘 7）
fistbump|10|9|单臂碰拳
akimbo|10|10|双臂叉腰
hug|10|11|双臂拥抱
"

usage() {
  cat <<'EOF'
elf3_cmd.sh —— 给 ELF3 发一条 motion_commands

用法:
  ./elf3_cmd.sh <动作名> [选项]

选项:
  --dry-run, -n     只打印将发送的消息，不真的发
  --topic <话题>    覆盖默认话题 (/motion_commands)
  --list, -l        列出所有动作名
  -h, --help        显示本帮助

例子:
  ./elf3_cmd.sh --list
  ./elf3_cmd.sh stand --dry-run
  ./elf3_cmd.sh stand
EOF
}

list_actions() {
  echo
  echo "可用动作（名字 -> 字段 = 值）："
  echo
  printf '%s\n' "$ACTIONS" | while IFS='|' read -r name idx val desc; do
    [ -z "${name:-}" ] && continue
    printf '  %-12s btn_%-3s = %-3s  %s\n' "$name" "$idx" "$val" "$desc"
  done
  echo
  echo "提示：先跑 stand（初始姿态），再发其他动作。"
  echo
}

# ── 参数解析 ────────────────────────────────────────────────────────────────
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help)  usage; exit 0 ;;
    -l|--list)  list_actions; exit 0 ;;
    -n|--dry-run) DRY_RUN=1; shift ;;
    --topic)    TOPIC="${2:-}"; shift 2 ;;
    --topic=*)  TOPIC="${1#*=}"; shift ;;
    -*)         echo "未知选项：$1" >&2; usage; exit 2 ;;
    *)          ACTION="$1"; shift ;;
  esac
done

if [ -z "$ACTION" ]; then
  echo "错误：没有指定动作名。用 --list 看有哪些。" >&2
  echo >&2
  usage >&2
  exit 2
fi

# ── 动作名 -> btn 序号/值 ───────────────────────────────────────────────────
BTN_IDX=""
BTN_VAL=""
BTN_DESC=""
while IFS='|' read -r name idx val desc; do
  [ -z "${name:-}" ] && continue
  if [ "$name" = "$ACTION" ]; then
    BTN_IDX="$idx"; BTN_VAL="$val"; BTN_DESC="$desc"
    break
  fi
done <<EOF
$ACTIONS
EOF

if [ -z "$BTN_IDX" ]; then
  echo "错误：不认识的动作名 '$ACTION'。用 --list 看有哪些。" >&2
  exit 2
fi

# ── 组装消息（除目标 btn 外全部为 0） ───────────────────────────────────────
build_msg() {
  local target="$1" value="$2"
  local btn_part="" i v sep=""
  for i in 1 2 3 4 5 6 7 8 9 10; do
    if [ "$i" = "$target" ]; then v="$value"; else v=0; fi
    btn_part="${btn_part}${sep}btn_${i}: ${v}"
    sep=", "
  done
  printf "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, %s, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" "$btn_part"
}

MSG="$(build_msg "$BTN_IDX" "$BTN_VAL")"

echo
echo "动作   : $ACTION （$BTN_DESC）"
echo "字段   : btn_${BTN_IDX} = ${BTN_VAL}"
echo "话题   : $TOPIC"
echo "消息类型: $MSGTYPE"
echo

if [ "$DRY_RUN" = "1" ]; then
  echo "--- dry-run：下面这条不会真的发出去 ---"
  echo
  echo "ros2 topic pub $TOPIC $MSGTYPE \\"
  echo "\"$MSG\" --once"
  echo
  echo "确认无误后去掉 --dry-run 再跑一次。"
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2 命令。" >&2
  echo "  · 真机上要先： source /opt/ros/humble/setup.bash" >&2
  echo "                     source /opt/bxi/bxi_ros2_pkg/setup.bash" >&2
  echo "                     source <workspace>/install/setup.bash" >&2
  echo "  · 只想看会发什么：加 --dry-run" >&2
  exit 1
fi

echo ">>> 正在发送……"
if ros2 topic pub "$TOPIC" "$MSGTYPE" "$MSG" --once; then
  echo
  echo "已发送。没反应的话按这个顺序查："
  echo "  1) 是不是还没发过 stand（初始姿态）"
  echo "  2) ROS_DOMAIN_ID 是否和机器人硬件节点一致（第一大坑）"
  echo "  3) 是不是在普通用户 shell 里跑的（真机侧要 root）"
  exit 0
else
  echo
  echo "发送失败。先确认 $TOPIC 这个话题存在：" >&2
  echo "  ros2 topic list | grep motion" >&2
  exit 1
fi

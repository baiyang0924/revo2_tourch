#!/usr/bin/env bash
# run_arm_demo.sh —— 一键复现：ROS 2 仿真里右臂被覆盖通道驱动，双画面录成视频
#
# 前置：仿真已在跑（见 start_sim.sh），且 ROS_DOMAIN_ID 与仿真一致。
#
# 产出：
#   shots/final2.mp4   双画面视频（左＝全身，右＝手臂特写）
#   shots/final2.gif   轻量版
#   shots/final2.log   观察者日志（含右腕位移客观量）
#   shots/arm.log      激励源日志（指令 vs 实测 + 限位检查）
#
# 为什么要错开 3 s：让观察者先进入录制、录到一段"动作开始前"的基线，
# 视频里才有静止对照。激励源自己的轨迹里也再留 2 s 待机。

# ⚠️ 顺序不能改：ROS 2 的 setup.bash 第 8 行会读 AMENT_TRACE_SETUP_FILES，
#    在 `set -u` 下非交互 shell 会因"未绑定的变量"**静默退出**（连一行输出都没有）。
#    所以 source 之前不能开 -u。
set +u

MODEL=${MODEL:-$HOME/elf3_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml}
OUT=${OUT:-shots}
DOMAIN=${DOMAIN:-42}
SECONDS_TOTAL=${SECONDS_TOTAL:-21}

source "$HOME/elf3_env.sh"
export ROS_DOMAIN_ID="$DOMAIN"
export MUJOCO_GL=egl
cd "$(dirname "$0")"

set -u

echo "=== 环境 ==="
echo "  ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "  MUJOCO_GL     = $MUJOCO_GL"
echo "  MODEL         = $MODEL"
[ -f "$MODEL" ] || { echo "[错误] 模型不存在"; exit 1; }
mkdir -p "$OUT"

rm -f "$OUT/final2.mp4" "$OUT/final2.gif" "$OUT/final2.log" "$OUT/arm.log"

PIN_ARG=""
[ "${PIN_BASE:-0}" = "1" ] && PIN_ARG="--pin-base"
HOLD_ARG=""
[ "${FREEZE_LOWER:-0}" = "1" ] && PIN_ARG="$PIN_ARG --freeze-lower"
[ "${HOLD_LEGS:-0}" = "1" ] && HOLD_ARG="--hold-legs"

echo
echo "=== [1/2] 起观察者（后台），录 ${SECONDS_TOTAL}s，幅度 ×${SCALE:-1.0} ==="
python3 ros2_mujoco_spectator.py \
  --model "$MODEL" \
  --out "$OUT/final2.mp4" \
  --gif "$OUT/final2.gif" --gif-width 560 --gif-stride 3 \
  --seconds "$SECONDS_TOTAL" --fps 30 \
  --cam-mode follow --azimuth 312 --distance 3.0 --elevation -8 \
  --follow-z-offset -0.35 \
  --split --cam2-distance 1.75 --cam2-lookat-offset "0,-0.14,-0.10" \
  --skeleton $PIN_ARG \
  --label "FULL BODY" --label2 "ARM CLOSE-UP" \
  --width 640 --height 720 \
  > "$OUT/final2.log" 2>&1 &
SPEC=$!
echo "  PID $SPEC  ${PIN_ARG:-(基座不钉)}"

sleep 3

echo
echo "=== [2/2] 起激励源 ==="
python3 arm_wave_demo.py --rate 20 --scale "${SCALE:-1.0}" $HOLD_ARG 2>&1 | tee "$OUT/arm.log"

wait $SPEC

echo
echo "===== 观察者日志 ====="
cat "$OUT/final2.log"
echo
echo "===== 产出 ====="
ls -la "$OUT/final2.mp4" "$OUT/final2.gif" 2>/dev/null

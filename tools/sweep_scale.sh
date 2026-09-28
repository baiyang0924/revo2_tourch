#!/usr/bin/env bash
# sweep_scale.sh —— 幅度扫描：覆盖右臂的动作幅度 vs 机体被平衡控制器带走多少
#
# 背景：大幅覆盖右臂时，官方平衡控制器会迈步把机体带走（实测 4 m 量级）。
#       这既让镜头跑偏，也说明"手臂动作会扰动平衡"。
#       本脚本扫几档幅度，找出「手臂明显在动、机体基本不走」的上限。
#
# 用法： ./sweep_scale.sh            # 默认 0 0.35 0.55
#        SCALES="0 0.5" ./sweep_scale.sh

set +u
source "$HOME/elf3_env.sh"
export ROS_DOMAIN_ID=${DOMAIN:-42}
export MUJOCO_GL=egl
cd "$(dirname "$0")"
set -u

MODEL=${MODEL:-$HOME/elf3_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml}
SCALES=${SCALES:-"0 0.35 0.55"}
SECS=${SECS:-20}
OUT=${OUT:-shots}
mkdir -p "$OUT"

echo "=============================================="
echo "  幅度扫描：scales = $SCALES，每档 ${SECS}s"
echo "  scale=0 表示**不起激励源**（空跑对照）"
echo "=============================================="

for S in $SCALES; do
  echo
  echo "############ scale = $S ############"
  LOG="$OUT/sweep_$S.log"
  if [ "$S" = "0" ] || [ "$S" = "0.0" ]; then
    # 空跑对照：只录，不发任何指令
    timeout 120 python3 ros2_mujoco_spectator.py \
      --model "$MODEL" \
      --out "$OUT/sweep_$S.mp4" --seconds "$SECS" --fps 10 \
      --cam-mode follow --azimuth 312 --distance 3.0 --elevation -8 \
      --follow-z-offset -0.35 --skeleton --width 480 --height 540 \
      > "$LOG" 2>&1
  else
    python3 ros2_mujoco_spectator.py \
      --model "$MODEL" \
      --out "$OUT/sweep_$S.mp4" --seconds "$SECS" --fps 10 \
      --cam-mode follow --azimuth 312 --distance 3.0 --elevation -8 \
      --follow-z-offset -0.35 --skeleton --width 480 --height 540 \
      > "$LOG" 2>&1 &
    SPEC=$!
    sleep 3
    python3 arm_wave_demo.py --rate 20 --scale "$S" > "$OUT/sweep_arm_$S.log" 2>&1
    wait $SPEC
  fi

  echo "--- ① 手臂动作 ---"
  grep -A4 "① 手臂动作" "$LOG" | tail -4
  echo "--- ② 机体走动 ---"
  grep -A4 "② 机体走动" "$LOG" | tail -4
  echo "--- 覆盖跟随残差（激励源侧）---"
  if [ -f "$OUT/sweep_arm_$S.log" ]; then
    grep -E "残差" "$OUT/sweep_arm_$S.log" | tail -3
  fi
done

echo
echo "=============================================="
echo "  汇总（② 越小越好）"
echo "=============================================="
for S in $SCALES; do
  LOG="$OUT/sweep_$S.log"
  B=$(grep -A2 "② 机体走动" "$LOG" | grep "最大" | sed "s/.*最大 *//" | awk "{print \$1}")
  W=$(grep -A2 "① 手臂动作" "$LOG" | grep "最大" | sed "s/.*最大 *//" | awk "{print \$1}")
  printf "  scale %-5s  手臂 %-12s  机体走动 %s\n" "$S" "${W:-?}mm" "${B:-?}m"
done

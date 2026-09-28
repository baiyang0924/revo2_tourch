#!/bin/bash
# ============================================================
#  灵巧手控制台  ——  在 172.16.10.133 上运行
# ------------------------------------------------------------
#  用法：
#     ~/hand.sh show            动作A(波浪握拳×3) + 动作B(连续OK)
#     ~/hand.sh show --reps 1   只跑 1 组波浪
#     ~/hand.sh wave            只跑波浪握拳/张开
#     ~/hand.sh fist            单次握拳→张开
#     ~/hand.sh info            查看手的信息（不动手指）
#     ~/hand.sh ok              只跑连续 OK 手势
#
#  第一次运行会提示输入机器人 sudo 密码（凭据向主管索取），
#  之后 15 分钟内不用再输。
# ============================================================
set -u

ROBOT=bxi@172.16.10.47
SSHOPT="-o StrictHostKeyChecking=no -o BatchMode=yes"

run() {
    local script="$1"
    shift
    ssh -t $SSHOPT "$ROBOT" "sudo bash /home/bxi/run.sh $script $*"
}

case "${1:-help}" in
    show)
        run /home/bxi/hand_show.py "${@:2}"
        ;;
    wave)
        run /home/bxi/hand_wave.py "${@:2}"
        ;;
    fist)
        run /home/bxi/hand_fist.py "${@:2}"
        ;;
    info|probe)
        run /home/bxi/hand_probe.py "${@:2}"
        ;;
    ok)
        run /home/bxi/hand_show.py --skip-wave "${@:2}"
        ;;
    help|-h|--help|"")
        cat <<'EOF'
灵巧手控制台 —— 用法

  ~/hand.sh show              动作A(波浪握拳×3组) + 动作B(连续OK手势)
  ~/hand.sh show --reps 1     只跑 1 组波浪
  ~/hand.sh show --verify     跑完回读实际位置（核对用）
  ~/hand.sh wave              只跑波浪握拳/张开
  ~/hand.sh wave --seg 0.6    波浪慢一点（单指时长，秒）
  ~/hand.sh fist              单次握拳 → 保持 → 张开
  ~/hand.sh info              查看手的型号/固件/当前位置（不发动作）
  ~/hand.sh help              显示本说明

第一次会提示输入机器人 sudo 密码（向主管索取）
EOF
        ;;
    *)
        echo "未知命令：$1"
        echo "敲 ~/hand.sh help 看用法"
        exit 2
        ;;
esac

#!/bin/bash
# 灵巧手通用运行器（必须在 sudo 下执行 —— 硬件节点在 root，环境变量要对齐）
# 用法: sudo bash /home/bxi/run.sh <脚本.py> [参数...]
set +u
source /home/bxi/hand_env.sh
SCRIPT="$1"
shift
if [ ! -f "$SCRIPT" ]; then
    echo "脚本不存在: $SCRIPT"
    exit 2
fi
exec python3 "$SCRIPT" "$@"

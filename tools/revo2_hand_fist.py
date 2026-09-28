#!/usr/bin/env python3
"""Revo2 灵巧手 · 最小握拳动作

只做一件事：连接指定侧的手 → 握拳 → 保持 → 张开。
不跑官方那套「全部 demo」，避免发出多余动作。

用法：
    python3 hand_fist.py --hand right --read-only   # 只读：确认手在线、看当前状态
    python3 hand_fist.py --hand right               # 握拳 → 保持 3 s → 张开
    python3 hand_fist.py --hand right --hold 5      # 握拳保持 5 s

总线与 ID（与 docs/01 一致）：左 CAN5 / 126，右 CAN6 / 127，master_id=1。
"""
from __future__ import annotations

import argparse
import asyncio
import sys

SRC = "/home/bxi/bxi_ws/bxi_revo2_example/src"
if SRC not in sys.path:
    sys.path.insert(0, SRC)

import rclpy  # noqa: E402
from bxi_can_node import (  # noqa: E402
    cleanup_bxipci_device,
    init_bxipci_device,
    libstark,
    stop_bxipci_runtime,
)

HANDS = {"left": (5, 126), "right": (6, 127)}

# 手指位置量程 0~1000（0 = 完全张开，1000 = 完全弯曲）。
# 前两位 = 拇指（屈曲 / 对掌），后四位 = 食指 / 中指 / 无名指 / 小指。
# 这组握拳值取自官方示例 demo_basic_position()。
FIST = [500, 500, 1000, 1000, 1000, 1000]
OPEN = [0, 0, 0, 0, 0, 0]


async def run(args) -> int:
    bus, slave = HANDS[args.hand]
    libstark.init_logging()
    ctx = None
    try:
        print(f"[连接] {args.hand} hand  bus={bus}  slave_id={slave}  CANFD  master_id=1")
        ctx = await init_bxipci_device(
            bus, slave, master_id=1, is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )
        await asyncio.sleep(0.5)

        info = await ctx.handle.get_device_info(ctx.slave_id)
        print(f"[设备] 硬件={info.hardware_type}  固件={info.firmware_version}  "
              f"序列号={info.serial_number}")

        st = await ctx.handle.get_motor_status(ctx.slave_id)
        print(f"[当前] 位置 = {list(st.positions)}")
        print(f"[当前] 电流 = {list(st.currents)}")
        print(f"[当前] 状态 = {list(st.states)}")

        if args.read_only:
            print("[只读] 不发送任何指令，退出。")
            return 0

        print(f"[动作] 握拳 → {FIST}")
        await ctx.handle.set_finger_positions(ctx.slave_id, FIST)
        await asyncio.sleep(args.hold)

        st_fist = await ctx.handle.get_motor_status(ctx.slave_id)
        print(f"[握拳] 实际位置 = {list(st_fist.positions)}")
        print(f"[握拳] 实际电流 = {list(st_fist.currents)}")

        print(f"[动作] 张开 → {OPEN}")
        await ctx.handle.set_finger_positions(ctx.slave_id, OPEN)
        await asyncio.sleep(1.0)

        st2 = await ctx.handle.get_motor_status(ctx.slave_id)
        print(f"[结果] 张开位置 = {list(st2.positions)}")
        print("[完成]")
        return 0
    finally:
        if ctx is not None:
            await cleanup_bxipci_device(ctx)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", choices=sorted(HANDS), default="right")
    ap.add_argument("--read-only", action="store_true", help="只读取状态，不发指令")
    ap.add_argument("--hold", type=float, default=3.0, help="握拳保持秒数")
    args = ap.parse_args()

    rclpy.init(args=None)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0
    finally:
        stop_bxipci_runtime()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())

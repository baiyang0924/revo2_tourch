#!/usr/bin/env python3
"""Revo2 灵巧手 · 手势秀（观赏性动作组合）

包含四个段子，衔接紧凑：
  1) 轮指波浪  —— 弯曲像水波一样从食指流到小指，再流回来（往返多轮）
  2) 数字 1-5  —— 依次比出 1、2、3、4、5（中国式手势）
  4) 点赞      —— 拇指伸出、四指握紧

位置量程 0~1000：0 = 完全伸直，1000 = 完全弯曲。
数组顺序 [拇指屈曲, 拇指对掌, 食指, 中指, 无名指, 小指]。
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

OPEN = [0, 0, 0, 0, 0, 0]
FIST = [1000, 1000, 1000, 1000, 1000, 1000]

# 数字手势（中国式：1 只伸食指 … 4 除拇指外四指伸，5 全伸）
NUM = {
    1: [1000, 1000,    0, 1000, 1000, 1000],
    2: [1000, 1000,    0,    0, 1000, 1000],
    3: [1000, 1000,    0,    0,    0, 1000],
    4: [1000, 1000,    0,    0,    0,    0],
    5: [   0,    0,    0,    0,    0,    0],
}

ROCK = [1000, 1000, 1000, 1000, 1000, 1000]      # 石头
SCISSOR = [1000, 1000, 0, 0, 1000, 1000]         # 剪刀（食指+中指）
PAPER = [0, 0, 0, 0, 0, 0]                       # 布
THUMBSUP = [0, 0, 1000, 1000, 1000, 1000]        # 点赞

# 轮指：食指 → 中指 → 无名指 → 小指（拇指保持伸直不参与）
RIPPLE_FINGERS = [(2, "食指"), (3, "中指"), (4, "无名指"), (5, "小指")]


async def ripple(h, sid, args, forward: bool) -> None:
    """一轮轮指波浪：弯曲像波一样依次流过四指。"""
    seq = RIPPLE_FINGERS if forward else list(reversed(RIPPLE_FINGERS))
    period, delay, rate = args.ripple_period, args.ripple_delay, args.rate
    dur = period + delay * (len(seq) - 1)
    dt = 1.0 / rate
    t = 0.0
    while t <= dur + 1e-9:
        pos = [0] * 6
        for k, (idx, _n) in enumerate(seq):
            ph = ((t - k * delay) / period) % 1.0
            v = 1.0 - abs(2.0 * ph - 1.0)          # 三角波 0→1→0
            pos[idx] = int(round(max(0.0, min(1.0, v)) * 1000))
        await h.set_finger_positions(sid, pos)
        await asyncio.sleep(dt)
        t += dt
    await h.set_finger_positions(sid, [0] * 6)


async def pose(h, sid, target, hold: float, label: str) -> None:
    await h.set_finger_positions(sid, target)
    print(f"      · {label}", flush=True)
    await asyncio.sleep(hold)


async def run(args) -> int:
    libstark.init_logging()
    ctx = None
    try:
        print("[连接] right hand  bus=6  slave_id=127  CANFD")
        ctx = await init_bxipci_device(
            6, 127, master_id=1, is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )
        h, sid = ctx.handle, ctx.slave_id
        await asyncio.sleep(0.5)
        info = await h.get_device_info(sid)
        print(f"[设备] {info.hardware_type}  固件={info.firmware_version}")

        if args.read_only:
            st = await h.get_motor_status(sid)
            print(f"[只读] {list(st.positions)}")
            return 0

        await h.set_finger_positions(sid, OPEN)
        await asyncio.sleep(0.8)

        # ---- 1) 轮指波浪 ----
        print("\n========== 1. 轮指波浪 ==========")
        for r in range(1, args.ripple_rounds + 1):
            print(f"  第 {r}/{args.ripple_rounds} 轮  正向（食指→小指）")
            await ripple(h, sid, args, forward=True)
            await asyncio.sleep(args.gap)
            print(f"  第 {r}/{args.ripple_rounds} 轮  反向（小指→食指）")
            await ripple(h, sid, args, forward=False)
            await asyncio.sleep(args.gap)

        # ---- 2) 数字 1-5 ----
        print("\n========== 2. 数字 1 - 5 ==========")
        for n in range(1, 6):
            await pose(h, sid, NUM[n], args.hold, f"{n}  ← {NUM[n]}")
            await asyncio.sleep(args.gap)

        # ---- 4) 点赞 ----
        print("\n========== 4. 点赞 ==========")
        await pose(h, sid, THUMBSUP, args.hold * 2, "拇指伸出、四指握紧")
        await asyncio.sleep(args.gap)

        await h.set_finger_positions(sid, OPEN)
        await asyncio.sleep(0.8)
        st = await h.get_motor_status(sid)
        print(f"\n[结束] 位置 = {list(st.positions)}")
        return 0
    finally:
        if ctx is not None:
            await cleanup_bxipci_device(ctx)


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手手势秀：轮指波浪 + 数字 + 点赞")
    ap.add_argument("--hold", type=float, default=0.75, help="每个静态手势保持时长（秒）")
    ap.add_argument("--gap", type=float, default=0.10, help="手势之间的最短间隔（秒）")
    ap.add_argument("--rate", type=float, default=30.0, help="轮指控制频率 Hz")
    ap.add_argument("--ripple-rounds", type=int, default=2, help="轮指往返轮数")
    ap.add_argument("--ripple-period", type=float, default=0.50, help="轮指单指起伏周期（秒）")
    ap.add_argument("--ripple-delay", type=float, default=0.12, help="轮指相邻手指相位延迟（秒）")
    ap.add_argument("--rps-rounds", type=int, default=2, help="石头剪刀布轮数")
    ap.add_argument("--read-only", action="store_true")
    args = ap.parse_args()

    rclpy.init(args=None)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[中断]")
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

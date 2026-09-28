#!/usr/bin/env python3
"""实测真机每个电机的行程上限（下发 1000 实际能到多少）

背景：OK 手势下发拇指 821/801 只到 687/703，下发 1000 只到 761，
说明真机拇指行程小于仿真模型，标定值需要按真机可达范围重算。
"""
from __future__ import annotations

import asyncio
import sys
import time

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

NAMES = ["Thumb屈曲", "ThumbAux对掌", "Index食指", "Middle中指", "Ring无名指", "Pinky小指"]


async def wait_stable(h, sid, timeout: float = 4.0, tol: int = 3, poll: float = 0.1):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        st = await h.get_motor_status(sid)
        cur = list(st.positions)
        if last is not None and max(abs(a - b) for a, b in zip(cur, last)) <= tol:
            return cur, time.time() - t0
        last = cur
        await asyncio.sleep(poll)
    return last, time.time() - t0


async def main() -> int:
    ctx = await init_bxipci_device(
        6, 127, master_id=1, is_canfd=True,
        hw_type=libstark.StarkHardwareType.Revo2Basic,
    )
    h, sid = ctx.handle, ctx.slave_id
    try:
        await asyncio.sleep(0.5)
        print("=== 逐关节满行程测试：单独下发 1000，看实际能到多少 ===")
        print(f"{'关节':<16} {'下发':>6} {'实际':>6} {'达成率':>8}  用时")
        upper = {}
        for i, nm in enumerate(NAMES):
            pos = [0] * 6
            pos[i] = 1000
            await h.set_finger_positions(sid, [0] * 6)
            await wait_stable(h, sid, timeout=2.0)
            await h.set_finger_positions(sid, pos)
            cur, dt = await wait_stable(h, sid)
            got = cur[i]
            upper[i] = got
            print(f"{nm:<16} {1000:>6} {got:>6} {got/10.0:>7.1f}%  {dt:.2f}s")
        await h.set_finger_positions(sid, [0] * 6)
        await wait_stable(h, sid)

        print("\n=== 汇总：真机可达上限（下发 0~1000 的实际范围）===")
        for i, nm in enumerate(NAMES):
            print(f"  {nm:<16} 上限 ≈ {upper[i]}")

        print("\n=== 拇指对掌单独慢速到 1000（排除多关节干扰）===")
        await h.set_finger_positions(sid, [0] * 6)
        await wait_stable(h, sid)
        await h.set_finger_positions(sid, [0, 1000, 0, 0, 0, 0])
        cur, dt = await wait_stable(h, sid, timeout=6.0)
        print(f"  下发 [0,1000,0,0,0,0] → 实际 {list(cur)}  ({dt:.2f}s)")

        print("\n=== 拇指屈曲单独到 1000 ===")
        await h.set_finger_positions(sid, [0] * 6)
        await wait_stable(h, sid)
        await h.set_finger_positions(sid, [1000, 0, 0, 0, 0, 0])
        cur, dt = await wait_stable(h, sid, timeout=6.0)
        print(f"  下发 [1000,0,0,0,0,0] → 实际 {list(cur)}  ({dt:.2f}s)")

        await h.set_finger_positions(sid, [0] * 6)
        await wait_stable(h, sid)
    finally:
        await cleanup_bxipci_device(ctx)
    return 0


if __name__ == "__main__":
    rclpy.init(args=None)
    try:
        raise SystemExit(asyncio.run(main()))
    finally:
        stop_bxipci_runtime()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass

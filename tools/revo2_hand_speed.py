#!/usr/bin/env python3
"""实测：以最大允许速度走完全行程需要多久 —— 用于确定「最短允许间隔」。

硬件实测到的 max_speed：Thumb 145 / ThumbAux 160 / 四指 130。
本脚本用 set_finger_positions_and_speeds 以这些速度下发全行程，
轮询到位置稳定，测出实际耗时。
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

# 顺序 = [Thumb屈曲, ThumbAux对掌, Index, Middle, Ring, Pinky]
MAX_SPD = [145, 160, 130, 130, 130, 130]
OPEN = [0] * 6
FIST = [1000] * 6


async def wait_stable(h, sid, timeout: float = 12.0, tol: int = 5,
                      poll: float = 0.05):
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
        await h.set_finger_positions(sid, OPEN)
        p, _ = await wait_stable(h, sid)
        print(f"[起点] {p}")

        print("\n=== 以最大速度下发全握 ===")
        t0 = time.time()
        await h.set_finger_positions_and_speeds(sid, FIST, MAX_SPD)
        p1, dt1 = await wait_stable(h, sid)
        print(f"  指令下发→稳定: {time.time()-t0:.2f}s（稳定判定用时 {dt1:.2f}s）")
        print(f"  到位 {p1}")
        print(f"  => 单指全行程用时约 {dt1:.2f}s（速度档 {MAX_SPD}）")

        await asyncio.sleep(0.3)
        print("\n=== 以最大速度下发全张 ===")
        t0 = time.time()
        await h.set_finger_positions_and_speeds(sid, OPEN, MAX_SPD)
        p2, dt2 = await wait_stable(h, sid)
        print(f"  指令下发→稳定: {time.time()-t0:.2f}s（稳定判定用时 {dt2:.2f}s）")
        print(f"  到位 {p2}")

        print("\n=== 对照：不指定速度（默认速度）全握 ===")
        await h.set_finger_positions(sid, OPEN)
        await wait_stable(h, sid)
        t0 = time.time()
        await h.set_finger_positions(sid, FIST)
        p3, dt3 = await wait_stable(h, sid)
        print(f"  用时 {dt3:.2f}s  到位 {p3}")
        await h.set_finger_positions(sid, OPEN)
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

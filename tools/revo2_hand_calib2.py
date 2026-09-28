#!/usr/bin/env python3
"""Revo2 标定探测 v2 —— 等位置稳定后再采样

v1 的问题：固定等 1.4s 就读，手势还在执行中（Fist 读出四指=0，不合理）。
v2 改为「连续两次读数一致」才算稳定，并额外采集触觉数据用于判断接触。
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


async def wait_stable(h, sid, timeout: float = 5.0, tol: int = 4,
                      poll: float = 0.25):
    """等位置稳定：连续两次读数最大差 <= tol 即认为到位。返回 (位置, 用时)。"""
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

        print("=== 内置手势（等稳定后采样）===")
        print(f"{'gesture':<26} {'稳定耗时':>8}  位置")
        gestures = ("DefaultGestureOpen", "DefaultGestureFist",
                    "DefaultGesturePoint", "DefaultGesturePinchTwo",
                    "DefaultGesturePinchThree", "DefaultGesturePinchSide")
        for gname in gestures:
            g = getattr(libstark.ActionSequenceId, gname)
            await h.run_action_sequence(sid, g)
            pos, dt = await wait_stable(h, sid)
            print(f"{gname:<26} {dt:>7.2f}s  {pos}")

        print("\n回张开位...")
        await h.run_action_sequence(sid, libstark.ActionSequenceId.DefaultGestureOpen)
        await wait_stable(h, sid)

        print("\n=== 触觉相关接口 ===")
        for fn in ("get_touch_sensor_status", "get_touch_sensor_enabled",
                   "get_force3d_touch_summary", "get_modulus_touch_summary",
                   "get_force_level"):
            f = getattr(h, fn, None)
            if f is None:
                print(f"  {fn}: N/A")
                continue
            try:
                print(f"  {fn}: {await f(sid)}")
            except Exception as e:  # noqa: BLE001
                print(f"  {fn}: ERR {type(e).__name__}: {e}")

        print("\n=== 手动构造：拇指+食指 OK（以 PinchTwo 为基准复现）===")
        await h.set_finger_positions(sid, [446, 750, 317, 0, 0, 0])
        pos, dt = await wait_stable(h, sid)
        print(f"  下发 [446,750,317,0,0,0] → 稳定 {dt:.2f}s 读到 {pos}")

        pos0, _ = await wait_stable(h, sid)
        print(f"\n[结束] 当前 {pos0}")
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

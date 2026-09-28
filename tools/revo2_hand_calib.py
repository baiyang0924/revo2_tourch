#!/usr/bin/env python3
"""Revo2 标定探测（需要动手指）

目的：
  1) 读出手指硬件参数：max_speed / max_position / max_current —— 用于「最大允许速度」
  2) 确认下发数组下标 ↔ FingerId 的映射（单指微动 + 回读）
  3) 采集内置手势（Pinch 系列 / Fist / Open / Point）的真实位置值
     —— 作为「OK 手势」的标定参考
  4) 查询 turbo 模式配置（最大速度相关）

所有动作时间都留了 settle，读数才是稳定的。
"""
from __future__ import annotations

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

FINGERS = ("Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky")


async def main() -> int:
    ctx = await init_bxipci_device(
        6, 127, master_id=1, is_canfd=True,
        hw_type=libstark.StarkHardwareType.Revo2Basic,
    )
    h, sid = ctx.handle, ctx.slave_id
    try:
        await asyncio.sleep(0.5)
        st = await h.get_motor_status(sid)
        print(f"[初始] {list(st.positions)}")

        print("\n=== 1) 手指硬件参数 ===")
        print(f"{'finger':<10} {'max_speed':>10} {'max_pos':>9} {'min_pos':>9} "
              f"{'max_cur':>9} {'prot_cur':>9}")
        for n in FINGERS:
            fid = getattr(libstark.FingerId, n)
            vals = []
            for fn in ("get_finger_max_speed", "get_finger_max_position",
                       "get_finger_min_position", "get_finger_max_current",
                       "get_finger_protected_current"):
                try:
                    vals.append(await getattr(h, fn)(sid, fid))
                except Exception as e:  # noqa: BLE001
                    vals.append(f"ERR:{type(e).__name__}")
            print(f"{n:<10} {vals[0]:>10} {vals[1]:>9} {vals[2]:>9} "
                  f"{vals[3]:>9} {vals[4]:>9}")

        print("\n=== 2) 数组下标 ↔ FingerId 映射（单指微动 600）===")
        for n in FINGERS:
            fid = getattr(libstark.FingerId, n)
            await h.set_finger_positions(sid, [0] * 6)
            await asyncio.sleep(0.55)
            await h.set_finger_position(sid, fid, 600)
            await asyncio.sleep(0.55)
            r = await h.get_motor_status(sid)
            changed = [i for i, v in enumerate(r.positions) if v > 200]
            print(f"  {n:<10} → 变化的下标 {changed}   位置={list(r.positions)}")
        await h.set_finger_positions(sid, [0] * 6)
        await asyncio.sleep(0.6)

        print("\n=== 3) 内置手势的真实位置值（OK 手势标定参考）===")
        gestures = ("DefaultGestureOpen", "DefaultGestureFist",
                    "DefaultGesturePinchTwo", "DefaultGesturePinchThree",
                    "DefaultGesturePinchSide", "DefaultGesturePoint")
        for gname in gestures:
            g = getattr(libstark.ActionSequenceId, gname)
            await h.run_action_sequence(sid, g)
            await asyncio.sleep(1.4)
            r = await h.get_motor_status(sid)
            print(f"  {gname:<26} {list(r.positions)}")
        await h.run_action_sequence(sid, libstark.ActionSequenceId.DefaultGestureOpen)
        await asyncio.sleep(1.0)

        print("\n=== 4) Turbo / 单位模式 ===")
        for fn in ("get_turbo_mode_enabled", "get_turbo_config", "get_finger_unit_mode",
                   "get_max_current", "get_voltage", "get_hand_type", "get_sku_type"):
            f = getattr(h, fn, None)
            if f is None:
                print(f"  {fn}: N/A")
                continue
            try:
                print(f"  {fn}: {await f(sid)}")
            except Exception as e:  # noqa: BLE001
                print(f"  {fn}: ERR {type(e).__name__}")
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

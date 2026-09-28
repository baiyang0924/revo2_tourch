#!/usr/bin/env python3
"""探测 Revo2 SDK 能力边界（只读，不动手指）

用途：确认「最大速度」「最短间隔」这类参数在 SDK 里到底怎么表达，
以及 FingerId / ActionSequenceId 有哪些可用值。
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


def dump_enum(name: str) -> None:
    obj = getattr(libstark, name, None)
    if obj is None:
        print(f"=== {name}: 不存在 ===")
        return
    print(f"=== {name} ===")
    try:
        for m in obj:
            print(f"    {m}  (value={getattr(m, 'value', '?')})")
    except TypeError:
        print(f"    (不可迭代) {obj}")


async def main() -> int:
    ctx = await init_bxipci_device(
        6, 127, master_id=1, is_canfd=True,
        hw_type=libstark.StarkHardwareType.Revo2Basic,
    )
    try:
        await asyncio.sleep(0.5)

        print("=== libstark 公开成员 ===")
        print("  " + " ".join(x for x in dir(libstark) if not x.startswith("_")))
        print()

        for n in ("FingerId", "ActionSequenceId", "FingerUnitMode"):
            dump_enum(n)
        print()

        print("=== handle 上所有 set_* / get_* / run_* 方法 ===")
        ms = sorted(x for x in dir(ctx.handle)
                    if x.startswith(("set_", "get_", "run_")) and not x.startswith("_"))
        for m in ms:
            print(f"    {m}")
        print()

        print("=== 各手指的硬件参数（速度 / 位置 / 电流上限）===")
        fids = []
        try:
            fids = list(libstark.FingerId)
        except TypeError:
            fids = []
        for fid in fids:
            row = [f"  {fid!s:<22}"]
            for label, fn in (("max_speed", "get_finger_max_speed"),
                              ("max_pos", "get_finger_max_position"),
                              ("min_pos", "get_finger_min_position"),
                              ("max_cur", "get_finger_max_current"),
                              ("prot_cur", "get_finger_protected_current")):
                f = getattr(ctx.handle, fn, None)
                if f is None:
                    row.append(f"{label}=N/A")
                    continue
                try:
                    row.append(f"{label}={await f(ctx.slave_id, fid)}")
                except Exception as e:  # noqa: BLE001
                    row.append(f"{label}=ERR({type(e).__name__})")
            print("  ".join(row))
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

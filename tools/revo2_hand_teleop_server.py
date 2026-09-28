#!/usr/bin/env python3
"""灵巧手遥操接收端（跑在机器人上，须以 root 运行）

监听 TCP，接收一行 "v1,v2,v3,v4,v5,v6"（每个 0~1000，0=伸直 1000=全弯），
校验后下发到灵巧手。

用法（在机器人上）：
    sudo bash /home/bxi/run.sh /home/bxi/hand_teleop_server.py --hand right
    sudo bash /home/bxi/run.sh /home/bxi/hand_teleop_server.py --hand right --port 9910

设计要点：
  * 限流：低于 MIN_GAP 的两次下发会合并，避免刷屏把总线占满
  * 校验：越界值直接拒绝并回送原因，不把脏数据发给硬件
  * 断线：客户端断开不会让手卡在最后一个姿态，超时后交回底层控制
"""
from __future__ import annotations

import argparse
import asyncio
import sys
import time

sys.path.insert(0, "/home/bxi/bxi_ws/bxi_revo2_example/src")

import rclpy  # noqa: E402
from bxi_can_node import (  # noqa: E402
    cleanup_bxipci_device,
    init_bxipci_device,
    libstark,
    stop_bxipci_runtime,
)

HAND = {"left": (5, 126), "right": (6, 127)}
N = 6
MIN_GAP = 0.045          # 最快约 22 Hz
IDLE_RELEASE = 1.5       # 超过这么久没数据就停止下发，交回底层控制


class TeleopServer:
    def __init__(self, bus: int, dev: int, port: int):
        self.bus, self.dev, self.port = bus, dev, port
        self.ctx = None
        self.last_send = 0.0
        self.last_ok = time.time()
        self.count = 0

    async def connect(self) -> None:
        self.ctx = await init_bxipci_device(
            self.bus, self.dev, master_id=1, is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )
        info = await self.ctx.handle.get_device_info(self.ctx.slave_id)
        print(f"✓ 手已连接：{info.hardware_type} 固件 {info.firmware_version}")

    async def handle(self, reader: asyncio.StreamReader,
                     writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        print(f"客户端接入 {peer}")
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                text = line.decode(errors="ignore").strip()
                if not text:
                    continue
                parts = text.split(",")
                if len(parts) != N:
                    writer.write(f"ERR 需要 {N} 个值，收到 {len(parts)}\n".encode())
                    await writer.drain()
                    continue
                try:
                    vals = [int(float(p)) for p in parts]
                except ValueError:
                    writer.write("ERR 非数值\n".encode())
                    await writer.drain()
                    continue
                bad = [i for i, v in enumerate(vals) if not (0 <= v <= 1000)]
                if bad:
                    writer.write(f"ERR 越界 index={bad} vals={vals}\n".encode())
                    await writer.drain()
                    continue

                now = time.time()
                if now - self.last_send < MIN_GAP:
                    continue     # 丢掉这一帧，下一帧再发（合并降频）
                self.last_send = now
                self.last_ok = now
                self.count += 1
                try:
                    await self.ctx.handle.set_finger_positions(self.ctx.slave_id, vals)
                except Exception as exc:  # noqa: BLE001
                    writer.write(f"ERR 下发失败 {exc}\n".encode())
                    await writer.drain()
                    continue
                if self.count % 40 == 1:
                    print(f"  #{self.count} {vals}")
        except asyncio.CancelledError:
            pass
        finally:
            print(f"客户端断开 {peer}")
            writer.close()

    async def watchdog(self) -> None:
        """长时间没有数据就停止占用，让手交回底层控制。"""
        while True:
            await asyncio.sleep(0.5)
            if time.time() - self.last_ok > IDLE_RELEASE and self.last_ok > 0:
                self.last_ok = 0.0     # 只提示一次
                print("空闲超时，已停止下发（手交回底层控制）")

    async def run(self) -> int:
        await self.connect()
        server = await asyncio.start_server(self.handle, "0.0.0.0", self.port)
        print(f"✓ 监听 0.0.0.0:{self.port}，等待本机客户端连接…")
        async with server:
            await asyncio.gather(server.serve_forever(), self.watchdog())
        return 0


async def amain() -> int:
    ap = argparse.ArgumentParser(description="灵巧手遥操接收端")
    ap.add_argument("--hand", choices=list(HAND), default="right")
    ap.add_argument("--port", type=int, default=9910)
    args = ap.parse_args()
    bus, dev = HAND[args.hand]
    srv = TeleopServer(bus, dev, args.port)
    try:
        return await srv.run()
    finally:
        if srv.ctx is not None:
            await cleanup_bxipci_device(srv.ctx)


def main() -> int:
    rclpy.init(args=None)
    try:
        return asyncio.run(amain())
    except KeyboardInterrupt:
        print("\n已停止")
        return 0
    finally:
        stop_bxipci_runtime()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())

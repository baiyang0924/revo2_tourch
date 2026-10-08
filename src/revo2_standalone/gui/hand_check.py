#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Revo2 灵巧手 · 逐指体检（本机 Modbus 直连）

用途：排查「某些手指不跟随」这类问题——先把手里的全部状态读出来，
再加 --wiggle 让每根手指单独小幅动一下，看是谁真的收到指令、谁没动。

默认只读，不会让手做任何动作。

用法：
    python hand_check.py --port COM24 --id 127                # 只读体检（安全）
    python hand_check.py --port COM24 --id 127 --wiggle --i-know-its-real
        ↑ 逐指小幅动作自检：每根手指单独 +15°（0.8 秒），读回后再回原位；
          最后再做一次「六个电机一起动」复现图形界面的下发方式。
          运行前确认手指活动范围内没有人、线缆和易损物。

判读方法（--wiggle 输出）：
    实际变化 ≈ +15° 且状态「运行中/空闲」 → 该指健康
    实际变化 ≈ 0  且状态「空闲」          → 指令没被接受（配置/校准问题）
    实际变化 ≈ 0  且状态「堵转」          → 被东西挡住或该指卡死
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 找 revo2_modbus：优先同目录（整个文件夹拷走也能独立跑），
# 其次上一级目录（仓库内布局：src/revo2_standalone/gui/ → src/revo2_standalone/）
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

import revo2_modbus as r2m  # noqa: E402

MOTORS = r2m.MOTOR_ORDER
LIMITS = list(r2m.ANGLE_LIMITS.values())


def hr(t: str) -> None:
    print("\n" + "─" * 68)
    print(t)
    print("─" * 68)


def line_of(vals) -> str:
    return "  ".join(f"{n[:3]}={v:g}" for n, v in zip(MOTORS, vals))


def main() -> int:
    ap = argparse.ArgumentParser(description="Revo2 逐指体检")
    ap.add_argument("--port", default="COM24")
    ap.add_argument("--id", type=int, default=127)
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--wiggle", action="store_true",
                    help="逐指小幅动作自检（手指会动！）")
    ap.add_argument("--i-know-its-real", dest="real", action="store_true",
                    help="确认是连着真机、周围已清空")
    args = ap.parse_args()

    if args.wiggle and not args.real:
        print("[拒绝] --wiggle 会让真机手指动作。确认周围安全后，"
              "请再加 --i-know-its-real。")
        return 2

    hand = r2m.Revo2Hand(port=args.port, device_id=args.id, baud=args.baud).open()
    exit_code = 0
    try:
        hr("① 设备信息")
        info = hand.read_info()
        print(f"  {info.side} | 固件 {info.firmware} | SN {info.serial_number or '（未读到）'}"
              f" | 触觉 {info.touch_vendor}")

        hr("② 关键配置寄存器（排查三指不动必看）")

        def rd(addr, n, holding=True):
            try:
                return hand.read_registers(addr, n, input_reg=not holding)
            except Exception as exc:  # noqa: BLE001
                print(f"    （读 {addr} 失败：{exc}）")
                return None

        unit = rd(937, 1)
        if unit is not None:
            mode = {0: "归一化(0-1000)", 1: "物理单位(度)"}.get(unit[0], "?")
            print(f"  单位模式 937        = {unit[0]}  → {mode}")
            print("    ⚠ 0（归一化）时写角度×10 会被按百分比解释；GUI 连接时会自动切到 1")
        auto = rd(1068, 1)
        if auto is not None:
            print(f"  位置自动校准 1068   = {auto[0]}  （1=开 0=关；关了就需手动校准）")
        prot = rd(930, 6)
        if prot:
            print(f"  保护电流 930-935    = {prot} mA")
        lock = rd(936, 1)
        if lock is not None:
            print(f"  拇指Aux锁定 936     = {lock[0]} mA")
        turbo = rd(1065, 3)
        if turbo:
            print(f"  Turbo/堵转/恢复     = {turbo}")
        lim = rd(946, 24, holding=True)
        if lim and len(lim) == 24:
            print("  限位读取 946-969（按『6 指一组』解读）：")
            print(f"    各组最小位置 = {lim[0:6]}")
            print(f"    各组最大位置 = {lim[6:12]}")
            print(f"    各组最大速度 = {lim[12:18]}")
            print(f"    各组最大电流 = {lim[18:24]}")

        hr("③ 当前实际状态")
        pos = hand.read_positions()
        cur = hand.read_currents()
        st = hand.read_motor_status()
        print("  实际位置（度）：")
        for n, p in zip(MOTORS, pos):
            print(f"    {n:<8} {p:>7.1f}")
        print(f"  实际电流（mA，含符号）：{line_of(cur)}")
        print(f"  电机状态：{ '  '.join(f'{n[:3]}={s}' for n, s in zip(MOTORS, st)) }")

        if not args.wiggle:
            print("\n  只读体检完成。要验证运动通路请加：--wiggle --i-know-its-real")
            return 0

        # ── 逐指小幅动作 ─────────────────────────────
        hr("④ 逐指小幅动作（每指 +15°，0.8 秒，单独进行，随后回原位）")
        print("  3 秒后开始，手指即将动作…")
        time.sleep(3)

        base = hand.read_positions()
        print(f"  基线位置：{line_of(base)}")
        results = []
        for i in range(6):
            target = list(base)
            target[i] = min(base[i] + 15.0, LIMITS[i] - 1.0)
            try:
                hand.set_angles(target, duration_ms=800, allow_write=True)
            except Exception as exc:  # noqa: BLE001
                print(f"  [{MOTORS[i]}] 下发失败：{exc}")
                results.append((MOTORS[i], 0.0, "下发失败"))
                continue
            time.sleep(1.1)
            p2 = hand.read_positions()
            c2 = hand.read_currents()
            s2 = hand.read_motor_status()
            d = round(p2[i] - base[i], 2)
            verdict = ("动了" if abs(d) > 5 else
                       ("被挡住/卡死" if s2[i] == "堵转" else "没动"))
            print(f"  [{i}] {MOTORS[i]:<8} 目标 +{15:g}° | 实际 {d:+.2f}° | "
                  f"电流 {c2[i]:>5} mA | 状态 {s2[i]}  → {verdict}")
            results.append((MOTORS[i], d, s2[i]))
            # 回原位
            hand.set_angles(base, duration_ms=800, allow_write=True)
            time.sleep(1.0)

        hr("⑤ 六个电机一起动（复现图形界面「开始跟随」的下发方式：全体到 30°）")
        try:
            hand.set_angles([30.0] * 6, duration_ms=900, allow_write=True)
            time.sleep(1.3)
            p3 = hand.read_positions()
            s3 = hand.read_motor_status()
            print(f"  目标 30° | 实际：{line_of(p3)}")
            print(f"  状态：{'  '.join(f'{n[:3]}={s}' for n, s in zip(MOTORS, s3))}")
            hand.set_angles(base, duration_ms=900, allow_write=True)
            time.sleep(1.3)
            print("  已回到基线")
        except Exception as exc:  # noqa: BLE001
            print(f"  六个电机测试失败：{exc}")

        hr("⑥ 结论")
        bad = [(n, d, s) for n, d, s in results if abs(d) <= 5]
        for n, d, s in results:
            print(f"  {n:<8} Δ={d:+.2f}°  状态={s}")
        if not bad:
            print("\n  全部六个电机都能响应指令 → 手和控制协议都正常。"
                  "若跟随仍不对，再查视觉侧（量程标定、手是否完整入镜）")
        else:
            print("\n  ⚠ 以下手指没有响应指令：")
            for n, d, s in bad:
                print(f"    {n}（Δ={d:+.2f}°，状态={s}）")
            print("  手侧问题确认 → 看上面 ② 的配置（单位模式/校准/限位）")
            exit_code = 1

    finally:
        hand.close()
        print("\n  串口已关闭")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Revo2 灵巧手自检脚本（Windows / Linux 通用，独立调试用）

用途：接上真机后，先用最小的动作验证「供电 → 通信 → 读反馈 → 写通路」，
      每一步都有明确判据，出问题能立刻定位到哪一层。

依赖：pyserial  +  同目录的 revo2_modbus.py

用法::

    # 不看硬件，先走一遍流程
    python 01_selfcheck.py --dry-run

    # 列出可用串口
    python 01_selfcheck.py --list-ports

    # 正式自检（只读，不发运动指令）
    python 01_selfcheck.py --port COM5 --id 126 --baud 460800

    # 加上写通路验证（开关手背灯，无机械风险）
    python 01_selfcheck.py --port COM5 --test-write

    # 加上极小幅度动作验证（手指会动，先确认周围清空！）
    python 01_selfcheck.py --port COM5 --test-motion

安全设计：
    - 默认**只读**，不会让手指动
    - 写操作必须显式加 --test-write / --test-motion
    - 动作测试固定用极小幅度（2°）和长时长，避免方向搞反顶到限位
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from revo2_modbus import (  # noqa: E402
        ANGLE_LIMITS, DEFAULT_BAUD, DEFAULT_DEVICE_ID, MOTOR_COUNT, MOTOR_ORDER,
        Revo2Error, Revo2Hand, TimeoutError_, CRCError, ModbusException_,
    )
except ImportError as exc:  # pragma: no cover
    print(f"[错误] 无法导入 revo2_modbus：{exc}")
    print("       请确认 revo2_modbus.py 与脚本在同一目录。")
    sys.exit(2)


# ── 输出辅助 ────────────────────────────────────────────────
class C:
    OK = "\033[32m"; WARN = "\033[33m"; BAD = "\033[31m"
    INFO = "\033[36m"; BOLD = "\033[1m"; DIM = "\033[2m"; END = "\033[0m"


if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
    for _n in ("OK", "WARN", "BAD", "INFO", "BOLD", "DIM", "END"):
        setattr(C, _n, "")


def step(n: int, total: int, text: str) -> None:
    print(f"\n{C.BOLD}[{n}/{total}] {text}{C.END}")


def ok(msg: str) -> None:
    print(f"      {C.OK}✓ {msg}{C.END}")


def warn(msg: str) -> None:
    print(f"      {C.WARN}! {msg}{C.END}")


def bad(msg: str) -> None:
    print(f"      {C.BAD}✗ {msg}{C.END}")


def info(msg: str) -> None:
    print(f"      {C.INFO}· {msg}{C.END}")


def todo(msg: str) -> None:
    print(f"        {C.DIM}→ {msg}{C.END}")


def list_ports() -> list[str]:
    try:
        import serial.tools.list_ports as lp  # type: ignore

        return [p.device for p in lp.comports()]
    except Exception:
        pass
    if sys.platform == "win32":
        try:
            import winreg  # type: ignore

            ports: list[str] = []
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
                i = 0
                while True:
                    try:
                        _, val, _ = winreg.EnumValue(key, i)
                        ports.append(str(val)); i += 1
                    except OSError:
                        break
            return sorted(ports)
        except Exception:
            return []
    import glob

    out: list[str] = []
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.usbserial*"):
        out.extend(glob.glob(pat))
    return sorted(out)


# ══════════════════════════════════════════════════════════════
def run_dry_run(argv: argparse.Namespace) -> int:
    """不接硬件，走一遍流程，让使用者知道每一步会看到什么。"""
    print(f"{C.BOLD}=== 演练模式（--dry-run）：不连接任何硬件 ==={C.END}")
    total = 6

    step(1, total, "检查依赖")
    try:
        import serial  # noqa: F401

        ok("pyserial 已安装")
    except Exception:
        warn("pyserial 未安装 → pip install pyserial")

    step(2, total, "枚举串口")
    ports = list_ports()
    ok(f"发现 {len(ports)} 个串口：{', '.join(ports) if ports else '（无）'}")

    step(3, total, "打开串口并握手")
    info(f"目标参数：port={argv.port or '<未指定>'} id={argv.id} baud={argv.baud}")
    todo("真机会在这里读寄存器 3000 取固件版本")
    todo("读出版本 = 物理层 + 帧格式 + CRC + 设备 ID 全对")

    step(4, total, "读设备信息")
    info("左右手（901）/ 固件版本（3000）/ SN（3010）/ 触觉厂商（970）")

    step(5, total, "读反馈数据")
    info(f"实际位置（2000–2005）/ 速度 / 电流 / 电机状态（2018–2023），共 {MOTOR_COUNT} 个电机")

    step(6, total, "写通路与动作验证")
    if argv.test_write:
        info("将开关手背灯（寄存器 904）验证写通路 —— 无机械风险")
    else:
        warn("未启用 --test-write，跳过写验证")
    if argv.test_motion:
        info("将让手指做极小幅度动作（2°，长时长）")
    else:
        warn("未启用 --test-motion，跳过动作验证")

    print(f"\n{C.BOLD}=== 演练结束 ==={C.END}")
    print("真机运行时请加 --port，例如：")
    print(f"  python {os.path.basename(__file__)} --port COM5 --id 126")
    print("\n参数对照：左手 ID 126 / 右手 ID 127；RS485 默认波特率 460800")
    print("角度顺序：" + " / ".join(MOTOR_ORDER))
    print("角度上限：" + " / ".join(f"{k}={v}°" for k, v in ANGLE_LIMITS.items()))
    return 0


# ══════════════════════════════════════════════════════════════
def run_real(argv: argparse.Namespace) -> int:
    total = 6
    print(f"{C.BOLD}=== Revo2 灵巧手自检 ==={C.END}")
    print(f"  串口：{argv.port}")
    print(f"  设备 ID：{argv.id}    波特率：{argv.baud}")

    # ── 1. 依赖 ────────────────────────────────────────
    step(1, total, "检查依赖")
    try:
        import serial  # noqa: F401

        ok("pyserial 已安装")
    except Exception:
        bad("缺少 pyserial")
        todo("pip install pyserial")
        return 2

    if os.environ.get("CONDA_DEFAULT_ENV"):
        warn(f"当前处于 conda 环境（{os.environ['CONDA_DEFAULT_ENV']}），"
             f"可能导致串口库版本冲突")

    # ── 2. 打开串口 ────────────────────────────────────
    step(2, total, "打开串口")
    hand = Revo2Hand(port=argv.port, device_id=argv.id, baud=argv.baud)
    try:
        hand.open()
        ok(f"已打开 {argv.port} @ {argv.baud}")
    except Exception as exc:
        bad(f"打开串口失败：{exc}")
        todo("① 串口号对不对（--list-ports 查）")
        todo("② 驱动装了吗（USB 转 485/CAN 芯片驱动）")
        todo("③ 模块是否被别的程序占用（关掉上位机再试）")
        todo("④ Linux 下：串口权限（dialout 组）")
        return 2

    exit_code = 0
    try:
        # ── 3. 握手：读固件版本 ────────────────────────
        step(3, total, "握手（读固件版本，寄存器 3000）")
        try:
            info_obj = hand.read_info()
            if info_obj.firmware:
                ok(f"固件版本：{info_obj.firmware}")
                ok("物理层 + 帧格式 + CRC + 设备 ID 全部正确")
            else:
                warn("读到了响应但版本字段为空（可能该型号地址不同）")
            if info_obj.side != "未知":
                ok(f"左右手识别：{info_obj.side}")
                expect_side = "左手" if argv.id == 126 else ("右手" if argv.id == 127 else "")
                if expect_side and info_obj.side != expect_side:
                    bad(f"左右手与 ID 不匹配！ID {argv.id} 一般是{expect_side}，"
                        f"但设备自称{info_obj.side}")
                    todo("检查是不是左右手接反了，或 ID 被改过")
            if info_obj.serial_number:
                info(f"SN：{info_obj.serial_number}")
            info(f"触觉模块：{info_obj.touch_vendor}")
            if info_obj.touch_vendor == "无触觉模块" and argv.expect_touch:
                warn("预期是触觉版，但设备报告无触觉模块")
        except (TimeoutError_, CRCError, ModbusException_) as exc:
            bad(f"握手失败：{type(exc).__name__}: {exc}")
            print()
            todo("按顺序排查：")
            todo("① A/B 线是否接反（485）或 CAN_H/CAN_L（CANFD）")
            todo("② 波特率是否 460800（若曾被恢复出厂则仍是 460800）")
            todo("③ 设备 ID 是否 126/127（长按按键 5 秒会恢复默认）")
            todo("④ 终端电阻 120Ω")
            todo("⑤ 手是否绿灯常亮（黄灯=供电过低）")
            todo("⑥ 是否插错接口（485 线插到 CANFD 口）")
            todo("⑦ 是否有别的程序正在占用同一只手")
            todo("详见 docs/05-故障排查手册.md 第 3、4 章")
            return 1

        # ── 4. 读反馈 ──────────────────────────────────
        step(4, total, "读反馈数据")
        try:
            pos = hand.read_positions()
            print("      实际位置（度）：")
            for i, (name, p) in enumerate(zip(MOTOR_ORDER, pos)):
                print(f"        [{i}] {name:<8} {p:>7.1f}°")
            ok("位置读取正常")
        except Revo2Error as exc:
            warn(f"位置读取失败：{exc}")

        try:
            cur = hand.read_currents()
            sta = hand.read_motor_status()
            print("      实际电流 / 状态：")
            for name, c, s in zip(MOTOR_ORDER, cur, sta):
                print(f"        {name:<8} {c:>6}  {s}")
            if any(s == "堵转" for s in sta):
                warn("有电机处于堵转状态 —— 手可能卡住了或正抓着东西")
            ok("电流与状态读取正常")
        except Revo2Error as exc:
            warn(f"电流/状态读取失败：{exc}")

        # 校准状态提示
        try:
            auto = hand.read_registers(1068, 1, input_reg=False)
            state = "开启（默认）" if auto and auto[0] == 1 else "关闭"
            info(f"位置自动校准：{state}")
            if state == "关闭":
                warn("自动校准已关闭 → 上电后必须手动校准（短按手背灯按键或写寄存器 1069）")
                todo("官方要求：上电后必须执行一次位置校准才能正常控制")
        except Revo2Error:
            pass

        # ── 5. 写通路验证 ──────────────────────────────
        step(5, total, "写通路验证（手背灯）")
        if not argv.test_write:
            warn("未启用 --test-write，跳过")
            info("建议加上它以验证写通路；手背灯无机械风险，是最安全的写测试")
        else:
            try:
                for state, label in ((False, "关"), (True, "开")):
                    hand.set_led(state, allow_write=True)
                    ok(f"写入「手背灯{label}」成功")
                    time.sleep(0.6)
                ok("写通路正常（请目视确认手背灯确实闪了一下）")
                todo("如果回应正常但灯没变化，检查寄存器 904 的当前值")
            except Revo2Error as exc:
                bad(f"写通路失败：{exc}")
                todo("读能通、写不通 → 多为参数或权限问题，重试一次")
                exit_code = max(exit_code, 1)

        # ── 6. 动作验证 ────────────────────────────────
        step(6, total, "动作验证（极小幅度）")
        if not argv.test_motion:
            warn("未启用 --test-motion，跳过（默认不动手指，安全）")
            info("启用前请务必确认：手指活动范围内没有人、线缆、易损物")
        else:
            print()
            print(f"      {C.WARN}{C.BOLD}⚠ 手指即将动作。请确认周围已清空。{C.END}")
            print(f"      {C.DIM}3 秒后开始...{C.END}")
            time.sleep(3)

            try:
                baseline = hand.read_positions()
                # 极小幅度：每个关节 +2°，长时长 1500ms，方向不对也不会顶到限位
                target = [min(p + 2.0, ANGLE_LIMITS[k])
                          for p, k in zip(baseline, ANGLE_LIMITS)]
                info(f"基线位置：{[round(p, 1) for p in baseline]}")
                info(f"目标位置：{[round(t, 1) for t in target]}（+2°）")

                hand.set_angles(target, duration_ms=1500, allow_write=True)
                time.sleep(2.0)

                after = hand.read_positions()
                print(f"      {C.OK}动作后位置：{[round(p, 1) for p in after]}{C.END}")

                moved = [round(a - b, 2) for a, b in zip(after, baseline)]
                print(f"      实际变化量：{moved}")
                if any(abs(m) > 0.3 for m in moved):
                    ok("手指有位移，运动通路正常")
                    if all(m >= -0.1 for m in moved):
                        info("方向符号与预期一致（正值为增大）")
                    else:
                        warn("部分关节向负方向变化 —— "
                             "说明该关节的角度方向约定相反，请核对官方文档再放开行程")
                else:
                    warn("没有检测到位移")
                    todo("① 绿灯是否常亮（位置校准完成了吗）")
                    todo("② 单位模式（937）是 0 归一化还是 1 物理单位")
                    todo("③ 保护电流（930–935）是否被写小")
                    todo("④ 期望时间是否超范围（1–2000ms）")

                # 回到基线
                hand.set_angles(baseline, duration_ms=1500, allow_write=True)
                time.sleep(2.0)
                ok("已回到基线位置")
            except Revo2Error as exc:
                bad(f"动作验证失败：{exc}")
                exit_code = max(exit_code, 1)

    finally:
        hand.close()
        info("串口已关闭")

    # ── 总结 ───────────────────────────────────────────
    print()
    if exit_code == 0:
        print(f"{C.OK}{C.BOLD}自检通过。{C.END}")
        print("下一步：")
        print("  · 独立调试继续 → docs/02-Windows单机调试指南.md")
        print("  · 整机联调继续 → docs/03-ELF3整机ROS2联调指南.md")
    else:
        print(f"{C.BAD}{C.BOLD}自检未全部通过。{C.END}")
        print("按上面的 → 提示逐项排查，或看 docs/05-故障排查手册.md")
    return exit_code


# ══════════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Revo2 灵巧手自检（默认只读，不会让手指动）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", default="", help="串口名，如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--id", type=int, default=DEFAULT_DEVICE_ID,
                    help="设备 ID（左手 126，右手 127）")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help="RS485 波特率，默认 460800")
    ap.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--dry-run", action="store_true",
                    help="不接硬件，走一遍流程看会输出什么")
    ap.add_argument("--test-write", action="store_true",
                    help="验证写通路（开关手背灯，无机械风险）")
    ap.add_argument("--test-motion", action="store_true",
                    help="验证运动通路（手指会动 2°，先确认周围清空！）")
    ap.add_argument("--expect-touch", action="store_true",
                    help="预期设备是触觉版（用于校验触觉模块检测结果）")
    argv = ap.parse_args()

    if argv.list_ports:
        ports = list_ports()
        if ports:
            print("可用串口：")
            for p in ports:
                print(f"  {p}")
        else:
            print("没有发现串口。")
            print("  ① 独立调试：先装 USB 转 485/CAN 芯片驱动，再插模块")
            print("  ② 整机调试：不需要串口，走机器人主控板 CAN5/CAN6")
        return 0

    if argv.dry_run:
        return run_dry_run(argv)

    if not argv.port:
        print("[错误] 必须指定 --port（或用 --list-ports 查看，或加 --dry-run 演练）")
        return 2

    return run_real(argv)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断。")
        sys.exit(130)

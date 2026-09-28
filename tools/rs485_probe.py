#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RS485 转接链路探测（BrainCo 485 转接模块专用）

用途
────
把这三件不确定的事，用一次扫描问清楚：

1. **485.0 / 485.1 哪个通道对应哪只手** —— 转接盒上两个口长得一样，丝印也只有编号；
2. **波特率到底是多少** —— 出厂写的是 460800，但被改过就不知道了；
3. **设备 ID 到底是 126 / 127 还是别的** —— 编号被改过同样会翻车。

⚠️ 安全承诺：本脚本**只读**。
   全程只发送功能码 ``0x03`` / ``0x04``（读保持寄存器 / 读输入寄存器），
   不发送任何写指令：不改设备 ID、不改波特率、不触发校准、不驱动电机。
   因此可以放心在**第一次接线时**就跑，不需要先读完整本手册。

用法
────
    # 1) 先看电脑识别到几个串口 —— 这是识别 485.0 / 485.1 的最强线索
    python tools/rs485_probe.py --list

    # 2) 快速扫描（默认）：常见波特率 × 常见设备 ID
    python tools/rs485_probe.py

    # 3) 只扫某一个串口（已经知道是哪两个口时）
    python tools/rs485_probe.py --port COM22 --port COM23

    # 4) 全量扫描：7 种波特率 × 地址 1..127（慢，但能兜住"参数被人改过"的情况）
    python tools/rs485_probe.py --full

    # 5) 无硬件演练：不打开任何串口，只看流程和输出样式
    python tools/rs485_probe.py --dry-run

    # 6) 只做 CRC 自检（验证协议实现没写错）
    python tools/rs485_probe.py --crc-selftest

前置条件
────────
    pip install pyserial
    ⚠️ 运行前请**关闭官方上位机**，否则串口被占用会打不开。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field

try:
    import serial  # type: ignore
    import serial.tools.list_ports as list_ports  # type: ignore
except Exception:  # pragma: no cover - 无 pyserial 时的降级路径
    serial = None
    list_ports = None


# ══════════════════════════════════════════════════════════
# 终端配色（Windows 10+ / 现代终端均支持 ANSI）
# ══════════════════════════════════════════════════════════
class C:
    BOLD = "\033[1m"
    DIM = "\033[2m"
    OK = "\033[92m"
    WARN = "\033[93m"
    BAD = "\033[91m"
    INFO = "\033[96m"
    ACC = "\033[95m"
    END = "\033[0m"

    @classmethod
    def off(cls) -> None:
        for name in ("BOLD", "DIM", "OK", "WARN", "BAD", "INFO", "ACC", "END"):
            setattr(cls, name, "")


def head(text: str) -> None:
    print(f"\n{C.BOLD}{C.INFO}{'═' * 62}{C.END}")
    print(f"{C.BOLD}{C.INFO}  {text}{C.END}")
    print(f"{C.BOLD}{C.INFO}{'═' * 62}{C.END}")


def sub(text: str) -> None:
    print(f"\n{C.BOLD}── {text} ──{C.END}")


def ok(msg: str) -> None:
    print(f"  {C.OK}✓{C.END} {msg}")


def warn(msg: str) -> None:
    print(f"  {C.WARN}!{C.END} {msg}")


def bad(msg: str) -> None:
    print(f"  {C.BAD}✗{C.END} {msg}")


def info(msg: str) -> None:
    print(f"  {C.DIM}{msg}{C.END}")


# ══════════════════════════════════════════════════════════
# Modbus-RTU 基础（只读用，与 src/revo2_standalone/revo2_modbus.py 同源）
# ══════════════════════════════════════════════════════════
FC_READ_HOLDING = 0x03
FC_READ_INPUT = 0x04

# 关键寄存器（详见 docs/04-通信协议速查-Modbus与CANFD.md）
REG_HAND_SIDE = 901       # 读：1 = 右手，2 = 左手
REG_TOUCH_VENDOR = 970    # 读：0 无 / 1 电容式 / 2 压阻式
REG_DEVICE_ID = 1000      # 设备 ID（1–254）
REG_RS485_BAUD = 1001     # RS485 波特率档位
REG_FW_VERSION = 3000     # 固件版本，20 字节 ASCII
REG_SN = 3010             # SN，20 字节 ASCII

BAUD_CODE_TO_VALUE = {
    0: 115200, 1: 57600, 2: 19200, 3: 460800,
    4: 1000000, 5: 2000000, 6: 5000000,
}
BAUD_VALUE_TO_CODE = {v: k for k, v in BAUD_CODE_TO_VALUE.items()}

# 扫描候选
BAUD_QUICK = [460800, 115200]
BAUD_FULL = [460800, 115200, 57600, 19200, 1000000, 2000000, 5000000]
ADDR_QUICK = [126, 127, 1, 2, 3]


def crc16_modbus(data: bytes) -> bytes:
    """Modbus-RTU CRC16，返回 2 字节（低字节在前）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_read(dev_id: int, func: int, addr: int, count: int) -> bytes:
    """构造读寄存器请求帧（功能码 0x03 / 0x04），只读，无副作用。"""
    body = bytes(
        [dev_id & 0xFF, func & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF,
         (count >> 8) & 0xFF, count & 0xFF]
    )
    return body + crc16_modbus(body)


def crc_ok(frame: bytes) -> bool:
    """整帧校验（含 CRC）。"""
    if len(frame) < 4:
        return False
    return crc16_modbus(frame[:-2]) == frame[-2:]


def decode_ascii(raw: bytes) -> str:
    """从寄存器原始字节里取出可打印 ASCII 字符串。"""
    text = raw.decode("ascii", errors="ignore")
    text = "".join(ch for ch in text if ch.isprintable())
    return text.strip("\x00 ").strip()


# ══════════════════════════════════════════════════════════
# 串口枚举
# ══════════════════════════════════════════════════════════
@dataclass
class PortDesc:
    device: str
    description: str = ""
    hwid: str = ""
    vid: int | None = None
    pid: int | None = None
    serial_number: str | None = None
    manufacturer: str | None = None


def enum_ports() -> list[PortDesc]:
    """枚举串口，尽量带上 USB 描述信息（用于识别转接盒的两路 COM）。"""
    if list_ports is not None:
        out: list[PortDesc] = []
        for p in list_ports.comports():
            out.append(PortDesc(
                device=p.device,
                description=p.description or "",
                hwid=p.hwid or "",
                vid=p.vid,
                pid=p.pid,
                serial_number=getattr(p, "serial_number", None),
                manufacturer=getattr(p, "manufacturer", None),
            ))
        return out

    # 无 pyserial：Windows 读注册表 / 类 Unix 扫 /dev
    if sys.platform == "win32":
        try:
            import winreg  # type: ignore

            ports: list[PortDesc] = []
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                                r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
                i = 0
                while True:
                    try:
                        _, val, _ = winreg.EnumValue(key, i)
                        ports.append(PortDesc(device=str(val)))
                        i += 1
                    except OSError:
                        break
            return sorted(ports, key=lambda d: d.device)
        except Exception:
            return []

    import glob

    cands: list[str] = []
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS[0-9]*",
                "/dev/cu.usbserial*", "/dev/cu.usbmodem*"):
        cands.extend(glob.glob(pat))
    return [PortDesc(device=d) for d in sorted(cands)]


def show_ports() -> None:
    """打印串口清单，并给出 485.0 / 485.1 的识别线索。"""
    head("串口清单")
    ports = enum_ports()
    if not ports:
        bad("没有枚举到任何串口。")
        if serial is None:
            info("当前环境没装 pyserial，只能做粗略枚举：pip install pyserial")
        info("检查：① 转接盒 TYPE-C 是否插好 ② USB 转串口驱动是否已装（CDM 驱动）")
        info("      ③ 换个 USB 口再试 ④ 设备管理器里有没有带感叹号的设备")
        return

    print(f"  {C.BOLD}{'端口':<8} {'描述':<44} {'VID:PID':<10}{C.END}")
    print(f"  {'-' * 8} {'-' * 44} {'-' * 10}")
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid and p.pid else "-"
        desc = (p.description or "")[:44]
        print(f"  {C.BOLD}{p.device:<8}{C.END} {desc:<44} {vidpid:<10}")
        if p.serial_number or p.manufacturer:
            extra = " / ".join(x for x in (p.manufacturer, p.serial_number) if x)
            print(f"  {C.DIM}         └ {extra}{C.END}")

    print()
    if len(ports) >= 2:
        ok(f"识别到 {len(ports)} 个串口 —— 很可能正好对应转接盒的 485.0 / 485.1 两路。")
        info("下一步：直接跑 `python tools/rs485_probe.py`，"
             "脚本会逐个口探测，告诉你哪个口上是哪只手。")
    else:
        warn(f"只识别到 {len(ports)} 个串口。")
        info("BrainCo 485 转接模块是双路输出，正常应枚举出 2 个串口。")
        info("只有一个的话：① 另一路确实没插 ② 该转接盒是单路版本 "
             "③ 驱动没装全（装 CDM 驱动后重新插拔）")


# ══════════════════════════════════════════════════════════
# 单点探测
# ══════════════════════════════════════════════════════════
@dataclass
class Hit:
    """一次成功应答的记录。"""
    port: str
    baud: int
    addr: int
    func: int
    raw: bytes
    exception_code: int | None = None   # 非 None = 设备存在但请求被拒
    fw: str = ""
    side: str = ""
    sn: str = ""
    touch: str = ""
    device_id_reg: int | None = None
    baud_reg: int | None = None
    extra: dict[str, str] = field(default_factory=dict)

    @property
    def side_zh(self) -> str:
        return {"left": "左手", "right": "右手"}.get(self.side, self.side or "未知")


def _read_raw(ser, dev_id: int, func: int, addr: int, count: int) -> bytes | None:
    """发一帧读请求，返回原始响应（可能为空）。"""
    req = build_read(dev_id, func, addr, count)
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    resp = ser.read(5 + count * 2)
    if not resp:
        return None
    if crc_ok(resp):
        return resp
    # CRC 不匹配：可能响应还没收全，补读一次
    time.sleep(0.03)
    more = ser.read(64)
    combined = resp + more
    return combined if crc_ok(combined) else None


def probe(ser, dev_id: int, timeout_tag: str = "") -> Hit | None:
    """探测指定地址上是否有 Revo2 设备。

    先试读固件版本（最能说明"这是个什么设备"），
    命中后继续读左右手 / SN / 设备 ID / 波特率档位。
    """
    del timeout_tag  # 保留签名，便于后续扩展
    # ① 主探测：读固件版本前 10 个寄存器（20 字节）
    resp = None
    used_func = FC_READ_INPUT
    for func in (FC_READ_INPUT, FC_READ_HOLDING):
        resp = _read_raw(ser, dev_id, func, REG_FW_VERSION, 10)
        if resp is not None:
            used_func = func
            break
    if resp is None:
        return None

    hit = Hit(port="", baud=0, addr=dev_id, func=used_func, raw=resp)

    # 异常响应（功能码最高位置 1）也说明"该地址上有设备在应答"
    if len(resp) >= 5 and resp[1] & 0x80:
        hit.exception_code = resp[2]
        return hit

    # 数据已被 CRC 校验通过，开始解析
    body = resp[3:-2]
    if hit.func == FC_READ_INPUT:
        hit.fw = decode_ascii(body[:20])

    # ② 补读左右手 / 触觉厂商（保持寄存器）
    try:
        r = _read_raw(ser, dev_id, FC_READ_HOLDING, REG_HAND_SIDE, 1)
        if r is not None and crc_ok(r) and len(r) >= 7 and not (r[1] & 0x80):
            val = (r[3] << 8) | r[4]
            hit.side = {1: "right", 2: "left"}.get(val, f"未知({val})")
    except Exception:
        pass

    try:
        r = _read_raw(ser, dev_id, FC_READ_HOLDING, REG_TOUCH_VENDOR, 1)
        if r is not None and crc_ok(r) and len(r) >= 7 and not (r[1] & 0x80):
            val = (r[3] << 8) | r[4]
            hit.touch = {0: "无触觉模块", 1: "电容式", 2: "压阻式"}.get(
                val, f"未知({val})")
    except Exception:
        pass

    # ③ 补读 SN
    for func in (FC_READ_INPUT, FC_READ_HOLDING):
        r = _read_raw(ser, dev_id, func, REG_SN, 10)
        if r is not None and crc_ok(r) and len(r) > 5 and not (r[1] & 0x80):
            hit.sn = decode_ascii(r[3:-2][:20])
            if hit.sn:
                break

    # ④ 补读设备 ID 与 485 波特率档位（只读！）
    try:
        r = _read_raw(ser, dev_id, FC_READ_HOLDING, REG_DEVICE_ID, 1)
        if r is not None and crc_ok(r) and len(r) >= 7 and not (r[1] & 0x80):
            hit.device_id_reg = (r[3] << 8) | r[4]
    except Exception:
        pass

    try:
        r = _read_raw(ser, dev_id, FC_READ_HOLDING, REG_RS485_BAUD, 1)
        if r is not None and crc_ok(r) and len(r) >= 7 and not (r[1] & 0x80):
            hit.baud_reg = (r[3] << 8) | r[4]
    except Exception:
        pass

    return hit


# ══════════════════════════════════════════════════════════
# 扫描主流程
# ══════════════════════════════════════════════════════════
def scan_port(port: str, bauds: list[int], addrs: list[int],
              timeout: float, verbose: bool) -> list[Hit]:
    """在单个串口上按 波特率 × 地址 扫描，返回全部命中。"""
    if serial is None:
        bad("缺少 pyserial，无法打开串口：pip install pyserial")
        return []

    hits: list[Hit] = []
    for baud in bauds:
        try:
            ser = serial.Serial(port=port, baudrate=baud, bytesize=8,
                                parity="N", stopbits=1, timeout=timeout)
        except Exception as exc:
            bad(f"{port} @ {baud} 打开失败：{exc}")
            info("常见原因：串口被官方上位机占用 / 没有权限 / 设备已拔出。")
            info("→ 先关闭上位机再跑本脚本。")
            return hits

        try:
            time.sleep(0.15)
            found_here = 0
            for addr in addrs:
                hit = probe(ser, addr)
                if hit is not None:
                    hit.port = port
                    hit.baud = baud
                    hits.append(hit)
                    found_here += 1
                    if verbose or found_here <= 3:
                        tag = "（异常响应，设备在但请求被拒）" if hit.exception_code else ""
                        ok(f"{port} @ {baud}  地址 {addr} 有应答 {tag}")
                    if found_here >= 2 and len(addrs) > 5:
                        info(f"{port} @ {baud} 已发现 2 个设备，跳过本波特率的其余地址")
                        break
            if not found_here and verbose:
                info(f"{port} @ {baud}  无应答")
        finally:
            try:
                ser.close()
            except Exception:
                pass
    return hits


def print_hits(hits: list[Hit]) -> None:
    """打印命中结果表。"""
    if not hits:
        return
    head("探测结果")
    for h in hits:
        print(f"  {C.BOLD}{C.OK}● {h.port}{C.END}  "
              f"波特率 {C.BOLD}{h.baud}{C.END}  设备 ID {C.BOLD}{h.addr}{C.END}")
        if h.exception_code is not None:
            warn(f"设备有应答但返回异常码 {h.exception_code}"
                 "（地址对了，寄存器或功能码不合适）")
            continue
        if h.side_zh != "未知":
            print(f"     左右手     ：{C.BOLD}{h.side_zh}{C.END}")
        if h.fw:
            print(f"     固件版本   ：{h.fw}")
            info("     判读：固件 ≤ V0.0.14 不能直接刷，先找供应方确认升级路径")
        if h.sn:
            print(f"     SN         ：{h.sn}")
        if h.touch:
            print(f"     触觉模块   ：{h.touch}")
        if h.device_id_reg is not None:
            print(f"     设备 ID 寄存器(1000)：{h.device_id_reg}")
        if h.baud_reg is not None:
            baud_now = BAUD_CODE_TO_VALUE.get(h.baud_reg, f"未知档位({h.baud_reg})")
            print(f"     RS485 波特率寄存器(1001)：档位 {h.baud_reg} → {baud_now}")
            if h.baud_reg != BAUD_VALUE_TO_CODE.get(h.baud, -1):
                warn("注意：寄存器里的波特率档位与你刚刚通上的波特率不一致，"
                     "可能刚被改过，以寄存器值为准复核。")
        print()


def print_yaml_snippet(hits: list[Hit]) -> None:
    """输出可直接粘贴进 config/hand_params.yaml 的片段。"""
    if not hits:
        return
    head("可直接粘贴进 config/hand_params.yaml 的片段")

    by_side: dict[str, Hit] = {}
    for h in hits:
        if h.exception_code is not None:
            continue
        key = h.side or f"addr{h.addr}"
        by_side.setdefault(key, h)

    print("standalone_debug:")
    print("  adapter:")
    print("    type: \"485\"")
    print("    model: \"BrainCo 485 转接模块（TYPE-C 双路 485.0 / 485.1）\"")
    print("    driver_installed: true")
    for key, name in (("left", "left"), ("right", "right")):
        h = by_side.get(key)
        print(f"  {name}:")
        if h is None:
            print("    port: \"\"          # 本次扫描未发现，确认该路是否已接")
            print("    device_id: " + ("126" if name == "left" else "127"))
            print("    baud_rate: 460800")
        else:
            print(f"    port: \"{h.port}\"          # 扫描实测")
            print(f"    device_id: {h.addr}")
            print(f"    baud_rate: {h.baud}")
            if h.side_zh != "未知":
                print(f"    # 已确认：{h.side_zh}"
                      f"{'（固件 ' + h.fw + '）' if h.fw else ''}")
    print()
    info("把上面几行合并进 config/hand_params.yaml 的 standalone_debug 段，"
         "下次出问题先对比这里。")
    print(f"\n  {C.WARN}⚠️ 本次扫描只读，没有改动手上任何参数。{C.END}")


def summarize(hits: list[Hit], scanned_ports: list[str]) -> None:
    """结果解读与下一步建议。"""
    head("结果解读")

    real = [h for h in hits if h.exception_code is None]
    if not real:
        bad("没有发现任何应答。按下面顺序排查：")
        print()
        info("① 供电：手背灯是否绿灯常亮？黄灯闪烁 = 供电过低；完全不亮 = 没上电。")
        info("② 接线：转接盒 485.o / 485.1 的 A/B 与手上 485-A / 485-B 是否同极性？")
        info("   —— 接反不会烧，但一定不通。对调一次再扫。")
        info("③ 共地：转接盒 GND 与手端 GND 是否连通？桌面单机调试建议接上。")
        info("④ 端口：设备管理器里两个 COM 口是否都在？是否被上位机占用？")
        info("⑤ 参数被改过：跑 `--full`，用 7 种波特率 × 地址 1–127 兜一遍。")
        return

    # 建立 通道 → 手 的映射
    sides = {h.side_zh for h in real if h.side_zh != "未知"}
    print(f"  发现 {C.BOLD}{len(real)}{C.END} 个在线设备，"
          f"分布在 {C.BOLD}{len({h.port for h in real})}{C.END} 个串口上。")
    print()
    for h in sorted(real, key=lambda x: (x.port, x.baud)):
        print(f"  {C.BOLD}{h.port}{C.END} @ {h.baud}  →  地址 {h.addr}"
              f"  {C.ACC}{h.side_zh}{C.END}")
    print()

    if "左手" in sides and "右手" in sides:
        ok("左右手都找到了 —— 接线、供电、通信链路全通。")
        info("接下来：① 用上位机或 01_selfcheck.py 复核 ② 把参数写进 hand_params.yaml")
    elif len(real) == 1:
        warn("只找到一只手。若两只手都接在转接盒上，检查另一路的 A/B 与端口。")
    else:
        warn("找到了设备但读不出左右手标识 —— 固件可能较老，"
             "以物理观察（拇指朝向来判断左右）和接线位置为准。")

    print()
    sub("下一步建议")
    info("1. 打开官方上位机，选 MODBUS 协议，用**扫描出来的**端口 + 波特率 + 设备 ID 连接；")
    info("   能读出固件版本 = 与本次扫描互相印证。")
    info("2. 进 Motor 页，把一根手指的滑块推到 5%，确认手指跟随，再看方向。")
    info("3. 把参数抄进 config/hand_params.yaml。")
    print()
    warn("桌面调试期间**不要改设备 ID 和波特率**。")
    info("   手上的 ID / 波特率是「一个手一套参数」，改完不记，")
    info("   装回机器人（走 CANFD，期望 ID 左 126 / 右 127）时就会找不到手。")


# ══════════════════════════════════════════════════════════
# 无硬件演练 / CRC 自检
# ══════════════════════════════════════════════════════════
def dry_run() -> int:
    head("无硬件演练（--dry-run）")
    info("本模式不打开任何串口，只演示流程与输出样式。")
    print()
    sub("流程")
    for i, step in enumerate([
        "枚举串口（对应转接盒的 485.0 / 485.1 两路）",
        "对每个串口 × 每个波特率 × 每个候选地址，发一帧只读请求",
        "收到 CRC 正确的应答 → 判为在线设备",
        "补读左右手 / 固件版本 / SN / 触觉模块 / 设备 ID / 波特率档位",
        "汇总成「哪个口是哪只手」的结论 + 可粘贴的 YAML 片段",
    ], 1):
        print(f"  {i}. {step}")
    print()
    sub("本脚本会发送的全部报文类型（共 2 种，都是只读）")
    demo_addrs = [(126, FC_READ_INPUT, REG_FW_VERSION, 10),
                  (126, FC_READ_HOLDING, REG_HAND_SIDE, 1)]
    for dev_id, func, addr, count in demo_addrs:
        frame = build_read(dev_id, func, addr, count)
        name = "读输入寄存器" if func == FC_READ_INPUT else "读保持寄存器"
        print(f"  {name} 0x{func:02X}  寄存器 {addr} x{count}")
        print(f"    {frame.hex(' ').upper()}")
    print()
    ok("演练完成。全程没有任何写指令，也没有打开串口。")
    return 0


def crc_selftest() -> int:
    """用公开的 Modbus 标准测试向量验证 CRC16 实现。"""
    head("CRC16 自检")
    info("向量取自 Modbus-RTU 广泛引用的标准示例帧，CRC 为「低字节在前」的线序。")

    # (待校验报文, 期望 CRC 线序)
    vectors = [
        (bytes.fromhex("010300000001"), bytes.fromhex("840A")),
        (bytes.fromhex("010600010003"), bytes.fromhex("980B")),
    ]
    failures = 0
    for body, expect in vectors:
        got = crc16_modbus(body)
        line = (f"{body.hex(' ').upper():<24} → 实得 {got.hex(' ').upper():<6}"
                f" 期望 {expect.hex(' ').upper()}")
        if got == expect:
            ok(line)
        else:
            bad(line)
            failures += 1

    # 整帧自校验：验证「构造 → 校验」闭环
    sub("整帧闭环自校验（构造 → 校验）")
    for dev_id, func, addr, count, name in (
        (126, FC_READ_INPUT, REG_FW_VERSION, 10, "读左手固件版本"),
        (127, FC_READ_INPUT, REG_FW_VERSION, 10, "读右手固件版本"),
        (126, FC_READ_HOLDING, REG_HAND_SIDE, 1, "读左右手标识"),
    ):
        frame = build_read(dev_id, func, addr, count)
        flag = C.OK + "✓" + C.END if crc_ok(frame) else C.BAD + "✗" + C.END
        print(f"  {flag} {name:<14} {frame.hex(' ').upper()}")
        if not crc_ok(frame):
            failures += 1

    print()
    if failures:
        bad(f"{failures} 项未通过，协议实现有问题，先别接真机。")
        return 1
    ok("全部通过，CRC16 实现与本项目帧构造均正确。")
    return 0


# ══════════════════════════════════════════════════════════
# 入口
# ══════════════════════════════════════════════════════════
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rs485_probe.py",
        description="BrainCo Revo2 485 转接链路探测（只读，不改动手上任何参数）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python tools/rs485_probe.py --list\n"
            "  python tools/rs485_probe.py\n"
            "  python tools/rs485_probe.py --port COM22 --port COM23 --full\n"
        ),
    )
    parser.add_argument("--list", action="store_true",
                        help="只列出串口清单，不做探测")
    parser.add_argument("--port", action="append", default=None,
                        metavar="COMx", help="指定串口，可重复；默认扫描全部串口")
    parser.add_argument("--full", action="store_true",
                        help="全量扫描：7 种波特率 × 地址 1–127（慢）")
    parser.add_argument("--addr", type=int, action="append", default=None,
                        help="指定设备 ID，可重复（覆盖默认候选）")
    parser.add_argument("--baud", type=int, action="append", default=None,
                        help="指定波特率，可重复（覆盖默认候选）")
    parser.add_argument("--timeout", type=float, default=0.06,
                        help="单次等待响应的秒数，默认 0.06")
    parser.add_argument("--dry-run", action="store_true",
                        help="无硬件演练：不打开串口")
    parser.add_argument("--crc-selftest", action="store_true",
                        help="只做 CRC16 实现自检")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="打印每一个探测组合（含无应答的）")
    parser.add_argument("--no-color", action="store_true",
                        help="关闭彩色输出（重定向到文件时用）")

    args = parser.parse_args(argv)

    if args.no_color or not sys.stdout.isatty():
        C.off()

    if args.crc_selftest:
        return crc_selftest()

    print(f"{C.BOLD}Revo2 RS485 转接链路探测{C.END}")
    info("只读工具：全程只发功能码 0x03 / 0x04，不改 ID、不改波特率、不驱动电机。")

    if args.dry_run:
        return dry_run()

    if args.list:
        show_ports()
        return 0

    if serial is None:
        bad("缺少 pyserial 依赖：pip install pyserial")
        info("没有 pyserial 只能枚举串口（--list），无法做实际探测。")
        return 2

    # 确定要扫的端口
    if args.port:
        ports = [PortDesc(device=p) for p in args.port]
    else:
        ports = enum_ports()
        if not ports:
            bad("没有枚举到任何串口。先跑 `--list` 看设备管理器情况。")
            return 1
        head("待扫描串口")
        for p in ports:
            desc = f"  —— {p.description}" if p.description else ""
            print(f"  {p.device}{desc}")
        info("提示：转接盒应识别出 2 个串口。若只有 1 个，先看是否驱动没装全。")

    bauds = args.baud or (BAUD_FULL if args.full else BAUD_QUICK)
    if args.addr:
        addrs = args.addr
    else:
        addrs = list(range(1, 128)) if args.full else ADDR_QUICK

    head("开始扫描")
    combos = len(ports) * len(bauds) * len(addrs)
    print(f"  串口 {len(ports)} 个 × 波特率 {len(bauds)} 个 × 地址 {len(addrs)} 个"
          f" = 最多 {C.BOLD}{combos}{C.END} 次探测")
    info(f"波特率候选：{', '.join(str(b) for b in bauds)}")
    if args.full:
        info("地址候选：1–127（全量）")
        info(f"预计耗时约 {combos * args.timeout:.0f} 秒（每端口），请耐心等待。")
    else:
        info(f"地址候选：{', '.join(str(a) for a in addrs)}")
        info(f"预计耗时约 {combos * args.timeout:.1f} 秒（每端口）。"
             "没扫到就加 --full 全量兜底。")
    print()

    all_hits: list[Hit] = []
    t0 = time.time()
    for p in ports:
        sub(f"扫描 {p.device}")
        hits = scan_port(p.device, bauds, addrs, args.timeout, args.verbose)
        if hits:
            for h in hits:
                side = h.side_zh if h.exception_code is None else "异常响应"
                ok(f"{p.device} @ {h.baud}  地址 {h.addr}  →  {side}")
        else:
            info(f"{p.device}：无应答")
        all_hits.extend(hits)

    elapsed = time.time() - t0
    print()
    info(f"扫描完成，用时 {elapsed:.1f} 秒。")

    if all_hits:
        print_hits(all_hits)
        print_yaml_snippet(all_hits)
    summarize(all_hits, [p.device for p in ports])
    return 0 if any(h.exception_code is None for h in all_hits) else 1


if __name__ == "__main__":
    sys.exit(main())

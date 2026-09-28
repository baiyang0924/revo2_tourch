#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通信链路分层诊断工具（Revo2 灵巧手）

按「供电 → 物理接线 → 参数匹配 → 软件环境 → 应用逻辑」分层自检，
逐层给出结论和下一步动作。

设计原则：**只做只读探测**，绝不发送运动指令，可以放心在真机上运行。

用法：
    python tools/comm_diagnose.py                 # 跑全部检查
    python tools/comm_diagnose.py --layer 3       # 只查参数/串口层
    python tools/comm_diagnose.py --probe         # 额外尝试读固件版本（会打开串口）
    python tools/comm_diagnose.py --port COM5 --id 126 --baud 460800 --probe
    python tools/comm_diagnose.py --crc-selftest  # 只验证 CRC16 实现

依赖：
    必需：无（只用标准库）
    可选：pyserial（用于枚举串口和实际探测）
"""

from __future__ import annotations

import argparse
import os
import platform
import socket
import sys
import time

# ── Windows 控制台中文输出 ────────────────────────────────
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


# ══════════════════════════════════════════════════════════
# 输出辅助
# ══════════════════════════════════════════════════════════
class C:
    OK = "\033[32m"
    WARN = "\033[33m"
    BAD = "\033[31m"
    INFO = "\033[36m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    END = "\033[0m"

    @classmethod
    def off(cls) -> None:
        for name in ("OK", "WARN", "BAD", "INFO", "BOLD", "DIM", "END"):
            setattr(cls, name, "")


if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
    C.off()

results: list[tuple[str, str]] = []  # (等级, 消息)


def head(text: str) -> None:
    print(f"\n{C.BOLD}{'═' * 62}{C.END}")
    print(f"{C.BOLD}  {text}{C.END}")
    print(f"{C.BOLD}{'═' * 62}{C.END}")


def sub(text: str) -> None:
    print(f"\n{C.BOLD}── {text} ──{C.END}")


def ok(msg: str) -> None:
    print(f"  {C.OK}[通过]{C.END} {msg}")
    results.append(("OK", msg))


def warn(msg: str) -> None:
    print(f"  {C.WARN}[注意]{C.END} {msg}")
    results.append(("WARN", msg))


def bad(msg: str) -> None:
    print(f"  {C.BAD}[异常]{C.END} {msg}")
    results.append(("BAD", msg))


def info(msg: str) -> None:
    print(f"  {C.INFO}[信息]{C.END} {msg}")


def todo(msg: str) -> None:
    print(f"    {C.DIM}→ {msg}{C.END}")


def check(desc: str, value: str | None = None) -> None:
    """打印人工确认项。"""
    tail = f"  当前值：{value}" if value else ""
    print(f"  {C.WARN}[待确认]{C.END} {desc}{tail}")


# ══════════════════════════════════════════════════════════
# Modbus-RTU 基础
# ══════════════════════════════════════════════════════════
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


def build_read_registers(dev_id: int, func: int, addr: int, count: int) -> bytes:
    """构造读寄存器请求帧。func: 0x03 保持寄存器 / 0x04 输入寄存器。"""
    body = bytes(
        [dev_id & 0xFF, func & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF,
         (count >> 8) & 0xFF, count & 0xFF]
    )
    return body + crc16_modbus(body)


def verify_crc_frame(frame: bytes) -> bool:
    """校验整帧（含 CRC）是否正确。"""
    if len(frame) < 4:
        return False
    return crc16_modbus(frame[:-2]) == frame[-2:]


# ══════════════════════════════════════════════════════════
# 第 1 层：供电
# ══════════════════════════════════════════════════════════
LAYER1_ITEMS = [
    ("XT30 供电接头是否插到底（有明确手感）", "docs/01 4.3"),
    ("电源适配器开关 / 插排开关是否打开", "docs/01 4.3"),
    ("手端实测电压是否在型号额定范围内", "基础版 12-28V；进阶/触觉版 12-64V"),
    ("负载状态下电压是否明显跌落", "最大电流可达 4.65A @24V"),
    ("上电后手背灯是否为绿灯常亮", "绿闪=开机中；黄闪=供电过低；红=异常"),
    ("上电后手指是否自动张开（位置校准完成）", "docs/01 4.4"),
]


def layer1_power(argv: argparse.Namespace) -> None:
    sub("第 1 层 · 供电")
    info("本层需要人工观察与万用表测量，工具无法代劳。")
    print()
    for desc, hint in LAYER1_ITEMS:
        check(desc, hint)
    print()
    info("判读表：")
    print("    绿灯闪烁 → 开机中，等待复位完成")
    print("    绿灯常亮 → 正常，可以进入第 3 层")
    print("    黄灯闪烁 → 供电过低，先解决供电再往下查")
    print("    红灯常亮 → 硬件异常，断电重启；仍红灯请联系售后")
    print("    完全不亮 → 没有供电，检查接头 / 开关 / 电源")


# ══════════════════════════════════════════════════════════
# 第 2 层：物理接线
# ══════════════════════════════════════════════════════════
def layer2_wiring(argv: argparse.Namespace) -> None:
    sub("第 2 层 · 物理接线")
    info("本层需要人工确认插接情况，工具无法探测。")
    print()

    print(f"  {C.BOLD}【整机（RS 路径 A）】{C.END}")
    check("左手线接在 CAN5，右手线接在 CAN6", "接反不会烧，但程序找不到手")
    check("手臂末端线缆接头插紧、无松动")
    check("线缆没有被关节夹住、没有绷紧")
    check("腕部法兰螺钉旋入深度 < 3 mm", "螺纹孔深 3.5mm，超了会压裂内部板")
    print()

    print(f"  {C.BOLD}【独立调试 485（路径 B）】{C.END}")
    check("A / B 线极性是否正确", "接反通常不通但不会烧")
    check("总线首尾是否装了 120Ω 终端电阻")
    check("现场干扰大时两端 GND 是否互连")
    print()

    print(f"  {C.BOLD}【独立调试 CANFD（路径 B）】{C.END}")
    check("CAN_H / CAN_L 极性是否正确")
    check("仲裁域波特率是否为固定 1 Mbps", "官方规定固定值，不可改")
    check("数据段波特率是否与手一致", "出厂默认 5 Mbps")
    check("两端是否装了 120Ω 终端电阻")
    print()

    print(f"  {C.BOLD}【务必排除的误区】{C.END}")
    warn("不要去找「协议拨码开关」——那是基础版才有的，进阶版/触觉版没有")
    todo("进阶版/触觉版的 485 / CANFD / EtherCAT 是三个独立物理接口，插哪个用哪个")


# ══════════════════════════════════════════════════════════
# 第 3 层：参数匹配 / 串口
# ══════════════════════════════════════════════════════════
BAUD_MAP = {0: 115200, 1: 57600, 2: 19200, 3: 460800, 4: 1000000, 5: 2000000, 6: 5000000}
CANFD_BAUD_MAP = {
    0: "100k", 1: "125k", 2: "200k", 3: "250k", 4: "400k", 5: "500k",
    6: "800k", 7: "1M", 8: "2M", 9: "4M", 10: "5M",
}

SERIAL_FALLBACK_WIN = (
    r"HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM"
)


def list_serial_ports() -> tuple[list[str], str]:
    """枚举串口。返回 (端口列表, 使用的方法说明)。"""
    # 首选 pyserial
    try:
        import serial.tools.list_ports as lp  # type: ignore

        ports = [p.device for p in lp.comports()]
        return ports, "pyserial"
    except Exception:
        pass

    # Windows 回退：读注册表
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
                        ports.append(str(val))
                        i += 1
                    except OSError:
                        break
            return sorted(ports), "注册表（建议安装 pyserial 以获得更详细信息）"
        except Exception:
            return [], "Windows 注册表读取失败"

    # Linux / macOS 回退：扫 /dev
    import glob

    cands: list[str] = []
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS[0-9]*",
                "/dev/cu.usbserial*", "/dev/cu.usbmodem*"):
        cands.extend(glob.glob(pat))
    return sorted(cands), "扫描 /dev"


def try_import_serial():
    try:
        import serial  # type: ignore

        return serial
    except Exception:
        return None


def read_firmware_version(serial_mod, port: str, dev_id: int, baud: int,
                          timeout: float = 1.0) -> tuple[bool, str]:
    """尝试读固件版本（输入寄存器 3000，20 字节）。返回 (成功, 说明)。"""
    try:
        ser = serial_mod.Serial(port=port, baudrate=baud,
                                bytesize=8, parity="N", stopbits=1,
                                timeout=timeout)
    except Exception as exc:
        return False, f"打开串口失败：{exc}"

    try:
        time.sleep(0.1)
        # 先试读输入寄存器 0x04；部分实现把版本放在保持寄存器，故 0x03 也试一次
        for func, addr, count in ((0x04, 3000, 20), (0x03, 3000, 20)):
            ser.reset_input_buffer()
            req = build_read_registers(dev_id, func, addr, count)
            ser.write(req)
            ser.flush()
            time.sleep(0.25)
            raw = ser.read(128)
            if not raw:
                continue
            if not verify_crc_frame(raw):
                continue
            # 从数据段提取 ASCII
            text = "".join(chr(b) if 32 <= b < 127 else "" for b in raw)
            text = text.strip()
            if text:
                return True, f"func=0x{func:02X} addr={addr} 原始响应={raw.hex(' ')}\n" \
                             f"       提取到的可读字符：{text}"
            return True, f"func=0x{func:02X} addr={addr} 收到 {len(raw)} 字节，CRC 校验通过"
        return False, "没有收到有效响应（CRC 不匹配或超时）"
    finally:
        try:
            ser.close()
        except Exception:
            pass


def layer3_params(argv: argparse.Namespace) -> None:
    sub("第 3 层 · 参数匹配 / 串口")

    # 串口枚举
    ports, method = list_serial_ports()
    if ports:
        ok(f"发现 {len(ports)} 个串口（{method}）：{', '.join(ports)}")
        todo("独立调试时，这个列表里应该有你的 USB 转接器")
    else:
        warn(f"没有发现串口（{method}）")
        todo("独立调试：① 先装 USB 转 485/CAN 芯片驱动（如 CDM 驱动）② 再插模块")
        todo("整机调试：不需要串口，走机器人主控板的 CAN5/CAN6，转第 4 层")

    if "pyserial" not in method:
        info("提示：pip install pyserial 可获得更完整的串口信息与探测能力")

    # 参数核对
    print()
    info("参数必须三方一致：设备端 / 物理接线 / 程序里写的")
    print("    左手：总线 CAN5，设备 ID 126 (0x7E)，master_id 1")
    print("    右手：总线 CAN6，设备 ID 127 (0x7F)，master_id 1")
    print("    RS485 波特率默认 460800；CANFD 数据段默认 5M，仲裁域固定 1M")
    print()
    info("RS485 波特率对照（寄存器 1001）：")
    for k, v in BAUD_MAP.items():
        print(f"      {k} → {v}")
    print()
    info("CANFD 波特率对照（寄存器 1002）：")
    for k, v in CANFD_BAUD_MAP.items():
        print(f"      {k} → {v}")
    print()
    warn("高频事故：误触「长按手背按键 5 秒恢复出厂设置」→ ID 与波特率全部回到默认")
    todo("症状：昨天正常，今天完全不通，且没人改过程序 → 检查参数是否被复位")

    # 实际探测
    if argv.probe:
        print()
        serial_mod = try_import_serial()
        if serial_mod is None:
            bad("--probe 需要 pyserial：pip install pyserial")
            return
        port = argv.port
        if not port:
            if not ports:
                bad("没有可用串口，无法探测")
                return
            port = ports[0]
            info(f"未指定 --port，使用第一个可用串口：{port}")

        info(f"探测 {port}（设备 ID {argv.id}，波特率 {argv.baud}）...")
        okflag, detail = read_firmware_version(serial_mod, port, argv.id, argv.baud)
        if okflag:
            ok("收到有效响应，物理层 + 帧格式 + CRC + 设备 ID 全部正确")
            print(f"      {detail}")
            todo("下一步：读实际位置（输入寄存器 2000~2005）")
            todo("再下一步：写手背灯开关（保持寄存器 904）验证写通路 —— 用无风险寄存器先试")
        else:
            bad(detail)
            todo("逐项排查：① A/B 是否接反 ② 波特率 ③ 设备 ID ④ 终端电阻 ⑤ CRC 实现")
            todo("注意：本工具读的是寄存器 3000（固件版本），若该型号地址不同请对照官方协议文档")


# ══════════════════════════════════════════════════════════
# 第 4 层：软件环境
# ══════════════════════════════════════════════════════════
def layer4_environment(argv: argparse.Namespace) -> None:
    sub("第 4 层 · 软件环境")

    # Python
    info(f"Python {platform.python_version()} ({sys.executable})")

    # SDK
    sdk = None
    for name in ("bc_stark_sdk", "bc_stark_sdk_v2", "libstark", "stark"):
        try:
            mod = __import__(name)
            sdk = (name, getattr(mod, "__version__", "未知版本"),
                   os.path.dirname(getattr(mod, "__file__", "") or ""))
            break
        except Exception:
            continue
    if sdk:
        ok(f"已安装 SDK：{sdk[0]} {sdk[1]}")
        info(f"路径：{sdk[2]}")
        todo("首次接真机前，核对本目录下 .pyi 里的方法名是否与脚本一致")
    else:
        warn("没有检测到强脑 Python SDK（bc_stark_sdk）")
        todo("安装：pip install bc-stark-sdk==1.5.1 --index-url https://pypi.org/simple/")
        todo("注意：官方指定 1.5.1，别装最新版；装在系统 Python，别在 conda 里")

    # conda 干扰
    conda = os.environ.get("CONDA_DEFAULT_ENV")
    if conda:
        warn(f"当前处在 conda 环境：{conda}")
        todo("这是「装了 SDK 却报 ModuleNotFoundError」的头号原因，先 conda deactivate")

    # ROS 2 环境
    print()
    if os.environ.get("ROS_DISTRO"):
        ok(f"ROS 2 环境已加载：ROS_DISTRO={os.environ['ROS_DISTRO']}")
        todo("确认是 humble")
    else:
        info("未检测到 ROS 2 环境（整机联调时才需要）")

    ros_domain = os.environ.get("ROS_DOMAIN_ID")
    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if ros_domain is not None:
        info(f"ROS_DOMAIN_ID={ros_domain}")
    else:
        warn("ROS_DOMAIN_ID 未设置")
        todo("整机联调必须设置，且要与机器人硬件节点一致")
        todo("官方规则：遥控器启动 = 30 + 机器人序号；App 启动固定 22")
        todo("查法：grep -r ROS_DOMAIN_ID /etc/systemd/system/")
    if rmw:
        info(f"RMW_IMPLEMENTATION={rmw}")
    else:
        warn("RMW_IMPLEMENTATION 未设置")
        todo("整机联调需设为 rmw_cyclonedds_cpp，否则可能与硬件节点无法通信")

    if platform.system() == "Linux":
        print()
        info("Linux 额外提示：")
        todo("串口权限：sudo usermod -aG dialout $USER 后重新登录")
        todo("检查串口是否存在：ls -l /dev/ttyUSB* /dev/ttyACM*")
        todo("CAN 接口状态：ip -details link show can0")

    print()
    warn("整机联调第一坑：不要「先在普通用户终端设变量，再 sudo su 切 root」")
    todo("sudo su 不会继承这些环境变量，必须在 root shell 里重新 export")


# ══════════════════════════════════════════════════════════
# 第 5 层：应用逻辑
# ══════════════════════════════════════════════════════════
def layer5_application(argv: argparse.Namespace) -> None:
    sub("第 5 层 · 应用逻辑")
    info("本层是「链路已通但行为不对」，按症状对号入座：")
    print()
    rows = [
        ("手指方向反了", "先用单指极小幅度试方向，确认后再放开行程"),
        ("动一下就停", "保护电流偏小，检查寄存器 930~935（默认 500mA）"),
        ("报超范围拒绝写入", "单位模式不一致，检查寄存器 937：0 归一化 / 1 物理单位"),
        ("手指抖动/异响", "负载超规格（单指捏力 ≥15N、整手负载 ≥20kg）"),
        ("Turbo 不生效", "掉电恢复默认关闭，需重新开启寄存器 1065"),
        ("关机后参数全没了", "正常行为：限位/速度/电流/保护电流掉电恢复默认"),
        ("动作卡在中间", "期望时间超范围，寄存器 1010 允许 1~2000 ms"),
        ("触觉数据读不到", "开关未开：电容式 4000~4004 / 压阻式 6130~6134"),
        ("读不到数据但终端刷帧", "大概率左右手接反：程序在 CAN5 找 126，手却在 CAN6"),
    ]
    for sym, fix in rows:
        print(f"  {C.WARN}·{C.END} {sym}")
        todo(fix)


# ══════════════════════════════════════════════════════════
# 汇总
# ══════════════════════════════════════════════════════════
def summary() -> None:
    head("诊断汇总")
    n_ok = sum(1 for lv, _ in results if lv == "OK")
    n_warn = sum(1 for lv, _ in results if lv == "WARN")
    n_bad = sum(1 for lv, _ in results if lv == "BAD")

    print(f"  通过 {C.OK}{n_ok}{C.END} 项　"
          f"注意 {C.WARN}{n_warn}{C.END} 项　"
          f"异常 {C.BAD}{n_bad}{C.END} 项")

    if n_bad == 0:
        print(f"\n  {C.OK}没有发现硬性异常。{C.END}")
    else:
        print(f"\n  {C.BAD}存在 {n_bad} 项异常，建议先解决它们再看其它项。{C.END}")

    print(f"\n  {C.DIM}排查原则：一次只改一个变量；先分层再定位；"
          f"上层的问题不要在下层找。{C.END}")
    print(f"  {C.DIM}完整排查逻辑见 docs/05-故障排查手册.md{C.END}")


def crc_selftest() -> int:
    head("CRC16 自检")
    cases = [
        (b"\x01\x03\x00\x00\x00\x01", "84 0a"),
        (b"\x7e\x04\x0b\xb8\x00\x14", None),
        (b"\x7f\x03\x03\x88\x00\x06", None),
    ]
    fail = 0
    for data, expect in cases:
        got = crc16_modbus(data).hex(" ")
        flag = ""
        if expect is not None:
            if got == expect:
                flag = f"{C.OK}正确{C.END}"
            else:
                flag = f"{C.BAD}错误（期望 {expect}）{C.END}"
                fail += 1
        print(f"  {data.hex(' '):<24} → CRC {got}  {flag}")
    print()
    if fail == 0:
        ok("CRC16 实现正确，可用于 RS485 通信")
    else:
        bad("CRC16 实现有误，先修这个再调通信")
    return 1 if fail else 0


# ══════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Revo2 灵巧手通信链路分层诊断（只读，不会发运动指令）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--layer", type=int, choices=[1, 2, 3, 4, 5],
                    help="只跑指定层；不指定则跑 1~5 层")
    ap.add_argument("--probe", action="store_true",
                    help="实际打开串口尝试读取固件版本（需要 pyserial）")
    ap.add_argument("--port", default="", help="串口名，例如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--id", type=int, default=126,
                    help="设备 ID（左手 126，右手 127）")
    ap.add_argument("--baud", type=int, default=460800,
                    help="RS485 波特率，默认 460800")
    ap.add_argument("--crc-selftest", action="store_true",
                    help="只运行 CRC16 自检")
    argv = ap.parse_args()

    if argv.crc_selftest:
        return crc_selftest()

    head("Revo2 灵巧手通信链路分层诊断")
    print(f"  主机：{platform.system()} {platform.release()}")
    print(f"  时间：{time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  {C.DIM}本工具只做只读探测，不会发送运动指令{C.END}")

    layers = {
        1: layer1_power,
        2: layer2_wiring,
        3: layer3_params,
        4: layer4_environment,
        5: layer5_application,
    }
    wanted = [argv.layer] if argv.layer else [1, 2, 3, 4, 5]
    for n in wanted:
        head(f"第 {n} 层")
        layers[n](argv)

    summary()

    print("\n  下一步建议：")
    print("    · 通信层已通 → 转 docs/03 或 docs/02 做实际控制")
    print("    · 通信层不通 → 按 docs/05 第 3、4 章逐项排查")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

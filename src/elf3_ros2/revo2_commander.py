#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Revo2 灵巧手上位控制端（跑在你的电脑上）

角色：**commander**
    只发高层指令，不碰底层总线。指令通过 ROS 2 话题发给机器人上的
    revo2_bridge_node.py，由 bridge 翻译成 SDK 调用。

侧别：本项目 ELF3 实机装配的是**右手**（CAN6 / ID 127），
    故 `--hand` 默认值即 `right`。切左手需显式 `--hand left`（CAN5 / ID 126）。

用法（在你电脑上）::

    # 看有哪些手势 / 序列
    python3 revo2_commander.py list

    # 发一个手势（默认右手）
    python3 revo2_commander.py gesture fist

    # 直接指定 6 个关节角（度）和时长
    python3 revo2_commander.py angles 0,0,0,0,0,0 --duration 1200

    # 播放动作序列
    python3 revo2_commander.py sequence 基础自检

    # 播放序列并循环
    python3 revo2_commander.py sequence 挥手打招呼 --loop

    # 监视反馈（位置 / 电流）
    python3 revo2_commander.py monitor

    # 只看会发什么，不真的发（先确认参数）
    python3 revo2_commander.py gesture fist --print-only

    # 明示指定左手
    python3 revo2_commander.py gesture fist --hand left

环境要求：
    电脑上需要能跑 rclpy，且与机器人在同一 ROS 2 域（ROS_DOMAIN_ID 一致）。
    详见 docs/03-ELF3整机ROS2联调指南.md

⚠️ 安全设计
    - 发送前会做角度范围和时长范围校验，超范围直接拒绝
    - --print-only 可以不发送只预览
    - 序列播放每步之间会打印当前步骤，方便随时 Ctrl+C 中断
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any

# ⚠️ 以下为**兜底默认值**：启动时会被 config/revo2_hardware.yaml 覆盖
#    （见 apply_hardware_config）。调整硬件参数请改配置文件，不必改这里。
MOTOR_COUNT = 6
MOTOR_ORDER = ["拇指Flex", "拇指Aux", "食指", "中指", "无名指", "小拇指"]
ANGLE_LIMITS = [59.0, 89.0, 80.8, 80.8, 80.8, 80.8]
DURATION_MIN, DURATION_MAX = 1, 2000


def default_hardware_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (
        os.path.join(here, "..", "..", "config", "revo2_hardware.yaml"),
        os.path.join(here, "config", "revo2_hardware.yaml"),
    ):
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return ""


def apply_hardware_config(path: str = "") -> None:
    """用硬件常量文件覆盖模块级常量（电机顺序、角度上限、时长范围）。

    与 bridge 节点读同一份 config/revo2_hardware.yaml。
    集中到单点后，改动硬件参数只需维护该文件，
    不会出现「改了一处、漏了另一处」的情况。
    """
    global MOTOR_ORDER, ANGLE_LIMITS, DURATION_MIN, DURATION_MAX
    p = path or default_hardware_path()
    if not p:
        info("未找到 revo2_hardware.yaml，使用内置默认硬件常量")
        return
    data = load_yaml(p)
    if not data:
        return
    order = data.get("motor_order")
    if isinstance(order, list) and len(order) == MOTOR_COUNT:
        MOTOR_ORDER = [str(x) for x in order]
    lim = data.get("angle_limits")
    if isinstance(lim, list) and len(lim) == MOTOR_COUNT:
        ANGLE_LIMITS = [float(x) for x in lim]
    dr = data.get("duration_range")
    if isinstance(dr, list) and len(dr) == 2:
        DURATION_MIN, DURATION_MAX = int(dr[0]), int(dr[1])
    info(f"硬件常量已加载：{os.path.basename(p)}"
         f"（上限 {' / '.join(f'{v}°' for v in ANGLE_LIMITS)}）")


# ── 输出辅助 ────────────────────────────────────────────────
class C:
    OK = "\033[32m"; WARN = "\033[33m"; BAD = "\033[31m"
    INFO = "\033[36m"; BOLD = "\033[1m"; DIM = "\033[2m"; END = "\033[0m"


if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
    for _n in ("OK", "WARN", "BAD", "INFO", "BOLD", "DIM", "END"):
        setattr(C, _n, "")


def ok(m: str) -> None: print(f"{C.OK}[成功]{C.END} {m}")
def warn(m: str) -> None: print(f"{C.WARN}[注意]{C.END} {m}")
def bad(m: str) -> None: print(f"{C.BAD}[失败]{C.END} {m}")
def info(m: str) -> None: print(f"{C.INFO}[信息]{C.END} {m}")


def default_config_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (
        os.path.join(here, "config", "sequences.yaml"),
        os.path.join(here, "..", "..", "src", "elf3_ros2", "config", "sequences.yaml"),
    ):
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return ""


def default_gesture_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (
        os.path.join(here, "..", "..", "config", "gestures.yaml"),
        os.path.join(here, "config", "gestures.yaml"),
    ):
        if os.path.exists(cand):
            return os.path.abspath(cand)
    return ""


def load_yaml(path: str) -> dict[str, Any]:
    """读 YAML：优先用 pyyaml，没装则回落到内置的零依赖解析器。"""
    if not path or not os.path.exists(path):
        bad(f"文件不存在：{path or '(未找到配置文件)'}")
        print("  提示：确认 config/gestures.yaml 与 "
              "src/elf3_ros2/config/sequences.yaml 存在")
        return {}
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        ".."))
        from miniyaml import MiniYamlError, load as _load  # type: ignore

        return _load(path) or {}
    except ImportError:
        bad("既没有 pyyaml 也找不到 miniyaml.py")
        print("  pip install pyyaml   或者确认 src/miniyaml.py 存在")
        return {}
    except Exception as exc:
        bad(f"解析失败 {path}：{exc}")
        return {}


def validate(angles: list[float], duration: int) -> str | None:
    """校验参数，返回错误说明或 None。"""
    if len(angles) != MOTOR_COUNT:
        return f"需要 {MOTOR_COUNT} 个角度，收到 {len(angles)} 个"
    for i, (a, lim) in enumerate(zip(angles, ANGLE_LIMITS)):
        if not (0 <= a <= lim):
            return f"{MOTOR_ORDER[i]} 的角度 {a}° 超出安全范围 0–{lim}°"
    if not (DURATION_MIN <= duration <= DURATION_MAX):
        return (f"时长 {duration}ms 超出范围 "
                f"{DURATION_MIN}–{DURATION_MAX}ms")
    return None


# ══════════════════════════════════════════════════════════════
class Commander:
    def __init__(self, hand: str, ros_domain: str | None,
                 print_only: bool = False):
        self.hand = hand
        self.print_only = print_only
        if ros_domain:
            os.environ["ROS_DOMAIN_ID"] = ros_domain

        self.node = None
        self.pub_cmd = None
        self.pub_gesture = None
        self._last_state: list[float] = []

        if print_only:
            info("--print-only：只预览，不会真的发送")
            return

        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float64MultiArray, String
        except ImportError:
            bad("找不到 rclpy。请在你的电脑上装 ROS 2 并加载环境：")
            print("        source /opt/ros/humble/setup.bash")
            print("        或者加 --print-only 只做参数预览")
            raise SystemExit(2)

        self.rclpy = rclpy
        self.F64 = Float64MultiArray
        self.Str = String

        rclpy.init()
        self.node = Node(f"revo2_commander_{hand}")
        self.pub_cmd = self.node.create_publisher(
            Float64MultiArray, f"/revo2/{hand}/command", 10)
        self.pub_gesture = self.node.create_publisher(
            String, f"/revo2/{hand}/gesture", 10)
        self.node.create_subscription(
            Float64MultiArray, f"/revo2/{hand}/state", self._on_state, 10)
        self.node.create_subscription(
            String, f"/revo2/{hand}/status", self._on_status, 10)

        info(f"commander 就绪：hand={hand} "
             f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(未设置)')}")

    def _on_state(self, msg) -> None:
        self._last_state = list(msg.data)

    def _on_status(self, msg) -> None:
        print(f"{C.DIM}[bridge]{C.END} {msg.data}")

    def spin(self, seconds: float) -> None:
        """转一会儿，让回调有机会执行。"""
        if self.print_only or self.node is None:
            time.sleep(min(seconds, 0.3))
            return
        end = time.time() + seconds
        while time.time() < end and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def send_angles(self, angles: list[float], duration: int) -> None:
        err = validate(angles, duration)
        if err:
            bad(f"参数不合法：{err}")
            raise SystemExit(2)
        print(f"  目标角度：{[round(a, 1) for a in angles]}（度）")
        print(f"  时长：{duration} ms")
        print(f"  电机顺序：{' / '.join(MOTOR_ORDER)}")
        if self.print_only:
            ok("预览完毕（未发送）")
            return
        msg = self.F64()
        msg.data = [float(a) for a in angles] + [float(duration)]
        self.pub_cmd.publish(msg)
        self.spin(0.4)
        ok("指令已发送")

    def send_gesture(self, name: str) -> None:
        if self.print_only:
            ok(f"预览：将发送手势「{name}」（未发送）")
            return
        msg = self.Str()
        msg.data = name
        self.pub_gesture.publish(msg)
        self.spin(0.4)
        ok(f"手势「{name}」已发送")

    def play_sequence(self, steps: list[dict], loop: bool,
                      sequence_name: str) -> None:
        if not steps:
            bad("序列没有步骤")
            return
        print(f"\n{C.BOLD}播放序列「{sequence_name}」，共 {len(steps)} 步{C.END}")
        print(f"{C.WARN}⚠ 请确认手指活动范围内没有人、线缆、易损物{C.END}\n")

        rounds = 0
        while True:
            rounds += 1
            if loop:
                print(f"{C.BOLD}── 第 {rounds} 轮 ──{C.END}")
            for i, st in enumerate(steps, 1):
                name = st.get("name") or f"步骤{i}"
                angles = st.get("angles") or []
                dur = int(st.get("duration_ms") or 1000)
                expect = st.get("expect")
                phase = st.get("robot_phase")

                print(f"\n[{i}/{len(steps)}] {name}")
                if phase:
                    print(f"  {C.INFO}本体配合：{phase}{C.END}")
                if expect:
                    print(f"  {C.INFO}预期现象：{expect}{C.END}")

                try:
                    self.send_angles([float(a) for a in angles], dur)
                except SystemExit:
                    raise

                if self.print_only:
                    continue
                # 等这一步走完，再进下一步
                self.spin(dur / 1000.0 + 0.15)

            if not loop:
                break
            print(f"\n{C.DIM}--- 本轮结束，循环继续（Ctrl+C 停止）---{C.END}")
            self.spin(0.2)

    def monitor(self, hz: float, duration: float | None) -> None:
        print(f"{C.BOLD}监视反馈（位置 / 电流），Ctrl+C 退出{C.END}")
        print(f"{C.DIM}{'时间':<9}"
              + "".join(f"{n:>9}" for n in MOTOR_ORDER)
              + f"{'|':>3}" + "".join(f"{n:>9}" for n in MOTOR_ORDER)
              + f"{C.END}")
        print(f"{C.DIM}{'':<9}" + "位置(度)".center(9 * 6)
              + f"{'|':>3}" + "电流".center(9 * 6) + f"{C.END}")

        start = time.time()
        try:
            while True:
                if duration and (time.time() - start) > duration:
                    break
                self.spin(1.0 / hz)
                d = self._last_state
                if len(d) >= MOTOR_COUNT * 2:
                    pos = d[:MOTOR_COUNT]
                    cur = d[MOTOR_COUNT:MOTOR_COUNT * 2]
                    ts = time.strftime("%H:%M:%S")
                    line = f"{ts:<9}" + "".join(f"{p:>9.1f}" for p in pos) \
                        + f"{'|':>3}" + "".join(f"{c:>9.0f}" for c in cur)
                    print(line, end="\r")
        except KeyboardInterrupt:
            print()
            info("监视已停止")

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if not self.print_only:
            try:
                self.rclpy.shutdown()
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════
def cmd_list(argv: argparse.Namespace) -> int:
    gp = argv.gestures_file or default_gesture_path()
    sp = argv.sequences_file or default_config_path()

    print(f"{C.BOLD}手势库{C.END}  ({gp})")
    gdata = load_yaml(gp)
    gestures = gdata.get("gestures") or {}
    if not gestures:
        warn("没有读到手势。检查文件是否存在、pyyaml 是否安装。")
    for name, g in gestures.items():
        desc = g.get("description", "")
        risk = g.get("risk", "")
        angles = g.get("angles", [])
        print(f"  {name:<16} {desc}")
        print(f"  {'':<16} 角度={angles} 风险={risk}")

    print(f"\n{C.BOLD}动作序列{C.END}  ({sp})")
    sdata = load_yaml(sp)
    seqs = sdata.get("sequences") or {}
    if not seqs:
        warn("没有读到序列。")
    for name, s in seqs.items():
        steps = s.get("steps") or []
        loop = "循环" if s.get("loop") else "单次"
        sync = " 需与本体同步" if s.get("robot_sync") else ""
        print(f"  {name:<16} {s.get('description', '')}")
        print(f"  {'':<16} {len(steps)} 步 / {loop}{sync}")

    print(f"\n{C.DIM}角度顺序固定：{' / '.join(MOTOR_ORDER)}{C.END}")
    print(f"{C.DIM}上限：{' / '.join(f'{k}={v}°' for k, v in zip(MOTOR_ORDER, ANGLE_LIMITS))}{C.END}")
    return 0


def cmd_gesture(argv: argparse.Namespace, cmd: Commander) -> int:
    print(f"{C.BOLD}发送手势「{argv.name}」→ {argv.hand}{C.END}")
    cmd.send_gesture(argv.name)
    return 0


def cmd_angles(argv: argparse.Namespace, cmd: Commander) -> int:
    raw = argv.values
    try:
        angles = [float(x) for x in raw.replace(" ", "").split(",") if x != ""]
    except ValueError:
        bad(f"角度格式错误：「{raw}」")
        print("  正确写法：-- 0,0,0,0,0,0     或    50,70,78,78,78,78")
        return 2
    print(f"{C.BOLD}发送关节角 → {argv.hand}{C.END}")
    cmd.send_angles(angles, argv.duration)
    return 0


def cmd_sequence(argv: argparse.Namespace, cmd: Commander) -> int:
    sp = argv.sequences_file or default_config_path()
    data = load_yaml(sp)
    seqs = data.get("sequences") or {}
    name = argv.name
    if name not in seqs:
        bad(f"没有找到序列「{name}」")
        if seqs:
            print("  可用序列：" + "、".join(seqs))
        return 2
    s = seqs[name]
    loop = argv.loop or bool(s.get("loop"))
    if s.get("robot_sync") and not argv.print_only:
        warn("该序列标记为「需与本体动作同步」，请确认本体控制器已就绪")
    cmd.play_sequence(s.get("steps") or [], loop, name)
    return 0


def cmd_monitor(argv: argparse.Namespace, cmd: Commander) -> int:
    cmd.monitor(argv.hz, argv.duration if argv.duration > 0 else None)
    return 0


# ══════════════════════════════════════════════════════════════
def main() -> int:
    # 公共选项：既允许写在子命令前面，也允许写在后面
    # （用 SUPPRESS 作默认值，这样写在前面时不会被子命令的空默认值覆盖）
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--hand", choices=["left", "right"],
                        default=argparse.SUPPRESS, help="控制哪只手")
    common.add_argument("--ros-domain", default=argparse.SUPPRESS,
                        help="覆盖 ROS_DOMAIN_ID（必须与机器人一致）")
    common.add_argument("--print-only", action="store_true",
                        default=argparse.SUPPRESS,
                        help="只预览要发送的内容，不真的发送")
    common.add_argument("--gestures-file", default=argparse.SUPPRESS,
                        help="手势库 yaml 路径")
    common.add_argument("--sequences-file", default=argparse.SUPPRESS,
                        help="序列库 yaml 路径")
    common.add_argument("--hardware-config", default=argparse.SUPPRESS,
                        help="硬件常量 yaml 路径（电机顺序、角度上限、时长范围）")

    ap = argparse.ArgumentParser(
        description="Revo2 灵巧手上位控制端（发高层指令，经 ROS 2 发给机器人上的 bridge）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--hand", choices=["left", "right"], default="right",
                    help="控制哪只手，默认 right（本项目实机为右手）")
    ap.add_argument("--ros-domain", default=None,
                    help="覆盖 ROS_DOMAIN_ID（必须与机器人一致）")
    ap.add_argument("--print-only", action="store_true",
                    help="只预览要发送的内容，不真的发送")
    ap.add_argument("--gestures-file", default="", help="手势库 yaml 路径")
    ap.add_argument("--sequences-file", default="", help="序列库 yaml 路径")
    ap.add_argument("--hardware-config", default="",
                    help="硬件常量 yaml 路径（电机顺序、角度上限、时长范围）。"
                         "默认自动查找 config/revo2_hardware.yaml；"
                         "找不到时使用内置默认值")

    sub = ap.add_subparsers(dest="cmd", required=True)

    p_list = sub.add_parser("list", parents=[common],
                            help="列出手势库与序列库")
    # 注意：带 parents 的子解析器下 set_defaults() 不可靠，这里显式分发
    DISPATCH = {
        "list": cmd_list,
        "gesture": cmd_gesture,
        "angles": cmd_angles,
        "sequence": cmd_sequence,
        "monitor": cmd_monitor,
    }

    p_g = sub.add_parser("gesture", parents=[common], help="发送一个手势")
    p_g.add_argument("name", help="手势名，见 list 的输出")

    p_a = sub.add_parser("angles", parents=[common],
                         help="发送 6 个目标关节角（度，逗号分隔）")
    p_a.add_argument("values", help="例如 0,0,0,0,0,0")
    p_a.add_argument("--duration", type=int, default=1000,
                     help="期望时长 ms，范围 1–2000，默认 1000")

    p_s = sub.add_parser("sequence", parents=[common], help="播放动作序列")
    p_s.add_argument("name", help="序列名，见 list 的输出")
    p_s.add_argument("--loop", action="store_true", help="循环播放")

    p_m = sub.add_parser("monitor", parents=[common],
                         help="监视位置与电流反馈")
    p_m.add_argument("--hz", type=float, default=5.0, help="刷新频率，默认 5")
    p_m.add_argument("--duration", type=float, default=0.0,
                     help="监视时长（秒），0 表示一直监视")

    argv = ap.parse_args()

    # 硬件常量：在派发任何子命令之前统一加载，
    # 使 validate() 与各处提示都用同一份来源
    apply_hardware_config(getattr(argv, "hardware_config", "") or "")

    # 子命令里没写的选项，回落到顶层默认值（SUPPRESS 机制下属性可能不存在）
    for attr, fallback in (
        ("hand", "left"), ("ros_domain", None), ("print_only", False),
        ("gestures_file", ""), ("sequences_file", ""),
    ):
        if not hasattr(argv, attr):
            setattr(argv, attr, fallback)

    # list 不需要连 ROS，先处理
    if argv.cmd == "list":
        return cmd_list(argv)

    try:
        cmd = Commander(argv.hand, argv.ros_domain, argv.print_only)
    except SystemExit as exc:
        return int(exc.code or 0)

    try:
        return DISPATCH[argv.cmd](argv, cmd)
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    finally:
        cmd.close()


if __name__ == "__main__":
    sys.exit(main())

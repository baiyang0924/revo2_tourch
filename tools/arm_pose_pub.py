#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按名字把手臂位姿持续发布到 ActuatorCmds 覆盖话题（ELF3 双臂 7 关节）。

为什么需要这个工具
------------------
官方文档里的覆盖命令是手写的 `ros2 topic pub -r 20 ...` 加上一串
30 个数字的数组。第一次上真机时这么干有两个真实风险：

  1. **数组和关节名错位** —— 数组是按位置对应的，错一位就是另一根关节
     在动，而且不会报错；
  2. **覆盖超时 0.2 s** —— 一旦发布断了，机器人会在 0.2 s 内混合回
     当前策略输出。手臂抓着东西的时候这意味着东西会被甩掉。

所以本工具做三件事：把位姿存在 yaml 里按名字取、保证持续发布不断流、
在真正把电机动起来之前拦住明显不安全的参数。

用法
----
    python3 tools/arm_pose_pub.py --list
    python3 tools/arm_pose_pub.py zero --check-only
    python3 tools/arm_pose_pub.py zero --dry-run
    python3 tools/arm_pose_pub.py zero --rate 20 --seconds 5

    # 真机（必须显式表明你知情）
    python3 tools/arm_pose_pub.py zero \
        --topic-prefix hardware --i-know-its-real --rate 20 --seconds 5

    # 主动释放覆盖
    python3 tools/arm_pose_pub.py --release

安全设计（都会被强制执行，不是提示）
------------------------------------
  * 默认只发**仿真**话题。真机必须同时给 `--topic-prefix hardware`
    和 `--i-know-its-real`，少一个就拒绝执行。
  * 单关节角度超过 `meta.max_angle_rad`（默认 0.35 rad ≈ 20°）直接拒绝。
  * 发布频率低于 5 Hz 直接拒绝（官方覆盖超时 0.2 s，低于 5 Hz 必然断续）。
  * 位姿里没填的、标了未验证的、标了仅仿真的，各有独立拦截。
  * 从零位平滑爬升到目标（`--ramp-seconds`），不做突跳。
  * `--dry-run` 完全不依赖 ROS，任何机器上都能先审一遍要发什么。

退出码
------
    0  成功（或在 dry-run/无 rclpy 环境下成功打印了命令）
    2  参数或台账问题（位姿没填、长度不对、名字不存在）
    3  安全门禁拒绝（真机确认缺失、角度超限、频率过低、仅仿真位姿上真机）
    4  运行环境问题（rclpy 导入失败以外的错误、发布失败）
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, os.path.join(ROOT, "src"))

try:
    import miniyaml
except ImportError as exc:  # pragma: no cover
    print(f"找不到 src/miniyaml.py：{exc}", file=sys.stderr)
    sys.exit(4)

DEFAULT_POSES_FILE = os.path.join(ROOT, "config", "arm_poses.yaml")

EXIT_OK = 0
EXIT_INPUT = 2
EXIT_GUARD = 3
EXIT_RUNTIME = 4

C_RED = "\033[31m"
C_YEL = "\033[33m"
C_GRN = "\033[32m"
C_CYA = "\033[36m"
C_DIM = "\033[2m"
C_OFF = "\033[0m"


# ---------------------------------------------------------------- 输出helpers
class UI:
    def __init__(self, color: bool) -> None:
        self.color = color

    def _c(self, code: str, text: str) -> str:
        return f"{code}{text}{C_OFF}" if self.color else text

    def head(self, text: str) -> None:
        print()
        print(self._c(C_CYA, f"── {text} " + "─" * max(0, 60 - len(text))))

    def ok(self, text: str) -> None:
        print(f"  {self._c(C_GRN, '✔')} {text}")

    def warn(self, text: str) -> None:
        print(f"  {self._c(C_YEL, '!')} {text}")

    def bad(self, text: str) -> None:
        print(f"  {self._c(C_RED, '✘')} {text}")

    def info(self, text: str) -> None:
        print(f"  {self._c(C_DIM, '·')} {text}")


# ---------------------------------------------------------------- 台账读取
def load_poses(path: str, ui: UI) -> dict:
    if not os.path.exists(path):
        ui.bad(f"找不到位姿台账：{path}")
        sys.exit(EXIT_INPUT)
    try:
        data = miniyaml.load(path)
    except Exception as exc:
        ui.bad(f"解析 yaml 失败：{exc}")
        sys.exit(EXIT_INPUT)
    if not isinstance(data, dict):
        ui.bad("位姿台账顶层不是映射（mapping）")
        sys.exit(EXIT_INPUT)
    if "meta" not in data or "poses" not in data:
        ui.bad("位姿台账缺少 meta 或 poses 段")
        sys.exit(EXIT_INPUT)
    return data


def meta_limits(meta: dict) -> tuple[float, float]:
    try:
        max_angle = float(meta.get("max_angle_rad", 0.35))
    except (TypeError, ValueError):
        max_angle = 0.35
    try:
        min_rate = float(meta.get("min_publish_rate_hz", 5))
    except (TypeError, ValueError):
        min_rate = 5.0
    return max_angle, min_rate


def _as_float_list(value, joints: list[str], side: str) -> list[float] | None:
    """把 yaml 里的数组转成 float 列表并校验长度。返回 None 表示未填。"""
    if value is None:
        return None
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{side} 不是数组（实际 {type(value).__name__}）")
    if len(value) != len(joints):
        raise ValueError(f"{side} 有 {len(value)} 个值，但该臂有 {len(joints)} 个关节")
    out = []
    for i, v in enumerate(value):
        if v is None:
            raise ValueError(f"{side} 第 {i} 项（{joints[i]}）是空的")
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            raise ValueError(f"{side} 第 {i} 项（{joints[i]}）不是数字：{v!r}")
    return out


# ---------------------------------------------------------------- list
def cmd_list(data: dict, ui: UI) -> int:
    meta = data["meta"]
    max_angle, min_rate = meta_limits(meta)
    left = meta["arm_joint_names"]["left"]
    right = meta["arm_joint_names"]["right"]

    ui.head("台账元信息")
    ui.info(f"单位：{meta.get('unit', '?')}    消息：{meta.get('msg_type', '?')}")
    ui.info(f"单关节上限：{max_angle} rad（≈{math.degrees(max_angle):.1f}°）")
    ui.info(f"最低发布频率：{min_rate} Hz")
    ui.info(f"左臂 {len(left)} 关节：{', '.join(left)}")
    ui.info(f"右臂 {len(right)} 关节：{', '.join(right)}")

    ui.head("可用位姿")
    print(f"  {'名字':<14}{'左臂':<10}{'右臂':<10}{'已验证':<8}{'仅仿真':<7}说明")
    print(f"  {'-'*14}{'-'*10}{'-'*10}{'-'*8}{'-'*7}{'-'*30}")
    for name, pose in data["poses"].items():
        if not isinstance(pose, dict):
            continue
        try:
            lv = _as_float_list(pose.get("left"), left, "left")
        except ValueError:
            lv = None
        try:
            rv = _as_float_list(pose.get("right"), right, "right")
        except ValueError:
            rv = None
        lmark = "已填" if lv else "待填"
        rmark = "已填" if rv else "待填"
        ver = "是" if pose.get("verified") else "否"
        sim = "是" if pose.get("simulation_only") else "否"
        note = str(pose.get("note") or pose.get("label") or "").strip().splitlines()[0][:40]
        print(f"  {name:<14}{lmark:<10}{rmark:<10}{ver:<8}{sim:<7}{note}")
    print()
    ui.warn("未填的位姿必须先用官方 IK 脚本算出角度再填入，不要凭感觉填。")
    print()
    return EXIT_OK


# ---------------------------------------------------------------- 组装消息
def build_arrays(pose: dict, meta: dict, sides: list[str], ui: UI) -> tuple[list[str], list[float], list[float], list[float]]:
    names: list[str] = []
    pos: list[float] = []
    kp: list[float] = []
    kd: list[float] = []

    default_kp = meta.get("default_kp") or []
    default_kd = meta.get("default_kd") or []
    pose_kp = pose.get("kp")
    pose_kd = pose.get("kd")

    for side in sides:
        joints = list(meta["arm_joint_names"][side])
        # 台账写错了要给出人能看懂的报错，而不是抛栈
        try:
            vals = _as_float_list(pose.get(side), joints, side)
        except ValueError as exc:
            ui.bad(f"位姿「{pose.get('_name', '?')}」的 {side} 有问题：{exc}")
            sys.exit(EXIT_INPUT)
        if vals is None:
            ui.bad(f"位姿「{pose.get('_name', '?')}」的 {side} 还没填角度（当前是空的）。")
            ui.info("用官方 IK 脚本算出 7 个关节角后填进 config/arm_poses.yaml。")
            ui.info("参考 docs/13-抓取任务技术路线与里程碑.md §4.2")
            sys.exit(EXIT_INPUT)
        names.extend(joints)
        pos.extend(vals)

        this_kp = pose_kp if isinstance(pose_kp, list) else default_kp
        this_kd = pose_kd if isinstance(pose_kd, list) else default_kd
        if len(this_kp) != len(joints) or len(this_kd) != len(joints):
            ui.bad(f"{side} 的 kp/kd 数量与关节数不符（{len(this_kp)}/{len(this_kd)} vs {len(joints)}）")
            sys.exit(EXIT_INPUT)
        kp.extend(float(x) for x in this_kp)
        kd.extend(float(x) for x in this_kd)

    return names, pos, kp, kd


def resolve_sides(pose: dict, requested: str, meta: dict, ui: UI) -> list[str]:
    """决定这次要发哪几侧。

    台账里只填了左手（抓瓶子就是单手活）是很正常的情况。
    `--sides auto`（默认）会**只发填了的那些侧**，并把跳过的明确喊出来；
    显式写 `--sides left,right` 则保持严格：没填就报错。
    这样既不会逼你每次多敲一个参数，也不会让「以为双臂都在动」蒙混过关。
    """
    if requested != "auto":
        sides = [s.strip() for s in requested.split(",") if s.strip()]
        for s in sides:
            if s not in meta["arm_joint_names"]:
                ui.bad(f"未知的侧别：{s}（只支持 left / right）")
                sys.exit(EXIT_INPUT)
        return sides

    filled = [s for s in ("left", "right") if pose.get(s) is not None]
    skipped = [s for s in ("left", "right") if pose.get(s) is None]
    if not filled:
        ui.bad(f"位姿「{pose.get('_name', '?')}」左右两侧都还没填角度。")
        ui.info("用官方 IK 脚本算出 7 个关节角后填进 config/arm_poses.yaml。")
        sys.exit(EXIT_INPUT)
    ui.info(f"本次发布：{'、'.join(filled)}"
            + (f"（{'、'.join(skipped)} 台账里为空，自动跳过）" if skipped else ""))
    return filled


def fmt_arr(vals: list[float]) -> str:
    return "[" + ", ".join(f"{v:.6g}" for v in vals) + "]"


def fmt_names(names: list[str]) -> str:
    return "[" + ", ".join(f'"{n}"' for n in names) + "]"


def msg_text(names: list[str], pos: list[float], kp: list[float], kd: list[float]) -> str:
    return (
        "{\n"
        f"  actuators_name: {fmt_names(names)},\n"
        f"  pos: {fmt_arr(pos)},\n"
        f"  kp: {fmt_arr(kp)},\n"
        f"  kd: {fmt_arr(kd)}\n"
        "}"
    )


# ---------------------------------------------------------------- 安全检查
def run_guards(pose: dict, meta: dict, names: list[str], pos: list[float],
               rate: float, args, ui: UI) -> int:
    """返回 EXIT_OK 或 EXIT_GUARD。

    这里每一条都是「拦截」而不是「提醒」：宁可让你多敲一个参数，
    也别让一根关节在错误的增益和角度下动起来。
    """
    max_angle, min_rate = meta_limits(meta)
    limit = args.max_angle if args.max_angle is not None else max_angle
    problems = 0

    ui.head("安全检查")

    # G1 真机确认
    if args.topic_prefix == "hardware":
        if args.i_know_its_real:
            ui.warn("目标是【真机】话题。确认已按官方要求做完前置：")
            ui.info("  落地承重 → 先进入 com.bxi.basic_actions/hello 或 applause（不依赖手臂维持平衡）")
            ui.info("  吊装     → 底层状态只能停 initial_pos 或 pd_brake")
            ui.info("  两种情况下都要：清空手臂运动范围、急停可达、人站侧面")
        else:
            ui.bad("要发真机必须同时给 --i-know-its-real，否则本工具拒绝执行。")
            problems += 1
    else:
        ui.ok("目标是仿真话题（不会让真机动）")

    # G2 仅仿真位姿上真机
    if pose.get("simulation_only") and args.topic_prefix == "hardware":
        if args.force:
            ui.warn("该位姿标了 simulation_only，但 --force 已给，放行（后果自负）")
        else:
            ui.bad(f"位姿标了 simulation_only=true，不允许发真机。确认过再加 --force。")
            problems += 1
    elif pose.get("simulation_only"):
        ui.ok("位姿标注「仅仿真」，与当前仿真话题一致")

    # G3 未验证
    if not pose.get("verified"):
        if args.allow_unverified or args.topic_prefix == "simulation":
            ui.warn("该位姿尚未在真机上验证过（verified: false）")
        else:
            ui.bad("位姿未验证，真机发布要显式加 --allow-unverified")
            problems += 1
    else:
        ui.ok("位姿标注为已验证")

    # G4 角度限幅
    over = [(names[i], pos[i]) for i in range(len(pos)) if abs(pos[i]) > limit]
    if over:
        for n, v in over:
            ui.bad(f"{n} = {v:.4f} rad（≈{math.degrees(v):.1f}°）超过上限 {limit} rad")
        if args.force:
            ui.warn("--force 已给，越过角度限幅（后果自负）")
        else:
            ui.bad("拒绝发布。确实需要更大行程就用 --max-angle 显式提高，或加 --force。")
            ui.info("第一次调试应该比上限更小，而不是更大。")
            problems += 1
    else:
        worst = max((abs(v) for v in pos), default=0.0)
        ui.ok(f"角度全部在上限内（最大 {worst:.4f} rad ≈ {math.degrees(worst):.1f}°）")

    # G5 频率
    if rate < min_rate:
        ui.bad(f"发布频率 {rate} Hz 低于最低 {min_rate} Hz；官方覆盖超时 0.2 s，断流就会掉覆盖。")
        problems += 1
    else:
        ui.ok(f"发布频率 {rate} Hz（≥ {min_rate} Hz，不会掉覆盖）")

    # G6 长度一致性（关节名与各数组必须一一对应）
    if len(names) != len(pos):
        ui.bad(f"关节名 {len(names)} 个，但位置值 {len(pos)} 个")
        problems += 1
    else:
        ui.ok(f"关节名与数组长度一致（{len(names)} 项）")

    print()
    return EXIT_GUARD if problems else EXIT_OK


# ---------------------------------------------------------------- 发布
def ramp_traj(start: list[float], target: list[float], t: float, ramp_s: float) -> list[float]:
    if ramp_s <= 0:
        return list(target)
    ratio = min(1.0, t / ramp_s)
    return [s + (g - s) * ratio for s, g in zip(start, target)]


def publish(args, names: list[str], pos: list[float], kp: list[float], kd: list[float],
            ui: UI) -> int:
    topic = f"/{args.topic_prefix}/actuators_cmds_override"
    ramp_s = args.ramp_seconds
    start = list(pos) if args.no_ramp else [0.0] * len(pos)

    try:
        import rclpy
        from rclpy.node import Node
        from communication.msg import ActuatorCmds
    except ImportError as exc:
        ui.head("当前环境没有 ROS 2 / communication 包")
        ui.info(f"原因：{exc}")
        ui.info("下面是等价的手工命令，请在机器人上（root shell、source 过 bxi_ros2_pkg）执行：")
        print()
        print(f"  ros2 topic pub -r {args.rate:g} {topic} communication/msg/ActuatorCmds \\")
        for line in msg_text(names, pos, kp, kd).splitlines():
            print(f"    {line}")
        print()
        ui.warn("手工命令不会做角度爬升，是直接跳变。第一次务必先 --release 再小幅度试。")
        return EXIT_OK

    rclpy.init()
    node = Node("arm_pose_pub")
    pub = node.create_publisher(ActuatorCmds, topic, 10)
    period = 1.0 / args.rate
    sent = 0
    t0 = time.time()

    ui.head("开始发布")
    ui.info(f"话题：{topic}    频率：{args.rate} Hz    "
            f"时长：{'直到 Ctrl+C' if args.seconds <= 0 else f'{args.seconds}s'}")
    ui.info(f"爬升：{ramp_s}s（从零位）" if not args.no_ramp else "爬升：关闭（直接跳变）")
    if args.topic_prefix == "hardware":
        ui.warn("真机发布中 —— 保持终端别断，Ctrl+C 会立刻停止发布")
    print()

    try:
        last_report = -1.0
        while rclpy.ok():
            loop_start = time.time()
            now = loop_start - t0
            if args.seconds > 0 and now > args.seconds:
                break
            msg = ActuatorCmds()
            msg.actuators_name = list(names)
            msg.pos = [float(x) for x in ramp_traj(start, pos, now, ramp_s)]
            msg.kp = [float(x) for x in kp]
            msg.kd = [float(x) for x in kd]
            pub.publish(msg)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0.0)

            if now - last_report >= 1.0:
                last_report = now
                ratio = min(1.0, now / ramp_s) * 100 if (ramp_s > 0 and not args.no_ramp) else 100.0
                print(f"  [{now:6.1f}s] 已发 {sent:6d} 条   爬升 {ratio:3.0f}%")

            # 补足周期，保证平均频率接近 --rate，避免发布间隔抖动导致掉覆盖
            spent = time.time() - loop_start
            if spent < period:
                time.sleep(period - spent)

    except KeyboardInterrupt:
        print()
        ui.warn("收到 Ctrl+C，停止发布。")
        ui.info(f"覆盖将在 {ACTUATOR_OVERRIDE_TIMEOUT_S} s 内自动超时并混合回当前状态。")
        ui.info("想立即释放可以再跑一次：--release")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    elapsed = time.time() - t0
    ui.ok(f"共发布 {sent} 条 / 用时 {elapsed:.1f}s / 平均 {sent / max(elapsed, 1e-6):.1f} Hz")
    return EXIT_OK


ACTUATOR_OVERRIDE_TIMEOUT_S = 0.2


def publish_release(args, ui: UI) -> int:
    topic = f"/{args.topic_prefix}/actuators_cmds_override"
    ui.head("释放覆盖")
    if args.topic_prefix == "hardware" and not args.i_know_its_real:
        ui.bad("要操作真机话题必须同时给 --i-know-its-real")
        return EXIT_GUARD
    try:
        import rclpy
        from rclpy.node import Node
        from communication.msg import ActuatorCmds
    except ImportError as exc:
        ui.info(f"当前环境没有 ROS 2（{exc}）")
        ui.info("请在机器人上执行：")
        print()
        print(f"  ros2 topic pub --once {topic} communication/msg/ActuatorCmds '{{}}'")
        print()
        return EXIT_OK

    rclpy.init()
    node = Node("arm_pose_release")
    pub = node.create_publisher(ActuatorCmds, topic, 10)
    pub.publish(ActuatorCmds())
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    ui.ok(f"已向 {topic} 发送空消息，覆盖已释放")
    return EXIT_OK


# ---------------------------------------------------------------- main
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="arm_pose_pub.py",
        description="按名字把手臂位姿持续发布到 ELF3 的 ActuatorCmds 覆盖话题",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""示例：
  python3 tools/arm_pose_pub.py --list
  python3 tools/arm_pose_pub.py zero --check-only
  python3 tools/arm_pose_pub.py zero --dry-run
  python3 tools/arm_pose_pub.py zero --rate 20 --seconds 5

真机（必须显式表明知情）：
  python3 tools/arm_pose_pub.py zero --topic-prefix hardware --i-know-its-real \\
      --rate 20 --seconds 5
""",
    )
    p.add_argument("pose", nargs="?", help="位姿名（用 --list 查看）")
    p.add_argument("--poses-file", default=DEFAULT_POSES_FILE, help="位姿台账路径")
    p.add_argument("--sides", default="auto",
                   help="发哪几侧：auto（默认，只发台账里填了的）/ left / right / left,right")
    p.add_argument("--topic-prefix", default="simulation", choices=["simulation", "hardware"],
                   help="话题前缀：仿真 simulation（默认）/ 真机 hardware")
    p.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz（默认 20，最低 5）")
    p.add_argument("--seconds", type=float, default=5.0,
                   help="发布时长秒；<=0 表示直到 Ctrl+C（默认 5）")
    p.add_argument("--ramp-seconds", type=float, default=1.5,
                   help="从零位爬到目标的时长秒（默认 1.5，0 表示不爬升）")
    p.add_argument("--no-ramp", action="store_true", help="直接从零位跳到目标（不建议）")
    p.add_argument("--max-angle", type=float, default=None,
                   help="临时覆盖单关节角度上限（弧度）；不推荐")
    p.add_argument("--release", action="store_true", help="发送空消息释放覆盖后退出")
    p.add_argument("--i-know-its-real", dest="i_know_its_real", action="store_true",
                   help="确认目标是真机话题（发 hardware 时必填）")
    p.add_argument("--allow-unverified", action="store_true",
                   help="允许发布未被验证过的位姿")
    p.add_argument("--force", action="store_true", help="越过 simulation_only / 限幅门禁")
    # 模式
    p.add_argument("--list", action="store_true", help="列出台账里所有位姿")
    p.add_argument("--check-only", action="store_true", help="只做安全检查，不发布")
    p.add_argument("--dry-run", action="store_true",
                   help="打印将要发布的内容与手工命令，不连接 ROS")
    p.add_argument("--no-color", action="store_true", help="禁用彩色输出")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    ui = UI(color=not args.no_color and sys.stdout.isatty())
    if args.no_color:
        ui.color = False

    data = load_poses(os.path.abspath(args.poses_file), ui)
    meta = data["meta"]

    if args.list:
        return cmd_list(data, ui)

    if args.release:
        return publish_release(args, ui)

    if not args.pose:
        ui.bad("没给位姿名。先跑 --list 看有哪些。")
        return EXIT_INPUT

    poses = data["poses"]
    if args.pose not in poses:
        ui.bad(f"台账里没有位姿「{args.pose}」。")
        ui.info("可用的：" + ", ".join(poses.keys()))
        return EXIT_INPUT
    pose = poses[args.pose]
    if not isinstance(pose, dict):
        ui.bad(f"位姿「{args.pose}」格式不对")
        return EXIT_INPUT
    pose = dict(pose)
    pose["_name"] = args.pose

    sides = resolve_sides(pose, args.sides, meta, ui)

    if args.rate < 5 and not args.force:
        ui.bad(f"--rate {args.rate} 低于 5 Hz，官方覆盖超时 0.2 s，必然断续。")
        return EXIT_GUARD

    ui.head(f"位姿：{args.pose}")
    if pose.get("label"):
        ui.info(str(pose["label"]))
    if pose.get("note"):
        for line in str(pose["note"]).strip().splitlines():
            ui.info(line.strip())

    names, pos, kp, kd = build_arrays(pose, meta, sides, ui)

    if args.no_ramp:
        args.ramp_seconds = 0.0

    rc = run_guards(pose, meta, names, pos, args.rate, args, ui)
    if rc != EXIT_OK:
        ui.bad("安全检查未通过，未发布。")
        return rc

    ui.head("将发布的消息")
    print(msg_text(names, pos, kp, kd))
    print()

    if args.check_only:
        ui.ok("仅检查模式：一切通过，未发布。")
        return EXIT_OK

    if args.dry_run:
        topic = f"/{args.topic_prefix}/actuators_cmds_override"
        ui.ok("演练模式：未连接 ROS。手工等价命令：")
        print()
        print(f"  ros2 topic pub -r {args.rate:g} {topic} communication/msg/ActuatorCmds \\")
        for line in msg_text(names, pos, kp, kd).splitlines():
            print(f"    {line}")
        print()
        return EXIT_OK

    return publish(args, names, pos, kp, kd, ui)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Revo2 灵巧手 ROS 2 bridge 节点（跑在精灵 3 的 NUC 上）

角色：**driver / bridge**
    - 占住机器人主控板的 CAN 链路
    - 把上位机发来的高层指令（目标关节角 / 手势名）翻译成 SDK 调用
    - 把实际位置、电流等反馈发布出去

设计意图：把「底层总线操作」和「上层控制逻辑」分开。
    电脑上的 commander 只发语义清晰的指令，不需要知道 CAN 细节；
    换机器人、换总线、换 ID 时，只改这一处。

话题接口（全部使用标准消息类型，避免引入自定义 msg）::

    订阅  /revo2/<hand>/command    std_msgs/Float64MultiArray
          data = [角度1..角度6, duration_ms]   单位：度 / 毫秒
    订阅  /revo2/<hand>/gesture    std_msgs/String        手势名，见 config/gestures.yaml
    发布  /revo2/<hand>/state      std_msgs/Float64MultiArray
          data = [位置1..6（度）, 电流1..6, 时间戳]
    发布  /revo2/<hand>/status     std_msgs/String        人类可读状态

<hand> = left 或 right

本项目 ELF3 实机装配的是**右手**（CAN6 / ID 127），
故 `--hand` 默认值为 `right`，bus/id 也会自动推导为 6 / 127。

用法（在机器人上，先按 docs/03 设好 ROS 环境）::

    # 先检查 SDK 接口是否可用（强烈建议第一步就跑这个）
    python3 revo2_bridge_node.py --check-api

    # 启动右手 bridge（本项目默认，bus/id 自动推导为 6 / 127）
    python3 revo2_bridge_node.py --hand right

    # 显式写全参数（等效，bus 与 id 也支持手工覆盖）
    python3 revo2_bridge_node.py --hand right --bus 6 --id 127 --master-id 1

    # 启动左手 bridge（备选）
    python3 revo2_bridge_node.py --hand left --bus 5 --id 126 --master-id 1

    # 无硬件时用模拟器测试上位机侧逻辑
    python3 revo2_bridge_node.py --hand right --sim

⚠️ SDK 兼容性说明
    本脚本按 bc-stark-sdk v2.x 的文档 API 编写，并做了符号名兜底。
    不同 SDK 版本的方法名可能不同。**首次使用请务必先跑 --check-api**，
    它会列出当前环境里实际可用的符号，你据此调整下面的 SDK Adapter 区即可。
    适配逻辑集中在「SDK Adapter」一节，是唯一需要随版本改动的地方。
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import sys
import time
from typing import Any

# ── ROS 2 ───────────────────────────────────────────────────
# 允许在没有 rclpy 的环境里执行 --check-api（只查 SDK 接口），
# 所以这里不直接退出，而是记录状态，在 main() 里按需报错。
ROS_AVAILABLE = True
ROS_IMPORT_ERROR: Exception | None = None
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String
except ImportError as _exc:  # pragma: no cover
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = _exc

    class Node:  # type: ignore[no-redef]
        """rclpy 缺失时的占位类，只为让下面的类定义能通过。"""

        def __init__(self, *a, **kw):
            raise RuntimeError("rclpy 不可用")

    Float64MultiArray = String = object  # type: ignore[assignment,misc]

    def _ros_missing_hint() -> None:
        print("[错误] 找不到 rclpy。请先加载 ROS 2 环境：")
        print("        source /opt/ros/humble/setup.bash")
        print("        source /opt/bxi/bxi_ros2_pkg/setup.bash")
        print(f"        原始错误：{ROS_IMPORT_ERROR}")

else:
    def _ros_missing_hint() -> None:  # pragma: no cover
        pass

MOTOR_COUNT = 6
# ⚠️ MOTOR_ORDER / ANGLE_LIMITS / DURATION_* 均为**兜底默认值**。
#    正常启动时会被 config/revo2_hardware.yaml 覆盖（见 _load_hardware_config）。
#    调整硬件参数请改配置文件，不必改这里——此处仅保证配置缺失时仍能运行。
MOTOR_ORDER = ["拇指Flex", "拇指Aux", "食指", "中指", "无名指", "小拇指"]

# 安全角度上限（度）—— 取「本仓库合并模型」与「官方驱动文档」两者的**交集**，
# 任何一个都不超过，这样仿真与真机都不会越界。
#
#   电机      对应关节（官方驱动）              合并模型 ctrlrange   官方驱动 README   取交集
#   拇指Flex  right_thumb_proximal_joint        0~59.0°              0~60°             59
#   拇指Aux   right_thumb_metacarpal_joint      0~90.0°              0~89°             89   ← 原为 90，超官方 1°
#   食指      right_index_proximal_joint        0~80.8° (1.41 rad)   0~81°             80.8
#   中指      right_middle_proximal_joint       同上                 同上              80.8
#   无名指    right_ring_proximal_joint         同上                 同上              80.8
#   小拇指    right_pinky_proximal_joint        同上                 同上              80.8
#
# 出处：合并模型 elf3_revo2_right.xml 的 <position ctrlrange=...>；
#       官方驱动 brainco_hand_driver/README_CN.md「关节映射」表。
ANGLE_LIMITS = [59.0, 89.0, 80.8, 80.8, 80.8, 80.8]
DURATION_MIN, DURATION_MAX = 1, 2000

# 硬件常量配置文件（相对本文件定位，找不到则回落到上面的默认值）
_HW_CONFIG_CANDIDATES = (
    os.path.join("..", "..", "config", "revo2_hardware.yaml"),
    os.path.join("config", "revo2_hardware.yaml"),
)


def _load_hardware_config(path: str = "") -> dict:
    """读取硬件常量配置（关节上限、电机顺序、时长范围、总线与 ID）。

    与 _load_gestures 同样走 miniyaml，保持零依赖。
    任何一步失败都回落到模块级默认值，不影响节点启动——
    配置只用于覆盖，不作为启动前提。
    """
    cands = [path] if path else []
    here = os.path.dirname(os.path.abspath(__file__))
    cands += [os.path.join(here, c) for c in _HW_CONFIG_CANDIDATES]
    target = next((os.path.abspath(c) for c in cands
                   if c and os.path.exists(c)), "")
    if not target:
        print("[提示] 未找到 revo2_hardware.yaml，使用内置默认硬件常量")
        return {}
    try:
        sys.path.insert(0, os.path.join(here, ".."))
        from miniyaml import load as _load  # type: ignore

        data = _load(target) or {}
        print(f"[配置] 硬件常量已加载：{target}")
        return data
    except Exception as exc:  # noqa: BLE001
        print(f"[警告] 读取硬件常量失败（{exc}），使用内置默认值")
        return {}


# ══════════════════════════════════════════════════════════════
# SDK Adapter —— 唯一需要随 SDK 版本调整的地方
# ══════════════════════════════════════════════════════════════
class SDKAdapter:
    """把不同版本的强脑 SDK 收敛成统一接口。

    需要三个能力：
        1. 初始化设备（总线号、设备 ID、master_id、CANFD）
        2. 下发目标关节角
        3. 读回实际位置 / 电流

    用候选名列表 + 反射查找的方式做兜底，找不到时给出清晰提示，
    而不是抛一个让人看不懂的 AttributeError。
    """

    # 候选模块名（不同发行版 / 版本命名不一致）
    MODULE_CANDIDATES = [
        "bc_stark_sdk", "bc_stark_sdk_v2", "libstark", "stark",
        "bc_stark_sdk.main_mod", "bc_stark_sdk.mod",
    ]

    # 候选初始化函数名
    INIT_CANDIDATES = [
        "init_bxipci_device", "init_bxi_pci_device",
        "init_device", "init_hand", "open_device",
    ]

    def __init__(self, bus: int, device_id: int, master_id: int = 1,
                 is_canfd: bool = True, hw_type: str | None = None,
                 sim: bool = False):
        self.bus = bus
        self.device_id = device_id
        self.master_id = master_id
        self.is_canfd = is_canfd
        self.hw_type = hw_type
        self.sim = sim

        self.mod: Any = None
        self.ctx: Any = None
        self._init_fn: Any = None

        if not sim:
            self._load()

    # ── 加载与自检 ──────────────────────────────────
    def _load(self) -> None:
        for name in self.MODULE_CANDIDATES:
            try:
                self.mod = importlib.import_module(name)
                print(f"[SDK] 已加载模块：{name}")
                break
            except Exception:
                continue
        if self.mod is None:
            raise RuntimeError(
                "没有找到强脑 SDK 模块。请确认已安装：\n"
                "    pip install bc-stark-sdk==1.5.1 --index-url https://pypi.org/simple/\n"
                "注意：官方指定 1.5.1，且要装在系统 Python（不要用 conda）。"
            )

        for cand in self.INIT_CANDIDATES:
            fn = getattr(self.mod, cand, None)
            if callable(fn):
                self._init_fn = fn
                print(f"[SDK] 使用初始化函数：{cand}")
                break
        if self._init_fn is None:
            raise RuntimeError(
                "SDK 里没找到初始化函数。请运行 --check-api 查看实际可用的符号，\n"
                "然后修改本文件的 INIT_CANDIDATES 列表。"
            )

    @classmethod
    def check_api(cls) -> int:
        """打印当前环境里 SDK 的实际接口，便于对照调整。"""
        print("=" * 62)
        print("  SDK 接口自检")
        print("=" * 62)

        found_mod = None
        for name in cls.MODULE_CANDIDATES:
            try:
                mod = importlib.import_module(name)
                print(f"\n[模块] {name}  → {getattr(mod, '__file__', '?')}")
                found_mod = mod
                break
            except Exception:
                continue

        if found_mod is None:
            print("\n没有找到强脑 SDK 模块。")
            print("请安装：pip install bc-stark-sdk==1.5.1 "
                  "--index-url https://pypi.org/simple/")
            return 1

        public = sorted(n for n in dir(found_mod) if not n.startswith("_"))
        print(f"\n[公开符号] 共 {len(public)} 个：")
        for n in public:
            obj = getattr(found_mod, n, None)
            kind = "class" if isinstance(obj, type) else (
                "callable" if callable(obj) else type(obj).__name__)
            print(f"    {n:<40} {kind}")

        print("\n[初始化函数候选]")
        for cand in cls.INIT_CANDIDATES:
            mark = "✓ 存在" if callable(getattr(found_mod, cand, None)) else "✗ 无"
            print(f"    {mark}  {cand}")

        # 常见关键字搜索
        print("\n[按关键字搜索]")
        for kw in ("init", "device", "position", "angle", "motor",
                   "status", "current", "touch"):
            hits = [n for n in public if kw in n.lower()]
            if hits:
                print(f"    *{kw}*  →  {', '.join(hits[:8])}"
                      + (" ..." if len(hits) > 8 else ""))

        print("\n参考：官方 SDK 文档")
        print("    https://www.brainco-hz.com/docs/revolimb-hand/revo2/python_sdk.html")
        print("\n看完把实际函数名填进本文件的 INIT_CANDIDATES，")
        print("并在 connect() / set_angles() / read_state() 里对接即可。")
        return 0

    # ── 连接 ────────────────────────────────────────
    def connect(self) -> None:
        if self.sim:
            print("[模拟] 跳过真实连接")
            return

        kwargs: dict[str, Any] = {"master_id": self.master_id,
                                  "is_canfd": self.is_canfd}
        if self.hw_type:
            hw_enum = None
            for holder in (self.mod, getattr(self.mod, "libstark", None)):
                if holder is None:
                    continue
                for attr in ("StarkHardwareType", "HardwareType"):
                    enum_cls = getattr(holder, attr, None)
                    if enum_cls is not None and hasattr(enum_cls, self.hw_type):
                        hw_enum = getattr(enum_cls, self.hw_type)
                        break
                if hw_enum is not None:
                    break
            if hw_enum is not None:
                kwargs["hw_type"] = hw_enum
                print(f"[SDK] hw_type = {self.hw_type}")
            else:
                print(f"[SDK] 警告：没找到硬件类型枚举 {self.hw_type}，"
                      f"按默认值连接。请用 --check-api 核对正确的枚举名。")

        print(f"[SDK] 连接：bus={self.bus} id={self.device_id} "
              f"master_id={self.master_id} canfd={self.is_canfd}")
        self.ctx = self._init_fn(self.bus, self.device_id, **kwargs)
        print("[SDK] 连接成功")

    # ── 控制 / 反馈 ──────────────────────────────────
    def set_angles(self, angles: list[float], duration_ms: int) -> None:
        """下发 6 个目标关节角（度）。"""
        if self.sim:
            print(f"[模拟] set_angles({[round(a, 1) for a in angles]}, {duration_ms}ms)")
            return
        if self.ctx is None:
            raise RuntimeError("设备未连接")

        # 候选方法名（按可能性排序）
        for name in ("set_finger_positions", "set_positions", "set_angles",
                     "set_position", "move_fingers", "set_joint_positions"):
            fn = getattr(self.ctx, name, None) or getattr(self.mod, name, None)
            if callable(fn):
                try:
                    fn(self.ctx, angles, duration_ms)
                except TypeError:
                    fn(angles, duration_ms)
                return
        raise RuntimeError(
            "找不到下发关节角的方法。请运行 --check-api 查看可用方法名，"
            "然后修改 SDKAdapter.set_angles()。"
        )

    def read_state(self) -> tuple[list[float], list[float]]:
        """读回 (位置[度], 电流)。"""
        if self.sim:
            t = time.time() % 6.0
            pos = [abs(math.sin(t + i)) * 40 for i in range(MOTOR_COUNT)]
            cur = [100 + i * 5 for i in range(MOTOR_COUNT)]
            return pos, cur

        if self.ctx is None:
            raise RuntimeError("设备未连接")

        pos: list[float] = [0.0] * MOTOR_COUNT
        cur: list[float] = [0.0] * MOTOR_COUNT

        for name in ("get_motor_status", "get_status", "read_status",
                     "get_state", "read_state"):
            fn = getattr(self.ctx, name, None) or getattr(self.mod, name, None)
            if not callable(fn):
                continue
            try:
                st = fn(self.ctx)
            except TypeError:
                st = fn()
            pos, cur = _parse_status(st)
            break
        return pos, cur

    def close(self) -> None:
        if self.sim or self.ctx is None:
            return
        for name in ("close", "disconnect", "release"):
            fn = getattr(self.ctx, name, None)
            if callable(fn):
                try:
                    fn()
                except Exception:
                    pass
                return


def _parse_status(st: Any) -> tuple[list[float], list[float]]:
    """把 SDK 返回的状态对象尽力解析成 (位置, 电流)。

    不同版本返回结构不同（dataclass / dict / list），这里做容错解析，
    解析不出来就返回全 0 并在日志里提示。
    """
    pos = [0.0] * MOTOR_COUNT
    cur = [0.0] * MOTOR_COUNT

    def pick(obj: Any, keys: tuple[str, ...]) -> list[float] | None:
        for k in keys:
            if isinstance(obj, dict) and k in obj:
                v = obj[k]
                if isinstance(v, (list, tuple)) and len(v) >= MOTOR_COUNT:
                    return [float(x) for x in v[:MOTOR_COUNT]]
            v = getattr(obj, k, None)
            if isinstance(v, (list, tuple)) and len(v) >= MOTOR_COUNT:
                return [float(x) for x in v[:MOTOR_COUNT]]
        return None

    p = pick(st, ("positions", "position", "pos", "actual_positions"))
    c = pick(st, ("currents", "current", "cur", "actual_currents"))
    if p:
        pos = p
    if c:
        cur = c
    return pos, cur


# ══════════════════════════════════════════════════════════════
# ROS 2 节点
# ══════════════════════════════════════════════════════════════
class Revo2Bridge(Node):
    def __init__(self, argv: argparse.Namespace):
        super().__init__(f"revo2_bridge_{argv.hand}")
        self.argv = argv
        self.hand = argv.hand

        self.adapter = SDKAdapter(
            bus=argv.bus, device_id=argv.id, master_id=argv.master_id,
            is_canfd=True, hw_type=argv.hw_type, sim=argv.sim,
        )
        self.adapter.connect()

        self.pub_state = self.create_publisher(
            Float64MultiArray, f"/revo2/{self.hand}/state", 10)
        self.pub_status = self.create_publisher(
            String, f"/revo2/{self.hand}/status", 10)

        self.create_subscription(
            Float64MultiArray, f"/revo2/{self.hand}/command",
            self.on_command, 10)
        self.create_subscription(
            String, f"/revo2/{self.hand}/gesture",
            self.on_gesture, 10)

        self.timer = self.create_timer(1.0 / argv.rate, self.publish_state)

        self._gestures = _load_gestures(argv.gestures)

        # 硬件常量：优先取配置文件，缺失项回落到模块级默认值
        hw = _load_hardware_config(getattr(argv, "hardware_config", "") or "")
        self.motor_order = list(hw.get("motor_order") or MOTOR_ORDER)
        self.angle_limits = [float(x) for x in (hw.get("angle_limits") or ANGLE_LIMITS)]
        _dr = hw.get("duration_range") or [DURATION_MIN, DURATION_MAX]
        self.duration_range = (int(_dr[0]), int(_dr[1]))
        if (len(self.motor_order) != MOTOR_COUNT
                or len(self.angle_limits) != MOTOR_COUNT):
            self.get_logger().warn(
                f"硬件配置长度异常（motor_order={len(self.motor_order)}、"
                f"angle_limits={len(self.angle_limits)}），已回落默认值")
            self.motor_order = list(MOTOR_ORDER)
            self.angle_limits = list(ANGLE_LIMITS)

        self.publish_status(
            f"bridge 就绪 hand={self.hand} bus={self.bus} "
            f"id={self.id} sim={self.sim}")

        self.get_logger().info(
            f"bridge 启动：hand={self.hand} bus={self.bus} / device_id={self.id}"
            f"{' （模拟模式）' if self.sim else ''}")
        self.get_logger().info(
            f"订阅 /revo2/{self.hand}/command 与 /revo2/{self.hand}/gesture；"
            f"发布 /revo2/{self.hand}/state")

    # ── 发布 ────────────────────────────────────────
    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(text)

    def publish_state(self) -> None:
        try:
            pos, cur = self.adapter.read_state()
        except Exception as exc:
            self.get_logger().warn(f"读状态失败：{exc}")
            return
        msg = Float64MultiArray()
        msg.data = [float(x) for x in pos] + [float(x) for x in cur] + [time.time()]
        self.pub_state.publish(msg)

    # ── 安全校验（command 与 gesture 共用同一条路径）──
    def validate_target(self, angles: list[float], duration: int,
                        source: str) -> str:
        """校验目标角度与时长。通过返回空串，否则返回拒绝原因。

        单点实现：on_command 与 on_gesture 都必须经过此处，
        避免任一路径漏检。角度上限与时长范围取自硬件常量
        （config/revo2_hardware.yaml，缺失时用内置默认）。
        """
        if len(angles) != MOTOR_COUNT:
            return (f"{source}：角度数量应为 {MOTOR_COUNT} 个，"
                    f"实际收到 {len(angles)} 个")
        for i, (a, lim) in enumerate(zip(angles, self.angle_limits)):
            if not (0 <= a <= lim):
                name = self.motor_order[i] if i < len(self.motor_order) else f"#{i}"
                return f"{source}：{name} 的角度 {a}° 超出安全范围 0–{lim}°"
        lo, hi = self.duration_range
        if not (lo <= duration <= hi):
            return f"{source}：duration_ms={duration} 超出范围 {lo}–{hi}"
        return ""

    # ── 订阅回调 ────────────────────────────────────
    def on_command(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < MOTOR_COUNT + 1:
            self.publish_status(
                f"指令长度不足：需要 {MOTOR_COUNT + 1} 个值"
                f"（6 个角度 + 1 个时长），收到 {len(data)} 个")
            return

        angles = data[:MOTOR_COUNT]
        duration = int(data[MOTOR_COUNT])

        reason = self.validate_target(angles, duration, "指令")
        if reason:
            self.publish_status(f"拒绝执行：{reason}")
            return

        try:
            self.adapter.set_angles(angles, duration)
            self.publish_status(
                f"已下发 {[round(a, 1) for a in angles]} / {duration}ms")
        except Exception as exc:
            self.publish_status(f"下发失败：{exc}")

    def on_gesture(self, msg: String) -> None:
        name = msg.data.strip()
        g = self._gestures.get(name)
        if not g:
            avail = ", ".join(sorted(self._gestures)) or "（手势库为空）"
            self.publish_status(f"未知手势「{name}」。可用：{avail}")
            return
        try:
            angles = [float(a) for a in (g.get("angles") or [])]
            duration = int(g.get("duration_ms") or 1000)
        except (TypeError, ValueError) as exc:
            # 手势来自 yaml，值是任意对象；非数值在此拦下，
            # 否则会在下发阶段抛异常，还要麻烦调用方自己去猜原因
            self.publish_status(
                f"拒绝执行：手势「{name}」的角度或时长不是合法数值（{exc}）")
            return

        # ★ 手势路径与 command 路径走同一套安全校验。
        #   此前该路径仅检查角度数量、未校验范围，已补齐。
        reason = self.validate_target(angles, duration, f"手势「{name}」")
        if reason:
            self.publish_status(f"拒绝执行：{reason}")
            return

        try:
            self.adapter.set_angles(angles, duration)
            self.publish_status(
                f"执行手势「{name}」：{angles} / {duration}ms")
        except Exception as exc:
            self.publish_status(f"手势「{name}」执行失败：{exc}")

    def destroy_node(self) -> bool:
        try:
            self.adapter.close()
        except Exception:
            pass
        return super().destroy_node()


def _load_gestures(path: str) -> dict[str, dict]:
    """从 gestures.yaml 读手势库。

    优先用 pyyaml；没装则回落到仓库内置的零依赖解析器 src/miniyaml.py，
    这样在干净环境里也能直接跑。
    """
    if not path or not os.path.exists(path):
        print(f"[警告] 手势文件不存在：{path}")
        return {}
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, os.path.join(here, ".."))
        from miniyaml import load as _load  # type: ignore

        data = _load(path) or {}
        return data.get("gestures") or {}
    except ImportError:
        print("[警告] 找不到 miniyaml.py，手势话题不可用")
        return {}
    except Exception as exc:
        print(f"[警告] 读取手势文件失败：{exc}")
        return {}


# ══════════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Revo2 灵巧手 ROS 2 bridge 节点（跑在机器人上）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--hand", choices=["left", "right"], required=True,
                    help="控制哪只手（本项目实机为 right，即右手）")
    ap.add_argument("--bus", type=int, default=None,
                    help="总线号：右手 6（CAN6），左手 5（CAN5）；不给则按 --hand 推导")
    ap.add_argument("--id", type=int, default=None,
                    help="设备 ID：右手 127，左手 126；不给则按 --hand 推导")
    ap.add_argument("--master-id", type=int, default=1, help="master_id，默认 1")
    ap.add_argument("--hw-type", default=None,
                    help="硬件类型枚举名，例如 Revo2Basic / Revo2Pro / Revo2Touch。"
                         "不确定就用 --check-api 查")
    ap.add_argument("--rate", type=float, default=10.0, help="反馈发布频率 Hz，默认 10")
    ap.add_argument("--gestures", default="", help="手势库 yaml 路径")
    ap.add_argument("--hardware-config", default="",
                    help="硬件常量 yaml 路径（关节上限、电机顺序、时长范围、总线与 ID）。"
                         "默认自动查找 config/revo2_hardware.yaml；"
                         "找不到时使用脚本内置默认值，不影响启动")
    ap.add_argument("--sim", action="store_true",
                    help="模拟模式：不连硬件，用于验证上位机侧逻辑")
    ap.add_argument("--check-api", action="store_true",
                    help="只打印当前 SDK 的实际接口后退出")
    argv = ap.parse_args()

    if argv.check_api:
        return SDKAdapter.check_api()

    if not ROS_AVAILABLE:
        _ros_missing_hint()
        return 2

    # 默认值按左右手推导
    if argv.bus is None:
        argv.bus = 5 if argv.hand == "left" else 6
    if argv.id is None:
        argv.id = 126 if argv.hand == "left" else 127

    if not argv.gestures:
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (
            os.path.join(here, "..", "..", "config", "gestures.yaml"),
            os.path.join(here, "config", "gestures.yaml"),
        ):
            if os.path.exists(cand):
                argv.gestures = os.path.abspath(cand)
                break

    rclpy.init()
    node = None
    try:
        node = Revo2Bridge(argv)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到中断，正在关闭...")
    except Exception as exc:
        print(f"[错误] {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

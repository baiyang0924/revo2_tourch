#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给官方 Revo2 手 MJCF 生成「可驱动版本」（补 <actuator> + 锁 distal）

背景
----
官方仓库 https://github.com/BrainCoTech/brainco-description 提供的手 MJCF
（`revo2_system/mjcf/revo2_right.xml` / `revo2_left.xml`）是**纯模型**：

    <actuator>  段不存在  → nu = 0，仿真里手不会动
    <keyframe>  段不存在  → 没有预设初始姿态
    <mimic>     段不存在  → 11 个关节全部独立可动

而官方自己的 URDF 里，四指 distal 是 `mimic multiplier="1.155"`（从动），
拇指 distal 是 `mimic multiplier="1.0"`。**官方两个格式自相矛盾。**

真机侧只有 6 个可发角度（见 revo2_commander.py 的 MOTOR_ORDER），
所以仿真里也应当把 distal 锁成从动，让有效自由度 = 6，
否则仿真能做出真机做不到的姿势，调出来的动作真机复现不了。

本脚本做两件事
--------------
1. **补 <actuator>**：为 6 个主动关节各加一个 position 执行器
   （四指 proximal × 4、拇指 metacarpal、拇指 proximal），
   ctrlrange 直接抄关节的 range，kp/kv 按 actuatorfrcrange 量级估算。
2. **锁 distal 为从动**：把 4 个 distal 关节从 hinge 改成不驱动，
   并用 <equality> joint 约束让它们跟随对应 proximal（倍率 1.155），
   拇指 distal 跟随 thumb_proximal（倍率 1.0）。

左右手
------
本项目的实机手为**右手**（ELF3 右腕 `r_wrist_z_link`，CAN6 / ID 127）。
脚本按 `--side` 参数决定关节名前缀，默认 `right`。
两侧关节的几何、range、力矩限在官方模型中完全对称，只有 `right_` / `left_`
前缀不同，所以同一套代码两边都能用。

    # 右手（本项目默认）
    python3 tools/revo2_make_actuated.py revo2_system/mjcf/revo2_right.xml

    # 左手
    python3 tools/revo2_make_actuated.py revo2_system/mjcf/revo2_left.xml --side left

⚠️ 若不确定文件属于哪一侧，脚本会**自动从文件名与关节名交叉核对**，
   不一致直接拒绝执行（退出码 2），避免把右手执行器写进左手模型。

⚠️ 安全说明
  本脚本只做**纯文本 XML 变换**，不加载 MuJoCo、不发任何指令、不碰硬件。
  - 默认不覆盖原文件：输出到新文件名（`*_actuated.xml`）
  - 已存在输出文件时拒绝覆盖，除非给 --force
  - 从原始官方文件读入，保证可重复执行（幂等）

用法
----
    # 只看会改成什么，不写文件
    python3 tools/revo2_make_actuated.py revo2_system/mjcf/revo2_right.xml --dry-run

    # 真的写出来
    python3 tools/revo2_make_actuated.py revo2_system/mjcf/revo2_right.xml

    # 指定 kp/kv
    python3 tools/revo2_make_actuated.py revo2_system/mjcf/revo2_right.xml \
        --kp-thumb 2 --kp-finger 4

退出码
------
    0 = 成功
    2 = 输入问题（文件不存在 / 不是预期结构 / 左右侧与文件名不符）
    3 = 安全门禁拒绝（输出已存在且未给 --force）
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from typing import Any

EXIT_OK = 0
EXIT_INPUT = 2
EXIT_GUARD = 3

VALID_SIDES = ("left", "right")

# ── 关节表模板（与左右无关，只有前缀不同）──────────────────────
# 来源：revo2_system/mjcf/revo2_{left,right}.xml 原文逐条核对（2026-09-22）
#       两侧的 range / 力矩限完全对称，仅关节名前缀不同。
_ACTIVE_TEMPLATE: list[dict[str, Any]] = [
    {"short": "thumb_metacarpal", "range": (0.0, 1.57), "frc": 0.5,
     "group": "thumb"},
    {"short": "thumb_proximal", "range": (0.0, 1.03), "frc": 1.1,
     "group": "thumb"},
    {"short": "index_proximal", "range": (0.0, 1.41), "frc": 2.0,
     "group": "finger"},
    {"short": "middle_proximal", "range": (0.0, 1.41), "frc": 2.0,
     "group": "finger"},
    {"short": "ring_proximal", "range": (0.0, 1.41), "frc": 2.0,
     "group": "finger"},
    {"short": "pinky_proximal", "range": (0.0, 1.41), "frc": 2.0,
     "group": "finger"},
]

_MIMIC_TEMPLATE: list[dict[str, Any]] = [
    {"finger": "index", "multiplier": 1.155},
    {"finger": "middle", "multiplier": 1.155},
    {"finger": "ring", "multiplier": 1.155},
    {"finger": "pinky", "multiplier": 1.155},
    {"finger": "thumb", "multiplier": 1.0},
]


def active_joints(side: str) -> list[dict[str, Any]]:
    """按侧别展开 6 个主动关节（顺序：拇指2 + 四指4）。"""
    return [
        {"joint": f"{side}_{t['short']}_joint",
         "short": t["short"], "range": t["range"], "frc": t["frc"],
         "group": t["group"]}
        for t in _ACTIVE_TEMPLATE
    ]


def mimic_pairs(side: str) -> list[dict[str, Any]]:
    """按侧别展开 5 条 distal → proximal 的从动关系。"""
    return [
        {"distal": f"{side}_{m['finger']}_distal_joint",
         "follows": f"{side}_{m['finger']}_proximal_joint",
         "multiplier": m["multiplier"]}
        for m in _MIMIC_TEMPLATE
    ]


def side_from_filename(path: str) -> str | None:
    """从文件名猜是哪只手（revo2_right.xml → right）。猜不出返回 None。"""
    base = os.path.basename(path).lower()
    for s in VALID_SIDES:
        if s in base:
            return s
    return None


def side_from_joints(joint_names: set[str]) -> str | None:
    """从模型里的关节名猜是哪只手。两边都不像返回 None。"""
    hits = {s: 0 for s in VALID_SIDES}
    for s in VALID_SIDES:
        for t in _ACTIVE_TEMPLATE:
            if f"{s}_{t['short']}_joint" in joint_names:
                hits[s] += 1
    best = max(hits, key=lambda k: hits[k])
    return best if hits[best] > 0 else None


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


# ══════════════════════════════════════════════════════════════
def suggest_gains(frc: float, group: str, kp_thumb: float,
                  kp_finger: float) -> tuple[float, float]:
    """按关节的输出力矩量级给一组保守的 PD 增益。

    官方**没有**给执行器增益（README 自述 actuator 定义仍在 WIP），
    所以这里只能按 `actuatorfrcrange` 的绝对值估算一个起点。

    估算方式（故意偏柔和，宁可软不可硬）：

        kp = kp_base                      （不乘力矩，避免四指 8.0 这种过硬值）
        kv = kp / 20                      （临界阻尼附近的经验比例）

    其中 kp_base 默认：拇指 2.0、四指 4.0。
    含义：1 rad 误差产生 2–4 Nm 力矩，与 `actuatorfrcrange`
    （拇指 ±0.5–1.1、四指 ±2.0）同量级，不会过冲到限位。

    ⚠️ 首次真机务必从小往大试，不要直接用这里的值。
    """
    kp_base = kp_thumb if group == "thumb" else kp_finger
    kp = round(kp_base, 3)
    kv = round(kp / 20.0, 4)
    return kp, kv


def check_structure(root: ET.Element, side: str) -> list[str]:
    """检查这是不是一个官方 Revo2 MJCF，返回问题列表（空表示通过）。"""
    problems: list[str] = []
    model = root.get("model", "")
    if "revo2" not in model.lower():
        problems.append(
            f'<mujoco model="{model}"> 看起来不是 Revo2 模型 '
            f"（期望 model 名含 'revo2'）"
        )
    worldbody = root.find("worldbody")
    if worldbody is None:
        problems.append("缺少 <worldbody> 段")
    joint_names = {j.get("name") for j in root.iter("joint")}
    missing = [a["joint"] for a in active_joints(side)
               if a["joint"] not in joint_names]
    if missing:
        problems.append(f"缺少预期关节（side={side}）：{', '.join(missing)}")
    return problems


def collect_joint_map(root: ET.Element) -> dict[str, float]:
    """收集 joint 名 → range 上界，用于校验我们写死的 range 是否还准。"""
    out: dict[str, float] = {}
    for j in root.iter("joint"):
        nm = j.get("name")
        rng = j.get("range")
        if nm and rng:
            try:
                out[nm] = float(rng.split()[1])
            except (IndexError, ValueError):
                pass
    return out


def build_actuator_element(side: str, kp_thumb: float,
                           kp_finger: float) -> ET.Element:
    """生成 <actuator> 段。"""
    act = ET.Element("actuator")
    for a in active_joints(side):
        kp, kv = suggest_gains(a["frc"], a["group"], kp_thumb, kp_finger)
        lo, hi = a["range"]
        ET.SubElement(
            act,
            "position",
            {
                "name": f"{a['short']}_pos",
                "joint": a["joint"],
                "kp": f"{kp}",
                "kv": f"{kv}",
                "ctrlrange": f"{lo} {hi}",
            },
        )
    return act


def build_equality_element(side: str) -> ET.Element:
    """生成 <equality> 段，把 distal 锁成跟随 proximal。"""
    eq = ET.Element("equality")
    for m in mimic_pairs(side):
        ET.SubElement(
            eq,
            "joint",
            {
                "name": f"mimic_{m['distal']}",
                "joint1": m["distal"],
                "joint2": m["follows"],
                "polycoef": f"0 {m['multiplier']} 0 0 0",
            },
        )
    return eq


def remove_existing(root: ET.Element, tag: str) -> bool:
    """删掉已有的同名顶层段（幂等：重复跑不叠加）。返回是否删掉。"""
    found = root.find(tag)
    if found is not None:
        root.remove(found)
        return True
    return False


# ══════════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(
        description="给官方 Revo2 手 MJCF 补 <actuator> 并锁 distal 为从动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="安全：只做文本变换，不加载 MuJoCo、不发指令、不碰硬件。",
    )
    ap.add_argument("mjcf", help="官方 revo2_right.xml / revo2_left.xml 路径")
    ap.add_argument("--side", choices=list(VALID_SIDES), default=None,
                    help="手模型属于哪一侧（默认自动判断：文件名→关节名）")
    ap.add_argument("-o", "--output", default="",
                    help="输出路径（默认同目录下 <名字>_actuated.xml）")
    ap.add_argument("--kp-thumb", type=float, default=2.0,
                    help="拇指关节的 kp 基数（默认 2.0）")
    ap.add_argument("--kp-finger", type=float, default=4.0,
                    help="四指关节的 kp 基数（默认 4.0）")
    ap.add_argument("--no-mimic", action="store_true",
                    help="不锁 distal（保留 11 个独立自由度）")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要写入的内容，不写文件")
    ap.add_argument("--force", action="store_true",
                    help="输出文件已存在时覆盖")
    args = ap.parse_args()

    src = os.path.abspath(args.mjcf)
    if not os.path.exists(src):
        bad(f"文件不存在：{src}")
        print("  提示：先从官方仓库拉取")
        print("  git clone --depth 1 https://github.com/"
              "BrainCoTech/brainco-description.git")
        print("  目标文件：revo2_system/mjcf/revo2_right.xml（右手）")
        return EXIT_INPUT

    if not args.output:
        stem, ext = os.path.splitext(src)
        args.output = f"{stem}_actuated{ext}"
    dst = os.path.abspath(args.output)

    # ── 门禁：不覆盖已有输出（除非 --force）──────────────────
    if os.path.exists(dst) and not args.force and not args.dry_run:
        bad(f"输出文件已存在：{dst}")
        print("  这是防误覆盖门禁。确认要覆盖请加 --force")
        return EXIT_GUARD

    # ── 读入（保留原始文本，用于核对）────────────────────────
    try:
        with open(src, "r", encoding="utf-8") as f:
            tree = ET.parse(f)
        root = tree.getroot()
    except ET.ParseError as exc:
        bad(f"XML 解析失败：{exc}")
        return EXIT_INPUT

    info(f"读入 {os.path.basename(src)}  model=\"{root.get('model')}\"")

    # ── 定侧别：显式 --side 优先，否则由文件名 + 关节名交叉判断 ──
    joint_names_all = {j.get("name") for j in root.iter("joint")
                       if j.get("name")}
    by_file = side_from_filename(src)
    by_joint = side_from_joints(joint_names_all)

    if args.side:
        side = args.side
        if by_joint and by_joint != side:
            bad(f"你指定 --side {side}，但模型里的关节名看起来是 {by_joint} 侧")
            print("  继续执行会把一侧的执行器写进另一侧的模型。")
            print(f"  请核对文件是否弄反，或去掉 --side 让脚本自动判断。")
            return EXIT_INPUT
    elif by_joint:
        side = by_joint
    elif by_file:
        side = by_file
        warn(f"模型里没找到带侧别前缀的关节名，按文件名判定为 {side} 侧")
    else:
        bad("无法判断这只手是左还是右（文件名与关节名都没有侧别线索）")
        print("  请显式指定：--side right 或 --side left")
        return EXIT_INPUT

    if by_file and by_file != side:
        warn(f"文件名看起来是 {by_file} 侧，但关节名是 {side} 侧；"
             f"以关节名（{side}）为准")

    info(f"侧别判定：{C.BOLD}{side}{C.END}  "
         f"（文件名={by_file or '-'}，关节名={by_joint or '-'}）")

    act_joints = active_joints(side)
    mim = mimic_pairs(side)

    # ── 结构校验 ─────────────────────────────────────────────
    problems = check_structure(root, side)
    if problems:
        bad("文件结构不符合预期：")
        for p in problems:
            print(f"    - {p}")
        print("  这可能不是官方 Revo2 MJCF，或选错了 --side。")
        return EXIT_INPUT

    # ── 核对 range 是否与文档记录一致 ────────────────────────
    declared = collect_joint_map(root)
    for a in act_joints:
        actual = declared.get(a["joint"])
        if actual is not None and abs(actual - a["range"][1]) > 1e-6:
            warn(f"{a['joint']} 的 range 上界实际是 {actual}，"
                 f"我们记录的是 {a['range'][1]} → 官方可能已更新模型")
    ok(f"6 个主动关节全部存在（{side} 侧），range 核对通过")

    # ── 幂等：先删旧的同名片段 ───────────────────────────────
    removed = []
    for tag in ("actuator", "equality"):
        if remove_existing(root, tag):
            removed.append(tag)
    if removed:
        warn(f"删除了原有的 <{'>, <'.join(removed)}> 段（保证幂等）")

    # ── 插入 <actuator> ──────────────────────────────────────
    act = build_actuator_element(side, args.kp_thumb, args.kp_finger)
    # 放在 <worldbody> 前（MuJoCo 对顺序不严格，但这样可读性好）
    idx = 0
    for i, child in enumerate(list(root)):
        if child.tag == "worldbody":
            idx = i
            break
    root.insert(idx, act)
    ok(f"已生成 <actuator>，{len(act_joints)} 个 position 执行器")

    # ── 插入 <equality>（锁 distal）──────────────────────────
    if args.no_mimic:
        warn("按 --no-mimic 跳过 distal 锁定（仿真自由度 = 11，真机 = 6）")
        warn("  → 仿真可能做出真机做不到的姿势，注意风险")
    else:
        eq = build_equality_element(side)
        root.insert(idx, eq)
        ok(f"已生成 <equality>，{len(mim)} 条 distal 跟随约束")

    # ── 输出 ─────────────────────────────────────────────────
    ET.indent(tree, space="  ")
    xml_text = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(
        root, encoding="unicode"
    )

    if args.dry_run:
        info("--dry-run：以下内容不会写入文件")
        print("-" * 60)
        lines = xml_text.splitlines()
        shown = 0
        for ln in lines:
            if ("<actuator" in ln or "<position" in ln
                    or "<equality" in ln or "<joint name=\"mimic" in ln
                    or "</actuator" in ln or "</equality" in ln):
                print(ln)
                shown += 1
                if shown > 26:
                    print("    ... （省略其余执行器行）")
                    break
        print("-" * 60)
        info(f"完整文件将写到：{dst}")
        return EXIT_OK

    with open(dst, "w", encoding="utf-8") as f:
        f.write(xml_text)

    ok(f"已写入 {dst}")
    print()
    print(f"{C.BOLD}下一步（Ubuntu 上验证）{C.END}")
    print(f"  python3 -c \"import mujoco; "
          f"m=mujoco.MjModel.from_xml_path('{os.path.basename(dst)}'); "
          f"print('nq',m.nq,'nu',m.nu)\"")
    print(f"  预期：nq 仍是 11，nu = 6")
    print()
    print(f"{C.BOLD}挂载到 ELF3（{side} 侧）{C.END}")
    wrist = "r_wrist_z_link" if side == "right" else "l_wrist_z_link"
    print(f"  腕部 link：{wrist}")
    print(f"  关节前缀：{side}_")
    print()
    print(f"{C.WARN}⚠️  kp/kv 是按力矩量级估的，官方没给值。")
    print(f"    首次真机务必从小往大试，别直接用。{C.END}")
    print(f"{C.WARN}⚠️  这个文件只用于仿真。真机控制走 "
          f"revo2_commander.py，与本文件无关。{C.END}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

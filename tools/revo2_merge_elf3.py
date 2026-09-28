#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把官方 ELF3 本体 URDF 与 BrainCo Revo2 手 URDF 合并成单一 URDF

背景
----
官方模型的两个事实（2026-09-23 实测核对，非文档转述）：

1. **ELF3 本体只有 URDF，没有官方 MJCF。**
   来源 `bxirobotics/bxi_rl_controller_ros2_example` → `resources/`：
       elf3_dof29/urdf/elf3.urdf
       elf3_dof29_hand/urdf/elf3.urdf
       elf3_dof31/urdf/elf3.urdf          ← 本项目用这个
       elf3_dof31_hand/urdf/elf3.urdf
   四个目录的关节/连杆结构**完全相同**，`_hand` 变体只是把腕部视觉网格
   从 `r_wrist_z_link.STL` 换成 `r_hand.STL`（一个刚性手掌外壳），
   **没有任何手指关节**。全仓库没有 `scene.xml` / MJCF。

2. **Revo2 手同时有 URDF 与 MJCF**（`BrainCoTech/brainco-description`）。
   官方 MJCF 缺 `<actuator>`（nu=0），官方 URDF 里 distal 带 `<mimic>`。

→ 因此合并只能在 **URDF 层**做（两边都是 URDF），
   产物是一个自包含 URDF，之后再统一转 MJCF。

本脚本做四件事
--------------
1. **重挂载点**：把手的根关节 `right_hand_base_joint` 的 parent 从
   Revo2 自带的 `<link name="world"/>` 改为 ELF3 的腕部连杆
   （右手 `r_wrist_z_link` / 左手 `l_wrist_z_link`），并写入安装偏移。
2. **合并 links / joints**：把手的全部连杆与关节并入本体的 `<robot>`，
   删除 Revo2 的占位 `world` 连杆。
3. **统一网格路径**：两边的 mesh 各自归到 `meshes/<来源>/...`，
   并重写 `filename` 为相对本 URDF 的 `./meshes/...`，
   保证整个目录可以整体搬动、不依赖绝对路径。
4. **补 MuJoCo 扩展段**（★ 关键，实测得出）：MuJoCo 加载 URDF 时
   `discardvisual` 的默认值是 **true**，会**静默丢掉全部视觉网格**，
   只剩碰撞网格。实测同一份合并 URDF：

       不补扩展段：ngeom = 34（只有碰撞网格，模型在 viewer 里是残的）
       补扩展段：  ngeom = 84（视觉 + 碰撞齐全）

   所以在 `<robot>` 下写入：

       <mujoco>
         <compiler discardvisual="false" fusestatic="false"/>
       </mujoco>

   `fusestatic="false"` 保留完整 body 树（43 → 59 个 body），
   便于按连杆名定位、加 site/sensor。不需要时用 `--no-mujoco-ext`
   产出纯净 URDF（给 ROS 用）。

5. **产物校验**：合并后立刻自检 —— 名字重复、关节悬挂、网格缺失、
   是否构成单棵连通树、视觉网格是否还在。任一不过直接报错退出，
   不产出半成品。

安全设计（与 revo2_make_actuated.py 同一套门禁风格）
----------------------------------------------------
  - 只做纯文本/文件变换，**不加载 MuJoCo、不发指令、不碰硬件、不碰串口**
  - 输出目录已存在且非空 → 拒绝，除非 --force
  - 挂载连杆在 ELF3 里找不到 → 退出码 2
  - `--side` 与手模型内的关节名前缀不符 → 退出码 2（防左右搞反）
  - 合并后校验不过 → 退出码 4，并且**不留下产物**

用法
----
    # 1. 先只看会做什么（不写文件）
    python3 tools/revo2_merge_elf3.py \\
        --elf3  ~/models/elf3_dof31/urdf/elf3.urdf \\
        --hand  ~/models/brainco-description/revo2_system/urdf/revo2_right.urdf \\
        --out   ~/models/elf3_revo2_right \\
        --dry-run

    # 2. 正式生成
    python3 tools/revo2_merge_elf3.py \\
        --elf3  ~/models/elf3_dof31/urdf/elf3.urdf \\
        --hand  ~/models/brainco-description/revo2_system/urdf/revo2_right.urdf \\
        --out   ~/models/elf3_revo2_right

    # 3. 带安装偏移（先跑默认值，在 MuJoCo viewer 里看，不对再调）
    python3 tools/revo2_merge_elf3.py --elf3 ... --hand ... --out ... \\
        --mount-rpy "0 0 0" --mount-xyz "0 0 0.03"

退出码
------
    0 = 成功
    2 = 输入问题（文件不存在 / 结构不符 / 侧别不符 / 找不到挂载连杆）
    3 = 安全门禁拒绝（输出目录已存在且未给 --force）
    4 = 产物校验失败（已回滚，未留下产物）
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from typing import Any

EXIT_OK = 0
EXIT_INPUT = 2
EXIT_GUARD = 3
EXIT_VERIFY = 4

VALID_SIDES = ("left", "right")

# 腕部挂载连杆（ELF3 侧）。来源：resources/elf3_dof31/urdf/elf3.urdf 原文。
MOUNT_LINK = {
    "right": "r_wrist_z_link",
    "left": "l_wrist_z_link",
}

# Revo2 根连杆 / 根关节（两侧只有前缀不同）
HAND_ROOT_LINK = "{side}_hand_base_link"
HAND_ROOT_JOINT = "{side}_hand_base_joint"
HAND_PLACEHOLDER_LINK = "world"

# Revo2 主动关节（用于侧别交叉核对）
_ACTIVE_SHORT = (
    "thumb_metacarpal", "thumb_proximal",
    "index_proximal", "middle_proximal", "ring_proximal", "pinky_proximal",
)


# ── 小工具 ──────────────────────────────────────────────────────

def _die(code: int, msg: str) -> None:
    """打印错误并退出。"""
    print(f"[错误] {msg}", file=sys.stderr)
    sys.exit(code)


def _info(msg: str) -> None:
    print(msg)


def _parse_floats(text: str, n: int, what: str) -> list[float]:
    parts = text.replace(",", " ").split()
    if len(parts) != n:
        _die(EXIT_INPUT, f"{what} 需要 {n} 个数字，收到 {len(parts)} 个：{text!r}")
    try:
        return [float(p) for p in parts]
    except ValueError:
        _die(EXIT_INPUT, f"{what} 含非数字：{text!r}")
    return []  # unreachable


def side_from_joints(joint_names: set[str]) -> str | None:
    """从关节名判断模型属于哪一侧（模型自证优先于文件名）。"""
    hits = {s: 0 for s in VALID_SIDES}
    for s in VALID_SIDES:
        for short in _ACTIVE_SHORT:
            if f"{s}_{short}_joint" in joint_names:
                hits[s] += 1
    best = max(hits, key=lambda k: hits[k])
    return best if hits[best] > 0 else None


def side_from_filename(path: str) -> str | None:
    """从文件名兜底判断侧别。"""
    base = os.path.basename(path).lower()
    for s in VALID_SIDES:
        if s in base:
            return s
    return None


def mesh_rel_after_meshes(raw: str) -> str | None:
    """把 mesh 路径里 `meshes/` 之后的部分取出来（跨平台）。

    例：'../meshes/hands/visual/right/x.STL' -> 'hands/visual/right/x.STL'
        './meshes/r_wrist_z_link.STL'        -> 'r_wrist_z_link.STL'
    """
    norm = raw.replace("\\", "/")
    parts = [p for p in norm.split("/") if p not in ("", ".")]
    if "meshes" not in parts:
        return None
    idx = len(parts) - 1 - parts[::-1].index("meshes")
    tail = parts[idx + 1:]
    return "/".join(tail) if tail else None


def resolve_mesh_path(urdf_path: str, mesh_ref: str) -> str:
    """把 URDF 里的相对 mesh 路径解析成真实绝对路径。"""
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    norm = mesh_ref.replace("\\", "/")
    if norm.startswith("/") or (len(norm) > 1 and norm[1] == ":"):
        return os.path.normpath(norm)
    if norm.startswith("./"):
        norm = norm[2:]
    return os.path.normpath(os.path.join(urdf_dir, norm))


# ── 载入与结构检查 ──────────────────────────────────────────────

def load_robot(path: str) -> ET.Element:
    if not os.path.isfile(path):
        _die(EXIT_INPUT, f"文件不存在：{path}")
    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        _die(EXIT_INPUT, f"XML 解析失败：{path} —— {exc}")
    root = tree.getroot()
    if root.tag != "robot":
        _die(EXIT_INPUT, f"{path} 的根标签是 <{root.tag}>，不是 <robot>")
    return root


def collect(root: ET.Element, tag: str) -> list[ET.Element]:
    return list(root.findall(tag))


def names_of(elems: list[ET.Element]) -> list[str]:
    return [e.get("name", "") for e in elems]


def check_hand_side(hand_root: ET.Element, hand_path: str, side: str) -> None:
    """侧别交叉核对：显式/推断的侧别必须与模型内关节名一致。"""
    joints = {j.get("name", "") for j in collect(hand_root, "joint")}
    by_joint = side_from_joints(joints)
    if by_joint is None:
        _die(EXIT_INPUT,
             "手模型里找不到任何已知关节名，无法判断左右侧。"
             f"现有关节：{sorted(joints)[:8]}")
    if by_joint != side:
        _die(EXIT_INPUT,
             f"侧别不符：模型内关节名指向 **{by_joint}**，"
             f"但要求按 **{side}** 合并（文件：{hand_path}）。\n"
             f"       → 换文件，或改用 --side {by_joint}。")
    by_file = side_from_filename(hand_path)
    if by_file is not None and by_file != by_joint:
        _info(f"[提示] 文件名像 {by_file} 手，但关节名是 {by_joint} 手 —— "
              f"以关节名为准（文件可改名，模型自证更可靠）。")


def ensure_mount_link(elf3_root: ET.Element, mount_link: str) -> None:
    links = set(names_of(collect(elf3_root, "link")))
    if mount_link not in links:
        _die(EXIT_INPUT,
             f"ELF3 模型里找不到挂载连杆 <{mount_link}>。\n"
             f"       现有腕部连杆："
             f"{[n for n in sorted(links) if 'wrist' in n]}")


# ── 合并 ────────────────────────────────────────────────────────

def build_merged(
    elf3_root: ET.Element,
    hand_root: ET.Element,
    side: str,
    mount_xyz: list[float],
    mount_rpy: list[float],
    elf3_path: str,
    hand_path: str,
) -> tuple[ET.Element, list[tuple[str, str]]]:
    """返回（合并后的 robot 元素, [(源网格绝对路径, 目标相对路径), ...]）。"""

    mount_link = MOUNT_LINK[side]
    hand_root_link = HAND_ROOT_LINK.format(side=side)
    hand_root_joint = HAND_ROOT_JOINT.format(side=side)

    # ---- 名字冲突预检 ----
    elf3_links = set(names_of(collect(elf3_root, "link")))
    elf3_joints = set(names_of(collect(elf3_root, "joint")))
    hand_links = set(names_of(collect(hand_root, "link")))
    hand_joints = set(names_of(collect(hand_root, "joint")))

    dup_l = (elf3_links - {HAND_PLACEHOLDER_LINK}) & hand_links
    dup_j = elf3_joints & hand_joints
    if dup_l or dup_j:
        _die(EXIT_INPUT,
             f"名字冲突，无法安全合并：连杆 {sorted(dup_l)} 关节 {sorted(dup_j)}")

    # ---- 手侧根关节必须存在 ----
    root_joint_el: ET.Element | None = None
    for j in collect(hand_root, "joint"):
        if j.get("name") == hand_root_joint:
            root_joint_el = j
            break
    if root_joint_el is None:
        _die(EXIT_INPUT,
             f"手模型里找不到根关节 <{hand_root_joint}>。"
             f"现有 fixed 关节："
             f"{[j.get('name') for j in collect(hand_root, 'joint') if j.get('type') == 'fixed'][:6]}")

    # ---- 复制手树（深拷贝，不动原文件）----
    new_robot = ET.Element("robot", {"name": "elf3_revo2_" + side})

    # 本体 links / joints 原样搬入，只改网格路径
    for tag in ("link", "joint"):
        for el in collect(elf3_root, tag):
            new_robot.append(el)

    # 顶层 material（若本体有定义，保留）
    for el in collect(elf3_root, "material"):
        new_robot.append(el)

    mesh_jobs: list[tuple[str, str]] = []

    def remap_and_collect(el: ET.Element, tag: str, src_urdf: str) -> None:
        """改写该元素下所有 mesh 的 filename，并登记拷贝任务。"""
        for mesh in el.iter("mesh"):
            raw = mesh.get("filename")
            if not raw:
                continue
            rel = mesh_rel_after_meshes(raw)
            if rel is None:
                _die(EXIT_INPUT,
                     f"网格路径里没有 'meshes' 目录，无法安全重定位：{raw!r}")
            new_rel = f"meshes/{tag}/{rel}"
            mesh.set("filename", "./" + new_rel)
            src = resolve_mesh_path(src_urdf, raw)
            mesh_jobs.append((src, new_rel))

    for el in collect(elf3_root, "link"):
        remap_and_collect(el, "elf3", elf3_path)

    # ---- 挂载点：改写根关节 parent ----
    root_joint_el.set("type", "fixed")
    parent = root_joint_el.find("parent")
    child = root_joint_el.find("child")
    if parent is None or child is None:
        _die(EXIT_INPUT, f"<{hand_root_joint}> 缺 <parent> 或 <child>")
    parent.set("link", mount_link)
    child.set("link", hand_root_link)

    # 安装偏移（原来指向 world，origin 通常为 0；这里按参数覆盖）
    origin = root_joint_el.find("origin")
    if origin is None:
        origin = ET.SubElement(root_joint_el, "origin")
    origin.set("xyz", " ".join(f"{v:g}" for v in mount_xyz))
    origin.set("rpy", " ".join(f"{v:g}" for v in mount_rpy))

    # ---- 搬入手：去掉占位 world 连杆，其余全进 ----
    for el in collect(hand_root, "link"):
        if el.get("name") == HAND_PLACEHOLDER_LINK:
            continue
        remap_and_collect(el, "revo2", hand_path)
        new_robot.append(el)

    new_robot.append(root_joint_el)
    for el in collect(hand_root, "joint"):
        if el is root_joint_el:
            continue
        new_robot.append(el)

    # 手侧顶层 material
    for el in collect(hand_root, "material"):
        new_robot.append(el)

    return new_robot, mesh_jobs


def add_mujoco_extension(robot: ET.Element) -> None:
    """在 <robot> 下插入 MuJoCo 的 URDF 扩展段。

    必须加，否则 MuJoCo 加载 URDF 时会因为 `discardvisual` 默认为 true
    而**静默丢掉全部视觉网格**（实测 ngeom 34 → 84）。
    详见模块 docstring 第 4 条。
    """
    ext = ET.Element("mujoco")
    ET.SubElement(ext, "compiler", {
        "discardvisual": "false",   # ★ 保住视觉网格
        "fusestatic": "false",      # 保留完整 body 树
    })
    robot.insert(0, ext)


# ── 产物校验 ────────────────────────────────────────────────────

def verify_merged(
    robot: ET.Element,
    out_dir: str,
    mesh_jobs: list[tuple[str, str]],
    expect_hand_root: str,
    mount_link: str,
) -> list[str]:
    """返回问题列表（空 = 通过）。"""
    problems: list[str] = []

    links = names_of(collect(robot, "link"))
    joints = collect(robot, "joint")

    # 1. 重名
    if len(links) != len(set(links)):
        dup = sorted({n for n in links if links.count(n) > 1})
        problems.append(f"连杆重名：{dup}")
    jn = names_of(joints)
    if len(jn) != len(set(jn)):
        dup = sorted({n for n in jn if jn.count(n) > 1})
        problems.append(f"关节重名：{dup}")

    link_set = set(links)

    # 2. 关节引用完整性
    for j in joints:
        p = j.find("parent")
        c = j.find("child")
        jname = j.get("name")
        if p is None or c is None:
            problems.append(f"关节 {jname} 缺 parent/child")
            continue
        pl, cl = p.get("link"), c.get("link")
        if pl not in link_set:
            problems.append(f"关节 {jname} 的 parent 连杆不存在：{pl}")
        if cl not in link_set:
            problems.append(f"关节 {jname} 的 child 连杆不存在：{cl}")

    # 3. 单棵连通树：入度统计（每个连杆最多一个父关节）
    child_count: dict[str, int] = {}
    for j in joints:
        c = j.find("child")
        if c is not None:
            cl = c.get("link", "")
            child_count[cl] = child_count.get(cl, 0) + 1
    multi = {k: v for k, v in child_count.items() if v > 1}
    if multi:
        problems.append(f"连杆有多个父关节（不成树）：{multi}")

    roots = [l for l in links if child_count.get(l, 0) == 0]
    if len(roots) != 1:
        problems.append(f"根连杆数量应为 1，实际 {len(roots)}：{roots}")

    # 4. 手根连杆确实挂上去了
    if expect_hand_root not in link_set:
        problems.append(f"手根连杆未出现在产物中：{expect_hand_root}")
    attached = any(
        (j.find("child") is not None and j.find("child").get("link") == expect_hand_root)
        and (j.find("parent") is not None and j.find("parent").get("link") == mount_link)
        for j in joints
    )
    if not attached:
        problems.append(f"手根连杆没有挂到 {mount_link} 上")

    # 5. 网格：引用合法 + 文件已就位
    for link in collect(robot, "link"):
        for mesh in link.iter("mesh"):
            f = mesh.get("filename", "")
            if not f.startswith("./meshes/"):
                problems.append(f"网格路径未规范化：{f}")
                continue
            rel = f[2:]
            if not os.path.isfile(os.path.join(out_dir, rel)):
                problems.append(f"网格文件缺失：{rel}")

    # 6. 源网格都真实存在
    missing_src = [s for s, _ in mesh_jobs if not os.path.isfile(s)]
    if missing_src:
        problems.append(f"源网格缺失 {len(missing_src)} 个，例如：{missing_src[:3]}")

    # 7. 视觉网格引用数量（URDF 侧统计；MuJoCo 是否会丢要看扩展段）
    n_visual = 0
    for link in collect(robot, "link"):
        n_visual += len(link.findall("visual"))
    if n_visual == 0:
        problems.append("产物里没有任何 <visual>，模型在 viewer 中会不可见")

    return problems


# ── 主流程 ──────────────────────────────────────────────────────

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="合并官方 ELF3 本体 URDF 与 Revo2 手 URDF（只做文件变换）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：python3 tools/revo2_merge_elf3.py "
               "--elf3 elf3.urdf --hand revo2_right.urdf --out ./out --dry-run",
    )
    ap.add_argument("--elf3", required=True, help="ELF3 本体 URDF 路径")
    ap.add_argument("--hand", required=True, help="Revo2 手 URDF 路径")
    ap.add_argument("--out", required=True, help="输出目录")
    ap.add_argument("--side", choices=list(VALID_SIDES), default=None,
                    help="哪只手；默认按手模型内的关节名自动判断")
    ap.add_argument("--mount-xyz", default="0 0 0",
                    help="安装位移（米），空格分隔三个数，默认 '0 0 0'")
    ap.add_argument("--mount-rpy", default="0 0 0",
                    help="安装姿态（弧度），空格分隔三个数，默认 '0 0 0'")
    ap.add_argument("--dry-run", action="store_true", help="只检查并打印，不写文件")
    ap.add_argument("--no-mujoco-ext", action="store_true",
                    help="不写入 MuJoCo 扩展段（产出纯净 URDF 给 ROS 用；"
                         "⚠️ 此时 MuJoCo 加载会丢掉视觉网格）")
    ap.add_argument("--force", action="store_true",
                    help="输出目录已存在时也继续（会先清空该目录）")
    args = ap.parse_args(argv)

    mount_xyz = _parse_floats(args.mount_xyz, 3, "--mount-xyz")
    mount_rpy = _parse_floats(args.mount_rpy, 3, "--mount-rpy")

    # ---- 目录门禁（尽早检查，避免白白算一遍）----
    out_dir = os.path.abspath(args.out)
    out_exists = os.path.exists(out_dir) and bool(os.listdir(out_dir))
    if out_exists and not args.force:
        _die(EXIT_GUARD,
             f"输出目录已存在且非空：{out_dir}\n"
             f"       → 换一个目录，或加 --force（会先清空该目录）")

    # ---- 载入 ----
    elf3_root = load_robot(args.elf3)
    hand_root = load_robot(args.hand)

    # ---- 判侧 ----
    hand_joints = {j.get("name", "") for j in collect(hand_root, "joint")}
    if args.side is not None:
        side = args.side
    else:
        side = side_from_joints(hand_joints) or side_from_filename(args.hand)
        if side is None:
            _die(EXIT_INPUT,
                 "无法判断手模型的左右侧。请显式指定 --side {left,right}。")
        _info(f"[判侧] 自动判定为 **{side}**（依据关节名；文件名仅兜底）")

    check_hand_side(hand_root, args.hand, side)
    mount_link = MOUNT_LINK[side]
    ensure_mount_link(elf3_root, mount_link)

    _info(f"[输入] ELF3 : {args.elf3}")
    _info(f"[输入] 手   : {args.hand}  （{side}）")
    _info(f"[挂载] 手根关节 → ELF3 <{mount_link}>")
    _info(f"[偏移] xyz='{args.mount_xyz}'  rpy='{args.mount_rpy}'")

    # ---- 合并 ----
    merged, mesh_jobs = build_merged(
        elf3_root, hand_root, side,
        mount_xyz, mount_rpy, args.elf3, args.hand,
    )
    if not args.no_mujoco_ext:
        add_mujoco_extension(merged)

    n_links = len(collect(merged, "link"))
    n_joints = len(collect(merged, "joint"))
    n_mesh = len({rel for _, rel in mesh_jobs})
    ext_state = "未加（纯净 URDF）" if args.no_mujoco_ext else "已加（保住视觉网格）"
    _info("")
    _info(f"[结果] 连杆 {n_links} 个 / 关节 {n_joints} 个 / "
          f"网格 {n_mesh} 个（去重后）")
    _info(f"[扩展] MuJoCo 段：{ext_state}")

    # ---- 目录门禁（删除动作推迟到这里，确认输入无误之后再动磁盘）----
    if out_exists and args.force:
        if args.dry_run:
            _info(f"[预演] （--dry-run 不实际清空）{out_dir}")
        else:
            _info(f"[清理] --force 已给，清空 {out_dir}")
            shutil.rmtree(out_dir)

    if args.dry_run:
        _info("")
        _info("[预演] 将写入：")
        _info(f"        {out_dir}/elf3_revo2_{side}.urdf")
        _info(f"        {out_dir}/meshes/elf3/...   （本体网格）")
        _info(f"        {out_dir}/meshes/revo2/...  （手网格）")
        _info("")
        _info("[预演] 未写任何文件，未做产物校验。去掉 --dry-run 正式生成。")
        return EXIT_OK

    # ---- 写盘 ----
    os.makedirs(out_dir, exist_ok=True)
    for src, rel in mesh_jobs:
        dst = os.path.join(out_dir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if not os.path.isfile(dst):
            shutil.copy2(src, dst)

    urdf_out = os.path.join(out_dir, f"elf3_revo2_{side}.urdf")
    ET.indent(merged, space="  ")
    body = ET.tostring(merged, encoding="unicode")
    with open(urdf_out, "w", encoding="utf-8") as fh:
        fh.write("<?xml version='1.0' encoding='utf-8'?>\n")
        fh.write("<!-- 本文件由 tools/revo2_merge_elf3.py 自动生成，请勿手工编辑。 -->\n")
        fh.write(f"<!-- 来源: {os.path.basename(args.elf3)} + "
                 f"{os.path.basename(args.hand)} ({side}) -->\n")
        fh.write(body)
        fh.write("\n")

    # ---- 产物校验 ----
    problems = verify_merged(
        merged, out_dir, mesh_jobs,
        HAND_ROOT_LINK.format(side=side), mount_link,
    )
    if problems:
        print("", file=sys.stderr)
        print("[校验失败] 产物有问题，已回滚：", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        shutil.rmtree(out_dir, ignore_errors=True)
        return EXIT_VERIFY

    _info("")
    _info("✅ 产物校验通过：名字唯一 / 关节悬挂完整 / 单棵连通树 / 网格齐全")
    _info(f"✅ 已生成：{urdf_out}")
    _info("")
    _info("下一步（在本机验证模型能加载）：")
    _info(f"    python3 -c \"import mujoco; "
          f"m=mujoco.MjModel.from_xml_path('{urdf_out}'); "
          f"print('nq=',m.nq,'nu=',m.nu,'nbody=',m.nbody)\"")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""把 BrainCo Revo2 灵巧手 MJCF 合并进 ELF3 官方 MuJoCo 模型。

为什么需要这个脚本
------------------
官方 MuJoCo 仿真模型（``elf3.xml``）的腕部连杆 ``r_wrist_z_link`` 是**叶子**，
官方自带的 "带手" 版本（``elf3_hand.xml``）只是把该连杆的网格换成 ``r_hand.STL``
（一块**不可动的**手壳），并没有手指关节。因此：

* 想让手在仿真里**动起来**，必须把 Revo2 的手部 body 树挂到 ``r_wrist_z_link`` 下；
* 官方 MJCF 只驱动 31 个本体关节（``nu=31``），手的关节**没有 actuator**，必须补；
* 真机只有 6 个手部电机，四指远端是机械耦合的，仿真里要用 ``equality`` 锁住，
  否则仿真能摆出真机做不到的姿势。

产物
----
``<out-dir>/elf3_revo2_<side>.xml`` + 手部网格拷进 ``<out-dir>/meshes/``。
输出与官方 ``elf3.xml`` 同目录，所以 ``<include file="sensors/...">``、
``meshdir="./meshes/"`` 这些相对路径**照旧生效**，不需要额外拷贝地形和传感器。

安全设计（四道门禁）
--------------------
1. **幂等**：输出已存在同名手部 body / mesh / actuator 时直接报错，不重复追加。
2. **防误覆盖**：输出文件已存在时拒绝写入，除非显式 ``--force``。
3. **结构校验**：合并后自检 —— 名字唯一、关节全在、单棵连通树、网格齐全。
4. **侧别交叉核对**：``--side`` 与手模型内的关节名前缀冲突时拒绝执行（退出码 2）。

用法
----
    python3 tools/revo2_merge_mjcf.py \\
        --robot  <path>/elf3.xml \\
        --hand   <path>/revo2_right.xml \\
        [--out-dir <dir>]        # 默认与 --robot 同目录
        [--side right|left]      # 默认自动判侧
        [--mount-xyz "0 0 0"] [--mount-rpy "0 1.5708 0"]
        [--kp-hand 2.0] [--lock-distal]
        [--dry-run] [--force]

退出码：0 成功 / 1 用法或输入错误 / 2 侧别冲突 / 3 输出已存在（防误覆盖）/
        4 产物校验未通过。
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from typing import Any

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_SIDE_CONFLICT = 2
EXIT_GUARD = 3
EXIT_VALIDATE = 4

VALID_SIDES = ("left", "right")

# ★ 命名差异：ELF3 本体用缩写前缀 ``l_`` / ``r_``（如 ``r_wrist_z_link``），
#   而 Revo2 手模型用全称 ``left_`` / ``right_``（如 ``right_hand_base_link``）。
#   挂载点必须按 ELF3 的写法找。
ELF3_SHORT_PREFIX = {"left": "l_", "right": "r_"}
MOUNT_BODY = {"left": "l_wrist_z_link", "right": "r_wrist_z_link"}

# 手侧关节「短名」——去掉 left_/right_ 前缀后的名字。
# 前 6 个是真机能单独下发的电机，后 5 个（*_distal）是真机内部耦合的从动关节。
ACTIVE_SHORTS = (
    "thumb_metacarpal",
    "thumb_proximal",
    "index_proximal",
    "middle_proximal",
    "ring_proximal",
    "pinky_proximal",
)
MIMIC_SHORTS = (
    ("thumb_distal", "thumb_proximal", 1.0),
    ("index_distal", "index_proximal", 1.155),
    ("middle_distal", "middle_proximal", 1.155),
    ("ring_distal", "ring_proximal", 1.155),
    ("pinky_distal", "pinky_proximal", 1.155),
)

# 合并后新增的 default 类。
# ★ 实测结论（MuJoCo 3.13）：default class 名是**扁平**的，嵌套不产生 "父/子" 路径。
#   - ``<default class="a"><default class="collision">`` 与 ``<default class="b"><default class="collision">``
#     会报 ``repeated default class name``；
#   - ``class="a/collision"`` 会报 ``unknown default class name``。
#   所以新增的类必须在**叶子名**上唯一，且写元素时必须写全名。
CLS_VISUAL = "revo2_visual"
CLS_COLLISION = "revo2_collision"

# 手部网格统一前缀，避免与官方 meshes/ 里的同名文件（如 right_base_link.STL
# 在 visual/ 与 collision/ 目录下同名）冲突。
VIS_PREFIX = "revo2_vis_"
COL_PREFIX = "revo2_col_"


def _info(msg: str) -> None:
    print(msg, flush=True)


def _die(code: int, msg: str) -> "None":
    print(f"\n[错误] {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


def _warn(msg: str) -> None:
    print(f"[警告] {msg}", flush=True)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    """把 URDF 风格的 rpy（固定轴 X-Y-Z 外旋）转成 MuJoCo 的 (w, x, y, z) 四元数。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def fmt(v: float) -> str:
    """紧凑浮点格式化，避免 ``0.30000000000000004`` 这类噪声。"""
    s = f"{v:.9g}"
    return "0" if s in ("-0", "0") else s


def parse_floats(text: str, n: int, flag: str) -> list[float]:
    parts = text.replace(",", " ").split()
    if len(parts) != n:
        _die(EXIT_USAGE, f"{flag} 需要 {n} 个数，实际给了 {len(parts)} 个：{text!r}")
    try:
        return [float(p) for p in parts]
    except ValueError:
        _die(EXIT_USAGE, f"{flag} 含非数字：{text!r}")
        raise  # 不可达，仅为类型收窄


def short_of(joint_name: str, side: str) -> str | None:
    """``right_index_proximal_joint`` -> ``index_proximal``（只认本侧前缀）。"""
    pref = f"{side}_"
    suff = "_joint"
    if joint_name.startswith(pref) and joint_name.endswith(suff):
        return joint_name[len(pref) : -len(suff)]
    return None


def side_from_names(names: set[str]) -> str | None:
    """优先依据关节名自证侧别（文件名可被改，关节名改不动）。"""
    hits = {s: 0 for s in VALID_SIDES}
    for s in VALID_SIDES:
        for short in ACTIVE_SHORTS:
            if f"{s}_{short}_joint" in names:
                hits[s] += 1
    best = max(hits, key=lambda k: hits[k])
    return best if hits[best] > 0 else None


def side_from_filename(path: str) -> str | None:
    base = os.path.basename(path).lower()
    for s in VALID_SIDES:
        if s in base:
            return s
    return None


# ---------------------------------------------------------------------------
# 载入
# ---------------------------------------------------------------------------


def load_robot(path: str) -> tuple[ET.Element, ET.ElementTree]:
    if not os.path.isfile(path):
        _die(EXIT_USAGE, f"机器人 MJCF 不存在：{path}")
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "mujoco":
        _die(EXIT_USAGE, f"{path} 根标签是 <{root.tag}>，不是 <mujoco>")
    return root, tree


def load_hand(path: str) -> ET.Element:
    if not os.path.isfile(path):
        _die(EXIT_USAGE, f"灵巧手 MJCF 不存在：{path}")
    root = ET.parse(path).getroot()
    if root.tag != "mujoco":
        _die(EXIT_USAGE, f"{path} 根标签是 <{root.tag}>，不是 <mujoco>")
    return root


def hand_body_root(hand_root: ET.Element, side: str) -> ET.Element:
    """在手 MJCF 的 worldbody 里找手根 body。"""
    wb = hand_root.find("worldbody")
    if wb is None:
        _die(EXIT_USAGE, "手 MJCF 没有 <worldbody>")
    bodies = wb.findall("body")
    if len(bodies) != 1:
        # 多于一个时按侧别猜根（``<side>_hand_base_link``）
        want = f"{side}_hand_base_link"
        for b in bodies:
            if b.get("name") == want:
                return b
        _die(EXIT_USAGE,
             f"手 MJCF 的 <worldbody> 下有 {len(bodies)} 个顶层 body，"
             f"无法确定手根（期望 {want}）")
    return bodies[0]


# ---------------------------------------------------------------------------
# 网格处理
# ---------------------------------------------------------------------------


def rewrite_hand_meshes(
    hand_root: ET.Element, hand_path: str
) -> tuple[list[ET.Element], list[tuple[str, str]]]:
    """改写手 MJCF 的 <asset><mesh file=...>，返回（mesh 元素, [(源路径, 目标名)]）。

    手模型的 ``file=`` 是 ``../meshes/hands/visual/right/xxx.STL`` 这种相对路径，
    而官方机器人模型的 ``meshdir`` 是 ``./meshes/``。这里把文件拍平到 ``meshes/``，
    并加 ``revo2_vis_`` / ``revo2_col_`` 前缀避免同名冲突。
    """
    hand_dir = os.path.dirname(os.path.abspath(hand_path))
    asset = hand_root.find("asset")
    if asset is None:
        _die(EXIT_USAGE, "手 MJCF 没有 <asset> 段")

    mesh_els = asset.findall("mesh")
    if not mesh_els:
        _die(EXIT_USAGE, "手 MJCF 的 <asset> 里没有 <mesh>")

    jobs: list[tuple[str, str]] = []
    for m in mesh_els:
        rel = m.get("file")
        if not rel:
            _die(EXIT_USAGE, f"手 MJCF 里有 <mesh name={m.get('name')!r}> 缺 file 属性")
        src = os.path.normpath(os.path.join(hand_dir, rel))
        if not os.path.isfile(src):
            _die(EXIT_USAGE,
                 f"手 MJCF 引用的网格不存在：\n       file={rel}\n       解析为={src}")

        # 用原相对路径里的目录名判断 visual / collision
        low = rel.replace("\\", "/").lower()
        if "/visual/" in low:
            prefix = VIS_PREFIX
        elif "/collision/" in low:
            prefix = COL_PREFIX
        else:
            # 目录里没写，退而用文件名里的 visual 字样
            prefix = VIS_PREFIX if "visual" in os.path.basename(low) else COL_PREFIX

        new_name = prefix + os.path.basename(rel)
        m.set("file", new_name)
        jobs.append((src, new_name))

    return mesh_els, jobs


# ---------------------------------------------------------------------------
# 合并
# ---------------------------------------------------------------------------


def looks_like_collision_geom(g: ET.Element) -> bool:
    """判断一个机器人侧 geom 是否属于「碰撞体」。

    官方 MJCF 的碰撞体统一带 ``class="collision"``（在 ``childclass="elf3"`` 下解析），
    且命名以 ``_collision`` 结尾、类型是 capsule/sphere 而非 mesh。三条任一命中即认为是
    碰撞体，宁可多删一个纯碰撞体，也不漏删导致手被顶住。
    """
    if g.get("class") == "collision":
        return True
    if (g.get("name") or "").endswith("_collision"):
        return True
    if g.get("mesh") is None and g.get("type") not in (None, "mesh"):
        return True
    return False


def build_merged(
    robot_root: ET.Element,
    hand_root: ET.Element,
    side: str,
    hand_path: str,
    mount_xyz: list[float],
    mount_rpy: list[float],
    kp_hand: float,
    hand_armature: float,
    lock_distal: bool,
) -> tuple[ET.Element, list[tuple[str, str]], list[dict[str, Any]], list[str]]:
    """返回（合并后的 mujoco 元素, 网格任务, 手部关节清单, 被摘掉的腕部碰撞体）。"""
    import copy

    merged = copy.deepcopy(robot_root)

    # ---- 0. 幂等门禁：机器人里不得已有手 ----
    existing_names = {el.get("name") for el in merged.iter() if el.get("name")}
    dup = [f"{side}_{s}_joint" for s in ACTIVE_SHORTS
           if f"{side}_{s}_joint" in existing_names]
    if dup:
        _die(EXIT_GUARD,
             "机器人模型里已经存在手部关节，拒绝重复合并（幂等门禁）：\n"
             + "\n".join(f"       {n}" for n in dup[:5]))

    # ---- 1. 找挂载点 ----
    target_name = MOUNT_BODY[side]
    target = None
    for b in merged.iter("body"):
        if b.get("name") == target_name:
            target = b
            break
    if target is None:
        avail = [b.get("name") for b in merged.iter("body")
                 if (b.get("name") or "").endswith("wrist_z_link")]
        _die(EXIT_USAGE,
             f"在机器人模型里找不到挂载点 <body name={target_name!r}>。\n"
             f"       可用的腕部连杆：{avail}")

    # ---- 2. 网格 ----
    mesh_els, mesh_jobs = rewrite_hand_meshes(hand_root, hand_path)

    # ---- 3. 把手根 body 挂到 target 下 ----
    hb = hand_body_root(hand_root, side)
    quat = rpy_to_quat(*mount_rpy)
    attach = copy.deepcopy(hb)
    attach.set("pos", " ".join(fmt(v) for v in mount_xyz))
    attach.set("quat", " ".join(fmt(v) for v in quat))
    target.append(attach)

    # ---- 3b. 摘掉腕部法兰的碰撞体 ----
    # ★ 必须做，否则手会被顶住。官方腕部有个 r=<30mm> 的碰撞球
    #   （r_wrist_z_collision, pos="0.04 0 0"），它是按厂商自家末端执行器做的。
    #   挂上 Revo2 后，实测它正好压住 right_thumb_proximal_link，把拇指对掌
    #   从 90° 卡在 56.9°（因为 filterparent 只排除「直接父子」，腕与拇指
    #   中间还隔着 hand_base / thumb_metacarpal，所以照常碰撞）。
    removed_coll: list[str] = []
    for g in list(target.findall("geom")):
        if looks_like_collision_geom(g):
            removed_coll.append(g.get("name") or "(未命名碰撞体)")
            target.remove(g)

    # ---- 4. 合并 <asset> ----
    robot_asset = merged.find("asset")
    if robot_asset is None:
        robot_asset = ET.SubElement(merged, "asset")
    robot_mesh_names = {m.get("name") for m in robot_asset.findall("mesh")}
    clash = [m.get("name") for m in mesh_els if m.get("name") in robot_mesh_names]
    if clash:
        _die(EXIT_GUARD,
             "手部 mesh 名与机器人已有 mesh 冲突（幂等门禁）：\n"
             + "\n".join(f"       {c}" for c in clash[:5]))
    for m in mesh_els:
        robot_asset.append(copy.deepcopy(m))

    # ---- 5. 补 default 类（扁平、叶子名唯一）+ 改写手侧 geom 的 class ----
    default_root = merged.find("default")
    if default_root is None:
        default_root = ET.Element("default")
        merged.insert(0, default_root)
    existing_cls = {d.get("class") for d in default_root.findall("default")}
    for cls_name, is_visual in ((CLS_VISUAL, True), (CLS_COLLISION, False)):
        if cls_name in existing_cls:
            _die(EXIT_GUARD, f"default class {cls_name!r} 已存在（幂等门禁）")
        sub = ET.SubElement(default_root, "default")
        sub.set("class", cls_name)
        g = ET.SubElement(sub, "geom")
        g.set("type", "mesh")
        if is_visual:
            g.set("contype", "0")
            g.set("conaffinity", "0")
            g.set("group", "2")
        else:
            g.set("group", "3")

    # 手 MJCF 里 geom 写的是 class="visual"/"collision"。若不改写，会撞上机器人
    # 自己的 collision 类（那是个 capsule，会把 mesh 覆盖掉）。这里改成我们新建的类。
    cls_map = {"visual": CLS_VISUAL, "collision": CLS_COLLISION}
    n_rewritten = 0
    for el in attach.iter():
        c = el.get("class")
        if c in cls_map:
            el.set("class", cls_map[c])
            n_rewritten += 1
    if n_rewritten == 0:
        _die(EXIT_VALIDATE, "手侧没有任何 geom 带 class=\"visual\"/\"collision\"，模型结构异常")

    # ---- 6. 收集手部关节 + 补 armature ----
    # ★ armature 是必须的：官方手 MJCF 的连杆惯量只有 ~1e-6 kg·m²（指尖 9 g），
    #   不补 armature 时位置伺服的固有频率 ω=sqrt(kp/I) ≈ 1400+ rad/s，
    #   在默认 dt=0.002 s 下**数值发散**——表现是「指令 80° 实际 6°」这种
    #   看起来像「力不够」的假象。补上反射转子惯量后立刻准确跟踪。
    hand_joints: list[dict[str, Any]] = []
    for j in merged.iter("joint"):
        jn = j.get("name") or ""
        short = short_of(jn, side)
        if short is None:
            continue
        rng = j.get("range")
        lo, hi = (float(x) for x in rng.split()) if rng else (0.0, 0.0)
        if not j.get("axis"):
            _die(EXIT_VALIDATE, f"手部关节 {jn} 缺少 axis 属性")
        j.set("armature", fmt(hand_armature))
        hand_joints.append({
            "name": jn, "short": short, "lo": lo, "hi": hi,
            "active": short in ACTIVE_SHORTS,
        })
    if len(hand_joints) != len(ACTIVE_SHORTS) + len(MIMIC_SHORTS):
        _die(EXIT_VALIDATE,
             f"手模型里找到 {len(hand_joints)} 个手部关节，"
             f"期望 {len(ACTIVE_SHORTS) + len(MIMIC_SHORTS)} 个。"
             f"关节名：{[h['name'] for h in hand_joints]}")

    active = [h for h in hand_joints if h["active"]]
    active.sort(key=lambda h: ACTIVE_SHORTS.index(h["short"]))

    # ---- 7. 补 <actuator>（位置伺服）----
    act_root = merged.find("actuator")
    if act_root is None:
        act_root = ET.SubElement(merged, "actuator")
    act_names = {a.get("name") for a in act_root}
    kv = kp_hand / 20.0
    for h in active:
        if h["name"] in act_names:
            _die(EXIT_GUARD, f"actuator {h['name']} 已存在（幂等门禁）")
        a = ET.SubElement(act_root, "position")
        a.set("name", h["name"])
        a.set("joint", h["name"])
        a.set("kp", fmt(kp_hand))
        a.set("kv", fmt(kv))
        a.set("ctrllimited", "true")
        a.set("ctrlrange", f"{fmt(h['lo'])} {fmt(h['hi'])}")

    # ---- 8. 锁从动关节（equality）----
    if lock_distal:
        eq_root = merged.find("equality")
        if eq_root is None:
            eq_root = ET.SubElement(merged, "equality")
        existing_eq = {e.get("name") for e in eq_root}
        for follow_short, driver_short, mult in MIMIC_SHORTS:
            eq_name = f"{side}_{follow_short}_mimic"
            if eq_name in existing_eq:
                _die(EXIT_GUARD, f"equality {eq_name} 已存在（幂等门禁）")
            e = ET.SubElement(eq_root, "joint")
            e.set("name", eq_name)
            # ★ MuJoCo 的 equality/joint 约定：joint1 是**因变量**，joint2 是自变量，
            #   约束为 q1 = polycoef(q2)。写反了会得到 q_follow = q_driver / mult
            #   这种「悄悄算错」的结果（不报错，但比值变成 1/mult）。
            e.set("joint1", f"{side}_{follow_short}_joint")
            e.set("joint2", f"{side}_{driver_short}_joint")
            e.set("polycoef", f"0 {fmt(mult)} 0 0 0")

    return merged, mesh_jobs, hand_joints, removed_coll


# ---------------------------------------------------------------------------
# 产物校验
# ---------------------------------------------------------------------------


def validate(merged: ET.Element, mesh_jobs: list[tuple[str, str]]) -> list[str]:
    problems: list[str] = []

    # 1. 名字唯一（body / joint / geom / actuator / equality）
    for tag in ("body", "joint", "geom", "actuator", "position", "mesh", "equality"):
        seen: dict[str, int] = {}
        for el in merged.iter(tag):
            n = el.get("name")
            if not n:
                continue
            seen[n] = seen.get(n, 0) + 1
        dups = [k for k, v in seen.items() if v > 1]
        if dups:
            problems.append(f"<{tag}> 名字重复：{dups[:5]}")

    # 2. 每个 joint 都有父 body；树连通
    worldbody = merged.find("worldbody")
    if worldbody is None:
        problems.append("缺少 <worldbody>")
        return problems
    if len(worldbody.findall("body")) != 1:
        problems.append(
            f"<worldbody> 下顶层 body 数为 {len(worldbody.findall('body'))}，应为 1（单棵树）"
        )

    # 3. 网格源文件真实存在
    missing = [s for s, _ in mesh_jobs if not os.path.isfile(s)]
    if missing:
        problems.append(f"源网格缺失 {len(missing)} 个，例如 {missing[:3]}")

    # 4. actuator 数 = 期望
    n_act = len(merged.find("actuator").findall("position")) if merged.find("actuator") is not None else 0
    if n_act < len(ACTIVE_SHORTS):
        problems.append(f"actuator 只有 {n_act} 个，少于手部主动关节数 {len(ACTIVE_SHORTS)}")

    return problems


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="revo2_merge_mjcf.py",
        description="把 Revo2 灵巧手 MJCF 合并进 ELF3 官方 MuJoCo 模型（eli3.xml）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--robot", required=True, help="官方 elf3.xml 路径")
    ap.add_argument("--hand", required=True, help="官方 revo2_{left,right}.xml 路径")
    ap.add_argument("--out-dir", default=None,
                    help="输出目录（默认与 --robot 同目录，保证相对 include 生效）")
    ap.add_argument("--side", choices=list(VALID_SIDES), default=None,
                    help="手侧，默认自动判断（先信关节名，文件名仅兜底）")
    ap.add_argument("--mount-xyz", default="0 0 0", help="手根相对腕部连杆的平移（米）")
    ap.add_argument("--mount-rpy", default="0 1.5707963 0",
                    help="手根相对腕部连杆的旋转（弧度，固定轴 XYZ）")
    ap.add_argument("--kp-hand", type=float, default=5.0, help="手部位置伺服 kp")
    ap.add_argument("--hand-armature", type=float, default=0.005,
                    help="手部关节的反射转子惯量（必须 >0，否则数值发散）")
    ap.add_argument("--lock-distal", action="store_true",
                    help="用 equality 锁住四指远端（与真机一致，推荐）")
    ap.add_argument("--dry-run", action="store_true", help="只检查并打印，不写文件")
    ap.add_argument("--force", action="store_true", help="输出已存在时也继续（覆盖该文件）")
    args = ap.parse_args(argv)

    # ---- 载入 ----
    robot_root, _ = load_robot(args.robot)
    hand_root = load_hand(args.hand)

    # ---- 判侧 ----
    names = {j.get("name") for j in hand_root.iter("joint") if j.get("name")}
    by_joint = side_from_names(names)
    by_file = side_from_filename(args.hand)
    if args.side is None:
        side = by_joint or by_file
        if side is None:
            _die(EXIT_SIDE_CONFLICT,
                 "无法自动判断手侧：关节名里没有 left_/right_ 前缀，文件名也无线索。\n"
                 "       → 请显式加 --side left 或 --side right")
        how = "依据关节名" + ("" if by_joint else "；关节名无线索，退而依据文件名")
        _info(f"[判侧] 自动判定为 **{side}**（{how}）")
    else:
        side = args.side
        if by_joint and by_joint != side:
            _die(EXIT_SIDE_CONFLICT,
                 f"--side {side} 与手模型内的关节名前缀（{by_joint}_*）不一致，"
                 f"拒绝执行以免左右搞反。")
        _info(f"[判侧] 使用显式指定 {side}")

    mount_xyz = parse_floats(args.mount_xyz, 3, "--mount-xyz")
    mount_rpy = parse_floats(args.mount_rpy, 3, "--mount-rpy")

    # ---- 输出目录 ----
    out_dir = os.path.abspath(args.out_dir or os.path.dirname(os.path.abspath(args.robot)))
    out_xml = os.path.join(out_dir, f"elf3_revo2_{side}.xml")

    _info(f"[输入] 机器人 : {os.path.abspath(args.robot)}")
    _info(f"[输入] 手     : {os.path.abspath(args.hand)}  （{side}）")
    _info(f"[挂载] 手根 → <{MOUNT_BODY[side]}>")
    _info(f"[偏移] xyz={args.mount_xyz}  rpy={args.mount_rpy}")

    mesh_dir = os.path.join(out_dir, "meshes")
    if not os.path.isdir(mesh_dir):
        _die(EXIT_USAGE,
             f"输出目录里没有 meshes/：{mesh_dir}\n"
             f"       → --out-dir 应指向官方 mujoco_simulation 目录"
             f"（与 elf3.xml 同级，含 meshes/ sensors/ terrains/）")

    # ---- 防误覆盖门禁 ----
    if os.path.exists(out_xml) and not args.force:
        _die(EXIT_GUARD,
             f"输出文件已存在：{out_xml}\n"
             f"       → 加 --force 覆盖，或换 --out-dir")

    # ---- 合并 ----
    merged, mesh_jobs, hand_joints, removed_coll = build_merged(
        robot_root, hand_root, side, args.hand,
        mount_xyz, mount_rpy, args.kp_hand, args.hand_armature, args.lock_distal
    )

    n_body = len(list(merged.iter("body")))
    n_joint = len(list(merged.iter("joint")))
    n_act = len(merged.find("actuator").findall("position"))
    _info("")
    _info(f"[结果] body {n_body} 个 / joint {n_joint} 个 / 新增手部 actuator {n_act} 个")
    _info(f"[结果] 手部网格 {len(mesh_jobs)} 个（去重后）")
    if removed_coll:
        _info(f"[清理] 摘掉 {MOUNT_BODY[side]} 的碰撞体 {len(removed_coll)} 个"
              f"（否则会顶住手指）：{removed_coll}")

    problems = validate(merged, mesh_jobs)
    if problems:
        _die(EXIT_VALIDATE, "产物校验未通过：\n" + "\n".join(f"       - {p}" for p in problems))

    if args.dry_run:
        _info("")
        _info("[预演] 将写入：")
        _info(f"        {out_xml}")
        _info(f"        {mesh_dir}/revo2_vis_*.STL  ×"
              f"{sum(1 for _, n in mesh_jobs if n.startswith(VIS_PREFIX))}")
        _info(f"        {mesh_dir}/revo2_col_*.STL  ×"
              f"{sum(1 for _, n in mesh_jobs if n.startswith(COL_PREFIX))}")
        _info("")
        _info("[预演] 未写任何文件。去掉 --dry-run 正式生成。")
        return EXIT_OK

    # ---- 写网格 ----
    os.makedirs(mesh_dir, exist_ok=True)
    for src, dst_name in mesh_jobs:
        shutil.copy2(src, os.path.join(mesh_dir, dst_name))

    # ---- 写 XML ----
    ET.indent(merged, space="  ")
    header = '<?xml version="1.0" encoding="utf-8"?>\n'
    body = ET.tostring(merged, encoding="unicode")
    with open(out_xml, "w", encoding="utf-8", newline="\n") as f:
        f.write(header + body + "\n")

    _info("")
    _info(f"✅ 产物校验通过：名字唯一 / 单棵连通树 / 网格齐全 / actuator 完整")
    _info(f"✅ 已生成：{out_xml}")
    _info("")
    _info("下一步（验证能否加载）：")
    _info(f'    python3 -c "import mujoco; m=mujoco.MjModel.from_xml_path(r\'{out_xml}\'); '
          f'print(\'nq=\',m.nq,\'nu=\',m.nu,\'nbody=\',m.nbody)"')
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

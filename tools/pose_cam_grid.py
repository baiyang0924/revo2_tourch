#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_cam_grid.py —— 离线「姿态 × 机位」网格渲染器（不需要跑仿真）

【为什么需要它】
远程观察者录出来的视频里，右臂动作"看不出在动"。原因只可能是两个：
  (a) 机位不对 —— 手臂和躯干轮廓重叠，白色叠白色；
  (b) 幅度太小 —— 位移小于画面可辨识尺度。
靠肉眼试错太慢。这个脚本把机器人**离线**摆成若干指定姿态，
用若干候选机位各渲一张，拼成网格图，一次看完再决定。

【它不做什么】
不订阅任何 ROS 话题、不发任何指令、不写任何状态、不改官方 XML。
纯读模型 + 离屏渲染，跑一次几秒钟。

【额外产出】
打印右臂 / 腰 / 头各关节的**模型真实限位**（jnt_range），
用来核对文档里记的限位是不是官方 XML 的真值。

用法：
  MUJOCO_GL=egl python3 pose_cam_grid.py \
      --model /path/to/elf3.xml \
      --out grid.png \
      --poses rest,reach_a,reach_b,reach_c \
      --azimuths 0,45,90,315
"""

import argparse
import os
import sys

import numpy as np
import mujoco

try:
    import cv2
except ImportError:
    print("[错误] 需要 opencv-python（cv2）")
    sys.exit(1)


# ---- 姿态库：关节名 → 相对 keyframe 0 的**绝对增量**（rad）----
# 只列右臂，其余关节保持 keyframe 0 的值。
#
# 【坐标系事实】模型里机器人朝 **+x**；静止时右手腕在 y 为负的一侧（−0.178 m），
#   即"身体右侧 = −y"。所以：
#     相机 azimuth   0° → 从 +x 看  → 正面，双臂都在视野里
#     相机 azimuth  90° → 从 +y 看  → 机器人**左侧**（右臂被躯干完全挡住）★踩过的坑
#     相机 azimuth 315° → 从 +x,−y 看 → **右前三刻钟，右臂离相机最近** ✅
POSE_LIB = {
    "rest": {},
    # y 轴抬肩（矢状面内前摆）
    "reach_y": {"r_shoulder_y_joint": 0.60, "r_elbow_y_joint": -0.50},
    # shoulder_z 正 / 负，看哪个方向是"往外甩"
    "shz_pos": {"r_shoulder_z_joint": +0.90},
    "shz_neg": {"r_shoulder_z_joint": -0.90},
    # shoulder_x 负 / 正（右臂负向才是有大行程的那一侧，−175°）
    "shx_neg": {"r_shoulder_x_joint": -0.90},
    "shx_pos": {"r_shoulder_x_joint": +0.20},
    # 综合：抬肩 + 外展 + 屈肘 + 转腕
    "reach_a": {"r_shoulder_y_joint": 0.50, "r_shoulder_x_joint": -0.50,
                "r_elbow_y_joint": -0.60, "r_wrist_z_joint": 0.40},
    # 高举：主要靠 shoulder_z（绕臂轴，右臂大幅侧展只能靠它）
    "reach_c": {"r_shoulder_y_joint": 0.30, "r_shoulder_z_joint": 0.90,
                "r_elbow_y_joint": -0.40},
}

WATCH = [
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
    "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "head_z_joint", "head_y_joint",
]


def joint_map(model: mujoco.MjModel) -> dict:
    out = {}
    for j in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if nm:
            out[nm] = (int(model.jnt_qposadr[j]), int(j), int(model.jnt_type[j]))
    return out


def print_limits(model: mujoco.MjModel, jm: dict) -> None:
    print()
    print("=" * 78)
    print("  关节真实限位（来自模型 jnt_range，单位 deg）")
    print("=" * 78)
    print(f"  {'关节名':<24}{'下限':>10}{'上限':>10}{'可动范围':>12}{'当前值':>10}")
    print("-" * 78)
    for nm in WATCH:
        if nm not in jm:
            print(f"  {nm:<24}{'（模型里没有）':>30}")
            continue
        _, jid, _ = jm[nm]
        lo, hi = model.jnt_range[jid]
        print(f"  {nm:<24}{np.degrees(lo):>10.1f}{np.degrees(hi):>10.1f}"
              f"{np.degrees(hi - lo):>12.1f}")
    print("=" * 78)
    print("  注：jnt_range 为 (0,0) 表示该关节**无软限位**。")
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", default="grid.png")
    ap.add_argument("--poses", default="rest,reach_a,reach_b,reach_c")
    ap.add_argument("--azimuths", default="0,45,90,315")
    ap.add_argument("--elevation", type=float, default=-12.0)
    ap.add_argument("--distance", type=float, default=4.2)
    ap.add_argument("--follow-z-offset", type=float, default=-0.35)
    ap.add_argument("--width", type=int, default=560)
    ap.add_argument("--height", type=int, default=560)
    ap.add_argument("--keyframe", type=int, default=0)
    args = ap.parse_args()

    if not os.path.isfile(args.model):
        print(f"[错误] 模型不存在：{args.model}")
        return 1

    print(f"[模型] {args.model}")
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    print(f"       nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} ncam={model.ncam} nkey={model.nkey}")

    jm = joint_map(model)
    for nm in ("world_joint",):
        if nm not in jm:
            print(f"[错误] 找不到 {nm}")
            return 1

    # 基座（keyframe 0 的站立位）
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, args.keyframe)
        print(f"       已载入 keyframe {args.keyframe}")
    else:
        mujoco.mj_resetData(model, data)
    qpos_base = np.array(data.qpos, dtype=np.float64).copy()

    print_limits(model, jm)

    # ---- 渲染器 ----
    old_w, old_h = model.vis.global_.offwidth, model.vis.global_.offheight
    model.vis.global_.offwidth = max(int(old_w), args.width)
    model.vis.global_.offheight = max(int(old_h), args.height)
    print(f"[渲染] 离屏帧缓冲 {old_w}x{old_h} → "
          f"{model.vis.global_.offwidth}x{model.vis.global_.offheight}")
    renderer = mujoco.Renderer(model, args.height, args.width)

    root_body = int(model.jnt_bodyid[0])
    root_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root_body)
    print(f"[相机] 视线中心 = body#{root_body} '{root_name}' "
          f"+ z{args.follow_z_offset:+.2f} m，距离 {args.distance} m，"
          f"俯仰 {args.elevation}°")

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance = args.distance
    cam.elevation = args.elevation
    cam.azimuth = 0.0

    poses = [p.strip() for p in args.poses.split(",") if p.strip()]
    azimuths = [float(a) for a in args.azimuths.split(",") if a.strip()]

    rows = []
    for pname in poses:
        if pname not in POSE_LIB:
            print(f"[跳过] 未知姿态 {pname}")
            continue
        delta = POSE_LIB[pname]

        # 摆姿势
        data.qpos[:] = qpos_base
        applied = []
        for k, dv in delta.items():
            if k not in jm:
                print(f"[警告] 模型里没有 {k}")
                continue
            qadr, jid, _ = jm[k]
            lo, hi = model.jnt_range[jid]
            target = data.qpos[qadr] + dv
            clamped = target
            if lo != 0.0 or hi != 0.0:
                clamped = min(max(target, lo), hi)
            data.qpos[qadr] = clamped
            applied.append((k, np.degrees(dv), np.degrees(clamped)))
        mujoco.mj_forward(model, data)

        print(f"\n[姿态] {pname}")
        for k, dv, c in applied:
            print(f"       {k:<24} 增量 {dv:+7.1f}°  →  末值 {c:+7.1f}°")

        # 关键点：右腕末端的世界坐标，用来量化"手到底挪了多远"
        tip = None
        for bn in ("r_wrist_z_link", "r_wrist_y_link", "r_hand_base_link",
                   "r_elbow_y_link"):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bn)
            if bid >= 0:
                tip = (bn, data.xpos[bid].copy())
                break
        if tip is not None:
            print(f"       末端参考 {tip[0]:<20} "
                  f"({tip[1][0]:+.3f}, {tip[1][1]:+.3f}, {tip[1][2]:+.3f}) m")
            if pname == poses[0]:
                ref_tip = tip[1].copy()
            else:
                d = tip[1] - ref_tip
                print(f"       相对 rest 位移  |Δ| = {np.linalg.norm(d) * 1000:.1f} mm"
                      f"   (Δx {d[0]*1000:+.0f}  Δy {d[1]*1000:+.0f}  "
                      f"Δz {d[2]*1000:+.0f} mm)")

        look = data.xpos[root_body] + np.array([0.0, 0.0, args.follow_z_offset])
        cam.lookat[:] = look

        tiles = []
        for az in azimuths:
            cam.azimuth = az
            renderer.update_scene(data, camera=cam)
            img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

            # 只在每行第一格标姿态名，每列第一行标方位角
            if az == azimuths[0]:
                cv2.putText(img, pname, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.85, (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(img, pname, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.85, (0, 255, 255), 2, cv2.LINE_AA)
            if pname == poses[0]:
                cv2.putText(img, f"az {az:g}", (12, 62), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(img, f"az {az:g}", (12, 62), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (120, 255, 120), 2, cv2.LINE_AA)
            cv2.line(img, (0, 1), (args.width, 1), (90, 90, 90), 2)
            cv2.line(img, (1, 0), (1, args.height), (90, 90, 90), 2)
            tiles.append(img)

        rows.append(np.hstack(tiles))

    if not rows:
        print("[错误] 没有可用姿态")
        return 1

    grid = np.vstack(rows)
    cv2.imwrite(args.out, grid)
    print(f"\n[完成] {len(rows)} 姿态 × {len(azimuths)} 机位 → {args.out}"
          f"  ({grid.shape[1]}x{grid.shape[0]})")
    renderer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

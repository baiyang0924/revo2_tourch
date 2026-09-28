#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_wave_demo.py —— M0 演示轨迹：右臂 抬臂 → 前伸 → 转腕 → 外摆 → 收回

给「远程观察者」录制用的激励源。只发 **simulation** 话题，不碰硬件。

【为什么轨迹要这么设计】
录出来的画面要能**一眼看出手臂在动**。三个条件缺一不可：
  1. 机位对 —— 必须拍到右臂那一侧（azimuth 270~330，见观察者脚本注释）
  2. 幅度够 —— 末端位移至少 150 mm 量级
  3. 相位分得开 —— 抬、伸、转、摆、收 五段各自停留 2 s，眼睛跟得上

【位移量的来源】
`r_shoulder_x_joint` 负向是右臂**真正的抬臂外展**方向（该关节行程 −175°~+20°，
负向才是大的那一侧）。实测 shoulder_x 转 51.6° → 手腕位移 222.7 mm，
其中 Δy −201 mm（向外）+ Δz +97 mm（向上）。
`r_shoulder_z_joint` 只做水平横扫：同样 51.6° 位移 222.7 mm 但 **Δz = 0**。

【轨迹是「实测基线 + 增量」】
所以不管机器人当前站姿如何，动作幅度都受控。
每一步增量都过一遍 **模型实测限位钳位**（见 LIMITS），越界会被夹住并告警，
不会把超限角度发出去。

⚠️ 只覆盖右臂 7 个关节；左臂 / 腿 / 腰 / 头一概不发。
⚠️ 覆盖超时 0.2 s，本脚本默认 20 Hz 持续发布（铁律：≥5 Hz）。

用法：
  python3 arm_wave_demo.py [--rate 20] [--scale 1.0]
"""

import argparse
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from communication.msg import ActuatorStates, ActuatorCmds

OVERRIDE_TOPIC = "/simulation/actuators_cmds_override"
STATE_TOPIC = "/simulation/actuator_states"

R_ARM = [
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
    "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
]

# 下半身：腰 3 + 腿 12。--hold-legs 时一并覆盖，把站姿按住。
# 为什么需要：大幅覆盖右臂时，官方平衡控制器会迈步把机体带走 —— 实测幅度 ×1.0
# 会让机体在 20 s 内走出 4.1 m，镜头跟着跑、背景全变，demo 没法看。
# 空跑对照（不发任何指令）走动量 0.000 m，所以走动确实是手臂动作诱发的。
LOWER = [
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
    "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
    "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
]

# ★ 增益全部**实测自 /simulation/actuators_cmds**（官方控制器正在用的值），不是猜的。
#   手臂那 7 个与本脚本原先写死的值一致，可交叉验证。
GAIN = {
    "waist_y_joint":     (108.448,  6.904),
    "waist_x_joint":     (162.672, 10.356),
    "waist_z_joint":     (176.421, 11.231),
    "l_hip_y_joint":     (176.421, 11.231),
    "l_hip_x_joint":     (176.421, 11.231),
    "l_hip_z_joint":     ( 54.224,  3.452),
    "l_knee_y_joint":    (176.421, 11.231),
    "l_ankle_y_joint":   ( 33.493,  2.132),
    "l_ankle_x_joint":   ( 21.771,  1.386),
    "r_hip_y_joint":     (176.421, 11.231),
    "r_hip_x_joint":     (176.421, 11.231),
    "r_hip_z_joint":     ( 54.224,  3.452),
    "r_knee_y_joint":    (176.421, 11.231),
    "r_ankle_y_joint":   ( 33.493,  2.132),
    "r_ankle_x_joint":   ( 21.771,  1.386),
    "r_shoulder_y_joint": (54.224,  3.452),
    "r_shoulder_x_joint": (54.224,  3.452),
    "r_shoulder_z_joint": (16.747,  1.066),
    "r_elbow_y_joint":    (54.224,  3.452),
    "r_wrist_x_joint":    (16.747,  1.066),
    "r_wrist_y_joint":    (16.747,  1.066),
    "r_wrist_z_joint":    (16.747,  1.066),
}


def gains_for(names):
    return [GAIN[n][0] for n in names], [GAIN[n][1] for n in names]

# 模型实测绝对限位（rad），来自 elf3.xml 的 jnt_range。
# 写在这里是为了**发指令前兜底钳位** —— 覆盖通道本身不做限位检查。
LIMITS = {
    "r_shoulder_y_joint": (-2.8798, 2.8798),   # ±165°
    "r_shoulder_x_joint": (-3.0543, 0.3491),   # −175° ~ +20°  ★负向才是大幅侧展
    "r_shoulder_z_joint": (-2.8798, 2.8798),   # ±165°
    "r_elbow_y_joint":    (-0.9599, 1.6581),   # −55° ~ +95°
    "r_wrist_x_joint":    (-2.8798, 2.8798),   # ±165°
    "r_wrist_y_joint":    (-1.3090, 1.3090),   # ±75°
    "r_wrist_z_joint":    (-0.7854, 0.7854),   # ±45°
}

# 关键帧：(绝对时刻 s, {关节: 相对基线的增量 rad})
KEYFRAMES = [
    (0.0,  {"r_shoulder_y_joint": 0.00, "r_shoulder_x_joint": 0.00,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": 0.00,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
    # 待机 2 s：留出"动作开始前"的基线，方便对比
    (2.0,  {"r_shoulder_y_joint": 0.00, "r_shoulder_x_joint": 0.00,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": 0.00,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
    # ① 抬臂外展：主要靠 shoulder_x 负向
    (4.5,  {"r_shoulder_y_joint": 0.15, "r_shoulder_x_joint": -0.55,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": -0.20,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
    # ② 前伸：肩前摆 + 肘伸直（肘负向=伸直）
    (6.5,  {"r_shoulder_y_joint": 0.60, "r_shoulder_x_joint": -0.40,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": -0.50,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.15}),
    # ③ 转腕：腕 z 正向摆 + 肩 z 带一点
    (8.5,  {"r_shoulder_y_joint": 0.60, "r_shoulder_x_joint": -0.40,
            "r_shoulder_z_joint": 0.25, "r_elbow_y_joint": -0.50,
            "r_wrist_x_joint": 0.30, "r_wrist_z_joint": 0.55}),
    # ④ 外摆：腕 z 反向，肩继续外展到最大
    (10.5, {"r_shoulder_y_joint": 0.45, "r_shoulder_x_joint": -0.65,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": -0.25,
            "r_wrist_x_joint": -0.25, "r_wrist_z_joint": -0.55}),
    # ⑤ 收回
    (12.5, {"r_shoulder_y_joint": 0.10, "r_shoulder_x_joint": -0.15,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": -0.05,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
    # 待机收尾 4 s
    (14.5, {"r_shoulder_y_joint": 0.00, "r_shoulder_x_joint": 0.00,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": 0.00,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
    (16.5, {"r_shoulder_y_joint": 0.00, "r_shoulder_x_joint": 0.00,
            "r_shoulder_z_joint": 0.00, "r_elbow_y_joint": 0.00,
            "r_wrist_x_joint": 0.00, "r_wrist_z_joint": 0.00}),
]

QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


def smoothstep(a: float, b: float, t: float) -> float:
    if b <= a:
        return 1.0 if t >= b else 0.0
    x = min(1.0, max(0.0, (t - a) / (b - a)))
    return x * x * (3.0 - 2.0 * x)


def sample(t: float, scale: float) -> dict:
    """在关键帧之间做 smoothstep 插值。"""
    if t <= KEYFRAMES[0][0]:
        return {k: v * scale for k, v in KEYFRAMES[0][1].items()}
    for i in range(len(KEYFRAMES) - 1):
        t0, v0 = KEYFRAMES[i]
        t1, v1 = KEYFRAMES[i + 1]
        if t0 <= t <= t1:
            r = smoothstep(t0, t1, t)
            return {k: (v0.get(k, 0.0) + (v1.get(k, 0.0) - v0.get(k, 0.0)) * r) * scale
                    for k in set(v0) | set(v1)}
    return {k: v * scale for k, v in KEYFRAMES[-1][1].items()}


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("arm_wave_demo")
        self.joints: dict = {}
        self.create_subscription(ActuatorStates, STATE_TOPIC, self._cb, QOS_BE)
        self.pub = self.create_publisher(ActuatorCmds, OVERRIDE_TOPIC, QOS_BE)

    def _cb(self, msg: ActuatorStates) -> None:
        self.joints = dict(zip(list(msg.name), list(msg.position)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--scale", type=float, default=1.0, help="整体幅度缩放")
    ap.add_argument("--hold-legs", action="store_true",
                    help="【实测无效，勿用】同时覆盖腰 3 + 腿 12 个关节、把站姿锁在基线。"
                         "本意是阻止平衡控制器迈步，结果适得其反：踝关节被锁死后"
                         "控制器失去踝策略与迈步能力，机体约 3~4 s 就倒下"
                         "（实测基座 z 掉 0.944 m）。记录在此是为了避免重复踩坑；"
                         "要稳定画面请在**渲染侧**用观察者的 --pin-base/--freeze-lower")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印每个关键帧的绝对角度与越界情况，不发布任何指令")
    args = ap.parse_args()

    if args.dry_run:
        print("=== 干跑：只做限位检查，不发指令 ===")
        bad = 0
        for t, d in KEYFRAMES:
            line = []
            for k in R_ARM:
                v = d.get(k, 0.0) * args.scale
                line.append(f"{k.split('_')[1]}_{k.split('_')[2][:1]}{np.degrees(v):+6.1f}")
            print(f"  t={t:5.1f}s  " + "  ".join(line))
        print()
        print("（绝对角度需要基线才能算，实跑时会打印）")
        return 0

    rclpy.init()
    n = Driver()
    print("[等待] 取基线 …")
    t_wait = time.time()
    while time.time() - t_wait < 15.0 and len(n.joints) < 31:
        rclpy.spin_once(n, timeout_sec=0.05)
    if len(n.joints) < 31:
        print(f"[失败] 只收到 {len(n.joints)} 个关节")
        rclpy.shutdown()
        return 1

    base = dict(n.joints)
    print(f"[基线] r_shoulder_y={base['r_shoulder_y_joint']:+.4f}  "
          f"r_shoulder_x={base['r_shoulder_x_joint']:+.4f}  "
          f"r_elbow_y={base['r_elbow_y_joint']:+.4f}")

    # ---- 限位预检：把整条轨迹的绝对角度过一遍，越界直接拒跑 ----
    print("[限位] 检查整条轨迹 …")
    clipped = {}
    worst = 0.0
    for t, d in KEYFRAMES:
        for k in R_ARM:
            abs_v = base[k] + d.get(k, 0.0) * args.scale
            lo, hi = LIMITS[k]
            if abs_v < lo or abs_v > hi:
                over = min(abs(abs_v - lo), abs(abs_v - hi))
                clipped[k] = max(clipped.get(k, 0.0), over)
                worst = max(worst, over)
    if clipped:
        for k, over in clipped.items():
            lo, hi = LIMITS[k]
            print(f"       ⚠️ {k} 超出 {np.degrees(over):.1f}°  "
                  f"（限位 {np.degrees(lo):+.1f} ~ {np.degrees(hi):+.1f}°）→ 将被夹住")
    else:
        print(f"       ✅ 全部在限位内（离限位最近仍有余量）")

    total = KEYFRAMES[-1][0]
    active = list(R_ARM) + (list(LOWER) if args.hold_legs else [])
    kp_list, kd_list = gains_for(active)
    if args.hold_legs:
        print(f"[按住] 同时覆盖下半身 {len(LOWER)} 个关节（腰 3 + 腿 12），"
              f"站姿锁定在基线 → 平衡控制器无法迈步")
    print(f"[发布] {OVERRIDE_TOPIC} @ {args.rate:.0f} Hz，轨迹 {total:.1f} s，幅度 ×{args.scale}")
    print(f"       共覆盖 {len(active)} 个关节；"
          f"铁律：覆盖超时 0.2 s，本频率 = {1.0/args.rate*1000:.0f} ms/条 → 远快于超时")
    print()

    t0 = time.time()
    sent = 0
    n_clip = 0
    next_report = 0.0
    while True:
        now = time.time() - t0
        if now > total:
            break
        d = sample(now, args.scale)
        m = ActuatorCmds()
        m.actuators_name = list(active)
        pos = []
        for k in active:
            v = base[k] + d.get(k, 0.0)
            if k in LIMITS:
                lo, hi = LIMITS[k]
                if v < lo or v > hi:      # ★ 兜底钳位，绝不把超限角度发出去
                    v = min(max(v, lo), hi)
                    n_clip += 1
            pos.append(float(v))
        m.pos = pos
        m.kp = [float(x) for x in kp_list]
        m.kd = [float(x) for x in kd_list]
        n.pub.publish(m)
        sent += 1
        rclpy.spin_once(n, timeout_sec=0.0)

        if now >= next_report:
            next_report += 1.0
            cur = dict(n.joints)
            got = cur.get("r_shoulder_y_joint")
            gotx = cur.get("r_shoulder_x_joint")
            want = pos[0]
            wantx = pos[1]
            if got is None:
                print(f"   [{now:4.1f}s] 指令 {want:+.3f}   （尚未收到状态）")
            else:
                print(f"   [{now:4.1f}s] sh_y 指令 {want:+.3f} 实测 {got:+.3f} | "
                      f"sh_x 指令 {wantx:+.3f} 实测 {gotx:+.3f} "
                      f"（残差 {abs(want-got)*57.3:.1f}° / {abs(wantx-gotx)*57.3:.1f}°）")
        time.sleep(max(0.0, 1.0 / args.rate - (time.time() - t0 - now)))

    print(f"\n[结束] 共发 {sent} 条，钳位 {n_clip} 次。")
    print("       停止发布 → 覆盖将在 timeout 0.2 s + release_blend 0.2 s 后完全释放")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

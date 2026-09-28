#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M0 验证探针 —— 手臂关节「覆盖通道」到底通不通。

要回答四个问题（每个都有硬判据，不看屏幕）：
  A. 指定关节是否真的朝目标动？
  B. 没点名的关节会不会被误动？
  C. 从第 1 条消息发出，到关节真的开始动，延迟多久？（接管延迟）
  D. 停止发布后多久完全回归策略输出？（释放延迟）

方法：
  1. 订阅 /simulation/actuator_states，取基线
  2. 只向 /simulation/actuators_cmds_override 发「右臂 7 关节 = 基线 + 小增量」
  3. 停发，继续观察

⚠️ 本脚本**只发 simulation 话题**，代码里没有任何 hardware 路径。
   目标角度 = 实测基线 + 增量，不是凭空填的数 —— 它只验证「通道」，
   不构成任何可上真机的抓取位姿。

用法：
  python3 m0_override_check.py [--hold 4.0] [--rate 20] [--delta 0.20]
"""

import argparse
import sys
import time

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
# 与 config/arm_poses.yaml 的 default_kp/kd 一致（官方 B 组，较柔和）
KP = [54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747]
KD = [3.452, 3.452, 1.066, 3.452, 1.066, 1.066, 1.066]

# 期望「纹丝不动」的对照组关节
CONTROL_JOINTS = [
    "waist_y_joint", "waist_x_joint",
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_elbow_y_joint",
    "r_hip_y_joint", "r_knee_y_joint",
    "head_z_joint",
]

# ★ 实测（ros2 topic info -v）：仿真侧 /simulation/actuator_states 与
#   /simulation/actuators_cmds_override **都是 BEST_EFFORT / VOLATILE**。
#   rclpy 默认 RELIABLE，会报
#     "offering incompatible QoS ... Last incompatible policy: RELIABILITY"
#   然后一条也收不到。必须显式把两端都设成 BEST_EFFORT 才匹配。
QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

MOVE_EPS = 0.01          # 判定「开始动了」的阈值（rad）
RETURN_EPS = 0.05        # 判定「已回归」的阈值（rad）


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("m0_override_check")
        self.latest: dict = {}
        self.n_seen = 0
        self.t_last: float = 0.0
        self.create_subscription(ActuatorStates, STATE_TOPIC, self._on_state, QOS_BE)
        self.pub = self.create_publisher(ActuatorCmds, OVERRIDE_TOPIC, QOS_BE)

    def _on_state(self, msg: ActuatorStates) -> None:
        self.latest = dict(zip(list(msg.name), list(msg.position)))
        self.n_seen += 1
        self.t_last = time.time()

    def wait_states(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if len(self.latest) >= 31:
                return True
        return False

    def drain(self, seconds: float) -> None:
        """持续 spin 指定时长，保持 latest 最新。"""
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.005)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", type=float, default=4.0, help="覆盖持续秒数")
    ap.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz")
    ap.add_argument("--delta", type=float, default=0.20, help="两个关节的增量 rad")
    args = ap.parse_args()

    rclpy.init()
    n = Probe()

    print("=" * 74)
    print(" M0 验证：手臂关节覆盖通道（/simulation/actuators_cmds_override）")
    print("=" * 74)
    print(f" 覆盖话题 : {OVERRIDE_TOPIC}")
    print(f" 状态话题 : {STATE_TOPIC}")
    print(f" 覆盖范围 : 右臂 7 关节（左臂 / 腿 / 腰 / 头 一律不发）")
    print()

    if not n.wait_states(10.0):
        print(f"[失败] 10 s 内只收到 {len(n.latest)} 个关节（期望 31）")
        rclpy.shutdown()
        return 1

    # 多等几帧让基线稳定
    n.drain(0.6)
    base = dict(n.latest)
    print(f"[基线] {len(base)} 个关节，共收到 {n.n_seen} 帧")

    target = {k: base[k] for k in R_ARM}
    target["r_shoulder_y_joint"] = base["r_shoulder_y_joint"] + args.delta
    target["r_elbow_y_joint"] = base["r_elbow_y_joint"] - args.delta

    print("\n[目标] 右臂覆盖值（= 实测基线 + 增量）")
    for k in R_ARM:
        mark = f"  ← {'+' if args.delta > 0 else ''}{args.delta:+.2f} 增量" \
            if k in ("r_shoulder_y_joint", "r_elbow_y_joint") else ""
        print(f"    {k:22s} {base[k]:+9.5f} → {target[k]:+9.5f}{mark}")

    watch = ("r_shoulder_y_joint", "r_elbow_y_joint")

    # ---------------- 发布阶段 ----------------
    print(f"\n[发布] {args.rate:.0f} Hz × {args.hold:.0f} s  ……")
    names = list(R_ARM)
    pos = [float(target[k]) for k in names]

    t0 = time.time()
    sent = 0
    next_report = 0.0
    t_first_move = None
    n_moved_frame = 0
    n_total_frame = 0
    held = None
    worst_err_hold = 0.0      # 持握段（t>1s）的最大残差 —— 用来判断覆盖有没有掉
    n_err_hold = 0

    while True:
        now = time.time() - t0
        if now > args.hold:
            break
        m = ActuatorCmds()
        m.actuators_name = list(names)
        m.pos = list(pos)
        m.kp = [float(x) for x in KP]
        m.kd = [float(x) for x in KD]
        n.pub.publish(m)
        sent += 1
        rclpy.spin_once(n, timeout_sec=0.0)

        if n.latest:
            n_total_frame += 1
            moved = max(abs(n.latest.get(k, 0.0) - base[k]) for k in watch)
            if moved > MOVE_EPS:
                n_moved_frame += 1
                if t_first_move is None:
                    t_first_move = now
            if t_first_move is not None and now > t_first_move + 0.5:
                # ★ 只在「已经接管成功」之后再统计，否则会把接管延迟误判成掉覆盖
                e = max(abs(n.latest.get(k, 0.0) - target[k]) for k in watch)
                worst_err_hold = max(worst_err_hold, e)
                n_err_hold += 1

        if now >= next_report:
            next_report += 0.5
            cur = dict(n.latest)
            err = max(abs(cur.get(k, 0.0) - target[k]) for k in watch)
            print(f"   [{now:4.1f}s] r_shoulder_y={cur.get('r_shoulder_y_joint', float('nan')):+8.4f}"
                  f"   r_elbow_y={cur.get('r_elbow_y_joint', float('nan')):+8.4f}"
                  f"   两关节最大残差={err:7.4f} rad")
        time.sleep(max(0.0, 1.0 / args.rate - (time.time() - t0 - now)))

    held = dict(n.latest)
    print(f"   § 已发 {sent} 条；其中有 {n_moved_frame}/{n_total_frame} 帧观察到关节已偏离基线")
    print(f"   § 持握段(接管成功后+0.5s起) 最大残差 = {worst_err_hold:.4f} rad"
          f"（{worst_err_hold * 57.2958:.2f}°），共 {n_err_hold} 帧"
          f"  ← 这个值大 = 覆盖中途真的掉了")

    # ---------------- 释放阶段 ----------------
    print("\n[停发] 观察释放（框架参数：timeout=0.200s, release_blend=0.200s）……")
    checkpoints = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.75, 1.00, 1.50, 2.00]
    obs = []
    t1 = time.time()
    idx = 0
    while idx < len(checkpoints):
        rclpy.spin_once(n, timeout_sec=0.002)
        dt = time.time() - t1
        if dt >= checkpoints[idx]:
            obs.append((dt, dict(n.latest)))
            idx += 1

    # ---------------- 汇总 ----------------
    print()
    print("=" * 74)
    print(" 结果")
    print("=" * 74)

    print("\n① 指定关节是否跟到目标（判据 A）")
    print(f"   {'关节':22s} {'基线':>10s} {'目标':>10s} {'持握末':>10s} {'残差':>9s}")
    ok_a = True
    for k in R_ARM:
        r = abs(held.get(k, 0.0) - target[k])
        flag = ""
        if k in watch:
            moved = abs(held.get(k, 0.0) - base[k])
            good = (r < abs(args.delta) * 0.5) and moved > 0.05
            ok_a &= good
            flag = "  ✓" if good else "  ✗"
        print(f"   {k:22s} {base[k]:+10.5f} {target[k]:+10.5f} {held.get(k, 0.0):+10.5f} {r:9.5f}{flag}")
    print(f"   → 残差来源：仿真用纯 PD，无重力补偿，稳态会有静差（与 MuJoCo 侧同一现象）")

    print("\n② 未点名的关节是否被误动（判据 B）")
    print(f"   {'关节':22s} {'基线':>10s} {'持握末':>10s} {'漂移':>10s}")
    worst_control, worst_name = 0.0, ""
    for k in CONTROL_JOINTS:
        if k not in base:
            continue
        d = abs(held.get(k, 0.0) - base[k])
        if d > worst_control:
            worst_control, worst_name = d, k
        print(f"   {k:22s} {base[k]:+10.5f} {held.get(k, 0.0):+10.5f} {d:10.5f}")
    print(f"   → 最大漂移 {worst_control:.5f} rad（{worst_control * 57.2958:.2f}°）@ {worst_name}")
    ok_b = worst_control < 0.10

    print("\n③ 接管延迟（判据 C：第 1 条消息 → 关节开始动）")
    if t_first_move is None:
        print("   ❌ 全程未观察到关节偏离基线")
        ok_c = False
    else:
        print(f"   → 实测 {t_first_move:.2f} s（含 DDS 匹配 + 框架接管混合）")
        print(f"   → 编排动作时这段必须计入，不能假设「发了就动」")
        ok_c = t_first_move < 2.0

    print("\n④ 释放延迟（判据 D：停发 → 完全回归策略）")
    print(f"   {'时刻':>8s} {'r_shoulder_y':>14s} {'r_elbow_y':>12s} {'相对基线残差':>14s}")
    t_start_release = None
    t_fully_back = None
    for dt, snap in obs:
        rs = snap.get("r_shoulder_y_joint", float("nan"))
        re_ = snap.get("r_elbow_y_joint", float("nan"))
        res = max(abs(rs - base["r_shoulder_y_joint"]), abs(re_ - base["r_elbow_y_joint"]))
        mark = ""
        if t_start_release is None and abs(dt - 0.20) < 0.03:
            t_start_release = (dt, res)
            mark = "  ← 官方 timeout=0.2s"
        elif t_fully_back is None and res < RETURN_EPS:
            t_fully_back = dt
            mark = "  ← 已回归 (<0.05 rad)"
        print(f"   {dt:7.2f}s {rs:+14.5f} {re_:+12.5f} {res:14.5f}{mark}")

    ok_d = (t_fully_back is not None) and (t_fully_back <= 0.60)
    if t_start_release:
        print(f"   → 0.2 s 时刻仍有 {t_start_release[1]:.4f} rad 残留："
              f"说明 0.2 s 只是「停止覆盖」，不是「已回位」")
    if t_fully_back:
        print(f"   → 完全回归用时 {t_fully_back:.2f} s ≈ timeout 0.2 s + release_blend 0.2 s")
    else:
        print("   → ⚠️ 2 s 内未观察到完全回归")

    print()
    print("=" * 74)
    print(f" 判据 A 指定关节跟到目标 : {'✅ 通过' if ok_a else '❌ 未通过'}")
    print(f" 判据 B 其他关节不被误动 : {'✅ 通过' if ok_b else '❌ 未通过'}"
          f"（最大 {worst_control * 57.2958:.2f}°）")
    print(f" 判据 C 接管延迟          : {'✅ 通过' if ok_c else '❌ 未通过'}")
    print(f" 判据 D 释放延迟          : {'✅ 通过' if ok_d else '❌ 未通过'}")
    print("=" * 74)

    n.destroy_node()
    rclpy.shutdown()
    return 0 if (ok_a and ok_b and ok_c and ok_d) else 2


if __name__ == "__main__":
    sys.exit(main())

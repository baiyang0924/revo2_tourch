#!/usr/bin/env python3
"""用 MuJoCo 手模型标定「OK 手势」的关节角。

OK 手势定义：拇指指尖触碰当前手指指尖，其余未参与的手指保持伸直。

方法：在手的运动学模型上做数值优化，最小化
      「拇指指尖」与「目标手指指尖」的距离，
      同时把不参与的手指锁在 0（伸直）。

输出：换算成 SDK 的 0~1000 位置值（Normalized 模式 1:1 对应关节行程比例）。
"""
from __future__ import annotations

import json
import sys

import mujoco
import numpy as np

MODEL = r"D:\sim\elf3_revo2_sim\elf3_revo2_right.xml"

# 关节 → (qpos 自由度名, ROM 上限弧度)  —— ROM 取自模型 jnt_range
J = {
    "thumb_mcp": ("right_thumb_metacarpal_joint", 1.5700),   # 对掌
    "thumb_prox": ("right_thumb_proximal_joint", 1.0300),    # 屈曲
    "index": ("right_index_proximal_joint", 1.4100),
    "middle": ("right_middle_proximal_joint", 1.4100),
    "ring": ("right_ring_proximal_joint", 1.4100),
    "pinky": ("right_pinky_proximal_joint", 1.4100),
}
TIP_BODY = {
    "index": "right_index_tip_link",
    "middle": "right_middle_tip_link",
    "ring": "right_ring_tip_link",
    "pinky": "right_pinky_tip_link",
}
THUMB_TIP = "right_thumb_tip_link"

ALL_JOINTS = ["thumb_prox", "thumb_mcp", "index", "middle", "ring", "pinky"]


def to_sdk(q_rad: dict) -> list[int]:
    """关节弧度 → SDK 0~1000（按各自 ROM 比例）"""
    out = []
    for k in ALL_JOINTS:
        name, rom = J[k]
        v = q_rad.get(k, 0.0)
        out.append(int(round(max(0.0, min(1.0, v / rom)) * 1000)))
    return out


def main() -> int:
    m = mujoco.MjModel.from_xml_path(MODEL)
    d = mujoco.MjData(m)

    adr = {k: int(m.jnt_qposadr[mujoco.mj_name2id(
        m, mujoco.mjtObj.mjOBJ_JOINT, jn)]) for k, (jn, _) in J.items()}
    bid = {k: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, v)
           for k, v in TIP_BODY.items()}
    thumb_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, THUMB_TIP)

    def tips(q: dict):
        for k in ALL_JOINTS:
            d.qpos[adr[k]] = q.get(k, 0.0)
        mujoco.mj_forward(m, d)
        return d.xpos[thumb_bid].copy(), {k: d.xpos[b].copy() for k, b in bid.items()}

    def dist(q: dict, tgt: str) -> float:
        th, fs = tips(q)
        return float(np.linalg.norm(th - fs[tgt]))

    result = {}
    print("=== 逐指求解「拇指指尖触碰该指指尖」（其余手指伸直）===")
    for tgt in ("index", "middle", "ring", "pinky"):
        lo = np.array([0.0, 0.0, 0.0])
        hi = np.array([J["thumb_mcp"][1], J["thumb_prox"][1], J[tgt][1]])

        best_q, best_c = None, None
        rng = np.random.default_rng(20260924)
        # 粗搜：随机采样
        for _ in range(40000):
            c = rng.uniform(lo, hi)
            q = {"thumb_mcp": c[0], "thumb_prox": c[1], tgt: c[2]}
            v = dist(q, tgt)
            if best_c is None or v < best_c:
                best_c, best_q = v, c.copy()
        # 细搜：围绕当前最优做递减步长的坐标扰动
        step = hi * 0.06
        for _ in range(600):
            improved = False
            for i in range(3):
                for sgn in (+1, -1):
                    c = best_q.copy()
                    c[i] = min(max(c[i] + sgn * step[i], lo[i]), hi[i])
                    q = {"thumb_mcp": c[0], "thumb_prox": c[1], tgt: c[2]}
                    v = dist(q, tgt)
                    if v < best_c - 1e-9:
                        best_c, best_q, improved = v, c, True
            if not improved:
                step *= 0.55
                if float(np.max(step)) < 1e-6:
                    break

        q = {"thumb_mcp": float(best_q[0]), "thumb_prox": float(best_q[1]),
             tgt: float(best_q[2])}
        sdk = to_sdk(q)
        result[tgt] = {"rad": [round(float(x), 4) for x in q.values()],
                       "sdk": sdk, "tip_gap_mm": round(best_c * 1000, 1)}
        print(f"  {tgt:<7} 指尖距 {best_c*1000:6.1f} mm   "
              f"thumb_mcp={np.degrees(best_q[0]):5.1f}° "
              f"thumb_prox={np.degrees(best_q[1]):5.1f}° "
              f"{tgt}={np.degrees(best_q[2]):5.1f}°  → SDK {sdk}")

    print("\n=== 汇总（可直接下发的 6 元组）===")
    for tgt, v in result.items():
        print(f"  OK(拇指+{tgt:<7}) = {v['sdk']}   (仿真指尖距 {v['tip_gap_mm']} mm)")
    print(f"\n  完全张开      = {to_sdk({})}")

    with open(r"D:\sim\out\ok_gesture.json", "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print("\n已保存 D:\\sim\\out\\ok_gesture.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())

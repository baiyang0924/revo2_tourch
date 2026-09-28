#!/usr/bin/env python3
"""拇指 v2 映射的合成数据测试：不连机器人、不开摄像头，纯几何验证。

用例：
  A 右手、掌心对摄像头、拇指张开   → aux 角应为负、norm≈0
  B 右手、掌心对摄像头、拇指贴掌   → aux 角应为正、norm≈1
  C 右手、手背对摄像头、拇指张开   → 与 A 同号（姿态翻转不变性）
  D 右手、手背对摄像头、拇指贴掌   → 与 B 同号
  E 只弯拇指 IP（掌骨方向不变）    → 拇指屈明显变大、aux 基本不动（解耦）
  F 四指伸直 vs 弯曲              → 弯曲度 0 vs 接近 1
  G to_norm 两端点与反向区间       → 0 / 1 / 自动方向
"""
import importlib.util
import math
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
# 仓库里叫 revo2_hand_teleop_client.py，本机调试目录叫 hand_teleop_client.py，都认
for _name in ("revo2_hand_teleop_client.py", "hand_teleop_client.py"):
    _p = os.path.join(_HERE, _name)
    if os.path.exists(_p):
        break
spec = importlib.util.spec_from_file_location("client", _p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class P:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


def base_hand(mirror: bool) -> list:
    """右手 21 点骨架（y 向下为正，手指朝上）。mirror=True 表示手背对摄像头。"""
    s = -1.0 if mirror else 1.0
    pts = [
        P(0.00, 0.00),                                    # 0 腕
        P(-0.30 * s, -0.45), P(0, 0, 0), P(0, 0, 0), P(0, 0, 0),   # 1-4 拇指（后三点按姿势填）
        P(-0.12 * s, -1.00), P(-0.14 * s, -1.35), P(-0.16 * s, -1.65), P(-0.18 * s, -1.90),  # 食指
        P(0.02, -1.05), P(0.02, -1.42), P(0.02, -1.72), P(0.02, -1.98),                      # 中指
        P(0.15 * s, -1.00), P(0.16 * s, -1.34), P(0.16 * s, -1.62), P(0.16 * s, -1.86),      # 无名指
        P(0.27 * s, -0.92), P(0.29 * s, -1.20), P(0.30 * s, -1.42), P(0.31 * s, -1.62),      # 小指
    ]
    return pts


def straight_thumb(pts, open_: bool):
    # 拇指在食指根的同一侧并向外伸：食指根在负 x 侧，拇指坐标就按写好的负 x 用
    s = 1.0 if pts[5].x < 0 else -1.0
    if open_:                            # 张开：往拇指侧伸出去
        pts[1] = P(-0.30 * s, -0.45)
        pts[2] = P(-0.72 * s, -0.28)
        pts[3] = P(-1.00 * s, -0.22)
        pts[4] = P(-1.25 * s, -0.18)
    else:                                # 贴掌：掌骨转过来，指尖落在食指/中指根附近（IP 仍直）
        pts[1] = P(-0.30 * s, -0.45)
        pts[2] = P(-0.10 * s, -0.90)
        pts[3] = P(0.02 * s, -1.30)
        pts[4] = P(0.10 * s, -1.65)
    return pts


def curled_thumb(pts):
    """在「张开」姿势上只弯 IP 关节（掌骨方向 CMC→MCP 不动）。"""
    pts = straight_thumb(pts, True)
    s = -1.0 if pts[5].x < 0 else 1.0
    pts[3] = P(-0.92 * s, -0.50)         # IP 绕过去
    pts[4] = P(-0.85 * s, -0.75)         # 指尖钩回来
    return pts


def norm_aux(pts, rng):
    nat = m.native_values(pts)
    return m.to_norm(nat[1], rng[1][0], rng[1][1])


def approx(a, b, tol):
    return abs(a - b) <= tol


fails = []
def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name + ("" if cond else f"   [{detail}]"))
    if not cond:
        fails.append(name)


rng = [list(r) for r in m.DEFAULT_RANGE]

# A/B 掌心对摄像头
a = straight_thumb(base_hand(False), True)
aux_a = m.thumb_aux_deg(a)
check("A 张开 aux 为负且 norm≈0", aux_a < -40 and norm_aux(a, rng) < 0.15, f"aux={aux_a:.1f}")
b = straight_thumb(base_hand(False), False)
aux_b = m.thumb_aux_deg(b)
check("B 贴掌 aux 为正且 norm≈1", aux_b > 0 and norm_aux(b, rng) > 0.8, f"aux={aux_b:.1f}")

# C/D 手背对摄像头：姿态翻转后符号必须不变
c = straight_thumb(base_hand(True), True)
aux_c = m.thumb_aux_deg(c)
check("C 背面张开 与 A 同号且 norm≈0", aux_c < -40 and norm_aux(c, rng) < 0.15, f"aux={aux_c:.1f}")
d = straight_thumb(base_hand(True), False)
aux_d = m.thumb_aux_deg(d)
check("D 背面贴掌 与 B 同号且 norm≈1", aux_d > 0 and norm_aux(d, rng) > 0.8, f"aux={aux_d:.1f}")

# E 解耦：只弯 IP，aux 基本不动、拇指屈明显上升
e = curled_thumb(base_hand(False))
aux_e = m.thumb_aux_deg(e)
flex_open = m.thumb_flex(straight_thumb(base_hand(False), True))
flex_curled = m.thumb_flex(e)
check("E1 弯 IP 后 aux 变化 < 15°", abs(aux_e - aux_a) < 15, f"aux {aux_a:.1f}→{aux_e:.1f}")
check("E2 弯 IP 后拇指屈 ≥ 0.45", flex_curled >= 0.45, f"{flex_open:.2f}→{flex_curled:.2f}")
check("E3 伸直拇指屈 < 0.15", flex_open < 0.15, f"{flex_open:.2f}")

# F 四指
f_open = m.finger_curl(base_hand(False), 5, 6, 7, 8)
f_pts = base_hand(False)
f_pts[8] = P(-0.02, -1.15)   # 食指整段折回来（tip 贴近 MCP）
f_curled = m.finger_curl(f_pts, 5, 6, 7, 8)
check("F1 四指伸直 ≈ 0", f_open < 0.1, f"{f_open:.2f}")
check("F2 四指弯曲 ≥ 0.7", f_curled >= 0.7, f"{f_curled:.2f}")

# G to_norm：端点与反向区间
check("G1 open→0", m.to_norm(10.0, 10.0, 90.0) == 0.0)
check("G2 closed→1", m.to_norm(90.0, 10.0, 90.0) == 1.0)
check("G3 反向区间 closed(小)→1", m.to_norm(-70.0, 15.0, -55.0) == 1.0)
check("G4 超界截断", m.to_norm(200.0, 10.0, 90.0) == 1.0 and m.to_norm(-5.0, 10.0, 90.0) == 0.0)

# H 标定方向自适配：假设某种手/标定给出「闭合值比张开值还小」的反向量程
rev = [list(r) for r in rng]
rev[1] = [-20.0, -80.0]     # open=-20, closed=-80（反向）
check("H 反向量程：闭合值→1、张开值→0",
      m.to_norm(-80.0, rev[1][0], rev[1][1]) == 1.0 and m.to_norm(-20.0, rev[1][0], rev[1][1]) == 0.0,
      "open=-20 closed=-80")

print()
if fails:
    print(f"未通过 {len(fails)} 项：{fails}")
    raise SystemExit(1)
print("全部通过 ✓")

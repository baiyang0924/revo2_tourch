#!/usr/bin/env python3
"""ELF3 + Revo2 右手 —— 实时遥控仿真窗口。

打开即弹出三维窗口：机器人站立于桌前，桌面放置水瓶。
可用键盘或命令行指令控制「抬臂角度、握拳角度」，窗口实时跟随。

启动（场景文件由 tools/revo2_pick_bottle_demo.py 生成，需用 --model 指定）：

    python teleop_arm_hand.py --model <场景文件路径>

    # 场景未生成时，先跑一次离线 demo（任一模式都会写出 scene_bottle.xml）：
    python revo2_pick_bottle_demo.py --model <合并模型路径> --mode combo --no-record

常用指令（在启动终端输入，回车生效）：
    arm 60        抬起手臂 60 度（0 = 自然下垂）
    hand 100      手指握拳 100%（0 = 张开）
    reach         把手伸到瓶子上方
    down          下降到抓取高度
    grasp         握紧（抓住瓶子）
    lift          把瓶子抬起来
    auto          一键跑完整流程：抬臂 → 伸手 → 抓 → 抬起
    home          回到起始位
    status        打印当前角度
    q             退出

键盘（先点一下三维窗口使其获得焦点）：
    ↑/↓  抬手臂 ±5°      ←/→  手臂横扫 ±5°     W/S  手肘 ±5°
    [ ]  握拳 ∓10%        G 握紧   O 张开       R 回起始位
    1 伸到瓶上方  2 下降  3 抬起   A 完整流程   H 帮助
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from elf3rig import (  # noqa: E402
    ARM_JOINTS, BOTTLE_AXIS_IN_HAND, HAND_GRASP, HAND_OPEN, HAND_Y_AXIS,
    ArmIK, Rig, home_arm_q,
)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

# 起始位手根目标：必须远离桌面工作区。
# ⚠️ 不能直接用「零位关节角」——实测它的手根落在 [0.276,-0.231,0.838]，
#    离桌上瓶子只有 30 mm（瓶半径 32.5 mm），手臂一开局就压在瓶子上，一动就崩飞。
HOME_HAND_TARGET = np.array([-0.04, -0.06, 0.98])
APPROACH_HIGH = np.array([0.11, -0.24, 1.20])

# 手根相对肩的「抬起」关节：shoulder_x 负向 = 外展抬起（限位 -175°~+20°）
LIFT_IDX = 1

# 命令到关节的映射（用户给的度数 → 关节弧度）
JOINT_CMD = {
    "sy": (0, "r_shoulder_y 前后摆"),
    "sx": (1, "r_shoulder_x 侧摆（负=抬起）"),
    "sz": (2, "r_shoulder_z 横扫"),
    "elbow": (3, "r_elbow_y 肘屈伸"),
    "wx": (4, "r_wrist_x"),
    "wy": (5, "r_wrist_y"),
    "wz": (6, "r_wrist_z"),
}

HELP = """
命令（度 / 百分比，回车生效）
  arm <0~175>     抬起手臂多少度（最常用；0=垂下）
  sy/sx/sz <deg>  肩关节 前后 / 侧摆 / 横扫（sx 负值是抬起）
  elbow <deg>     手肘屈伸（-55~95）
  wx/wy/wz <deg>  手腕三轴
  hand <0~100>    手指握拳百分比（0=张开 100=握紧）
  finger <deg>    六指统一给一个角度
  ---------------- 一键动作 ----------------
  reach   伸到瓶子上方      down   下降到抓取高度
  grasp   握紧（抓瓶）      open   张开手
  lift    把瓶子抬起来      home   回自然下垂
  auto    完整流程：抬臂→伸手→下降→抓→抬起
  status  打印当前状态      help   这张表      q  退出
键盘（点 3D 窗口获得焦点）
  ↑/↓ 抬臂±5°   ←/→ 横扫±5°   W/S 肘±5°   [ ] 握拳∓10%
  G 握紧  O 张开  R 回起始  1 伸到瓶上方  2 下降  3 抬起  A 完整流程
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def smoothstep(s: float) -> float:
    s = clamp(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


# ---------------------------------------------------------------------------
# 抓取位姿求解：多起点 IK，按「推算瓶心 vs 桌上真瓶」的偏差挑最优
# ---------------------------------------------------------------------------


def solve_grasp_q(rig: Rig, bottle_pos: np.ndarray,
                  u_guess=(0.035, 1.0, 0.0), n: int = 48, iters: int = 160):
    """解出能让手真正包住桌上那个瓶子的 7 个右臂关节角。

    手根 Y 轴必须竖直（瓶轴才竖直），但绕瓶轴的滚转是自由的 —— 同一个手根
    位置对应无穷多种握法。所以多起点求解后按「瓶心偏差 + 手腕别扭程度」挑。
    """
    m, d = rig.m, rig.d
    ik = ArmIK(rig)
    u = np.asarray(u_guess, dtype=float)
    u = u / (np.linalg.norm(u) + 1e-12)
    target0 = bottle_pos - 0.085 * u
    t = ik._make_targets(target0, None, HAND_Y_AXIS, u)

    rng = np.random.default_rng(7)
    lo = np.maximum(rig.arm_rng[:, 0], -2.2)
    hi = np.minimum(rig.arm_rng[:, 1], 2.2)
    seeds = [home_arm_q()] + [rng.uniform(lo, hi) for _ in range(n)]

    best = None
    saved = d.qpos.copy()
    for s in seeds:
        q = np.clip(ik._iterate(s.copy(), t, iters),
                    rig.arm_rng[:, 0], rig.arm_rng[:, 1])
        d.qpos[rig.arm_qadr] = q
        mujoco.mj_forward(m, d)
        p, R = rig.hand_pose()
        ap = p + R @ BOTTLE_AXIS_IN_HAND
        e_pos = float(np.linalg.norm(target0 - p))
        e_axis = float(np.linalg.norm(HAND_Y_AXIS - R[:, 1]))
        err = float(np.linalg.norm(ap - bottle_pos))
        # 偏好：瓶心对准 > 手根到位 > 轴向竖直 > 手腕别扭太狠
        cost = err * 1000.0 + e_pos * 400.0 + e_axis * 60.0 \
            + 0.30 * float(np.sum(np.abs(q[4:7])))
        if best is None or cost < best[0]:
            best = (cost, q.copy(), err, e_pos)
    d.qpos[:] = saved
    mujoco.mj_forward(m, d)
    return best[1], best[2], best[3]


# ---------------------------------------------------------------------------
# 主控
# ---------------------------------------------------------------------------


class Teleop:
    def __init__(self, scene: str, cam=None) -> None:
        self.rig = Rig(scene)
        self.m, self.d = self.rig.m, self.rig.d
        self.ik = ArmIK(self.rig)

        mujoco.mj_resetData(self.m, self.d)
        self.rig.pin_base()
        self.bottle0 = self.rig.bottle_pos().copy()

        # 起始位 = IK 解出来的远离桌面的位姿（不是零位关节角）
        q_home, _, _ = self.ik.solve(HOME_HAND_TARGET, q_seed=home_arm_q(),
                                     axis_y=HAND_Y_AXIS, restarts=24)
        self.q_home = np.clip(q_home, self.rig.arm_rng[:, 0], self.rig.arm_rng[:, 1])

        self.arm_cmd = self.q_home.copy()       # 目标关节角（弧度）
        self.arm_act = self.q_home.copy()       # 实际下发给 PD 的（一阶滤波）
        self.hand_cmd = HAND_OPEN.copy()        # 目标手指角
        self.hand_act = HAND_OPEN.copy()
        self.hold_pct = 0.0                     # 握拳百分比（显示用）

        self.cart = None                        # 笛卡尔移动任务
        self.servo_target = None
        self.grabbed = False
        self.t_sim = 0.0
        self._last_ik = -1.0
        self.cmdq: queue.Queue[str] = queue.Queue()
        self.stop = False
        self.trace = False
        self._last_trace = -1.0
        self.cam = cam

        # 碰撞分组：手和瓶子不互相碰撞，瓶子仍然被桌面托着。
        # 原因：官方 Revo2 手的碰撞网格与 Ø65 mm 瓶互相嵌入约 40 mm（三种手型都嵌），
        # 接触求解器对深嵌入会给出巨大的弹开力 —— 实测手一靠近瓶子就被崩飞 800 mm。
        # 这不是调参能解决的，所以抓取改由「合拢瞬间把瓶子锁在手根上」实现（同 demo）。
        #   碰撞条件 = (contype1 & conaffinity2) || (contype2 & conaffinity1)
        #   瓶 2/2   桌地 1/3   机器人 1/1  →  瓶×手 不碰，瓶×桌 碰，手×桌 碰
        m = self.m
        for i in range(m.ngeom):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            if n.startswith("bottle"):
                m.geom_contype[i] = 2
                m.geom_conaffinity[i] = 2
            elif n.startswith("table") or n == "floor":
                m.geom_contype[i] = 1
                m.geom_conaffinity[i] = 3
            else:
                m.geom_contype[i] = 1
                m.geom_conaffinity[i] = 1

        self.d.qpos[self.rig.arm_qadr] = self.arm_cmd
        mujoco.mj_forward(self.m, self.d)
        hp = self.rig.hand_pose()[0]
        self.home_gap = float(np.linalg.norm(hp - self.bottle0))

    # ---- 底层 ----
    def set_arm(self, idx: int, deg: float) -> str:
        lo, hi = self.rig.arm_rng[idx]
        self.arm_cmd[idx] = clamp(deg * D2R, lo, hi)
        self.cart = None
        return "%s = %.1f°（限位 %.0f~%.0f）" % (
            ARM_JOINTS[idx], deg, lo * R2D, hi * R2D)

    def set_lift(self, deg: float) -> str:
        deg = clamp(deg, 0.0, 175.0)
        self.arm_cmd[LIFT_IDX] = -deg * D2R
        self.cart = None
        return "手臂抬起 %.1f°（shoulder_x = %.1f°）" % (deg, -deg)

    def set_hand_pct(self, pct: float) -> str:
        pct = clamp(pct, 0.0, 100.0)
        self.hold_pct = pct
        self.hand_cmd = HAND_OPEN + (HAND_GRASP - HAND_OPEN) * (pct / 100.0)
        return "握拳 %d%%（六指目标 %s 度）" % (
            int(pct), np.round(np.degrees(self.hand_cmd), 1).tolist())

    def set_hand_deg(self, deg: float) -> str:
        self.hand_cmd = np.clip(np.full(6, deg * D2R), 0.0, 1.57)
        self.hold_pct = 100.0 * float(np.mean(self.hand_cmd)) / float(np.mean(HAND_GRASP))
        self.hold_pct = clamp(self.hold_pct, 0.0, 100.0)
        return "六指各 %.1f°（约等于握拳 %d%%）" % (deg, int(self.hold_pct))

    def goto_home(self) -> str:
        self.cart = None
        self.arm_cmd = self.q_home.copy()
        self.set_hand_pct(0.0)
        self.grabbed = False
        return "回到自然下垂，手张开"

    # ---- 轨迹规划：离线逐点 IK（每段以上一段为 seed，保证连续不跳解）----
    def plan_path(self, p0: np.ndarray, p1: np.ndarray,
                  segs: int = 14, seed: np.ndarray | None = None) -> np.ndarray:
        rig, m, d = self.rig, self.m, self.d
        saved = d.qpos.copy()
        q = rig.arm_q().copy() if seed is None else np.asarray(seed, float).copy()
        az = getattr(self, "u_des", None)
        qs: list[np.ndarray] = []
        for k in range(segs + 1):
            s = k / segs
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * s
            q, _, _ = self.ik.solve(p, q_seed=q, axis_y=HAND_Y_AXIS, axis_z=az,
                                    restarts=0, iters=120)
            q = np.clip(q, rig.arm_rng[:, 0], rig.arm_rng[:, 1])
            qs.append(q.copy())
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)
        return np.array(qs)

    def _start_traj(self, name: str, qs: np.ndarray, p1: np.ndarray,
                    dur: float, hand_pct: float | None = None) -> None:
        self.servo_target = np.asarray(p1, float).copy()
        self.cart = {"name": name, "qs": np.asarray(qs), "p1": np.asarray(p1, float),
                     "t0": self.t_sim, "dur": dur}
        if hand_pct is not None:
            self.set_hand_pct(hand_pct)

    def _ensure_grasp(self) -> str:
        """解出抓取位（只算一次并缓存）。"""
        if hasattr(self, "p_grasp"):
            return ""
        q, err, epos = solve_grasp_q(self.rig, self.bottle0)
        self.q_grasp = q
        saved = self.d.qpos.copy()
        self.d.qpos[self.rig.arm_qadr] = q
        mujoco.mj_forward(self.m, self.d)
        self.p_grasp, _ = self.rig.hand_pose()
        self.d.qpos[:] = saved
        mujoco.mj_forward(self.m, self.d)
        # 手指伸出方向（手根 Z 轴）：从手根指向瓶心，投影到水平面
        u = self.bottle0 - self.p_grasp
        u[2] = 0.0
        self.u_des = u / (np.linalg.norm(u) + 1e-12)
        return "IK 抓取位：瓶心偏差 %.1f mm / 手根误差 %.1f mm" % (
            err * 1000, epos * 1000)

    def cmd_reach(self) -> str:
        msg = self._ensure_grasp()
        p0, _ = self.rig.hand_pose()
        p_hover = self.p_grasp + np.array([0.0, 0.0, 0.18])
        # 先抬到高处再平移，避免末端沿弧线把桌上的瓶子扫飞
        p_safe = APPROACH_HIGH
        qs = np.vstack([self.plan_path(p0, p_safe, 10),
                        self.plan_path(p_safe, p_hover, 12, seed=None)])
        self._start_traj("伸到瓶子上方", qs, p_hover, 3.2, 0.0)
        return (msg + " → 伸手（先抬高再平移）").strip(" →")

    def cmd_down(self) -> str:
        self._ensure_grasp()
        p0, _ = self.rig.hand_pose()
        qs = self.plan_path(p0, self.p_grasp, 14)
        self._start_traj("下降到抓取高度", qs, self.p_grasp, 2.2, 0.0)
        return "下降到抓取高度"

    def cmd_lift(self) -> str:
        self._ensure_grasp()
        p0, _ = self.rig.hand_pose()
        p1 = self.p_grasp + np.array([0.0, 0.0, 0.14])
        qs = self.plan_path(p0, p1, 14)
        self._start_traj("抬起瓶子", qs, p1, 2.2, None)
        return "抬起（手指保持当前握紧度）"

    def cmd_grasp(self) -> str:
        p, R = self.rig.hand_pose()
        d_bottle = float(np.linalg.norm(
            self.rig.bottle_pos() - (p + R @ BOTTLE_AXIS_IN_HAND)))
        if d_bottle > 0.14:
            return ("手离瓶子还有 %.0f mm，先敲 down 下降到抓取高度再 grasp"
                    % (d_bottle * 1000))
        # 官方手碰撞网格与 Ø65 mm 瓶互相嵌入（实测 ~40 mm），纯接触必然把瓶崩飞。
        # 与 demo 一致：手已到位 → 合拢前就把瓶子锁在手根上，再闭手指。
        self.grabbed = True
        return self.set_hand_pct(100.0) + " → 已锁瓶并开始握紧"

    def cmd_open(self) -> str:
        self.grabbed = False
        return self.set_hand_pct(0.0) + " → 松手"

    def cmd_auto(self) -> str:
        self.auto_seq = [
            ("抬起手臂", "arm"),
            ("伸手", "reach"),
            ("下降", "down"),
            ("握紧抓瓶", "grasp"),
            ("抬起瓶子", "lift"),
        ]
        self.auto_i = 0
        self.auto_t0 = self.t_sim
        self.set_lift(55.0)
        return "自动流程开始：抬臂 → 伸手 → 下降 → 抓 → 抬起"

    # ---- 每帧推进 ----
    def step(self, dt_frame: float) -> None:
        m, d, rig = self.m, self.d, self.rig
        n_step = max(1, int(round(dt_frame / m.opt.timestep)))

        # 笛卡尔任务：先播预规划关节序列，播完再用外环补偿 PD 下垂
        if self.cart is not None:
            c = self.cart
            el = self.t_sim - c["t0"]
            qs = c["qs"]
            if el < c["dur"]:
                u = smoothstep(el / max(1e-6, c["dur"])) * (len(qs) - 1)
                i0 = int(u)
                i1 = min(i0 + 1, len(qs) - 1)
                f = u - i0
                self.arm_cmd = qs[i0] * (1.0 - f) + qs[i1] * f
                self.arm_act = self.arm_cmd.copy()
            elif (self.t_sim - self._last_ik) > 0.5:
                e = c["p1"] - rig.hand_pose()[0]
                if np.linalg.norm(e) < 0.010 or el > c["dur"] + 6.0:
                    self.cart = None           # 到位（或放弃补偿，避免发散）
                else:
                    # 补偿量限幅 ±60 mm：纯 PD 撑不住自重会稳态下垂 50~110 mm
                    self.servo_target = (
                        self.servo_target + np.clip(e, -0.06, 0.06) * 0.7)
                    q, _, _ = self.ik.solve(self.servo_target, q_seed=qs[-1],
                                            axis_y=HAND_Y_AXIS,
                                            axis_z=getattr(self, "u_des", None),
                                            restarts=0, iters=120)
                    self.arm_cmd = np.clip(q, rig.arm_rng[:, 0], rig.arm_rng[:, 1])
                    self.arm_act = self.arm_cmd.copy()
                    self._last_ik = self.t_sim

        for _ in range(n_step):
            # 基座钉死（演示用：不让机体被手臂反作用带走）
            d.qpos[0:3] = (0.0, 0.0, 1.0525)
            d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
            d.qvel[0:6] = 0.0

            # 一阶滤波，动作平滑（约 0.25 s 时间常数）
            a = 1.0 - math.exp(-m.opt.timestep / 0.25)
            self.arm_act += (self.arm_cmd - self.arm_act) * a
            self.hand_act += (self.hand_cmd - self.hand_act) * a

            rig.hold_lower_body()
            rig.drive_arm(self.arm_act)
            rig.drive_hand(self.hand_act)

            # 抓住判定：握紧 ≥85% 且手离瓶够近 → 运动学锁瓶
            p, R = rig.hand_pose()
            bp = rig.bottle_pos()
            close = np.linalg.norm(bp - (p + R @ BOTTLE_AXIS_IN_HAND)) < 0.14
            if self.hold_pct >= 85.0 and close:
                self.grabbed = True
            if self.grabbed and self.hold_pct >= 85.0:
                rig.lock_bottle_to_hand()
            elif self.grabbed and self.hold_pct < 85.0:
                self.grabbed = False

            mujoco.mj_step(m, d)
            self.t_sim += m.opt.timestep

        if self.trace and (self.t_sim - self._last_trace) > 0.5:
            self._last_trace = self.t_sim
            p, _ = rig.hand_pose()
            tgt = self.cart["p1"] if self.cart is not None else None
            print("  [%.1fs] 手根 %s%s  手-瓶 %3.0f mm  %s"
                  % (self.t_sim, np.round(p, 3).tolist(),
                     (" → 目标 %s" % np.round(tgt, 3).tolist()) if tgt is not None else "",
                     np.linalg.norm(rig.bottle_pos() - p) * 1000,
                     "抓" if self.grabbed else ""), flush=True)

    # ---- 命令 ----
    def run_cmd(self, line: str) -> str:
        t = line.strip().lower()
        if not t:
            return ""
        parts = t.split()
        c = parts[0]
        try:
            if c == "arm" and len(parts) > 1:
                return self.set_lift(float(parts[1]))
            if c in JOINT_CMD and len(parts) > 1:
                return self.set_arm(JOINT_CMD[c][0], float(parts[1]))
            if c == "hand" and len(parts) > 1:
                return self.set_hand_pct(float(parts[1]))
            if c == "finger" and len(parts) > 1:
                return self.set_hand_deg(float(parts[1]))
            if c == "reach":
                return self.cmd_reach()
            if c == "down":
                return self.cmd_down()
            if c == "grasp":
                return self.cmd_grasp()
            if c == "open":
                return self.cmd_open()
            if c == "lift":
                return self.cmd_lift()
            if c == "home":
                return self.goto_home()
            if c == "auto":
                return self.cmd_auto()
            if c == "status":
                return self.status()
            if c in ("help", "?", "h"):
                return HELP
            if c in ("q", "quit", "exit"):
                self.stop = True
                return "退出"
        except Exception as exc:                       # noqa: BLE001
            return "命令出错：%r" % (exc,)
        return "不认识的命令「%s」，敲 help 看全部" % c

    # ---- 键盘动作 ----
    # 手动调角会重设目标关节角，而 set_arm / set_lift 内部会把当前笛卡尔任务清空。
    # 因此自动流程执行期间按方向键，会静默中断流程（现象是「机器人突然不走了」）。
    # 这里把中断显式化并给出提示，避免误判为按键抖动或程序卡死。
    def key_lift(self, delta_deg: float) -> str:
        return self._key_guard(
            self.set_lift(-self.arm_cmd[LIFT_IDX] * R2D + delta_deg))

    def key_swing(self, delta_deg: float) -> str:
        return self._key_guard(self.set_arm(2, self.arm_cmd[2] * R2D + delta_deg))

    def key_elbow(self, delta_deg: float) -> str:
        return self._key_guard(self.set_arm(3, self.arm_cmd[3] * R2D + delta_deg))

    def key_hand(self, delta_pct: float) -> str:
        return self.set_hand_pct(self.hold_pct + delta_pct)

    def _key_guard(self, msg: str) -> str:
        if hasattr(self, "auto_seq") and self.auto_i < len(self.auto_seq):
            self.auto_i = len(self.auto_seq)
            self.cart = None
            return msg + "（已中断自动流程；敲 auto 可重跑）"
        return msg

    def status(self) -> str:
        q = np.degrees(self.rig.arm_q())
        p, _ = self.rig.hand_pose()
        bp = self.rig.bottle_pos()
        return (
            "手臂关节(度) " + " ".join("%s=%.0f" % (n.split("_")[1][:4], v)
                                      for n, v in zip(ARM_JOINTS, q))
            + "\n  抬起角度 %.0f°（shoulder_x）   手根 %s"
            % (-q[LIFT_IDX], np.round(p, 3).tolist())
            + "\n  握拳 %d%%   手指(度) %s"
            % (int(self.hold_pct), np.round(np.degrees(self.rig.hand_angles()), 1).tolist())
            + "\n  瓶子 %s   %s" % (np.round(bp, 3).tolist(),
                                    "已抓住" if self.grabbed else "未抓住")
        )

    # ---- 自动序列：每一步都等「真的做完了」再进下一步 ----
    def pump_auto(self) -> None:
        if not hasattr(self, "auto_seq") or self.auto_i >= len(self.auto_seq):
            return
        name, kind = self.auto_seq[self.auto_i]
        if self._auto_started != self.auto_i:
            self._auto_started = self.auto_i
            fn = {"arm": lambda: self.set_lift(55.0), "reach": self.cmd_reach,
                  "down": self.cmd_down, "grasp": self.cmd_grasp,
                  "lift": self.cmd_lift}[kind]
            msg = fn()
            self.auto_t0 = self.t_sim
            print("  ▶ %-10s %s" % (name, msg), flush=True)
            return

        elapsed = self.t_sim - self.auto_t0
        if kind in ("arm", "grasp"):
            done = elapsed > 2.0          # 纯关节 / 手指动作，给固定时长
        else:
            # 笛卡尔动作：等轨迹播完 + 外环补偿结束（cart 被清空），最多等 9 s
            done = (self.cart is None) or (elapsed > 9.0)
        if done:
            self.auto_i += 1
            if self.auto_i >= len(self.auto_seq):
                p, R = self.rig.hand_pose()
                gap = float(np.linalg.norm(
                    self.rig.bottle_pos() - (p + R @ BOTTLE_AXIS_IN_HAND)))
                print("  ✅ 自动流程结束（手-瓶 %.0f mm，%s）"
                      % (gap * 1000, "已抓住" if self.grabbed else "未抓住"),
                      flush=True)

    _auto_started = -1


# ---------------------------------------------------------------------------


def stdin_reader(q: queue.Queue) -> None:
    while True:
        try:
            line = sys.stdin.readline()
        except Exception:                             # noqa: BLE001
            return
        if not line:
            return
        q.put(line)


def _find_scene() -> str | None:
    """按常见布局找 scene_bottle.xml。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(here, "..", "..", "elf3_revo2_sim", "scene_bottle.xml"),
        os.path.join(here, "scene_bottle.xml"),
    ]
    for c in cands:
        c = os.path.normpath(c)
        if os.path.isfile(c):
            return c
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="ELF3 + Revo2 实时遥控仿真")
    ap.add_argument("--model", default=None,
                    help="场景文件（含机器人 + 桌子 + 瓶子）。缺省时按 <合并模型同目录>/scene_bottle.xml 查找")
    ap.add_argument("--no-auto", action="store_true", help="启动后不自动演示")
    ap.add_argument("--nogui", action="store_true", help="不弹窗（自检用）")
    ap.add_argument("--seconds", type=float, default=0.0, help="自检时长")
    ap.add_argument("--trace", action="store_true", help="每 0.5s 打印手根位置")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                 # noqa: BLE001
        pass

    model = args.model or _find_scene()
    if not model:
        print("找不到场景文件。请用 --model 指定，或先跑一次 revo2_pick_bottle_demo.py 生成 "
              "scene_bottle.xml（场景由该脚本现场写出，不在模型资产里）。", flush=True)
        return 1
    print(" 场景：%s" % model, flush=True)

    tele = Teleop(model)
    tele.trace = args.trace
    print("=" * 62)
    print(" ELF3 + Revo2 右手 —— 实时遥控")
    print(" 瓶子在 %s，桌面前方；起始手根离瓶 %.0f mm"
          % (np.round(tele.bottle0, 3).tolist(), tele.home_gap * 1000))
    print(" 敲命令回车生效；help 看全部；点 3D 窗口后可用键盘")
    print("=" * 62, flush=True)

    threading.Thread(target=stdin_reader, args=(tele.cmdq,), daemon=True).start()

    if not args.no_auto:
        print(tele.cmd_auto(), flush=True)

    if args.nogui:
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < args.seconds:
            tele.pump_auto()
            tele.step(1.0 / 60.0)
            while not tele.cmdq.empty():
                print("> " + tele.run_cmd(tele.cmdq.get()), flush=True)
            time.sleep(1.0 / 60.0)
        print(tele.status(), flush=True)
        return 0

    # GLFW 键码
    K_UP, K_DOWN, K_LEFT, K_RIGHT = 265, 264, 263, 262
    keys = {
        K_UP: lambda: tele.key_lift(+5),
        K_DOWN: lambda: tele.key_lift(-5),
        K_RIGHT: lambda: tele.key_swing(-5),
        K_LEFT: lambda: tele.key_swing(+5),
        87: lambda: tele.key_elbow(+5),                          # W
        83: lambda: tele.key_elbow(-5),                          # S
        91: lambda: tele.key_hand(-10),                          # [
        93: lambda: tele.key_hand(+10),                          # ]
        71: lambda: tele.cmd_grasp(),                             # G
        79: lambda: tele.cmd_open(),                              # O
        82: lambda: tele.goto_home(),                             # R
        72: lambda: print(HELP, flush=True),                      # H
        49: lambda: tele.cmd_reach(),                             # 1
        50: lambda: tele.cmd_down(),                              # 2
        51: lambda: tele.cmd_lift(),                              # 3
        65: lambda: tele.cmd_auto(),                              # A
    }

    def on_key(k: int) -> None:
        fn = keys.get(k)
        if fn is not None:
            msg = fn()
            if msg:
                print("  [键盘] " + msg, flush=True)

    with mujoco.viewer.launch_passive(tele.m, tele.d, key_callback=on_key) as v:
        # 相机：机器人朝 +x、右侧 = −y；从正右侧看，手里的瓶子不被躯干挡住
        v.cam.lookat[:] = [0.16, -0.22, 0.95]
        v.cam.distance = 1.9
        v.cam.azimuth = 270.0
        v.cam.elevation = 6.0
        last = time.perf_counter()
        while v.is_running() and not tele.stop:
            while not tele.cmdq.empty():
                msg = tele.run_cmd(tele.cmdq.get())
                if msg:
                    print("> " + msg, flush=True)
            tele.pump_auto()
            now = time.perf_counter()
            dt = clamp(now - last, 0.0, 0.05)
            last = now
            tele.step(dt)
            time.sleep(max(0.0, 1.0 / 60.0 - (time.perf_counter() - now)))
            v.sync()
    return 0


if __name__ == "__main__":
    sys.exit(main())

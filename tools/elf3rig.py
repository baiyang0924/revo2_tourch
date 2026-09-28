#!/usr/bin/env python3
"""ELF3 + Revo2 右手的仿真控制小库（供 teleop_arm_hand.py 使用）。

内容全部对齐 dexterous-hand/tools/revo2_pick_bottle_demo.py：
  * Rig   —— 手臂 PD（含重力/科氏前馈）+ 下半身硬固定 + 手部位置伺服
  * ArmIK —— 阻尼最小二乘逆运动学（MuJoCo 自带雅可比，无需 pinocchio/scipy）
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

SIDE = "right"
ARM_JOINTS = (
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
    "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
)
HAND_MOTORS = (
    "right_thumb_metacarpal_joint", "right_thumb_proximal_joint",
    "right_index_proximal_joint", "right_middle_proximal_joint",
    "right_ring_proximal_joint", "right_pinky_proximal_joint",
)
HAND_ROOT = "right_hand_base_link"

# 官方被注释掉的 <general> 执行器给出的 PD 增益
KP_DEFAULT, KD_DEFAULT = 54.2241, 3.45201
KP_SOFT, KD_SOFT = 50.24, 3.198
KP_LEG, KD_LEG = 176.421, 11.2313
KP_HIPZ, KD_HIPZ = 54.2241, 3.45201
KP_ANKLE, KD_ANKLE = 33.4934, 2.13225
KP_ANKLE_X, KD_ANKLE_X = 10.0, 1.0

BASE_Z = 1.0525                 # 全零位时脚底正好落地

BOTTLE_R = 0.0325               # 半径 32.5 mm
BOTTLE_H = 0.10                 # 半高（总高 200 mm）

# 抓握时瓶心在手根坐标系里的位置（由「合拢手 + 五指贴面」拟合）
BOTTLE_AXIS_IN_HAND = np.array([0.001, 0.0, 0.085])

HAND_Y_AXIS = np.array([0.0, 0.0, 1.0])   # 手根 Y 轴竖直 → 瓶轴竖直

# 手指：0 = 完全张开，1 = 完全握紧（弧度）
HAND_OPEN = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
HAND_GRASP = np.array([1.20, 1.00, 1.36, 1.37, 1.37, 1.37])


# ---------------------------------------------------------------------------
# 数学小工具
# ---------------------------------------------------------------------------


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).flatten())
    return q


def rot_error(target_R: np.ndarray, cur_R: np.ndarray) -> np.ndarray:
    """当前姿态到目标姿态的 3 维误差（表达在当前坐标系）。"""
    qa = quat_from_matrix(target_R)
    qb = quat_from_matrix(cur_R)
    res = np.zeros(3)
    mujoco.mju_subQuat(res, qa, qb)
    return res


def home_arm_q() -> np.ndarray:
    """自然下垂的起始关节角（弧度）。"""
    q = np.zeros(len(ARM_JOINTS))
    q[0] = -0.10     # shoulder_y 略前
    q[1] = -0.16     # shoulder_x 略外展（右臂外展上限只有 +20°，取负方向）
    q[3] = 0.30      # elbow_y 微屈
    return q


# ---------------------------------------------------------------------------
# 模型封装
# ---------------------------------------------------------------------------


class Rig:
    """把合并模型包装成「手臂 IK + PD 力矩 + 手部位置伺服」。"""

    def __init__(self, scene_path: str) -> None:
        self.m = mujoco.MjModel.from_xml_path(scene_path)
        self.d = mujoco.MjData(self.m)
        m = self.m

        self.arm_qadr = np.array([
            m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in ARM_JOINTS])
        self.arm_dadr = np.array([
            m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in ARM_JOINTS])
        self.arm_rng = np.array([
            np.asarray(m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)])
            for j in ARM_JOINTS])

        self.hand_act = np.array([
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in HAND_MOTORS])
        if (self.hand_act < 0).any():
            raise RuntimeError("模型里找不到手部 actuator")

        self.hand_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, HAND_ROOT)
        self.bottle_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "bottle")
        _bj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "bottle_free")
        if _bj < 0:
            raise RuntimeError("场景里找不到 bottle_free 自由关节")
        self.bottle_qadr = int(m.jnt_qposadr[_bj])
        self.bottle_dadr = int(m.jnt_dofadr[_bj])

        self._build_gains()

    # ---- 增益表 ----
    @staticmethod
    def _kp_kd(joint: str) -> tuple[float, float]:
        j = joint[2:] if joint[:2] in ("l_", "r_") else joint
        if "hip_z" in j:
            return KP_HIPZ, KD_HIPZ
        if "hip_y" in j or "hip_x" in j or "knee_y" in j:
            return KP_LEG, KD_LEG
        if "ankle_x" in j:
            return KP_ANKLE_X, KD_ANKLE_X
        if "ankle_y" in j:
            return KP_ANKLE, KD_ANKLE
        if "shoulder_z" in j or "wrist" in j:
            return KP_SOFT, KD_SOFT
        return KP_DEFAULT, KD_DEFAULT

    def _build_gains(self) -> None:
        m = self.m
        self.kp = np.zeros(m.nu)
        self.kd = np.zeros(m.nu)
        self.tau_lim = np.zeros(m.nu)
        self.is_hand = np.zeros(m.nu, dtype=bool)
        self.body_act: dict[str, int] = {}
        self.body_dof: dict[str, int] = {}
        self.lower_qadr: list[int] = []
        self.lower_dadr: list[int] = []
        for i in range(m.nu):
            name = m.actuator(i).name or ""
            lo, hi = np.asarray(m.actuator_ctrlrange[i])
            self.tau_lim[i] = min(abs(lo), abs(hi))
            if name.startswith("right_") and name.endswith("_joint"):
                self.is_hand[i] = True
                continue
            self.kp[i], self.kd[i] = self._kp_kd(name)
            self.body_act[name] = i
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.body_dof[name] = int(m.jnt_dofadr[jid])
            if name not in ARM_JOINTS:
                self.lower_qadr.append(int(m.jnt_qposadr[jid]))
                self.lower_dadr.append(int(m.jnt_dofadr[jid]))

    # ---- 基座钉住 ----
    def pin_base(self) -> None:
        d = self.d
        d.qpos[0:3] = (0.0, 0.0, BASE_Z)
        d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        d.qvel[0:6] = 0.0
        if self.lower_qadr:
            d.qpos[np.asarray(self.lower_qadr)] = 0.0
            d.qvel[np.asarray(self.lower_dadr)] = 0.0
        mujoco.mj_forward(self.m, d)

    # ---- PD（含重力/科氏前馈）----
    def _pd(self, name: str, target: float) -> None:
        m, d = self.m, self.d
        aid = self.body_act[name]
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        q = d.qpos[m.jnt_qposadr[jid]]
        v = d.qvel[m.jnt_dofadr[jid]]
        # ★ 没有这项前馈，纯 PD 力矩撑不住自重，手臂会稳态下垂（实测偏低 80 mm）
        bias = d.qfrc_bias[self.body_dof[name]]
        tau = bias + self.kp[aid] * (target - q) - self.kd[aid] * v
        d.ctrl[aid] = float(np.clip(tau, -self.tau_lim[aid], self.tau_lim[aid]))

    def hold_lower_body(self) -> None:
        for name in self.body_act:
            if name not in ARM_JOINTS:
                self._pd(name, 0.0)

    def drive_arm(self, q_des: np.ndarray) -> None:
        for k, name in enumerate(ARM_JOINTS):
            self._pd(name, float(q_des[k]))

    # ---- 手部位置伺服 ----
    def drive_hand(self, angles) -> None:
        for aid, ang in zip(self.hand_act, angles):
            lo, hi = self.m.actuator_ctrlrange[aid]
            self.d.ctrl[aid] = float(np.clip(ang, lo, hi))

    def hand_angles(self) -> np.ndarray:
        m, d = self.m, self.d
        return np.array([
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]]
            for j in HAND_MOTORS])

    def hand_error(self, angles) -> float:
        return float(np.max(np.abs(self.hand_angles() - np.asarray(angles))))

    # ---- 查询 ----
    def arm_q(self) -> np.ndarray:
        return self.d.qpos[self.arm_qadr].copy()

    def arm_error(self, q_des: np.ndarray) -> float:
        return float(np.max(np.abs(self.arm_q() - q_des)))

    def hand_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return (self.d.xpos[self.hand_body].copy(),
                self.d.xmat[self.hand_body].reshape(3, 3).copy())

    def bottle_pos(self) -> np.ndarray:
        return self.d.xpos[self.bottle_body].copy()

    # ---- 瓶子运动学锁定（抓取后每帧调用）----
    def lock_bottle_to_hand(self) -> None:
        d = self.d
        p, R = self.hand_pose()
        target = p + R @ BOTTLE_AXIS_IN_HAND
        a = self.bottle_qadr
        d.qpos[a:a + 3] = target
        d.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)   # 保持竖直
        d.qvel[self.bottle_dadr:self.bottle_dadr + 6] = 0.0


# ---------------------------------------------------------------------------
# 逆运动学
# ---------------------------------------------------------------------------


class ArmIK:
    """把 right_hand_base_link 摆到指定位姿，解 7 个右臂关节角。

    axis_y 模式只约束「手根 Y 轴指向」，留 1 个滚转自由度——抓瓶子必须用这个，
    硬约束 6 自由度反而解不出来（实测三腕全贴限位、位置误差 180 mm）。
    """

    def __init__(self, rig: Rig, w_rot: float = 0.35, w_axis: float = 0.30,
                 lam: float = 0.03) -> None:
        self.rig = rig
        self.m = rig.m
        self.d = rig.d
        self.w_rot = w_rot
        self.w_axis = w_axis
        self.lam = lam

    def _make_targets(self, target_pos, target_R, axis_y, axis_z=None) -> dict:
        t: dict = {"pos": np.asarray(target_pos, dtype=float)}
        if axis_y is not None:
            v = np.asarray(axis_y, dtype=float)
            t["axis_y"] = v / (np.linalg.norm(v) + 1e-12)
        elif target_R is not None:
            t["rot"] = np.asarray(target_R, dtype=float)
        else:
            raise ValueError("必须给 target_R 或 axis_y 之一")
        if axis_z is not None:
            v = np.asarray(axis_z, dtype=float)
            t["axis_z"] = v / (np.linalg.norm(v) + 1e-12)
        return t

    def _iterate(self, q: np.ndarray, t: dict, iters: int) -> np.ndarray:
        m, d, rig = self.m, self.d, self.rig
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        for _ in range(iters):
            d.qpos[rig.arm_qadr] = q
            mujoco.mj_forward(m, d)
            p, R = rig.hand_pose()
            mujoco.mj_jacBody(m, d, jacp, jacr, rig.hand_body)
            Jp = jacp[:, rig.arm_dadr]
            Jr = jacr[:, rig.arm_dadr]

            rows = [(Jp, t["pos"] - p)]
            if "rot" in t:
                rows.append((self.w_rot * Jr, self.w_rot * rot_error(t["rot"], R)))
            if "axis_y" in t:
                y = R[:, 1]
                skew = np.array([[0, -y[2], y[1]], [y[2], 0, -y[0]], [-y[1], y[0], 0]])
                rows.append((-self.w_axis * (skew @ Jr), self.w_axis * (t["axis_y"] - y)))
            if "axis_z" in t:
                # 手根 Z 轴 = 手指伸出的方向。抓瓶子必须让它指向瓶心，
                # 否则手根到位了、手指却朝着空气（实测手-瓶 77 mm 却抓不到）。
                z = R[:, 2]
                skew = np.array([[0, -z[2], z[1]], [z[2], 0, -z[0]], [-z[1], z[0], 0]])
                rows.append((-self.w_axis * (skew @ Jr), self.w_axis * (t["axis_z"] - z)))

            J = np.vstack([r[0] for r in rows])
            e = np.concatenate([r[1] for r in rows])
            if np.linalg.norm(e) < 1e-5:
                break
            A = J @ J.T + (self.lam ** 2) * np.eye(J.shape[0])
            q = np.clip(q + 0.85 * (J.T @ np.linalg.solve(A, e)),
                        rig.arm_rng[:, 0], rig.arm_rng[:, 1])
        return q

    def _errors(self, q: np.ndarray, t: dict) -> tuple[float, float]:
        m, d, rig = self.m, self.d, self.rig
        saved = d.qpos.copy()
        d.qpos[rig.arm_qadr] = q
        mujoco.mj_forward(m, d)
        p, R = rig.hand_pose()
        e_pos = float(np.linalg.norm(t["pos"] - p))
        e_rot = float(np.linalg.norm(rot_error(t["rot"], R))) if "rot" in t else \
            float(np.linalg.norm(t["axis_y"] - R[:, 1])) if "axis_y" in t else 0.0
        if "axis_z" in t:
            e_rot += float(np.linalg.norm(t["axis_z"] - R[:, 2]))
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)
        return e_pos, e_rot

    def solve(self, target_pos, q_seed=None, target_R=None, axis_y=None,
              axis_z=None, iters: int = 320, restarts: int = 28):
        t = self._make_targets(target_pos, target_R, axis_y, axis_z)
        rig = self.rig
        rng = np.random.default_rng(20260922)
        lo = np.maximum(rig.arm_rng[:, 0], -2.2)
        hi = np.minimum(rig.arm_rng[:, 1], 2.2)
        if q_seed is not None and restarts <= 0:
            seeds = [np.asarray(q_seed, dtype=float)]
        else:
            seeds = [np.asarray(q_seed, dtype=float)] if q_seed is not None else []
            seeds.append(home_arm_q())
            for _ in range(restarts):
                seeds.append(rng.uniform(lo, hi))

        best_q, best_cost, best_err = None, None, None
        for s in seeds:
            q = self._iterate(s.copy(), t, iters)
            e_pos, e_rot = self._errors(q, t)
            cost = e_pos * 1000.0 + e_rot * 20.0
            if best_cost is None or cost < best_cost:
                best_cost, best_q, best_err = cost, q.copy(), (e_pos, e_rot)
        return best_q, best_err[0], best_err[1]

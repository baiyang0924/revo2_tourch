#!/usr/bin/env python3
"""ELF3 + Revo2 右手 仿真演示：拿起瓶装水并递给对方。

这个脚本回答三个问题
--------------------
1. **手模型怎么用起来** —— 加载合并后的 ``elf3_revo2_right.xml``，把 37 个执行器
   分成两组：31 个本体关节（官方是纯力矩 ``<motor>``，要自己写 PD）+ 6 个手部关节
   （位置伺服，直接给角度）。
2. **怎么分两条命令控制** —— ``--mode arm`` 只动手臂、``--mode hand`` 只动手。
3. **怎么合成一条命令** —— ``--mode combo`` 跑完整的「拿起瓶子 → 递给对方」序列。

抓握几何是怎么定的（关键设计）
------------------------------
不是「先摆好瓶子再让手去够」，而是**反过来**：

1. 解手臂 IK，让手根 Y 轴竖直（= 瓶轴竖直），手指朝前，掌心朝机器人左侧；
2. 在**合拢**的手上测出瓶轴该在手根坐标系的哪儿（``[1, 0, 85] mm``，沿手根 Y 轴）；
3. 用**实际解算出来的**手根位姿反推瓶子在世界里的位置，再据此摆桌子和瓶子。

这样瓶子和小手是几何自洽的，不依赖 IK 的残余误差。

动作序列（combo）
-----------------
::

    home      手臂自然下垂，手张开
    approach  移到瓶子上方 10 cm
    descend   下降到抓取高度
    grasp     手闭合（6 个电机到抓握角，四指远端由 equality 带动）
    lift      抬起 14 cm
    handover  前伸抬高到递给对方的位置
    release   张开手
    retract   手臂收回

仿真设置上的三个取舍
--------------------
* **基座钉住**：整机有自由关节，跑动力学必然摔。手臂演示不需要全身平衡，
  所以每步把基座位姿写回、速度清零，腿部关节 PD 保持零位。
  手臂和手仍是**真动力学 + 真接触**，只是身体不参与平衡。
* **保留重力**：抓瓶子靠接触摩擦，关重力就没意义了。
* **合拢瞬间刚性夹持**（`--grip rigid`，默认）：官方 Revo2 的**碰撞网格**和
  Ø65 mm 瓶子互相不兼容 —— 握紧时手指近端关节的网格落在瓶轴附近，整只手与
  瓶子网格互相嵌入 20~40 mm（`mj_geomDistance` 实测）。接触求解器对这种深嵌入
  会给出巨大的弹开力，表现为「手一合就把瓶子崩飞」。这是模型网格的问题，不是
  调参能解决的。所以：**抓取之前一切照实算**（手臂轨迹、桌板接触、瓶子被碰倒
  都真实），**只在手指合拢完成的那一刻把瓶子刚性锁在手根上**，松手时解锁。
  看纯接触抓取：`--grip contact`。

用法
----

::

    python3 tools/revo2_pick_bottle_demo.py --model <dir>/elf3_revo2_right.xml
    python3 tools/revo2_pick_bottle_demo.py --model ... --mode arm
    python3 tools/revo2_pick_bottle_demo.py --model ... --mode hand
    python3 tools/revo2_pick_bottle_demo.py --model ... --mode combo --record out/

`--release hold|drop` 控制松手后瓶子停在原地（默认）还是自由落体。

退出码：0 正常 / 1 用法或模型问题 / 5 动作没跑成（瓶子没被提起）。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover
    print("需要 mujoco：pip install mujoco", file=sys.stderr)
    sys.exit(1)

EXIT_OK = 0
EXIT_USAGE = 1
EXIT_FAILED = 5

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

# 官方 commented-out <general> 执行器给出的 PD 增益：
#   torque = gainprm*(ctrl - q) - |biasprm[2]|*qdot
# 官方 MJCF 里本体关节是纯力矩 <motor>，所以要自己用这组增益做 PD。
KP_DEFAULT, KD_DEFAULT = 54.2241, 3.45201     # 肩 y/x、肘
# 腕/肩 z 官方给的是 16.7467；腕部关节 armature 小（0.0042），可以用更大的增益，
# 而官方那套增益是按「真机上有重力补偿的伺服」配的，纯 PD 用它会明显滞后。
KP_SOFT, KD_SOFT = 50.24, 3.198               # 肩 z、三个腕（官方值 ×3）
KP_LEG, KD_LEG = 176.421, 11.2313             # 髋 y/x、膝
KP_HIPZ, KD_HIPZ = 54.2241, 3.45201
KP_ANKLE, KD_ANKLE = 33.4934, 2.13225
KP_ANKLE_X, KD_ANKLE_X = 10.0, 1.0

# 站立基座高度：全零位时脚底正好落在地面
BASE_Z = 1.0525

# 瓶子规格（500 ml 瓶装水）
BOTTLE_R = 0.0325     # 半径 32.5 mm（Ø65 mm）
BOTTLE_H = 0.10       # 半高（总高 200 mm）
BOTTLE_MASS = 0.50    # 500 g

# 抓握时瓶轴在手根坐标系里的位置（由「合拢手 + 五指贴面」拟合而来，见 --explain-grasp）
BOTTLE_AXIS_IN_HAND = np.array([0.001, 0.0, 0.085])

# 抓取时手根的目标位置（世界坐标，米）—— 前伸、右手侧、桌面高度
GRASP_HAND_TARGET = np.array([0.26, -0.30, 0.86])

# 起始（home）时手根的目标位置 —— 必须同时满足两条：
#   ① 远离瓶子所在的工作区。手从手根还要往前伸约 150 mm（掌心 + 手指），
#      手根离瓶心小于 200 mm 时一开局手臂就会压在瓶子上把它撞飞。
#   ② **必须落在桌面板的平面范围之外**。桌板半边长 0.18 m、中心跟着瓶子
#      （约 x=0.26 / y=-0.30），所以桌板覆盖 x∈[0.08,0.44]、y∈[-0.48,-0.12]。
#      起始位落在里面就会被桌板顶住，PD 顶不过去，手臂会跑到别的地方。
HOME_HAND_TARGET = np.array([-0.04, -0.06, 0.98])

# 接近路线的第一站：先把手高高抬起到工作区上方（远离瓶子），
# 再水平平移过去。否则从起始位直接插值到瓶子上方，末端会划弧扫过瓶子。
APPROACH_HIGH = np.array([0.11, -0.24, 1.20])

# 预抓取时手根在抓取位上方抬起的量：手是水平伸出的，
# 抬 100 mm 时指尖刚好擦到瓶顶（瓶高 200 mm），改用 180 mm。
PREGRASP_LIFT = 0.18

# 抬起 / 递交时手根相对抓取位的偏移（米）。
# ⚠️ 递交位的 y 分量**不能为正**（不能往身体中线挪）：瓶子是从掌心往外伸 85 mm 的，
# 手往中线一挪，瓶子就直接插进机器人自己的胸口（实测画面：瓶子扎进躯干）。
# 所以递交是「往前 + 往上」，横向基本保持不动。
LIFT_DELTA = np.array([0.0, 0.0, 0.14])
HANDOVER_DELTA = np.array([0.12, -0.01, 0.30])

# 哪些阶段让末端走笛卡尔直线（而不是关节空间插值）。
# ★ 现在**所有会动手臂的阶段**都要走直线。关节空间插值是「各关节各自线性走」，
#   末端轨迹完全不受控 —— 实测第一段「approach 抬起」就把桌面上的瓶子扫飞了
#   （水平位移 544 mm，直接掉到地上）。关节空间插值唯一安全的地方是「手臂根本不动」
#   的阶段（grasp 闭手 / release 张手）。
CARTESIAN_KEYWORDS = ("approach", "descend", "lift", "handover", "retract", "锁定")
CARTESIAN_SEGS = 24       # 直线切成几段，每段重解一次 IK

# 手根 Y 轴的目标指向：竖直向上，这样瓶轴也是竖直的
HAND_Y_AXIS = np.array([0.0, 0.0, 1.0])

HAND_OPEN = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
HAND_GRASP = (1.20, 1.00, 1.36, 1.37, 1.37, 1.37)

# 刚性夹持（默认开）
# ------------------
# 官方 Revo2 的**碰撞网格**和 Ø65 mm 的瓶子互相不兼容：握紧时手指近端关节的
# 网格落在瓶轴附近，整只手与瓶子网格互相嵌入 20~40 mm（用 mj_geomDistance 实测）。
# 接触求解器对这种深嵌入会给出巨大的弹开力，表现为「手一合就把瓶子崩飞」。
# 这不是调参能解决的（张开/半蜷/握紧三种状态都嵌入），是模型网格的问题。
# 所以仿真的做法是：**抓取之前一切照实算**（手臂轨迹、桌板接触、瓶子被碰倒都真实），
# **只在「手合拢完成」那一刻把瓶子刚性锁在手根上**，松手时解锁。
# 想关掉看纯接触抓取：--grip contact
GRIP_MODES = ("rigid", "contact")

# 哪些阶段触发「锁定 / 解锁」。
# grasp 阶段末（手指已合拢）锁定；release 阶段末（手指已张开）解锁。
GRIP_LATCH_KEYWORDS = ("grasp",)
GRIP_RELEASE_KEYWORDS = ("release",)

# 松手之后瓶子怎么办：
#   hold —— 原地停住不动。仿真里没有「第二个人」的模型，瓶子停在交接点
#           就代表被对方接住了（默认，画面最像话）。
#   drop —— 恢复自由落体，真的掉下去（物理更诚实，但会滚下桌子）。
RELEASE_MODES = ("hold", "drop")

# 夹持期间瓶子的姿态怎么办：
#   level —— **瓶子始终竖直**（默认）。位置跟着手走，姿态不跟。
#            这才是「递一瓶水」该有的样子：真人端水时手腕会一直把瓶子扶正。
#            如果用 rigid，手在抬臂过程中转了 20~30°，瓶子就跟着歪 —— 画面上
#            等于在洒水（实测第 70 帧瓶子歪了约 25°）。
#   rigid —— 完全刚性：瓶子相对手根固定，手怎么转瓶子怎么转。
#            物理上更「诚实」，但视觉上像在洒水。
# 真机对照：保持瓶子竖直这件事，在真机上由腕关节（r_wrist_*）的轨迹规划负责，
#          不需要「夹持」这个仿真替身来做。
CARRY_MODES = ("level", "rigid")


def info(msg: str) -> None:
    print(msg, flush=True)


def die(code: int, msg: str) -> None:
    print(f"\n[错误] {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


# ---------------------------------------------------------------------------
# 场景：桌子 + 瓶子，机器人用 <include> 拉进来
# ---------------------------------------------------------------------------


def write_scene(model_path: str, out_path: str, bottle_xyz: np.ndarray,
                table_top_z: float, bottle_collide: bool = True,
                table_xy: np.ndarray | None = None) -> str:
    """生成带桌子和瓶子的场景 XML（与合并模型同目录，靠 <include> 复用）。

    ``bottle_collide=False``：瓶子照常摆着，但关掉碰撞。**标定阶段必须这样** ——
    标定要的是「桌子这个真实环境约束下，手臂发什么角度才会停在哪」，此时若是
    张开的手压在瓶子上，外环迭代就会一边压瓶、一边被瓶顶回来，直接发散。

    ``table_xy``：桌子中心（不给则跟着瓶子走）。**正式流程里必须显式给** ——
    桌子一旦跟着瓶子动，瓶子每轮微调就会带着桌子一起挪，桌板迟早顶住手臂，
    标定会突然崩掉（实测手根偏 819 mm）。桌子是环境，环境不能跟着目标跑。
    """
    model_file = os.path.basename(model_path)
    bx, by, bz = (float(v) for v in bottle_xyz)
    tx, ty = (float(v) for v in (table_xy if table_xy is not None else bottle_xyz[:2]))
    _cc = "1" if bottle_collide else "0"
    # 桌腿从地面一直伸到桌板底面，别悬空
    _tt = float(table_top_z)
    _leg_half = max(0.02, (_tt - 0.04) / 2.0)

    scene = f'''<mujoco model="elf3_revo2_bottle">
  <!-- 机器人本体（含 Revo2 右手）从这里拉进来 -->
  <include file="{model_file}"/>

  <option cone="elliptic" impratio="10"/>
  <statistic center="{bx:.4f} {by:.4f} {bz:.4f}" extent="1.6"/>

  <visual>
    <headlight diffuse="0.8 0.8 0.8" ambient="0.32 0.32 0.32"/>
    <global offwidth="960" offheight="960"/>
  </visual>

  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.02" rgba="0.30 0.32 0.35 1"
          contype="1" conaffinity="1"/>

    <!-- 桌子（桌面高度由抓取几何反推得到；中心固定在 table_xy，不跟着瓶子动） -->
    <body name="table" pos="{tx:.4f} {ty:.4f} {table_top_z - 0.02:.4f}">
      <geom name="table_top" type="box" size="0.18 0.18 0.02"
            rgba="0.55 0.42 0.28 1" contype="1" conaffinity="1"/>
      <geom name="table_leg" type="cylinder" size="0.028 {_leg_half:.4f}"
            pos="0 0 {-table_top_z / 2.0:.4f}" rgba="0.35 0.30 0.25 1"
            contype="1" conaffinity="1"/>
    </body>

    <!-- 瓶装水（自由关节，可被抓起） -->
    <body name="bottle" pos="{bx:.4f} {by:.4f} {bz:.4f}">
      <freejoint name="bottle_free"/>
      <inertial pos="0 0 0" mass="{BOTTLE_MASS}" diaginertia="0.0018 0.0018 0.0002"/>
      <geom name="bottle_body" type="cylinder" size="{BOTTLE_R:.4f} {BOTTLE_H:.4f}"
            rgba="0.25 0.55 0.85 0.72" contype="{_cc}" conaffinity="{_cc}"
            condim="4" friction="1.4 0.06 0.002" solref="0.006 1"/>
      <geom name="bottle_cap" type="cylinder" size="0.016 0.014"
            pos="0 0 {BOTTLE_H + 0.012:.4f}"
            rgba="0.20 0.20 0.22 1" contype="{_cc}" conaffinity="{_cc}"/>
    </body>
  </worldbody>
</mujoco>
'''
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(scene)
    return out_path


# ---------------------------------------------------------------------------
# 数学小工具
# ---------------------------------------------------------------------------


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).ravel())
    return q


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.ascontiguousarray(q, dtype=float))
    return R.reshape(3, 3)


def rot_error(target_R: np.ndarray, cur_R: np.ndarray) -> np.ndarray:
    """当前姿态到目标姿态的 3 维误差（表达在当前坐标系）。"""
    qa = quat_from_matrix(target_R)
    qb = quat_from_matrix(cur_R)
    res = np.zeros(3)
    mujoco.mju_subQuat(res, qa, qb)
    return res


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
            die(EXIT_USAGE,
                "模型里找不到手部 actuator。\n"
                "       → 确认 --model 是由 tools/revo2_merge_mjcf.py 生成的")

        self.hand_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, HAND_ROOT)
        self.bottle_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "bottle")
        # 瓶子的自由关节：初始化时必须一起复位，否则会被 qpos[:]=0 扔到世界原点
        _bj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "bottle_free")
        if _bj < 0:
            die(EXIT_USAGE, "场景里找不到 bottle_free 自由关节"
                            "（--model 必须是合并模型，场景由本脚本生成）")
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
        # 除手臂外的所有本体关节（腰/腿/头）：手臂演示里这些要「硬固定」，
        # 不能只靠 PD 保持 —— PD 撑不住腿自重，手臂的反作用会把腿甩起来，
        # 实测腿会横到桌面上，把瓶子扫倒（看起来像手臂撞的，其实不是）。
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
        # 钉住后必须重新前向一次，否则 xpos/xmat 与 qfrc_bias 仍是钉住前的旧值
        mujoco.mj_forward(self.m, d)

    def _pd(self, name: str, target: float) -> None:
        m, d = self.m, self.d
        aid = self.body_act[name]
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        q = d.qpos[m.jnt_qposadr[jid]]
        v = d.qvel[m.jnt_dofadr[jid]]
        # ★ 重力/科氏前馈：没有这一项，纯 PD 力矩撑不住自重，
        #   手臂会稳态下垂（实测手根偏低 80 mm），IK 算得再准也没用。
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

    def arm_q(self) -> np.ndarray:
        return self.d.qpos[self.arm_qadr].copy()

    def arm_error(self, q_des: np.ndarray) -> float:
        return float(np.max(np.abs(self.arm_q() - q_des)))

    # ---- 手部位置伺服 ----
    def drive_hand(self, angles: tuple[float, ...]) -> None:
        for aid, ang in zip(self.hand_act, angles):
            lo, hi = self.m.actuator_ctrlrange[aid]
            self.d.ctrl[aid] = float(np.clip(ang, lo, hi))

    def hand_error(self, angles: tuple[float, ...]) -> float:
        m, d = self.m, self.d
        worst = 0.0
        for jn, tgt in zip(HAND_MOTORS, angles):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            worst = max(worst, abs(d.qpos[m.jnt_qposadr[jid]] - tgt))
        return worst

    # ---- 查询 ----
    def hand_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return (self.d.xpos[self.hand_body].copy(),
                self.d.xmat[self.hand_body].reshape(3, 3).copy())

    def bottle_pos(self) -> np.ndarray:
        return self.d.xpos[self.bottle_body].copy()


def home_arm_q(rig: Rig) -> np.ndarray:
    """自然下垂的起始关节角。"""
    q = np.zeros(len(ARM_JOINTS))
    q[0] = -0.10    # shoulder_y 略前
    q[1] = -0.16    # shoulder_x 略外展（右臂外展上限只有 +20°，取负方向）
    q[3] = 0.30     # elbow_y 微屈
    return q


# ---------------------------------------------------------------------------
# 手臂逆运动学（阻尼最小二乘，用 MuJoCo 自己的雅可比）
# ---------------------------------------------------------------------------


class ArmIK:
    """把 ``right_hand_base_link`` 摆到指定位姿，解 7 个右臂关节角。

    两种约束模式：

    * ``target_R``   —— 位置 3 + 姿态 3 = 6 个约束，姿态完全指定。
    * ``axis_y``     —— 位置 3 + 「手根 Y 轴指向」2 = 5 个约束。
      抓瓶子必须用这个：瓶轴要竖直 ⟺ 手根 Y 要竖直；而**绕瓶轴的滚转**（拇指朝上
      还是朝下）不影响能不能握住，硬约束 6 自由度反而解不出来（实测三腕全贴限位、
      位置误差 180 mm）。留 1 个自由度后立刻收敛。
    """

    def __init__(self, rig: Rig, w_rot: float = 0.35, w_axis: float = 0.30,
                 lam: float = 0.03) -> None:
        self.rig = rig
        self.m = rig.m
        self.d = rig.d
        self.w_rot = w_rot
        self.w_axis = w_axis
        self.lam = lam

    def _make_targets(self, target_pos, target_R, axis_y) -> dict:
        t: dict = {"pos": np.asarray(target_pos, dtype=float)}
        if axis_y is not None:
            v = np.asarray(axis_y, dtype=float)
            t["axis_y"] = v / (np.linalg.norm(v) + 1e-12)
        elif target_R is not None:
            t["rot"] = np.asarray(target_R, dtype=float)
        else:
            raise ValueError("必须给 target_R 或 axis_y 之一")
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

            rows: list[tuple[np.ndarray, np.ndarray]] = [(Jp, t["pos"] - p)]
            if "rot" in t:
                rows.append((self.w_rot * Jr, self.w_rot * rot_error(t["rot"], R)))
            if "axis_y" in t:
                y = R[:, 1]
                skew = np.array([[0, -y[2], y[1]], [y[2], 0, -y[0]], [-y[1], y[0], 0]])
                rows.append((-self.w_axis * (skew @ Jr), self.w_axis * (t["axis_y"] - y)))

            J = np.vstack([r[0] for r in rows])
            e = np.concatenate([r[1] for r in rows])
            if np.linalg.norm(e) < 1e-5:
                break
            A = J @ J.T + (self.lam ** 2) * np.eye(J.shape[0])
            q = np.clip(q + 0.85 * (J.T @ np.linalg.solve(A, e)),
                        rig.arm_rng[:, 0], rig.arm_rng[:, 1])
        return q

    def _errors(self, q: np.ndarray, t: dict) -> tuple[float, float]:
        """返回 (位置误差, 姿态或轴向误差)。注意会把 qpos 恢复原状。"""
        m, d, rig = self.m, self.d, self.rig
        saved = d.qpos.copy()
        d.qpos[rig.arm_qadr] = q
        mujoco.mj_forward(m, d)
        p, R = rig.hand_pose()
        e_pos = float(np.linalg.norm(t["pos"] - p))
        e_rot = float(np.linalg.norm(rot_error(t["rot"], R))) if "rot" in t else \
            float(np.linalg.norm(t["axis_y"] - R[:, 1])) if "axis_y" in t else 0.0
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)
        return e_pos, e_rot

    def solve(self, target_pos, q_seed: np.ndarray | None = None,
              target_R: np.ndarray | None = None, axis_y=None,
              iters: int = 320, restarts: int = 28) -> tuple[np.ndarray, float, float]:
        """多起点求解，返回 (关节角, 位置误差, 姿态/轴向误差)。

        ``restarts=0`` 且给了 ``q_seed`` 时，只从该 seed 出发 —— 这是**轨迹连续性**
        的要求：沿路径逐点求解时必须依赖上一点的解，否则多起点会随机跳到另一个
        同位置不同滚转的解分支，手就会沿路旋转、指尖划弧把瓶子扫飞。
        """
        t = self._make_targets(target_pos, target_R, axis_y)

        rig = self.rig
        rng = np.random.default_rng(20260922)
        lo = np.maximum(rig.arm_rng[:, 0], -2.2)
        hi = np.minimum(rig.arm_rng[:, 1], 2.2)
        if q_seed is not None and restarts <= 0:
            seeds = [np.asarray(q_seed, dtype=float)]
        else:
            seeds = [np.asarray(q_seed, dtype=float)] if q_seed is not None else []
            seeds.append(home_arm_q(rig))
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


# ---------------------------------------------------------------------------
# 抓握几何规划
# ---------------------------------------------------------------------------


def plan_grasp(rig: Rig) -> dict:
    """解出抓取位姿，并由「实际手根位姿」反推瓶子与桌子的位置。"""
    m, d = rig.m, rig.d
    ik = ArmIK(rig)

    # 基座摆到站立高度。plan_grasp 只解 IK，瓶子位置无关紧要，
    # 但仍用 mj_resetData 初始化（qpos[:]=0 会把瓶子自由关节清零）。
    mujoco.mj_resetData(m, d)
    d.qpos[2] = BASE_Z
    d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(m, d)

    q_grasp, e_pos, e_axis = ik.solve(GRASP_HAND_TARGET, q_seed=home_arm_q(rig),
                                      axis_y=HAND_Y_AXIS)

    # 用解出来的关节角做正向运动学，拿**实际**手根位姿
    saved = d.qpos.copy()
    d.qpos[rig.arm_qadr] = q_grasp
    mujoco.mj_forward(m, d)
    p_hand, R_hand = rig.hand_pose()
    d.qpos[:] = saved
    mujoco.mj_forward(m, d)

    axis_pt = p_hand + R_hand @ BOTTLE_AXIS_IN_HAND
    axis_dir = R_hand @ np.array([0.0, 1.0, 0.0])   # 手根 Y → 世界

    # 瓶轴要竖直才放得住：检查实际倾角
    tilt = math.degrees(math.acos(min(1.0, abs(float(axis_dir[2])))))

    bottle_xyz = axis_pt.copy()
    table_top_z = float(bottle_xyz[2]) - BOTTLE_H

    return {
        "ik": ik,
        "q_grasp": q_grasp,
        "ik_pos_err": e_pos,
        "ik_axis_err": e_axis,
        "hand_pos": p_hand,
        "hand_R": R_hand,
        "axis_pt": axis_pt,
        "axis_dir": axis_dir,
        "axis_tilt_deg": tilt,
        "bottle_xyz": bottle_xyz,
        "table_top_z": table_top_z,
    }


def settle_arm_to(rig: Rig, q_target: np.ndarray, steps: int = 1400,
                  ramp: float = 0.5, hand_target: tuple = HAND_OPEN,
                  quiet: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """把手臂 PD 驱动到 q_target 并跑到收敛，返回 **实际** 手根位姿。

    PD 力矩撑不住自重，稳态手根会比指令低几十毫米 —— 所以「解一次 IK」
    得到的指令，真正执行出来并不落在目标点上。必须实测。
    """
    m, d = rig.m, rig.d
    q0 = rig.arm_q().copy()
    n_ramp = max(1, int(steps * ramp))
    for k in range(steps):
        a = min(1.0, (k + 1) / n_ramp)
        a = a * a * (3 - 2 * a)
        rig.drive_arm(q0 + a * (q_target - q0))
        rig.hold_lower_body()
        rig.drive_hand(hand_target)
        mujoco.mj_step(m, d)
        rig.pin_base()
    del quiet
    return rig.hand_pose()


def solve_reachable_arm(rig: Rig, ik: "ArmIK", p_des: np.ndarray,
                        q_seed: np.ndarray, iters: int = 5,
                        tol: float = 0.003, verbose: bool = True,
                        axis_y: np.ndarray | None = HAND_Y_AXIS,
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """外环迭代：反复抬高 IK 指令，直到 PD 驱动后的 **实际** 手根落在 p_des 上。

    每一轮：解 IK → PD 真跑到收敛 → 量实际手根 → 把残余误差加进下一轮指令。
    这样得到的 q 是「真机上按这个角度发下去，手臂会停在目标点」的角度。

    ``axis_y=None`` 表示不约束手根 Y 轴方向（起始位姿用）。
    """
    p_cmd = p_des.copy()
    q_cmd = q_seed
    p_act = p_des.copy()
    R_act = np.eye(3)
    # 记录「到目前为止最好的一轮」。外环是迭代补偿，不是单调收敛：一旦被桌板之类的
    # 环境顶住，越补越远（实测 333 → 695 → 819 mm 一路发散），必须停手并回退到最好的
    # 那一轮，否则会把一个荒谬的关节角当成标定结果传下去。
    best_err = float("inf")
    best = (np.asarray(q_seed, dtype=float).copy(), p_des.copy(), np.eye(3))
    for it in range(iters):
        # 第一轮用多起点找全局解；之后依赖上一轮的解当 seed（restarts=0），
        # 保证迭代沿同一条解分支收敛，而不是每轮跳到一个不同的滚转解。
        q_cmd, _, _ = ik.solve(p_cmd, q_seed=q_cmd, axis_y=axis_y,
                               restarts=(28 if it == 0 else 0))
        p_act, R_act = settle_arm_to(rig, q_cmd)
        err = p_des - p_act
        e = float(np.linalg.norm(err))
        if verbose:
            info(f"[外环] 第 {it + 1}/{iters} 轮：实际手根残余误差"
                 f" {e * 1000:5.1f} mm")
        if e < best_err:
            best_err = e
            best = (q_cmd.copy(), p_act.copy(), R_act.copy())
        elif e > best_err * 1.6 + 0.015:
            if verbose:
                info(f"[外环] 残余误差在变大（{e * 1000:.0f} mm）→ 停止补偿，"
                     f"回退到最好的一轮（{best_err * 1000:.1f} mm）")
            break
        if e < tol:
            return q_cmd, p_act, R_act
        p_cmd = p_cmd + err
    return best[0], best[1], best[2]


def calibrate_reachable_targets(rig: Rig, plan: dict, verbose: bool = True,
                                ) -> tuple[dict, np.ndarray, np.ndarray]:
    """在**当前场景下**把所有阶段目标标定成「PD 真能到位」的关节角。

    ⚠️ 必须在与正式跑**完全相同**的场景里做：桌子是跟着瓶子摆的，
    若标定时场景里没有桌子，手臂会「以为」能伸到某处，实际却被桌板顶住
    （实测手根偏低 150 mm，手直接伸不到瓶子）。
    """
    ik = plan["ik"]
    QA: dict[str, np.ndarray] = {}

    # 起始位：远离工作区，且不能落在桌板范围内
    QA["home"], p_home, _ = solve_reachable_arm(
        rig, ik, HOME_HAND_TARGET, np.zeros(len(ARM_JOINTS)), verbose=False)
    if verbose:
        info(f"[标定] 起始  手根 = [{p_home[0]:.3f}, {p_home[1]:.3f}, {p_home[2]:.3f}]")

    # 抓取位：整条动作的基准，瓶子位置由「实际抓取位姿」反推
    QA["grasp"], hand_pos, hand_R = solve_reachable_arm(
        rig, ik, GRASP_HAND_TARGET, plan["q_grasp"])
    if verbose:
        info(f"[标定] 抓取  手根 = [{hand_pos[0]:.3f}, {hand_pos[1]:.3f}, "
             f"{hand_pos[2]:.3f}]，残余 {float(np.linalg.norm(GRASP_HAND_TARGET - hand_pos)) * 1000:.1f} mm")

    # 其余目标全部基于「实际抓取位姿」往上 / 往外推，并逐个标定
    chain = (
        ("pre", np.asarray(APPROACH_HIGH, dtype=float), "高抬  "),
        ("above", hand_pos + np.array([0.0, 0.0, PREGRASP_LIFT]), "预抓取"),
        ("lift", hand_pos + LIFT_DELTA, "抬起  "),
        ("hand", hand_pos + HANDOVER_DELTA, "递交  "),
    )
    seed = QA["grasp"]
    for key, tgt, label in chain:
        QA[key], p, _ = solve_reachable_arm(rig, ik, tgt, seed, verbose=False)
        if verbose:
            info(f"[标定] {label}手根 = [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}]")
        seed = QA[key]
    return QA, hand_pos, hand_R


def explain_grasp() -> None:
    """打印「瓶轴在手根坐标系里该在哪」的推导结果（只读，不依赖模型文件）。"""
    info("抓握几何推导（合拢手 + 五指触觉片拟合 Ø65 mm 瓶）")
    info("=" * 62)
    info("做法：把 6 个手部电机打到抓握角，量出 5 个指尖触觉片在手根坐标系里的位置，")
    info("      再求解「轴沿手根 Y 的圆柱」的轴心，使 5 个指尖到轴的距离都等于瓶半径。")
    info("")
    info("6 个手部电机抓握角（度）：")
    names = ("拇指对掌", "拇指屈", "食指", "中指", "无名指", "小指")
    for n, a in zip(names, HAND_GRASP):
        info(f"    {n:<8} = {math.degrees(a):5.1f}°")
    info("    四指远端由 equality 以 ×1.155 跟随，不用单独给")
    info("")
    info("拟合结果：")
    info(f"    瓶轴在手根坐标系 = [{BOTTLE_AXIS_IN_HAND[0] * 1000:.0f}, "
         f"{BOTTLE_AXIS_IN_HAND[1] * 1000:.0f}, {BOTTLE_AXIS_IN_HAND[2] * 1000:.0f}] mm")
    info("    瓶轴方向         = 手根 Y 轴")
    info("    5 个指尖到瓶面的平均偏差 ≈ 1.3 mm，最大 ≈ 2.0 mm")
    info("")
    info("推论：既然瓶轴 = 手根 Y 轴，而瓶子要立在桌上，")
    info("      所以**抓取时手根的 Y 轴必须竖直**。脚本就是按这个约束解 IK 的。")


# ---------------------------------------------------------------------------
# 序列
# ---------------------------------------------------------------------------


def run_sequence(rig: Rig, plan: dict, mode: str, record_dir: str | None,
                 steps_scale: float = 1.0, verbose: bool = True) -> dict:
    m, d = rig.m, rig.d
    ik: ArmIK = plan["ik"]

    # 用模型默认位姿初始化：瓶子落在场景给定的桌面位置，机器人基座再抬到站立高度。
    # ⚠️ 绝不能用 d.qpos[:] = 0 —— 那会把瓶子的自由关节一起清零，瓶子直接被扔到世界原点。
    mujoco.mj_resetData(m, d)
    d.qpos[2] = BASE_Z
    rig.pin_base()
    mujoco.mj_forward(m, d)

    # 手臂直接摆到起始位（纯运动学，无运动过程）。
    # ⚠️ 不能从「模型零位」慢慢挪过来 —— 零位手臂就在工作区里，
    #    缓慢移动会把瓶子撞飞（实测瓶子水平位移 412 mm）。
    q_home_pre = plan.get("q_home")
    if q_home_pre is not None:
        d.qpos[rig.arm_qadr] = np.asarray(q_home_pre, dtype=float)
        rig.pin_base()

    _bp = rig.bottle_pos()
    _bb = float(_bp[2]) - BOTTLE_H
    if verbose:
        info(f"[场景] 瓶子初始位置 = [{_bp[0]:.3f}, {_bp[1]:.3f}, {_bp[2]:.3f}]"
             f"，瓶底 {_bb:.3f} m / 桌面 {plan['table_top_z']:.3f} m")
    if abs(_bb - plan["table_top_z"]) > 0.03:
        die(EXIT_FAILED,
            f"瓶子没落在桌面上（瓶底 {_bb:.3f} m，桌面 {plan['table_top_z']:.3f} m）。\n"
            "       → 场景 XML 与传入的 table_top_z 不一致，先核对 write_scene()")

    # 标定好的关节角（main 里逐轮补偿 PD 稳态误差得到）；缺失时回退到现场解 IK
    QA = plan.get("QA") or {}
    _qh = QA.get("home", plan.get("q_home"))
    q_home = np.asarray(_qh, dtype=float) if _qh is not None else home_arm_q(rig)
    q_grasp = np.asarray(QA.get("grasp", plan["q_grasp"]), dtype=float)
    p_grasp = plan["hand_pos"]

    def _ik_to(p_abs: np.ndarray, seed: np.ndarray, label: str) -> np.ndarray:
        # restarts=0 + 用上一段的解当 seed ⇒ 轨迹连续。
        # 多起点求解会在同位置的不同滚转解之间乱跳，手会沿路旋转、指尖划弧扫飞瓶子。
        q, ep, ea = ik.solve(p_abs, q_seed=seed, axis_y=HAND_Y_AXIS, restarts=0)
        if verbose:
            info(f"[IK] {label:<10} 位置误差 {ep * 1000:5.1f} mm，轴向误差 {ea:.3f}")
        return q

    def reach(delta: np.ndarray, seed: np.ndarray, label: str) -> np.ndarray:
        return _ik_to(p_grasp + delta, seed, label)

    def reach_abs(p_abs: np.ndarray, seed: np.ndarray, label: str) -> np.ndarray:
        return _ik_to(p_abs, seed, label)

    # 各阶段目标：优先用 main 里标定好的关节角（那些角是「发下去手臂真会停在
    # 目标点」的角）。没标定数据时才退化成现场解 IK —— 后者会有几十毫米的
    # PD 稳态误差，手会伸不到瓶子那里。
    q_pre = np.asarray(QA["pre"], dtype=float) if "pre" in QA \
        else reach_abs(APPROACH_HIGH, q_grasp, "高抬")
    q_above = np.asarray(QA["above"], dtype=float) if "above" in QA \
        else reach(np.array([0.0, 0.0, PREGRASP_LIFT]), q_pre, "预抓取")
    q_lift = np.asarray(QA["lift"], dtype=float) if "lift" in QA \
        else reach(LIFT_DELTA, q_above, "抬起")
    q_hand = np.asarray(QA["hand"], dtype=float) if "hand" in QA \
        else reach(HANDOVER_DELTA, q_lift, "递交")

    # ---- 阶段表 ----
    S = max(1, int(round(steps_scale)))
    stages: list[tuple[str, int, np.ndarray, tuple]] = []
    if mode == "hand":
        stages += [("锁定手臂到抓取位姿", 500 * S, q_grasp, HAND_OPEN)]
    else:
        stages += [
            ("approach 抬起", 500 * S, q_pre, HAND_OPEN),
            ("approach 平移到瓶子上方", 600 * S, q_above, HAND_OPEN),
            ("descend 下降到抓取高度", 650 * S, q_grasp, HAND_OPEN),
        ]
    if mode in ("hand", "combo"):
        stages += [("grasp 手闭合", 800 * S, q_grasp, HAND_GRASP)]
    if mode != "hand":
        stages += [
            ("lift 抬起", 800 * S, q_above, HAND_GRASP),
            ("handover 递给对方", 1100 * S, q_hand, HAND_GRASP),
        ]
    if mode in ("hand", "combo"):
        stages += [("release 张开手", 700 * S, q_hand, HAND_OPEN)]
    if mode != "hand":
        stages += [("retract 收回", 800 * S, q_home, HAND_OPEN)]

    # ---- 录像准备 ----
    renderer = cam = None
    frames: list[np.ndarray] = []
    if record_dir:
        os.makedirs(record_dir, exist_ok=True)
        m.vis.global_.offwidth = max(int(m.vis.global_.offwidth), 960)
        m.vis.global_.offheight = max(int(m.vis.global_.offheight), 960)
        renderer = mujoco.Renderer(m, height=640, width=640)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(m, cam)
        # 默认右前 3/4 机位。正对前方（azimuth≈180）时，「往前递」这一步是朝着
        # 镜头走的，透视上会被机器人的胸口盖住，看起来像把瓶子杵在自己胸前。
        _az, _el, _ds = plan.get("cam", (250.0, -14.0, 1.45))
        cam.lookat[:] = [0.28, -0.26, 0.95]
        cam.distance = float(_ds)
        cam.azimuth = float(_az)
        cam.elevation = float(_el)
        if verbose:
            info(f"[录像] 机位 方位角 {_az:g}° / 俯仰 {_el:g}° / 距离 {_ds:g} m")

    # ---- 执行 ----
    # ⚠️ 关键：ArmIK.solve() 会把解算的中间状态留在 d.qpos 里（它只在 _errors()
    #    内部临时恢复）。上面几行 reach()/reach_abs() 调完 IK 后，手臂已经不在
    #    起始位，而是停在最后一次 IK 的解（实测停在「递交」位姿，手根偏了 430 mm），
    #    第一步就从那儿出发会直接扫过瓶子把它撞飞。这里必须重新摆回起始位。
    d.qpos[rig.arm_qadr] = q_home
    d.qvel[:] = 0.0
    rig.pin_base()
    if verbose:
        _h0 = rig.hand_pose()[0]
        info(f"[起始] 手根 = [{_h0[0]:+.3f}, {_h0[1]:+.3f}, {_h0[2]:+.3f}]"
             f"，离瓶心 {float(np.linalg.norm(_h0 - rig.bottle_pos())) * 1000:.0f} mm")

    bottle_z0 = float(rig.bottle_pos()[2])
    lift_max = 0.0
    hand_err_max = 0.0
    gstep = 0
    q_arm = q_home.copy()

    # ---- 工具：纯运动学求某组关节角对应的手根位置（不改动最终状态）----
    def fk_hand_pos(q: np.ndarray) -> np.ndarray:
        saved = d.qpos.copy()
        d.qpos[rig.arm_qadr] = q
        mujoco.mj_forward(m, d)
        p = rig.hand_pose()[0].copy()
        d.qpos[:] = saved
        mujoco.mj_forward(m, d)
        return p

    # ---- 刚性夹持：抓住的瞬间把瓶子锁在手根上，松手时解锁（见 GRIP_MODES 注释）----
    grip_mode = plan.get("grip_mode", "rigid")
    carry_mode = plan.get("carry_mode", "level")
    _bottle_geoms = []
    for _gn in ("bottle_body", "bottle_cap"):
        _g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, _gn)
        if _g >= 0:
            _bottle_geoms.append(int(_g))
    _dot = rig.bottle_dadr
    # mode: None=瓶子自由 / "carry"=跟着手根走 / "hold"=停在原地不动
    # R_level：夹持「锁定那一刻」瓶子的姿态。carry_mode=level 时全程沿用它，
    #          这样瓶子不会跟着手腕一起歪。
    grip_state: dict = {"mode": None, "rel": None, "hold": None, "R_level": None}

    def _bottle_set_free(free: bool) -> None:
        """瓶子是否参与碰撞（夹持期间关掉，免得嵌入网格把场景崩飞）。"""
        v = 1 if free else 0
        for _g in _bottle_geoms:
            m.geom_contype[_g] = v
            m.geom_conaffinity[_g] = v

    def _bottle_pin_to_table(pos: np.ndarray) -> None:
        d.qpos[rig.bottle_qadr:rig.bottle_qadr + 3] = pos
        d.qpos[rig.bottle_qadr + 3:rig.bottle_qadr + 7] = (1.0, 0.0, 0.0, 0.0)
        d.qvel[_dot:_dot + 6] = 0.0

    def grip_latch() -> None:
        """合拢完成的那一步：把瓶子摆正，记下「手根→瓶子」的固定相对位姿。"""
        _bottle_pin_to_table(np.asarray(plan["bottle_xyz"], dtype=float))
        mujoco.mj_forward(m, d)
        p_h, R_h = rig.hand_pose()
        T_h = np.eye(4)
        T_h[:3, :3], T_h[:3, 3] = R_h, p_h
        T_b = np.eye(4)
        T_b[:3, 3] = np.asarray(plan["bottle_xyz"], dtype=float)
        grip_state["rel"] = np.linalg.inv(T_h) @ T_b
        grip_state["R_level"] = T_b[:3, :3].copy()   # 锁定时的瓶子姿态（竖直）
        grip_state["mode"] = "carry"
        _bottle_set_free(False)
        if verbose:
            info("      ✱ 手已合拢 → 瓶子刚性锁定在手根上（--grip contact 可关掉）")

    def grip_release(hold: bool = True) -> None:
        """松手：瓶子交给对方（hold）或恢复自由落体（drop）。"""
        if hold:
            # 记下松手那一刻瓶子的世界位姿，之后每步原样写回 —— 相当于对方接住。
            T_b = np.eye(4)
            T_b[:3, 3] = rig.bottle_pos()
            T_b[:3, :3] = quat_to_matrix(d.qpos[rig.bottle_qadr + 3:
                                               rig.bottle_qadr + 7])
            grip_state["hold"] = T_b
            grip_state["mode"] = "hold"
            if verbose:
                _b = T_b[:3, 3]
                info(f"      ✱ 松手 → 瓶子停在 [{_b[0]:.3f}, {_b[1]:.3f}, {_b[2]:.3f}]"
                     f"（代表被对方接住；--release drop 可让它真掉）")
        else:
            grip_state["mode"] = None
            _bottle_set_free(True)
            d.qvel[_dot:_dot + 6] = 0.0
            if verbose:
                info("      ✱ 松手 → 瓶子恢复自由落体")

    def grip_follow() -> None:
        """每步把瓶子的自由关节写回「应到的位姿」。"""
        mode = grip_state["mode"]
        if mode == "carry" and grip_state["rel"] is not None:
            p_h, R_h = rig.hand_pose()
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R_h, p_h
            Tb = T @ grip_state["rel"]
            if carry_mode == "level" and grip_state["R_level"] is not None:
                # 位置跟着手走，姿态固定成锁定那一刻的（竖直）
                Tb[:3, :3] = grip_state["R_level"]
        elif mode == "hold" and grip_state["hold"] is not None:
            Tb = grip_state["hold"]
        else:
            return
        d.qpos[rig.bottle_qadr:rig.bottle_qadr + 3] = Tb[:3, 3]
        d.qpos[rig.bottle_qadr + 3:rig.bottle_qadr + 7] = quat_from_matrix(Tb[:3, :3])
        d.qvel[_dot:_dot + 6] = 0.0

    def run_seg(qa: np.ndarray, qb: np.ndarray, n_steps: int,
                hand_target: tuple) -> None:
        """驱动手臂从 qa 到 qb，共 n_steps 步。"""
        nonlocal lift_max, hand_err_max, q_arm, gstep
        for k in range(n_steps):
            a = min(1.0, (k + 1) / max(1.0, n_steps * 0.60))
            a = a * a * (3 - 2 * a)
            rig.drive_arm(qa + a * (qb - qa))
            rig.hold_lower_body()
            rig.drive_hand(hand_target)
            mujoco.mj_step(m, d)
            rig.pin_base()
            grip_follow()
            gstep += 1
            lift_max = max(lift_max, float(rig.bottle_pos()[2]) - bottle_z0)
            if verbose and gstep % 400 == 0:
                _h = rig.hand_pose()[0]
                _b = rig.bottle_pos()
                _tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                         "right_middle_tip_link")
                _t = d.xpos[_tid] if _tid >= 0 else _h
                info(f"        · 手根=[{_h[0]:+.3f},{_h[1]:+.3f},{_h[2]:+.3f}]"
                     f" 指尖=[{_t[0]:+.3f},{_t[1]:+.3f},{_t[2]:+.3f}]"
                     f" 瓶=[{_b[0]:+.3f},{_b[1]:+.3f},{_b[2]:+.3f}]")
            if renderer is not None and gstep % 16 == 0:
                renderer.update_scene(d, cam)
                frames.append(renderer.render().copy())
        q_arm = qb

    release_mode = plan.get("release_mode", "hold")
    # 刚性夹持时，瓶子从「手开始贴近它」的那一刻起就钉住不动。
    # 原因：官方手网格比 Ø65 mm 的瓶子「胖」20~40 mm（见文件头 GRIP_MODES 注释），
    # 手在下降和合拢的过程中一定会把瓶子推开（实测 descend 推 21 mm、grasp 推 95 mm），
    # 到锁定时再把它拽回设计位 —— 画面上就是瓶子瞬移一下。与其这样，
    # 不如让它在手靠近的那一刻就定住：瓶子不参与碰撞，也不被推走。
    # 只对「真的要抓」的 combo/hand 模式生效；--mode arm 保留真实碰撞，
    # 因为那个模式的意义就是「张开的手扫过瓶子会不会撞到」。
    bottle_pinned_from = None
    if grip_mode == "rigid" and mode != "arm":
        bottle_pinned_from = "descend"
        info(f"[夹持] 从「{bottle_pinned_from}」阶段起瓶子定在设计位不再参与碰撞"
             f"（官方手网格与 Ø65 mm 瓶不兼容，见文件头说明）")

    for name, steps, q_target, hand_target in stages:
        if verbose:
            info(f"  ▸ {name:<26} {steps} 步")
        _bx0 = rig.bottle_pos().copy()

        # 进入「手开始贴近瓶子」的阶段：钉住瓶子
        if (bottle_pinned_from and name.startswith(bottle_pinned_from)
                and grip_state["mode"] is None):
            _bottle_pin_to_table(np.asarray(plan["bottle_xyz"], dtype=float))
            _Tb = np.eye(4)
            _Tb[:3, 3] = np.asarray(plan["bottle_xyz"], dtype=float)
            grip_state["hold"] = _Tb
            grip_state["mode"] = "hold"
            _bottle_set_free(False)

        # 瓶子在本阶段开始时是否还是「自由/在桌上」的状态 —— 只在这种情况下，
        # 「水平移动」才代表「被手撞到了」，路径预检也才有意义。
        _was_free = grip_state["mode"] is None

        # 所有会动手臂的阶段都走笛卡尔直线：把这段路径切成若干小段，每段重解 IK。
        # 关节空间直接插值的话，末端轨迹完全不受控 —— 实测第一段「approach 抬起」
        # 就把桌上的瓶子扫飞出 544 mm，直接掉地。
        if any(w in name for w in CARTESIAN_KEYWORDS):
            # p0 必须取「当前真实手根位置」—— 不能先把 qpos 硬设成关节目标，
            # 那会让手臂瞬移（实测起点手根从 0.968 跳到 1.036），等于凭空撞一下。
            p0 = rig.hand_pose()[0].copy()
            # p1 用纯运动学目标（IK 也是纯运动学的，二者才自洽）：
            # IK(fk(q_grasp)) ≈ q_grasp，PD 的真实偏差由「瓶子按实际手根位置摆放」吸收。
            p1 = fk_hand_pos(q_target)

            # ---- 路径预检（纯运动学，不动动力学，所以绝不会碰倒瓶子）----
            # 沿这条直线撒点做 FK，量「机器人最近的部件中心」离瓶心多远。
            # 这只是中心到中心的粗估（部件本身还有体积），所以阈值留得比较宽。
            # 只在瓶子还是自由的时候才有意义：瓶子一旦被抓在手上就是跟着手走的，
            # 再拿「离瓶子当初的位置多远」判断撞不撞纯属误报。
            if verbose and _was_free:
                _env = {0}                       # world
                for _bn in ("table", "bottle"):
                    _i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, _bn)
                    if _i >= 0:
                        _env.add(int(_i))
                _saved = d.qpos.copy()
                _mind, _minseg = 1e9, -1
                _q0 = q_arm.copy()
                _qt = np.asarray(q_target, dtype=float)
                for si in range(CARTESIAN_SEGS + 1):
                    p_s = p0 + (p1 - p0) * (si / CARTESIAN_SEGS)
                    q_nom = _q0 + (_qt - _q0) * (si / CARTESIAN_SEGS)
                    q_s, _, _ = ik.solve(p_s, q_seed=q_nom,
                                         axis_y=HAND_Y_AXIS, restarts=0)
                    d.qpos[rig.arm_qadr] = q_s
                    mujoco.mj_forward(m, d)
                    for bid in range(m.nbody):
                        if bid in _env:
                            continue
                        _dd = float(np.linalg.norm(d.xpos[bid] - _bx0))
                        if _dd < _mind:
                            _mind, _minseg = _dd, si
                d.qpos[:] = _saved
                mujoco.mj_forward(m, d)
                info(f"      路径预检：最近部件离瓶心 {_mind * 1000:5.0f} mm"
                     f"（第 {_minseg}/{CARTESIAN_SEGS} 段，瓶半径 {BOTTLE_R * 1000:.0f} mm）"
                     f"{'   ⚠ 有撞瓶风险' if _mind < 0.12 else ''}")

            per = max(1, steps // CARTESIAN_SEGS)
            q_start = q_arm.copy()
            q_tgt = np.asarray(q_target, dtype=float)
            prev = q_start.copy()
            for si in range(1, CARTESIAN_SEGS + 1):
                p_s = p0 + (p1 - p0) * (si / CARTESIAN_SEGS)
                # ★ seed 用「起点→目标的关节空间插值」，**不要**用上一段的解。
                #   IK 只有 5 个约束、手臂 7 个自由度，留了 2 个自由量；只靠
                #   「上一段的解」当 seed 一路传下去，24 段累积后会漂到另一条解
                #   分支 —— 手的位置是对的，但关节构型已经面目全非，于是这阶段
                #   结束时手根停在 0.960（该到 0.860），下一阶段再插值回目标就
                #   把手臂整个抡了一圈（实测手根瞬时跑到 [-0.14,-0.54,1.10]）。
                #   每段都锚回关节插值线，漂移就累积不起来。
                q_nom = q_start + (q_tgt - q_start) * (si / CARTESIAN_SEGS)
                q_s, _, _ = ik.solve(p_s, q_seed=q_nom, axis_y=HAND_Y_AXIS, restarts=0)
                run_seg(prev, q_s, per, hand_target)
                prev = q_s
        else:
            run_seg(q_arm.copy(), q_target, steps, hand_target)

        # 只在阶段末尾（已收敛）统计手部跟踪误差 —— 起始瞬间必然是大误差，不具意义
        if hand_target != HAND_OPEN:
            hand_err_max = max(hand_err_max, rig.hand_error(hand_target))
        if verbose:
            _bp1 = rig.bottle_pos()
            _hp, _HR = rig.hand_pose()
            _dxy = float(np.hypot(_bp1[0] - _bx0[0], _bp1[1] - _bx0[1])) * 1000
            # 手根 Y 轴偏离竖直多少度 —— 「瓶子会不会歪」的根源指标。
            _tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(_HR[:, 1] @ HAND_Y_AXIS)))))
            if _was_free and grip_state["mode"] is None:
                # 瓶子全程自由 → 这个数字就是「有没有被手碰动」的判据
                _tail = f"  阶段内水平移动 {_dxy:5.1f} mm"
                _tail += "   ⚠ 瓶子被撞动了" if _dxy > 5 else ""
            else:
                # 瓶子在手上了 → 位移是「跟着手走了多远」，正常现象
                _tail = f"  瓶子随手动 {_dxy:5.1f} mm（夹持中，正常）"
            info(f"      ↳ 阶段末 手根=[{_hp[0]:.3f}, {_hp[1]:.3f}, {_hp[2]:.3f}]"
                 f"  瓶 z={_bp1[2]:.3f}  手根Y轴偏竖直 {_tilt:4.1f}°{_tail}")

        # ---- 抓握的锁定 / 解锁（必须等阶段跑完，手指才真的合拢/张开到位）----
        if grip_mode == "rigid":
            if any(w in name for w in GRIP_LATCH_KEYWORDS):
                grip_latch()
            if any(w in name for w in GRIP_RELEASE_KEYWORDS):
                _hold = (release_mode == "hold")
                grip_release(hold=_hold)
                # "hold"：关掉瓶子碰撞，后续每步 grip_follow() 把它按住不动
                # "drop"：恢复碰撞与自由落体
                _bottle_set_free(not _hold)

    res = {
        "bottle_lift_max_mm": lift_max * 1000.0,
        "bottle_final_mm": (float(rig.bottle_pos()[2]) - bottle_z0) * 1000.0,
        "hand_err_deg": math.degrees(hand_err_max),
        "n_frames": len(frames),
    }

    if renderer is not None and frames:
        try:
            from PIL import Image
            step = max(1, len(frames) // 110)
            imgs = [Image.fromarray(f) for f in frames[::step]]
            gif = os.path.join(record_dir, f"pick_bottle_{mode}.gif")
            imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                         duration=80, loop=0, optimize=True)
            res["gif"] = gif

            picks = [frames[i] for i in np.linspace(0, len(frames) - 1, 8).astype(int)]
            sheet = Image.new("RGB", (640 * 4, 640 * 2))
            for i, fr in enumerate(picks):
                sheet.paste(Image.fromarray(fr), ((i % 4) * 640, (i // 4) * 640))
            png = os.path.join(record_dir, f"pick_bottle_{mode}_关键帧.png")
            sheet.resize((4 * 320, 2 * 320)).save(png)
            res["contact_sheet"] = png
        except ImportError:
            info("[提示] 没装 Pillow，跳过录像输出（pip install pillow）")

    return res


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="revo2_pick_bottle_demo.py",
        description="ELF3 + Revo2 右手仿真：拿起瓶装水并递给对方",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--model", help="合并后的 elf3_revo2_right.xml 路径")
    ap.add_argument("--mode", choices=("arm", "hand", "combo"), default="combo",
                    help="arm=只动手臂 / hand=只动手 / combo=完整拿瓶子递人（默认）")
    ap.add_argument("--scene", default=None, help="场景 XML 输出路径（默认与模型同目录）")
    ap.add_argument("--record", default=None, help="输出 GIF 与关键帧拼图的目录")
    ap.add_argument("--steps-scale", type=float, default=1.0,
                    help="整体步数缩放，调大跑得更慢更稳")
    ap.add_argument("--explain-grasp", action="store_true",
                    help="只打印抓握几何的推导，不跑仿真")
    ap.add_argument("--grip", choices=GRIP_MODES, default="rigid",
                    help="rigid=合拢瞬间把瓶子刚性锁在手根（默认，推荐的演示方式）"
                         " / contact=纯接触摩擦抓取（官方手网格与 Ø65 mm 瓶不兼容，会崩飞）")
    ap.add_argument("--release", choices=RELEASE_MODES, default="hold",
                    help="hold=松手后瓶子原地停住，代表被对方接住（默认）"
                         " / drop=松手后自由落体")
    ap.add_argument("--carry", choices=CARRY_MODES, default="level",
                    help="level=搬运时瓶子始终竖直（默认，画面正常）"
                         " / rigid=瓶子完全跟着手腕转（会歪，像在洒水）")
    ap.add_argument("--cam", default="250,-14,1.45",
                    help="录像机位，格式 方位角,俯仰角,距离（默认 250,-14,1.45，"
                         "右前 3/4 视角）")
    args = ap.parse_args(argv)

    if args.explain_grasp:
        explain_grasp()
        return EXIT_OK

    if not args.model:
        die(EXIT_USAGE, "缺少 --model（或用 --explain-grasp 只看几何推导）")
    if not os.path.isfile(args.model):
        die(EXIT_USAGE, f"合并模型不存在：{args.model}")

    scene = args.scene or os.path.join(os.path.dirname(os.path.abspath(args.model)),
                                       "scene_bottle.xml")

    # ---- 第 0 遍：纯运动学初值。桌子丢到远处不干扰，只为解一次 IK 拿几何初值 ----
    write_scene(args.model, scene, np.array([1.5, 1.5, 0.30]), 0.75)
    rig = Rig(scene)
    info(f"[模型] nq={rig.m.nq} nv={rig.m.nv} nu={rig.m.nu}"
         f"（本体 {int((~rig.is_hand).sum())} + 手 {int(rig.is_hand.sum())}）")

    plan = plan_grasp(rig)
    info(f"[IK] 抓取位姿：位置误差 {plan['ik_pos_err'] * 1000:.1f} mm，"
         f"手根 Y 轴指向误差 {plan['ik_axis_err']:.3f}")
    info(f"[IK] 运动学手根位置 = [{plan['hand_pos'][0]:.3f}, {plan['hand_pos'][1]:.3f}, "
         f"{plan['hand_pos'][2]:.3f}]")

    # ---- 环境定型：桌子位置一旦定下就**冻结**，全程不再移动 ----
    # 这里原本是个循环依赖：桌子会顶住手臂（不能不管），桌子位置又由「手臂真停在
    # 哪儿」反推。上一版让桌子跟着瓶子每轮一起挪，结果瓶子微调 12 mm 就把桌板挪到
    # 起始位底下，第 4 轮标定直接崩（手根偏 819 mm）。
    # 结论：**桌子是环境，环境不能跟着目标跑**。桌子按设计初值摆一次，之后只动瓶子。
    table_xy = np.asarray(plan["bottle_xyz"][:2], dtype=float)
    table_top = float(plan["table_top_z"])
    info("")
    info(f"[环境] 桌子中心冻结在 [{table_xy[0]:.3f}, {table_xy[1]:.3f}]，"
         f"桌面高度 {table_top:.3f} m（全程不动）")
    info("[标定] PD 力矩撑不住自重，命令位姿与实际位姿差几十毫米；逐轮补偿中…")

    # ---- 第 1 遍：真实场景（桌子已冻结，瓶子在运动学设计位置）里标定 ----
    write_scene(args.model, scene, plan["bottle_xyz"], table_top,
                bottle_collide=False, table_xy=table_xy)
    rig = Rig(scene)
    plan["ik"] = ArmIK(rig)
    QA, hand_pos_real, hand_R_real = calibrate_reachable_targets(rig, plan)

    # ---- 第 2 遍：桌子不动，只把瓶子搬到「手臂实际会停的那个抓取点」上 ----
    # 瓶子 z 直接锁死在桌面上（瓶底 = 桌面），保证瓶子不会悬空或陷进桌板。
    _b_act = hand_pos_real + hand_R_real @ np.array(BOTTLE_AXIS_IN_HAND)
    bottle = np.array([_b_act[0], _b_act[1], table_top + BOTTLE_H])
    info(f"[几何] 设计瓶心 [{plan['bottle_xyz'][0]:.3f}, {plan['bottle_xyz'][1]:.3f}]"
         f" → 按实际手根反推 [{_b_act[0]:.3f}, {_b_act[1]:.3f}]"
         f" → 定案 [{bottle[0]:.3f}, {bottle[1]:.3f}]（水平挪 "
         f"{float(np.linalg.norm(bottle[:2] - np.asarray(plan['bottle_xyz'][:2]))) * 1000:.0f} mm）")

    write_scene(args.model, scene, bottle, table_top,
                bottle_collide=False, table_xy=table_xy)
    rig = Rig(scene)
    plan["ik"] = ArmIK(rig)
    QA, hand_pos_real, hand_R_real = calibrate_reachable_targets(rig, plan)

    _resid = float(np.linalg.norm(
        (hand_pos_real + hand_R_real @ np.array(BOTTLE_AXIS_IN_HAND)) - bottle))
    _tgt_err = float(np.linalg.norm(GRASP_HAND_TARGET - hand_pos_real)) * 1000
    info(f"[几何] 实际手根 [{hand_pos_real[0]:.3f}, {hand_pos_real[1]:.3f}, "
         f"{hand_pos_real[2]:.3f}]；偏离设计抓取点 {_tgt_err:.1f} mm，"
         f"瓶子↔手根残差 {_resid * 1000:.1f} mm")
    if _tgt_err > 30.0:
        info("⚠️ 实际抓取点偏离设计点较多 —— 多半是被桌板顶住了。"
             "可调 HOME_HAND_TARGET / GRASP_HAND_TARGET，或把桌子挪开")

    # ---- 执行场景：同一张桌子、同一个瓶子，只是瓶子恢复碰撞 ----
    write_scene(args.model, scene, bottle, table_top, table_xy=table_xy)
    rig = Rig(scene)
    plan["ik"] = ArmIK(rig)
    plan["QA"] = QA
    plan["q_home"] = QA["home"]
    plan["q_grasp"] = QA["grasp"]
    plan["hand_pos"] = hand_pos_real
    plan["hand_R"] = hand_R_real
    plan["bottle_xyz"] = bottle
    plan["table_top_z"] = table_top
    info(f"[几何] 瓶子位置 = [{bottle[0]:.3f}, {bottle[1]:.3f}, {bottle[2]:.3f}]，"
         f"桌面高度 {table_top:.3f} m")
    info(f"[场景] 已生成 {scene}")
    info(f"[模式] {args.mode}   抓握方式：{args.grip}   松手：{args.release}"
         f"   搬运姿态：{args.carry}")
    plan["grip_mode"] = args.grip
    plan["release_mode"] = args.release
    plan["carry_mode"] = args.carry
    try:
        _az, _el, _ds = (float(v) for v in args.cam.split(","))
        plan["cam"] = (_az, _el, _ds)
    except ValueError:
        die(EXIT_USAGE, f"--cam 格式应为 方位角,俯仰角,距离，收到：{args.cam}")

    res = run_sequence(rig, plan, args.mode, args.record,
                       steps_scale=args.steps_scale)

    info("")
    info("=" * 58)
    info("结果")
    info("=" * 58)
    info(f"  瓶子最大抬升 : {res['bottle_lift_max_mm']:7.1f} mm")
    info(f"  瓶子最终高度 : {res['bottle_final_mm']:7.1f} mm")
    info(f"  手部最大误差 : {res['hand_err_deg']:7.1f}°")
    if "gif" in res:
        info(f"  动图         : {res['gif']}")
    if "contact_sheet" in res:
        info(f"  关键帧       : {res['contact_sheet']}")

    if args.mode == "combo" and res["bottle_lift_max_mm"] < 40.0:
        info("")
        info("⚠️ 瓶子没被明显提起（< 40 mm）。常见原因与调整方向：")
        info("   ① 抓取高度/前后不对 → 调 GRASP_HAND_TARGET")
        info("   ② 抓握角不够贴合    → 调 HAND_GRASP 或 BOTTLE_AXIS_IN_HAND")
        info("   ③ 摩擦不足          → 调 bottle_body 的 friction")
        return EXIT_FAILED

    info("")
    if args.mode == "combo":
        info("✅ 动作完成：瓶子被抓起并递出")
    else:
        info("✅ 动作完成")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())

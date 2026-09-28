#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_mujoco_spectator.py —— 远程观察者：把 ROS 2 仿真渲染成视频

【为什么需要它】
仿真跑在一台只有 SSH 的远程机上，MuJoCo 的 GUI 窗口用的是自由相机，
默认不在机器人身上 —— 抓窗口截图只能看到远处的城市地形，看不见机器人本身。
而仿真的相机都装在机器人自己身上（胸口 / 头部），也拍不到自己。

【做法】
绕开 GUI：订阅机器人状态，写回**同一份官方模型**的 qpos，
用自己定义的相机离线渲染，直接输出 MP4。想看哪个角度就看哪个角度。

订阅：
  /simulation/odom              基座位姿（对应模型的 free joint `world_joint`）
  /simulation/actuator_states   31 个关节角（按名字映射到 qpos）

⚠️ 实测这两个话题都是 **BEST_EFFORT** QoS。用 rclpy 默认的 RELIABLE
   会报 "offering incompatible QoS" 并且**一条消息都收不到，还不抛异常**。

【相机方位角的坑 ★】
模型里机器人朝 **+x**；静止时右手腕在 y 为负的一侧（−0.178 m），即「身体右侧 = −y」。
  azimuth   0° → 从 +x 看   → 正面，双臂都在视野里
  azimuth  90° → 从 +y 看   → 机器人**左侧**，右臂被躯干完全挡住（拍了个寂寞）
  azimuth 315° → 从 +x,−y 看 → 右前三刻钟，**右臂离相机最近、轮廓最清楚** ✅
录右臂动作请用 270~330，不要用 90。

【双画面】
--split 时左半幅是全身机位、右半幅是第二机位特写。两个机位在同一次渲染循环里出，
时间严格同源。合帧后才叠 HUD，避免叠加层画两遍。

【骨架线】
--skeleton 会在画面里用连线把右臂「肩→肘→腕」标出来。坐标取的是模型真值
（data.xpos），不是估算，也**不改变**机器人本身的姿态或动力学。

【用法】
  MUJOCO_GL=egl python3 ros2_mujoco_spectator.py \
      --model /path/to/elf3.xml \
      --out demo.mp4 --seconds 18 --fps 30 \
      --cam-mode follow --azimuth 312 --distance 3.0 --elevation -8 \
      --split --cam2-distance 1.7 --cam2-follow-z-offset 0.18 \
      --skeleton --gif demo.gif
"""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

import mujoco

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from nav_msgs.msg import Odometry
from communication.msg import ActuatorStates


QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# 右臂骨架用的躯体（按优先级取第一个存在的）
SKEL_SLOTS = (
    ("肩", ("r_shoulder_z_link", "r_shoulder_x_link", "r_shoulder_y_link")),
    ("肘", ("r_elbow_y_link", "r_elbow_z_link")),
    ("腕", ("r_wrist_z_link", "r_wrist_y_link", "r_hand_base_link")),
)

# 下肢关节（腰 3 + 腿 12）。--freeze-lower 用它判断哪些关节冻结在首帧观测值。
LOWER_SET = frozenset((
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
    "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
    "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
))


class StateHub(Node):
    """把两个话题的最新值攒在一起。"""

    def __init__(self, odom_topic: str, state_topic: str) -> None:
        super().__init__("mujoco_spectator")
        self.base_pos = None
        self.base_quat = None          # (w, x, y, z)
        self.joints: dict = {}
        self.n_odom = 0
        self.n_state = 0

        self.create_subscription(Odometry, odom_topic, self._on_odom, QOS_BE)
        self.create_subscription(ActuatorStates, state_topic, self._on_state, QOS_BE)

    def _on_odom(self, msg: Odometry) -> None:
        # ★ /simulation/odom 实测是 nav_msgs/Odometry（不是 PoseStamped）
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.base_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        # ROS 四元数顺序是 (x, y, z, w)；MuJoCo free joint 要 (w, x, y, z)
        self.base_quat = np.array([q.w, q.x, q.y, q.z], dtype=np.float64)
        self.n_odom += 1

    def _on_state(self, msg: ActuatorStates) -> None:
        self.joints = dict(zip(list(msg.name), list(msg.position)))
        self.n_state += 1

    @property
    def ready(self) -> bool:
        return self.base_pos is not None and len(self.joints) >= 31


def build_joint_map(model: mujoco.MjModel) -> dict:
    """关节名 → qpos 下标。"""
    out = {}
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name:
            out[name] = int(model.jnt_qposadr[j])
    return out


def first_body(model: mujoco.MjModel, names):
    for n in names:
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        if b >= 0:
            return n, int(b)
    return None, -1


def draw_overlay(img: np.ndarray, t: float, total: float, joints: dict) -> None:
    """左上角叠时间与右臂关键角度（用 ASCII，避免中文字形缺失）。"""
    def put(txt, y, scale=0.55, color=(255, 255, 255)):
        cv2.putText(img, txt, (18, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, txt, (18, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 2, cv2.LINE_AA)

    put("ELF3  |  right-arm override via ROS 2  (/simulation/actuators_cmds_override)",
        34, 0.62)
    put(f"t = {t:5.1f} / {total:.0f} s", 66, 0.55, (180, 255, 180))
    for i, k in enumerate(("r_shoulder_y_joint", "r_elbow_y_joint",
                           "r_shoulder_x_joint")):
        v = joints.get(k)
        if v is not None:
            put(f"{k:22s} {np.degrees(v):+7.1f} deg", 98 + i * 26, 0.52,
                (200, 230, 255))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="官方 MuJoCo 模型 XML")
    ap.add_argument("--out", default="spectator.mp4", help="输出视频")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--delay", type=float, default=0.0,
                    help="就绪后再等 N 秒才开始录制/取静帧（用来等激励源跑到想要的时刻）")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1280,
                    help="单幅画面宽度；--split 时输出宽度是它的 2 倍")
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cam-mode", choices=("follow", "fixed", "tracking"), default="follow")
    ap.add_argument("--distance", type=float, default=4.2, help="自由相机距离 m")
    ap.add_argument("--azimuth", type=float, default=145.0, help="方位角 deg")
    ap.add_argument("--elevation", type=float, default=-12.0, help="俯仰角 deg")
    ap.add_argument("--follow-z-offset", type=float, default=-0.35,
                    help="follow 模式下视线高度相对基座的偏移 m（基座≈骨盆，"
                         "取负值把视线落到腰腹，机器人才能整体入画）")
    ap.add_argument("--stills", default=None,
                    help="逗号分隔的方位角列表；只出静帧拼图后退出（用来挑机位）")
    ap.add_argument("--stills-out", default=None, help="静帧拼图输出路径")
    ap.add_argument("--lookat", default="0,0,0.95", help="fixed 模式看向的世界点 x,y,z")
    ap.add_argument("--odom-topic", default="/simulation/odom")
    ap.add_argument("--state-topic", default="/simulation/actuator_states")
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("--gif", default=None, help="可选：同时输出 GIF（降分辨率）")
    ap.add_argument("--gif-width", type=int, default=640)
    ap.add_argument("--gif-stride", type=int, default=2, help="GIF 抽帧间隔")

    # ---- 双画面 ----
    ap.add_argument("--split", action="store_true",
                    help="左右双画面：左＝全身机位，右＝第二机位特写。"
                         "输出尺寸为 (2×--width) × --height")
    ap.add_argument("--cam2-distance", type=float, default=1.7)
    ap.add_argument("--cam2-azimuth", type=float, default=None, help="默认同 --azimuth")
    ap.add_argument("--cam2-elevation", type=float, default=None, help="默认同 --elevation")
    ap.add_argument("--cam2-follow-z-offset", type=float, default=0.18,
                    help="特写视线相对基座的偏移 m（正值抬到胸肩）")
    ap.add_argument("--cam2-lookat-offset", default=None,
                    help="特写视线的三维偏移 'x,y,z' m（相对机体）。给了它就覆盖 "
                         "--cam2-follow-z-offset。机器人朝 +x、右臂在 −y 侧，"
                         "所以把中心挪到右肩要用类似 '0,-0.22,0.10'")
    ap.add_argument("--label", default=None, help="左画面角标文字（ASCII）")
    ap.add_argument("--label2", default=None, help="右画面角标文字（ASCII）")

    # ---- 骨架线 ----
    ap.add_argument("--skeleton", action="store_true",
                    help="用连线标出右臂「肩→肘→腕」（坐标取模型真值，不影响动力学）")

    # ---- 基座 / 下肢 处理 ----
    ap.add_argument("--pin-base", action="store_true",
                    help="把基座（free joint）钉在录制开始时的位姿，只让关节角动。"
                         "用于：覆盖手臂时平衡控制器会迈步把机体带走（实测可达数米），"
                         "钉住基座能把镜头和背景固定住，画面只反映关节动作。"
                         "★ 这是**渲染期的选择**，不改变仿真本身的任何状态。")
    ap.add_argument("--freeze-lower", action="store_true",
                    help="下肢（腰 3 + 腿 12）不跟随 ROS 更新，冻结在首帧观测到的站姿。"
                         "配合 --pin-base 用：机体不动、下肢站姿干净，画面只剩右臂在动。"
                         "★ 同样是**渲染期的选择**，不影响仿真；实测锁定下肢关节"
                         "（arm_wave_demo.py --hold-legs）会让机器人失去踝策略而摔倒，"
                         "所以不能靠覆盖通道实现，只能在渲染侧做。")

    args = ap.parse_args()

    if not os.path.isfile(args.model):
        print(f"[错误] 模型不存在：{args.model}")
        return 1

    print(f"[模型] {args.model}")
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    print(f"       nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} ncam={model.ncam}")

    jmap = build_joint_map(model)
    need = ["waist_y_joint", "waist_x_joint", "waist_z_joint",
            "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
            "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
            "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
            "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
            "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
            "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint", "l_wrist_z_joint",
            "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
            "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
            "head_z_joint", "head_y_joint"]
    missing = [n for n in need if n not in jmap]
    if missing:
        print(f"[错误] 模型里缺少这些关节：{missing}")
        return 1
    print(f"       关节映射就绪：{len(need)} 个，基座 qpos 起始下标 = {jmap['world_joint']}")

    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        print(f"       已载入 keyframe 0 作为场景初值")
    else:
        mujoco.mj_resetData(model, data)

    # ---- 骨架点 ----
    skel = []
    if args.skeleton:
        for tag, cands in SKEL_SLOTS:
            nm, bid = first_body(model, cands)
            if bid >= 0:
                skel.append((tag, nm, bid))
        if len(skel) < 2:
            print(f"[警告] 骨架点只找到 {len(skel)} 个，--skeleton 忽略")
            skel = []
        else:
            print("[骨架] " + "  ".join(f"{t}={n}" for t, n, _ in skel))

    # ---- ROS ----
    rclpy.init()
    hub = StateHub(args.odom_topic, args.state_topic)
    print(f"[订阅] {args.odom_topic}")
    print(f"       {args.state_topic}")
    print("[等待] 等第一帧机器人状态 …")

    t_wait = time.time()
    while time.time() - t_wait < 20.0 and not hub.ready:
        rclpy.spin_once(hub, timeout_sec=0.05)
    if not hub.ready:
        print(f"[失败] 20 s 内没凑齐状态（odom {hub.n_odom} 帧 / 关节 {len(hub.joints)} 个）")
        print("       → 先确认话题名与 ROS_DOMAIN_ID；再确认 QoS 是不是 BEST_EFFORT")
        rclpy.shutdown()
        return 1
    print(f"[就绪] odom {hub.n_odom} 帧 / 关节 {len(hub.joints)} 个")

    # ---- 渲染器 ----
    # ★ 离屏帧缓冲默认只有 640x480（模型 XML 里的 <visual><global offwidth>）。
    #   不顶上去的话，Renderer 会直接报
    #   "Image width 960 > framebuffer width 640"。
    #   这里在构造 Renderer 之前改内存里的模型字段，避免去改动官方 XML 文件。
    old_w, old_h = model.vis.global_.offwidth, model.vis.global_.offheight
    model.vis.global_.offwidth = max(int(old_w), args.width)
    model.vis.global_.offheight = max(int(old_h), args.height)
    print(f"[渲染] 离屏帧缓冲 {old_w}x{old_h} → "
          f"{model.vis.global_.offwidth}x{model.vis.global_.offheight}"
          f"（单幅 {args.width}x{args.height}）")

    try:
        renderer = mujoco.Renderer(model, args.height, args.width)
    except Exception as exc:
        print(f"[失败] 建渲染器出错：{exc}")
        print("       → 若有 DISPLAY 缺失相关报错，设 MUJOCO_GL=egl；"
              "若报 framebuffer 太小，检查上面的 offwidth 是否已生效")
        rclpy.shutdown()
        return 1

    fixed_lookat = np.array([float(x) for x in args.lookat.split(",")], dtype=np.float64)

    def make_cam(distance, azimuth, elevation):
        c = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(c)
        c.distance = distance
        c.azimuth = azimuth
        c.elevation = elevation
        c.lookat[:] = fixed_lookat
        return c

    az2 = args.azimuth if args.cam2_azimuth is None else args.cam2_azimuth
    el2 = args.elevation if args.cam2_elevation is None else args.cam2_elevation
    cam = make_cam(args.distance, args.azimuth, args.elevation)
    cam2 = make_cam(args.cam2_distance, az2, el2)

    # 注视偏移：三维，相对机体世界坐标（机器人朝 +x、右臂在 −y 侧）
    off1 = np.array([0.0, 0.0, args.follow_z_offset], dtype=np.float64)
    if args.cam2_lookat_offset:
        off2 = np.array([float(v) for v in args.cam2_lookat_offset.split(",")],
                        dtype=np.float64)
        if off2.size != 3:
            print(f"[错误] --cam2-lookat-offset 需要 3 个数字，收到 {off2.size} 个")
            return 1
    else:
        off2 = np.array([0.0, 0.0, args.cam2_follow_z_offset], dtype=np.float64)

    if args.split:
        print(f"[相机] 左：距离 {args.distance} m / 方位 {args.azimuth}° / 俯仰 {args.elevation}°"
              f" / 注视偏移 z{args.follow_z_offset:+.2f}")
        print(f"       右：距离 {args.cam2_distance} m / 方位 {az2}° / 俯仰 {el2}°"
              f" / 注视偏移 {off2.round(3).tolist()}")
    else:
        print(f"[相机] 距离 {args.distance} m / 方位 {args.azimuth}° / 俯仰 {args.elevation}°")

    base_frozen = [None]        # --pin-base 时存 (pos, quat) 的录制起始值
    lower_held = {}             # --freeze-lower 时存 关节名 → 首帧角度
    root_body = int(model.jnt_bodyid[0])

    def apply_state() -> None:
        """把最新 ROS 状态写进 qpos。"""
        if hub.base_pos is not None:
            if args.pin_base:
                if base_frozen[0] is None:
                    base_frozen[0] = (hub.base_pos.copy(), hub.base_quat.copy())
                data.qpos[0:3] = base_frozen[0][0]
                data.qpos[3:7] = base_frozen[0][1]
            else:
                data.qpos[0:3] = hub.base_pos
                data.qpos[3:7] = hub.base_quat
        for nm in need:
            v = hub.joints.get(nm)
            if v is None:
                continue
            if args.freeze_lower and nm in LOWER_SET:
                lower_held.setdefault(nm, v)
                v = lower_held[nm]
            data.qpos[jmap[nm]] = v
        mujoco.mj_forward(model, data)

    # ★ 视线中心用「模型自己算出来的机体世界坐标」，不要用 odom。
    #   odom 的原点与 MuJoCo 世界系存在偏移（实测侧视时机器人明显偏出画面中心），
    #   data.xpos 是 mj_forward 后的真值，天然与模型一致。
    print(f"[注视] 机体 body#{root_body} "
          f"'{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, root_body)}'"
          + ("  （基座已钉住 --pin-base）" if args.pin_base else ""))
    if args.freeze_lower:
        print(f"[冻结] 下肢 {len(LOWER_SET)} 个关节不跟随 ROS，冻结在首帧站姿"
              f"（仅右臂 7 个关节按 ROS 实测更新）")

    def add_segment(scn, p1, p2, rgba, width=0.016) -> None:
        if scn.ngeom >= scn.maxgeom - 1:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3), np.zeros(3), np.eye(3).reshape(-1),
            np.asarray(rgba, dtype=np.float32))
        mujoco.mjv_connector(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE, float(width),
            np.asarray(p1, dtype=np.float64),
            np.asarray(p2, dtype=np.float64))
        scn.ngeom += 1

    def add_marker(scn, pos, radius, rgba) -> None:
        if scn.ngeom >= scn.maxgeom - 1:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, radius, radius], dtype=np.float64),
            np.asarray(pos, dtype=np.float64), np.eye(3).reshape(-1),
            np.asarray(rgba, dtype=np.float32))
        scn.ngeom += 1

    def draw_skeleton(scn) -> None:
        if not skel:
            return
        pts = [data.xpos[bid].copy() for _, _, bid in skel]
        for i in range(len(pts) - 1):
            add_segment(scn, pts[i], pts[i + 1], (1.0, 0.30, 0.05, 0.95), width=0.026)
        # 关节处放"动捕点"：连杆埋在网格内部看不见，能露出来的是球
        for i, p in enumerate(pts):
            add_marker(scn, p, 0.060 if i == len(pts) - 1 else 0.048,
                       (0.10, 0.85, 1.0, 0.98))

    def render_cam(c, off):
        c.lookat[:] = data.xpos[root_body] + np.asarray(off, dtype=np.float64)
        renderer.update_scene(data, camera=c)
        if args.skeleton:
            draw_skeleton(renderer.scene)
        # ★ render() 返回的可能是内部缓冲的视图，双画面时必须 copy
        return np.array(renderer.render(), copy=True)

    # ---- 只出静帧拼图（挑机位用）----
    if args.stills:
        if args.delay > 0:
            print(f"[等待] {args.delay:.1f} s 后再取静帧 …")
            t_d = time.time()
            while time.time() - t_d < args.delay:
                rclpy.spin_once(hub, timeout_sec=0.02)
        azimuths = [float(x) for x in args.stills.split(",")]
        out_png = args.stills_out or "stills.png"
        tiles = []
        for az in azimuths:
            rclpy.spin_once(hub, timeout_sec=0.0)
            apply_state()
            cam.azimuth = az
            rgb = render_cam(cam, off1)
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.putText(frame, f"azimuth {az:g}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(frame, f"azimuth {az:g}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (80, 255, 120), 2, cv2.LINE_AA)
            tiles.append(cv2.resize(frame, (args.width // 2, args.height // 2)))
        rows = [np.hstack(tiles[i:i + 2]) for i in range(0, len(tiles), 2)]
        cv2.imwrite(out_png, np.vstack(rows))
        print(f"[静帧] 方位角 {azimuths} → {out_png}")
        renderer.close()
        hub.destroy_node()
        rclpy.shutdown()
        return 0

    # ---- 视频写出 ----
    out_w = args.width * (2 if args.split else 1)
    out_h = args.height

    writer = None
    fourcc = None
    if HAS_CV2:
        for cc in ("mp4v", "avc1", "MJPG"):
            w = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*cc),
                                args.fps, (out_w, out_h))
            if w.isOpened():
                writer, fourcc = w, cc
                break
            w.release()
    if writer is None:
        if HAS_CV2:
            print("[警告] 没有可用的 MP4 编码器，改为只输出 GIF")
        else:
            print("[警告] 没有 opencv，改为只输出 GIF")

    gif_frames = []

    if args.delay > 0:
        print(f"[等待] {args.delay:.1f} s 后再开始录制 …")
        t_d = time.time()
        while time.time() - t_d < args.delay:
            rclpy.spin_once(hub, timeout_sec=0.02)

    print(f"[录制] {args.seconds:.0f} s @ {args.fps:.0f} fps → {args.out}"
          f"  ({out_w}x{out_h})")
    t0 = time.time()
    n_frame = 0
    periods = []
    next_t = 0.0
    trace_wrist = []    # 腕**相对躯体**的位置 → 这才是"手臂动作"的量
    trace_base = []     # 躯体的世界位置      → 这是"机器人有没有被平衡控制器带走"

    while True:
        elapsed = time.time() - t0
        if elapsed > args.seconds:
            break
        if elapsed < next_t:
            time.sleep(min(next_t - elapsed, 0.005))
            continue
        next_t += 1.0 / args.fps

        loop_start = time.time()

        # 状态 → qpos
        apply_state()

        if args.cam_mode == "tracking":
            renderer.update_scene(data,
                                  camera=("tracking" if model.ncam > 0 else None))
            if args.skeleton:
                draw_skeleton(renderer.scene)
            left = cv2.cvtColor(np.array(renderer.render(), copy=True),
                                cv2.COLOR_RGB2BGR)
        else:
            left = cv2.cvtColor(render_cam(cam, off1),
                                cv2.COLOR_RGB2BGR)

        # ② 用 odom 的真值而不是渲染用的 xpos —— 这样即使 --pin-base 也能测出真实走动量
        if hub.base_pos is not None:
            trace_base.append(np.asarray(hub.base_pos, dtype=np.float64).copy())
        if skel:
            trace_wrist.append(
                (data.xpos[skel[-1][2]] - data.xpos[root_body]).copy())

        if args.split:
            right = cv2.cvtColor(render_cam(cam2, off2),
                                 cv2.COLOR_RGB2BGR)
            frame_for_video = np.hstack([left, right])
            if args.label:
                cv2.putText(frame_for_video, args.label,
                            (18, out_h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(frame_for_video, args.label,
                            (18, out_h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (120, 240, 255), 2, cv2.LINE_AA)
            if args.label2:
                cv2.putText(frame_for_video, args.label2,
                            (args.width + 18, out_h - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(frame_for_video, args.label2,
                            (args.width + 18, out_h - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (120, 240, 255), 2, cv2.LINE_AA)
            # 渲染期处理必须写在画面上，不能只在日志里 —— 否则视频脱离上下文会被误读
            notes = []
            if args.pin_base:
                notes.append("base pinned")
            if args.freeze_lower:
                notes.append("lower body held at t0")
            if notes:
                txt = ("render note: " + " + ".join(notes)
                       + "  |  arm joints are live ROS data")
                cv2.putText(frame_for_video, txt, (18, out_h - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame_for_video, txt, (18, out_h - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (150, 220, 150), 1, cv2.LINE_AA)
            cv2.line(frame_for_video, (args.width, 0), (args.width, out_h),
                     (70, 70, 70), 2)
        else:
            frame_for_video = left

        if not args.no_overlay and HAS_CV2:
            draw_overlay(frame_for_video, elapsed, args.seconds, hub.joints)

        if writer is not None and frame_for_video is not None:
            writer.write(frame_for_video)

        if args.gif and n_frame % max(1, args.gif_stride) == 0 and HAS_CV2:
            gw = min(args.gif_width, out_w)
            gh = int(round(gw * out_h / out_w))
            small = cv2.resize(frame_for_video, (gw, gh), interpolation=cv2.INTER_AREA)
            gif_frames.append(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))

        n_frame += 1
        periods.append(time.time() - loop_start)
        if n_frame % max(1, int(args.fps)) == 0:
            rclpy.spin_once(hub, timeout_sec=0.0)
            print(f"   [{elapsed:5.1f}s] 第 {n_frame} 帧  "
                  f"渲染 {np.mean(periods[-30:]) * 1000:.0f} ms/帧")
        else:
            rclpy.spin_once(hub, timeout_sec=0.0)

    if writer is not None:
        writer.release()
        print(f"[完成] 写出 {n_frame} 帧 → {args.out}（编码 {fourcc}）")
        print(f"       文件大小 {os.path.getsize(args.out) / 1e6:.1f} MB")

    if args.gif and gif_frames:
        from PIL import Image
        dur = int(round(1000.0 / max(1.0, args.fps / max(1, args.gif_stride))))
        imgs = [Image.fromarray(a) for a in gif_frames]
        imgs[0].save(args.gif, save_all=True, append_images=imgs[1:],
                     duration=dur, loop=0, optimize=True)
        print(f"[完成] GIF {len(gif_frames)} 帧 @ {dur} ms → {args.gif}"
              f"（{os.path.getsize(args.gif) / 1e6:.1f} MB）")

    # ★ 客观量化。两个口径必须分开报，否则会把"机器人走了"算成"手臂动了"：
    #   ① 腕相对躯体的位移  → 手臂动作
    #   ② 躯体的世界位移    → 平衡控制器有没有迈步把机体带走
    if trace_wrist:
        arr = np.vstack(trace_wrist)
        rel = arr - arr[0]
        dist = np.linalg.norm(rel, axis=1) * 1000.0
        k = int(np.argmax(dist))
        print()
        print("[① 手臂动作] 右腕相对躯体的位移（这才是手臂本身的行程）")
        print(f"       最大 {dist.max():7.1f} mm  @ 第 {k} 帧（{k / args.fps:.1f} s）")
        print(f"       末帧 {dist[-1]:7.1f} mm")
        print(f"       分量 x {rel[k][0]*1000:+7.1f}  y {rel[k][1]*1000:+7.1f}  "
              f"z {rel[k][2]*1000:+7.1f} mm")

    if trace_base:
        arr = np.vstack(trace_base)
        rel = arr - arr[0]
        dist = np.linalg.norm(rel, axis=1) * 1000.0
        k = int(np.argmax(dist))
        print()
        print("[② 机体走动] 躯体在世界系里的位移（越小越好；大=平衡控制器在迈步）")
        print(f"       最大 {dist.max() / 1000:6.3f} m  @ 第 {k} 帧（{k / args.fps:.1f} s）")
        print(f"       末帧 {dist[-1] / 1000:6.3f} m   "
              f"终点 ({rel[-1][0]:+.3f}, {rel[-1][1]:+.3f}, {rel[-1][2]:+.3f}) m")
        if args.pin_base:
            print("       ⚠️ 本次用了 --pin-base，渲染时基座被钉住，实际走动量已不影响画面")
        elif dist.max() > 0.5:
            print("       ⚠️ 走动量 >0.5 m，镜头会跟着跑、背景会变 → "
                  "建议降幅度（arm_wave_demo.py --scale）或加 --pin-base")

    hub.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

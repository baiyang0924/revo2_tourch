#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Revo2 触觉版灵巧手 · 摄像头手势模仿（本机直连 GUI）

用笔记本摄像头识别右手，实时映射到强脑 Revo2 触觉版灵巧手（6 个电机），
手通过 RS485/Modbus 直接连在本机（不经过机器人本体）。

界面分三块：
  1. 视频画面 —— 摄像头 + 手部关键点 + 6 通道数值
  2. 力矩显示 —— 各指电机电流（电流 ≈ 力矩，读协议寄存器 2012–2017）
     以及触觉法向力（寄存器 4200–4229，官方语义：法向力/100 = 牛顿）
  3. 力矩调节 —— 各指保护电流滑块（寄存器 930–935，100–1500 mA，默认 500），
     松手即写入，立即生效；保护电流越大，单指能出的最大力越大

安全约定：
  * 写动作指令走 revo2_modbus 的 set_angles，自带角度限位（0–59/89/81 度）
    和 allow_write 门禁
  * 启动后默认「未使能」，点「开始跟随」才真正下发运动指令
  * 「急停」只停止下发、不额外发指令（手停在当前位）
  * 「全部张开」是显式的 0° 一次性指令

用法：
  python revo2_touch_gui.py                # 自动探测串口，默认右手(127)
  python revo2_touch_gui.py --dry-run      # 只跑识别和界面，不连手
  python revo2_touch_gui.py --id 126       # 左手
  python revo2_touch_gui.py --model D:\\path\\hand_landmarker.task

⚠️ 运行前先关闭强脑「上位机」软件，否则串口被占用，本程序打不开。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import statistics
import sys
import threading
import time

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

try:
    from PIL import Image, ImageTk
except Exception:  # pragma: no cover
    Image = ImageTk = None

import tkinter as tk
from tkinter import ttk, messagebox

# 找 revo2_modbus：优先同目录（整个文件夹拷走也能独立跑），
# 其次上一级目录（仓库内布局：src/revo2_standalone/gui/ → src/revo2_standalone/）
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

import revo2_modbus as r2m  # noqa: E402

# ══════════════════════════════════════════════════════════════
# 常量与手势映射（复用摄像头遥操 v2 的算法）
# ══════════════════════════════════════════════════════════════

W = 0
T_CMC, T_MCP, T_IP, T_TIP = 1, 2, 3, 4
I_MCP, I_PIP, I_DIP, I_TIP = 5, 6, 7, 8
M_MCP, M_PIP, M_DIP, M_TIP = 9, 10, 11, 12
R_MCP, R_PIP, R_DIP, R_TIP = 13, 14, 15, 16
P_MCP, P_PIP, P_DIP, P_TIP = 17, 18, 19, 20

NAMES = ["拇指屈", "拇指掌", "食指", "中指", "无名指", "小指"]

# 每个通道原生值默认量程 (张开, 闭合)；做过标定后整表替换。
# 拇指掌（对掌关节）的量程按「自然张开」定 0 点：
#   * 实测：手掌伸直、拇指自然外展时 ≈ -15°（旧默认按「最大外伸 -55°」当 0 点，
#     同样的姿势会算成 0.58 → 拇指被收到 ~51°，看起来是"往掌心收拢"，2026-10-08 修正）
#   * 拇指自然外展 (-15°) → 0（完全向外张开）
#   * 拇指压向掌心 (+20°) → 1（完全收向掌心）
DEFAULT_RANGE = [
    (0.05, 0.88),      # 拇指屈
    (-15.0, 20.0),     # 拇指掌（度）：自然张开≈-15，贴掌≈+20
    (0.08, 0.95),      # 食指
    (0.08, 0.95),
    (0.08, 0.95),
    (0.08, 0.95),
]

# 各通道死区：归一化值低于死区一律按 0 处理（软死区，平滑过渡不会跳变）。
# 拇指掌给 0.10：自然张开时"完全外展"状态稳定不抖，稍微动一下才跟。
DEADZONE = [0.02, 0.10, 0.02, 0.02, 0.02, 0.02]

# 各电机角度上限（度），与 revo2_modbus.ANGLE_LIMITS 一致。
# 归一化值 0~1 乘上限 = 目标角度。
ANGLE_LIMITS = [59.0, 89.0, 81.0, 81.0, 81.0, 81.0]

CALIB_FILE = "teleop_calib.json"
SAMPLE_FRAMES = 30
SAMPLE_TIMEOUT = 6.0

# 触觉寄存器（官方语义：法向力 = 值 ÷ 100，单位 N，量程 0–25 N）
REG_TOUCH_FORCE_CAP = 4200
REG_TOUCH_FORCE_COUNT = 30

# 识别模型候选路径：程序同目录（一键启动会自动下载到这里）
MODEL_CANDIDATES = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task"),
    "hand_landmarker.task",
]


def d3(a, b):
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def clamp(v, lo=0.0, hi=1.0):
    return max(lo, min(hi, v))


def _sub(a, b):
    return (a.x - b.x, a.y - b.y, a.z - b.z)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def angle_at(a, b, c):
    v1, v2 = _sub(a, b), _sub(c, b)
    l1 = math.sqrt(_dot(v1, v1))
    l2 = math.sqrt(_dot(v2, v2))
    if l1 < 1e-9 or l2 < 1e-9:
        return 180.0
    cosv = max(-1.0, min(1.0, _dot(v1, v2) / (l1 * l2)))
    return math.degrees(math.acos(cosv))


def finger_curl(L, mcp, pip, dip, tip):
    bone = d3(L[mcp], L[pip]) + d3(L[pip], L[dip]) + d3(L[dip], L[tip])
    if bone < 1e-9:
        return 0.0
    r = d3(L[mcp], L[tip]) / bone
    return clamp((1.0 - r) / 0.55)


def thumb_aux_deg(L):
    u = _sub(L[I_MCP], L[W])
    v = _sub(L[P_MCP], L[W])
    n_raw = _cross(u, v)
    m = math.sqrt(_dot(n_raw, n_raw))
    if m < 1e-9:
        return -70.0
    n = (n_raw[0] / m, n_raw[1] / m, n_raw[2] / m)
    t = _sub(L[T_MCP], L[T_CMC])
    mt = math.sqrt(_dot(t, t))
    if mt < 1e-9:
        return -70.0
    t = (t[0] / mt, t[1] / mt, t[2] / mt)
    dn = _dot(t, n)
    t = (t[0] - n[0] * dn, t[1] - n[1] * dn, t[2] - n[2] * dn)
    mt = math.sqrt(_dot(t, t))
    if mt < 1e-9:
        return -70.0
    t = (t[0] / mt, t[1] / mt, t[2] / mt)
    return math.degrees(math.atan2(_dot(_cross(u, t), n), _dot(u, t)))


def thumb_flex(L):
    chain = finger_curl(L, T_MCP, T_IP, T_TIP, T_TIP)
    ip = clamp((168.0 - angle_at(L[T_MCP], L[T_IP], L[T_TIP])) / 60.0)
    return clamp(0.5 * chain + 0.5 * ip)


def native_values(L):
    return [
        thumb_flex(L),
        thumb_aux_deg(L),
        finger_curl(L, I_MCP, I_PIP, I_DIP, I_TIP),
        finger_curl(L, M_MCP, M_PIP, M_DIP, M_TIP),
        finger_curl(L, R_MCP, R_PIP, R_DIP, R_TIP),
        finger_curl(L, P_MCP, P_PIP, P_DIP, P_TIP),
    ]


def to_norm(native, open_v, closed_v):
    if abs(closed_v - open_v) < 1e-6:
        return 0.0
    return clamp((native - open_v) / (closed_v - open_v))


def median6(samples):
    cols = list(zip(*samples))
    return [statistics.median(c) for c in cols]


def calib_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), CALIB_FILE)


def load_calib():
    try:
        with open(calib_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
        if (isinstance(data.get("open"), list)
                and isinstance(data.get("closed"), list)
                and len(data["open"]) == 6 and len(data["closed"]) == 6):
            return data
    except Exception:
        pass
    return None


def save_calib(data):
    try:
        with open(calib_path(), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
    except Exception as exc:  # noqa: BLE001
        print("标定存盘失败：", exc)


# ══════════════════════════════════════════════════════════════
# 线程间共享状态
# ══════════════════════════════════════════════════════════════
class SafeBox:
    """camera 线程 → GUI / hand 线程 之间的最新值交换，加锁防竞态。"""

    def __init__(self):
        self._l = threading.Lock()
        self.targets_norm = [0.0] * 6   # 0~1，最新目标
        self.target_seq = 0             # 有新目标时递增
        self.currents = []              # 6 电流 mA
        self.positions = []             # 6 角度（度）
        self.motor_status = []          # 6 状态文字
        self.touch_raw = []             # 触觉原始寄存器
        self.touch_vendor = ""
        self.info_str = "未连接"
        self.armed = False
        self.protect_request = None     # GUI → hand：要写入的保护电流 [6]
        self.protect_echo = None        # hand → GUI：已生效的保护电流 [6]
        self.calib_request = None       # "open" / "close" / "clear"
        self.calib_state = "未标定，用默认量程"
        self.calib_ranges = [list(r) for r in DEFAULT_RANGE]
        self.hand_note = ""             # 状态栏提示

    def set_targets(self, norm):
        with self._l:
            self.targets_norm = list(norm)
            self.target_seq += 1

    def get_targets(self):
        with self._l:
            return list(self.targets_norm), self.target_seq

    def get_armed(self):
        with self._l:
            return self.armed

    def set_armed(self, v):
        with self._l:
            self.armed = v

    def set_telemetry(self, currents, positions, status):
        with self._l:
            self.currents = currents
            self.positions = positions
            self.motor_status = status

    def set_touch(self, raw, vendor):
        with self._l:
            self.touch_raw = raw
            self.touch_vendor = vendor

    def set_info(self, s):
        with self._l:
            self.info_str = s

    def pop_protect_request(self):
        with self._l:
            v, self.protect_request = self.protect_request, None
            return v

    def push_protect_request(self, vals):
        with self._l:
            self.protect_request = list(vals)

    def set_protect_echo(self, vals):
        with self._l:
            self.protect_echo = list(vals)

    def set_calib_request(self, kind):
        with self._l:
            self.calib_request = kind

    def pop_calib_request(self):
        with self._l:
            v, self.calib_request = self.calib_request, None
            return v

    def set_calib_state(self, s):
        with self._l:
            self.calib_state = s

    def set_note(self, s):
        with self._l:
            self.hand_note = s

    def snapshot(self):
        """GUI 轮询用的一次性快照。"""
        with self._l:
            return {
                "targets": list(self.targets_norm),
                "currents": list(self.currents),
                "positions": list(self.positions),
                "motor_status": list(self.motor_status),
                "touch_raw": list(self.touch_raw),
                "touch_vendor": self.touch_vendor,
                "info": self.info_str,
                "armed": self.armed,
                "protect_echo": list(self.protect_echo) if self.protect_echo else None,
                "calib_state": self.calib_state,
                "note": self.hand_note,
            }


# ══════════════════════════════════════════════════════════════
# 工作线程
# ══════════════════════════════════════════════════════════════
def camera_worker(box: SafeBox, stop: threading.Event, frame_q: queue.Queue,
                  args):
    """摄像头采集 + MediaPipe 识别 → 目标归一化值 + 标注帧。"""
    try:
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=args.model),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        landmarker = vision.HandLandmarker.create_from_options(opts)
    except Exception as exc:  # noqa: BLE001
        box.set_note(f"MediaPipe 初始化失败：{exc}")
        return

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        box.set_note(f"打不开摄像头 {args.cam}")
        landmarker.close()
        return
    # 延迟优化：只保留最新帧，丢掉驱动内部缓冲；分辨率拉高提升远处/小手的识别精度
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    ranges = [list(r) for r in DEFAULT_RANGE]
    calib = load_calib()
    if calib:
        ranges = [[o, c] for o, c in zip(calib["open"], calib["closed"])]
        box.set_calib_state("已加载标定")

    t0 = int(time.time() * 1000)
    smooth = [0.0] * 6
    pending = None       # 标定采样：{"label","samples","t0"}
    fps_t, frames, fps = time.time(), 0, 0.0

    while not stop.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ts = t0 + int((time.time() * 1000) - t0)
        res = landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

        # 标定采样（GUI 按钮触发）
        req = box.pop_calib_request()
        if req == "clear":
            calib = None
            ranges = [list(r) for r in DEFAULT_RANGE]
            try:
                os.remove(calib_path())
            except FileNotFoundError:
                pass
            box.set_calib_state("已清除标定，退回默认量程")
        elif req in ("open", "close") and not pending:
            pending = {"label": req, "samples": [], "t0": time.time()}
            if req == "open":
                box.set_calib_state("采样中：自然张开（手指伸直、拇指自然外展即可，"
                                    "不用伸到最开；保持约 1 秒）")
            else:
                box.set_calib_state("采样中：闭合（握拳、拇指压向掌心；保持约 1 秒）")

        vals_norm = None
        handlabel = ""
        if res.hand_landmarks:
            skip = False
            if res.handedness:
                mp_side = res.handedness[0][0].category_name
                user_side = "right" if mp_side == "Left" else "left"
                handlabel = f"{user_side}(MP={mp_side})"
                if args.hand != "auto" and user_side != args.hand:
                    skip = True
            if not skip:
                L = res.hand_landmarks[0]
                nat = native_values(L)

                if pending:
                    pending["samples"].append(nat)
                    if len(pending["samples"]) >= SAMPLE_FRAMES:
                        med = median6(pending["samples"])
                        calib = calib or {}
                        calib[pending["label"]] = med
                        if "open" in calib and "closed" in calib:
                            ranges = [[o, c] for o, c in
                                      zip(calib["open"], calib["closed"])]
                            save_calib(calib)
                            box.set_calib_state("✓ 标定完成并存盘")
                        else:
                            box.set_calib_state(
                                f"已记录{'张开' if pending['label'] == 'open' else '闭合'}，"
                                f"再采另一半即完成")
                        pending = None
                    elif time.time() - pending["t0"] > SAMPLE_TIMEOUT:
                        pending = None
                        box.set_calib_state("采样超时，请保持手在画面里重新采")

                norm = [to_norm(nat[i], ranges[i][0], ranges[i][1])
                        for i in range(6)]
                norm = [clamp(v * args.gain) for v in norm]
                # 自适应平滑：变化大 → 快速跟上（减延迟）；
                # 变化小 → 强平滑（抗抖动）；再叠加 0.02 死区压住静止微动。
                jump = max(abs(norm[i] - smooth[i]) for i in range(6))
                alpha = 0.75 if jump > 0.12 else 0.40
                for i in range(6):
                    smooth[i] = smooth[i] + alpha * (norm[i] - smooth[i])
                vals_norm = [0.0 if v <= DEADZONE[i]
                             else clamp((v - DEADZONE[i]) / (1.0 - DEADZONE[i]))
                             for i, v in enumerate(smooth)]

                h, w = frame.shape[:2]
                for p in L:
                    cv2.circle(frame, (int(p.x * w), int(p.y * h)),
                               3, (0, 255, 0), -1)

        if vals_norm is not None:
            box.set_targets(vals_norm)

        # 跟随状态横幅（画面底部）：现在到底发不发指令，一眼分清
        armed_now = box.get_armed()
        fh, fw = frame.shape[:2]
        if armed_now:
            cv2.rectangle(frame, (8, fh - 54), (235, fh - 16), (0, 80, 0), -1)
            cv2.putText(frame, "FOLLOWING", (18, fh - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        else:
            cv2.rectangle(frame, (8, fh - 54), (345, fh - 16), (0, 0, 80), -1)
            cv2.putText(frame, "NOT FOLLOWING", (18, fh - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 90, 255), 2)

        # HUD
        frames += 1
        now = time.time()
        if now - fps_t >= 1.0:
            fps = frames / (now - fps_t)
            frames = 0
            fps_t = now
        cv2.putText(frame, f"FPS {fps:.0f}  gain {args.gain:.2f}  hand {handlabel or '-'}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y = 50
        for i, n in enumerate(NAMES):
            v = int(round(vals_norm[i] * 1000)) if vals_norm else 0
            cv2.putText(frame, f"{n}: {v:4d}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if vals_norm else (120, 120, 120), 2)
            cv2.rectangle(frame, (110, y - 11), (110 + v // 10, y - 2),
                          (0, 200, 255), -1)
            y += 20
        if pending:
            cv2.putText(frame, f"[采样 {len(pending['samples'])}/{SAMPLE_FRAMES} 保持不动]",
                        (10, y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        elif not vals_norm:
            cv2.putText(frame, "no hand", (10, y + 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 显示帧缩到 960 宽（720p 处理、小图显示，界面不拖累识别循环）；
        # 队列只保留最新一帧，旧的丢弃
        disp = frame
        if frame.shape[1] > 960:
            sc = 960.0 / frame.shape[1]
            disp = cv2.resize(frame, (960, int(frame.shape[0] * sc)),
                              interpolation=cv2.INTER_AREA)
        rgb_small = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        try:
            frame_q.put_nowait(rgb_small)
        except queue.Full:
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
            try:
                frame_q.put_nowait(rgb_small)
            except queue.Full:
                pass

    cap.release()
    landmarker.close()


def hand_worker(box: SafeBox, stop: threading.Event, hand: r2m.Revo2Hand):
    """读最新目标 → 下发角度；读电流/触觉；应用保护电流滑块。"""
    last_norm = [0.0] * 6
    last_send = 0.0
    period = 1.0 / 30.0    # 最快 30 Hz（配合更短的执行时长，跟手更紧）
    last_telemetry = 0.0
    last_touch = 0.0
    last_move = time.time()
    idle_s = 2.5
    idle_noted = False

    while not stop.is_set():
        now = time.time()

        # 保护电流滑块 → 写入
        req = box.pop_protect_request()
        if req is not None:
            try:
                hand.write_registers(r2m.REG_FINGER_CURRENT_PROTECT,
                                     [int(v) for v in req], allow_write=True)
                box.set_protect_echo(req)
                box.set_note("保护电流已更新")
            except Exception as exc:  # noqa: BLE001
                box.set_note(f"保护电流写入失败：{exc}")

        # 读取目标并下发
        norm, _seq = box.get_targets()
        armed = box.get_armed()
        moved = max(abs(norm[i] - last_norm[i]) for i in range(6))
        moving = moved > 0.015
        if moving:
            last_move = now

        if armed and moving and now - last_send >= period:
            deg = [clamp(norm[i]) * ANGLE_LIMITS[i] for i in range(6)]
            try:
                hand.set_angles(deg, duration_ms=80, allow_write=True)
                last_send = now
                last_norm = list(norm)
                idle_noted = False
            except Exception as exc:  # noqa: BLE001
                box.set_note(f"下发失败：{exc}")
        elif armed and not moving and now - last_move > idle_s:
            if not idle_noted:   # 只提示一次，别刷屏
                box.set_note("手静止，已停止下发（电机卸力降温）")
                idle_noted = True

        # 读电流 / 位置 / 状态（力矩展示）
        if now - last_telemetry >= 0.2:
            try:
                cur = hand.read_currents()
                pos = hand.read_positions()
                st = hand.read_motor_status()
                box.set_telemetry(cur, pos, st)
            except Exception as exc:  # noqa: BLE001
                box.set_note(f"读反馈失败：{exc}")
            last_telemetry = now

        # 读触觉（4200–4229 是输入寄存器；法向力单位 100×N）
        if now - last_touch >= 0.5:
            try:
                raw = hand.read_registers(REG_TOUCH_FORCE_CAP,
                                          REG_TOUCH_FORCE_COUNT,
                                          input_reg=True)
                box.set_touch(raw, box.snapshot()["touch_vendor"])
            except Exception:
                pass  # 读不到就保留上一次，不刷屏
            last_touch = now

        time.sleep(0.02)


# ══════════════════════════════════════════════════════════════
# GUI
# ══════════════════════════════════════════════════════════════
class App:
    def __init__(self, root: tk.Tk, args):
        self.root = root
        self.args = args
        self.box = SafeBox()
        self.stop = threading.Event()
        self.frame_q = queue.Queue(maxsize=1)
        self.hand = None
        self.cam_thread = None
        self.hand_thread = None
        self.photo = None

        root.title("Revo2 触觉版灵巧手 · 摄像头手势模仿")
        root.geometry("1120x840")   # 默认开大一点：右侧四段面板全高约 780px
        root.minsize(960, 600)      # 再矮也允许滚动，按钮永远够得着
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._poll()

        # 启动摄像头识别线程（不连手也能看识别效果）
        self.cam_thread = threading.Thread(
            target=camera_worker, args=(self.box, self.stop, self.frame_q, args),
            daemon=True)
        self.cam_thread.start()

    # ── 界面构建 ──────────────────────────────────────
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        # 左：视频
        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)
        if Image is None:
            self.video_label = ttk.Label(left, text="缺少 Pillow，无法在窗口内显示视频。\n"
                                                    "pip install pillow 后重试。")
        else:
            self.video_label = ttk.Label(left)
        self.video_label.pack(padx=2, pady=2)

        # 右：控制面板。整块内容比窗口高，包一层可滚动容器，
        # 保证「开始跟随」按钮在矮窗口下也操作得到
        #（之前窗口一矮，按钮被底边裁掉，手就一直不动——本次问题的根源）。
        outer = ttk.Frame(main, width=400)
        outer.pack(side="right", fill="y", padx=(8, 0))
        outer.pack_propagate(False)
        canvas = tk.Canvas(outer, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(canvas, padding=(2, 2))
        _win = canvas.create_window((0, 0), window=right, anchor="nw")
        right.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(_win, width=e.width))
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))

        # 顶部常驻状态条：跟随/未跟随一眼分清，不依赖底部状态栏
        self.follow_state = tk.StringVar(value="● 未跟随 —— 连接后点「开始跟随」")
        self.follow_label = ttk.Label(
            right, textvariable=self.follow_state,
            foreground="#c00000", font=("Microsoft YaHei", 11, "bold"))
        self.follow_label.pack(fill="x", pady=(0, 6))

        # 连接区
        conn = ttk.LabelFrame(right, text="① 连接（RS485/Modbus）", padding=6)
        conn.pack(fill="x", pady=(0, 6))
        row = ttk.Frame(conn); row.pack(fill="x", pady=2)
        ttk.Label(row, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, width=10)
        self.port_combo["values"] = self._list_ports()
        self.port_combo.pack(side="left", padx=4)
        if self.port_combo["values"]:
            self.port_combo.current(0)
        ttk.Label(row, text="ID").pack(side="left", padx=(8, 0))
        self.id_var = tk.StringVar(value=str(self.args.id))
        ttk.Entry(row, textvariable=self.id_var, width=5).pack(side="left", padx=4)

        row2 = ttk.Frame(conn); row2.pack(fill="x", pady=2)
        ttk.Label(row2, text="波特率").pack(side="left")
        self.baud_var = tk.StringVar(value="460800")
        ttk.Entry(row2, textvariable=self.baud_var, width=8).pack(side="left", padx=4)
        self.conn_btn = ttk.Button(row2, text="连接", command=self._connect)
        self.conn_btn.pack(side="left", padx=(8, 0))
        self.disc_btn = ttk.Button(row2, text="断开", command=self._disconnect,
                                   state="disabled")
        self.disc_btn.pack(side="left", padx=4)

        self.info_label = ttk.Label(conn, text="未连接", foreground="#666")
        self.info_label.pack(fill="x", pady=(4, 0))

        # 力矩显示区
        torq = ttk.LabelFrame(right, text="② 力矩（电流 mA ≈ 力矩，读 2012–2017）", padding=6)
        torq.pack(fill="x", pady=6)
        self.cur_vars = []
        self.cur_pbs = []
        for i, n in enumerate(NAMES):
            rr = ttk.Frame(torq); rr.pack(fill="x", pady=1)
            ttk.Label(rr, text=n, width=8, anchor="w").pack(side="left")
            v = tk.StringVar(value="--")
            self.cur_vars.append(v)
            ttk.Label(rr, textvariable=v, width=8, anchor="e",
                      font=("Consolas", 10, "bold")).pack(side="left")
            pb = ttk.Progressbar(rr, maximum=1500, length=140)
            pb.pack(side="left", padx=6)
            self.cur_pbs.append(pb)

        self.touch_label = ttk.Label(
            torq, text="触觉法向力（4200–4229，单位 N）：--", foreground="#888")
        self.touch_label.pack(fill="x", pady=(4, 0))

        # 力矩调节区
        adj = ttk.LabelFrame(right, text="③ 保护电流滑块（930–935，100–1500 mA，松手生效）",
                             padding=6)
        adj.pack(fill="x", pady=6)
        self.protect_vars = []
        for i, n in enumerate(NAMES):
            rr = ttk.Frame(adj); rr.pack(fill="x", pady=1)
            ttk.Label(rr, text=n, width=8, anchor="w").pack(side="left")
            v = tk.IntVar(value=500)
            self.protect_vars.append(v)
            sc = ttk.Scale(rr, from_=100, to=1500, variable=v, length=170)
            sc.pack(side="left", padx=4)
            sc.bind("<ButtonRelease-1>", lambda e, i=i: self._on_protect_change(i))
            val = ttk.Label(rr, textvariable=v, width=5, anchor="e")
            val.pack(side="left")

        # 动作区
        act = ttk.Frame(right); act.pack(fill="x", pady=6)
        self.follow_btn = ttk.Button(act, text="开始跟随", command=self._toggle_follow)
        self.follow_btn.pack(side="left")
        self.estop_btn = ttk.Button(act, text="急停", command=self._estop)
        self.estop_btn.pack(side="left", padx=4)
        self.open_btn = ttk.Button(act, text="全部张开", command=self._open_all)
        self.open_btn.pack(side="left")

        # 标定区
        calib = ttk.LabelFrame(right, text="④ 量程标定（可选，各人手型不同）", padding=6)
        calib.pack(fill="x", pady=6)
        ttk.Button(calib, text="采「张开」",
                   command=lambda: self.box.set_calib_request("open")).pack(side="left", padx=2)
        ttk.Button(calib, text="采「闭合」",
                   command=lambda: self.box.set_calib_request("close")).pack(side="left", padx=2)
        ttk.Button(calib, text="清除标定",
                   command=lambda: self.box.set_calib_request("clear")).pack(side="left", padx=2)
        self.calib_label = ttk.Label(calib, text="未标定，用默认量程", foreground="#666")
        self.calib_label.pack(fill="x", pady=(4, 0))

        self.status = ttk.Label(right, text="就绪", foreground="#333",
                                wraplength=360, justify="left")
        self.status.pack(fill="x", pady=(4, 0))

    @staticmethod
    def _list_ports():
        try:
            from serial.tools import list_ports
            return [p.device for p in list_ports.comports()]
        except Exception:
            return ["COM22"]  # 兜底，用户可手改

    # ── 连接 ─────────────────────────────────────────
    def _connect(self):
        if self.args.dry_run:
            self.info_label.config(text="--dry-run：不连手，仅识别")
            return
        port = self.port_var.get().strip()
        try:
            dev = int(self.id_var.get())
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "ID 和波特率要是整数")
            return
        try:
            self.hand = r2m.Revo2Hand(port=port, device_id=dev, baud=baud).open()
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("连接失败",
                                 f"打不开 {port}（{exc}）。\n"
                                 "先确认强脑「上位机」软件已关闭，串口没被占用。")
            return
        try:
            info = self.hand.read_info()
            # 单位口径检查：必须是「物理单位（度）」。
            # 若是归一化模式（0-1000），下发的度数会被当百分比解释，幅度全错。
            unit_note = ""
            try:
                if self.hand.read_unit_mode() != 1:
                    ok = self.hand.ensure_physical_unit_mode(allow_write=True)
                    unit_note = " | 单位:物理(度)" if ok else " | 单位异常!"
                    self.box.set_note(
                        "手原为归一化单位模式，已自动切换为物理单位（度）"
                        if ok else "单位模式切换失败，跟随幅度可能不准")
                else:
                    unit_note = " | 单位:物理(度)"
            except Exception:
                pass
            txt = (f"{info.side} | 固件 {info.firmware or '未知'}"
                   f" | 触觉 {info.touch_vendor}{unit_note}")
            self.box.set_info(txt)
            self.info_label.config(text=txt, foreground="#000")
            self.box.set_touch([], info.touch_vendor)
            # 读当前保护电流回填滑块
            try:
                cur_prot = self.hand.read_registers(
                    r2m.REG_FINGER_CURRENT_PROTECT, 6, input_reg=False)
                if len(cur_prot) == 6:
                    for i in range(6):
                        self.protect_vars[i].set(int(cur_prot[i]))
                    self.box.set_protect_echo(list(cur_prot))
            except Exception:
                pass
        except Exception as exc:  # noqa: BLE001
            self.info_label.config(text=f"已打开串口但读信息失败：{exc}",
                                   foreground="#a00")
        self.conn_btn.config(state="disabled")
        self.disc_btn.config(state="normal")
        self.hand_thread = threading.Thread(
            target=hand_worker, args=(self.box, self.stop, self.hand),
            daemon=True)
        self.hand_thread.start()
        self.status.config(text="已连接。点「开始跟随」后才会下发动作。")

    def _disconnect(self):
        self.box.set_armed(False)
        if self.hand:
            try:
                self.hand.close()
            except Exception:
                pass
            self.hand = None
        self.conn_btn.config(state="normal")
        self.disc_btn.config(state="disabled")
        self.follow_state.set("● 未跟随 —— 连接后点「开始跟随」")
        self.follow_label.config(foreground="#c00000")
        self.info_label.config(text="已断开", foreground="#666")
        self.status.config(text="已断开")

    # ── 动作 ─────────────────────────────────────────
    def _toggle_follow(self):
        if not self.hand and not self.args.dry_run:
            messagebox.showwarning("未连接", "先点「连接」")
            return
        armed = self.box.get_armed()
        self.box.set_armed(not armed)
        self.follow_btn.config(text="暂停跟随" if not armed else "开始跟随")
        if not armed:
            self.follow_state.set("● 跟随中（手静止 2.5 秒自动停发卸力）")
            self.follow_label.config(foreground="#007000")
        else:
            self.follow_state.set("○ 未跟随 —— 已暂停，点「开始跟随」恢复")
            self.follow_label.config(foreground="#c00000")
        self.status.config(text="跟随中（手静止会自动停发卸力）" if not armed
                           else "已暂停，手停在当前位")

    def _estop(self):
        self.box.set_armed(False)
        self.follow_btn.config(text="开始跟随")
        self.follow_state.set("○ 已急停 —— 点「开始跟随」恢复")
        self.follow_label.config(foreground="#c00000")
        self.status.config(text="已急停：停止下发，手停在当前位")

    def _open_all(self):
        if not self.hand:
            if not self.args.dry_run:
                messagebox.showwarning("未连接", "先点「连接」")
            return
        try:
            self.hand.set_angles([0.0] * 6, duration_ms=600, allow_write=True)
            self.status.config(text="已下发「全部张开」")
        except Exception as exc:  # noqa: BLE001
            self.status.config(text=f"张开失败：{exc}")

    def _on_protect_change(self, idx):
        if not self.hand:
            return
        vals = [int(v.get()) for v in self.protect_vars]
        self.box.push_protect_request(vals)

    # ── 轮询刷新界面 ─────────────────────────────────
    def _poll(self):
        # 视频帧
        try:
            rgb = self.frame_q.get_nowait()
            if Image is not None:
                self.photo = ImageTk.PhotoImage(image=Image.fromarray(rgb))
                self.video_label.config(image=self.photo)
        except queue.Empty:
            pass

        s = self.box.snapshot()

        # 力矩（电流）显示
        if s["currents"] and len(s["currents"]) == 6:
            for i, v in enumerate(s["currents"]):
                self.cur_vars[i].set(f"{int(v)}mA")
                self.cur_pbs[i]["value"] = max(0, min(1500, int(v)))
        # 触觉：法向力 = 寄存器值/100，单位 N（0–25 N，官方语义）
        if s["touch_raw"]:
            raw = s["touch_raw"]
            if len(raw) >= 15:
                forces = [raw[3 * i] / 100.0 for i in range(5)]
                txt = ("触觉法向力(N)： " + "  ".join(
                    f"{n}{v:.2f}" for n, v in
                    zip(("拇", "食", "中", "无", "小"), forces)))
            else:
                txt = f"触觉原始值：{raw[:12]}"
            self.touch_label.config(text=txt, foreground="#555")
        self.calib_label.config(text=s["calib_state"])

        if s["note"]:
            self.status.config(text=s["note"])

        self.root.after(30, self._poll)

    def _on_close(self):
        self.stop.set()
        self.box.set_armed(False)
        for t in (self.cam_thread, self.hand_thread):
            if t and t.is_alive():
                t.join(timeout=1.5)
        if self.hand:
            try:
                self.hand.close()
            except Exception:
                pass
        self.root.destroy()


# ══════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Revo2 触觉手 · 摄像头手势模仿 GUI")
    ap.add_argument("--id", type=int, default=127, help="设备 ID：右手 127，左手 126")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--model", default="", help="hand_landmarker.task 路径")
    ap.add_argument("--hand", choices=["left", "right", "auto"], default="right")
    ap.add_argument("--gain", type=float, default=1.0, help="灵敏度")
    ap.add_argument("--dry-run", action="store_true", help="只识别不连手")
    args = ap.parse_args()

    if not args.model:
        args.model = next((m for m in MODEL_CANDIDATES if os.path.exists(m)), "")
    if not args.model:
        print("找不到 hand_landmarker.task。请用 --model 指定路径。")
        print("下载：https://storage.googleapis.com/mediapipe-models/"
              "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
        return 2

    root = tk.Tk()
    App(root, args)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

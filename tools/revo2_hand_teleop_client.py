#!/usr/bin/env python3
"""人手 → 灵巧手 实时模仿（本机 Windows 端）v2

摄像头拍到手 → MediaPipe 出 21 个关键点 → 算出 6 个电机值 → TCP 发给机器人

用法：
    python hand_teleop_client.py                 # 默认连 172.16.10.47:9910
    python hand_teleop_client.py --cam 1         # 换摄像头
    python hand_teleop_client.py --dry-run       # 只显示不发送（先调参用）

操作（窗口按键盘）：
    c      —— 采「张开」量程：手掌张开、拇指往外伸到最开，按一下 c，保持 1 秒
    v      —— 采「闭合」量程：握拳、拇指压向手掌，按一下 v，保持 1 秒
             两步都做完会自动存盘（teleop_calib.json），下次启动自动加载
    x      —— 清除标定，退回默认量程
    +/-    —— 整体灵敏度
    q      —— 退出

拇指 v2 说明（与旧版的区别）：
    旧版用「拇指尖到食指根的距离」当侧摆，方向是反的（拇指往里靠机器人往外张），
    且弯曲和侧摆混在一起。
    v2 侧摆改用「拇指掌骨方向绕掌法向的带符号夹角」：
      - 拇指往手掌里靠 → 角度变大 → 机器人拇指往里收（方向正确）
      - 手掌正对 / 手背正对摄像头，结果一致
      - 与拇指弯曲（IP 关节）解耦：只弯不摆时侧摆值不动
    弯曲改为「链长比值 + IP 关节角」各占一半，拇指指向摄像头（透视缩短）时更稳。
    标定（c/v）把每个人的手测量程记下来，替代拍脑袋的经验区间。

⚠️ 帧率与延迟：摄像头 30fps，识别后以约 20Hz 下发；
   灵巧手跟随会有 0.1~0.3 s 延迟，属正常。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import statistics
import sys
import time

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

# ── MediaPipe 21 个关键点索引 ──────────────────────────────
W = 0                                    # 腕
T_CMC, T_MCP, T_IP, T_TIP = 1, 2, 3, 4   # 拇指
I_MCP, I_PIP, I_DIP, I_TIP = 5, 6, 7, 8  # 食指
M_MCP, M_PIP, M_DIP, M_TIP = 9, 10, 11, 12
R_MCP, R_PIP, R_DIP, R_TIP = 13, 14, 15, 16
P_MCP, P_PIP, P_DIP, P_TIP = 17, 18, 19, 20

NAMES = ["拇指屈", "拇指掌", "食指", "中指", "无名指", "小指"]

# 每个通道的「原生值」量程默认值 (张开, 闭合)。
# 原生值：拇指屈 / 四指 = 0~1 弯曲度；拇指掌 = 带符号角（度），往手掌里靠为正。
# 做过 c/v 标定后整表替换。
DEFAULT_RANGE = [
    (0.05, 0.88),     # 拇指屈
    (-55.0, 15.0),    # 拇指掌（度）：张开约 -55 以下，贴掌约 +15 以上
    (0.08, 0.95),     # 食指
    (0.08, 0.95),
    (0.08, 0.95),
    (0.08, 0.95),
]

CALIB_FILE = "teleop_calib.json"      # 存在脚本同目录
SAMPLE_FRAMES = 30                    # 标定采样帧数（约 1 秒）
SAMPLE_TIMEOUT = 6.0                  # 采样最长时间，超时取消


def d3(a, b) -> float:
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def clamp(v, lo=0.0, hi=1.0) -> float:
    return max(lo, min(hi, v))


def _sub(a, b):
    return (a.x - b.x, a.y - b.y, a.z - b.z)


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def angle_at(a, b, c) -> float:
    """以 b 为顶点的夹角（度），用三维坐标算。"""
    v1, v2 = _sub(a, b), _sub(c, b)
    l1 = math.sqrt(_dot(v1, v1))
    l2 = math.sqrt(_dot(v2, v2))
    if l1 < 1e-9 or l2 < 1e-9:
        return 180.0
    cosv = max(-1.0, min(1.0, _dot(v1, v2) / (l1 * l2)))
    return math.degrees(math.acos(cosv))


def finger_curl(L, mcp: int, pip: int, dip: int, tip: int) -> float:
    """单指弯曲度：指尖到掌指关节的直线距离 / 该指骨骼总长。

    用比值而非绝对距离，这样手离摄像头远近不影响结果。
    伸直时比值≈0.95，完全弯曲时≈0.40。
    拇指传 (T_MCP, T_IP, T_TIP, T_TIP)：拇指没有 DIP，末段长度记 0。
    """
    bone = d3(L[mcp], L[pip]) + d3(L[pip], L[dip]) + d3(L[dip], L[tip])
    if bone < 1e-9:
        return 0.0
    r = d3(L[mcp], L[tip]) / bone
    return clamp((1.0 - r) / 0.55)


def thumb_aux_deg(L) -> float:
    """拇指侧摆（v2）：拇指掌骨方向绕「掌法向」的带符号夹角（度）。

    掌轴 u = 腕→食指根，小指轴 v = 腕→小指根，法向 n = u×v。
    拇指掌骨方向 t（CMC→MCP）投影到掌平面后，与 u 的夹角绕 n 带符号测量：

    - 拇指往外张开 → 约 -90°~-55°（负得越多越开）
    - 拇指往手掌里靠 → 趋近 0° 并转正（+10°~+30°）
    - 手掌正对或手背正对摄像头，符号一致（n 与拇指同时翻转，互相抵消）
    - 只弯拇指 IP 关节不动掌骨方向 → 本值不变，与「拇指屈」解耦
    """
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
    # 去掉离面分量，只留掌平面内的摆动
    dn = _dot(t, n)
    t = (t[0] - n[0] * dn, t[1] - n[1] * dn, t[2] - n[2] * dn)
    mt = math.sqrt(_dot(t, t))
    if mt < 1e-9:
        return -70.0
    t = (t[0] / mt, t[1] / mt, t[2] / mt)
    return math.degrees(math.atan2(_dot(_cross(u, t), n), _dot(u, t)))


def thumb_flex(L) -> float:
    """拇指弯曲度（v2）：链长比值 + IP 关节角，各占一半。

    链长比值（MCP→IP→TIP）在拇指指向摄像头时会因透视缩短而虚高，
    IP 关节角用三维坐标算、不受缩短影响，互补。
    """
    chain = finger_curl(L, T_MCP, T_IP, T_TIP, T_TIP)
    ip = clamp((168.0 - angle_at(L[T_MCP], L[T_IP], L[T_TIP])) / 60.0)
    return clamp(0.5 * chain + 0.5 * ip)


def native_values(L) -> list:
    """6 个通道的「原生值」：拇指屈 0~1，拇指掌为角度值，四指 0~1。"""
    return [
        thumb_flex(L),
        thumb_aux_deg(L),
        finger_curl(L, I_MCP, I_PIP, I_DIP, I_TIP),
        finger_curl(L, M_MCP, M_PIP, M_DIP, M_TIP),
        finger_curl(L, R_MCP, R_PIP, R_DIP, R_TIP),
        finger_curl(L, P_MCP, P_PIP, P_DIP, P_TIP),
    ]


def to_norm(native: float, open_v: float, closed_v: float) -> float:
    """原生值 → 0~1。open→0（张开），closed→1（闭合）。两端点谁大谁小都可以。"""
    if abs(closed_v - open_v) < 1e-6:
        return 0.0
    return clamp((native - open_v) / (closed_v - open_v))


def calib_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), CALIB_FILE)


def load_calib() -> dict | None:
    try:
        with open(calib_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data.get("open"), list) and isinstance(data.get("closed"), list):
            if len(data["open"]) == 6 and len(data["closed"]) == 6:
                return data
    except Exception:
        pass
    return None


def save_calib(data: dict) -> None:
    try:
        with open(calib_path(), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
    except Exception as exc:  # noqa: BLE001
        print("标定存盘失败：", exc)


def median6(samples: list) -> list:
    cols = list(zip(*samples))
    return [statistics.median(c) for c in cols]


def main() -> int:
    ap = argparse.ArgumentParser(description="摄像头人手 → 灵巧手实时模仿")
    ap.add_argument("--host", default="172.16.10.47", help="机器人 IP")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--cam", type=int, default=0, help="摄像头编号，默认 0")
    ap.add_argument("--rate", type=float, default=20.0, help="下发频率 Hz")
    ap.add_argument("--model", default=r"D:\sim\hand_landmarker.task")
    ap.add_argument("--gain", type=float, default=1.0, help="灵敏度倍率")
    ap.add_argument("--hand", choices=["left", "right", "auto"], default="right",
                    help="跟随操作者的哪只手，默认 right。auto = 跟先检测到的那只")
    ap.add_argument("--idle", type=float, default=2.5,
                    help="手静止超过这么多秒就停止下发，交回底层控制以省电降温，默认 2.5")
    ap.add_argument("--dry-run", action="store_true", help="只显示不发送")
    args = ap.parse_args()

    calib = load_calib()
    ranges = [list(r) for r in DEFAULT_RANGE]
    if calib:
        ranges = [[o, c] for o, c in zip(calib["open"], calib["closed"])]
    print("标定：" + ("已加载 %s（按 x 清除）" % CALIB_FILE if calib
                     else "未标定，用默认量程。建议先按 c 再按 v 做一次（窗口内按键）"))

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=args.model),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    landmarker = vision.HandLandmarker.create_from_options(opts)

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"打不开摄像头 {args.cam}")
        return 1

    sock = None
    if not args.dry_run:
        try:
            sock = socket.create_connection((args.host, args.port), timeout=2)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"✓ 已连接 {args.host}:{args.port}")
        except Exception as exc:  # noqa: BLE001
            print(f"连不上机器人（{exc}）。可先 --dry-run 看识别效果")
            return 1

    print("\n窗口内按键：c 采张开 → v 采闭合 → 自动存盘 | x 清除标定 | +/- 灵敏度 | q 退出\n")

    smooth = [0.0] * 6
    gain = args.gain
    t0 = int(time.time() * 1000)
    last_send = 0.0
    period = 1.0 / args.rate
    fps_t = time.time()
    frames = 0
    fps = 0.0
    last_vals = [0] * 6
    last_move = time.time()

    pending = None      # 标定采样状态：{"label","samples","t0"}
    aux_show = 0.0
    ip_show = 180.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)              # 镜像，像照镜子
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ts = t0 + int((time.time() * 1000) - t0)
        res = landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

        vals = None
        handlabel = ""
        skip = False
        if res.hand_landmarks:
            # 画面做过镜像，因此 MediaPipe 报的左右与操作者实际相反
            if res.handedness:
                mp_side = res.handedness[0][0].category_name          # "Left" / "Right"
                user_side = "right" if mp_side == "Left" else "left"
                handlabel = f"{user_side}(MP={mp_side})"
                if args.hand != "auto" and user_side != args.hand:
                    skip = True          # 不是要跟的那只手，本帧跳过
            if not skip:
                L = res.hand_landmarks[0]
                nat = native_values(L)
                aux_show, ip_show = nat[1], angle_at(L[T_MCP], L[T_IP], L[T_TIP])

                # 标定采样
                if pending:
                    pending["samples"].append(nat)
                    if len(pending["samples"]) >= SAMPLE_FRAMES:
                        med = median6(pending["samples"])
                        calib = calib or {}
                        calib[pending["label"]] = med
                        if "open" in calib and "closed" in calib:
                            ranges = [[o, c] for o, c in zip(calib["open"], calib["closed"])]
                            save_calib(calib)
                            print(f"✓ 标定完成并存盘：{CALIB_FILE}")
                            print("   张开:", [round(v, 2) for v in calib['open']])
                            print("   闭合:", [round(v, 2) for v in calib['closed']])
                        else:
                            half = "张开" if pending["label"] == "open" else "闭合"
                            print(f"✓ 已记录{half}，再采另一半即完成标定")
                        pending = None
                    elif time.time() - pending["t0"] > SAMPLE_TIMEOUT:
                        print("采样超时（手要一直保持在画面里），已取消，重新按键再采")
                        pending = None

                norm = [to_norm(nat[i], ranges[i][0], ranges[i][1]) for i in range(6)]
                norm = [clamp(v * gain) for v in norm]
                for i in range(6):
                    smooth[i] = smooth[i] + 0.45 * (norm[i] - smooth[i])
                vals = [int(round(v * 1000)) for v in smooth]

                h, w = frame.shape[:2]
                for p in L:
                    cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3, (0, 255, 0), -1)

        # 发送（含静止降频：手不动就停止下发，让电机卸力降温）
        now = time.time()
        moving = False
        if vals:
            moved = max(abs(vals[i] - last_vals[i]) for i in range(6))
            if moved > 15:            # 变化超过 1.5% 视为在动（阈值从 2.5% 降下来，小动作也跟）
                moving = True
                last_move = now
        if sock:
            # 非阻塞读回执，服务端报错（如"下发失败"）能立刻看到，不再黑箱
            try:
                sock.setblocking(False)
                err = sock.recv(300)
                if err:
                    print("  [服务端]", err.decode(errors="ignore").strip())
                sock.setblocking(True)
            except Exception:
                try:
                    sock.setblocking(True)
                except Exception:
                    pass

            if vals and moving and now - last_send >= period:
                try:
                    sock.sendall((",".join(str(v) for v in vals) + "\n").encode())
                    last_send = now
                    last_vals = list(vals)
                except Exception as exc:  # noqa: BLE001
                    print("发送失败：", exc)
                    break
            elif vals and not moving and now - last_move > args.idle:
                pass                  # 静止超时：不下发，服务端会在 1.5s 后交回底层控制

        # HUD
        frames += 1
        if now - fps_t >= 1.0:
            fps = frames / (now - fps_t)
            frames = 0
            fps_t = now
        calib_txt = "已标定" if (calib and "open" in calib and "closed" in calib) else "默认量程"
        cv2.putText(frame, f"FPS {fps:.0f}  gain {gain:.2f}  hand {handlabel or '-'}  [{calib_txt}]",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y = 50
        for i, n in enumerate(NAMES):
            v = vals[i] if vals else 0
            cv2.putText(frame, f"{n}: {v:4d}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if vals else (120, 120, 120), 2)
            cv2.rectangle(frame, (110, y - 11), (110 + int(v / 10), y - 2),
                          (0, 200, 255), -1)
            y += 20
        # 拇指调试行：原始侧摆角 / IP 角，标定与排障用
        cv2.putText(frame, f"aux {aux_show:6.1f}deg  IP {ip_show:5.1f}deg", (10, y + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        if pending:
            cv2.putText(frame,
                        f"['{'张开' if pending['label'] == 'open' else '闭合'}' 采样中 "
                        f"{len(pending['samples'])}/{SAMPLE_FRAMES} 保持不动]",
                        (10, y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        elif not vals:
            cv2.putText(frame, "no hand", (10, y + 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("hand teleop (q quit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("+") or key == ord("="):
            gain = min(2.0, gain + 0.1)
        elif key == ord("-"):
            gain = max(0.3, gain - 0.1)
        elif key in (ord("c"), ord("v")) and not pending:
            pending = {"label": "open" if key == ord("c") else "closed",
                       "samples": [], "t0": time.time()}
            side = "张开（拇指往外伸到最开）" if key == ord("c") else "闭合（握拳，拇指压向手掌）"
            print(f"开始采样「{side}」，保持 1 秒…")
        elif key == ord("x") and calib:
            calib = None
            ranges = [list(r) for r in DEFAULT_RANGE]
            try:
                os.remove(calib_path())
                print("已清除标定，退回默认量程")
            except FileNotFoundError:
                pass

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if sock:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

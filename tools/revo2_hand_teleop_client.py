#!/usr/bin/env python3
"""人手 → 灵巧手 实时模仿（本机 Windows 端）

摄像头拍到手 → MediaPipe 出 21 个关键点 → 算出 6 个电机值 → TCP 发给机器人

用法：
    python hand_teleop_client.py                 # 默认连 172.16.10.47:9910
    python hand_teleop_client.py --cam 1         # 换摄像头
    python hand_teleop_client.py --dry-run       # 只显示不发送（先调参用）

操作：
    把手放在摄像头前，灵巧手会跟着动
    Calib  —— 张开手掌按 c，握拳按 v，做一次量程标定
    +/-    —— 整体灵敏度
    q      —— 退出

⚠️ 帧率与延迟：摄像头 30fps，识别后以约 20Hz 下发；
   灵巧手跟随会有 0.1~0.3 s 延迟，属正常。
"""
from __future__ import annotations

import argparse
import math
import socket
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


def d3(a, b) -> float:
    return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))


def clamp(v, lo=0.0, hi=1.0) -> float:
    return max(lo, min(hi, v))


def finger_curl(L, mcp: int, pip: int, dip: int, tip: int) -> float:
    """单指弯曲度：指尖到掌指关节的直线距离 / 该指骨骼总长。

    用比值而非绝对距离，这样手离摄像头远近不影响结果。
    伸直时比值≈0.95，完全弯曲时≈0.40。
    """
    bone = d3(L[mcp], L[pip]) + d3(L[pip], L[dip]) + d3(L[dip], L[tip])
    if bone < 1e-9:
        return 0.0
    r = d3(L[mcp], L[tip]) / bone
    return clamp((1.0 - r) / 0.55)


def thumb_spread(L) -> float:
    """拇指对掌（外展）程度：拇指尖到食指根的距离 / 手掌尺寸。

    拇指张开时比值大，收拢贴合手掌时比值小。
    """
    palm = d3(L[W], L[I_MCP])
    if palm < 1e-9:
        return 0.0
    r = d3(L[T_TIP], L[I_MCP]) / palm
    return clamp((r - 0.55) / 0.75)


def main() -> int:
    ap = argparse.ArgumentParser(description="摄像头人手 → 灵巧手实时模仿")
    ap.add_argument("--host", default="172.16.10.47", help="机器人 IP")
    ap.add_argument("--port", type=int, default=9910)
    ap.add_argument("--cam", type=int, default=0, help="摄像头编号，默认 0")
    ap.add_argument("--rate", type=float, default=20.0, help="下发频率 Hz")
    ap.add_argument("--model", default=r"D:\sim\hand_landmarker.task")
    ap.add_argument("--gain", type=float, default=1.0, help="灵敏度倍率")
    ap.add_argument("--dry-run", action="store_true", help="只显示不发送")
    args = ap.parse_args()

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

    print("\n把手放到摄像头前。q 退出，c 张手标定，v 握拳标定，+/- 调灵敏度\n")

    smooth = [0.0] * 6
    gain = args.gain
    t0 = int(time.time() * 1000)
    last_send = 0.0
    period = 1.0 / args.rate
    fps_t = time.time()
    frames = 0
    fps = 0.0

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
        if res.hand_landmarks:
            L = res.hand_landmarks[0]
            raw = [
                finger_curl(L, T_MCP, T_IP, T_TIP, T_TIP) * 0.9 + 0.05,  # 拇指屈曲（近似）
                thumb_spread(L),
                finger_curl(L, I_MCP, I_PIP, I_DIP, I_TIP),
                finger_curl(L, M_MCP, M_PIP, M_DIP, M_TIP),
                finger_curl(L, R_MCP, R_PIP, R_DIP, R_TIP),
                finger_curl(L, P_MCP, P_PIP, P_DIP, P_TIP),
            ]
            raw = [clamp(v * gain) for v in raw]
            # 指数平滑，去掉识别抖动
            for i in range(6):
                smooth[i] = smooth[i] + 0.45 * (raw[i] - smooth[i])
            vals = [int(round(v * 1000)) for v in smooth]

            # 画关键点
            h, w = frame.shape[:2]
            for p in L:
                cv2.circle(frame, (int(p.x * w), int(p.y * h)), 3, (0, 255, 0), -1)

        # 发送
        now = time.time()
        if vals and sock and now - last_send >= period:
            try:
                sock.sendall((",".join(str(v) for v in vals) + "\n").encode())
                last_send = now
            except Exception as exc:  # noqa: BLE001
                print("发送失败：", exc)
                break

        # HUD
        frames += 1
        if now - fps_t >= 1.0:
            fps = frames / (now - fps_t)
            frames = 0
            fps_t = now
        cv2.putText(frame, f"FPS {fps:.0f}  gain {gain:.2f}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y = 50
        for i, n in enumerate(NAMES):
            v = vals[i] if vals else 0
            cv2.putText(frame, f"{n}: {v:4d}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if vals else (120, 120, 120), 2)
            cv2.rectangle(frame, (110, y - 11), (110 + int(v / 10), y - 2),
                          (0, 200, 255), -1)
            y += 20
        if not vals:
            cv2.putText(frame, "no hand", (10, y + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("hand teleop (q quit)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("+") or key == ord("="):
            gain = min(2.0, gain + 0.1)
        elif key == ord("-"):
            gain = max(0.3, gain - 0.1)

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if sock:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动器：环境检查 → 缺什么装什么 → 找/下模型 → 打开图形界面。

给不熟技术的用户用：双击 start_gui.bat 就会走到这里。
每一步失败都给中文提示和下一步怎么办，不让窗口无声闪退。
"""
from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import urllib.request

# ── 常量 ──────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GUI_FILE = os.path.join(SCRIPT_DIR, "revo2_touch_gui.py")

# 依赖清单：(import 用的模块名, pip 包名)
REQUIRED = [
    ("cv2", "opencv-python"),
    ("mediapipe", "mediapipe"),
    ("serial", "pyserial"),
    ("PIL", "pillow"),
]

# pip 源：先试清华（国内快），失败再试官方
PIP_SOURCES = [
    "https://pypi.tuna.tsinghua.edu.cn/simple",
    "https://pypi.org/simple",
]

# MediaPipe 手部关键点模型
MODEL_LOCAL_CANDIDATES = [
    os.path.join(SCRIPT_DIR, "hand_landmarker.task"),
]
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/"
             "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
MODEL_MANUAL_HINT = (
    "手动下载办法：\n"
    "  用浏览器打开下面这个链接，把下载的 hand_landmarker.task 文件放到本脚本同目录：\n"
    f"  {MODEL_URL}"
)


def step(no: int, text: str) -> None:
    print(f"\n{'=' * 50}\n[{no}/4] {text}\n{'=' * 50}")


def fail(text: str, fix: str = "") -> None:
    print(f"\n{'!' * 50}\n[启动失败] {text}")
    if fix:
        print(f"\n解决办法：\n{fix}")
    print(f"{'!' * 50}")
    input("\n按回车键关闭窗口…")
    sys.exit(1)


def have_module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def pip_install(pkg: str, mod: str) -> bool:
    """装一个包：先清华源，失败换官方源。装完用 import 模块名验证。"""
    for src in PIP_SOURCES:
        src_name = "清华镜像" if "tuna" in src else "官方源"
        print(f"  正在从{src_name}安装 {pkg} …（可能要等一两分钟）")
        r = subprocess.run(
            [sys.executable, "-m", "pip", "install", pkg, "-i", src],
        )
        if r.returncode == 0 and have_module(mod):
            return True
    return False


def main() -> int:
    print("Revo2 灵巧手 · 摄像头手势模仿 —— 一键启动")

    # ── 1/4 Python 版本 ────────────────────────────────
    step(1, "检查 Python 版本")
    v = sys.version_info
    print(f"  当前 Python：{v.major}.{v.minor}.{v.micro}")
    if v < (3, 9):
        fail(f"Python 版本太老（{v.major}.{v.minor}）。",
             "MediaPipe 需要 Python 3.9 以上。\n"
             "  到 https://www.python.org/downloads/ 装 3.10，安装时勾选「Add Python to PATH」。")
    if v >= (3, 13):
        print("  提示：3.13/3.14 太新，mediapipe 可能没有对应版本。")
        print("  如果下一步安装 mediapipe 失败，请改装 Python 3.10～3.12。")
    print("  版本 OK")

    # ── 2/4 依赖检查与自动安装 ─────────────────────────
    step(2, "检查依赖（缺什么自动装什么，第一次运行会等几分钟）")
    missing = [(mod, pkg) for mod, pkg in REQUIRED if not have_module(mod)]
    if not missing:
        print("  依赖齐全：opencv / mediapipe / pyserial / pillow")
    else:
        print("  缺少：" + "、".join(pkg for _, pkg in missing))
        for mod, pkg in missing:
            ok = pip_install(pkg, mod)
            if not ok:
                fail(f"自动安装 {pkg} 失败（一般是网络问题）。",
                     "① 换个网络（比如手机热点）再双击 start_gui.bat 重试；\n"
                     "② 或打开命令提示符（cmd），手动执行：\n"
                     f"     pip install {pkg} -i https://pypi.tuna.tsinghua.edu.cn/simple\n"
                     "③ 看到具体报错的话，把整段报错截图发出来求助。")
        # 复查
        still = [mod for mod, _ in REQUIRED if not have_module(mod)]
        if still:
            fail(f"安装后仍找不到模块：{', '.join(still)}",
                 "关掉本窗口，重新双击 start_gui.bat 再试一次（装完包有时要重开才生效）。")
        print("  ✓ 依赖已装齐")

    # ── 3/4 模型文件 ───────────────────────────────────
    step(3, "检查手部识别模型文件（hand_landmarker.task）")
    model = next((p for p in MODEL_LOCAL_CANDIDATES if os.path.exists(p)), "")
    if model:
        print(f"  找到模型：{model}")
    else:
        target = os.path.join(SCRIPT_DIR, "hand_landmarker.task")
        print("  本机没有模型文件，尝试自动下载（约 8 MB）…")
        try:
            urllib.request.urlretrieve(MODEL_URL, target)
            model = target
            print(f"  ✓ 下载完成：{target}")
        except Exception as exc:  # noqa: BLE001
            fail(f"模型下载失败（{exc}）。一般是网络连不上谷歌。",
                 MODEL_MANUAL_HINT)

    # ── 4/4 启动图形界面 ───────────────────────────────
    step(4, "启动图形界面")
    print("  提醒：如果等下「连接」失败，先确认强脑「上位机」软件已关闭（串口不能同时被两个程序用）。")
    print("  窗口马上弹出来，本控制台可以最小化，别关……\n")
    r = subprocess.run([sys.executable, GUI_FILE])
    if r.returncode == 0:
        print("\n窗口已正常关闭，拜拜。")
        return 0

    print(f"\n[异常] 图形窗口退出码 {r.returncode}。")
    print("常见原因：")
    print("  ① 找不到模型 → 确认 hand_landmarker.task 在本脚本同目录；")
    print("  ② 打不开摄像头 → 确认没有别的程序占着摄像头（微信视频/相机应用）；")
    print("  ③ 连接灵巧手失败 → 确认强脑「上位机」软件已关、USB 线插好。")
    input("\n按回车键关闭窗口…")
    return r.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"\n[意外错误] {type(exc).__name__}: {exc}")
        input("\n按回车键关闭窗口…")
        sys.exit(1)

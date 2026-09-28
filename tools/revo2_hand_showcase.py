#!/usr/bin/env python3
"""Revo2 灵巧手 · 手势大全（12 个动作段子）

用法：
    python3 hand_showcase.py                 全部连播
    python3 hand_showcase.py --list          列出所有段子
    python3 hand_showcase.py --only 心跳     只跑指定段子
    python3 hand_showcase.py --only 倒计时 --hold 0.6

位置量程 0~1000：0 = 完全伸直，1000 = 完全弯曲。
数组顺序 [拇指屈曲, 拇指对掌, 食指, 中指, 无名指, 小指]。
"""
from __future__ import annotations

import argparse
import asyncio
import sys

SRC = "/home/bxi/bxi_ws/bxi_revo2_example/src"
if SRC not in sys.path:
    sys.path.insert(0, SRC)

import rclpy  # noqa: E402
from bxi_can_node import (  # noqa: E402
    cleanup_bxipci_device,
    init_bxipci_device,
    libstark,
    stop_bxipci_runtime,
)


def P(tf=0, ta=0, i=0, m=0, r=0, p=0):
    """构造一个 6 元组位置（拇指屈曲/对掌, 食, 中, 无名, 小）。"""
    return [tf, ta, i, m, r, p]


OPEN = P()
FIST = P(1000, 1000, 1000, 1000, 1000, 1000)

# 手指下标
I_IND, I_MID, I_RNG, I_PNK = 2, 3, 4, 5

# 数字 1~10（中国式）
NUM = {
    1: P(1000, 1000,    0, 1000, 1000, 1000),   # 只伸食指
    2: P(1000, 1000,    0,    0, 1000, 1000),
    3: P(1000, 1000,    0,    0,    0, 1000),
    4: P(1000, 1000,    0,    0,    0,    0),   # 除拇指外四指
    5: P(   0,    0,    0,    0,    0,    0),   # 全伸
    6: P(   0,    0, 1000, 1000, 1000,    0),   # 拇指 + 小指（打电话）
    7: P( 646,  983,    0,  908,    0,    0),   # 拇指食中捏（取自 OK 标定）
    8: P(   0,    0,    0, 1000, 1000, 1000),   # 拇指 + 食指（枪）
    9: P(1000, 1000,  600, 1000, 1000, 1000),   # 食指成钩
   10: P(1000, 1000, 1000, 1000, 1000, 1000),   # 握拳
}

ROCKN = P(0, 0, 0, 1000, 1000, 0)                 # 摇滚 🤘 拇指+食指+小指伸直
CALL = P(0, 0, 1000, 1000, 1000, 0)               # 打电话 ☎️ 拇指+小指伸直
GUN = P(0, 0, 0, 1000, 1000, 1000)                # 手枪 🔫 拇指+食指伸直
GUN_FIRE = P(0, 0, 450, 1000, 1000, 1000)         # 扣扳机（食指弯曲）
THUMBSUP = P(0, 0, 1000, 1000, 1000, 1000)
HEART = P(821, 801, 777, 1000, 1000, 1000)      # 比心（拇指+食指成环，其余握）

# 段子注册表：名字 -> (说明, 函数)
ACTS: dict[str, tuple[str, object]] = {}


def act(name: str, desc: str):
    def deco(fn):
        ACTS[name] = (desc, fn)
        return fn
    return deco


# ----------------------------------------------------------------------
# 基础动作原语
# ----------------------------------------------------------------------

async def ramp(h, sid, frm, to, dur: float, rate: float) -> None:
    """把位置从 frm 线性过渡到 to。"""
    dt = 1.0 / rate
    n = max(1, int(dur * rate))
    for k in range(1, n + 1):
        s = k / n
        pos = [int(round(a + (b - a) * s)) for a, b in zip(frm, to)]
        await h.set_finger_positions(sid, pos)
        await asyncio.sleep(dt)


async def hold_pose(h, sid, pos, dur: float) -> None:
    await h.set_finger_positions(sid, pos)
    await asyncio.sleep(dur)


async def ripple_pass(h, sid, seq, period: float, delay: float, rate: float) -> None:
    """一轮轮指：弯曲像波一样按 seq 顺序流过。"""
    dur = period + delay * (len(seq) - 1)
    dt = 1.0 / rate
    t = 0.0
    while t <= dur + 1e-9:
        pos = [0] * 6
        for k, idx in enumerate(seq):
            ph = ((t - k * delay) / period) % 1.0
            v = 1.0 - abs(2.0 * ph - 1.0)          # 三角波 0→1→0
            pos[idx] = int(round(max(0.0, min(1.0, v)) * 1000))
        await h.set_finger_positions(sid, pos)
        await asyncio.sleep(dt)
        t += dt
    await h.set_finger_positions(sid, [0] * 6)


# ----------------------------------------------------------------------
# 1. 轮指波浪
# ----------------------------------------------------------------------

@act("轮指波浪", "弯曲像水波依次流过四指，往返多轮")
async def act_ripple(h, sid, a) -> None:
    fwd = [I_IND, I_MID, I_RNG, I_PNK]
    for r in range(1, a.ripple_rounds + 1):
        print(f"  第 {r}/{a.ripple_rounds} 轮 正向（食指→小指）", flush=True)
        await ripple_pass(h, sid, fwd, a.ripple_period, a.ripple_delay, a.rate)
        await asyncio.sleep(a.gap)
        print(f"  第 {r}/{a.ripple_rounds} 轮 反向（小指→食指）", flush=True)
        await ripple_pass(h, sid, list(reversed(fwd)), a.ripple_period,
                          a.ripple_delay, a.rate)
        await asyncio.sleep(a.gap)


# ----------------------------------------------------------------------
# 2. 数字 1-5 / 6-10
# ----------------------------------------------------------------------

@act("数字1-5", "依次比出 1 到 5")
async def act_num15(h, sid, a) -> None:
    for n in range(1, 6):
        await hold_pose(h, sid, NUM[n], a.hold)
        print(f"    {n}", flush=True)
        await asyncio.sleep(a.gap)


@act("数字6-10", "接着比出 6 到 10")
async def act_num610(h, sid, a) -> None:
    for n in range(6, 11):
        await hold_pose(h, sid, NUM[n], a.hold)
        print(f"    {n}", flush=True)
        await asyncio.sleep(a.gap)


@act("数字1-10", "一口气从 1 数到 10")
async def act_num110(h, sid, a) -> None:
    for n in range(1, 11):
        await hold_pose(h, sid, NUM[n], a.hold * 0.7)
        print(f"    {n}", flush=True)
        await asyncio.sleep(a.gap * 0.6)


# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
# 4. 点赞
# ----------------------------------------------------------------------

@act("点赞", "拇指伸出、四指握紧")
async def act_thumbsup(h, sid, a) -> None:
    await hold_pose(h, sid, OPEN, a.hold * 0.5)
    await ramp(h, sid, OPEN, THUMBSUP, 0.35, a.rate)
    for _ in range(a.pump):
        await hold_pose(h, sid, THUMBSUP, a.hold)
        await ramp(h, sid, THUMBSUP,
                   P(0, 0, 800, 800, 800, 800), 0.18, a.rate)
        await ramp(h, sid, P(0, 0, 800, 800, 800, 800),
                   THUMBSUP, 0.18, a.rate)
    print("    赞！", flush=True)


# ----------------------------------------------------------------------
# 5. 倒计时
# ----------------------------------------------------------------------

@act("倒计时", "5→4→3→2→1→0，逐根收手指")
async def act_countdown(h, sid, a) -> None:
    seq = [NUM[5], P(0, 0, 0, 0, 0, 1000), P(0, 0, 0, 0, 1000, 1000),
           P(0, 0, 0, 1000, 1000, 1000), P(0, 0, 1000, 1000, 1000, 1000),
           FIST]
    for lbl, pos in zip((5, 4, 3, 2, 1, 0), seq):
        await hold_pose(h, sid, pos, a.hold)
        print(f"    {lbl}", flush=True)
        await asyncio.sleep(a.gap)


# ----------------------------------------------------------------------
# 6. 心跳 / 脉动
# ----------------------------------------------------------------------

@act("心跳", "五指同步快速收放，模拟心跳节奏")
async def act_heartbeat(h, sid, a) -> None:
    amp = P(350, 350, 400, 400, 400, 400)
    for beat in range(1, a.beats + 1):
        await ramp(h, sid, OPEN, amp, 0.09, a.rate)       # 咚
        await ramp(h, sid, amp, OPEN, 0.13, a.rate)
        await asyncio.sleep(0.06)
        await ramp(h, sid, OPEN, P(250, 250, 300, 300, 300, 300), 0.08, a.rate)
        await ramp(h, sid, P(250, 250, 300, 300, 300, 300), OPEN, 0.18, a.rate)
        print(f"    ♥ {beat}", flush=True)
        await asyncio.sleep(0.22)


# ----------------------------------------------------------------------
# 7. 敲桌子
# ----------------------------------------------------------------------

@act("敲桌子", "四指像弹钢琴一样依次落下")
async def act_tap(h, sid, a) -> None:
    hit = 700
    for r in range(1, a.tap_rounds + 1):
        for idx in (I_IND, I_MID, I_RNG, I_PNK):
            pos = [0] * 6
            pos[idx] = hit
            await ramp(h, sid, [0] * 6, pos, 0.05, a.rate)
            await ramp(h, sid, pos, [0] * 6, 0.07, a.rate)
        print(f"    第 {r}/{a.tap_rounds} 轮", flush=True)
        await asyncio.sleep(0.12)


# ----------------------------------------------------------------------
# 8. 螺旋（含拇指，逐根卷入再展开）
# ----------------------------------------------------------------------

@act("螺旋", "从拇指到小指逐根卷入，再逐根展开")
async def act_spiral(h, sid, a) -> None:
    order = [0, 1, I_IND, I_MID, I_RNG, I_PNK]
    cur = [0] * 6
    for k in order:
        cur = list(cur)
        cur[k] = 1000
        await ramp(h, sid, [v if j != k else 0 for j, v in enumerate(cur)],
                   cur, 0.16, a.rate)
    print("    → 全握", flush=True)
    await asyncio.sleep(a.gap)
    for k in reversed(order):
        nxt = list(cur)
        nxt[k] = 0
        await ramp(h, sid, cur, nxt, 0.16, a.rate)
        cur = nxt
    print("    → 全展", flush=True)


# ----------------------------------------------------------------------
# 9. 抓握演示
# ----------------------------------------------------------------------

@act("抓握", "五指收拢成抓球状，反复松紧")
async def act_grasp(h, sid, a) -> None:
    ball = P(700, 700, 780, 800, 800, 800)
    tight = P(850, 850, 950, 950, 950, 950)
    loose = P(200, 200, 250, 250, 250, 250)
    await ramp(h, sid, OPEN, ball, 0.35, a.rate)
    for i in range(1, a.pump + 1):
        await ramp(h, sid, ball, tight, 0.22, a.rate)
        await ramp(h, sid, tight, loose, 0.28, a.rate)
        await ramp(h, sid, loose, ball, 0.28, a.rate)
        print(f"    松紧 {i}/{a.pump}", flush=True)
    await ramp(h, sid, ball, OPEN, 0.3, a.rate)


# ----------------------------------------------------------------------
# 手势秀（客户演示）
# ----------------------------------------------------------------------

@act("摇滚", "摇滚手势 🤘 摆出后律动两次")
async def act_rock(h, sid, a) -> None:
    mid = P(300, 300, 300, 1000, 1000, 300)
    await ramp(h, sid, OPEN, ROCKN, 0.30, a.rate)
    for i in range(1, 3):
        await ramp(h, sid, ROCKN, mid, 0.22, a.rate)
        await ramp(h, sid, mid, ROCKN, 0.22, a.rate)
        print(f"    🤘 {i}/2", flush=True)
    await asyncio.sleep(a.hold * 0.8)
    await ramp(h, sid, ROCKN, OPEN, 0.25, a.rate)


@act("打电话", "拇指小指一伸一收 ☎️")
async def act_call(h, sid, a) -> None:
    tip = P(0, 0, 1000, 1000, 1000, 300)
    await ramp(h, sid, OPEN, CALL, 0.30, a.rate)
    await asyncio.sleep(a.hold * 0.8)
    for i in range(1, 3):
        await ramp(h, sid, CALL, tip, 0.18, a.rate)
        await ramp(h, sid, tip, CALL, 0.18, a.rate)
        print(f"    ☎️ {i}/2", flush=True)
    await asyncio.sleep(a.hold * 0.5)
    await ramp(h, sid, CALL, OPEN, 0.25, a.rate)


@act("手枪射击", "摆出手枪连扣三次扳机 🔫")
async def act_gun(h, sid, a) -> None:
    await ramp(h, sid, OPEN, GUN, 0.30, a.rate)
    for i in range(1, 4):
        await ramp(h, sid, GUN, GUN_FIRE, 0.09, a.rate)
        await ramp(h, sid, GUN_FIRE, GUN, 0.09, a.rate)
        print(f"    🔫 {i}/3", flush=True)
        await asyncio.sleep(0.15)
    await asyncio.sleep(a.hold * 0.5)
    await ramp(h, sid, GUN, OPEN, 0.25, a.rate)


# ----------------------------------------------------------------------
# 10. 比心
# ----------------------------------------------------------------------

@act("比心", "拇指与食指成环、其余握起（近似比心）")
async def act_heart(h, sid, a) -> None:
    await ramp(h, sid, OPEN, HEART, 0.40, a.rate)
    await asyncio.sleep(a.hold)
    for _ in range(a.pump):
        await ramp(h, sid, HEART, P(760, 740, 700, 1000, 1000, 1000),
                   0.22, a.rate)
        await ramp(h, sid, P(760, 740, 700, 1000, 1000, 1000), HEART,
                   0.22, a.rate)
    print("    ♡", flush=True)
    await asyncio.sleep(a.hold)


# ----------------------------------------------------------------------
# 11. 招手 / 再见
# ----------------------------------------------------------------------

@act("招手", "四指快速卷曲伸展，模拟挥手告别")
async def act_wave_hello(h, sid, a) -> None:
    curl = P(0, 0, 650, 650, 650, 650)
    for i in range(1, a.waves + 1):
        await ramp(h, sid, OPEN, curl, 0.13, a.rate)
        await ramp(h, sid, curl, OPEN, 0.13, a.rate)
        print(f"    挥手 {i}/{a.waves}", flush=True)
    await asyncio.sleep(a.gap)


# ----------------------------------------------------------------------

async def run(args) -> int:
    if args.list:
        print("可用段子：")
        for i, (n, (d, _)) in enumerate(ACTS.items(), 1):
            print(f"  {i:2d}. {n:<10} {d}")
        return 0

    libstark.init_logging()
    ctx = None
    try:
        print("[连接] right hand  bus=6  slave_id=127  CANFD")
        ctx = await init_bxipci_device(
            6, 127, master_id=1, is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        )
        h, sid = ctx.handle, ctx.slave_id
        await asyncio.sleep(0.5)
        info = await h.get_device_info(sid)
        print(f"[设备] {info.hardware_type}  固件={info.firmware_version}")

        if args.read_only:
            st = await h.get_motor_status(sid)
            print(f"[只读] {list(st.positions)}")
            return 0

        await h.set_finger_positions(sid, OPEN)
        await asyncio.sleep(0.8)

        names = list(ACTS)
        if args.only:
            sel = str(args.only).strip()
            if sel.isdigit():
                idx = int(sel)
                if not (1 <= idx <= len(names)):
                    print(f"编号超出范围：1 ~ {len(names)}")
                    return 2
                todo = [names[idx - 1]]
            elif sel in ACTS:
                todo = [sel]
            else:
                print(f"没有这个段子：{sel}  （用 --list 查看，或直接给编号）")
                return 2
        else:
            todo = names

        for name in todo:
            desc, fn = ACTS[name]
            print(f"\n========== {name} —— {desc} ==========", flush=True)
            await fn(h, sid, args)
            await asyncio.sleep(args.gap * 2)
            await h.set_finger_positions(sid, OPEN)
            await asyncio.sleep(0.5)

        st = await h.get_motor_status(sid)
        print(f"\n[结束] 位置 = {list(st.positions)}")
        return 0
    finally:
        if ctx is not None:
            await cleanup_bxipci_device(ctx)


def main() -> int:
    ap = argparse.ArgumentParser(description="灵巧手手势大全（12 个段子）")
    ap.add_argument("--list", action="store_true", help="列出所有段子")
    ap.add_argument("--only", default=None,
                    help="只跑指定段子：给名字，或给编号（1 起，见 --list）")
    ap.add_argument("--hold", type=float, default=0.75, help="静态手势保持时长（秒）")
    ap.add_argument("--gap", type=float, default=0.10, help="段子/动作间最短间隔（秒）")
    ap.add_argument("--rate", type=float, default=30.0, help="控制频率 Hz")
    ap.add_argument("--pump", type=int, default=3, help="松紧/脉动重复次数")
    ap.add_argument("--beats", type=int, default=4, help="心跳拍数")
    ap.add_argument("--tap-rounds", type=int, default=3, help="敲桌子轮数")
    ap.add_argument("--waves", type=int, default=4, help="招手次数")
    ap.add_argument("--ripple-rounds", type=int, default=2, help="轮指往返轮数")
    ap.add_argument("--ripple-period", type=float, default=0.50)
    ap.add_argument("--ripple-delay", type=float, default=0.12)
    ap.add_argument("--rps-rounds", type=int, default=2, help="石头剪刀布轮数")
    ap.add_argument("--read-only", action="store_true")
    args = ap.parse_args()

    rclpy.init(args=None)
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[中断]")
        return 0
    finally:
        stop_bxipci_runtime()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    raise SystemExit(main())

# 19 · Windows 一键操作灵巧手

> 面向不习惯命令行、也不想在 Windows 上装 ROS 的使用者。
> 本篇说明如何在 Windows 上**双击一个文件**就让灵巧手做动作，
> 以及第一次使用要做什么、出问题怎么查。
>
> 前置阅读：[`docs/18-真机灵巧手控制与手势库.md`](18-真机灵巧手控制与手势库.md)

---

## 0. 它解决什么问题

按 `docs/18` 的做法控制灵巧手，需要记住一串 SSH 命令、以 root 身份运行、
并且正确设置 ROS 域与 CycloneDDS 环境变量——**任何一项漏掉都会静默失败**。

本入口把这些全部固化进一个 `.bat` 文件：

- 不需要安装 ROS 或任何 CAN 驱动
- 不需要手动输入 SSH 命令
- 首次运行自动配置免密登录
- 菜单式选择，无需记忆参数

---

## 1. 前置条件

| 项 | 要求 |
|---|---|
| 操作系统 | Windows 10 / 11（自带 OpenSSH 客户端） |
| 网络 | 与机器人在**同一网段**，能 `ping` 通机器人 |
| 机器人 | 已上电、硬件节点已启动（`/hardware_elf3` 在跑） |
| 灵巧手 | 已装配并上电，指示灯常亮 |

**快速自检**：在命令提示符里执行 `ping 172.16.10.47`，通则具备条件。

---

## 2. 获取与放置

从仓库取 `tools/revo2_hand_console.bat`，**双击运行**即可。

> ⚠️ **放置路径请用纯英文**。含中文的路径在部分 Windows 环境下会导致批处理解析异常。
> 建议放在如 `D:\sim\` 这样的目录下。

---

## 3. 第一次运行：自动配置免密

首次运行会检测本机是否已能免密登录机器人。若尚未配置，会提示：

```
FIRST RUN - setting up passwordless login
    robot : bxi@172.16.10.47
    passwd: ask your supervisor (typing below)
```

此时输入机器人登录密码（**向主管索取，本文档不记录凭据**）。工具会自动：

1. 生成 SSH 密钥对（若本机还没有）
2. 把公钥追加到机器人的 `authorized_keys`
3. 完成后续免密登录

**配置成功后，之后运行不再需要输入密码。**

---

## 4. 菜单

```
[1] Full showcase   ALL 13 acts            about 55s
[2] Pick one act    enter act number 1-13
[3] List acts       show all act names
[4] Wave fist       wave fist x3 + continuous OK
[5] Single fist     fist - hold - open
[6] Hand info       read model / firmware   NO motion
[0] Exit
```

| 选项 | 说明 | 是否动作 |
|---|---|---|
| **1** | 连续播放全部 13 段手势，约 55 秒 | 是 |
| **2** | 输入编号单独播放某一段（编号见选项 3） | 是 |
| **3** | 列出 13 段手势的名称与编号 | 否 |
| **4** | 波浪握拳 + 连续 OK 手势 | 是 |
| **5** | 单次握拳 → 保持 → 张开 | 是 |
| **6** | 读取手的型号、固件、当前位置，**不发任何动作** | 否 |

**建议首次使用先选 `6`**：确认能读到手的信息，再执行动作。

### 13 段手势编号

| 1 轮指波浪 | 2 数字 1-5 | 3 数字 6-10 | 4 数字 1-10 |
|---|---|---|---|
| 5 石头剪刀布 | 6 点赞 | 7 倒计时 | 8 心跳 |
| 9 敲桌子 | 10 螺旋 | 11 抓握 | 12 比心 |
| 13 招手 | | | |

---

## 5. 典型流程

```
① 双击 bat
② 首次：输入一次机器人密码完成免密配置
③ 选 [6] 确认读到手的信息（型号 / 固件 / SN）
④ 选 [3] 看段子编号
⑤ 选 [2] 挑一段试跑，或选 [1] 跑完整场
```

---

## 6. 常见问题

| 现象 | 原因与处理 |
|---|---|
| 一闪而过 / 窗口立即关闭 | 双击后窗口关闭属正常结束；若来不及看结果，可在命令提示符里运行该 bat |
| `Permission denied (publickey,password)` | 免密未配置成功。确认步骤 3 输入了正确密码；或删除本机 `~/.ssh/known_hosts` 中该主机记录后重试 |
| 提示 "cannot reach the robot" | 机器人未开机 / 不在同一网段 / 硬件节点未启动。先 `ping` 测试 |
| 菜单能出来，但手不动 | 检查机器人硬件节点是否在跑；灵巧手指示灯是否常亮 |
| 中文显示乱码 | 文件已含 `chcp 65001`（切 UTF-8）；若仍乱码，改用 Windows Terminal 或 PowerShell 运行 |
| 提示 `sudo: password` 反复要求 | sudo 凭据默认 15 分钟有效，过期后需重新输入一次 |

---

## 7. 它背后做了什么

了解原理有助于排查问题。双击 bat 后，实际执行链是：

```
revo2_hand_console.bat
    │  ssh（已免密）
    ▼
机器人 bxi@172.16.10.47
    │  sudo bash /home/bxi/run.sh <脚本> [参数]
    ▼
run.sh：source /home/bxi/hand_env.sh
    │       （设置 ROS_DOMAIN_ID=22 与 CycloneDDS 配置）
    ▼
python3 <脚本>  →  经 /canfd_packet 驱动灵巧手
```

**必须以 root（`sudo`）运行**：硬件节点以 root 身份运行，
非 root 下环境变量与通信配置不一致，会导致"看似连上、手却无应答"。

---

## 8. 相关文件

| 文件 | 作用 |
|---|---|
| `tools/revo2_hand_console.bat` | 本入口（Windows） |
| `tools/revo2_hand_console.sh` | Linux 上位机控制台（等价功能） |
| `tools/revo2_hand_run.sh` | 机器人侧运行封装（固化环境变量） |
| `tools/revo2_hand_showcase.py` | 13 段手势库 |
| `tools/revo2_hand_gestures.py` | 4 段精选手势 |
| `tools/revo2_hand_fist.py` | 最小示例（握拳 → 保持 → 张开） |

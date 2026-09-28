# Ubuntu 22.04 环境搭建与 MuJoCo 仿真（ROG G733PZ 笔记本版）

> **适用对象**：已经装完 Ubuntu 22.04 + ROS 2 Humble，但**其余什么都没做**，
> 且被"下载太慢 / 到后面直接超时"卡住的人。
>
> **目标机器**：华硕 ROG Strix **G733PZ**（见下方硬件台账）。
> **最终产出**：能跑 MuJoCo 仿真、能开 RViz2、能编译并运行半醒官方控制器示例。
>
> 本文所有 `sudo apt` / `pip` 命令都按国内网络写过源，**可以直接照抄**。

---

## 0. 先看这一页

### 0.1 你这台机器的硬件台账（截图核对过）

| 部件 | 实际型号 | 对本次任务的影响 |
|---|---|---|
| CPU | **AMD Ryzen 9 7945HX**（16 核 32 线程） | 编译 `colcon` 很快，不是瓶颈 |
| 内存 | **32 GB DDR5-5200**（16+16） | 够用；MuJoCo + RViz2 同时开也没压力 |
| 独显 | **NVIDIA RTX 4080 Laptop 12 GB** | ★ 需要 ≥525 版驱动；MuJoCo 渲染要靠它 |
| 核显 | **AMD Radeon 610M**（在 7945HX 里） | ★ 双显卡，默认可能跑在核显上，渲染会卡 |
| 屏幕 | 17.1" 2560×1440（NE173QHM-NZ2） | 正常 |
| 系统盘 | SAMSUNG MZVL21T0HCLR 1 TB | Windows 在这块 |
| 数据盘 | Predator SSD GM7 M.2 2 TB | ★ Ubuntu 大概率在这块 |
| 有线网卡 | **Realtek Gaming 2.5GbE** | ★ 连机器人就用它，免驱 |
| 无线网卡 | **MediaTek Wi-Fi 6E MT7922 (RZ616)** | ★ 老内核对它支持差，见 §2.1 |
| 原系统 | Win11 Home China 23H2 | 双系统 |

### 0.2 三个"先天坑"，先知道能省几小时

| # | 坑 | 症状 | 对策 |
|---|---|---|---|
| 1 | **网络** | `apt` / `pip` / GitHub 下载慢到超时 | **第 1 步就换源**，这是所有问题的根 |
| 2 | **MT7922 无线网卡** | 装完 Ubuntu 后 Wi-Fi 图标都没有 / 频繁掉线 | 装 HWE 内核（§2.1）；连机器人改走**有线** |
| 3 | **双显卡** | 风扇狂转但画面卡、MuJoCo 黑屏、`nvidia-smi` 报错 | 装 NVIDIA 驱动 + PRIME 设置（§2.2–2.3） |

> **强烈建议**：先把网线插上再开始。有线（Realtek 2.5G）在 Ubuntu 下免驱，
> 而且你后面连机器人本来就得用有线，等于一次做两件事。

### 0.3 七步总览

| 步 | 做什么 | 大约耗时 | 是否必须 |
|---|---|---|---|
| 0 | 在 **Windows** 侧先把大文件下好（§1） | 10 分钟 | 强烈建议 |
| 1 | **换源**（apt / ROS2 / pip / GitHub） | 10 分钟 | ★ 必须，先做 |
| 2 | HWE 内核 + NVIDIA 驱动 | 20 分钟（含重启） | ★ 必须 |
| 3 | ROS2 Humble 补全（**含 RViz2**） | 15 分钟 | ★ 必须 |
| 4 | MuJoCo（官方仿真环境 / pip 独立） | 30 分钟 | ★ 你要的 |
| 5 | 官方控制器示例编译 + 跑仿真 | 20 分钟 | ★ 重点 |
| 6 | 验证清单 | 5 分钟 | 必须 |

**救急版**：如果你只想最快看到东西动起来，按 **1 → 2 → 3 → 4B** 走，
跳过第 5 步的官方控制器（那步最耗时，且需要 63 MB 的 GitHub 包）。

---

## 1. 第 0 步：在 Windows 侧先做的事（省时间的关键）

你现在还在 Windows。**这一步能让你少受一半网络的苦。**

### 1.1 优先路线：直接从机器人上拷（最快的路）

机器人机载电脑上**本来就有整套环境**，不用下载：

```bash
# 在 Windows 上（Git Bash 或 PowerShell 都行）
# 先问同事要机器人 IP，假设是 elf3-82
ssh bxi@elf3-82.local          # 口令问主管/负责同事
# 进去看一眼有没有这些东西：
ls /opt/bxi/                   # 应该有 bxi_ros2_pkg 和 bxi_rl_controller_ros2_example
du -sh /opt/bxi/*
```

如果都在，直接整个拷回来（**比从 GitHub 下载快得多**）：

```bash
# 在 Windows 的 Git Bash 里执行，拷到 D 盘临时目录
mkdir -p /d/bxi_backup
scp -r bxi@elf3-82.local:/opt/bxi/bxi_ros2_pkg /d/bxi_backup/
scp -r bxi@elf3-82.local:/opt/bxi/bxi_rl_controller_ros2_example /d/bxi_backup/
```

### 1.2 备选路线：在 Windows 上下载官方 63 MB 预编译包

> ⚠️ **实测（2026-09-20）**：本网络到 GitHub 的 release 资产
> （`release-assets.githubusercontent.com` / `codeload.github.com`）**直连不可达**，
> 会一直卡住直到超时 —— 这就是你"下到后面直接超时"的直接原因。
> `api.github.com` 和 `raw.githubusercontent.com` 反而正常。

所以下载 GitHub 上的大文件，**必须加国内加速前缀**：

| 通道 | 上次实测 | 用法 |
|---|---|---|
| `ghfast.top` | ✅ 可用（但会失效，逐个试） | 在原 URL 前拼 `https://ghfast.top/` |
| `gh-proxy.com` | ✅ 可用 | 同上 |
| `ghproxy.net` | ✅ 可用 | 同上 |
| `github.moeyy.xyz` | ❌ 不通 | — |
| `hub.gitmirror.com` | ❌ 不通 | — |

```bash
# 官方控制器示例的预编译包（63 MB，不用自己编译）
# 在 Windows 上用一个能下的通道拉下来，放到 D 盘
U="https://github.com/bxirobotics/bxi_controller_ros2/releases/download/v260920/bxi_rl_controller_ros2_example_v260920_amd64.tar.gz"
curl -L --retry 5 -C - -o /d/bxi_backup/ctrl_v260920.tar.gz "https://ghfast.top/$U"
# 校验（很重要，代理可能返回残缺文件）
curl -L -o /d/bxi_backup/ctrl.tar.gz.sha256 "https://ghfast.top/$U.sha256"
cd /d/bxi_backup && sha256sum -c ctrl.tar.gz.sha256
```

若 `ghfast.top` 也不行，就换 `gh-proxy.com` / `ghproxy.net` 再试；
**再不行就直接用 §1.1 从机器人拷**，不要硬啃。

### 1.3 把本仓库在 Ubuntu 里找出来

本仓库在 Windows 的 `D:\Desktop\机器人实习\灵巧手\elf3-humanoid\`。
切到 Ubuntu 后，NTFS 分区需要挂载：

```bash
lsblk -f                              # 找 ntfs 分区，记下设备名，例如 nvme1n1p3
sudo mkdir -p /mnt/win
sudo mount -t ntfs3 /dev/nvme1n1p3 /mnt/win
ls "/mnt/win/Desktop/机器人实习/灵巧手/elf3-humanoid/dexterous-hand"
```

> 想开机自动挂载就写 `/etc/fstab`，但 **NTFS 用 `ntfs3` 驱动别用老的 `ntfs-3g`**，
> 后者中文路径和权限容易出问题。图省事就手动 `mount`。

---

## 2. 第 1 步：换源 —— 从根上治"下载慢 / 超时"

> 这一步没做，后面每一步都会卡。**先做这一步，再干别的。**

### 2.1 Ubuntu apt 源 → 清华（本网络实测最快最稳）

**实测结果（2026-09-20，本机这条网络）**：

| 镜像 | 结果 |
|---|---|
| 清华 `mirrors.tuna.tsinghua.edu.cn` | ✅ **可用，推荐** |
| 中科大 `mirrors.ustc.edu.cn` | ✅ 可用 |
| 华为云 `mirrors.huaweicloud.com` | ✅ 可用 |
| 腾讯云 `mirrors.cloud.tencent.com` | ✅ 可用 |
| 阿里云 `mirrors.aliyun.com` | ❌ **本网络不通**（返回 502），别用 |

```bash
# ① 备份原源
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak.$(date +%F)

# ② 写入清华源（Ubuntu 22.04 = jammy）
sudo tee /etc/apt/sources.list > /dev/null <<'EOF'
# 清华 TUNA 镜像 —— Ubuntu 22.04 jammy
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse
EOF

# ③ 生效
sudo apt update
```

> **22.04 之后如果还有 `ubuntu.sources`（deb822 格式）**，
> `/etc/apt/sources.list` 可能是空的、真正生效的是
> `/etc/apt/sources.list.d/ubuntu.sources`。先确认：
> `grep -r "URIs\|^deb " /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null`
> 哪个文件里有内容就改哪个。

### 2.2 ROS 2 apt 源 → 清华 ros2 镜像

ROS 2 的包**不在 Ubuntu 源里**，在 `packages.ros.org`，国内也很慢。清华有专门镜像：

```bash
# ① 装密钥（走 Ubuntu 源，此时已经很快了）
sudo apt install -y curl gnupg lsb-release ca-certificates

# ② 如果之前配过官方源，先删掉
sudo rm -f /etc/apt/sources.list.d/ros2.list

# ③ 写入清华 ROS2 镜像源
echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ④ ROS2 源的签名密钥（用清华的，别去 keyserver.ubuntu.com 慢慢等）
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ros2-archive-keyring.gpg \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

> 如果 `apt update` 报 **`NO_PUBKEY`**，说明密钥没进来。用清华镜像里的
> `ros2-archive-keyring.gpg`（上面这条），或从机器人上拷
> `/usr/share/keyrings/ros-archive-keyring.gpg` 过来 —— **别去 keyserver**，那个在国内基本连不上。

### 2.3 pip 源 → 清华 PyPI

```bash
mkdir -p ~/.pip
cat > ~/.pip/pip.conf <<'EOF'
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
extra-index-url = https://mirrors.ustc.edu.cn/pypi/simple
timeout = 60
retries = 5
EOF

# 顺手把 pip 自己升级了
python3 -m pip install -U pip
```

> **为什么 `timeout`/`retries` 要写**：你遇到的"下到后面超时"，
> pip 默认超时只有 15 秒、重试 5 次，国内网络偶尔抽风就直接失败。
> 调大能显著减少假失败。

### 2.4 一键做完上面全部

```bash
cd "/mnt/win/Desktop/机器人实习/灵巧手/elf3-humanoid/dexterous-hand"
bash tools/ubuntu_bootstrap.sh --stage mirror
```

（脚本会先备份、再改，支持 `--dry-run` 只看不改，详见脚本头注释。）

### 2.5 换源后的常见报错

| 报错 | 原因 | 解决 |
|---|---|---|
| `Could not resolve 'mirrors.aliyun.com'` | 用了本网络不通的阿里云 | 换清华源 |
| `NO_PUBKEY F42ED6FBAB17C654` | ROS2 密钥缺失 | 用 §2.2 第 ④ 步的清华 keyring |
| `Certificate verification failed` | 系统时间不对 | `sudo apt install --reinstall ca-certificates`；`timedatectl` 看时间 |
| `Hash Sum mismatch` | 镜像正在同步 | 过几分钟重试，或换中科大源 |
| `E: 无法获得锁 /var/lib/dpkg/lock` | 另一个 apt 在跑 | 等它结束，别强杀 |

---

## 3. 第 2 步：内核与驱动（这台笔记本的真正门槛）

### 3.1 先确认内核版本（MT7922 无线网卡的关键）

```bash
uname -r
```

- **≥ 6.2** → 无线网卡没问题，跳到 §3.2；
- **5.15（22.04 出厂内核）** → MT7922 大概率认不出来或频繁掉线，**必须升 HWE 内核**。

```bash
# 装 HWE 内核（22.04 的硬件支持内核，会带到 6.8 系列）
sudo apt install -y linux-generic-hwe-22.04
sudo reboot

# 重启后再看
uname -r          # 应该变成 6.8.x
nmcli device      # 看 wifi 有没有出现
```

> **连不上网怎么办**：HWE 内核要联网下载。所以顺序是
> **先插网线**做 §2 换源 → 再升内核。有线是 Realtek 2.5G（`r8169` 驱动），免驱。

### 3.2 NVIDIA 驱动（RTX 4080 Laptop）

**本机实测**：Ubuntu 22.04 的 `jammy-updates` 仓库里有
`535 / 545 / 550 / 570 / 580 / 590` 多个系列 —— 4080 是 Ada 架构，**需要 ≥525**，
所以上面这些都能用。

**推荐做法（让系统自己选）**：

```bash
# ① 让它自己推荐
ubuntu-drivers devices

# ② 自动装推荐的那个
sudo ubuntu-drivers install
```

**如果自动装不上，手动指定**（550 是 22.04 上比较稳的一个）：

```bash
sudo apt install -y nvidia-driver-550
# 也可考虑开源内核模块版本（40 系支持好，二者选一）
# sudo apt install -y nvidia-driver-550-open
sudo reboot
```

**验证**：

```bash
nvidia-smi
# 期望：看到 RTX 4080 Laptop，Driver Version 550.x，显存 12282MiB
```

#### ⚠️ Secure Boot（这一步不过，驱动永远装不上）

Ubuntu 装驱动时会弹一个蓝底白字的 **MOK 注册界面**：

1. 要求你设一个**一次性密码**（8 位以上，自己记一记）；
2. 重启后出现蓝色 **`Perform MOK management`** 界面；
3. 选 **`Enroll MOK`** → `Continue` → `Yes` → 输入刚才那个密码 → `Reboot`。

**如果错过了这个界面**：`sudo mokutil --import /var/lib/shim-signed/mok/MOK.der`，
它会再要你设一次密码，然后重启重走一遍。

**如果不想折腾**：进 BIOS 关掉 Secure Boot（ROG 是按 `F2` 进 BIOS，
`Security` → `Secure Boot Control` → `Disabled`）。这是最省事的，
但双系统下 Win11 也可能抱怨，自行权衡。

> 驱动装完 `nvidia-smi` 还是报 `NVIDIA-SMI has failed because it couldn't communicate
> with the NVIDIA driver` —— 九成是 Secure Boot / MOK 没弄好，或者内核换了
> 但 DKMS 没跟着重建。后者：`sudo apt install --reinstall nvidia-dkms-550`。

### 3.3 双显卡：AMD 610M 核显 + RTX 4080 独显

Ubuntu 默认走 **核显（省电）**，这时候跑 MuJoCo / RViz2 会又慢又花屏。
用 PRIME 切换：

```bash
# 查看当前模式
prime-select query

# 插电做仿真时：全走独显（最省心，功耗高）
sudo prime-select nvidia
# 想省电：按需切换（默认走核显，需要用独显的程序手动指定）
# sudo prime-select on-demand

sudo reboot
```

**`on-demand` 模式下想让某个程序走独显**，命令前加环境变量：

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia <你的命令>
```

> **仿真建议直接用 `prime-select nvidia`**：少一层折腾，
> 反正你插着电跑仿真，散热是 G733PZ 的强项。

### 3.4 ASUS ROG 专有工具（可选，但推荐）

ROG 笔记本的**风扇曲线、性能模式（静音/平衡/涡轮）、键盘灯、GPU MUX 切换**
在 Ubuntu 下默认不可控。社区项目 `asusctl` + `supergfxctl` 补上：

```bash
sudo add-apt-repository -y ppa:asus-linux/asusctl
sudo apt update
sudo apt install -y asusctl supergfxctl

# 看风扇/温度
asusctl fan-curve -m Quiet
# GPU 模式
supergfxctl -g
```

> **注意**：`supergfxctl` 的 GPU 切换要**注销/重启**才生效，且和
> `prime-select` 有职责重叠 —— **别两个都用来切 GPU**，容易打架。
> 你只用 MuJoCo/RViz2 的话，**装 `asusctl` 管风扇就够了，GPU 交给 `prime-select`**。

---

## 4. 第 3 步：ROS 2 Humble 补全（含你要的 RViz2）

### 4.1 先查你现在的 ROS2 装的是哪个版本

```bash
source /opt/ros/humble/setup.bash
ros2 --version 2>/dev/null || echo "(ros2 命令不在)"
which rviz2 && echo "RViz2 已有" || echo "RViz2 缺失 → 需要补装"
```

ROS2 的安装变体决定了有没有 GUI 工具：

| 变体 | 含 RViz2？ | 说明 |
|---|---|---|
| `ros-humble-ros-base` | ❌ | 只有通信和基础库，**没有 rviz2/rqt** |
| `ros-humble-desktop` | ✅ | **推荐**，含 RViz2、rqt、demo 节点 |
| `ros-humble-desktop-full` | ✅ | 再加仿真（Gazebo 等），体积大 |

> **这是最常见的"我明明装了 ROS2 却没有 rviz2"的原因。**
> 如果你当时装的是 `ros-base`，下面这条会帮你补上。

### 4.2 补装 desktop（含 RViz2）与常用工具

```bash
sudo apt update

# 核心：desktop 变体（含 RViz2）
sudo apt install -y ros-humble-desktop

# 你"以后说不定要用到"的常用工具，一并装上
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rqt ros-humble-rqt-graph ros-humble-rqt-common-plugins \
  ros-humble-plotjuggler-ros \
  ros-humble-xacro \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools \
  ros-humble-rosbag2 \
  ros-humble-usb-cam \
  python3-colcon-common-extensions python3-argcomplete

# 让 ros2 / colcon 命令能自动补全
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

> 以上包名**已逐个在清华 ROS2 索引里核对过存在**（索引内共 7858 个包）。

### 4.3 RViz2 在这台双显卡机器上的正确打开方式

RViz2 基于 OGRE，**在双显卡机器上默认可能跑在核显上**，表现为
"打开就卡死""窗口全黑""一拖就闪退"。

```bash
# 方式 A：已经把 prime-select 设成 nvidia —— 直接开就行
rviz2

# 方式 B：prime-select on-demand —— 必须显式指定独显
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia rviz2
```

**验证 RViz2 真的能用**（不接机器人、不看任何设备）：

```bash
# 开一个终端
rviz2
# 另一个终端发一个可视化话题给它
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

RViz2 里 `Add` → `TF`，能看到 `map → base_link` 的坐标系 = RViz2 正常。

### 4.4 ROS2 环境变量（整机联调前必须处理）

```bash
# 看当前 Domain ID
echo $ROS_DOMAIN_ID

# 要把 ROS2 命令写到 .bashrc 里，但【先别急着固定 Domain ID】
# 因为机器人上的值和你的一致才能通信，见 docs/11
```

> **`ROS_DOMAIN_ID` 是整机联调第一大坑**：两边不一致 → 互相看不见对方的话题，
> 现象像"网络不通"，其实是通信域隔离。规则（官方）：手柄启动 = `机器人序号 + 30`；
> APP 启动 = `22`。详情见 `docs/11-真机与仿真指令速查.md`。

---

## 5. 第 4 步：MuJoCo —— 两条路线，按需选

先说结论：

| | 路线 A：官方 BXI 仿真环境 | 路线 B：pip 独立 MuJoCo |
|---|---|---|
| 是什么 | 半醒基于 ROS2 + MuJoCo 的**整机**仿真 | 通用 MuJoCo Python 包 + 官方机器人模型 |
| 能做什么 | 跑官方控制器策略、全身控制、和真机无缝切换 | 自己写 RL / 测试算法 / 加载 MJCF 模型 |
| 依赖 | `bxi_ros2_pkg`（95 MB）+ 控制器（63 MB） | `pip install mujoco` |
| 难度 | ⭐⭐⭐ | ⭐ |
| 建议 | **要做整机控制和部署，走 A** | **想快速上手、自己玩模型，走 B** |

**两个都装不冲突。** 下面分别写。

### 5.1 路线 A：官方 BXI MuJoCo 仿真环境（整机）

> 依据：官方 README（`bxirobotics/bxi_controller_ros2`）与
> `BXI_Wiki/docs/elf3/developer/overview.zh.md`。**以下步骤照抄官方。**

架构上，MuJoCo 仿真器本身**就在官方二进制包 `bxi_ros2_pkg` 里**
（`share/mujoco/`），不需要你自己编译 MuJoCo。

**① 把 `bxi_ros2_pkg` 放到 `/opt/bxi/`**

```bash
sudo mkdir -p /opt/bxi
sudo chown -R "$USER":"$USER" /opt/bxi      # 免得以后每次都 sudo

# 方式 1：从机器人拷（最快，见 §1.1）
scp -r bxi@elf3-82.local:/opt/bxi/bxi_ros2_pkg /opt/bxi/

# 方式 2：git clone（95 MB 仓库，加 --depth 1 省一半）
cd /opt/bxi
git clone --depth 1 https://ghfast.top/https://github.com/bxirobotics/bxi_ros2_pkg.git

# 激活
source /opt/bxi/bxi_ros2_pkg/setup.bash
```

**② 装 MuJoCo 运行依赖**（官方明确要求）

```bash
sudo apt install -y libglfw3-dev libglew-dev libosmesa6-dev
```

**③ 拿控制器示例（63 MB 预编译包，比 clone 425 MB 快得多）**

```bash
# 方式 1：如果你在 Windows 侧已经下好了（见 §1.2），直接解压
mkdir -p ~/bxi_ws && cd ~/bxi_ws
tar -xzf "/mnt/win/bxi_backup/ctrl_v260920.tar.gz"

# 方式 2：在 Ubuntu 里直接下（走加速前缀）
U="https://github.com/bxirobotics/bxi_controller_ros2/releases/download/v260920/bxi_rl_controller_ros2_example_v260920_amd64.tar.gz"
curl -L --retry 5 -C - -o ctrl.tar.gz "https://ghfast.top/$U"
tar -xzf ctrl.tar.gz
```

**④ 编译**

```bash
cd ~/bxi_ws/bxi_rl_controller_ros2_example

source /opt/bxi/bxi_ros2_pkg/setup.bash
source /opt/ros/humble/setup.bash

bash build.sh                      # 编译 ./src 下全部源码
source ./install/setup.bash
```

**⑤ 跑仿真！**

```bash
# 终端 1：启动仿真环境 + 学习控制策略
ros2 launch bxi_example_py_elf3 example_launch_demo.py

# 终端 2：启动键盘控制节点
ros2 launch remote_controller remote_controller_keyboard.launch.py
```

**官方给的四个必须知道的约束**：

1. **仿真话题带 `simulation/` 前缀，真机话题带 `hardware/` 前缀** ——
   同一套控制程序，靠不同 launch 文件切换。**这是仿真和真机的唯一区别。**
2. **仿真里机器人启动时带"虚拟悬挂"，启动后需要释放悬挂**（否则它吊在空中不动）。
3. 仿真有全局里程计话题 `odm`，真机没有。
4. **失控保护：控制命令中断超过 100 ms → 电机失能**，必须重新初始化。
   真机同理，这是安全机制不是 bug。

> ⚠️ **所有带 `hw` 后缀的 launch 文件会启动真实硬件。**
> `example_launch_demo.py` = 仿真；`example_launch_demo_hw.py` / `example_demo_hw.launch.py` = **真机**。
> **手已经装在机器人上时，敲错文件名后果很直接。** 敲之前看一眼有没有 `hw`。

### 5.2 路线 B：独立 MuJoCo（pip，最快见效）

**版本核实**：PyPI 上 MuJoCo 最新为 **3.2.2**，并提供
`cp310-manylinux_x86_64` 轮子 —— 正好对应 Ubuntu 22.04 的 Python 3.10。

```bash
# 系统依赖
sudo apt install -y libglfw3-dev libglew-dev libosmesa6-dev

# 装 MuJoCo（pip 已换清华源，会很快）
pip install mujoco

# 验证
python3 -c "import mujoco; print('MuJoCo', mujoco.__version__)"
```

**看一个官方模型动起来**（用半醒开源的人形模型，规模合适）：

```bash
# 官方模型仓库（26 MB，含 elf2 的 URDF + MJCF + mesh）
mkdir -p ~/mujoco_models && cd ~/mujoco_models
git clone --depth 1 https://ghfast.top/https://github.com/bxirobotics/robot_descriptions.git

# 用 MuJoCo 自带查看器打开场景
python3 -m mujoco.viewer --mjcf=bxirobot_descriptions/elf2_dof25/xml/scene.xml
```

> 官方 `robot_descriptions/elf2_dof25/xml/README.md` 写明：
> **需要 MuJoCo 2.3.3 或以上**，模型是 Apache-2.0 许可。
> （`elf2` 是上一代；`elf3` 的 URDF 在 `bxi_ros2_pkg` 的描述包里，
> MJCF 可参考官方推荐的 [unofficial models](https://github.com/MelodyAI/TienKung-Lab-bxi/tree/main/legged_lab/assets/elf3_lite)。）

**写个最小仿真脚本确认渲染正常**（存成 `~/mujoco_test.py`）：

```python
import mujoco, numpy as np

m = mujoco.MjModel.from_xml_path(
    "/home/<你的用户名>/mujoco_models/robot_descriptions/elf2_dof25/xml/scene.xml")
d = mujoco.MjData(m)

print("nq =", m.nq, " nu =", m.nu, " njnt =", m.njnt)

# 空跑 1000 步，确认物理引擎正常
for _ in range(1000):
    mujoco.mj_step(m, d)
print("位置范围:", d.qpos.min(), "~", d.qpos.max())

# 开交互查看器（需要图形界面）
mujoco.viewer.launch(m, d)
```

```bash
python3 ~/mujoco_test.py
```

### 5.3 渲染走独显（MuJoCo 黑屏 / 极卡的解药）

MuJoCo 的查看器也是基于 OpenGL 的，同样受双显卡影响：

```bash
# 强制走 RTX 4080
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia python3 ~/mujoco_test.py

# 验证到底用的哪块卡
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia glxinfo | grep -i "OpenGL renderer"
# 期望看到 NVIDIA GeForce RTX 4080 Laptop GPU
```

**无头渲染**（跑在 SSH 里 / 没有显示器 / 批处理出图）：

```bash
# 用 EGL 后端（不需要 X11）
export MUJOCO_GL=egl
python3 ~/mujoco_test.py
```

```bash
# 或者用离屏渲染（OSMesa，配合上面装的 libosmesa6-dev）
export MUJOCO_GL=osmesa
```

> 三个后端按需切：有显示器用默认 `glfw`，SSH/无头用 `egl`，
> `egl` 报错就退到 `osmesa`。**遇到 `GLFW error` / 黑屏，第一个动作就是切后端。**

### 5.4 也可以直接用 apt 的 MuJoCo ROS2 包（替代方案）

如果你的目标是"用 ROS2 的 controller 驱动 MuJoCo"，除了官方那套，
还可以装社区维护的：

```bash
sudo apt install -y ros-humble-mujoco-ros2-control ros-humble-mujoco-ros2-control-demos
```

> 这是 `ros2_control` 生态的 MuJoCo 仿真硬件接口，**和官方 BXI 那套是两回事**。
> 官方那套更贴合本项目（它带 elf3 的描述文件和策略），
> 这个包适合你自己搭 `ros2_control` 链路时用。**先别装，需要时再说。**

---

## 6. 第 5 步：把灵巧手接进这套环境

先说清楚一件事，**避免白折腾**：

| 场景 | 用什么 | 说明 |
|---|---|---|
| 桌面单独调手 | **Windows + 485 转接盒** | 见 `docs/08`，与本机 Ubuntu 无关 |
| 手上装到机器人身上 | **Ubuntu 整机 ROS2** | 手走 CANFD，见 `docs/09`、`docs/11` |
| 在仿真里**只跑手** | MuJoCo + 手的 MJCF 模型 | 官方已提供 `brainco-description`（含 Revo2 MJCF），但**缺 `<actuator>` 需自己补**。详见 `docs/14` |
| 在仿真里跑**整机+手** | 官方 BXI 仿真（§5.1） | 官方 elf3 描述文件里手腕是 7 自由度关节，手本体不在其中 |

**关键点**：仿真里的 elf3 是 **29/31 自由度本体**（腰颈 5 + 腿 12 + 臂 14），
**不含灵巧手的 6 个手指自由度**。所以：

- **想做"整机 + 手"的联合仿真** → 需要基于官方 elf3 描述文件，
  在手腕末端（`l_wrist_z_joint` / `r_wrist_z_joint`）挂一个手的 MJCF，
  并把手的指关节加进 `nu`。**这是你后面要自己做的活**，不是现成的。
  > **补充（2026-09-22 核实）**：手部 MJCF 官方已提供 ——
  > 仓库 `github.com/BrainCoTech/brainco-description`，
  > 文件 `revo2_system/mjcf/revo2_left.xml`（11 个 hinge 关节，含左右手）。
  > 但官方未提供 `<actuator>` 段（`nu = 0`，官方 README 自述该项仍在 WIP），
  > 故"将手指关节纳入 nu"一步**仍需自行实现**。
  > 完整拼接方案见 `docs/14-模型拼接与复合动作编排.md`。
- **本仓库已有的 `src/elf3_ros2/revo2_bridge_node.py` 是给真机用的**
  （它直接占主控板的 CAN 链路），**在仿真里跑不起来**，别搞混。

关节索引与 CAN 总线对照（官方，仿真和真机共用同一套顺序）：

| 部位 | CAN 总线 | 关节索引 |
|---|---|---|
| 腰 / 颈 | CANFD_0 | 0–2, 29–30 |
| 左腿 | CANFD_1 | 3–8 |
| 右腿 | CANFD_2 | 9–14 |
| 左臂 | CANFD_3 | 15–21 |
| 右臂 | CANFD_4 | 22–28 |
| **左右灵巧手** | **CAN5 / CAN6** | 不在经典 31 关节内，单独两路 |

> 灵巧手走 **CAN5（左手, ID 126）/ CAN6（右手, ID 127）**，
> **不属于本体那 5 条总线（bus 0–4）**，互不冲突。详见 `docs/01` §5.2。

---

## 7. 验证清单（做完逐项打勾）

```bash
# 一键跑完下面全部检查
bash tools/ubuntu_bootstrap.sh --stage verify
```

| # | 检查项 | 命令 | 期望结果 |
|---|---|---|---|
| 1 | 内核版本 | `uname -r` | ≥ 6.2（最好 6.8.x） |
| 2 | 有线网卡 | `ip -br link` | 有 `enp*` 且 `UP` |
| 3 | 无线网卡 | `nmcli device` | `wifi` 已连接 |
| 4 | NVIDIA 驱动 | `nvidia-smi` | 看到 RTX 4080 Laptop |
| 5 | 当前显卡模式 | `prime-select query` | `nvidia`（做仿真时） |
| 6 | OpenGL 渲染器 | `glxinfo \| grep renderer` | NVIDIA（不是 AMD） |
| 7 | ROS2 | `ros2 --version` | 有输出 |
| 8 | **RViz2** | `rviz2` | 窗口正常打开不卡 |
| 9 | MuJoCo | `python3 -c "import mujoco;print(mujoco.__version__)"` | 3.2.x |
| 10 | MuJoCo 渲染 | `python3 ~/mujoco_test.py` | 无 GLFW 报错 |
| 11 | 官方仿真环境 | `ls /opt/bxi/bxi_ros2_pkg/share/mujoco` | 目录存在 |
| 12 | 官方控制器 | `ls ~/bxi_ws/*/install/setup.bash` | 文件存在 |
| 13 | 仿真能起来 | `ros2 launch bxi_example_py_elf3 example_launch_demo.py` | 出现 MuJoCo 窗口 |

---

## 8. 故障速查

### 8.1 网络类

| 症状 | 原因 | 解决 |
|---|---|---|
| `apt update` 卡在 `Waiting for headers` | 在用慢源 | 换清华源（§2.1） |
| `pip install` 下到一半超时 | pip 默认超时太短 | `~/.pip/pip.conf` 里调 `timeout=60 retries=5`（§2.3） |
| GitHub 下载 0 字节 / 卡死 | release 资产被墙 | 加 `ghfast.top/` 前缀；或从机器人拷 |
| `git clone` 中断 `invalid index-pack output` | 仓库大 + 网络抖 | `git clone --depth 1`，或 `curl -C -` 断点续传 |
| `NO_PUBKEY` | 密钥缺失 | 用清华 keyring，别用 keyserver |

### 8.2 显卡类

| 症状 | 原因 | 解决 |
|---|---|---|
| `nvidia-smi` 报 `couldn't communicate` | Secure Boot 没注册 MOK / DKMS 没重建 | §3.2 的 MOK 步骤；或 `apt install --reinstall nvidia-dkms-550` |
| 外接显示器没信号 | 画面走核显 | `prime-select nvidia`，或线插到独显直连的口 |
| MuJoCo / RViz2 黑屏、极卡 | 跑在核显上 | 加 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` |
| `GLFW error` | 无头环境 | `export MUJOCO_GL=egl`（不行换 `osmesa`） |
| 点应用图标启动的程序还是走核显 | 环境变量没传进去 | 改 `.desktop` 文件，或从终端启动 |

### 8.3 Wi-Fi 类

| 症状 | 原因 | 解决 |
|---|---|---|
| 完全没有 Wi-Fi 选项 | 内核太老，MT7922 不认 | 升 HWE 内核（§3.1） |
| Wi-Fi 时断时续 | MT7922 电源管理 | `sudo iwconfig wlan0 power off`，写进 rc.local |
| `mt7921e` 报 firmware 错误 | 固件缺失 | `sudo apt install linux-firmware` 后重启 |

### 8.4 ROS2 / 仿真类

| 症状 | 原因 | 解决 |
|---|---|---|
| `ros2: command not found` | 没 source | `source /opt/ros/humble/setup.bash`，写进 `.bashrc`（§4.2） |
| 装过 ROS2 但没 `rviz2` | 装的是 `ros-base` | `sudo apt install ros-humble-desktop`（§4.2） |
| `ros2 launch` 只看到 `hw` 的文件 | 看错了 launch | 仿真用**不带 `hw`** 的那个（§5.1 ④） |
| 仿真里机器人吊着不动 | 虚拟悬挂没释放 | 官方说明：启动后需释放悬挂 |
| 能 ping 通机器人但看不到话题 | `ROS_DOMAIN_ID` 不一致 | 两边设成相同值（§4.4） |
| 仿真能跑，真机不行 | 用错 launch / 不是 root | 真机必须 `root`，见 `docs/11` |

---

## 9. 附录

### 9.1 换源一键回滚

```bash
# 恢复原始 apt 源
sudo cp /etc/apt/sources.list.bak.* /etc/apt/sources.list
sudo apt update
```

### 9.2 官方资源地址速查

| 资源 | 地址 | 体积 |
|---|---|---|
| 官方 Wiki（**遇到分歧以它为准**） | `github.com/bxirobotics/BXI_Wiki` | — |
| 灵巧手开发文档（官方） | `BXI_Wiki/docs/elf3/developer/dexterous_hand.zh.md` | — |
| 控制器示例仓库 | `github.com/bxirobotics/bxi_controller_ros2` | 425 MB（clone） |
| **控制器预编译包** | Releases `v260920` → `...amd64.tar.gz` | **63 MB** |
| 仿真二进制包 | `github.com/bxirobotics/bxi_ros2_pkg` | 95 MB |
| 官方机器人模型 | `github.com/bxirobotics/robot_descriptions` | 26 MB |
| 灵巧手官方示例 | `github.com/konodoki/bxi_revo2_example` | 64 KB |
| 强脑 Python SDK | `pip install bc-stark-sdk==1.5.1` | 有 Windows 轮子 |

### 9.3 本文档的数据来源与可信度

| 结论 | 来源 |
|---|---|
| 清华/中科大/华为/腾讯源可用、**阿里云不可用** | 本机实测（2026-09-20） |
| GitHub release 资产直连不可达 | 本机实测（请求直接超时） |
| `nvidia-driver-535/545/550/570/580/590` 在 jammy-updates | 本机抓 `Packages.gz` 核对 |
| `linux-generic-hwe-22.04` 在 main-updates | 本机抓 `Packages.gz` 核对 |
| MuJoCo 最新 3.2.2、有 cp310 manylinux 轮子 | 本机查 PyPI 核对 |
| `ros-humble-rviz2` 等 20+ 个包名存在 | 本机抓清华 ROS2 索引核对（7858 包） |
| 仿真启动命令、`simulation/` 前缀、100 ms 失控保护 | 官方 README |
| 31 关节 CAN 分配、`ROS_DOMAIN_ID` 规则 | 官方 Wiki `overview.zh.md` |
| MuJoCo 需要 `libglfw3-dev` | 官方 README |
| elf2 MJCF 需要 MuJoCo ≥ 2.3.3 | 官方 `robot_descriptions` README |

> **本文档里的 Ubuntu 操作我无法在本机执行验证**（本机是 Windows），
> 所以每条命令都标注了来源。**跑之前先 `--dry-run`，跑之中注意报错，跑之后过一遍 §7 清单。**

---

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-09-20 | 首版：按 G733PZ 实际硬件（7945HX / RTX 4080 Laptop / MT7922）写可照抄的七步流程；换源实测表；HWE 内核与 MOK；双显卡 PRIME；ROS2 desktop 补装含 RViz2；MuJoCo 两条路线（官方 BXI 仿真 / pip 独立）；双显卡渲染与三种 GL 后端；仿真与真机边界说明；验证清单与四类故障速查 |

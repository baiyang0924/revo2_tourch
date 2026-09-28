# ELF3 双系统装机完整方案 + 真机/仿真指令记录

> **📦 这是一份归档的参考资料，不是本目录的正式流程文档。**
>
> | 项目 | 说明 |
> |---|---|
> | 来源 | 同事整理并分享的方案文件，**正文逐字保留，未做改写** |
> | 归档日期 | 2026-09-20 |
> | 适用硬件 | **i9-14900K + RTX 4090 + 技嘉 Z790 台式机**（不是笔记本） |
> | 已做处理 | ① SSH 明文口令已脱敏；② 文首增加本说明 |
>
> ---
>
> ## 读这份文件时的 4 个提醒
>
> 1. **第 2–10 章（BIOS / `nomodeset` / 4090 驱动）是给那台 4090 台式机写的。**
>    笔记本没有独立 4090，**不要照抄 `nomodeset` 那一套** ——
>    它解决的是 40 系显卡与开源驱动 `nouveau` 的兼容问题，笔记本上不存在这个问题。
> 2. **第 16 章「下载官方仓库」已有更省事的替代方案。**
>    原文的 `git clone` 要拉 **425 MB**，官方在 Releases 同时提供 **63 MB 预编译包**；
>    还国内加速前缀与"局域网直拷"两条路线。完整方案见
>    [`../10-官方资源获取与国内网络下载方案.md`](../10-官方资源获取与国内网络下载方案.md)。
> 3. **`JRT` 段落里的 launch 文件名有两种写法**：
>    这里记的是 `example_demo_hw.launch.py`，官方 README 写的是 `example_launch_demo_hw.py`。
>    **以机器人上 `ls src/bxi_example_py_elf3/launch/` 的实际结果为准。**
>    这些真机/仿真指令已按官方 README 与 Wiki 校对、并结构化整理到
>    [`../11-真机与仿真指令速查.md`](../11-真机与仿真指令速查.md)，
>    其中动作指令还配了工具 `tools/elf3_cmd.sh`（不用再手抄那 20 个字段的 JSON）。
> 4. **本文末尾「真机」段的 SSH 口令已脱敏**，需要时请向主管或负责同事索取。
>
> ---

---

# ELF 3 · 你的电脑（i9-14900K + RTX 4090 + 技嘉 Z790）双系统装机完整方案

> 本文件是完整版，包含：你的 3 个问题的完整回答 + 豆包方案逐条评价 +
> 从 Windows 备份到跑起 ELF3 官方仿真的**全部步骤**，一步不省。
> 你的硬件（已从截图确认）：
> - CPU：Intel i9-14900K（24 核，带 UHD 770 核显）
> - 显卡：NVIDIA GeForce RTX 4090 24GB（微星）
> - 主板：技嘉 Z790 EAGLE AX
> - 内存：64GB DDR5 4800
> - 硬盘：WD_BLACK SN770 2TB
> - 系统：Win11 Pro 23H2
> - 你已完成：Secure Boot 已关闭；BitLocker/设备加密未开启（无需处理恢复密钥）

---

# 第一部分：你的三个问题，完整回答

## 问题 1：Win11 里 NVIDIA App 更新显卡驱动失败，需要先修好吗？

**结论：不需要。装 Ubuntu 完全不受影响，直接忽略。**

原因：双系统的两个系统驱动互相独立、互不干扰——

- Windows 用 Windows 的驱动（NVIDIA App 管理的是这个）
- Ubuntu 装系统时会自己装 Linux 版驱动（后面第 7 章有完整步骤）
- Win11 里驱动新旧、装没装好，对 Ubuntu 一点影响都没有

NVIDIA App 更新失败的常见原因是它的下载服务器在国内不稳定，这是 Windows 侧的小毛病，不影响你接下来的任何操作。

**如果想顺手修（可选，跟装 Ubuntu 无关）**：
1. 浏览器打开 nvidia.cn
2. 驱动程序 → GeForce 驱动程序
3. 产品选择：GeForce RTX 40 系列 / Windows 11 / 64-bit / DCH
4. 下载后安装，选「自定义」→ 勾「执行清洁安装」

不修也完全没关系。

## 问题 2：Secure Boot 已关，但 BitLocker 没开——有什么要注意的？

**结论：你这个情况是最省心的组合，没有任何额外风险，继续往下走即可。**

BitLocker 的风险点在于：开了 BitLocker 的机器关掉 Secure Boot 后，Windows 可能要求输入恢复密钥才能启动。你已经确认没开 BitLocker/设备加密，所以：

- 关 Secure Boot 不会影响你的 Windows 正常启动
- 不需要导出任何恢复密钥
- 之前通用手册里的「BitLocker 检查」步骤对你直接跳过

## 问题 3：豆包给的方案怎么样？有更好的吗？

**结论：豆包方案整体靠谱（评分 8/10），核心步骤全部正确，可以直接照做；
但有 2 个重要遗漏 + 1 个小修正，本文件第三部分已全部补上并整合成最终流程。**

### 豆包方案逐条评价表

| 豆包的建议 | 评价 | 说明 |
|-----------|------|------|
| Z790 主板 BIOS 里关 VMD Controller | ✅ 正确 | 14 代酷睿平台 VMD 默认开启，不关的话 Ubuntu 安装器看不到你的 SN770 固态，硬盘列表空白 |
| RTX 4090 黑屏 → 安装时加 nomodeset | ✅ 正确 | 40 系显卡开源驱动 nouveau 兼容性差，这是标准解法 |
| 台式机 14900K 不需要加 acpi=off | ✅ 正确 | 加了反而可能出问题，只加 nomodeset 就够 |
| 装完系统首次从硬盘启动还要再加一次 nomodeset | ✅ 正确，且很关键 | 很多人栽在这一步，以为装好了结果第一次开机黑屏 |
| 装 nvidia-driver-550-open | ✅ 方向正确 | 40 系显卡确实优先用 open 内核模块；但**不要写死版本号**（见下方修正） |

### 豆包的 2 个重要遗漏（必须补上，否则可能翻车）

**遗漏 ①：没有提醒你检查显示器视频线插在哪个显卡上。**

你的 i9-14900K 带核显（UHD Graphics 770），你的截图显示核显处于启用状态，
说明主板视频口也能亮屏。**必须确认：视频线（HDMI/DP）插在 RTX 4090 显卡
的接口上，而不是插在主板的视频接口上。**

- 插错了的后果：Ubuntu 装好 4090 驱动后，系统只往 4090 输出画面，
  你的屏幕走的是核显，**必然黑屏**，而且会让你误以为驱动装坏了。

**检查方法**：看主机背面，视频线接的是**下方独立大显卡（4090）的横排接口**
（微星 4090 一般是 1 个 HDMI + 3 个 DP）就是对的；接在**上方主板自带的
竖排接口**就是错的，拔下来插到 4090 上。

**遗漏 ②：没有提醒关闭 Windows 快速启动。**

不关的后果：Ubuntu 挂载不了 Windows 分区（访问不了 Windows 里的文件）、
双系统切换后时间错乱 8 小时。

**关闭方法**：Windows 控制面板 → 电源选项 → 选择电源按钮的功能 →
「更改当前不可用的设置」→ 取消勾选「启用快速启动」→ 保存修改。
（Win11 找不到控制面板的话：右键开始按钮 → 设置 → 系统 → 电源和电池 →
屏幕和睡眠 → 里面找不到就直接搜索「控制面板」打开，按上面路径走。）

### 豆包的 1 个小修正

豆包直接让你装 `nvidia-driver-550-open`，版本号写死了。更稳的做法是先看
系统推荐哪个版本再装（Ubuntu 22.04.5 的推荐版本未来可能升级为 560/570 等）：

```bash
ubuntu-drivers devices
```

看输出里带 `(recommended)` 字样的那一行（例如 `nvidia-driver-550-open`），
然后装它。装带 `-open` 后缀的版本（40 系显卡优先用开源内核模块）。

---

# 第二部分：装机前准备（Windows 里完成）

## 1. 备份重要数据（必做，双系统没有后悔药）

把 Windows 盘里的重要个人文件（文档、照片、项目）备份到移动硬盘或网盘。
分区操作正常情况下很安全，但备份是最后一道保险。

## 2. 关闭 Windows 快速启动（豆包遗漏 ②，必做）

见第一部分问题 3 的操作步骤。

## 3. 检查视频线插在 RTX 4090 上（豆包遗漏 ①，必做）

见第一部分问题 3 的检查方法。

## 4. 在 Windows 里腾出磁盘空间

1. 右键「此电脑」→「管理」→「磁盘管理」
2. 选中空间最富裕的盘 → 右键 →「压缩卷」
3. 输入压缩大小（MB）：建议 **153600（150GB）**，最少 102400（100GB）
   （你 2TB 的盘，给 Ubuntu 200GB 也不心疼）
4. 点「压缩」，完成后会看到一块「未分配」黑色区域——**保持原样，什么都不要做**

## 5. 下载 Ubuntu 22.04.5 镜像（清华源）

浏览器打开：

```
https://mirrors.tuna.tsinghua.edu.cn/ubuntu-releases/22.04/
```

下载 **`ubuntu-22.04.5-desktop-amd64.iso`**（约 5GB）。

必须是 22.04：ROS 2 Humble 官方只支持这个版本，且与你们团队/实机环境一致。
**不要下 24.04。**

## 6. 制作启动 U 盘（≥ 8GB，制作过程会清空它）

1. 下载 Rufus：<https://rufus.ie/zh/>（选 rufus-4.x.exe，免安装）
2. 插入 U 盘，打开 Rufus：
   - 设备：你的 U 盘
   - 引导类型选择：点「选择」→ 选刚下载的 iso
   - 其他全部默认
3. 点「开始」→ 选「以 ISO 镜像模式写入（推荐）」→ 等待完成

---

# 第三部分：BIOS 设置 + U 盘安装 Ubuntu（含 4090 黑屏处理）

## 7. BIOS 设置（技嘉 Z790 EAGLE AX）

1. **完全关机**（按住 Shift 点关机，不要休眠）
2. 插好 U 盘，开机狂按 **Del** 键进 BIOS（技嘉是 Del）
3. 需要改/确认的项目：

| 项目 | 位置（技嘉 BIOS 大致位置） | 设置 | 状态 |
|------|--------------------------|------|------|
| Secure Boot | Boot 或 Security 页 | Disabled | ✅ 你已完成 |
| VMD Controller | Settings → IO Ports（或 Storage 相关页） | Disabled | ⬜ 待确认 |
| 启动模式 | Boot 页 | UEFI | 一般默认就是 |

**关于 VMD 的说明**：14 代酷睿平台 VMD 默认开启，不关的话 Ubuntu 安装器
看不到 SN770 硬盘。技嘉部分主板出厂就是关的——**如果 BIOS 里翻遍了都找不到
这个选项，不用慌**：直接继续装，只要安装器里能看到你的 2TB 硬盘就没问题。

4. 按 **F10** 保存退出

## 8. U 盘启动 + nomodeset（4090 黑屏的关键处理）

1. 开机狂按 **F12**（技嘉启动菜单键），选 **UEFI: 你的U盘名**，回车
2. 出现紫色 GRUB 菜单，光标停在 `Try or Install Ubuntu`——**不要回车**
3. 按 **`e`** 键进入编辑，用方向键找到这一行：

```
linux ...... quiet splash $vt_handoff
```

4. 在 `quiet splash` 后面**空一格**，添加 `nomodeset`，改完是这样：

```
quiet splash nomodeset $vt_handoff
```

> 只加 nomodeset，**不要**加 acpi=off（台式机 14900K 不需要，加了反而出问题）。

5. 按 **F10** 启动
6. 进入图形界面（等一两分钟），开始安装

## 9. 安装 Ubuntu 22.04（图形界面步骤）

1. 语言拉到最底选 **中文（简体）** → 「安装 Ubuntu」
2. 键盘布局：Chinese → 继续
3. 选「正常安装」+ 勾选「为图形或无线硬件……安装第三方软件」→ 继续
4. **安装类型（关键）**：
   - 出现「安装 Ubuntu，与 Windows Boot Manager 共存」→ **选它**，全自动最省心
   - 没出现就选「其他选项」手动分区，在未分配空间上：
     - 新建分区 1：大小 **8192 MB**，逻辑分区，**交换空间 swap**（你 64GB 内存，8GB swap 足够）
     - 新建分区 2：**剩余全部空间**，主分区，**Ext4 日志文件系统**，挂载点 **/**（一个斜杠）
     - 「安装启动引导器的设备」保持默认（选中有 Windows 的那块盘）
     - **千万不要动带 Windows / ntfs 字样的分区**
5. 时区 Shanghai → 设置用户名和密码（密码别忘，天天要 sudo）
6. 「立即安装」→ 确认 → 等约 10 分钟 → 「现在重启」→ **拔掉 U 盘**

## 10. 首次从硬盘启动——还要再加一次 nomodeset（重要！）

装好后第一次开机进 GRUB 菜单（Ubuntu / Windows Boot Manager 的选择界面）：

1. 光标停在 **Ubuntu**——**不要回车**
2. 按 **`e`**，和第 8 章一样在 `quiet splash` 后加 `nomodeset`，按 **F10** 启动
3. 成功进入桌面

> 为什么：此时系统里还没有 4090 的官方驱动，只能靠 nomodeset 用基础显示模式
> 凑合进桌面。下一步装好驱动后就再也不需要 nomodeset 了。

---

# 第四部分：Ubuntu 系统初始化 + 装 4090 驱动

## 11. 进系统后的第一件事：联网

右下角网络图标连 WiFi（你的 Intel AX211 网卡 Ubuntu 自带支持，开箱即用）。
台式机插网线更稳，二选一。

## 12. 换国内软件源（下载快几倍）

打开终端（`Ctrl + Alt + T`）：

```bash
sudo sed -i 's@//.*archive.ubuntu.com@//mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list
sudo apt update && sudo apt upgrade -y
```

> 输密码时屏幕不显示任何字符是正常的，输完直接回车。

## 13. 安装 RTX 4090 驱动（nomodeset 的终结者）

```bash
sudo apt update
ubuntu-drivers devices
```

看输出里带 **`(recommended)`** 的那一行，例如 `nvidia-driver-550-open`，
然后安装（把下面命令里的版本换成你看到的推荐版本，**优先选带 `-open`
后缀的**，40 系显卡用开源内核模块稳定性更好）：

```bash
sudo apt install -y nvidia-driver-550-open
sudo reboot
```

重启时 GRUB 菜单**直接回车进 Ubuntu，不再加 nomodeset**。

验证（终端执行）：

```bash
nvidia-smi
```

**看到一张表：NVIDIA GeForce RTX 4090、24564MiB 显存、驱动版本号——成功。**
MuJoCo 的 3D 窗口从此流畅，不需要设任何 `MUJOCO_GL` 环境变量。

> 如果装完驱动重启黑屏：99% 是视频线插在主板（核显）上——回第 2 章检查；
> 其次进 GRUB 按 `e` 加回 nomodeset 进系统，终端跑 `sudo apt install -y nvidia-driver-550`（换不带 -open 的版本）再重启。

## 14. 基础工具

```bash
sudo apt install -y git curl wget build-essential gedit
```

---

# 第五部分：安装 ROS 2 + 官方仿真框架（与你之前的手册一致）

## 15. 安装 ROS 2 Humble（30~60 分钟，看网速）

逐段复制执行。

**1）语言环境：**

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

**2）添加 ROS 2 软件源：**

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> `raw.githubusercontent.com` 连不上的话：开手机热点重试；或把命令里的
> `raw.githubusercontent.com` 换成 `ghproxy.net/https://raw.githubusercontent.com`。

**3）安装桌面版（下载 1GB+，耐心等）：**

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

**4）环境变量自动加载：**

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**5）验证：**

```bash
ros2 --help
```

> ⚠️ 这台机器上**不要装 Miniconda**——官方框架跑在系统 Python 上，不装 conda
> 环境最干净，没有 `(base)` 污染问题。

## 16. 下载官方仓库（2 个）

**仓库 1：控制策略 + 示例框架**（含走路/跑步/跳舞/前空翻等全部预训练 ONNX 模型）：

```bash
mkdir -p ~/elf3_ws && cd ~/elf3_ws
git clone https://github.com/bxirobotics/bxi_rl_controller_ros2_example.git
```

（此网址会自动跳转到官方真名仓库 `bxi_controller_ros2`，正常现象。）

验证策略模型在不在：

```bash
ls ~/elf3_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/mods/com.bxi.basic_actions/assets/
```

应看到 `model_normal.onnx、amp_run.onnx、recover.onnx、forward_flip.onnx` 等。

**仓库 2：官方二进制仿真器包**（MuJoCo 仿真器节点 + 机器人通信包，约 95MB）：

```bash
sudo mkdir -p /opt/bxi
cd /opt/bxi
sudo git clone https://github.com/bxirobotics/bxi_ros2_pkg.git
```

验证：

```bash
ls /opt/bxi/bxi_ros2_pkg/setup.bash
```

## 17. 安装依赖 + 编译

```bash
sudo apt install -y libglfw3-dev

cd ~/elf3_ws/bxi_rl_controller_ros2_example
./deploy_environment.sh        # 官方脚本，自动装 Python 依赖，走清华源

source /opt/bxi/bxi_ros2_pkg/setup.bash
bash build.sh                  # 编译 5~15 分钟
```

> 编译报错可只编译必需的两个包：
> ```bash
> colcon build --merge-install --packages-select bxi_example_py_elf3 remote_controller
> ```

## 18. 一键环境脚本

```bash
cat > ~/elf3_env.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source ~/elf3_ws/bxi_rl_controller_ros2_example/install/setup.bash
echo "ELF3 仿真环境已就绪"
EOF
chmod +x ~/elf3_env.sh
```

---

# 第六部分：启动仿真

## 19. 每次开机 3 条命令

```bash
# 终端 1：仿真器 + 控制策略
source ~/elf3_env.sh
ros2 launch bxi_example_py_elf3 example_launch_demo.py

# 终端 2（新开）：键盘控制，鼠标点住这个窗口保持激活
source ~/elf3_env.sh
ros2 launch remote_controller remote_controller_keyboard.launch.py
```

MuJoCo 窗口弹出后机器人自动复位站好。按 `1` 进入行走模式。

## 20. 键盘速查表

| 按键 | 动作 | 按键 | 动作 |
|------|------|------|------|
| `1` | 站立/行走模式 | `5` | 中速奔跑 2m/s |
| `W/S` | 前进/后退 | `6` | 后空翻（官方未发布，无反应属正常） |
| `A/D` | 左移/右移 | `7` | 前空翻 |
| `Q/E` | 左转/右转 | `8` | 鼓掌 |
| `空格` | 停止 | `9` | 打招呼（挥手） |
| `2` | 起身（摔倒后爬起） | `0` | 深度相机行走 |
| `3` | 跳舞 | `Shift+1` | PD 刹车回初始姿态 |
| `4` | 高速奔跑 4m/s | `Tab` | 切换相机（跟踪/头部第一人称/躯干） |

> ⚠️ 绝对不要运行任何带 `hw` 的 launch 文件——那是驱动真实机器人硬件的。

---

# 第七部分：常见问题速查（含本机专属）

| 症状 | 解决 |
|------|------|
| 装完重启直接进 Windows，没有 GRUB 菜单 | 开机按 F12 手选 ubuntu；或进 BIOS 把 ubuntu 调到启动顺序第一位 |
| GRUB 菜单里 Windows 启动不了 | Ubuntu 里执行 `sudo os-prober` 确认识别，再 `sudo update-grub` |
| Windows 和 Ubuntu 时间差 8 小时 | `timedatectl set-local-rtc 1 --adjust-system-clock` |
| 安装器里看不到 SN770 硬盘 | VMD 没关：回 BIOS 找 `VMD Controller` → Disabled |
| U 盘启动黑屏 | nomodeset 没加对：回第 8 章，`quiet splash` 后面空格再加 |
| 装好驱动重启还是黑屏 | 视频线插在主板核显上了：插到 4090 上（第 2 章） |
| 装完 4090 驱动黑屏但线没问题 | GRUB 按 `e` 加 nomodeset 进系统，改装不带 `-open` 的版本：`sudo apt install -y nvidia-driver-550` |
| `git clone` 连不上 GitHub | 手机热点；或地址前加 `https://ghproxy.net/` 前缀 |
| 启动时刷 `cannot configure control thread scheduling ... SCHED_FIFO` | 只是警告，不影响仿真，忽略；想消除：`sudo prlimit --pid $$ --rtprio=99:99` 后重启终端 |
| `robot reset service not available; waiting` | MuJoCo 窗口还没弹出来，等一下自动继续 |
| 机器人动一半瘫软 | 超时保护触发：按 `2` 起身或空格 + `1`；不行重启两个终端 |
| 键盘按了没反应 | 焦点不在终端 2，鼠标点一下那个窗口 |
| `ros2 launch` 找不到包 | 忘了 `source ~/elf3_env.sh` |

---

# 第八部分：收工清单

```bash
# 一次性（按顺序）
① Windows：备份数据 → 关快速启动 → 检查视频线插在 4090 → 压缩出 150GB 空间
② 下载 ubuntu-22.04.5 iso（清华源）→ Rufus 做 U 盘
③ BIOS：Secure Boot 关（已完成）→ VMD 关 → F10 保存
④ U 盘启动 + nomodeset → 安装 22.04（与 Windows 共存）
⑤ 首次硬盘启动 + 再加一次 nomodeset 进桌面
⑥ 换源 → ubuntu-drivers 装 4090 驱动 → nvidia-smi 验证
⑦ 装 ROS 2 Humble → 克隆 2 个官方仓库 → deploy_environment.sh → build.sh
⑧ 写好 ~/elf3_env.sh

# 每次开机
终端1: source ~/elf3_env.sh && ros2 launch bxi_example_py_elf3 example_launch_demo.py
终端2: source ~/elf3_env.sh && ros2 launch remote_controller remote_controller_keyboard.launch.py
然后按键盘玩：1 行走 / 4 冲刺 / 7 前空翻 / 3 跳舞 / Tab 换视角
```


# ##################### ！仿真！！######################################################
### JRT 启动仿真
'''
# cd ~/elf3_ws/bxi_rl_controller_ros2_example
# rm -rf build/ install/ log/   # 彻底清理
# colcon build --symlink-install
# source install/setup.bash
cat ~/elf3_env.sh
ros2 launch bxi_example_py_elf3 example_launch_demo.py
source ~/elf3_env.sh
ros2 launch remote_controller remote_controller_keyboard.launch.py  #键盘控制
‘‘’  
###

### JRT 将新生成动作代码文件部署到本地仿真工作空间
# 将新生成动作代码文件复制路径到本地方仿真空间（此为路径例子）
python3 apply_presets_to_bxiws.py \
  --src-dir ~/下载 \
  --mod ~/elf3_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/mods/com.bxi.basic_actions \
  --config ~/elf3_ws/bxi_rl_controller_ros2_example/src/remote_controller/config/xbox_default.yaml
# 重新编译并测试 
cd ~/elf3_ws/bxi_rl_controller_ros2_example
colcon build --packages-select bxi_example_py_elf3 remote_controller --symlink-install
source install/setup.bash
###

### JRT 仿真单臂碰拳（btn_10: 9）
source ~/elf3_env.sh
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 0, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 9, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###

### JRT 仿真双臂叉腰（btn_10: 10）
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 0, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 10, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###

### JRT 仿真双臂拥抱（btn_10: 11）
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 0, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 11, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###




# ##################### ！！真机！！######################################################
### JRT 连接机器人
ssh bxi@elf3-82.local   # 或使用 IP
# 密码：〈内部凭据，向主管/负责同事索取，不要提交到仓库〉
sudo -i
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source /home/bxi/bxi_ws/bxi_rl_controller_ros2_example/install/setup.bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py
###

### JRT 将更改的动作代码部署到机器人，此为例子：
# 1. 把本地项目复制到机器人（在本地电脑执行）
scp -r ~/elf3_ws/bxi_rl_controller_ros2_example bxi@elf3-82.local:/home/bxi/bxi_ws/
# 2. SSH 登录机器人，编译
ssh bxi@elf3-82.local
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
bash build.sh 
# 3. 重启服务
sudo systemctl restart ros_elf_launch.service
###


### JRT 启动机器人，启动控制节点
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py
###


### JRT 零位模式
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 0, btn_4: 1, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 0, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###

### JRT 初始姿态
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 1, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 0, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###

### JRT normal 走路模式
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 1, btn_2: 0, btn_3: 0, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 0, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###


### JRT cheer模式
ros2 topic pub /motion_commands communication/msg/MotionCommands "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, vel_des: {x: 0.0, y: 0.0, z: 0.0}, height_des: 0.0, yawdot_des: 0.0, mode: 0, btn_1: 0, btn_2: 0, btn_3: 0, btn_4: 0, btn_5: 0, btn_6: 0, btn_7: 0, btn_8: 0, btn_9: 0, btn_10: 7, axis_1: 0, axis_2: 0, axis_3: 0, axis_4: 0, axis_5: 0, axis_6: 0, axis_7: 0, axis_8: 0, axis_9: 0, axis_10: 0}" --once
###

### JRT 校准头部电机零位漂移
sudo /opt/bxi/bxi_ros2_pkg/lib/hardware_elf3/hw_lib_test_elf3
mit_zero_set_single 29
sudo /opt/bxi/bxi_ros2_pkg/lib/hardware_elf3/hw_lib_test_elf3
mit_zero_set_single 30
###


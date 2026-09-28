# ELF3 × Revo2 灵巧手二次开发

精灵 3（ELF 3）人形机器人 + 强脑 Revo2 触觉版灵巧手的整机联调与二次开发项目。

**项目目标**：实现上位机远程控制机器人，驱动机器人手臂末端的灵巧手完成指定动作；
后续开发机器人本体动作，最终将本体动作与手部动作融合成成套复合动作。

> **侧别约定**：本项目实机装配的是 **右手**（右腕 `r_wrist_z_link`，主控板 CAN6 / ID 127）。
> 文档与脚本的命令示例默认按右手给出（如 `--hand right` / `--sides right`）。
> 左右两侧通路完全对称，切左手只需把 `right` 换成 `left`（CAN5 / ID 126）。

---

## 一、先搞清楚环境边界（最重要的一条）

| 环境 | 能做什么 | 不能做什么 |
|---|---|---|
| **Windows** | 单独调试灵巧手：官方上位机、Python / C SDK、485 / USB-CANFD 接线、写手势动作代码 | ❌ 不能用 ROS 2 联动精灵 3 整机 |
| **Ubuntu 22.04（双系统）** | ROS 2 Humble 全功能：精灵 3 整机联调、机器人端下发指令远程控制灵巧手、动作序列录制回放 | — |

**「Windows 装不了 ROS 2」是错的。** ROS 2 Humble 官方提供 Windows 10 二进制安装包。
真正卡住的是**厂商链路**（机器人机载主控是 Ubuntu、强脑 ROS 驱动只按 Ubuntu 构建验证、
CAN 底层 Linux 走 SocketCAN 而 Windows 只能靠转接器 DLL、EtherCAT 官方只支持 Linux）。
完整分析见 [`docs/00-环境边界与方案选型.md`](docs/00-环境边界与方案选型.md)。

> **结论**：本项目走「Windows 单独调手 + Ubuntu 22.04 ROS 2 整机联调」的双轨方案。

> **Ubuntu 落在哪台机器上？两种都行，别搞混：**
>
> - **机器人机载 NUC**（出厂自带 Ubuntu 22.04 + ROS 2）—— **从 Windows SSH 进去就能跑整机联调**，
>   不需要给自己电脑装任何 Linux。第一次联调走这条最快。
> - **调试机另装 Ubuntu 22.04 双系统** —— 只有出现「想从本机发 ROS 2 话题远程控制机器人」
>   「本地看话题 / 录 bag / 跑仿真」这些需求时才需要。
>
> 注意：**即使不装双系统也能完成整机联调**；而**即使装了双系统，bridge 节点也仍然必须跑在机器人上**
> （CAN 硬件在那台机器里）。详见 [`docs/09`](docs/09-装配到机器人后的首次整机联调.md)。

---

## 二、硬件接线速记

| 灵巧手 | 机器人主控板接口 | 默认设备 ID | 供电 |
|---|---|---|---|
| **右手（本项目装配）** | **CAN6** | 127（`0x7F`） | 由机器人手臂线缆直接供 **58V** |
| 左手 | CAN5 | 126（`0x7E`） | 同上 |

- ⚠️ **只有 Revo2 进阶版 / 触觉版支持 58V**，基础版接入机器人会烧毁。
- ⚠️ 手腕法兰螺钉旋入深度必须 **< 3 mm**（螺纹孔深 3.5 mm，超了会压裂内部电路板）。
- 上电后手背灯 **绿灯闪烁 → 手指自动张开（位置校准）→ 绿灯常亮**，此时才能发控制指令。

细节见 [`docs/01-灵巧手硬件接线与上电操作手册.md`](docs/01-灵巧手硬件接线与上电操作手册.md)。

---

## 三、仓库结构

```
.
├── README.md                      # 本文件：项目总览
├── CHANGELOG.md                   # 变更记录（每次提交前更新）
├── .gitignore                     # 禁止提交的文件类型
├── .gitattributes                 # 统一换行符，避免跨系统整文件 diff
│
├── docs/                          # 文档（主要产出）
│   ├── 00-环境边界与方案选型.md
│   ├── 01-灵巧手硬件接线与上电操作手册.md   ★ 主要成果
│   ├── 02-Windows单机调试指南.md
│   ├── 03-ELF3整机ROS2联调指南.md
│   ├── 04-通信协议速查-Modbus与CANFD.md
│   ├── 05-故障排查手册.md
│   ├── 06-Git协作与仓库规范.md
│   ├── 07-上位机动作编辑与代码化对照.md    ★ 上位机真实参数 / 动作存储机制 / 接口对照
│   ├── 08-桌面首次连线实操SOP.md   ★ 从接线上电到代码控制，逐步骤照着做
│   ├── 09-装配到机器人后的首次整机联调.md  ★ 装回机器人：装配清单 / 联网 SSH / 首次验收
│   ├── 10-官方资源获取与国内网络下载方案.md  ★ 官方仓库/Wiki/发布包清单 + 四条下载路线
│   ├── 11-真机与仿真指令速查.md   ★ SSH 登录 / root 环境 / 动作字段映射 / 部署流程
│   ├── 12-Ubuntu22.04环境搭建与MuJoCo仿真-笔记本版.md
│   │                              ★ 本机 Ubuntu 还是一片空白？从这条走（含换源治超时）
│   ├── 13-抓取任务技术路线与里程碑.md  ★ 要「拿起瓶装水」怎么起步：M0–M6 六阶段，先做哪个
│   ├── 14-模型拼接与复合动作编排.md  ★ 仿真里本体不含手，怎么把强脑手拼进去
│   ├── 15-M0覆盖通道验证与实测参数.md  ★ 覆盖通道实测：QoS / 释放两段共 0.4s / 频率边界
│   ├── 16-并行开发交接核查与联合路线决议.md
│   │                              ★ 两线并行的交接怎么验收、哪些成果可复用、路线怎么定
│   ├── reference/                 # 归档的参考资料（同事整理的装机方案与指令记录）
│   └── images/                    # 文档插图（照片、截图、尺寸图）
│
├── src/                           # 源码
│   ├── revo2_standalone/          # Windows 单手调试脚本
│   ├── miniyaml.py                # 零依赖 YAML 子集解析器（含 `|` `>` 块标量）
│   └── elf3_ros2/                 # 精灵 3 整机 ROS 2 节点
│       └── config/                # 动作序列、参数配置
│
├── config/
│   ├── hand_params.yaml           # 现场参数台账（ID / 总线 / 波特率 / DOMAIN_ID）
│   ├── gestures.yaml              # 手部手势库（含 bottle_grasp 抓瓶）
│   ├── arm_poses.yaml             # 手臂位姿台账（抓瓶的三个位姿 + PD 增益）
│   └── revo2_model_map.yaml       # ★ 官方仿真模型 ↔ 上位机电机 的权威映射（含 3 条分歧）
│
├── tools/                         # 辅助工具
│   ├── comm_diagnose.py           # 通信链路分布诊断（5 层模型）
│   ├── rs485_probe.py             # ★ 485 转接链路只读扫描：探明哪个口是哪只手 / 波特率 / ID
│   ├── elf3_cmd.sh                # ★ 按动作名给整机发 motion_commands（免手抄 20 个字段的 JSON）
│   ├── arm_pose_pub.py            # ★ 按名字持续发布手臂位姿到 actuators_cmds_override（带安全门禁）
│   ├── revo2_make_actuated.py     # ★ 给官方手 MJCF 补 actuator + 锁 distal（仿真用）
│   ├── ubuntu_bootstrap.sh        # ★ Ubuntu 22.04 环境分阶段一键安装
│   ├── m0_override_check.py       # ★ 覆盖通道验证探针（判据 A–D + 频率扫描，只发仿真话题）
│   ├── arm_wave_demo.py           # ★ 演示轨迹激励源（抬臂→前伸→转腕→外摆→收回，带限位钳位）
│   ├── ros2_mujoco_spectator.py   # ★ 远程观察者：订阅状态离线渲视频（双画面 / 骨架线 / 基座钉住）
│   ├── pose_cam_grid.py           # ★ 离线「姿态 × 机位」网格渲染，用于挑机位 + 打印关节真实限位
│   ├── sweep_scale.sh             # ★ 幅度扫描：测不同覆盖幅度下机体被平衡控制器带走多少
│   ├── run_arm_demo.sh            # ★ 一键复现：起观察者 + 起激励源 + 出指标
│   ├── grab_frame.py              # ★ 从 ROS 2 图像话题抓一帧存 PNG
│   ├── xwd2png.py                 # ★ X11 xwd 截图转 PNG（零依赖，无显示器环境用）
│   └── git_*.sh                   # 一键提交 / 同步脚本
│
└── logs/                          # 运行日志（不入库，只有说明文件）
```

---

## 四、快速开始

### 1. 拿到仓库：本目录是主仓库的子目录

公司主仓库是 **`elf3-humanoid`**（多人共用，根目录是公共区域，分支为 **`master`**）。
本目录的内容放在它的 **`dexterous-hand/`** 子目录里，**不要单独 clone 或单独 `git init`**。

仓库完整体积约 11 MB / 465 个文件（含大量 STL、npz）。网络慢时用**稀疏检出**，
只下载目录树和自己那一块，几秒即可完成：

```bash
cd /d/Desktop/机器人实习/灵巧手

# 只取目录树，不下载文件内容
git clone --filter=blob:none --no-checkout --depth 1 --single-branch \
  git@github.com:Bake-Humanoid/elf3-humanoid.git elf3-humanoid

cd elf3-humanoid
git sparse-checkout set --cone dexterous-hand   # 只检出自己这块
git checkout master
ls -A                                           # 顶层几个小文件 + dexterous-hand/
```

想看看同事都放了什么（不下载文件内容）：

```bash
git ls-tree -r HEAD --name-only | head -60
```

> ⚠ `sparse-checkout` 这一步**不能省**。在 `--no-checkout` 的克隆上直接提交，
> Git 会理解成「删除了全部 465 个文件」。详见 [`docs/06`](docs/06-Git协作与仓库规范.md) 方案 B+。

> **本目录已完成并入**（`dexterous-hand/` 已存在于主仓库）。日常只需「改了 → 提交」：

```bash
cd /d/Desktop/机器人实习/灵巧手/elf3-humanoid
./dexterous-hand/tools/git_sync.sh "docs: 说明改了什么"
```

若要在**新机器上从零重建**这份工作区，才用 `tools/join_main_repo.sh`
（自动完成：取结构 → 打印现有目录 → 确认子目录名 → 稀疏检出 → 拷贝 →
只提交本目录 → 推送）。

### 2. 读文档，按顺序来

1. [`docs/00`](docs/00-环境边界与方案选型.md) —— 先确认任务属于哪条路径
2. [`docs/01`](docs/01-灵巧手硬件接线与上电操作手册.md) —— 接线、上电、验收
3. [`docs/02`](docs/02-Windows单机调试指南.md) 或 [`docs/03`](docs/03-ELF3整机ROS2联调指南.md) —— 按环境进入对应指南
4. [`docs/08`](docs/08-桌面首次连线实操SOP.md) —— **手在桌面上、上位机已打开？**
   从这条走：进度表 → 串口↔左右手判别 → 单位标定 → 用代码接手
5. [`docs/09`](docs/09-装配到机器人后的首次整机联调.md) —— **手要装回机器人了？**
   从这条走：装配前桌面收尾 → 装配清单 → 联网 SSH → 首次验收
6. [`docs/07`](docs/07-上位机动作编辑与代码化对照.md) —— **不接硬件也能读**：
   上位机真实连接参数、动作序列存在哪、怎么用代码替代上位机
7. [`docs/11`](docs/11-真机与仿真指令速查.md) —— **机器人装好了、要发指令跑动作？**
   SSH 登录 → root 环境与 `ROS_DOMAIN_ID` → 服务重启 → 动作字段映射 → 部署代码
8. [`docs/10`](docs/10-官方资源获取与国内网络下载方案.md) —— **官方库下不下来？**
   官方仓库/Wiki/发布包清单（含实测体积）+ 四条下载路线（其中一条 0 下载）
9. [`docs/12`](docs/12-Ubuntu22.04环境搭建与MuJoCo仿真-笔记本版.md) ——
   **本机 Ubuntu 还是一片空白、下载一直超时？** 从这条走：
   换源（治超时的根）→ HWE 内核（无线网卡）→ NVIDIA 驱动与双显卡 →
   ROS2 补装含 RViz2 → MuJoCo 两条路线 → 官方仿真跑起来
10. [`docs/13`](docs/13-抓取任务技术路线与里程碑.md) ——
    **要「拿起瓶装水」，但不知道先编手还是先编臂？** 从这条走：
    两套独立通路的差异 → 抓瓶的物理难点 → M0–M6 六阶段路线 →
    官方 IK 与 `actuators_cmds_override` 要点 → **落地覆盖手臂的安全红线**
11. [`docs/14`](docs/14-模型拼接与复合动作编排.md) ——
    **仿真里本体不含手，怎么把强脑手模型拼进去？怎么一条命令同时控臂和手？**
    从这条走：官方手模型现状（含修正）→ 实物 6 电机 vs 官方 11 关节的
    三条分歧 → 三种拼接方案 → 「先两条命令、再组合成一条」的编排写法
12. [`docs/15`](docs/15-M0覆盖通道验证与实测参数.md) ——
    **覆盖手臂发出去没反应？不知道能发多慢？或者想录一段「手臂在动」的视频却怎么也看不出来？**
    从这条走：
    M0 四项判据实测结论 → **QoS 必须设 `BEST_EFFORT`**（否则节点静默收不到）→
    释放是 `timeout` + `release_blend` **两段共 0.4 s** → 2–20 Hz 频率边界扫描 →
    **相机方位角**（机器人朝 +x、右侧为 −y，录右臂要用 270–330°）→
    官方控制器 31 关节实测增益表 → 模型侧关节真实限位 →
    **大幅动臂会把机体带着走**（含空跑对照与阈值扫描）→ 无显示器环境的出图方案
13. [`docs/16`](docs/16-并行开发交接核查与联合路线决议.md) ——
    **两条并行开发线怎么交接？对方说「模型不带手臂、必须换平台」，这话可信吗？**
    从这条走：交接清单逐条核查结果（属实 / 不成立）→
    **`_hand` 变体仅多出两块固定手掌网格、没有任何手指自由度** →
    「三个坑」按栈分类（哪个通用、哪两个换栈即消失）→
    联合路线决议 → `.133` 进程归属与清场教训（`simulation` 不响应 SIGTERM）
14. [`docs/17`](docs/17-实时遥控仿真窗口.md) ——
    **不想跑完才看录像，想当场演示、还想随手改角度？**
    从这条走：离线录像改实时窗口的实现 → 命令行与键盘两套控制接口 →
    **三个失效环节的实测定位**（起始位落在瓶体范围内 / 手根到位但手指朝向错误 /
    张开的手指下降时将瓶弹飞）→ 外环补偿为何必须限幅并降频 → 实时窗口的相机机位
15. [`docs/19`](docs/19-Windows一键操作灵巧手.md) ——
    **不想敲命令，就在 Windows 上双击？**
    从这条走：一键入口做了什么 → 前置条件与网络自检 → 首次运行自动配免密 →
    菜单七项与 13 段手势编号 → 常见失败的自查表 → 它背后的执行链
16. [`docs/20`](docs/20-摄像头人手遥操灵巧手.md) ——
    **想让机器人的手跟着我的手实时做动作？**
    从这条走：架构与为何不走 ROS → 环境与 MediaPipe 1.0 的 API 变更（模型要单独下）→
    部署与运行 → 姿态映射算法（为何用比值）→ 已知限制（拇指/手别/过热）→ 排查清单
17. [`docs/18`](docs/18-真机灵巧手控制与手势库.md) ——
    **灵巧手装到机器人上之后，怎么从电脑控制它？**
    从这条走：控制链路与「为何控制程序必须跑在机器人上」→
    **运行环境两个故障**（须与硬件节点对齐 `ROS_DOMAIN_ID`+CycloneDDS；
    `CYCLONEDDS_URI` 标签大小写错会导致话题列表静默为空）→
    控制接口与下发数组顺序 → 硬件参数与行程实测 →
    **用 MuJoCo 数值优化标定姿态**（内置手势与触觉在本机不可用）→
    13 段手势库 → 相对时序的写法与真机收敛等待

> **想一条命令把上面第 9 步做完？**
> ```bash
> bash tools/ubuntu_bootstrap.sh --list                   # 先看有哪些阶段
> bash tools/ubuntu_bootstrap.sh --dry-run --stage all    # 再看它会做什么
> bash tools/ubuntu_bootstrap.sh --stage mirror           # 换源（必须先做）
> ```
> 阶段可自由组合（`mirror,kernel,nvidia,ros2,mujoco`），改系统文件前自动备份到
> `~/apt-backup-<时间戳>/`。**只支持 Ubuntu 22.04**，其他版本会被拒绝。

> **官方权威资料**（本目录文档的对照基准）：
> - 官方 Wiki：`BXI_Wiki/docs/elf3/developer/dexterous_hand.zh.md`（外接 Revo2 灵巧手）
> - 官方示例仓库：`github.com/konodoki/bxi_revo2_example`（64 KB，含 C++/Python 两个示例）

> **手上已经拿着硬件、准备第一次接电脑？** 直接跳 **`docs/01` §5.3**。
> 那里有 BrainCo 485 调试模块（转接盒 + 手腕转接小板）的实拍图、端子定义、
> 以及最关键的一步 —— **`485.0` / `485.1` 哪个口是哪只手，没有规定，必须实测**：
>
> ```bash
> pip install pyserial
> python tools/rs485_probe.py          # 纯只读，不改手上任何参数
> ```
>
> 一句话记法：**连电脑调手走 485，连机器人整机走 CANFD**（两个独立的物理接插件）。

### 3. 提交改动

```bash
./tools/git_sync.sh "feat: 补充左手接线照片"
```

该脚本**只会提交本目录下的改动**，不会碰同事的文件。

不熟悉 Git 的话，**先读 [`docs/06-Git协作与仓库规范.md`](docs/06-Git协作与仓库规范.md)**。

---

## 五、协作约定

### 5.1 主仓库硬性约定（根目录 `AGENTS.md`，必须遵守）

本目录是 [`elf3-humanoid`](https://github.com/Bake-Humanoid/elf3-humanoid) 主仓库的子目录，
根目录 [`AGENTS.md`](../AGENTS.md) 规定了硬性约束，**本目录的代码与文档全部按此编写**：

| 约定 | 内容 | 本目录如何落实 |
|---|---|---|
| ROS 版本 | **只用 ROS 2 Humble**（Ubuntu 22.04）。禁止 ROS 1 —— 不得出现 `rospy` / `roscpp` / `catkin` / `roslaunch` / `rosrun` | `src/elf3_ros2/` 全部基于 `rclpy`，无任何 ROS 1 痕迹 |
| RMW | **CycloneDDS**，`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | `docs/01` §6.2、`docs/03` §3、`docs/05` §5 均显式要求导出 |
| Python | **3.10**，必须写类型标注 | 全部脚本含类型标注；依赖极简，见 `requirements.txt` |
| 密钥 | **不得硬编码**，从环境变量读取（参考 [`../.env.example`](../.env.example)） | 本目录不含任何凭据；`tools/git_sync.sh` 提交前扫描密钥特征 |
| 关节编号 | **不得猜测**，本体「关节索引 → 总线 → CAN ID」以 [`../docs/hardware.md`](../docs/hardware.md) 为准 | `docs/01` §5.2 说明灵巧手走 CAN5/CAN6，与本体 bus 0–4 属两套接口 |

> **灵巧手接口不在本体总线编号内。** `docs/hardware.md` 记录的是本体 5 条 CANFD 总线
> （bus 0–4，31 关节）；灵巧手是独立付费外设（hardware.md 中 Hands 记为
> "Paid option, separate unit"），使用 BXI 主控板另外两路 **CAN5 / CAN6**。
> 两套编号不冲突，详见 `docs/01` §5.2 的说明框。

### 5.2 本目录约定

- **分支**：`master`（主仓库默认分支）；日常开发走 `feature/<主题>`，合并前提 PR。
- **提交信息**：沿用主仓库风格，用简短中文说明；如需前缀则 `<类型>: <描述>`，
  类型取 `feat` / `fix` / `docs` / `refactor` / `chore`。
- **不入库**：官方 PDF、`7z` 安装包、`mhtml` 手册、密钥文件、`install/` `build/`。
  大文件走公司文件服务器，README 里放链接。
- **图片**：先压缩，单张 < 500 KB。
- **只为自己的目录提交**：在本仓库里**不要用 `git add -A`**，用
  `git add -- dexterous-hand`（`tools/git_sync.sh` 已限定范围）。

完整规范见 [`docs/06`](docs/06-Git协作与仓库规范.md)。

---

## 六、参考资料

### 6.1 主仓库内的既有资料（动手前先读，避免重复造轮子）

| 路径 | 内容 |
|---|---|
| [`AGENTS.md`](../AGENTS.md) | 主仓库硬性约定（见 §5.1） |
| [`docs/hardware.md`](../docs/hardware.md) | 本体硬件权威来源：31 关节索引/CAN ID 对照表、5 条本体 CANFD 总线、`ROS_DOMAIN_ID` 规则、电机 MIT 接口、整机规格 |
| [`middleware/`](../middleware) | ROS 2 工作空间。其中 `bxi_rl_controller_ros2_example` 的 `mods/com.bxi.basic_actions/` **已实现握手 / 拥抱 / 鼓掌 / 开机问候等成套动作状态机**，`remote_controller/` 含 xbox / ps4 手柄按键配置 |
| `data/mujoco_simulation/elf3_hand.xml` | 手的 MuJoCo 仿真模型，无真机时可先验证动作序列 |

> ⚠️ **后续「本体动作 + 灵巧手动作融合」应基于 `middleware/` 里已有的
> `mods/com.bxi.basic_actions` 框架扩展，不要另起一套。** 本目录的手势 / 序列库
> 定位是「灵巧手侧的动作原语」，最终要挂到那套状态机上去。

### 6.2 外部官方资料

| 对象 | 链接 |
|---|---|
| 半醒科技 BXI Wiki | <https://wiki.bxirobotics.cn/elf3/overview/> |
| BXI 外接灵巧手官方文档 | <https://wiki.bxirobotics.cn/elf3/developer/dexterous_hand/> |
| BXI 官方示例仓库 | <https://github.com/konodoki/bxi_revo2_example> |
| 强脑 Revo2 文档中心 | <https://www.brainco-hz.com/docs/revolimb-hand/revo2/introduction.html> |
| 强脑 Revo2 ROS 2 驱动 | <https://github.com/BrainCoTech/brainco_hand_ros2> |
| 强脑官方 SDK | <https://github.com/BrainCoTech/brainco-hand-sdk> |

---

## 七、维护信息

- 维护人：（待填）
- 项目阶段：实训 / 原型验证
- 最后更新：2026-09-20

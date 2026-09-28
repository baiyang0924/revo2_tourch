# ELF3 × Revo2 整机 ROS2 联调指南

> 环境：Ubuntu 22.04 + ROS 2 Humble（双系统）。
> 目标：**电脑远程控制机器人，驱动机器人手臂末端的灵巧手完成动作。**
>
> 这是本项目的正式生产环境。Windows 只能调手，这里是唯一能联动整机的地方。
>
> **手刚从桌面拆下来、第一次装到机器人上？** 先看
> [`09-装配到机器人后的首次整机联调.md`](09-装配到机器人后的首次整机联调.md)：
> 那里回答了「是不是必须在自己电脑上装 Ubuntu」（**不用，SSH 进机器人就能跑**）、
> 装配前必做的桌面收尾、装配清单、网络连通与首次验收的完整顺序。

---

## 1. 系统架构

```
┌──────────────────────────────┐
│  你的电脑（上位控制端）        │   发指令：ROS 2 话题
│  Ubuntu 22.04 / 或 SSH 进机器人│
└──────────────┬───────────────┘
               │  ROS 2 话题（DDS，跨机自动发现）
               ▼
┌──────────────────────────────┐
│  精灵 3 机载 NUC（主脑）       │   Ubuntu 22.04 + ROS 2 Humble
│  ┌────────────────────────┐  │
│  │ bxi_ros2_pkg 硬件节点    │  │   root 下运行
│  │  /canfd_packet/rx       │  │
│  │  /canfd_packet/tx       │  │
│  └───────────┬────────────┘  │
└──────────────┼───────────────┘
               │  PCIe
               ▼
┌──────────────────────────────┐
│  BXI 主控板（躯干内）          │
│  CAN5 ── 手臂走线 ── 左手 126  │   58V 供电 + CANFD 通信（二合一）
│  CAN6 ── 手臂走线 ── 右手 127  │
└──────────────────────────────┘
```

**设计的两种角色分工**（本仓库脚本按这个设计）：

| 角色 | 跑在哪 | 干什么 |
|---|---|---|
| **driver / bridge** | 机器人 NUC | 占住 CAN 链路，把高层指令翻译成 CANFD 帧，读回反馈 |
| **commander** | 你的电脑 | 只发高层指令（目标关节角、动作序列），不碰底层总线 |

这样拆分的好处：电脑上改控制逻辑不用碰机器人环境；
两台机器之间只传语义清晰的指令，不传原始 CAN 帧。

---

## 2. 一次性环境搭建

### 2.1 在机器人 NUC 上

前置：机器人已正常启动，且能 SSH 登录（路由器后台查机器人 IP）。

```bash
# ── ① 确认硬件节点已起 ──────────────────────────
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
ros2 topic list | grep canfd_packet
#   期望：/canfd_packet/rx  /canfd_packet/tx

# ── ② 克隆官方示例包 ───────────────────────────
cd ~/bxi_ws
git clone https://gh-proxy.com/https://github.com/konodoki/bxi_revo2_example.git
#   直连慢就带 gh-proxy 前缀；还不行换 https://ghproxy.net/...

# ── ③ 下载 C++ SDK（编译前必须，否则缺 stark-sdk.h）──
cd ~/bxi_ws/bxi_revo2_example
./download-lib.sh
#   会生成 dist/include/、dist/shared/linux/libbc_stark_sdk.so、VERSION

# ── ④ 装 Python SDK（跑 Python 示例用）─────────
pip install bc-stark-sdk==1.5.1 --index-url https://pypi.org/simple/
#   官方指定版本，别装最新；装在系统 Python，别在 conda 里

# ── ⑤ 编译 ─────────────────────────────────────
cd ~/bxi_ws/bxi_revo2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
colcon build
source install/setup.bash
```

### 2.2 在你的电脑上

如果电脑也要发 ROS 2 话题（远程控制端），需要：

```bash
sudo apt update
sudo apt install -y ros-humble-desktop   # 或至少 ros-humble-rmw-cyclonedds-cpp
```

并确认电脑和机器人**在同一网段**，且 DDS 能互相发现。

---

## 3. 每次使用的标准流程

这一套每次都要走，**顺序不能变**。

```bash
# ── ① 启动机器人 ───────────────────────────────
#   开机后系统自启动 ros_elf_launch.service 拉起遥控器程序
#   按遥控器 Start 键（右摇杆按下）启动真机程序
#   全身电机指示灯亮起，约 6 秒后自检通过
#   ★ 灵巧手自动张开 = 硬件链路正常

# ── ② 切 root 并设环境（硬件节点在 root 下，必须一致）──
sudo su
export ROS_DOMAIN_ID=<机器人的DOMAIN_ID>
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

# ── ③ 确认话题在 ───────────────────────────────
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
ros2 topic list | grep canfd_packet

# ── ④ 加载工作空间 ─────────────────────────────
cd ~/bxi_ws/bxi_revo2_example
source install/setup.bash
```

### 3.1 `ROS_DOMAIN_ID` 怎么确定

官方规则：**遥控器启动 = 30 + 机器人序号；App 启动固定 22**。

```bash
# 方法一
grep -r ROS_DOMAIN_ID /etc/systemd/system/ 2>/dev/null

# 方法二：逐个试
for id in 0 22 30 31 32; do
  ROS_DOMAIN_ID=$id RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic list 2>/dev/null \
    | grep -q canfd_packet && echo "正确的是 DOMAIN_ID=$id"
done
```

> ⚠️ 不要在普通用户终端设好变量再 `sudo su`——变量不会带过去，**必须在 root shell 里重新 export**。

---

## 4. 第一步：跑通官方示例

**在做任何二次开发之前，先证明整条链路是通的。**

```bash
ros2 run bxi_revo2_example bxi_revo2_example.py
# C++ 版本：ros2 run bxi_revo2_example bxi_revo2_example
```

**期望现象**（官方验收标准）——手会依次执行：

1. 基本位置控制：握拳、张开、单指移动；
2. 速度、电流、PWM 控制；
3. 高级控制：单位模式、位置+时间、位置+速度、多指联动；
4. 内置动作序列：Open、Fist、Pinch、Point 等；
5. 设备信息与参数查询。

默认**先右手、再左手**（本项目实机为右手），两只手都跑完且终端无持续报错即部署完成。
（只接了一只手时，另一只那半段超时报错属正常。）

> ⚠️ 跑之前确认手指活动范围内**没有人体、线缆和易损物体**。
> 握拳动作行程大，第一次建议机器人摆好、手朝外、人站侧面。

---

## 5. 第二步：远程控制手（本仓库脚本）

### 5.1 在机器人上起 bridge

```bash
cd <本仓库>/src/elf3_ros2
python3 revo2_bridge_node.py --hand right --bus 6 --id 127 --master-id 1
# 另一只手：--hand left --bus 5 --id 126 --master-id 1
```

bridge 的职责：

- 订阅高层指令话题（目标关节角 / 动作名）
- 调用 SDK 下发到对应总线与 ID
- 发布实际位置 / 速度 / 电流反馈

### 5.2 在电脑上发指令

```bash
# 单个手势（--hand 默认 right，即本项目右手）
python3 revo2_commander.py --gesture fist

# 指定关节角（度）
python3 revo2_commander.py --angles 0,0,0,0,0,0        # 张开
python3 revo2_commander.py --angles 50,70,78,78,78,78  # 握拳（示例值）

# 播放动作序列
python3 revo2_sequence_player.py --sequence config/sequences.yaml --name 喝水

# 换左手时显式指定
python3 revo2_commander.py --hand left --gesture fist
```

> ⚠️ 关节角顺序固定为：
> `[拇指Flex, 拇指Aux, 食指, 中指, 无名指, 小拇指]`
> 角度上限：拇指 Flex 59°、拇指 Aux 90°、其余四指 81°。
> **上面是示例值，第一次务必先小幅度确认方向。**

### 5.3 关于脚本里的 SDK 调用

脚本按 `bc-stark-sdk` v2.x 文档 API 编写，并对方法名做了兜底。
**不同 SDK 版本的方法名可能不同**，首次使用请：

1. 运行 `python3 revo2_bridge_node.py --check-api` 打印当前 SDK 暴露的符号；
2. 与 `python3 -c "import bc_stark_sdk,os;print(os.path.dirname(bc_stark_sdk.__file__))"`
   目录下的 `.pyi` 定义核对；
3. 需要时按你的 SDK 版本调整适配层（适配逻辑集中在文件开头的 `SDK Adapter` 区）。

---

## 6. 第三步：动作序列录制与回放

### 6.1 数据流

```
录制：读反馈（实际位置 2000~2005） → 按固定频率采样 → 存成序列文件
回放：读序列文件 → 按时间戳下发（位置+时间控制，寄存器 1010） → 观察复现
```

### 6.2 手动手势的落地方式

| 方式 | 做法 | 适合 |
|---|---|---|
| 上位机 GUI 编辑 | Windows 上位机 `Action Sequence` 页编辑后导出 | 少量、简单的固定手势 |
| 代码定义 | 直接写进 `config/sequences.yaml` | 参数明确、需要版本管理 |
| 录制回放 | 手动摆位 → 采样 → 存成序列 → 回放 | 复杂动作、想先从人手里"学"下来 |

### 6.3 序列文件格式

见 `src/elf3_ros2/config/sequences.yaml`。字段说明：

```yaml
sequences:
  喝水:
    description: 拿起杯子并送到嘴边（示例）
    loop: false
    steps:
      - name: 张开准备
        angles: [0, 0, 0, 0, 0, 0]      # 度，[拇Flex, 拇Aux, 食, 中, 无名, 小]
        duration_ms: 800                  # 期望时间，范围 1~2000
      - name: 闭合握杯
        angles: [45, 60, 60, 60, 60, 60]
        duration_ms: 1200
      - name: 保持
        angles: [45, 60, 60, 60, 60, 60]
        duration_ms: 2000
```

### 6.4 后续：与本体动作融合

融合的本质是**在同一时间轴上编排两类动作**：

```
时间轴 ──────────────────────────────────────────────►
本体动作   抬臂到高位 ──── 前伸 ──── 停 ──── 回收
手部动作        张开 ── 闭合 ── 保持 ── 张开
关节目标   手臂7关节 + 手6关节 一起下发（需要各自的控制器同步）
```

要点：

1. **两套控制器要同步**：本体走运控算法，手走 CANFD，节奏不一致会导致"手比手臂先到"；
2. **留出安全余量**：手闭合时手臂不要同时做剧烈动作，避免线缆受力；
3. **先在仿真里验证**：`demo.bxirobotics.cn` 的运控算法 demo、以及
   ELF3 的 MuJoCo 仿真环境，可以先跑通再去真机。

---

## 7. 常用调试命令速查

```bash
# 节点与话题
ros2 node list
ros2 topic list
ros2 topic list | grep canfd_packet
ros2 topic echo /canfd_packet/rx --once        # 看有没有帧进来
ros2 topic hz /canfd_packet/rx                 # 看帧率

# ros2_control（如果用了 brainco_hand_ros2）
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states

# 发一条关节轨迹（强脑 ROS 2 驱动风格）
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: ["right_thumb_proximal_joint","right_thumb_metacarpal_joint",
                  "right_index_proximal_joint","right_middle_proximal_joint",
                  "right_ring_proximal_joint","right_pinky_proximal_joint"],
    points: [{positions: [0.0,0.0,0.0,0.0,0.0,0.0], time_from_start: {sec: 1}}]}'
```

---

## 8. 出问题

按顺序查：

1. `/canfd_packet/rx` `tx` 在不在 → 不在是硬件节点没起（`docs/05` 5.1）
2. `ROS_DOMAIN_ID` 是否一致（整机第一坑）
3. 左右手有没有接反（症状：终端刷数据但手指不动）
4. 有没有别的进程在抢同一只手
5. 完整排查见 `docs/05-故障排查手册.md`

---

## 9. 参考来源

| 内容 | 链接 |
|---|---|
| BXI 官方 wiki《外接灵巧手》 | <https://wiki.bxirobotics.cn/elf3/developer/dexterous_hand/> |
| BXI 运动控制开发指南 | <https://wiki.bxirobotics.cn/elf3/developer/motioncontrol/> |
| BXI 官方示例仓库 | <https://github.com/konodoki/bxi_revo2_example> |
| BXI PCIe 转 CANFD 驱动 | <https://github.com/bxirobotics/bxi_pci_drv> |
| 强脑 Revo2 ROS 2 项目 | <https://www.brainco-hz.com/docs/revolimb-hand/revo2/ros2/overview.html> |
| 强脑 ROS 2 驱动仓库 | <https://github.com/BrainCoTech/brainco_hand_ros2> |
| 运控算法 demo | <https://demo.bxirobotics.cn/> |

---

## 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-09-20 | 首版：系统架构、环境搭建、标准流程、官方示例验收、远程控制、动作序列录制回放、与本体动作融合思路 |
| v1.1 | 2026-09-20 | 文首新增指向 `docs/09` 的入口（装配到机器人后的首次整机联调 SOP） |

# ELF3 × Revo2 灵巧手：整机集成与二次开发

在人形机器人 **ELF3（半醒 BXI，31 自由度）** 的右腕上集成 **强脑 Revo2 Touch 触觉版灵巧手**，
从硬件接线、通信链路、真机控制、仿真标定一路做到手势库、一键操作与人手遥操的完整工程记录。

> 仓库性质：实习期间个人产出的归档。代码与文档均为本人编写，不含第三方同事的文件。
> 内容以「可直接照做的结论 + 实测数据」为主，不含任何凭据与内部资料。

---

## 一、成果速览

| # | 成果 | 交付物 | 状态 |
|---|---|---|---|
| 1 | 打通「PC → SSH → 机器人 → BXI 主控板 → CANFD → 灵巧手」全链路 | `src/elf3_ros2/revo2_bridge_node.py` | 真机已跑通 |
| 2 | 13 段手势库 + 编排运行时（相对时序、真机收敛等待） | `tools/revo2_hand_showcase.py` | 真机已跑通 |
| 3 | 硬件参数集中到单一配置文件，消除散落各处的硬编码 | `config/revo2_hardware.yaml` | 已按评审意见整改 |
| 4 | 所有下发指令统一做安全校验（关节限位 / 时长范围 / 类型） | `revo2_bridge_node.py::validate_target` | 已按评审意见整改 |
| 5 | Windows 双击即用的一键菜单（免密登录自动配置） | `tools/revo2_hand_console.bat` | 已交付 |
| 6 | 用笔记本摄像头识别人手姿态，机器人灵巧手实时模仿 | `tools/revo2_hand_teleop_client.py` + `..._server.py` | 真机已跑通 |
| 7 | 用 MuJoCo 数值优化标定指尖触瓶姿态（内置手势/触觉不可用时的替代方案） | `tools/revo2_hand_calib.py` | 仿真标定完成 |
| 8 | 整机 M0 覆盖通道实测：QoS、释放时序、频率边界、平衡耦合阈值 | `docs/15` | 实测数据齐备 |
| 9 | 官方手模型拼入整机模型，抓水瓶复合动作端到端仿真 | `tools/revo2_pick_bottle_demo.py` | 仿真基准达成 |
| 10 | 21 篇过程文档，覆盖环境边界、接线、协议、排障、路线决议 | `docs/` | 持续维护 |

---

## 二、硬件与系统构成

```
[Windows / Ubuntu 调试机]
        │  SSH
        ▼
[机器人机载电脑 Ubuntu 22.04 + ROS 2 Humble]
        │  ROS 2 话题 /canfd_packet/rx|tx
        ▼
[BXI 主控板 PCI 设备]  ← SDK 必须跑在机器人上，PC 上没有该 PCI 设备
        │  CANFD（右手 CAN6 / ID 127，左手 CAN5 / ID 126）
        ▼
[Revo2 Touch 灵巧手，6 电机]
```

| 项 | 值 |
|---|---|
| 机器人 | ELF3，31 自由度，58.8 V 供电，机载 i7-1370P |
| 灵巧手 | Revo2 Touch 触觉版，**右手**，主控板 CAN6 / 设备 ID 127 |
| 下发数组 | `[拇指弯曲, 拇指侧摆, 食指, 中指, 无名指, 小指]`，归一化 **0–1000**（0 = 张开，1000 = 完全握拳） |
| 电机行程 | 拇指弯曲 59°、拇指侧摆 89°、四指各 80.8° |
| 全行程耗时 | 0.69 s（默认速度最快；显式指定 `max_speed` 反而慢约 6 倍且到不了目标） |
| 运行环境 | root + `ROS_DOMAIN_ID=22` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI` |

> ⚠️ **控制程序必须跑在机器人上。** 灵巧手走 BXI 主控板的 PCI 设备，调试机上没有该硬件，
> 直接在本机跑 SDK 会失败。链路只能通过 SSH 转发。

---

## 三、关键实测结论

这一节是整套工作里最有复用价值的部分，全部为实测数据而非文档转述。

### 3.1 通信环境必须逐项对齐，否则静默失败

硬件节点运行在 root 下，其 `ROS_DOMAIN_ID` / RMW / CycloneDDS 配置**只从运行中的进程读取**才准：

```bash
sudo tr "\0" "\n" < /proc/<pid>/environ | grep -E "ROS_DOMAIN|RMW|CYCLONEDDS"
```

- `CYCLONEDDS_URI` 的闭合标签大小写写错（如 `</Clonedds>`）→ parser error → `rmw_create_node` 失败；
  而 `ros2 topic list` **返回空、退出码仍是 0**，不报错。修复后话题数从 2 恢复到 54。
- 覆盖通道 `/hardware/actuators_cmds_override` 两端 QoS 均为 **BEST_EFFORT**；
  rclpy 默认 RELIABLE → 一条都收不到且不抛异常。**排查时第一件事就是查 QoS**。

### 3.2 手臂覆盖通道的时序与频率边界

- 释放 = `timeout` 0.2 s + `release_blend` 0.2 s，**两段共 0.4 s**。
- 维持接管需 **≥ 5 Hz** 持续发送（实践用 20 Hz）；断流超过 0.2 s 控制权即被收回。
- 接管延迟约等于一个发布周期。
- ⛔ **不要用覆盖通道锁下肢**：踝关节锁死后 3–4 s 内摔倒。

### 3.3 动臂会把机体带走（平衡耦合）

同一动作按不同幅度缩放，机体世界位移呈强超线性：

| 幅度系数 | 躯体位移 |
|---|---|
| ×0.35 | 0.121 m |
| ×0.55 | 0.471 m |
| ×1.00 | **4.096 m** |

肩关节外展的安全阈值实测：−15° 时腰扭稳定在约 3.9°；−30° 时升到 8.4° 并开始迈步。
结论：**想让机器人站着只动手臂，必须在覆盖前先进入不依赖手臂维持平衡的状态。**

### 3.4 灵巧手模型与真机的口径差异

- 官方 MJCF 手模型有 11 个 hinge（拇指 3 + 四指各 2），但实物只有 **6 个可下发角度**；
  URDF 里四指远节带 `<mimic ×1.155>`，MJCF 没有 → **仿真必须同样锁住远节**才能对得上。
- 官方手碰撞网格与 Ø65 mm 瓶身不兼容（近节嵌入 39.8 mm）→ 纯接触抓取必然把瓶崩飞。

### 3.5 仿真标定的三个必踩坑

1. 起始位不能用零位关节角（手根离瓶仅 30 mm，一动就崩），须用 IK 解到安全起始位。
2. IK 只约束 `axis_y` 是 5 自由度 → 手指朝向随机；**必须同时约束 `axis_z`**。
3. 张开手指下降会插进瓶体把瓶弹飞 → 运行时改碰撞分组。

端到端基准（臂 + 手 + 抓瓶）：**372.9 mm / 283.3 mm / 6.9°**，Linux 与 Windows 逐项一致。

### 3.6 人手遥操的已知限制

- 拇指映射精度低于四指：拇指是双自由度（弯曲 + 侧摆），单目视觉对侧摆角的估计本就困难。
- 同一 SDK 实例只允许一个客户端占用一只手：若另有脚本初始化 SDK，会抢走控制权，
  表现为「命令发出去但手不动」。
- 长时间连续驱动会触发电机过热保护，遥操客户端因此内置了静止卸力阈值。

---

## 四、仓库结构

```
.
├── README.md                 # 本文件
├── CHANGELOG.md              # 变更记录
│
├── docs/                     # 21 篇过程文档（主要产出）
│   ├── 00-环境边界与方案选型          ★ 为什么必须走「Windows 调手 + Ubuntu 联整机」双轨
│   ├── 01-灵巧手硬件接线与上电操作手册 ★ 接线 / 上电 / 验收，含法兰螺钉 <3 mm 红线
│   ├── 02-Windows单机调试指南
│   ├── 03-ELF3整机ROS2联调指南
│   ├── 04-通信协议速查-Modbus与CANFD
│   ├── 05-故障排查手册
│   ├── 06-Git协作与仓库规范
│   ├── 07-上位机动作编辑与代码化对照   ★ 上位机真实参数 / 动作存储机制 / 接口对照
│   ├── 08-桌面首次连线实操SOP          ★ 从接线上电到代码控制，逐步照做
│   ├── 09-装配到机器人后的首次整机联调 ★ 装配清单 / 联网 SSH / 首次验收
│   ├── 10-官方资源获取与国内网络下载方案
│   ├── 11-真机与仿真指令速查           ★ SSH / root 环境 / 动作字段映射 / 部署
│   ├── 12-Ubuntu22.04环境搭建与MuJoCo仿真
│   ├── 13-抓取任务技术路线与里程碑     ★ M0–M6 六阶段路线
│   ├── 14-模型拼接与复合动作编排       ★ 官方手模型拼进整机 + 臂手联合编排
│   ├── 15-M0覆盖通道验证与实测参数     ★ QoS / 释放时序 / 频率边界 / 相机机位
│   ├── 16-并行开发交接核查与联合路线决议
│   ├── 17-实时遥控仿真窗口
│   ├── 18-真机灵巧手控制与手势库       ★ 链路、运行环境故障、13 段手势
│   ├── 19-Windows一键操作灵巧手        ★ 双击即用入口与自查表
│   ├── 20-摄像头人手遥操灵巧手         ★ MediaPipe 1.0 API 变更 / 映射算法 / 限制
│   └── images/                        # 接线照片与截图
│
├── src/
│   ├── elf3_ros2/                     # ROS 2 节点：bridge（驱动）+ commander（上位）
│   └── revo2_standalone/              # Windows 单手调试脚本
│
├── config/
│   ├── revo2_hardware.yaml            # ★ 硬件参数单一来源（限位 / 总线 / ID / 关节映射）
│   ├── gestures.yaml                  # 手势库
│   ├── arm_poses.yaml                 # 手臂位姿台账
│   ├── hand_params.yaml               # 现场参数台账
│   └── revo2_model_map.yaml           # 官方模型 ↔ 上位机电机的权威映射
│
└── tools/                             # 36 个脚本：手势、标定、遥操、诊断、仿真、部署
```

---

## 五、按场景入口

| 场景 | 入口 |
|---|---|
| 手上已拿硬件，第一次接电脑 | `docs/01` §5.3 → `python tools/rs485_probe.py`（纯只读扫描，判明哪个口是哪只手） |
| 手装到机器人上了，要从电脑控它 | `docs/18` |
| 不想敲命令，Windows 上双击就用 | `docs/19` → `tools/revo2_hand_console.bat` |
| 想让机器人的手跟着自己的手做动作 | `docs/20` → `tools/revo2_teleop.bat` |
| 仿真里本体不含手，想把手拼进去 | `docs/14` |
| 覆盖了手臂但没反应 / 想录视频看不出在动 | `docs/15` |
| 要「拿起瓶装水」，不知先编手还是先编臂 | `docs/13` |

一句话记法：**连电脑调手走 485，连机器人整机走 CANFD**，两个独立物理接插件。

---

## 六、技术栈

ROS 2 Humble（rclpy）· CycloneDDS · CANFD · Modbus RTU · Python 3（类型标注） ·
MuJoCo 3（数值优化与实时窗口） · MediaPipe HandLandmarker · OpenCV · 零依赖 YAML 子集解析器

工程约束：只用 ROS 2 Humble，禁止 ROS 1；RMW 统一 CycloneDDS；凭据不入库，一律环境变量或现场询问。

---

## 七、官方资料对照基准

| 对象 | 链接 |
|---|---|
| BXI 官方 Wiki | <https://wiki.bxirobotics.cn/elf3/overview/> |
| BXI 外接灵巧手文档 | <https://wiki.bxirobotics.cn/elf3/developer/dexterous_hand/> |
| BXI 官方示例仓库 | <https://github.com/konodoki/bxi_revo2_example> |
| 强脑 Revo2 文档中心 | <https://www.brainco-hz.com/docs/revolimb-hand/revo2/introduction.html> |
| 强脑 Revo2 ROS 2 驱动 | <https://github.com/BrainCoTech/brainco_hand_ros2> |
| 强脑官方 SDK | <https://github.com/BrainCoTech/brainco-hand-sdk> |

---

## 八、维护信息

- 阶段：硬件集成 / 原型验证
- 实机侧别：右手（左右完全对称，切换只需改 `right` ↔ `left`、CAN6/127 ↔ CAN5/126）
- 时间跨度：2026-09-20 起

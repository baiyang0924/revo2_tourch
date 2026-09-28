#!/usr/bin/env bash
# ============================================================================
#  ubuntu_bootstrap.sh —— Ubuntu 22.04 (jammy) 环境引导脚本
#
#  把 docs/12-Ubuntu22.04环境搭建与MuJoCo仿真-笔记本版.md 里的长命令
#  固化成可重复执行的一条命令。每个阶段都是幂等的：重复跑不会出事。
#
#  用法：
#    bash tools/ubuntu_bootstrap.sh --list                  # 看有哪些阶段
#    bash tools/ubuntu_bootstrap.sh --dry-run --stage all   # 只看会做什么
#    bash tools/ubuntu_bootstrap.sh --stage mirror          # 只换源（先跑这个！）
#    bash tools/ubuntu_bootstrap.sh --stage kernel,nvidia   # 跑多个阶段
#    bash tools/ubuntu_bootstrap.sh --stage all             # 全流程
#    bash tools/ubuntu_bootstrap.sh --stage verify          # 只做体检
#
#  阶段（建议按顺序）：
#    mirror  apt / ROS2 / pip 换国内源      ← 治"下载慢、超时"，必须先做
#    kernel  HWE 内核（MT7922 无线网卡支持）
#    nvidia  显卡驱动 + PRIME 设置
#    ros2    补装 ros-humble-desktop（含 RViz2）与常用工具
#    mujoco  MuJoCo 依赖 + pip 安装
#    bxi     官方 BXI 仿真环境（bxi_ros2_pkg）
#    verify  体检：把 docs/12 §7 的检查项跑一遍
#
#  安全设计：
#    * 改任何系统文件前先备份，备份路径会打印出来
#    * --dry-run 只打印不执行，第一次务必先跑一遍
#    * 阶段之间互不依赖，可以只跑你需要的那个
#    * 不会自动重启；需要重启的地方会明确提示你自己重启
#
#  ⚠️ 只支持 Ubuntu 22.04 (jammy)。其他版本会直接退出，不要硬来。
# ============================================================================

set -uo pipefail

# ---- 颜色 -------------------------------------------------------------------
if [ -t 1 ] && [ "${TERM:-dumb}" != "dumb" ]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'
  C_BLU=$'\033[34m'; C_DIM=$'\033[2m';  C_RST=$'\033[0m'
else
  C_RED=''; C_GRN=''; C_YEL=''; C_BLU=''; C_DIM=''; C_RST=''
fi

ok()   { printf '%s  ✔ %s%s\n' "$C_GRN" "$1" "$C_RST"; }
bad()  { printf '%s  ✘ %s%s\n' "$C_RED" "$1" "$C_RST"; }
warn() { printf '%s  ! %s%s\n' "$C_YEL" "$1" "$C_RST"; }
info() { printf '%s    %s%s\n' "$C_DIM" "$1" "$C_RST"; }
head2(){ printf '\n%s=== %s ===%s\n' "$C_BLU" "$1" "$C_RST"; }

# ---- 参数 -------------------------------------------------------------------
STAGES_KNOWN="mirror kernel nvidia ros2 mujoco bxi verify"
STAGES="mirror"
MIRROR="tuna"
DRY_RUN=0
ASSUME_YES=0

usage() {
  cat <<'USAGE'
ubuntu_bootstrap.sh —— Ubuntu 22.04 (jammy) 环境引导脚本

用法：
  bash tools/ubuntu_bootstrap.sh --list                  # 看有哪些阶段
  bash tools/ubuntu_bootstrap.sh --dry-run --stage all   # 只看会做什么（可在任意系统上跑）
  bash tools/ubuntu_bootstrap.sh --stage mirror          # 只换源（先跑这个！）
  bash tools/ubuntu_bootstrap.sh --stage kernel,nvidia   # 跑多个阶段
  bash tools/ubuntu_bootstrap.sh --stage all             # 全流程
  bash tools/ubuntu_bootstrap.sh --stage verify          # 只做体检

阶段（建议按顺序）：
  mirror  apt / ROS2 / pip 换国内源      ← 治“下载慢、超时”，必须先做
  kernel  HWE 内核（MT7922 无线网卡支持）
  nvidia  显卡驱动 + PRIME 设置
  ros2    补装 ros-humble-desktop（含 RViz2）与常用工具
  mujoco  MuJoCo 依赖 + pip 安装
  bxi     官方 BXI 仿真环境（bxi_ros2_pkg）
  verify  体检：检查内核 / 网卡 / 显卡 / ROS2 / MuJoCo 状态

选项：
  --stage <阶段[,阶段]>   要执行的阶段，默认 mirror
  --mirror <源>           tuna(默认) / ustc / huawei / tencent
                          （aliyun 在本项目实测网络中不可达，已禁用）
  --dry-run, -n           只打印不执行；此时允许在非 Ubuntu 系统上运行
  --yes, -y               不交互确认
  --list                  列出阶段
  -h, --help              显示本帮助

安全：改系统文件前会备份到 ~/apt-backup-<时间戳>/；脚本不会自动重启。

目录文档：docs/12-Ubuntu22.04环境搭建与MuJoCo仿真-笔记本版.md

⚠️ 真正执行只支持 Ubuntu 22.04 (jammy)，其他版本会直接退出。
USAGE
  exit 0
}

while [ $# -gt 0 ]; do
  case "$1" in
    --stage)     STAGES="${2:-}"; shift 2 ;;
    --stage=*)   STAGES="${1#*=}"; shift ;;
    --mirror)    MIRROR="${2:-tuna}"; shift 2 ;;
    --mirror=*)  MIRROR="${1#*=}"; shift ;;
    --dry-run|-n) DRY_RUN=1; shift ;;
    --yes|-y)    ASSUME_YES=1; shift ;;
    --list)
      head2 "可用阶段"
      for s in $STAGES_KNOWN; do
        case "$s" in
          mirror) echo "  mirror   apt / ROS2 / pip 换国内源（治下载超时，先做这个）" ;;
          kernel) echo "  kernel   装 HWE 内核（MT7922 无线网卡 + 新硬件支持）" ;;
          nvidia) echo "  nvidia   显卡驱动 + PRIME 双显卡设置（RTX 4080 Laptop）" ;;
          ros2)   echo "  ros2     补装 ros-humble-desktop（含 RViz2）与常用工具" ;;
          mujoco) echo "  mujoco   MuJoCo 系统依赖 + pip 安装 + 自检" ;;
          bxi)    echo "  bxi      官方 BXI 仿真环境 bxi_ros2_pkg → /opt/bxi" ;;
          verify) echo "  verify   体检：检查内核/网卡/显卡/ROS2/MuJoCo 状态" ;;
        esac
      done
      echo
      echo "  特殊值：all = mirror,kernel,nvidia,ros2,mujoco,verify"
      echo
      exit 0 ;;
    -h|--help) usage ;;
    *) bad "未知参数：$1（用 --help 看用法）"; exit 2 ;;
  esac
done

# 把 --stage all 展开（bxi 比较重，不放进 all，需要时显式指定）
if [ "$STAGES" = "all" ]; then
  STAGES="mirror,kernel,nvidia,ros2,mujoco,verify"
fi

# 校验阶段名
IFS=',' read -r -a STAGE_ARR <<< "$STAGES"
for s in "${STAGE_ARR[@]}"; do
  case " $STAGES_KNOWN " in
    *" $s "*) ;;
    *) bad "未知阶段 '$s'（用 --list 看可用的）"; exit 2 ;;
  esac
done

has_stage() {
  local want="$1" s
  for s in "${STAGE_ARR[@]}"; do [ "$s" = "$want" ] && return 0; done
  return 1
}

# ---- 执行封装 ---------------------------------------------------------------
# run <人类可读说明> <命令...>
run() {
  local desc="$1"; shift
  if [ "$DRY_RUN" = "1" ]; then
    printf '%s  [dry-run] %s%s\n' "$C_DIM" "$desc" "$C_RST"
    return 0
  fi
  printf '%s  → %s%s\n' "$C_BLU" "$desc" "$C_RST"
  if "$@"; then return 0; else
    bad "失败：$desc"
    info "命令：$*"
    return 1
  fi
}

# 需要 sudo 时先确认一下
need_sudo() {
  if [ "$DRY_RUN" = "1" ]; then return 0; fi
  if [ "$(id -u)" = "0" ]; then return 0; fi
  if ! sudo -n true 2>/dev/null; then
    printf '%s  → 需要 sudo 权限，可能要求输入密码%s\n' "$C_BLU" "$C_RST"
    sudo -v || { bad "拿不到 sudo 权限"; return 1; }
  fi
}

confirm() {
  [ "$ASSUME_YES" = "1" ] && return 0
  [ "$DRY_RUN" = "1" ] && return 0
  local ans
  printf '%s  ? %s [y/N] %s' "$C_YEL" "$1" "$C_RST"
  read -r ans </dev/tty || ans=""
  case "$ans" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

# ---- 前置检查 ---------------------------------------------------------------
head2 "环境检查"

if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
fi

# 非 jammy：dry-run 时放行（方便先看看它会做什么），真正执行时拒绝
NOT_JAMMY=""
if [ "${ID:-}" != "ubuntu" ]; then
  NOT_JAMMY="这不是 Ubuntu（检测到 ID=${ID:-未知}）"
elif [ "${VERSION_CODENAME:-}" != "jammy" ]; then
  NOT_JAMMY="系统不是 jammy(22.04)，当前是 ${VERSION_CODENAME:-未知}"
fi

if [ -n "$NOT_JAMMY" ]; then
  if [ "$DRY_RUN" = "1" ]; then
    warn "$NOT_JAMMY"
    warn "--dry-run 模式下继续（只打印不执行），方便你先审一遍动作。"
    warn "真正执行时这个脚本会被直接拒绝。"
  else
    bad "$NOT_JAMMY"
    info "本脚本只针对 Ubuntu 22.04 编写，其他版本请勿使用。"
    exit 1
  fi
else
  ok "系统：Ubuntu 22.04 (jammy)"
fi

if [ "$(id -u)" = "0" ]; then
  warn "正在以 root 身份运行。建议用普通用户跑（脚本内部自己调 sudo）。"
fi

# 注意：不要用 $USER —— 有些非登录 shell（如 cron、部分 sudo 环境）里它不存在，
# 配合 set -u 会直接报 unbound variable。id -un 一定可靠。
REAL_USER="${SUDO_USER:-$(id -un)}"
REAL_HOME="$(getent passwd "$REAL_USER" 2>/dev/null | cut -d: -f6)"
[ -z "${REAL_HOME:-}" ] && REAL_HOME="${HOME:-/home/$REAL_USER}"
info "用户：$REAL_USER  家目录：$REAL_HOME"

STAMP="$(date +%F-%H%M%S)"
echo
printf '%s  本次计划执行的阶段：%s%s\n' "$C_BLU" "${STAGE_ARR[*]}" "$C_RST"
echo "  (dry-run=$DRY_RUN  mirror=$MIRROR  stamp=$STAMP)"
echo

command -v curl >/dev/null 2>&1 || {
  warn "缺 curl，先装一下"
  need_sudo && run "安装 curl" sudo apt-get install -y curl
}

# ============================================================================
#  阶段 1：mirror —— 换国内源
# ============================================================================
stage_mirror() {
  head2 "阶段 mirror：换国内源"

  # 实测：阿里云镜像在部分网络下不通（返回 502），所以不作为默认
  case "$MIRROR" in
    tuna)    APT_URL="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/";           PIP_URL="https://pypi.tuna.tsinghua.edu.cn/simple"; ROS2_URL="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu" ;;
    ustc)    APT_URL="https://mirrors.ustc.edu.cn/ubuntu/";                    PIP_URL="https://mirrors.ustc.edu.cn/pypi/simple";  ROS2_URL="https://mirrors.ustc.edu.cn/ros2/ubuntu" ;;
    huawei)  APT_URL="https://mirrors.huaweicloud.com/ubuntu/";                PIP_URL="https://repo.huaweicloud.com/repository/pypi/simple"; ROS2_URL="https://mirrors.huaweicloud.com/ros2/ubuntu" ;;
    tencent) APT_URL="https://mirrors.cloud.tencent.com/ubuntu/";              PIP_URL="https://mirrors.cloud.tencent.com/pypi/simple"; ROS2_URL="https://mirrors.cloud.tencent.com/ros2/ubuntu" ;;
    aliyun)  bad "aliyun 镜像在本项目实测网络中不可达（502）。请改用 tuna/ustc/huawei/tencent。"; return 1 ;;
    *)       bad "未知镜像：$MIRROR（可选 tuna/ustc/huawei/tencent）"; return 1 ;;
  esac
  info "apt 源   → $APT_URL"
  info "pip 源   → $PIP_URL"
  info "ROS2 源  → $ROS2_URL"

  # --- 1.1 先备份任何会被改动的文件 ---
  local BK="$REAL_HOME/apt-backup-$STAMP"
  if [ "$DRY_RUN" != "1" ]; then
    mkdir -p "$BK"
    for f in /etc/apt/sources.list /etc/apt/sources.list.d/ros2.list; do
      [ -f "$f" ] && cp "$f" "$BK/$(echo "$f" | tr '/' '_')"
    done
    # deb822 格式（22.04 之后某些镜像用它）
    [ -f /etc/apt/sources.list.d/ubuntu.sources ] && \
      cp /etc/apt/sources.list.d/ubuntu.sources "$BK/ubuntu.sources"
    ok "已备份到：$BK"
  fi

  need_sudo || return 1

  # --- 1.2 apt 主源 ---
  # 判断到底哪个文件在生效：sources.list 里有没有 deb 行
  local USE_DEB822=0
  if [ -f /etc/apt/sources.list.d/ubuntu.sources ] && \
     ! grep -qE '^[[:space:]]*deb ' /etc/apt/sources.list 2>/dev/null; then
    USE_DEB822=1
  fi

  if [ "$USE_DEB822" = "1" ]; then
    info "检测到 deb822 格式（ubuntu.sources），改这个文件"
    if [ "$DRY_RUN" != "1" ]; then
      sudo tee /etc/apt/sources.list.d/ubuntu.sources >/dev/null <<EOF
Types: deb
URIs: $APT_URL
Suites: jammy jammy-updates jammy-backports jammy-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
      ok "已写入 /etc/apt/sources.list.d/ubuntu.sources"
    else
      printf '%s  [dry-run] 写入 /etc/apt/sources.list.d/ubuntu.sources%s\n' "$C_DIM" "$C_RST"
    fi
  else
    if [ "$DRY_RUN" != "1" ]; then
      sudo tee /etc/apt/sources.list >/dev/null <<EOF
# 由 dexterous-hand/tools/ubuntu_bootstrap.sh 生成（mirror=$MIRROR）
deb ${APT_URL} jammy           main restricted universe multiverse
deb ${APT_URL} jammy-updates   main restricted universe multiverse
deb ${APT_URL} jammy-backports main restricted universe multiverse
deb ${APT_URL} jammy-security  main restricted universe multiverse
EOF
      ok "已写入 /etc/apt/sources.list"
    else
      printf '%s  [dry-run] 写入 /etc/apt/sources.list%s\n' "$C_DIM" "$C_RST"
    fi
  fi

  # --- 1.3 ROS2 源 + 密钥 ---
  # 密钥优先用镜像站的，而不是 keyserver.ubuntu.com（国内基本连不上）
  if [ "$DRY_RUN" != "1" ]; then
    if curl -fsSL --max-time 60 "$ROS2_URL/ros2-archive-keyring.gpg" \
         -o /tmp/ros2-keyring.gpg 2>/dev/null; then
      sudo install -m 0644 -o root -g root /tmp/ros2-keyring.gpg \
        /usr/share/keyrings/ros-archive-keyring.gpg
      rm -f /tmp/ros2-keyring.gpg
      ok "ROS2 密钥已安装（来自镜像站）"
    else
      warn "镜像站没有 ros2-archive-keyring.gpg，跳过；apt update 报 NO_PUBKEY 时手处理"
    fi

    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $ROS2_URL jammy main" | \
      sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    ok "已写入 /etc/apt/sources.list.d/ros2.list"
  fi

  # --- 1.4 pip 源 ---
  local PIPCONF="$REAL_HOME/.pip/pip.conf"
  if [ "$DRY_RUN" != "1" ]; then
    mkdir -p "$REAL_HOME/.pip"
    cat > "$PIPCONF" <<EOF
[global]
index-url = $PIP_URL
timeout = 60
retries = 5
EOF
    [ "$REAL_USER" != "$(id -un)" ] && chown -R "$REAL_USER" "$REAL_HOME/.pip" 2>/dev/null
    ok "pip 源已写入 $PIPCONF（timeout 已调大，减少假超时）"
  fi

  # --- 1.5 生效 ---
  need_sudo || return 1
  if ! run "apt update" sudo apt-get update; then
    warn "apt update 有报错。常见原因："
    info "  NO_PUBKEY        → ROS2 密钥没进来（见 docs/12 §2.5）"
    info "  Hash Sum mismatch → 镜像同步中，过几分钟重试"
    return 1
  fi
  ok "换源完成"
}

# ============================================================================
#  阶段 2：kernel —— HWE 内核
# ============================================================================
stage_kernel() {
  head2 "阶段 kernel：HWE 内核"

  local cur; cur="$(uname -r)"
  info "当前内核：$cur"
  case "$cur" in
    5.15.*)  warn "5.15 是 22.04 出厂内核，MT7922 无线网卡支持差 → 建议升级" ;;
    5.1[0-4].*|5.[0-9].*|4.*|3.*|2.*)
             warn "内核 $cur 偏老，MT7922 等新硬件支持可能不全 → 建议升级" ;;
    6.[0-9]*|[7-9].*|1[0-9][0-9].*)
             ok "内核 $cur（≥6.x，新硬件支持足够）" ;;
    *)       warn "内核 $cur —— 无法判断，HWE 内核装上不会更差" ;;
  esac

  need_sudo || return 1
  run "安装 HWE 内核 linux-generic-hwe-22.04" \
      sudo apt-get install -y linux-generic-hwe-22.04 || return 1

  warn "★ 需要重启才能生效，本脚本不会替你重启。"
  info "重启命令：sudo reboot"
  info "重启后确认：uname -r  （期望 6.8.x）"
}

# ============================================================================
#  阶段 3：nvidia —— 显卡驱动 + PRIME
# ============================================================================
stage_nvidia() {
  head2 "阶段 nvidia：显卡驱动与双显卡"

  # 先看有没有 GPU
  if command -v lspci >/dev/null 2>&1 && lspci 2>/dev/null | grep -qi 'vga\|3d controller'; then
    info "检测到的显示设备："
    lspci 2>/dev/null | grep -i 'vga\|3d controller' | sed 's/^/      /'
  fi

  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA 驱动已就绪："
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | sed 's/^/      /'
  else
    need_sudo || return 1
    run "安装 ubuntu-drivers-common" sudo apt-get install -y ubuntu-drivers-common
    info "系统推荐的驱动："
    ubuntu-drivers devices 2>/dev/null | grep -i 'driver\|recommended' | sed 's/^/      /' || true

    if ! run "自动安装推荐驱动（ubuntu-drivers install）" sudo ubuntu-drivers install; then
      warn "自动安装失败，改用显式版本 550"
      run "安装 nvidia-driver-550" sudo apt-get install -y nvidia-driver-550 || return 1
    fi

    warn "★ 需要重启，且第一次重启会弹 MOK 注册界面（蓝底白字）："
    info "  1) 装驱动时它会让你设一个一次性密码（记好）"
    info "  2) 重启后出现 'Perform MOK management' → 选 Enroll MOK"
    info "     → Continue → Yes → 输入那个密码 → Reboot"
    info "  3) 错过了就执行："
    info "     sudo mokutil --import /var/lib/shim-signed/mok/MOK.der"
    info "  不想折腾就在 BIOS 里关掉 Secure Boot。"
  fi

  # PRIME 模式
  if command -v prime-select >/dev/null 2>&1; then
    info "当前 PRIME 模式：$(prime-select query 2>/dev/null)"
    if [ "$(prime-select query 2>/dev/null)" != "nvidia" ]; then
      warn "做仿真建议切到独显（核显渲染会又慢又花屏）"
      if confirm "现在执行 sudo prime-select nvidia？（需重启生效）"; then
        run "切换 PRIME 到 nvidia" sudo prime-select nvidia
        warn "★ 需要重启：sudo reboot"
      fi
    else
      ok "已经是 nvidia 模式"
    fi
  else
    warn "没找到 prime-select（可能是单显卡机器或驱动没装好）"
  fi
}

# ============================================================================
#  阶段 4：ros2 —— 补装 desktop（含 RViz2）
# ============================================================================
stage_ros2() {
  head2 "阶段 ros2：ROS2 Humble 补全（含 RViz2）"

  if [ ! -d /opt/ros/humble ]; then
    bad "/opt/ros/humble 不存在 —— ROS2 Humble 好像没装（或装在别处）"
    info "确认一下：ls /opt/ros/"
    return 1
  fi
  ok "找到 /opt/ros/humble"

  if [ -x /opt/ros/humble/bin/rviz2 ] || command -v rviz2 >/dev/null 2>&1; then
    ok "RViz2 已存在"
  else
    warn "没有 rviz2 —— 很可能是当初装成了 ros-humble-ros-base（不含 GUI 工具）"
    info "下面会补装 ros-humble-desktop"
  fi

  need_sudo || return 1

  # 包名已在清华 ROS2 索引（7858 个包）里逐个核对过
  local PKGS=(
    ros-humble-desktop
    ros-humble-rviz2
    ros-humble-rqt ros-humble-rqt-graph ros-humble-rqt-common-plugins
    ros-humble-plotjuggler-ros
    ros-humble-xacro
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
    ros-humble-robot-state-publisher
    ros-humble-tf2-tools
    ros-humble-rosbag2
    ros-humble-usb-cam
    python3-colcon-common-extensions python3-argcomplete
  )

  local missing=()
  for p in "${PKGS[@]}"; do
    if dpkg -s "$p" >/dev/null 2>&1; then
      info "已有：$p"
    else
      missing+=("$p")
    fi
  done

  if [ "${#missing[@]}" = "0" ]; then
    ok "要装的包都已经装过了，跳过"
  else
    info "待装 ${#missing[@]} 个包：${missing[*]}"
    run "apt install ROS2 组件" sudo apt-get install -y "${missing[@]}" || {
      warn "有包装失败。若报 '无法定位软件包'，多半是 ROS2 源没配好："
      info "  cat /etc/apt/sources.list.d/ros2.list"
      info "  sudo apt update && sudo apt install -y ros-humble-desktop"
    }
  fi

  # source 写进 .bashrc（先查重，避免重复追加）
  local RC="$REAL_HOME/.bashrc"
  if [ "$DRY_RUN" != "1" ]; then
    if ! grep -q 'source /opt/ros/humble/setup.bash' "$RC" 2>/dev/null; then
      printf '\n# --- ROS 2 Humble (added by ubuntu_bootstrap.sh) ---\nsource /opt/ros/humble/setup.bash\n' >> "$RC"
      ok "已把 source /opt/ros/humble/setup.bash 写入 $RC"
    else
      info ".bashrc 里已有 ROS2 source，跳过"
    fi
    if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && \
       ! grep -q 'colcon-argcomplete' "$RC" 2>/dev/null; then
      printf 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\n' >> "$RC"
      ok "已启用 colcon 命令补全"
    fi
  fi

  warn "★ 别忘了 ROS_DOMAIN_ID：连机器人时两边必须一致，否则看不到对方话题"
  info "  官方规则：手柄启动 = 序号+30；APP 启动 = 22。详见 docs/11。"
}

# ============================================================================
#  阶段 5：mujoco —— 依赖 + pip 安装 + 自检
# ============================================================================
stage_mujoco() {
  head2 "阶段 mujoco：MuJoCo 环境"

  need_sudo || return 1
  # 官方 README 明确要求 libglfw3-dev；其余两个是离屏渲染需要的
  run "安装 MuJoCo 系统依赖" \
      sudo apt-get install -y libglfw3-dev libglew-dev libosmesa6-dev || return 1

  if [ "$DRY_RUN" = "1" ]; then
    printf '%s  [dry-run] pip install mujoco%s\n' "$C_DIM" "$C_RST"
    return 0
  fi

  # pip 已换源的话这一步很快
  run "pip install mujoco" python3 -m pip install --user -U mujoco || {
    warn "pip 装失败。若网络原因，重试一次；或确认 ~/.pip/pip.conf 已就绪"
    return 1
  }

  # 自检
  if python3 - <<'PY'
import sys
try:
    import mujoco
except Exception as e:
    print("IMPORT_FAIL:", e); sys.exit(1)
print("mujoco", mujoco.__version__)
m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><body><geom type='sphere' size='0.1'/></body></worldbody></mujoco>")
d = mujoco.MjData(m)
for _ in range(100):
    mujoco.mj_step(m, d)
print("physics OK  nq=%d nv=%d" % (m.nq, m.nv))
PY
  then
    ok "MuJoCo 导入 + 物理引擎自检通过"
  else
    bad "MuJoCo 自检失败"
    return 1
  fi

  info "查看器需要图形界面；SSH 无头环境请设：export MUJOCO_GL=egl（不行换 osmesa）"
  info "想看官方模型：见 docs/12 §5.2"
}

# ============================================================================
#  阶段 6：bxi —— 官方仿真环境
# ============================================================================
stage_bxi() {
  head2 "阶段 bxi：官方 BXI 仿真环境"

  local DEST="/opt/bxi/bxi_ros2_pkg"

  if [ "$DRY_RUN" = "1" ]; then
    # dry-run 下不真去检查目录（否则会给出误导性的"安装不完整"）
    info "目标目录：$DEST"
    info "动作：mkdir -p /opt/bxi → chown 给当前用户 → git clone --depth 1 或 scp 从机器人拷"
    info "校验：\$DEST/setup.bash 与 \$DEST/share/mujoco"
    return 0
  fi

  if [ -d "$DEST" ]; then
    ok "已存在：$DEST"
    info "大小：$(du -sh "$DEST" 2>/dev/null | cut -f1)"
  else
    need_sudo || return 1
    run "创建 /opt/bxi" sudo mkdir -p /opt/bxi
    run "chown /opt/bxi 给当前用户" sudo chown -R "$(id -un)":"$(id -gn)" /opt/bxi

    info "两条路："
    info "  A) 从机器人拷（最快）scp -r bxi@<机器人域名>:/opt/bxi/bxi_ros2_pkg /opt/bxi/"
    info "  B) git clone（95MB）"
    if confirm "现在用 B 方式 git clone 吗？（约 95MB，走 ghfast.top 加速）"; then
      run "git clone --depth 1 bxi_ros2_pkg" \
        git clone --depth 1 \
          "https://ghfast.top/https://github.com/bxirobotics/bxi_ros2_pkg.git" \
          "$DEST" || return 1
    else
      warn "已跳过下载。请手工拷到 $DEST 后再跑一次本阶段。"
      return 0
    fi
  fi

  if [ -f "$DEST/setup.bash" ]; then
    ok "setup.bash 存在"
    if [ -d "$DEST/share/mujoco" ]; then
      ok "MuJoCo 仿真包存在：$DEST/share/mujoco"
    else
      warn "没看到 share/mujoco，可能是精简版本"
    fi
  else
    bad "$DEST/setup.bash 不存在，安装可能不完整"
    return 1
  fi

  echo
  info "激活方式："
  info "  source /opt/bxi/bxi_ros2_pkg/setup.bash"
  info "下一步去拿控制器示例（63MB 预编译包），见 docs/12 §5.1 ③"
  info "  然后 bash build.sh && source ./install/setup.bash"
  info "  跑仿真：ros2 launch bxi_example_py_elf3 example_launch_demo.py"
  warn "⚠️ 带 hw 的 launch 文件是【真机】！仿真是不带 hw 的那个。"
}

# ============================================================================
#  阶段 7：verify —— 体检
# ============================================================================
stage_verify() {
  head2 "阶段 verify：环境体检"

  local pass=0 fail=0 warnc=0

  # 逐项手写，输出更清楚
  echo
  # 1 内核
  local k; k="$(uname -r)"
  case "$k" in
    5.15.*)  warn "内核 $k —— 偏老，MT7922 无线网卡可能不稳（建议 HWE）"; warnc=$((warnc+1)) ;;
    5.1[0-4].*|5.[0-9].*|4.*|3.*|2.*)
             warn "内核 $k —— 偏老，建议 HWE"; warnc=$((warnc+1)) ;;
    6.[0-9]*|[7-9].*|1[0-9][0-9].*)
             ok  "内核 $k"; pass=$((pass+1)) ;;
    *)       warn "内核 $k —— 无法判断"; warnc=$((warnc+1)) ;;
  esac

  # 2 有线网卡
  if ip -br link 2>/dev/null | grep -qE '^(en|eth)'; then
    ok "有线网卡：$(ip -br link 2>/dev/null | grep -E '^(en|eth)' | awk '{print $1, $2}' | tr '\n' ' ')"
    pass=$((pass+1))
  else
    warn "没看到有线网卡（en*/eth*）"; warnc=$((warnc+1))
  fi

  # 3 无线
  if command -v nmcli >/dev/null 2>&1; then
    if nmcli device 2>/dev/null | grep -q 'wifi.*连接\|wifi.*connected'; then
      ok "无线已连接：$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null | head -1)"
      pass=$((pass+1))
    elif nmcli device 2>/dev/null | grep -q 'wifi'; then
      warn "有无线设备但未连接"; warnc=$((warnc+1))
    else
      bad "完全没有无线设备（MT7922 没被识别 → 升 HWE 内核）"; fail=$((fail+1))
    fi
  else
    warn "没有 nmcli，跳过无线检查"; warnc=$((warnc+1))
  fi

  # 4 NVIDIA
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA：$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1)"
    pass=$((pass+1))
  else
    bad "nvidia-smi 不可用（驱动没装好 / Secure Boot 没注册 MOK）"; fail=$((fail+1))
  fi

  # 5 PRIME
  if command -v prime-select >/dev/null 2>&1; then
    local pm; pm="$(prime-select query 2>/dev/null)"
    if [ "$pm" = "nvidia" ]; then ok "PRIME 模式：nvidia（适合跑仿真）"; pass=$((pass+1))
    else warn "PRIME 模式：$pm —— 仿真可能跑在核显上，很卡"; warnc=$((warnc+1)); fi
  else
    warn "没有 prime-select"; warnc=$((warnc+1))
  fi

  # 6 OpenGL 渲染器
  if command -v glxinfo >/dev/null 2>&1; then
    local r; r="$(glxinfo 2>/dev/null | grep -i 'OpenGL renderer' | head -1)"
    case "$r" in
      *NVIDIA*) ok "$r"; pass=$((pass+1)) ;;
      *)        warn "$r  ← 不是 NVIDIA，渲染会慢"; warnc=$((warnc+1)) ;;
    esac
  else
    warn "缺 glxinfo（可装：sudo apt install mesa-utils）"; warnc=$((warnc+1))
  fi

  # 7 ROS2
  if [ -f /opt/ros/humble/setup.bash ]; then
    ok "ROS2 Humble 已安装：/opt/ros/humble"
    pass=$((pass+1))
  else
    bad "找不到 /opt/ros/humble"; fail=$((fail+1))
  fi

  # 8 RViz2
  if [ -x /opt/ros/humble/bin/rviz2 ]; then
    ok "RViz2 已安装"
    pass=$((pass+1))
  else
    bad "没有 RViz2（多半装的是 ros-base）→ 跑 --stage ros2"; fail=$((fail+1))
  fi

  # 9 MuJoCo
  local mv
  if mv="$(python3 -c 'import mujoco;print(mujoco.__version__)' 2>/dev/null)"; then
    ok "MuJoCo $mv"
    pass=$((pass+1))
  else
    bad "MuJoCo 未安装（或不在当前 python 环境）→ 跑 --stage mujoco"; fail=$((fail+1))
  fi

  # 10 MuJoCo GL 后端
  if python3 -c "import mujoco" >/dev/null 2>&1; then
    info "MuJoCo 渲染后端：MUJOCO_GL=${MUJOCO_GL:-（未设，默认 glfw 需图形界面）}"
  fi

  # 11 BXI 仿真包
  if [ -d /opt/bxi/bxi_ros2_pkg/share/mujoco ]; then
    ok "官方仿真环境 /opt/bxi/bxi_ros2_pkg"
    pass=$((pass+1))
  else
    warn "没看到 /opt/bxi/bxi_ros2_pkg（要跑整机仿真就跑 --stage bxi）"; warnc=$((warnc+1))
  fi

  # 12 Domain ID
  info "当前 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-（未设，默认 0）}  ← 连机器人时要和机器人一致"

  echo
  printf '%s  ---- 体检结果：通过 %d  警告 %d  失败 %d ----%s\n' \
    "$C_BLU" "$pass" "$warnc" "$fail" "$C_RST"
  echo
  if [ "$fail" -gt 0 ]; then
    warn "有失败项，按上面的提示逐个处理；对照 docs/12 §7 清单与 §8 故障速查"
    return 1
  fi
  ok "没有致命问题。"
}

# ============================================================================
#  执行
# ============================================================================
RC=0
for s in "${STAGE_ARR[@]}"; do
  case "$s" in
    mirror) stage_mirror || RC=1 ;;
    kernel) stage_kernel || RC=1 ;;
    nvidia) stage_nvidia || RC=1 ;;
    ros2)   stage_ros2   || RC=1 ;;
    mujoco) stage_mujoco || RC=1 ;;
    bxi)    stage_bxi    || RC=1 ;;
    verify) stage_verify || RC=1 ;;
  esac
done

head2 "结束"
if [ "$DRY_RUN" = "1" ]; then
  warn "以上是 dry-run，什么都没改。去掉 --dry-run 再跑一次才会真正执行。"
fi
if [ "$RC" = "0" ]; then
  ok "本次阶段全部完成。"
else
  bad "有阶段报了错，往上翻看具体信息。"
fi
echo
info "文档：dexterous-hand/docs/12-Ubuntu22.04环境搭建与MuJoCo仿真-笔记本版.md"
info "体检：bash tools/ubuntu_bootstrap.sh --stage verify"
echo
exit "$RC"

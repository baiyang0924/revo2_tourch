# Git 协作与仓库规范

> 写给「没用过 Git、但今天就要把文件传到公司仓库」的人。全文可以照着做。
> 读完你能独立完成：拿到仓库 → 改文件 → 提交 → 推送 → 发起合并请求。

---

## 第 0 章 先建立三个认知

### 0.1 Git 不是网盘

网盘是「把文件同步上去」。Git 是**给项目存快照**：

- 每次「提交（commit）」= 给整个项目拍一张照，并写一句说明。
- 你可以随时回到任何一张照片，也能看到某一行是谁在哪次改的。
- 所以：**提交要小、要勤、说明要准**。一次提交一件事。

### 0.2 四个区域（这是理解 Git 的关键）

```
你的电脑                                           GitHub
┌──────────────────────────────────┐          ┌──────────────┐
│  工作区          暂存区     本地仓库 │  push →  │  远程仓库     │
│ (改文件的地方) → (待提交) → (已存档) │ ← fetch  │ origin/master│
└──────────────────────────────────┘          └──────────────┘
        │              │            │
     git add       git commit    git push
```

| 名字 | 是什么 | 类比 |
|---|---|---|
| 工作区 Working Directory | 你正在编辑的文件 | 桌上的草稿 |
| 暂存区 Staging Area / Index | 「这次要提交哪些改动」的清单 | 装进信封的纸 |
| 本地仓库 Local Repository | 所有历史快照，存在 `.git/` 里 | 你的档案柜 |
| 远程仓库 Remote（通常叫 `origin`） | GitHub 上的那台服务器 | 公司档案室 |

**记住这条主线**：改文件 → `git add` → `git commit` → `git push`。

### 0.3 你要建立的两个习惯

1. **提交前先 `git status`**，看清楚自己到底动了什么。
2. **推送前先 `git pull`**，避免和同事的改动打架。

### 0.4 本地改了文件，GitHub 上会自动变吗？——**不会**

这是新手最容易误解的一点，明确写一遍：

```
你在本地文件夹里改 / 新建 / 删一个文件
        ▼
   ❌ GitHub 上不会有任何变化
        ▼
必须手动走完这三步，才会上去：
   ① git add     把改动装进「信封」
   ② git commit  在本地存一张快照（此时仍只在你电脑上）
   ③ git push    才真正上传到 GitHub
```

| 说法 | 对不对 |
|---|---|
| 本地文件夹改了，GitHub 自动同步 | ❌ **错**。Git 不是网盘/OneDrive，没有后台自动同步 |
| 保存文件（Ctrl+S）就等于提交 | ❌ 错。`Ctrl+S` 只改了工作区，连本地仓库都没进 |
| `git commit` 完，同事就能看到了 | ❌ 错。commit 只存在你电脑的 `.git/` 里，**必须 push** |
| 忘了 push 会怎样 | 你的成果只在自己机器上。换电脑、重装系统、删错目录都会丢 |
| 怎么确认真的推上去了 | 刷新 GitHub 网页看文件在不在，或 `git log origin/master --oneline -3` |

**所以有两条纪律：**

1. **不用等"全部做完"才推。** 一个改动讲清楚一件事就推一次，
   推完就是一个可回溯的存档点，比攒一大坨再推安全得多。
2. **收工前 push 一次。** 只在本地的东西不算"存好了"。

> 本仓库提供了一个傻瓜脚本，把 `add → commit → pull → push` 合成一步：
>
> ```bash
> cd <仓库>/dexterous-hand
> ./tools/git_sync.sh "docs: 补充接线照片"
> ```
>
> 它会先做安全检查（大文件、疑似密钥、提交范围），再让你确认改动清单，
> 最后才提交推送 —— 每一步都会告诉你结果，看不懂 Git 也能用。
> 它**只操作 `dexterous-hand/` 目录**，不会误提交同事的文件。

---

## 第 1 章 一次性环境准备（做完以后再也不用做）

### 1.1 确认 Git 已安装

在**任意目录**打开 **Git Bash**（开始菜单搜 "Git Bash"），执行：

```bash
git --version
```

看到 `git version 2.x.x` 就 OK。没有的话去 <https://git-scm.com/download/win> 装。

> 后面所有命令都在 Git Bash 里执行，不要用 CMD 或 PowerShell，
> 三者的命令写法不一样，混用会出各种奇怪问题。

### 1.2 配置提交身份（必做，只做一次）

Git 会把「谁提交的」永久写进历史，所以必须先告诉它你是谁。

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

建议用**公司邮箱**，主管一眼能看出是谁提交的。例如：

```bash
git config --global user.name "Xiao Tao"
git config --global user.email "tao@yourcompany.com"
```

> 想用 GitHub 的隐私邮箱（不暴露真实邮箱），把邮箱换成
> `你的用户名@users.noreply.github.com` 即可。

顺手做两个推荐配置：

```bash
# 让 git 记住你的密码/密钥，不用每次都输
git config --global credential.helper manager

# 把默认分支名设为 master（公司仓库用的是 master，保持一致免得混乱）
git config --global init.defaultBranch master

# 让中文文件名正常显示（不显示成 \344\270\255 这种）
git config --global core.quotepath false

# 查看当前所有配置，确认写进去了
git config --global --list
```

### 1.3 配置 SSH 密钥（推荐，一次配好终身免密）

有两种方式和 GitHub 通信，**推荐 SSH**：

| | SSH 密钥 | HTTPS + Token |
|---|---|---|
| 要不要输密码 | 不用，配一次就行 | 要，且 Token 会过期 |
| 安全性 | 高（私钥留在本地） | 中 |
| 适合 | **长期开发，推荐** | 临时、公司限制 SSH 端口时 |

**生成密钥：**

```bash
# 把邮箱换成你的公司邮箱
ssh-keygen -t ed25519 -C "tao@yourcompany.com"
```

- 连按三次回车（第一问是保存路径，默认即可；后两问是密码，留空表示不再额外设密码）。
- 生成后会得到两个文件，**默认在 `C:\Users\<你的用户名>\.ssh\`**：
  - `id_ed25519` —— **私钥，绝对不能给任何人、绝对不能提交到仓库**
  - `id_ed25519.pub` —— **公钥，要交给 GitHub**

**查看公钥内容并复制：**

```bash
cat ~/.ssh/id_ed25519.pub
```

会输出一整行，形如 `ssh-ed25519 AAAAC3Nza... tao@yourcompany.com`。
**从 `ssh-ed25519` 一直选到结尾**，全部复制。

**粘贴到 GitHub：**

1. 打开 GitHub → 右上角头像 → **Settings**
2. 左侧 **SSH and GPG keys** → 绿色按钮 **New SSH key**
3. Title 随便写（例如 `工作电脑-Windows`），Key type 保持 `Authentication Key`
4. Key 框里粘贴刚复制的公钥 → **Add SSH key**

**测试（在 Git Bash 里）：**

```bash
ssh -T git@github.com
```

第一次会问 `Are you sure you want to continue connecting?`，输入 `yes` 回车。
看到 `Hi <你的用户名>! You've successfully authenticated...` 就成功了。

> ⚠️ 如果提示 `Permission denied (publickey)`：公钥没粘对，或粘的时候多带了换行。
> 重新 `cat` 一次、完整复制再粘一遍。

### 1.4 如果只能用 HTTPS + Token

公司网络封了 22 端口时才需要这样。

1. GitHub → Settings → **Developer settings** → **Personal access tokens** →
   **Tokens (classic)** → **Generate new token (classic)**
2. 勾选权限：至少有 **`repo`**（读写私有仓库）
3. 设置有效期（建议 90 天），生成后**立刻复制保存**——只显示一次
4. 之后 `git push` 提示输密码时，**密码处粘贴这个 Token**，不是登录密码

---

## 第 2 章 第一次上手：把仓库拿到本地

你的公司仓库已经建好了，先拿到它的地址：

> 仓库页面 → 绿色 **Code** 按钮 → 选 **SSH** → 复制那一行，形如
> `git@github.com:Bake-Humanoid/xxx.git`

### 方案 A：仓库是空的（推荐从这个开始）

```bash
# 换到你平时放代码的目录
cd /d/Desktop/机器人实习/灵巧手

# 把已经准备好的本地目录变成 git 仓库（这个目录里已有 .git）
# 如果目录还没初始化过，执行：
git init

# 关联远程仓库
git remote add origin git@github.com:Bake-Humanoid/elf3-humanoid.git

# 确认关联成功
git remote -v
```

### 方案 B：仓库里已经有内容了（本项目就是这种情况）

`elf3-humanoid` 是**多人共用的主仓库**，根目录属于公共区域。
你必须先把它 clone 下来，再把资料放进**属于你的子目录**——
既不要往根目录直接丢文件，也不要自己 `git init`。

```bash
cd /d/Desktop/机器人实习/灵巧手
git clone git@github.com:Bake-Humanoid/elf3-humanoid.git
cd elf3-humanoid

# 先看看里面已经有什么、别人是怎么分目录的
ls
```

**目录怎么定**，按这个优先级：

1. 已经有同类目录（例如 `hardware/`、`docs/`）→ 放进它下面的子目录
2. 按模块分 → 新建一个能自解释的目录名，例如 `dexterous-hand/`
3. 按人分（有些团队这么做）→ 用你的花名

目录名用小写英文 + 短横线，别用中文和空格 —— 中文名在别的系统或脚本里容易出问题。

```bash
# 建你的子目录
mkdir -p elf3-humanoid/dexterous-hand

# 把本地资料拷进去（必须排除自带的 .git，否则会变成嵌套仓库）
cd /d/Desktop/机器人实习/灵巧手
tar -C elf3-revo2-hand-integration --exclude='./.git' -cf - . \
  | tar -C elf3-humanoid/dexterous-hand -xf -
```

> 上面这一整段（克隆 → 打印现有结构 → 确认目录名 → 拷贝 → 提交 → 推送）
> 已封装成脚本，直接跑即可：
>
> ```bash
> cd /d/Desktop/机器人实习/灵巧手/elf3-revo2-hand-integration
> ./tools/join_main_repo.sh
> ```
>
> 它的关键是**只提交你自己的那一个子目录**，不会碰到同事的文件。

### 首次推送的三条命令

```bash
# 1. 只把自己的目录放进暂存区（多人共用仓库，绝不要用 git add -A）
git add -- dexterous-hand

# 2. 存档并写说明
git commit -m "docs: 新增 Revo2 触觉版灵巧手接入与整机联调资料"

# 3. 推送到远程 master 分支（-u 表示以后直接 git push 就行）
git push -u origin master
```

推送成功后再刷一下 GitHub 页面，文件就上去了。

> **为什么不能 `git add -A`**：在共用仓库里，这个命令会把工作区里**所有**改动
> 一次性提交上去 —— 包括你拉下来但没动过的同事文件、临时脚本、日志，
> 甚至别人误留的敏感文件。养成 `git add -- <你的目录>` 的习惯。

### 方案 B+：网络慢时用稀疏检出（本项目当前采用）

公司仓库完整体积约 11 MB、465 个文件，里面还有大量 STL 模型和 npz 动作数据。
如果你的网络只有几十 KB/s，完整克隆会反复中断，报：

```
fatal: fetch-pack: invalid index-pack output
```

这不是仓库坏了，是传输中途断了。**稀疏检出（sparse-checkout）** 可以让 Git
只下载你需要的那个目录，其余文件完全不下载：

```bash
cd /d/Desktop/机器人实习/灵巧手

# ① 只取提交树，不取文件内容（实测 .git 只有约 100 KB，几秒完成）
git clone --filter=blob:none --no-checkout --depth 1 --single-branch \
  git@github.com:Bake-Humanoid/elf3-humanoid.git elf3-humanoid

cd elf3-humanoid

# ② 设定「只检出 dexterous-hand」这一块
git sparse-checkout set --cone dexterous-hand

# ③ 完成检出
git checkout master

# 确认：工作区只有顶层几个小文件 + 你的目录，status 是干净的
git status
```

想换目录名就改 `sparse-checkout set` 后面的名字。想看同事的文件时，把目录加进来即可：

```bash
git sparse-checkout add docs          # 把 docs 也检出到本地
git sparse-checkout list              # 看当前检出了哪些
```

> **⚠ 千万不要跳过第 ② 步。** 在 `--no-checkout` 的克隆上直接 `git add` 再提交，
> 索引是空的，Git 会把你这次提交理解成 **「删除了全部 465 个文件」** ——
> 这是能毁掉整个仓库的操作。稀疏检出会把其余文件标记为 `skip-worktree`，
> 这样它们才不会被误判为删除。

---

## 第 3 章 日常三件事：改 → 存 → 推

这是你 90% 的时间只会用到的操作。

```bash
# ── 第 0 步：先看看自己动了什么 ─────────────────
git status

# ── 第 1 步：把改动加入暂存区 ───────────────────
# 共用仓库的铁律：限定范围，绝不 git add -A
git add -- dexterous-hand     # 自己的整个目录（在仓库根运行）
git add docs/01-接线手册.md    # 只加某个文件
git add docs/                 # 只加某个目录

# ── 第 2 步：存档并写说明 ───────────────────────
git commit -m "docs: 补充左手 CAN5 接线照片与上电顺序"

# ── 第 3 步：拉一下同事的新提交，再推 ───────────
git pull --rebase             # 有冲突的话看第 6 章
git push
```

### 3.1 把「拉 + 提交 + 推」合成一条命令

仓库里已经准备了脚本，见 `tools/git_sync.sh`：

```bash
./tools/git_sync.sh "docs: 补充左手接线照片"
```

它的实际动作就是上面三章命令的合集，外加一层安全防护
（**拒绝提交大文件、拒绝提交密钥、提交前强制 `git status` 让你确认**）。

### 3.2 查看历史 / 看某人改了啥

```bash
git log --oneline -10          # 最近 10 次提交，一行一条
git log --oneline --graph      # 带分支图形
git show <提交编号>            # 看某次提交具体改了什么
git blame docs/01-接线手册.md  # 看每一行是谁哪次改的
git diff                       # 看当前还没 add 的改动
git diff --staged              # 看已经 add 但还没 commit 的改动
```

---

## 第 4 章 提交信息规范（公司仓库的门面）

格式：

```
<类型>: <一句话说明>
```

**类型只用这几个：**

| 类型 | 什么时候用 | 例子 |
|---|---|---|
| `feat` | 新增功能、新增文档 | `feat: 增加灵巧手捏合动作脚本` |
| `fix` | 修 bug、改错 | `fix: 修正右手默认 ID 写成 126 的错误` |
| `docs` | 只改文档 | `docs: 补充 485 接线排查步骤` |
| `refactor` | 重构，不改功能 | `refactor: 抽出 Modbus 帧编解码为独立函数` |
| `chore` | 杂项：配置、依赖、目录 | `chore: 更新 .gitignore 忽略 install 目录` |

**写说明的四个要求：**

1. **用动词开头**，说清「做了什么」：`fix: 修正...` 而不是 `修改了一下`
2. **一次提交只做一件事**：不要「改了文档+修了 bug+加了脚本」混在一起
3. **不要写「更新」「提交」「test」这种废话**——看历史的人无法从中获得信息
4. **中英文都行，但全文统一**（建议中文，本仓库统一中文）

**反面例子 vs 正面例子：**

| ❌ 差 | ✅ 好 |
|---|---|
| `update` | `docs: 补充触觉版与基础版接口差异说明` |
| `修改` | `fix: 修正上电顺序中校准步骤的位置` |
| `完成了灵巧手相关的很多东西` | `feat: 新增灵巧手自检脚本与接线文档` |
| `test` | `chore: 调整 .gitignore 忽略 build 产物` |

---

## 第 5 章 分支策略

### 5.1 三条规则

1. **`master` 分支始终保持可用**：任何时候拉下来都能用，不能是半成品。
2. **所有开发在 `feature/<主题>` 分支上做**，做完提 PR 合回 `master`。
3. **一个分支只做一件事**，合完就删。

### 5.2 完整流程示例

假设你要写灵巧手抓取脚本：

```bash
# ① 从最新的 master 开一个新分支
git checkout master
git pull
git checkout -b feature/revo2-grasp-script

# ② 干活，反复提交
git add -- dexterous-hand/src/revo2_standalone/03_grasp_demo.py
git commit -m "feat: 新增灵巧手抓取动作示例脚本"

git add -- dexterous-hand/docs/02-Windows单机调试指南.md
git commit -m "docs: 补充抓取脚本的运行前提说明"

# ③ 推送这个分支
git push -u origin feature/revo2-grasp-script

# ④ 去 GitHub 页面点 "Compare & pull request"，写清楚改了什么、为什么改，
#    指定审核人（通常是主管），等合并

# ⑤ 合并后清理本地分支
git checkout master
git pull
git branch -d feature/revo2-grasp-script
```

### 5.3 分支命名约定

| 前缀 | 用途 | 例子 |
|---|---|---|
| `feature/` | 新功能、新文档 | `feature/hand-ros2-bridge` |
| `fix/` | 修 bug | `fix/canfd-baudrate-mismatch` |
| `docs/` | 纯文档 | `docs/troubleshooting-guide` |
| `hotfix/` | 紧急修复（直接基于 master） | `hotfix/id-config-error` |

---

## 第 6 章 常见麻烦怎么解

### 6.1 推送被拒绝：`rejected - non-fast-forward`

说明远程有别人（或你自己在别的机器上）的新提交。**不要用 `--force`**，先拉：

```bash
git pull --rebase
# 如果没冲突，直接
git push
```

### 6.2 出现冲突（Conflict）

冲突长这样：

```
<<<<<<< HEAD
这是你本地写的内容
=======
这是远程别人写的内容
>>>>>>> origin/master
```

处理步骤：

1. 打开冲突文件，找到 `<<<<<<<` `=======` `>>>>>>>` 这三行标记；
2. **想清楚要保留哪部分**（可以两边都要、也可以改写成第三种）；
3. **删掉这三行标记本身**，把文件改成最终想要的样子；
4. 然后：

```bash
git add <改好的文件>
git rebase --continue    # 或者 git merge --continue
git push
```

> 拿不准就**先备份文件**，或者直接叫同事一起看。冲突处理错了好恢复，但别慌着 `--force`。

### 6.3 提交了不该提交的东西

**情况一：还没 push（还能干净地撤）**

```bash
# 撤销最后一次提交，但保留文件改动
git reset --soft HEAD~1

# 把误加的文件从暂存区拿掉
git restore --staged <误提交的文件>

# 重新提交
git commit -m "..."
```

**情况二：已经 push 了**

```bash
git rm --cached <误提交的文件>     # 从仓库移除，保留本地文件
# 把它加进 .gitignore
git add .gitignore
git commit -m "chore: 移除误提交的文件并补充 .gitignore"
git push
```

> ⚠️ 注意：**这不等于从历史里抹掉**，之前的快照里还留着。
> 如果误提交的是 **密钥 / 密码**，必须：① 立刻去对应平台作废这把密钥并重新生成；
> ② 通知主管，由主管决定是否清洗仓库历史（需要 `git filter-repo`，别自己瞎搞）。

### 6.4 误提交了大文件（几十 MB 的 7z / PDF）

同上，先 `git rm --cached` 移除并提交。**但历史里的大小不会变**，仓库克隆速度会一直受影响。
所以最好一开始就别提交——本仓库的 `.gitignore` 已经默认拦住了 `.7z` `.pdf` `.mhtml` 等类型。

**判断标准（超了就别进 Git）：**

- 单个文件 > 5 MB → 不要直接提交
- 二进制安装包、官方 PDF 手册、数据集 → 放公司文件服务器 / 网盘，README 里放链接
- 如果确实必须版本化二进制，用 **Git LFS**（需要组织开启配额，问主管）

### 6.5 提交身份写错了

```bash
# 改配置（只影响以后的提交）
git config --global user.name "正确名字"
git config --global user.email "正确邮箱"

# 只改最近一次提交的作者
git commit --amend --reset-author --no-edit
# 注意：会改变提交编号，已 push 的话需要 force push，务必先问主管
```

### 6.6 远程地址写错了

```bash
git remote -v                              # 先看现在是什么
git remote set-url origin <正确的URL>      # 改掉
```

### 6.7 「我改乱了，想全部推倒重来」

```bash
# 只丢弃工作区未提交的改动（危险，改动会真的消失）
git restore .

# 把本地分支重置到远程最新状态（危险）
git fetch origin
git reset --hard origin/master
```

> ⚠️ 这两条会**永久丢掉未提交的改动**，执行前先 `git stash` 存一下：
> `git stash` 暂存起来，之后 `git stash pop` 取回来。

### 6.8 每次都要输账号密码

```bash
# 确认用的是 SSH 地址（git@github.com:...）
git remote -v

# 或用凭据管理器记住 HTTPS 密码
git config --global credential.helper manager
```

---

## 第 7 章 这个仓库的具体规矩

| 项目 | 规矩 |
|---|---|
| 主仓库 | `git@github.com:Bake-Humanoid/elf3-humanoid.git`（**多人共用，根目录是公共区域**） |
| 本资料位置 | 该仓库下的 `dexterous-hand/` 子目录（以实际为准），**不往根目录放文件** |
| 提交范围 | 只用 `git add -- dexterous-hand`，**绝不用 `git add -A`**（会连带提交同事的改动） |
| 主分支 | `master` |
| 开发分支 | `feature/<主题>` |
| 提交信息 | 中文，`<类型>: <说明>`，类型见第 4 章 |
| 提交粒度 | 一次提交一件事，别攒一大坨 |
| 换行符 | 由 `.gitattributes` 统一为 LF，不要手动改 |
| 图片 | 压缩后放 `docs/images/`，单张 < 500 KB，命名 `<文档号>-<序号>-<内容>.png` |
| 大文件 | **不入库**。`.gitignore` 已拦 `.7z/.pdf/.mhtml/.stp/.iso/.exe` 等 |
| 密钥 | **绝对不入库**。已拦 `*.pem` `*.key` `id_rsa*` `.env` |
| 日志 | `logs/` 只保留说明文件，实际日志不入库 |
| 提交前 | 跑 `./tools/git_sync.sh "..."`，它会替你挡掉大文件和密钥 |

---

## 第 8 章 速查卡（建议打印贴显示器旁）

```bash
# ── 一次性 ─────────────────────────────
git config --global user.name  "名字"
git config --global user.email "邮箱"
ssh-keygen -t ed25519 -C "邮箱"     # 然后 cat ~/.ssh/id_ed25519.pub 粘到 GitHub

# ── 拿仓库（网络慢就用稀疏检出，只下自己那一块）──
git clone --filter=blob:none --no-checkout --depth 1 --single-branch \
  git@github.com:Bake-Humanoid/elf3-humanoid.git
cd elf3-humanoid
git sparse-checkout set --cone dexterous-hand
git checkout master

# ── 日常（90% 用这几条）────────────────
git status                           # 看改了什么
git add -- dexterous-hand            # ★ 只加自己那块，别用 git add -A
git commit -m "说明"
git pull --rebase                    # 先拉同事的更新
git push

# ── 开分支干活 ─────────────────────────
git checkout -b feature/xxx
git push -u origin feature/xxx       # 然后去 GitHub 提 Pull Request

# ── 看历史 ─────────────────────────────
git log --oneline -10
git diff

# ── 出事了先别慌 ───────────────────────
git status                           # 永远先看这个
git stash                            # 存起来，别丢
git restore .                        # 丢弃未提交改动（危险）
```

---

## 附录：术语中英对照

| 中文 | 英文 | 一句话解释 |
|---|---|---|
| 仓库 | repository / repo | 一个项目的整体 |
| 克隆 | clone | 把远程仓库整个下载到本地 |
| 提交 | commit | 给项目拍一张快照 |
| 暂存区 | staging area / index | 决定这次提交包含哪些改动 |
| 推送 | push | 把本地提交上传到远程 |
| 拉取 | pull / fetch | 把远程的新提交拿到本地 |
| 分支 | branch | 一条独立的开发线 |
| 合并 | merge | 把一条分支的改动并到另一条 |
| 变基 | rebase | 把你的提交「挪」到别人最新提交之后，历史更干净 |
| 冲突 | conflict | 两边改了同一处，Git 无法自动决定，需要人来定 |
| 合并请求 | Pull Request (PR) | 「请把我这条分支合进 master」的申请 |
| 远程 | remote / origin | 服务器上的那个仓库，默认叫 origin |

---

## 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-09-20 | 首版：环境准备、SSH 配置、日常流程、提交规范、分支策略、故障处理速查 |
| v1.1 | 2026-09-20 | 新增 §0.4「本地改了文件，GitHub 上会自动变吗？——不会」：明确 Git 无后台自动同步、commit≠push、两条推送纪律，并给出 `tools/git_sync.sh` 的用法；第 0 章由「三个认知」改为「四个认知」 |

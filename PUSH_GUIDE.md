# Git 推送代码到 GitHub 完整教程（超详细版）

这份教程用最直白的方式，一步一步教你把自己的代码传到 GitHub。所有命令都能直接复制，照着做就行。

> 说明：本教程在 Windows + Git Bash 环境下编写。如果还看不懂某个名词，直接跳到文末「九、名词解释」。

---

## 一、先搞清楚三个词

| 词 | 是什么 | 打个比方 |
|---|---|---|
| **本地仓库** | 你电脑上这个文件夹，由 Git 管理 | 你自己的草稿本 |
| **远程仓库** | GitHub 网站上的那个仓库 | 云端备份 |
| **推送 (push)** | 把本地改动上传到 GitHub | 把草稿抄到云端 |

所以"推送代码"整个过程 = **把本地文件先打包（commit），再上传到 GitHub（push）**。

---

## 二、一次性准备（每台电脑只做一次，做过的跳过）

### 1. 确认装了 Git

打开 Git Bash，输入：

```bash
git --version
```

如果显示类似 `git version 2.x.x`，说明装好了。如果提示"找不到命令"，先去 https://git-scm.com 下载安装，一路默认下一步。

### 2. 设置你的名字和邮箱（只做一次）

```bash
git config --global user.name "ztt"
git config --global user.email "3403917043@qq.com"
```

> 作用：告诉 Git"这些提交是谁写的"。名字邮箱随便填，会显示在 GitHub 提交记录上。

### 3. 搞定 GitHub 登录凭证（最关键，卡住最多的就是这步）

GitHub 从 2021 年起**不允许用账号密码推送**，必须用 **Token（令牌）**。生成一次即可：

1. 打开 https://github.com ，登录你的账号
2. 点右上角**头像** → **Settings**（设置）
3. 把页面拉到最底部，左侧点 **Developer settings**（开发者设置）
4. 点 **Personal access tokens** → **Tokens (classic)**
5. 点 **Generate new token** → **Generate new token (classic)**
6. Note 随便填个名字，如 `push-token`
7. 勾选 **repo** 这一项（整个方框）
8. 拉到最下面点 **Generate token**
9. **立刻复制**那串以 `ghp_` 开头的东西（这串只会显示这一次，关掉页面就再也看不到了）

**以后第一次推送时：**
- 用户名填：你的 GitHub 账号名（如 `Zhang060531`）
- 密码填：**刚才那串 token**（不是你的登录密码！）

Windows 会自动记住，之后就不用再输了。

> 如果中途忘了保存 token，回去重新生成一个就行，旧的可以删掉。

---

## 三、场景 A：全新项目第一次推送到 GitHub（最常用）

假设你的代码在文件夹 `my_project` 里，要推到 GitHub 上一个叫 `my_project` 的新仓库。

### 第 1 步：在 GitHub 上建一个空仓库

1. 打开 https://github.com ，登录
2. 点右上角 **＋** 号 → **New repository**
3. **Repository name** 填仓库名，如 `my_project`
4. 选 **Public**（公开）或 **Private**（私有）
5. ⚠️ **关键**：下面"Add a README file / .gitignore / license"**三个都不要勾**（保持空仓库，后面才不会冲突）
6. 点 **Create repository**

创建后页面会显示一个地址，形如：
```
https://github.com/你的账号/my_project.git
```
**复制这个地址**，下一步要用。

### 第 2 步：进入你的项目文件夹

```bash
cd /c/Users/ztt/Desktop/my_project
```

> 提示：Windows 的 `C:\Users\ztt\Desktop` 在 Git Bash 里写成 `/c/Users/ztt/Desktop`。

确认进对地方（应该能看到你的代码文件）：

```bash
ls
```

### 第 3 步：初始化 Git 仓库（文件夹里还没有 `.git` 时才需要）

```bash
git init
```

> 如果这个文件夹之前已经 `git init` 过（有 `.git` 文件夹），这步跳过，直接到第 4 步。

### 第 4 步：把文件打包（暂存 + 提交）

```bash
git add -A
git commit -m "首次提交"
```

- `git add -A`：把所有文件交给 Git 管理（包括新增、修改、删除）
- `git commit -m "..."`：真正"存档"一次，引号里写这次改了什么

### 第 5 步：关联远程仓库

```bash
git remote add origin https://github.com/你的账号/my_project.git
```

> 作用：告诉本地"我的云端地址是哪一个"。`origin` 是给这个地址起的名字，约定俗成叫 origin。

### 第 6 步：推上去

```bash
git branch -M main
git push -u origin main
```

- `git branch -M main`：把分支名统一成 `main`
- `git push -u origin main`：上传到云端；`-u` 让本地记住，以后直接 `git push` 就行

**看到类似下面的输出就是成功了：**
```
To https://github.com/你的账号/my_project.git
 * [new branch]      main -> main
```

刷新 GitHub 页面，代码就在上面了。✅

---

## 四、场景 B：本地已有仓库（有提交历史），新建一个远程仓库推上去

你这次 `fold_ztt` 就是这种情况。本地已经有 `.git` 和历史提交，只需：

1. 按「场景 A 第 1 步」在 GitHub 建好空仓库
2. 提交当前所有改动：

```bash
git add -A
git commit -m "提交说明"
```

3. 关联远程。如果之前已经有一个 `origin` 指向旧仓库，**覆盖它**：

```bash
git remote set-url origin https://github.com/你的账号/新仓库名.git
```

4. 推送：

```bash
git branch -M main
git push -u origin main
```

> 想**同时保留**旧仓库的关联，用另一个名字添加即可（不覆盖 origin）：
> ```bash
> git remote add newrepo https://github.com/你的账号/新仓库名.git
> git push -u newrepo main
> ```

---

## 五、场景 C：日常改代码后的推送（以后每天就三步）

改完代码，打开 Git Bash 进入项目目录，依次执行：

```bash
git add -A
git commit -m "说明这次改了什么"
git push
```

因为之前 `-u` 已经记住上游，所以直接 `git push` 就够了。

**随时可以查看当前状态：**

```bash
git status
```
- 红色文件 = 有改动但还没 `add`
- 绿色文件 = 已经 `add`，还没 `commit`
- 显示 `nothing to commit, working tree clean` = 全都提交了

---

## 六、场景 D：把仓库拉到新电脑上继续改

在新电脑上（先按「二」做一次性准备），然后：

```bash
git clone https://github.com/你的账号/my_project.git
cd my_project
```

之后改完代码，正常三步推送即可：

```bash
git add -A
git commit -m "..." 
git push
```

---

## 七、常见报错与解决办法

| 报错信息 | 原因 | 怎么解决 |
|---|---|---|
| `remote origin already exists` | 已经关联过远程了 | `git remote set-url origin <新地址>` |
| `failed to push ... remote contains work` | 建仓库时勾了 README，远程不是空的 | 先 `git pull --rebase origin main`，再 `git push` |
| 一直让你输密码，且总是失败 | 你把登录密码当 token 用了 | 密码填 `ghp_` 开头的 token |
| `master` 和 `main` 对不上 | 分支名不一致 | `git branch -M main` |
| `fatal: not a git repository` | 进错目录了 | 先 `cd` 到项目根目录 |
| `Please tell me who you are` | 没配置用户名邮箱 | 回到「二」第 2 步配置 |

---

## 八、总结：核心就这几条命令

**新项目首次推送：**
```bash
git init
git add -A
git commit -m "首次提交"
git remote add origin https://github.com/你的账号/仓库名.git
git branch -M main
git push -u origin main
```

**日常推送：**
```bash
git add -A
git commit -m "说明改了什么"
git push
```

---

## 九、名词解释

| 名词 | 大白话 |
|---|---|
| 仓库 (repository) | 一个由 Git 管理的项目文件夹 |
| 提交 (commit) | 给当前文件拍一张"快照"存档，能随时找回 |
| 暂存 (add) | 把要存的文件先放进"待存区" |
| 远程 (remote) | 云端仓库的地址，`origin` 是它的小名 |
| 分支 (branch) | 代码的一条独立线，`main` 是主分支 |
| 推送 (push) | 把本地存档上传到云端 |
| 拉取 (pull) | 把云端的改动下载到本地 |
| 克隆 (clone) | 把云端整个仓库下载到本地 |
| Token | 代替密码的授权串，`ghp_` 开头 |

# 在小电脑（NUC）上用 git 管理代码

目前小电脑上的 `~/myaim` 是 **tar+scp 拷贝**过去的（不含 `.git`），所以那边不是仓库。
下面把它换成 git 管理，以后 `git pull` 就能更新。

---

## 前置：小电脑要能上外网

小电脑通过 Windows 移动热点共享上网。先在 **Windows** 上确认：

```powershell
# 热点必须是开着的, 且存在 192.168.137.1
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like '192.168.*' }
Test-Connection 192.168.137.110 -Count 2 -Quiet
```

> **今天遇到的情况**：热点关了，`192.168.137.1` 消失，小电脑拿到 `169.254.x.x`（APIPA），
> 完全连不上。重开热点即可恢复。

在 **小电脑** 上确认能上网：

```bash
ping -c 2 192.168.137.1      # 通网关
ping -c 2 github.com         # 通外网
```

---

## 方案 A：HTTPS 克隆（最快，推荐明天用）

不需要配密钥，公开仓库直接拉：

```bash
cd ~
mv myaim myaim_bak 2>/dev/null        # 保留旧的构建产物做备份
git clone https://github.com/smith114/my_aim.git myaim
cd myaim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build                # 5 套离线测试, 30 秒
```

以后更新：

```bash
cd ~/myaim && git pull
cmake --build build -j4
```

> 如果仓库以后设为私有，HTTPS 需要 Personal Access Token：
> `git clone https://<用户名>:<token>@github.com/smith114/my_aim.git`

---

## 方案 B：SSH 克隆（一劳永逸，推送也方便）

```bash
# 1. 在小电脑上生成密钥（一路回车, 不用设密码）
ssh-keygen -t ed25519 -C "nuc-myaim" -f ~/.ssh/id_ed25519

# 2. 打印公钥
cat ~/.ssh/id_ed25519.pub
```

把打印出来的整行**贴到 GitHub**：`Settings → SSH and GPG keys → New SSH key`。

```bash
# 3. 验证
ssh -T git@github.com        # 应回 "Hi smith114!"

# 4. 克隆
cd ~ && git clone git@github.com:smith114/my_aim.git myaim
```

> 我在 Windows 上已经用过同样的方式（`C:\Users\badtime\.ssh\id_ed25519`），可以先试
> 能不能直接复用那把密钥；不行再在小电脑上生成新的。

---

## 注意：别再让 tar 覆盖 git 目录

我这边有个部署脚本会 **tar+scp 覆盖 `~/myaim`（不含 `.git`）**。改用 git 之后：

- **要么**用 git 管理（`git pull`），`~/myaim`
- **要么**用 tar 部署（不改 `.git` 就不会冲突，但会覆盖源码）

两者混用会覆盖本地改动。明天建议统一走 **git pull**。

---

## 明天的应急方案（网络万一不通）

如果小电脑连不上外网，用我这边已有的脚本推：

```powershell
# 在 Windows 上执行
cmd /c "C:\Users\badtime\Desktop\deepseek-harness\.dsh\deploy_nuc.cmd build"
```

它会打包源码 → scp → 在小电脑上编译，编译成功打印 `BUILD_OK`。
**不含 `.git`，但代码是最新的。**

---

## 上车后的标准流程

```bash
cd ~/myaim
git pull                              # 1. 更新
cmake --build build -j4               # 2. 编译
ctest --test-dir build                # 3. 离线测试(不需要硬件)
./build/gimbal_test --query           # 4. 看云台姿态 + mode
```

**第 3 步是硬性要求** —— 上车前先在离线测试上确认代码没问题，比在车上试错快得多。

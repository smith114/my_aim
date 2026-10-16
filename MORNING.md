# 明早上实车 · 启动流程

> 在 **小电脑的 VS Code 终端** 里逐条敲。每步都有"应该看到什么"和"不对怎么办"。

---

## 0. 出门前（在 Windows 上，30 秒）

```powershell
# 确认热点开着（不然后面小电脑连不上）
Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -like '192.168.*' }
Test-Connection 192.168.137.110 -Count 2 -Quiet      # 应回 True
```

看到 `192.168.137.1` 和 `True` 就行。**热点没开** → 设置 → 网络和 Internet → 移动热点 → 打开。

---

## 1. 小电脑：更新代码

VS Code 里 `` Ctrl+` `` 打开终端。

```bash
cd ~/myaim
git pull
```

**应该看到**：`Already up to date.` 或列出更新的文件。

<details>
<summary>如果 <code>git pull</code> 报错 / 说不是仓库 / 连不上网</summary>

第一次上 git（`~/myaim` 原本是 tar 拷过去的，没有 `.git`）：

```bash
cd ~
mv myaim myaim_bak                    # 保留旧的构建产物
git clone https://github.com/smith114/my_aim.git myaim
cd ~/myaim
```

连不上外网的话，用我这边推：
```powershell
# 在 Windows 上
cmd /c "C:\Users\badtime\Desktop\deepseek-harness\.dsh\deploy_nuc.cmd build"
```
</details>

---

## 2. 编译

```bash
cd ~/myaim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

**应该看到**：最后没有 `error`，configure 阶段打印一行：

```
-- 靶标预设: 通用 (aspect 上限 3.5, 按实机标定)
```

编译可执行文件：

| 文件 | 用途 |
| --- | --- |
| `build/my_aim` | **主程序（自瞄闭环）** |
| `build/cam_view` | 实时成像 + 检测标注 |
| `build/gimbal_test` | 云台自检（读姿态 / 锁住 / 扫） |
| `build/calibrate` | 相机标定 |
| `build/tests/detect_probe` | **检不到时的逐级诊断** |

**要 build 的就是这一条 `cmake --build build -j4`** —— 它会把上面全部编译出来。

---

## 3. 离线自检（不用硬件，30 秒，必做）

```bash
ctest --test-dir build --output-on-failure
```

**应该看到**：

```
100% tests passed, 0 tests failed out of 5
```

**这一步是硬性要求。** 5 套测试（协议 / 解算 / 检测 / 跟踪 / 闭环仿真）不需要任何硬件，能在上车前把代码问题挡掉。

如果这里就红了 → **先别上车**，把输出发我。

---

## 4. 看云台姿态和视觉模式

```bash
./build/gimbal_test --query
```

**应该看到**（示例）：

```
当前实测: yaw=-147.38 pitch=0.10 roll=-0.43 (mode=0 id=3)
```

**关键看 `mode`：**

| 值 | 含义 | 怎么做 |
| --- | --- | --- |
| `mode=0` | 视觉控制**没开**，下位机忽略我们的角度指令 | **按车上鼠标右键** |
| `mode=1` | 视觉控制已开，可以开始 | 继续第 5 步 |

想盯着 `mode` 变化去按右键：

```bash
./build/gimbal_test --watch          # Ctrl-C 退出
```

> **`mode=0` 时云台绝对不会跟随**，这是下位机行为（规则 3.3 节：视觉控制由鼠标右键进入/退出，由下位机实现），上位机开不了。

---

## 5. 看相机成像（最重要的一步）

```bash
./build/cam_view --hold-current
```

弹窗显示：

| 画面元素 | 含义 |
| --- | --- |
| 蓝框 / 红框 | 检出的灯条 |
| **黄框** | 配对成功的装甲板 |
| 左上角 HUD | `f=帧号 bars=N armor=N V_mean=亮度 bayer=bg` |

**快捷键**：`s` 存图到 `diag/` · `T` 换 Bayer 相位 · `d` 开关灯条显示 · `q` 退出

**在这里要确认两件事：**
1. `bars` ≥ 2（灯条能检出来）
2. `armor` ≥ 1（能配成装甲板）

**如果 `armor=0`**，按 `s` 存图，然后跳到第 8 步诊断。

---

## 6. 手动对准靶标

`mode=1` 之后云台会锁住姿态。**用手把云台推到靶标方向上。**

想让它稳稳停住（推得动、松手就停）：

```bash
# 先 Ctrl-C 退出 cam_view（它占着相机）
./build/gimbal_test --stay
```

`--stay` 每 50ms 重读实测角原样发回 → 你推它它跟着走，松手停在那儿，**Ctrl-C 退出也保持在那个角度，不会回 0/0**。

---

## 7. 跑自瞄闭环 ★

```bash
./build/my_aim --debug
```

（有窗口，能看到画面和检测框。想跑固定帧数用 `--headless --frames 300`）

**每 30 帧打一行，重点看这几个字段：**

```
[DET] f=90 bars=3.2 armor=1.2 solved=1.2 locked=1.0 state=TRACK cmd=192.7/2.6 fb=192.7/2.6 home=159.4/3.0 mode=1 th=-1.2/0.3 lock_age=0
```

| 字段 | 好的值 | 说明 |
| --- | --- | --- |
| `state` | **TRACK** | DETECT=在找，LOST=短暂丢失 |
| `locked` | **接近 1.0** | 本秒锁定帧占比。`0.1` = 基本锁不住 |
| `lock_age` | **0** | 距上次成功锁定多少帧，应该一直是 0 |
| `th` | **接近 0** | 目标偏离光轴的角度，应收敛 |
| `cmd` vs `fb` | 两者接近 | 命令角和实测角，说明云台跟得上 |
| `mode` | **1** | 是 0 就按右键 |

`Ctrl-C` 停止（会补发保持帧，云台停在原地）。

---

## 8. 检不到时：逐级诊断

```bash
./build/tests/detect_probe --cam --hold-current diag/probe.jpg -v
```

输出三段，**第三段最能对症**：

```
=== 配对约束逐条检查 ===
  i  j  color dLen  dy/h  dx/h  dAng  aspect | 结果
   1  2  B   0.97  0.09  1.12   3.7   1.13 | OK
   2  3  B   0.31  0.04  6.81  23.5   7.11 | 太远
```

`结果` 列会直接说被哪条拒了：`太远` / `太近` / `不平行` / `长度比` / `宽高比不当` / `中心不够暗` / `y未对齐` / `中间夹灯条`。

**第一段**列出每个连通域的面积/长短边/长宽比，以及每一级阈值过没过（`short_ok long_ok ratio_ok purity_ok`）。

标注图存到 `diag/probe.jpg`，VS Code 里直接点开看。

---

## 9. VS Code 任务（不用敲命令）

`Ctrl+Shift+P` → `Developer: Reload Window`（加载配置），然后 `Ctrl+Shift+P` → **`Tasks: Run Task`**：

| 任务 | 作用 |
| --- | --- |
| `1. 配置 CMake` / `2. 编译` | 等价第 2 步 |
| `运行: 自瞄闭环 my_aim` | 第 7 步 |
| `运行: 实时成像 cam_view` | 第 5 步 |
| **`★ 锁住当前姿态`** | 第 6 步（手动对准后用） |
| `诊断: 读云台实测姿态` | 第 4 步 |
| `诊断: 盯云台角/模式变化` | 盯 `mode` |
| `★★ 模拟锁定` | 无硬件验证闭环 |
| `★ 视频验证: 单帧逐级诊断` | 第 8 步（也可对视频用） |
| `刹车: 关掉 cam_view 释放相机` | 相机被占时 |

**调试**（F5）：左侧调试图标 → 选 `调试 my_aim (闭环)` / `调试 cam_view` / `调试 detect_probe`。

---

## 10. 相机或串口被占用

报 `MV_E_ACCESS_DENIED`（`0x80000203`）就是相机还被占着：

```bash
pkill -f 'build/cam_view'; pkill -f 'build/my_aim'
pkill -f 'gimbal_test';    pkill -f 'detect_probe'
sleep 1
fuser /dev/ttyACM0                 # 应无输出
fuser /dev/bus/usb/002/004         # 应无输出
```

**上位机程序独占串口和相机，同一时间只能跑一个。**

---

## 11. 完整命令清单（可直接整段复制）

```bash
cd ~/myaim

# 更新 + 编译 + 离线测试
git pull
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
ctest --test-dir build

# 看云台 (确认 mode=1, 否则按车上鼠标右键)
./build/gimbal_test --query

# 看成像 (确认 bars>=2 且 armor>=1)
./build/cam_view --hold-current

# ← 按右键进 mode=1, 手动对准靶标, 然后 Ctrl-C 退出 cam_view

# 跑闭环
./build/my_aim --debug
```

---

## 12. 出门前 checklist

- [ ] Windows 热点已开，`ping 192.168.137.110` 通
- [ ] `git pull` 成功（或 `deploy_nuc.cmd build` 打印 `BUILD_OK`）
- [ ] `cmake --build build -j4` 无 error
- [ ] `ctest` 5/5 通过
- [ ] 鼠标插在**下位机**的 USB HID 口上（规则 3.3）
- [ ] 相机镜头干净、对焦在靶标距离上

---

## 目前仍未验证的一件事（心里有数）

**真实相机上的持续锁定还没验证过。** 上次实机跑 `locked=0.1`，之后修了四个原因：

1. 自动摆扫（会把靶标转出画面）→ 默认关闭
2. 曝光 20ms → **2ms**（原来 LED 过曝成白团，形状糊掉）
3. 灯条长宽比下限 3.0 → **1.5**（远处灯条检不出来）
4. 跨板误配 → 新增"中间不夹灯条"约束

**明天第 5 步和第 7 步要看的就是这两个数**：`armor` 和 `locked`。

如果 `armor` 还是 0 或 `locked` 还是上不去，**按 `s` 存图 + 跑第 8 步诊断**，把图和输出发我 —— 有这两个东西我就能定位到具体是哪条约束/哪个阈值。

# 现场命令卡

> 在**小电脑的 VS Code 终端**里敲。所有命令前面省略了 `cd ~/myaim`，先敲一次即可。
> **上位机程序独占串口和相机 —— 同一时间只能跑一个。** 切换前先 `Ctrl-C`。

```bash
cd ~/myaim        # 每条命令前都需要, 只敲一次
```

---

## 一、调取摄像头

### 最常用：打开相机窗口看画面

```bash
./build/cam_view --hold-current
```

弹出窗口，显示实时画面 + 检测结果：

| 画面元素 | 含义 |
| --- | --- |
| 蓝框 / 红框 | 检出的灯条 |
| **黄框** | 配对成功的装甲板 |
| 左上 HUD | `f=帧号 bars=N armor=N V_mean=亮度 bayer=bg IMG_DIR` |

**快捷键**：`s` 立刻存图到 `diag/` · `T` 换 Bayer 相位 · `d` 开关灯条显示 · `q` 退出

**`--hold-current`** = 一直把当前实测角发回去，云台停着不动。

> 不加 `--hold-current` 也能开，程序不会把云台拽走（已修），但加上更保险。

---

## 二、现场速查表

| 我想干什么 | 敲什么 |
| --- | --- |
| **看相机画面** | `./build/cam_view --hold-current` |
| **跑自瞄闭环** | `./build/my_aim --debug` |
| 跑闭环但不要窗口（看日志） | `./build/my_aim --headless --frames 300` |
| **检不到，查哪条约束拒的** | `./build/tests/detect_probe --cam --hold-current diag/p.jpg -v` |
| 抓一张图存下来发我 | `./build/tests/detect_probe --cam --hold-current diag/p.jpg` |
| **读云台姿态 + mode** | `./build/gimbal_test --query` |
| 盯 `mode`（按右键时看） | `./build/gimbal_test --watch` |
| **手动对准后锁住** | `./build/gimbal_test --stay` |
| 自动横扫找靶标 | `./build/cam_view --seek` |
| 云台方向/量程自检 | `./build/gimbal_test --step yaw 15` |
| 相机被占，释放 | `pkill -f 'build/cam_view'; pkill -f 'build/my_aim'` |

---

## 三、推荐现场顺序

```bash
# ① 确认云台能控 (mode 必须是 1)
./build/gimbal_test --query
#   mode=0 -> 按车上鼠标右键, 再查一次

# ② 看相机画面, 确认能检出装甲板
./build/cam_view --hold-current
#   看 HUD: bars>=2 且 armor>=1
#   检不到就按 s 存图, 跳到 ④

# ③ 按右键进 mode=1, 手动把云台对准靶标, 然后 Ctrl-C 退出 cam_view
#    想让它停稳: ./build/gimbal_test --stay

# ④ 跑自瞄闭环
./build/my_aim --debug
```

---

## 四、`cam_view` 全部参数

```
用法: cam_view [--headless] [--hold-current] [--raw] [--bayer XX]
                [--out DIR] [--every N] [--frames N]
      cam_view --seek [--span DEG] [--seek-step DEG]
```

| 参数 | 作用 |
| --- | --- |
| `--hold-current` | 保持当前姿态（推荐总是加） |
| `--raw` | 不画检测框，只看原始画面（判断曝光/颜色用） |
| `--bayer rg\|bg\|gr\|gb` | 指定 Bayer 相位（颜色不对时用，或窗口里按 `T`） |
| `--out DIR` | 存图目录（默认 `diag/`） |
| `--every N` | 每 N 帧存一张图 |
| `--frames N` | 跑 N 帧后自动退出 |
| `--headless` | 不弹窗（配合 `--every` 批量存图） |
| `--seek` | **自动横扫找装甲板**，找到停在该角度 |

**自动找靶标**（不用手推云台）：

```bash
./build/cam_view --seek --span 150 --seek-step 5
```
`--span 150` = 左右各扫 75°，`--seek-step 5` = 每 5° 停一下。

---

## 五、`my_aim` 全部参数

```
用法: my_aim [--debug] [--headless] [--mock] [--save-video]
              [--scan] [--bayer rg|bg|gr|gb]
              [--snapshot-dir DIR] [--log FILE]
              [--frames N] [video|image]
```

| 参数 | 作用 |
| --- | --- |
| `--debug` | **有窗口**，实时看画面 + 检测框 + 状态字 |
| `--headless` | 无窗口（服务器/远程用） |
| `--frames N` | 跑 N 帧后退出 |
| `--snapshot-dir DIR` | 存快照目录 |
| `--log FILE` | 日志文件（默认也写 `/tmp/myaim.log`） |
| `--bayer XX` | 指定 Bayer 相位 |
| `--scan` | 无目标时自动摆扫（**默认关闭**，操作手已手动对准） |
| `--synth` | 模拟靶标跑完整链路（不碰硬件，答辩演示用） |

---

## 六、日志怎么读

`my_aim` 每 30 帧打一行：

```
[DET] f=90 bars=3.2 armor=1.2 solved=1.2 locked=1.0 state=TRACK cmd=192.7/2.6 fb=192.7/2.6 home=159.4/3.0 mode=1 th=-1.2/0.3 lock_age=0 rx=1000
```

| 字段 | 好的值 | 说明 |
| --- | --- | --- |
| `bars` | ≥ 2 | 本秒平均检出灯条数 |
| `armor` | ≥ 1 | 本秒平均配成装甲板数 |
| `state` | **TRACK** | DETECT=在找 · LOST=短暂丢失 |
| **`locked`** | **接近 1.0** | 本秒锁定帧占比。`0.1` = 基本锁不住 |
| **`lock_age`** | **0** | 距上次成功锁定多少帧，应一直是 0 |
| `th` | **接近 0** | 目标偏离光轴的角度，应收敛 |
| `cmd` / `fb` | 两者接近 | 命令角 / 实测角 |
| **`mode`** | **1** | 是 0 就按车上鼠标右键 |
| `rx` | 递增 | 收到的下位机帧数 |

**遇到 `mode=0` 会有明确告警**：
```
[MODE][WARN] 检出装甲板但 mode=0, 指令被忽略, 云台不会跟随。请在车上按鼠标右键进入视觉控制模式。
```

**mode 变化时**：
```
[MODE] mode 0 -> 1 : ON   视觉控制已开启 -> 角度指令生效
```

---

## 七、检不到装甲板：三段诊断

```bash
./build/tests/detect_probe --cam --hold-current diag/p.jpg -v
```

**第一段 · 逐级过滤** —— 每个连通域的各项指标，以及每一级阈值过没过：

```
  color=BLUE 形态学后轮廓 26 个
    #0 area=37.5 L=12.1 s=6.6 ratio=1.84 BGRmean=(64,56,47) short_ok=1 long_ok=1 ratio_ok=0 purity_ok=0
```
`ratio_ok=0` → 长宽比不够（调 `LB_RATIO_MIN`）

**第二段 · 灯条列表** —— 最终通过的灯条

**第三段 · 配对约束逐条**（**最能对症**）：

```
  i  j  color dLen  dy/h  dx/h  dAng  aspect | 结果
   1  2  B   0.97  0.09  1.12   3.7   1.13 | OK
   2  3  B   0.31  0.04  6.81  23.5   7.11 | 太远
```

`结果` 会直接说被哪条拒：`长度比` / `太远` / `太近` / `不平行` / `y未对齐` / `宽高比不当` / `中心不够暗` / `中间夹灯条`

标注图存到 `diag/p.jpg`，VS Code 里点开看。

---

## 八、对比诊断：对视频/图片跑

```bash
./build/tests/detect_probe 图片或视频 输出图.jpg --frames N -v
```

现场可以把 `diag/` 里存的图再拿出来反复分析，不用一直占着相机。

---

## 九、相机 / 串口被占用

```bash
pkill -f 'build/cam_view'; pkill -f 'build/my_aim'
pkill -f 'gimbal_test';    pkill -f 'detect_probe'
sleep 1

fuser /dev/ttyACM0              # 串口, 无输出=空闲
fuser /dev/bus/usb/002/004      # 相机, 无输出=空闲
```

报 `MV_E_ACCESS_DENIED`（`0x80000203`）= 相机还被别的进程占着。

---

## 十、VS Code 任务（不用敲命令）

`Ctrl+Shift+P` → `Developer: Reload Window` → `Tasks: Run Task`：

| 任务 | 等价命令 |
| --- | --- |
| `运行: 实时成像 cam_view` | `cam_view` |
| `运行: 自瞄闭环 my_aim` | `my_aim --debug` |
| **`★ 锁住当前姿态`** | `gimbal_test --stay` |
| `诊断: 读云台实测姿态` | `gimbal_test --query` |
| `诊断: 盯云台角/模式变化` | `gimbal_test --watch` |
| `★ 视频验证: 单帧逐级诊断` | `detect_probe ... -v` |
| `刹车: 关掉 cam_view 释放相机` | `pkill ...` |

**调试 F5**：左侧调试图标 → `调试 my_aim (闭环)` / `调试 cam_view` / `调试 detect_probe`

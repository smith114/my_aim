# 实机操作手册（在 NUC 终端里自己跑）

> 说明：上位机程序**独占**串口和相机，同一时间只能跑一个。
> 每个程序退出时都会把云台**保持在当前角度**（不会拉回 0/0）。
>
> 路径都是绝对路径，`cd ~/myaim` 后可省略前缀。

---

## 0. 每次上电后的第一件事

```bash
cd ~/myaim

# 看云台当前姿态 + 视觉控制开关
./build/gimbal_test --query
```

输出示例：
```
当前实测: yaw=-147.38 pitch=0.10 roll=-0.43 (mode=1 id=3)
```

- **`mode=0`** → 下位机忽略我们发的角度，云台不会跟随。**按车上鼠标右键**进 `mode=1`。
- `mode=1` → 视觉控制已开，可以开始。

想盯着 `mode` 变化按右键试，用：
```bash
./build/gimbal_test --watch        # Ctrl-C 退出
```

---

## 1. 看相机成像（最重要，先用这个）

```bash
./build/cam_view --hold-current
```

弹出一个窗口，显示：

| 画的东西 | 含义 |
|---|---|
| 蓝框 / 红框 | 检出的灯条 |
| **黄框** | 配对成功的装甲板 |
| 左上 HUD | `f=帧号 bars=N armor=N V_mean=亮度 bayer=bg` |

**快捷键**：`q`/`Esc` 退出 · `s` 存图到 `diag/` · `T` 换 Bayer 相位 · `d` 开关灯条显示

> `--hold-current` = 一直把当前实测角发回去，云台停着不动。
> 不加这个参数云台也不会乱动（程序不再有归零行为），但加上更保险。

**排查"检不到"时看 HUD：**
- `bars=0` 且 `V_mean` 很低（<40）→ 太暗，或灯条没过阈值
- `bars>0` 但 `armor=0` → 灯条检到了但配不成对，用下面的第 3 步看是哪条约束拒的
- 颜色不对（蓝红反了）→ 按 `T` 换 Bayer 相位

---

## 2. 跑自瞄闭环

```bash
# 有窗口（能看到画面和状态字）
./build/my_aim --debug

# 无窗口，跑 300 帧后退出（日志更干净）
./build/my_aim --headless --frames 300
```

终端会每 30 帧打一行：
```
[DET] f=90 bars=1.8 armor=0.3 locked=0.0 state=TRACK cmd=192.7/2.6 fb=192.7/2.6 home=159.4/3.0 mode=1 th=-18.4/0.7 lock_age=0
```

| 字段 | 看什么 |
|---|---|
| `state=TRACK` | 已锁定（DETECT=在找，LOST=短暂丢失） |
| `locked=1.0` | **本秒 100% 的帧都锁定**；`0.1` 说明基本锁不住 |
| `lock_age` | 距上次成功锁定过了多少帧，应该保持 0 |
| `th=` | 目标偏离光轴的角度，**应该收敛到接近 0** |
| `cmd` vs `fb` | 命令角 vs 实测角，两者接近说明云台跟得上 |
| `mode=0` | **指令被忽略**，云台不会动 —— 按右键 |

日志同时写在 `/tmp/myaim.log`。

**`ctrl-C` 停止**（会补发保持帧，云台停在原地）。

---

## 3. 检不到装甲板时：逐级诊断

**先录一段/存一张图**，然后：

```bash
# 抓实相机一帧，打印每一道过滤的结果
./build/tests/detect_probe --cam --hold-current diag/probe.jpg -v
```

输出分三段：

**① 逐级过滤** —— 每个连通域的面积/长短边/长宽比/BGR均值，以及每一级阈值过没过：
```
  color=BLUE 形态学后轮廓 26 个
    #0 area=37.5 L=12.1 s=6.6 ratio=1.84 BGRmean=(64,56,47) short_ok=1 long_ok=1 ratio_ok=0 purity_ok=0
```
`ratio_ok=0` 就说明长宽比不够。

**② 灯条列表** —— 最终通过筛选的灯条。

**③ 配对约束逐条** —— 每个候选配对被哪一条拒了：
```
  i  j  color dLen  dy/h  dx/h  dAng  aspect | 结果
   1  2  B   0.97  0.09  3.88   3.7   4.02 | OK
   2  3  B   0.91  0.04  6.81  23.5   7.11 | 太远
```
这一段最能对症：`太远`/`太近`/`不平行`/`宽高比不当`/`中心不够暗`。

标注图存到 `diag/probe.jpg`，在 VS Code 里直接点开看。

---

## 4. 没硬件时验证锁定（不碰相机串口）

```bash
# 合成靶标喂完整链路：检测/配对/PnP/跟踪/伺服全是真代码
./build/my_aim --synth --bearing 15 --dist 2.5 --frames 240 --headless
```

应看到 `state=TRACK locked=1.0 lock_age=0`，且 `th` 从 15 收敛到 1 以内。

| 参数 | 作用 |
|---|---|
| `--bearing DEG` | 靶标方位角，**视场内取 −15~+15**（水平半视场只有 ±22°） |
| `--dist M` | 距离，米 |
| `--noise N` | 图像噪声，考验鲁棒性 |
| `--distractors N` | 随机亮斑干扰数 |

---

## 5. 用录像跑（同济 demo）

```bash
./build/my_aim ~/sp_vision_25/assets/demo/demo.avi --headless --frames 900
./build/my_aim ~/sp_vision_25/assets/demo/demo.avi --debug        # 带窗口
```

> 注意：demo 视频曝光和我们相机差很多，检出的板会很少，
> 它只能验证**链路结构**，不能验证阈值。

---

## 6. 手动对准后锁住姿态

```bash
./build/gimbal_test --stay
```
每 50 ms 重读实测角再原样发回 → 你用手推云台它会跟着走，**松手就停在那儿**。
`Ctrl-C` 退出时也保持在那个角，不会回 0/0。

---

## 7. 云台量程 / 方向自检

```bash
./build/gimbal_test --step yaw 15      # 朝 +yaw 走 15 度, 看实测跟不跟、同向还是反向
./build/gimbal_test --sweep +yaw --stop 8 --max-deg 90   # 软件扫, 实测角连续 8 步不动=到限位
```

---

## 8. 相机标定（换镜头/重新对焦后必做）

```bash
./build/calibrate                                  # 交互: WASD转云台 G换规格 空格拍照
./build/calibrate --headless --shots 25 --out /tmp/calib   # 无显示器采集
./build/calibrate --compute /tmp/calib --cols 11 --rows 8  # 离线计算
```
把打印出的内参粘回 `include/config.hpp`。

---

## 9. 相机被占用 / 串口被占用

```bash
pkill -f 'build/cam_view'      # 释放相机
pkill -f 'build/my_aim'
pkill -f 'gimbal_test'
pkill -f 'detect_probe'

# 确认释放
fuser /dev/ttyACM0                 # 串口
fuser /dev/bus/usb/002/004         # 相机
```

报 `MV_E_ACCESS_DENIED`（`0x80000203`）就是相机还被别的进程占着。

---

## 10. 改完参数后必做

```bash
ctest --test-dir build --output-on-failure
```
5 套离线测试（协议/解算/检测/跟踪/闭环仿真）不需要硬件，30 秒出结果。

**上车前先跑这个** —— 比在车上试错快得多。

---

## 附：常用参数速查

| 想改什么 | 改哪里 |
|---|---|
| 灯条长宽比范围 | `include/config.hpp` 的 `LB_RATIO_MIN/MAX` |
| 二值化 S/V 下限 | `LB_SAT_MIN` / `LB_VAL_MIN` |
| 曝光 / 增益 | `CAM_EXPOSURE_US` / `CAM_GAIN_DB` |
| 装甲板宽高比 | CMake 开关 `-DUSE_STANDARD_TARGET=ON`，或 `PAIR_ASPECT_MIN/MAX` |
| 云台伺服增益/限速 | `SERVO_GAIN` / `SERVO_MAX_STEP` |
| 安全包络 | `GIMBAL_ENVELOPE_DEG` |
| 丢失后是否摆扫 | `ENABLE_AUTO_SCAN`，或命令行 `--scan` |

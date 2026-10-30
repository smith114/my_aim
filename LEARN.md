# 自瞄代码学习指南

> 读这份文档时请对照 VSCode 打开 `C:\Users\badtime\Desktop\deepseek-harness\srm-selfaim\`。
> 所有行号对应当前锁定版本（`5c8d362`）。
> 本文只讲**代码怎么组织的、每个功能在哪几行**，不复制代码。

---

## 一、先看全局：一帧数据怎么走完

```
                        ┌─────────────────────────────────────────────┐
  相机 1440x1080        │  main.cpp 的主循环                          │
  BayerRG8              │                                             │
      │                 │  while (g_stop == 0)                        │
      ▼                 │    frame = camera.read()      ← hik_camera   │
  hik_camera.hpp ───────┤    boxes = yolo->detect()     ← yolo_detector │
      │                 │    armors = yolo_to_armors()  ← yolo_armor    │
      ▼                 │    cand[i].solve = solve_pnp()← solver        │
  yolo_detector.hpp     │    tr = tracker.update()      ← tracker       │
      │                 │    follow = selector.pick()   ← select        │
      ▼                 │    aim = aimcore.step()       ← aim_core      │
  yolo_armor.hpp        │    gimbal.track()             ← gimbal        │
      │                 │    fire_ok = fire_decision()  ← fire          │
      ▼                 │    link.set_target()          ← protocol      │
  solver.hpp            │    imshow / LOG                             │
      │                 └─────────────────────────────────────────────┘
      ▼                                   │
  tracker.hpp                             ▼
      │                          串口 18 字节/帧, 100 Hz
      ▼                                   │
  select.hpp                              ▼
      │                            下位机（火控）
      ▼                                   │
  aim_core.hpp  ── theta_yaw/pitch ──► gimbal.hpp
                                          │
                                          ▼
                                    protocol.hpp
```

**一句话概括：**
> 相机看到装甲板 → PnP 算出"板偏多少度" → 决定瞄哪块 → 加补偿 → 变成云台角 → 发给下位机 → 偏差归零。

**闭环的含义**：`theta` 是"板相对光轴的角度"，云台转过去它就会变小。
所以控制目标永远是 **让 `theta` → 0**（盯圆心策略下是"让圆心 → 0"）。

---

## 二、文件清单：每个文件干什么，功能在哪

### 2.1 配置层

#### `include/config.hpp`（约 660 行）
**唯一的参数总表**。所有魔法数字都在这里，改参数不用翻代码。

| 参数组 | 内容 |
|---|---|
| 相机 | `CAM_FX/FY/CX/CY`、畸变 `CAM_K1/K2/P1/P2/K3`、`CAM_EXPOSURE_US`、`IMG_DIR` |
| 几何 | `ARMOR_W=135`、`ARMOR_H=56` |
| 检测 | `YOLO_CONF_THRESH`、`YOLO_NMS_THRESH`、`YOLO_PROFILE_SELECT` |
| 跟踪 | `TRACK_ASSOC_PX`、`TRACK_CONFIRM_FRAMES`、`MAX_LOST_FRAMES` |
| 选板 | `SELECT_SAME_TARGET_DEG` |
| 滤波 | `KF_ENABLE`、`KF_LATENCY_S`、`KF_Q_ANGLE`、`KF_Q_RATE`、`KF_R_ANGLE` |
| 延迟补偿 | `KF_LEAD_FLIGHT_TIME`、`KF_LEAD_PITCH`、`KF_LEAD_LP_COEF`、`KF_LEAD_SUPPRESS_DEG`、`KF_MAX_LEAD_DEG` |
| 换板 | `KF_SWITCH_JUMP_DEG` |
| 盯圆心 | `CENTER_HOLD_ENABLE`、`CENTER_LP_COEF`、`CENTER_PRED_ENABLE`、`CENTER_DEV_RATE_LP` |
| 伺服 | `SERVO_DIR_YAW/PITCH`、`SERVO_GAIN`、`SERVO_MAX_CORR_DEG`、`SERVO_CMD_SLEW_DEG`、`GIMBAL_DEADBAND*`、`GIMBAL_PITCH_MIN/MAX` |
| 开火 | `ENABLE_FIRE`、`FIRE_TOL_FRAC`、`FIRE_LOCK_FRAMES`、`FIRE_HOLD_FRAMES`、`FIRE_MAX_DIST_M` |
| 显示 | `DISPLAY_EVERY_N`、`DISPLAY_SCALE` |

**读法**：先读这个文件，每条注释都写了"为什么是这个值"和踩过的坑。
**这是全项目信息密度最高的文件。**

---

### 2.2 感知层

#### `include/hik_camera.hpp`（219 行）
海康工业相机的封装：枚举设备、设置像素格式 `BayerRG8`、固定曝光/增益、取帧、Bayer 转 BGR。

- 相机打不开时返回 `-2147483133`（`MV_E_ACCESS_DENIED`）= **被别的进程占用**
- `IMG_DIR = 3` 是 180° 旋转，**只对实相机帧生效**（对文件输入不生效，否则离线测试全废）

#### `include/yolo_detector.hpp`（206 行）
OpenVINO 推理封装。核心是 `YoloProfile` 结构，描述**模型的输出布局**：

```cpp
struct YoloProfile {
    std::string model_xml;
    int   input_size;      // 416 / 640
    int   class_num;
    int   class_start;     // 类别分数在输出里的偏移
    int   corner_start;    // 4 个 keypoint 的偏移
    bool  class_is_logit;  // 是否已过 sigmoid
    const char* class_names[...];
};
```

- `YOLO_PROFILE_YOLOV8_2CLASS`：416 输入，输出 `[1,14,3549]`（3549 = 52²+26²+13²，**这个数字反证输入是 416**）
- `YOLO_PROFILE_YOLO11_38CLASS`：640 输入，输出 `[1,50,8400]`

**这里是"换模型最容易踩坑"的地方** —— 三套模型的输出布局完全不同。

#### `include/yolo_armor.hpp`（132 行）
把 YOLO 的原始输出（分数 + 4 个角点）转成 `Armor` 结构，并按 `enemy` 颜色过滤。

- **关键**：4 个 keypoint 的顺序要归一化（`sort_armor_points`），否则 PnP 会解出翻转的位姿
- `bars=0.0` 这个日志字段是历史遗留（灯条法已删），恒为 0

#### `include/armor.hpp`（251 行）
装甲板的数据结构 + 几何工具：`Armor`、`ArmorType`、角点排序、四点退化检查（近共线时 PnP 会给出完全错误的位姿）。

---

### 2.3 解算层

#### `include/solver.hpp`（198 行）

| 功能 | 位置 |
|---|---|
| PnP 用的物体点（**135 × 56**） | `armor_object_points()` :48 |
| 位姿解算（含连续性 + 重投影校验） | `solve_pose()` :92 |
| 位姿 → 角度/距离 | `solve_pnp()` :171 |

**`solve_pose()` 里的三个设计点：**

1. **用上一帧位姿做初值**（`SOLVEPNP_ITERATIVE` + `useExtrinsicGuess`）
   原因：平面近正方形目标的 IPPE 有**两个解**（正/背面），重投影误差几乎相同，
   每帧独立求解会跳变 → 实测 `pitch` 摆 ±14°。

2. **重投影误差校验**（超过 3 px RMS 就丢弃初值重解）
   原因：初值会让 ITERATIVE 在大跳变时卡住 —— 实测 1m 的靶标解成 1.86m。

3. **IPPE 兜底**：没有上一帧、或初值解校验不过时用 IPPE。

**结果结构** `SolveResult{yaw, pitch, dist, px, py, pz, yaw_raw, ok}`：
- `yaw/pitch` = 相机系偏角（度）
- `yaw_raw` = 板面朝向（正对时接近 0），用于选板
- `dist` = 距离（米）

---

### 2.4 跟踪层

#### `include/tracker.hpp`（172 行）
跨帧关联：把这一帧的检测和上一帧的目标配对（`TRACK_ASSOC_PX` 像素门限），
连续 `TRACK_CONFIRM_FRAMES` 帧配上才算 `locked`，`MAX_LOST_FRAMES` 帧配不上就丢弃。

输出 `TrackResult{cand_index, locked, state, lost_count}`。

⚠️ **`locked` 这个标志对外不可靠**：旋转靶的检测是间歇的，`locked` 会在 0.2~1.0 之间飘。
所以开火门控**不用它**（见阶段 3）。

#### `include/select.hpp`（84 行）
多块板时选哪一块。

`pick()` :30 —— 代价函数：
```cpp
cost = 0.15 * |yaw_raw|     // 板面朝向（次要）
     + 1.0  * |theta - last| // 与上次选中的连续性（主导）
```
外加 `kSwitchMargin = 2.5`（新目标要好几倍才换）。

**两个踩过的坑写在注释里：**
- 连续性权重原来是 0.5，导致两块板来回切 → `theta` 跳 5~7° → 卡尔曼角速度炸掉
- 曾经在这里 `return -1`（"等几帧别乱切"），结果等于告诉主程序"没有目标"，把有效检测丢了

#### `include/kf.hpp`（165 行）
角度卡尔曼，两状态（角度 + 角速度）。

| 函数 | 位置 | 干什么 |
|---|---|---|
| `predict(dt)` | :55 | 状态外推 |
| `update(z)` | :93 | 测量更新，带 12° 新息门限 |
| `reseed(z)` | :84 | **换板时重设角度、保留角速度** |
| `predict_ahead(L)` | :135 | 外推 L 秒，带门限和上限 |
| `rate_trusted()` | :158 | 收敛 ≥5 帧才信任角速度 |

**`reseed` 的物理依据**：两块板在同一旋转体上，**角速度相同**，只有角度不同。
所以换板只重设角度。用 `reset()` 会把角速度清零 → 提前量从 9.5° 掉到 0 再涨回来。

**单位**：`omega_` 是**度/秒**，不是弧度。这里我搞错过一次（日志乘了 57.3）。

---

### 2.5 决策层

#### `include/aim_core.hpp`（149 行）
**瞄准决策的唯一实现** —— `aim_gui` 已调用它。

`AimCore::step(AimInput)` :55，输入本帧测量量，输出送伺服的角度：

```cpp
struct AimInput {
    float raw_yaw, raw_pitch;   // 选中板的偏角
    int   follow;               // 选中下标, -1 = 无目标
    float dist_m, bullet_spd;
    float dt;
    float cmd_yaw, cmd_pitch;   // 上一条命令（判断云台是否在追赶）
    float fb_yaw,  fb_pitch;    // 实测姿态
    bool  fb_fresh;
};
```

`step()` 内部四段：

| 段 | 位置 | 内容 |
|---|---|---|
| ① 换板检测 | :64 起 | 下标变了或 `theta` 跳 >4° |
| ② 预测外推时长 | ~:70 | `t_lead = 链路 80ms + 距离/弹速` |
| ③ 卡尔曼 + 提前量 | ~:95 | 换板 reseed、追赶抑制、提前量低通 |
| ④ 盯圆心 | ~:120 | 圆心平均、偏差外推、夹紧圆内 |

**⚠️ 重要现状**：`main.cpp` 里**还有一份等价的内联副本**（:719 起）。
`AimCore` 是从它逐行抽出来的，目前两者逻辑一致。
**但如果只改一边就会分叉** —— 这是已知待办。

#### `include/fire.hpp`（47 行）
纯数学，可离线测试。

| 函数 | 位置 |
|---|---|
| `plate_angular_width_deg(dist)` | :21 |
| `fire_tolerance_deg(dist)` | :29 |
| `fire_decision(...)` | :35 |

**容差随距离缩放**（同一个角度误差在不同距离含义完全不同）：

```
2m → 板角宽 4.0°，容差 0.45×4.0 = 1.8°
5m → 板角宽 1.55°，容差 0.45×1.55 = 0.70°
8m → 板角宽 1.0°，容差 0.45×1.0 = 0.45°
```

**⚠️ 单位陷阱（我连续错两次）**：
`plate_angular_width_deg()` 返回**全宽**，而 `fire_tolerance_deg()` 的值是**半角**。
所以 `FIRE_TOL_FRAC > 0.5` 就等于"允许瞄准点偏出板外"。
0.80 = 离板心 108mm（板半宽 67.5mm）→ 必然打空隙。

---

### 2.6 执行层

#### `include/gimbal.hpp`（476 行）
**伺服控制律 —— 全项目最该读懂的文件。**

| 函数 | 位置 |
|---|---|
| `track(theta_yaw, theta_pitch)` | :156 |
| `stop_tracking()` | :340 |
| `resync_switch(y, p)` | :146 |
| `apply_envelope()` | :414 |
| `smooth(in, state)` | :485 |

**`track()` 的控制律（五步）：**

```
1) 偏角低通   ty = smooth(theta)                     // smooth_coef = 0.35
2) 去偏        ty_aim = ty - aim_offset              // 枪口与光轴不平行
3) 限幅修正    cy = clamp(DIR * ty_aim * gain * ramp, ±6°)
4) 命令        cmd = meas_lp + cy                    // meas_lp = 实测角低通(0.10)
5) 限位 + 变化率限制  cmd = clamp(cmd, 行程); |cmd - prev| <= 3°/帧
```

**四个设计点，每个都是踩坑换来的：**

| 设计 | 为什么 |
|---|---|
| `cmd = meas_lp + 限幅修正`（不是累积步进） | 累积步进会让 `cmd` 跑到 `meas` 前面 → 过冲 30° 把靶标甩出画面 |
| 基准用**低通后的实测角** | 直接用 `meas` 会把编码器噪声继承进命令 → 极限环 |
| `resync_switch()` | 换板时把 `meas_lp` 和偏角低通**对齐到真值**，否则基准滞后会让命令"幻影前进" → 过冲 |
| 3°/帧变化率限制 | 削单帧突跳（抽动），稳态变化远小于此所以不影响跟踪 |

**注意 `apply_envelope()` 的包络在 config 里已被关闭**（`GIMBAL_ENVELOPE_DEG = 180`）。

#### `include/protocol.hpp`（350 行）
串口协议（《SRM 校内赛自瞄通信协议》）。

| 功能 | 位置 |
|---|---|
| 组发送帧 | `pack_tx()` :96 附近 |
| `SerialLink::set_target()` | :229 附近 |
| 接收线程 / 解析 | :278 / :293 / :306 |
| 状态结构 `Feedback` | :334 附近 |

**报文格式（固定长度，无魔数无 CRC）：**

```
上→下 18 字节: body_len=16, ID1 = yaw/pitch (f32 x2), ID2 = fire_flag (i32)
下→上 30 字节: body_len=28, ID1 = yaw/pitch/roll + mode/color,
                          ID2 = bullet_speed (f32 x1)
```

- 独立 100 Hz 发送线程（`SERIAL_SEND_HZ`）
- **`mode` 由遥控器决定**，`mode=0` 时下位机完全忽略 ID1
- **`fire_flag` 只是"建议"**，实际发射需火控允许 + 人工许可

#### `include/synth.hpp`（89 行）
合成靶标：按 `ARMOR_W/ARMOR_H` 投影出 4 个角点，喂给完整链路做离线验证。测试全靠它。

---

### 2.7 程序入口

#### `src/main.cpp`（1067 行）—— **实车运行的主程序**

| 功能 | 行号 |
|---|---|
| 命令行解析 | 200~260 |
| 加载 `aim_params.cfg` | ~370 |
| 打印 `[EFFECTIVE]` 全量参数 | :410 |
| 启动串口 + 相机 | 430~520 |
| **YOLO 推理** | :546 |
| 候选解算 + 跟踪 | ~600~680 |
| **换板检测** | ~700 |
| **圆心更新** | :719 |
| **预测夹紧（s_dev_amp）** | :824 |
| **开火判据** | :947 |
| `[FIRE-DBG]` 诊断行 | :975 |
| **下发命令** | :989 |

**参数优先级**：命令行 > `aim_params.cfg` > `config.hpp`

#### `src/aim_gui.cpp`（593 行）—— **可视化调参**

三个窗口：`camera`（图 + 检测框）、`params`（发什么/收什么/差多少）、`knobs`（滑条）。

- 滑条 `x10` 编码：`off_*` 的 100 = 0 度；`dir_*` 的 0 = -1
- **按 `s` 保存到 `aim_params.cfg`，按 `q` 退出**
- 瞄准决策已改为调用 `AimCore`（与实车同一份实现）
- `createTrackbar` 只拷贝初值，所以 `load_knobs()` 后必须 `sync_trackbars()`

#### 其它工具（都在 `src/`）

| 文件 | 用途 |
|---|---|
| `cam_view.cpp` | 只看相机和检测，不动云台 |
| `detect_probe.cpp` | 只打印检测结果（离线排查 YOLO） |
| `gimbal_test.cpp` | 单独测云台通道（`--query` / `--step` / `--hold`） |
| `servo_calib.cpp` | **测量伺服方向符号**（当初靠它定出 yaw=-1/pitch=+1） |
| `calibrate.cpp` | 相机内参标定 |
| `yolo_test.cpp` | 单独测模型加载 |

---

## 三、自瞄逻辑：四个阶段

### 阶段 1：检测（`yolo_detector` + `yolo_armor`）

```
1440×1080 BayerRG8
   ↓ Bayer → BGR
   ↓ resize 到 416×416
   ↓ OpenVINO 推理
   ↓ 输出 [1,14,3549]
   ↓ 解出 N 个 (类别, 置信度, 4 个角点)
   ↓ 按敌我颜色过滤 + NMS
Armor[] {pt[4], color, conf}
```

**判据**：`armor=1.4` 表示平均检出 1.4 块板；`armor=0` 就是没检到。

### 阶段 2：解算（`solver`）

```
4 个角点 → solvePnP(135×56 模型) → rvec, tvec
         → yaw   = atan2(x, z)
         → pitch = atan2(y, z)
         → dist  = |tvec|
```

**这一步的精度决定了后面一切。** 几何模型（135×56）错一点，`pitch` 就系统性偏。

### 阶段 3：决策（`tracker` → `select` → `aim_core` → `fire`）

```
tracker:  跨帧关联 → locked / state
select:   选哪块板 → follow
aim_core: 盯圆心 + 偏差外推 → theta_yaw / theta_pitch
fire:     该不该开火 → fire_ok
```

**盯圆心策略（当前使用）的物理依据：**

板在半径 R 处绕竖轴转，方位偏移：
```
θ(φ) = atan( R·sinφ / d )
φ=0（正对镜头）时 θ=0  ←  板正对时方位恰好等于圆心方位
```

所以**云台盯住圆心，板会自己转到瞄准点上**。

精度余量：
```
dθ/dφ = R/d ≈ 0.2/5 = 0.04
飞行 0.24s 内转 7.2° → 方位只变 0.29°  （板宽 1.55°）
```

**开火门控**：用"本帧是否检出板"（`follow >= 0`）而不是 `tr.locked`，
质量过滤交给连续帧数 `FIRE_LOCK_FRAMES = 3`。

### 阶段 4：执行（`gimbal` → `protocol`）

```
aim_core 输出 theta
   → gimbal.track() 算出 cmd_yaw/cmd_pitch
   → protocol.set_target(cmd_yaw, cmd_pitch, fire_flag)
   → 100 Hz 线程发 18 字节
   → 下位机执行（mode=1 才理会）
   → 回传 30 字节（含实测 yaw/pitch/roll）
```

**闭环就在这里合上**：回传的实测角变成下一帧的 `meas`，`theta` 因此变小。

---

## 四、相关功能对应哪些测试

**8 套单元测试，全部可离线跑：**

```bash
# WSL 里
cd ~/my_aim && ctest --test-dir build
# 或单个跑
./build/tests/test_solver
```

| 测试 | 测什么 | 对应源码 |
|---|---|---|
| `tests/test_solver.cpp` | PnP：距离扫描 1~7m、角度符号、退化拒绝 | `solver.hpp` |
| `tests/test_armor.cpp` | 角点排序、颜色过滤、退化判定 | `armor.hpp` `yolo_armor.hpp` |
| `tests/test_tracker.cpp` | 跨帧关联、确认帧数、丢失帧数 | `tracker.hpp` |
| `tests/test_kf.cpp` | 匀速跟踪精度、噪声抑制、离群拒绝、限幅 | `kf.hpp` |
| `tests/test_closed_loop.cpp` | **完整闭环**：合成靶标 → 检测 → 解算 → 控制 → 收敛 | 全链路 |
| `tests/test_fire.cpp` | 容差随距离缩放、各条否决条件 | `fire.hpp` |
| `tests/test_protocol.cpp` | 18/30 字节报文、字段偏移、小端 | `protocol.hpp` |
| `tests/gimbal_edge.cpp` | 行程限位、死区、极限环、发散保护 | `gimbal.hpp` |

**调试用工具（需要硬件或视频）：**

| 工具 | 用法 |
|---|---|
| `tests/box_probe` | `--img 图.jpg` / `--video 视频.mp4` 打印检测框宽高比，验证 `ARMOR_H` |
| `tests/synth_dump` | 把合成靶标存成图片，看检测器认不认 |
| `detect_probe` | 离线跑检测，看框和置信度 |

---

## 五、五种调试方法

| 方法 | 命令 | 用途 |
|---|---|---|
| **可视化调参** | `./build/aim_gui` | 看图 + 拖滑条，首选 |
| **日志** | `./build/my_aim --fire --debug` | 看 `th` 是否收敛、`[FIRE-DBG]` 哪条不满足 |
| **离线测试** | `ctest --test-dir build` | 改完代码先跑，比上车快 10 倍 |
| **单工具** | `./build/gimbal_test --step yaw 20` | 只验一个通道 |
| **合成靶标** | `./build/my_aim --synth --bearing 20` | 不碰硬件验证锁定 |

### 日志字段怎么读

`[DET]` 行：
```
f=540         帧号
armor=1.4     平均检出板数（0 = 没检到）
locked=1.0    跟踪置信度（**不可靠，仅供参考**）
state=TRACK   跟踪状态
want=-38.4    命令角
cmd=321.6     （含回绕的）命令角
fb=-38.2      下位机回传实测角
mode=1        自瞄开关（遥控器控制）
th=0.3/3.7    当前偏角 yaw/pitch  ← **最重要的两个数**
lock_age=0    丢了几帧
```

`[LEAD]` 行：
```
外推 324 ms (链路80 + 飞行244)   总外推时长
提前量 0.00/0.02 度             实际施加的补偿（盯圆心下 yaw 恒 0）
角速度 0.04/0.09 度每秒
dist=5.13 m
```

`[FIRE-DBG]` 行：
```
locked=1  streak=3  err=0.42  tol=1.13  dist=5.50  speed=22.0
                     ↑         ↑
              实际偏差   容差(半角)
```
**`err <= tol` 且 `streak >= FIRE_LOCK_FRAMES` 才会开火。**

---

## 六、故障对照表

| 现象 | 查哪里 | 具体 |
|---|---|---|
| 相机打不开 / `-2147483133` | 进程占用 | `pkill -f 'build/'` |
| `mode=0` 云台不动 | 不是 bug | 遥控器没开自瞄，下位机忽略指令 |
| 一直 `[FIRE] 停火` | `[FIRE-DBG]` 行 | 看 `locked/streak/err/tol/dist` 哪条不满足 |
| `err` 恒定远大于 `tol` | 圆心估计 | 圆心被换板带偏 → `CENTER_LP_COEF` |
| 打在两板之间 | `FIRE_TOL_FRAC` | >0.5 就允许偏出板外 |
| 云台抽动 | `SERVO_GAIN` | 1.0 会进极限环，应为 0.80 |
| 上下抖 | pitch 提前量 | `KF_LEAD_PITCH` 必须 false |
| 换板后过冲 | `resync_switch()` | `meas_lp` 滞后导致命令幻影前进 |
| 换板后抽动 | `KF_LEAD_SUPPRESS_DEG` | 追赶时 `theta` 变化不可信 |
| `pitch` 摆 ±14° | `solve_pose()` | PnP 双解翻转，靠上一帧初值 |
| 距离解不准 | `ARMOR_W/H` | keypoint 标的是灯条区域 135×56，不是板 140×125 |
| 画面卡但追踪正常 | `DISPLAY_EVERY_N` | 显示节流，不影响控制 |

---

## 七、验收可能被问的问题

**Q：为什么 `ARMOR_H` 是 56 而不是板高 125？**
> YOLO 的 4 个 keypoint 标的是**灯条区域**的四角，不是整块板。灯条长 56mm。
> 对齐同济的 `SMALL_ARMOR_POINTS{135, 56}`。交叉验证：5.4m 处 box 47×19px，
> 理论宽 140×1807/5400=47px ✓、高 56×1807/5400=19px ✓。
> 填 125 会让 PnP 拿宽高比 1.12 去拟合 2.55 的四边形，只能强行加俯仰 → pitch 不准。

**Q：为什么用盯圆心而不是追最优板？**
> 追板要求精确预测转速与相位，任何误差都变成打空隙（实测低速 75%，高速不可用）。
> 盯圆心利用"板正对时方位=圆心方位"这个几何事实，而且圆心附近 `dθ/dφ = R/d = 0.04`，
> 飞行期间方位只变 0.29° 而板宽 1.55° —— **容差是误差的 5 倍**。
> 不需要精确转速。

**Q：为什么不用完整 EKF？**
> 做过（同济 11 维模型，编译通过），实测预测误差 **1.53°**，
> 而 5m 处板半宽只有 **0.77°** —— 精度不够。
> 根因是**可观性**：6 rad/s 下每帧板只位移 12mm，PnP 位置噪声约 40mm。

**Q：两个伺服轴符号为什么相反？**
> "命令角增大"对两轴是相反的物理动作：yaw 增大向左，pitch 增大向下。
> 靠 `servo_calib` 实测确定：yaw = -1（偏角 5.26° → 0.51°），pitch = +1。

**Q：下坠怎么补偿？**
> **不用公式**。5m 是固定距离，253mm 下坠是常数，并进 `AIM_PITCH_OFFSET_DEG = 3.7`。
> 不依赖弹速（裁判系统未接入时回传 0.00，用公式反而引入误差）。

**Q：为什么 `fire_flag` 只是建议？**
> 协议规定：视觉侧只能表达"建议开火"，实际发射需**火控允许 + 人工许可**。
> 所以 `ENABLE_FIRE` 默认 false，必须 `--fire` 才允许输出 1。

**Q：射速能达标吗？**
> 实测 25 发/30 秒 = 0.833 发/秒，要求 50 发/60 秒 = 0.833 发/秒 —— **正好达标，余量为零**。
> 开火窗口占空比 38%（`FIRE_TOL_FRAC=0.43`），理论上远超所需，
> 所以瓶颈更可能在火控射速或人工许可键，不在视觉侧。

---

## 八、建议的阅读顺序（约 2 小时）

```
0:00-0:15   config.hpp          参数总表，理解每个量的物理含义
0:15-0:25   fire.hpp            47 行，最短，先建立"容差"的概念
0:25-0:45   solver.hpp          135×56 模型 + PnP 三个设计点
0:45-1:05   kf.hpp              reseed / predict_ahead / 单位
1:05-1:25   aim_core.hpp        盯圆心 + 偏差外推
1:25-1:55   gimbal.hpp          ★ track() 的五步控制律，必须能默写
1:55-2:10   main.cpp            只看 :546 / :700 / :947 / :989 四处
2:10-2:20   tests/              跑 ctest，看每套测什么
```

**`gimbal.hpp` 的 `track()`（:156）是必须能默写的一段** —— 那是整个项目的核心。

---

## 九、自测题

1. `theta` 是什么？为什么让它趋于 0 就是控制目标？
2. `cmd = meas_lp + 修正` 为什么不能写成"累积步进"？
3. `reseed()` 和 `reset()` 在换板时的行为差别是什么？为什么前者对？
4. `FIRE_TOL_FRAC = 0.8` 为什么必然打空隙？（算一下 5m 处离板心多少 mm）
5. 盯圆心策略为什么不需要精确转速？用 `dθ/dφ` 解释。
6. `KF_LEAD_PITCH = false` 的原因是什么？
7. PnP 的"两个解"指什么？怎么保证不翻转？
8. `ARMOR_H = 56` 和板高 125 的关系？
9. 开火判据里 `fire_flag` 为什么只是"建议"？
10. 8 套测试分别覆盖哪一层？如果改 `gimbal.hpp` 该跑哪个？

**答案都能在本文档和代码注释里找到。**

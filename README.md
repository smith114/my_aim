# my_aim —— SRM 校内赛自瞄上位机

上海大学 SRM 战队算法校内赛（自瞄赛道）上位机程序。
完成从**目标检测 → 目标解算 → 云台瞄准 → 发射反馈**的闭环。

- 赛道规则：见《上海大学 SRM 战队算法校内赛规则书》
- 通信协议：见《SRM 校内赛自瞄通信协议》——本仓库的协议实现是唯一依据
- 时间节点：**10-17 中期检查（30%）**、10-31 最终验收（70%）

---

## 1. 它做什么

```
海康 MV-CS016 USB 相机
      │  1440x1080 Bayer
      ▼
  图像方向校正 (IMG_DIR)
      │
      ▼
  灯条提取 (HSV 色调分割 + 形态学 + 细长条筛选 + BGR 纯度过滤)
      │
      ▼
  装甲板配对 (同色/等高/对齐/间距比例/平行/中心暗区)
      │
      ▼
  PnP 解算 (IPPE)  →  距离 + 相机系偏差角 (θ_yaw, θ_pitch)
      │
      ▼
  目标跟踪 (DETECT/TRACK/LOST 三态状态机)
      │
      ▼
  云台伺服 (期望角 = 实测角 + SERVO_DIR × 偏差角, 限步 + 低通 + 死区)
      │
      ▼
  USB CDC 100 Hz 定频单帧发送  ──►  下位机
      ▲
      └── 接收线程流式解帧: 实测 yaw/pitch/roll + 弹速
```

## 2. 目录结构

```
include/
  config.hpp        所有可调参数（实车调参只改这个文件）
  protocol.hpp      协议组帧/流式解帧/收发线程/看门狗
  armor.hpp         灯条提取 + 装甲板配对 + 角点排序
  solver.hpp        PnP 解算 + 偏差角 + 弹道补偿
  tracker.hpp       三态状态机
  gimbal.hpp        云台伺服（含 SERVO_DIR 自检）
  hik_camera.hpp    海康 MVS SDK 取流
  yolo_detector.hpp OpenVINO YOLO11 检测器（备选方案，不在主流程）
  synth.hpp         合成靶标生成器（测试用，无需相机）
  test_util.hpp     极简测试宏
src/
  main.cpp          主程序
  calibrate.cpp     相机内参标定（支持 headless 采集）
  detect_probe.cpp  检测链路诊断工具（逐级打印过滤结果）
  yolo_test.cpp     OpenVINO 环境自检
tests/
  test_protocol.cpp     协议逐字节一致性 + 流式解帧边界
  test_solver.cpp       PnP 精度（合成位姿往返）
  test_armor.cpp        检测端到端（合成靶标）
  test_tracker.cpp      状态机行为（含已修 bug 的回归）
  test_closed_loop.cpp  闭环仿真（收敛性）
  synth_dump.cpp        合成图导出（诊断用）
thirdparty/hikrobot/    海康 MVS SDK（头文件 + libMvCameraControl.so）
assets/yolo11.xml|bin   YOLO11 OpenVINO 模型
```

---

## 3. 构建

依赖：CMake ≥ 3.16、GCC（C++17）、OpenCV 4、pthread、**完整的海康 MVS SDK**。
主程序不依赖 OpenVINO（YOLO 是可选分支）。

> **必须装完整版 MVS SDK（默认 `/opt/MVS`）。**
> 项目自带的 `thirdparty/hikrobot/lib/libMvCameraControl.so` 只有 6.9 MB，
> **不带 USB3 Vision 传输层**，`MV_CC_EnumDevices` 会返回 `nDeviceNum=0`
> —— 表现为"`lsusb` 能看到相机，程序却说没相机"。
> CMake 会优先链接 `/opt/MVS/lib/64`，找不到才回退并打 WARNING。
> 确认实际链接的是哪个：`ldd ./build/my_aim | grep -i mv`。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2          # WSL 内存紧, 用 -j2
```

产物：`build/my_aim`、`build/calibrate`、`build/tests/*`。
头文件用项目自带的（与系统 SDK 同为 4.7.0.3，ABI 一致）。

### 跑测试（不需要相机、不需要下位机、不需要实车）

```bash
ctest --test-dir build --output-on-failure
```

5 套测试全部离线可跑，覆盖协议解析、PnP 精度、检测、状态机、闭环收敛。
**验收现场改完参数先跑一遍这个再上车。**

---

## 4. 运行

```bash
./build/my_aim                        # 实车：海康相机 + 下位机
./build/my_aim --debug                # 额外画出灯条与配对结果
./build/my_aim --headless             # 不开窗口（SSH 远程调试）
./build/my_aim --mock                 # 不打下位机，只跑逻辑
./build/my_aim --frames 300           # 跑 300 帧后退出
./build/my_aim clip.mp4 --headless    # 用录像离线跑（回归测试用）
./build/my_aim 照片.jpg               # 单张图片跑一遍并输出标注图
```

日志双写：终端 + `/tmp/myaim.log`。

### 关键运行期行为

| 行为 | 说明 |
| --- | --- |
| 定频发送 | 启动即 100 Hz 发送，即使还没有目标（协议 6.1/6.2 要求） |
| 看门狗 | 控制量超 500 ms 未更新 → 冻结在最后一个有效角并撤掉开火请求 |
| 退出补发 | Ctrl-C / 正常退出 / 异常都会补发 20 帧零命令 |
| 无回传不乱动 | 没收到下位机实测角时 `track()` 直接返回 false，绝不拿错误基准指 |
| 回传过期 | 实测角超过 250 ms 未更新 → 冻结命令并在日志里告警 |
| 开火 | `ENABLE_FIRE=false`，中期一律 `fire_flag=0` |

---

## 5. 上车前必须确认的四件事

### ① 云台角量程（**当前最不确定的一项**）

协议只说"角度制"，没说量程。**实机实测回传值是负的**（`yaw=-90.6, pitch=-38.3`），
所以旧注释里"0~180，中间 90 = 镜头朝前"是错的。

现在 `config.hpp` 按"不做人为限位"处理（yaw 整圈、pitch ±180），
把保护交给下位机的机械限位（规则 3.3 节：机械限位由下位机保证）。
**实测出真实量程后请改成精确区间**，这样异常指令能更早被发现。

### ② 云台方向符号 `SERVO_DIR`

`SERVO_DIR` 表示"增大 yaw 命令时，目标在画面里朝哪边移动"：

- `+1`：目标在画面右侧时需要增大 yaw
- `-1`：反之

控制律是 `期望角 = 实测角 + SERVO_DIR × 偏差角`，**锚在下位机实测角上**，
所以即使符号反了也不会发散（只会绕不动/来回摆），比旧版"自己积分估计角"安全得多。

程序默认开启自检（`auto_dir_check`）：按当前方向转一帧后如果偏差角反而变大，
就自动翻转并打日志 `[DIR] SERVO_DIR 反了, 已自动翻转`。想手动固定用 `--no-dir-check`。

**首次上车务必低速验证。**

### ③ 相机 Bayer 相位

传感器输出 Bayer 原始格式，相位判错会让颜色彻底反过来，蓝/红灯条的色调分割直接失效。
**自动判定在实机上不可靠**（三种判据互相矛盾，详见 `DEBUG_LOG.md`），所以：

```bash
./build/my_aim --bayer rg        # 或 bg / gr / gb
# 窗口里也可以按 T 键循环切换
```

判对错的快捷办法：对蓝靶标跑 `detect_probe -v`，看灯条 BGR 均值是否 B 明显大于 G 和 R。
默认值见 `config.hpp` 的 `BAYER_DEFAULT_NAME`。

### ④ 图像方向 `IMG_DIR`

`0=原样 1=水平翻转 2=垂直翻转 3=旋转180`。当前是 **3**（2026-10-15 实机试出）。
测试里用 `-DIMG_DIR_OVERRIDE=0` 覆盖，因为合成图是正的。

### 附：相机内参与光照阈值

内参是 2026-10-15 标定值（重投影误差 0.097 px）。换镜头/重新对焦后必须重标：

```bash
./build/calibrate                                   # 交互模式
./build/calibrate --headless --shots 25 --out /tmp/calib   # 远程采集
./build/calibrate --compute /tmp/calib --cols 11 --rows 8  # 离线计算
```

灯条阈值（`LB_*`）和中心暗区阈值（`ARMOR_CENTER_V_MAX`）与现场光照强相关，
比赛当天需要按实际光照微调。诊断工具：

```bash
./build/tests/detect_probe 某帧.jpg out.jpg -v
```

它会打印**逐级过滤**结果（形态学后轮廓数、面积、长短边、长宽比、BGR 均值、
每一级阈值是否通过），直接回答"为什么这一帧没检出"。

---

## 6. 调试纪律（血泪教训）

1. **先跑测试再看车。** 车上改一行就上电试，出了问题是相机、云台还是算法根本分不清。
2. **改完参数先 `ctest`。** 闭环仿真能在 30 秒内告诉你控制律是否还收敛。
3. **合成图是最便宜的复现手段。** `synth.hpp` 能按指定位姿生成靶标图，
   `detect_probe` 能告诉你哪一级阈值卡掉了目标。
4. **不要在实车上调"检测不到"的问题。** 先把录制帧拿回来用 `detect_probe -v` 分析。
5. **符号问题一律实测，不要推断。** 轴向、旋转方向、俯仰正负都各试一次并存日志。

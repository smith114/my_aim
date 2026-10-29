# 参数 → 代码位置对照表

> 自动生成，用于回答"这个参数到底在哪起作用"。
> 只列**实际使用点**，不含 config.hpp 里的定义本身。

## 一、调参最常动的

| 参数 | 当前值 | 首次作用点 |
| --- | --- | --- |
| `SERVO_DIR_YAW` | -1.0f | `include/gimbal.hpp:42` |
| `SERVO_DIR_PITCH` | 1.0f | `include/gimbal.hpp:52` |
| `SERVO_GAIN` | 1.0f | `include/gimbal.hpp:45` |
| `SERVO_MAX_CORR_DEG` | 6.0f | `src/aim_gui.cpp:507` |
| `AIM_YAW_OFFSET_DEG` | 0.0f | `include/gimbal.hpp:50` |
| `AIM_PITCH_OFFSET_DEG` | 3.7f | `include/gimbal.hpp:51` |
| `GIMBAL_DEADBAND` | 0.0f | `include/gimbal.hpp:49` |
| `GIMBAL_SMOOTH` | 0.35f | `include/gimbal.hpp:48` |
| `YOLO_CONF_THRESH` | 0.45f | `src/yolo_test.cpp:52` |
| `TRACK_ASSOC_PX` | 60.0f | `include/tracker.hpp:75` |
| `TRACK_CONFIRM_FRAMES` | 2 | `include/tracker.hpp:117` |
| `SELECT_SAME_TARGET_DEG` | 12.0f | `include/select.hpp:43` |
| `KF_LATENCY_S` | 0.080f | `src/main.cpp:428` |
| `KF_ENABLE` | true | `src/main.cpp:428` |
| `FIRE_TOL_FRAC` | 0.6f | `src/main.cpp:432` |
| `FIRE_LOCK_FRAMES` | 8 | `src/main.cpp:431` |
| `ENABLE_FIRE` | false | `src/main.cpp:431` |
| `ARMOR_W` | 140.0f | `src/aim_gui.cpp:445` |
| `ARMOR_H` | 57.0f | `src/main.cpp:417` |
| `IMG_DIR` | IMG_DIR_OVERRIDE | `src/aim_gui.cpp:234` |
| `GIMBAL_ENVELOPE_DEG` | 180.0f | `src/main.cpp:427` |
| `GIMBAL_PITCH_MARGIN` | 2.0f | `src/main.cpp:425` |

## 二、全部参数使用统计（引用数为 0 = 死参数）

| 参数 | 使用处数 | 首次出现 |
| --- | --- | --- |
| `CAM_FX` | 5 | `src/aim_gui.cpp:423` |
| `CAM_FY` | 3 | `src/aim_gui.cpp:426` |
| `CAM_CX` | 4 | `src/aim_gui.cpp:422` |
| `CAM_CY` | 3 | `src/aim_gui.cpp:425` |
| `CAM_K1` | 2 | `src/calibrate.cpp:109` |
| `CAM_K2` | 2 | `src/calibrate.cpp:110` |
| `CAM_P1` | 2 | `src/calibrate.cpp:111` |
| `CAM_P2` | 2 | `src/calibrate.cpp:112` |
| `CAM_K3` | 2 | `src/calibrate.cpp:113` |
| `IMG_DIR` | 18 | `src/aim_gui.cpp:234` |
| `LB_HUE_LO` | 2 | `include/armor.hpp:61` |
| `LB_HUE_HI` | 2 | `include/armor.hpp:61` |
| `LB_SAT_MIN` | 3 | `include/armor.hpp:65` |
| `LB_VAL_MIN` | 3 | `include/armor.hpp:65` |
| `LB_SHORT_MIN` | 2 | `src/detect_probe.cpp:212` |
| `LB_LONG_MIN` | 2 | `src/detect_probe.cpp:212` |
| `LB_RATIO_MIN` | 2 | `src/detect_probe.cpp:213` |
| `LB_RATIO_MAX` | 2 | `src/detect_probe.cpp:213` |
| `LB_PURITY` | 4 | `src/detect_probe.cpp:214` |
| `LB_MAX_CANDIDATES` | 3 | `include/armor.hpp:132` |
| `LB_MIN_CONTOUR_AREA` | 1 | `include/armor.hpp:98` |
| `LB_MORPH_CLOSE_SIZE` | 3 | `include/armor.hpp:79` |
| `LB_MORPH_OPEN_SIZE` | 3 | `include/armor.hpp:80` |
| `PAIR_LEN_RATIO_MIN` | 2 | `src/detect_probe.cpp:254` |
| `PAIR_DY_RATIO_MAX` | 2 | `src/detect_probe.cpp:255` |
| `PAIR_DIST_RATIO_MIN` | 2 | `src/detect_probe.cpp:256` |
| `PAIR_DIST_RATIO_MAX` | 2 | `src/detect_probe.cpp:257` |
| `PAIR_ANGLE_DIFF` | 2 | `src/detect_probe.cpp:258` |
| `ARMOR_CENTER_V_MAX` | 1 | `include/armor.hpp:198` |
| `ARMOR_W` | 11 | `src/aim_gui.cpp:445` |
| `ARMOR_H` | 7 | `src/main.cpp:417` |
| `SOLVE_DIST_MIN` | 2 | `src/main.cpp:571` |
| `SOLVE_DIST_MAX` | 3 | `src/main.cpp:571` |
| `SOLVE_ANGLE_MAX` | 2 | `src/main.cpp:572` |
| `MAX_LOST_FRAMES` | 4 | `include/tracker.hpp:8` |
| `TRACK_CONFIRM_FRAMES` | 11 | `include/tracker.hpp:117` |
| `SELECT_SAME_TARGET_DEG` | 1 | `include/select.hpp:43` |
| `FIRE_LOCK_FRAMES` | 4 | `src/main.cpp:431` |
| `FIRE_TOL_FRAC` | 2 | `src/main.cpp:432` |
| `FIRE_MAX_DIST_M` | 4 | `src/main.cpp:432` |
| `FIRE_MIN_BULLET_SPEED` | 4 | `src/main.cpp:764` |
| `TRACK_ASSOC_PX` | 2 | `include/tracker.hpp:75` |
| `TRACK_REACQUIRE_GAP` | 0 | `` |
| `SERVO_DIR_YAW` | 3 | `src/servo_calib.cpp:230` |
| `SERVO_DIR_PITCH` | 2 | `src/servo_calib.cpp:230` |
| `KF_ENABLE` | 3 | `src/main.cpp:428` |
| `KF_LATENCY_S` | 4 | `src/main.cpp:428` |
| `KF_Q_ANGLE` | 5 | `src/aim_gui.cpp:194` |
| `KF_Q_RATE` | 5 | `src/aim_gui.cpp:194` |
| `KF_R_ANGLE` | 5 | `src/aim_gui.cpp:194` |
| `KF_MOVING_RATE_DPS` | 0 | `` |
| `AIM_YAW_OFFSET_DEG` | 1 | `include/gimbal.hpp:50` |
| `AIM_PITCH_OFFSET_DEG` | 1 | `include/gimbal.hpp:51` |
| `SERVO_GAIN` | 2 | `include/gimbal.hpp:45` |
| `SERVO_MAX_CORR_DEG` | 12 | `src/aim_gui.cpp:507` |
| `SERVO_SOFT_START_FRAMES` | 4 | `include/gimbal.hpp:184` |
| `GIMBAL_SMOOTH` | 2 | `include/gimbal.hpp:48` |
| `GIMBAL_DEADBAND` | 1 | `include/gimbal.hpp:49` |
| `GIMBAL_MANUAL_STEP` | 4 | `src/main.cpp:894` |
| `GIMBAL_RECENTER_DPS` | 2 | `include/gimbal.hpp:312` |
| `GIMBAL_ENVELOPE_DEG` | 13 | `src/main.cpp:427` |
| `GIMBAL_YAW_MIN` | 2 | `include/gimbal.hpp:425` |
| `GIMBAL_YAW_MAX` | 2 | `include/gimbal.hpp:425` |
| `GIMBAL_PITCH_MARGIN` | 4 | `src/main.cpp:425` |
| `GIMBAL_PITCH_MIN` | 4 | `src/main.cpp:425` |
| `GIMBAL_PITCH_MAX` | 4 | `src/main.cpp:426` |
| `ENABLE_AUTO_SCAN` | 3 | `src/main.cpp:130` |
| `GIMBAL_SCAN_AMP` | 1 | `include/gimbal.hpp:285` |
| `GIMBAL_SCAN_SPEED` | 1 | `include/gimbal.hpp:284` |
| `GIMBAL_SCAN_HOLD` | 2 | `src/main.cpp:736` |
| `FEEDBACK_MAX_AGE_S` | 1 | `src/main.cpp:634` |
| `BALLISTIC_G` | 1 | `include/solver.hpp:153` |
| `BULLET_SPEED_DEFAULT` | 5 | `src/main.cpp:490` |
| `BALLISTIC_COMP_ENABLE` | 1 | `src/main.cpp:681` |
| `SERIAL_SEND_HZ` | 7 | `src/aim_gui.cpp:160` |
| `SERIAL_WATCHDOG_MS` | 4 | `src/gimbal_test.cpp:120` |
| `SERIAL_SHUTDOWN_ZERO_FRAMES` | 4 | `src/gimbal_test.cpp:120` |
| `ENABLE_FIRE` | 4 | `src/main.cpp:431` |
| `CAM_USE_AUTO_EXPOSURE` | 1 | `include/hik_camera.hpp:145` |
| `CAM_EXPOSURE_US` | 2 | `include/hik_camera.hpp:153` |
| `CAM_GAIN_DB` | 3 | `src/detect_probe.cpp:131` |
| `CALIB_SQUARE_MM` | 2 | `src/calibrate.cpp:142` |
| `YOLO_CONF_THRESH` | 4 | `src/yolo_test.cpp:52` |
| `YOLO_NMS_THRESH` | 5 | `src/aim_gui.cpp:172` |

## 三、模块依赖

```
#include "armor.hpp"
#include "gimbal.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "solver.hpp"
#include "synth.hpp"
#include "tracker.hpp"
#include "yolo_armor.hpp"
#include "yolo_detector.hpp"
```

## 四、控制律路径（核心）

```
solver.hpp  solve_pnp()
    tvec -> theta_yaw=atan2(x,z), theta_pitch=atan2(y,z)     [相机系偏角]
        |
        v
gimbal.hpp  GimbalCtrl::track(theta_yaw, theta_pitch)
    1) 低通   meas_lp = meas_lp*(1-k) + meas*k              [抑制编码器噪声]
    2) 去偏   ty_aim  = theta - aim_offset                  [枪口与光轴不平行]
    3) 修正   cy = clamp(DIR*ty_aim*gain*ramp, ±6deg)       [唯一限幅]
    4) 命令   cmd = meas_lp + cy                            [结构上不累积]
    5) 限位   pitch clamp -> [-33.8, +24.1]                 [防堵转]
        |
        v
protocol.hpp  pack_tx(cmd_yaw, cmd_pitch, fire_flag) -> 18 字节
```

**记住一句: cmd = 实测角(低通) + 限幅修正 → 结构上不可能累积过冲。**

// config.hpp: 所有可调参数集中在这里, 实车调参只改这个文件
#pragma once

namespace srm {

// ============================================================ 相机内参
// 实车标定 2026-10-15, 重投影误差 0.097 px (DFvision 012-400-30 棋盘格)
// 分辨率 1440x1080。更换相机/镜头/对焦后必须重新标定 (./build/calibrate)
constexpr float CAM_FX = 1807.14f;
constexpr float CAM_FY = 1806.85f;
constexpr float CAM_CX = 739.10f;
constexpr float CAM_CY = 536.96f;

// 畸变系数 (k1 k2 p1 p2 k3)
constexpr float CAM_K1 = -0.09274768545033601f;
constexpr float CAM_K2 = 0.4989976612237753f;
constexpr float CAM_P1 = -0.0002463839297907039f;
constexpr float CAM_P2 = 0.0001154872294196196f;
constexpr float CAM_K3 = -1.940683659670297f;

// ============================================================ 图像方向
// 0=原样 1=水平翻转 2=垂直翻转 3=旋转180
// 实车值 3 是 2026-10-15 在实机上试出来的(相机倒装)。
// 测试里用 -DIMG_DIR_OVERRIDE=0 覆盖: 合成图本来就是正的, 再转 180 会让
// 坐标全部镜像, 位置断言和角度符号都会反过来。
#ifndef IMG_DIR_OVERRIDE
#define IMG_DIR_OVERRIDE 3
#endif
constexpr int IMG_DIR = IMG_DIR_OVERRIDE;

// ============================================================ 灯条提取
// 灯条 = 装甲板上被点亮的 LED 灯带; HSV 色调分割后按细长条筛选
constexpr float LB_SHORT_MIN = 3.0f;      // 短边下限 px: 滤微噪点
constexpr float LB_LONG_MIN  = 8.0f;      // 长边下限 px: 滤微噪点
constexpr float LB_RATIO_MIN = 3.0f;      // 长宽比下限
constexpr float LB_RATIO_MAX = 80.0f;     // 长宽比上限(正面细长灯带比例可 >40)
constexpr float LB_PURITY    = 8.0f;      // BGR 纯度差: 目标色通道均值须高出另两通道这么多
constexpr int   LB_MAX_CANDIDATES = 60;   // 候选截断, 防配对 O(n^2) 卡顿
constexpr double LB_MIN_CONTOUR_AREA = 12.0;

// ============================================================ 装甲板配对
// 全部用"相对灯条长度"的比例, 与目标远近无关
constexpr float PAIR_LEN_RATIO_MIN  = 0.7f;   // 两灯条长度比下限 (真板等高)
constexpr float PAIR_DY_RATIO_MAX   = 0.5f;   // y 中心偏差 / 灯条长
constexpr float PAIR_DIST_RATIO_MIN = 0.8f;   // x 间距 / 灯条长
constexpr float PAIR_DIST_RATIO_MAX = 4.0f;   // 正面标准板约 2.5~3.5, 放宽兼容斜视
constexpr float PAIR_ANGLE_DIFF     = 15.0f;  // 两灯条平行度(度)

// 装甲板中心暗区验证: 真板两灯之间是黑底, 环境假配对中心往往也亮
constexpr float ARMOR_CENTER_V_MAX = 160.0f;  // 中心区域 HSV V 均值上限

// 装甲板物理尺寸 (校内赛靶标 140 x 125 mm)
constexpr float ARMOR_W = 140.0f;
constexpr float ARMOR_H = 125.0f;

// ============================================================ 解算有效性
constexpr float SOLVE_DIST_MIN  = 0.4f;   // m, 更近判为误检
constexpr float SOLVE_DIST_MAX  = 8.0f;   // m, 更远判为误检 (发射位 5 m)
constexpr float SOLVE_ANGLE_MAX = 60.0f;  // 度, 偏差角过大不可能是正对目标

// ============================================================ 目标选择 / 跟踪
constexpr int   MAX_LOST_FRAMES       = 30;   // 丢失保持帧数
constexpr int   TRACK_CONFIRM_FRAMES  = 3;    // 连续确认帧数, 防单帧误检驱动云台
constexpr float TRACK_ASSOC_PX        = 80.0f;// 帧间关联门限 px
constexpr int   TRACK_REACQUIRE_GAP   = 5;    // 中间连续多少帧无检测才允许重新确认

// ============================================================ 云台控制
// 控制律: 期望角 = 实测角 + SERVO_DIR * 相机系偏差角 * SERVO_GAIN
//
// SERVO_DIR 是唯一必须实机确认的符号, 含义是"增大 yaw 命令时, 目标在画面里
// 朝哪边移动"。+1 = 目标在画面右侧时需要增大 yaw。
// 原代码里的 GIMBAL_YAW_DIR=-1 是上一轮实机试出来的, 但那一版用的是"自己积分
// 估计角"的发散写法, 符号本身没有可信度; 首次上实车请用 --dir-check 实测,
// 或者直接看 NUC 上运行日志里 auto_dir_check 有没有把方向翻过来。
constexpr float SERVO_DIR       = 1.0f;
constexpr float SERVO_GAIN      = 1.0f;   // 偏差角 -> 需要转过的角度 的比例
constexpr float SERVO_MAX_STEP  = 2.0f;   // 每帧最大转动(度), 防大偏差猛甩
constexpr float GIMBAL_SMOOTH   = 0.35f;  // 偏差角低通系数
constexpr float GIMBAL_DEADBAND = 0.5f;   // 死区(度), 消极限环摆动
constexpr float GIMBAL_MANUAL_STEP = 1.5f;// 手动控制步长(度/次)

// 云台角量程 —— 实机实测(2026-10-16)回传值是**负的**(yaw=-90.6, pitch=-38.3),
// 所以旧注释里"协议 0~180, 中间 90 = 镜头朝前"是错的, 不能照抄。
// 现在按"接近整圈、不做人为限位"处理: yaw 允许整圈, pitch 放宽到 ±180,
// 让下位机自己的机械限位去做保护(规则第 3.3 节: 机械限位由下位机保证)。
// 一旦实测出真实量程, 改成精确区间能更早发现异常指令。
constexpr float GIMBAL_YAW_MIN   = 0.0f;
constexpr float GIMBAL_YAW_MAX   = 360.0f;   // >= 359 表示按整圈处理(wrap)
constexpr float GIMBAL_PITCH_MIN = -180.0f;
constexpr float GIMBAL_PITCH_MAX =  180.0f;

// 无目标时云台摆扫寻板
constexpr float GIMBAL_SCAN_AMP   = 90.0f;  // 扫描幅度(度)
constexpr float GIMBAL_SCAN_SPEED = 1.5f;   // 扫描速度(度/帧)
constexpr int   GIMBAL_SCAN_HOLD  = 45;     // 丢失后静默帧数(约1.5s)再开始摆扫

// 回传数据可用性: 超过这个时间没用上新角就冻结命令
constexpr double FEEDBACK_MAX_AGE_S = 0.25;

// ============================================================ 弹道
constexpr float BALLISTIC_G = 9.8f;             // 重力加速度 m/s^2
constexpr float BULLET_SPEED_DEFAULT = 22.0f;   // 裁判系统未注入时的默认弹速 m/s
constexpr bool  BALLISTIC_COMP_ENABLE = false;  // 中期先关, 终验打命中率时开

// ============================================================ 通信
constexpr int SERIAL_SEND_HZ = 100;        // 协议推荐 100 Hz 定频单帧发送
constexpr int SERIAL_WATCHDOG_MS = 500;    // 控制量超时自动归零
constexpr int SERIAL_SHUTDOWN_ZERO_FRAMES = 20;
// 优先用 by-id 路径, 避免 ttyACM 编号变化; 找不到再退回编号
constexpr const char* SERIAL_BY_ID =
    "/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_3855315E3439-if00";
constexpr const char* SERIAL_FALLBACK0 = "/dev/ttyACM0";
constexpr const char* SERIAL_FALLBACK1 = "/dev/ttyACM1";

// ============================================================ 开火
// 中期不发射: fire_flag 恒 0。终验开火须同时满足下位机火控与人工许可
constexpr bool ENABLE_FIRE = false;

// ============================================================ 相机 Bayer 相位
// 传感器输出的是 Bayer 原始格式, 去马赛克的相位判错会让颜色彻底反过来,
// 蓝/红灯条的色调分割就失效了。
//
// 实机结论(2026-10-16): **自动判定不可靠**, 三种判据在同一台相机同一场景下
// 给出互相矛盾的答案(详见 hik_camera.hpp 的注释)。所以这里固定一个默认值,
// 现场用 `--bayer rg|bg|gr|gb` 覆盖, 或在窗口里按 T 键逐个试,
// 以"实车看到的颜色正确"为准。
//
// 判对错的快捷办法: 对着靶标跑 `detect_probe -v`, 看灯条的 BGR 均值是不是
// 目标色通道明显占优(蓝靶标应满足 B >> G 且 B >> R)。
// 当前默认 "bg": 沿用 10-15 实机验证过的值, 但仍需在实车前用蓝靶标复核。
#define BAYER_DEFAULT_NAME "bg"

// ============================================================ 标定
// 棋盘格: DFvision 012-400-30, 格子边长 30 mm (型号尾数 = 30)
constexpr float CALIB_SQUARE_MM = 30.0f;

// ============================================================ 模型
constexpr const char* YOLO_MODEL_XML = "assets/yolo11.xml";
constexpr float YOLO_CONF_THRESH = 0.4f;
constexpr float YOLO_NMS_THRESH  = 0.3f;

}  // namespace srm

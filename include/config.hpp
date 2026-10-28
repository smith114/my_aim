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
constexpr int LB_HUE_LO = 100;   // 蓝色调范围 [100,130]
constexpr int LB_HUE_HI = 130;

// 饱和度/亮度下限。
// 实机实测(10-16, 用真实靶标帧做参数网格搜索):
//   60/60  太低 -> 光晕和背景反光一起进来, 掩码脏, 连通域被撑胖;
//   140/140 最佳 -> 灯条干净, 长宽比 3.2~3.9, 配对几何一次通过;
//   再高会开始丢远距离暗灯条。
// 注意: **不要**把"过曝白核心"并进掩码。试过, 核心会把相邻 LED 芯片粘连,
// 连通域变宽、长宽比掉到 2.3~2.8, 配对反而失败(灯条是"多颗芯片 + 暗缝"结构)。
constexpr int LB_SAT_MIN = 140;
constexpr int LB_VAL_MIN = 140;

constexpr float LB_SHORT_MIN = 3.0f;      // 短边下限 px: 滤微噪点
constexpr float LB_LONG_MIN  = 8.0f;      // 长边下限 px (同济 min_lightbar_length=8)
// 长宽比上下限 —— **这条是"远处识别不到装甲板"的直接原因, 已对齐同济**。
//
// 同济 configs/*.yaml: min_lightbar_ratio=1.5, max_lightbar_ratio=20
// 我们原来用 3.0 ~ 80, 下限严了一倍。
//
// 为什么下限这么关键: 远处灯条在画面里只有几像素宽, 但 LED 的光晕会把宽度
// 撑开(实测实验室物体的灯条: 35px 高 / 22px 宽 -> 比值只有 1.7)。
// 要求 >=3.0 就等于**把所有远处目标一次性滤掉**, 只剩近处的大灯条。
// 放宽到 1.5 之后远处目标才进得来, 代价是要靠后面的配对约束去滤误检。
// 上限收到 20(原来 80)反而更严: 极细长的东西基本都是环境反光拉出的条纹。
constexpr float LB_RATIO_MIN = 1.5f;
constexpr float LB_RATIO_MAX = 20.0f;
constexpr float LB_PURITY    = 8.0f;      // BGR 纯度差: 目标色通道均值须高出另两通道这么多
constexpr int   LB_MAX_CANDIDATES = 60;   // 候选截断, 防配对 O(n^2) 卡顿
constexpr double LB_MIN_CONTOUR_AREA = 12.0;

// 形态学核大小。**CLOSE 核必须小**:
// 一根灯带由多颗 LED 芯片串成, 芯片间有暗缝。5x5 CLOSE 会把相邻芯片并成一个
// 宽块, 长宽比从 ~3.9 掉到 2.3~2.8, 正好撞在 LB_RATIO_MIN=3 上 ——
// 灯条肉眼可见却检不出来(实测就是被这一条卡住)。3x3 足够补掉 JPEG 噪点
// 造成的小断裂, 又不会糊掉几何。
// OPEN 设 1 表示不执行: 分割已经干净, 再做开运算只会啃掉小灯条。
constexpr int LB_MORPH_CLOSE_SIZE = 3;
constexpr int LB_MORPH_OPEN_SIZE  = 1;

// ============================================================ 装甲板配对
// 全部用"相对灯条长度"的比例, 与目标远近无关
constexpr float PAIR_LEN_RATIO_MIN  = 0.7f;   // 两灯条长度比下限 (真板等高)
constexpr float PAIR_DY_RATIO_MAX   = 0.5f;   // y 中心偏差 / 灯条长
constexpr float PAIR_DIST_RATIO_MIN = 0.8f;   // x 间距 / 灯条长
constexpr float PAIR_DIST_RATIO_MAX = 4.0f;   // 正面标准板约 2.5~3.5, 放宽兼容斜视
constexpr float PAIR_ANGLE_DIFF     = 15.0f;  // 两灯条平行度(度)

// 装甲板**宽高比**上限 —— 拦跨板误配的关键参数。
// 它强烈依赖"靶标的灯条是不是占满板高", 而两种靶标差别很大, 所以做成可切换预设。
//
//   ---- 实测对照(10-16, 同一套代码) ----
//
//   A) 标准 RM 装甲板 (140x125mm, 满高灯带) —— 合成参考图实测:
//        灯条 len=82px, 灯条间距 101px  ->  宽高比 = 101/82 = 1.23
//        黄框紧贴板子
//      => 上限取 1.6 就能接受真板, 同时把 3~7 的跨板误配全部拦掉。**强约束**。
//
//   B) 实验室那个练习物体 (短 LED 段):
//        灯条 len=35px, 灯条间距 122px  ->  宽高比 = 4.02
//        真板 aspect=4.02, 跨板 aspect=6.91
//      => 上限必须放宽到 5.0 才收得下, 但也因此更容易放过误配。**弱约束**。
//
// A 和 B 不可能同时用同一个阈值。默认按当前场景取 B(实验室); 上比赛靶标前
// **务必改成 A** —— 那才是规则书写的 140x125mm 靶标。
#define ASPECT_PRESET_LAB  5.0f   // 实验室练习物体(短灯条)
#define ASPECT_PRESET_STD  1.6f   // 标准 140x125 满高灯带靶标(比赛用)

#ifndef PAIR_ASPECT_MAX
#define PAIR_ASPECT_MAX 5.0f
#endif
#ifndef PAIR_ASPECT_MIN
#define PAIR_ASPECT_MIN 0.6f
#endif
//   (标准 RM 装甲板就是那样), 那么宽高比理论值 ~1.1, 上限应改回 ~1.6 ——
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
// 连续确认帧数: 这里要权衡。太大 -> 目标已经移开了还没锁定; 太小 -> 单帧误检就驱动云台。
// 实测用 3 时, 检测逐帧抖动会让"连续 3 帧同一位置"很难满足, 结果一直不锁定,
// 主程序误以为没目标而去摆扫, 反而把目标转出画面。取 2 明显更容易咬住。
constexpr int   TRACK_CONFIRM_FRAMES  = 2;
constexpr float TRACK_ASSOC_PX        = 120.0f;// 帧间关联门限 px
constexpr int   TRACK_REACQUIRE_GAP   = 5;    // 中间连续多少帧无检测才允许重新确认

// ============================================================ 云台控制
// 控制律: 期望角 = 实测角 + SERVO_DIR_轴 * 相机系偏差角 * SERVO_GAIN * 软启动
//
// **yaw 和 pitch 的方向符号必须分开!**
//
// 实测推导(10-28 现场, 靶标被手动对准后):
//   检出板框 rect=[54x35 @(750,583)] -> 中心 (777.0, 600.5)
//   相机主点 (739.1, 537.0)
//   所以目标在光轴**右侧 38px (+1.20 度)**、**下方 63px (+2.01 度)**
//
//   另据实测: 云台指向天花板时 pitch=-32.89(接近机械下限 -35.8)
//   -> **pitch 减小 = 抬头, pitch 增大 = 低头**
//
//   pitch 轴: 目标偏下, 需要低头, 即 pitch 要增大 -> SERVO_DIR_PITCH = +1
//   yaw   轴: 现场两次实测, 命令增大 yaw 时云台向**左**转;
//             目标偏右需右转, 所以要求 yaw 减小 -> SERVO_DIR_YAW = -1
//
// 之前用一个 SERVO_DIR 同时管两轴, 结果无论取 +1 还是 -1 都只能修对一半:
//   =+1 时"往左偏"(yaw 错), =-1 时"往右上偏"(pitch 错)。
// 这就是那个看起来怎么调都不对的现象的根源。
//
// 两个符号都依赖相机安装方向(IMG_DIR)与云台轴向的实际关系。
// **若换了相机安装角度或改了 IMG_DIR, 这两个符号必须重新实测。**
constexpr float SERVO_DIR_YAW   = -1.0f;   // **实测确认**(servo_calib): 命令+6度使偏角 0.70->7.31
// pitch 符号由**已确认的物理事实**推出(不是猜):
//   实测: 云台指向天花板时 pitch=-32.89, 接近机械下限 -35.8
//     -> pitch 越小 = 抬头, pitch 越大 = 低头
//   theta_pitch = atan2(tvec.y, tvec.z), 而 OpenCV 相机系 +y 指向图像**下方**
//     -> theta_pitch > 0 表示目标在光轴**下方**
//   目标在下方, 光轴要低头才追得上 -> pitch 必须增大
//     -> SERVO_DIR_PITCH = +1
constexpr float SERVO_DIR_PITCH =  1.0f;

constexpr float SERVO_GAIN      = 1.0f;   // 偏差角 -> 需要转过的角度 的比例
constexpr float SERVO_MAX_STEP  = 2.0f;   // 每帧最大转动(度), 防大偏差猛甩
// 软启动帧数: 进跟踪后增益从 0 线性升到 SERVO_GAIN。
// 目的: 切换视觉模式的那一刻, 如果目标偏角较大, 不要一上来就满增益猛转。
// 20 帧 @ 约 16fps 约 1.2 秒, 足够平缓又不至于明显迟钝。
constexpr int   SERVO_SOFT_START_FRAMES = 20;
constexpr float GIMBAL_SMOOTH   = 0.35f;  // 偏差角低通系数
constexpr float GIMBAL_DEADBAND = 0.5f;   // 死区(度), 消极限环摆动
constexpr float GIMBAL_MANUAL_STEP = 1.5f;// 手动控制步长(度/次)

// 安全包络: 命令角离"启动姿态"最远允许多少度。
//
// 为什么要这一层: 云台 yaw 能整圈转, 但 pitch 有机械限位, 而**零位并不是机壳
// 行程的中点** —— 实测进视觉模式时 pitch=-38.3 度, 可机械行程只有约
// [-35.8, +26.1] (见下), 两者对不上, 所以"绝对区间"不能照抄测量值。
// 相对启动姿态限制则与零位无关, 是安全且可移植的做法。
//
// 一旦 pitch 顶到限位: 反馈会卡住不再跟随命令, 此时如果还按"绝对区间"去比较,
// 会算出一个错误的高度差, 云台就卡死在错的高度回不来。相对包络没有这个问题。
//
// 这个值同时决定摆扫寻板的搜索范围, 需要 >= GIMBAL_SCAN_AMP/2。
constexpr float GIMBAL_ENVELOPE_DEG = 30.0f;

// ============================================================ 云台量程
// 实机确认(2026-10-16):
//   yaw   : 左右 360 度全向, 无机械限位 -> 按 wrap 处理
//   pitch : **上下有限**, 实测机械行程约 [-35.8, +26.1] 度 (以视觉模式零位为参考)
//           命令继续增大时反馈卡住不动 = 顶到限位
//
// **pitch 的上下限就是实测机械行程, 不是数值保护。**
// 之前这里写的是 [-180, 180] 并注明"数值保护", 结果跨度正好 360, 让
// wrap_or_clamp_pitch 走进了 wrap 分支: -30 度被归一成 330 度发下去,
// 云台顶死机械限位。现在改成真实行程并强制 clamp。
//
// 为什么把实测行程写死是安全的: 相对包络(±GIMBAL_ENVELOPE_DEG)本身也有
// 同样的问题 —— home 靠近上限时对称包络会越过行程(实测 home=+25 时包络上限
// 是 +55, 超出机械上限 28.9 度), 云台照样顶死。用绝对行程硬限是唯一能真正
// 挡住这种情况的手段。两道限制叠加: 相对包络管"别离瞄准点太远",
// 绝对行程管"别顶到限位"。
//
// 风险与前提: 这两个数依赖下位机 pitch 零点不变。若以后零点被重新标定
// (比如换了固件/重新上电归零方式变了), **必须重新实测行程并更新这两个值**,
// 否则云台会在错误的高度上被限制住。测法: gimbal_test --sweep ±pitch --stop N
constexpr float GIMBAL_YAW_MIN   = 0.0f;
constexpr float GIMBAL_YAW_MAX   = 360.0f;   // >= 359 视为整圈(wrap)
constexpr float GIMBAL_PITCH_MIN = -35.8f;   // 实测机械下限
constexpr float GIMBAL_PITCH_MAX =  26.1f;   // 实测机械上限

// ============================================================ 无目标时的行为
// **默认不摆扫。** 这一条是实测踩坑之后定的。
//
// 原因: 比赛和调试的流程都是"操作手先把云台手动对准靶标, 再进视觉模式"。
// 也就是说**启动姿态本身就是靶标方向**。此时程序如果再左右摆扫, 等于主动把
// 靶标转出画面: 检出掉到 0 -> 程序更确信"没目标" -> 继续扫 -> 再也回不来。
// 实测(10-16) 就是这个现象: cmd 在 home ±22.5 度之间来回摆(GIMBAL_SCAN_AMP/2
// 正好 22.5), 而靶标本来就在正前方, 结果全程 armor=0、从未稳定锁定。
//
// 所以缺省行为改成"**原地保持, 等目标出现**"。需要自动搜索时用 --scan 打开。
// 如果哪天要真正无人值守地找靶, 摆扫才有意义, 但那时也应该先有一段
// "先扫一圈记录候选方位"的逻辑, 而不是盲目左右晃。
constexpr bool ENABLE_AUTO_SCAN = false;

// 无目标时云台摆扫寻板 (仅当 ENABLE_AUTO_SCAN 或 --scan 时生效)
constexpr float GIMBAL_SCAN_AMP   = 45.0f;  // 扫描幅度(度), 受 pitch 行程限制别开太大
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

// ============================================================ 相机曝光
// **短曝光 + 较高增益**, 对齐同济 configs/demo.yaml 的 exposure_ms=2 / gain=16。
//
// 这里纠正一个我之前判断反了的地方: 一直以为"画面暗所以要加曝光",
// 用 20 ms 长曝光, 结果是 LED **过曝成白团**、形状糊掉、长宽比暴跌,
// 远处的小灯条直接检不出来。正确的是**缩短曝光**让 LED 不过曝, 保持细长形状。
// 场景特点: 背景很暗 + 少数极亮 LED, 高动态范围 -> 必须压曝光保住 LED 形状。
//
// 实机验证方法: cam_view 的 HUD 里看 bars 数量, 以及灯条长宽比是否升高。
constexpr bool  CAM_USE_AUTO_EXPOSURE = false;  // true = 连续自动(高动态场景会振荡)
constexpr float CAM_EXPOSURE_US = 2000.0f;      // 2 ms (同济用 2 ms)
constexpr float CAM_GAIN_DB     = 16.0f;        // 16 dB (同济用 16)
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
// 三套模型与它们**各自的布局**。换模型必须同时用对布局, 否则静默出错
// (列偏移会读到角点坐标, 分数变成几百, 全图都成目标, 而且不报错)。
//
// 实测(10-28)布局:
//   yolo11  输入 640  输出 [1,50,8400]   4框 + 38类 + 8角点
//   yolov8  输入 416  输出 [1,14,3549]   4框 +  2类 + 8角点
//   yolov5  输入 640  输出 [1,25200,22]  4框 + 10类 + 8角点
//
// 样例图置信度(用户提供的真实装甲板照片):
//            红板"3"   蓝板"5"   实验室图
//   yolo11    0.903    0.320     0.043
//   yolov8    0.954    0.172     0.051
//
// v8 红板更强, yolo11 蓝板更强。比赛打蓝方时这个差异有实际意义,
// 所以**默认 yolo11, 但 v8 一键可切**(命令行 --model v8)。
// 最终用哪个等实车近距离实测再定, 不靠猜。
#define YOLO_PROFILE_YOLO11_38CLASS 0
#define YOLO_PROFILE_YOLOV8_2CLASS  1
#define YOLO_PROFILE_YOLOV5_10CLASS 2

#ifndef YOLO_PROFILE_SELECT
#define YOLO_PROFILE_SELECT YOLO_PROFILE_YOLOV8_2CLASS
#endif

// 置信度阈值 0.30 的来历:
//   红板(亮) 0.90 以上 -> 0.4 也够
//   蓝板(暗、仅 LED 自发光) 只有 0.32 -> 0.4 会漏
// 比赛场地正是"暗背景 + LED 自发光", 所以按 0.3 定阈。
// 误检由 NMS 和下游 PnP 有效性(0.4~8m / 偏角<60°)兜底。
constexpr float YOLO_CONF_THRESH = 0.30f;
constexpr float YOLO_NMS_THRESH  = 0.30f;

}  // namespace srm

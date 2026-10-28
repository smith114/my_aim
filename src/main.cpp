// main.cpp: 自瞄上位机主程序
//
// 闭环链路:
//   海康相机取流 -> 图像方向校正 -> 灯条提取 -> 装甲板配对 -> PnP 解算
//     -> 目标跟踪(三态状态机) -> 云台增量伺服 -> 100 Hz 定频发给下位机
//   串口接收线程同时解析下位机回传的云台姿态与弹速。
//
// 用法:
//   ./my_aim                     实车: 海康相机
//   ./my_aim --debug             额外画出灯条/配对结果
//   ./my_aim --headless          不开窗口(远程 SSH 调试用), 存快照
//   ./my_aim --mock              强制 mock 串口(不打下位机, 只跑逻辑)
//   ./my_aim <video|image>       用视频/图片替代相机
//   ./my_aim --snapshot-dir DIR  快照目录(默认 /tmp/myaim_shots)
#define _POSIX_C_SOURCE 200809L

#include <cctype>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>

#include "armor.hpp"
#include "config.hpp"
#include "gimbal.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "solver.hpp"
#include "synth.hpp"
#include "tracker.hpp"

// YOLO 检测分支只在装了 OpenVINO 时编入 (CMake 里 HAVE_OPENVINO)。
// 没装 OpenVINO 的机器照样能编译, 只是少掉 --yolo 这条路。
#ifdef HAVE_OPENVINO
#include <memory>

#include "yolo_armor.hpp"
#include "yolo_detector.hpp"
#endif

using namespace srm;

// ------------------------------------------------------------------ 全局
static volatile std::sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

static FILE* g_log = nullptr;
static void LOG(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    std::fflush(stdout);
    if (g_log) {
        va_start(ap, fmt);
        vfprintf(g_log, fmt, ap);
        va_end(ap);
        std::fflush(g_log);
    }
}

static bool dir_exists(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

static void apply_img_dir(cv::Mat& f) {
    switch (IMG_DIR) {
        case 1: cv::flip(f, f, 1); break;
        case 2: cv::flip(f, f, 0); break;
        case 3: cv::rotate(f, f, cv::ROTATE_180); break;
        default: break;
    }
}

static bool is_image_path(const std::string& p) {
    static const char* ext[] = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"};
    const size_t dot = p.find_last_of('.');
    if (dot == std::string::npos) return false;
    std::string e = p.substr(dot);
    for (auto& c : e) c = static_cast<char>(::tolower(c));
    for (const char* x : ext) {
        if (e == x) return true;
    }
    return false;
}

// ------------------------------------------------------------------ 主程序
// mode 字段的识别。
//
// 协议文档说 mode 是"参考固件固定发 0 的保留字段", 但实机实测它不是:
// **它实际是下位机的视觉控制开关** —— 由车上鼠标右键切换, 由下位机实现。
//   mode = 0 : 视觉控制关闭。下位机**完全忽略**上位机发的 yaw/pitch,
//              云台不动(而且电机通常没使能, 会缓慢垂下来)。
//   mode = 1 : 视觉控制开启, 我们的角度指令才生效。
//
// 这个区别极其重要, 因为两种情况在日志里长得一模一样(帧都在发、回传都正常),
// 唯一差别就是 mode。所以这里显式识别并打日志, 并且在 mode!=1 时
// **不更新伺服内部状态** —— 否则命令角会在一堆被忽略的指令上越积越偏。
static const char* mode_name(int m) {
    switch (m) {
        case 0:  return "OFF  视觉控制未开启 -> 下位机忽略角度指令, 云台不会跟随";
        case 1:  return "ON   视觉控制已开启 -> 角度指令生效";
        default: return "?    未知取值(协议未定义)";
    }
}

int main(int argc, char** argv) {
    std::string input_path;
    std::string snapshot_dir = "/tmp/myaim_shots";
    std::string log_path = "/tmp/myaim.log";
    bool show_debug  = false;
    bool headless    = false;
    bool force_mock  = false;
    bool save_video  = false;
    // 无目标时是否自动摆扫。默认关闭, 见 config.hpp 的 ENABLE_AUTO_SCAN 说明。
bool auto_scan = ENABLE_AUTO_SCAN;

// ---- YOLO 检测 ----
// 默认关闭: 模型目前在我们这种过曝/暗场景下会静默返回空(max score 0.04),
// 用 --yolo 显式开启。开启后默认带灯条法兜底(hybrid)。
// **YOLO 是唯一检测器** —— 灯条法已从主流程移除(sort 见下方检测段注释)。
bool use_yolo       = true;
bool hold_only      = false;   // --hold: 只保持姿态, 不推进伺服
// 运行时覆盖伺服方向, 便于现场一次试完 4 种组合而不用重编译。
// 0 表示"不覆盖, 用 config.hpp 的值"。
float cli_dir_yaw   = 0.0f;
float cli_dir_pitch = 0.0f;
int  want_color     = -1;      // 颜色过滤: -1=不过滤 0=只蓝 1=只红
int  yolo_profile_sel = YOLO_PROFILE_SELECT;   // 见 config.hpp, 当前默认 v11
float yolo_conf     = YOLO_CONF_THRESH;
int   stat_yolo_boxes = 0;
int   stat_detsrc[3] = {0, 0, 0};   // 0=灯条法 1=YOLO 2=YOLO空->回退
#ifdef HAVE_OPENVINO
std::unique_ptr<YoloDetector> yolo;
#endif

// ---- 模拟靶标模式 (--synth) ----
// 用合成装甲板代替相机, 喂给完整链路, 用来在没有硬件时验证"锁定"是否发生。
bool  synth = false;
float synth_bearing     = 20.0f;     // 靶标在世界系里的方位角(度), 相对初始光轴
float synth_dist_mm     = 2500.0f;   // 靶标距离(mm)
float synth_noise       = 0.0f;      // 图像高斯噪声标准差(0=干净)
int   synth_distractors = 0;         // 随机亮斑干扰数量(考验误检抑制)
    int  max_frames  = 0;
    int  log_every   = 30;   // 日志间隔帧数, --log-every 可改
    std::string bayer_name_arg;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--debug") {
            show_debug = true;
        } else if (a == "--headless") {
            headless = true;
        } else if (a == "--mock") {
            force_mock = true;
        } else if (a == "--save-video") {
            save_video = true;
        } else if (a == "--scan") {
            auto_scan = true;
        } else if (a == "--yolo") {
            use_yolo = true;   // 默认已开, 保留兼容
        } else if (a == "--dir-yaw" && i + 1 < argc) {
            cli_dir_yaw = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--dir-pitch" && i + 1 < argc) {
            cli_dir_pitch = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--hold") {
            // 只保持当前姿态: 不做任何伺服推进, 检测照跑(便于观察),
            // 云台一动不动。用于"确认切换瞬间不会动"和现场比对。
            hold_only = true;
        } else if (a == "--no-fallback") {
            // 兼容旧参数: 现在本来就没有灯条法兜底了, 保留只为不报错
        } else if (a == "--model" && i + 1 < argc) {
            const std::string m = argv[++i];
            // 支持简写, 避免每次写全路径
            if (m == "v8" || m == "yolov8")       yolo_profile_sel = YOLO_PROFILE_YOLOV8_2CLASS;
            else if (m == "v11" || m == "yolo11") yolo_profile_sel = YOLO_PROFILE_YOLO11_38CLASS;
            else if (m == "v5" || m == "yolov5")   yolo_profile_sel = YOLO_PROFILE_YOLOV5_10CLASS;
            else                                  yolo_profile_sel = -1;  // 自定义路径
        } else if (a == "--conf" && i + 1 < argc) {
            yolo_conf = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--enemy" && i + 1 < argc) {
            // 只想跟某一方时用它, 能直接滤掉对面装甲板
            const std::string c = argv[++i];
            want_color = (c == "red" || c == "r" || c == "1") ? 1 : 0;
        } else if (a == "--synth") {
            synth = true;
            force_mock = true;   // 模拟靶标必须配 mock 串口, 不能去打下位机
        } else if (a == "--bearing" && i + 1 < argc) {
            synth_bearing = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--dist" && i + 1 < argc) {
            synth_dist_mm = static_cast<float>(std::atof(argv[++i])) * 1000.0f;
        } else if (a == "--noise" && i + 1 < argc) {
            synth_noise = static_cast<float>(std::atof(argv[++i]));
        } else if (a == "--distractors" && i + 1 < argc) {
            synth_distractors = std::atoi(argv[++i]);
        } else if (a == "--bayer" && i + 1 < argc) {
            bayer_name_arg = argv[++i];
        } else if (a == "--snapshot-dir" && i + 1 < argc) {
            snapshot_dir = argv[++i];
        } else if (a == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (a == "--frames" && i + 1 < argc) {
            max_frames = std::atoi(argv[++i]);
        } else if (a == "--log-every" && i + 1 < argc) {
            log_every = std::max(1, std::atoi(argv[++i]));
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法: my_aim [--debug] [--headless] [--mock] [--save-video]\n"
                         "              [--yolo [--model XML] [--conf C] [--enemy blue|red]]\n"
                         "              [--scan] [--bayer rg|bg|gr|gb]\n"
                         "              [--snapshot-dir DIR] [--log FILE]\n"
                         "              [--frames N] [video|image]\n"
                         "  --scan          无目标时自动摆扫找板(默认关闭: 操作手已手动对准,\n"
                         "                  摆扫反而会把靶标转出画面)\n"
                         "  --synth         模拟靶标: 合成装甲板喂完整链路, 验证锁定(不碰硬件)\n"
                         "  --bearing DEG   模拟靶标方位角, 度(默认 20)\n"
                         "  --dist M        模拟靶标距离, 米(默认 2.5)\n"
                         "  --noise N       合成图高斯噪声标准差(默认 0)\n"
                         "  --distractors N 合成图随机亮斑干扰数(默认 0)\n"
                         "  --yolo          YOLO 神经网络检测为主(需 OpenVINO), 灯条法兜底\n"
                         "  --model XML     指定模型(默认见 config.hpp)\n"
                         "  --conf C        置信度阈值(默认 0.30)\n"
                         "  --enemy blue|red  只跟某一方装甲板\n"
                         "  --no-fallback   纯 YOLO, 不回退灯条法\n"
                         "  --bayer         指定相机 Bayer 相位(默认见 config.hpp)\n";
            return 0;
        } else {
            input_path = a;
        }
    }

#ifdef HAVE_OPENVINO
    if (use_yolo) {
        try {
            YoloProfile prof =
                (yolo_profile_sel < 0) ? make_yolo_profile(YOLO_PROFILE_SELECT)
                                       : make_yolo_profile(yolo_profile_sel);
            yolo = std::make_unique<YoloDetector>(prof, yolo_conf, YOLO_NMS_THRESH);
            LOG("[YOLO] 模型 %s  输入 %d  类别 %d  置信度 %.2f\n",
                prof.model_xml.c_str(), prof.input_size, prof.class_num,
                static_cast<double>(yolo_conf));
        } catch (const std::exception& e) {
            // **不再回退灯条法**: 按需求 "全面换成 YOLO", 检测器只有 YOLO 一条路。
            // 静默回退会让人以为在跑神经网络, 实际在跑灯条法 —— 这种"看不出来的
            // 降级"是排查时最难发现的一类问题。
            LOG("[YOLO][ERR] 模型加载失败: %s\n", e.what());
            LOG("        检测器只有 YOLO 一条路, 没有兜底。请检查模型路径与布局配置。\n");
            return 1;
        }
    }
#else
    // 检测器只有 YOLO。没有 OpenVINO 就无法工作, 明确失败而不是退回灯条法。
    LOG("[YOLO][ERR] 这个 build 没有编入 OpenVINO 支持, 无法运行检测。\n"
        "            请在装了 OpenVINO 的机器上重新 cmake 配置并编译。\n");
    (void)use_yolo;
    return 1;
    (void)want_color;
    (void)yolo_conf;
    (void)stat_yolo_boxes;
#endif


    std::signal(SIGTERM, on_signal);

    if ((g_log = std::fopen(log_path.c_str(), "a")) != nullptr) {
        const std::time_t t = std::time(nullptr);
        char ts[64];
        std::strftime(ts, sizeof(ts), "%F %T", std::localtime(&t));
        LOG("\n========== my_aim start %s ==========\n", ts);
    }

    // ---------------------------------------------------------- 输入源
    cv::VideoCapture file_cap;
    HikCamera hik;
    bool use_hik = false;
    const bool still_image = !input_path.empty() && is_image_path(input_path);

    if (synth) {
        LOG("[INPUT] 模拟靶标 (不打开相机/视频)\n");
        input_path.clear();
    } else if (!input_path.empty()) {
        if (still_image) {
            const cv::Mat img = cv::imread(input_path, cv::IMREAD_COLOR);
            if (img.empty()) {
                LOG("[ERR] 读不到图片: %s\n", input_path.c_str());
                return 1;
            }
            file_cap.open(input_path);
            LOG("[INPUT] image %s (%dx%d)\n", input_path.c_str(), img.cols, img.rows);
        } else {
            if (!file_cap.open(input_path)) {
                LOG("[ERR] 打不开视频: %s\n", input_path.c_str());
                return 1;
            }
            LOG("[INPUT] video %s\n", input_path.c_str());
        }
    } else {
        if (!bayer_name_arg.empty()) {
            hik.set_bayer(HikCamera::bayer_from_name(bayer_name_arg));
        }
        if (!hik.open()) {
            LOG("[ERR] 海康相机打开失败, 退出\n");
            return 1;
        }
        use_hik = true;
        LOG("[INPUT] Hik camera (bayer=%s)\n", hik.bayer_name());
    }

    // ---------------------------------------------------------- 串口
    const std::vector<std::string> ports{SERIAL_BY_ID, SERIAL_FALLBACK0, SERIAL_FALLBACK1};
    int serial_fd = -1;
    std::string serial_name;
    if (!force_mock) {
        serial_fd = open_first_available(ports, &serial_name);
    }
    SerialLink link;
    link.start(serial_fd, SERIAL_SEND_HZ, SERIAL_WATCHDOG_MS, SERIAL_SHUTDOWN_ZERO_FRAMES);
    if (force_mock || serial_fd < 0) {
        LOG("[SERIAL] MOCK 模式(未找到下位机, 只跑逻辑不发送)\n");
    } else {
        LOG("[SERIAL] %s (fd=%d, %d Hz)\n", serial_name.c_str(), serial_fd, SERIAL_SEND_HZ);
    }

    // ---------------------------------------------------------- 状态
    Tracker    tracker;
    GimbalCtrl gimbal;
    // 现场用 --dir-yaw / --dir-pitch 覆盖符号, 不用重编译
    if (cli_dir_yaw   != 0.0f) gimbal.servo_dir_yaw   = (cli_dir_yaw   > 0 ? 1.0f : -1.0f);
    if (cli_dir_pitch != 0.0f) gimbal.servo_dir_pitch = (cli_dir_pitch > 0 ? 1.0f : -1.0f);
    LOG("[SERVO] 方向符号 yaw=%+.0f pitch=%+.0f%s\n",
        gimbal.servo_dir_yaw, gimbal.servo_dir_pitch,
        (cli_dir_yaw != 0.0f || cli_dir_pitch != 0.0f) ? "  (命令行覆盖)" : "  (config.hpp)");

    // mock 模式没有下位机回传, 用"命令角即真实角"的假设把闭环跑起来,
    // 这样可以纯离线验证 检测->解算->跟踪->伺服 整条链路的逻辑。
    if (force_mock || serial_fd < 0) {
        gimbal.update_feedback(gimbal.cmd_yaw, gimbal.cmd_pitch);
        LOG("[MOCK] 用命令角当作实测角, 便于离线验证闭环\n");
    }

    if (const RxState st0 = link.state(); st0.valid) {
        gimbal.update_feedback(st0.yaw, st0.pitch);
        LOG("[SERIAL] 下位机首帧: yaw=%.2f pitch=%.2f roll=%.2f bullet=%.2f id=%d\n",
            st0.yaw, st0.pitch, st0.roll, st0.bullet_speed, st0.robot_id);
    } else if (!force_mock && serial_fd >= 0) {
        LOG("[WARN] 还没收到下位机回传, 云台将保持不动(避免拿错误基准乱指)\n");
    }

    // 只有在已经知道云台当前姿态时才发第一帧。
    // 绝不能用一个默认值(尤其 0,0)开局 —— ID1 是绝对角度, 那等于命令云台猛转到那个角。
    // SerialLink 在 set_target 之前一帧都不发, 这里配合它做门控。
    if (gimbal.have_meas || force_mock || serial_fd < 0) {
        link.set_target(gimbal.cmd_yaw, gimbal.cmd_pitch, 0);
        LOG("[GIMBAL] 起始命令角 = 实测角 %.2f/%.2f\n", gimbal.cmd_yaw, gimbal.cmd_pitch);
    } else {
        LOG("[GIMBAL] 等第一帧回传后再决定命令角(在此之前一帧都不发)\n");
    }

    if (synth) {
        LOG("[SYNTH] 模拟靶标: 方位 %.1f 度, 距离 %.2f m, 噪声 %.1f, 干扰 %d 个\n"
            "        图像由 synth.hpp 合成, 但检测/配对/PnP/跟踪/伺服全是真实代码\n",
            synth_bearing, synth_dist_mm / 1000.0f, synth_noise, synth_distractors);
    }
    LOG("[GIMBAL] 无目标时%s (摆扫%s)\n",
        auto_scan ? "自动摆扫找板" : "原地保持等目标",
        auto_scan ? "已开启" : "默认关闭: 操作手已手动对准, 摆扫会把靶标转出画面");

    // ---------------------------------------------------------- 快照目录
    if ((headless || still_image) && !dir_exists(snapshot_dir)) {
        ::mkdir(snapshot_dir.c_str(), 0755);
    }
    cv::VideoWriter vid_writer;
    if (save_video) {
        vid_writer.open(snapshot_dir + "/run.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                        20.0, cv::Size(1440, 1080));
    }

    cv::Mat frame;
    int    frame_cnt = 0;
    int    stat_bars = 0, stat_armor = 0, stat_solved = 0, stat_locked = 0;
    int    last_locked_frame = -100000;
    // 最近一次"画面里有候选装甲板"的帧号。摆扫的触发看它, 不看 last_locked_frame
    // (用 locked 判断会导致"没锁定就乱扫"的恶性循环, 见下方云台段说明)。
    int    last_candidate_frame = -100000;
    // mock 模式(用视频离线跑)下的被控对象状态: 模拟云台实际角, 一阶滞后跟随命令
    float  sim_yaw_   = 90.0f;
    float  sim_pitch_ = 90.0f;
    float  last_bullet_speed = BULLET_SPEED_DEFAULT;

    while (g_stop == 0) {
        // ------------------------------------------------ 取帧
        if (synth) {
            // 模拟靶标模式: 世界系里靶标固定在某个方位角, 相机光轴跟着模拟云台转。
            // 生成的图像喂给**完整的检测/配对/PnP/跟踪/伺服链路** —— 除了图像来源,
            // 后面每一个环节都是真的。用来在没有相机的情况下验证"锁定"是否发生。
            //
            // 几何: 目标相对光轴的夹角 theta = 目标方位 - 云台已转过的角度
            //   (云台顺转 delta, 光轴也顺转 delta, 固定目标相对光轴就减少 delta)
            const float turned = sim_yaw_ - 90.0f;
            const float theta  = synth_bearing - turned;
            srm::test::SynthPose pose;
            pose.x_mm = synth_dist_mm * std::sin(theta * static_cast<float>(CV_PI) / 180.0f);
            pose.z_mm = synth_dist_mm * std::cos(theta * static_cast<float>(CV_PI) / 180.0f);
            pose.color = 0;
            frame = srm::test::make_armor_image(pose, 1440, 1080, synth_noise).image;
            if (synth_distractors > 0) srm::test::add_distractors(frame, synth_distractors);
        } else if (use_hik) {
            if (!hik.read(frame)) continue;
        } else {
            file_cap >> frame;
            if (frame.empty()) break;
        }
        // IMG_DIR 是"相机倒装"的安装校正, **只对实相机帧生效**。
        //
        // 视频/图片文件本身就是正常朝向, 再转一次 180 度会让画面上下颠倒。
        // 灯条法对这种颠倒还算鲁棒(形状不变), 但 YOLO 完全失效 ——
        // 实测(10-24): 同一段同济 demo, 不转时 yolo_test 检出 score=0.88,
        // 转了之后 my_aim 里 YOLO 全程 0 检出, 只能靠灯条法兜底。
        // 这个 bug 之前一直存在, 只是因为没接 YOLO 才没暴露。
        if (use_hik) apply_img_dir(frame);
        tracker.set_frame_size(frame.size());
        ++frame_cnt;

        // ------------------------------------------------ 检测
        //
        // 检测器可替换: 两条路都产出 std::vector<Armor>, 下游 PnP/跟踪/伺服
        // 完全不知道上游是谁。所以切换检测器不影响任何已验证的环节。
        //
        //   --yolo      神经网络为主, 灯条法兜底(hybrid)
        //   默认        灯条法为主, 不开网络
        //
        // hybrid 的意义: YOLO 对"点亮的标准装甲板"召回高、且有 4 角点(不需要
        // 配对约束), 但对我们这种过曝/杂乱场景会静默返回空; 灯条法在空场景稳,
        // 在杂场景容易跨板误配。两者互补, 谁有结果用谁。
        std::vector<LightBar> bars;
        std::vector<Armor>    armors;
        int det_source = 0;   // 0=灯条法 1=YOLO 2=YOLO空->回退灯条法

#ifdef HAVE_OPENVINO
        if (use_yolo && yolo) {
            const auto boxes = yolo->detect(frame);
            stat_yolo_boxes += static_cast<int>(boxes.size());
            armors = filter_by_color(yolo_to_armors(boxes), want_color);
            if (!armors.empty()) {
                det_source = 1;
            }
        } else
#endif
        {
            bars = extract_lightbars(frame);
            cv::Mat hsv0, ch0[3];
            cv::cvtColor(frame, hsv0, cv::COLOR_BGR2HSV);
            cv::split(hsv0, ch0);
            armors = match_armors(bars, ch0[2]);
        }

        stat_bars  += static_cast<int>(bars.size());
        stat_armor += static_cast<int>(armors.size());
        stat_detsrc[det_source]++;

        // ------------------------------------------------ 解算 -> 候选
        std::vector<Target> candidates;
        candidates.reserve(armors.size());
        for (const auto& ar : armors) {
            if (ar.pts.size() != 4) continue;
            const SolveResult s = solve_pnp(ar.pts);
            if (!s.ok) continue;
            // 解算有效性: 距离与偏差角都要在合理范围, 否则是误配对
            if (s.dist < SOLVE_DIST_MIN || s.dist > SOLVE_DIST_MAX) continue;
            if (std::fabs(s.yaw) > SOLVE_ANGLE_MAX ||
                std::fabs(s.pitch) > SOLVE_ANGLE_MAX) continue;
            Target t;
            t.center = ar.center;
            t.solve  = s;
            t.color  = ar.color;
            candidates.push_back(t);
        }
        stat_solved += static_cast<int>(candidates.size());

        // ------------------------------------------------ 跟踪
        const TrackResult tr = tracker.update(candidates);
        if (!candidates.empty()) last_candidate_frame = frame_cnt;
        if (tr.locked && tr.index >= 0) {
            ++stat_locked;
            last_locked_frame = frame_cnt;
        }

        // ------------------------------------------------ 云台 -> 串口
        float theta_yaw = 0.0f, theta_pitch = 0.0f;

        // 选要跟的候选:
        //   已锁定 -> 用跟踪器选中的那个(保持目标连续性)
        //   未锁定但有候选 -> 直接选**离画面中心最近**的那个
        //
        // 为什么未锁定也要跟: 实测(10-16)发现"等 tracker 连续确认才驱动云台"
        // 这个门槛太高 —— 检测逐帧抖动时很难满足确认条件, 云台就一直不动,
        // 目标慢慢漂出画面, 然后彻底丢失。改成"只要有候选就先往它那儿转",
        // 云台把目标往画面中心压, 检测反而更稳, 锁定是水到渠成的结果。
        // 这也是常规做法: 跟踪器负责"跟哪一个 + 平滑", 而不是负责"要不要动"。
        int follow = -1;
        if (tr.locked && tr.index >= 0) {
            follow = tr.index;
        } else if (!candidates.empty()) {
            float best = 1e9f;
            const cv::Point2f img_c(static_cast<float>(frame.cols) * 0.5f,
                                    static_cast<float>(frame.rows) * 0.5f);
            for (size_t i = 0; i < candidates.size(); ++i) {
                const float d = cv::norm(candidates[i].center - img_c);
                if (d < best) {
                    best = d;
                    follow = static_cast<int>(i);
                }
            }
        }

        // ------------------------------------------------ 取回传
        // 真机走真实回传; mock 模式(没有下位机, 或用视频离线跑)用一个简单被控
        // 对象模型顶上: 实测角按一阶滞后跟随上一条命令角。
        //
        // 为什么必须补这个: 原来这里只认真实回传, 而 mock 下 link.state() 永远
        // 无效(age 极大), 于是 gimbal.track() 一次都不会被调用 —— 拿视频跑时伺服
        // 根本不工作, "视频 -> 检测 -> 解算 -> 伺服" 这条链没法验证。
        RxState fb = link.state();
        const bool sim = force_mock || serial_fd < 0;
        if (sim) {
            fb.valid = true;
            fb.mode  = 1;
            fb.stamp = now_s();
            sim_yaw_   += 0.35f * (gimbal.cmd_yaw   - sim_yaw_);
            sim_pitch_ += 0.35f * (gimbal.cmd_pitch - sim_pitch_);
            fb.yaw   = sim_yaw_;
            fb.pitch = sim_pitch_;
        }
        const bool fb_fresh = fb.valid && fb.age() < FEEDBACK_MAX_AGE_S;

        // ---- mode 识别: 变化时打一条醒目的日志 ----
        // 首次拿到回传也要打一次, 这样"现在到底能不能控"一眼可见。
        static int last_mode = -999;
        if (fb_fresh && fb.mode != last_mode) {
            if (last_mode == -999) {
                LOG("[MODE] 当前 mode=%d : %s\n", fb.mode, mode_name(fb.mode));
            } else {
                LOG("[MODE] mode %d -> %d : %s\n", last_mode, fb.mode, mode_name(fb.mode));
            }
            last_mode = fb.mode;
        }
        // mode != 1 时指令会被下位机丢掉, 此时绝不能推进伺服内部状态,
        // 否则命令角会在"没人听"的指令上持续累积, 等真正切到 mode=1 时会甩一下。
        // 严格按 mode==1 判断, **不做去抖**。
        // 曾经加过 60 帧的容忍, 但操作者确认 mode 的 0/1 跳变是他在拨遥控器开关,
        // 那就必须立即响应: 他去关自瞄, 我们不该再继续认为在自瞄。
        const bool vision_on = (fb.valid && fb.mode == 1);

        // **实测角必须无条件更新**, 与 mode 无关。
        //
        // 之前的写法是"mode!=1 就不 update_feedback", 目的是防止指令被丢弃时
        // 伺服内部状态累积。但副作用更严重: mode=0 时 meas 和 home 永远停在
        // 默认值(90/90 和 0/0), 日志里 cmd=90.00 fb=18.93 home=0.00 全是假的,
        // 而且 home 错了之后安全包络的中心也是错的 —— 一旦切到 mode=1,
        // 包络会以 0/0 为中心, 直接把云台拉向错误方向。
        //
        // 正确做法: 实测角/hom 始终跟真实回传走; 只有**推进伺服**那一步
        // (即 gimbal.track()) 才要求 vision_on, 因为那才是会产生指令的动作。
        if (fb_fresh) {
            gimbal.update_feedback(fb.yaw, fb.pitch);
        }

        if (hold_only) {
            // --hold: 检测照跑并打印(可以看 theta 有多大), 但**一帧都不推进伺服**。
            // 命令角保持不动, 云台就停在原地。
            // 用途: ① 验证"切换视觉模式时不动"; ② 量出目标相对光轴的偏角有多大,
            // 从而判断是"目标本来没对准"还是"解算有系统偏差"。
            if (follow >= 0) {
                theta_yaw   = candidates[follow].solve.yaw;
                theta_pitch = candidates[follow].solve.pitch;
            }
            gimbal.stop_tracking();
            if (fb_fresh) gimbal.update_feedback(fb.yaw, fb.pitch);
        } else if (follow >= 0) {
            float pitch_comp = 0.0f;
            if (BALLISTIC_COMP_ENABLE) {
                const float v = fb.has_bullet_speed ? fb.bullet_speed : BULLET_SPEED_DEFAULT;
                pitch_comp = ballistic_pitch_deg(candidates[follow].solve.dist, v);
            }
            theta_yaw   = candidates[follow].solve.yaw;
            theta_pitch = candidates[follow].solve.pitch + pitch_comp;
            if (fb_fresh && vision_on) {
                gimbal.track(theta_yaw, theta_pitch);
                if (tr.locked) last_locked_frame = frame_cnt;
            } else if (fb_fresh && !vision_on) {
                // 检出正常但下位机不接收 —— 每分钟只提醒一次, 避免刷屏
                static int warn_cnt = 0;
                if (++warn_cnt % 900 == 1) {
                    LOG("[MODE][WARN] 检出装甲板但 mode=%d, 指令被忽略, 云台不会跟随。"
                        "请在车上按鼠标右键进入视觉控制模式。\n", fb.mode);
                }
            } else {
                LOG("[WARN] 回传数据过期(%.3fs), 云台冻结\n", fb.age());
            }
        } else {
            gimbal.stop_tracking();
            // 没有候选时的行为。
            //
            // 默认 **原地保持**, 不摆扫 —— 理由见 config.hpp 的 ENABLE_AUTO_SCAN:
            // 操作手是先手动对准再进视觉模式的, 启动姿态就是靶标方向;
            // 程序再自动摆扫只会把靶标转出画面, 实测就是这么丢掉目标的。
            // 想自动搜索时用 --scan。
            if (auto_scan && frame_cnt - last_candidate_frame > GIMBAL_SCAN_HOLD) {
                if (fb_fresh) gimbal.scan();
            }
            // 不摆扫时什么都不做: 命令角保持不动, 云台就停在这儿等目标回来。
            // (下面仍然每帧 set_target, 否则会触发下位机看门狗)
        }

        const int32_t fire = (ENABLE_FIRE && tr.locked) ? 1 : 0;
        link.set_target(gimbal.cmd_yaw, gimbal.cmd_pitch, fire);

        // ------------------------------------------------ 可视化
        cv::Mat disp;
        const bool need_draw = !headless || show_debug || save_video || still_image;
        if (need_draw) {
            disp = frame.clone();
            if (show_debug) draw_debug(disp, bars, armors);
            if (tr.index >= 0 && tr.index < static_cast<int>(candidates.size())) {
                const cv::Point2f c = candidates[tr.index].center;
                cv::circle(disp, c, 12, cv::Scalar(0, 255, 0), 2);
                cv::line(disp, cv::Point2f(c.x - 22, c.y), cv::Point2f(c.x + 22, c.y),
                         cv::Scalar(0, 255, 0), 1);
                cv::line(disp, cv::Point2f(c.x, c.y - 22), cv::Point2f(c.x, c.y + 22),
                         cv::Scalar(0, 255, 0), 1);
            }
            const RxState& s = fb;   // 用伺服实际使用的回传(mock 下是模拟值)
            char buf[320];
            std::snprintf(buf, sizeof(buf),
                          "%s lock=%d cand=%d dist=%.2f th=%.1f/%.1f | cmd %.1f/%.1f"
                          " fb %.1f/%.1f v=%.1f dir=%.0f/%.0f",
                          tracker.state_name(), tr.locked ? 1 : 0,
                          static_cast<int>(candidates.size()),
                          tr.index >= 0 ? tr.target.solve.dist : 0.0f,
                          theta_yaw, theta_pitch,
                          gimbal.cmd_yaw, gimbal.cmd_pitch,
                          s.valid ? s.yaw : 0.0f, s.valid ? s.pitch : 0.0f, last_bullet_speed,
                          gimbal.servo_dir_yaw, gimbal.servo_dir_pitch);
            cv::putText(disp, buf, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }
        if (save_video && vid_writer.isOpened() && !disp.empty()) {
            vid_writer.write(disp);
        }

        if (frame_cnt % log_every == 0) {
            const RxState& s = fb;   // 同上: mock 下才有意义的值
            LOG("[DET] f=%d bars=%.1f armor=%.1f solved=%.1f locked=%.1f state=%s "
                "want=%.1f/%.1f cmd=%.1f/%.1f fb=%.1f/%.1f home=%.1f/%.1f mode=%d th=%.1f/%.1f "
                "lock_age=%d rx=%llu resync=%llu wd=%llu\n",
                frame_cnt, stat_bars / static_cast<double>(log_every),
                stat_armor / static_cast<double>(log_every),
                stat_solved / static_cast<double>(log_every),
                stat_locked / static_cast<double>(log_every), tracker.state_name(), gimbal.cmd_yaw, gimbal.cmd_pitch,
                s.valid ? s.yaw : 0.0f, s.valid ? s.pitch : 0.0f,
                gimbal.home_yaw, gimbal.home_pitch,
                s.valid ? s.mode : -1,
                theta_yaw, theta_pitch,
                gimbal.last_want_yaw, gimbal.last_want_pitch,
                frame_cnt - last_locked_frame,   // 距上次成功锁定过了多少帧
                static_cast<unsigned long long>(link.rx_frames()),
                static_cast<unsigned long long>(link.resync_drops()),
                static_cast<unsigned long long>(link.watchdog_trips()));
            // mode != 1 说明下位机没进入视觉控制模式, 此时它会完全忽略 ID1,
            // 表现为"检测锁定正常、命令也在发, 但云台一动不动"
            if (s.valid && s.mode != 1) {
                LOG("[WARN] 下位机 mode=%d 不是 1, 视觉控制未开启 -> 云台不会跟随!\n"
                     "       请在车上按鼠标右键进入视觉控制模式\n", s.mode);
            }
            stat_bars = stat_armor = stat_solved = stat_locked = 0;
        }

        // ------------------------------------------------ 交互 / 单帧
        if (still_image) {
            if (!disp.empty()) {
                const std::string out = snapshot_dir + "/still_annotated.jpg";
                cv::imwrite(out, disp);
                LOG("[SNAP] %s\n", out.c_str());
            }
            LOG("[SUMMARY] bars=%d armor=%d solved=%d locked=%d state=%s\n",
                static_cast<int>(bars.size()), static_cast<int>(armors.size()),
                static_cast<int>(candidates.size()), stat_locked, tracker.state_name());
            break;
        }

        if (!headless) {
            if (!disp.empty()) cv::imshow("my_aim", disp);
            const int k = cv::waitKey(1);
            if (k == 'q' || k == 27) break;
            if (k == 'w' || k == 'W') gimbal.manual_pitch( GIMBAL_MANUAL_STEP);
            if (k == 's' || k == 'S') gimbal.manual_pitch(-GIMBAL_MANUAL_STEP);
            if (k == 'a' || k == 'A') gimbal.manual_yaw(  -GIMBAL_MANUAL_STEP);
            if (k == 'd' || k == 'D') gimbal.manual_yaw(   GIMBAL_MANUAL_STEP);
            if (use_hik && (k == 't' || k == 'T')) hik.cycle_bayer();
        }

        if (max_frames > 0 && frame_cnt >= max_frames) break;
    }

    // ---------------------------------------------------------- 收尾
    if (save_video && vid_writer.isOpened()) vid_writer.release();
    LOG("[EXIT] frames=%d tx=%llu rx=%llu bad=%llu resync=%llu wd=%llu\n",
        frame_cnt,
        static_cast<unsigned long long>(link.tx_frames()),
        static_cast<unsigned long long>(link.rx_frames()),
        static_cast<unsigned long long>(link.rx_bad_frames()),
        static_cast<unsigned long long>(link.resync_drops()),
        static_cast<unsigned long long>(link.watchdog_trips()));

    link.stop();          // 会补发零帧再关串口
    hik.close();
    if (g_log) std::fclose(g_log);
    return 0;
}

// servo_calib.cpp: 实机自动测定伺服方向符号
//
// 为什么要这个工具:
//   手动试符号试了四轮, 每轮都被下一轮观测推翻。根本原因是"从遥测数字反推
//   坐标约定"不可靠 —— 还要同时依赖相机安装方向、云台轴向、图像翻转三重约定。
//
// 换成一个**可直接观测的判据**:
//
//   在锁住一个目标的情况下, 主动把命令角朝一个方向挪一小步,
//   然后看偏差角 |theta| 是变小还是变大。
//     变小 -> 这一步方向对 -> 该轴符号 = 挪动的方向
//     变大 -> 方向反了      -> 该轴符号取反
//
// 这个判据不依赖任何坐标约定: 它只问"往这边走, 目标有没有更靠中间"。
// yaw / pitch 分别独立测。
//
// 用法:
//   ./servo_calib                交互式: 等自瞄模式 -> 自动测两轴 -> 打印结果
//   ./servo_calib --step 6       单次挪动角度(默认 6 度)
//   ./servo_calib --settle 25    每次挪动后等待稳定的帧数(默认 25)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <unistd.h>

#include "config.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "tracker.hpp"
#include "yolo_armor.hpp"
#include "yolo_detector.hpp"

using namespace srm;

static const char* kSerialById = "/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_"
                                 "3855315E3439-if00";

struct Ctx {
    SerialLink link;
    HikCamera  cam;
    std::unique_ptr<YoloDetector> yolo;
};

// 抓一帧 -> YOLO -> 取离画面中心最近的板 -> 返回其偏角。
// 返回 false 表示这一帧没检到可用目标。
static bool grab_theta(Ctx& c, cv::Mat& frame, float& ty, float& tp, float& dist) {
    for (int tries = 0; tries < 30; ++tries) {
        if (!c.cam.read(frame)) continue;
        // IMG_DIR 只对实相机帧生效(与 main.cpp 一致); 这里就是相机帧。
        if (IMG_DIR == 1)      cv::flip(frame, frame, 1);
        else if (IMG_DIR == 2) cv::flip(frame, frame, 0);
        else if (IMG_DIR == 3) cv::flip(frame, frame, -1);
        const auto boxes = c.yolo->detect(frame);
        if (boxes.empty()) continue;
        const auto armors = yolo_to_armors(boxes);
        if (armors.empty()) continue;

        const cv::Point2f img_c(frame.cols * 0.5f, frame.rows * 0.5f);
        int best = -1;
        float bestd = 1e9f;
        for (size_t i = 0; i < armors.size(); ++i) {
            const float d = cv::norm(armors[i].center - img_c);
            if (d < bestd) { bestd = d; best = static_cast<int>(i); }
        }
        if (best < 0) continue;
        const SolveResult r = solve_pnp(armors[best].pts);
        if (!r.ok) continue;
        ty = r.yaw;
        tp = r.pitch;
        dist = r.dist;
        return true;
    }
    return false;
}

// 多次采样取平均, 抑制单帧噪声
static bool sample_theta(Ctx& c, int n, float& ty, float& tp, float& dist) {
    float sy = 0, sp = 0, sd = 0;
    int got = 0;
    for (int i = 0; i < n; ++i) {
        cv::Mat f;
        float a, b, d;
        if (grab_theta(c, f, a, b, d)) { sy += a; sp += b; sd += d; ++got; }
    }
    if (got == 0) return false;
    ty = sy / got; tp = sp / got; dist = sd / got;
    return got >= n / 3;   // 至少三分之一的帧有目标才算可信
}

// 等待 mode==1
static bool wait_aim_mode(Ctx& c, int seconds) {
    for (int i = 0; i < seconds; ++i) {
        const RxState s = c.link.state();
        std::printf("\r  wait %3ds  mode=%d   ", i, s.valid ? s.mode : -1);
        std::fflush(stdout);
        if (s.valid && s.mode == 1) { std::printf("\n"); return true; }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::printf("\n");
    return false;
}

int main(int argc, char** argv) {
    float step_deg  = 6.0f;
    int   settle    = 25;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--step") && i + 1 < argc)        step_deg = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--settle") && i + 1 < argc) settle   = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            std::printf("用法: servo_calib [--step DEG] [--settle N]\n"
                        "  实机自动测 yaw/pitch 的伺服方向符号。\n"
                        "  需要: 自瞄模式已开(遥控器), 画面里有装甲板。\n");
            return 0;
        }
    }

    Ctx c;
    int fd = open_first_available({kSerialById, "/dev/ttyACM0", "/dev/ttyACM1"});
    if (fd < 0) { std::fprintf(stderr, "[ERR] 串口打不开\n"); return 1; }
    c.link.start(fd, SERIAL_SEND_HZ);

    try {
        c.yolo = std::make_unique<YoloDetector>(
            make_yolo_profile(YOLO_PROFILE_SELECT), YOLO_CONF_THRESH, YOLO_NMS_THRESH);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ERR] 模型加载失败: %s\n", e.what());
        return 1;
    }
    if (!c.cam.open()) { std::fprintf(stderr, "[ERR] 相机打不开\n"); return 1; }

    std::printf("=====================================================\n");
    std::printf(" 请用【遥控器】打开自瞄, 并把靶标放在画面内\n");
    std::printf(" 标定过程中请勿触碰云台与摇杆\n");
    std::printf("=====================================================\n");
    if (!wait_aim_mode(c, 150)) {
        std::fprintf(stderr, "[TIMEOUT] 未进入 mode=1\n");
        return 1;
    }

    // 先测基准
    float ty0 = 0, tp0 = 0, d0 = 0;
    if (!sample_theta(c, 15, ty0, tp0, d0)) {
        std::fprintf(stderr, "[ERR] 抓不到稳定的装甲板, 请把靶标放到画面里\n");
        return 1;
    }
    RxState st = c.link.state();
    const float meas_yaw0   = st.yaw;
    const float meas_pitch0 = st.pitch;
    std::printf("\n基准: 实测 yaw=%.2f pitch=%.2f   偏角 th=%.2f/%.2f  距离 %.2f m\n",
                meas_yaw0, meas_pitch0, ty0, tp0, d0);

    // 保持角: 用实测角作为命令基准(绝对角), 挪一步后停住等待稳定
    auto probe_axis = [&](bool is_yaw, float sign) -> float {
        char cmd_y = meas_yaw0, cmd_p = meas_pitch0;
        if (is_yaw) cmd_y += sign * step_deg;
        else        cmd_p += sign * step_deg;
        c.link.set_target(cmd_y, cmd_p, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        for (int i = 0; i < settle; ++i) {
            c.link.set_target(cmd_y, cmd_p, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        // 读稳定后的偏角
        float ty, tp, dd;
        if (!sample_theta(c, 12, ty, tp, dd)) return 1e9f;
        return is_yaw ? std::fabs(ty) : std::fabs(tp);
    };

    // yaw: 先试 +step。|theta| 变小说明 +方向对
    std::printf("\n--- 测 yaw: 命令 +%.1f 度 ---\n", (double)step_deg);
    const float yaw_after_plus = probe_axis(true, +1.0f);
    // 回到基准
    c.link.set_target(meas_yaw0, meas_pitch0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    std::printf("\n--- 测 pitch: 命令 +%.1f 度 ---\n", (double)step_deg);
    const float pitch_after_plus = probe_axis(false, +1.0f);
    c.link.set_target(meas_yaw0, meas_pitch0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    const float base_yaw   = std::fabs(ty0);
    const float base_pitch = std::fabs(tp0);

    std::printf("\n================= 结果 =================\n");
    std::printf("  基准 |th|    yaw=%.2f   pitch=%.2f\n", (double)base_yaw, (double)base_pitch);
    std::printf("  命令+后 |th| yaw=%.2f   pitch=%.2f\n",
                (double)yaw_after_plus, (double)pitch_after_plus);

    auto verdict = [](const char* axis, float b, float a) {
        if (!std::isfinite(a) || a > 1e8f) {
            std::printf("  %-5s : 测不出来(挪动后丢目标)\n", axis);
            return;
        }
        if (a < b - 0.5f) {
            std::printf("  %-5s : SERVO_DIR = +1  (命令增大使偏角变小)\n", axis);
        } else if (a > b + 0.5f) {
            std::printf("  %-5s : SERVO_DIR = -1  (命令增大使偏角变大)\n", axis);
        } else {
            std::printf("  %-5s : 分辨不出(变化 %.2f 度, 太小)\n", axis, (double)(a - b));
        }
    };
    verdict("yaw",   base_yaw,   yaw_after_plus);
    verdict("pitch", base_pitch, pitch_after_plus);
    std::printf("========================================\n");
    std::printf("把上面两行结果写进 config.hpp 的 SERVO_DIR_YAW / SERVO_DIR_PITCH,\n");
    std::printf("或者直接用 --dir-yaw / --dir-pitch 在命令行指定。\n");

    c.link.stop();
    c.cam.close();
    return 0;
}

// aim_gui.cpp: 可视化调试界面 —— 两个独立窗口
//
// 两个设计决定, 都是踩过之后定的:
//
// 1) **图传与参数分成两个窗口**。
//    挤在同一张画布上时, 图传被压小、参数挤成小字, 两边都看不清。
//    现在 camera 窗口全给图传, params 窗口独立显示文本与滑条。
//
// 2) **所有屏幕文字必须是 ASCII**。
//    OpenCV 的 putText 用 Hershey 字体, **不含任何中文字形**, 写中文会全部
//    渲染成 '?', 界面上就是"字母全是问号"。所以下面标签一律英文缩写,
//    中文只留在注释里(注释不上屏, 没这个问题)。
//
// 3) 界面显示的值就是实际收发/实际生效的值, 不另算一份 —— 否则又会重演
//    "显示对但功能错"的假象。
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
#include "fire.hpp"
#include "gimbal.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "yolo_armor.hpp"
#include "yolo_detector.hpp"

using namespace srm;

static const char* kSerialById = "/dev/serial/by-id/usb-YueLuEmbedded_Vision_Comm_port_"
                                 "3855315E3439-if00";

static int tb_conf      = 45;    // /100
static int tb_enemy     = 0;     // 0=blue 1=red 2=both
static int tb_off_yaw   = 0;     // /10 deg
static int tb_off_pitch = 0;
static int tb_lat_ms    = 80;
static int tb_gain      = 100;   // /100

static void row(cv::Mat& m, int& y, const char* k, const std::string& v,
                const cv::Scalar& col) {
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%-10s %s", k, v.c_str());
    cv::putText(m, buf, cv::Point(14, y), cv::FONT_HERSHEY_SIMPLEX, 0.52, col, 1, cv::LINE_AA);
    y += 23;
}
static void head(cv::Mat& m, int& y, const char* t, const cv::Scalar& col) {
    cv::putText(m, t, cv::Point(14, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
    cv::line(m, cv::Point(14, y + 6), cv::Point(400, y + 6), col, 1);
    y += 26;
}

int main(int argc, char** argv) {
    bool use_cam = true;
    std::string model = "v8", img;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--model") && i + 1 < argc) model = argv[++i];
        else if (!std::strcmp(argv[i], "--img") && i + 1 < argc) { img = argv[++i]; use_cam = false; }
        else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            std::printf("usage: aim_gui [--model v11|v8] [--img FILE]\n");
            return 0;
        }
    }

    SerialLink link;
    const int fd = open_first_available({kSerialById, "/dev/ttyACM0", "/dev/ttyACM1"});
    if (fd >= 0) link.start(fd, SERIAL_SEND_HZ);
    else std::fprintf(stderr, "[WARN] no serial, display only\n");

    const int profile_sel = (model == "v11") ? YOLO_PROFILE_YOLO11_38CLASS
                          : (model == "v5")  ? YOLO_PROFILE_YOLOV5_10CLASS
                                             : YOLO_PROFILE_YOLOV8_2CLASS;
    std::unique_ptr<YoloDetector> yolo;
    int last_conf = -999;
    auto rebuild = [&](int c100) {
        try {
            yolo = std::make_unique<YoloDetector>(make_yolo_profile(profile_sel),
                                                  static_cast<float>(c100) / 100.0f,
                                                  YOLO_NMS_THRESH);
            last_conf = c100;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[ERR] model: %s\n", e.what());
        }
    };
    rebuild(tb_conf);
    if (!yolo) return 1;

    HikCamera cam;
    cv::Mat still;
    if (use_cam) {
        if (!cam.open()) { std::fprintf(stderr, "[ERR] camera\n"); return 1; }
    } else {
        still = cv::imread(img);
        if (still.empty()) { std::fprintf(stderr, "[ERR] no %s\n", img.c_str()); return 1; }
    }

    Tracker    tracker;
    GimbalCtrl gimbal;

    const char* kW1 = "camera";
    const char* kW2 = "params";
    cv::namedWindow(kW1, cv::WINDOW_NORMAL);
    cv::namedWindow(kW2, cv::WINDOW_NORMAL);
    cv::resizeWindow(kW1, 1100, 800);
    cv::resizeWindow(kW2, 420, 780);
    cv::createTrackbar("conf x100",      kW2, &tb_conf,      95);
    cv::createTrackbar("enemy 0B 1R 2all", kW2, &tb_enemy,   2);
    cv::createTrackbar("off yaw x10",    kW2, &tb_off_yaw,   100);
    cv::createTrackbar("off pitch x10",  kW2, &tb_off_pitch, 100);
    cv::createTrackbar("latency ms",     kW2, &tb_lat_ms,    300);
    cv::createTrackbar("gain x100",      kW2, &tb_gain,      200);

    double last_t = 0.0;

    while (true) {
        cv::Mat frame;
        if (use_cam) {
            if (!cam.read(frame)) continue;
            if (IMG_DIR == 1) cv::flip(frame, frame, 1);
            else if (IMG_DIR == 2) cv::flip(frame, frame, 0);
            else if (IMG_DIR == 3) cv::flip(frame, frame, -1);
        } else {
            frame = still.clone();
        }
        const double now = now_s();
        const double dt = (last_t > 0.0) ? (now - last_t) : (1.0 / 60.0);
        last_t = now;

        if (tb_conf != last_conf) rebuild(tb_conf);
        if (!yolo) continue;

        const auto boxes = yolo->detect(frame);
        const int want_color = (tb_enemy == 0) ? 0 : (tb_enemy == 1 ? 1 : -1);
        auto armors = filter_by_color(yolo_to_armors(boxes), want_color);

        std::vector<Target> cand;
        for (const auto& a : armors) {
            Target t;
            t.center = a.center;
            t.color  = 0;
            t.solve  = solve_pnp(a.pts);
            if (t.solve.ok) cand.push_back(t);
        }

        const TrackResult tr = tracker.update(cand);
        int follow = -1;
        if (!cand.empty()) follow = (tr.locked && tr.index >= 0) ? tr.index : 0;

        const RxState fb = link.state();
        const bool fresh = fb.valid && fb.age() < 0.2;
        const bool vis   = fb.valid && fb.mode == 1;
        if (fresh) gimbal.update_feedback(fb.yaw, fb.pitch);

        gimbal.servo_gain       = static_cast<float>(tb_gain) / 100.0f;
        gimbal.aim_offset_yaw   = static_cast<float>(tb_off_yaw) / 10.0f;
        gimbal.aim_offset_pitch = static_cast<float>(tb_off_pitch) / 10.0f;

        float th_y = 0.0f, th_p = 0.0f, dist = 0.0f;
        if (follow >= 0 && follow < static_cast<int>(cand.size())) {
            th_y = cand[follow].solve.yaw;
            th_p = cand[follow].solve.pitch;
            dist = cand[follow].solve.dist;
            if (fresh && vis) gimbal.track(th_y, th_p);
            else if (fresh)   gimbal.stop_tracking();
        } else {
            gimbal.stop_tracking();
        }
        const int32_t fire = 0;
        if (gimbal.have_meas) link.set_target(gimbal.cmd_yaw, gimbal.cmd_pitch, fire);

        // ------------------------------------------------ window 1: camera
        cv::Mat view = frame.clone();
        for (size_t i = 0; i < armors.size(); ++i) {
            const bool sel = (static_cast<int>(i) == follow);
            const cv::Scalar col = sel ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
            std::vector<cv::Point> poly;
            for (const auto& p : armors[i].pts)
                poly.push_back(cv::Point(cvRound(p.x), cvRound(p.y)));
            if (poly.size() == 4) cv::polylines(view, poly, true, col, sel ? 2 : 1);
            if (i < cand.size()) {
                char b[96];
                std::snprintf(b, sizeof(b), "y%.1f p%.1f d%.2f",
                              static_cast<double>(cand[i].solve.yaw),
                              static_cast<double>(cand[i].solve.pitch),
                              static_cast<double>(cand[i].solve.dist));
                cv::putText(view, b,
                            poly.empty() ? cv::Point(10, 50) : poly[0] + cv::Point(0, -6),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
            }
        }
        {   // 光轴中心十字 = theta 0 的位置。板不在十字上而 theta 接近 0 -> 标定问题
            const cv::Point c(view.cols / 2, view.rows / 2);
            cv::line(view, c + cv::Point(-22, 0), c + cv::Point(22, 0),
                     cv::Scalar(0, 255, 0), 1);
            cv::line(view, c + cv::Point(0, -22), c + cv::Point(0, 22),
                     cv::Scalar(0, 255, 0), 1);
        }
        {
            char b[192];
            std::snprintf(b, sizeof(b), "yolo=%d solved=%d locked=%d %s fps=%.0f",
                          static_cast<int>(boxes.size()), static_cast<int>(cand.size()),
                          tr.locked ? 1 : 0, tracker.state_name(), 1.0 / std::fmax(dt, 1e-6));
            cv::putText(view, b, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }
        cv::imshow(kW1, view);

        // ------------------------------------------------ window 2: params
        cv::Mat pnl(780, 420, CV_8UC3, cv::Scalar(28, 28, 28));
        int y = 30;
        char b[192];
        const cv::Scalar CY(200, 230, 120), GR(150, 240, 150), OR(120, 190, 255),
                         GY(190, 190, 190), RD(80, 80, 255), GN(80, 255, 80);

        head(pnl, y, "SENT", CY);
        std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(gimbal.cmd_yaw),
                      static_cast<double>(gimbal.cmd_pitch));
        row(pnl, y, "cmd", b, CY);
        std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(gimbal.last_want_yaw),
                      static_cast<double>(gimbal.last_want_pitch));
        row(pnl, y, "want", b, CY);
        row(pnl, y, "fire", std::to_string(fire), CY);

        y += 8;
        head(pnl, y, "RECV", GR);
        if (fb.valid) {
            std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(fb.yaw),
                          static_cast<double>(fb.pitch));
            row(pnl, y, "fb", b, GR);
            std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(fb.roll));
            row(pnl, y, "roll", b, GR);
            row(pnl, y, "mode", std::to_string(fb.mode) + (vis ? "  VISION ON" : "  off"), GR);
            std::snprintf(b, sizeof(b), "%.1f m/s", static_cast<double>(fb.bullet_speed));
            row(pnl, y, "bullet", b, GR);
            std::snprintf(b, sizeof(b), "%.0f ms%s", fb.age() * 1000.0, fresh ? "" : "  STALE");
            row(pnl, y, "age", b, fresh ? GR : RD);
        } else {
            row(pnl, y, "fb", "NO DATA", RD);
        }

        y += 8;
        head(pnl, y, "DETECT", OR);
        row(pnl, y, "yolo", std::to_string(boxes.size()), OR);
        row(pnl, y, "solved", std::to_string(cand.size()), OR);
        row(pnl, y, "state", tracker.state_name(), OR);
        row(pnl, y, "locked", tr.locked ? "1  LOCKED" : "0", tr.locked ? GN : OR);
        if (follow >= 0) {
            std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(th_y),
                          static_cast<double>(th_p));
            row(pnl, y, "theta", b, OR);
            std::snprintf(b, sizeof(b), "%.2f m", static_cast<double>(dist));
            row(pnl, y, "dist", b, OR);
            std::snprintf(b, sizeof(b), "%.1f deg",
                          static_cast<double>(plate_angular_width_deg(dist)));
            row(pnl, y, "plate w", b, OR);
            std::snprintf(b, sizeof(b), "%.1f deg",
                          static_cast<double>(fire_tolerance_deg(dist)));
            row(pnl, y, "fire tol", b, OR);
        }

        y += 8;
        head(pnl, y, "PARAM", GY);
        std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(tb_conf) / 100.0);
        row(pnl, y, "conf", b, GY);
        row(pnl, y, "enemy", tb_enemy == 0 ? "blue" : (tb_enemy == 1 ? "red" : "all"), GY);
        std::snprintf(b, sizeof(b), "%+.0f / %+.0f", gimbal.servo_dir_yaw,
                      gimbal.servo_dir_pitch);
        row(pnl, y, "servo dir", b, GY);
        std::snprintf(b, sizeof(b), "%.1f / %.1f", gimbal.effective_home_yaw(),
                      gimbal.effective_home_pitch());
        row(pnl, y, "home", b, GY);
        std::snprintf(b, sizeof(b), "%.1f / %.1f", static_cast<double>(gimbal.aim_offset_yaw),
                      static_cast<double>(gimbal.aim_offset_pitch));
        row(pnl, y, "offset", b, GY);
        row(pnl, y, "latency", std::to_string(tb_lat_ms) + " ms", GY);
        std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(gimbal.servo_gain));
        row(pnl, y, "gain", b, GY);

        cv::imshow(kW2, pnl);

        const int k = cv::waitKey(1);
        if (k == 27 || k == 'q') break;
    }

    link.stop();
    if (use_cam) cam.close();
    return 0;
}

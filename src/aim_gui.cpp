// aim_gui.cpp: 可视化调试界面
//
// 目的: 一屏同时看到「发出去什么 / 收到什么 / 检测到什么 / 当前参数」，
//       现场调参不用再读滚动日志。
//
// 布局:
//   左侧  相机画面 + 检测框 + 锁定标记
//   右侧  文本面板(分四块: 发送 / 接收 / 检测 / 参数)
//   顶部  滑条(实时调参, 不用重编译)
//
// 关键原则: 界面里显示的值就是**实际收发和实际生效**的值，不是另算一份。
//          否则和日志一样会出现"显示对但功能错"的假象。
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

// ---- 滑条变量(OpenCV trackbar 只认 int) ----
static int tb_conf      = 45;    // /100 -> 0.45
static int tb_enemy     = 0;     // 0=blue 1=red 2=both
static int tb_dir_yaw   = -1;    // 只在重启后生效, 这里仅显示
static int tb_dir_pitch = -1;
static int tb_off_yaw   = 0;     // /10 -> 度
static int tb_off_pitch = 0;
static int tb_lat_ms    = 80;    // 毫秒
static int tb_gain      = 100;   // /100

static void draw_panel(cv::Mat& canvas, int x0, int y0, int w,
                       const std::vector<std::pair<std::string, std::string>>& rows,
                       const cv::Scalar& color) {
    int y = y0;
    for (const auto& r : rows) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%-14s %s", r.first.c_str(), r.second.c_str());
        cv::putText(canvas, buf, cv::Point(x0, y), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    color, 1, cv::LINE_AA);
        y += 20;
    }
    (void)w;
}

int main(int argc, char** argv) {
    bool use_cam = true;
    std::string model = "v8";
    std::string img;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--model") && i + 1 < argc) model = argv[++i];
        else if (!std::strcmp(argv[i], "--img") && i + 1 < argc) { img = argv[++i]; use_cam = false; }
        else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            std::printf("用法: aim_gui [--model v11|v8|路径] [--img FILE]\n"
                        "  可视化: 相机画面 + 检测框 + 收发量 + 参数滑条\n");
            return 0;
        }
    }

    // ---- 串口 ----
    SerialLink link;
    int fd = open_first_available({kSerialById, "/dev/ttyACM0", "/dev/ttyACM1"});
    if (fd >= 0) { link.start(fd, SERIAL_SEND_HZ); }
    else { std::fprintf(stderr, "[WARN] 串口打不开, 只显示不回传\n"); }

    // ---- 检测器 ----
    std::unique_ptr<YoloDetector> yolo;
    const int      profile_sel = (model == "v11") ? YOLO_PROFILE_YOLO11_38CLASS
                               : (model == "v5")  ? YOLO_PROFILE_YOLOV5_10CLASS
                                                  : YOLO_PROFILE_YOLOV8_2CLASS;
    int last_conf = -999;
    auto rebuild_yolo = [&](int conf100) {
        try {
            yolo = std::make_unique<YoloDetector>(make_yolo_profile(profile_sel),
                                                 static_cast<float>(conf100) / 100.0f,
                                                 YOLO_NMS_THRESH);
            last_conf = conf100;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[ERR] 模型加载失败: %s\n", e.what());
        }
    };
    try {
        yolo = std::make_unique<YoloDetector>(make_yolo_profile(model == "v11" ? YOLO_PROFILE_YOLO11_38CLASS
                                              : (model == "v5" ? YOLO_PROFILE_YOLOV5_10CLASS
                                                               : YOLO_PROFILE_YOLOV8_2CLASS)),
                                             static_cast<float>(tb_conf) / 100.0f,
                                             YOLO_NMS_THRESH);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ERR] 模型加载失败: %s\n", e.what());
        return 1;
    }

    // ---- 相机 ----
    HikCamera cam;
    cv::Mat still;
    if (use_cam) {
        if (!cam.open()) { std::fprintf(stderr, "[ERR] 相机打不开\n"); return 1; }
    } else {
        still = cv::imread(img);
        if (still.empty()) { std::fprintf(stderr, "[ERR] 读不到 %s\n", img.c_str()); return 1; }
    }

    Tracker    tracker;
    GimbalCtrl gimbal;

    cv::namedWindow("aim_gui", cv::WINDOW_NORMAL);
    cv::resizeWindow("aim_gui", 1500, 820);
    cv::createTrackbar("conf x100", "aim_gui", &tb_conf, 95);
    cv::createTrackbar("enemy 0B 1R 2both", "aim_gui", &tb_enemy, 2);
    cv::createTrackbar("off yaw x10", "aim_gui", &tb_off_yaw, 100);
    cv::createTrackbar("off pitch x10", "aim_gui", &tb_off_pitch, 100);
    cv::createTrackbar("latency ms", "aim_gui", &tb_lat_ms, 300);
    cv::createTrackbar("gain x100", "aim_gui", &tb_gain, 200);

    const int kPanelW = 520;
    int frame_cnt = 0;
    double last_img_t = 0.0;

    while (true) {
        cv::Mat frame;
        if (use_cam) {
            if (!cam.read(frame)) continue;
            if (IMG_DIR == 1) cv::flip(frame, frame, 1);
            else if (IMG_DIR == 2) cv::flip(frame, frame, 0);
            else if (IMG_DIR == 3) cv::flip(frame, frame, -1);
        } else {
            frame = still.clone();
            // 静态图按 60fps 刷, 便于调参观察
        }
        ++frame_cnt;
        const double now = now_s();
        const double dt  = (last_img_t > 0.0) ? (now - last_img_t) : (1.0 / 60.0);
        last_img_t = now;

        // ---- 检测 ----
        if (tb_conf != last_conf) rebuild_yolo(tb_conf);   // 接口没有 set_conf, 重建
        const auto boxes = yolo->detect(frame);
        int want_color = (tb_enemy == 0) ? 0 : (tb_enemy == 1 ? 1 : -1);
        auto armors = filter_by_color(yolo_to_armors(boxes), want_color);

        // Target 才是"解算成功的候选"的载体(见 tracker.hpp), Armor 只有几何。
        std::vector<Target> solved_armors;
        for (const auto& a : armors) {
            Target t;
            t.center = a.center;
            t.color  = 0;
            t.solve  = solve_pnp(a.pts);
            if (t.solve.ok) solved_armors.push_back(t);
        }

        // ---- 跟踪 ----
        const TrackResult tr = tracker.update(solved_armors);
        int follow = -1;
        if (!solved_armors.empty()) {
            follow = (tr.locked && tr.index >= 0) ? tr.index : 0;
        }

        // ---- 伺服 ----
        const RxState fb = link.state();
        const bool fb_fresh = fb.valid && fb.age() < 0.2;
        const bool vision_on = fb.valid && fb.mode == 1;
        if (fb_fresh) gimbal.update_feedback(fb.yaw, fb.pitch);

        gimbal.servo_gain       = static_cast<float>(tb_gain) / 100.0f;
        gimbal.aim_offset_yaw   = static_cast<float>(tb_off_yaw) / 10.0f;
        gimbal.aim_offset_pitch = static_cast<float>(tb_off_pitch) / 10.0f;

        float theta_yaw = 0.0f, theta_pitch = 0.0f, dist = 0.0f;
        if (follow >= 0 && follow < static_cast<int>(solved_armors.size())) {
            theta_yaw   = solved_armors[follow].solve.yaw;
            theta_pitch = solved_armors[follow].solve.pitch;
            dist        = solved_armors[follow].solve.dist;
            if (fb_fresh && vision_on) gimbal.track(theta_yaw, theta_pitch);
            else if (fb_fresh) gimbal.stop_tracking();
        } else {
            gimbal.stop_tracking();
        }
        const int32_t fire = 0;
        if (gimbal.have_meas) link.set_target(gimbal.cmd_yaw, gimbal.cmd_pitch, fire);

        // ---- 画面 ----
        cv::Mat view = frame.clone();
        for (size_t i = 0; i < armors.size(); ++i) {
            const bool sel = (static_cast<int>(i) == follow);
            // 自己画: draw_debug 的签名是给 LightBar 用的
            const cv::Scalar col = sel ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
            std::vector<cv::Point> poly;
            for (const auto& pt : armors[i].pts)
                poly.push_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
            if (poly.size() == 4) {
                cv::polylines(view, poly, true, col, sel ? 2 : 1);
            }
            if (i < solved_armors.size() && !armors[i].pts.empty()) {
                char b[96];
                std::snprintf(b, sizeof(b), "y%.1f p%.1f d%.1f",
                              static_cast<double>(solved_armors[i].solve.yaw),
                              static_cast<double>(solved_armors[i].solve.pitch),
                              static_cast<double>(solved_armors[i].solve.dist));
                cv::putText(view, b, armors[i].pts[0] + cv::Point2f(0, -6),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
            }
        }
        // 画面中心十字: 偏角为 0 的位置
        {
            const cv::Point c(view.cols / 2, view.rows / 2);
            cv::line(view, c + cv::Point(-18, 0), c + cv::Point(18, 0), cv::Scalar(0, 255, 0), 1);
            cv::line(view, c + cv::Point(0, -18), c + cv::Point(0, 18), cv::Scalar(0, 255, 0), 1);
        }

        // ---- 拼右侧面板 ----
        if (view.rows < 700) cv::resize(view, view, cv::Size(), 700.0 / view.rows, 700.0 / view.rows);
        cv::Mat canvas(view.rows, view.cols + kPanelW, view.type(), cv::Scalar(30, 30, 30));
        view.copyTo(canvas(cv::Rect(0, 0, view.cols, view.rows)));

        const int px = view.cols + 12;
        const int pw = kPanelW - 20;

        char buf[256];

        // ---- 发送 ----
        std::vector<std::pair<std::string, std::string>> tx;
        std::snprintf(buf, sizeof(buf), "%.2f / %.2f",
                      static_cast<double>(gimbal.cmd_yaw), static_cast<double>(gimbal.cmd_pitch));
        tx.push_back({"发送 cmd", buf});
        tx.push_back({"发送 fire", std::to_string(fire)});
        std::snprintf(buf, sizeof(buf), "%.3f / %.3f",
                      static_cast<double>(gimbal.last_want_yaw),
                      static_cast<double>(gimbal.last_want_pitch));
        tx.push_back({"期望 want", buf});
        tx.push_back({"发送频率", std::to_string(SERIAL_SEND_HZ) + " Hz"});
        draw_panel(canvas, px, 24, pw, tx, cv::Scalar(120, 220, 255));

        // ---- 接收 ----
        std::vector<std::pair<std::string, std::string>> rx;
        if (fb.valid) {
            std::snprintf(buf, sizeof(buf), "%.2f / %.2f",
                          static_cast<double>(fb.yaw), static_cast<double>(fb.pitch));
            rx.push_back({"接收 fb", buf});
            std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(fb.roll));
            rx.push_back({"roll", buf});
            rx.push_back({"mode", std::to_string(fb.mode) +
                                  (fb.mode == 1 ? "  (视觉开)" : "  (视觉关)")});
            std::snprintf(buf, sizeof(buf), "%.1f m/s", static_cast<double>(fb.bullet_speed));
            rx.push_back({"弹速", buf});
            std::snprintf(buf, sizeof(buf), "%.0f ms%s", fb.age() * 1000.0,
                          fb_fresh ? "" : "  ** 过期 **");
            rx.push_back({"回传年龄", buf});
        } else {
            rx.push_back({"接收", "** 无数据 **"});
        }
        draw_panel(canvas, px, 140, pw, rx, cv::Scalar(180, 255, 180));

        // ---- 检测 ----
        std::vector<std::pair<std::string, std::string>> det;
        det.push_back({"yolo 框", std::to_string(boxes.size())});
        det.push_back({"解算成功", std::to_string(solved_armors.size())});
        det.push_back({"state", tracker.state_name()});
        det.push_back({"locked", tr.locked ? "1  ** 已锁定 **" : "0"});
        if (follow >= 0 && follow < static_cast<int>(solved_armors.size())) {
            std::snprintf(buf, sizeof(buf), "%.2f / %.2f",
                          static_cast<double>(theta_yaw), static_cast<double>(theta_pitch));
            det.push_back({"th", buf});
            std::snprintf(buf, sizeof(buf), "%.2f m", static_cast<double>(dist));
            det.push_back({"dist", buf});
            std::snprintf(buf, sizeof(buf), "%.2f deg",
                          static_cast<double>(fire_tolerance_deg(dist)));
            det.push_back({"开火容差", buf});
            std::snprintf(buf, sizeof(buf), "%.1f deg",
                          static_cast<double>(plate_angular_width_deg(dist)));
            det.push_back({"板角宽", buf});
        }
        draw_panel(canvas, px, 300, pw, det, cv::Scalar(255, 200, 120));

        // ---- 参数 ----
        std::vector<std::pair<std::string, std::string>> par;
        std::snprintf(buf, sizeof(buf), "%.2f (滑条)", static_cast<double>(tb_conf) / 100.0);
        par.push_back({"conf", buf});
        std::snprintf(buf, sizeof(buf), "%+.0f / %+.0f", gimbal.servo_dir_yaw,
                      gimbal.servo_dir_pitch);
        par.push_back({"伺服方向", buf});
        std::snprintf(buf, sizeof(buf), "%.1f / %.1f", gimbal.effective_home_yaw(),
                      gimbal.effective_home_pitch());
        par.push_back({"包络中心", buf});
        std::snprintf(buf, sizeof(buf), "%.1f / %.1f",
                      static_cast<double>(gimbal.aim_offset_yaw),
                      static_cast<double>(gimbal.aim_offset_pitch));
        par.push_back({"瞄准偏移", buf});
        par.push_back({"预测外推", std::to_string(tb_lat_ms) + " ms"});
        std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(gimbal.servo_gain));
        par.push_back({"增益", buf});
        std::snprintf(buf, sizeof(buf), "%.1f fps", 1.0 / std::fmax(dt, 1e-6));
        par.push_back({"帧率", buf});
        draw_panel(canvas, px, 520, pw, par, cv::Scalar(200, 200, 200));

        cv::imshow("aim_gui", canvas);
        const int k = cv::waitKey(1);
        if (k == 27 || k == 'q') break;
    }

    link.stop();
    if (use_cam) cam.close();
    return 0;
}

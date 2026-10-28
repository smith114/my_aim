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
#include "kf.hpp"
#include "gimbal.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "select.hpp"
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
// 伺服方向符号, 0 -> -1, 1 -> +1。**做成滑条而不是常量**:
// 这个符号依赖 相机朝向/云台轴向/图像翻转 三重约定, 只能靠人眼看一次大动作确认,
// 每次都重编译部署太慢。滑条拨一下就能看到效果。
static int tb_dir_yaw   = 0;     // 默认 -1
static int tb_dir_pitch = 0;     // 默认 -1
static int tb_smooth    = 35;    // /100, 低通系数
static int tb_deadband  = 5;     // /10 -> 度

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

    Tracker       tracker;
    GimbalCtrl    gimbal;
    PlateSelector selector;
    // 延迟补偿: latency 滑条要真的接到滤波器上, 否则它就是假旋钮
    AngleKF kf_yaw(KF_Q_ANGLE, KF_Q_RATE, KF_R_ANGLE);
    AngleKF kf_pitch(KF_Q_ANGLE, KF_Q_RATE, KF_R_ANGLE);

    // 三个窗口, 各干一件事。
    // 关键: **滑条必须单独一个窗口**。OpenCV 的滑条占用窗口内部空间,
    // 8 个滑条约占 200px, 会把下面的文本顶出可视区 —— 之前"重要参数看不到"
    // 就是被裁掉了, 不是没画。
    const char* kW1 = "camera";
    const char* kW2 = "params";
    const char* kW3 = "knobs";
    cv::namedWindow(kW1, cv::WINDOW_NORMAL);
    cv::namedWindow(kW2, cv::WINDOW_NORMAL);
    cv::namedWindow(kW3, cv::WINDOW_NORMAL);
    cv::resizeWindow(kW1, 1100, 800);
    cv::resizeWindow(kW2, 470, 700);
    cv::resizeWindow(kW3, 440, 430);
    cv::createTrackbar("conf x100",      kW3, &tb_conf,      95);
    cv::createTrackbar("enemy 0B 1R 2all", kW3, &tb_enemy,   2);
    cv::createTrackbar("off yaw x10",    kW3, &tb_off_yaw,   100);
    cv::createTrackbar("off pitch x10",  kW3, &tb_off_pitch, 100);
    cv::createTrackbar("latency ms",     kW3, &tb_lat_ms,    300);
    cv::createTrackbar("gain x100",      kW3, &tb_gain,      200);
    cv::createTrackbar("dirYaw 0=-1 1=+1", kW3, &tb_dir_yaw,   1);
    cv::createTrackbar("dirPitch 0=-1 1=+1", kW3, &tb_dir_pitch, 1);
    cv::createTrackbar("smooth x100",     kW3, &tb_smooth,   100);
    cv::createTrackbar("deadband x10",    kW3, &tb_deadband, 50);

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
        // 与主程序**同一份**选择逻辑(含迟滞), 否则界面上看到的抖动不代表实车行为
        const int follow = selector.pick(cand);

        const RxState fb = link.state();
        const bool fresh = fb.valid && fb.age() < 0.2;
        const bool vis   = fb.valid && fb.mode == 1;
        if (fresh) gimbal.update_feedback(fb.yaw, fb.pitch);

        gimbal.servo_dir_yaw   = (tb_dir_yaw   == 0) ? -1.0f : 1.0f;
        gimbal.servo_dir_pitch = (tb_dir_pitch == 0) ? -1.0f : 1.0f;
        gimbal.smooth_coef      = static_cast<float>(tb_smooth) / 100.0f;
        gimbal.deadband         = static_cast<float>(tb_deadband) / 10.0f;
        gimbal.servo_gain       = static_cast<float>(tb_gain) / 100.0f;
        gimbal.aim_offset_yaw   = static_cast<float>(tb_off_yaw) / 10.0f;
        gimbal.aim_offset_pitch = static_cast<float>(tb_off_pitch) / 10.0f;

        float th_y = 0.0f, th_p = 0.0f, dist = 0.0f;
        float raw_y = 0.0f, raw_p = 0.0f;   // 滤波前的原始测量, 用于对照
        if (follow >= 0 && follow < static_cast<int>(cand.size())) {
            raw_y = cand[follow].solve.yaw;
            raw_p = cand[follow].solve.pitch;
            dist  = cand[follow].solve.dist;
            kf_yaw.predict(static_cast<float>(dt));
            kf_pitch.predict(static_cast<float>(dt));
            kf_yaw.update(raw_y);
            kf_pitch.update(raw_p);
            th_y = kf_yaw.predict_ahead(static_cast<float>(tb_lat_ms) / 1000.0f);
            th_p = kf_pitch.predict_ahead(static_cast<float>(tb_lat_ms) / 1000.0f);
            if (fresh && vis) gimbal.track(th_y, th_p);
            else if (fresh)   gimbal.stop_tracking();
        } else {
            kf_yaw.reset();
            kf_pitch.reset();
            selector.reset();
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
        //
        // 排布顺序按"现场最需要对照"来定:
        //   RECV  —— 下位机给了我们什么(姿态/mode/弹速/帧号/年龄)
        //   SENT  —— 我们发出去什么(含 18 字节原始报文, 协议出问题直接看这个)
        //   DIFF  —— 发与收的差, 这是判断"云台跟没跟上"的唯一直接指标
        //   KNOBS —— 可调项
        cv::Mat pnl(700, 460, CV_8UC3, cv::Scalar(26, 26, 26));
        int y = 28;
        char b[256];
        const cv::Scalar CY(200, 230, 120), GR(150, 240, 150), OR(120, 190, 255),
                         GY(190, 190, 190), RD(80, 80, 255), GN(80, 255, 80);

        head(pnl, y, "RECV  <- lower machine", GR);
        if (fb.valid) {
            std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(fb.yaw),
                          static_cast<double>(fb.pitch));
            row(pnl, y, "fb (now)", b, GR);
            std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(fb.roll));
            row(pnl, y, "roll", b, GR);
            // mode 必须显眼: mode=0 时下位机忽略一切角度指令, 那时"云台不动"
            // 是正确行为, 不是 bug。以前它只是灰字里的一行, 很容易看漏。
            row(pnl, y, "mode", std::to_string(fb.mode) +
                             (vis ? "  VISION ON" : "  ** OFF - enable via RC **"),
                vis ? GN : cv::Scalar(80, 80, 255));
            std::snprintf(b, sizeof(b), "%.2f m/s", static_cast<double>(fb.bullet_speed));
            row(pnl, y, "bullet_spd", b, GR);
            std::snprintf(b, sizeof(b), "%.1f ms%s", fb.age() * 1000.0,
                          fresh ? "" : "   STALE!");
            row(pnl, y, "age", b, fresh ? GR : RD);
        } else {
            row(pnl, y, "fb (now)", "NO DATA", RD);
        }

        y += 10;
        head(pnl, y, "SENT  -> lower machine", CY);
        std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(gimbal.cmd_yaw),
                      static_cast<double>(gimbal.cmd_pitch));
        row(pnl, y, "cmd (ID1)", b, CY);
        row(pnl, y, "fire (ID2)", std::to_string(fire), CY);
        std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(raw_y),
                      static_cast<double>(raw_p));
        row(pnl, y, "raw meas", b, GY);
        std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(th_y),
                      static_cast<double>(th_p));
        row(pnl, y, "after KF", b, GY);

        // ---- 交叉校验: 用像素位置直接算偏角, 和 PnP 的结果对照 ----
        //
        // 这两者**必须一致**。它们走的是完全不同的路径:
        //   pixel: 板的检测中心 + 内参, 一步 atan2
        //   PnP  : 四个角点 -> solvePnP -> tvec -> atan2
        // 若差很多, 说明 PnP 链路有问题(角点顺序/内参/相机系约定),
        // 那闭环追的就是一个虚构角度, 云台必然对不上板、上下乱跑。
        if (follow >= 0 && follow < static_cast<int>(cand.size())) {
            const float cxp = cand[follow].center.x;
            const float cyp = cand[follow].center.y;
            const float py_y = static_cast<float>(
                std::atan2(static_cast<double>(cxp - CAM_CX),
                           static_cast<double>(CAM_FX)) * 57.29578);
            const float py_p = static_cast<float>(
                std::atan2(static_cast<double>(cyp - CAM_CY),
                           static_cast<double>(CAM_FY)) * 57.29578);
            std::snprintf(b, sizeof(b), "%.2f / %.2f", static_cast<double>(py_y),
                          static_cast<double>(py_p));
            const float d_y = std::fabs(py_y - raw_y);
            const float d_p = std::fabs(py_p - raw_p);
            const bool bad = (d_y > 2.0f || d_p > 2.0f);
            row(pnl, y, "pixel th", b, bad ? cv::Scalar(80, 80, 255) : GY);
            std::snprintf(b, sizeof(b), "%.2f / %.2f  %s", static_cast<double>(d_y),
                          static_cast<double>(d_p),
                          bad ? "** MISMATCH - PnP suspect **" : "match");
            row(pnl, y, "pnp-pixel", b, bad ? cv::Scalar(80, 80, 255) : GN);
        }
        std::snprintf(b, sizeof(b), "%.2f m", static_cast<double>(dist));
        row(pnl, y, "dist", b, OR);
        {
            std::snprintf(b, sizeof(b), "%.1f / %.1f deg",
                          static_cast<double>(plate_angular_width_deg(dist)),
                          static_cast<double>(fire_tolerance_deg(dist)));
            row(pnl, y, "plate/tol", b, OR);
        }
        {   // 18 字节原始报文: 协议层出问题时直接看这个, 不用再加调试打印
            if (gimbal.have_meas) {
                const auto pkt = pack_tx(gimbal.cmd_yaw, gimbal.cmd_pitch, fire);
                std::string hx;
                char t[8];
                for (size_t i2 = 0; i2 < pkt.size(); ++i2) {
                    std::snprintf(t, sizeof(t), "%02X ", pkt[i2]);
                    hx += t;
                }
                row(pnl, y, "18B hex", hx, CY);
            }
        }

        y += 10;
        head(pnl, y, "DIFF  sent - recv", GY);
        if (fb.valid && gimbal.have_meas) {
            float dy = gimbal.cmd_yaw - fb.yaw;
            while (dy > 180.0f)  dy -= 360.0f;
            while (dy < -180.0f) dy += 360.0f;
            const float dp = gimbal.cmd_pitch - fb.pitch;
            std::snprintf(b, sizeof(b), "%+.2f / %+.2f", static_cast<double>(dy),
                          static_cast<double>(dp));
            row(pnl, y, "cmd - fb", b, GY);
            // 差很大说明云台没跟上命令(或 mode=0 被忽略)
            std::snprintf(b, sizeof(b), "%s", (std::fabs(dy) > SERVO_MAX_CORR_DEG ||
                                               std::fabs(dp) > SERVO_MAX_CORR_DEG)
                                                  ? "LARGE - not following" : "ok");
            row(pnl, y, "judge", b, (std::fabs(dy) > SERVO_MAX_CORR_DEG ||
                                     std::fabs(dp) > SERVO_MAX_CORR_DEG) ? RD : GN);
        }

        y += 10;
        head(pnl, y, "KNOBS", OR);
        std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(tb_conf) / 100.0);
        row(pnl, y, "conf", b, OR);
        row(pnl, y, "enemy", tb_enemy == 0 ? "blue" : (tb_enemy == 1 ? "red" : "all"), OR);
        std::snprintf(b, sizeof(b), "%.1f / %.1f", static_cast<double>(gimbal.aim_offset_yaw),
                      static_cast<double>(gimbal.aim_offset_pitch));
        row(pnl, y, "aim offset", b, OR);
        row(pnl, y, "latency", std::to_string(tb_lat_ms) + " ms", OR);
        std::snprintf(b, sizeof(b), "%.2f", static_cast<double>(gimbal.servo_gain));
        row(pnl, y, "gain", b, OR);
        std::snprintf(b, sizeof(b), "%.2f / %.1f deg", static_cast<double>(gimbal.smooth_coef),
                      static_cast<double>(gimbal.deadband));
        row(pnl, y, "smooth/dbz", b, OR);
        std::snprintf(b, sizeof(b), "%+.0f / %+.0f  (dirYaw/dirPitch)",
                      gimbal.servo_dir_yaw, gimbal.servo_dir_pitch);
        row(pnl, y, "servo dir", b, OR);
        std::snprintf(b, sizeof(b), "%.1f / %.1f", gimbal.effective_home_yaw(),
                      gimbal.effective_home_pitch());
        row(pnl, y, "home", b, GY);
        row(pnl, y, "detect", "yolo=" + std::to_string(boxes.size()) +
                              " solved=" + std::to_string(cand.size()) +
                              " " + tracker.state_name() +
                              (tr.locked ? " LOCKED" : ""), OR);

        cv::imshow(kW2, pnl);

        // knobs 窗口本身只放滑条(OpenCV 会自己画), 这里补一句提示
        {
            static cv::Mat hint(120, 420, CV_8UC3, cv::Scalar(26, 26, 26));
            static bool once = false;
            if (!once) {
                int hy = 24;
                cv::putText(hint, "KNOBS - drag to change live", cv::Point(12, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(120, 190, 255), 1);
                hy += 26;
                cv::putText(hint, "dirYaw/dirPitch: servo sign", cv::Point(12, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(190, 190, 190), 1);
                hy += 20;
                cv::putText(hint, "off yaw/pitch: aim offset", cv::Point(12, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(190, 190, 190), 1);
                hy += 20;
                cv::putText(hint, "latency: KF predict horizon", cv::Point(12, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(190, 190, 190), 1);
                hy += 20;
                cv::putText(hint, "gain: servo gain", cv::Point(12, hy),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(190, 190, 190), 1);
                cv::imshow(kW3, hint);
                once = true;
            }
        }

        const int k = cv::waitKey(1);
        if (k == 27 || k == 'q') break;
    }

    link.stop();
    if (use_cam) cam.close();
    return 0;
}

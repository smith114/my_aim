// detect_probe.cpp: 检测链路诊断工具
//
// 对一张图(或实相机当前帧)跑 灯条提取 -> 装甲板配对 -> PnP,
// 把中间结果全部打印出来, 并输出带标注的调试图。
// 用于回答"为什么这一帧没检出目标"。
//
// 用法:
//   ./detect_probe <图片|视频> [输出图] [--frames N] [-v]
//   ./detect_probe --cam [输出图] [-v]                 实相机抓一帧分析
//   ./detect_probe --cam --hold YAW PITCH [输出图] [-v] 先把云台转到该角再抓
//
// --hold 模式会持续发送该角度直到抓帧完成, 用于"云台停在这里看看画面里有什么"。
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "armor.hpp"
#include "config.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"
#include "solver.hpp"

using namespace srm;

static void apply_img_dir(cv::Mat& f) {
    switch (IMG_DIR) {
        case 1: cv::flip(f, f, 1); break;
        case 2: cv::flip(f, f, 0); break;
        case 3: cv::rotate(f, f, cv::ROTATE_180); break;
        default: break;
    }
}

int main(int argc, char** argv) {
    std::string input;
    std::string output = "/tmp/probe_out.jpg";
    int   frames   = 1;
    bool  verbose  = false;
    bool  use_cam  = false;
    bool  hold     = false;
    bool  hold_current = false;
    float hold_yaw = 0.0f, hold_pitch = 0.0f;
    std::string bayer_arg;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--cam") {
            use_cam = true;
        } else if (a == "--hold-current") {
            hold = true;
            hold_current = true;
        } else if (a == "--hold" && i + 2 < argc) {
            hold = true;
            hold_yaw   = std::atof(argv[++i]);
            hold_pitch = std::atof(argv[++i]);
        } else if (a == "--frames" && i + 1 < argc) {
            frames = std::atoi(argv[++i]);
        } else if (a == "--bayer" && i + 1 < argc) {
            bayer_arg = argv[++i];
        } else if (a == "-v") {
            verbose = true;
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法:\n"
                         "  detect_probe <图片|视频> [输出图] [--frames N] [-v]\n"
                         "  detect_probe --cam [输出图] [-v]\n"
                         "  detect_probe --cam --hold YAW PITCH [输出图] [-v]\n"
                         "  detect_probe --cam --hold-current [输出图] [-v]  原地抓帧, 不动云台\n";
            return 0;
        } else if (!a.empty() && a[0] != '-') {
            if (input.empty() && !use_cam) input = a;
            else                             output = a;
        }
    }

    cv::Mat frame;
    HikCamera cam;
    SerialLink link;

    if (use_cam) {
        // 可选: 先把云台转到指定角度再抓帧
        int fd = -1;
        if (hold) {
            fd = open_first_available({SERIAL_BY_ID, SERIAL_FALLBACK0, SERIAL_FALLBACK1});
            if (fd < 0) {
                std::cerr << "[ERR] --hold 需要下位机串口, 但打不开\n";
                return 1;
            }
            link.start(fd, SERIAL_SEND_HZ, SERIAL_WATCHDOG_MS, SERIAL_SHUTDOWN_ZERO_FRAMES);
            RxState st;
            for (int i = 0; i < 300; ++i) {
                st = link.state();
                if (st.valid) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (hold_current) {
                // 原地保持: 用当前实测角当命令, 云台不会动
                hold_yaw   = st.yaw;
                hold_pitch = st.pitch;
                std::printf("[HOLD-CURRENT] 保持 yaw=%.2f pitch=%.2f (mode=%d)\n",
                            hold_yaw, hold_pitch, st.mode);
            }
            for (int i = 0; i < 60; ++i) {
                link.set_target(hold_yaw, hold_pitch, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            st = link.state();
            std::printf("[HOLD] cmd=%.2f/%.2f  fb=%.2f/%.2f  mode=%d\n", hold_yaw, hold_pitch,
                        st.yaw, st.pitch, st.mode);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (!bayer_arg.empty()) cam.set_bayer(HikCamera::bayer_from_name(bayer_arg));
        if (!cam.open()) {
            std::cerr << "[ERR] 相机打不开\n";
            return 1;
        }
        for (int i = 0; i < 90; ++i) {
            if (cam.read(frame)) break;
        }
        if (frame.empty()) {
            std::cerr << "[ERR] 抓不到帧\n";
            return 1;
        }
    } else if (input.empty()) {
        std::cerr << "用法: detect_probe <图片|视频> [输出图] [--frames N] [-v]\n"
                     "      detect_probe --cam --hold YAW PITCH [输出图] [-v]\n";
        return 2;
    } else {
        const bool is_video =
            input.size() > 4 && (input.substr(input.size() - 4) == ".mp4" ||
                                 input.substr(input.size() - 4) == ".avi");
        if (is_video) {
            cv::VideoCapture cap(input);
            if (!cap.isOpened()) {
                std::cerr << "打不开视频\n";
                return 1;
            }
            for (int i = 0; i < frames; ++i) cap >> frame;
        } else {
            frame = cv::imread(input, cv::IMREAD_COLOR);
        }
    }
    if (frame.empty()) {
        std::cerr << "读不到输入\n";
        return 1;
    }
    if (!use_cam) apply_img_dir(frame);
    std::printf("图像 %dx%d, IMG_DIR=%d%s\n", frame.cols, frame.rows, IMG_DIR,
                use_cam ? " (实相机, bayer 已由相机模块处理)" : "");

    // ---------------------------------------------------------- 逐级过滤统计
    if (verbose) {
        const float scale = 1.0f;   // 灯条提取已改为全分辨率
        const cv::Mat& proc = frame;
        cv::Mat hsv;
        cv::cvtColor(proc, hsv, cv::COLOR_BGR2HSV);
        cv::Mat masks[2];
        cv::inRange(hsv, cv::Scalar(100, 60, 60), cv::Scalar(130, 255, 255), masks[0]);
        {
            cv::Mat r1, r2;
            cv::inRange(hsv, cv::Scalar(0, 60, 60), cv::Scalar(10, 255, 255), r1);
            cv::inRange(hsv, cv::Scalar(170, 60, 60), cv::Scalar(180, 255, 255), r2);
            cv::bitwise_or(r1, r2, masks[1]);
        }
        std::vector<cv::Mat> bgr;
        cv::split(proc, bgr);
        const cv::Mat kc = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        const cv::Mat ko = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        std::printf("\n=== 逐级过滤 (proc %dx%d, scale=%.2f) ===\n", proc.cols, proc.rows, scale);
        for (int c = 0; c < 2; ++c) {
            cv::Mat bin;
            cv::morphologyEx(masks[c], bin, cv::MORPH_CLOSE, kc);
            cv::morphologyEx(bin, bin, cv::MORPH_OPEN, ko);
            std::vector<std::vector<cv::Point>> cs;
            cv::findContours(bin, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::printf("  color=%s 形态学后轮廓 %zu 个\n", c == 0 ? "BLUE" : "RED", cs.size());
            for (size_t i = 0; i < cs.size() && i < 8; ++i) {
                const double area = cv::contourArea(cs[i]);
                const cv::RotatedRect rr = cv::minAreaRect(cs[i]);
                const float L = std::max(rr.size.width, rr.size.height);
                const float s = std::min(rr.size.width, rr.size.height);
                cv::Mat mask = cv::Mat::zeros(bin.size(), CV_8U);
                std::vector<std::vector<cv::Point>> one{cs[i]};
                cv::drawContours(mask, one, -1, cv::Scalar(255), cv::FILLED);
                const double mB = cv::mean(bgr[0], mask)[0];
                const double mG = cv::mean(bgr[1], mask)[0];
                const double mR = cv::mean(bgr[2], mask)[0];
                std::printf(
                    "    #%zu area=%.1f L=%.1f s=%.1f ratio=%.2f BGRmean=(%.0f,%.0f,%.0f) "
                    "short_ok=%d long_ok=%d ratio_ok=%d purity_ok=%d\n",
                    i, area, L, s, L / std::max(s, 1e-3f), mB, mG, mR,
                    (s >= LB_SHORT_MIN / scale), (L >= LB_LONG_MIN / scale),
                    (L / std::max(s, 1e-3f) >= LB_RATIO_MIN && L / std::max(s, 1e-3f) <= LB_RATIO_MAX),
                    (c == 0 ? (mB > mG + LB_PURITY && mB > mR + LB_PURITY)
                            : (mR > mG + LB_PURITY && mR > mB + LB_PURITY)));
            }
        }
    }

    // ---------------------------------------------------------- 灯条
    const auto bars = extract_lightbars(frame);
    std::printf("\n=== 灯条 %zu 个 ===\n", bars.size());
    std::printf("  idx  color  center(px)        len    wid   ratio   dir\n");
    for (size_t i = 0; i < bars.size(); ++i) {
        const auto& b = bars[i];
        std::printf("  %3zu  %5s  (%7.1f,%7.1f)  %6.1f %5.1f  %6.2f  %5.1f\n", i,
                    b.color == 0 ? "BLUE" : "RED", b.center.x, b.center.y, b.length, b.width,
                    b.length / std::max(b.width, 1e-3f), b.dir);
    }

    // ---------------------------------------------------------- 配对逐条约束
    if (verbose) {
        const cv::Mat vch = hsv_value_channel(frame);
        printf("\n=== 配对约束逐条检查 (前 12 根) ===\n");
        printf("  i  j  color dLen  dy/h  dx/h  dAng  aspect | 结果\n");
        const int n = static_cast<int>(bars.size());
        for (int i = 0; i < n && i < 12; ++i) {
            for (int j = i + 1; j < n && j < 12; ++j) {
                const auto& a = bars[i];
                const auto& b = bars[j];
                const float hmax = std::max(a.length, b.length);
                const float hmin = std::min(a.length, b.length);
                const float dlen = hmin / std::max(hmax, 1e-3f);
                const float dy = std::fabs(a.center.y - b.center.y) / hmax;
                const float dx = std::fabs(a.center.x - b.center.x) / hmax;
                float ad = std::fabs(a.dir - b.dir);
                if (ad > 90.0f) ad = 180.0f - ad;
                // 构造板的宽高比(与 match_armors 一致)
                const float dxp = std::fabs(a.center.x - b.center.x);
                const float aspect =
                    (dxp + std::max(a.width, b.width)) / std::max(hmax, 1e-3f);
                const char* why = "OK";
                if (a.color != b.color)                              why = "异色";
                else if (dlen < PAIR_LEN_RATIO_MIN)                  why = "长度比";
                else if (dy > PAIR_DY_RATIO_MAX)                     why = "y未对齐";
                else if (dx < PAIR_DIST_RATIO_MIN)                   why = "太近";
                else if (dx > PAIR_DIST_RATIO_MAX)                   why = "太远";
                else if (ad > PAIR_ANGLE_DIFF)                       why = "不平行";
                else if (aspect < PAIR_ASPECT_MIN ||
                         aspect > PAIR_ASPECT_MAX)                   why = "宽高比不当(疑似跨板)";
                else if (!vch.empty() &&
                         !armor_center_dark(vch, (a.center + b.center) * 0.5f,
                                            std::fabs(a.center.x - b.center.x) * 0.35f,
                                            hmax * 0.5f))               why = "中心不够暗";
                printf("  %2d %2d  %s  %5.2f %5.2f %5.2f %5.1f %6.2f | %s\n", i, j,
                       a.color == 0 ? "B" : "R", dlen, dy, dx, ad, aspect, why);
            }
        }
    }

    // ---------------------------------------------------------- 配对
    const cv::Mat v = hsv_value_channel(frame);
    const auto armors = match_armors(bars, v);
    std::printf("\n=== 装甲板配对 %zu 个 ===\n", armors.size());
    for (size_t i = 0; i < armors.size(); ++i) {
        const auto& a = armors[i];
        std::printf("  %3zu  color=%s center=(%7.1f,%7.1f)  w=%6.1f h=%6.1f\n", i,
                    a.color == 0 ? "BLUE" : "RED", a.center.x, a.center.y, a.width, a.height);
        for (int k = 0; k < 4; ++k)
            std::printf("        原始 pt%d=(%7.1f,%7.1f)\n", k, a.pts[k].x, a.pts[k].y);
        {
            const auto srt = sort_armor_points(a.pts);
            for (int k = 0; k < 4; ++k)
                std::printf("        排序 pt%d=(%7.1f,%7.1f)\n", k, srt[k].x, srt[k].y);
            std::printf("        is_valid_quad=%d\n", is_valid_quad(srt) ? 1 : 0);
            const PoseResult pr = solve_pose(srt);
            if (pr.ok)
                std::printf("        未过滤位姿 tvec=(%.1f, %.1f, %.1f)\n",
                            pr.tvec.at<double>(0), pr.tvec.at<double>(1), pr.tvec.at<double>(2));
        }
        const SolveResult r = solve_pnp(a.pts);
        if (r.ok)
            std::printf("        -> PnP dist=%.3f m  yaw=%.2f  pitch=%.2f\n", r.dist, r.yaw,
                        r.pitch);
        else
            std::printf("        -> PnP 失败\n");
    }

    // ---------------------------------------------------------- 标注输出
    draw_debug(frame, bars, armors);
    for (size_t i = 0; i < bars.size(); ++i) {
        char t[32];
        std::snprintf(t, sizeof(t), "b%zu", i);
        cv::putText(frame, t, bars[i].center + cv::Point2f(4, -4), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 1);
    }
    cv::imwrite(output, frame);
    std::printf("\n标注图已存: %s\n", output.c_str());
    link.stop();
    cam.close();
    return 0;
}

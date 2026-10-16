// cam_view.cpp: 实时成像 + 检测标注查看器
//
// 用途: 直观地看"相机看到了什么 + 检测器认为哪里是灯条/装甲板"。
// 排查"为什么没检出目标"时, 比读日志快得多。
//
// 用法:
//   ./cam_view                     窗口实时显示 (需要 DISPLAY)
//   ./cam_view --headless          不开窗口, 每 N 帧存一张标注图到 --out 目录
//   ./cam_view --hold-current      顺便保持云台当前角(防止它自己漂走)
//   ./cam_view --bayer rg|bg|gr|gb 指定 Bayer 相位
//   ./cam_view --raw               显示未旋转的原始帧(不套 IMG_DIR)
//
// 快捷键: q/Esc 退出, s 存图, T 切换 Bayer 相位, d 切换是否画灯条
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

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
    bool headless = false;
    bool hold_current = false;
    bool raw = false;
    bool show_bars = true;
    std::string bayer_arg;
    std::string out_dir = "diag";
    int snap_every = 30;
    int max_frames = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless")            headless = true;
        else if (a == "--hold-current")   hold_current = true;
        else if (a == "--raw")            raw = true;
        else if (a == "--bayer" && i + 1 < argc) bayer_arg = argv[++i];
        else if (a == "--out" && i + 1 < argc)   out_dir = argv[++i];
        else if (a == "--every" && i + 1 < argc) snap_every = std::atoi(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) max_frames = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::printf("用法: cam_view [--headless] [--hold-current] [--raw] [--bayer XX]\n"
                        "                [--out DIR] [--every N] [--frames N]\n");
            return 0;
        }
    }

    // ---------------------------------------------------------- 串口(可选)
    SerialLink link;
    bool have_serial = false;
    if (hold_current) {
        const int fd = open_first_available({SERIAL_BY_ID, SERIAL_FALLBACK0,
                                             SERIAL_FALLBACK1});
        if (fd >= 0) {
            link.start(fd, SERIAL_SEND_HZ, SERIAL_WATCHDOG_MS, SERIAL_SHUTDOWN_ZERO_FRAMES);
            have_serial = true;
            RxState st;
            for (int i = 0; i < 300; ++i) {
                st = link.state();
                if (st.valid) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (st.valid) {
                std::printf("[SERIAL] 保持 yaw=%.2f pitch=%.2f (mode=%d)\n", st.yaw, st.pitch,
                            st.mode);
                link.set_target(st.yaw, st.pitch, 0);
            } else {
                std::printf("[SERIAL] 收不到回传, 不保持姿态\n");
            }
        } else {
            std::printf("[SERIAL] 打不开串口, 跳过\n");
        }
    }

    // ---------------------------------------------------------- 相机
    HikCamera cam;
    if (!bayer_arg.empty()) cam.set_bayer(HikCamera::bayer_from_name(bayer_arg));
    if (!cam.open()) {
        std::fprintf(stderr, "相机打不开\n");
        if (have_serial) link.stop();
        return 1;
    }

    if (headless) ::mkdir(out_dir.c_str(), 0755);

    std::printf("\n快捷键: q/Esc 退出 | s 立刻存图 | T 换 Bayer 相位 | d 显示/隐藏灯条\n");
    std::printf("IMG_DIR=%d%s\n\n", IMG_DIR, raw ? " (--raw: 显示原始帧)" : "");

    cv::Mat frame, disp;
    int frame_cnt = 0;
    int saved = 0;

    while (true) {
        if (!cam.read(frame)) continue;
        if (!raw) apply_img_dir(frame);
        ++frame_cnt;

        // 检测
        const auto bars = extract_lightbars(frame);
        const auto armors = match_armors(bars, hsv_value_channel(frame));

        // 标注
        disp = frame.clone();
        if (show_bars) draw_debug(disp, bars, armors);
        for (size_t i = 0; i < armors.size(); ++i) {
            const SolveResult r = solve_pnp(armors[i].pts);
            if (!r.ok) continue;
            char t[96];
            std::snprintf(t, sizeof(t), "%.2fm y%.1f p%.1f", r.dist, r.yaw, r.pitch);
            cv::putText(disp, t, armors[i].center + cv::Point2f(8, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
        {
            const cv::Scalar mv = cv::mean(hsv_value_channel(frame));
            char hud[256];
            std::snprintf(hud, sizeof(hud),
                          "f=%d bars=%d armor=%d  V_mean=%.0f  bayer=%s  %s",
                          frame_cnt, static_cast<int>(bars.size()),
                          static_cast<int>(armors.size()), mv[0], cam.bayer_name(),
                          raw ? "RAW" : "IMG_DIR");
            cv::putText(disp, hud, cv::Point(12, 32), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        cv::Scalar(0, 255, 255), 2);
        }

        // 存图
        if (headless && (frame_cnt % snap_every == 0)) {
            char p[256];
            std::snprintf(p, sizeof(p), "%s/view_%04d.jpg", out_dir.c_str(), saved++);
            cv::imwrite(p, disp);
            std::printf("[SNAP] %s  bars=%d armor=%d\n", p, static_cast<int>(bars.size()),
                        static_cast<int>(armors.size()));
        }

        if (have_serial) {
            const RxState st = link.state();
            if (st.valid) link.set_target(st.yaw, st.pitch, 0);   // 原地保持
        }

        if (!headless) {
            cv::imshow("cam_view", disp);
            const int k = cv::waitKey(1);
            if (k == 'q' || k == 27) break;
            if (k == 's') {
                char p[256];
                std::snprintf(p, sizeof(p), "%s/manual_%04d.jpg", out_dir.c_str(), saved++);
                ::mkdir(out_dir.c_str(), 0755);
                cv::imwrite(p, disp);
                std::printf("[SAVE] %s\n", p);
            }
            if (k == 'T') cam.cycle_bayer();
            if (k == 'd') show_bars = !show_bars;
        }

        if (headless && frame_cnt % 60 == 0) {
            const cv::Scalar mv = cv::mean(hsv_value_channel(frame));
            std::printf("[RUN] f=%d bars=%d armor=%d V_mean=%.0f\n", frame_cnt,
                        static_cast<int>(bars.size()), static_cast<int>(armors.size()), mv[0]);
            std::fflush(stdout);
        }
        if (max_frames > 0 && frame_cnt >= max_frames) break;
    }

    if (have_serial) link.stop();
    cam.close();
    return 0;
}

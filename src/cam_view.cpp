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

#ifdef HAVE_OPENVINO
#include <memory>

#include "yolo_armor.hpp"
#include "yolo_detector.hpp"
#endif
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

#ifdef HAVE_OPENVINO
static std::unique_ptr<srm::YoloDetector> g_yolo;
#endif
static int g_want_color = -1;
static const bool g_want_color_used = (void(g_want_color), true);

// 两个分支都能取"检测到的框数", 供 HUD/日志使用
#ifdef HAVE_OPENVINO
#define BOXCOUNT static_cast<int>(boxes.size())
#else
#define BOXCOUNT boxes_size
#endif

int main(int argc, char** argv) {
    bool headless = false;
    bool hold_current = false;
    bool raw = false;
    bool show_bars = true;
    bool seek = false;
    float seek_span = 150.0f;    // yaw 搜索范围 ±span/2 度
    float seek_step = 5.0f;
    std::string bayer_arg;
    std::string out_dir = "diag";
    int snap_every = 30;
    int max_frames = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless")            headless = true;
        else if (a == "--hold-current")   hold_current = true;
        else if (a == "--raw")            raw = true;
        else if (a == "--seek")           { seek = true; headless = true; hold_current = true; }
        else if (a == "--span" && i + 1 < argc)      seek_span = std::atof(argv[++i]);
        else if (a == "--seek-step" && i + 1 < argc) seek_step = std::atof(argv[++i]);
        else if (a == "--bayer" && i + 1 < argc) bayer_arg = argv[++i];
        else if (a == "--out" && i + 1 < argc)   out_dir = argv[++i];
        else if (a == "--every" && i + 1 < argc) snap_every = std::atoi(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) max_frames = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::printf("用法: cam_view [--headless] [--hold-current] [--raw] [--bayer XX]\n"
                        "                [--out DIR] [--every N] [--frames N]\n"
                        "      cam_view --seek [--span DEG] [--seek-step DEG]\n"
                        "  --seek  自动横扫找装甲板, 找到就停在该角度并报出来(全 ASCII 输出)\n");
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
    // 初始化 YOLO（与 my_aim 同一套检测器，含布局校验）。
    // 加载失败直接退出：宁可明确报错，也不要让人以为在看 YOLO 结果而实际没有。
    #ifdef HAVE_OPENVINO
    try {
        g_yolo = std::make_unique<srm::YoloDetector>(
            srm::make_yolo_profile(YOLO_PROFILE_SELECT),
            srm::YOLO_CONF_THRESH, srm::YOLO_NMS_THRESH);
    } catch (const std::exception& e) {
        std::cerr << "[YOLO][ERR] 模型加载失败: " << e.what() << "\n";
        return 1;
    }
    #else
    std::cerr << "[YOLO][ERR] 本 build 未编入 OpenVINO，无法检测\n";
    return 1;
    #endif
    HikCamera cam;
    if (!bayer_arg.empty()) cam.set_bayer(HikCamera::bayer_from_name(bayer_arg));
    if (!cam.open()) {
        std::fprintf(stderr, "相机打不开\n");
        if (have_serial) link.stop();
        return 1;
    }

    if (headless) ::mkdir(out_dir.c_str(), 0755);

    // ---------------------------------------------------------- --seek
    // 自动横扫找装甲板: 以当前位置为中心, 在 yaw 上左右扫, 每个角度抓几帧跑检测,
    // 一找到配对的装甲板就停在那儿并报出角度。
    // 输出全部是 ASCII, 方便在 PowerShell/SSH 里解析(中文会被控制台编码搞乱)。
    if (seek) {
        if (!have_serial) {
            std::fprintf(stderr, "[SEEK] 需要串口来控制云台\n");
            cam.close();
            return 1;
        }
        RxState st = link.state();
        const float y0 = st.yaw, p0 = st.pitch;
        std::printf("[SEEK] start yaw=%.1f pitch=%.1f mode=%d span=+/-%.0f step=%.1f\n",
                    y0, p0, st.mode, seek_span * 0.5f, seek_step);
        if (st.mode != 1) {
            std::printf("[SEEK][WARN] mode=%d != 1, lower machine will ignore us; "
                        "press right mouse button to enter vision control\n", st.mode);
        }

        const int   half_n = static_cast<int>(seek_span * 0.5f / seek_step);
        int         best_bars = 0, best_armor = 0;
        float       best_y = y0, best_p = p0;
        bool        found = false;

        // 也扫 pitch: 只扫 yaw 时如果靶标在别的仰角就永远找不到。
        // 顺序: 先在该 pitch 上由内向外扫 yaw, 再换下一个 pitch。
        const float pitch_list[] = {p0, p0 + 10.0f, p0 - 10.0f, p0 + 20.0f, p0 - 20.0f,
                                    p0 + 30.0f, p0 - 30.0f};
        for (const float ptest : pitch_list) {
            if (found) break;
            std::printf("[SEEK] ---- pitch=%.1f ----\n", ptest);
            for (int k = 0; k <= half_n && !found; ++k) {
                for (int sgn = 0; sgn < 2 && !found; ++sgn) {
                    if (k == 0 && sgn == 1) continue;
                    const float off =
                        (k == 0) ? 0.0f : (sgn == 0 ? +1.0f : -1.0f) * k * seek_step;
                    const float yaw = y0 + off;
                    for (int t = 0; t < 12; ++t) {
                        link.set_target(yaw, ptest, 0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(25));
                    }
                    int bars_n = 0, armor_n = 0;
                    float vmean = 0.0f;
                    for (int f = 0; f < 4; ++f) {
                        cv::Mat img;
                        if (!cam.read(img)) continue;
                        if (!raw) apply_img_dir(img);
                        const cv::Mat vch = hsv_value_channel(img);
                        vmean = static_cast<float>(cv::mean(vch)[0]);
                        const auto bs = extract_lightbars(img);
                        const auto as = match_armors(bs, vch);
                        if (static_cast<int>(as.size()) > armor_n) {
                            armor_n = static_cast<int>(as.size());
                            bars_n = static_cast<int>(bs.size());
                        } else if (static_cast<int>(bs.size()) > bars_n) {
                            bars_n = static_cast<int>(bs.size());
                        }
                    }
                    std::printf("[SEEK] pitch=%7.1f off=%+7.1f yaw=%8.2f yolo=%d armor=%d "
                                "V=%.0f\n",
                                ptest, off, yaw, bars_n, armor_n, vmean);
                    std::fflush(stdout);

                    if (armor_n > best_armor ||
                        (armor_n == best_armor && bars_n > best_bars)) {
                        best_armor = armor_n;
                        best_bars = bars_n;
                        best_y = yaw;
                        best_p = ptest;
                    }
                    if (armor_n >= 1) {
                        found = true;
                        std::printf("[SEEK][FOUND] armor=%d bars=%d at yaw=%.2f pitch=%.2f\n",
                                    armor_n, bars_n, yaw, ptest);
                    }
                }
            }
        }

        // 停到最佳位置
        for (int t = 0; t < 20; ++t) {
            link.set_target(best_y, best_p, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        std::printf("[SEEK][DONE] found=%d best armor=%d bars=%d yaw=%.2f pitch=%.2f\n",
                    found ? 1 : 0, best_armor, best_bars, best_y, best_p);
        // 存一张找到位置的标注图
        {
            cv::Mat img;
            if (cam.read(img)) {
                if (!raw) apply_img_dir(img);
                const auto bs = extract_lightbars(img);
                const auto as = match_armors(bs, hsv_value_channel(img));
                cv::Mat d = img.clone();
                draw_debug(d, bs, as);
                const std::string p = out_dir + "/seek_found.jpg";
                cv::imwrite(p, d);
                std::printf("[SEEK] saved %s\n", p.c_str());
            }
        }
        link.stop();
        cam.close();
        return 0;
    }

    std::printf("\n快捷键: q/Esc 退出 | s 立刻存图 | T 换 Bayer 相位 | d 显示/隐藏灯条\n");
    std::printf("IMG_DIR=%d%s\n\n", IMG_DIR, raw ? " (--raw: 显示原始帧)" : "");

    cv::Mat frame, disp;
    int frame_cnt = 0;
    int saved = 0;

    while (true) {
        if (!cam.read(frame)) continue;
        if (!raw) apply_img_dir(frame);
        ++frame_cnt;

        // 检测: 用 **YOLO**, 与 my_aim 完全一致。
        // 之前这里还是灯条法, 结果"窗口里看到的"和"主程序实际跑的"不是一套检测,
        // 看着能检出但跑 my_aim 没反应, 或者反过来 —— 这种不一致非常误导。
        std::vector<LightBar> bars;
        std::vector<Armor>    armors;
#ifdef HAVE_OPENVINO
        // boxes 只在编入 OpenVINO 时存在; 没编入时程序已经在启动阶段退出,
        // 这里只是为了让 else 分支也能通过编译。
        std::vector<YoloBox> boxes;
        if (g_yolo) {
            boxes  = g_yolo->detect(frame);
            armors = filter_by_color(yolo_to_armors(boxes), g_want_color);
        }
#else
        const int boxes_size = 0;   // 占位, 仅供下面日志宏取用
#endif

        // 标注
        disp = frame.clone();
        // 灯条框已不参与检测, 但 YOLO 的框仍然画出来(带颜色/分数),
        // 这样一眼能看出模型判成蓝还是红、置信度多少。
        if (show_bars) {
            draw_debug(disp, bars, armors);
#ifdef HAVE_OPENVINO
            for (const auto& b : boxes) {
                const cv::Scalar col = yolo_color_of(b) == 0 ? cv::Scalar(255, 0, 0)
                                                             : cv::Scalar(0, 0, 255);
                cv::rectangle(disp, b.rect, col, 2);
                char t[96];
                std::snprintf(t, sizeof(t), "%s %.2f",
                              b.name.empty() ? "?" : b.name.c_str(),
                              static_cast<double>(b.score));
                cv::putText(disp, t, b.rect.tl() + cv::Point(0, -6),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
                for (const auto& c : b.corners)
                    cv::circle(disp, c, 4, cv::Scalar(0, 0, 255), -1);
            }
#endif
        }
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
                          "f=%d yolo=%d armor=%d  V_mean=%.0f  bayer=%s  %s",
                          frame_cnt, BOXCOUNT,
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
            std::printf("[SNAP] %s  yolo=%d armor=%d\n", p, BOXCOUNT, static_cast<int>(armors.size()));
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
            std::printf("[RUN] f=%d yolo=%d armor=%d V_mean=%.0f\n", frame_cnt,
                        BOXCOUNT, static_cast<int>(armors.size()), mv[0]);
            std::fflush(stdout);
        }
        if (max_frames > 0 && frame_cnt >= max_frames) break;
    }

    if (have_serial) link.stop();
    cam.close();
    return 0;
}

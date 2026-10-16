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
#include "tracker.hpp"

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
int main(int argc, char** argv) {
    std::string input_path;
    std::string snapshot_dir = "/tmp/myaim_shots";
    std::string log_path = "/tmp/myaim.log";
    bool show_debug  = false;
    bool headless    = false;
    bool force_mock  = false;
    bool save_video  = false;
    bool dir_autocheck = true;   // 默认开启 SERVO_DIR 自检
    int  max_frames  = 0;
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
        } else if (a == "--no-dir-check") {
            dir_autocheck = false;
        } else if (a == "--bayer" && i + 1 < argc) {
            bayer_name_arg = argv[++i];
        } else if (a == "--snapshot-dir" && i + 1 < argc) {
            snapshot_dir = argv[++i];
        } else if (a == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (a == "--frames" && i + 1 < argc) {
            max_frames = std::atoi(argv[++i]);
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法: my_aim [--debug] [--headless] [--mock] [--save-video]\n"
                         "              [--no-dir-check] [--bayer rg|bg|gr|gb]\n"
                         "              [--snapshot-dir DIR] [--log FILE]\n"
                         "              [--frames N] [video|image]\n"
                         "  --no-dir-check  关闭云台 SERVO_DIR 自动纠正(默认开启)\n"
                         "  --bayer         指定相机 Bayer 相位(默认见 config.hpp)\n";
            return 0;
        } else {
            input_path = a;
        }
    }

    std::signal(SIGINT, on_signal);
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

    if (!input_path.empty()) {
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
    float  last_bullet_speed = BULLET_SPEED_DEFAULT;

    while (g_stop == 0) {
        // ------------------------------------------------ 取帧
        if (use_hik) {
            if (!hik.read(frame)) continue;
        } else {
            file_cap >> frame;
            if (frame.empty()) break;
        }
        apply_img_dir(frame);
        tracker.set_frame_size(frame.size());
        ++frame_cnt;

        // ------------------------------------------------ 检测
        const auto bars = extract_lightbars(frame);
        cv::Mat hsv, hsv_ch[3];
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, hsv_ch);
        const auto armors = match_armors(bars, hsv_ch[2]);

        stat_bars  += static_cast<int>(bars.size());
        stat_armor += static_cast<int>(armors.size());

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
        if (tr.locked && tr.index >= 0) {
            ++stat_locked;
            last_locked_frame = frame_cnt;
        }

        // ------------------------------------------------ 云台 -> 串口
        float theta_yaw = 0.0f, theta_pitch = 0.0f;
        if (tr.locked) {
            float pitch_comp = 0.0f;
            if (BALLISTIC_COMP_ENABLE) {
                const RxState s = link.state();
                const float v = s.has_bullet_speed ? s.bullet_speed : BULLET_SPEED_DEFAULT;
                pitch_comp = ballistic_pitch_deg(tr.target.solve.dist, v);
            }
            theta_yaw   = tr.target.solve.yaw;
            theta_pitch = tr.target.solve.pitch + pitch_comp;

            // 回传数据必须新鲜, 否则冻结命令(下位机侧已有看门狗兜底)
            if (const RxState s = link.state(); s.valid && s.age() < FEEDBACK_MAX_AGE_S) {
                gimbal.update_feedback(s.yaw, s.pitch);
                gimbal.track(theta_yaw, theta_pitch);
                if (dir_autocheck) {
                    // 自检: 若按当前方向转完偏差反而变大, 说明 SERVO_DIR 反了
                    static float prev_theta = 0.0f;
                    static float prev_cmd = 90.0f;
                    if (gimbal.auto_dir_check(prev_theta, theta_yaw, gimbal.cmd_yaw - prev_cmd)) {
                        LOG("[DIR] SERVO_DIR 反了, 已自动翻转 -> %.0f\n", gimbal.servo_dir);
                    }
                    prev_theta = theta_yaw;
                    prev_cmd   = gimbal.cmd_yaw;
                }
                last_locked_frame = frame_cnt;
            } else {
                LOG("[WARN] 回传数据过期(%.3fs), 云台冻结\n", s.age());
            }
        } else {
            gimbal.stop_tracking();
            // 长时间没目标: 摆扫找回; 刚丢失则原地保持(继续发同一个角, 云台不动)
            if (frame_cnt - last_locked_frame > GIMBAL_SCAN_HOLD) {
                if (const RxState s = link.state(); s.valid && s.age() < FEEDBACK_MAX_AGE_S) {
                    gimbal.update_feedback(s.yaw, s.pitch);
                }
                gimbal.scan();
            }
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
            const RxState s = link.state();
            char buf[320];
            std::snprintf(buf, sizeof(buf),
                          "%s lock=%d cand=%d dist=%.2f th=%.1f/%.1f | cmd %.1f/%.1f"
                          " fb %.1f/%.1f v=%.1f dir=%.0f",
                          tracker.state_name(), tr.locked ? 1 : 0,
                          static_cast<int>(candidates.size()),
                          tr.index >= 0 ? tr.target.solve.dist : 0.0f,
                          theta_yaw, theta_pitch,
                          gimbal.cmd_yaw, gimbal.cmd_pitch,
                          s.valid ? s.yaw : 0.0f, s.valid ? s.pitch : 0.0f, last_bullet_speed,
                          gimbal.servo_dir);
            cv::putText(disp, buf, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }
        if (save_video && vid_writer.isOpened() && !disp.empty()) {
            vid_writer.write(disp);
        }

        if (frame_cnt % 30 == 0) {
            const RxState s = link.state();
            LOG("[DET] f=%d bars=%.1f armor=%.1f solved=%.1f locked=%.1f state=%s "
                "cmd=%.1f/%.1f fb=%.1f/%.1f home=%.1f/%.1f mode=%d th=%.1f/%.1f dir=%.0f "
                "rx=%llu resync=%llu wd=%llu\n",
                frame_cnt, stat_bars / 30.0, stat_armor / 30.0, stat_solved / 30.0,
                stat_locked / 30.0, tracker.state_name(), gimbal.cmd_yaw, gimbal.cmd_pitch,
                s.valid ? s.yaw : 0.0f, s.valid ? s.pitch : 0.0f,
                gimbal.home_yaw, gimbal.home_pitch,
                s.valid ? s.mode : -1,
                theta_yaw, theta_pitch, gimbal.servo_dir,
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

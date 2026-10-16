// calibrate.cpp: 相机内参标定 (海康 MVS SDK 取流)
//
// 两种用法:
//   1) 交互模式(带显示器): ./calibrate
//        A/D 左右转云台  W/S 上下转   每次 0.5 度
//        G   切换棋盘格规格(内角点数)
//        B   切换二值化预览(判断画面亮度/对比度)
//        空格 对准棋盘格后拍照, 拍够 20 张自动计算
//        Esc 退出
//   2) headless 模式(SSH 远程): ./calibrate --headless --shots 25 --out /tmp/calib
//        先转动棋盘格/云台拍够 N 张不同角度的图, 再拷回本地跑计算:
//        ./calibrate --compute /tmp/calib --cols 11 --rows 8
//
// 标定结果直接打印成可以粘到 config.hpp 的常量。
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>

#include "config.hpp"
#include "hik_camera.hpp"
#include "protocol.hpp"

using namespace srm;

static void apply_img_dir(cv::Mat& f) {
    switch (IMG_DIR) {
        case 1: cv::flip(f, f, 1); break;
        case 2: cv::flip(f, f, 0); break;
        case 3: cv::rotate(f, f, cv::ROTATE_180); break;
        default: break;
    }
}

static bool dir_exists(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// 棋盘格规格(内角点数), 现场按 G 切换
static const std::vector<cv::Size> kBoards = {
    cv::Size(11, 8),   // 12x9 格
    cv::Size(12, 9),   // 13x10 格
    cv::Size(9, 6),    // 10x7 格
    cv::Size(8, 6),    // 9x7 格
    cv::Size(10, 7),   // 11x8 格
};

// ---------------------------------------------------------------- 标定计算
static int compute_calibration(const std::string& dir, cv::Size board, float square_mm) {
    std::vector<cv::String> files;
    cv::glob(dir + "/*.jpg", files, false);
    cv::glob(dir + "/*.png", files, false);
    std::sort(files.begin(), files.end());
    if (files.size() < 5) {
        std::cerr << "样本太少(" << files.size() << "), 至少 5 张\n";
        return 1;
    }

    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board.height; ++i)
        for (int j = 0; j < board.width; ++j)
            obj.emplace_back(j * square_mm, i * square_mm, 0.0f);

    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Size img_size;

    for (const auto& f : files) {
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        if (img.empty()) continue;
        apply_img_dir(img);
        if (img_size.empty()) img_size = img.size();
        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            img, board, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) {
            std::cout << "  [skip] " << f << " 未找到角点\n";
            continue;
        }
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                          30, 0.001));
        obj_points.push_back(obj);
        img_points.push_back(corners);
    }
    if (obj_points.size() < 5) {
        std::cerr << "有效样本太少(" << obj_points.size() << ")\n";
        return 1;
    }

    cv::Mat K, dist, rvecs, tvecs;
    const double err = cv::calibrateCamera(obj_points, img_points, img_size, K, dist, rvecs, tvecs);
    std::printf("\n=== 标定结果 (%zu 张有效样本, 图像 %dx%d) ===\n", obj_points.size(),
                img_size.width, img_size.height);
    std::printf("constexpr float CAM_FX = %.2ff;\n", K.at<double>(0, 0));
    std::printf("constexpr float CAM_FY = %.2ff;\n", K.at<double>(1, 1));
    std::printf("constexpr float CAM_CX = %.2ff;\n", K.at<double>(0, 2));
    std::printf("constexpr float CAM_CY = %.2ff;\n", K.at<double>(1, 2));
    std::printf("constexpr float CAM_K1 = %.10ff;\n", dist.at<double>(0));
    std::printf("constexpr float CAM_K2 = %.10ff;\n", dist.at<double>(1));
    std::printf("constexpr float CAM_P1 = %.10ff;\n", dist.at<double>(2));
    std::printf("constexpr float CAM_P2 = %.10ff;\n", dist.at<double>(3));
    std::printf("constexpr float CAM_K3 = %.10ff;\n", dist.at<double>(4));
    std::printf("重投影误差: %.4f px (应 < 1.0, 越小越好)\n", err);
    return 0;
}

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
    std::string out_dir = "/tmp/calib";
    std::string serial_dev;
    int  shots = 20;
    bool headless_grab = false;
    cv::Size board = kBoards[0];

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") {
            headless_grab = true;
        } else if (a == "--shots" && i + 1 < argc) {
            shots = std::atoi(argv[++i]);
        } else if (a == "--out" && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (a == "--serial" && i + 1 < argc) {
            serial_dev = argv[++i];
        } else if (a == "--cols" && i + 1 < argc) {
            board.width = std::atoi(argv[++i]);
        } else if (a == "--rows" && i + 1 < argc) {
            board.height = std::atoi(argv[++i]);
        } else if (a == "--compute" && i + 1 < argc) {
            const std::string d = argv[++i];
            return compute_calibration(d, board, CALIB_SQUARE_MM);
        } else if (a == "-h" || a == "--help") {
            std::cout << "用法:\n"
                         "  calibrate                                    交互模式\n"
                         "  calibrate --headless --shots 25 --out DIR    无显示器采集\n"
                         "  calibrate --compute DIR --cols 11 --rows 8   离线计算\n";
            return 0;
        }
    }

    HikCamera hik;
    if (!hik.open()) {
        std::cerr << "打不开相机\n";
        return 1;
    }

    // 云台串口(可选): 标定时靠键盘把棋盘格转进画面
    int fd = -1;
    if (!serial_dev.empty()) {
        fd = open_port(serial_dev);
    } else {
        fd = open_first_available({SERIAL_BY_ID, SERIAL_FALLBACK0, SERIAL_FALLBACK1});
    }
    std::cout << (fd >= 0 ? "[SERIAL] 已连接, 可用键盘转云台\n"
                          : "[SERIAL] 未找到串口, 只能手动搬棋盘格\n");

    float ty = 90.0f, tp = 90.0f;   // 云台角估计(协议 0~180)

    // ---------------------------------------------------------- headless 采集
    if (headless_grab) {
        if (!dir_exists(out_dir)) ::mkdir(out_dir.c_str(), 0755);
        int saved = 0;
        std::cout << "采集 " << shots << " 张到 " << out_dir
                  << " (每张之间请改变棋盘格姿态/距离/角度)\n";
        while (saved < shots) {
            cv::Mat frame;
            if (!hik.read(frame)) continue;
            apply_img_dir(frame);
            const std::string path =
                out_dir + "/calib_" + std::to_string(saved) + ".jpg";
            cv::imwrite(path, frame);
            std::cout << "[SAVE] " << path << "\n";
            ++saved;
            sleep(1);
        }
        if (fd >= 0) close(fd);
        hik.close();
        std::cout << "采集完成, 现在跑: " << argv[0] << " --compute " << out_dir
                  << " --cols " << board.width << " --rows " << board.height << "\n";
        return 0;
    }

    // ---------------------------------------------------------- 交互模式
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Size img_size;
    size_t bi = 0;
    bool binary_preview = false;
    bool last_found = false;
    int  frame_cnt = 0;

    std::cout << "WASD 转云台对准棋盘格, G 切规格, B 二值预览, 空格拍照, Esc 退出\n";
    while (static_cast<int>(img_points.size()) < shots) {
        cv::Mat frame;
        if (!hik.read(frame)) continue;
        apply_img_dir(frame);
        if (img_size.empty()) img_size = frame.size();
        ++frame_cnt;

        bool found = last_found;
        if (frame_cnt % 3 == 0) {   // 检测降频: 每 3 帧一次, 全分辨率保精度
            found = cv::findChessboardCorners(
                frame, board, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
            if (found) {
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                                 cv::TermCriteria(cv::TermCriteria::EPS |
                                                      cv::TermCriteria::COUNT,
                                                  30, 0.001));
            }
            last_found = found;
        }
        if (found) cv::drawChessboardCorners(frame, board, corners, true);

        cv::Mat disp = frame;
        if (binary_preview) {
            cv::Mat g, b;
            cv::cvtColor(frame, g, cv::COLOR_BGR2GRAY);
            cv::threshold(g, b, 128, 255, cv::THRESH_BINARY);
            disp = b;
        }
        if (disp.cols > 960) cv::resize(disp, disp, cv::Size(960, disp.rows * 960 / disp.cols));

        cv::putText(disp, "captured: " + std::to_string(img_points.size()) + "/" +
                              std::to_string(shots),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        char hud[200];
        std::snprintf(hud, sizeof(hud),
                      "board %dx%d  gimbal %.1f/%.1f  found=%d  [G B WASD space]",
                      board.width, board.height, ty, tp, found ? 1 : 0);
        cv::putText(disp, hud, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 2);
        cv::imshow("calibrate", disp);

        const int k = cv::waitKey(30);
        if (k == 27) break;
        if (k == 'a' || k == 'A') ty -= 0.5f;
        if (k == 'd' || k == 'D') ty += 0.5f;
        if (k == 'w' || k == 'W') tp += 0.5f;
        if (k == 's' || k == 'S') tp -= 0.5f;
        if (k == 'a' || k == 'd' || k == 'A' || k == 'D' || k == 'w' || k == 's' ||
            k == 'W' || k == 'S') {
            if (fd >= 0) {
                const auto pkt = pack_tx(ty, tp, 0);
                const ssize_t n = write(fd, pkt.data(), pkt.size());
                (void)n;
            }
            std::cout << "gimbal -> yaw=" << ty << " pitch=" << tp << "\n";
        }
        if (k == 'g' || k == 'G') {
            bi = (bi + 1) % kBoards.size();
            board = kBoards[bi];
            img_points.clear();
            std::cout << "[board] 切换到 " << board.width << "x" << board.height
                      << " (内角点数), 已拍样本清空\n";
        }
        if (k == 'b' || k == 'B') {
            binary_preview = !binary_preview;
            std::cout << "[binary] " << (binary_preview ? "on" : "off") << "\n";
        }
        if (k == ' ' && found) {
            if (!dir_exists(out_dir)) ::mkdir(out_dir.c_str(), 0755);
            const std::string path = out_dir + "/calib_" +
                                     std::to_string(img_points.size()) + ".jpg";
            cv::imwrite(path, frame);
            img_points.push_back(corners);
            std::cout << "captured " << img_points.size() << " -> " << path << "\n";
        }
    }
    if (fd >= 0) close(fd);
    hik.close();

    if (img_points.empty()) {
        std::cerr << "没有拍到样本\n";
        return 1;
    }
    return compute_calibration(out_dir, board, CALIB_SQUARE_MM);
}

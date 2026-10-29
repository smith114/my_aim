// box_probe.cpp: 用**我们自己的 YOLO 检测器**跑图片/视频, 打印检测框几何
//
// 为什么需要: 要验证 ARMOR_H 是否等于灯条高度(55mm), 唯一可靠的判据是
//   "检测框的宽高比" 是否等于 ARMOR_H / ARMOR_W。
//   正对相机时, 140x55 的四边形在图像里应呈 55/140 = 0.393 的宽高比。
//   这个比值与相机内参、距离都无关, 所以用手机拍的视频也能验证。
//
// 用法:
//   ./box_probe --img frames/p01.jpg
//   ./box_probe --video VID.mp4 --stride 10     每 10 帧取一帧
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "yolo_armor.hpp"
#include "yolo_detector.hpp"

using namespace srm;

int main(int argc, char** argv) {
    std::string img, video, model = "v8";
    int stride = 10;
    float conf = 0.35f;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--img") && i + 1 < argc)        img = argv[++i];
        else if (!std::strcmp(argv[i], "--video") && i + 1 < argc) video = argv[++i];
        else if (!std::strcmp(argv[i], "--model") && i + 1 < argc) model = argv[++i];
        else if (!std::strcmp(argv[i], "--stride") && i + 1 < argc) stride = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--conf") && i + 1 < argc)  conf = static_cast<float>(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            std::printf("用法: box_probe --img FILE | --video FILE [--stride N] [--conf C]\n"
                        "  打印检测框宽高比, 用于核对 ARMOR_H / ARMOR_W\n");
            return 0;
        }
    }
    if (img.empty() && video.empty()) { std::fprintf(stderr, "需要 --img 或 --video\n"); return 1; }

    const int sel = (model == "v11") ? YOLO_PROFILE_YOLO11_38CLASS
                  : (model == "v5")  ? YOLO_PROFILE_YOLOV5_10CLASS
                                     : YOLO_PROFILE_YOLOV8_2CLASS;
    std::unique_ptr<YoloDetector> yolo;
    try {
        yolo = std::make_unique<YoloDetector>(make_yolo_profile(sel), conf, YOLO_NMS_THRESH);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ERR] 模型加载失败: %s\n", e.what());
        return 1;
    }

    const float target_ratio = ARMOR_H / ARMOR_W;   // 期望的宽高比
    std::printf("期望宽高比 ARMOR_H/ARMOR_W = %.0f/%.0f = %.4f\n\n",
                static_cast<double>(ARMOR_H), static_cast<double>(ARMOR_W),
                static_cast<double>(target_ratio));

    std::vector<double> ratios;
    auto handle = [&](const cv::Mat& frame, int idx) {
        if (frame.empty()) return;
        cv::Mat f = frame;
        if (IMG_DIR == 1) cv::flip(f, f, 1);
        else if (IMG_DIR == 2) cv::flip(f, f, 0);
        else if (IMG_DIR == 3) cv::flip(f, f, -1);
        const auto boxes = yolo->detect(f);
        if (boxes.empty()) return;
        const auto armors = yolo_to_armors(boxes);
        for (size_t k = 0; k < armors.size(); ++k) {
            const auto& P = armors[k].pts;
            if (P.size() != 4) continue;
            float mnx = P[0].x, mxx = P[0].x, mny = P[0].y, mxy = P[0].y;
            for (const auto& q : P) {
                mnx = std::fmin(mnx, q.x); mxx = std::fmax(mxx, q.x);
                mny = std::fmin(mny, q.y); mxy = std::fmax(mxy, q.y);
            }
            const float bw = mxx - mnx, bh = mxy - mny;
            if (bw < 4.0f) continue;
            const double r = bh / bw;
            ratios.push_back(r);
            std::printf("  #%-4d 板%d  box %4.0f x %3.0f px   宽高比 %.4f  (期望 %.4f, 偏差 %+.1f%%)\n",
                        idx, static_cast<int>(k), static_cast<double>(bw), static_cast<double>(bh),
                        r, static_cast<double>(target_ratio),
                        (r - target_ratio) / target_ratio * 100.0);
            for (size_t j = 0; j < 4; ++j)
                std::printf("            kp%zu (%.0f, %.0f)\n", j,
                            static_cast<double>(P[j].x), static_cast<double>(P[j].y));
        }
    };

    if (!img.empty()) {
        cv::Mat m = cv::imread(img);
        if (m.empty()) { std::fprintf(stderr, "[ERR] 读不到 %s\n", img.c_str()); return 1; }
        handle(m, 0);
    } else {
        cv::VideoCapture cap(video);
        if (!cap.isOpened()) { std::fprintf(stderr, "[ERR] 打不开 %s\n", video.c_str()); return 1; }
        cv::Mat m;
        int idx = 0, used = 0;
        while (cap.read(m)) {
            if (idx % stride == 0) { handle(m, idx); ++used; }
            ++idx;
        }
        std::printf("\n共 %d 帧, 抽样 %d 帧\n", idx, used);
    }

    if (ratios.empty()) {
        std::printf("\n没检到任何板 —— 可能视频里太远/太暗, 或 conf 偏高(--conf 0.2 试试)\n");
        return 0;
    }
    double sum = 0.0;
    for (double r : ratios) sum += r;
    const double mean = sum / static_cast<double>(ratios.size());
    std::printf("\n================ 统计 (%d 个检测) ================\n", static_cast<int>(ratios.size()));
    std::printf("  实测量到的宽高比平均 = %.4f\n", mean);
    std::printf("  ARMOR_H/ARMOR_W 期望 = %.4f\n", static_cast<double>(target_ratio));
    std::printf("  偏差 = %+.1f%%\n", (mean - target_ratio) / target_ratio * 100.0);
    std::printf("  以 W=140 反推 H = %.1f mm\n", 140.0 * mean);
    std::printf("==================================================\n");
    return 0;
}

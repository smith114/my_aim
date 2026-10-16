// synth_dump.cpp: 生成合成靶标图用于离线诊断 (配合 detect_probe 使用)
//
// 用法: ./synth_dump /tmp/synth --dist 1.5,3,5 --bar-w 0.16 --gap 1.0
// 每张图旁边写一个同名 .txt 记录真值(距离/角度), 方便和 detect_probe 的输出对照。
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include "synth.hpp"

using namespace srm;
using namespace srm::test;

// 更接近真实装甲板的合成器: 灯条宽度/高度/间距都可调
static cv::Mat make_armor_parametric(float x_mm, float y_mm, float z_mm, float bar_w_ratio,
                                     float bar_h_ratio, float gap_ratio, int color) {
    const int W = 1440, H = 1080;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(18, 18, 18));

    const float hw = ARMOR_W * 0.5f;
    const float hh = ARMOR_H * 0.5f;
    const float bar_w = ARMOR_W * bar_w_ratio;
    const float bar_h = ARMOR_H * bar_h_ratio;
    const float bar_x = hw * gap_ratio - bar_w * 0.5f;

    const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
    const cv::Mat tvec = (cv::Mat_<double>(3, 1) << x_mm, y_mm, z_mm);
    const cv::Scalar col = (color == 0) ? cv::Scalar(255, 90, 20) : cv::Scalar(20, 90, 255);

    for (int side = -1; side <= 1; side += 2) {
        const float cx = side * bar_x;
        const std::vector<cv::Point3f> bar{
            {cx - bar_w * 0.5f, -bar_h * 0.5f, 0.0f},
            {cx + bar_w * 0.5f, -bar_h * 0.5f, 0.0f},
            {cx + bar_w * 0.5f, bar_h * 0.5f, 0.0f},
            {cx - bar_w * 0.5f, bar_h * 0.5f, 0.0f}};
        std::vector<cv::Point2f> ip;
        cv::projectPoints(bar, rvec, tvec, camera_matrix(), distortion_coeffs(), ip);
        std::vector<cv::Point> poly;
        for (const auto& p : ip) poly.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::fillConvexPoly(img, poly, col, cv::LINE_8);
    }
    (void)hh;
    return img;
}

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1 && argv[1][0] != '-') ? argv[1] : "/tmp/synth";
    float bar_w_ratio = 0.16f, bar_h_ratio = 0.9f, gap_ratio = 1.0f;
    std::vector<float> dists{1.5f, 3.0f, 5.0f};
    std::string dist_str = "1.5,3,5";

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--bar-w" && i + 1 < argc) bar_w_ratio = std::atof(argv[++i]);
        if (a == "--bar-h" && i + 1 < argc) bar_h_ratio = std::atof(argv[++i]);
        if (a == "--gap" && i + 1 < argc) gap_ratio = std::atof(argv[++i]);
        if (a == "--dist" && i + 1 < argc) dist_str = argv[++i];
    }
    dists.clear();
    {
        std::string cur;
        for (const char c : dist_str) {
            if (c == ',') {
                if (!cur.empty()) dists.push_back(std::atof(cur.c_str()));
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
        if (!cur.empty()) dists.push_back(std::atof(cur.c_str()));
    }

    ::mkdir(out_dir.c_str(), 0755);
    std::printf("输出目录 %s  灯条宽=%.2f*板宽 高=%.2f*板高 间距=%.2f\n", out_dir.c_str(),
                bar_w_ratio, bar_h_ratio, gap_ratio);

    for (const float d : dists) {
        const float z = d * 1000.0f;
        // 正对
        {
            const cv::Mat img = make_armor_parametric(0, 0, z, bar_w_ratio, bar_h_ratio,
                                                      gap_ratio, 0);
            char name[256];
            std::snprintf(name, sizeof(name), "%s/center_%.2fm.jpg", out_dir.c_str(), d);
            cv::imwrite(name, img);
            std::printf("  %s  (真值 dist=%.2f yaw=0 pitch=0)\n", name, d);
        }
        // 偏右 8 度
        {
            const float x = z * std::tan(8.0f * static_cast<float>(CV_PI) / 180.0f);
            const cv::Mat img = make_armor_parametric(x, 0, z, bar_w_ratio, bar_h_ratio,
                                                      gap_ratio, 0);
            char name[256];
            std::snprintf(name, sizeof(name), "%s/right8_%.2fm.jpg", out_dir.c_str(), d);
            cv::imwrite(name, img);
            const auto s = make_armor_image(SynthPose{});
            (void)s;
            std::printf("  %s  (真值 dist≈%.2f yaw=8 pitch=0)\n", name, d);
        }
    }
    return 0;
}

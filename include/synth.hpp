// synth.hpp: 合成靶标图像生成器 (测试用)
//
// 为什么需要它: 实车只有一台, 相机也只有一台, 不能拿实车反复试错。
// 用已知位姿合成一张"装甲板照片", 就能在没有相机的情况下验证
//   PnP 解算精度 / 灯条提取 / 配对 / 跟踪状态机 / 闭环收敛性
// 这是把这个项目做成可回归测试的关键工具。
#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "solver.hpp"

namespace srm {
namespace test {

// 目标位姿描述: 相对相机光心的位置(mm)与朝向(度)
struct SynthPose {
    float x_mm = 0.0f;      // 目标中心在相机系下的位置, +X 右, +Y 下, +Z 前
    float y_mm = 0.0f;
    float z_mm = 5000.0f;
    float yaw_deg = 0.0f;   // 绕 y 轴旋转(绕竖直轴左右转)
    float pitch_deg = 0.0f; // 绕 x 轴旋转(抬头低头)
    int   color = 0;        // 0=蓝 1=红
};

struct SynthResult {
    cv::Mat image;
    std::vector<cv::Point2f> corners;   // 左上/右上/右下/左下, 注入用
    bool ok = false;
};

// 生成一张标准 bayer 风格的图(实际是 BGR), 黑底 + 两块 LED 灯条 + 白点噪声
inline SynthResult make_armor_image(const SynthPose& pose,
                                    int width = 1440, int height = 1080,
                                    float noise = 0.0f, uint32_t seed = 12345) {
    SynthResult out;
    out.image = cv::Mat(height, width, CV_8UC3, cv::Scalar(18, 18, 18));

    // 物点: 装甲板 4 角 (与 solver 的物点一致)
    const float hw = ARMOR_W * 0.5f;
    const float hh = ARMOR_H * 0.5f;
    const std::vector<cv::Point3f> obj{
        {-hw, -hh, 0.0f}, {hw, -hh, 0.0f}, {hw, hh, 0.0f}, {-hw, hh, 0.0f}};

    // 目标位姿: 先绕 Y(yaw) 再绕 X(pitch), 然后平移到 pose 位置
    const double ry = pose.yaw_deg * CV_PI / 180.0;
    const double rx = pose.pitch_deg * CV_PI / 180.0;
    cv::Matx33d Ry(std::cos(ry), 0, std::sin(ry), 0, 1, 0, -std::sin(ry), 0, std::cos(ry));
    cv::Matx33d Rx(1, 0, 0, 0, std::cos(rx), -std::sin(rx), 0, std::sin(rx), std::cos(rx));
    const cv::Matx33d R = Ry * Rx;

    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(R), rvec);
    const cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.x_mm, pose.y_mm, pose.z_mm);
    cv::projectPoints(obj, rvec, tvec, camera_matrix(), distortion_coeffs(), out.corners);
    out.ok = true;

    // 灯条: 装甲板左右两侧的竖直长条, 长度取装甲板高度, 宽度取板宽的 10%
    const float bar_w_obj = ARMOR_W * 0.10f;
    const float bar_x_obj = hw - bar_w_obj * 0.5f;
    const float bar_h_obj = ARMOR_H * 0.98f;

    const cv::Scalar blue(255, 90, 20);    // BGR: 偏蓝
    const cv::Scalar red (20, 90, 255);
    const cv::Scalar col = (pose.color == 0) ? blue : red;

    for (int side = -1; side <= 1; side += 2) {
        const float cx = side * bar_x_obj;
        const std::vector<cv::Point3f> bar{
            {cx - bar_w_obj * 0.5f, -bar_h_obj * 0.5f, 0.0f},
            {cx + bar_w_obj * 0.5f, -bar_h_obj * 0.5f, 0.0f},
            {cx + bar_w_obj * 0.5f,  bar_h_obj * 0.5f, 0.0f},
            {cx - bar_w_obj * 0.5f,  bar_h_obj * 0.5f, 0.0f}};
        std::vector<cv::Point2f> img;
        cv::projectPoints(bar, rvec, tvec, camera_matrix(), distortion_coeffs(), img);
        std::vector<cv::Point> ip;
        ip.reserve(img.size());
        for (const auto& p : img) ip.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::fillConvexPoly(out.image, ip, col, cv::LINE_AA);
    }

    if (noise > 0.0f) {
        cv::Mat n(height, width, CV_8UC3);
        cv::RNG rng(seed);
        rng.fill(n, cv::RNG::NORMAL, cv::Scalar(0, 0, 0), cv::Scalar(noise, noise, noise));
        out.image += n;
    }
    return out;
}

// 生成"环境干扰": 随机亮斑, 用来验证误检抑制
inline void add_distractors(cv::Mat& img, int count, uint32_t seed = 999) {
    cv::RNG rng(seed);
    for (int i = 0; i < count; ++i) {
        const int x = rng.uniform(0, img.cols);
        const int y = rng.uniform(0, img.rows);
        const int w = rng.uniform(3, 12);
        const int h = rng.uniform(10, 60);
        cv::rectangle(img, cv::Rect(x, y, w, h),
                      cv::Scalar(rng.uniform(120, 255), rng.uniform(120, 255),
                                 rng.uniform(120, 255)),
                      cv::FILLED);
    }
}

}  // namespace test
}  // namespace srm

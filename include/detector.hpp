// detector.hpp: 灯条检测 + 装甲板配对
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "config.hpp"

struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length, width, angle;
};

struct Armor {
    cv::Point2f center;
    float dist, yaw, pitch;  // solver 填, 这里先占位
};

// 二值化后找灯条
inline std::vector<Lightbar> detect_lightbars(const cv::Mat& binary) {
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(binary, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Lightbar> lbs;
    for (auto& c : cs) {
        if (cv::contourArea(c) < 10) continue;
        cv::RotatedRect rr = cv::minAreaRect(c);
        float w = rr.size.width, h = rr.size.height;
        float L = std::max(w, h), s = std::min(w, h);
        if (s < LB_SHORT_MIN || L < LB_LONG_MIN) continue;
        if (L/s < LB_RATIO_MIN || L/s > LB_RATIO_MAX) continue;
        float ang = rr.angle; if (w < h) ang += 90;
        if (std::abs(ang-90) > 45 && std::abs(ang-90) < 135) continue;
        Lightbar lb; lb.rect = rr; lb.center = rr.center;
        lb.length = L; lb.width = s; lb.angle = ang;
        lbs.push_back(lb);
    }
    std::sort(lbs.begin(), lbs.end(),
              [](const Lightbar&a, const Lightbar&b){return a.center.x < b.center.x;});
    return lbs;
}

// 两灯条配对, 返回所有候选装甲板中心
inline std::vector<cv::Point2f> pair_armors(const std::vector<Lightbar>& lbs,
                                             std::vector<cv::Point2f>* corners_out = nullptr) {
    std::vector<cv::Point2f> centers;
    for (size_t i = 0; i < lbs.size(); i++) {
        for (size_t j = i+1; j < lbs.size(); j++) {
            auto& L = lbs[i]; auto& R = lbs[j];
            float d = cv::norm(L.center - R.center);
            float dy = std::abs(L.center.y - R.center.y);
            float lr = std::max(L.length,R.length)/std::min(L.length,R.length);
            float dAng = std::abs(L.angle - R.angle);
            if (dAng > PAIR_ANGLE_DIFF && dAng < 165) continue;
            if (d < PAIR_DIST_MIN || d > PAIR_DIST_MAX) continue;
            if (dy > PAIR_DY_MAX || lr > PAIR_LEN_RATIO) continue;
            float ar = d / ((L.length + R.length)/2.0f);
            if (ar < ARMOR_RATIO_MIN || ar > ARMOR_RATIO_MAX) continue;
            centers.push_back((L.center + R.center)/2.0f);
            if (corners_out) {
                float aw = d, ah = (L.length+R.length)/2.0f;
                cv::Point2f c = centers.back();
                corners_out->push_back(cv::Point2f(c.x-aw/2, c.y-ah/2));
                corners_out->push_back(cv::Point2f(c.x+aw/2, c.y-ah/2));
                corners_out->push_back(cv::Point2f(c.x+aw/2, c.y+ah/2));
                corners_out->push_back(cv::Point2f(c.x-aw/2, c.y+ah/2));
            }
        }
    }
    return centers;
}
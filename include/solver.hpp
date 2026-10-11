// solver.hpp: PnP 解算
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "config.hpp"

struct PnPResult {
    float dist;    // 米
    float yaw;     // 度
    float pitch;   // 度
};

inline cv::Mat camera_matrix() {
    return (cv::Mat_<double>(3,3) << CAM_FX,0,CAM_CX, 0,CAM_FY,CAM_CY, 0,0,1);
}

inline std::vector<cv::Point3f> armor_object_points() {
    float hw = ARMOR_W/2, hh = ARMOR_H/2;
    return {
        cv::Point3f(-hw,-hh,0), cv::Point3f(hw,-hh,0),
        cv::Point3f(hw,hh,0),   cv::Point3f(-hw,hh,0)
    };
}

// 给四个像素角点, 解出距离和角度
inline bool solve_pnp(const std::vector<cv::Point2f>& img_pts, PnPResult& out) {
    if (img_pts.size() != 4) return false;
    cv::Mat rvec, tvec;
    cv::Mat K = camera_matrix();
    cv::Mat dist = cv::Mat::zeros(5,1,CV_64F);
    auto obj = armor_object_points();
    bool ok = cv::solvePnP(obj, img_pts, K, dist, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) return false;
    out.dist   = tvec.at<double>(2)/1000.0;
    out.yaw    = std::atan2(tvec.at<double>(0), tvec.at<double>(2))*180.0/CV_PI;
    out.pitch  = std::atan2(tvec.at<double>(1), tvec.at<double>(2))*180.0/CV_PI;
    return true;
}
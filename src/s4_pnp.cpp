// S4: PnP 解算 —— 装甲板像素四角点 -> 相机系 3D 位姿
// 对应同济 solver.cpp: solvePnP + 弹道/云台角度
// 比赛装甲板物理尺寸: 140mm x 125mm
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length, width, angle;
};

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " <video>\n"; return -1; }
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) { std::cerr << "open fail\n"; return -1; }

    // ===== 相机内参（demo 视频近似值，实车需标定）=====
    // 假设分辨率 1280x800, 焦距 fx=fy=800
    float fx = 1818.37f, fy = 1822.49f;
    float cx = 751.06f, cy = 530.44f;
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) <<
        fx, 0, cx,
        0, fy, cy,
        0,  0,  1);
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);  // 暂不考虑畸变

    // ===== 装甲板物理坐标 (mm, 以中心为原点, x右 y下 z=0) =====
    // 140mm x 125mm, 四角: tl tr br bl
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(-70, -62.5f, 0),
        cv::Point3f( 70, -62.5f, 0),
        cv::Point3f( 70,  62.5f, 0),
        cv::Point3f(-70,  62.5f, 0),
    };

    int threshold_val = 200;
    cv::Mat frame, gray, binary;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        int W = frame.cols, H = frame.rows;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, threshold_val, 255, cv::THRESH_BINARY);

        // --- 灯条检测（同 S2/S3）---
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Lightbar> lightbars;
        for (auto& c : contours) {
            float ca = (float)cv::contourArea(c);
            if (ca < 10) continue;
            cv::RotatedRect rr = cv::minAreaRect(c);
            float w = rr.size.width, h = rr.size.height;
            float longSide = std::max(w, h), shortSide = std::min(w, h);
            if (shortSide < 3 || longSide < 8) continue;
            if (longSide/shortSide < 3.0 || longSide/shortSide > 15.0) continue;
            float ang = rr.angle;
            if (w < h) ang += 90;
            float dev = std::abs(ang - 90);
            if (dev > 45 && dev < 135) continue;
            Lightbar lb; lb.rect = rr; lb.center = rr.center;
            lb.length = longSide; lb.width = shortSide; lb.angle = ang;
            lightbars.push_back(lb);
        }
        std::sort(lightbars.begin(), lightbars.end(),
                  [](const Lightbar& a, const Lightbar& b){ return a.center.x < b.center.x; });

        // --- 配对成装甲板（取第一对）---
        for (size_t i = 0; i < lightbars.size(); i++) {
            for (size_t j = i+1; j < lightbars.size(); j++) {
                auto& L = lightbars[i]; auto& R = lightbars[j];
                float dist = cv::norm(L.center - R.center);
                float dy = std::abs(L.center.y - R.center.y);
                float len_ratio = std::max(L.length,R.length)/std::min(L.length,R.length);
                float dang = std::abs(L.angle - R.angle);
                if (dang > 15 && dang < 165) continue;
                if (dist < 10 || dist > 500 || dy > 30 || len_ratio > 2.0) continue;

                // 装甲板中心与四角像素坐标（外接矩形近似）
                cv::Point2f center = (L.center + R.center)/2.0f;
                float aw = dist;
                float ah = (L.length + R.length)/2.0f;
                std::vector<cv::Point2f> image_points = {
                    cv::Point2f(center.x - aw/2, center.y - ah/2),
                    cv::Point2f(center.x + aw/2, center.y - ah/2),
                    cv::Point2f(center.x + aw/2, center.y + ah/2),
                    cv::Point2f(center.x - aw/2, center.y + ah/2),
                };

                // ===== PnP 解算 =====
                cv::Mat rvec, tvec;
                cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                             rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

                float z = tvec.at<double>(2)/1000.0f;   // mm -> m
                float yaw   = std::atan2(tvec.at<double>(0), tvec.at<double>(2)) * 180.0f / CV_PI;
                float pitch = std::atan2(tvec.at<double>(1), tvec.at<double>(2)) * 180.0f / CV_PI;

                // 画装甲板框 + 打印
                cv::rectangle(frame,
                    cv::Point(center.x-aw/2, center.y-ah/2),
                    cv::Point(center.x+aw/2, center.y+ah/2),
                    cv::Scalar(0,0,255), 2);
                char buf[128];
                snprintf(buf, sizeof(buf), "dist:%.2fm  yaw:%.1f  pitch:%.1f", z, yaw, pitch);
                cv::putText(frame, buf, cv::Point(10,30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
                std::cout << buf << "\n";
                goto done;  // 只解算第一个装甲板
            }
        }
        done:
        cv::imshow("S4 pnp", frame);
        cv::imshow("binary", binary);
        if (cv::waitKey(30) == 'q') break;
    }
    return 0;
}
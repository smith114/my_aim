// S2 v2: 灯条检测 + 面积/填充率过滤
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length, width;
};

// 几何筛选：传入轮廓面积 contour_area，用于算填充率
bool isLightbar(const cv::RotatedRect& r, float contour_area) {
    float w = r.size.width, h = r.size.height;
    float longSide  = std::max(w, h);
    float shortSide = std::min(w, h);
    if (shortSide < 3) return false;
    if (longSide  < 8) return false;

    float ratio = longSide / shortSide;
    if (ratio < 3.0 || ratio > 15.0) return false;

    // 角度：接近垂直
    float ang = r.angle;
    if (w < h) ang += 90;
    float dev = std::abs(ang - 90);
    if (dev > 45 && dev < 135) return false;

    // 新增1：面积范围（灯条是中等大小的亮块，太大是背景，太小是噪点）
    float rect_area = w * h;
    if (rect_area < 15.0 || rect_area > 700.0) return false;

    // 新增2：填充率（实心度）= 轮廓面积 / 旋转矩形面积
    // 灯条是实心长条，solidity 应较高；碎块/不规则高光 solidity 低
    float solidity = contour_area / rect_area;
    if (solidity < 0.35) return false;

    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " <video>\n"; return -1; }
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) { std::cerr << "open fail\n"; return -1; }

    int threshold_val = 200;
    cv::Mat frame, gray, binary;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, threshold_val, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Lightbar> lightbars;
        for (auto& c : contours) {
            float ca = (float)cv::contourArea(c);
            if (ca < 10) continue;                          // 极小轮廓直接丢
            cv::RotatedRect rr = cv::minAreaRect(c);
            if (isLightbar(rr, ca)) {
                Lightbar lb;
                lb.rect = rr;
                lb.center = rr.center;
                lb.length = std::max(rr.size.width, rr.size.height);
                lb.width  = std::min(rr.size.width, rr.size.height);
                lightbars.push_back(lb);
            }
        }

        for (auto& lb : lightbars) {
            cv::Point2f pts[4];
            lb.rect.points(pts);
            for (int i = 0; i < 4; i++)
                cv::line(frame, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 2);
            cv::circle(frame, lb.center, 3, cv::Scalar(0,0,255), -1);
        }

        cv::putText(frame, "lightbars: " + std::to_string(lightbars.size()),
                    cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,255), 2);
        cv::imshow("S2 lightbar", frame);
        cv::imshow("binary", binary);

        int key = cv::waitKey(30);
        if (key == 'q' || key == 27) break;
    }
    return 0;
}
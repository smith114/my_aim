// S3: 灯条配对成装甲板 —— 用装甲板几何约束过滤误检
// 对应同济 detector.cpp: 灯条按 x 排序 -> 两两配对 -> check_geometry armor
// 比赛装甲板: 140mm x 125mm, 两灯条平行等高, 宽高比约 1.1
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

struct Lightbar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length, width;
    float angle;   // 长边方向角
};

struct Armor {
    Lightbar left, right;
    cv::Point2f center;
    float width;    // 两灯条中心距
    float height;   // 平均灯条长度
    cv::Point2f pts[4];  // 装甲板四角: tl, tr, br, bl
};

// 宽高比得分: 比赛装甲板 140:125 = 1.12
bool checkArmor(const Lightbar& a, const Lightbar& b) {
    // 保证 left 在左, right 在右
    const Lightbar* L = a.center.x < b.center.x ? &a : &b;
    const Lightbar* R = a.center.x < b.center.x ? &b : &a;

    // 1. 两灯条中心距（装甲板宽度）
    float dist = cv::norm(L->center - R->center);
    if (dist < 10 || dist > 500) return false;

    // 2. 两灯条长度相近（比在 0.5~2.0 之间）
    float len_ratio = std::max(L->length, R->length) / std::min(L->length, R->length);
    if (len_ratio > 2.0) return false;

    // 3. 两灯条高度差（中心 y 差不能太大，装甲板应水平）
    float dy = std::abs(L->center.y - R->center.y);
    if (dy > 30) return false;

    // 4. 两灯条平行（角度差 < 15°）
    float dang = std::abs(L->angle - R->angle);
    if (dang > 15 && dang < 165) return false;

    // 5. 装甲板宽高比: width/height 应接近 1.1 (140/125)
    float avg_len = (L->length + R->length) / 2.0f;
    float armor_ratio = dist / avg_len;
    if (armor_ratio < 0.6 || armor_ratio > 2.0) return false;

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

        // --- 找灯条 ---
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
            float ratio = longSide / shortSide;
            if (ratio < 3.0 || ratio > 15.0) continue;
            float ang = rr.angle;
            if (w < h) ang += 90;
            float dev = std::abs(ang - 90);
            if (dev > 45 && dev < 135) continue;

            Lightbar lb;
            lb.rect = rr;
            lb.center = rr.center;
            lb.length = longSide;
            lb.width  = shortSide;
            lb.angle  = ang;
            lightbars.push_back(lb);
        }

        // --- 按 x 排序，两两配对成装甲板 ---
        std::sort(lightbars.begin(), lightbars.end(),
                  [](const Lightbar& a, const Lightbar& b){ return a.center.x < b.center.x; });

        std::vector<Armor> armors;
        for (size_t i = 0; i < lightbars.size(); i++) {
            for (size_t j = i+1; j < lightbars.size(); j++) {
                if (checkArmor(lightbars[i], lightbars[j])) {
                    Armor ar;
                    ar.left  = lightbars[i];
                    ar.right = lightbars[j];
                    ar.center = (lightbars[i].center + lightbars[j].center) / 2.0f;
                    ar.width  = cv::norm(lightbars[i].center - lightbars[j].center);
                    ar.height = (lightbars[i].length + lightbars[j].length) / 2.0f;
                    armors.push_back(ar);
                }
            }
        }

        // --- 画：灯条绿框，装甲板红框 ---
        for (auto& lb : lightbars) {
            cv::Point2f pts[4];
            lb.rect.points(pts);
            for (int k = 0; k < 4; k++)
                cv::line(frame, pts[k], pts[(k+1)%4], cv::Scalar(0,255,0), 1);
        }
        for (auto& ar : armors) {
            // 装甲板外接矩形
            cv::rectangle(frame,
                cv::Point(ar.center.x - ar.width/2, ar.center.y - ar.height/2),
                cv::Point(ar.center.x + ar.width/2, ar.center.y + ar.height/2),
                cv::Scalar(0,0,255), 2);
            cv::circle(frame, ar.center, 4, cv::Scalar(255,0,255), -1);
        }

        cv::putText(frame, "bars:" + std::to_string(lightbars.size()) +
                          " armors:" + std::to_string(armors.size()),
                    cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0,255,255), 2);
        cv::imshow("S3 armor", frame);
        cv::imshow("binary", binary);

        int key = cv::waitKey(30);
        if (key == 'q' || key == 27) break;
    }
    return 0;
}
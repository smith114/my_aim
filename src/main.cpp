// main.cpp: YOLO主检测 + 灯条辅助验证 + PnP + 状态机 + 串口
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "config.hpp"
#include "detector.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "serial.hpp"
#include "yolo_detector.hpp"

// 在ROI内验证灯条: 至少2条亮灯条才算真装甲板
static bool verify_lightbars(const cv::Mat& frame, const cv::Rect& box) {
    cv::Rect roi = box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width < 10 || roi.height < 10) return false;
    cv::Mat gray, binary;
    cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, BINARIZE_THRESH, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(binary, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int lightbar_count = 0;
    for (auto& c : cs) {
        if (cv::contourArea(c) < 5) continue;
        cv::RotatedRect rr = cv::minAreaRect(c);
        float L = std::max(rr.size.width, rr.size.height);
        float s = std::min(rr.size.width, rr.size.height);
        if (s < LB_SHORT_MIN || L < LB_LONG_MIN) continue;
        if (L/s < 2.0f) continue;
        lightbar_count++;
    }
    return lightbar_count >= 2;
}

int main(int argc, char** argv) {
    cv::VideoCapture cap;
    if (argc < 2) cap.open(0);
    else cap.open(argv[1]);
    if (!cap.isOpened()) { std::cerr << "cannot open video\n"; return -1; }

    // YOLO
    std::string model = "/home/badtime/sp_vision_25/assets/yolo11.xml";
    YoloDetector yolo(model, 0.4f, 0.3f);

    int serial_fd = open_serial("/dev/ttyACM0");
    if (serial_fd < 0) serial_fd = open_serial("/dev/ttyACM1");
    bool mock = (serial_fd < 0);
    if (mock) std::cout << "[MOCK]\n";
    else      std::cout << "[SERIAL] fd=" << serial_fd << "\n";

    Tracker tracker;
    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 1. YOLO 检测
        auto boxes = yolo.detect(frame);

        // 2. 灯条辅助验证 + 选最优
        YoloBox best;
        bool has_best = false;
        float best_score = 0;
        for (auto& b : boxes) {
            if (!verify_lightbars(frame, b.rect)) continue;  // 框内没灯条, 丢
            if (b.score > best_score) {
                best_score = b.score;
                best = b;
                has_best = true;
            }
        }

        // 3. PnP (YOLO角点顺序: 0=左上 1=左下 2=右下 3=右上)
        //    PnP需要: 左上 右上 右下 左下 = [0,3,2,1]
        std::vector<cv::Point2f> centers;
        std::vector<PnPResult> results;
        if (has_best && best.corners.size() == 4) {
            std::vector<cv::Point2f> ordered = {
                best.corners[0], best.corners[3],
                best.corners[2], best.corners[1]
            };
            PnPResult r;
            if (solve_pnp(ordered, r)) {
                centers.push_back((best.corners[0]+best.corners[2])/2.0f);
                results.push_back(r);
            }
        }

        // 4. 状态机
        tracker.update(centers, results);

        // 5. 发送
        auto pkt = pack_send(tracker.yaw, tracker.pitch, 0);
        if (mock) {
            printf("yaw=%6.2f pitch=%6.2f dist=%.2f score=%.2f n=%d\n",
                   tracker.yaw, tracker.pitch, tracker.dist, best_score, (int)boxes.size());
        } else {
            write(serial_fd, pkt.data(), pkt.size());
        }

        // 6. 画
        if (has_best) {
            cv::rectangle(frame, best.rect, cv::Scalar(0,255,0), 2);
            for (auto& p : best.corners) cv::circle(frame, p, 4, cv::Scalar(0,0,255), -1);
        }
        if (tracker.state == TrackState::TRACK || tracker.lost_count > 0)
            cv::circle(frame, tracker.center, 6, cv::Scalar(0,255,255), -1);
        char buf[128];
        snprintf(buf, sizeof(buf), "yaw:%.1f pitch:%.1f dist:%.2f lost:%d",
                 tracker.yaw, tracker.pitch, tracker.dist, tracker.lost_count);
        cv::putText(frame, buf, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,255,255), 2);
        cv::imshow("my_aim", frame);
        if (cv::waitKey(30) == 'q') break;
    }
    if (serial_fd >= 0) close(serial_fd);
    return 0;
}
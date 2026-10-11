// main.cpp: 主循环, 串起所有模块
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "config.hpp"
#include "detector.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "serial.hpp"

int main(int argc, char** argv) {
    cv::VideoCapture cap;
    if (argc < 2) cap.open(0);
    else cap.open(argv[1]);
    if (!cap.isOpened()) { std::cerr << "cannot open video\n"; return -1; }

    int serial_fd = open_serial("/dev/ttyACM0");
    if (serial_fd < 0) serial_fd = open_serial("/dev/ttyACM1");
    bool mock = (serial_fd < 0);
    if (mock) std::cout << "[MOCK] no /dev/ttyACM*\n";
    else      std::cout << "[SERIAL] fd=" << serial_fd << "\n";

    Tracker tracker;
    cv::Mat frame, gray, binary;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, BINARIZE_THRESH, 255, cv::THRESH_BINARY);

        // 1. 灯条检测
        auto lbs = detect_lightbars(binary);

        // 2. 配对, 同时拿到四个角点
        std::vector<cv::Point2f> corners;
        auto centers = pair_armors(lbs, &corners);

        // 3. 每个候选解 PnP
        std::vector<PnPResult> results;
        for (size_t i = 0; i < corners.size(); i += 4) {
            std::vector<cv::Point2f> img_pts(corners.begin()+i, corners.begin()+i+4);
            PnPResult r;
            if (solve_pnp(img_pts, r)) results.push_back(r);
        }

        // 4. 状态机选优
        tracker.update(centers, results);

        // 5. 组帧发送
        auto pkt = pack_send(tracker.yaw, tracker.pitch, 0);
        if (mock) {
            printf("yaw=%6.2f pitch=%6.2f lost=%d n=%d | ",
                   tracker.yaw, tracker.pitch, tracker.lost_count, (int)centers.size());
            for (auto b : pkt) printf("%02x ", b);
            printf("\n");
        } else {
            write(serial_fd, pkt.data(), pkt.size());
        }

        // 6. 画
        cv::Scalar color = tracker.state == TrackState::TRACK ? cv::Scalar(0,255,0) :
                            tracker.lost_count > 0 ? cv::Scalar(0,165,255) : cv::Scalar(0,0,255);
        if (tracker.state == TrackState::TRACK || tracker.lost_count > 0)
            cv::circle(frame, tracker.center, 6, color, -1);
        char buf[128];
        snprintf(buf, sizeof(buf), "dist:%.2f yaw:%.1f pitch:%.1f lost:%d",
                 tracker.dist, tracker.yaw, tracker.pitch, tracker.lost_count);
        cv::putText(frame, buf, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0,255,255), 2);
        cv::imshow("my_aim", frame);
        if (cv::waitKey(30) == 'q') break;
    }
    if (serial_fd >= 0) close(serial_fd);
    return 0;
}
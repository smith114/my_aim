// yolo_test.cpp: YOLO/OpenVINO 环境自检
//
// 主流程不依赖 YOLO(用的是全图灯条法), 这个工具只用来确认
// OpenVINO 与模型文件在目标机器上是可用的, 并输出一张画了框和角点的图。
//
// 用法:
//   ./yolo_test <图片或视频> [输出图]
//   ./yolo_test yolo_frame0.jpg /tmp/yolo_out.jpg
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "yolo_detector.hpp"

int main(int argc, char** argv) {
    const std::string model = "assets/" + std::string("yolo11.xml");
    const std::string input = (argc > 1) ? argv[1] : "yolo_frame0.jpg";
    const std::string output = (argc > 2) ? argv[2] : "/tmp/yolo_out.jpg";

    std::cout << "[YOLO] model=" << model << " input=" << input << "\n";
    srm::YoloDetector det(model, srm::YOLO_CONF_THRESH, srm::YOLO_NMS_THRESH);

    cv::Mat frame;
    if (input.size() > 4 &&
        (input.substr(input.size() - 4) == ".mp4" || input.substr(input.size() - 4) == ".avi")) {
        cv::VideoCapture cap(input);
        if (!cap.isOpened()) {
            std::cerr << "[YOLO] 打不开视频: " << input << "\n";
            return 1;
        }
        for (int i = 0; i < 30; ++i) cap >> frame;
    } else {
        frame = cv::imread(input, cv::IMREAD_COLOR);
    }
    if (frame.empty()) {
        std::cerr << "[YOLO] 读不到输入: " << input << "\n";
        return 1;
    }
    std::cout << "[YOLO] frame " << frame.cols << "x" << frame.rows << "\n";

    const auto boxes = det.detect(frame);
    std::cout << "[YOLO] boxes=" << boxes.size() << "\n";
    for (const auto& b : boxes) {
        cv::rectangle(frame, b.rect, cv::Scalar(0, 255, 0), 2);
        std::cout << "  box score=" << b.score << " rect=" << b.rect << " corners="
                  << b.corners.size() << "\n";
        for (size_t k = 0; k < b.corners.size(); ++k) {
            cv::circle(frame, b.corners[k], 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(frame, std::to_string(k), b.corners[k] + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
    }
    cv::imwrite(output, frame);
    std::cout << "[YOLO] saved " << output << "\n";
    return 0;
}

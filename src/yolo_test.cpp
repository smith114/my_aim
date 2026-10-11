// yolo_test.cpp: 终端打印 YOLO 检测结果
#include <opencv2/opencv.hpp>
#include <iostream>
#include "yolo_detector.hpp"

int main(int argc, char** argv) {
    std::string model = "/home/badtime/sp_vision_25/assets/yolo11.xml";
    std::string video = "/home/badtime/sp_vision_25/assets/demo/demo.avi";
    if (argc > 1) video = argv[1];

    std::cout << "[INIT] loading model...\n";
    YoloDetector det(model, 0.5f, 0.3f);
    std::cout << "[INIT] model loaded\n";

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) { std::cerr << "no video\n"; return -1; }

    cv::Mat frame;
    int frame_id = 0;
    while (frame_id < 100) {
        cap >> frame;
        if (frame.empty()) break;
        auto boxes = det.detect(frame);
        if (frame_id % 10 == 0) {
            std::cout << "frame " << frame_id << ": " << boxes.size() << " boxes";
            for (auto& b : boxes) {
                printf(" [x=%d y=%d w=%d h=%d s=%.2f]",
                       b.rect.x, b.rect.y, b.rect.width, b.rect.height, b.score);
            }
            std::cout << "\n";
        }
        frame_id++;
    }
    std::cout << "[DONE] processed " << frame_id << " frames\n";
    return 0;
}
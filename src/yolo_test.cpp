#include <opencv2/opencv.hpp>
#include <iostream>
#include "yolo_detector.hpp"
int main() {
    YoloDetector det("/home/badtime/sp_vision_25/assets/yolo11.xml", 0.4f, 0.3f);
    cv::VideoCapture cap("/home/badtime/sp_vision_25/assets/demo/demo.avi");
    cv::Mat frame;
    for (int i = 0; i < 30; i++) cap >> frame;
    auto boxes = det.detect(frame);
    std::cout << "boxes: " << boxes.size() << "\n";
    for (auto& b : boxes) {
        cv::rectangle(frame, b.rect, cv::Scalar(0,255,0), 2);
        for (size_t k = 0; k < b.corners.size(); k++) {
            cv::circle(frame, b.corners[k], 5, cv::Scalar(0,0,255), -1);
            cv::putText(frame, std::to_string(k), b.corners[k]+cv::Point2f(5,-5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);
        }
    }
    cv::imwrite("/home/badtime/my_aim/yolo_corners.jpg", frame);
    return 0;
}
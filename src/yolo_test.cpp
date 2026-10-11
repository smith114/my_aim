#include <opencv2/opencv.hpp>
#include <iostream>
#include "yolo_detector.hpp"
int main() {
    YoloDetector det("/home/badtime/sp_vision_25/assets/yolo11.xml", 0.4f, 0.3f);
    cv::VideoCapture cap("/home/badtime/sp_vision_25/assets/demo/demo.avi");
    cv::Mat frame;
    int id = 0;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        auto boxes = det.detect(frame);
        if (id % 10 == 0) {
            printf("frame %d: %d boxes", id, (int)boxes.size());
            for (auto& b : boxes)
                printf(" [x=%d y=%d w=%d h=%d s=%.2f]",
                       b.rect.x, b.rect.y, b.rect.width, b.rect.height, b.score);
            printf("\n");
        }
        id++;
    }
    return 0;
}
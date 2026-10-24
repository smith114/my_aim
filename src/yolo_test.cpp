// yolo_test.cpp: YOLO/OpenVINO 环境自检
//
// 主流程不依赖 YOLO(用的是全图灯条法), 这个工具只用来确认
// OpenVINO 与模型文件在目标机器上是可用的, 并输出一张画了框和角点的图。
//
// 用法:
//   ./yolo_test <图片或视频> [输出图] [--model path/to/model.xml]
//   ./yolo_test yolo_frame0.jpg /tmp/yolo_out.jpg
//
// 换模型用 --model, 不用改代码:
//   ./yolo_test frame.jpg out.jpg --model assets/yolov8.xml
// 注意换模型后输出布局必须一致(38 类 + 4 角点), 否则后处理会静默给出错误框,
// 详见 config.hpp 的 YOLO_MODEL_XML 说明。
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "yolo_detector.hpp"

int main(int argc, char** argv) {
    std::string model = srm::YOLO_MODEL_XML;
    std::string input = "yolo_frame0.jpg";
    std::string output = "/tmp/yolo_out.jpg";
    bool have_input = false, have_output = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--model" && i + 1 < argc) {
            model = argv[++i];
        } else if (!have_input) {
            input = a;
            have_input = true;
        } else if (!have_output) {
            output = a;
            have_output = true;
        }
    }

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

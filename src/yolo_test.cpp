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
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "config.hpp"
#include "yolo_armor.hpp"
#include "yolo_detector.hpp"

int main(int argc, char** argv) {
    srm::YoloProfile prof = srm::make_yolo_profile(YOLO_PROFILE_SELECT);
    std::string input = "yolo_frame0.jpg";
    std::string output = "/tmp/yolo_out.jpg";
    bool have_input = false, have_output = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--model" && i + 1 < argc) {
            const std::string m = argv[++i];
            if (m == "v8" || m == "yolov8")       prof = srm::make_yolo_profile(YOLO_PROFILE_YOLOV8_2CLASS);
            else if (m == "v11" || m == "yolo11") prof = srm::make_yolo_profile(YOLO_PROFILE_YOLO11_38CLASS);
            else if (m == "v5" || m == "yolov5")   prof = srm::make_yolo_profile(YOLO_PROFILE_YOLOV5_10CLASS);
            else                                  prof.model_xml = m;
        } else if (!have_input) {
            input = a;
            have_input = true;
        } else if (!have_output) {
            output = a;
            have_output = true;
        }
    }

    std::cout << "[YOLO] model=" << prof.model_xml << " input=" << input << "\n";
    // 模型加载可能失败(路径错/布局不符/IR 版本不兼容), 必须捕获,
    // 否则整个程序直接 abort, 现场根本看不出是"模型没读到"还是别的。
    std::unique_ptr<srm::YoloDetector> det;
    try {
        det = std::make_unique<srm::YoloDetector>(prof, srm::YOLO_CONF_THRESH,
                                                  srm::YOLO_NMS_THRESH);
    } catch (const std::exception& e) {
        std::cerr << "[YOLO][ERR] 模型加载失败: " << e.what() << "\n";
        return 1;
    }

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

    const auto boxes = det->detect(frame);
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

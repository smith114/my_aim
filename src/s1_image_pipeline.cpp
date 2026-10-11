// S1: 图像管线 —— 读帧 / 灰度 / 二值化
// 对应同济 detector.cpp 的 detect() 前 3 行：cvtColor + threshold
// 运行: ./build/s1_image_pipeline ~/sp_vision_25/assets/demo/demo.avi
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // 1. 解析参数
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <video_path>\n";
        return -1;
    }

    // 2. 打开视频
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] fail to open video: " << argv[1] << "\n";
        return -1;
    }

    // 3. 阈值先写死，S2 再学用 yaml 配置
    int threshold_val = 200;
    std::cout << "[INFO] threshold = " << threshold_val
              << ", press q to quit, press +/- to adjust\n";

    cv::Mat frame, gray, binary;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;            // 视频读完

        // 4. 核心三步（同济 detector.cpp 同款）
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);          // 彩色 -> 灰度
        cv::threshold(gray, binary, threshold_val, 255,
                      cv::THRESH_BINARY);                         // 灰度 -> 二值
        //   THRESH_BINARY: 像素 > threshold_val ? 255 : 0
        //   灯条是自发光高亮条，二值化后就是白色亮块

        // 5. 显示
        cv::imshow("original", frame);
        cv::imshow("binary",  binary);

        // 6. 按键处理：q 退出，+/- 调阈值（边调边看灯条变化）
        int key = cv::waitKey(30);
        if (key == 'q' || key == 27) break;
        if (key == '+' || key == '=') { threshold_val += 10; std::cout << "threshold=" << threshold_val << "\n"; }
        if (key == '-')                { threshold_val -= 10; std::cout << "threshold=" << threshold_val << "\n"; }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
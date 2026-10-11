// calibrate.cpp: 拍棋盘格标定相机内参
// 用法: ./calibrate
// 按空格拍照, 拍够20张后自动计算
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { std::cerr << "no camera\n"; return -1; }

    cv::Size board(9, 6);  // 内角点 9x6
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f>> img_points;
    cv::Size img_size;

    std::cout << "对准棋盘格, 按空格拍照, 拍够20张自动计算\n";
    while (img_points.size() < 20) {
        cv::Mat frame;
        cap >> frame;
        if (img_size.empty()) img_size = frame.size();

        bool found = cv::findChessboardCorners(frame, board, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT, 30, 0.001));
            cv::drawChessboardCorners(frame, board, corners, found);
        }
        cv::putText(frame, "captured: " + std::to_string(img_points.size()) + "/20",
                    cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
        cv::imshow("calibrate", frame);
        char k = cv::waitKey(30);
        if (k == 27) break;
        if (k == ' ' && found) {
            img_points.push_back(corners);
            std::cout << "captured " << img_points.size() << "\n";
        }
    }

    // 计算内参
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board.height; i++)
        for (int j = 0; j < board.width; j++)
            obj.push_back(cv::Point3f(j*25, i*25, 0));  // 25mm格子
    std::vector<std::vector<cv::Point3f>> obj_points(img_points.size(), obj);

    cv::Mat K, dist, rvecs, tvecs;
    double err = cv::calibrateCamera(obj_points, img_points, img_size, K, dist, rvecs, tvecs);

    std::cout << "\n=== 标定结果 (复制到 config.hpp) ===\n";
    printf("constexpr float CAM_FX = %.2ff;\n", K.at<double>(0,0));
    printf("constexpr float CAM_FY = %.2ff;\n", K.at<double>(1,1));
    printf("constexpr float CAM_CX = %.2ff;\n", K.at<double>(0,2));
    printf("constexpr float CAM_CY = %.2ff;\n", K.at<double>(1,2));
    std::cout << "重投影误差: " << err << " px (应<1.0)\n";
    std::cout << "畸变系数: " << dist.t() << "\n";
    return 0;
}
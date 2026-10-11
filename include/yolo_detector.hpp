// yolo_detector.hpp: OpenVINO YOLO11 装甲板检测 (带4角点)
#pragma once
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>
#include <string>
#include <iostream>

struct YoloBox {
    cv::Rect rect;
    float score;
    std::vector<cv::Point2f> corners;  // 装甲板4角点
};

class YoloDetector {
public:
    YoloDetector(const std::string& model_xml, float conf_thresh = 0.4f, float nms_thresh = 0.3f)
        : conf_thresh_(conf_thresh), nms_thresh_(nms_thresh) {
        auto model = core_.read_model(model_xml);
        ov::preprocess::PrePostProcessor ppp(model);
        auto& input = ppp.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.model().set_layout("NCHW");
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale(255.0);
        model = ppp.build();
        compiled_ = core_.compile_model(model, "CPU");
        infer_request_ = compiled_.create_infer_request();
    }

    std::vector<YoloBox> detect(const cv::Mat& bgr) {
        double scale = std::min(640.0/bgr.rows, 640.0/bgr.cols);
        int h = bgr.rows * scale, w = bgr.cols * scale;
        cv::Mat input(640, 640, CV_8UC3, cv::Scalar(0,0,0));
        cv::resize(bgr, input(cv::Rect(0,0,w,h)), {w, h});

        ov::Tensor input_tensor(ov::element::u8, {1,640,640,3}, input.data);
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        auto out = infer_request_.get_output_tensor();
        auto shape = out.get_shape();
        cv::Mat output(shape[1], shape[2], CV_32F, out.data<float>());
        cv::transpose(output, output);

        constexpr int CLASS_NUM = 38;
        // 列: 0-3=xywh, 4-41=38类分数, 42-49=4角点(x,y)
        std::vector<YoloBox> results;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<std::vector<cv::Point2f>> all_corners;

        for (int i = 0; i < output.rows; i++) {
            float cx = output.at<float>(i,0);
            float cy = output.at<float>(i,1);
            float ww = output.at<float>(i,2);
            float hh = output.at<float>(i,3);
            float max_score = 0;
            for (int c = 4; c < 4+CLASS_NUM; c++) {
                float s = output.at<float>(i,c);
                if (s > max_score) max_score = s;
            }
            if (max_score < conf_thresh_) continue;

            // 角点: 列 42~49, 每两个一组
            std::vector<cv::Point2f> corners;
            for (int k = 0; k < 4; k++) {
                float px = output.at<float>(i, 42 + k*2);
                float py = output.at<float>(i, 42 + k*2 + 1);
                corners.push_back(cv::Point2f(px/scale, py/scale));
            }

            cv::Rect r;
            r.x = (cx - ww/2) / scale;
            r.y = (cy - hh/2) / scale;
            r.width  = ww / scale;
            r.height = hh / scale;
            r &= cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (r.width < 5 || r.height < 5) continue;

            boxes.push_back(r);
            scores.push_back(max_score);
            all_corners.push_back(corners);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, conf_thresh_, nms_thresh_, indices);
        for (int idx : indices) {
            results.push_back({boxes[idx], scores[idx], all_corners[idx]});
        }
        return results;
    }

private:
    ov::Core core_;
    ov::CompiledModel compiled_;
    ov::InferRequest infer_request_;
    float conf_thresh_, nms_thresh_;
};
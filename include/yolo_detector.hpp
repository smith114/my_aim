// yolo_detector.hpp: OpenVINO YOLO11 装甲板检测 (简化版)
#pragma once
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <vector>
#include <string>
#include <iostream>

struct YoloBox {
    cv::Rect rect;
    float score;
};

class YoloDetector {
public:
    YoloDetector(const std::string& model_xml, float conf_thresh = 0.5f, float nms_thresh = 0.3f)
        : conf_thresh_(conf_thresh), nms_thresh_(nms_thresh) {
        auto model = core_.read_model(model_xml);
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC");
        ppp.input().model().set_layout("NCHW");
        model = ppp.build();
        compiled_ = core_.compile_model(model, "CPU");
        infer_request_ = compiled_.create_infer_request();
    }

    std::vector<YoloBox> detect(const cv::Mat& bgr) {
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(640, 640));

        ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, resized.data);
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        auto out = infer_request_.get_output_tensor();
        auto shape = out.get_shape();
        // shape: [1, 54, 8400] -> transpose -> [8400, 54]
        cv::Mat output(shape[1], shape[2], CV_32F, out.data<float>());
        cv::transpose(output, output);

        // 列布局: 0-3=xywh, 4-41=38类分数, 42-53=关键点
        constexpr int CLASS_START = 4;
        constexpr int CLASS_NUM = 38;

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        float sx = (float)bgr.cols / 640.0f;
        float sy = (float)bgr.rows / 640.0f;

        for (int i = 0; i < output.rows; i++) {
            float cx = output.at<float>(i, 0);
            float cy = output.at<float>(i, 1);
            float w  = output.at<float>(i, 2);
            float h  = output.at<float>(i, 3);
            // 只在类分数列里找最大
            float max_score = 0;
            for (int c = CLASS_START; c < CLASS_START + CLASS_NUM; c++) {
                float s = output.at<float>(i, c);
                if (s > max_score) max_score = s;
            }
            if (max_score < conf_thresh_) continue;
            cv::Rect r;
            r.x = (cx - w/2) * sx;
            r.y = (cy - h/2) * sy;
            r.width  = w * sx;
            r.height = h * sy;
            boxes.push_back(r);
            scores.push_back(max_score);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, conf_thresh_, nms_thresh_, indices);
        std::vector<YoloBox> result;
        for (int idx : indices) result.push_back({boxes[idx], scores[idx]});
        return result;
    }

private:
    ov::Core core_;
    ov::CompiledModel compiled_;
    ov::InferRequest infer_request_;
    float conf_thresh_, nms_thresh_;
};
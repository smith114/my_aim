// yolo_detector.hpp: OpenVINO YOLO11 装甲板检测器 (带 4 角点输出)
//
// 说明: 主流程用的是 armor.hpp 的全图灯条法, 这个模块是备选/对照方案。
// 保留它的原因: (1) 可作为灯条法漏检时的补充;  (2) 已经过实测, 输出角点准确。
//
// 预处理与同济一致: BGR(u8) -> RGB -> /255, 缩放到 640x640, 左上角对齐填充。
// 后处理输出布局(单输出张量, 转置后每行一个候选):
//   [0..3]   cx, cy, w, h        (letterbox 后的 640 坐标系)
//   [4..41]  38 类分数
//   [42..49] 4 个角点 (x, y)
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace srm {

struct YoloBox {
    cv::Rect rect;
    float    score = 0.0f;
    std::vector<cv::Point2f> corners;   // 装甲板 4 角点(原图坐标)
};

class YoloDetector {
public:
    static constexpr int kInputSize = 640;
    static constexpr int kClassNum  = 38;
    static constexpr int kAttrStart = 4;                      // 类别分数起始列
    static constexpr int kCornerStart = kAttrStart + kClassNum;  // 角点起始列 = 42

    YoloDetector(const std::string& model_xml, float conf_thresh = 0.4f,
                 float nms_thresh = 0.3f, const std::string& device = "CPU")
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
        compiled_ = core_.compile_model(model, device);
        infer_request_ = compiled_.create_infer_request();
    }

    std::vector<YoloBox> detect(const cv::Mat& bgr) {
        std::vector<YoloBox> results;
        if (bgr.empty()) return results;

        // 等比缩放到 640x640 画布, 左上角对齐
        const double scale = std::min(static_cast<double>(kInputSize) / bgr.rows,
                                      static_cast<double>(kInputSize) / bgr.cols);
        const int w = static_cast<int>(bgr.cols * scale);
        const int h = static_cast<int>(bgr.rows * scale);
        cv::Mat canvas(kInputSize, kInputSize, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::resize(bgr, canvas(cv::Rect(0, 0, w, h)), {w, h});

        ov::Tensor input_tensor(ov::element::u8, {1, kInputSize, kInputSize, 3}, canvas.data);
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        const auto out = infer_request_.get_output_tensor();
        const auto shape = out.get_shape();
        // 期望 [1, N, 4+38+8]
        if (shape.size() != 3) return results;
        cv::Mat raw(static_cast<int>(shape[1]), static_cast<int>(shape[2]), CV_32F,
                    out.data<float>());
        cv::Mat output;
        cv::transpose(raw, output);   // -> [attrs, N] 的转置 = [N, attrs]
        if (output.cols < kCornerStart + 8) return results;

        std::vector<cv::Rect> boxes;
        std::vector<float>    scores;
        std::vector<std::vector<cv::Point2f>> all_corners;

        for (int i = 0; i < output.rows; ++i) {
            float max_score = 0.0f;
            for (int c = kAttrStart; c < kAttrStart + kClassNum; ++c) {
                const float s = output.at<float>(i, c);
                if (s > max_score) max_score = s;
            }
            if (max_score < conf_thresh_) continue;

            const float cx = output.at<float>(i, 0);
            const float cy = output.at<float>(i, 1);
            const float ww = output.at<float>(i, 2);
            const float hh = output.at<float>(i, 3);

            std::vector<cv::Point2f> corners;
            corners.reserve(4);
            for (int k = 0; k < 4; ++k) {
                corners.emplace_back(output.at<float>(i, kCornerStart + k * 2) / scale,
                                     output.at<float>(i, kCornerStart + k * 2 + 1) / scale);
            }

            cv::Rect r(cvRound((cx - ww * 0.5f) / scale), cvRound((cy - hh * 0.5f) / scale),
                       cvRound(ww / scale), cvRound(hh / scale));
            r &= cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (r.width < 5 || r.height < 5) continue;

            boxes.push_back(r);
            scores.push_back(max_score);
            all_corners.push_back(std::move(corners));
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, conf_thresh_, nms_thresh_, indices);
        results.reserve(indices.size());
        for (const int idx : indices) {
            results.push_back({boxes[idx], scores[idx], all_corners[idx]});
        }
        return results;
    }

    const std::string& device() const { return device_; }

private:
    ov::Core          core_;
    ov::CompiledModel compiled_;
    ov::InferRequest  infer_request_;
    float             conf_thresh_ = 0.4f;
    float             nms_thresh_  = 0.3f;
    std::string       device_      = "CPU";
};

}  // namespace srm

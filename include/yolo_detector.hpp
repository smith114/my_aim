// yolo_detector.hpp: OpenVINO 装甲板检测器 (带 4 角点输出)
//
// **支持多种模型布局, 不写死任何一套。**
//
// 为什么必须参数化: 实测(10-28)三个训练版本的后处理布局完全不同,
// 想换模型就得连解析一起换, 而"拿错布局"的后果是**静默给出错误框**
// (列偏移恰好读到角点坐标, 分数变成几百), 完全不报错。
//
//   yolo11 (我们原本的)  输入 640  输出 [1,50,8400]   4框 + 38类 + 8角点
//   yolov8 (参考实现用)  输入 416  输出 [1,14,3549]   4框 +  2类 + 8角点
//   yolov5 (参考实现备选) 输入 640  输出 [1,25200,22]  4框 + 10类 + 8角点
//
// 所以类别数、输入尺寸、角点偏移全部由 YoloProfile 传入, 构造函数里校验一致性。
//
// 预处理与参考实现一致: BGR(u8) -> RGB -> /255, letterbox 到 input_size,
// 填充值 114 (模型元数据 pad_value, 不是 0)。
// 类别分数**已过 sigmoid**(值域 [0,1]), 不要再套一次 —— 套了会让 8400 个 anchor
// 全部变成 0.5 左右, 等于全图都是目标。
#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace srm {

struct YoloBox {
    cv::Rect rect;                      // 原图坐标
    float    score = 0.0f;
    std::vector<cv::Point2f> corners;   // 装甲板 4 角点(原图坐标)
    int         cls  = -1;
    std::string name;
};

// 一个模型的布局描述。
//
// **三个模型的列顺序完全不同, 所以不能只描述"类别数"**:
//   yolo11:  [box 0..3][cls 4..41][kp 42..49]      已 sigmoid
//   yolov8:  [box 0..3][cls 4..5 ][kp 6..13 ]      已 sigmoid
//   yolov5:  [box 0..3][kp 4..11 ][cls 12..21]     **原始 logits**
// 注意 v5 的角点在类别**前面**, 而且要自己套 sigmoid —— 少任一条都会得到垃圾结果
// (把角点坐标当分数, 分数变成几百, 全图都是目标)。
struct YoloProfile {
    std::string model_xml;
    int         input_size   = 640;   // 网络输入边长(正方形)
    int         class_num    = 38;    // 类别数
    int         class_start  = 4;     // 类别分数起始列
    int         corner_start = 42;    // 角点起始列
    bool        class_is_logit = false;  // true=需要 sigmoid
    // 类别名, 顺序必须与训练时一致。38 类那套有完整命名;
    // 2 类 / 10 类那套只有颜色或未知, 见各自定义。
    std::vector<std::string> class_names;

    int attr_total() const {
        const int a = class_start + class_num;
        const int b = corner_start + 8;
        return a > b ? a : b;
    }
};

class YoloDetector {
public:
    // letterbox 填充值。模型元数据写死 pad_value=114, 用 0 与训练分布不一致。
    static constexpr int kPadValue = 114;

    explicit YoloDetector(YoloProfile profile, float conf_thresh = 0.30f,
                          float nms_thresh = 0.30f, const std::string& device = "CPU")
        : profile_(std::move(profile)), conf_thresh_(conf_thresh),
          nms_thresh_(nms_thresh), device_(device) {
        if (static_cast<int>(profile_.class_names.size()) != profile_.class_num) {
            std::cerr << "[YOLO] warn: 类别名数量(" << profile_.class_names.size()
                      << ") 与 class_num(" << profile_.class_num << ") 不一致\n";
        }
        auto model = core_.read_model(profile_.model_xml);

        // 开机就核对输入尺寸, 免得跑到一半才因尺寸不符崩掉
        const auto in_shape = model->input().get_shape();
        if (in_shape.size() != 4) throw std::runtime_error("YOLO 模型输入不是 4 维");
        const int mh = static_cast<int>(in_shape[2]);
        const int mw = static_cast<int>(in_shape[3]);
        std::cout << "[YOLO] 模型 " << profile_.model_xml << "  输入 " << mw << "x" << mh
                  << "  类别 " << profile_.class_num
                  << "  cls@" << profile_.class_start
                  << "  kp@" << profile_.corner_start
                  << (profile_.class_is_logit ? "  cls=logit(需sigmoid)" : "  cls=sigmoid后")
                  << "  期望属性数 " << profile_.attr_total() << "\n";
        if (mw != profile_.input_size || mh != profile_.input_size) {
            std::cerr << "[YOLO][WARN] 配置 input_size=" << profile_.input_size
                      << " 与模型实际 " << mw << " 不符, 用模型值\n";
            profile_.input_size = mw;
        }

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
        compiled_ = core_.compile_model(model, device_);
        infer_request_ = compiled_.create_infer_request();
    }

    const YoloProfile& profile() const { return profile_; }

    std::vector<YoloBox> detect(const cv::Mat& bgr) {
        std::vector<YoloBox> results;
        if (bgr.empty()) return results;

        const int S = profile_.input_size;

        // 等比 letterbox, 左上角对齐, 114 填充
        const double scale = std::min(static_cast<double>(S) / bgr.rows,
                                      static_cast<double>(S) / bgr.cols);
        const int w = static_cast<int>(bgr.cols * scale);
        const int h = static_cast<int>(bgr.rows * scale);
        cv::Mat canvas(S, S, CV_8UC3, cv::Scalar(kPadValue, kPadValue, kPadValue));
        cv::resize(bgr, canvas(cv::Rect(0, 0, w, h)), {w, h});

        ov::Tensor input_tensor(ov::element::u8,
                                {1, static_cast<size_t>(S), static_cast<size_t>(S), 3},
                                canvas.data);
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();

        const auto out = infer_request_.get_output_tensor();
        const auto shape = out.get_shape();
        if (shape.size() != 3) return results;

        // 统一成 [N, attrs]。yolo11 是 [1, attrs, N]; yolov8/v5 是 [1, N, attrs]
        const int d1 = static_cast<int>(shape[1]);
        const int d2 = static_cast<int>(shape[2]);
        float* out_ptr = const_cast<float*>(out.data<float>());
        cv::Mat raw(d1, d2, CV_32F, out_ptr);   // 只读使用
        cv::Mat output;
        const int want = profile_.attr_total();
        if (d2 == want) {
            output = raw;                    // [N, attrs]
        } else if (d1 == want) {
            cv::transpose(raw, output);      // [attrs, N] -> [N, attrs]
        } else {
            std::cerr << "[YOLO][ERR] 输出 " << d1 << "x" << d2 << " 与期望属性数 "
                      << want << " 不匹配, 疑似模型与布局配置不符, 本帧丢弃\n";
            return results;
        }

        const int kCornerStart = profile_.corner_start;
        const int kClassStart  = profile_.class_start;

        std::vector<cv::Rect> boxes;
        std::vector<float>    scores;
        std::vector<std::vector<cv::Point2f>> all_corners;
        std::vector<int>      all_cls;

        for (int i = 0; i < output.rows; ++i) {
            float max_score = 0.0f;
            int   best_cls  = -1;
            for (int c = 0; c < profile_.class_num; ++c) {
                float s = output.at<float>(i, kClassStart + c);
                if (profile_.class_is_logit) {
                    // v5 输出的是原始 logit, 必须过 sigmoid 才是概率
                    s = 1.0f / (1.0f + std::exp(-s));
                }
                if (s > max_score) {
                    max_score = s;
                    best_cls  = c;
                }
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
            all_cls.push_back(best_cls);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, conf_thresh_, nms_thresh_, indices);
        results.reserve(indices.size());
        for (const int idx : indices) {
            YoloBox b;
            b.rect    = boxes[idx];
            b.score   = scores[idx];
            b.corners = all_corners[idx];
            b.cls     = all_cls[idx];
            if (b.cls >= 0 && b.cls < static_cast<int>(profile_.class_names.size())) {
                b.name = profile_.class_names[b.cls];
            }
            results.push_back(std::move(b));
        }
        return results;
    }

    const std::string& device() const { return device_; }

private:
    YoloProfile       profile_;
    ov::Core          core_;
    ov::CompiledModel compiled_;
    ov::InferRequest  infer_request_;
    float             conf_thresh_ = 0.30f;
    float             nms_thresh_  = 0.30f;
    std::string       device_      = "CPU";
};

}  // namespace srm

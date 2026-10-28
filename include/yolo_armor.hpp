// yolo_armor.hpp: 把 YOLO 的输出转成主流程的 Armor
//
// 存在意义: 让"检测器"变成可替换的一层。下游 PnP / 跟踪器 / 伺服完全不知道
// 上游是灯条法还是神经网络 —— 它们只吃 Armor。
//
//   extract_lightbars + match_armors  ─┐
//                                      ├─> std::vector<Armor> ─> solve_pose ─> Tracker
//   YoloDetector::detect + 本文件转换  ─┘
//
// 所以切换检测器不需要改 PnP/跟踪/伺服/协议任何一行。
#pragma once

#include <string>
#include <vector>

#include "armor.hpp"
#include "yolo_detector.hpp"

namespace srm {

// 按选择号构造模型布局。三套布局差别很大, 全部在这里集中定义,
// 免得散落在各处对不上。
//
// 列顺序对照(实测 10-28):
//   yolo11:  [box 0..3][cls 4..41][kp 42..49]   640 输入  已 sigmoid
//   yolov8:  [box 0..3][cls 4..5 ][kp 6..13 ]   416 输入  已 sigmoid
//   yolov5:  [box 0..3][kp 4..11 ][cls 12..21]  640 输入  **原始 logits**
inline YoloProfile make_yolo_profile(int select) {
    YoloProfile p;
    if (select == YOLO_PROFILE_YOLOV8_2CLASS) {
        p.model_xml       = "assets/yolov8.xml";
        p.input_size      = 416;
        p.class_num       = 2;
        p.class_start     = 4;
        p.corner_start    = 6;      // 4 + 2
        p.class_is_logit  = false;
        p.class_names     = {"blue", "red"};
    } else if (select == YOLO_PROFILE_YOLOV5_10CLASS) {
        p.model_xml       = "assets/yolov5.xml";
        p.input_size      = 640;
        p.class_num       = 10;
        p.class_start     = 12;     // 角点在前, 类别在后
        p.corner_start    = 4;
        p.class_is_logit  = true;   // 必须自己 sigmoid
        // v5 那套没有附带类别名元数据, 用序号占位; 颜色无法从名字判断,
        // 需要靠调用方另行处理(见 yolo_color_of 的说明)。
        for (int i = 0; i < 10; ++i) p.class_names.push_back("cls" + std::to_string(i));
    } else {
        // 默认: yolo11, 38 类, 640 输入
        p.model_xml       = "assets/yolo11.xml";
        p.input_size      = 640;
        p.class_num       = 38;
        p.class_start     = 4;
        p.corner_start    = 42;
        p.class_is_logit  = false;
        p.class_names     = {
            "Bsentry", "Rsentry", "Esentry",
            "Bone", "Rone", "Eone",
            "Btwo", "Rtwo", "Etwo",
            "Bthree", "Rthree", "Ethree",
            "Bfour", "Rfour", "Efour",
            "Bfive", "Rfive", "Efive",
            "Boutpost", "Routpost", "Eoutpost",
            "Bbase", "Rbase", "Ebase", "Pbase",
            "Bbasesmall", "Rbasesmall", "Ebasesmall", "Pbasesmall",
            "Bbalancethree", "Rbalancethree", "Ebalancethree",
            "Bbalancefour", "Rbalancefour", "Ebalancefour",
            "Bbalancefive", "Rbalancefive", "Ebalancefive"};
    }
    return p;
}

// 从类别名取灯条颜色: 0=蓝 1=红。
// 两套模型命名不同, 都要认:
//   38 类: "Bsentry"/"Rthree"/"Eone"...  首字母 B=蓝 R=红
//    2 类: "blue"/"red"                  整词匹配
// E/P 类不区分敌我, 统一按蓝处理(本赛区打蓝方); 要严格区分就在调用处按 name 过滤。
inline int yolo_color_of(const YoloBox& b) {
    if (b.name.empty()) return 0;
    const char c0 = b.name[0];
    if (c0 == 'R' || c0 == 'r') return 1;
    if (c0 == 'B' || c0 == 'b') return 0;
    return 0;
}

// YoloBox -> Armor
//
// 角点顺序: 模型输出的 4 个角点顺序不保证, 必须过 sort_armor_points 规整成
// 左上/右上/右下/左下, 否则 PnP 的物点对不上, 会解出自交或翻转的位姿。
// (这个坑在灯条法里踩过: 未排序时四点连成"蝴蝶结", PnP 全部失败。)
inline Armor yolo_to_armor(const YoloBox& b) {
    Armor ar;
    ar.pts   = sort_armor_points(b.corners);
    ar.color = yolo_color_of(b);

    // 中心用 4 角点均值, 不用 bbox 中心 —— 斜视时 bbox 中心与真实几何中心有偏差
    cv::Point2f c(0.0f, 0.0f);
    for (const auto& p : ar.pts) c += p;
    c *= 0.25f;
    ar.center = c;

    // 宽高用规整后的角点算: 上边宽度、左边高度
    ar.width  = static_cast<float>(cv::norm(ar.pts[1] - ar.pts[0]));
    ar.height = static_cast<float>(cv::norm(ar.pts[3] - ar.pts[0]));
    return ar;
}

inline std::vector<Armor> yolo_to_armors(const std::vector<YoloBox>& boxes) {
    std::vector<Armor> out;
    out.reserve(boxes.size());
    for (const auto& b : boxes) {
        // 角点缺失或退化就丢弃, 不要送进 PnP
        if (b.corners.size() != 4) continue;
        const Armor ar = yolo_to_armor(b);
        if (ar.width < 3.0f || ar.height < 3.0f) continue;
        out.push_back(ar);
    }
    return out;
}

// 只保留某一方颜色的装甲板(0=蓝 1=红, -1=不过滤)。
// 比赛里我们固定打某一方, 直接过滤掉对面能显著减少误跟。
inline std::vector<Armor> filter_by_color(std::vector<Armor> v, int want) {
    if (want < 0) return v;
    std::vector<Armor> out;
    out.reserve(v.size());
    for (auto& a : v) {
        if (a.color == want) out.push_back(a);
    }
    return out;
}

}  // namespace srm

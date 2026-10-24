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

// 从类别名取灯条颜色: 0=蓝 1=红。
// 模型 38 类的命名是 "颜色前缀 + 机器人类型", B=蓝 R=红 E/P=其他(工程/基地)。
// E/P 类不区分敌我, 这里按蓝色处理, 因为本赛区我们打蓝方装甲板;
// 若以后要区分, 在 main 里按 name 过滤即可。
inline int yolo_color_of(const YoloBox& b) {
    if (!b.name.empty() && b.name[0] == 'R') return 1;
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

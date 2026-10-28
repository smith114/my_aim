// armor.hpp: 装甲板检测 —— 全图灯条提取 + 灯条配对
//
// 思路:
//   装甲板 = 两块点亮的平行 LED 灯带 + 中间黑色底板
//   先在 HSV 空间按色调分割出"灯条", 再两两配对, 用几何约束
//   (等高 / 对齐 / 间距比例 / 平行 / 中心暗区) 把假配对挤掉。
//   单看灯条本身无法判断真假(背景反光点几何上也像灯条), 必须成对判断。
//
// 为什么不用 YOLO 框: 本场景全图灯条法召回更稳; YOLO 分支保留在
// yolo_detector.hpp 作为备选, 不进主流程。
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

#include "config.hpp"

namespace srm {

struct LightBar {
    cv::RotatedRect rr;          // 全图坐标
    cv::Point2f     center;      // 全图坐标
    float           length = 0;  // 长边 px
    float           width  = 0;  // 短边 px
    float           dir    = 0;  // 长边方向角, 度, [0,180)
    int             color  = 0;  // 0=蓝 1=红
};

struct Armor {
    std::vector<cv::Point2f> pts;      // 4 角点: 左上/右上/右下/左下 (对应 PnP 物点)
    cv::Point2f center;
    float width  = 0;   // px
    float height = 0;   // px
    int   color  = 0;
};

// 灯条长边方向角(度, [0,180)): minAreaRect 的 angle 落在 [-90,0), 需要规整
inline float lightbar_dir(const cv::RotatedRect& rr) {
    float ang = rr.angle;
    if (rr.size.width < rr.size.height) ang += 90.0f;
    if (ang < 0.0f) ang += 180.0f;
    if (ang >= 180.0f) ang -= 180.0f;
    return ang;
}

// ---------------------------------------------------------------- 灯条提取
// 全分辨率处理。早期版本在宽度 >1280 时缩半以提速, 但实测发现 5 m 处的
// 灯条在缩半后只剩 4 px 宽, 会被最小宽度阈值直接滤掉(整个靶标丢失),
// 而全分辨率下 HSV+轮廓的耗时在 NUC 上完全可以接受。精度优先。
inline std::vector<LightBar> extract_lightbars(const cv::Mat& frame) {
    if (frame.empty()) return {};

    const cv::Mat& proc = frame;

    cv::Mat hsv;
    cv::cvtColor(proc, hsv, cv::COLOR_BGR2HSV);

    // 蓝: H LB_HUE_LO~LB_HUE_HI;  红: H 0~10 与 170~180
    // S/V 下限的取值理由见 config.hpp —— 它决定"灯条 vs 光晕/背景反光"的分界。
    // 实测不要并入"过曝白核心": 那会把相邻 LED 芯片粘连, 灯条变宽、长宽比跌破阈值。
    std::vector<cv::Mat> masks(2);
    cv::inRange(hsv, cv::Scalar(LB_HUE_LO, LB_SAT_MIN, LB_VAL_MIN),
                cv::Scalar(LB_HUE_HI, 255, 255), masks[0]);
    {
        cv::Mat r1, r2;
        cv::inRange(hsv, cv::Scalar(0, LB_SAT_MIN, LB_VAL_MIN), cv::Scalar(10, 255, 255), r1);
        cv::inRange(hsv, cv::Scalar(170, LB_SAT_MIN, LB_VAL_MIN), cv::Scalar(180, 255, 255), r2);
        cv::bitwise_or(r1, r2, masks[1]);
    }

    std::vector<cv::Mat> bgr;
    cv::split(proc, bgr);

    // 形态学: CLOSE 补掉 LED 芯片间因压缩噪点造成的小断裂; OPEN 可选(默认不做)。
    // 核必须小, 否则相邻芯片会被并成宽块, 细长灯条变成一个方块。
    const bool do_close = LB_MORPH_CLOSE_SIZE > 1;
    const bool do_open  = LB_MORPH_OPEN_SIZE  > 1;
    const cv::Mat kernel_close = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(std::max(1, LB_MORPH_CLOSE_SIZE),
                                 std::max(1, LB_MORPH_CLOSE_SIZE)));
    const cv::Mat kernel_open = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(std::max(1, LB_MORPH_OPEN_SIZE),
                                 std::max(1, LB_MORPH_OPEN_SIZE)));

    std::vector<LightBar> bars;
    for (int c = 0; c < 2; ++c) {
        cv::Mat bin = masks[c].clone();
        if (do_close) cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel_close);
        if (do_open)  cv::morphologyEx(bin, bin, cv::MORPH_OPEN,  kernel_open);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& ct : contours) {
            if (cv::contourArea(ct) < LB_MIN_CONTOUR_AREA) continue;

            // BGR 纯度二级过滤: 灯条区域的目标色通道均值须显著占优,
            // 用来滤掉灰蓝/灰红这种"色相沾边但不饱和"的背景
            {
                cv::Mat mask = cv::Mat::zeros(bin.size(), CV_8U);
                std::vector<std::vector<cv::Point>> one{ct};
                cv::drawContours(mask, one, -1, cv::Scalar(255), cv::FILLED);
                const double mB = cv::mean(bgr[0], mask)[0];
                const double mG = cv::mean(bgr[1], mask)[0];
                const double mR = cv::mean(bgr[2], mask)[0];
                if (c == 0 && !(mB > mG + LB_PURITY && mB > mR + LB_PURITY)) continue;
                if (c == 1 && !(mR > mG + LB_PURITY && mR > mB + LB_PURITY)) continue;
            }

            const cv::RotatedRect rr = cv::minAreaRect(ct);
            const float L = std::max(rr.size.width, rr.size.height);
            const float s = std::min(rr.size.width, rr.size.height);
            if (s < LB_SHORT_MIN || L < LB_LONG_MIN) continue;
            const float ratio = L / std::max(s, 1e-3f);
            if (ratio < LB_RATIO_MIN || ratio > LB_RATIO_MAX) continue;

            LightBar lb;
            lb.center = cv::Point2f(rr.center.x, rr.center.y);
            lb.length = L;
            lb.width  = s;
            lb.dir    = lightbar_dir(rr);
            lb.color  = c;
            lb.rr     = rr;
            bars.push_back(lb);
        }
    }

    // 候选截断: 只留面积最大的若干个, 防配对 O(n^2) 把 CPU 吃满
    if (static_cast<int>(bars.size()) > LB_MAX_CANDIDATES) {
        std::partial_sort(bars.begin(), bars.begin() + LB_MAX_CANDIDATES, bars.end(),
                          [](const LightBar& a, const LightBar& b) {
                              return a.length * a.width > b.length * b.width;
                          });
        bars.resize(LB_MAX_CANDIDATES);
    }
    return bars;
}

// ---------------------------------------------------------------- 角点排序
// 输出顺序固定为 左上(0) / 右上(1) / 右下(2) / 左下(3), 必须与
// armor_object_points() 的 (-hw,-hh) (hw,-hh) (hw,hh) (-hw,hh) 一一对应。
// 图像 y 向下, 所以物点 y 最小的那一对在上边。
//
// 做法(稳健, 不依赖输入顺序):
//   1) 按绕质心的极角排序 -> 得到一个凸的、不自交的环(顺时针或逆时针)
//   2) 检查有向面积, 统一成图像坐标系下的顺时针(左上->右上->右下->左下)
//   3) 旋转环, 让"左上角"(x+y 最小) 排到下标 0
//
// 踩过的坑: 直接用 (x+y) 排序再把中间两点填进 1/2 号位, 会把顺序变成
// 左上/左下/右上/右下, 画出来是个"蝴蝶结", PnP 直接给出错误位姿。
inline std::vector<cv::Point2f> sort_armor_points(std::vector<cv::Point2f> p) {
    if (p.size() != 4) return p;

    cv::Point2f c(0.0f, 0.0f);
    for (const auto& q : p) c += q;
    c *= 0.25f;

    // 1) 按极角排序成环
    std::sort(p.begin(), p.end(), [&c](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
    });

    // 2) 统一绕向: 图像坐标(y 向下)里, 左上->右上->右下->左下 的鞋带面积为正
    double area2 = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2f& a = p[i];
        const cv::Point2f& b = p[(i + 1) % 4];
        area2 += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    if (area2 < 0.0) std::reverse(p.begin(), p.end());

    // 3) 旋转环, 让 x+y 最小的点(左上)落在下标 0
    int start = 0;
    float best = p[0].x + p[0].y;
    for (int i = 1; i < 4; ++i) {
        const float s = p[i].x + p[i].y;
        if (s < best) {
            best = s;
            start = i;
        }
    }
    std::vector<cv::Point2f> out(4);
    for (int i = 0; i < 4; ++i) out[i] = p[(start + i) % 4];
    return out;
}

// 装甲板中心区域应为暗色(真板两灯之间是黑底)。
// 太靠边或太小的区域无法判断, 直接放行, 避免漏检。
inline bool armor_center_dark(const cv::Mat& hsv_v, const cv::Point2f& c,
                              float half_w, float half_h) {
    cv::Rect r(cvRound(c.x - half_w), cvRound(c.y - half_h),
               cvRound(2 * half_w), cvRound(2 * half_h));
    r &= cv::Rect(0, 0, hsv_v.cols, hsv_v.rows);
    if (r.width < 2 || r.height < 2) return true;
    return cv::mean(hsv_v(r))[0] < ARMOR_CENTER_V_MAX;
}

// ---------------------------------------------------------------- 灯条配对
// hsv_v: HSV 的 V 通道(可为空, 为空则跳过中心暗区校验)
inline std::vector<Armor> match_armors(const std::vector<LightBar>& bars,
                                       const cv::Mat& hsv_v) {
    std::vector<Armor> armors;
    for (size_t i = 0; i < bars.size(); ++i) {
        for (size_t j = i + 1; j < bars.size(); ++j) {
            const LightBar& a = bars[i];
            const LightBar& b = bars[j];
            if (a.color != b.color) continue;                                  // 同色

            const float hmax = std::max(a.length, b.length);
            const float hmin = std::min(a.length, b.length);
            if (hmin / std::max(hmax, 1e-3f) < PAIR_LEN_RATIO_MIN) continue;   // 等高

            const float dy = std::abs(a.center.y - b.center.y);
            if (dy / hmax > PAIR_DY_RATIO_MAX) continue;                       // y 对齐

            const float dx  = std::abs(a.center.x - b.center.x);
            const float dxr = dx / hmax;
            if (dxr < PAIR_DIST_RATIO_MIN || dxr > PAIR_DIST_RATIO_MAX) continue;

            float ad = std::abs(a.dir - b.dir);
            if (ad > 90.0f) ad = 180.0f - ad;
            if (ad > PAIR_ANGLE_DIFF) continue;                                // 平行

            const cv::Point2f c = (a.center + b.center) * 0.5f;
            if (!hsv_v.empty() &&
                !armor_center_dark(hsv_v, c, dx * 0.35f, hmax * 0.5f)) continue;

            // **两根灯条之间不允许再夹着别的同色灯条。**
            //
            // 这条是几何无关的强判据, 专治跨板误配:
            //   真板的两根灯条就是这块板自己的左右边缘, 中间是暗板面, 不可能再夹灯条;
            //   而把 A 板的灯条和 B 板的灯条配起来时, 中间往往还夹着别的灯条。
            //
            // 实测(10-16)踩的坑正是这个: 我把 x=579 和 x=752 配成了一块"板"
            // (aspect 4.02), 而真正夹着白色数字"2"的那对是 x=579 和 x=659 ——
            // 也就是 x=659 正好落在 579 和 752 中间。加上本条这种配对直接被否掉。
            // 这条比宽高比可靠: 它不依赖"灯条是否占满板高"这个未必成立的假设。
            {
                const float lo = std::min(a.center.x, b.center.x);
                const float hi = std::max(a.center.x, b.center.x);
                bool blocked = false;
                for (size_t k = 0; k < bars.size(); ++k) {
                    if (&bars[k] == &a || &bars[k] == &b) continue;
                    if (bars[k].color != a.color) continue;
                    const float bx = bars[k].center.x;
                    if (bx > lo + 2.0f && bx < hi - 2.0f) { blocked = true; break; }
                }
                if (blocked) continue;
            }

            // 宽高比约束 —— 拦明显不像板的形状。
            // 注意"板高"这里是用**灯条长度**近似的, 只在灯条接近满板高时才准。
            // 实测该靶标的灯条比板高短不少(灯条 31px 而板高约 45px+), 所以上下限
            // 只能取得比较宽, 抓跨板主要靠上面那条"中间不能夹灯条"。
            const float half_h = hmax * 0.5f;
            const float half_w = dx * 0.5f + std::max(a.width, b.width) * 0.5f;
            const float aspect = (2.0f * half_w) / std::max(2.0f * half_h, 1e-3f);
            if (aspect < PAIR_ASPECT_MIN || aspect > PAIR_ASPECT_MAX) continue;

            // 构造装甲板四点: 中心 + 两灯条平均方向
            Armor ar;
            ar.color  = a.color;
            ar.center = c;
            const float dir_rad = (a.dir + b.dir) * 0.5f * static_cast<float>(CV_PI) / 180.0f;
            const cv::Point2f u(std::cos(dir_rad), std::sin(dir_rad));  // 沿灯条(竖直)
            const cv::Point2f v(-u.y, u.x);                             // 垂直(水平)
            std::vector<cv::Point2f> pts{
                c - v * half_w - u * half_h,   // 左上
                c + v * half_w - u * half_h,   // 右上
                c + v * half_w + u * half_h,   // 右下
                c - v * half_w + u * half_h,   // 左下
            };
            ar.pts    = sort_armor_points(pts);
            ar.width  = 2.0f * half_w;
            ar.height = 2.0f * half_h;
            armors.push_back(ar);
        }
    }
    return armors;
}

// 便捷函数: 从 BGR 图里取 HSV 的 V 通道(调用方在主循环里只需算一次)
inline cv::Mat hsv_value_channel(const cv::Mat& frame) {
    cv::Mat hsv, ch[3];
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::split(hsv, ch);
    return ch[2];
}

// ---------------------------------------------------------------- 调试绘制
inline void draw_debug(cv::Mat& frame, const std::vector<LightBar>& bars,
                       const std::vector<Armor>& armors) {
    for (const auto& lb : bars) {
        cv::Point2f p[4];
        lb.rr.points(p);
        // 灯条框按**判定出来的颜色**上色: 蓝灯条画蓝框, 红灯条画红框。
        // 顺手在框边标字母 B/R —— 截图压缩、投影仪偏色时颜色可能看不准,
        // 但字母不会歧义。
        const cv::Scalar col = (lb.color == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        for (int i = 0; i < 4; ++i) cv::line(frame, p[i], p[(i + 1) % 4], col, 1);
        cv::putText(frame, lb.color == 0 ? "B" : "R",
                    lb.center + cv::Point2f(-4.0f, -lb.length * 0.5f - 3.0f),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, col, 1);
    }
    // 装甲板框改用**绿色**, 不再用黄色。
    // 原来画黄色, 结果"配成的一块板"看起来像"把蓝色灯条误判成了橙黄色", 极易误读
    // (实际使用者反馈过一次)。绿框 + 蓝/红灯条框, 三色互不混淆;
    // 框内再写 BLUE/RED, 直接读出判定结果。
    for (const auto& ar : armors) {
        const cv::Scalar box_col(0, 255, 0);
        for (int i = 0; i < 4; ++i)
            cv::line(frame, ar.pts[i], ar.pts[(i + 1) % 4], box_col, 2);
        cv::putText(frame, ar.color == 0 ? "BLUE" : "RED",
                    ar.center + cv::Point2f(-18.0f, 5.0f),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, box_col, 1);
    }
}

}  // namespace srm

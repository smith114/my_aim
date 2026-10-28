// select.hpp: 从多个候选里选"打哪一块" —— **唯一实现, 主程序与 GUI 共用**
//
// 为什么必须共用:
//   之前迟滞逻辑写在 main.cpp 里, aim_gui.cpp 自己另写了一套简单选择。
//   结果操作手在 GUI 上看到的是"两块板左右来回跳"(没有迟滞), 而在主程序里
//   已经修好了 —— 两份实现分叉, 调试看到的现象和实车行为不一致。
//
// 问题本身: 旋转靶标在某些角度会**同时看到两块板**, 于是选点在两块之间跳,
//   云台就左右飘。单纯"取离画面中心最近的"必然抖 —— 两块大致对称时距离几乎
//   相等, 噪声就决定了选谁。
//
// 三条规则:
//   1) 板面朝向: 选 |yaw_raw| 更小的 —— 它更正对相机, 有效命中面积最大。
//      这是物理判据, 不是启发式。
//   2) 迟滞: 还能看到原来那块就不换。
//   3) 换目标要够大的改善(kSwitchMargin), 避免在临界点抖动。
#pragma once

#include <cmath>
#include <vector>

#include "config.hpp"
#include "tracker.hpp"

namespace srm {

class PlateSelector {
public:
    // 返回 candidates 里的下标, 无候选返回 -1
    int pick(const std::vector<Target>& cand) {
        if (cand.empty()) {
            has_prev_ = false;
            return -1;
        }
        int   best_i = -1, prev_i = -1;
        float best_c = 1e9f, prev_c = 1e9f;

        for (size_t i = 0; i < cand.size(); ++i) {
            const float facing = std::fabs(cand[i].solve.yaw_raw) * 57.29578f;
            const float cont   = std::fabs(cand[i].solve.yaw - last_yaw_);
            const float cost   = facing + 0.5f * cont;
            if (cost < best_c) { best_c = cost; best_i = static_cast<int>(i); }
            if (has_prev_ && cont < SELECT_SAME_TARGET_DEG && cost < prev_c) {
                prev_c = cost; prev_i = static_cast<int>(i);
            }
        }

        int pick_i = best_i;
        // 迟滞: 原来那块还在, 且新最优没有明显更好 -> 不换
        if (prev_i >= 0 && best_c * kSwitchMargin > prev_c) pick_i = prev_i;

        if (pick_i >= 0) {
            last_yaw_  = cand[pick_i].solve.yaw;
            has_prev_  = true;
        }
        return pick_i;
    }

    void reset() { has_prev_ = false; last_yaw_ = 0.0f; }
    int  last_index() const { return last_pick_; }

private:
    static constexpr float kSwitchMargin = 1.35f;
    bool  has_prev_ = false;
    float last_yaw_ = 0.0f;
    int   last_pick_ = -1;
};

}  // namespace srm

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

        // ******** 连续性权重远大于板面朝向 ********
        //
        // 实测(10-29): theta 逐帧在 -3.3 和 +3.8 之间跳(7 度), 而 5m 处板只有
        // 1.5 度宽 —— 那不是噪声, 是**选中的板在换**。后果很严重:
        // 卡尔曼看到 7 度的新息, 角速度估计直接炸掉(-500 度/秒),
        // 提前量正负翻转, 命令来回摆 -> 抽动。
        //
        // 所以: 连续性几乎决定一切(权重 1.0), 板面朝向只作为次要打破平局(0.15)。
        for (size_t i = 0; i < cand.size(); ++i) {
            const float facing = std::fabs(cand[i].solve.yaw_raw) * 57.29578f;
            const float cont   = std::fabs(cand[i].solve.yaw - last_yaw_);
            const float cost   = 0.15f * facing + 1.0f * cont;
            if (cost < best_c) { best_c = cost; best_i = static_cast<int>(i); }
            if (has_prev_ && cont < SELECT_SAME_TARGET_DEG && cost < prev_c) {
                prev_c = cost; prev_i = static_cast<int>(i);
            }
        }

        int pick_i = best_i;
        if (prev_i >= 0) {
            miss_cnt_ = 0;                       // 旧板还在
            if (best_c * kSwitchMargin > prev_c) pick_i = prev_i;   // 迟滞
        } else {
            ++miss_cnt_;                         // 旧板这一帧没配上
        }

        // ******** 永远返回一块能用的板, 绝不返回 -1 ********
        //
        // 曾经在这里写成"旧板没配上就 return -1, 让它等几帧别乱切"。
        // 那是错的: 返回 -1 在 main 里等于"这一帧没有目标" -> stop_tracking(),
        // 于是**明明画面里有板却被丢弃**。实测(10-29): armor 频繁落到 0,
        // locked 一直在 0.0~0.4, 永远开不了火。
        //
        // 正确含义: "换一块板"是正常行为, 不是"没有目标"。
        // 抑制切换应该靠上面的迟滞和换门限, 而不是靠丢弃检测。
        if (pick_i >= 0) {
            last_yaw_  = cand[pick_i].solve.yaw;
            has_prev_  = true;
        }
        return pick_i;
    }

    void reset() { has_prev_ = false; last_yaw_ = 0.0f; miss_cnt_ = 0; }
    int  last_index() const { return last_pick_; }

private:
    static constexpr float kSwitchMargin = 2.5f;   // 新目标要好几倍才换
    // 旧板必须连续这么多帧一直看不到, 才允许换成别的板。
    // 只是"这一帧没配上"不算消失 —— 否则两块板交替可见时会来回切。
    static constexpr int   kSwitchHoldFrames = 3;   // 仅用于统计, 不再丢弃目标
    bool  has_prev_ = false;
    int   miss_cnt_ = 0;
    float last_yaw_ = 0.0f;
    int   last_pick_ = -1;
};

}  // namespace srm

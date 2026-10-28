// fire.hpp: 开火判据(纯数学, 可离线测试)
//
// 规则约束: 上位机只能通过 ID2 的 fire_flag 表达"建议开火";
// **人工发射许可由鼠标左键给出, 由下位机实现**。所以这里是建议, 不是决定。
//
// 核心是容差必须**随距离缩放**: 同一个角度误差在不同距离上含义完全不同。
// 装甲板 140x125mm, 角宽度 = 2*atan(70mm / dist)。
//   2m -> 4.0 度
//   5m -> 1.6 度
//   8m -> 1.0 度
// 取 0.6 倍打板中央, 留余量吸收预测误差与散布。
#pragma once

#include <cmath>

#include "config.hpp"

namespace srm {

// 装甲板在当前距离下的角宽度(度)
inline float plate_angular_width_deg(float dist_m) {
    if (!(dist_m > 0.05f)) return 0.0f;
    const float half = static_cast<float>(
        std::atan2(static_cast<double>(ARMOR_W) * 0.5 * 0.001, static_cast<double>(dist_m)));
    return 2.0f * half * 57.29578f;
}

// 允许开火的角度容差(度)
inline float fire_tolerance_deg(float dist_m) {
    return FIRE_TOL_FRAC * plate_angular_width_deg(dist_m);
}

// 综合判据。返回 true 表示可以建议开火。
// 每个条件都是"没有它就可能打错":
inline bool fire_decision(bool enable, bool locked, bool vision_on, bool fb_fresh,
                          bool has_target, float dist_m, float theta_yaw_deg,
                          float theta_pitch_deg, float bullet_speed, int lock_streak) {
    if (!enable || !locked || !vision_on || !fb_fresh || !has_target) return false;
    if (!(dist_m > 0.5f) || dist_m > FIRE_MAX_DIST_M) return false;
    if (!(bullet_speed > FIRE_MIN_BULLET_SPEED)) return false;   // 弹道补偿会算错
    if (lock_streak < FIRE_LOCK_FRAMES) return false;            // 单帧误检不触发
    const float err = std::sqrt(theta_yaw_deg * theta_yaw_deg +
                               theta_pitch_deg * theta_pitch_deg);
    return err <= fire_tolerance_deg(dist_m);
}

}  // namespace srm

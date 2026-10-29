// kf.hpp: 两状态匀速卡尔曼滤波 (角度 + 角速度), 用于**延迟补偿**
//
// 为什么需要它:
//   整条链路的延迟(曝光+取流+推理+PnP+串口+下位机执行)约 60~100 ms。
//   目标以 0.4 rad/s 横移时, 5 m 距离上 100 ms 就是 2.3 度滞后,
//   而 5 m 处 140mm 装甲板只有 1.6 度宽 —— 不补偿必打偏。
//
// 为什么只做"角度+角速度"两状态, 不做完整 3D EKF:
//   · 我们只需要外推几十毫秒, 局域网内目标运动近似匀速
//   · 两状态好验证、好调、不易发散; 完整 EKF 需要目标尺寸/旋转模型
//   · 3 天时间内, 能验证的东西优先
//
// 约定:
//   dt 用秒。predict() 每帧调用一次, update() 每帧有新测量时调用一次。
//   predict_ahead(L) 给出"L 秒之后"的角度估计 —— 这就是要下发的瞄准角。
#pragma once

#include <cmath>

#include "config.hpp"

namespace srm {

class AngleKF {
public:
    // R: 测量噪声(度^2)。由静止靶标时 theta 的抖动实测得到。
    // Q: 过程噪声。q_angle 影响"信任测量", q_rate 影响"信任匀速假设"。
    AngleKF(float q_angle = 1e-2f, float q_rate = 1e1f, float r = 0.5f)
        : q_angle_(q_angle), q_rate_(q_rate), r_(r) {}

    void set_noise(float q_angle, float q_rate, float r) {
        q_angle_ = q_angle;
        q_rate_  = q_rate;
        r_       = r;
    }

    void reset() {
        theta_ = 0.0f;
        omega_ = 0.0f;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) P_[i][j] = 0.0f;
        P_[0][0] = 10.0f;   // 初始不知道位置
        P_[1][1] = 100.0f;  // 更不知道速度
        inited_ = false;
        frames_ = 0;
        rejected_ = 0;
    }

    bool inited() const { return inited_; }
    int  frames() const { return frames_; }
    float theta() const { return theta_; }
    float omega() const { return omega_; }

    // 状态预测。dt 秒
    void predict(float dt) {
        if (dt <= 0.0f) return;
        theta_ += omega_ * dt;

        // P = F P F^T + Q,  F = [[1,dt],[0,1]]
        const float p00 = P_[0][0] + dt * (P_[1][0] + P_[0][1]) + dt * dt * P_[1][1];
        const float p01 = P_[0][1] + dt * P_[1][1];
        const float p10 = P_[1][0] + dt * P_[1][1];
        const float p11 = P_[1][1];
        P_[0][0] = p00 + q_angle_ * dt;
        P_[0][1] = p01;
        P_[1][0] = p10;
        P_[1][1] = p11 + q_rate_ * dt;
    }

    // 用测量值修正。z 度
    void update(float z) {
        if (!inited_) {
            // 第一次测量直接初始化, 省掉收敛过程
            theta_  = z;
            omega_  = 0.0f;
            inited_ = true;
            frames_ = 1;
            return;
        }
        ++frames_;
        const float y  = z - theta_;          // 新息

        // **新息门限**: 单帧测量偏离预测超过 kGateDeg 就判为离群并丢弃。
        // 为什么必须有: 卡尔曼对"离谱测量"并不会自动免疫 —— 若 P00 较大,
        // 增益接近 1, 一帧 200 度的误检会把 theta 直接拽到 200 度, 然后 omega
        // 被带偏, 预测立刻飞掉。实测(10-29) 单帧 200 度离群造成 26.9 度跳变。
        // 物理上也不可能有这么大的单帧变化: 60fps 下 90 度/秒 才 1.5 度/帧。
        if (std::fabs(y) > kGateDeg) {
            ++rejected_;
            return;
        }
        const float S  = P_[0][0] + r_;       // 新息方差
        if (S <= 0.0f) return;
        const float k0 = P_[0][0] / S;
        const float k1 = P_[1][0] / S;

        theta_ += k0 * y;
        omega_ += k1 * y;

        const float n00 = (1.0f - k0) * P_[0][0];
        const float n01 = (1.0f - k0) * P_[0][1];
        const float n10 = P_[1][0] - k1 * P_[0][0];
        const float n11 = P_[1][1] - k1 * P_[0][1];
        P_[0][0] = n00; P_[0][1] = n01;
        P_[1][0] = n10; P_[1][1] = n11;

        // 速度限幅: 防止单帧误检把角速度推到荒谬值
        if (omega_ >  kMaxRate) omega_ =  kMaxRate;
        if (omega_ < -kMaxRate) omega_ = -kMaxRate;
    }

    // 外推 L 秒后的角度 —— 这就是延迟补偿的输出
    float predict_ahead(float L) const {
        float a = theta_ + omega_ * L;
        // 外推限幅: 速度已知不可靠时不要把角度推飞
        const float cap = kMaxLeadDeg;
        if (a >  theta_ + cap) a = theta_ + cap;
        if (a <  theta_ - cap) a = theta_ - cap;
        return a;
    }

    // 收敛程度: 角速度估计是否可信(用于决定能不能开火)
    bool rate_trusted() const { return frames_ >= kTrustFrames; }
    int  rejected() const { return rejected_; }   // 被门限丢掉的离群帧数

private:
    float theta_ = 0.0f;
    float omega_ = 0.0f;
    float P_[2][2] = {{10.0f, 0.0f}, {0.0f, 100.0f}};
    float q_angle_ = 1e-2f;
    float q_rate_  = 1e1f;
    float r_       = 0.5f;
    bool  inited_  = false;
    int   frames_  = 0;
    int   rejected_ = 0;

    static constexpr float kMaxRate     = 90.0f;  // 度/秒, 超过就不像是装甲板
    // 单次外推上限, 由 config 决定(旋转靶需要更大提前量)
    static constexpr float kMaxLeadDeg  = KF_MAX_LEAD_DEG;
    static constexpr int   kTrustFrames = 5;      // 至少 5 帧后才信任速度
    static constexpr float kGateDeg     = 12.0f;  // 新息门限(度): 超过判为离群
};

}  // namespace srm

// kf3d.hpp: 目标三维位置/速度卡尔曼滤波 (恒定速度模型), 用于**延迟 + 弹丸飞行补偿**
//
// 为什么必须是三维, 而不是两轴角度:
//
//   之前那版只对 (yaw, pitch) 两个角度做滤波。它能补链路延迟, 但补不了
//   弹丸飞行时间带来的偏差 —— 因为飞行期间目标的**距离也在变**,
//   而角度的变化率依赖距离(角速度 = 横向速度 / 距离)。只滤角度等于假设
//   距离不变, 近距/远距都会算错提前量。
//
//   进三维后可以:
//     · 预测"子弹到达时刻"目标的三维位置 (位置 + 速度*T)
//     · 由预测位置反算 yaw/pitch, 深度变化自动包含在内
//     · 弹道下坠用预测出的距离算, 而不是当前距离
//
// 关于"是不是 EKF":
//   恒定速度模型下状态转移与观测都是线性的, 所以这一层数学上就是**线性卡尔曼**,
//   EKF 与它完全等价 —— 写成 EKF 只是形式, 不会多出任何精度。
//   EKF 真正有价值的是**状态含板面朝向**的时候(小陀螺): 那时观测模型是
//   非线性的(角度->观测), 必须线性化。那一步需要目标旋转模型, 是下一步。
//
// 坐标系: 相机系, 单位 mm (与 PnP 的 tvec 一致), 速度 mm/s。
#pragma once

#include <cmath>

namespace srm {

class CVKalman3D {
public:
    // acc_sigma: 目标加速度标准差(mm/s^2), 决定"多信任匀速假设"
    // meas_sigma: PnP 位置测量噪声(mm), 由静止靶标时 tvec 的抖动实测
    CVKalman3D(float acc_sigma = 800.0f, float meas_sigma = 30.0f)
        : acc_sigma_(acc_sigma), meas_sigma_(meas_sigma) { reset(); }

    void reset() {
        for (int i = 0; i < 6; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 6; ++j) P_[i][j] = 0.0f;
        }
        // 初始不确定度: 位置大(不知道在哪), 速度更大
        for (int i = 0; i < 3; ++i) P_[i][i]       = 1e6f;
        for (int i = 3; i < 6; ++i) P_[i][i]       = 1e8f;
        inited_ = false;
        frames_ = 0;
        rejected_ = 0;
    }

    bool inited() const { return inited_; }
    int  frames() const { return frames_; }
    int  rejected() const { return rejected_; }

    // 当前位置(mm)与速度(mm/s)
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float pz() const { return x_[2]; }
    float vx() const { return x_[3]; }
    float vy() const { return x_[4]; }
    float vz() const { return x_[5]; }
    float speed() const {
        return std::sqrt(x_[3] * x_[3] + x_[4] * x_[4] + x_[5] * x_[5]);
    }

    // 状态预测。dt 秒
    void predict(float dt) {
        if (dt <= 0.0f) return;
        // x = F x,  F = [[I, dt*I],[0, I]]
        for (int i = 0; i < 3; ++i) x_[i] += x_[i + 3] * dt;

        // P = F P F^T + Q
        // 先算 FP 存到临时
        float FP[6][6];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                FP[i][j] = P_[i][j];
                if (i < 3) FP[i][j] += dt * P_[i + 3][j];
            }
        // 再乘 F^T
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                P_[i][j] = FP[i][j];
                if (j < 3) P_[i][j] += dt * FP[i][j + 3];
            }
        // 过程噪声: 加速度模型, 位置项 dt^4/4, 耦合 dt^3/2, 速度项 dt^2
        const float a2 = acc_sigma_ * acc_sigma_;
        const float q11 = a2 * dt * dt * dt * dt * 0.25f;
        const float q12 = a2 * dt * dt * dt * 0.5f;
        const float q22 = a2 * dt * dt;
        for (int i = 0; i < 3; ++i) {
            P_[i][i]         += q11;
            P_[i][i + 3]     += q12;
            P_[i + 3][i]     += q12;
            P_[i + 3][i + 3] += q22;
        }
    }

    // 用 PnP 位置(mm)修正
    void update(float mx, float my, float mz) {
        if (!inited_) {
            x_[0] = mx; x_[1] = my; x_[2] = mz;
            x_[3] = x_[4] = x_[5] = 0.0f;
            const float r = meas_sigma_ * meas_sigma_;
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j) P_[i][j] = 0.0f;
            for (int i = 0; i < 3; ++i) P_[i][i] = r;
            for (int i = 3; i < 6; ++i) P_[i][i] = 1e8f;   // 速度还未知
            inited_ = true;
            frames_ = 1;
            return;
        }
        ++frames_;

        const float m[3] = {mx, my, mz};
        const float r   = meas_sigma_ * meas_sigma_;

        // 新息 y = z - H x, H = [I 0]
        float y[3];
        for (int i = 0; i < 3; ++i) y[i] = m[i] - x_[i];

        // **新息门限**: 位置跳变超过 gate 视为离群(误检/换目标)
        // 门限按"期望单帧位移 + 3sigma"给出, 避免硬编码
        const float expect = (speed() * 0.1f) + 3.0f * meas_sigma_ + 200.0f;
        const float ynorm  = std::sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
        if (ynorm > expect) { ++rejected_; return; }

        // S = H P H^T + R (3x3, 只用 P 的左上 3x3)
        float S[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) S[i][j] = P_[i][j];
        for (int i = 0; i < 3; ++i) S[i][i] += r;

        // S 求逆 (解析 3x3)
        const float a = S[0][0], b = S[0][1], c = S[0][2];
        const float d = S[1][0], e = S[1][1], f = S[1][2];
        const float g = S[2][0], h = S[2][1], i2 = S[2][2];
        const float det = a * (e * i2 - f * h) - b * (d * i2 - f * g) + c * (d * h - e * g);
        if (std::fabs(det) < 1e-12f) return;
        const float inv = 1.0f / det;
        float Si[3][3];
        Si[0][0] =  (e * i2 - f * h) * inv;
        Si[0][1] = -(b * i2 - c * h) * inv;
        Si[0][2] =  (b * f - c * e) * inv;
        Si[1][0] = -(d * i2 - f * g) * inv;
        Si[1][1] =  (a * i2 - c * g) * inv;
        Si[1][2] = -(a * f - c * d) * inv;
        Si[2][0] =  (d * h - e * g) * inv;
        Si[2][1] = -(a * h - b * g) * inv;
        Si[2][2] =  (a * e - b * d) * inv;

        // K = P H^T S^-1 ; H^T = [I;0] 所以 K 前 3 行 = P[0..2][0..2] * Si, 后 3 行同理
        float K[6][3];
        for (int r2 = 0; r2 < 6; ++r2)
            for (int c2 = 0; c2 < 3; ++c2) {
                float s = 0.0f;
                for (int k = 0; k < 3; ++k) s += P_[r2][k] * Si[k][c2];
                K[r2][c2] = s;
            }

        // x += K y
        for (int r2 = 0; r2 < 6; ++r2) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += K[r2][k] * y[k];
            x_[r2] += s;
        }

        // P -= K H P  (H P = P 的前 3 行)
        float KH[6][6];
        for (int r2 = 0; r2 < 6; ++r2)
            for (int c2 = 0; c2 < 6; ++c2) {
                if (c2 < 3) {
                    KH[r2][c2] = K[r2][c2];
                } else {
                    KH[r2][c2] = 0.0f;
                }
            }
        float KP[6][6];
        for (int r2 = 0; r2 < 6; ++r2)
            for (int c2 = 0; c2 < 6; ++c2) {
                float s = 0.0f;
                for (int k = 0; k < 6; ++k) s += KH[r2][k] * P_[k][c2];
                KP[r2][c2] = s;
            }
        for (int r2 = 0; r2 < 6; ++r2)
            for (int c2 = 0; c2 < 6; ++c2) P_[r2][c2] -= KP[r2][c2];

        // 速度限幅: 8 m/s 以上不像装甲板
        const float sp = speed();
        if (sp > kMaxSpeed) {
            const float s = kMaxSpeed / sp;
            x_[3] *= s; x_[4] *= s; x_[5] *= s;
        }
    }

    // 外推 T 秒后的位置(mm)。用于"子弹到达时刻"的瞄准点。
    void predict_pos(float T, float& ox, float& oy, float& oz) const {
        ox = x_[0] + x_[3] * T;
        oy = x_[1] + x_[4] * T;
        oz = x_[2] + x_[5] * T;
    }

    static constexpr float kMaxSpeed = 8000.0f;   // mm/s

private:
    float x_[6] = {0, 0, 0, 0, 0, 0};
    float P_[6][6] = {};
    float acc_sigma_  = 800.0f;
    float meas_sigma_ = 30.0f;
    bool  inited_ = false;
    int   frames_ = 0;
    int   rejected_ = 0;
};

}  // namespace srm

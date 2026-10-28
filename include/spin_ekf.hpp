// spin_ekf.hpp: **旋转目标(小陀螺)专用 EKF**
//
// 为什么必须用 EKF, 不能用匀速卡尔曼:
//
//   小陀螺的装甲板绕自转轴做**圆周运动**。观测到的是"板此刻在哪",
//   而它是转子相位 φ 的非线性函数:
//
//       板位置 = (x + R·sinφ,  z + R·cosφ)
//
//   用匀速模型去拟, 等于把圆弧当直线 —— 预测点会系统性偏到弧外,
//   转速越高偏得越多。而观测对 φ 是非线性的, 所以必须线性化 -> EKF。
//
// 状态(5 维, 水平面):
//   [0] x     自转轴在相机系的 x (mm)
//   [1] z     自转轴在相机系的 z (mm)
//   [2] R     板中心到自转轴的距离(mm), 约等于板半宽+安装偏移
//   [3] phi   转子相位(rad), 0 表示板正对相机
//   [4] omega 转速(rad/s), 符号表示旋向
//
// 观测(2 维): 板中心 (mx, mz)
//   h(x) = [ x_axis + R·sinφ ,  z_axis + R·cosφ ]
//   H    = [ 1, 0, sinφ,  R·cosφ, 0 ]
//          [ 0, 1, cosφ, -R·sinφ, 0 ]
//
// 注意: 转速高时板只在部分相位可见 -> 观测是**间歇**的。
// EKF 在无观测帧只做 predict, 这正是它比"按帧算角度"强的地方。
#pragma once

#include <cmath>

namespace srm {

class SpinEKF {
public:
    // acc: 轴心/半径的过程噪声; q_omega: 转速过程噪声(转速会变)
    // meas: 板位置测量噪声(mm)
    SpinEKF(float acc = 600.0f, float q_radius = 50.0f, float q_omega = 20.0f,
            float meas = 40.0f)
        : acc_(acc), q_r_(q_radius), q_w_(q_omega), meas_(meas) { reset(); }

    void reset() {
        for (int i = 0; i < 5; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 5; ++j) P_[i][j] = 0.0f;
        }
        P_[0][0] = 1e6f; P_[1][1] = 1e6f;
        P_[2][2] = 1e4f; P_[3][3] = 4.0f; P_[4][4] = 400.0f;
        inited_ = false;
        frames_ = 0;
        rejected_ = 0;
    }

    bool  inited() const { return inited_; }
    int   frames() const { return frames_; }
    int   rejected() const { return rejected_; }
    float x_axis() const { return x_[0]; }
    float z_axis() const { return x_[1]; }
    float radius() const { return x_[2]; }
    float phase() const { return x_[3]; }
    float omega() const { return x_[4]; }          // rad/s
    float omega_dps() const { return x_[4] * 57.29578f; }

    // 板当前在哪儿(由状态算, 不依赖本帧是否有观测)
    void plate_pos(float& px, float& pz) const {
        px = x_[0] + x_[2] * std::sin(x_[3]);
        pz = x_[1] + x_[2] * std::cos(x_[3]);
    }

    // 外推 T 秒后板的位置。这是"子弹到达时刻"的瞄准点。
    void plate_pos_ahead(float T, float& px, float& pz) const {
        const float ph = x_[3] + x_[4] * T;
        px = x_[0] + x_[2] * std::sin(ph);
        pz = x_[1] + x_[2] * std::cos(ph);
    }

    // 外推 T 秒后的相位是否"面向相机"(|φ| 小 -> 板面正对)
    float phase_ahead(float T) const { return x_[3] + x_[4] * T; }

    void predict(float dt) {
        if (dt <= 0.0f) return;
        // 相位随转速前进; 轴心/半径/转速按随机游走
        x_[3] += x_[4] * dt;

        // P = F P F^T + Q, F = I 除 (3,4)=dt
        float FP[5][5];
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j) {
                FP[i][j] = P_[i][j];
                if (i == 3) FP[i][j] += dt * P_[4][j];
            }
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j) {
                P_[i][j] = FP[i][j];
                if (j == 3) P_[i][j] += dt * FP[i][4];
            }
        P_[0][0] += acc_ * acc_ * dt;
        P_[1][1] += acc_ * acc_ * dt;
        P_[2][2] += q_r_ * q_r_ * dt;
        P_[3][3] += P_[4][4] * dt * dt + 1e-4f;
        P_[4][4] += q_w_ * q_w_ * dt;
    }

    // 用观测到的板位置修正。返回 false 表示被门限拒绝
    bool update(float mx, float mz) {
        const float s = std::sin(x_[3]);
        const float c = std::cos(x_[3]);

        if (!inited_) {
            // 首次观测: 只能初始化, 轴心未知 -> 先假定轴心=板位置, 半径=0
            x_[0] = mx; x_[1] = mz; x_[2] = 0.0f;
            x_[3] = 0.0f; x_[4] = 0.0f;
            const float r = meas_ * meas_;
            for (int i = 0; i < 5; ++i)
                for (int j = 0; j < 5; ++j) P_[i][j] = 0.0f;
            P_[0][0] = r; P_[1][1] = r;
            P_[2][2] = 1e4f; P_[3][3] = 4.0f; P_[4][4] = 400.0f;
            inited_ = true;
            frames_ = 1;
            return true;
        }
        ++frames_;

        // 观测预测
        const float hx = x_[0] + x_[2] * s;
        const float hz = x_[1] + x_[2] * c;
        float y[2] = {mx - hx, mz - hz};

        // 新息门限(按半径与转速估计"单帧最大合理位移")
        const float expect = std::fabs(x_[4]) * 0.1f * std::fabs(x_[2]) + 3.0f * meas_ + 150.0f;
        if (std::sqrt(y[0] * y[0] + y[1] * y[1]) > expect) { ++rejected_; return false; }

        // H = [[1,0,s, R*c, 0],[0,1,c,-R*s,0]]
        float H[2][5] = {{1, 0, s,  x_[2] * c, 0},
                         {0, 1, c, -x_[2] * s, 0}};
        // S = H P H^T + R (2x2)
        float HP[2][5];
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 5; ++j) {
                float t = 0.0f;
                for (int k = 0; k < 5; ++k) t += H[i][k] * P_[k][j];
                HP[i][j] = t;
            }
        float S[2][2];
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                float t = 0.0f;
                for (int k = 0; k < 5; ++k) t += HP[i][k] * H[j][k];
                S[i][j] = t;
            }
        S[0][0] += meas_ * meas_;
        S[1][1] += meas_ * meas_;
        const float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (std::fabs(det) < 1e-9f) return false;
        const float inv = 1.0f / det;
        float Si[2][2] = {{ S[1][1] * inv, -S[0][1] * inv},
                          {-S[1][0] * inv,  S[0][0] * inv}};

        // K = (P H^T) S^-1  (5x2)
        float K[5][2];
        float PHT[5][2];
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 2; ++j) {
                float t = 0.0f;
                for (int k = 0; k < 5; ++k) t += P_[i][k] * H[j][k];
                PHT[i][j] = t;
            }
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 2; ++j) {
                K[i][j] = PHT[i][0] * Si[0][j] + PHT[i][1] * Si[1][j];
            }

        // x += K y
        for (int i = 0; i < 5; ++i) x_[i] += K[i][0] * y[0] + K[i][1] * y[1];

        // P -= K H P
        float KHP[5][5];
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j) {
                float t = 0.0f;
                for (int k = 0; k < 2; ++k) t += K[i][k] * HP[k][j];
                KHP[i][j] = t;
            }
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j) P_[i][j] -= KHP[i][j];

        // 相位归一化到 (-pi, pi], 避免长期累积
        while (x_[3] >  3.14159265f) x_[3] -= 6.2831853f;
        while (x_[3] <= -3.14159265f) x_[3] += 6.2831853f;

        // 转速限幅: ±60 rad/s (约 ±10 圈/s) 已远超实际小陀螺
        if (x_[4] >  60.0f) x_[4] =  60.0f;
        if (x_[4] < -60.0f) x_[4] = -60.0f;
        // 半径非负
        if (x_[2] < 0.0f) x_[2] = 0.0f;
        return true;
    }

private:
    float x_[5] = {0, 0, 0, 0, 0};
    float P_[5][5] = {};
    float acc_  = 600.0f;
    float q_r_  = 50.0f;
    float q_w_  = 20.0f;
    float meas_ = 40.0f;
    bool  inited_ = false;
    int   frames_ = 0;
    int   rejected_ = 0;
};

}  // namespace srm

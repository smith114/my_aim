// aim_core.hpp: 瞄准决策的**唯一实现**
//
// 为什么要有这个文件:
//   之前 main.cpp(实车) 和 aim_gui.cpp(调参工具) 各自写了一份瞄准逻辑,
//   然后实车那侧陆续加了"盯圆心""提前量低通""换板重设角度保留角速度"
//   "追赶时抑制提前量""偏差外推"等修正, GUI 那侧一条都没跟上。
//   后果很严重: **操作手在 GUI 里调出来的参数, 验的是一个旧算法**,
//   而实车跑的是另一个。两边看到的 th/cmd 根本不是一回事。
//
//   所以把决策集中到这里, main 和 GUI 都只调用 step()。
//   要改算法只改这一个文件, 不可能再分叉。
#pragma once

#include <cmath>

#include "config.hpp"
#include "kf.hpp"

namespace srm {

// 一次 step 的输入: 都是当前帧的测量量, 不含任何"内部状态"
struct AimInput {
    float raw_yaw   = 0.0f;   // 选中板的 yaw 偏角(相机系, 度)
    float raw_pitch = 0.0f;   // 含弹道补偿后的 pitch 偏角(度)
    int   follow    = -1;     // 选中的候选下标, -1 = 本帧无目标
    float dist_m    = 0.0f;   // 距离(米), <=0 表示不可用
    float bullet_spd= 22.0f;  // 弹速(m/s)
    float dt        = 0.01f;  // 距上一帧的秒数
    float cmd_yaw   = 0.0f;   // 上一条下发的命令(用于判断云台是否在追赶)
    float cmd_pitch = 0.0f;
    float fb_yaw    = 0.0f;   // 下位机回传的实测姿态
    float fb_pitch  = 0.0f;
    bool  fb_fresh  = false;  // 回传是否新鲜
};

// 一次 step 的输出: 送伺服的角度 + 开火判据需要的量
struct AimOutput {
    float theta_yaw   = 0.0f;  // 送伺服的 yaw 偏角
    float theta_pitch = 0.0f;  // 送伺服的 pitch 偏角
    float lead_yaw    = 0.0f;  // 实际施加的提前量(日志用)
    float lead_pitch  = 0.0f;
    float t_lead      = 0.0f;  // 外推总时间(链路+飞行, 秒)
    float pos_dev     = 0.0f;  // 板相对圆心瞄准点的偏差(盯圆心策略的开火判据)
    bool  center_valid= false; // 圆心估计是否可用
    bool  switched    = false; // 本帧是否判定换板
};

class AimCore {
public:
    // 换板时把伺服内部状态也对齐到真值(见 AngleKF::reseed 的说明)
    float center_rel()  const { return center_rel_; }
    bool  center_valid()const { return center_init_; }
    float lead_lp_yaw() const { return lead_lp_y_; }

    AimOutput step(const AimInput& in) {
        AimOutput out;
        if (in.follow < 0) {
            // 无目标: 不动内部状态, theta 由调用方决定(保持或归零)
            out.center_valid = center_init_;
            out.pos_dev      = 0.0f;
            return out;
        }

        // ---------- ① 换板检测 ----------
        switched_ = (prev_follow_ != in.follow) ||
                    (std::fabs(in.raw_yaw - prev_raw_y_) > KF_SWITCH_JUMP_DEG);
        prev_follow_ = in.follow;
        prev_raw_y_  = in.raw_yaw;
        out.switched = switched_;

        // ---------- ② 预测外推时长: 链路 + 飞行 ----------
        float t_lead = KF_LATENCY_S;
        if (KF_LEAD_FLIGHT_TIME && in.dist_m > 0.1f) {
            float v = in.bullet_spd;
            if (!(v > FIRE_MIN_BULLET_SPEED)) v = BULLET_SPEED_DEFAULT;
            t_lead += in.dist_m / v;
        }
        out.t_lead = t_lead;

        float theta_yaw   = in.raw_yaw;
        float theta_pitch = in.raw_pitch;

        if (KF_ENABLE) {
            kf_yaw_.predict(in.dt);
            kf_pitch_.predict(in.dt);
            if (switched_) {           // 重设角度, 保留角速度(转速连续)
                kf_yaw_.reseed(in.raw_yaw);
                kf_pitch_.reseed(in.raw_pitch);
            }
            kf_yaw_.update(in.raw_yaw);
            kf_pitch_.update(in.raw_pitch);

            theta_yaw   = kf_yaw_.predict_ahead(t_lead);
            // pitch 不外推: 旋转靶绕竖轴转, 该轴无真实运动, 外推只会放大噪声
            theta_pitch = KF_LEAD_PITCH ? kf_pitch_.predict_ahead(t_lead) : kf_pitch_.theta();

            float raw_lead_y = theta_yaw   - in.raw_yaw;
            float raw_lead_p = theta_pitch - in.raw_pitch;

            // 云台正在追赶时, theta 的变化主要来自云台自己, 此刻的角速度虚高
            if (in.fb_fresh) {
                float fe_y = in.cmd_yaw - in.fb_yaw;
                while (fe_y >  180.0f) fe_y -= 360.0f;
                while (fe_y < -180.0f) fe_y += 360.0f;
                const float fe_p = in.cmd_pitch - in.fb_pitch;
                if (std::fabs(fe_y) > KF_LEAD_SUPPRESS_DEG ||
                    std::fabs(fe_p) > KF_LEAD_SUPPRESS_DEG) {
                    raw_lead_y = 0.0f;
                    raw_lead_p = 0.0f;
                }
            }

            // 提前量本身做低通: 角速度估计逐帧跳, 但真实转速是慢变量
            lead_lp_y_ += KF_LEAD_LP_COEF * (raw_lead_y - lead_lp_y_);
            lead_lp_p_ += KF_LEAD_LP_COEF * (raw_lead_p - lead_lp_p_);

            theta_yaw   = in.raw_yaw   + lead_lp_y_;
            theta_pitch = in.raw_pitch + lead_lp_p_;
        }

        out.lead_yaw   = lead_lp_y_;
        out.lead_pitch = lead_lp_p_;

        // ---------- ③ 盯圆心 ----------
        // 圆心 = 板方位在**相机系**里的慢速平均。不用绝对方位:
        // fb 与 raw 的延迟不同, 云台一转两者相加就不守恒(实测漂 18 度)。
        if (CENTER_HOLD_ENABLE) {
            if (!center_init_) {
                center_rel_  = in.raw_yaw;
                center_init_ = true;
            } else {
                center_rel_ += CENTER_LP_COEF * (in.raw_yaw - center_rel_);
            }

            const float pos_dev = in.raw_yaw - center_rel_;

            // 偏差变化率(低通), 外推到子弹到达时刻 —— 这就是"用旋转模型预测":
            // 偏差是正弦, 零点附近近似线性, 0.24 秒内只变约 0.25 度(板宽 1.5)。
            if (!dev_init_) { dev_prev_ = pos_dev; dev_init_ = true; }
            const float dt_safe = std::fmax(in.dt, 1e-4f);
            const float inst    = (pos_dev - dev_prev_) / dt_safe;
            dev_rate_ += CENTER_DEV_RATE_LP * (inst - dev_rate_);
            dev_prev_  = pos_dev;

            out.pos_dev      = CENTER_PRED_ENABLE ? (pos_dev + dev_rate_ * t_lead) : pos_dev;
            out.center_valid = true;

            // 瞄准圆心, 不加提前量 —— 不追就没有追的误差
            theta_yaw = center_rel_;
            out.lead_yaw = 0.0f;
        }

        out.theta_yaw   = theta_yaw;
        out.theta_pitch = theta_pitch;
        return out;
    }

    // 丢失目标足够久后, 圆心失效(下次重新建立)
    void on_target_lost(int lost_frames) {
        if (lost_frames > MAX_LOST_FRAMES) center_init_ = false;
    }

private:
    AngleKF kf_yaw_, kf_pitch_;
    float   lead_lp_y_ = 0.0f, lead_lp_p_ = 0.0f;
    float   center_rel_ = 0.0f;
    bool    center_init_ = false;
    float   dev_rate_ = 0.0f, dev_prev_ = 0.0f;
    bool    dev_init_ = false;
    int     prev_follow_ = -1;
    float   prev_raw_y_ = 0.0f;
    bool    switched_ = false;
};

}   // namespace srm

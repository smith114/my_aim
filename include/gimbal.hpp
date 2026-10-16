// gimbal.hpp: 云台伺服
//
// 协议只允许上位机"写"绝对目标角 (ID1 的 yaw/pitch), 没有任何"设置当前角度"
// 的指令; 下位机会回传当前实际姿态。据此设计控制律:
//
//   期望角 = 当前实测角 + SERVO_DIR * 相机系偏差角
//
// 为什么用"实测角 + 偏差角"而不是自己积分:
//   偏差角 theta 的定义是"目标相对光轴的夹角", 要把它消掉, 光轴就得朝目标方向
//   转 theta。实测角是下位机给的真值, 用它当基准不会累积漂移; 而且只要光轴真的
//   转过去, 下一帧 theta 自然变小, 闭环自校正。
//
// 为什么不能累加自己的估计值(前一版的写法):
//   前一版把 theta 乘上符号后累加进自己的估计角 yaw_est。如果符号反了, 云台朝
//   远离目标的方向转, theta 反而变大, 下一帧继续往同方向推 -> 正反馈一路转到限位。
//   仿真里实测就是这种发散(目标在左, 云台却一直加 yaw)。
//   把它锚在实测角上, 即使符号反了, 也只会表现为"绕不动/来回摆", 不会发散发疯。
//
// SERVO_DIR 是唯一需要实机确认的符号: 它表示"增大 yaw 命令时, 目标在画面里
// 朝哪个方向移动"。开机自检 (calibrate_dir) 会实测这一点并自动纠正。
#pragma once

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace srm {

class GimbalCtrl {
public:
    // 最近一次回传的实测姿态(度)
    float meas_yaw   = 90.0f;
    float meas_pitch = 90.0f;
    bool  have_meas  = false;

    // 最近一次实际下发的命令角(度)
    float cmd_yaw   = 90.0f;
    float cmd_pitch = 90.0f;

    // 伺服方向, 初值来自 config; 开机自检可翻转
    float servo_dir = SERVO_DIR;

    // 状态
    bool  tracking = false;
    float last_theta_yaw = 0.0f;    // 上一帧相机系偏差(自检用)

    // 更新实测姿态(由接收线程的数据喂入)
    void update_feedback(float yaw, float pitch) {
        meas_yaw   = yaw;
        meas_pitch = pitch;
        have_meas  = true;
        if (!inited_) {
            cmd_yaw   = yaw;
            cmd_pitch = pitch;
            // 记录启动姿态, 作为安全包络的中心
            home_yaw   = yaw;
            home_pitch = pitch;
            inited_    = true;
        }
    }

    // 一帧跟踪: 输入相机系偏差角(度, +yaw = 目标偏右, +pitch = 目标偏下)
    // 返回是否产生了新的命令
    bool track(float theta_yaw, float theta_pitch) {
        if (!have_meas) return false;   // 没有实测角就不动(避免拿错误基准乱指)

        tracking = true;
        last_theta_yaw = theta_yaw;

        // 低通 + 死区, 抑制角点噪声引起的抖动
        const float ty = smooth(theta_yaw, sm_yaw_);
        const float tp = smooth(theta_pitch, sm_pitch_);

        // 期望角 = 实测角 + 方向 * 偏差
        const float want_yaw   = meas_yaw   + servo_dir * ty * SERVO_GAIN;
        const float want_pitch = meas_pitch + servo_dir * tp * SERVO_GAIN;

        // 限幅步进: 每帧最多转 SERVO_MAX_STEP 度, 防大偏差时猛甩
        cmd_yaw   = step_toward(cmd_yaw,   want_yaw,   SERVO_MAX_STEP);
        cmd_pitch = step_toward(cmd_pitch, want_pitch, SERVO_MAX_STEP);
        cmd_yaw   = wrap_or_clamp_yaw(cmd_yaw);
        cmd_pitch = clamp_val(cmd_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        apply_envelope();   // 再套一层安全包络
        return true;
    }

    // 手动控制(键盘调试)
    bool manual_yaw(float d) {
        cmd_yaw = wrap_or_clamp_yaw(cmd_yaw + d);
        sm_yaw_ = 0.0f;
        apply_envelope();
        return true;
    }
    bool manual_pitch(float d) {
        cmd_pitch = clamp_val(cmd_pitch + d, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        sm_pitch_ = 0.0f;
        apply_envelope();
        return true;
    }

    // 无目标时在启动姿态附近三角波摆扫寻板
    bool scan() {
        if (++scan_cnt_ < GIMBAL_SCAN_HOLD) return false;
        cmd_yaw = wrap_or_clamp_yaw(cmd_yaw + scan_dir_ * GIMBAL_SCAN_SPEED);
        const float half = GIMBAL_SCAN_AMP * 0.5f;
        const float dy = wrap_delta(cmd_yaw - home_yaw);
        if (dy >  half) scan_dir_ = -1.0f;
        if (dy < -half) scan_dir_ =  1.0f;
        apply_envelope();
        return true;
    }
    void reset_scan() {
        scan_cnt_ = 0;
        scan_dir_ = 1.0f;
    }

    void stop_tracking() { tracking = false; }

    float home_yaw   = 0.0f;   // 启动姿态(安全包络中心)
    float home_pitch = 0.0f;

    // 安全包络: 命令角不允许离启动姿态超过 ENVELOPE 度。
    //
    // 为什么需要它: 云台的真实角量程事先不知道(协议没规定, 固件还会再限位)。
    // 但控制律锚在实测角上, 本身不会主动往限位撞, 所以只要再加一圈"别跑太远"
    // 的软保护就够了 —— 比去猜一个绝对区间可靠得多。
    // 真出问题时(比如符号错), 云台最多在启动姿态附近小幅晃动, 不会一路转到底。
    void apply_envelope() {
        const float dy = wrap_delta(cmd_yaw - home_yaw);
        if (dy >  GIMBAL_ENVELOPE_DEG) cmd_yaw = wrap360(home_yaw + GIMBAL_ENVELOPE_DEG);
        if (dy < -GIMBAL_ENVELOPE_DEG) cmd_yaw = wrap360(home_yaw - GIMBAL_ENVELOPE_DEG);
        cmd_pitch = clamp_val(cmd_pitch, home_pitch - GIMBAL_ENVELOPE_DEG,
                                          home_pitch + GIMBAL_ENVELOPE_DEG);
    }

    // 查询/调试用: 直接给一个绝对命令角(绕过包络, 供 gimbal_test 工具使用)
    void force_set(float yaw, float pitch) {
        cmd_yaw   = yaw;
        cmd_pitch = pitch;
    }

    // 开机自检: 判断 SERVO_DIR 是否正确。
    // 原理: 按当前方向发一个 yaw 命令, 若光轴真的朝目标转了, 下一帧的偏差角
    // 幅值应当变小(或符号翻转)。若连续两帧幅值都变大, 说明方向反了 -> 翻转。
    // 返回 true 表示本次翻转了方向。
    bool auto_dir_check(float theta_before, float theta_after, float cmd_delta) {
        if (std::fabs(cmd_delta) < 0.3f) return false;          // 没动, 判不了
        if (std::fabs(theta_before) < 2.0f) return false;       // 偏差太小, 噪声占主导
        const bool grew = std::fabs(theta_after) > std::fabs(theta_before) + 0.3f;
        if (grew) {
            ++dir_bad_streak_;
            if (dir_bad_streak_ >= 2) {
                servo_dir = -servo_dir;
                dir_bad_streak_ = 0;
                return true;
            }
        } else {
            dir_bad_streak_ = 0;
        }
        return false;
    }

private:
    static float clamp_val(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }
    static float wrap360(float v) {
        v = std::fmod(v, 360.0f);
        if (v < 0.0f) v += 360.0f;
        return v;
    }
    float wrap_or_clamp_yaw(float v) const {
        if (GIMBAL_YAW_MAX - GIMBAL_YAW_MIN >= 359.0f) return wrap360(v);
        return clamp_val(v, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    }
    // 低通: 把输入滤波到 state 里, 并在死区内归零
    static float smooth(float in, float& state) {
        state = state * (1.0f - GIMBAL_SMOOTH) + in * GIMBAL_SMOOTH;
        if (std::fabs(state) < GIMBAL_DEADBAND) state = 0.0f;
        return state;
    }
    static float step_toward(float cur, float want, float max_step) {
        const float d = clamp_val(want - cur, -max_step, max_step);
        return cur + d;
    }

    bool  inited_ = false;
    float sm_yaw_ = 0.0f;
    float sm_pitch_ = 0.0f;
    float scan_dir_ = 1.0f;
    int   scan_cnt_ = 0;
    int   dir_bad_streak_ = 0;

    // 把角度差规整到 (-180, 180], 用于跨 0/360 的比较
    static float wrap_delta(float d) {
        while (d >  180.0f) d -= 360.0f;
        while (d <= -180.0f) d += 360.0f;
        return d;
    }
};

}  // namespace srm

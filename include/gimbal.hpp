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
            // 记录启动姿态, 作为安全包络的中心。
            // 注意: 必须等数值稳定之后再锁 home, 否则开机第一帧的抖动值会把
            // 整个包络定在错误的位置上, 之后云台就在错的区间里活动。
            home_yaw     = yaw;
            home_pitch   = pitch;
            home_locked_ = false;
            home_cnt_    = 0;
            inited_      = true;
            return;
        }
        // 前若干帧用滑动平均把 home 稳定下来
        if (!home_locked_) {
            home_yaw   = home_yaw * 0.7f + yaw * 0.3f;
            home_pitch = home_pitch * 0.7f + pitch * 0.3f;
            if (++home_cnt_ >= HOME_STABLE_FRAMES) {
                home_locked_ = true;
                std::printf("[GIMBAL] 启动姿态锁定 home=%.2f/%.2f\n", home_yaw, home_pitch);
            }
        }
    }

    bool home_stable() const { return home_locked_; }

    // 一帧跟踪: 输入相机系偏差角(度, +yaw = 目标偏右, +pitch = 目标偏下)
    // 返回是否产生了新的命令
    bool track(float theta_yaw, float theta_pitch) {
        if (!have_meas) return false;   // 没有实测角就不动(避免拿错误基准乱指)

        // 一开始跟踪就立刻冻结 home, 不能再继续做滑动平均。
        //
        // 为什么: home 是安全包络的中心, 它必须在**跟踪开始之前**就定死。
        // 原来只在 update_feedback 里累计 20 帧才锁, 如果那 20 帧落在跟踪开始
        // 之后(或操作手在 mode=0 时正用手推云台), home 会跟着云台一起漂移 ——
        // 包络中心跟着动, 包络就等于失效。实测该场景 home 从 0.0 漂到 -32.64,
        // 于是"命令角不超过 home±30"这条断言直接被破坏。
        // 这里用当前已累计的估计值当场锁定, 之后不再变。
        if (!home_locked_) {
            home_locked_ = true;
            std::printf("[GIMBAL] 开始跟踪, 冻结包络中心 home=%.2f/%.2f\n",
                        static_cast<double>(home_yaw), static_cast<double>(home_pitch));
        }

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
        cmd_pitch = wrap_or_clamp_pitch(cmd_pitch);
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
        cmd_pitch = wrap_or_clamp_pitch(cmd_pitch + d);
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
    // 为什么用相对启动姿态而不是绝对区间:
    //   yaw 能整圈转, 但 pitch 有机械限位, 而零位并不是机壳行程的中点。
    //   一旦 pitch 顶到限位, 反馈会卡住不再跟随命令; 如果还按绝对区间去比较,
    //   会得到一个错误的高度差, 云台就卡死在错的高度回不来。
    //
    // **yaw 与 pitch 必须区别对待, 这是修过的 bug**:
    //   yaw 是回绕轴, 跨 ±180 用 wrap_delta 比较、结果用 wrap360 归一是对的;
    //   pitch 是有限行程轴, 既不能 wrap_delta 也不能 wrap360。
    //   原实现两轴都套了 wrap360, 于是 home_pitch=3 时下包络 -27 被算成 333,
    //   下位机收到 333 会理解成"转到最高", 云台直接顶死机械限位且无法自行恢复。
    //   tests/gimbal_edge.cpp 专门测这一条。
    void apply_envelope() {
        // yaw: 回绕轴
        const float dy = wrap_delta(cmd_yaw - home_yaw);
        if (dy >  GIMBAL_ENVELOPE_DEG) cmd_yaw = wrap360(home_yaw + GIMBAL_ENVELOPE_DEG);
        if (dy < -GIMBAL_ENVELOPE_DEG) cmd_yaw = wrap360(home_yaw - GIMBAL_ENVELOPE_DEG);

        // pitch: 有限行程轴 —— 先按相对包络限, 再按机械行程硬限, 全程 clamp。
        // 两道都要: 相对包络防"离瞄准点太远", 机械行程防"顶到限位"。
        const float dp = cmd_pitch - home_pitch;
        if (dp >  GIMBAL_ENVELOPE_DEG) cmd_pitch = home_pitch + GIMBAL_ENVELOPE_DEG;
        if (dp < -GIMBAL_ENVELOPE_DEG) cmd_pitch = home_pitch - GIMBAL_ENVELOPE_DEG;
        cmd_pitch = clamp_val(cmd_pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
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
    // pitch **永远 clamp, 永不 wrap**。
    //
    // 原来这里和 yaw 一样按"跨度 >= 359 就 wrap"判断, 而配置里 pitch 正好是
    // [-180, 180] 跨度 360, 于是走进了 wrap 分支 —— 结果 -30 度被归一成 330 度,
    // 下位机据此把云台顶到最高处。pitch 是机械限位轴, 回绕在物理上就没有意义,
    // 所以这里无条件 clamp, 不设 wrap 分支。
    float wrap_or_clamp_pitch(float v) const {
        return clamp_val(v, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
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
    bool  home_locked_ = false;
    int   home_cnt_ = 0;
    static constexpr int HOME_STABLE_FRAMES = 20;   // 约 0.2s @100Hz
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

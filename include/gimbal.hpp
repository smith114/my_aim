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

    // 伺服方向: **yaw 与 pitch 分开**, 两者实测就是相反的(见 config.hpp 推导)。
    float servo_dir_yaw   = SERVO_DIR_YAW;
    float servo_dir_pitch = SERVO_DIR_PITCH;

    // 状态
    bool  tracking = false;
    // 上一帧算出的"期望角"(未限幅/未包络), 仅用于日志诊断:
    // 看 want 与 cmd 的差就知道限速/包络是否在削, 看 want 与 fb 的差就知道跟没跟上。
    // ---- 自适应符号: 自己判断每根轴的方向 ----
    // 判据: 正确方向下, 施加修正后 |theta| 应变小。
    // 若连续 kFlipFrames 帧 |theta| 都变大, 说明该轴符号反了 -> 翻转。
    // 每轴最多翻 kMaxFlips 次, 防止来回抖。








    float last_want_yaw   = 0.0f;
    float last_want_pitch = 0.0f;
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

        // 中断够久才重新软启动, 否则沿用之前的进度 —— 见 stop_tracking 的说明。
        if (gap_cnt_ > kRegapFrames) {
            soft_cnt_ = 0;
            sm_yaw_   = 0.0f;
            sm_pitch_ = 0.0f;
        }
        gap_cnt_ = 0;

        // 软启动: 进跟踪后的前 SERVO_SOFT_START_FRAMES 帧, 增益从 0 线性升到 1。
        // 目的: 切换视觉模式那一瞬间, 如果目标偏角较大, 不要立刻满增益猛拽一下 ——
        // 使用者反馈过"一开自瞄就往一边偏", 除了符号问题, 起手的阶跃感也需要平掉。
        // soft_cnt_ 在 stop_tracking() 里清零, 所以每次切模式都会重新软启动。
        const float ramp =
            (SERVO_SOFT_START_FRAMES <= 0)
                ? 1.0f
                : std::min(1.0f, static_cast<float>(soft_cnt_) /
                                      static_cast<float>(SERVO_SOFT_START_FRAMES));
        if (soft_cnt_ < SERVO_SOFT_START_FRAMES) ++soft_cnt_;

        last_theta_yaw = theta_yaw;

        // 低通 + 死区, 抑制角点噪声引起的抖动
        const float ty = smooth(theta_yaw, sm_yaw_);
        const float tp = smooth(theta_pitch, sm_pitch_);

        // ---- 发散保护 ----
        // 符号错时闭环是正反馈: |theta| 会一路上涨, 云台转到限位还在转。
        // 这里监测"|theta| 是否连续多帧单调变大", 一旦确认就停止推进并报错,
        // 让操作者看到明确的失败而不是云台自己乱转。
        const float mag = std::fabs(ty);
        if (mag > last_mag_ + 0.3f) ++grow_streak_;
        else if (mag < last_mag_)   grow_streak_ = 0;
        last_mag_ = mag;
        if (grow_streak_ >= kDivergeFrames) {
            if (!diverged_) {
                std::printf("[GIMBAL][ERR] 偏角连续 %d 帧增大(%.2f->%.2f), 判定为发散,"
                            " 停止推进。请检查 SERVO_DIR_YAW/PITCH 符号。\n",
                            grow_streak_, static_cast<double>(last_mag_),
                            static_cast<double>(mag));
                diverged_ = true;
            }
            return false;   // 不再推进伺服
        }

        // ------------------------------------------------------------
        // **期望角 = 实测角 + 限幅后的修正量**
        //
        // 之前是 "cmd = step_toward(cmd, want, 2.0)": 在**累积的 cmd** 上做速率限制。
        // 问题: cmd 自己以 2 度/帧往前走, 而 meas 因为机械惯性在后面追, 于是
        // cmd 领先 meas 越来越多 —— 实测 cmd 和 fb 差了 25~33 度, 云台一路冲过
        // 目标 30 多度, 直接把装甲板甩出画面(日志里 armor 从 1.0 掉到 0)。
        //
        // 改成每帧都从 meas 重新算: 修正量 = DIR*theta, 但**限幅**在
        // SERVO_MAX_CORR_DEG 之内。于是:
        //   |theta| 小 -> 修正小 -> 慢慢贴上去, 不过冲
        //   |theta| 大 -> 修正被限在 8 度 -> 分段逼近, 不会一次猛甩 30 度
        // 结构上**不可能累积**: 命令永远只比实测角大有限的一点点。
        const float cy = clamp_val(servo_dir_yaw   * ty * SERVO_GAIN * ramp,
                                   -SERVO_MAX_CORR_DEG, SERVO_MAX_CORR_DEG);
        const float cp = clamp_val(servo_dir_pitch * tp * SERVO_GAIN * ramp,
                                   -SERVO_MAX_CORR_DEG, SERVO_MAX_CORR_DEG);

        last_want_yaw   = meas_yaw   + cy;
        last_want_pitch = meas_pitch + cp;

        cmd_yaw   = wrap_or_clamp_yaw(meas_yaw + cy);
        cmd_pitch = wrap_or_clamp_pitch(meas_pitch + cp);
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

    void stop_tracking() {
        tracking = false;
        // **不要立刻清软启动计数**。
        //
        // 实测(10-28): 下位机的 mode 在刚开自瞄时会 0/1 快速闪烁(每 0.5~1 秒一次),
        // 加上检测本身逐帧抖动, 如果这里一丢就清零, 软启动永远升不到满增益,
        // 云台就表现为"几乎不动"。
        // 改成: 先只累计中断帧数, 真正清状态放到 track() 里按"中断够久"判断。
        ++gap_cnt_;
    }

    // 中断超过这么多帧才认为真的断了(重新软启动 + 清低通);
    // 短暂的 mode 闪烁 / 单帧漏检不清状态。
    static constexpr int kRegapFrames = 25;

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
        cmd_pitch = clamp_val(cmd_pitch, pitch_lo(), pitch_hi());
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
                servo_dir_yaw = -servo_dir_yaw;
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
    // 带安全余量的 pitch 上下限: 绝不把命令角发到贴边, 避免堵转卡死。
    float pitch_lo() const { return GIMBAL_PITCH_MIN + GIMBAL_PITCH_MARGIN; }
    float pitch_hi() const { return GIMBAL_PITCH_MAX - GIMBAL_PITCH_MARGIN; }

    // pitch **永远 clamp, 永不 wrap**。
    //
    // 原来这里和 yaw 一样按"跨度 >= 359 就 wrap"判断, 而配置里 pitch 正好是
    // [-180, 180] 跨度 360, 于是走进了 wrap 分支 —— 结果 -30 度被归一成 330 度,
    // 下位机据此把云台顶到最高处。pitch 是机械限位轴, 回绕在物理上就没有意义,
    // 所以这里无条件 clamp, 不设 wrap 分支。
    float wrap_or_clamp_pitch(float v) const {
        return clamp_val(v, pitch_lo(), pitch_hi());
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
    int   soft_cnt_ = 0;      // 软启动帧计数
    int   gap_cnt_  = 0;      // 跟踪中断帧数, 超过 kRegapFrames 才真正重置
    float last_mag_ = 0.0f;   // 上一帧 |theta|, 用于发散检测
    int   grow_streak_ = 0;   // |theta| 连续增大的帧数
    bool  diverged_ = false;  // 已判定发散
    static constexpr int kDivergeFrames = 40;   // 约 0.4~2.5 秒, 看帧率

    // 把角度差规整到 (-180, 180], 用于跨 0/360 的比较
    static float wrap_delta(float d) {
        while (d >  180.0f) d -= 360.0f;
        while (d <= -180.0f) d += 360.0f;
        return d;
    }
};

}  // namespace srm

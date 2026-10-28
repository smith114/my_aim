// test_closed_loop.cpp: 闭环仿真 —— 不需要相机和实车, 验证"检测->解算->云台"真能收敛
//
// 仿真模型:
//   世界上有一个静止目标, 相对初始光轴的方位角为 bearing;
//   云台有真实角度 gimbal_actual, 按伺服滞后跟随命令 cmd_yaw;
//   相机装在云台上, 所以目标相对光轴的偏差
//       theta = bearing - gimbal_actual
//   (云台顺转 delta, 光轴也顺转 delta, 固定目标相对光轴的角就减少 delta)
//   每轮: 按 theta 合成图 -> 检测 -> 配对 -> PnP -> tracker -> gimbal.track()
//
// 注意: 相机水平半视场只有约 ±21.7 度 (1440/2 / 1807 * 180/pi), 超过这个范围
// 目标根本不在画面里 —— 这正是生产代码里要摆扫寻板的原因。所以这里只测
// 视场内的收敛性, 视场外的场景交给"摆扫"逻辑。
#include <cmath>

#include "armor.hpp"
#include "gimbal.hpp"
#include "synth.hpp"
#include "test_util.hpp"
#include "tracker.hpp"

using namespace srm;
using namespace srm::test;

// 视场半角(度)
static float half_fov_deg() {
    return static_cast<float>(std::atan2(CAM_CX, CAM_FX) * 180.0 / CV_PI);
}

// 角度差规整到 (-180,180] —— 命令角是 0~360 归一化的, 直接相减会得到 330 这种值
static float wrap_delta(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

struct LoopOutcome {
    int   locked_at = -1;
    float turned    = 0.0f;   // 云台实际转过角度
    float residual  = 0.0f;   // 末帧残余偏差角
    float overshoot = 0.0f;   // 锁定后最大超出量
};

// servo_lag: 一阶伺服跟随系数(1.0 = 瞬间到位)
static LoopOutcome run_loop(float bearing_rel_deg, float dist_mm, int steps,
                            float servo_lag = 0.6f, bool verbose = false) {
    Tracker    tracker;
    GimbalCtrl gimbal;
        // 本测试检验的是**控制律算术**(期望角 = 实测角 + DIR*偏差 是否会收敛),
        // 它内置的被控对象模型假定 "编码器增大 -> 光轴同向转", 即 DIR=+1。
        // 实机 DIR 的符号是另一个问题(见 config.hpp 的实测说明), 在这里固定为 +1,
        // 这样符号一旦在配置里改动, 不会把控制律本身的回归测试带崩。
        gimbal.servo_dir = 1.0f;
    tracker.set_frame_size(cv::Size(1440, 1080));
    gimbal.update_feedback(90.0f, 90.0f);   // 下位机回传: 光轴在 0 位

    const float base = 90.0f;
    float gimbal_actual = 0.0f;   // 相对初始光轴已转过的角度
    LoopOutcome out;

    for (int step = 0; step < steps; ++step) {
        const float theta = bearing_rel_deg - gimbal_actual;
        SynthPose pose;
        pose.x_mm = dist_mm * std::sin(theta * static_cast<float>(CV_PI) / 180.0f);
        pose.z_mm = dist_mm * std::cos(theta * static_cast<float>(CV_PI) / 180.0f);

        const auto img = make_armor_image(pose);
        const auto bars = extract_lightbars(img.image);
        const auto armors = match_armors(bars, hsv_value_channel(img.image));

        std::vector<Target> cands;
        for (const auto& ar : armors) {
            const SolveResult r = solve_pnp(ar.pts);
            if (!r.ok || r.dist < SOLVE_DIST_MIN || r.dist > SOLVE_DIST_MAX) continue;
            Target t;
            t.center = ar.center;
            t.solve  = r;
            cands.push_back(t);
        }

        const auto tr = tracker.update(cands);
        if (tr.locked) {
            if (out.locked_at < 0) out.locked_at = step;
            // 下位机回传的实测角 = base + 云台实际角
            gimbal.update_feedback(base + gimbal_actual, base);
            gimbal.track(tr.target.solve.yaw, tr.target.solve.pitch);
        }

        // 伺服: 实际角一阶跟随命令角
        const float cmd_delta = gimbal.cmd_yaw - base;
        gimbal_actual += servo_lag * (cmd_delta - gimbal_actual);

        if (verbose && step % 5 == 0) {
            std::printf("    step %3d theta=%7.2f cmd=%7.2f actual=%7.2f\n", step, theta,
                        gimbal.cmd_yaw - base, gimbal_actual);
        }
        if (out.locked_at >= 0) {
            const float over = std::fabs(gimbal_actual - bearing_rel_deg);
            if (over > out.overshoot) out.overshoot = over;
        }
    }
    out.turned   = gimbal_actual;
    out.residual = bearing_rel_deg - gimbal_actual;
    return out;
}

int main() {
    const float fov = half_fov_deg();
    std::printf("  相机水平半视场 = %.1f 度 (超过这个角目标不在画面里)\n", fov);

    // ---------------------------------------------------------- 场景 1: 目标在左侧 10 度
    {
        const auto r = run_loop(-10.0f, 5000.0f, 200);
        CHECK(r.locked_at >= 0);
        CHECK(r.locked_at <= 15);
        CHECK_NEAR(r.turned, -10.0f, 1.5f);
        CHECK_NEAR(r.residual, 0.0f, 1.5f);
        std::printf("  [场景1] 左侧10度: 第 %d 帧锁定, 转过 %.2f 度, 残余 %.2f 度, 超调 %.2f\n",
                    r.locked_at, r.turned, r.residual, r.overshoot);
    }

    // ---------------------------------------------------------- 场景 2: 目标在右侧 12 度
    {
        const auto r = run_loop(12.0f, 5000.0f, 200);
        CHECK(r.locked_at >= 0);
        CHECK_NEAR(r.turned, 12.0f, 1.5f);
        CHECK_NEAR(r.residual, 0.0f, 1.5f);
        std::printf("  [场景2] 右侧12度: 第 %d 帧锁定, 转过 %.2f 度, 残余 %.2f 度\n",
                    r.locked_at, r.turned, r.residual);
    }

    // ---------------------------------------------------------- 场景 3: 多个方位角
    for (const float bearing : {-18.0f, -8.0f, -3.0f, 3.0f, 8.0f, 18.0f}) {
        const auto r = run_loop(bearing, 5000.0f, 300);
        CHECK(r.locked_at >= 0);
        CHECK_NEAR(r.turned, bearing, 2.0f);
        CHECK_NEAR(r.residual, 0.0f, 2.0f);
        std::printf("  [场景3] bearing=%+6.1f -> 转过 %+7.2f, 残余 %+6.2f, 锁定帧 %d\n",
                    bearing, r.turned, r.residual, r.locked_at);
    }

    // ---------------------------------------------------------- 场景 4: 不同距离都能收敛
    for (const float d : {1500.0f, 3000.0f, 5000.0f}) {
        const auto r = run_loop(-6.0f, d, 200);
        CHECK(r.locked_at >= 0);
        CHECK_NEAR(r.residual, 0.0f, 2.0f);
        std::printf("  [场景4] dist=%.1fm -> 残余 %+6.2f\n", d / 1000.0f, r.residual);
    }

    // ---------------------------------------------------------- 场景 5: 目标消失 -> 永不锁定
    {
        Tracker tracker;
        tracker.set_frame_size(cv::Size(1440, 1080));
        const cv::Mat dark(1080, 1440, CV_8UC3, cv::Scalar(20, 20, 20));
        const auto bars = extract_lightbars(dark);
        const auto armors = match_armors(bars, hsv_value_channel(dark));
        CHECK(bars.empty());
        CHECK(armors.empty());
        for (int i = 0; i < 50; ++i) {
            CHECK(!tracker.update({}).locked);
        }
        CHECK(tracker.state() == TrackState::DETECT);
    }

    // ---------------------------------------------------------- 场景 6: 目标已在光轴上 -> 云台不动
    {
        Tracker tracker;
        GimbalCtrl gimbal;
        // 本测试检验的是**控制律算术**(期望角 = 实测角 + DIR*偏差 是否会收敛),
        // 它内置的被控对象模型假定 "编码器增大 -> 光轴同向转", 即 DIR=+1。
        // 实机 DIR 的符号是另一个问题(见 config.hpp 的实测说明), 在这里固定为 +1,
        // 这样符号一旦在配置里改动, 不会把控制律本身的回归测试带崩。
        gimbal.servo_dir = 1.0f;
        tracker.set_frame_size(cv::Size(1440, 1080));
        gimbal.update_feedback(90.0f, 90.0f);
        SynthPose pose;
        pose.z_mm = 5000.0f;
        for (int i = 0; i < 60; ++i) {
            const auto img = make_armor_image(pose);
            const auto bars = extract_lightbars(img.image);
            const auto armors = match_armors(bars, hsv_value_channel(img.image));
            std::vector<Target> cands;
            for (const auto& ar : armors) {
                const SolveResult r = solve_pnp(ar.pts);
                if (!r.ok) continue;
                Target t;
                t.center = ar.center;
                t.solve = r;
                cands.push_back(t);
            }
            const auto tr = tracker.update(cands);
            if (tr.locked) {
                gimbal.update_feedback(90.0f, 90.0f);
                gimbal.track(tr.target.solve.yaw, tr.target.solve.pitch);
            }
        }
        CHECK_NEAR(gimbal.cmd_yaw, 90.0f, 0.5f);   // 死区内不应漂移
    }

    // ---------------------------------------------------------- 场景 7: 没有回传数据时绝不乱动
    {
        GimbalCtrl gimbal;
        // 本测试检验的是**控制律算术**(期望角 = 实测角 + DIR*偏差 是否会收敛),
        // 它内置的被控对象模型假定 "编码器增大 -> 光轴同向转", 即 DIR=+1。
        // 实机 DIR 的符号是另一个问题(见 config.hpp 的实测说明), 在这里固定为 +1,
        // 这样符号一旦在配置里改动, 不会把控制律本身的回归测试带崩。
        gimbal.servo_dir = 1.0f;
        CHECK(!gimbal.have_meas);
        CHECK(!gimbal.track(-15.0f, 5.0f));        // 无实测角 -> 拒绝跟踪
        CHECK_NEAR(gimbal.cmd_yaw, 90.0f, 1e-6);
        std::printf("  [场景7] 无回传时 track() 返回 false 且命令角保持不变\n");
    }

    // ---------------------------------------------------------- 场景 8: pitch 顶到机械限位后能自己回来
    //
    // 这是实机上真实存在的坑: yaw 能整圈转, 但 pitch 上下有限
    // (实测行程约 [-35.8, +26.1], 而零位并不在行程中点)。
    // 目标在限位之外时, 反馈会卡在限位值不再跟随命令。如果按"绝对区间"去
    // 比较就会算错高度差、卡死在错的高度; 用"相对启动姿态"的包络则能自己回来。
    {
        GimbalCtrl gimbal;
        // 本测试检验的是**控制律算术**(期望角 = 实测角 + DIR*偏差 是否会收敛),
        // 它内置的被控对象模型假定 "编码器增大 -> 光轴同向转", 即 DIR=+1。
        // 实机 DIR 的符号是另一个问题(见 config.hpp 的实测说明), 在这里固定为 +1,
        // 这样符号一旦在配置里改动, 不会把控制律本身的回归测试带崩。
        gimbal.servo_dir = 1.0f;
        const float PITCH_LO = -35.8f;   // 实测下限
        const float PITCH_HI =  26.1f;   // 实测上限
        // 先喂满启动反馈把 home 锁定, 再开始跟踪。
        // 真实系统就是这样: 下位机 48 Hz 持续回传, 操作手在 mode=0 下手动对准,
        // 等进视觉模式开始跟踪时 home 早就稳定了。
        // 若跳过这一步, home 会在跟踪过程中继续平均、跟着云台漂, 包络失效。
        for (int i = 0; i < 30; ++i) gimbal.update_feedback(0.0f, 0.0f);
        const float home_p = gimbal.home_pitch;
        CHECK(gimbal.home_stable());

        // 目标在很下方: 偏差角持续给 -60 度, 想让云台一直往下走
        for (int i = 0; i < 400; ++i) {
            // 模拟实测角: 命令被限制在机械行程内
            float meas = gimbal.cmd_pitch;
            if (meas < PITCH_LO) meas = PITCH_LO;
            if (meas > PITCH_HI) meas = PITCH_HI;
            gimbal.update_feedback(0.0f, meas);
            // 目标在画面下方 -> 偏差角为负, 云台应往下压
            gimbal.track(0.0f, -60.0f);
        }
        // 命令角绝不能超出启动姿态 ± 包络(包络比机械行程小, 所以更早被拦住)
        const float dp = wrap_delta(gimbal.cmd_pitch - home_p);
        CHECK(std::fabs(dp) <= GIMBAL_ENVELOPE_DEG + 1e-3f);
        std::printf("  [场景8] 目标在限位外: 命令 pitch 停在 %+.1f 度(包络 %.0f 度), 未失控\n",
                    dp, GIMBAL_ENVELOPE_DEG);

        // 目标回到画面中心 -> 云台应该能回到包络内, 不会卡死在限位
        for (int i = 0; i < 400; ++i) {
            float meas = gimbal.cmd_pitch;
            if (meas < PITCH_LO) meas = PITCH_LO;
            if (meas > PITCH_HI) meas = PITCH_HI;
            gimbal.update_feedback(0.0f, meas);
            gimbal.track(0.0f, 0.0f);              // 偏差为 0
        }
        CHECK(std::fabs(wrap_delta(gimbal.cmd_pitch - home_p)) <= GIMBAL_ENVELOPE_DEG + 1e-3f);
        std::printf("  [场景8] 偏差归零后命令 pitch = %+.2f 度 (相对启动姿态)\n",
                    wrap_delta(gimbal.cmd_pitch - home_p));
    }

    // ---------------------------------------------------------- 场景 9: yaw 整圈可用
    {
        GimbalCtrl gimbal;
        // 本测试检验的是**控制律算术**(期望角 = 实测角 + DIR*偏差 是否会收敛),
        // 它内置的被控对象模型假定 "编码器增大 -> 光轴同向转", 即 DIR=+1。
        // 实机 DIR 的符号是另一个问题(见 config.hpp 的实测说明), 在这里固定为 +1,
        // 这样符号一旦在配置里改动, 不会把控制律本身的回归测试带崩。
        gimbal.servo_dir = 1.0f;
        gimbal.update_feedback(0.0f, 0.0f);
        gimbal.force_set(0.0f, 0.0f);
        // 连续正向手动转, 应能绕整圈而不被夹住
        for (int i = 0; i < 10; ++i) gimbal.force_set(gimbal.cmd_yaw + 100.0f, 0.0f);
        gimbal.manual_yaw(5.0f);
        CHECK(gimbal.cmd_yaw >= 0.0f && gimbal.cmd_yaw < 360.0f);
        std::printf("  [场景9] yaw 可连续旋转, 当前 %.1f 度 (0~360 归一化)\n", gimbal.cmd_yaw);
    }

    TEST_MAIN_END();
}

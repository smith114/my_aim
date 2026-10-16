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
        CHECK(!gimbal.have_meas);
        CHECK(!gimbal.track(-15.0f, 5.0f));        // 无实测角 -> 拒绝跟踪
        CHECK_NEAR(gimbal.cmd_yaw, 90.0f, 1e-6);
        std::printf("  [场景7] 无回传时 track() 返回 false 且命令角保持不变\n");
    }

    TEST_MAIN_END();
}

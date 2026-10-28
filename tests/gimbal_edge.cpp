// 云台锁定逻辑的边界测试 —— 专门查包络/回绕/限位的交互
#include <cstdio>
#include <cmath>

#include "config.hpp"
#include "gimbal.hpp"

using srm::GimbalCtrl;

static int fails = 0;
#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("  [FAIL] %s  (%s:%d)\n", msg, __FILE__, __LINE__); \
            ++fails;                                                       \
        } else {                                                           \
            std::printf("  [ ok ] %s\n", msg);                              \
        }                                                                  \
    } while (0)

// 把 home 稳定在给定值
static void prime(GimbalCtrl& g, float yaw, float pitch) {
    // 本测试检验包络/回绕/限位这些**算术**, 与实机伺服符号无关。
    // 固定 +1, 这样 config.hpp 里实机符号改动不会把这里带崩。
    g.servo_dir = 1.0f;
    for (int i = 0; i < 40; ++i) g.update_feedback(yaw, pitch);
}

int main() {
    std::printf("GIMBAL_ENVELOPE_DEG = %.1f  SERVO_MAX_STEP = %.1f\n\n",
                static_cast<double>(srm::GIMBAL_ENVELOPE_DEG),
                static_cast<double>(srm::SERVO_MAX_STEP));

    // ---------------------------------------------------------------- 1
    // pitch 包络**不能回绕**。home_pitch=3 时, 允许下限是 -27,
    // 绝不能变成 333 (wrap360(-27)) —— 那样下位机会收到一个荒谬的高度角。
    std::printf("=== 1. pitch 包络是否回绕 ===\n");
    {
        GimbalCtrl g;
        prime(g, 100.0f, 3.0f);
        std::printf("  home = %.2f / %.2f\n", static_cast<double>(g.home_yaw),
                    static_cast<double>(g.home_pitch));
        // 强行把 pitch 往下压, 触发下包络
        g.cmd_pitch = g.home_pitch - 5.0f;
        for (int i = 0; i < 60; ++i) g.track(0.0f, +40.0f);   // theta_pitch 大正值
        std::printf("  [+方向] after track: cmd_pitch = %.2f\n",
                    static_cast<double>(g.cmd_pitch));
        {
            const float dp = g.cmd_pitch - g.home_pitch;
            CHECK(std::fabs(dp) <= srm::GIMBAL_ENVELOPE_DEG + 1.0f,
                  "pitch 偏离 home 不超过包络(正方向)");
            CHECK(g.cmd_pitch < 180.0f && g.cmd_pitch > -180.0f,
                  "pitch 落在 [-180,180] 内(正方向没有回绕)");
        }

        // **负方向**才是 wrap 会出问题的地方:
        // home_pitch=3 时下包络是 -27, 若实现里用 wrap360 就会变成 333。
        // 下位机收到 333 会认为要转到最高处, 云台直接顶到机械限位。
        {
            GimbalCtrl g2;
            prime(g2, 100.0f, 3.0f);
            for (int i = 0; i < 80; ++i) g2.track(0.0f, -40.0f);   // 往下压
            std::printf("  [-方向] after track: cmd_pitch = %.2f  (期望约 %.2f)\n",
                        static_cast<double>(g2.cmd_pitch),
                        static_cast<double>(g2.home_pitch - srm::GIMBAL_ENVELOPE_DEG));
            CHECK(g2.cmd_pitch < 0.0f,
                  "pitch 负方向没有回绕成 3xx");
            CHECK(g2.cmd_pitch >= g2.home_pitch - srm::GIMBAL_ENVELOPE_DEG - 1.0f,
                  "pitch 负方向不越包络");
        }
    }

    // ---------------------------------------------------------------- 2
    // home 接近机械行程边界时, 最终命令角必须仍在行程内。
    // (对称 ±30 度包络本身会越过行程, 靠绝对 clamp 兜住, 所以这里断言最终值。)
    std::printf("\n=== 2. home 靠近行程边界时, 最终命令角是否越界 ===\n");
    {
        for (float home : {0.0f, 10.0f, 20.0f, 25.0f}) {
            GimbalCtrl g;
            prime(g, 100.0f, home);
            for (int i = 0; i < 80; ++i) g.track(0.0f, -40.0f);   // 一直往负方向压
            std::printf("  home=%+6.1f -> 负向压满 cmd_pitch=%+7.2f (行程上限 %+.1f)\n",
                        static_cast<double>(home), static_cast<double>(g.cmd_pitch),
                        static_cast<double>(srm::GIMBAL_PITCH_MAX));
            CHECK(g.cmd_pitch <= srm::GIMBAL_PITCH_MAX + 0.01f,
                  "负向压满时命令角不超过行程上限");

            GimbalCtrl g2;
            prime(g2, 100.0f, home);
            for (int i = 0; i < 80; ++i) g2.track(0.0f, +40.0f);  // 一直往正方向推
            std::printf("           正向推满 cmd_pitch=%+7.2f (行程下限 %+.1f)\n",
                        static_cast<double>(g2.cmd_pitch),
                        static_cast<double>(srm::GIMBAL_PITCH_MIN));
            CHECK(g2.cmd_pitch >= srm::GIMBAL_PITCH_MIN - 0.01f,
                  "正向推满时命令角不低于行程下限");
        }
    }

    // ---------------------------------------------------------------- 3
    // yaw 跨 0/360 时, 包络与步进必须连续, 不能跳 360
    std::printf("\n=== 3. yaw 跨 0/360 ===\n");
    {
        GimbalCtrl g;
        prime(g, 5.0f, 0.0f);     // home 在 5 度, 靠近回绕点
        float max_jump = 0.0f;
        float prev = g.cmd_yaw;
        for (int i = 0; i < 200; ++i) {
            g.track(-30.0f, 0.0f);          // 持续往一个方向推
            float d = g.cmd_yaw - prev;
            if (d > 180.0f) d -= 360.0f;
            if (d < -180.0f) d += 360.0f;
            max_jump = std::max(max_jump, std::fabs(d));
            prev = g.cmd_yaw;
        }
        std::printf("  cmd_yaw = %.2f   单帧最大跳变 = %.2f 度\n",
                    static_cast<double>(g.cmd_yaw), static_cast<double>(max_jump));
        CHECK(max_jump <= srm::SERVO_MAX_STEP + 0.01f, "yaw 单帧跳变不超过 SERVO_MAX_STEP");
        CHECK(g.cmd_yaw >= 0.0f && g.cmd_yaw < 360.0f, "yaw 命令角在 [0,360) 内");
    }

    // ---------------------------------------------------------------- 4
    // 闭环收敛: 目标固定偏右 8 度, 云台应把偏差压到死区内
    std::printf("\n=== 4. 闭环收敛(带一个简单被控对象) ===\n");
    {
        GimbalCtrl g;
        prime(g, 100.0f, 0.0f);
        float plant_yaw = 100.0f;
        const float world_target = 108.0f;   // 世界系里目标在 108 度
        for (int i = 0; i < 300; ++i) {
            const float theta = world_target - plant_yaw;
            g.update_feedback(plant_yaw, 0.0f);
            g.track(theta, 0.0f);
            plant_yaw += 0.35f * (g.cmd_yaw - plant_yaw);
        }
        const float residual = world_target - plant_yaw;
        std::printf("  plant_yaw = %.2f   目标 %.2f   残差 = %+.2f 度\n",
                    static_cast<double>(plant_yaw), static_cast<double>(world_target),
                    static_cast<double>(residual));
        // 1.5 度而不是更严: 这个被控对象是"每帧走 35%% 的一阶滞后 + 每帧限速 2 度",
        // 存在一拍延迟, 因此会有小幅极限环而不是精确收敛到 0。
        // 实测残差约 1.07 度, 属于这个对象特性, 不是控制律错。
        // 若以后改了 SERVO_GAIN / SERVO_MAX_STEP / GIMBAL_SMOOTH, 这个数会跟着变,
        // 那时应重新评估而不是直接放宽阈值。
        CHECK(std::fabs(residual) < 1.5f, "稳态残差 < 1.5 度(该对象存在约1度的极限环)");
        std::printf("  注: 残差 %.2f 度对应 5m 处 140mm 板宽约 %.1f%% 的板宽\n",
                    static_cast<double>(std::fabs(residual)),
                    static_cast<double>(std::fabs(residual) / (140.0f / 5000.0f * 57.2958f) * 100.0f));
    }

    // ---------------------------------------------------------------- 5
    // 没有实测角时绝不能动
    std::printf("\n=== 5. 无实测角时不动 ===\n");
    {
        GimbalCtrl g;
        const float before_y = g.cmd_yaw, before_p = g.cmd_pitch;
        const bool r = g.track(20.0f, 20.0f);
        CHECK(!r, "track() 返回 false");
        CHECK(g.cmd_yaw == before_y && g.cmd_pitch == before_p, "命令角未改变");
    }

    std::printf("\n%s (%d 个失败)\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}

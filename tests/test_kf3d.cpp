// test_kf3d.cpp: 三维卡尔曼(延迟+飞行时间补偿)的离线验证
//
// 核心判据: 目标匀速运动时, 预测"子弹到达时刻"的三维位置 → 反算角度,
//           应比"用当前测量角度"更接近真实瞄准角。
#include <cmath>
#include <cstdio>

#include "kf3d.hpp"

using srm::CVKalman3D;

static int fails = 0;
#define CHECK(c, m)                                                        \
    do {                                                                   \
        if (!(c)) { std::printf("  [FAIL] %s (%d)\n", m, __LINE__); ++fails; } \
        else       { std::printf("  [ ok ] %s\n", m); }                     \
    } while (0)

static float yaw_of(float x, float z)   { return std::atan2(x, z) * 57.29578f; }

int main() {
    const float dt   = 1.0f / 60.0f;
    const float lat  = 0.080f;              // 链路延迟 80ms
    const float v    = 2000.0f;             // 目标横移 2 m/s
    const float z0   = 5000.0f;             // 5 m
    const float dist_m = z0 / 1000.0f;
    const float flight = dist_m / 22.0f;    // 弹丸飞行 22 m/s
    const float T    = lat + flight;

    std::printf("延迟 %.0f ms + 飞行 %.0f ms = 需外推 %.0f ms\n",
                lat * 1000, flight * 1000, T * 1000);

    // ---------------------------------------------------------------- 1
    std::printf("\n=== 1. 匀速横移目标: 预测 vs 不补偿 ===\n");
    {
        CVKalman3D kf;
        float worst_pred = 0, worst_raw = 0;
        for (int i = 0; i < 200; ++i) {
            const float t = i * dt;
            const float tx = v * t, ty = 0.0f, tz = z0;
            kf.predict(dt);
            kf.update(tx, ty, tz);
            if (i > 60) {
                float px, py, pz;
                kf.predict_pos(T, px, py, pz);
                // 真值: 子弹到达时刻目标位置
                const float fx = v * (t + T), fz = z0;
                worst_pred = std::fmax(worst_pred, std::fabs(yaw_of(px, pz) - yaw_of(fx, fz)));
                // 不补偿: 直接用当前测量角度
                worst_raw  = std::fmax(worst_raw,  std::fabs(yaw_of(tx, tz) - yaw_of(fx, fz)));
            }
        }
        std::printf("  yaw 误差: 预测 %.3f 度, 不补偿 %.3f 度\n",
                    (double)worst_pred, (double)worst_raw);
        CHECK(worst_pred < 0.4f, "预测 yaw 误差 < 0.4 度");
        CHECK(worst_raw > 2.0f, "不补偿误差明显更大(证明补偿有意义)");
    }

    // ---------------------------------------------------------------- 2
    // 距离变化: 目标朝相机移动。这是两轴角度滤波做不到的部分。
    std::printf("\n=== 2. 目标同时横向+朝相机移动(深度变化) ===\n");
    {
        CVKalman3D kf;
        const float vz = -500.0f;   // 每秒靠近 0.5 m
        float worst_pred = 0;
        for (int i = 0; i < 200; ++i) {
            const float t = i * dt;
            const float tx = v * t, ty = 0.0f, tz = z0 + vz * t;
            kf.predict(dt);
            kf.update(tx, ty, tz);
            if (i > 60) {
                float px, py, pz;
                kf.predict_pos(T, px, py, pz);
                const float fx = v * (t + T), fz = z0 + vz * (t + T);
                worst_pred = std::fmax(worst_pred, std::fabs(yaw_of(px, pz) - yaw_of(fx, fz)));
            }
        }
        std::printf("  深度变化下的 yaw 预测误差 = %.3f 度\n", (double)worst_pred);
        CHECK(worst_pred < 0.5f, "深度变化时仍 < 0.5 度(角度滤波做不到这点)");
    }

    // ---------------------------------------------------------------- 3
    std::printf("\n=== 3. 静止目标: 不应凭空产生速度 ===\n");
    {
        CVKalman3D kf;
        for (int i = 0; i < 200; ++i) {
            kf.predict(dt);
            kf.update(100.0f, -50.0f, 4000.0f);
        }
        std::printf("  速度估计 = %.2f mm/s\n", (double)kf.speed());
        CHECK(kf.speed() < 50.0f, "静止目标速度接近 0");
    }

    // ---------------------------------------------------------------- 4
    std::printf("\n=== 4. 单帧离群值不应带偏 ===\n");
    {
        CVKalman3D kf;
        for (int i = 0; i < 100; ++i) {
            kf.predict(dt);
            kf.update(v * i * dt, 0.0f, z0);
        }
        const float before = kf.px();
        kf.predict(dt);
        kf.update(9000.0f, 0.0f, z0);    // 离谱的一帧
        std::printf("  px: %.1f -> %.1f (变化 %.1f mm), 丢弃 %d 帧\n",
                    (double)before, (double)kf.px(),
                    (double)std::fabs(kf.px() - before), kf.rejected());
        CHECK(kf.rejected() == 1, "离群帧被门限丢弃");
        CHECK(std::fabs(kf.px() - before) < 500.0f, "位置未被带偏");
    }

    std::printf("\n%s (%d 个失败)\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}

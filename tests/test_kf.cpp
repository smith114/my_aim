// test_kf.cpp: 延迟补偿滤波器的离线验证
//
// 判据(不需要硬件):
//   匀速目标, 滤波器稳定后, predict_ahead(L) 的误差应远小于"不补偿"的滞后。
#include <cmath>
#include <cstdio>

#include "kf.hpp"

using srm::AngleKF;

static int fails = 0;
#define CHECK(c, m)                                                        \
    do {                                                                   \
        if (!(c)) {                                                        \
            std::printf("  [FAIL] %s  (%d)\n", m, __LINE__);                \
            ++fails;                                                       \
        } else {                                                           \
            std::printf("  [ ok ] %s\n", m);                                \
        }                                                                  \
    } while (0)

int main() {
    const float dt = 1.0f / 60.0f;     // 60 fps
    const float rate = 23.0f;          // 度/秒, 对应 5m 处 2 m/s 横移
    const float L = 0.08f;             // 80 ms 链路延迟

    // ---------------------------------------------------------------- 1
    // 匀速目标: 无噪声 -> 预测应几乎精确
    std::printf("=== 1. 匀速目标, 无测量噪声 ===\n");
    {
        AngleKF kf;
        const float true_rate = rate;
        float worst_pred = 0.0f, worst_raw = 0.0f;
        for (int i = 0; i < 200; ++i) {
            const float t = i * dt;
            const float true_theta = true_rate * t;
            kf.predict(dt);
            kf.update(true_theta);
            if (i > 60) {   // 给收敛时间
                const float pred = kf.predict_ahead(L);
                const float truth_future = true_rate * (t + L);
                worst_pred = std::fmax(worst_pred, std::fabs(pred - truth_future));
                worst_raw  = std::fmax(worst_raw,  std::fabs(kf.theta() - truth_future));
            }
        }
        std::printf("  预测误差最大 %.4f 度, 不补偿的滞后 %.4f 度\n",
                    (double)worst_pred, (double)worst_raw);
        CHECK(worst_pred < 0.3f, "预测误差 < 0.3 度");
        CHECK(worst_raw > 1.0f, "不补偿确实有明显滞后(证明这个滤波器有用)");
    }

    // ---------------------------------------------------------------- 2
    // 加测量噪声 -> 预测仍然可用
    std::printf("\n=== 2. 匀速目标 + 1 度测量噪声 ===\n");
    {
        AngleKF kf;
        unsigned seed = 12345;
        auto rnd = [&seed]() {
            seed = seed * 1103515245u + 12345u;
            return ((seed >> 16) & 0x7FFF) / 32767.0f - 0.5f;   // [-0.5,0.5]
        };
        float worst_pred = 0.0f;
        float sum_pred = 0.0f;
        int n = 0;
        for (int i = 0; i < 400; ++i) {
            const float t = i * dt;
            const float noise = rnd() * 2.0f;          // ±1 度
            kf.predict(dt);
            kf.update(rate * t + noise);
            if (i > 100) {
                const float pred = kf.predict_ahead(L);
                const float err = std::fabs(pred - rate * (t + L));
                worst_pred = std::fmax(worst_pred, err);
                sum_pred += err;
                ++n;
            }
        }
        std::printf("  预测误差: 平均 %.4f 度, 最大 %.4f 度\n",
                    (double)(sum_pred / n), (double)worst_pred);
        CHECK(sum_pred / n < 0.6f, "含噪声时平均预测误差 < 0.6 度");
    }

    // ---------------------------------------------------------------- 3
    // 静止目标: 外推不能凭空造出速度
    std::printf("\n=== 3. 静止目标, 不应产生速度 ===\n");
    {
        AngleKF kf;
        for (int i = 0; i < 200; ++i) {
            kf.predict(dt);
            kf.update(3.0f);   // 恒定 3 度
        }
        std::printf("  估计角速度 = %.4f 度/秒, 预测 80ms 后 = %.4f 度\n",
                    (double)kf.omega(), (double)kf.predict_ahead(L));
        CHECK(std::fabs(kf.omega()) < 0.5f, "静止目标角速度估计接近 0");
        CHECK(std::fabs(kf.predict_ahead(L) - 3.0f) < 0.3f, "外推几乎不偏离 3 度");
    }

    // ---------------------------------------------------------------- 4
    // 单帧离群值(误检)不应把预测推飞
    std::printf("\n=== 4. 单帧离群值不应推飞 ===\n");
    {
        AngleKF kf;
        for (int i = 0; i < 100; ++i) {
            kf.predict(dt);
            kf.update(rate * (i * dt));
        }
        const float before = kf.predict_ahead(L);
        kf.predict(dt);
        kf.update(200.0f);   // 一帧离谱的测量
        const float after = kf.predict_ahead(L);
        std::printf("  离群前后预测: %.2f -> %.2f 度 (变化 %.2f)\n",
                    (double)before, (double)after, (double)std::fabs(after - before));
        CHECK(std::fabs(after - before) < 1.0f, "单帧离群被新息门限丢弃, 预测几乎不变");
        std::printf("  被门限丢弃的帧数 = %d\n", kf.rejected());
        CHECK(kf.rejected() == 1, "恰好丢弃 1 帧");
    }

    // ---------------------------------------------------------------- 5
    // 速度限幅
    std::printf("\n=== 5. 角速度限幅 ===\n");
    {
        AngleKF kf;
        for (int i = 0; i < 100; ++i) {
            kf.predict(dt);
            kf.update((i % 2) ? 60.0f : -60.0f);   // 剧烈交替
        }
        std::printf("  剧烈交替测量下的角速度 = %.1f 度/秒\n", (double)kf.omega());
        CHECK(std::fabs(kf.omega()) <= 90.0f + 1e-3f, "角速度被限幅在 90 度/秒内");
    }

    std::printf("\n%s (%d 个失败)\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}
